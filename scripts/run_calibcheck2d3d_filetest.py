#!/usr/bin/env python3
"""calibcheck2d3d (カメラずれ診断) を UI を介さずファイル入力モードで自動実行するスクリプト。

前提: config/settings.ini が File_Input = True (ファイル入力モード) であること。

流れ:
  1. operation_mode=0 (周辺監視) の状態から argus_start_simple.sh を起動する。
     (起動時に StartupResetPolicy が CalibMode 系フラグを強制リセットするため、
      事前に isRunning2D3Dcheck 等を True にしておいても意味がない。)
  2. 起動ログで安定稼働を確認したのち、settings.ini を
     operation_mode=1 (校正) + isRunning2D3Dcheck=True に書き換える。
     (watchdog によるファイル監視で自動的にプロセスへ反映される。)
  3. ログの framecounter を監視し、指定フレーム数に到達したら
     start2D3DCheckCalc=True に切り替えて評価(post_app_loopmain)へ移行する。
  4. Camera0/1/2 の判定結果ログ ("CameraN result: ...") を待って表示する。
  5. 全フラグを安全な初期値 (operation_mode=0, 各 CalibMode フラグ False) へ戻し、
     プロセスを終了する。

使い方:
    python scripts/run_calibcheck2d3d_filetest.py --frames 500

いずれの段階で失敗しても、finally 節で必ず設定を安全な状態に戻し、
起動したプロセスを終了させる。
"""

from __future__ import annotations

import argparse
import re
import subprocess
import sys
import time
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
SETTINGS_PATH = REPO_ROOT / "config" / "settings.ini"
LOG_PATH = REPO_ROOT / "log" / "argus3d_core.log"
START_SCRIPT = REPO_ROOT / "argus_start_simple.sh"

FRAMECOUNTER_RE = re.compile(r"framecounter=(\d+)")
APP_MANAGER_ALIVE_RE = re.compile(r"AppManager alive")
CALIBCHECK_STARTED_RE = re.compile(r"boss - .*calibcheck2d3d_app started")
CAMERA_RESULT_RE = re.compile(r"Camera(\d) result: (\w+), reason:(\d+)")

SAFE_FLAGS = {
    "operation_mode": "0",
    "isRunning3D3Dcalib": "False",
    "isRunning2D3Dcalib": "False",
    "start2D3DCalibCalc": "False",
    "isRunning2D3Dcheck": "False",
    "start2D3DCheckCalc": "False",
}


class TimeoutWaitingForLog(RuntimeError):
    pass


def _set_ini_flags(flags: dict[str, str]) -> None:
    """settings.ini の指定キーだけを書き換える(コメント・他行は保持する)。"""
    text = SETTINGS_PATH.read_text(encoding="utf-8")
    for key, value in flags.items():
        pattern = re.compile(rf"^{re.escape(key)} *=.*$", re.MULTILINE)
        if not pattern.search(text):
            raise RuntimeError(f"settings.ini に {key} が見つかりません")
        text = pattern.sub(f"{key} = {value}", text)
    SETTINGS_PATH.write_text(text, encoding="utf-8")


def _wait_for_log_pattern(
    pattern: re.Pattern[str],
    *,
    start_offset: int,
    timeout: float,
    poll_interval: float = 1.0,
) -> tuple[re.Match[str], int]:
    """LOG_PATH の start_offset バイト以降から pattern に最初にマッチする行を待つ。

    戻り値は (マッチオブジェクト, マッチ行が含まれていた時点でのファイル末尾オフセット)。
    """
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if LOG_PATH.exists():
            with LOG_PATH.open(encoding="utf-8", errors="ignore") as f:
                f.seek(start_offset)
                while True:
                    line = f.readline()
                    if not line:
                        break
                    match = pattern.search(line)
                    if match:
                        return match, f.tell()
        time.sleep(poll_interval)
    raise TimeoutWaitingForLog(f"{pattern.pattern!r} がタイムアウトまでに出現しませんでした")


def _wait_for_framecount(
    target_frames: int,
    *,
    start_offset: int,
    timeout: float,
    poll_interval: float = 1.0,
) -> int:
    """framecounter が target_frames 以上になるまで待つ。到達時点のファイルオフセットを返す。"""
    deadline = time.monotonic() + timeout
    last_offset = start_offset
    while time.monotonic() < deadline:
        if LOG_PATH.exists():
            with LOG_PATH.open(encoding="utf-8", errors="ignore") as f:
                f.seek(start_offset)
                text = f.read()
                last_offset = f.tell()
            frame_numbers = [int(m.group(1)) for m in FRAMECOUNTER_RE.finditer(text)]
            if frame_numbers and max(frame_numbers) >= target_frames:
                print(f"[INFO] framecounter={max(frame_numbers)} に到達")
                return last_offset
        time.sleep(poll_interval)
    raise TimeoutWaitingForLog(
        f"framecounter が {target_frames} に到達する前にタイムアウトしました"
    )


def _collect_camera_results(
    *,
    start_offset: int,
    timeout: float,
    poll_interval: float = 1.0,
) -> dict[int, tuple[str, int]]:
    """Camera0/1/2 の判定結果ログを全て集めるまで待つ。"""
    deadline = time.monotonic() + timeout
    results: dict[int, tuple[str, int]] = {}
    while time.monotonic() < deadline:
        if LOG_PATH.exists():
            with LOG_PATH.open(encoding="utf-8", errors="ignore") as f:
                f.seek(start_offset)
                for line in f:
                    match = CAMERA_RESULT_RE.search(line)
                    if match:
                        camera_ix = int(match.group(1))
                        results[camera_ix] = (match.group(2), int(match.group(3)))
        if len(results) >= 3:
            return results
        time.sleep(poll_interval)
    raise TimeoutWaitingForLog("カメラ判定結果ログが揃う前にタイムアウトしました")


def run(frames: int, startup_timeout: float, eval_timeout: float) -> int:
    existing = subprocess.run(
        ["pgrep", "-f", "argus_synchro"], capture_output=True, text=True, check=False
    )
    if existing.stdout.strip():
        print("[ERROR] argus_synchro が既に起動中です。先に終了させてください。")
        return 1

    _set_ini_flags(SAFE_FLAGS)

    log_start_offset = LOG_PATH.stat().st_size if LOG_PATH.exists() else 0
    process = subprocess.Popen(
        [str(START_SCRIPT)],
        cwd=REPO_ROOT,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )
    print(f"[INFO] argus_start_simple.sh 起動 (pid={process.pid})")

    try:
        # AppManagerProcess の起動完了(watchdog observer開始後に出る定期ログ)を待ってから
        # 校正モードへ切り替える。固定スリープだと起動が遅い環境で設定変更が
        # observer起動前に行われてしまい、反映されないことがある。
        _wait_for_log_pattern(
            APP_MANAGER_ALIVE_RE,
            start_offset=log_start_offset,
            timeout=startup_timeout,
        )
        time.sleep(3)
        _set_ini_flags({"operation_mode": "1", "isRunning2D3Dcheck": "True"})
        print("[INFO] operation_mode=1, isRunning2D3Dcheck=True に切替")

        _, offset_after_start = _wait_for_log_pattern(
            CALIBCHECK_STARTED_RE,
            start_offset=log_start_offset,
            timeout=startup_timeout,
        )
        print("[INFO] calibcheck2d3d_app started を確認")

        offset_after_frames = _wait_for_framecount(
            frames,
            start_offset=offset_after_start,
            timeout=eval_timeout,
        )

        _set_ini_flags({"start2D3DCheckCalc": "True"})
        print("[INFO] start2D3DCheckCalc=True に切替、評価へ移行")

        results = _collect_camera_results(
            start_offset=offset_after_frames,
            timeout=eval_timeout,
        )

        print("\n=== calibcheck2d3d 診断結果 ===")
        for camera_ix in sorted(results):
            result_str, reason = results[camera_ix]
            print(f"Camera{camera_ix}: {result_str} (reason:{reason})")
        return 0
    finally:
        _set_ini_flags(SAFE_FLAGS)
        print("[INFO] フラグを安全な初期値へ戻しました")
        if process.poll() is None:
            process.terminate()
            try:
                process.wait(timeout=15)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=15)
        subprocess.run(["pkill", "-f", "argus_synchro"], check=False)
        print("[INFO] プロセスを終了しました")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--frames",
        type=int,
        default=500,
        help="post_app_loopmain へ移行するまでに処理させるフレーム数(既定値: 500)",
    )
    parser.add_argument(
        "--startup-timeout",
        type=float,
        default=120.0,
        help="calibcheck2d3d_app の起動確認タイムアウト秒数(既定値: 120)",
    )
    parser.add_argument(
        "--eval-timeout",
        type=float,
        default=120.0,
        help="フレーム到達・評価完了それぞれの待ちタイムアウト秒数(既定値: 120)",
    )
    args = parser.parse_args()
    try:
        return run(args.frames, args.startup_timeout, args.eval_timeout)
    except TimeoutWaitingForLog as e:
        print(f"[ERROR] {e}")
        return 1


if __name__ == "__main__":
    sys.exit(main())
