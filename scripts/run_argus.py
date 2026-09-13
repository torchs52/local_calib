from __future__ import annotations

import argparse
import subprocess
import sys
from collections.abc import Generator
from contextlib import contextmanager, nullcontext
from pathlib import Path

from argus_synchro.common.paths import DEFAULT_MMAP_DIR
from settings_swap import swap_settings

DEFAULT_GODOT_MAIN_SH = (
    Path(__file__).resolve().parents[2] / "argus_godot_ui" / "main.sh"
)
DEFAULT_GODOT_WORK_DIR = DEFAULT_GODOT_MAIN_SH.parent / "godot_engine"
DEFAULT_GODOT_SETTINGS_CFG = (
    DEFAULT_GODOT_MAIN_SH.parent / "settings" / "godotSettings.cfg"
)
DEFAULT_ARGUS_ROOT_DIR = Path(__file__).resolve().parents[1]
GODOT_PATH_PLACEHOLDER = "/home/shiuser/argus_pipe_filter/"


@contextmanager
def swap_godot_settings_path(
    godot_settings_cfg: Path, argus_root_dir: Path, mode: str
) -> Generator[None, None, None]:
    """godotSettings.cfg 内の argus パスを実行環境向けに一時置換する"""
    if not godot_settings_cfg.exists():
        raise FileNotFoundError(
            f"Godot 設定ファイルが見つかりません: {godot_settings_cfg}"
        )
    if not godot_settings_cfg.is_file():
        raise FileNotFoundError(f"Godot 設定ファイルが不正です: {godot_settings_cfg}")
    if not argus_root_dir.exists():
        raise FileNotFoundError(f"argus ルートが見つかりません: {argus_root_dir}")
    if not argus_root_dir.is_dir():
        raise NotADirectoryError(
            f"argus ルートがディレクトリではありません: {argus_root_dir}"
        )

    backup_path = godot_settings_cfg.with_name(f"{godot_settings_cfg.name}.backup.{mode}")
    if backup_path.exists():
        raise FileExistsError(f"既存バックアップがあるため中断します: {backup_path}")

    replacement_prefix = argus_root_dir.as_posix().rstrip("/") + "/"
    source_bytes = godot_settings_cfg.read_bytes()
    replaced_bytes = source_bytes.replace(
        GODOT_PATH_PLACEHOLDER.encode(), replacement_prefix.encode()
    )
    if source_bytes == replaced_bytes:
        print(
            "[WARN] godotSettings.cfg に置換対象が見つかりませんでした: "
            f"{GODOT_PATH_PLACEHOLDER}"
        )
    else:
        print(
            "[INFO] godotSettings.cfg path replaced: "
            f"{GODOT_PATH_PLACEHOLDER} -> {replacement_prefix}"
        )

    godot_settings_cfg.rename(backup_path)
    godot_settings_cfg.write_bytes(replaced_bytes)

    try:
        yield
    finally:
        if godot_settings_cfg.exists():
            godot_settings_cfg.unlink()
        backup_path.rename(godot_settings_cfg)
        print(f"[INFO] {godot_settings_cfg.name} restored from backup")


@contextmanager
def launch_godot_ui(main_sh_path: Path, godot_work_dir: Path) -> Generator[None, None, None]:
    """Godot UIを起動し、終了時に後始末する"""
    if not main_sh_path.exists():
        raise FileNotFoundError(f"Godot UI起動スクリプトが見つかりません: {main_sh_path}")
    if not main_sh_path.is_file():
        raise FileNotFoundError(f"Godot UI起動スクリプトが不正です: {main_sh_path}")
    if not godot_work_dir.exists():
        raise FileNotFoundError(
            f"Godot UIのワーキングディレクトリが見つかりません: {godot_work_dir}"
        )
    if not godot_work_dir.is_dir():
        raise NotADirectoryError(
            f"Godot UIのワーキングディレクトリがディレクトリではありません: {godot_work_dir}"
        )

    print(f"[INFO] launching Godot UI: {main_sh_path} (cwd={godot_work_dir})")
    process = subprocess.Popen(
        ["bash", str(main_sh_path)],
        cwd=godot_work_dir,
    )
    try:
        yield
    finally:
        if process.poll() is None:
            print("[INFO] stopping Godot UI")
            process.terminate()
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                print("[WARN] Godot UI did not stop in time; killing process")
                process.kill()
                process.wait()
        print(f"[INFO] Godot UI process exited with code {process.returncode}")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="argus_synchro run/prof launcher")
    parser.add_argument(
        "--config-dir",
        default="./config",
        help="設定ファイル(config)ディレクトリのパス",
    )
    parser.add_argument(
        "--log-dir",
        default="./log",
        help="ログディレクトリのパス",
    )
    parser.add_argument(
        "--mmap-dir",
        default=str(DEFAULT_MMAP_DIR),
        help="mmapディレクトリのパス",
    )
    parser.add_argument(
        "mode",
        nargs="?",
        choices=["run", "prof"],
        default="run",
        help="実行モード (run: 通常起動, prof: profiling.py起動)",
    )
    parser.add_argument(
        "--godot-main-sh",
        default=str(DEFAULT_GODOT_MAIN_SH),
        help="Godot UI起動スクリプト(main.sh)のパス",
    )
    parser.add_argument(
        "--godot-work-dir",
        default=str(DEFAULT_GODOT_WORK_DIR),
        help="Godot UI起動時のワーキングディレクトリ",
    )
    parser.add_argument(
        "--godot-settings-cfg",
        default=str(DEFAULT_GODOT_SETTINGS_CFG),
        help="Godot 設定ファイル(godotSettings.cfg)のパス",
    )
    parser.add_argument(
        "--argus-root-dir",
        default=str(DEFAULT_ARGUS_ROOT_DIR),
        help="godotSettings.cfg の置換先となる argus ルートディレクトリ",
    )
    parser.add_argument(
        "--with-ui",
        action="store_true",
        help="run/prof モード時に Godot UI を起動する",
    )
    parser.add_argument(
        "--no-swap-settings",
        action="store_true",
        help="settings.ini の環境別差し替えを行わない",
    )
    parser.add_argument(
        "--prof-run",
        choices=["fps", "target", "all"],
        default="all",
        help=(
            "prof モード時の計測フェーズ。"
            "fps: prof_fpsのみ, target: prof_targetのみ, all: 両方 (従来どおり)"
        ),
    )
    args, extra_args = parser.parse_known_args()
    args.extra_args = extra_args
    return args


def run_argus_core(
    config_dir: Path,
    log_dir: Path,
    mmap_dir: Path,
    extra_args: list[str],
) -> subprocess.CompletedProcess[bytes]:
    return subprocess.run(
        [
            sys.executable,
            "-m",
            "argus_synchro",
            "--config-dir",
            str(config_dir),
            "--log-dir",
            str(log_dir),
            "--mmap-dir",
            str(mmap_dir),
            *extra_args,
        ],
        check=False,
    )


def run_prof_core(
    config_dir: Path,
    log_dir: Path,
    mmap_dir: Path,
    extra_args: list[str],
    prof_run: str,
) -> subprocess.CompletedProcess[bytes]:
    return subprocess.run(
        [
            sys.executable,
            "argus_synchro/profiling.py",
            "--config-dir",
            str(config_dir),
            "--log-dir",
            str(log_dir),
            "--mmap-dir",
            str(mmap_dir),
            "--prof-run",
            prof_run,
            *extra_args,
        ],
        check=False,
    )


def run_app(
    config_dir: Path,
    log_dir: Path,
    mmap_dir: Path,
    extra_args: list[str],
    godot_main_sh: Path,
    godot_work_dir: Path,
    godot_settings_cfg: Path,
    argus_root_dir: Path,
    with_ui: bool,
    no_swap_settings: bool,
) -> int:
    settings_context = (
        nullcontext()
        if no_swap_settings
        else swap_settings(config_dir, mode="run")
    )

    with settings_context:
        if with_ui:
            with swap_godot_settings_path(godot_settings_cfg, argus_root_dir, mode="run"):
                with launch_godot_ui(godot_main_sh, godot_work_dir):
                    completed = run_argus_core(
                        config_dir, log_dir, mmap_dir, extra_args
                    )
        else:
            completed = run_argus_core(config_dir, log_dir, mmap_dir, extra_args)

    return completed.returncode


def run_prof(
    config_dir: Path,
    log_dir: Path,
    mmap_dir: Path,
    extra_args: list[str],
    godot_settings_cfg: Path,
    argus_root_dir: Path,
    godot_main_sh: Path,
    godot_work_dir: Path,
    with_ui: bool,
    no_swap_settings: bool,
    prof_run: str,
) -> int:
    settings_context = (
        nullcontext()
        if no_swap_settings
        else swap_settings(config_dir, mode="prof")
    )

    with settings_context:
        if with_ui:
            with swap_godot_settings_path(godot_settings_cfg, argus_root_dir, mode="prof"):
                with launch_godot_ui(godot_main_sh, godot_work_dir):
                    completed = run_prof_core(
                        config_dir, log_dir, mmap_dir, extra_args, prof_run
                    )
        else:
            completed = run_prof_core(
                config_dir, log_dir, mmap_dir, extra_args, prof_run
            )

    return completed.returncode


def main() -> int:
    args = parse_args()
    config_dir = Path(args.config_dir)
    log_dir = Path(args.log_dir)
    mmap_dir = Path(args.mmap_dir)
    godot_main_sh = Path(args.godot_main_sh).expanduser().resolve()
    godot_work_dir = Path(args.godot_work_dir).expanduser().resolve()
    godot_settings_cfg = Path(args.godot_settings_cfg).expanduser().resolve()
    argus_root_dir = Path(args.argus_root_dir).expanduser().resolve()
    if args.mode == "prof":
        return run_prof(
            config_dir,
            log_dir,
            mmap_dir,
            args.extra_args,
            godot_settings_cfg,
            argus_root_dir,
            godot_main_sh,
            godot_work_dir,
            args.with_ui,
            args.no_swap_settings,
            args.prof_run,
        )
    return run_app(
        config_dir,
        log_dir,
        mmap_dir,
        args.extra_args,
        godot_main_sh,
        godot_work_dir,
        godot_settings_cfg,
        argus_root_dir,
        args.with_ui,
        args.no_swap_settings,
    )


if __name__ == "__main__":
    raise SystemExit(main())
