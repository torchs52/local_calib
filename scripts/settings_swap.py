from __future__ import annotations

import argparse
import platform
import shutil
from collections.abc import Generator
from contextlib import contextmanager
from pathlib import Path

SWAP_DEST_NAMES = ("settings.ini", "SCX900-3_settings.ini")


def is_jetson_environment() -> bool:
    """Jetson実行環境かを判定する"""
    return Path("/etc/nv_tegra_release").exists() and platform.machine() == "aarch64"


def is_wsl_environment() -> bool:
    """WSL実行環境かを判定する"""
    return "microsoft" in platform.release().lower()


def select_swap_files(config_dir: Path, mode: str) -> list[tuple[Path, Path]]:
    """実行環境に応じた差し替え対象(dst, src)を返す"""
    if is_jetson_environment():
        suffix = "jetson"
    elif is_wsl_environment():
        suffix = "wsl"
    else:
        return []

    settings_name = (
        f"settings_{suffix}_prof.ini" if mode == "prof" else f"settings_{suffix}.ini"
    )
    return [
        (config_dir / "settings.ini", config_dir / settings_name),
        (
            config_dir / "SCX900-3_settings.ini",
            config_dir / f"SCX900-3_settings_{suffix}.ini",
        ),
    ]


def _backup_path(dst: Path, mode: str) -> Path:
    return dst.with_name(f"{dst.name}.backup.{mode}")


def restore_settings(config_dir: Path, mode: str) -> None:
    """バックアップがあれば元の設定へ戻す。無ければ何もしない。"""
    dests = [config_dir / name for name in SWAP_DEST_NAMES]
    for dst in reversed(dests):
        backup_path = _backup_path(dst, mode)
        if not backup_path.exists():
            continue
        if dst.exists():
            dst.unlink()
        backup_path.rename(dst)
        print(f"[INFO] {dst.name} restored from backup")


def apply_settings(
    config_dir: Path,
    mode: str,
    *,
    restore_existing: bool = False,
) -> None:
    """実行環境に応じた設定へ差し替える。"""
    if restore_existing:
        restore_settings(config_dir, mode)

    for dst, src in select_swap_files(config_dir, mode):
        if not dst.exists():
            raise FileNotFoundError(f"設定ファイルが見つかりません: {dst}")
        if not src.exists():
            raise FileNotFoundError(f"設定ファイルが見つかりません: {src}")
        backup_path = _backup_path(dst, mode)
        if backup_path.exists():
            raise FileExistsError(
                f"既存バックアップがあるため中断します: {backup_path}"
            )

        print(f"[INFO] {mode} config selected: {src.name} -> {dst.name}")
        dst.rename(backup_path)
        shutil.copy2(src, dst)


@contextmanager
def swap_settings(config_dir: Path, mode: str) -> Generator[None, None, None]:
    """実行中のみ環境別設定へ差し替える"""
    apply_settings(config_dir, mode)
    try:
        yield
    finally:
        restore_settings(config_dir, mode)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="argus settings swap helper")
    parser.add_argument("action", choices=["apply", "restore"])
    parser.add_argument("--mode", choices=["run", "prof"], default="run")
    parser.add_argument("--config-dir", default="./config")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    config_dir = Path(args.config_dir)
    if args.action == "apply":
        apply_settings(config_dir, args.mode, restore_existing=True)
    else:
        restore_settings(config_dir, args.mode)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
