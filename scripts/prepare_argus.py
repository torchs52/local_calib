from __future__ import annotations

import argparse
import os
import pwd
import grp
from pathlib import Path

from argus_synchro.common.paths import DEFAULT_MMAP_DIR


def get_uid_gid(user: str, group: str) -> tuple[int, int]:
    uid = pwd.getpwnam(user).pw_uid
    gid = grp.getgrnam(group).gr_gid
    return uid, gid


def ensure_dir(directory: Path, uid: int, gid: int, mode: int) -> None:
    if directory.exists():
        if not directory.is_dir():
            raise NotADirectoryError(f"Expected directory but found file: {directory}")
        print(f"[INFO] already exists: {directory}")
    else:
        directory.mkdir(parents=True, exist_ok=True)
        print(f"[INFO] created directory: {directory}")

    os.chown(directory, uid, gid)
    directory.chmod(mode)


def ensure_status_mmap(mmap_dir: Path, uid: int, gid: int) -> None:
    status_mmap = mmap_dir / "status.mmap"

    if status_mmap.exists():
        if not status_mmap.is_file():
            raise FileExistsError(f"Expected file but found directory: {status_mmap}")
        print(f"[INFO] already exists: {status_mmap}")
    else:
        print(f"[INFO] created: {status_mmap}")

    # 毎回 INIT(0) で初期化する
    status_mmap.write_bytes(b"\x00" * 4)

    os.chown(status_mmap, uid, gid)
    status_mmap.chmod(0o664)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Prepare Argus runtime directories")

    parser.add_argument("--config-dir", default="./config")
    parser.add_argument("--log-dir", default="./log")
    parser.add_argument("--mmap-dir", default=str(DEFAULT_MMAP_DIR))

    parser.add_argument("--user", required=True)
    parser.add_argument("--group", required=True)

    return parser.parse_args()


def main() -> int:
    args = parse_args()

    uid, gid = get_uid_gid(args.user, args.group)

    config_dir = Path(args.config_dir).expanduser().resolve()
    log_dir = Path(args.log_dir).expanduser().resolve()
    mmap_dir = Path(args.mmap_dir).expanduser().resolve()

    ensure_dir(config_dir, uid, gid, 0o755)
    ensure_dir(log_dir, uid, gid, 0o755)
    if mmap_dir.resolve() != DEFAULT_MMAP_DIR.resolve():
        ensure_dir(mmap_dir, uid, gid, 0o775)

    ensure_status_mmap(mmap_dir, uid, gid)

    print("[INFO] prepare_argus.py completed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())