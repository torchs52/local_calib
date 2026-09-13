from __future__ import annotations

import os
import time


def ensure_dir(path: str) -> None:
    os.makedirs(path, exist_ok=True)


def iso_now() -> str:
    return time.strftime("%Y-%m-%dT%H:%M:%S", time.localtime())


def safe_ts_compact(ts_iso: str) -> str:
    return ts_iso.replace("-", "").replace(":", "").replace("T", "_")


def atomic_write(path: str, text: str) -> None:
    ensure_dir(os.path.dirname(path) or ".")
    tmp = path + ".tmp"
    with open(tmp, "w", encoding="utf-8") as f:
        f.write(text)
    os.replace(tmp, path)


def read_int(path: str | None) -> int | None:
    if not path:
        return None
    try:
        with open(path, "r", encoding="utf-8") as f:
            return int(f.read().strip())
    except Exception:
        return None


def f_or_neg1(x: float | None) -> float:
    return x if x is not None else -1.0


def i_or_neg1(x: int | None) -> int:
    return x if x is not None else -1


def mib_to_gib(mib: int | None) -> float | None:
    if mib is None:
        return None
    return mib / 1024.0


def mib_to_gib_or_neg1(mib: int | None) -> float:
    v = mib_to_gib(mib)
    return -1.0 if v is None else v
