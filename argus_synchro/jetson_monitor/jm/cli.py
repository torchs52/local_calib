from __future__ import annotations

import argparse

DEFAULT_INTERVAL_SEC = 2.0
DEFAULT_WINDOW_SEC = 600


def load_config_args(path: str) -> list[str]:
    out: list[str] = []
    with open(path, "r", encoding="utf-8") as f:
        for ln in f:
            s = ln.strip()
            if not s or s.startswith("#"):
                continue
            out.extend(s.split())
    return out


def strip_config_from_argv(argv: list[str]) -> tuple[str | None, list[str]]:
    cfg: str | None = None
    rest: list[str] = []
    i = 0
    while i < len(argv):
        a = argv[i]
        if a == "--config":
            if i + 1 < len(argv):
                cfg = argv[i + 1]
                i += 2
                continue
        rest.append(a)
        i += 1
    return cfg, rest


def build_parser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser()

    ap.add_argument("--config", type=str, default=None)
    ap.add_argument("--settings-ini", type=str, default=None)

    ap.add_argument("--interval", type=float, default=DEFAULT_INTERVAL_SEC)
    ap.add_argument("--window-sec", type=int, default=DEFAULT_WINDOW_SEC)

    ap.add_argument("--metrics-path", type=str, default=None)
    ap.add_argument("--log-jsonl-ts-name", action="store_true")

    ap.add_argument("--jsonl-rotate-mb", type=int, default=0)
    ap.add_argument("--jsonl-rotate-keep", type=int, default=5)

    ap.add_argument("--disk-guard-gib", type=float, default=1.0)
    ap.add_argument("--disk-path", type=str, default="/")

    return ap


def parse_args(argv: list[str]):
    ap = build_parser()
    cfg_path, rest_argv = strip_config_from_argv(argv)

    merged = rest_argv[:]
    if cfg_path:
        merged = load_config_args(cfg_path) + rest_argv

    return ap.parse_args(merged)
