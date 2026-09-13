#!/usr/bin/env python3
from __future__ import annotations

import argparse
import contextlib
import sys
from pathlib import Path

import numpy as np
import pandas as pd

TARGET_FILES = (
    "fps_GetDataProcess.csv",
    "fps_PointsRefineProcess.csv",
    "fps_VisualProcess.csv",
)
TARGET_COLUMN = "pcd_delay[s]"


def _skip_first_rows(csv_path: Path, skip_count: int) -> pd.DataFrame:
    if skip_count <= 0:
        return pd.read_csv(csv_path)
    return pd.read_csv(csv_path, skiprows=range(1, skip_count + 1))


def load_pcd_delays(csv_path: Path) -> list[float]:
    try:
        frame = _skip_first_rows(csv_path, 10)
    except Exception as exc:
        raise ValueError(f"Failed to read {csv_path}: {exc}") from exc

    if TARGET_COLUMN not in frame.columns:
        raise ValueError(f"Missing '{TARGET_COLUMN}' in {csv_path}")

    series = pd.to_numeric(frame[TARGET_COLUMN], errors="coerce").dropna()
    return series.to_numpy(dtype=float).tolist()


def find_target_dirs(result_dir: Path) -> list[Path]:
    dirs: set[Path] = set()
    for path in result_dir.rglob(TARGET_FILES[0]):
        parent = path.parent
        if all((parent / name).is_file() for name in TARGET_FILES):
            dirs.add(parent)
    return sorted(dirs)


def main() -> int:
    parser = argparse.ArgumentParser(
        description=(
            "Summarize pcd_delay[s] for directories that contain the target CSVs."
        )
    )
    parser.add_argument(
        "--result-dir",
        type=Path,
        default=Path("result"),
        help="Result root directory to scan (default: result)",
    )
    args = parser.parse_args()
    result_dir = args.result_dir

    if not result_dir.exists():
        print(f"Result directory not found: {result_dir}", file=sys.stderr)
        return 1

    target_dirs = find_target_dirs(result_dir)
    if not target_dirs:
        print(
            f"No directories with {', '.join(TARGET_FILES)} found under {result_dir}",
            file=sys.stderr,
        )
        return 1

    print(
        "dir,"
        "get_mean,get_max,get_min,"
        "points_mean,points_max,points_min,"
        "visual_mean,visual_max,visual_min"
    )
    for target_dir in target_dirs:
        rel_dir = target_dir
        with contextlib.suppress(ValueError):
            rel_dir = target_dir.relative_to(result_dir)
        stats: list[tuple[float, float, float]] = []
        for file_name in TARGET_FILES:
            csv_path = target_dir / file_name
            values = load_pcd_delays(csv_path)
            if not values:
                print(
                    f"No '{TARGET_COLUMN}' values in {csv_path}",
                    file=sys.stderr,
                )
                stats = []
                break
            data = np.asarray(values, dtype=float)
            stats.append(
                (float(np.mean(data)), float(np.max(data)), float(np.min(data)))
            )
        if not stats:
            continue
        formatted = ",".join(
            f"    , {stat[0]:.3f}, {stat[1]:.3f}, {stat[2]:.3f}" for stat in stats
        )
        print(f"{rel_dir.as_posix()},{formatted}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
