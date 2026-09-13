from __future__ import annotations

import json
import os

from .models import Metrics
from .utils import (
    atomic_write,
    ensure_dir,
    f_or_neg1,
    i_or_neg1,
    mib_to_gib,
    safe_ts_compact,
)


def to_float(v, digits: int | None = None, invalid_value: float = -1.0) -> float:
    if v is None:
        return invalid_value

    x = float(v)
    if digits is not None:
        x = round(x, digits)

    return x


def to_int(v) -> int:
    if v is None:
        return -1

    return int(v)


def resolve_metrics_path(
    default_metrics_out: str, args_metrics_path: str | None
) -> str:
    if not args_metrics_path:
        return default_metrics_out

    if os.path.isdir(args_metrics_path):
        return os.path.join(args_metrics_path, "metrics.txt")

    return args_metrics_path


def make_ts_named_jsonl_path(base_path: str, start_ts_iso: str) -> str:
    d = os.path.dirname(base_path) or "."
    base = os.path.basename(base_path)
    stem, dot, ext = base.partition(".")
    ext2 = ext if dot else "jsonl"
    return os.path.join(d, f"{stem}_{safe_ts_compact(start_ts_iso)}.{ext2}")


def make_log_record_dict(m: Metrics) -> dict:
    return {
        "schema": 3,
        "ts": m.ts_iso,
        "cpu": {
            "user_pct": m.cpu_us,
            "system_pct": m.cpu_sy,
            "wait_pct": m.cpu_wa,
            "core_count": m.cpu_core_count,
            "online_count": m.cpu_online_count,
            "freq_each_mhz": m.cpu_freq_each_mhz,
            "freq_max_mhz": m.cpu_fmax_mhz,
            "freq_min_mhz": m.cpu_fmin_mhz,
            "load_avg_pct": m.cpu_load_avg,
            "load_min_pct": m.cpu_load_min,
            "load_max_pct": m.cpu_load_max,
        },
        "gpu": {
            "util_pct": m.gr3d_pct,
            "freq_cur_mhz": m.gpu_cur_mhz,
            "freq_limit_mhz": m.gpu_max_mhz,
        },
        "power_mw": {
            "in": m.p_in_mw,
            "gpu": m.p_gpu_mw,
            "cpu": m.p_cpu_mw,
        },
        "memory": {
            "ram_used_mb": m.ram_used_mb,
            "ram_total_mb": m.ram_total_mb,
            "lfb_n": m.lfb_n,
            "lfb_mb": m.lfb_mb,
            "swap_used_mb": m.swap_used_mb,
            "swap_total_mb": m.swap_total_mb,
            "swap_cached_mb": m.swap_cached_mb,
        },
        "temperature_c": {
            "tj": m.tj_c,
            "cpu": m.cpu_c,
            "gpu": m.gpu_c,
        },
        "throttle_win_s": {
            "any": m.thr_any_win_s,
            "cpu": m.thr_cpu_win_s,
            "gpu": m.thr_gpu_win_s,
        },
        "disk": {
            "root": {
                "mount": "/",
                "avail_gib": m.disk_root_avail_gib,
                "used_pct": m.disk_root_used_pct,
            },
            "data": {
                "mount": "/mnt/nvme",
                "expected": bool(m.disk_data_expected),
                "mounted": bool(m.disk_data_mounted),
                "avail_gib": m.disk_data_avail_gib,
                "used_pct": m.disk_data_used_pct,
            },
        },
        "health": {
            "state": int(m.state),
            "reason": int(m.reason),
            "reason_text": m.reason_text,
        },
    }


def make_godot_snapshot_dict(m: Metrics) -> dict:
    return {
        "ts": m.ts_iso,
        "cpu_us": to_float(m.cpu_us, 1),
        "cpu_sy": to_float(m.cpu_sy, 1),
        "cpu_wa": to_float(m.cpu_wa, 1),
        "cpu_avg": to_float(m.cpu_load_avg, 1),
        "cpu_loadmax": to_float(m.cpu_load_max, 1),
        "cpu_loadmin": to_float(m.cpu_load_min, 1),
        "cpu_fmax": to_int(m.cpu_fmax_mhz),
        "cpu_fmin": to_int(m.cpu_fmin_mhz),
        "gr3d_pct": to_float(m.gr3d_pct, 1),
        "gpu_fcur": to_int(m.gpu_cur_mhz),
        "gpu_flimit": to_int(m.gpu_max_mhz),
        "tj": to_float(m.tj_c, 1, -300.0),
        "tc": to_float(m.cpu_c, 1, -300.0),
        "tg": to_float(m.gpu_c, 1, -300.0),
        "ram_u_gb": to_float(mib_to_gib(m.ram_used_mb), 2),
        "ram_t_gb": to_float(mib_to_gib(m.ram_total_mb), 2),
        "lfb_n": to_int(m.lfb_n),
        "lfb_mb": to_int(m.lfb_mb),
        "swap_u_mb": to_float(m.swap_used_mb, 1),
        "p_in_mw": to_int(m.p_in_mw),
        "p_gpu_mw": to_int(m.p_gpu_mw),
        "p_cpu_mw": to_int(m.p_cpu_mw),
        "disk_root_avail": to_float(m.disk_root_avail_gib, 2),
        "disk_root_used": to_float(m.disk_root_used_pct, 1),
        "disk_data_expected": 1 if m.disk_data_expected else 0,
        "disk_data_mounted": 1 if m.disk_data_mounted else 0,
        "disk_data_avail": to_float(m.disk_data_avail_gib, 2),
        "disk_data_used": to_float(m.disk_data_used_pct, 1),
        "thr_any": to_float(m.thr_any_win_s, 1),
        "thr_cpu": to_float(m.thr_cpu_win_s, 1),
        "thr_gpu": to_float(m.thr_gpu_win_s, 1),
        "state": to_int(m.state),
        "reason": to_int(m.reason),
    }


def format_text_for_ui(m: Metrics, interval_sec: float, window_sec: int) -> str:
    ram_used_gb = mib_to_gib(m.ram_used_mb)
    ram_total_gb = mib_to_gib(m.ram_total_mb)

    root_avail_text = (
        f"{f_or_neg1(m.disk_root_avail_gib):.1f}GB Available "
        f"({i_or_neg1(m.disk_root_used_pct)}% used)"
    )

    if not m.disk_data_expected:
        data_disk_text = "not configured"
    elif not m.disk_data_mounted:
        data_disk_text = "not mounted"
    else:
        data_disk_text = (
            f"{f_or_neg1(m.disk_data_avail_gib):.1f}GB Available "
            f"({i_or_neg1(m.disk_data_used_pct)}% used)"
        )

    return (
        f"ts: {m.ts_iso}\n"
        f"interval: {interval_sec:.3f}s  window: {window_sec}s\n\n"
        f"CPU稼働率： user {f_or_neg1(m.cpu_us):.1f}% "
        f"/ system {f_or_neg1(m.cpu_sy):.1f}% "
        f"/ wait {f_or_neg1(m.cpu_wa):.1f}%\n"
        f"内部温度：{f_or_neg1(m.tj_c):.1f}℃"
        f"（CPU {f_or_neg1(m.cpu_c):.1f}℃/ GPU {f_or_neg1(m.gpu_c):.1f}℃）\n\n"
        f"メモリ使用量： RAM {(-1.0 if ram_used_gb is None else ram_used_gb):.1f}GB "
        f"/ Total {(-1.0 if ram_total_gb is None else ram_total_gb):.1f}GB"
        f" / lfb {i_or_neg1(m.lfb_n)}x{i_or_neg1(m.lfb_mb)}MB"
        f" / SWAP {i_or_neg1(m.swap_used_mb)}MB"
        f" (cached {i_or_neg1(m.swap_cached_mb)}MB)\n\n"
        f"CPUクロック： max {i_or_neg1(m.cpu_fmax_mhz)} MHz "
        f"/ min {i_or_neg1(m.cpu_fmin_mhz)} MHz "
        f"/ cores {i_or_neg1(m.cpu_online_count)}/{i_or_neg1(m.cpu_core_count)}\n"
        f"CPUコア使用率(tegrastats)： avg {f_or_neg1(m.cpu_load_avg):.1f}% "
        f"/ max {f_or_neg1(m.cpu_load_max):.1f}% "
        f"/ min {f_or_neg1(m.cpu_load_min):.1f}%\n\n"
        f"GPU稼働率： {i_or_neg1(m.gr3d_pct)}%"
        f"（現在 {i_or_neg1(m.gpu_cur_mhz)} MHz"
        f" / 制限 {i_or_neg1(m.gpu_max_mhz)} MHz）\n\n"
        f"消費電力： input {i_or_neg1(m.p_in_mw)}mW "
        f"/ GPU_SOC {i_or_neg1(m.p_gpu_mw)}mW "
        f"/ CPU_CV {i_or_neg1(m.p_cpu_mw)}mW\n\n"
        f"サーマルスロットル（過去10min）：{i_or_neg1(m.thr_any_win_s)}sec"
        f"（CPU {i_or_neg1(m.thr_cpu_win_s)}sec / GPU {i_or_neg1(m.thr_gpu_win_s)}sec）\n"
        f"ディスク容量(root /)： {root_avail_text}\n"
        f"ディスク容量(data /mnt/nvme)： {data_disk_text}\n"
        f"ハードウェアステータス： state={int(m.state)} "
        f"reason={int(m.reason)} text={m.reason_text}\n"
    )


def write_godot_snapshot(path: str, m: Metrics) -> None:
    snap = make_godot_snapshot_dict(m)
    atomic_write(
        path, json.dumps(snap, ensure_ascii=False, separators=(",", ":")) + "\n"
    )


def rotate_file_numbered(path: str, keep: int) -> None:
    keep = max(1, keep)
    ensure_dir(os.path.dirname(path) or ".")

    oldest = f"{path}.{keep}"
    if os.path.exists(oldest):
        os.remove(oldest)

    for i in range(keep - 1, 0, -1):
        src = f"{path}.{i}"
        dst = f"{path}.{i + 1}"
        if os.path.exists(src):
            os.replace(src, dst)

    if os.path.exists(path):
        os.replace(path, f"{path}.1")


def append_jsonl_record(
    path: str,
    record: dict,
    rotate_bytes: int = 0,
    rotate_keep: int = 5,
) -> None:
    ensure_dir(os.path.dirname(path) or ".")

    if rotate_bytes > 0 and os.path.exists(path):
        if os.path.getsize(path) >= rotate_bytes:
            rotate_file_numbered(path, rotate_keep)

    with open(path, "a", encoding="utf-8") as f:
        f.write(json.dumps(record, ensure_ascii=False, separators=(",", ":")) + "\n")
