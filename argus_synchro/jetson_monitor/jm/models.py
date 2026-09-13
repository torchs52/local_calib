from __future__ import annotations

from dataclasses import dataclass
from enum import IntEnum


class HealthState(IntEnum):
    OK = 0
    WARN = 1
    THROTTLE = 2


class ReasonId(IntEnum):
    OK = 0
    WARN_TJ_HIGH = 1
    WARN_HOT_LOAD = 2
    WARN_FREQ_HEADROOM = 3
    WARN_METRIC_MISS = 4
    THR_10MIN = 5
    THR_TJ_FREQ = 6
    WARN_DISK_LOW = 7


@dataclass
class Parsed:
    ram_used_mb: int | None = None
    ram_total_mb: int | None = None
    lfb_n: int | None = None
    lfb_mb: int | None = None
    swap_used_mb: int | None = None
    swap_total_mb: int | None = None
    swap_cached_mb: int | None = None

    cpu_load_avg: float | None = None
    cpu_load_min: float | None = None
    cpu_load_max: float | None = None
    cpu_freq_min_mhz: int | None = None
    cpu_freq_max_mhz: int | None = None

    cpu_core_count: int | None = None
    cpu_online_count: int | None = None
    cpu_freq_each_mhz: list[int | None] | None = None

    gr3d_pct: int | None = None

    tj_c: float | None = None
    cpu_c: float | None = None
    gpu_c: float | None = None

    p_in_mw: int | None = None
    p_gpu_mw: int | None = None
    p_cpu_mw: int | None = None


@dataclass(frozen=True)
class Metrics:
    ts_iso: str

    tj_c: float | None
    cpu_c: float | None
    gpu_c: float | None

    ram_used_mb: int | None
    ram_total_mb: int | None
    lfb_n: int | None
    lfb_mb: int | None
    swap_used_mb: int | None
    swap_total_mb: int | None
    swap_cached_mb: int | None

    cpu_core_count: int | None
    cpu_online_count: int | None
    cpu_freq_each_mhz: list[int | None] | None

    cpu_fmax_mhz: int | None
    cpu_fmin_mhz: int | None
    cpu_load_avg: float | None
    cpu_load_min: float | None
    cpu_load_max: float | None

    cpu_us: float | None
    cpu_sy: float | None
    cpu_wa: float | None

    gr3d_pct: int | None
    gpu_cur_mhz: int | None
    gpu_max_mhz: int | None  # 内部名は既存維持。出力時は freq_limit_mhz として出す。

    p_in_mw: int | None
    p_gpu_mw: int | None
    p_cpu_mw: int | None

    disk_root_avail_gib: float | None
    disk_root_used_pct: int | None

    disk_data_expected: bool
    disk_data_mounted: bool
    disk_data_avail_gib: float | None
    disk_data_used_pct: int | None

    thr_any_win_s: int | None
    thr_cpu_win_s: int | None
    thr_gpu_win_s: int | None

    cpu_th: int | None
    gpu_th: int | None

    state: int
    reason: int
    reason_text: str


@dataclass(frozen=True)
class HealthConfig:
    tj_warn_low: float
    tj_warn: float
    tj_critical: float

    thr_any_min_s: int

    cpu_throttle_drop_ratio: float
    gpu_throttle_drop_ratio: float

    cpu_warn_drop_ratio: float
    gpu_warn_drop_ratio: float

    cpu_load_high: float
    gr3d_high: float
