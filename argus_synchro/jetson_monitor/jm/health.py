from __future__ import annotations

from .models import HealthConfig, HealthState, Metrics, ReasonId


def judge_health(
    m: Metrics,
    cpu_design_max_mhz: int | None,
    cfg: HealthConfig,
) -> tuple[int, int, str]:
    if m.thr_any_win_s is not None and m.thr_any_win_s >= cfg.thr_any_min_s:
        cpu_s = m.thr_cpu_win_s or 0
        gpu_s = m.thr_gpu_win_s or 0
        if cpu_s > 0 and gpu_s > 0:
            return (HealthState.THROTTLE, ReasonId.THR_10MIN, "throttle detected (last window): CPU+GPU")
        if cpu_s > 0:
            return (HealthState.THROTTLE, ReasonId.THR_10MIN, "throttle detected (last window): CPU")
        if gpu_s > 0:
            return (HealthState.THROTTLE, ReasonId.THR_10MIN, "throttle detected (last window): GPU")
        return (HealthState.THROTTLE, ReasonId.THR_10MIN, "throttle detected (last window)")

    if m.tj_c is not None and m.tj_c >= cfg.tj_critical:
        cpu_drop = (
            m.cpu_fmax_mhz is not None
            and cpu_design_max_mhz is not None
            and m.cpu_fmax_mhz < cpu_design_max_mhz * cfg.cpu_throttle_drop_ratio
        )
        gpu_drop = (
            m.gpu_cur_mhz is not None
            and m.gpu_max_mhz is not None
            and m.gpu_cur_mhz < m.gpu_max_mhz * cfg.gpu_throttle_drop_ratio
        )
        if cpu_drop or gpu_drop:
            return (HealthState.THROTTLE, ReasonId.THR_TJ_FREQ, "high temp + freq drop")

    missing = []
    if m.tj_c is None:
        missing.append("tj")
    if m.cpu_fmax_mhz is None:
        missing.append("cpu_freq")
    if m.gr3d_pct is None:
        missing.append("gr3d")
    if m.gpu_cur_mhz is None or m.gpu_max_mhz is None:
        missing.append("gpu_freq")
    if missing:
        return (HealthState.WARN, ReasonId.WARN_METRIC_MISS, "metrics missing: " + "/".join(missing))

    if m.tj_c is not None and m.tj_c >= cfg.tj_warn:
        return (HealthState.WARN, ReasonId.WARN_TJ_HIGH, "Tj high")

    high_load = False
    if m.cpu_load_avg is not None and m.cpu_load_avg >= cfg.cpu_load_high:
        high_load = True
    if m.gr3d_pct is not None and m.gr3d_pct >= cfg.gr3d_high:
        high_load = True
    if m.tj_c is not None and m.tj_c >= cfg.tj_warn_low and high_load:
        return (HealthState.WARN, ReasonId.WARN_HOT_LOAD, "warm + high load")

    cpu_soft_drop = (
        m.cpu_fmax_mhz is not None
        and cpu_design_max_mhz is not None
        and m.cpu_fmax_mhz < cpu_design_max_mhz * cfg.cpu_warn_drop_ratio
    )
    gpu_soft_drop = (
        m.gpu_cur_mhz is not None
        and m.gpu_max_mhz is not None
        and m.gpu_cur_mhz < m.gpu_max_mhz * cfg.gpu_warn_drop_ratio
    )
    if cpu_soft_drop or gpu_soft_drop:
        return (HealthState.WARN, ReasonId.WARN_FREQ_HEADROOM, "freq below expected")

    return (HealthState.OK, ReasonId.OK, "normal")
