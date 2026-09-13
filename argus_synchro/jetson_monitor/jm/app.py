from __future__ import annotations

import os
import threading
import time
from collections import deque
from collections.abc import Callable
from dataclasses import dataclass

from argus_synchro.config.app_config import AppConfig
from argus_synchro.process.operation_mode import OPERATION_MODE as OPM
from argus_synchro.shared_app_config import SharedAppConfig

from .health import judge_health
from .jm_config import AppSettings, load_settings
from .models import HealthState, Metrics, Parsed, ReasonId
from .parser_tegrastats import parse_tegrastats_line
from .readers import (
    CPUStatReader,
    find_gpu_devfreq_paths,
    is_mount_configured_in_fstab,
    read_cpu_freq_each_mhz,
    read_gpu_freq_mhz,
    read_mounted_disk_free,
    read_root_disk_free,
    start_tegrastats,
)
from .systemd_notify import notify_ready, notify_watchdog
from .utils import atomic_write, ensure_dir, iso_now
from .writers import (
    append_jsonl_record,
    format_text_for_ui,
    make_log_record_dict,
    make_ts_named_jsonl_path,
    resolve_metrics_path,
    write_godot_snapshot,
)

DATA_DISK_MOUNT = "/mnt/nvme"


@dataclass(frozen=True, slots=True)
class JetsonMonitorArgs:
    settings_ini: str
    interval: float
    write_interval: float
    window_sec: int
    metrics_path: str | None
    log_jsonl_ts_name: bool
    jsonl_rotate_mb: int
    jsonl_rotate_keep: int
    disk_guard_gib: float


def build_metrics(
    now_iso: str,
    ts_parsed: Parsed,
    cpu_us: float | None,
    cpu_sy: float | None,
    cpu_wa: float | None,
    cpu_core_count: int | None,
    cpu_online_count: int | None,
    cpu_freq_each_mhz: list[int | None] | None,
    gpu_cur_mhz: int | None,
    gpu_limit_mhz: int | None,
    disk_root_avail_gib: float | None,
    disk_root_used_pct: int | None,
    disk_data_expected: bool,
    disk_data_avail_gib: float | None,
    disk_data_used_pct: int | None,
    disk_data_mounted: bool,
    thr_any_s: int,
    thr_cpu_s: int,
    thr_gpu_s: int,
    cpu_th: int | None,
    gpu_th: int | None,
) -> Metrics:
    return Metrics(
        ts_iso=now_iso,
        tj_c=ts_parsed.tj_c,
        cpu_c=ts_parsed.cpu_c,
        gpu_c=ts_parsed.gpu_c,
        ram_used_mb=ts_parsed.ram_used_mb,
        ram_total_mb=ts_parsed.ram_total_mb,
        lfb_n=ts_parsed.lfb_n,
        lfb_mb=ts_parsed.lfb_mb,
        swap_used_mb=ts_parsed.swap_used_mb,
        swap_total_mb=ts_parsed.swap_total_mb,
        swap_cached_mb=ts_parsed.swap_cached_mb,
        cpu_core_count=cpu_core_count,
        cpu_online_count=cpu_online_count,
        cpu_freq_each_mhz=cpu_freq_each_mhz,
        cpu_fmax_mhz=ts_parsed.cpu_freq_max_mhz,
        cpu_fmin_mhz=ts_parsed.cpu_freq_min_mhz,
        cpu_load_avg=ts_parsed.cpu_load_avg,
        cpu_load_min=ts_parsed.cpu_load_min,
        cpu_load_max=ts_parsed.cpu_load_max,
        cpu_us=cpu_us,
        cpu_sy=cpu_sy,
        cpu_wa=cpu_wa,
        gr3d_pct=ts_parsed.gr3d_pct,
        gpu_cur_mhz=gpu_cur_mhz,
        gpu_max_mhz=gpu_limit_mhz,
        disk_root_avail_gib=disk_root_avail_gib,
        disk_root_used_pct=disk_root_used_pct,
        disk_data_expected=disk_data_expected,
        disk_data_mounted=disk_data_mounted,
        disk_data_avail_gib=disk_data_avail_gib,
        disk_data_used_pct=disk_data_used_pct,
        p_in_mw=ts_parsed.p_in_mw,
        p_gpu_mw=ts_parsed.p_gpu_mw,
        p_cpu_mw=ts_parsed.p_cpu_mw,
        thr_any_win_s=thr_any_s,
        thr_cpu_win_s=thr_cpu_s,
        thr_gpu_win_s=thr_gpu_s,
        cpu_th=cpu_th,
        gpu_th=gpu_th,
        state=0,
        reason=0,
        reason_text="",
    )


def detect_throttle_flags(
    ts_parsed: Parsed,
    cpu_design_max: int | None,
    gpu_cur_mhz: int | None,
    gpu_limit_mhz: int | None,
    st: AppSettings,
) -> tuple[int, int]:
    tj_hot = ts_parsed.tj_c is not None and ts_parsed.tj_c >= st.tj_th_c

    cpu_th = 0
    if (
        tj_hot
        and ts_parsed.cpu_load_avg is not None
        and ts_parsed.cpu_freq_max_mhz is not None
        and cpu_design_max is not None
        and ts_parsed.cpu_load_avg >= st.cpu_load_th
        and ts_parsed.cpu_freq_max_mhz < int(cpu_design_max * st.cpu_freq_drop_ratio)
    ):
        cpu_th = 1

    gpu_th = 0
    if (
        tj_hot
        and ts_parsed.gr3d_pct is not None
        and gpu_cur_mhz is not None
        and gpu_limit_mhz is not None
        and ts_parsed.gr3d_pct >= st.gpu_load_th
        and gpu_cur_mhz < int(gpu_limit_mhz * st.gpu_freq_drop_ratio)
    ):
        gpu_th = 1

    return cpu_th, gpu_th


def update_throttle_windows(
    cpu_win: deque[int],
    gpu_win: deque[int],
    any_win: deque[int],
    cpu_th: int,
    gpu_th: int,
    interval_sec: float,
) -> tuple[int, int, int]:
    any_th = 1 if (cpu_th or gpu_th) else 0

    cpu_win.append(cpu_th)
    gpu_win.append(gpu_th)
    any_win.append(any_th)

    thr_cpu_s = int(sum(cpu_win) * interval_sec)
    thr_gpu_s = int(sum(gpu_win) * interval_sec)
    thr_any_s = int(sum(any_win) * interval_sec)

    return thr_any_s, thr_cpu_s, thr_gpu_s


def apply_disk_guard(m: Metrics, disk_guard_gib: float) -> tuple[Metrics, bool]:
    data_disk_bad = False
    reason_text = ""

    if disk_guard_gib > 0 and m.disk_data_expected:
        if not m.disk_data_mounted:
            data_disk_bad = True
            reason_text = "data disk expected but not mounted: /mnt/nvme"
        elif m.disk_data_avail_gib is None:
            data_disk_bad = True
            reason_text = "data disk unavailable: /mnt/nvme"
        elif m.disk_data_avail_gib < disk_guard_gib:
            data_disk_bad = True
            reason_text = (
                f"data disk low: avail={m.disk_data_avail_gib:.2f}GiB "
                f"< guard={disk_guard_gib:.2f}GiB"
            )

    if not data_disk_bad:
        return m, True

    m2 = Metrics(
        **{
            **m.__dict__,
            "state": int(HealthState.WARN),
            "reason": int(ReasonId.WARN_DISK_LOW),
            "reason_text": reason_text,
        }
    )

    return m2, False


def write_outputs(
    m: Metrics,
    metrics_path: str,
    interval_sec: float,
    window_sec: int,
    godot_json_path: str | None,
    jsonl_path: str | None,
    jsonl_enabled: bool,
    jsonl_rotate_bytes: int,
    jsonl_rotate_keep: int,
) -> None:
    if godot_json_path:
        write_godot_snapshot(godot_json_path, m)

    if jsonl_enabled and jsonl_path:
        rec = make_log_record_dict(m)
        append_jsonl_record(
            jsonl_path,
            rec,
            rotate_bytes=jsonl_rotate_bytes,
            rotate_keep=jsonl_rotate_keep,
        )

    txt = format_text_for_ui(m, interval_sec=interval_sec, window_sec=window_sec)
    atomic_write(metrics_path, txt)


def main(
    sac: SharedAppConfig,
    diag_func: Callable[[Metrics], int],
    stop_event: threading.Event,
) -> None:
    app_config: AppConfig = sac.read()
    _last_updated: int = sac.last_updated

    args = JetsonMonitorArgs(
        settings_ini=app_config.jetson_monitor.settings_ini,
        interval=app_config.jetson_monitor.interval,
        write_interval=app_config.jetson_monitor.write_interval,
        window_sec=app_config.jetson_monitor.window_sec,
        metrics_path=app_config.jetson_monitor.metrics_path,
        log_jsonl_ts_name=app_config.jetson_monitor.log_jsonl_ts_name,
        jsonl_rotate_mb=app_config.jetson_monitor.jsonl_rotate_mb,
        jsonl_rotate_keep=app_config.jetson_monitor.jsonl_rotate_keep,
        disk_guard_gib=app_config.jetson_monitor.disk_guard_gib,
    )

    st = load_settings(args.settings_ini)

    interval_sec = float(args.interval)
    if interval_sec <= 0:
        raise SystemExit("--interval must be > 0")

    write_interval_sec = float(args.write_interval)
    if write_interval_sec <= 0:
        raise SystemExit("--write-interval must be > 0")

    window_sec = int(args.window_sec)
    if window_sec <= 0:
        raise SystemExit("--window-sec must be > 0")

    interval_ms: int = round(interval_sec * 1000.0)
    window_n: int = max(1, int(window_sec / interval_sec))

    ensure_dir(os.path.dirname(st.raw_out) or ".")
    ensure_dir(os.path.dirname(st.default_metrics_out) or ".")

    if st.log_jsonl_out:
        ensure_dir(os.path.dirname(st.log_jsonl_out) or ".")

    if st.godot_json_out:
        ensure_dir(os.path.dirname(st.godot_json_out) or ".")

    start_ts = iso_now()

    metrics_path = resolve_metrics_path(st.default_metrics_out, args.metrics_path)

    jsonl_path = st.log_jsonl_out
    if jsonl_path and args.log_jsonl_ts_name:
        jsonl_path = make_ts_named_jsonl_path(jsonl_path, start_ts)

    jsonl_rotate_bytes = max(0, int(args.jsonl_rotate_mb) * 1024 * 1024)
    jsonl_rotate_keep = max(1, int(args.jsonl_rotate_keep))

    gpu_cur_path, gpu_limit_path = find_gpu_devfreq_paths()
    disk_data_expected = is_mount_configured_in_fstab(DATA_DISK_MOUNT)

    cpu_win: deque[int] = deque([0] * window_n, maxlen=window_n)
    gpu_win: deque[int] = deque([0] * window_n, maxlen=window_n)
    any_win: deque[int] = deque([0] * window_n, maxlen=window_n)

    cpu_design_max: int | None = None
    cpu_reader = CPUStatReader()

    proc = start_tegrastats(interval_ms)
    assert proc.stdout is not None

    last_emit = 0.0
    jsonl_enabled = True

    notify_ready()

    try:
        while not stop_event.is_set():
            if sac.last_updated > _last_updated:
                app_config = sac.read()
                _last_updated = sac.last_updated

            raw_line = proc.stdout.readline()
            if not raw_line:
                break

            raw = raw_line.strip()
            if not raw:
                continue

            now_mono = time.monotonic()
            if last_emit != 0.0 and (now_mono - last_emit) < write_interval_sec:
                is_required_write = False
            else:
                is_required_write = True
                last_emit = now_mono
                atomic_write(st.raw_out, raw + "\n")

            ts_parsed: Parsed = parse_tegrastats_line(raw)

            if cpu_design_max is None and ts_parsed.cpu_freq_max_mhz is not None:
                cpu_design_max = ts_parsed.cpu_freq_max_mhz

            gpu_cur_mhz, gpu_limit_mhz = read_gpu_freq_mhz(gpu_cur_path, gpu_limit_path)
            cpu_core_count, cpu_online_count, cpu_freq_each_mhz = (
                read_cpu_freq_each_mhz()
            )

            cpu_us, cpu_sy, cpu_wa = cpu_reader.read()

            disk_root_avail_gib, disk_root_used_pct = read_root_disk_free("/")
            disk_data_avail_gib, disk_data_used_pct, disk_data_mounted = (
                read_mounted_disk_free(DATA_DISK_MOUNT)
            )

            cpu_th, gpu_th = detect_throttle_flags(
                ts_parsed=ts_parsed,
                cpu_design_max=cpu_design_max,
                gpu_cur_mhz=gpu_cur_mhz,
                gpu_limit_mhz=gpu_limit_mhz,
                st=st,
            )

            thr_any_s, thr_cpu_s, thr_gpu_s = update_throttle_windows(
                cpu_win=cpu_win,
                gpu_win=gpu_win,
                any_win=any_win,
                cpu_th=cpu_th,
                gpu_th=gpu_th,
                interval_sec=interval_sec,
            )

            tmp = build_metrics(
                now_iso=iso_now(),
                ts_parsed=ts_parsed,
                cpu_us=cpu_us,
                cpu_sy=cpu_sy,
                cpu_wa=cpu_wa,
                cpu_core_count=cpu_core_count,
                cpu_online_count=cpu_online_count,
                cpu_freq_each_mhz=cpu_freq_each_mhz,
                gpu_cur_mhz=gpu_cur_mhz,
                gpu_limit_mhz=gpu_limit_mhz,
                disk_root_avail_gib=disk_root_avail_gib,
                disk_root_used_pct=disk_root_used_pct,
                disk_data_expected=disk_data_expected,
                disk_data_avail_gib=disk_data_avail_gib,
                disk_data_used_pct=disk_data_used_pct,
                disk_data_mounted=disk_data_mounted,
                thr_any_s=thr_any_s,
                thr_cpu_s=thr_cpu_s,
                thr_gpu_s=thr_gpu_s,
                cpu_th=cpu_th,
                gpu_th=gpu_th,
            )

            state_enum, reason_id, reason_text = judge_health(
                tmp, cpu_design_max, st.health
            )
            m = Metrics(
                **{
                    **tmp.__dict__,
                    "state": int(state_enum),
                    "reason": int(reason_id),
                    "reason_text": reason_text,
                }
            )

            m, can_write_jsonl = apply_disk_guard(
                m=m,
                disk_guard_gib=float(args.disk_guard_gib),
            )

            if not can_write_jsonl:
                jsonl_enabled = False

            if is_required_write:
                write_outputs(
                    m=m,
                    metrics_path=metrics_path,
                    interval_sec=interval_sec,
                    window_sec=window_sec,
                    godot_json_path=st.godot_json_out,
                    jsonl_path=jsonl_path,
                    jsonl_enabled=jsonl_enabled,
                    jsonl_rotate_bytes=jsonl_rotate_bytes,
                    jsonl_rotate_keep=jsonl_rotate_keep,
                )

            if app_config.General.operation_mode == OPM.SCRUT:
                # NOTE: 周辺監視モード時のみ診断実施する
                diag_func(m)

            notify_watchdog()

    except KeyboardInterrupt:
        pass

    finally:
        if proc.poll() is None:
            proc.terminate()
            try:
                proc.wait(timeout=1.0)
            except Exception:
                proc.kill()
