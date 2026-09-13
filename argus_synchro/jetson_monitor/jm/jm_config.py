from __future__ import annotations

import configparser
import os
from dataclasses import dataclass

from .models import HealthConfig

DEFAULT_SETTINGS_INI = "/etc/jetson-monitor/normal.ini"


@dataclass(frozen=True)
class AppSettings:
    raw_out: str
    default_metrics_out: str
    log_jsonl_out: str | None
    godot_json_out: str | None

    tj_th_c: float
    cpu_load_th: float
    gpu_load_th: float
    cpu_freq_drop_ratio: float
    gpu_freq_drop_ratio: float

    health: HealthConfig


def load_settings(path: str | None) -> AppSettings:
    ini_path = path or DEFAULT_SETTINGS_INI

    if not os.path.isfile(ini_path):
        raise FileNotFoundError(f"settings.ini not found: {ini_path}")

    cp = configparser.ConfigParser()
    cp.read(ini_path, encoding="utf-8")

    raw_out = cp.get("paths", "raw_out")
    default_metrics_out = cp.get("paths", "default_metrics_out")

    log_jsonl_out = cp.get("paths", "log_jsonl_out", fallback=None)
    godot_json_out = cp.get("paths", "godot_json_out", fallback=None)

    tj_th_c = cp.getfloat("throttle", "tj_th_c")
    cpu_load_th = cp.getfloat("throttle", "cpu_load_th")
    gpu_load_th = cp.getfloat("throttle", "gpu_load_th")
    cpu_freq_drop_ratio = cp.getfloat("throttle", "cpu_freq_drop_ratio")
    gpu_freq_drop_ratio = cp.getfloat("throttle", "gpu_freq_drop_ratio")

    health = HealthConfig(
        tj_warn_low=cp.getfloat("health", "tj_warn_low"),
        tj_warn=cp.getfloat("health", "tj_warn"),
        tj_critical=cp.getfloat("health", "tj_critical"),
        thr_any_min_s=cp.getint("health", "thr_any_min_s"),
        cpu_throttle_drop_ratio=cp.getfloat("health", "cpu_throttle_drop_ratio"),
        gpu_throttle_drop_ratio=cp.getfloat("health", "gpu_throttle_drop_ratio"),
        cpu_warn_drop_ratio=cp.getfloat("health", "cpu_warn_drop_ratio"),
        gpu_warn_drop_ratio=cp.getfloat("health", "gpu_warn_drop_ratio"),
        cpu_load_high=cp.getfloat("health", "cpu_load_high"),
        gr3d_high=cp.getfloat("health", "gr3d_high"),
    )

    return AppSettings(
        raw_out=raw_out,
        default_metrics_out=default_metrics_out,
        log_jsonl_out=log_jsonl_out,
        godot_json_out=godot_json_out,
        tj_th_c=tj_th_c,
        cpu_load_th=cpu_load_th,
        gpu_load_th=gpu_load_th,
        cpu_freq_drop_ratio=cpu_freq_drop_ratio,
        gpu_freq_drop_ratio=gpu_freq_drop_ratio,
        health=health,
    )
