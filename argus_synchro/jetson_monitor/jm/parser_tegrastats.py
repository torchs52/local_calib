from __future__ import annotations

import re

from .models import Parsed

INVALID_TEMP_C = -100.0

RE_RAM = re.compile(r"RAM\s+(\d+)/(\d+)MB")
RE_LFB = re.compile(r"\(lfb\s+(\d+)x(\d+)MB\)")
RE_SWAP = re.compile(r"SWAP\s+(\d+)/(\d+)MB\s+\(cached\s+(\d+)MB\)")
RE_CPU_BR = re.compile(r"CPU\s*\[([^\]]+)\]")
RE_GR3D = re.compile(r"GR3D_FREQ\s+(\d+)%")

RE_TJ = re.compile(r"[tT][jJ]@(-?\d+(?:\.\d+)?)C")
RE_CPU_TEMP = re.compile(r"[cC][pP][uU]@(-?\d+(?:\.\d+)?)C")
RE_GPU_TEMP = re.compile(r"[gG][pP][uU]@(-?\d+(?:\.\d+)?)C")
RE_ANY_TEMP = re.compile(r"([A-Za-z0-9_\-]+)@(-?\d+(?:\.\d+)?)C")

RE_POWER = re.compile(r"\b(VDD_[A-Za-z0-9_]+|VIN_[A-Za-z0-9_]+)\s+(\d+)mW/(\d+)mW")


def parse_tegrastats_line(line: str) -> Parsed:
    s = Parsed()

    m = RE_RAM.search(line)
    if m:
        s.ram_used_mb = int(m.group(1))
        s.ram_total_mb = int(m.group(2))

    m = RE_LFB.search(line)
    if m:
        s.lfb_n = int(m.group(1))
        s.lfb_mb = int(m.group(2))

    m = RE_SWAP.search(line)
    if m:
        s.swap_used_mb = int(m.group(1))
        s.swap_total_mb = int(m.group(2))
        s.swap_cached_mb = int(m.group(3))

    m = RE_GR3D.search(line)
    if m:
        s.gr3d_pct = int(m.group(1))

    m = RE_CPU_BR.search(line)
    if m:
        inner = m.group(1)
        parts = [p.strip() for p in inner.split(",") if p.strip()]

        loads: list[float] = []
        freqs: list[int] = []
        freq_each_mhz: list[int | None] = []

        for p in parts:
            mm = re.search(r"(\d+)%@(\d+)", p)
            if mm:
                loads.append(float(mm.group(1)))
                freq = int(mm.group(2))
                freqs.append(freq)
                freq_each_mhz.append(freq)
            else:
                freq_each_mhz.append(None)

        if parts:
            s.cpu_core_count = len(parts)
            s.cpu_online_count = len([v for v in freq_each_mhz if v is not None])
            s.cpu_freq_each_mhz = freq_each_mhz

        if loads:
            s.cpu_load_avg = sum(loads) / len(loads)
            s.cpu_load_min = min(loads)
            s.cpu_load_max = max(loads)

        if freqs:
            s.cpu_freq_min_mhz = min(freqs)
            s.cpu_freq_max_mhz = max(freqs)

    m = RE_CPU_TEMP.search(line)
    if m:
        try:
            s.cpu_c = float(m.group(1))
        except ValueError:
            pass

    m = RE_GPU_TEMP.search(line)
    if m:
        try:
            s.gpu_c = float(m.group(1))
        except ValueError:
            pass

    m = RE_TJ.search(line)
    if m:
        try:
            s.tj_c = float(m.group(1))
        except ValueError:
            pass

    temps_all: dict[str, float] = {}
    for name, v in RE_ANY_TEMP.findall(line):
        try:
            t = float(v)
        except ValueError:
            continue

        if t <= INVALID_TEMP_C:
            continue

        temps_all[name.strip().lower()] = t

    if s.gpu_c is None and "gpu" in temps_all:
        s.gpu_c = temps_all["gpu"]

    if s.cpu_c is None and "cpu" in temps_all:
        s.cpu_c = temps_all["cpu"]

    if s.tj_c is None and temps_all:
        s.tj_c = max(temps_all.values())

    for name, cur_mw, _avg_mw in RE_POWER.findall(line):
        key = name.strip().upper()
        val = int(cur_mw)

        if key == "VIN_SYS_5V0":
            s.p_in_mw = val
        elif key == "VDD_GPU_SOC":
            s.p_gpu_mw = val
        elif key == "VDD_CPU_CV":
            s.p_cpu_mw = val

    return s