from __future__ import annotations

import os
import subprocess
import time

from .utils import read_int


def find_gpu_devfreq_paths() -> tuple[str | None, str | None]:
    base = "/sys/class/devfreq"
    if not os.path.isdir(base):
        return None, None

    candidates: list[tuple[str, str]] = []
    for d in os.listdir(base):
        p = os.path.join(base, d)
        if not os.path.isdir(p):
            continue

        if os.path.isfile(os.path.join(p, "cur_freq")) and os.path.isfile(
            os.path.join(p, "max_freq")
        ):
            name_path = os.path.join(p, "name")
            nm = ""

            if os.path.isfile(name_path):
                try:
                    nm = (
                        open(name_path, "r", encoding="utf-8", errors="ignore")
                        .read()
                        .strip()
                        .lower()
                    )
                except Exception:
                    nm = ""
            else:
                nm = os.path.basename(p).lower()

            candidates.append((p, nm))

    for p, nm in candidates:
        if ("gpu" in nm) or ("ga10b" in nm) or ("gv11b" in nm):
            return os.path.join(p, "cur_freq"), os.path.join(p, "max_freq")

    return None, None


class CPUStatReader:
    def __init__(self) -> None:
        self.prev: tuple[int, int, int, int, int, int, int, int] | None = None

    def read(self) -> tuple[float | None, float | None, float | None]:
        try:
            with open("/proc/stat", "r", encoding="utf-8") as f:
                line = f.readline()
        except Exception:
            return None, None, None

        if not line.startswith("cpu "):
            return None, None, None

        parts = line.split()
        nums = [int(x) for x in parts[1:]]
        while len(nums) < 8:
            nums.append(0)

        user, nice, system, idle, iowait, irq, softirq, steal = nums[:8]
        cur = (user, nice, system, idle, iowait, irq, softirq, steal)

        if self.prev is None:
            self.prev = cur
            return None, None, None

        prev = self.prev
        self.prev = cur

        diff = [c - p for c, p in zip(cur, prev)]
        total = sum(diff)
        if total <= 0:
            return None, None, None

        user_d, nice_d, system_d, idle_d, iowait_d, irq_d, softirq_d, steal_d = diff

        user_pct = (user_d + nice_d) * 100.0 / total
        system_pct = (system_d + irq_d + softirq_d) * 100.0 / total
        wait_pct = iowait_d * 100.0 / total

        return user_pct, system_pct, wait_pct


def read_root_disk_free(path: str = "/") -> tuple[float | None, int | None]:
    try:
        st = os.statvfs(path)
    except Exception:
        return None, None

    block_size = st.f_frsize if st.f_frsize > 0 else st.f_bsize

    free_bytes = st.f_bavail * block_size
    used_bytes = (st.f_blocks - st.f_bfree) * block_size

    denom = used_bytes + free_bytes
    if denom <= 0:
        return None, None

    free_gib = free_bytes / (1024**3)
    used_pct = int((used_bytes * 100 + denom - 1) // denom)

    return free_gib, used_pct


def start_tegrastats(interval_ms: int) -> subprocess.Popen[str]:
    cmd = ["tegrastats", "--interval", str(interval_ms)]
    try:
        proc = subprocess.Popen(
            cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            bufsize=1,
        )
        t0 = time.monotonic()
        while True:
            if proc.poll() is not None:
                break
            if (time.monotonic() - t0) > 0.25:
                return proc
    except FileNotFoundError:
        raise RuntimeError("tegrastats not found in PATH")

    proc2 = subprocess.Popen(
        ["tegrastats"],
        stdout=subprocess.PIPE,
        stderr=subprocess.DEVNULL,
        text=True,
        bufsize=1,
    )
    return proc2


def read_gpu_freq_mhz(
    gpu_cur_path: str | None,
    gpu_limit_path: str | None,
) -> tuple[int | None, int | None]:
    gpu_cur_hz = read_int(gpu_cur_path)
    gpu_limit_hz = read_int(gpu_limit_path)

    gpu_cur_mhz = int(gpu_cur_hz / 1_000_000) if gpu_cur_hz else None
    gpu_limit_mhz = int(gpu_limit_hz / 1_000_000) if gpu_limit_hz else None

    return gpu_cur_mhz, gpu_limit_mhz


def read_cpu_freq_each_mhz() -> tuple[int, int, list[int | None]]:
    base = "/sys/devices/system/cpu"

    cpu_ids: list[int] = []

    try:
        for name in os.listdir(base):
            if not name.startswith("cpu"):
                continue

            n = name[3:]
            if not n.isdigit():
                continue

            cpu_ids.append(int(n))
    except Exception:
        return 0, 0, []

    if not cpu_ids:
        return 0, 0, []

    max_cpu_id = max(cpu_ids)
    freq_each_mhz: list[int | None] = [None] * (max_cpu_id + 1)

    online_count = 0

    for cpu_id in sorted(cpu_ids):
        cpu_dir = os.path.join(base, f"cpu{cpu_id}")
        online_path = os.path.join(cpu_dir, "online")

        online = True

        if os.path.isfile(online_path):
            v = read_int(online_path)
            online = v == 1

        if not online:
            freq_each_mhz[cpu_id] = None
            continue

        online_count += 1

        freq_path = os.path.join(cpu_dir, "cpufreq", "scaling_cur_freq")
        freq_khz = read_int(freq_path)

        if freq_khz is None:
            freq_each_mhz[cpu_id] = None
        else:
            freq_each_mhz[cpu_id] = int(freq_khz / 1000)

    core_count = len(cpu_ids)

    return core_count, online_count, freq_each_mhz


def read_mounted_disk_free(path: str) -> tuple[float | None, int | None, bool]:
    if not os.path.ismount(path):
        return None, None, False

    free_gib, used_pct = read_root_disk_free(path)
    return free_gib, used_pct, True


def is_mount_configured_in_fstab(
    mount_point: str,
    fstab_path: str = "/etc/fstab",
) -> bool:
    try:
        with open(fstab_path, "r", encoding="utf-8") as f:
            lines = f.readlines()
    except Exception:
        return False

    for line in lines:
        line = line.strip()

        if not line or line.startswith("#"):
            continue

        parts = line.split()
        if len(parts) < 2:
            continue

        if parts[1] == mount_point:
            return True

    return False
