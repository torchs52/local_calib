from __future__ import annotations

import re
import subprocess
import threading
import time
from _thread import LockType
from collections.abc import Callable
from contextlib import suppress

from argus_synchro.shared_errors import SharedErrors, StateErrorIndex


class CommandDaemon:
    def __init__(
        self,
        *,
        name: str,
        cmd: list[str],
        parser: Callable[[str], None] | None = None,
        timeout_sec: float = 2.0,
    ) -> None:
        self._name: str = name
        self._cmd: list[str] = cmd
        self._parser: Callable[[str], None] | None = parser
        self._timeout = float(timeout_sec)

        self._p: subprocess.Popen[str] | None = None
        self._th: threading.Thread | None = None
        self._stop = threading.Event()

        self._lock: LockType = threading.Lock()

    def start(self) -> None:
        if self._th and self._th.is_alive():
            return
        self._p = self._spawn()
        self._stop.clear()
        self._th = threading.Thread(
            target=self._run, name=f"{self._name}-daemon-reader", daemon=True
        )
        self._th.start()

    def stop(self) -> None:
        self._stop.set()
        self._terminate()

    def _run(self) -> None:
        last_line_at: float = time.monotonic()
        while not self._stop.is_set():
            line: str = self._read_line()
            if line:
                last_line_at = time.monotonic()
                self._handle_line(line)
            elif (time.monotonic() - last_line_at) > self._timeout:
                self._restart()
            time.sleep(0.02)  # 過剰ループを抑制

        self._terminate()

    def _read_line(self) -> str:
        p: subprocess.Popen[str] | None = self._p
        if p is None or p.stdout is None:
            return ""
        try:
            return p.stdout.readline()
        except Exception:
            return ""

    def _handle_line(self, line: str) -> None:
        if self._parser:
            self._parser(line)
        self._p = None

    def _restart(self) -> None:
        self._terminate()
        self._p = self._spawn()

    def _spawn(self) -> subprocess.Popen[str]:
        return subprocess.Popen(
            self._cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,
        )

    def _terminate(self) -> None:
        if self._p and self._p.poll() is None:
            with suppress(Exception):
                self._p.terminate()


RAM_RE = re.compile(r"RAM\s+(\d+)/(\d+)MB")
GPU_RE = re.compile(r"GR3D_FREQ\s+(\d+)%")
CPU_TEMP_RE = re.compile(r"\bcpu@(\d+(?:\.\d+)?)C\b", re.IGNORECASE)
SOC0_TEMP_RE = re.compile(r"\bsoc0@(\d+(?:\.\d+)?)C\b", re.IGNORECASE)
SOC1_TEMP_RE = re.compile(r"\soc1@(\d+(?:\.\d+)?)C\b", re.IGNORECASE)
SOC2_TEMP_RE = re.compile(r"\bsoc2@(\d+(?:\.\d+)?)C\b", re.IGNORECASE)
TJ_TEMP_RE = re.compile(r"\btj@(\d+(?:\.\d+)?)C\b", re.IGNORECASE)
MB_TO_GB: float = 1.0 / 1024.0

def parse_tegrastats(line: str, ser: SharedErrors) -> None:
    m: re.Match[str] | None = RAM_RE.search(line)
    if m:
        used = float(m.group(1))
        total = float(m.group(2))
        now = time.perf_counter()
        ser.state_errors_A_C[StateErrorIndex.OUT_OF_MEMORY].errors_diagnosis(
            used * MB_TO_GB,
            total * MB_TO_GB,
            now,
        )

    g: re.Match[str] | None = GPU_RE.search(line)
    if g:
        util = float(g.group(1))
        ser.state_errors_A_C[StateErrorIndex.GPU_PERFORMANCE_DEGRADED].errors_diagnosis(
            util
        )

    cpu_temperature: re.Match[str] | None = CPU_TEMP_RE.search(line)
    soc0_temperature: re.Match[str] | None = SOC0_TEMP_RE.search(line)
    soc1_temperature: re.Match[str] | None = SOC1_TEMP_RE.search(line)
    soc2_temperature: re.Match[str] | None = SOC2_TEMP_RE.search(line)
    tj_temperature: re.Match[str] | None = TJ_TEMP_RE.search(line)
    if (
        cpu_temperature
        and soc0_temperature
        and soc1_temperature
        and soc2_temperature
        and tj_temperature
    ):
        cpu_temp = float(cpu_temperature.group(1))
        soc0_temp = float(soc0_temperature.group(1))
        soc1_temp = float(soc1_temperature.group(1))
        soc2_temp = float(soc2_temperature.group(1))
        tg_temp = float(tj_temperature.group(1))
        ser.state_errors_A_C[
            StateErrorIndex.INTERNAL_TEMPERATURE_RISE
        ].errors_diagnosis(cpu_temp, soc0_temp, soc1_temp, soc2_temp, tg_temp)
        ser.state_errors_A_C[
            StateErrorIndex.TEMPERATURE_SENSOR_ABNORMAL
        ].errors_diagnosis(cpu_temp, soc0_temp, soc1_temp, soc2_temp, tg_temp)
        ser.state_errors_A_C[
            StateErrorIndex.TEMPERATURE_RISE_TREND_CONTINUES
        ].errors_diagnosis(cpu_temp, soc0_temp, soc1_temp, soc2_temp, tg_temp)
