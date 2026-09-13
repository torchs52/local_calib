from __future__ import annotations

import json
import multiprocessing as mp
import time
from multiprocessing.sharedctypes import Synchronized
from multiprocessing.synchronize import Barrier
from pathlib import Path
from unittest.mock import MagicMock

import pytest

from argus_synchro.__main__ import stop_error_monitor
from argus_synchro.common.app_logger import AppLoggerFactory
from argus_synchro.common.paths import DirectoryConfig
from argus_synchro.process.process import (
    ProcessBase,
    ProcessCpuAffinityConfigError,
    ProcessManager,
)
from argus_synchro.process.synchronizer import ProcessActivator
from argus_synchro.shared_excepts import SharedGetDataExcept, SharedProcessExcept


class DummyProcess(ProcessBase):
    def __init__(
        self,
        spe: SharedProcessExcept,
        activator: ProcessActivator,
        name: str,
        startup_sec: float,
        process_count: int,
        startup_done_count: Synchronized[int],
        loop_started_count: Synchronized[int],
        loop_before_all_startup: Synchronized[bool],
        startup_barrier: Barrier | None = None,
    ) -> None:
        super().__init__(spe, activator, name)
        self._startup_sec = startup_sec
        self._process_count = process_count
        self._startup_done_count = startup_done_count
        self._loop_started_count = loop_started_count
        self._loop_before_all_startup = loop_before_all_startup
        self._startup_barrier = startup_barrier

    def _startup(self) -> None:
        if self._startup_barrier is not None:
            self._startup_barrier.wait()
        time.sleep(self._startup_sec)
        with self._startup_done_count.get_lock():
            self._startup_done_count.value += 1

    def _loop(self) -> None:
        with self._startup_done_count.get_lock():
            if self._startup_done_count.value < self._process_count:
                self._loop_before_all_startup.value = True
        with self._loop_started_count.get_lock():
            self._loop_started_count.value += 1
        while self._process_activator.value:
            time.sleep(0.02)

    def create_producer_and_consumer(self) -> None:
        pass

    def restart_completed(self) -> None:
        pass

    def _start_restart(self) -> None:
        pass


class UnsubscribeProcess(ProcessBase):
    def _loop(self) -> None:
        pass

    def create_producer_and_consumer(self) -> None:
        pass

    def restart_completed(self) -> None:
        pass

    def _start_restart(self) -> None:
        pass


def _make_manager() -> tuple[ProcessManager, ProcessActivator]:
    activator = ProcessActivator()
    activator.enable()
    logger = AppLoggerFactory.from_name("ProcessManagerTest", to_console=False)
    return ProcessManager(activator, logger), activator


def _stop(manager: ProcessManager, activator: ProcessActivator) -> None:
    activator.disable()
    manager.kill()
    manager.join()


def _wait_loop_started(loop_started_count: Synchronized[int], expected: int) -> None:
    deadline = time.perf_counter() + 2.0
    while time.perf_counter() < deadline:
        if loop_started_count.value >= expected:
            return
        time.sleep(0.02)
    pytest.fail(f"loop did not start: got {loop_started_count.value}, expected {expected}")


def test_get_process_cpu_affinity_accepts_non_empty_integer_list(
    tmp_path: Path,
) -> None:
    config_path = tmp_path / "process_cpu_affinity.json"
    config_path.write_text('{"TargetProcess": [0, 2]}', encoding="utf-8")
    manager, _ = _make_manager()

    assert manager.get_process_cpu_affinity(
        str(config_path), "TargetProcess"
    ) == [0, 2]


@pytest.mark.parametrize(
    "content",
    (
        "[]",
        '{"TargetProcess": []}',
        '{"TargetProcess": "0"}',
        '{"TargetProcess": [0, true]}',
        '{"OtherProcess": [0]}',
    ),
)
def test_get_process_cpu_affinity_rejects_invalid_structure(
    tmp_path: Path,
    content: str,
) -> None:
    config_path = tmp_path / "process_cpu_affinity.json"
    config_path.write_text(content, encoding="utf-8")
    manager, _ = _make_manager()

    with pytest.raises(ProcessCpuAffinityConfigError):
        manager.get_process_cpu_affinity(str(config_path), "TargetProcess")


@pytest.mark.parametrize("create_file", (True, False))
def test_get_process_cpu_affinity_wraps_read_errors(
    tmp_path: Path,
    create_file: bool,
) -> None:
    config_path = tmp_path / "process_cpu_affinity.json"
    if create_file:
        config_path.write_text("{", encoding="utf-8")
    manager, _ = _make_manager()

    with pytest.raises(ProcessCpuAffinityConfigError) as exc_info:
        manager.get_process_cpu_affinity(str(config_path), "TargetProcess")

    assert str(config_path) in str(exc_info.value)
    assert isinstance(exc_info.value.__cause__, (OSError, json.JSONDecodeError))


def test_stop_error_monitor_forces_shutdown_before_close() -> None:
    processes = MagicMock(spec=ProcessManager)
    closables = MagicMock()

    stop_error_monitor(closables, processes)

    processes.graceful_stop_all.assert_called_once_with(
        t_grace=3.0,
        t_term=2.0,
        t_kill=1.0,
    )
    closables.close.assert_called_once_with()


def test_process_unsubscribe_logs_without_changing_flow_stop() -> None:
    process = UnsubscribeProcess(MagicMock(), ProcessActivator(), "TestProcess")
    process._logger = MagicMock()
    flow = MagicMock()
    synchronizer = MagicMock()
    flow.synchronizer = synchronizer
    process._subscribe(flow)

    process._unsubscribe()

    process._logger.warning.assert_called_once_with(
        "%s unsubscribing message flows", "TestProcess"
    )
    synchronizer.stop.assert_called_once_with()


def test_process_unsubscribe_stops_flow_before_logger_initialization() -> None:
    process = UnsubscribeProcess(MagicMock(), ProcessActivator(), "TestProcess")
    flow = MagicMock()
    synchronizer = MagicMock()
    flow.synchronizer = synchronizer
    process._subscribe(flow)

    process._unsubscribe()

    synchronizer.stop.assert_called_once_with()


def test_process_manager_start_runs_startup_concurrently(
    directory_config: DirectoryConfig,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    process_count = 3
    monkeypatch.setattr(ProcessManager, "STARTUP_TIMEOUT_SEC", 2.0)
    startup_done_count = mp.Value("i", 0)
    loop_started_count = mp.Value("i", 0)
    loop_before_all_startup = mp.Value("b", False)
    startup_barrier = mp.Barrier(process_count)
    manager, activator = _make_manager()
    for i in range(process_count):
        DummyProcess(
            SharedGetDataExcept(),
            activator,
            f"DummyProcess[{i}]",
            0.0,
            process_count,
            startup_done_count,
            loop_started_count,
            loop_before_all_startup,
            startup_barrier,
        ).add_to(manager)

    try:
        manager.start("", directory_config)
        _wait_loop_started(loop_started_count, process_count)
    finally:
        _stop(manager, activator)


def test_process_manager_start_waits_for_all_startup_before_loop(
    directory_config: DirectoryConfig,
) -> None:
    process_count = 3
    startup_done_count = mp.Value("i", 0)
    loop_started_count = mp.Value("i", 0)
    loop_before_all_startup = mp.Value("b", False)
    manager, activator = _make_manager()
    for i in range(process_count):
        DummyProcess(
            SharedGetDataExcept(),
            activator,
            f"DummyProcess[{i}]",
            0.05,
            process_count,
            startup_done_count,
            loop_started_count,
            loop_before_all_startup,
        ).add_to(manager)

    try:
        manager.start("", directory_config)
        _wait_loop_started(loop_started_count, process_count)
        assert not loop_before_all_startup.value
    finally:
        _stop(manager, activator)


def test_process_manager_start_raises_on_startup_timeout(
    directory_config: DirectoryConfig,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setattr(ProcessManager, "STARTUP_TIMEOUT_SEC", 0.2)
    startup_done_count = mp.Value("i", 0)
    loop_started_count = mp.Value("i", 0)
    loop_before_all_startup = mp.Value("b", False)
    manager, activator = _make_manager()
    DummyProcess(
        SharedGetDataExcept(),
        activator,
        "DummyProcess[timeout]",
        10.0,
        1,
        startup_done_count,
        loop_started_count,
        loop_before_all_startup,
    ).add_to(manager)

    try:
        with pytest.raises(RuntimeError, match="process startup timeout: name=DummyProcess\\[timeout\\]"):
            manager.start("", directory_config)
    finally:
        _stop(manager, activator)
