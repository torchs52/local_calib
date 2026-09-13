# ruff: noqa: SLF001

from collections import defaultdict
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import MagicMock

import pytest

from argus_synchro.diagnosis.error_diagnosis import ResultDiagnosis
from argus_synchro.process.app_manager_process import AppManagerProcess
from argus_synchro.shared_errors import (
    ActionErrorIndex,
    ModuleErrorIndex,
    StateErrorDIndex,
    StateErrorIndex,
)


def _diagnosis() -> MagicMock:
    diagnosis = MagicMock()
    diagnosis.errors_diagnosis.return_value = (
        ResultDiagnosis.NORMAL,
        ResultDiagnosis.NORMAL,
    )
    return diagnosis


def test_jetson_monitor_skips_diagnoses_with_missing_numeric_inputs(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    diagnoses = {
        StateErrorIndex.STORAGE_SPACE_LOW: _diagnosis(),
        StateErrorIndex.OUT_OF_MEMORY: _diagnosis(),
        StateErrorIndex.GPU_PERFORMANCE_DEGRADED: _diagnosis(),
        StateErrorIndex.INTERNAL_TEMPERATURE_RISE: _diagnosis(),
        StateErrorIndex.TEMPERATURE_SENSOR_ABNORMAL: _diagnosis(),
        StateErrorIndex.TEMPERATURE_RISE_TREND_CONTINUES: _diagnosis(),
    }
    memory_leak = _diagnosis()
    process = object.__new__(AppManagerProcess)
    process._ser = SimpleNamespace(
        state_errors_A_C=diagnoses,
        state_errors_D={StateErrorDIndex.MEMORY_LEAK_DETECTED: memory_leak},
    )
    process._is_thermal_throttling = True
    data = SimpleNamespace(
        disk_root_avail_gib=10.0,
        disk_data_avail_gib=None,
        ram_used_mb=100,
        ram_total_mb=None,
        gpu_th=None,
        cpu_th=None,
        tj_c=40.0,
        cpu_c=None,
        gpu_c=35.0,
    )
    monkeypatch.setattr(
        "argus_synchro.process.app_manager_process.time.perf_counter", lambda: 20.0
    )

    process._jetson_monitor_diag(data)

    diagnoses[StateErrorIndex.STORAGE_SPACE_LOW].errors_diagnosis.assert_not_called()
    diagnoses[StateErrorIndex.OUT_OF_MEMORY].errors_diagnosis.assert_not_called()
    diagnoses[
        StateErrorIndex.GPU_PERFORMANCE_DEGRADED
    ].errors_diagnosis.assert_not_called()
    diagnoses[
        StateErrorIndex.INTERNAL_TEMPERATURE_RISE
    ].errors_diagnosis.assert_not_called()
    diagnoses[
        StateErrorIndex.TEMPERATURE_RISE_TREND_CONTINUES
    ].errors_diagnosis.assert_not_called()
    memory_leak.errors_diagnosis.assert_called_once_with(100, 20.0)
    diagnoses[
        StateErrorIndex.TEMPERATURE_SENSOR_ABNORMAL
    ].errors_diagnosis.assert_called_once_with(20.0, 40.0, None, 35.0)
    assert process._is_thermal_throttling is False


def _make_monitor_process(
    heartbeat_path: Path,
) -> tuple[AppManagerProcess, MagicMock, MagicMock]:
    file_io_error = _diagnosis()
    monitor_not_responding = _diagnosis()
    process = object.__new__(AppManagerProcess)
    process._monitor_argus_last_heartbeat_path = heartbeat_path
    process._ser = SimpleNamespace(
        state_errors_D={StateErrorDIndex.FILE_IO_ERROR: file_io_error},
        state_errors_A_C={
            StateErrorIndex.MONITOR_PROCESS_NOT_RESPONDING: monitor_not_responding
        },
    )
    return process, file_io_error, monitor_not_responding


def test_monitor_argus_read_failure_reports_file_io_and_missing_heartbeat(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    heartbeat_path = tmp_path / "missing-heartbeat"
    process, file_io_error, monitor_not_responding = _make_monitor_process(
        heartbeat_path
    )
    monkeypatch.setattr(
        "argus_synchro.process.app_manager_process.time.perf_counter", lambda: 30.0
    )

    process._monitor_argus_healthy_check()

    file_io_error.errors_diagnosis.assert_called_once_with(True)
    file_io_error.log_output.assert_called_once_with(
        ResultDiagnosis.NORMAL,
        ResultDiagnosis.NORMAL,
        StateErrorDIndex.FILE_IO_ERROR,
        str(heartbeat_path),
        "read MonitorArgus heartbeat",
        f"FileNotFoundError: [Errno 2] No such file or directory: '{heartbeat_path}'",
    )
    monitor_not_responding.errors_diagnosis.assert_called_once_with(30.0, None)


def test_monitor_argus_read_success_clears_file_io_and_reports_heartbeat(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    heartbeat_path = tmp_path / "heartbeat"
    heartbeat_path.write_text("25.5", encoding="utf-8")
    process, file_io_error, monitor_not_responding = _make_monitor_process(
        heartbeat_path
    )
    monkeypatch.setattr(
        "argus_synchro.process.app_manager_process.time.perf_counter", lambda: 30.0
    )

    process._monitor_argus_healthy_check()

    file_io_error.errors_diagnosis.assert_called_once_with(False)
    file_io_error.log_output.assert_not_called()
    monitor_not_responding.errors_diagnosis.assert_called_once_with(30.0, 25.5)


def test_err_config_load_updates_file_io_diagnosis() -> None:
    error_config = object()
    state_errors_a_c = defaultdict(MagicMock)
    state_errors_d = defaultdict(MagicMock)
    module_errors = defaultdict(MagicMock)
    process = object.__new__(AppManagerProcess)
    process._num_cameras = 0
    process._num_lidars = 0
    process._ser = SimpleNamespace(
        shared_err_conf=SimpleNamespace(read=lambda: error_config),
        action_errors_A_C=defaultdict(MagicMock),
        state_errors_A_C=state_errors_a_c,
        state_errors_D=state_errors_d,
        module_errors=module_errors,
    )

    process._err_config_load()

    state_errors_d[StateErrorDIndex.FILE_IO_ERROR].update.assert_called_once_with(
        error_config
    )
    process._ser.action_errors_A_C[
        ActionErrorIndex.CONFIG_FILE_MISSING
    ].update.assert_called_once_with(error_config)
    module_errors[ModuleErrorIndex.APP_MANAGER_MODULE_ERROR].update.assert_called_once_with(
        error_config
    )


def test_reload_error_config_writes_then_applies(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    calls: list[str] = []
    process = object.__new__(AppManagerProcess)
    process._ser = SimpleNamespace(
        shared_err_conf=SimpleNamespace(write=lambda: calls.append("write"))
    )
    monkeypatch.setattr(
        AppManagerProcess,
        "_err_config_load",
        lambda _self: calls.append("load"),
    )

    process._reload_error_config()

    assert calls == ["write", "load"]


def test_reload_error_config_keeps_previous_config_on_failure(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    error = ValueError("invalid error config")
    process = object.__new__(AppManagerProcess)
    process._ser = SimpleNamespace(
        shared_err_conf=SimpleNamespace(
            write=MagicMock(side_effect=error)
        )
    )
    process._logger = MagicMock()
    load = MagicMock()
    monkeypatch.setattr(AppManagerProcess, "_err_config_load", load)

    process._reload_error_config()

    load.assert_not_called()
    process._logger.warning.assert_called_once_with(
        "error_config reload failed; keeping previous configuration: %s: %s",
        "ValueError",
        error,
    )
