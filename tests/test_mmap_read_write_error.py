from types import SimpleNamespace
from unittest.mock import MagicMock

import pytest

from argus_synchro.__main__ import _write_status_safe
from argus_synchro import __main__ as app_main
from argus_synchro.diagnosis.action_errors import MmapReadWriteErrorDiagnosis
from argus_synchro.diagnosis.error_config import ErrorConfig
from argus_synchro.process.error_monitor_process import ErrorMonitorProcess
from argus_synchro.process.visual_process import VisualProcess
from argus_synchro.shared_errors import ActionErrorIndex, StateErrorIndex
from argus_synchro.SystemMonitor.status_mmap import StatusCode


@pytest.mark.parametrize(
    "error",
    (
        OSError("disk failure"),
        ValueError("closed mmap"),
        BufferError("exported buffer"),
        RuntimeError("writer failure"),
    ),
)
def test_mmap_read_write_error_counts_target_exceptions(error: Exception) -> None:
    diagnosis = MmapReadWriteErrorDiagnosis()
    error_config = ErrorConfig()
    error_config.mmap_read_write_error.is_enabled = True
    diagnosis.update(error_config)

    assert diagnosis.excepts_diagnosis(error) is True
    assert diagnosis.err_cnt.value == 1


def test_mmap_read_write_error_ignores_non_target_exception() -> None:
    diagnosis = MmapReadWriteErrorDiagnosis()
    error_config = ErrorConfig()
    error_config.mmap_read_write_error.is_enabled = True
    diagnosis.update(error_config)

    assert diagnosis.excepts_diagnosis(TypeError("unexpected")) is False
    assert diagnosis.err_cnt.value == 0


def test_mmap_read_write_error_is_disabled_by_default() -> None:
    diagnosis = MmapReadWriteErrorDiagnosis()

    assert diagnosis.excepts_diagnosis(OSError("mmap unavailable")) is False
    assert diagnosis.err_cnt.value == 0


def test_mmap_read_write_error_owns_error_log_output() -> None:
    diagnosis = MmapReadWriteErrorDiagnosis()
    diagnosis._logger = MagicMock()
    error = ValueError("closed mmap")

    diagnosis.log_output(
        True,
        False,
        ActionErrorIndex.MMAP_READ_WRITE_ERROR,
        "status.mmap write failed",
        error,
    )

    diagnosis._logger.error.assert_called_once_with(
        "CE011: MMAP_READ_WRITE_ERROR: status.mmap write failed: "
        "ValueError('closed mmap')",
        exc_info=True,
    )


def test_error_monitor_dispatches_mmap_failure_and_continues() -> None:
    diagnosis = MmapReadWriteErrorDiagnosis()
    diagnosis._logger = MagicMock()
    error_config = ErrorConfig()
    error_config.mmap_read_write_error.is_enabled = True
    diagnosis.update(error_config)
    process = object.__new__(ErrorMonitorProcess)
    process._is_last_app_manager_diag_enabled = False
    process._ser = SimpleNamespace(
        AppMan_ex=SimpleNamespace(
            is_heartbeat_enabled=SimpleNamespace(value=False)
        ),
        action_errors_A_C={ActionErrorIndex.MMAP_READ_WRITE_ERROR: diagnosis},
        state_errors=(),
        action_errors=(),
    )
    process._mmap = MagicMock()
    process._mmap.start_write.side_effect = BufferError("busy mmap")

    process._update()

    assert diagnosis.err_cnt.value == 1
    diagnosis._logger.error.assert_called_once_with(
        "CE011: MMAP_READ_WRITE_ERROR: ErrorMMapWriter transaction failed: "
        "BufferError('busy mmap')",
        exc_info=True,
    )


def test_error_monitor_logs_mmap_error_values_and_changes() -> None:
    process = object.__new__(ErrorMonitorProcess)
    process._ser = SimpleNamespace(action_errors=(object(), object()))
    process._logger = MagicMock()
    process._last_state_err_value = None
    process._last_action_err_values = None

    state_bit = 1 << int(StateErrorIndex.LIDAR0_CONNECTION_ERROR)
    process._debug_log_state_errors(state_bit.to_bytes(16, "little"))
    process._debug_log_state_errors((0).to_bytes(16, "little"))
    process._debug_log_action_errors(bytes((0, 1)) + bytes(30))
    process._debug_log_action_errors(bytes((0, 2)) + bytes(30))

    assert process._logger.debug.call_count == 4
    process._logger.error.assert_any_call(
        "state_error %s: bit%d=%s",
        "ON ",
        int(StateErrorIndex.LIDAR0_CONNECTION_ERROR),
        StateErrorIndex.LIDAR0_CONNECTION_ERROR.name,
    )
    process._logger.error.assert_any_call(
        "state_error %s: bit%d=%s",
        "OFF",
        int(StateErrorIndex.LIDAR0_CONNECTION_ERROR),
        StateErrorIndex.LIDAR0_CONNECTION_ERROR.name,
    )
    process._logger.error.assert_any_call(
        "action_error counters: %s",
        "index1=SENSOR_CALIBRATION_REQUIRED:1->2",
    )


def test_visual_ui_dispatches_mmap_failure_and_reraises(monkeypatch) -> None:
    diagnosis = MmapReadWriteErrorDiagnosis()
    diagnosis._logger = MagicMock()
    error_config = ErrorConfig()
    error_config.mmap_read_write_error.is_enabled = True
    diagnosis.update(error_config)
    process = object.__new__(VisualProcess)
    process._ProcessBase__process = None
    process._ser = SimpleNamespace(
        action_errors_A_C={ActionErrorIndex.MMAP_READ_WRITE_ERROR: diagnosis}
    )
    process._app_config = SimpleNamespace(
        Scrutinizer=SimpleNamespace(s_frame=0),
        General=SimpleNamespace(
            rotation_radius=1.0,
            has_external_guard=False,
            external_guard_offset=0.0,
        ),
        camera=SimpleNamespace(count=1),
    )
    process._directory_config = MagicMock()
    process._logger = MagicMock()
    error = OSError("status mmap unavailable")
    monkeypatch.setattr(
        "argus_synchro.process.visual_process.GodotUIVisualizer",
        MagicMock(side_effect=error),
    )

    with pytest.raises(OSError, match="status mmap unavailable"):
        process._create_visual_ui(MagicMock())

    assert diagnosis.err_cnt.value == 1
    diagnosis._logger.error.assert_called_once_with(
        "CE011: MMAP_READ_WRITE_ERROR: Godot UI MMAP initialization failed: "
        "OSError('status mmap unavailable')",
        exc_info=True,
    )


def test_main_status_write_dispatches_mmap_failure_and_continues() -> None:
    diagnosis = MmapReadWriteErrorDiagnosis()
    diagnosis._logger = MagicMock()
    error_config = ErrorConfig()
    error_config.mmap_read_write_error.is_enabled = True
    diagnosis.update(error_config)
    status = MagicMock()
    status.write_status.side_effect = RuntimeError("status writer failed")
    ser = SimpleNamespace(
        action_errors_A_C={ActionErrorIndex.MMAP_READ_WRITE_ERROR: diagnosis}
    )

    _write_status_safe(status, StatusCode.REBOOT, ser)

    assert diagnosis.err_cnt.value == 1
    diagnosis._logger.error.assert_called_once_with(
        "CE011: MMAP_READ_WRITE_ERROR: status.mmap write failed (REBOOT): "
        "RuntimeError('status writer failed')",
        exc_info=True,
    )


def test_main_load_err_config_updates_mmap_diagnosis() -> None:
    error_config = ErrorConfig()
    diagnosis = MagicMock()
    action_errors = MagicMock()
    action_errors.__getitem__.return_value = diagnosis
    ser = SimpleNamespace(
        shared_err_conf=SimpleNamespace(read=MagicMock(return_value=error_config)),
        state_errors_D=MagicMock(),
        action_errors_A_C=action_errors,
        module_errors=MagicMock(),
    )

    app_main.load_err_config(ser)

    action_errors.__getitem__.assert_any_call(ActionErrorIndex.MMAP_READ_WRITE_ERROR)
    diagnosis.update.assert_any_call(error_config)


def test_error_monitor_updates_mmap_diagnosis() -> None:
    error_config = ErrorConfig()
    diagnosis = MagicMock()
    action_errors = MagicMock()
    action_errors.__getitem__.return_value = diagnosis
    process = object.__new__(ErrorMonitorProcess)
    process._ser = SimpleNamespace(
        shared_err_conf=SimpleNamespace(read=MagicMock(return_value=error_config)),
        state_errors_A_C=MagicMock(),
        action_errors_A_C=action_errors,
    )

    process._err_config_load()

    action_errors.__getitem__.assert_called_once_with(
        ActionErrorIndex.MMAP_READ_WRITE_ERROR
    )
    diagnosis.update.assert_called_once_with(error_config)


def test_visual_updates_mmap_diagnosis() -> None:
    error_config = ErrorConfig()
    diagnosis = MagicMock()
    action_errors = MagicMock()
    action_errors.__getitem__.return_value = diagnosis
    process = object.__new__(VisualProcess)
    process._ser = SimpleNamespace(
        shared_err_conf=SimpleNamespace(read=MagicMock(return_value=error_config)),
        state_errors_A_C=MagicMock(),
        state_errors_D=MagicMock(),
        action_errors_A_C=action_errors,
        module_errors=MagicMock(),
    )

    process._err_config_load()

    action_errors.__getitem__.assert_any_call(ActionErrorIndex.MMAP_READ_WRITE_ERROR)
    diagnosis.update.assert_any_call(error_config)