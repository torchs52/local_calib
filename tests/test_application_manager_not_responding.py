from types import SimpleNamespace
from unittest.mock import MagicMock

import pytest

from argus_synchro.diagnosis.error_config import ErrorConfig
from argus_synchro.diagnosis.error_diagnosis import ResultDiagnosis
from argus_synchro.diagnosis.state_errors import (
    ApplicationManagerNotRespondingDiagnosis,
)
from argus_synchro.process.app_manager_process import AppManagerProcess
from argus_synchro.process.error_monitor_process import ErrorMonitorProcess
from argus_synchro.shared_excepts import SharedAppManagerExcept, SharedExcepts
from argus_synchro.shared_errors import StateErrorIndex


def _make_diagnosis() -> ApplicationManagerNotRespondingDiagnosis:
    diagnosis = ApplicationManagerNotRespondingDiagnosis()
    diagnosis.is_enabled = True
    diagnosis.param = SimpleNamespace(error_threshold_sec=5.0)
    return diagnosis


def test_application_manager_not_responding_uses_error_config_parameters() -> None:
    diagnosis = ApplicationManagerNotRespondingDiagnosis()
    diagnosis.update(ErrorConfig())

    assert diagnosis.param.error_threshold_sec == 5.0
    diagnosis.errors_diagnosis(10.0, 10.0)
    assert diagnosis.errors_diagnosis(16.0, 10.0) == (
        ResultDiagnosis.NORMAL,
        ResultDiagnosis.NORMAL,
    )


def test_application_manager_not_responding_detects_stalled_heartbeat() -> None:
    diagnosis = _make_diagnosis()

    assert diagnosis.errors_diagnosis(10.0, 10.0) == (
        ResultDiagnosis.NORMAL,
        ResultDiagnosis.NORMAL,
    )
    assert diagnosis.errors_diagnosis(14.0, 10.0) == (
        ResultDiagnosis.NORMAL,
        ResultDiagnosis.NORMAL,
    )
    assert diagnosis.errors_diagnosis(19.0, 10.0) == (
        ResultDiagnosis.DETECTION,
        ResultDiagnosis.DETECTION,
    )


def test_application_manager_not_responding_detects_backward_jump() -> None:
    diagnosis = _make_diagnosis()
    diagnosis.errors_diagnosis(20.0, 20.0)

    assert diagnosis.errors_diagnosis(21.0, 14.0) == (
        ResultDiagnosis.DETECTION,
        ResultDiagnosis.DETECTION,
    )


def test_application_manager_not_responding_recovers_on_fresh_heartbeat() -> None:
    diagnosis = _make_diagnosis()
    diagnosis.errors_diagnosis(10.0, 10.0)
    diagnosis.errors_diagnosis(14.0, 10.0)
    diagnosis.errors_diagnosis(19.0, 10.0)

    assert diagnosis.errors_diagnosis(20.0, 20.0) == (
        ResultDiagnosis.RECOVERY,
        ResultDiagnosis.RECOVERY,
    )


def test_application_manager_not_responding_rejects_invalid_arguments() -> None:
    diagnosis = _make_diagnosis()

    with pytest.raises(ValueError, match="args must be"):
        diagnosis.errors_diagnosis(1, 1.0)


def test_application_manager_not_responding_owns_logs() -> None:
    diagnosis = _make_diagnosis()
    diagnosis._logger = MagicMock()

    diagnosis.log_output(
        ResultDiagnosis.DETECTION,
        ResultDiagnosis.DETECTION,
        StateErrorIndex.APPLICATION_MANAGER_NOT_RESPONDING,
    )
    diagnosis.log_output(
        ResultDiagnosis.RECOVERY,
        ResultDiagnosis.RECOVERY,
        StateErrorIndex.APPLICATION_MANAGER_NOT_RESPONDING,
    )

    diagnosis._logger.error.assert_called_once_with(
        "SE039: APPLICATION_MANAGER_NOT_RESPONDING detected"
    )
    assert diagnosis._logger.info.call_count == 2


def test_shared_app_manager_heartbeat_is_initially_disabled() -> None:
    shared = SharedAppManagerExcept()

    assert bool(shared.is_started.value) is False
    assert shared.last_heartbeat.value == -1.0
    assert bool(shared.is_heartbeat_enabled.value) is False


def test_shared_excepts_reuses_app_manager_shared_state(app_config) -> None:
    app_manager_shared = SharedAppManagerExcept()

    shared_excepts = SharedExcepts(
        app_config=app_config,
        app_manager_ex=app_manager_shared,
    )

    assert shared_excepts.AppMan_ex is app_manager_shared


def test_app_manager_updates_shared_heartbeat(monkeypatch) -> None:
    process = object.__new__(AppManagerProcess)
    process._ser = SimpleNamespace(AppMan_ex=SharedAppManagerExcept())
    monkeypatch.setattr(
        "argus_synchro.process.app_manager_process.time.monotonic", lambda: 12.5
    )

    process._update_heartbeat()

    assert bool(process._ser.AppMan_ex.is_started.value) is True
    assert process._ser.AppMan_ex.last_heartbeat.value == 12.5
    assert bool(process._ser.AppMan_ex.is_heartbeat_enabled.value) is True


def test_app_manager_alive_log_is_periodic() -> None:
    process = object.__new__(AppManagerProcess)
    process._last_alive_log_mono = 0.0
    process._logger = MagicMock()

    process._log_alive_periodically(100.0)
    process._log_alive_periodically(104.9)
    process._log_alive_periodically(105.0)

    assert process._logger.info.call_count == 2
    process._logger.info.assert_called_with("AppManager alive")


def test_app_manager_shutdown_disables_heartbeat_monitoring() -> None:
    process = object.__new__(AppManagerProcess)
    process._ser = SimpleNamespace(AppMan_ex=SharedAppManagerExcept())
    process._ser.AppMan_ex.is_started.value = True
    process._ser.AppMan_ex.is_heartbeat_enabled.value = True
    process._js_th = None

    process._shutdown()

    assert bool(process._ser.AppMan_ex.is_started.value) is False
    assert bool(process._ser.AppMan_ex.is_heartbeat_enabled.value) is False


@pytest.mark.parametrize("is_enabled", (False, True))
def test_error_monitor_dispatches_app_manager_diagnosis(
    monkeypatch, is_enabled: bool
) -> None:
    diagnosis = MagicMock()
    diagnosis.errors_diagnosis.return_value = (
        ResultDiagnosis.DETECTION,
        ResultDiagnosis.DETECTION,
    )
    app_manager_shared = SimpleNamespace(
        is_heartbeat_enabled=SimpleNamespace(value=is_enabled),
        last_heartbeat=SimpleNamespace(value=10.0),
    )
    process = object.__new__(ErrorMonitorProcess)
    process._is_last_app_manager_diag_enabled = False
    process._ser = SimpleNamespace(
        AppMan_ex=app_manager_shared,
        state_errors_A_C={
            StateErrorIndex.APPLICATION_MANAGER_NOT_RESPONDING: diagnosis
        },
        state_errors=(),
        action_errors=(),
        get_cameras_connected=lambda: (),
        get_lidars_connected=lambda: (),
        reduced_load_mode=SimpleNamespace(enabled=False),
    )
    process._mmap = MagicMock()
    process._logger = MagicMock()
    monkeypatch.setattr(
        "argus_synchro.process.error_monitor_process.time.monotonic", lambda: 15.0
    )

    process._update()

    if is_enabled:
        diagnosis.clear.assert_called_once_with()
        diagnosis.errors_diagnosis.assert_called_once_with(15.0, 10.0)
        diagnosis.log_output.assert_called_once_with(
            ResultDiagnosis.DETECTION,
            ResultDiagnosis.DETECTION,
            StateErrorIndex.APPLICATION_MANAGER_NOT_RESPONDING,
        )
    else:
        diagnosis.clear.assert_not_called()
        diagnosis.errors_diagnosis.assert_not_called()
        diagnosis.log_output.assert_not_called()