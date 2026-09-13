from types import SimpleNamespace
from unittest.mock import MagicMock

import pytest

from argus_synchro.diagnosis.error_config import ErrorConfig
from argus_synchro.diagnosis.error_diagnosis import ResultDiagnosis
from argus_synchro.diagnosis.state_errors import LogOutputStoppedDiagnosis
from argus_synchro.process.app_manager_process import AppManagerProcess
from argus_synchro.shared_errors import StateErrorIndex


def _make_diagnosis() -> LogOutputStoppedDiagnosis:
    diagnosis = LogOutputStoppedDiagnosis()
    diagnosis.is_enabled = True
    diagnosis.param = SimpleNamespace(
        error_threshold_sec=5.0,
        recovery_receive_interval_sec=1.0,
        error_recovery_confirm_duration_sec=5.0,
        failsafe_recovery_confirm_duration_sec=5.0,
    )
    return diagnosis


def test_log_output_stopped_uses_error_config_parameters() -> None:
    diagnosis = LogOutputStoppedDiagnosis()
    diagnosis.update(ErrorConfig())

    assert diagnosis.param.error_threshold_sec == 5.0
    assert diagnosis.param.error_recovery_confirm_duration_sec == 5.0
    assert diagnosis.param.failsafe_recovery_confirm_duration_sec == 5.0
    assert diagnosis.param.recovery_receive_interval_sec == 1.0
    assert diagnosis.errors_diagnosis(10.0, 15.0) == (
        ResultDiagnosis.DETECTION,
        ResultDiagnosis.DETECTION,
    )


def test_log_output_stopped_detects_threshold_and_keeps_state() -> None:
    diagnosis = _make_diagnosis()

    assert diagnosis.errors_diagnosis(10.0, 14.9) == (
        ResultDiagnosis.NORMAL,
        ResultDiagnosis.NORMAL,
    )
    assert diagnosis.errors_diagnosis(10.0, 15.0) == (
        ResultDiagnosis.DETECTION,
        ResultDiagnosis.DETECTION,
    )
    assert diagnosis.errors_diagnosis(10.0, 16.0) == (
        ResultDiagnosis.KEEPING,
        ResultDiagnosis.KEEPING,
    )


def test_log_output_stopped_recovers_after_continuous_updates() -> None:
    diagnosis = _make_diagnosis()
    diagnosis.errors_diagnosis(10.0, 15.0)

    assert diagnosis.errors_diagnosis(16.0, 16.0) == (
        ResultDiagnosis.KEEPING,
        ResultDiagnosis.KEEPING,
    )
    assert diagnosis.errors_diagnosis(21.0, 21.0) == (
        ResultDiagnosis.RECOVERY,
        ResultDiagnosis.RECOVERY,
    )


def test_log_output_stopped_clamps_backward_clock_elapsed() -> None:
    diagnosis = _make_diagnosis()

    assert diagnosis.errors_diagnosis(20.0, 10.0) == (
        ResultDiagnosis.NORMAL,
        ResultDiagnosis.NORMAL,
    )


def test_log_output_stopped_rejects_invalid_arguments() -> None:
    diagnosis = _make_diagnosis()

    with pytest.raises(ValueError, match="args must be"):
        diagnosis.errors_diagnosis(1, 1.0)


def test_log_output_stopped_owns_logs() -> None:
    diagnosis = _make_diagnosis()
    diagnosis._logger = MagicMock()

    diagnosis.log_output(
        ResultDiagnosis.DETECTION,
        ResultDiagnosis.DETECTION,
        StateErrorIndex.LOG_OUTPUT_STOPPED,
    )
    diagnosis.log_output(
        ResultDiagnosis.RECOVERY,
        ResultDiagnosis.RECOVERY,
        StateErrorIndex.LOG_OUTPUT_STOPPED,
    )

    diagnosis._logger.error.assert_called_once_with(
        "SE042: LOG_OUTPUT_STOPPED detected: file log output has stopped"
    )
    assert diagnosis._logger.info.call_count == 2


def _make_app_manager_for_log_monitor(
    diagnosis: MagicMock, log_file_path: object
) -> AppManagerProcess:
    process = object.__new__(AppManagerProcess)
    process._ser = SimpleNamespace(
        state_errors_A_C={StateErrorIndex.LOG_OUTPUT_STOPPED: diagnosis}
    )
    process._log_monitor_enabled = log_file_path is not None
    process._log_file_path = log_file_path
    process._log_watch_last_mono = 10.0
    process._log_watch_last_mtime = 0.0
    process._log_watch_last_size = -1
    return process


def test_app_manager_reports_unchanged_log_timestamp(monkeypatch, tmp_path) -> None:
    log_path = tmp_path / "app.log"
    log_path.write_text("existing log", encoding="utf-8")
    stat = log_path.stat()
    diagnosis = MagicMock()
    diagnosis.errors_diagnosis.return_value = (
        ResultDiagnosis.DETECTION,
        ResultDiagnosis.DETECTION,
    )
    process = _make_app_manager_for_log_monitor(diagnosis, log_path)
    process._log_watch_last_mtime = float(stat.st_mtime)
    process._log_watch_last_size = int(stat.st_size)
    monkeypatch.setattr(
        "argus_synchro.process.app_manager_process.time.monotonic", lambda: 15.0
    )

    process._update_log_output_stopped()

    diagnosis.errors_diagnosis.assert_called_once_with(10.0, 15.0)
    diagnosis.log_output.assert_called_once_with(
        ResultDiagnosis.DETECTION,
        ResultDiagnosis.DETECTION,
        StateErrorIndex.LOG_OUTPUT_STOPPED,
    )


def test_app_manager_refreshes_timestamp_when_log_changes(monkeypatch, tmp_path) -> None:
    log_path = tmp_path / "app.log"
    log_path.write_text("updated log", encoding="utf-8")
    diagnosis = MagicMock()
    diagnosis.errors_diagnosis.return_value = (
        ResultDiagnosis.NORMAL,
        ResultDiagnosis.NORMAL,
    )
    process = _make_app_manager_for_log_monitor(diagnosis, log_path)
    monkeypatch.setattr(
        "argus_synchro.process.app_manager_process.time.monotonic", lambda: 15.0
    )

    process._update_log_output_stopped()

    diagnosis.errors_diagnosis.assert_called_once_with(15.0, 15.0)
    assert process._log_watch_last_size == log_path.stat().st_size


@pytest.mark.parametrize("monitor_enabled", (False, True))
def test_app_manager_handles_unavailable_log_file(
    monkeypatch, tmp_path, monitor_enabled: bool
) -> None:
    diagnosis = MagicMock()
    diagnosis.errors_diagnosis.return_value = (
        ResultDiagnosis.NORMAL,
        ResultDiagnosis.NORMAL,
    )
    log_path = tmp_path / "missing.log" if monitor_enabled else None
    process = _make_app_manager_for_log_monitor(diagnosis, log_path)
    monkeypatch.setattr(
        "argus_synchro.process.app_manager_process.time.monotonic", lambda: 15.0
    )

    process._update_log_output_stopped()

    expected_last_update = 10.0 if monitor_enabled else 15.0
    diagnosis.errors_diagnosis.assert_called_once_with(expected_last_update, 15.0)