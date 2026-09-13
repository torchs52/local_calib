import logging
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import MagicMock

import pytest

from argus_synchro import __main__ as app_main
from argus_synchro.common.app_logger import AppLoggerFactory
from argus_synchro.diagnosis.error_config import ErrorConfig
from argus_synchro.diagnosis.error_diagnosis import ResultDiagnosis
from argus_synchro.diagnosis.state_d_errors import LogTimeReversal
from argus_synchro.process.app_manager_process import AppManagerProcess
from argus_synchro.shared_errors import (
    ActionErrorIndex,
    SharedErrors,
    StateErrorDIndex,
)


@pytest.mark.parametrize("compress", [True, False])
def test_file_handler_reports_record_times_after_first_record(
    tmp_path, compress: bool
) -> None:
    logger = AppLoggerFactory.from_name(
        f"TimeReversalCallbackTest-{compress}",
        to_console=False,
        to_file=str(tmp_path / "app.log"),
        compress=compress,
    )
    callback = MagicMock()
    logger.set_time_reversal_callback(callback)
    handler = logger._logger.handlers[0]
    first = logging.LogRecord("test", logging.INFO, "", 0, "first", (), None)
    first.created = 10.0
    second = logging.LogRecord("test", logging.INFO, "", 0, "second", (), None)
    second.created = 9.0

    handler.emit(first)
    callback.assert_not_called()
    handler.emit(second)

    callback.assert_called_once_with(10.0, 9.0)
    handler.close()


def test_log_time_reversal_respects_allowed_backward_seconds() -> None:
    diagnosis = LogTimeReversal()
    error_config = ErrorConfig()
    error_config.log_time_reversal.is_enabled = True
    error_config.log_time_reversal.allowed_backward_sec = 0.5
    diagnosis.update(error_config)

    diagnosis.report_record_time(10.0, 9.6)
    assert diagnosis.errors_diagnosis() == (
        ResultDiagnosis.NORMAL,
        ResultDiagnosis.NORMAL,
    )

    diagnosis.report_record_time(10.0, 9.4)
    assert diagnosis.errors_diagnosis() == (
        ResultDiagnosis.DETECTION,
        ResultDiagnosis.NORMAL,
    )
    assert diagnosis.errors_diagnosis() == (
        ResultDiagnosis.RECOVERY,
        ResultDiagnosis.NORMAL,
    )


def test_factory_keeps_time_reversal_callback_when_handlers_are_updated(
    tmp_path,
) -> None:
    factory = AppLoggerFactory(
        to_console=False,
        to_file=str(tmp_path / "app.log"),
    )
    callback = MagicMock()
    factory.set_time_reversal_callback(callback)
    logger = factory.register_from_name("TimeReversalFactoryTest")

    factory.update()

    handler = logger._logger.handlers[0]
    assert handler.time_reversal_callback is callback
    handler.close()


def test_app_manager_consumes_time_reversal_event() -> None:
    diagnosis = MagicMock()
    diagnosis.errors_diagnosis.return_value = (
        ResultDiagnosis.DETECTION,
        ResultDiagnosis.NORMAL,
    )
    state_errors_d: list[object | None] = [None] * (
        StateErrorDIndex.LOG_TIME_REVERSAL + 1
    )
    state_errors_d[StateErrorDIndex.LOG_TIME_REVERSAL] = diagnosis
    process = object.__new__(AppManagerProcess)
    process._ser = SimpleNamespace(state_errors_D=state_errors_d)

    process._update_log_time_reversal()

    diagnosis.errors_diagnosis.assert_called_once_with()
    diagnosis.log_output.assert_called_once_with(
        ResultDiagnosis.DETECTION,
        ResultDiagnosis.NORMAL,
        StateErrorDIndex.LOG_TIME_REVERSAL,
    )


def test_log_time_reversal_index_maps_to_registered_diagnosis() -> None:
    shared_errors = SharedErrors(Path("config/error_config.json"))
    try:
        assert isinstance(
            shared_errors.state_errors_D[StateErrorDIndex.LOG_TIME_REVERSAL],
            LogTimeReversal,
        )
    finally:
        shared_errors.shared_err_conf.close()


def test_load_err_config_initializes_time_reversal_before_logger_callback() -> None:
    shared_errors = SharedErrors(Path("config/error_config.json"))
    try:
        app_main.load_err_config(shared_errors)
        diagnosis = shared_errors.state_errors_D[StateErrorDIndex.LOG_TIME_REVERSAL]

        diagnosis.report_record_time(10.0, 9.0)

        assert diagnosis.param is not None
    finally:
        shared_errors.shared_err_conf.close()


def test_load_err_config_initializes_shi_action_error_settings() -> None:
    shared_errors = SharedErrors(Path("config/error_config.json"))
    try:
        app_main.load_err_config(shared_errors)

        for error_index in (
            ActionErrorIndex.CONFIG_FILE_MISSING,
            ActionErrorIndex.AI_MODEL_LOAD_FAILED,
            ActionErrorIndex.LOG_FILE_IO_ERROR,
        ):
            diagnosis = shared_errors.action_errors_A_C[error_index]
            assert diagnosis.is_enabled is True
            assert diagnosis.param is not None
    finally:
        shared_errors.shared_err_conf.close()
