import logging
from unittest.mock import MagicMock

import pytest

from argus_synchro.common.app_logger import AppLoggerFactory
from argus_synchro.diagnosis.action_errors import LogFileIoErrorDiagnosis
from argus_synchro.diagnosis.error_config import ErrorConfig


def _enabled_diagnosis() -> LogFileIoErrorDiagnosis:
    diagnosis = LogFileIoErrorDiagnosis()
    diagnosis.update(ErrorConfig())
    return diagnosis


@pytest.mark.parametrize("compress", [True, False])
def test_file_handler_reports_write_error_without_logging_recursion(
    tmp_path, compress: bool
) -> None:
    logger = AppLoggerFactory.from_name(
        f"IoErrorCallbackTest-{compress}",
        to_console=False,
        to_file=str(tmp_path / "app.log"),
        compress=compress,
    )
    callback = MagicMock()
    logger.set_io_error_callback(callback)

    handler = logger._logger.handlers[0]
    handler.stream.write = MagicMock(side_effect=OSError("disk write failed"))

    previous_raise_exceptions = logging.raiseExceptions
    logging.raiseExceptions = False
    try:
        logger.warning("write failure")
    finally:
        logging.raiseExceptions = previous_raise_exceptions
        handler.close()

    callback.assert_called_once()
    assert isinstance(callback.call_args.args[0], OSError)


def test_factory_keeps_io_error_callback_when_handlers_are_updated(tmp_path) -> None:
    factory = AppLoggerFactory(
        to_console=False,
        to_file=str(tmp_path / "app.log"),
    )
    callback = MagicMock()
    factory.set_io_error_callback(callback)
    logger = factory.register_from_name("IoErrorFactoryTest")

    factory.update()

    handler = logger._logger.handlers[0]
    assert handler.io_error_callback is callback
    handler.close()


def test_log_file_io_diagnosis_throttles_same_error(monkeypatch) -> None:
    diagnosis = _enabled_diagnosis()
    monotonic_values = iter((10.0, 10.5, 11.1))
    monkeypatch.setattr(
        "argus_synchro.diagnosis.action_errors.time.monotonic",
        lambda: next(monotonic_values),
    )

    assert diagnosis.excepts_diagnosis(OSError("disk full")) is True
    assert diagnosis.err_cnt.value == 1
    assert diagnosis.excepts_diagnosis(OSError("disk full")) is True
    assert diagnosis.err_cnt.value == 1
    assert diagnosis.excepts_diagnosis(OSError("disk full")) is True
    assert diagnosis.err_cnt.value == 2


def test_log_file_io_diagnosis_ignores_non_os_error() -> None:
    diagnosis = _enabled_diagnosis()

    assert diagnosis.excepts_diagnosis(RuntimeError("format failed")) is False
    assert diagnosis.err_cnt.value == 0


def test_log_file_io_diagnosis_honors_disabled_setting() -> None:
    error_config = ErrorConfig()
    error_config.log_file_io_error.is_enabled = False
    diagnosis = LogFileIoErrorDiagnosis()
    diagnosis.update(error_config)

    assert diagnosis.excepts_diagnosis(OSError("disk full")) is False
    assert diagnosis.err_cnt.value == 0