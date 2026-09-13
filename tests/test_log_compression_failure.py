from pathlib import Path
from types import SimpleNamespace
from unittest.mock import MagicMock

from argus_synchro.common import app_logger
from argus_synchro.common.app_logger import AppLoggerFactory
from argus_synchro.diagnosis.error_config import ErrorConfig
from argus_synchro.diagnosis.error_diagnosis import ResultDiagnosis
from argus_synchro.diagnosis.state_d_errors import LogCompressionFailure
from argus_synchro.process.app_manager_process import AppManagerProcess
from argus_synchro.shared_errors import SharedErrors, StateErrorDIndex


def test_compression_failure_keeps_uncompressed_backup_without_ce015(
    tmp_path, monkeypatch
) -> None:
    log_path = tmp_path / "app.log"
    logger = AppLoggerFactory.from_name(
        "CompressionFailureTest",
        to_console=False,
        to_file=str(log_path),
        rotate_size=1,
        backup_count=1,
        compress=True,
    )
    io_error_callback = MagicMock()
    compression_error_callback = MagicMock()
    logger.set_io_error_callback(io_error_callback)
    logger.set_compression_error_callback(compression_error_callback)
    monkeypatch.setattr(
        app_logger.gzip,
        "open",
        MagicMock(side_effect=OSError("gzip failed")),
    )

    logger.warning("trigger rollover")

    compression_error_callback.assert_called_once()
    assert isinstance(compression_error_callback.call_args.args[0], OSError)
    io_error_callback.assert_not_called()
    assert Path(f"{log_path}.1").exists()
    logger._logger.handlers[0].close()


def test_log_compression_failure_reports_one_event_edge() -> None:
    diagnosis = LogCompressionFailure()
    error_config = ErrorConfig()
    error_config.log_compression_failure.is_enabled = True
    diagnosis.update(error_config)

    diagnosis.report_event(OSError("gzip failed"))

    assert diagnosis.errors_diagnosis() == (
        ResultDiagnosis.DETECTION,
        ResultDiagnosis.NORMAL,
    )
    assert diagnosis.errors_diagnosis() == (
        ResultDiagnosis.RECOVERY,
        ResultDiagnosis.NORMAL,
    )
    assert diagnosis.errors_diagnosis() == (
        ResultDiagnosis.NORMAL,
        ResultDiagnosis.NORMAL,
    )


def test_factory_keeps_compression_callback_when_handlers_are_updated(
    tmp_path,
) -> None:
    factory = AppLoggerFactory(
        to_console=False,
        to_file=str(tmp_path / "app.log"),
    )
    callback = MagicMock()
    factory.set_compression_error_callback(callback)
    logger = factory.register_from_name("CompressionCallbackFactoryTest")

    factory.update()

    handler = logger._logger.handlers[0]
    assert handler.compression_error_callback is callback
    handler.close()


def test_app_manager_consumes_compression_failure_event() -> None:
    diagnosis = MagicMock()
    diagnosis.errors_diagnosis.return_value = (
        ResultDiagnosis.DETECTION,
        ResultDiagnosis.NORMAL,
    )
    state_errors_d: list[object | None] = [None] * (
        StateErrorDIndex.LOG_COMPRESSION_FAILURE + 1
    )
    state_errors_d[StateErrorDIndex.LOG_COMPRESSION_FAILURE] = diagnosis
    process = object.__new__(AppManagerProcess)
    process._ser = SimpleNamespace(state_errors_D=state_errors_d)

    process._update_log_compression_failure()

    diagnosis.errors_diagnosis.assert_called_once_with()
    diagnosis.log_output.assert_called_once_with(
        ResultDiagnosis.DETECTION,
        ResultDiagnosis.NORMAL,
        StateErrorDIndex.LOG_COMPRESSION_FAILURE,
    )


def test_log_compression_failure_index_maps_to_registered_diagnosis() -> None:
    shared_errors = SharedErrors(Path("config/error_config.json"))
    try:
        assert isinstance(
            shared_errors.state_errors_D[StateErrorDIndex.LOG_COMPRESSION_FAILURE],
            LogCompressionFailure,
        )
    finally:
        shared_errors.shared_err_conf.close()