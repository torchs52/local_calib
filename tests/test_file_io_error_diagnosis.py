from pathlib import Path

import pytest

from argus_synchro.diagnosis.error_config import ErrorConfig
from argus_synchro.diagnosis.error_diagnosis import ResultDiagnosis
from argus_synchro.diagnosis.state_d_errors import FileIoError
from argus_synchro.shared_errors import SharedErrors, StateErrorDIndex


def test_file_io_error_detection_respects_configuration() -> None:
    diagnosis = FileIoError()
    error_config = ErrorConfig()

    error_config.file_io_error.is_enabled = True
    diagnosis.update(error_config)
    assert diagnosis.errors_diagnosis(True) == (
        ResultDiagnosis.DETECTION,
        ResultDiagnosis.NORMAL,
    )
    assert diagnosis.errors_diagnosis(True) == (
        ResultDiagnosis.KEEPING,
        ResultDiagnosis.NORMAL,
    )
    assert diagnosis.errors_diagnosis(False) == (
        ResultDiagnosis.RECOVERY,
        ResultDiagnosis.NORMAL,
    )
    assert diagnosis.errors_diagnosis(False) == (
        ResultDiagnosis.NORMAL,
        ResultDiagnosis.NORMAL,
    )
    assert diagnosis.errors_diagnosis(True) == (
        ResultDiagnosis.DETECTION,
        ResultDiagnosis.NORMAL,
    )

    diagnosis.reset_error()
    assert diagnosis.errors_diagnosis(True) == (
        ResultDiagnosis.DETECTION,
        ResultDiagnosis.NORMAL,
    )

    error_config.file_io_error.is_enabled = False
    diagnosis.update(error_config)
    assert diagnosis.errors_diagnosis(True) == (
        ResultDiagnosis.NORMAL,
        ResultDiagnosis.NORMAL,
    )


@pytest.mark.parametrize("invalid_value", [None, 1, "true"])
def test_file_io_error_rejects_non_boolean_input(invalid_value: object) -> None:
    diagnosis = FileIoError()
    diagnosis.update(ErrorConfig())

    with pytest.raises(ValueError, match="has_file_io_error"):
        diagnosis.errors_diagnosis(invalid_value)


def test_file_io_error_log_requires_path_operation_and_detail() -> None:
    diagnosis = FileIoError()

    diagnosis.log_output(
        ResultDiagnosis.DETECTION,
        ResultDiagnosis.NORMAL,
        StateErrorDIndex.FILE_IO_ERROR,
        "/tmp/input.dat",
        "read file input",
        "OSError: input failed",
    )

    with pytest.raises(ValueError, match="path, operation, error_detail"):
        diagnosis.log_output(
            ResultDiagnosis.DETECTION,
            ResultDiagnosis.NORMAL,
            StateErrorDIndex.FILE_IO_ERROR,
            "/tmp/input.dat",
            "read file input",
        )


def test_file_io_error_index_maps_to_registered_diagnosis() -> None:
    shared_errors = SharedErrors(Path("config/error_config.json"))
    try:
        assert isinstance(
            shared_errors.state_errors_D[StateErrorDIndex.FILE_IO_ERROR],
            FileIoError,
        )
    finally:
        shared_errors.shared_err_conf.close()