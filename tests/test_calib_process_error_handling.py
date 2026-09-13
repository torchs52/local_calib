from __future__ import annotations

from types import SimpleNamespace
from unittest.mock import MagicMock

from argus_synchro.diagnosis.error_diagnosis import ResultDiagnosis
from argus_synchro.process.calib_process import CalibProcess
from argus_synchro.shared_errors import (
    ActionErrorIndex,
    ModuleErrorIndex,
    StateErrorDIndex,
)


def test_err_config_load_enables_file_io_diagnosis() -> None:
    error_config = object()
    file_io_error = MagicMock()
    invalid_data_input = MagicMock()
    array_shape_error = MagicMock()
    ai_model_load_failed = MagicMock()
    calibration_module_error = MagicMock()
    process = object.__new__(CalibProcess)
    process._ser = SimpleNamespace(
        shared_err_conf=SimpleNamespace(read=lambda: error_config),
        state_errors_D={
            StateErrorDIndex.FILE_IO_ERROR: file_io_error,
            StateErrorDIndex.INVALID_DATA_INPUT: invalid_data_input,
            StateErrorDIndex.ARRAY_SHAPE_ERROR: array_shape_error,
        },
        action_errors_A_C={ActionErrorIndex.AI_MODEL_LOAD_FAILED: ai_model_load_failed},
        module_errors={
            ModuleErrorIndex.CALIBRATION_MODULE_ERROR: calibration_module_error
        },
    )

    process._err_config_load()

    file_io_error.update.assert_called_once_with(error_config)
    invalid_data_input.update.assert_called_once_with(error_config)
    array_shape_error.update.assert_called_once_with(error_config)
    ai_model_load_failed.update.assert_called_once_with(error_config)
    calibration_module_error.update.assert_called_once_with(error_config)


def test_report_file_io_error_uses_importance_d_index_and_exception_path() -> None:
    diagnosis = MagicMock()
    diagnosis.errors_diagnosis.return_value = (
        ResultDiagnosis.DETECTION,
        ResultDiagnosis.NORMAL,
    )
    process = object.__new__(CalibProcess)
    process._ser = SimpleNamespace(
        state_errors_D={StateErrorDIndex.FILE_IO_ERROR: diagnosis}
    )
    process.inifilepath = "/config/calib_settings.ini"
    error = OSError(5, "read failed", "/config/calibration/camera0.csv")

    process._report_file_io_error(error, "read calibration auxiliary file")

    diagnosis.errors_diagnosis.assert_called_once_with(True)
    diagnosis.log_output.assert_called_once_with(
        ResultDiagnosis.DETECTION,
        ResultDiagnosis.NORMAL,
        StateErrorDIndex.FILE_IO_ERROR,
        "/config/calibration/camera0.csv",
        "read calibration auxiliary file",
        "OSError: [Errno 5] read failed: '/config/calibration/camera0.csv'",
    )


def test_set_calib3d3d_error_state_updates_common_and_unexpected_errors() -> None:
    monitor = MagicMock()

    CalibProcess._set_calib3d3d_error_state(monitor)

    monitor.set_errors_calibcommon.assert_called_once_with(2)
    monitor.set_errorcode_unexpected_exception.assert_called_once_with(True)
