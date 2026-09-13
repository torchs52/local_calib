from types import SimpleNamespace
from unittest.mock import MagicMock

from argus_synchro.diagnosis.action_errors import (
    CameraXCalibDataInvalidDiagnosis,
    SensorCalibDataInvalidDiagnosis,
)
from argus_synchro.diagnosis.error_config import ErrorConfig
from argus_synchro.shared_errors import (
    ActionErrorIndex,
    diagnose_startup_calibration_data,
)


def test_startup_calibration_diagnosis_dispatches_enabled_camera_slots() -> None:
    sensor_diagnosis = MagicMock(spec=SensorCalibDataInvalidDiagnosis)
    camera_diagnoses = [
        MagicMock(spec=CameraXCalibDataInvalidDiagnosis) for _ in range(4)
    ]
    action_errors = {
        ActionErrorIndex.SENSOR_CALIB_DATA_INVALID: sensor_diagnosis,
        ActionErrorIndex.CAMERA0_CALIB_DATA_INVALID: camera_diagnoses[0],
        ActionErrorIndex.CAMERA1_CALIB_DATA_INVALID: camera_diagnoses[1],
        ActionErrorIndex.CAMERA2_CALIB_DATA_INVALID: camera_diagnoses[2],
        ActionErrorIndex.CAMERA3_CALIB_DATA_INVALID: camera_diagnoses[3],
    }
    shared_errors = SimpleNamespace(action_errors_A_C=action_errors)
    calibration_conf = MagicMock()
    error_config = ErrorConfig()
    reference_paths = ["lidar0.csv", "lidar1.csv"]

    diagnose_startup_calibration_data(
        shared_errors,
        calibration_conf,
        reference_paths,
        camera_count=2,
        error_config=error_config,
    )

    sensor_diagnosis.update.assert_called_once_with(error_config)
    sensor_diagnosis.diagnose_calibration_matrices.assert_called_once_with(
        calibration_conf,
        ActionErrorIndex.SENSOR_CALIB_DATA_INVALID,
        lidar2crane_reference_paths=reference_paths,
    )
    for camera_index in range(2):
        camera_diagnoses[camera_index].update.assert_called_once_with(error_config)
        camera_diagnoses[
            camera_index
        ].diagnose_calibration_data.assert_called_once_with(
            camera_index,
            calibration_conf,
            ActionErrorIndex(ActionErrorIndex.CAMERA0_CALIB_DATA_INVALID + camera_index),
        )
    camera_diagnoses[2].update.assert_not_called()
    camera_diagnoses[3].update.assert_not_called()


def test_startup_calibration_diagnosis_ignores_negative_camera_count() -> None:
    sensor_diagnosis = MagicMock(spec=SensorCalibDataInvalidDiagnosis)
    camera_diagnosis = MagicMock(spec=CameraXCalibDataInvalidDiagnosis)
    shared_errors = SimpleNamespace(
        action_errors_A_C={
            ActionErrorIndex.SENSOR_CALIB_DATA_INVALID: sensor_diagnosis,
            ActionErrorIndex.CAMERA0_CALIB_DATA_INVALID: camera_diagnosis,
            ActionErrorIndex.CAMERA1_CALIB_DATA_INVALID: camera_diagnosis,
            ActionErrorIndex.CAMERA2_CALIB_DATA_INVALID: camera_diagnosis,
            ActionErrorIndex.CAMERA3_CALIB_DATA_INVALID: camera_diagnosis,
        }
    )

    diagnose_startup_calibration_data(
        shared_errors,
        MagicMock(),
        [],
        camera_count=-1,
        error_config=ErrorConfig(),
    )

    camera_diagnosis.update.assert_not_called()
    camera_diagnosis.diagnose_calibration_data.assert_not_called()


def test_startup_sensor_validation_error_is_reported_as_ce006() -> None:
    error = ValueError(
        "lidar2crane_reference_paths must match Lidar_calib_files length"
    )
    sensor_diagnosis = MagicMock(spec=SensorCalibDataInvalidDiagnosis)
    sensor_diagnosis.diagnose_calibration_matrices.side_effect = error
    sensor_diagnosis.excepts_diagnosis.return_value = True
    camera_diagnosis = MagicMock(spec=CameraXCalibDataInvalidDiagnosis)
    shared_errors = SimpleNamespace(
        action_errors_A_C={
            ActionErrorIndex.SENSOR_CALIB_DATA_INVALID: sensor_diagnosis,
            ActionErrorIndex.CAMERA0_CALIB_DATA_INVALID: camera_diagnosis,
            ActionErrorIndex.CAMERA1_CALIB_DATA_INVALID: camera_diagnosis,
            ActionErrorIndex.CAMERA2_CALIB_DATA_INVALID: camera_diagnosis,
            ActionErrorIndex.CAMERA3_CALIB_DATA_INVALID: camera_diagnosis,
        }
    )

    diagnose_startup_calibration_data(
        shared_errors,
        MagicMock(),
        ["lidar0.csv"],
        camera_count=1,
        error_config=ErrorConfig(),
    )

    sensor_diagnosis.excepts_diagnosis.assert_called_once_with(error)
    sensor_diagnosis.log_output.assert_called_once_with(
        True,
        False,
        ActionErrorIndex.SENSOR_CALIB_DATA_INVALID,
        error,
    )
    camera_diagnosis.diagnose_calibration_data.assert_called_once()


def test_startup_sensor_unclassified_error_is_reraised() -> None:
    error = TypeError("unexpected validator failure")
    sensor_diagnosis = MagicMock(spec=SensorCalibDataInvalidDiagnosis)
    sensor_diagnosis.diagnose_calibration_matrices.side_effect = error
    sensor_diagnosis.excepts_diagnosis.return_value = False
    shared_errors = SimpleNamespace(
        action_errors_A_C={
            ActionErrorIndex.SENSOR_CALIB_DATA_INVALID: sensor_diagnosis,
        }
    )

    try:
        diagnose_startup_calibration_data(
            shared_errors,
            MagicMock(),
            [],
            camera_count=0,
            error_config=ErrorConfig(),
        )
    except TypeError as caught:
        assert caught is error
    else:
        raise AssertionError("expected unclassified validation error")

    sensor_diagnosis.log_output.assert_not_called()


def test_startup_camera_validation_error_is_reported_and_next_camera_runs() -> None:
    error = FileNotFoundError("camera0 calibration disappeared")
    sensor_diagnosis = MagicMock(spec=SensorCalibDataInvalidDiagnosis)
    camera_diagnoses = [
        MagicMock(spec=CameraXCalibDataInvalidDiagnosis) for _ in range(4)
    ]
    camera_diagnoses[0].diagnose_calibration_data.side_effect = error
    camera_diagnoses[0].excepts_diagnosis.return_value = True
    shared_errors = SimpleNamespace(
        action_errors_A_C={
            ActionErrorIndex.SENSOR_CALIB_DATA_INVALID: sensor_diagnosis,
            ActionErrorIndex.CAMERA0_CALIB_DATA_INVALID: camera_diagnoses[0],
            ActionErrorIndex.CAMERA1_CALIB_DATA_INVALID: camera_diagnoses[1],
            ActionErrorIndex.CAMERA2_CALIB_DATA_INVALID: camera_diagnoses[2],
            ActionErrorIndex.CAMERA3_CALIB_DATA_INVALID: camera_diagnoses[3],
        }
    )

    diagnose_startup_calibration_data(
        shared_errors,
        MagicMock(),
        [],
        camera_count=2,
        error_config=ErrorConfig(),
    )

    camera_diagnoses[0].excepts_diagnosis.assert_called_once_with(error)
    camera_diagnoses[0].log_output.assert_called_once_with(
        True,
        False,
        ActionErrorIndex.CAMERA0_CALIB_DATA_INVALID,
        error,
    )
    camera_diagnoses[1].diagnose_calibration_data.assert_called_once()


def test_startup_camera_unclassified_error_is_reraised() -> None:
    error = TypeError("unexpected camera validator failure")
    sensor_diagnosis = MagicMock(spec=SensorCalibDataInvalidDiagnosis)
    camera_diagnosis = MagicMock(spec=CameraXCalibDataInvalidDiagnosis)
    camera_diagnosis.diagnose_calibration_data.side_effect = error
    camera_diagnosis.excepts_diagnosis.return_value = False
    shared_errors = SimpleNamespace(
        action_errors_A_C={
            ActionErrorIndex.SENSOR_CALIB_DATA_INVALID: sensor_diagnosis,
            ActionErrorIndex.CAMERA0_CALIB_DATA_INVALID: camera_diagnosis,
        }
    )

    try:
        diagnose_startup_calibration_data(
            shared_errors,
            MagicMock(),
            [],
            camera_count=1,
            error_config=ErrorConfig(),
        )
    except TypeError as caught:
        assert caught is error
    else:
        raise AssertionError("expected unclassified camera validation error")

    camera_diagnosis.log_output.assert_not_called()