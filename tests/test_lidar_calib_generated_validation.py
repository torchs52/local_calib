from types import SimpleNamespace
from unittest.mock import MagicMock

import numpy as np

from argus_synchro.calibration_mat_generator_modules.ctrl.calibration3d3d import (
    calibration3d3d_class,
)
from argus_synchro.diagnosis.action_errors import SensorCalibDataInvalidDiagnosis
from argus_synchro.diagnosis.calib_matrix_validator import LidarCalibValidator
from argus_synchro.diagnosis.error_config import SensorCalibDataInvalidParameters
from argus_synchro.shared_errors import ActionErrorIndex


def _validator() -> LidarCalibValidator:
    return LidarCalibValidator(SensorCalibDataInvalidParameters())


def test_generated_lidar_calibration_accepts_valid_matrix() -> None:
    matrix = np.eye(4, dtype=np.float64)

    assert _validator().validate_matrices([matrix], matrix_paths=["lidar0.csv"]) == []


def test_generated_lidar_calibration_uses_startup_validation_rules() -> None:
    invalid = np.eye(4, dtype=np.float64)
    invalid[0, 0] = np.nan
    wrong_shape = np.eye(3, dtype=np.float64)

    invalid_issues = _validator().validate_matrices([invalid])
    shape_issues = _validator().validate_matrices([wrong_shape])

    assert any("non-finite" in issue.detail for issue in invalid_issues)
    assert any("matrix shape" in issue.detail for issue in shape_issues)


def test_generated_lidar_calibration_rejects_mismatched_metadata_lengths() -> None:
    matrix = np.eye(4, dtype=np.float64)

    try:
        _validator().validate_matrices([matrix], matrix_paths=[])
    except ValueError as error:
        assert str(error) == "matrix_paths must have the same length as matrices"
    else:
        raise AssertionError("expected matrix path length validation")


def test_generated_lidar_calibration_detects_reference_pose_difference() -> None:
    params = SensorCalibDataInvalidParameters(
        translation_threshold_m=0.5,
        rotation_threshold_deg=10.0,
        max_xy_displacement_threshold_m=1.0,
    )
    estimated = np.eye(4, dtype=np.float64)
    estimated[0, 3] = 1.1

    issues = LidarCalibValidator(params).validate_matrices(
        [estimated],
        matrix_paths=["lidar0.csv"],
        reference_matrices=[np.eye(4, dtype=np.float64)],
    )

    assert [issue.detail.split(":", maxsplit=1)[0] for issue in issues] == [
        "translation error exceeds threshold",
        "max XY displacement exceeds threshold",
    ]


def test_generated_validation_uses_sensor_calibration_diagnosis_policy() -> None:
    diagnosis = SensorCalibDataInvalidDiagnosis()
    diagnosis.param = SensorCalibDataInvalidParameters()
    diagnosis.is_enabled = True
    invalid = np.eye(4, dtype=np.float64)
    invalid[0, 0] = np.nan

    diagnosis._logger = MagicMock()
    issues = diagnosis.diagnose_matrices(
        [invalid],
        ActionErrorIndex.SENSOR_CALIB_DATA_INVALID,
        matrix_paths=["generated-lidar0.csv"],
    )

    assert len(issues) == 1
    assert issues[0].matrix_path == "generated-lidar0.csv"
    assert issues[0].detail == "matrix contains non-finite values"
    assert diagnosis.err_cnt.value == 1
    diagnosis._logger.error.assert_called_once()


def test_calibration3d3d_reports_generated_matrix_validation_result(
    monkeypatch,
) -> None:
    diagnosis = MagicMock()
    issue = SimpleNamespace(detail="matrix contains non-finite values")
    diagnosis.diagnose_matrices.return_value = (issue,)
    calibration = object.__new__(calibration3d3d_class)
    calibration._ser = SimpleNamespace(
        action_errors_A_C={ActionErrorIndex.SENSOR_CALIB_DATA_INVALID: diagnosis}
    )
    calibration._logger = MagicMock()
    matrix = np.eye(4, dtype=np.float64)
    monkeypatch.setattr(
        "argus_synchro.calibration_mat_generator_modules.ctrl.calibration3d3d.calibrateLidars2Crane",
        lambda **kwargs: (MagicMock(), [], [matrix], MagicMock()),
    )
    reference_paths = ["reference-lidar0.csv"]
    app_config_calib = SimpleNamespace(
        Calib3d3d_CalibParams=SimpleNamespace(lidars_calib_path=reference_paths)
    )

    assert calibration.calib3d3d_once(
        lidar_pts=[np.empty((0, 3), dtype=np.float64)],
        angle_data=0.0,
        resultmat_paths=["lidar0.csv"],
        app_config=MagicMock(),
        app_config_calib=app_config_calib,
    )
    diagnosis.diagnose_matrices.assert_called_once_with(
        [matrix],
        ActionErrorIndex.SENSOR_CALIB_DATA_INVALID,
        matrix_paths=["lidar0.csv"],
        reference_matrix_paths=reference_paths,
    )


def test_calibration3d3d_sets_ui_error_when_input_is_missing() -> None:
    calibration = object.__new__(calibration3d3d_class)
    calibration._logger = MagicMock()
    calibration.validcount = 0
    monitor = MagicMock()

    assert (
        calibration.dataproc(
            readresult_pop=None,  # type: ignore[arg-type]
            monitor=monitor,
            sec=MagicMock(),
            sac=MagicMock(),
            app_config_calib=MagicMock(),
            resultmat_paths=[],
        )
        is False
    )
    monitor.set_errors_calibcommon.assert_called_once_with(2)