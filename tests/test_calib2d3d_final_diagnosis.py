from __future__ import annotations

import numpy as np

from argus_synchro.diagnosis.calib2d3d_result_diagnosis import (
    Calib2d3dFinalDiagnosis,
    Calib2d3dFinalDiagnosisConfig,
    Calib2d3dFinalObservation,
    CameraCalibrationStatus,
)


def _observation(**overrides: object) -> Calib2d3dFinalObservation:
    values: dict[str, object] = {
        "matrix": np.eye(4),
        "accuracy_value": 1.0,
        "accuracy_threshold": 2.0,
        "accuracy_check_enabled": True,
    }
    values.update(overrides)
    return Calib2d3dFinalObservation(**values)  # type: ignore[arg-type]


def test_valid_matrix_succeeds_even_when_runtime_warnings_remain() -> None:
    result = Calib2d3dFinalDiagnosis().diagnose(
        _observation(
            poor_tracking_3d=True,
            poor_tracking_2d=True,
            poor_sensor_correspondence=True,
            invalid_person_count=True,
        )
    )
    assert result.status is CameraCalibrationStatus.CALIBRATION_SUCCEEDED


def test_accuracy_failure_is_matrix_invalid() -> None:
    result = Calib2d3dFinalDiagnosis().diagnose(
        _observation(accuracy_value=2.0, accuracy_threshold=2.0)
    )
    assert result.status is CameraCalibrationStatus.CALIBRATION_MATRIX_INVALID


def test_invalid_shape_and_nonfinite_matrix_are_rejected() -> None:
    diagnosis = Calib2d3dFinalDiagnosis()
    assert (
        diagnosis.diagnose(_observation(matrix=np.eye(3))).status
        is CameraCalibrationStatus.CALIBRATION_MATRIX_INVALID
    )
    matrix = np.eye(4)
    matrix[0, 0] = np.nan
    assert (
        diagnosis.diagnose(_observation(matrix=matrix)).status
        is CameraCalibrationStatus.CALIBRATION_MATRIX_INVALID
    )


def test_each_final_check_can_be_disabled() -> None:
    diagnosis = Calib2d3dFinalDiagnosis(
        Calib2d3dFinalDiagnosisConfig(
            enable_accuracy=False,
            enable_matrix_shape=False,
            enable_matrix_finite=False,
        )
    )
    result = diagnosis.diagnose(
        _observation(
            matrix=np.array([[np.nan]]),
            accuracy_value=100.0,
            poor_tracking_3d=True,
        )
    )
    assert result.status is CameraCalibrationStatus.CALIBRATION_SUCCEEDED


def test_matrix_invalid_has_priority_over_runtime_warnings() -> None:
    result = Calib2d3dFinalDiagnosis().diagnose(
        _observation(
            matrix=np.eye(3),
            invalid_person_count=True,
            poor_tracking_3d=True,
        )
    )
    assert result.status is CameraCalibrationStatus.CALIBRATION_MATRIX_INVALID


def test_reference_difference_is_disabled_by_default() -> None:
    result = Calib2d3dFinalDiagnosis().diagnose(
        _observation(reference_matrix_difference_invalid=True)
    )
    assert result.status is CameraCalibrationStatus.CALIBRATION_SUCCEEDED


def test_enabled_reference_difference_failure_is_matrix_invalid() -> None:
    diagnosis = Calib2d3dFinalDiagnosis(
        Calib2d3dFinalDiagnosisConfig(enable_reference_matrix_difference=True)
    )
    result = diagnosis.diagnose(
        _observation(reference_matrix_difference_invalid=True)
    )
    assert result.status is CameraCalibrationStatus.CALIBRATION_MATRIX_INVALID


def test_skipped_reference_difference_check_does_not_fail() -> None:
    result = Calib2d3dFinalDiagnosis().diagnose(
        _observation(reference_matrix_difference_invalid=None)
    )
    assert result.status is CameraCalibrationStatus.CALIBRATION_SUCCEEDED
