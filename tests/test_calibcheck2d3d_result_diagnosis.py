from __future__ import annotations

import pytest

from argus_synchro.diagnosis.calibcheck2d3d_result_diagnosis import (
    CameraCalibCheckStatus,
    CameraCalibCheckStatusDiagnosis,
    validate_camera_calibcheck_status,
)


def test_camera_calibcheck_status_values_match_ui_contract() -> None:
    assert {status.name: int(status) for status in CameraCalibCheckStatus} == {
        "CALIBRATION_NOT_REQUIRED": 0,
        "FORBIDDEN": 1,
        "CALIBRATION_REQUIRED": 2,
        "UNKNOWN_INSUFFICIENT_DATA": 3,
        "UNKNOWN_PERSON_NOT_DETECTED": 4,
        "UNKNOWN_POOR_PERSON_DETECTION": 5,
        "RESERVED_6": 6,
        "RESERVED_7": 7,
    }


@pytest.mark.parametrize("value", [0, 2, 3, 4, 5])
def test_validate_camera_calibcheck_status_accepts_writable_values(value: int) -> None:
    assert validate_camera_calibcheck_status(value) is CameraCalibCheckStatus(value)


@pytest.mark.parametrize("value", [-1, 1, 6, 7, 8])
def test_validate_camera_calibcheck_status_rejects_non_writable_values(
    value: int,
) -> None:
    with pytest.raises(ValueError):
        validate_camera_calibcheck_status(value)


@pytest.mark.parametrize(
    ("reason_code", "expected"),
    [
        (1, CameraCalibCheckStatus.UNKNOWN_INSUFFICIENT_DATA),
        (2, CameraCalibCheckStatus.UNKNOWN_INSUFFICIENT_DATA),
        (3, CameraCalibCheckStatus.UNKNOWN_INSUFFICIENT_DATA),
        (4, CameraCalibCheckStatus.UNKNOWN_POOR_PERSON_DETECTION),
        (5, CameraCalibCheckStatus.UNKNOWN_POOR_PERSON_DETECTION),
        (6, CameraCalibCheckStatus.UNKNOWN_POOR_PERSON_DETECTION),
        (7, CameraCalibCheckStatus.UNKNOWN_POOR_PERSON_DETECTION),
        (8, CameraCalibCheckStatus.UNKNOWN_POOR_PERSON_DETECTION),
        (9, CameraCalibCheckStatus.UNKNOWN_PERSON_NOT_DETECTED),
        (10, CameraCalibCheckStatus.UNKNOWN_INSUFFICIENT_DATA),
        (11, CameraCalibCheckStatus.UNKNOWN_PERSON_NOT_DETECTED),
        (99, CameraCalibCheckStatus.UNKNOWN_INSUFFICIENT_DATA),
    ],
)
def test_diagnosis_maps_failure_reason_to_ui_status(
    reason_code: int,
    expected: CameraCalibCheckStatus,
) -> None:
    diagnosis = CameraCalibCheckStatusDiagnosis()

    assert diagnosis.diagnose(
        reason_code=reason_code,
        calibration_is_acceptable=True,
    ) is expected


def test_diagnosis_maps_acceptable_result_to_not_required() -> None:
    diagnosis = CameraCalibCheckStatusDiagnosis()

    assert diagnosis.diagnose(
        reason_code=0,
        calibration_is_acceptable=True,
    ) is CameraCalibCheckStatus.CALIBRATION_NOT_REQUIRED


def test_diagnosis_maps_unacceptable_result_to_required() -> None:
    diagnosis = CameraCalibCheckStatusDiagnosis()

    assert diagnosis.diagnose(
        reason_code=0,
        calibration_is_acceptable=False,
    ) is CameraCalibCheckStatus.CALIBRATION_REQUIRED
