from __future__ import annotations

from unittest.mock import MagicMock

import pytest

from argus_synchro.calibration_mat_generator_modules.ctrl.calibration2d3d import (
    calibration2d3d_class,
)
from argus_synchro.diagnosis.calib2d3d_result_diagnosis import (
    Calib2d3dDiagnosisPhase,
    Calib2d3dDiagnosisSession,
    Calib2d3dErrorCommon,
    Calib2d3dResultDiagnosis,
    CameraCalibrationStatus,
    CameraCalibrationStatusDiagnosis,
    is_fatal_calib2d3d_error,
    normalize_calib2d3d_error_common,
    normalize_camera_calibration_status,
)


def test_ui_code_values_are_stable() -> None:
    assert int(Calib2d3dErrorCommon.DEFAULT) == 0
    assert int(Calib2d3dErrorCommon.UNSUITABLE_CONDITION) == 7
    assert int(Calib2d3dErrorCommon.LONG_DURATION_CALIBRATION) == 16
    assert int(Calib2d3dErrorCommon.TRACKING_IMPOSSIBLE) == 18
    assert int(CameraCalibrationStatus.CALIBRATION_SUCCEEDED) == 1
    assert int(CameraCalibrationStatus.CALIBRATION_MATRIX_INVALID) == 6
    assert int(CameraCalibrationStatus.CALIBRATION_ABORTED) == 7


@pytest.mark.parametrize("value", [16, 17, 18, 31])
def test_common_errors_16_through_31_are_fatal(value: int) -> None:
    assert is_fatal_calib2d3d_error(value)


@pytest.mark.parametrize("value", [0, 1, 7, 15])
def test_common_errors_0_through_15_are_not_fatal(value: int) -> None:
    assert not is_fatal_calib2d3d_error(value)


def test_invalid_ui_codes_are_rejected() -> None:
    with pytest.raises(ValueError, match="common error"):
        normalize_calib2d3d_error_common(32)
    with pytest.raises(ValueError, match="camera calibration status"):
        normalize_camera_calibration_status(8)


def test_runtime_warning_does_not_change_camera_status() -> None:
    session = Calib2d3dDiagnosisSession(camera_id=2)

    result = session.diagnose_runtime(
        Calib2d3dErrorCommon.POOR_PERSON_DETECTION
    )

    assert result.camera_id == 2
    assert result.common_error is Calib2d3dErrorCommon.POOR_PERSON_DETECTION
    assert result.camera_status is None


def test_runtime_finding_keeps_phase_message_and_details_for_logging() -> None:
    session = Calib2d3dDiagnosisSession(camera_id=2)

    result = session.diagnose_runtime(
        Calib2d3dErrorCommon.POOR_TRACKING_2D,
        phase=Calib2d3dDiagnosisPhase.TRACKING_2D,
        details={
            "tracked_frame_count": 8,
            "required_frame_count": 50,
        },
    )

    assert len(result.findings) == 1
    finding = result.findings[0]
    assert finding.camera_id == 2
    assert finding.phase is Calib2d3dDiagnosisPhase.TRACKING_2D
    assert finding.common_error is Calib2d3dErrorCommon.POOR_TRACKING_2D
    assert finding.camera_status is None
    assert finding.message == "2D側の追跡不具合"
    assert finding.details == {
        "tracked_frame_count": 8,
        "required_frame_count": 50,
    }


def test_duplicate_finding_is_recorded_only_once() -> None:
    session = Calib2d3dDiagnosisSession(camera_id=0)

    for _ in range(3):
        session.diagnose_runtime(
            Calib2d3dErrorCommon.POOR_PERSON_DETECTION,
            phase=Calib2d3dDiagnosisPhase.PERSON_DETECTION,
        )

    assert len(session.current_result.findings) == 1


def test_fatal_runtime_error_sets_camera_status_to_aborted() -> None:
    session = Calib2d3dDiagnosisSession(camera_id=1)

    result = session.diagnose_runtime(
        Calib2d3dErrorCommon.PERSON_DETECTION_IMPOSSIBLE
    )

    assert result.common_error is Calib2d3dErrorCommon.PERSON_DETECTION_IMPOSSIBLE
    assert result.camera_status is CameraCalibrationStatus.CALIBRATION_ABORTED


def test_fatal_runtime_error_is_latched_until_session_reset() -> None:
    session = Calib2d3dDiagnosisSession(camera_id=1)
    session.diagnose_runtime(Calib2d3dErrorCommon.TRACKING_IMPOSSIBLE)

    latched = session.diagnose_runtime(Calib2d3dErrorCommon.DEFAULT)
    assert latched.common_error is Calib2d3dErrorCommon.TRACKING_IMPOSSIBLE
    assert latched.camera_status is CameraCalibrationStatus.CALIBRATION_ABORTED

    session.reset(camera_id=0)
    reset = session.diagnose_runtime()
    assert reset.camera_id == 0
    assert reset.common_error is Calib2d3dErrorCommon.DEFAULT
    assert reset.camera_status is None
    assert reset.findings == ()


def test_fatal_runtime_error_takes_priority_over_final_success() -> None:
    session = Calib2d3dDiagnosisSession(camera_id=0)
    session.diagnose_runtime(Calib2d3dErrorCommon.LONG_DURATION_CALIBRATION)

    result = session.diagnose_final(
        CameraCalibrationStatus.CALIBRATION_SUCCEEDED
    )

    assert result.camera_status is CameraCalibrationStatus.CALIBRATION_ABORTED


def test_recovered_nonfatal_runtime_warning_is_not_inherited_by_final_result() -> None:
    session = Calib2d3dDiagnosisSession(camera_id=0)
    session.diagnose_runtime(Calib2d3dErrorCommon.WALKING_PERSON_COUNT_INVALID)
    recovered = session.diagnose_runtime(Calib2d3dErrorCommon.DEFAULT)

    assert recovered.common_error is Calib2d3dErrorCommon.DEFAULT
    result = session.diagnose_final(CameraCalibrationStatus.CALIBRATION_SUCCEEDED)
    assert result.camera_status is CameraCalibrationStatus.CALIBRATION_SUCCEEDED


def test_final_finding_keeps_calculation_evidence() -> None:
    session = Calib2d3dDiagnosisSession(camera_id=1)

    result = session.diagnose_final(
        CameraCalibrationStatus.CALIBRATION_MATRIX_INVALID,
        phase=Calib2d3dDiagnosisPhase.MATRIX_VALIDATION,
        message="校正行列に非有限値を検出",
        details={"finite": False, "shape": (4, 4)},
    )

    finding = result.findings[0]
    assert finding.phase is Calib2d3dDiagnosisPhase.MATRIX_VALIDATION
    assert finding.common_error is None
    assert (
        finding.camera_status
        is CameraCalibrationStatus.CALIBRATION_MATRIX_INVALID
    )
    assert finding.message == "校正行列に非有限値を検出"
    assert finding.details == {"finite": False, "shape": (4, 4)}


def test_diagnosis_defaults_preserve_existing_behavior() -> None:
    assert Calib2d3dResultDiagnosis().diagnose() is Calib2d3dErrorCommon.DEFAULT
    assert (
        CameraCalibrationStatusDiagnosis().diagnose()
        is CameraCalibrationStatus.DEFAULT
    )


def test_controller_publishes_fatal_common_and_camera_status_together() -> None:
    calibration = object.__new__(calibration2d3d_class)
    calibration._diagnosis_session = Calib2d3dDiagnosisSession(camera_id=2)
    monitor = MagicMock()

    calibration._update_errors_calibcommon(
        monitor,
        Calib2d3dErrorCommon.TRACKING_IMPOSSIBLE,
    )

    monitor.set_errors_calibcommon.assert_called_once_with(18)
    monitor.set_camera_calibration_status.assert_called_once_with(
        camera_id=2,
        value=7,
    )


def test_controller_warning_does_not_overwrite_camera_status() -> None:
    calibration = object.__new__(calibration2d3d_class)
    calibration._diagnosis_session = Calib2d3dDiagnosisSession(camera_id=2)
    monitor = MagicMock()

    calibration._update_errors_calibcommon(
        monitor,
        Calib2d3dErrorCommon.POOR_TRACKING_2D,
    )

    monitor.set_errors_calibcommon.assert_called_once_with(6)
    monitor.set_camera_calibration_status.assert_not_called()
