from __future__ import annotations

import pytest

from argus_synchro.diagnosis.calibcheck2d3d_result_diagnosis import (
    BBoxLogObservation,
    CalibCheck2d3dDiagnosis,
    CalibCheckFailureReason,
    CalibCheckFrameObservation,
    CameraCalibCheckStatus,
    EvaluationMatchingStatistics,
    EvaluationObservation,
    EvaluationStatistics,
    EvaluationTimeSyncStatistics,
    JudgementObservation,
    StatisticsObservation,
    TrackingObservation,
    calibcheck_reason_to_status,
    validate_camera_calibcheck_status,
)


def test_camera_calibcheck_status_values_match_ui_contract() -> None:
    assert {status.name: int(status) for status in CameraCalibCheckStatus} == {
        "CALIBRATION_NOT_REQUIRED": 0,
        "FORBIDDEN": 1,
        "CALIBRATION_REQUIRED": 2,
        "UNKNOWN_INSUFFICIENT_DATA": 3,
        "PERSON_NOT_DETECTED": 4,
        "POOR_PERSON_DETECTION": 5,
        "POOR_SENSOR_DATA": 6,
        "POOR_EVALUATION_RESULT": 7,
    }


@pytest.mark.parametrize("value", [0, 2, 3, 4, 5, 6, 7])
def test_validate_camera_calibcheck_status_accepts_writable_values(value: int) -> None:
    assert validate_camera_calibcheck_status(value) is CameraCalibCheckStatus(value)


@pytest.mark.parametrize("value", [-1, 1, 8])
def test_validate_camera_calibcheck_status_rejects_non_writable_values(
    value: int,
) -> None:
    with pytest.raises(ValueError):
        validate_camera_calibcheck_status(value)


@pytest.mark.parametrize(
    ("reason_code", "expected"),
    [
        (1, CameraCalibCheckStatus.POOR_SENSOR_DATA),
        (2, CameraCalibCheckStatus.UNKNOWN_INSUFFICIENT_DATA),
        (3, CameraCalibCheckStatus.UNKNOWN_INSUFFICIENT_DATA),
        (4, CameraCalibCheckStatus.POOR_PERSON_DETECTION),
        (5, CameraCalibCheckStatus.POOR_PERSON_DETECTION),
        (6, CameraCalibCheckStatus.POOR_EVALUATION_RESULT),
        (7, CameraCalibCheckStatus.POOR_EVALUATION_RESULT),
        (8, CameraCalibCheckStatus.POOR_EVALUATION_RESULT),
        (9, CameraCalibCheckStatus.PERSON_NOT_DETECTED),
        (10, CameraCalibCheckStatus.UNKNOWN_INSUFFICIENT_DATA),
        (11, CameraCalibCheckStatus.PERSON_NOT_DETECTED),
        (99, CameraCalibCheckStatus.UNKNOWN_INSUFFICIENT_DATA),
    ],
)
def test_diagnosis_maps_failure_reason_to_ui_status(
    reason_code: int,
    expected: CameraCalibCheckStatus,
) -> None:
    assert calibcheck_reason_to_status(
        reason_code,
        calibration_is_acceptable=True,
    ) is expected


def test_diagnosis_maps_acceptable_result_to_not_required() -> None:
    assert calibcheck_reason_to_status(
        0,
        calibration_is_acceptable=True,
    ) is CameraCalibCheckStatus.CALIBRATION_NOT_REQUIRED


def test_diagnosis_maps_unacceptable_result_to_required() -> None:
    assert calibcheck_reason_to_status(
        0,
        calibration_is_acceptable=False,
    ) is CameraCalibCheckStatus.CALIBRATION_REQUIRED


def test_diagnosis_session_collects_stage_reasons_per_camera() -> None:
    diagnosis = CalibCheck2d3dDiagnosis(camera_count=3)

    diagnosis.diagnose_bbox_logs(
        BBoxLogObservation(
            total_frame_count=10,
            valid_3d_frame_count=10,
            valid_2d_frame_counts=(10, 0, 10),
            minimum_3d_bbox_count_per_frame=1,
            minimum_3d_valid_frame_ratio=0.5,
            minimum_2d_bbox_count_per_frame=1,
            minimum_2d_valid_frame_ratio=0.5,
        )
    )
    diagnosis.diagnose_tracking(
        TrackingObservation(
            total_3d_track_count=1,
            alive_3d_track_count=1,
            total_2d_track_counts=(1, 1, 1),
            alive_2d_track_counts=(1, 1, 0),
            minimum_3d_alive_track_count=1,
            minimum_2d_alive_track_count=1,
            proximity_warning_count=0,
            proximity_warning_fails_validation=True,
        )
    )
    diagnosis.diagnose_evaluation(
        EvaluationObservation(
            scores=(1.0, 0.0, 0.0),
            reasons=(
                CalibCheckFailureReason.NONE,
                CalibCheckFailureReason.NONE,
                CalibCheckFailureReason.MATCHED_PERSON_NOT_DETECTED,
            ),
        )
    )

    results = diagnosis.finalize((True, False, False))

    assert results[0].status is CameraCalibCheckStatus.CALIBRATION_NOT_REQUIRED
    assert results[1].primary_reason is CalibCheckFailureReason.BBOX2D_LOG_INVALID
    assert results[1].status is CameraCalibCheckStatus.UNKNOWN_INSUFFICIENT_DATA
    assert results[2].all_reasons == (
        CalibCheckFailureReason.TRACKING2D_INVALID,
        CalibCheckFailureReason.MATCHED_PERSON_NOT_DETECTED,
    )


def test_out_of_range_evaluation_score_is_recorded_as_reason_6() -> None:
    diagnosis = CalibCheck2d3dDiagnosis(camera_count=1)
    invalid_score = 1.1

    diagnosis.diagnose_evaluation(
        EvaluationObservation(
            scores=(invalid_score,),
            reasons=(CalibCheckFailureReason.NONE,),
        )
    )

    finding = diagnosis.findings[0]
    assert finding.reason is CalibCheckFailureReason.EVALUATION_RESULT_INVALID
    assert finding.details["score"] == invalid_score


def test_time_sync_failure_records_frame_correspondence_as_reason_10() -> None:
    diagnosis = CalibCheck2d3dDiagnosis(camera_count=1)

    diagnosis.diagnose_evaluation(
        EvaluationObservation(
            scores=(0.0,),
            reasons=(CalibCheckFailureReason.SENSOR_TIME_SYNC_INVALID,),
            time_sync_statistics=(
                EvaluationTimeSyncStatistics(
                    visible_3d_frame_count=3,
                    tracked_2d_frame_count=2,
                    common_frame_count=0,
                    visible_3d_frame_range=(10, 12),
                    tracked_2d_frame_range=(20, 21),
                ),
            ),
        )
    )

    finding = diagnosis.findings[0]
    assert finding.reason is CalibCheckFailureReason.SENSOR_TIME_SYNC_INVALID
    assert finding.details["common_frame_count"] == 0
    assert finding.details["visible_3d_frame_range"] == (10, 12)
    assert finding.details["tracked_2d_frame_range"] == (20, 21)


@pytest.mark.parametrize(
    ("visible_3d_frame_count", "failure_stage"),
    [
        (0, "no_visible_3d_projection"),
        (3, "no_valid_2d3d_comparison"),
    ],
)
def test_matching_failure_records_detailed_reason_11(
    visible_3d_frame_count: int,
    failure_stage: str,
) -> None:
    diagnosis = CalibCheck2d3dDiagnosis(camera_count=1)

    diagnosis.diagnose_evaluation(
        EvaluationObservation(
            scores=(0.0,),
            reasons=(CalibCheckFailureReason.MATCHED_PERSON_NOT_DETECTED,),
            matching_statistics=(
                EvaluationMatchingStatistics(
                    tracked_3d_frame_count=3,
                    visible_3d_frame_count=visible_3d_frame_count,
                    tracked_2d_frame_count=3,
                    common_frame_count=3 if visible_3d_frame_count else 0,
                    comparison_candidate_count=(
                        3 if visible_3d_frame_count else 0
                    ),
                    overlap_candidate_count=0,
                    score_returned_count=0,
                    valid_comparison_count=0,
                ),
            ),
        )
    )

    finding = diagnosis.findings[0]
    assert finding.reason is CalibCheckFailureReason.MATCHED_PERSON_NOT_DETECTED
    assert finding.details["failure_stage"] == failure_stage
    assert finding.details["valid_comparison_count"] == 0


def test_bbox_log_diagnosis_accepts_ratio_equal_to_threshold() -> None:
    diagnosis = CalibCheck2d3dDiagnosis(camera_count=2)

    validity = diagnosis.diagnose_bbox_logs(
        BBoxLogObservation(
            total_frame_count=10,
            valid_3d_frame_count=5,
            valid_2d_frame_counts=(5, 4),
            minimum_3d_bbox_count_per_frame=1,
            minimum_3d_valid_frame_ratio=0.5,
            minimum_2d_bbox_count_per_frame=1,
            minimum_2d_valid_frame_ratio=0.5,
        )
    )

    assert validity == (True, False)
    finding = diagnosis.findings[0]
    assert finding.reason is CalibCheckFailureReason.BBOX2D_LOG_INVALID
    assert finding.camera_index == 1
    assert finding.details == {
        "total_frames": 10,
        "valid_frames": 4,
        "valid_frame_ratio": 0.4,
        "minimum_bbox_count_per_frame": 1,
        "minimum_valid_frame_ratio": 0.5,
    }


def test_empty_bbox_log_records_common_3d_and_camera_specific_2d_reasons() -> None:
    diagnosis = CalibCheck2d3dDiagnosis(camera_count=2)

    validity = diagnosis.diagnose_bbox_logs(
        BBoxLogObservation(
            total_frame_count=0,
            valid_3d_frame_count=0,
            valid_2d_frame_counts=(0, 0),
            minimum_3d_bbox_count_per_frame=1,
            minimum_3d_valid_frame_ratio=0.1,
            minimum_2d_bbox_count_per_frame=1,
            minimum_2d_valid_frame_ratio=0.1,
        )
    )

    assert validity == (False, False)
    finding_keys = [
        (finding.reason, finding.camera_index) for finding in diagnosis.findings
    ]
    assert finding_keys == [
        (CalibCheckFailureReason.BBOX3D_LOG_INVALID, None),
        (CalibCheckFailureReason.BBOX2D_LOG_INVALID, 0),
        (CalibCheckFailureReason.BBOX2D_LOG_INVALID, 1),
    ]


def test_frame_diagnosis_uses_elapsed_time_for_person_not_detected() -> None:
    elapsed_times = iter((100.0, 111.0))
    diagnosis = CalibCheck2d3dDiagnosis(
        camera_count=2,
        person_not_detected_sec=10.0,
        sync_invalid_frame_threshold=2,
        elapsed_time=lambda: next(elapsed_times),
    )
    observation = CalibCheckFrameObservation(
        camera_detection_counts=(0, 1),
        synchronization_valid=(True, False),
    )

    diagnosis.diagnose_frame(observation)
    assert diagnosis.findings == []
    diagnosis.diagnose_frame(observation)

    results = diagnosis.finalize((False, False))
    assert (
        results[0].primary_reason
        is CalibCheckFailureReason.CAMERA_PERSON_NOT_DETECTED
    )
    assert (
        results[1].primary_reason
        is CalibCheckFailureReason.SENSOR_TIME_SYNC_INVALID
    )


def test_person_detection_during_observation_prevents_reason_9() -> None:
    elapsed_times = iter((100.0, 105.0, 111.0))
    diagnosis = CalibCheck2d3dDiagnosis(
        camera_count=1,
        person_not_detected_sec=10.0,
        elapsed_time=lambda: next(elapsed_times),
    )

    diagnosis.diagnose_frame(CalibCheckFrameObservation(camera_detection_counts=(0,)))
    diagnosis.diagnose_frame(CalibCheckFrameObservation(camera_detection_counts=(1,)))
    diagnosis.diagnose_frame(CalibCheckFrameObservation(camera_detection_counts=(0,)))

    result = diagnosis.finalize((True,))[0]
    assert result.primary_reason is CalibCheckFailureReason.NONE
    assert diagnosis.frame_state.camera_person_detected_frames == [1]
    assert diagnosis.frame_state.max_consecutive_person_missing == [1]


def test_person_detection_elapsed_time_must_reach_threshold() -> None:
    elapsed_times = iter((100.0, 109.9))
    diagnosis = CalibCheck2d3dDiagnosis(
        camera_count=1,
        person_not_detected_sec=10.0,
        elapsed_time=lambda: next(elapsed_times),
    )

    diagnosis.diagnose_frame(CalibCheckFrameObservation(camera_detection_counts=(0,)))
    diagnosis.diagnose_frame(CalibCheckFrameObservation(camera_detection_counts=(0,)))

    assert (
        diagnosis.finalize((False,))[0].primary_reason
        is CalibCheckFailureReason.NONE
    )


def test_invalid_judgement_is_recorded_as_reason_8() -> None:
    diagnosis = CalibCheck2d3dDiagnosis(camera_count=1)

    diagnosis.diagnose_judgement(
        JudgementObservation(
            results=(None,),
            scores=(0.8,),
            evaluation_reasons=(CalibCheckFailureReason.NONE,),
            threshold=0.5,
        )
    )

    result = diagnosis.finalize((None,))[0]
    assert (
        result.primary_reason
        is CalibCheckFailureReason.CALIBRATION_JUDGEMENT_INVALID
    )
    assert result.status is CameraCalibCheckStatus.POOR_EVALUATION_RESULT


def test_invalid_statistics_is_recorded_as_reason_7() -> None:
    diagnosis = CalibCheck2d3dDiagnosis(camera_count=2)

    diagnosis.diagnose_statistics(
        StatisticsObservation(
            statistics=(
                EvaluationStatistics(4, 5, 0.8, 4.0, 5, 0.8, 0.8, False),
                EvaluationStatistics(
                    4, 5, 0.8, 4.0, 5, float("nan"), float("nan"), True
                ),
            ),
            evaluation_reasons=(
                CalibCheckFailureReason.NONE,
                CalibCheckFailureReason.NONE,
            ),
            minimum_sample_count=5,
        )
    )

    results = diagnosis.finalize((True, None))
    assert results[0].primary_reason is CalibCheckFailureReason.NONE
    assert (
        results[1].primary_reason
        is CalibCheckFailureReason.EVALUATION_STATISTICS_INVALID
    )


def test_insufficient_evaluation_samples_are_recorded_as_reason_7() -> None:
    diagnosis = CalibCheck2d3dDiagnosis(camera_count=1)
    observed_sample_count = 2

    diagnosis.diagnose_statistics(
        StatisticsObservation(
            statistics=(
                EvaluationStatistics(
                    1,
                    observed_sample_count,
                    0.5,
                    1.0,
                    observed_sample_count,
                    0.5,
                    0.5,
                    False,
                ),
            ),
            evaluation_reasons=(CalibCheckFailureReason.NONE,),
            minimum_sample_count=3,
        )
    )

    finding = diagnosis.findings[0]
    assert finding.reason is CalibCheckFailureReason.EVALUATION_STATISTICS_INVALID
    assert finding.details["selected_sample_count"] == observed_sample_count
    assert finding.details["sample_count_is_sufficient"] is False


def test_statistics_does_not_duplicate_evaluation_unavailable_reason() -> None:
    diagnosis = CalibCheck2d3dDiagnosis(camera_count=1)

    diagnosis.diagnose_statistics(
        StatisticsObservation(
            statistics=(
                EvaluationStatistics(0, 0, 0.0, 0.0, 0, 0.0, 0.0, False),
            ),
            evaluation_reasons=(
                CalibCheckFailureReason.MATCHED_PERSON_NOT_DETECTED,
            ),
            minimum_sample_count=5,
        )
    )

    assert diagnosis.findings == []


def test_judgement_inconsistent_with_score_is_recorded_as_reason_8() -> None:
    diagnosis = CalibCheck2d3dDiagnosis(camera_count=1)

    diagnosis.diagnose_judgement(
        JudgementObservation(
            results=(False,),
            scores=(0.8,),
            evaluation_reasons=(CalibCheckFailureReason.NONE,),
            threshold=0.5,
        )
    )

    finding = diagnosis.findings[0]
    assert finding.reason is CalibCheckFailureReason.CALIBRATION_JUDGEMENT_INVALID
    assert finding.details["expected_result"] is True
