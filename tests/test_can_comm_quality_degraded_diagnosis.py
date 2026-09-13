from __future__ import annotations

import argus_synchro.diagnosis.error_config as ErrConf
from argus_synchro.diagnosis.error_diagnosis import ResultDiagnosis
from argus_synchro.diagnosis.state_errors import CanCommQualityDegradedDiagnosis

ANGLE_CAN_ID = "18FFD1D1"
LEVER_CAN_ID = "18FC4401"


def _create_diagnosis() -> CanCommQualityDegradedDiagnosis:
    diagnosis = CanCommQualityDegradedDiagnosis()
    err_conf = ErrConf.ErrorConfig().load_from_dict(
        {
            "can_comm_quality_degraded": {
                "is_enabled": True,
                "error_threshold_count": {
                    ANGLE_CAN_ID: 5,
                    LEVER_CAN_ID: 5,
                },
                "error_threshold_sec": {
                    ANGLE_CAN_ID: 1.0,
                    LEVER_CAN_ID: 1.0,
                },
                "recovery_receive_interval_sec": {
                    ANGLE_CAN_ID: 1.0,
                    LEVER_CAN_ID: 1.0,
                },
                "fail_safe_recovery_receive_interval_sec": {
                    ANGLE_CAN_ID: 1.0,
                    LEVER_CAN_ID: 1.0,
                },
                "recovery_threshold_count": {
                    ANGLE_CAN_ID: 5,
                    LEVER_CAN_ID: 5,
                },
                "fail_safe_recovery_threshold_count": {
                    ANGLE_CAN_ID: 5,
                    LEVER_CAN_ID: 5,
                },
            }
        }
    )
    diagnosis.update(err_conf)
    return diagnosis


def _diagnose(
    diagnosis: CanCommQualityDegradedDiagnosis,
    can_id: str,
    failed_count: int,
    timestamp: float | None,
    now: float,
) -> tuple[ResultDiagnosis, ResultDiagnosis]:
    return diagnosis.errors_diagnosis(can_id, failed_count, timestamp, now)


def test_angle_update_and_stopped_lever_detects_error() -> None:
    diagnosis = _create_diagnosis()

    assert _diagnose(diagnosis, ANGLE_CAN_ID, 0, 0.0, 0.0)[0] == ResultDiagnosis.NORMAL
    assert _diagnose(diagnosis, ANGLE_CAN_ID, 0, 0.5, 0.5)[0] == ResultDiagnosis.NORMAL
    result = _diagnose(diagnosis, ANGLE_CAN_ID, 0, 1.0, 1.0)

    assert result[0] == ResultDiagnosis.DETECTION
    assert diagnosis.is_error.value


def test_one_can_id_reaching_five_failures_detects_degraded_error() -> None:
    diagnosis = _create_diagnosis()

    for failed_count in range(1, 5):
        now = failed_count / 10.0
        result = _diagnose(diagnosis, ANGLE_CAN_ID, failed_count, now, now)
        assert result[0] == ResultDiagnosis.NORMAL

    result = _diagnose(diagnosis, ANGLE_CAN_ID, 5, 0.5, 0.5)

    assert result[0] == ResultDiagnosis.DETECTION
    assert diagnosis.is_error.value


def test_recovery_requires_five_successes_for_both_can_ids() -> None:
    diagnosis = _create_diagnosis()
    _diagnose(diagnosis, ANGLE_CAN_ID, 5, 0.0, 0.0)
    _diagnose(diagnosis, LEVER_CAN_ID, 5, 0.0, 0.0)

    for success_count in range(1, 5):
        angle_result = _diagnose(
            diagnosis, ANGLE_CAN_ID, 0, float(success_count), float(success_count)
        )
        lever_result = _diagnose(
            diagnosis, LEVER_CAN_ID, 0, float(success_count), float(success_count)
        )
        assert angle_result[0] != ResultDiagnosis.RECOVERY
        assert lever_result[0] != ResultDiagnosis.RECOVERY
        assert diagnosis.is_error.value

    final_angle_result = _diagnose(diagnosis, ANGLE_CAN_ID, 0, 5.0, 5.0)
    final_lever_result = _diagnose(diagnosis, LEVER_CAN_ID, 0, 5.0, 5.0)
    assert final_angle_result[0] != ResultDiagnosis.RECOVERY
    assert final_lever_result[0] == ResultDiagnosis.RECOVERY
    assert not diagnosis.is_error.value


def test_empty_can_id_does_not_increment_failure_count_for_any_can_id() -> None:
    diagnosis = _create_diagnosis()

    _diagnose(diagnosis, ANGLE_CAN_ID, 0, 0.0, 0.0)
    _diagnose(diagnosis, LEVER_CAN_ID, 0, 0.0, 0.0)

    for timeout_count in range(4):
        now = (timeout_count + 1) / 10.0
        result = _diagnose(diagnosis, "", timeout_count + 1, None, now)
        assert result[0] == ResultDiagnosis.NORMAL

    result = _diagnose(diagnosis, "", 5, None, 0.5)
    assert result[0] == ResultDiagnosis.NORMAL

    result = _diagnose(diagnosis, "", 0, None, 1.0)

    assert result[0] == ResultDiagnosis.DETECTION


def test_two_second_period_detection_recovers_after_returning_to_point_one_second() -> (
    None
):
    diagnosis = _create_diagnosis()

    _diagnose(diagnosis, ANGLE_CAN_ID, 0, 0.0, 0.0)
    _diagnose(diagnosis, LEVER_CAN_ID, 0, 0.0, 0.0)
    _diagnose(diagnosis, ANGLE_CAN_ID, 0, 0.1, 0.1)
    _diagnose(diagnosis, LEVER_CAN_ID, 0, 0.1, 0.1)
    detection = _diagnose(diagnosis, "", 1, None, 2.1)

    assert detection[0] == ResultDiagnosis.DETECTION
    assert diagnosis.is_error.value

    recovery_results: list[tuple[ResultDiagnosis, ResultDiagnosis]] = []
    for success_count in range(1, 6):
        now = 2.1 + success_count / 10.0
        _diagnose(diagnosis, ANGLE_CAN_ID, 0, now, now)
        recovery_results.append(_diagnose(diagnosis, LEVER_CAN_ID, 0, now, now))
        timeout_result = _diagnose(diagnosis, "", 1, None, now + 0.01)
        assert timeout_result[0] != ResultDiagnosis.DETECTION

    assert (
        sum(result[0] == ResultDiagnosis.RECOVERY for result in recovery_results) == 1
    )
    assert not diagnosis.is_error.value

    for success_count in range(1, 31):
        now = 1.5 + success_count / 10.0
        result = _diagnose(diagnosis, ANGLE_CAN_ID, 0, now, now)
        assert result[0] == ResultDiagnosis.NORMAL
        result = _diagnose(diagnosis, LEVER_CAN_ID, 0, now, now)
        assert result[0] == ResultDiagnosis.NORMAL

    assert not diagnosis.is_error.value


def test_empty_can_id_cannot_trigger_recovery() -> None:
    diagnosis = _create_diagnosis()

    for _ in range(5):
        _diagnose(diagnosis, ANGLE_CAN_ID, 0, 0.0, 0.0)
        _diagnose(diagnosis, LEVER_CAN_ID, 0, 0.0, 0.0)

    diagnosis.is_error.value = True
    diagnosis.is_fail_safe.value = True

    result = _diagnose(diagnosis, "", 0, None, 0.1)

    assert result == (ResultDiagnosis.KEEPING, ResultDiagnosis.KEEPING)
    assert diagnosis.is_error.value
    assert diagnosis.is_fail_safe.value


def test_timestamp_is_updated_only_for_matching_can_id() -> None:
    diagnosis = _create_diagnosis()

    _diagnose(diagnosis, ANGLE_CAN_ID, 0, 10.0, 10.0)
    _diagnose(diagnosis, LEVER_CAN_ID, 0, 20.0, 20.0)

    assert diagnosis.can_last_update_time[ANGLE_CAN_ID] == 10.0
    assert diagnosis.can_last_update_time[LEVER_CAN_ID] == 20.0

    _diagnose(diagnosis, ANGLE_CAN_ID, 0, 30.0, 30.0)

    assert diagnosis.can_last_update_time[ANGLE_CAN_ID] == 30.0
    assert diagnosis.can_last_update_time[LEVER_CAN_ID] == 20.0
