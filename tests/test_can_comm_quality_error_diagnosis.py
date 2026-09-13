from __future__ import annotations

import argus_synchro.diagnosis.error_config as ErrConf
from argus_synchro.diagnosis.error_diagnosis import ResultDiagnosis
from argus_synchro.diagnosis.state_errors import CanCommQualityErrorDiagnosis

ANGLE_CAN_ID = "18FFD1D1"
LEVER_CAN_ID = "18FC4401"


def _create_diagnosis() -> CanCommQualityErrorDiagnosis:
    diagnosis = CanCommQualityErrorDiagnosis()
    err_conf = ErrConf.ErrorConfig().load_from_dict(
        {
            "can_comm_quality_error": {
                "is_enabled": True,
                "read_error_threshold_sec": {
                    ANGLE_CAN_ID: 3.0,
                    LEVER_CAN_ID: 3.0,
                },
                "read_error_threshold_count": {
                    ANGLE_CAN_ID: 10,
                    LEVER_CAN_ID: 10,
                },
                "read_error_rate_window_sec": {
                    ANGLE_CAN_ID: 10.0,
                    LEVER_CAN_ID: 10.0,
                },
                "read_error_rate_threshold": {
                    ANGLE_CAN_ID: 0.3,
                    LEVER_CAN_ID: 0.3,
                },
                "recovery_receive_interval_sec": {
                    ANGLE_CAN_ID: 1.0,
                    LEVER_CAN_ID: 1.0,
                },
                "fail_safe_recovery_receive_interval_sec": {
                    ANGLE_CAN_ID: 1.0,
                    LEVER_CAN_ID: 1.0,
                },
                "read_error_recovery_confirm_duration_sec": {
                    ANGLE_CAN_ID: 3.0,
                    LEVER_CAN_ID: 3.0,
                },
                "fail_safe_recovery_confirm_duration_sec": {
                    ANGLE_CAN_ID: 3.0,
                    LEVER_CAN_ID: 3.0,
                },
                "read_error_recovery_threshold_count": {
                    ANGLE_CAN_ID: 10,
                    LEVER_CAN_ID: 10,
                },
                "fail_safe_recovery_threshold_count": {
                    ANGLE_CAN_ID: 10,
                    LEVER_CAN_ID: 10,
                },
                "error_rate_recovery_confirm_duration_sec": {
                    ANGLE_CAN_ID: 10.0,
                    LEVER_CAN_ID: 10.0,
                },
                "fail_safe_rate_recovery_confirm_duration_sec": {
                    ANGLE_CAN_ID: 10.0,
                    LEVER_CAN_ID: 10.0,
                },
            }
        }
    )
    diagnosis.update(err_conf)
    return diagnosis


def _diagnose(
    diagnosis: CanCommQualityErrorDiagnosis,
    can_id: str,
    failed_count: int,
    timestamp: float | None,
    now: float,
) -> tuple[ResultDiagnosis, ResultDiagnosis]:
    return diagnosis.errors_diagnosis(can_id, failed_count, timestamp, now)


def test_timestamp_error_is_detected_at_exactly_three_seconds() -> None:
    diagnosis = _create_diagnosis()

    assert _diagnose(diagnosis, ANGLE_CAN_ID, 0, 0.0, 0.0)[0] == ResultDiagnosis.NORMAL
    result = _diagnose(diagnosis, ANGLE_CAN_ID, 0, 0.0, 3.0)

    assert result[0] == ResultDiagnosis.DETECTION
    assert diagnosis.is_error.value


def test_failure_rate_is_kept_independently_for_each_can_id() -> None:
    diagnosis = _create_diagnosis()

    _diagnose(diagnosis, ANGLE_CAN_ID, 1, 0.0, 0.0)
    _diagnose(diagnosis, LEVER_CAN_ID, 0, 0.1, 0.1)

    assert diagnosis._failure_rate_by_canid[ANGLE_CAN_ID] == 1.0
    assert diagnosis._failure_rate_by_canid[LEVER_CAN_ID] == 0.0


def test_empty_can_id_does_not_update_failure_streak_or_failure_rate() -> None:
    diagnosis = _create_diagnosis()

    _diagnose(diagnosis, ANGLE_CAN_ID, 0, 0.0, 0.0)
    _diagnose(diagnosis, LEVER_CAN_ID, 0, 0.0, 0.0)
    result = _diagnose(diagnosis, "", 1, None, 0.1)

    assert result[0] == ResultDiagnosis.NORMAL


def test_recent_failure_prevents_error_recovery_within_ten_seconds() -> None:
    diagnosis = _create_diagnosis()
    _diagnose(diagnosis, ANGLE_CAN_ID, 10, 0.0, 0.0)
    _diagnose(diagnosis, LEVER_CAN_ID, 0, 0.0, 0.0)

    last_result = (ResultDiagnosis.NORMAL, ResultDiagnosis.NORMAL)
    for second in range(1, 11):
        now = float(second)
        _diagnose(diagnosis, ANGLE_CAN_ID, 0, now, now)
        last_result = _diagnose(diagnosis, LEVER_CAN_ID, 0, now, now)

    assert last_result[0] != ResultDiagnosis.RECOVERY
    assert diagnosis.is_error.value
