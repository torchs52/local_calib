from __future__ import annotations

import argus_synchro.diagnosis.error_config as ErrConf
from argus_synchro.diagnosis.error_diagnosis import ResultDiagnosis
from argus_synchro.diagnosis.state_errors import StorageSpaceLowDiagnosis


def _create_diagnosis() -> StorageSpaceLowDiagnosis:
    diagnosis = StorageSpaceLowDiagnosis()
    err_conf = ErrConf.ErrorConfig().load_from_dict(
        {
            "storage_space_low": {
                "is_enabled": True,
                "error_threshold_gb": 10.0,
                "error_recovery_threshold_gb": 10.0,
                "fail_safe_recovery_threshold_gb": 10.0,
                "error_duration_sec": 5.0,
                "error_recovery_duration_sec": 5.0,
                "fail_safe_recovery_duration_sec": 5.0,
            }
        }
    )
    diagnosis.update(err_conf)
    return diagnosis


def test_errors_diagnosis_detects_error_when_either_volume_is_low_for_5sec() -> None:
    diagnosis = _create_diagnosis()

    assert diagnosis.errors_diagnosis(12.0, 12.0, 0.0) == (
        ResultDiagnosis.NORMAL,
        ResultDiagnosis.NORMAL,
    )
    assert diagnosis.errors_diagnosis(9.9, 12.0, 1.0) == (
        ResultDiagnosis.NORMAL,
        ResultDiagnosis.NORMAL,
    )
    assert diagnosis.errors_diagnosis(9.9, 12.0, 5.9) == (
        ResultDiagnosis.NORMAL,
        ResultDiagnosis.NORMAL,
    )
    assert diagnosis.errors_diagnosis(9.9, 12.0, 6.0) == (
        ResultDiagnosis.DETECTION,
        ResultDiagnosis.DETECTION,
    )


def test_errors_diagnosis_normal_to_error_to_recovery_to_reerror() -> None:
    diagnosis = _create_diagnosis()

    # 正常
    assert diagnosis.errors_diagnosis(12.0, 12.0, 0.0) == (
        ResultDiagnosis.NORMAL,
        ResultDiagnosis.NORMAL,
    )

    # どちらか一方が閾値未満の状態が5秒継続してエラー
    assert diagnosis.errors_diagnosis(9.5, 12.0, 1.0) == (
        ResultDiagnosis.NORMAL,
        ResultDiagnosis.NORMAL,
    )
    assert diagnosis.errors_diagnosis(9.5, 12.0, 6.0) == (
        ResultDiagnosis.DETECTION,
        ResultDiagnosis.DETECTION,
    )

    # 両方が閾値超えの状態を5秒継続して復帰
    assert diagnosis.errors_diagnosis(12.0, 12.0, 7.0) == (
        ResultDiagnosis.KEEPING,
        ResultDiagnosis.KEEPING,
    )
    assert diagnosis.errors_diagnosis(12.0, 12.0, 12.0) == (
        ResultDiagnosis.RECOVERY,
        ResultDiagnosis.RECOVERY,
    )

    # 復帰後に再びどちらか一方が閾値未満で5秒継続すると再エラー
    assert diagnosis.errors_diagnosis(9.0, 12.0, 13.0) == (
        ResultDiagnosis.NORMAL,
        ResultDiagnosis.NORMAL,
    )
    assert diagnosis.errors_diagnosis(9.0, 12.0, 18.0) == (
        ResultDiagnosis.DETECTION,
        ResultDiagnosis.DETECTION,
    )
