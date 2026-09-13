from types import SimpleNamespace
from unittest.mock import MagicMock

import pytest

from argus_synchro.diagnosis.error_config import LidarCommQualityErrorParameters
from argus_synchro.diagnosis.error_diagnosis import ResultDiagnosis
from argus_synchro.diagnosis.state_errors import LidarNCommQualityErrorDiagnosis
from argus_synchro.process.app_manager_process import AppManagerProcess
from argus_synchro.shared_errors import StateErrorIndex


def _diagnosis() -> LidarNCommQualityErrorDiagnosis:
    diagnosis = LidarNCommQualityErrorDiagnosis()
    diagnosis.param = LidarCommQualityErrorParameters()
    diagnosis.is_enabled = True
    return diagnosis


def test_quality_error_requires_thirty_seconds_degraded() -> None:
    diagnosis = _diagnosis()

    assert diagnosis.errors_diagnosis(10.0, True)[0] == ResultDiagnosis.NORMAL
    assert diagnosis.errors_diagnosis(39.9, True)[0] == ResultDiagnosis.NORMAL
    assert diagnosis.errors_diagnosis(40.0, True)[0] == ResultDiagnosis.DETECTION


def test_quality_error_confirmation_resets_when_degradation_stops() -> None:
    diagnosis = _diagnosis()

    diagnosis.errors_diagnosis(10.0, True)
    diagnosis.errors_diagnosis(30.0, False)
    diagnosis.errors_diagnosis(31.0, True)

    assert diagnosis.errors_diagnosis(60.9, True)[0] == ResultDiagnosis.NORMAL
    assert diagnosis.errors_diagnosis(61.0, True)[0] == ResultDiagnosis.DETECTION


def test_quality_error_and_failsafe_recover_independently() -> None:
    diagnosis = _diagnosis()
    diagnosis.is_error.value = True
    diagnosis.is_fail_safe.value = True

    assert diagnosis.errors_diagnosis(100.0, False) == (
        ResultDiagnosis.KEEPING,
        ResultDiagnosis.KEEPING,
    )
    assert diagnosis.errors_diagnosis(130.0, False) == (
        ResultDiagnosis.RECOVERY,
        ResultDiagnosis.KEEPING,
    )
    assert diagnosis.errors_diagnosis(160.0, False) == (
        ResultDiagnosis.NORMAL,
        ResultDiagnosis.RECOVERY,
    )


@pytest.mark.parametrize("args", [(1, True), (1.0, 1), (1.0,)])
def test_quality_error_rejects_invalid_arguments(args: tuple[object, ...]) -> None:
    with pytest.raises(ValueError):
        _diagnosis().errors_diagnosis(*args)


def test_app_manager_passes_quality_state_as_bool() -> None:
    connection = MagicMock()
    connection.is_error.value = False
    connection.errors_diagnosis.return_value = (
        ResultDiagnosis.NORMAL,
        ResultDiagnosis.NORMAL,
    )
    degraded = MagicMock()
    degraded.is_error.value = 1
    degraded.errors_diagnosis.return_value = (
        ResultDiagnosis.KEEPING,
        ResultDiagnosis.KEEPING,
    )
    quality_error = MagicMock()
    quality_error.errors_diagnosis.return_value = (
        ResultDiagnosis.NORMAL,
        ResultDiagnosis.NORMAL,
    )
    invalid_data = MagicMock()
    invalid_data.errors_diagnosis.return_value = (
        ResultDiagnosis.NORMAL,
        ResultDiagnosis.NORMAL,
    )
    process = object.__new__(AppManagerProcess)
    process._num_lidars = 1
    process._is_last_lidar_diag_enabled = [True]
    process._sec = SimpleNamespace(
        LiDAR_ex=[
            SimpleNamespace(
                is_heartbeat_enabled=SimpleNamespace(value=True),
                last_heartbeat=SimpleNamespace(value=9.5),
                last_quality_degraded=SimpleNamespace(value=9.8),
                invalid_data_ratio=SimpleNamespace(value=0.0),
            )
        ]
    )
    process._ser = SimpleNamespace(
        state_errors_A_C={
            StateErrorIndex.LIDAR0_CONNECTION_ERROR: connection,
            StateErrorIndex.LIDAR0_COMM_QUALITY_DEGRADED: degraded,
            StateErrorIndex.LIDAR0_COMM_QUALITY_ERROR: quality_error,
            StateErrorIndex.LIDAR0_INVALID_DATA: invalid_data,
        }
    )

    process._lidar_healthy_check(10.0)

    quality_error.errors_diagnosis.assert_called_once_with(10.0, True)


def test_quality_error_log_contains_lidar_index() -> None:
    diagnosis = _diagnosis()
    diagnosis._logger = MagicMock()

    diagnosis._error_log_output(StateErrorIndex.LIDAR1_COMM_QUALITY_ERROR, 1)

    diagnosis._logger.warning.assert_called_once_with(
        "SE015: Lidar[1] comm quality error detected."
    )