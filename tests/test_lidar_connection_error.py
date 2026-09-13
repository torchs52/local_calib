from types import SimpleNamespace
from unittest.mock import MagicMock

from argus_synchro.diagnosis.error_config import LidarNConnectionErrorParameters
from argus_synchro.diagnosis.state_errors import LidarNConnectionErrorDiagnosis
from argus_synchro.process.app_manager_process import AppManagerProcess
from argus_synchro.shared_errors import StateErrorIndex


def _diagnosis() -> LidarNConnectionErrorDiagnosis:
    diagnosis = LidarNConnectionErrorDiagnosis()
    diagnosis.param = LidarNConnectionErrorParameters()
    return diagnosis


def test_lidar_connection_ignores_uninitialized_and_first_heartbeat() -> None:
    diagnosis = _diagnosis()

    assert diagnosis.detect_error(10.0, -1.0) is False
    assert diagnosis.detect_error(10.5, 10.5) is False


def test_lidar_connection_detects_five_seconds_without_heartbeat() -> None:
    diagnosis = _diagnosis()
    diagnosis.detect_error(10.0, 10.0)

    assert diagnosis.detect_error(15.0, 10.0) is True
    assert bool(diagnosis.is_error.value) is True
    assert bool(diagnosis.is_fail_safe.value) is True


def test_lidar_connection_recovers_after_fresh_heartbeats() -> None:
    diagnosis = _diagnosis()
    diagnosis.is_error.value = True
    diagnosis.is_fail_safe.value = True

    for heartbeat in (10.0, 11.0, 12.0, 13.0, 14.0, 15.0):
        assert diagnosis.detect_recovery_error(heartbeat, heartbeat) is False
        assert diagnosis.detect_recovery_fail_safe(heartbeat, heartbeat) is False
    assert diagnosis.detect_recovery_error(16.0, 16.0) is True
    assert diagnosis.detect_recovery_fail_safe(16.0, 16.0) is True


def test_lidar_connection_owns_sensor_specific_log() -> None:
    diagnosis = _diagnosis()
    diagnosis._logger = MagicMock()

    diagnosis._error_log_output(StateErrorIndex.LIDAR1_CONNECTION_ERROR, 1)

    diagnosis._logger.error.assert_called_once_with(
        "SE002: Lidar[1] connection error detected."
    )


def test_app_manager_dispatches_only_enabled_lidar() -> None:
    diagnoses = {
        StateErrorIndex.LIDAR0_CONNECTION_ERROR + i: MagicMock() for i in range(2)
    }
    for diagnosis in diagnoses.values():
        diagnosis.errors_diagnosis.return_value = ("normal", "normal")
    process = object.__new__(AppManagerProcess)
    process._num_lidars = 2
    process._is_last_lidar_diag_enabled = [False, False]
    process._sec = SimpleNamespace(
        LiDAR_ex=[
            SimpleNamespace(
                is_heartbeat_enabled=SimpleNamespace(value=True),
                last_heartbeat=SimpleNamespace(value=9.5),
            ),
            SimpleNamespace(
                is_heartbeat_enabled=SimpleNamespace(value=False),
                last_heartbeat=SimpleNamespace(value=-1.0),
            ),
        ]
    )
    process._ser = SimpleNamespace(state_errors_A_C=diagnoses)

    process._lidar_healthy_check(10.0)

    diagnoses[StateErrorIndex.LIDAR0_CONNECTION_ERROR].clear.assert_called_once_with()
    diagnoses[
        StateErrorIndex.LIDAR0_CONNECTION_ERROR
    ].errors_diagnosis.assert_called_once_with(10.0, 9.5)
    diagnoses[
        StateErrorIndex.LIDAR1_CONNECTION_ERROR
    ].errors_diagnosis.assert_not_called()