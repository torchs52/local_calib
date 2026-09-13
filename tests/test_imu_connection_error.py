from collections import deque
from types import SimpleNamespace
from unittest.mock import MagicMock

import numpy as np

from argus_synchro.process.imu_process import ImuProviderProcess
from argus_synchro.diagnosis.state_errors import ImuNConnectionErrorDiagnosis
from argus_synchro.diagnosis.error_config import ImuNConnectionErrorParameters
from argus_synchro.process.app_manager_process import AppManagerProcess
from argus_synchro.shared_errors import StateErrorIndex


def _make_process(provider) -> ImuProviderProcess:
    process = object.__new__(ImuProviderProcess)
    process._ProcessBase__process = None
    process._provider = provider
    process._sec_imu = SimpleNamespace(
        last_heartbeat=SimpleNamespace(value=-1.0),
        is_heartbeat_enabled=SimpleNamespace(value=True),
    )
    process._heartbeat_interval = 0.5
    process._last_heartbeat = 10.0
    process._diagnosis_warmup_start = None
    process._err_config = SimpleNamespace(
        imu_n_connection_error=SimpleNamespace(
            recovery_receive_interval_sec=1.0,
            error_recovery_confirm_duration_sec=5.0,
            failsafe_recovery_confirm_duration_sec=5.0,
        )
    )
    return process


def test_imu_update_refreshes_heartbeat_only_for_received_data(monkeypatch) -> None:
    provider = SimpleNamespace(
        get_accum_point=lambda: (
            deque([np.zeros((2, 3), dtype=np.float64)]),
            11.0,
        )
    )
    process = _make_process(provider)
    monkeypatch.setattr(
        "argus_synchro.process.imu_process.time.perf_counter", lambda: 11.0
    )

    result = process._update()

    assert result is not None
    assert process._sec_imu.last_heartbeat.value == 11.0


def test_imu_update_does_not_refresh_heartbeat_for_empty_data() -> None:
    provider = SimpleNamespace(get_accum_point=lambda: (deque(), 11.0))
    process = _make_process(provider)

    result = process._update()

    assert result is not None
    assert process._sec_imu.last_heartbeat.value == -1.0


def test_imu_update_does_not_refresh_heartbeat_on_timeout() -> None:
    def raise_timeout():
        raise TimeoutError("IMU timeout")

    process = _make_process(SimpleNamespace(get_accum_point=raise_timeout))

    assert process._update() is None
    assert process._sec_imu.last_heartbeat.value == -1.0


def test_imu_shutdown_disables_heartbeat_monitoring() -> None:
    process = object.__new__(ImuProviderProcess)
    process._ProcessBase__process = None
    process._sec_imu = SimpleNamespace(
        is_heartbeat_enabled=SimpleNamespace(value=True)
    )

    process._shutdown()

    assert process._sec_imu.is_heartbeat_enabled.value is False


def test_imu_start_diagnosis_resets_heartbeat_before_enabling(monkeypatch) -> None:
    process = object.__new__(ImuProviderProcess)
    process._ProcessBase__process = None
    process._sec_imu = SimpleNamespace(
        last_heartbeat=SimpleNamespace(value=1.0),
        is_heartbeat_enabled=SimpleNamespace(value=False),
    )
    process._last_heartbeat = 1.0
    monkeypatch.setattr(
        "argus_synchro.process.imu_process.time.perf_counter", lambda: 42.0
    )

    process.start_diagnosis()

    assert process._sec_imu.last_heartbeat.value == -1.0
    assert process._last_heartbeat == 42.0
    assert process._sec_imu.is_heartbeat_enabled.value is False


def test_imu_diagnosis_starts_after_continuous_heartbeat_warmup() -> None:
    process = _make_process(SimpleNamespace())
    process._sec_imu.is_heartbeat_enabled.value = False

    for now in (10.6, 11.2, 11.8, 12.4, 13.0, 13.6, 14.2, 14.8, 15.4):
        process._update_heartbeat(now)
        assert process._sec_imu.is_heartbeat_enabled.value is False

    process._update_heartbeat(16.0)

    assert process._sec_imu.is_heartbeat_enabled.value is True


def test_imu_diagnosis_warmup_restarts_after_receive_gap() -> None:
    process = _make_process(SimpleNamespace())
    process._sec_imu.is_heartbeat_enabled.value = False

    process._update_heartbeat(10.6)
    process._update_heartbeat(11.2)
    process._update_heartbeat(13.0)

    assert process._diagnosis_warmup_start == 13.0
    assert process._sec_imu.is_heartbeat_enabled.value is False


def _make_diagnosis() -> ImuNConnectionErrorDiagnosis:
    diagnosis = ImuNConnectionErrorDiagnosis()
    diagnosis.param = ImuNConnectionErrorParameters()
    return diagnosis


def test_imu_connection_detects_stale_heartbeat_after_initial_sample() -> None:
    diagnosis = _make_diagnosis()

    assert diagnosis.detect_error(10.0, -1.0) is False
    assert diagnosis.detect_error(20.0, 10.0) is True
    assert bool(diagnosis.is_error.value) is True
    assert bool(diagnosis.is_fail_safe.value) is True


def test_imu_connection_recovers_after_five_seconds_of_fresh_heartbeats() -> None:
    diagnosis = _make_diagnosis()
    diagnosis.is_error.value = True
    diagnosis.is_fail_safe.value = True

    for heartbeat in (10.0, 11.0, 12.0, 13.0, 14.0, 15.0):
        assert diagnosis.detect_recovery_error(heartbeat, heartbeat) is False
        assert diagnosis.detect_recovery_fail_safe(heartbeat, heartbeat) is False
    assert diagnosis.detect_recovery_error(16.0, 16.0) is True
    assert diagnosis.detect_recovery_fail_safe(16.0, 16.0) is True


def test_app_manager_dispatches_enabled_imu_heartbeat() -> None:
    diagnoses = {
        StateErrorIndex.IMU0_CONNECTION_ERROR + i: MagicMock() for i in range(2)
    }
    for diagnosis in diagnoses.values():
        diagnosis.errors_diagnosis.return_value = ("normal", "normal")
    process = object.__new__(AppManagerProcess)
    process._num_lidars = 2
    process._is_last_imu_diag_enabled = [False, False]
    process._sec = SimpleNamespace(
        IMU_ex=[
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

    process._imu_healthy_check(10.0)

    diagnoses[StateErrorIndex.IMU0_CONNECTION_ERROR].clear.assert_called_once_with()
    diagnoses[StateErrorIndex.IMU0_CONNECTION_ERROR].errors_diagnosis.assert_called_once_with(
        10.0, 9.5
    )
    diagnoses[StateErrorIndex.IMU1_CONNECTION_ERROR].errors_diagnosis.assert_not_called()


def test_imu_connection_owns_sensor_specific_log() -> None:
    diagnosis = _make_diagnosis()
    diagnosis._logger = MagicMock()

    diagnosis._error_log_output(StateErrorIndex.IMU1_CONNECTION_ERROR, 1)

    diagnosis._logger.warning.assert_called_once_with(
        "SE041: IMU[1] connection error detected."
    )