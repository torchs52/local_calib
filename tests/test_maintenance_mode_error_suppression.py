from pathlib import Path
from types import SimpleNamespace

import pytest

from argus_synchro.diagnosis.action_errors import (
    LidarPositionMisalignmentDetectedDiagnosis,
    RebootLoopDetectedDiagnosis,
    SensorCalibrationRequiredDiagnosis,
)
from argus_synchro.diagnosis.error_diagnosis import DiagnosisRuntimePolicy
from argus_synchro.diagnosis.state_errors import (
    ApplicationManagerNotRespondingDiagnosis,
    CanConnectionErrorDiagnosis,
    LidarNConnectionErrorDiagnosis,
    MonitorProcessNotRespondingDiagnosis,
    SurroundMonitorModuleNotRespondingDiagnosis,
    YawAngleInfoErrorDiagnosis,
)
from argus_synchro.process.app_manager_process import AppManagerProcess
from argus_synchro.shared_errors import ActionErrorIndex, SharedErrors, StateErrorIndex
from argus_synchro.shared_excepts import SharedLidarShiftMonitorExcept


@pytest.mark.parametrize(
    ("diagnosis_factory", "prepare", "args"),
    (
        (
            LidarNConnectionErrorDiagnosis,
            lambda diagnosis: (
                setattr(diagnosis, "param", SimpleNamespace(error_threshold_sec=5.0)),
                setattr(diagnosis, "_previous_heartbeat", 0.0),
            ),
            (10.0, 0.0),
        ),
        (
            CanConnectionErrorDiagnosis,
            lambda diagnosis: (
                setattr(diagnosis, "param", SimpleNamespace(error_threshold_sec=5.0)),
                setattr(diagnosis, "_previous_heartbeat", 0.0),
            ),
            (10.0, 0.0),
        ),
        (
            YawAngleInfoErrorDiagnosis,
            lambda diagnosis: setattr(
                diagnosis, "param", SimpleNamespace(error_duration_sec=5.0)
            ),
            (0.0, 10.0),
        ),
        (
            MonitorProcessNotRespondingDiagnosis,
            lambda diagnosis: (
                setattr(diagnosis, "param", SimpleNamespace(error_threshold_sec=5.0)),
                setattr(diagnosis, "_last_time", 0.0),
            ),
            (10.0, None),
        ),
        (
            SurroundMonitorModuleNotRespondingDiagnosis,
            lambda diagnosis: setattr(
                diagnosis, "param", SimpleNamespace(error_threshold_sec=5.0)
            ),
            ([6.0],),
        ),
        (
            ApplicationManagerNotRespondingDiagnosis,
            lambda diagnosis: (
                setattr(diagnosis, "param", SimpleNamespace(error_threshold_sec=5.0)),
                setattr(diagnosis, "_previous_heartbeat", 10.0),
                setattr(diagnosis, "_last_time", 10.0),
            ),
            (16.0, 10.0),
        ),
    ),
)
def test_maintenance_mode_suppresses_selected_state_error_detection(
    diagnosis_factory, prepare, args
) -> None:
    policy = DiagnosisRuntimePolicy(in_factory=True)
    diagnosis = diagnosis_factory(policy)
    prepare(diagnosis)

    assert diagnosis.detect_error(*args) is False
    assert bool(diagnosis.is_error.value) is False
    assert bool(diagnosis.is_fail_safe.value) is False


def test_maintenance_mode_suppresses_lidar_misalignment_action(
    tmp_path: Path,
) -> None:
    policy = DiagnosisRuntimePolicy(in_factory=True)
    diagnosis = LidarPositionMisalignmentDetectedDiagnosis(policy)
    shared = SharedLidarShiftMonitorExcept(tmp_path / "not_calibrated")
    shared.is_shifted_fast.value = True

    assert diagnosis.detect_error(shared) is False
    assert diagnosis.err_cnt.value == 0
    assert bool(diagnosis.is_idle.value) is False
    assert bool(diagnosis.is_fail_safe.value) is False


def test_maintenance_mode_suppresses_sensor_calibration_required(
    tmp_path: Path,
) -> None:
    policy = DiagnosisRuntimePolicy(in_factory=True)
    diagnosis = SensorCalibrationRequiredDiagnosis(policy)
    shared = SharedLidarShiftMonitorExcept(tmp_path / "not_calibrated")
    shared._has_not_calibrated.value = True

    assert diagnosis.detect_error(shared, 123) is False
    assert diagnosis.err_cnt.value == 0
    assert bool(diagnosis.is_idle.value) is False
    assert bool(diagnosis.is_fail_safe.value) is False


def test_maintenance_mode_suppresses_reboot_loop_detection() -> None:
    policy = DiagnosisRuntimePolicy(in_factory=True)
    diagnosis = RebootLoopDetectedDiagnosis(policy)
    diagnosis.param = SimpleNamespace(required_boot_count=5, window_sec=600.0)

    assert diagnosis.detect_error([1000.0, 900.0, 800.0, 700.0, 600.0]) is False


def test_runtime_policy_switches_detection_without_recreating_diagnosis() -> None:
    policy = DiagnosisRuntimePolicy(in_factory=True)
    diagnosis = RebootLoopDetectedDiagnosis(policy)
    diagnosis.param = SimpleNamespace(required_boot_count=5, window_sec=600.0)
    boot_times = [1000.0, 900.0, 800.0, 700.0, 600.0]

    assert diagnosis.detect_error(boot_times) is False

    policy.update_in_factory(False)

    assert diagnosis.detect_error(boot_times) is True


def test_shared_errors_injects_policy_only_into_selected_diagnoses() -> None:
    shared_errors = SharedErrors(Path("config/error_config.json"))
    policy = shared_errors.diagnosis_runtime_policy
    state_targets = {
        StateErrorIndex.LIDAR0_CONNECTION_ERROR,
        StateErrorIndex.LIDAR1_CONNECTION_ERROR,
        StateErrorIndex.CAN_CONNECTION_ERROR,
        StateErrorIndex.YAW_ANGLE_INFO_ERROR,
        StateErrorIndex.MONITOR_PROCESS_NOT_RESPONDING,
        StateErrorIndex.SURROUND_MONITOR_MODULE_NOT_RESPONDING,
        StateErrorIndex.APPLICATION_MANAGER_NOT_RESPONDING,
    }
    action_targets = {
        ActionErrorIndex.LIDAR_POSITION_MISALIGNMENT_DETECTED,
        ActionErrorIndex.SENSOR_CALIBRATION_REQUIRED,
        ActionErrorIndex.REBOOT_LOOP_DETECTED,
    }

    for index, diagnosis in enumerate(shared_errors.state_errors_A_C):
        expected = policy if index in state_targets else None
        assert diagnosis._runtime_policy is expected
    for index, diagnosis in enumerate(shared_errors.action_errors_A_C):
        expected = policy if index in action_targets else None
        assert diagnosis._runtime_policy is expected


def test_app_manager_config_reload_updates_runtime_policy() -> None:
    policy = DiagnosisRuntimePolicy(in_factory=False)
    app_config = SimpleNamespace(
        General=SimpleNamespace(in_factory=True),
        AppManager=SimpleNamespace(logmode=0, logtime=0.0, interval=1.0),
        Lidar=SimpleNamespace(count=2),
        camera=SimpleNamespace(count=4),
        DEFAULT=SimpleNamespace(File_Input=False, debug_log=""),
    )
    process = object.__new__(AppManagerProcess)
    process._sac = SimpleNamespace(read=lambda: app_config, last_updated=1)
    process._ser = SimpleNamespace(diagnosis_runtime_policy=policy)

    process._config_load()

    assert policy.in_factory is True