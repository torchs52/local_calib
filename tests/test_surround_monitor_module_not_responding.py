from types import SimpleNamespace
from unittest.mock import MagicMock

import pytest

from argus_synchro.diagnosis.error_diagnosis import ResultDiagnosis
from argus_synchro.diagnosis.state_errors import (
    SurroundMonitorModuleNotRespondingDiagnosis,
)
from argus_synchro.process.app_manager_process import AppManagerProcess
from argus_synchro.process.can_process import CanDataProviderProcess
from argus_synchro.process.get_data_process import GetDataProcess
from argus_synchro.process.image_process import CameraProviderProcess
from argus_synchro.process.lidar_shift_monitor_process import LidarShiftMonitorProcess
from argus_synchro.process.object_detect_process import ObjectDetectProcess
from argus_synchro.process.points_process import PointsProviderProcess
from argus_synchro.process.points_refine_process import PointsRefineProcess
from argus_synchro.process.visual_process import VisualProcess
from argus_synchro.shared_excepts import INVALID_TIMESTAMP, SharedExcepts
from argus_synchro.shared_errors import StateErrorIndex


def _make_diagnosis() -> SurroundMonitorModuleNotRespondingDiagnosis:
    diagnosis = SurroundMonitorModuleNotRespondingDiagnosis()
    diagnosis.is_enabled = True
    diagnosis.param = SimpleNamespace(error_threshold_sec=5.0)
    return diagnosis


def test_surround_monitor_module_detects_any_elapsed_over_threshold() -> None:
    diagnosis = _make_diagnosis()

    assert diagnosis.errors_diagnosis([1.0, 5.0, -1.0, 5.1]) == (
        ResultDiagnosis.DETECTION,
        ResultDiagnosis.DETECTION,
    )
    assert diagnosis._last_max_elapsed_sec == 5.1


def test_surround_monitor_module_ignores_inactive_heartbeats() -> None:
    diagnosis = _make_diagnosis()

    assert diagnosis.errors_diagnosis([-1.0, -1.0]) == (
        ResultDiagnosis.NORMAL,
        ResultDiagnosis.NORMAL,
    )


def test_surround_monitor_module_recovers_when_all_active_modules_respond() -> None:
    diagnosis = _make_diagnosis()
    diagnosis.errors_diagnosis([6.0, 1.0])

    assert diagnosis.errors_diagnosis([0.1, 0.2]) == (
        ResultDiagnosis.RECOVERY,
        ResultDiagnosis.RECOVERY,
    )


@pytest.mark.parametrize("elapsed_list", ([], [1], ["1.0"]))
def test_surround_monitor_module_rejects_invalid_elapsed_list(elapsed_list) -> None:
    diagnosis = _make_diagnosis()

    with pytest.raises(ValueError):
        diagnosis.errors_diagnosis(elapsed_list)


def test_surround_monitor_module_owns_logs() -> None:
    diagnosis = _make_diagnosis()
    diagnosis._logger = MagicMock()
    diagnosis.errors_diagnosis([6.25])

    diagnosis.log_output(
        ResultDiagnosis.DETECTION,
        ResultDiagnosis.DETECTION,
        StateErrorIndex.SURROUND_MONITOR_MODULE_NOT_RESPONDING,
    )

    diagnosis._logger.error.assert_called_once_with(
        "SE037: SURROUND_MONITOR_MODULE_NOT_RESPONDING detected: "
        "max_elapsed=%.3f sec",
        6.25,
    )


def test_surround_monitor_processes_have_independent_heartbeats(app_config) -> None:
    shared = SharedExcepts(app_config)

    assert shared.ObjDet_ex is not shared.Scruti_ex
    assert shared.PointsRefine_ex is not shared.Scruti_ex
    assert shared.ObjDet_ex is not shared.PointsRefine_ex
    assert shared.getData_ex.last_heartbeat.value == INVALID_TIMESTAMP
    assert shared.ObjDet_ex.last_heartbeat.value == INVALID_TIMESTAMP
    assert shared.PointsRefine_ex.last_heartbeat.value == INVALID_TIMESTAMP
    assert shared.Visu_ex.last_heartbeat.value == INVALID_TIMESTAMP
    assert not shared.getData_ex.is_heartbeat_enabled.value
    assert not shared.ObjDet_ex.is_heartbeat_enabled.value
    assert not shared.PointsRefine_ex.is_heartbeat_enabled.value
    assert not shared.Visu_ex.is_heartbeat_enabled.value


@pytest.mark.parametrize(
    ("process_type", "shared_attribute"),
    (
        (CameraProviderProcess, "_sec_cam"),
        (CanDataProviderProcess, "_sec_can"),
        (PointsProviderProcess, "_sec_lid"),
        (GetDataProcess, "_sec_get_data"),
        (ObjectDetectProcess, "_spe"),
        (PointsRefineProcess, "_spe"),
        (VisualProcess, "_sec_visu_ex"),
    ),
)
def test_surround_process_diagnosis_lifecycle(
    process_type, shared_attribute: str
) -> None:
    shared = SimpleNamespace(
        last_heartbeat=SimpleNamespace(value=10.0),
        is_heartbeat_enabled=SimpleNamespace(value=False),
        last_received=SimpleNamespace(value=10.0),
        is_received_enabled=SimpleNamespace(value=False),
    )
    process = object.__new__(process_type)
    setattr(process, shared_attribute, shared)

    process.start_diagnosis()

    assert shared.last_heartbeat.value == INVALID_TIMESTAMP
    assert shared.is_heartbeat_enabled.value is True

    process.stop_diagnosis()

    assert shared.is_heartbeat_enabled.value is False


def _make_lidar_shift_monitor_process() -> LidarShiftMonitorProcess:
    process = object.__new__(LidarShiftMonitorProcess)
    process._sec_lidar_sm = SimpleNamespace(
        last_heartbeat=SimpleNamespace(value=10.0),
        is_heartbeat_enabled=SimpleNamespace(value=False),
    )
    process._last_heartbeat = 10.0
    process._diagnosis_warmup_start = None
    process._err_config = SimpleNamespace(
        lidar_position_misalignment_not_responding=SimpleNamespace(
            recovery_receive_interval_sec=1.0,
            error_recovery_confirm_duration_sec=5.0,
            failsafe_recovery_confirm_duration_sec=5.0,
        )
    )
    return process


def test_lidar_shift_start_diagnosis_waits_for_stable_input(monkeypatch) -> None:
    process = _make_lidar_shift_monitor_process()
    process._sec_lidar_sm.is_heartbeat_enabled.value = True
    monkeypatch.setattr(
        "argus_synchro.process.lidar_shift_monitor_process.time.perf_counter",
        lambda: 42.0,
    )

    process.start_diagnosis()

    assert process._sec_lidar_sm.last_heartbeat.value == INVALID_TIMESTAMP
    assert process._last_heartbeat == 42.0
    assert process._sec_lidar_sm.is_heartbeat_enabled.value is False


def test_lidar_shift_diagnosis_starts_after_continuous_heartbeat_warmup() -> None:
    process = _make_lidar_shift_monitor_process()

    for now in (10.6, 11.2, 11.8, 12.4, 13.0, 13.6, 14.2, 14.8, 15.4):
        process._update_heartbeat(now)
        assert process._sec_lidar_sm.is_heartbeat_enabled.value is False

    process._update_heartbeat(16.0)

    assert process._sec_lidar_sm.is_heartbeat_enabled.value is True


def test_lidar_shift_diagnosis_warmup_restarts_after_receive_gap() -> None:
    process = _make_lidar_shift_monitor_process()

    process._update_heartbeat(10.6)
    process._update_heartbeat(11.2)
    process._update_heartbeat(13.0)

    assert process._diagnosis_warmup_start == 13.0
    assert process._sec_lidar_sm.is_heartbeat_enabled.value is False


def test_app_manager_dispatches_surround_monitor_elapsed(monkeypatch) -> None:
    diagnosis = MagicMock()
    diagnosis.errors_diagnosis.return_value = (
        ResultDiagnosis.DETECTION,
        ResultDiagnosis.DETECTION,
    )
    process = object.__new__(AppManagerProcess)
    process._is_last_surround_diag_enabled = False
    process._sec = SimpleNamespace(
        getData_ex=SimpleNamespace(
            last_heartbeat=SimpleNamespace(value=8.0),
            is_heartbeat_enabled=SimpleNamespace(value=True),
        ),
        ObjDet_ex=SimpleNamespace(
            last_heartbeat=SimpleNamespace(value=9.0),
            is_heartbeat_enabled=SimpleNamespace(value=True),
        ),
        PointsRefine_ex=SimpleNamespace(
            last_heartbeat=SimpleNamespace(value=5.0),
            is_heartbeat_enabled=SimpleNamespace(value=False),
        ),
        Visu_ex=SimpleNamespace(
            last_heartbeat=SimpleNamespace(value=7.0),
            is_heartbeat_enabled=SimpleNamespace(value=True),
        ),
    )
    process._ser = SimpleNamespace(
        state_errors_A_C={
            StateErrorIndex.SURROUND_MONITOR_MODULE_NOT_RESPONDING: diagnosis
        }
    )
    monkeypatch.setattr(
        "argus_synchro.process.app_manager_process.time.monotonic", lambda: 10.0
    )

    process._surround_monitor_modules_healthy_check()

    diagnosis.clear.assert_called_once_with()
    diagnosis.errors_diagnosis.assert_called_once_with([2.0, 1.0, -1.0, 3.0])
    diagnosis.log_output.assert_called_once_with(
        ResultDiagnosis.DETECTION,
        ResultDiagnosis.DETECTION,
        StateErrorIndex.SURROUND_MONITOR_MODULE_NOT_RESPONDING,
    )