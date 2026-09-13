from types import SimpleNamespace
from unittest.mock import MagicMock

import numpy as np
import pytest

from argus_synchro.diagnosis.error_config import LidarInvalidDataParameters
from argus_synchro.diagnosis.error_diagnosis import ResultDiagnosis
from argus_synchro.diagnosis.state_errors import LidarNInvalidDataDiagnosis
from argus_synchro.process.app_manager_process import AppManagerProcess
from argus_synchro.process.points_process import PointsProviderProcess
from argus_synchro.provider.point_cloud import Mid360PointCloudProvider
from argus_synchro.shared_errors import StateErrorDIndex, StateErrorIndex


def _diagnosis() -> LidarNInvalidDataDiagnosis:
    diagnosis = LidarNInvalidDataDiagnosis()
    diagnosis.param = LidarInvalidDataParameters()
    diagnosis.is_enabled = True
    return diagnosis


def test_invalid_ratio_requires_three_seconds_at_threshold() -> None:
    diagnosis = _diagnosis()

    assert diagnosis.errors_diagnosis(10.0, 0.7)[0] == ResultDiagnosis.NORMAL
    assert diagnosis.errors_diagnosis(12.9, 0.7)[0] == ResultDiagnosis.NORMAL
    assert diagnosis.errors_diagnosis(13.0, 0.7)[0] == ResultDiagnosis.DETECTION


def test_invalid_ratio_confirmation_resets_below_threshold() -> None:
    diagnosis = _diagnosis()

    diagnosis.errors_diagnosis(10.0, 0.7)
    diagnosis.errors_diagnosis(12.0, 0.69)
    diagnosis.errors_diagnosis(13.0, 0.7)

    assert diagnosis.errors_diagnosis(15.9, 0.7)[0] == ResultDiagnosis.NORMAL
    assert diagnosis.errors_diagnosis(16.0, 0.7)[0] == ResultDiagnosis.DETECTION


def test_error_and_failsafe_recover_after_three_and_five_seconds() -> None:
    diagnosis = _diagnosis()
    diagnosis.is_error.value = True
    diagnosis.is_fail_safe.value = True

    assert diagnosis.errors_diagnosis(20.0, 0.29) == (
        ResultDiagnosis.KEEPING,
        ResultDiagnosis.KEEPING,
    )
    assert diagnosis.errors_diagnosis(23.0, 0.29) == (
        ResultDiagnosis.RECOVERY,
        ResultDiagnosis.KEEPING,
    )
    assert diagnosis.errors_diagnosis(25.0, 0.29) == (
        ResultDiagnosis.NORMAL,
        ResultDiagnosis.RECOVERY,
    )


def test_recovery_requires_ratio_strictly_below_threshold() -> None:
    diagnosis = _diagnosis()
    diagnosis.is_error.value = True

    diagnosis.errors_diagnosis(20.0, 0.29)
    diagnosis.errors_diagnosis(22.0, 0.3)

    assert diagnosis.errors_diagnosis(24.9, 0.29)[0] == ResultDiagnosis.KEEPING
    assert diagnosis.errors_diagnosis(27.8, 0.29)[0] == ResultDiagnosis.KEEPING
    assert diagnosis.errors_diagnosis(27.9, 0.29)[0] == ResultDiagnosis.RECOVERY


@pytest.mark.parametrize("args", [(1, 0.7), (1.0, 1), (1.0,)])
def test_invalid_data_rejects_invalid_arguments(args: tuple[object, ...]) -> None:
    with pytest.raises(ValueError):
        _diagnosis().errors_diagnosis(*args)


def test_mid360_provider_calculates_origin_ratio() -> None:
    points = np.array(
        [
            [0.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
            [1.0, 2.0, 3.0, 1.0],
            [4.0, 5.0, 6.0, 1.0],
        ]
    )
    device = MagicMock()
    device.get_points.side_effect = ((points, 1.0), (points, 1.2))
    provider = Mid360PointCloudProvider(device)

    provider.get_accum_points(max_accum_time=0.1)

    assert provider.last_invalid_ratio == 0.5


def test_points_process_forwards_invalid_ratio() -> None:
    data_missing = MagicMock()
    data_missing.errors_diagnosis.return_value = (
        ResultDiagnosis.NORMAL,
        ResultDiagnosis.NORMAL,
    )
    process = object.__new__(PointsProviderProcess)
    process._provider = SimpleNamespace(
        get_accum_points=lambda: [[1.0, 2.0, 3.0]],
        last_invalid_ratio=0.75,
    )
    process._file_input = False
    process._index = 0
    process._ser = SimpleNamespace(
        set_lidar_connected=lambda *_args: None,
        state_errors_A_C={
            StateErrorIndex.LIDAR0_CONNECTION_ERROR: SimpleNamespace(
                is_error=SimpleNamespace(value=False)
            )
        },
        state_errors_D={StateErrorDIndex.LIDAR_DATA_MISSING: data_missing},
    )
    process._sec_lid = SimpleNamespace(
        invalid_data_ratio=SimpleNamespace(value=0.0),
        last_heartbeat=SimpleNamespace(value=-1.0),
    )
    process._last_heartbeat = float("inf")
    process._heartbeat_interval = 0.5
    process._clockPprovider = SimpleNamespace(
        get_time=lambda: 1.0,
        isframeexceeded=lambda: False,
    )
    process._frame = 3

    process._update()

    assert process._sec_lid.invalid_data_ratio.value == 0.75


def _app_manager(connection_error: bool) -> tuple[AppManagerProcess, MagicMock]:
    connection = MagicMock()
    connection.is_error.value = connection_error
    connection.errors_diagnosis.return_value = (
        ResultDiagnosis.KEEPING if connection_error else ResultDiagnosis.NORMAL,
        ResultDiagnosis.KEEPING if connection_error else ResultDiagnosis.NORMAL,
    )
    degraded = MagicMock()
    degraded.is_error.value = False
    degraded.errors_diagnosis.return_value = (
        ResultDiagnosis.NORMAL,
        ResultDiagnosis.NORMAL,
    )
    quality_error = MagicMock()
    quality_error.errors_diagnosis.return_value = (
        ResultDiagnosis.NORMAL,
        ResultDiagnosis.NORMAL,
    )
    invalid_data = MagicMock()
    invalid_data.errors_diagnosis.return_value = (
        ResultDiagnosis.DETECTION,
        ResultDiagnosis.DETECTION,
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
                invalid_data_ratio=SimpleNamespace(value=0.8),
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
    return process, invalid_data


def test_app_manager_dispatches_invalid_ratio_when_connected() -> None:
    process, invalid_data = _app_manager(connection_error=False)

    process._lidar_healthy_check(10.0)

    invalid_data.errors_diagnosis.assert_called_once_with(10.0, 0.8)
    invalid_data.log_output.assert_called_once_with(
        ResultDiagnosis.DETECTION,
        ResultDiagnosis.DETECTION,
        StateErrorIndex.LIDAR0_INVALID_DATA,
        0,
    )


def test_app_manager_skips_invalid_data_during_connection_error() -> None:
    process, invalid_data = _app_manager(connection_error=True)

    process._lidar_healthy_check(10.0)

    invalid_data.errors_diagnosis.assert_not_called()


def test_invalid_data_log_contains_lidar_index() -> None:
    diagnosis = _diagnosis()
    diagnosis._logger = MagicMock()

    diagnosis._error_log_output(StateErrorIndex.LIDAR1_INVALID_DATA, 1)

    diagnosis._logger.warning.assert_called_once_with(
        "SE021: Lidar[1] invalid data detected."
    )