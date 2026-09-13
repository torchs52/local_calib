from types import SimpleNamespace
from unittest.mock import Mock

import numpy as np
import pytest

from argus_synchro.diagnosis.error_diagnosis import ResultDiagnosis
from argus_synchro.diagnosis.state_d_errors import LidarDataMissing
from argus_synchro.process.points_process import PointsProviderProcess
from argus_synchro.shared_errors import StateErrorDIndex, StateErrorIndex


def _diagnosis(min_point_count: int = 100) -> LidarDataMissing:
    diagnosis = LidarDataMissing()
    diagnosis.update(
        SimpleNamespace(
            lidar_data_missing=SimpleNamespace(
                is_enabled=True,
                min_point_count=min_point_count,
            )
        )
    )
    return diagnosis


def test_lidar_data_missing_detects_only_partial_data() -> None:
    diagnosis = _diagnosis()

    assert diagnosis.errors_diagnosis(1, False)[0] == ResultDiagnosis.DETECTION
    assert diagnosis.errors_diagnosis(99, False)[0] == ResultDiagnosis.KEEPING
    assert diagnosis.errors_diagnosis(100, False)[0] == ResultDiagnosis.RECOVERY
    assert diagnosis.errors_diagnosis(0, False)[0] == ResultDiagnosis.NORMAL


def test_lidar_data_missing_ignores_connection_error() -> None:
    diagnosis = _diagnosis()

    assert diagnosis.errors_diagnosis(1, True)[0] == ResultDiagnosis.NORMAL


def test_lidar_data_missing_logs_vendor_error_number_and_lidar_index() -> None:
    diagnosis = _diagnosis()
    diagnosis._logger = Mock()

    result = diagnosis.errors_diagnosis(50, False)
    diagnosis.log_output(*result, StateErrorDIndex.LIDAR_DATA_MISSING, 1)

    diagnosis._logger.info.assert_called_once_with(
        "SE011: Lidarデータ欠落: Lidar1の蓄積点数が閾値を下回りました。"
    )


def test_points_process_passes_point_count_and_connection_state() -> None:
    diagnosis = Mock()
    diagnosis.errors_diagnosis.return_value = (
        ResultDiagnosis.DETECTION,
        ResultDiagnosis.NORMAL,
    )
    state_errors_d: list[object | None] = [None] * (
        StateErrorDIndex.LIDAR_DATA_MISSING + 1
    )
    state_errors_d[StateErrorDIndex.LIDAR_DATA_MISSING] = diagnosis
    state_errors_a_c: list[object | None] = [None] * (
        StateErrorIndex.LIDAR1_CONNECTION_ERROR + 1
    )
    state_errors_a_c[StateErrorIndex.LIDAR1_CONNECTION_ERROR] = SimpleNamespace(
        is_error=SimpleNamespace(value=False)
    )

    process = object.__new__(PointsProviderProcess)
    process._provider = SimpleNamespace(
        get_accum_points=lambda: np.zeros((50, 3), dtype=np.float64)
    )
    process._file_input = False
    process._index = 1
    process._ser = SimpleNamespace(
        state_errors_D=state_errors_d,
        state_errors_A_C=state_errors_a_c,
        set_lidar_connected=lambda *_args: None,
    )
    process._last_heartbeat = float("inf")
    process._heartbeat_interval = 0.5
    process._clockPprovider = SimpleNamespace(
        get_time=lambda: 1.0,
        isframeexceeded=lambda: False,
    )
    process._frame = 3

    output = process._update()

    assert output is not None
    diagnosis.errors_diagnosis.assert_called_once_with(50, False)
    diagnosis.log_output.assert_called_once_with(
        ResultDiagnosis.DETECTION,
        ResultDiagnosis.NORMAL,
        StateErrorDIndex.LIDAR_DATA_MISSING,
        1,
    )


@pytest.mark.parametrize("args", [(1,), (1.0, False), (1, 0)])
def test_lidar_data_missing_rejects_invalid_arguments(args: tuple[object, ...]) -> None:
    with pytest.raises(ValueError):
        _diagnosis().errors_diagnosis(*args)