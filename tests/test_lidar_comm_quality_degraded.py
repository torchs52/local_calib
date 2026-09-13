from types import SimpleNamespace
from typing import Any
from unittest.mock import MagicMock

import numpy as np
import pytest

from argus_synchro.device.lidar.mid360_points import MID360Points
from argus_synchro.diagnosis.error_config import LidarCommQualityDegradedParameters
from argus_synchro.diagnosis.error_diagnosis import ResultDiagnosis
from argus_synchro.diagnosis.state_errors import LidarNCommQualityDegradedDiagnosis
from argus_synchro.process.app_manager_process import AppManagerProcess
from argus_synchro.process.points_process import PointsProviderProcess
from argus_synchro.provider.point_cloud import Mid360PointCloudProvider
from argus_synchro.shared_errors import StateErrorDIndex, StateErrorIndex


def _packet(dot_num: int, udp_cnt: int) -> bytes:
    packet = bytearray(1500)
    packet[5:7] = dot_num.to_bytes(2, "little")
    packet[7:9] = udp_cnt.to_bytes(2, "little")
    return bytes(packet)


def _diagnosis() -> LidarNCommQualityDegradedDiagnosis:
    diagnosis = LidarNCommQualityDegradedDiagnosis()
    diagnosis.param = LidarCommQualityDegradedParameters()
    diagnosis.is_enabled = True
    return diagnosis


def test_mid360_vectorized_decode_preserves_packet_fields() -> None:
    point_dtype = np.dtype(
        [("x", "<i4"), ("y", "<i4"), ("z", "<i4"), ("reflect", "i1"), ("tag", "u1")]
    )
    raw_points = np.zeros(96, dtype=point_dtype)
    raw_points[0] = (1234, -5678, 9000, -1, 17)
    raw_points[-1] = (-2000, 3000, -4000, 42, 255)
    packet = bytearray(1500)
    packet[5:7] = (96).to_bytes(2, "little")
    packet[7:9] = (1).to_bytes(2, "little")
    packet[28:36] = (1_250_000_000).to_bytes(8, "little", signed=True)
    start = 36
    packet[start : start + raw_points.nbytes] = raw_points.tobytes()
    device: Any = object.__new__(MID360Points)
    device._socket = MagicMock()
    device._logger = MagicMock()
    device._socket.recvfrom.return_value = (bytes(packet), None)
    device._prev_udp_cnt = None
    device._dot_num_low_threshold = 50
    device._last_quality_degraded = 0.0

    points, timestamp = device.get_points()

    assert points.shape == (96, 4)
    assert points.dtype == np.float64
    assert points.flags.c_contiguous
    assert timestamp == 1.25
    np.testing.assert_array_equal(points[0], [1.234, -5.678, 9.0, -1.0])
    np.testing.assert_array_equal(points[-1], [-2.0, 3.0, -4.0, 42.0])


def test_mid360_vectorized_decode_rejects_short_packet() -> None:
    required_size = 36 + 14 * 96
    device: Any = object.__new__(MID360Points)
    device._socket = MagicMock()
    device._logger = MagicMock()
    device._socket.recvfrom.return_value = (bytes(required_size - 1), None)
    device._prev_udp_cnt = None
    device._dot_num_low_threshold = 50
    device._last_quality_degraded = 0.0

    with pytest.raises(
        ValueError,
        match=rf"MID360 point packet is too short: {required_size - 1} < {required_size}",
    ):
        device.get_points()


def test_mid360_detects_udp_gap_and_low_point_count() -> None:
    device = object.__new__(MID360Points)
    device._socket = MagicMock()
    device._logger = MagicMock()
    device._prev_udp_cnt = None
    device._dot_num_low_threshold = 50

    assert device._check_packet_quality(_packet(96, 1)) is False
    assert device._check_packet_quality(_packet(96, 2)) is False
    assert device._check_packet_quality(_packet(96, 4)) is True
    assert device._check_packet_quality(_packet(49, 5)) is True
    assert device._check_packet_quality(_packet(96, 0)) is False


def test_mid360_records_degraded_event_with_perf_counter(monkeypatch) -> None:
    socket = MagicMock()
    socket.recvfrom.side_effect = (
        (_packet(96, 1), None),
        (_packet(96, 3), None),
    )
    device = object.__new__(MID360Points)
    device._socket = socket
    device._logger = MagicMock()
    device._prev_udp_cnt = None
    device._dot_num_low_threshold = 50
    device._last_quality_degraded = 0.0
    monkeypatch.setattr(
        "argus_synchro.device.lidar.mid360_points.time.perf_counter", lambda: 42.0
    )

    device.get_points()
    device.get_points()

    assert device.last_quality_degraded == 42.0


def test_mid360_provider_forwards_degraded_event() -> None:
    provider = Mid360PointCloudProvider(
        SimpleNamespace(last_quality_degraded=4.2)  # type: ignore[arg-type]
    )

    assert provider.last_quality_degraded == 4.2


def test_points_process_forwards_latest_degraded_event() -> None:
    data_missing = MagicMock()
    data_missing.errors_diagnosis.return_value = (
        ResultDiagnosis.NORMAL,
        ResultDiagnosis.NORMAL,
    )
    process = object.__new__(PointsProviderProcess)
    process._provider = SimpleNamespace(
        get_accum_points=lambda: [[0.0, 0.0, 0.0]],
        last_quality_degraded=12.0,
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
        last_quality_degraded=SimpleNamespace(value=11.0),
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

    assert process._sec_lid.last_quality_degraded.value == 12.0


def test_degraded_event_requires_three_seconds_confirmation() -> None:
    diagnosis = _diagnosis()

    assert diagnosis.errors_diagnosis(10.0, 10.0)[0] == ResultDiagnosis.NORMAL
    assert diagnosis.errors_diagnosis(12.9, 12.9)[0] == ResultDiagnosis.NORMAL
    assert diagnosis.errors_diagnosis(13.0, 13.0)[0] == ResultDiagnosis.DETECTION


def test_degraded_event_recovers_after_five_normal_seconds() -> None:
    diagnosis = _diagnosis()
    diagnosis.is_error.value = True
    diagnosis.is_fail_safe.value = True

    assert diagnosis.errors_diagnosis(20.0, 18.0)[0] == ResultDiagnosis.KEEPING
    result = diagnosis.errors_diagnosis(25.0, 18.0)

    assert result == (ResultDiagnosis.RECOVERY, ResultDiagnosis.RECOVERY)


def _app_manager(connection_error: bool) -> tuple[AppManagerProcess, MagicMock]:
    connection = MagicMock()
    connection.is_error.value = connection_error
    connection.errors_diagnosis.return_value = (
        ResultDiagnosis.KEEPING if connection_error else ResultDiagnosis.NORMAL,
        ResultDiagnosis.KEEPING if connection_error else ResultDiagnosis.NORMAL,
    )
    quality = MagicMock()
    quality.errors_diagnosis.return_value = (
        ResultDiagnosis.DETECTION,
        ResultDiagnosis.DETECTION,
    )
    quality.is_error.value = False
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
            StateErrorIndex.LIDAR0_COMM_QUALITY_DEGRADED: quality,
            StateErrorIndex.LIDAR0_COMM_QUALITY_ERROR: quality_error,
            StateErrorIndex.LIDAR0_INVALID_DATA: invalid_data,
        }
    )
    return process, quality


def test_app_manager_skips_quality_diagnosis_during_connection_error() -> None:
    process, quality = _app_manager(connection_error=True)

    process._lidar_healthy_check(10.0)

    quality.errors_diagnosis.assert_not_called()


def test_app_manager_dispatches_quality_event_when_connected() -> None:
    process, quality = _app_manager(connection_error=False)

    process._lidar_healthy_check(10.0)

    quality.errors_diagnosis.assert_called_once_with(10.0, 9.8)
    quality.log_output.assert_called_once_with(
        ResultDiagnosis.DETECTION,
        ResultDiagnosis.DETECTION,
        StateErrorIndex.LIDAR0_COMM_QUALITY_DEGRADED,
        0,
    )


def test_quality_log_contains_lidar_index() -> None:
    diagnosis = _diagnosis()
    diagnosis._logger = MagicMock()

    diagnosis._error_log_output(StateErrorIndex.LIDAR1_COMM_QUALITY_DEGRADED, 1)

    diagnosis._logger.info.assert_called_once_with(
        "SE009: Lidar[1] comm quality degraded detected."
    )