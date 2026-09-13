# ruff: noqa: ANN001, ANN201, ARG002, SLF001

from collections import deque
from types import SimpleNamespace
from unittest.mock import MagicMock

import numpy as np

from argus_synchro.message.scrutinizer_message import (
    CanAngleData,
    CanLeverData,
    DeltaYawData,
    RemovePointsData,
)
from argus_synchro.process.points_refine_process import PointsRefineProcess

EXPECTED_MODE_CHANGE_LOGS = 2


class FakeAccumPoints:
    def accum_point_cloud(
        self,
        point_cloud,
        counter,
        accum_points_dq,
        accum_ground_dq,
        accum_counter,
        delta_yaw,
        app_config,
        crane_state,
        is_reduced_load_mode,
    ):
        accum_points_dq.append(point_cloud)
        accum_ground_dq.append(point_cloud)
        return (
            counter,
            point_cloud,
            point_cloud,
            accum_points_dq,
            accum_ground_dq,
            accum_counter,
        )


def test_accum_buffer_log_is_emitted_only_when_reduced_load_mode_changes() -> None:
    process = object.__new__(PointsRefineProcess)
    process._ProcessBase__process = None
    process._logger = MagicMock()
    process._last_accum_reduced_load_mode = None
    process._crane_state_est = MagicMock(update=MagicMock(return_value=False))
    process._counter = np.empty(0, dtype=np.int32)
    process._accum_points_dq = deque()
    process._accum_ground_dq = deque()
    process._accum_counter = 0
    process._accum_point_cloud = FakeAccumPoints()
    process._app_config = SimpleNamespace(
        Accumulation=SimpleNamespace(
            max_accumulated_frames=10,
            max_accumulated_frames_reduced_load=5,
            max_accumulated_frames_ground=8,
            max_accumulated_frames_ground_reduced_load=4,
        )
    )
    reduced_load_mode = SimpleNamespace(enabled=False, update_pcd_nums=MagicMock())
    process._ser = SimpleNamespace(reduced_load_mode=reduced_load_mode)

    points = np.zeros((3, 3), dtype=np.float64)
    lever = CanLeverData(frame=1, lever_pressure=np.zeros(1, dtype=np.float16))
    angle = CanAngleData(frame=1, yaw_angle_deg=0)

    for frame in (1, 2):
        process._update_accum(
            RemovePointsData(frame=frame, time=float(frame), point_cloud=points),
            DeltaYawData(frame=frame, delta_yaw=0.0),
            lever,
            angle,
        )

    assert process._logger.info.call_count == 1
    assert process._logger.info.call_args.args[1:] == (
        "normal",
        1,
        0,
        1,
        10,
        0,
        1,
        8,
    )

    reduced_load_mode.enabled = True
    process._update_accum(
        RemovePointsData(frame=3, time=3.0, point_cloud=points),
        DeltaYawData(frame=3, delta_yaw=0.0),
        lever,
        angle,
    )

    assert process._logger.info.call_count == EXPECTED_MODE_CHANGE_LOGS
    assert process._logger.info.call_args.args[1:] == (
        "reduced_load",
        3,
        2,
        3,
        5,
        2,
        3,
        4,
    )
