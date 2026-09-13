from __future__ import annotations

from abc import ABC, abstractmethod
from collections import deque

import numpy as np
import open3d as o3d
from numpy.typing import NDArray

from argus_synchro import SubScrutinizer
from argus_synchro.AccumulatePoints import filter_lidar_points
from argus_synchro.config.app_config import AppConfig


class FilterAccumPointsInterface(ABC):
    @abstractmethod
    def accum_point_cloud(
        self,
        xyz_machine_points_removed: NDArray[np.float64],
        counter: NDArray[np.int32],
        accum_points_dq: deque[NDArray[np.float64]],
        accum_points_with_ground_dq: deque[NDArray[np.float64]],
        accum_counter: int,
        delta_yaw: float,
        app_config: AppConfig,
        crane_state: bool | None,
        is_reduced_load_mode: bool = False,
    ) -> tuple[
        NDArray[np.int32],  # counter
        NDArray[np.float64] | None,  # accumulated_points
        NDArray[np.float64] | None,  # accumulated_ground_points
        deque[NDArray[np.float64]],  # accum_points_dq
        deque[NDArray[np.float64]],  # accum_ground_dq
        int,  # accum_counter
    ]:
        pass


class StaticAccumPoints(FilterAccumPointsInterface):
    def __init__(self) -> None:
        # multi_scale_icpソースからターゲットへの初期変換
        self.init_source_to_target: o3d.core.Tensor = o3d.core.Tensor.eye(4, o3d.core.Dtype.Float32)

    def accum_point_cloud(
        self,
        xyz_machine_points_removed: NDArray[np.float64],
        counter: NDArray[np.int32],
        accum_points_dq: deque[NDArray[np.float64]],
        accum_points_with_ground_dq: deque[NDArray[np.float64]],
        accum_counter: int,
        delta_yaw: float,
        app_config: AppConfig,
        crane_state: bool | None,
        is_reduced_load_mode: bool = False,
    ) -> tuple[
        NDArray[np.int32],  # counter
        NDArray[np.float64] | None,  # accumulated_points
        NDArray[np.float64] | None,  # accumulated_ground_points
        deque[NDArray[np.float64]],  # accum_points_dq
        deque[NDArray[np.float64]],  # accum_ground_dq
        int,  # accum_counter
    ]:
        (
            accumulated_points,
            init_source_to_target,
        ) = SubScrutinizer.exe_accumulation(
            xyz_machine_points_removed,
            counter,
            accum_points_dq,
            accum_points_with_ground_dq,
            self.init_source_to_target,
            accum_counter,
            delta_yaw,
            app_config.Accumulation,
            app_config.LiDARGrid,
            app_config.General,
            app_config.EdgeDetection.is_applied,
            crane_state,
            is_reduced_load_mode=is_reduced_load_mode,
        )
        
        self.init_source_to_target = init_source_to_target

        return accumulated_points


class AccumAxisPointsCloud(FilterAccumPointsInterface):
    def accum_point_cloud(
        self,
        xyz_machine_points_removed: NDArray[np.float64],
        counter: NDArray[np.int32],
        accum_points_dq: deque[NDArray[np.float64]],
        accum_points_with_ground_dq: deque[NDArray[np.float64]],
        accum_counter: int,
        delta_yaw: float,
        app_config: AppConfig,
        crane_state: bool | None,
        is_reduced_load_mode: bool = False,
    ) -> tuple[
        NDArray[np.int32],  # counter
        NDArray[np.float64] | None,  # accumulated_points
        NDArray[np.float64] | None,  # accumulated_ground_points
        deque[NDArray[np.float64]],  # accum_points_dq
        deque[NDArray[np.float64]],  # accum_ground_dq
        int,  # accum_counter
    ]:
        new_accum_points: NDArray[np.float64] | None
        ground_points: NDArray[np.float64] | None
        new_accum_points, ground_points = filter_lidar_points(
            xyz_machine_points_removed,
            app_config.LiDARGrid,
            app_config.General,
        )
        return (
            counter,
            new_accum_points,
            ground_points,
            accum_points_dq,
            accum_points_with_ground_dq,
            accum_counter,
        )
