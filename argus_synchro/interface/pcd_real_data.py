from __future__ import annotations

from abc import ABC, abstractmethod

import numpy as np
from numpy.typing import NDArray

from argus_synchro.config.app_config import AppConfig, CalibrationConf
from argus_synchro.interface.pcd_calib import PCDCalibInterface
from argus_synchro.message.input_message import PointCloudData


class PCDDataInterface(ABC):
    @abstractmethod
    def get_pcd_data(
        self,
        ref_t: int,
        app_config: AppConfig,
        pcd_data: tuple[PointCloudData, ...],
    ) -> NDArray[np.float64]:
        pass


class PCDRealData(PCDDataInterface):
    __slots__ = ("pcd_proofreading",)

    def __init__(
        self,
        pcd_proofreading: PCDCalibInterface,
    ) -> None:
        self.pcd_proofreading: PCDCalibInterface = pcd_proofreading

    def update(
        self,
        pcd_proofreading: PCDCalibInterface,
    ) -> None:
        self.pcd_proofreading: PCDCalibInterface = pcd_proofreading

    def get_pcd_data(
        self,
        ref_t: int,
        app_config: AppConfig,
        pcd_data: tuple[PointCloudData, ...],
    ) -> NDArray[np.float64]:
        # FileInput = Falseのときは空の3次元配列が返される.
        return self.get_combined_points(
            pcd_data=pcd_data,
            app_config=app_config,
            calibration_conf=app_config.calibration,
        )

    def get_combined_points(
        self,
        pcd_data: tuple[PointCloudData, ...],
        app_config: AppConfig,
        calibration_conf: CalibrationConf,
    ) -> NDArray[np.float64]:
        """
        任意台数の LiDAR から点群を取得し、ダウンサンプリング→キャリブレーション→結合を行う

        Parameters
        ----------
        sc : Shared_class
        voxel_down_sample : float
            voxel_down_sample のパラメータ
        xyz_data :list[np.ndarray]
            未使用
        calibration_conf : CalibrationConf
            キャリブレーション設定

        Returns
        -------
        np.ndarray
            すべてのLiDARのキャリブレーション後点群を結合した(N×3)配列
        """
        # 1. 各 LiDAR の最新点群をリストで取得
        xyz_list = [d.point_cloud for d in pcd_data]
        # 台数チェックを挿入
        if len(xyz_list) != app_config.Lidar.count:
            raise ValueError(
                f"Expected {app_config.Lidar.count} LiDAR point clouds, "
                f"but got {len(xyz_list)}. "
                f"settings.iniの[Lidar.count]と[Lidar.lidar_files]の要素数が一致するか確認してください"
            )
        # 2. キャリブレーション(可変長の numpy.ndarray に対応)
        calibrated: (
            NDArray[np.float64]
            | list[NDArray[np.float64]]
            | tuple[NDArray[np.float64], ...]
        ) = self.pcd_proofreading.calib_lidar(xyz_list)

        # 3. 戻り値をiterableに統一
        if isinstance(calibrated, np.ndarray):
            calibrated_list = (calibrated,)
        else:
            calibrated_list = calibrated

        # 4. 全点群を縦方向に結合して返却
        return np.vstack(calibrated_list)
