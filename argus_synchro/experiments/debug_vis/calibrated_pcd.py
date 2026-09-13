"""崖検出でLiDAR毎に点群を分けて保持するために作成したインターフェース
試作プログラムのため、interfaceディレクトリから分けて使っていて、実際に使う場合は、interfaceディレクトリに置く
"""

from abc import ABC, abstractmethod

import numpy as np
import pandas as pd
from argus_sychro.SharedClasses import Shared_class
from numpy.typing import NDArray

from argus_synchro.common.common import t_np_float
from argus_synchro.config.app_config import AppConfig, CalibrationConf
from argus_synchro.interface.pcd_calib import PCDCalibInterface
from argus_synchro.interface.pcd_data import PCDRealDataInterface
from argus_synchro.interface.real_point_data import DirectionPointDataInterface


class DebugPCDDataInterface(ABC):
    """
    デバッグ用の点群データ生成クラス
    元々のget_pcd_dataはNDArrayを返しているが、LiDAR毎に情報を保持した場合に用いる
    """

    @abstractmethod
    def get_pcd_data(
        self,
        ref_t: int,
        app_config: AppConfig,
        sc: Shared_class,
    ) -> list[t_np_float]:
        pass


class CalibratedListedPCD(DebugPCDDataInterface):
    def __init__(
        self,
        pcd_real_data: PCDRealDataInterface,
        real_direction_point: DirectionPointDataInterface,
        pcd_proofreading: PCDCalibInterface,
    ) -> None:
        self.pcd_real_data: PCDRealDataInterface = pcd_real_data
        self.real_direction_point: DirectionPointDataInterface = real_direction_point
        self.pcd_proofreading: PCDCalibInterface = pcd_proofreading

    def update(
        self,
        pcd_real_data: PCDRealDataInterface,
        real_direction_point: DirectionPointDataInterface,
        pcd_proofreading: PCDCalibInterface,
    ) -> None:
        self.pcd_real_data: PCDRealDataInterface = pcd_real_data
        self.real_direction_point: DirectionPointDataInterface = real_direction_point
        self.pcd_proofreading: PCDCalibInterface = pcd_proofreading

    def get_pcd_data(
        self,
        ref_t: int,
        app_config: AppConfig,
        sc: Shared_class,
    ) -> NDArray[np.float64] | list[t_np_float]:
        xyz_file_data: list[NDArray[np.float64]] = self.pcd_real_data.get_pcd_real_data(
            ref_t,
            app_config.Lidar.lidar_files,
        )
        # FileInput = Falseのときは空の3次元配列が返される.
        return self.get_combined_points(
            sc=sc,
            app_config=app_config,
            xyz_data=xyz_file_data,
            calibration_conf=app_config.calibration,
        )

    def get_combined_points(
        self,
        sc: Shared_class,
        app_config: AppConfig,
        xyz_data: list[NDArray[np.float64]],
        calibration_conf: CalibrationConf,
    ) -> list[t_np_float]:
        """
        任意台数の LiDAR から点群を取得し、ダウンサンプリング→キャリブレーション→結合を行う

        Parameters
        ----------
        sc : Shared_class
        voxel_down_sample : float
            voxel_down_sample のパラメータ
        xyz_data : list[np.ndarray]
            未使用
        calibration_conf : CalibrationConf
            キャリブレーション設定

        Returns
        -------
        np.ndarray
            すべてのLiDARのキャリブレーション後点群を結合した(N×3)配列
        """
        # 1. 各 LiDAR の最新点群をリストで取得
        xyz_list: list[NDArray[np.float64]] = (
            self.real_direction_point.get_direction_point(xyz_data, sc)
        )
        # 台数チェックを挿入
        if len(xyz_list) != app_config.Lidar.count:
            raise ValueError(
                f"Expected {app_config.Lidar.count} LiDAR point clouds, "
                f"but got {len(xyz_list)}. "
                f"settings.iniの[Lidar.count]と[Lidar.lidar_files]の要素数が一致するか確認してください"
            )
        # 2. キャリブレーション（可変長の numpy.ndarray に対応）
        calibrated: list[NDArray[np.float64]] = self.pcd_proofreading.calib_lidar(
            *xyz_list,
            app_config=app_config,
            calibration_conf=calibration_conf,
        )

        # 3. 戻り値をiterableに統一
        if isinstance(calibrated, np.ndarray):
            calibrated_list = [calibrated]
        else:
            calibrated_list = calibrated

        # 4. 全点群を縦方向に結合して返却
        return calibrated_list

    def decompose_calib_data(
        self,
        calibrated_xyz_data: list[NDArray[np.float64]],
        calibration_conf: CalibrationConf,
    ) -> list[t_np_float]:
        return [
            inverse_calib(one_calib_xyz, calib_path)
            for one_calib_xyz, calib_path in zip(
                calibrated_xyz_data, calibration_conf.Lidar_calib_files
            )
        ]


def load_transmat(calib_path: str) -> t_np_float:
    return pd.read_csv(calib_path, header=None).values


def inverse_transform(xyz: t_np_float, trans_mat_44: NDArray) -> t_np_float:
    trans_vec = trans_mat_44[:3, -1]
    rot_mat = trans_mat_44[:3, :3]
    return (xyz - trans_vec) @ rot_mat


def inverse_calib(xyz: t_np_float, calib_path: str) -> t_np_float:
    calib_mat = load_transmat(calib_path)
    return inverse_transform(xyz, calib_mat)
