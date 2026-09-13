from abc import ABC, abstractmethod

import numpy as np
import pandas as pd
from numpy.typing import NDArray

from argus_synchro import calibration
from argus_synchro.config.app_config import CalibrationConf, LidarConf


class PCDCalibInterface(ABC):
    @abstractmethod
    def calib_lidar(
        self, point_arrays: list[NDArray[np.float64]]
    ) -> list[NDArray[np.float64]]:
        pass


# class LRCalib(PCDCalibInterface):
#    def calib_lidar(
#        self,
#        *point_arrays,
#        app_config: AppConfig,
#        calibration_conf: CalibrationConf,
#    ) -> list[PointCloud]:
#        pcd_left = calibration.calib_lidar(pcd_left, calibration_conf.Lidar1)
#        pcd_righ = calibration.calib_lidar(pcd_righ, calibration_conf.Lidar2)
#        return pcd_left, pcd_righ


class MultiCalib(PCDCalibInterface):
    __slots__ = ("_transform_values",)

    def __init__(
        self,
        lidar_conf: LidarConf,
        calibration_conf: CalibrationConf,
    ) -> None:
        self.update(lidar_conf, calibration_conf)

    def update(
        self,
        lidar_conf: LidarConf,
        calibration_conf: CalibrationConf,
    ) -> None:
        files = calibration_conf.Lidar_calib_files
        if len(files) != lidar_conf.count:
            raise ValueError(
                f"Expected {lidar_conf.count} Calibration files,"
                f"but got {len(files)}. "
                f"settings.iniの[Lidar.count]と[calibration.lidar_calib_files]の要素数が一致するか確認してください"
            )
        self._transform_values: list[NDArray[np.float64]] = [
            pd.read_csv(fpath, header=None).values for fpath in files
        ]

    def calib_lidar(
        self, point_arrays: list[NDArray[np.float64]]
    ) -> list[NDArray[np.float64]]:
        """
        任意台数の numpy 点群をキャリブレーションする。

        Parameters
        ----------
        *point_arrays : tuple of (N_i,3) arrays
            各LiDARの生点群

        Returns
        -------
        List of (N_i,3) arrays
            キャリブレーション後の各点群
        """
        calibrated: list[NDArray[np.float64]] = [
            calibration.calib_lidar(pts, values)
            for pts, values in zip(point_arrays, self._transform_values, strict=False)
        ]
        return calibrated


class RCalib(PCDCalibInterface):
    __slots__ = ("_transform_values",)

    def __init__(self, calibration_conf: CalibrationConf) -> None:
        self.update(calibration_conf)

    def update(self, calibration_conf: CalibrationConf) -> None:
        self._transform_values: NDArray[np.float64] = pd.read_csv(
            calibration_conf.BothLidars,
            header=None,
        ).values

    def calib_lidar(
        self, point_arrays: list[NDArray[np.float64]]
    ) -> list[NDArray[np.float64]]:
        pcd_right = calibration.calib_lidar(
            point_arrays[1],
            self._transform_values,
        )
        return [point_arrays[0], pcd_right]
