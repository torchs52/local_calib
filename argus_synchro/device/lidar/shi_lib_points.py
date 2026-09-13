from __future__ import annotations

import builtins
import contextlib
from typing import TYPE_CHECKING

from argus_synchro.common.app_logger import AppLogger, AppLoggerFactory
from argus_synchro.config.app_config import LidarConf

if TYPE_CHECKING:
    import pybind_shi_sensor_lib as shi
with contextlib.suppress(builtins.BaseException):
    import pybind_shi_sensor_lib as shi


class ShiLibPoints:
    def __init__(
        self,
        index: int,
        lidar_conf: LidarConf,
        app_logger_factory: AppLoggerFactory,
    ) -> None:
        self._logger: AppLogger = app_logger_factory.register_from_type(self.__class__)

        self._index = index
        self._config: str = lidar_conf.config_file
        self._accum_time = lidar_conf.accum_time
        self.lidar_dev = shi.Lidar()
        self._pcd = shi.LidarPointCloudArray()

    def connect(self) -> None:
        self.lidar_dev.init(self._config)

    def disconnect(self) -> None:
        err_msg = f"class: {self.__class__.__name__}, method: disconnect()"
        raise NotImplementedError(err_msg)

    def get_points(self) -> tuple[list[tuple[float, float, float, int]], float]:
        timestamp_us = 2000000
        is_latest = True
        handle: int = self._index

        # 点群データの取得
        rts = self.lidar_dev.getSensorData(handle, self._pcd, timestamp_us, is_latest)

        if rts == shi.ApiStatus.SUCCESS:
            # 取得成功時
            self._logger.info(
                f"GET pcd ({handle}), elapsed_time, {elapsed_time_ms / 1e6}, Number of Data:{self._pcd.data_num}, TIMESTAMP:{self._pcd.timestamp_ms}",
            )

            pcd = [
                (data.x / 1e3, data.y / 1e3, data.z / 1e3, data.reflectivity)
                for data in self._pcd.data[: self._pcd.data_num]
            ]
            current_ts = self._pcd.timestamp_ms / 1e3
            return pcd, current_ts

        # 取得失敗時
        self._logger.info(f"get lidar pcd error. status:{rts}")
        return [], -1
