import time
from abc import ABC, abstractmethod
from collections.abc import Sequence
from enum import IntEnum
from typing import final

import numpy as np
import open3d as o3d
from numpy.typing import NDArray

from argus_synchro.common.app_logger import AppLogger, AppLoggerFactory
from argus_synchro.common.error import NotStartedError
from argus_synchro.config.app_config_calibration import (
    DataCaptureConf,
)
from argus_synchro.device.lidar.airy96_points import AIRY96Points
from argus_synchro.device.lidar.airy192_points import AIRY192Points
from argus_synchro.device.lidar.mid360_points import MID360Points, MID360PointsFile
from argus_synchro.device.lidar.os0128_points import OS0128Points
from argus_synchro.device.lidar.shi_lib_points import ShiLibPoints


# 点群データ(計測データとして)
###################################
##### for rawdate CH = 5
##### for voxel   CH = 3
class PCD(IntEnum):
    X = 0
    Y = 1
    Z = 2
    INTENSITY = 3  # 反射強度
    TIME = 4  # TTimeStamp
    CH = 3  # チャンネル数
    # enum型の思想からするとイレギュラーな数値定義
    XYZ = 3  # xyzに限定したいデータ処理用


def _as_xyz(
    points: Sequence[Sequence[float]] | NDArray[np.float64],
) -> NDArray[np.float64]:
    array = np.asarray(points, dtype=np.float64)
    if array.size == 0:
        return np.empty((0, PCD.XYZ.value), dtype=np.float64)
    if array.ndim == 1:
        array = array.reshape(1, -1)
    return np.ascontiguousarray(array[:, : PCD.XYZ.value])


def _origin_point_ratio(points: NDArray[np.float64]) -> float:
    if len(points) == 0:
        return 0.0
    is_origin = np.all(points[:, : PCD.XYZ.value] == 0.0, axis=1)
    return float(np.count_nonzero(is_origin)) / len(points)


class PointCloudProvider(ABC):
    @abstractmethod
    def get_points(self) -> tuple[NDArray[np.float64], float] | None:
        """点群を取得する"""
        ...

    @abstractmethod
    def get_accum_points(
        self, max_accum_time: float = 0.1
    ) -> NDArray[np.float64] | None:
        """
        点群データを取得
        max_accum_time: 点群の蓄積時間を指定。デフォルト0.1秒
        """
        ...

    def handle_no_input(self) -> tuple[bool, NDArray[np.float64] | None]:
        """
        入力がNoneのときに処理をする。
        停止させるかどうかと停止させないときに、仮のFrameを返す。

        :return: 停止可否とframe.Trueで停止
        :rtype: tuple[bool, NDArray[float64] | None]
        """
        return False, np.zeros((0, 3))  # 入力が無いときに空データを返す


@final
class NotStartedPointCloudProvider(PointCloudProvider):
    def get_points(self) -> tuple[NDArray[np.float64], float] | None:
        raise NotStartedError

    def get_accum_points(
        self, max_accum_time: float = 0.1
    ) -> NDArray[np.float64] | None:
        raise NotStartedError


@final
class Airy96FilePointCloudProvider(PointCloudProvider):
    def __init__(self, file: str, start_frame: int) -> None:
        self._file: str = file
        self._frame: int = start_frame

    def get_points(self) -> tuple[NDArray[np.float64], float] | None:
        frame = f"{self._frame}".zfill(6)
        lidar_file = self._file + frame + ".npy"
        xyz: NDArray[np.float64] = np.load(lidar_file, allow_pickle=True)

        for idx in range(1, 1):
            frame = f"{self._frame + idx}".zfill(6)
            lidar_file = self._file + frame + ".npy"
            xyz = np.append(
                xyz,
                np.load(lidar_file, allow_pickle=True),
                axis=0,
            )
        self._frame += 1
        return xyz, time.perf_counter()

    def get_accum_points(
        self, max_accum_time: float = 0.1
    ) -> NDArray[np.float64] | None:
        ret = self.get_points()
        if ret is None:
            return None
        points, _ = ret
        return points


class Airy96PointCloudProvider(PointCloudProvider):
    def __init__(self, device: AIRY96Points) -> None:
        self._device: AIRY96Points = device

    def get_points(self) -> tuple[NDArray[np.float64], float] | None:
        points, ts = self._device.get_points()
        return _as_xyz(points), ts

    def get_accum_points(
        self, max_accum_time: float = 0.1
    ) -> NDArray[np.float64] | None:
        # 点群の蓄積時間
        accum_time: float = 0.0
        count: int = 0
        # LiDARから送信されるデータ格納用(Byte).
        packet_chunks: list[NDArray[np.float64]] = []
        ts_init: float = 0.0

        # max_accum_timeまで点群を蓄積する
        while accum_time < max_accum_time:
            try:
                points, ts = self._device.get_points()
            except TimeoutError:
                return None
            xyz = _as_xyz(points)
            if xyz.size:
                packet_chunks.append(xyz)
            if count == 0:
                ts_init = ts
            else:
                accum_time = ts - ts_init
            count += 1
        if not packet_chunks:
            return None
        return np.concatenate(packet_chunks, axis=0)


@final
class Airy96DebugPointCloudProvider(Airy96PointCloudProvider):
    def __init__(self, device: AIRY96Points) -> None:
        super().__init__(device=device)

    def get_points(self) -> tuple[NDArray[np.float64], float] | None:
        ret = super().get_points()
        if ret is None:
            return None
        points, ts = ret
        self._debug(points)
        return points, ts

    def get_accum_points(
        self, max_accum_time: float = 0.1
    ) -> NDArray[np.float64] | None:
        points = super().get_accum_points(
            max_accum_time=max_accum_time,
        )
        if points is None:
            return None
        self._debug(points)
        return points

    def _debug(self, points: NDArray[np.float64]) -> None:
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points[:, :3])
        o3d.visualization.draw_geometries(  # type: ignore
            geometry_list=[pcd],
            window_name="THIS APPEARES WHNE DEBUG MODE IS TRUE",
        )


@final
class Airy192FilePointCloudProvider(PointCloudProvider):
    def __init__(self, file: str, start_frame: int) -> None:
        self._file: str = file
        self._frame: int = start_frame

    def get_points(self) -> tuple[NDArray[np.float64], float] | None:
        frame = f"{self._frame}".zfill(6)
        lidar_file = self._file + frame + ".npy"
        xyz: NDArray[np.float64] = np.load(lidar_file, allow_pickle=True)

        for idx in range(1, 1):
            frame = f"{self._frame + idx}".zfill(6)
            lidar_file = self._file + frame + ".npy"
            xyz = np.append(
                xyz,
                np.load(lidar_file, allow_pickle=True),
                axis=0,
            )
        self._frame += 1
        return xyz, time.perf_counter()

    def get_accum_points(
        self, max_accum_time: float = 0.1
    ) -> NDArray[np.float64] | None:
        ret = self.get_points()
        if ret is None:
            return None
        points, _ = ret
        return points


class Airy192PointCloudProvider(PointCloudProvider):
    def __init__(self, device: AIRY192Points) -> None:
        self._device: AIRY192Points = device

    def get_points(self) -> tuple[NDArray[np.float64], float] | None:
        points, ts = self._device.get_points()
        return _as_xyz(points), ts

    def get_accum_points(
        self, max_accum_time: float = 0.1
    ) -> NDArray[np.float64] | None:
        # 点群の蓄積時間
        accum_time: float = 0.0
        count: int = 0
        # LiDARから送信されるデータ格納用(Byte).
        packet_chunks: list[NDArray[np.float64]] = []
        ts_init: float = 0.0

        # max_accum_timeまで点群を蓄積する
        while accum_time < max_accum_time:
            try:
                points, ts = self._device.get_points()
            except TimeoutError:
                return None
            xyz = _as_xyz(points)
            if xyz.size:
                packet_chunks.append(xyz)
            if count == 0:
                ts_init = ts
            else:
                accum_time = ts - ts_init
            count += 1
        if not packet_chunks:
            return None
        return np.concatenate(packet_chunks, axis=0)


@final
class Airy192DebugPointCloudProvider(Airy192PointCloudProvider):
    def __init__(self, device: AIRY192Points) -> None:
        super().__init__(device=device)

    def get_points(self) -> tuple[NDArray[np.float64], float] | None:
        ret = super().get_points()
        if ret is None:
            return None
        points, ts = ret
        self._debug(points)
        return points, ts

    def get_accum_points(
        self, max_accum_time: float = 0.1
    ) -> NDArray[np.float64] | None:
        points = super().get_accum_points(
            max_accum_time=max_accum_time,
        )
        if points is None:
            return None
        self._debug(points)
        return points

    def _debug(self, points: NDArray[np.float64]) -> None:
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points[:, :3])
        o3d.visualization.draw_geometries(  # type: ignore
            geometry_list=[pcd],
            window_name="THIS APPEARES WHNE DEBUG MODE IS TRUE",
        )


@final
class Mid360FilePointCloudProvider(PointCloudProvider):
    def __init__(self, device: MID360PointsFile, start_frame: int) -> None:
        self._device: MID360PointsFile = device
        self._ref_t: int = start_frame

    def change_file_name_index(self, file_path: str, index: int) -> None:
        self._device.change_file_name_index(file_path)
        self._ref_t = index

    def get_points(self) -> tuple[NDArray[np.float64], float] | None:
        points, _ = self._device.get_points(self._ref_t)
        xyz = _as_xyz(points)

        self._ref_t += 1

        return xyz, time.perf_counter()

    def get_accum_points(
        self, max_accum_time: float = 0.1
    ) -> NDArray[np.float64] | None:
        ret = self.get_points()
        if ret is None:
            return None
        points, _ = ret
        return points


@final
class CalibMid360FilePointCloudProvider(PointCloudProvider):
    def __init__(
        self,
        index: int,
        device: MID360PointsFile,
        lidarconfig: DataCaptureConf.LidarConf,
        start_frame: int,
        app_logger_factory: AppLoggerFactory,
    ) -> None:
        self._logger: AppLogger = app_logger_factory.register_from_type(self.__class__)
        self._index: int = index
        self._device: MID360PointsFile = device
        self.LidarConfig = lidarconfig
        self._ref_t: int = start_frame

    def change_file_name_index(self, file_path: str, index: int) -> None:
        self._logger.info(f"change filepath to {file_path}")
        self._device.change_file_name_index(file_path)
        self._logger.info(f"change index to {index}")
        self._ref_t = index

    def get_points(self) -> tuple[NDArray[np.float64], float] | None:
        points, _ = self._device.get_points(self._ref_t)
        array = np.asarray(points, dtype=np.float64)
        if array.size == 0:
            xyz = np.empty((0, 4), dtype=np.float64)
        else:
            if array.ndim == 1:
                array = array.reshape(1, -1)
            xyz = np.ascontiguousarray(array[:, :4])
        # xyz = np.asarray(points, dtype=np.float64)
        xyz = xyz[xyz[:, 3] != 0]  # 輝度ゼロの点を消去
        xyz = np.ascontiguousarray(xyz[:, :3])

        self._ref_t += 1

        return xyz, time.perf_counter()

    def get_accum_points(
        self, max_accum_time: float = 0.1
    ) -> NDArray[np.float64] | None:
        ret = self.get_points()
        if ret is None:
            return None
        if self.LidarConfig.capture_latency_ms > 0:
            time.sleep(self.LidarConfig.capture_latency_ms / 1000.0)
        points, _ = ret
        return points

    def handle_no_input(self) -> tuple[bool, NDArray[np.float64] | None]:
        self._logger.info(f"lidar{self._index} : cannot read")
        if self.LidarConfig.allow_lack is True:
            return False, np.zeros((0, 4))
        self._logger.info(f"frame None:{self._index = }")
        # self.working_task_flags[task_index] = False
        return True, None


class CalibMid360PointCloudProvider(PointCloudProvider):
    def __init__(
        self,
        index: int,
        device: MID360Points,
        lidarconfig: DataCaptureConf.LidarConf,
        app_logger_factory: AppLoggerFactory,
    ) -> None:
        self._logger: AppLogger = app_logger_factory.register_from_type(self.__class__)
        self._index: int = index
        self._device: MID360Points = device
        self.LidarConfig = lidarconfig
        self._last_invalid_ratio = 0.0

    @property
    def last_quality_degraded(self) -> float:
        """デバイスの最終品質低下時刻"""
        return self._device.last_quality_degraded

    @property
    def last_invalid_ratio(self) -> float:
        """直近フレームの原点点割合 (SE020用)"""
        return self._last_invalid_ratio

    def get_points(self) -> tuple[NDArray[np.float64], float] | None:
        try:
            points, ts = self._device.get_points()
        except TimeoutError:
            return None

        return _as_xyz(points), ts

    def get_accum_points(
        self,
        max_accum_time: float = 0.1,  # TODO: config追加
    ) -> NDArray[np.float64] | None:
        # 点群の蓄積時間
        accum_time: float = 0.0
        count: int = 0
        # LiDARから送信されるデータ格納用(Byte).
        packet_chunks: list[NDArray[np.float64]] = []
        ts_init: float = 0.0

        # max_accum_timeまで点群を蓄積する
        while accum_time < max_accum_time:
            try:
                points, ts = self._device.get_points()
            except TimeoutError:
                return None
            xyz = np.asarray(points, dtype=np.float64)
            if xyz.size:
                packet_chunks.append(xyz)
            if count == 0:
                ts_init = ts
            else:
                accum_time = ts - ts_init
            count += 1
        if not packet_chunks:
            return None
        frame = np.concatenate(packet_chunks, axis=0)
        self._last_invalid_ratio = _origin_point_ratio(frame)
        frame = frame[frame[:, 3] != 0]  # 輝度ゼロの点群を弾く
        xyz = _as_xyz(frame)
        return xyz

    def handle_no_input(self) -> tuple[bool, NDArray[np.float64] | None]:
        self._logger.info(f"lidar{self._index} : cannot read")
        if self.LidarConfig.allow_lack is True:
            return False, np.zeros((0, 4))

        # self.working_task_flags[task_index] = False
        self._logger.info(f"frame None:{self._index = }")
        return True, None


class Mid360PointCloudProvider(PointCloudProvider):
    def __init__(self, device: MID360Points) -> None:
        self._device: MID360Points = device
        self._last_invalid_ratio = 0.0

    @property
    def last_quality_degraded(self) -> float:
        return self._device.last_quality_degraded

    @property
    def last_invalid_ratio(self) -> float:
        return self._last_invalid_ratio

    def get_points(self) -> tuple[NDArray[np.float64], float] | None:
        try:
            points, ts = self._device.get_points()
        except TimeoutError:
            return None

        return _as_xyz(points), ts

    def get_accum_points(
        self, max_accum_time: float = 0.1
    ) -> NDArray[np.float64] | None:
        # 点群の蓄積時間
        accum_time: float = 0.0
        count: int = 0
        # LiDARから送信されるデータ格納用(Byte).
        packet_chunks: list[NDArray[np.float64]] = []
        ts_init: float = 0.0

        # max_accum_timeまで点群を蓄積する
        while accum_time < max_accum_time:
            try:
                points, ts = self._device.get_points()
            except TimeoutError:
                return None
            xyz = _as_xyz(points)
            if xyz.size:
                packet_chunks.append(xyz)
            if count == 0:
                ts_init = ts
            else:
                accum_time = ts - ts_init
            count += 1
        if not packet_chunks:
            return None
        frame = np.concatenate(packet_chunks, axis=0)
        self._last_invalid_ratio = _origin_point_ratio(frame)
        return frame


@final
class Mid360DebugPointCloudProvider(Mid360PointCloudProvider):
    def __init__(self, device: MID360Points) -> None:
        super().__init__(device=device)

    def get_points(self) -> tuple[NDArray[np.float64], float] | None:
        ret = super().get_points()
        if ret is None:
            return None
        points, ts = ret
        self._debug(points)
        return points, ts

    def get_accum_points(
        self, max_accum_time: float = 0.1
    ) -> NDArray[np.float64] | None:
        points = super().get_accum_points(
            max_accum_time=max_accum_time,
        )
        if points is None:
            return None
        self._debug(points)
        return points

    def _debug(self, points: NDArray[np.float64]) -> None:
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points[:, :3])
        o3d.visualization.draw_geometries(  # type: ignore
            geometry_list=[pcd],
            window_name="THIS APPEARES WHNE DEBUG MODE IS TRUE",
        )


@final
class Os0128FilePointCloudProvider(PointCloudProvider):
    def __init__(self, file: str, start_frame: int) -> None:
        self._file: str = file
        self._frame: int = start_frame

    def get_points(self) -> tuple[NDArray[np.float64], float] | None:
        frame = f"{self._frame}".zfill(6)
        lidar_file = self._file + frame + ".npy"
        xyz: NDArray[np.float64] = np.load(lidar_file, allow_pickle=True)

        for idx in range(1, 1):
            frame = f"{self._frame + idx}".zfill(6)
            lidar_file = self._file + frame + ".npy"
            xyz = np.append(
                xyz,
                np.load(lidar_file, allow_pickle=True),
                axis=0,
            )
        self._frame += 1

        return xyz, time.perf_counter()

    def get_accum_points(
        self, max_accum_time: float = 0.1
    ) -> NDArray[np.float64] | None:
        ret = self.get_points()
        if ret is None:
            return None
        points, _ = ret
        return points


class Os0128PointCloudProvider(PointCloudProvider):
    def __init__(self, device: OS0128Points) -> None:
        self._device: OS0128Points = device

    def get_points(self) -> tuple[NDArray[np.float64], float] | None:
        points, ts = self._device.get_points()

        return _as_xyz(points), ts

    def get_accum_points(
        self, max_accum_time: float = 0.1
    ) -> NDArray[np.float64] | None:
        # 点群の蓄積時間
        accum_time: float = 0.0
        count: int = 0
        # LiDARから送信されるデータ格納用(Byte).
        packet_chunks: list[NDArray[np.float64]] = []
        ts_init: float = 0.0

        # max_accum_timeまで点群を蓄積する
        while accum_time < max_accum_time:
            points, ts = self._device.get_points()
            xyz = _as_xyz(points)
            if xyz.size:
                packet_chunks.append(xyz)
            if count == 0:
                ts_init = ts
            else:
                accum_time = ts - ts_init
            count += 1
        if not packet_chunks:
            return None
        return np.concatenate(packet_chunks, axis=0)


@final
class Os0128DebugPointCloudProvider(Os0128PointCloudProvider):
    def __init__(self, device: OS0128Points) -> None:
        super().__init__(device=device)

    def get_points(self) -> tuple[NDArray[np.float64], float] | None:
        ret = super().get_points()
        if ret is None:
            return None
        points, ts = ret
        self._debug(points)
        return points, ts

    def get_accum_points(
        self, max_accum_time: float = 0.1
    ) -> NDArray[np.float64] | None:
        points = super().get_accum_points(
            max_accum_time=max_accum_time,
        )
        if points is None:
            return None
        self._debug(points)
        return points

    def _debug(self, points: NDArray[np.float64]) -> None:
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points[:, :3])
        o3d.visualization.draw_geometries(  # type: ignore
            geometry_list=[pcd],
            window_name="THIS APPEARES WHNE DEBUG MODE IS TRUE",
        )


class ShiLibPointCloudProvider(PointCloudProvider):
    def __init__(self, device: ShiLibPoints) -> None:
        self._device: ShiLibPoints = device

    def get_points(self) -> tuple[NDArray[np.float64], float] | None:
        points, ts = self._device.get_points()

        return _as_xyz(points), ts

    def get_accum_points(
        self, max_accum_time: float = 0.1
    ) -> NDArray[np.float64] | None:
        # 点群の蓄積時間
        accum_time: float = 0.0
        count: int = 0
        # LiDARから送信されるデータ格納用(Byte).
        packet_chunks: list[NDArray[np.float64]] = []
        ts_init: float = 0.0

        # max_accum_timeまで点群を蓄積する
        while accum_time < max_accum_time:
            points, ts = self._device.get_points()
            xyz = _as_xyz(points)
            if xyz.size:
                packet_chunks.append(xyz)
            if count == 0:
                ts_init = ts
            else:
                accum_time = ts - ts_init
            count += 1
        if not packet_chunks:
            return None
        return np.concatenate(packet_chunks, axis=0)


@final
class ShiLibDebugPointCloudProvider(ShiLibPointCloudProvider):
    def __init__(self, device: ShiLibPoints) -> None:
        super().__init__(device=device)

    def get_points(self) -> tuple[NDArray[np.float64], float] | None:
        ret = super().get_points()
        if ret is None:
            return None
        points, ts = ret
        self._debug(points)
        return points, ts

    def get_accum_points(
        self, max_accum_time: float = 0.1
    ) -> NDArray[np.float64] | None:
        points = super().get_accum_points(
            max_accum_time=max_accum_time,
        )
        if points is None:
            return None
        self._debug(points)
        return points

    def _debug(self, points: NDArray[np.float64]) -> None:
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points[:, :3])
        o3d.visualization.draw_geometries(  # type: ignore
            geometry_list=[pcd],
            window_name="THIS APPEARES WHNE DEBUG MODE IS TRUE",
        )
