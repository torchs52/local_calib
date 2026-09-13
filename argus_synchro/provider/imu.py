from abc import ABC, abstractmethod
from collections import deque
from typing import final

import numpy as np
from numpy.typing import NDArray

from argus_synchro.common.error import NotStartedError
from argus_synchro.device.lidar.airy96_imu import AIRY96Imu, AIRY96ImuFile
from argus_synchro.device.lidar.airy192_imu import AIRY192Imu, AIRY192ImuFile
from argus_synchro.device.lidar.mid360_imu import MID360Imu, MID360ImuFile
from argus_synchro.device.lidar.os0128_imu import OS0128Imu, OS0128ImuFile
from argus_synchro.device.lidar.shi_lib_imu import ShiLibImu
from argus_synchro.message.input_message import IMUConfig


class ImuProvider(ABC):
    def __init__(self) -> None:
        self._TRACK_SAMPLES: int = int(
            IMUConfig.IMU_RATE_HZ * IMUConfig.TRACK_SEC
        )  # 200Hz*N秒分のデータ
        self._imu_ring: deque[NDArray[np.float64]] = deque(maxlen=self._TRACK_SAMPLES)

    def get_accum_point(self) -> tuple[deque[NDArray[np.float64]], float]:
        imu, t = self.get_imu()
        # リングバッファ更新
        # maxlenを超過した分は自動で捨てられる
        self._imu_ring.append(np.asarray(imu, dtype=np.float64))
        return self._imu_ring, t

    @abstractmethod
    def get_imu(self) -> tuple[NDArray[np.float64], float]:
        """imuデータを取得する"""
        ...


@final
class NotStartedImuProvider(ImuProvider):
    def get_imu(self) -> tuple[NDArray[np.float64], float]:
        raise NotStartedError


@final
class Airy96ImuFileProvider(ImuProvider):
    def __init__(self, device: AIRY96ImuFile) -> None:
        super().__init__()
        self._device: AIRY96ImuFile = device

    def get_imu(self) -> tuple[NDArray[np.float64], float]:
        gyro, acce, ts = self._device.get_imu()
        return np.array((gyro, acce), dtype=np.float64), ts


@final
class Airy96ImuProvider(ImuProvider):
    def __init__(self, device: AIRY96Imu) -> None:
        super().__init__()
        self._device: AIRY96Imu = device

    def get_imu(self) -> tuple[NDArray[np.float64], float]:
        gyro, acce, ts = self._device.get_imu()
        return np.array((gyro, acce), dtype=np.float64), ts


@final
class Airy192ImuFileProvider(ImuProvider):
    def __init__(self, device: AIRY192ImuFile) -> None:
        super().__init__()
        self._device: AIRY192ImuFile = device

    def get_imu(self) -> tuple[NDArray[np.float64], float]:
        gyro, acce, ts = self._device.get_imu()
        return np.array((gyro, acce), dtype=np.float64), ts


@final
class Airy192ImuProvider(ImuProvider):
    def __init__(self, device: AIRY192Imu) -> None:
        super().__init__()
        self._device: AIRY192Imu = device

    def get_imu(self) -> tuple[NDArray[np.float64], float]:
        gyro, acce, ts = self._device.get_imu()
        return np.array((gyro, acce), dtype=np.float64), ts


@final
class Mid360ImuFileProvider(ImuProvider):
    def __init__(self, device: MID360ImuFile) -> None:
        super().__init__()
        self._device: MID360ImuFile = device

    def get_imu(self) -> tuple[NDArray[np.float64], float]:
        gyro, acce, ts = self._device.get_imu()
        return np.array((gyro, acce), dtype=np.float64), ts


@final
class Mid360ImuProvider(ImuProvider):
    def __init__(self, device: MID360Imu) -> None:
        super().__init__()
        self._device: MID360Imu = device

    def get_imu(self) -> tuple[NDArray[np.float64], float]:
        gyro, acce, ts = self._device.get_imu()
        return np.array((gyro, acce), dtype=np.float64), ts


@final
class Os0128ImuFileProvider(ImuProvider):
    def __init__(self, device: OS0128ImuFile) -> None:
        super().__init__()
        self._device: OS0128ImuFile = device

    def get_imu(self) -> tuple[NDArray[np.float64], float]:
        gyro, acce, ts = self._device.get_imu()
        return np.array((gyro, acce), dtype=np.float64), ts


@final
class Os0128ImuProvider(ImuProvider):
    def __init__(self, device: OS0128Imu) -> None:
        super().__init__()
        self._device: OS0128Imu = device

    def get_imu(self) -> tuple[NDArray[np.float64], float]:
        gyro, acce, ts = self._device.get_imu()
        return np.array((gyro, acce), dtype=np.float64), ts


@final
class ShiLibImuProvider(ImuProvider):
    def __init__(self, device: ShiLibImu) -> None:
        super().__init__()
        self._device: ShiLibImu = device

    def get_imu(self) -> tuple[NDArray[np.float64], float]:
        gyro, acce, ts = self._device.get_imu()
        return np.array((gyro, acce), dtype=np.float64), ts
