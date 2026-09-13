from __future__ import annotations

from dataclasses import dataclass
from enum import IntEnum
from typing import Protocol, final

import numpy as np
from numpy.typing import NDArray

from argus_synchro.process.message import SlotMessage
from argus_synchro.shared_data import SharedArraySlotData, SharedScalarSlotData


class ResolutionLike(Protocol):
    WIDTH: int
    HEIGHT: int


class IMU(IntEnum):
    GYRO_X = 0
    GYRO_Y = 1
    GYRO_Z = 2
    ACCE_X = 3
    ACCE_Y = 4
    ACCE_Z = 5
    CH = 6


class PCD(IntEnum):
    X = 0
    Y = 1
    Z = 2
    INTENSITY = 3  # 反射強度
    TIME = 4  # TTimeStamp
    CH = 3  # チャンネル数
    # enum型の思想からするとイレギュラーな数値定義
    XYZ = 3  # xyzに限定したいデータ処理用


@dataclass(frozen=True, slots=True)
class PointCloudData:
    frame: int
    time: float
    point_cloud: NDArray[np.float64]


@dataclass(frozen=True, slots=True)
class ImuData:
    frame: int
    time: float
    imu: NDArray[np.float64]


@dataclass(frozen=True, slots=True)
class CameraData:
    index: int
    frame: int
    time: float
    image: NDArray[np.uint8]


@dataclass(frozen=True, slots=True)
class CanData:
    yaw_angle_deg: float
    lever_pressure: NDArray[np.float16]
    frame: int
    time: float


# NOTE: ファイル入力だと点群数がオーバーするため、
# 元のargus_synchroと同じ表示にしたい場合は100000にする
class PcdData:
    SIZE = 40000  # 一度に入力される点群データの数


class IMUConfig:
    IMU_RATE_HZ = 200  # IMUの周波数
    TRACK_SEC = 5.0  # 保持するIMUデータの秒数


class PCDDataMessage(SlotMessage[PointCloudData]):
    __slots__ = ("_frame", "_num", "_point_cloud", "_time")

    def __init__(self) -> None:
        super().__init__()
        self._frame: SharedScalarSlotData[int] = SharedScalarSlotData(int, 0)
        self._time: SharedScalarSlotData[float] = SharedScalarSlotData(float, 0.0)
        self._num: SharedScalarSlotData[int] = SharedScalarSlotData(int, 0)
        self._point_cloud: SharedArraySlotData[np.float64] = SharedArraySlotData(
            (PcdData.SIZE, 3), np.float64
        )

    def write_slot(self, slot: int, value: PointCloudData) -> None:
        self._frame.write_slot(slot, int(value.frame))
        self._time.write_slot(slot, float(value.time))
        num = min(value.point_cloud.shape[0], PcdData.SIZE)
        self._num.write_slot(slot, int(num))
        self._point_cloud.write_slot_slice(slot, value.point_cloud[:num])

    def borrow_slot(self, slot: int) -> PointCloudData:
        num = int(self._num.read_slot_value(slot))
        return PointCloudData(
            frame=int(self._frame.read_slot_value(slot)),
            time=float(self._time.read_slot_value(slot)),
            point_cloud=self._point_cloud.borrow_slot_slice(
                slot, (slice(0, num), slice(None))
            ),
        )

    def _close(self) -> None:
        self._frame.close()
        self._time.close()
        self._num.close()
        self._point_cloud.close()
        self._close_slot_controller()


@final
class ImuMessage(SlotMessage[ImuData]):
    __slots__ = (
        "_frame",
        "_imu",
        "_max_cols",
        "_max_rows",
        "_num_cols",
        "_num_rows",
        "_time",
    )

    def __init__(self) -> None:
        super().__init__()
        self._max_rows = int(IMUConfig.IMU_RATE_HZ * IMUConfig.TRACK_SEC)
        self._max_cols = int(IMU.CH)
        self._frame: SharedScalarSlotData[int] = SharedScalarSlotData(int, 0)
        self._time: SharedScalarSlotData[float] = SharedScalarSlotData(float, 0.0)
        self._num_rows: SharedScalarSlotData[int] = SharedScalarSlotData(
            int, self._max_rows
        )
        self._num_cols: SharedScalarSlotData[int] = SharedScalarSlotData(
            int, self._max_cols
        )
        self._imu: SharedArraySlotData[np.float64] = SharedArraySlotData(
            (self._max_rows, self._max_cols), np.float64
        )

    def write_slot(self, slot: int, value: ImuData) -> None:
        self._frame.write_slot(slot, int(value.frame))
        self._time.write_slot(slot, float(value.time))
        rows = min(value.imu.shape[0], self._max_rows)
        cols = min(value.imu.shape[1], self._max_cols)
        self._num_rows.write_slot(slot, int(rows))
        self._num_cols.write_slot(slot, int(cols))
        self._imu.write_slot_slice(slot, value.imu[:rows, :cols])

    def borrow_slot(self, slot: int) -> ImuData:
        rows = int(self._num_rows.read_slot_value(slot))
        cols = int(self._num_cols.read_slot_value(slot))
        return ImuData(
            frame=int(self._frame.read_slot_value(slot)),
            time=float(self._time.read_slot_value(slot)),
            imu=self._imu.borrow_slot_slice(slot, (slice(0, rows), slice(0, cols))),
        )

    def _close(self) -> None:
        self._frame.close()
        self._time.close()
        self._num_rows.close()
        self._num_cols.close()
        self._imu.close()
        self._close_slot_controller()


@final
class CanDataMessage(SlotMessage[CanData]):
    __slots__ = ("_frame", "_handle_lever", "_time", "_yaw_angle_deg")

    def __init__(self) -> None:
        super().__init__()
        self._yaw_angle_deg: SharedScalarSlotData[float] = SharedScalarSlotData(
            float, 0.0
        )
        self._handle_lever: SharedArraySlotData[np.float16] = SharedArraySlotData(
            (4,), np.float16
        )
        self._time: SharedScalarSlotData[float] = SharedScalarSlotData(float, 0.0)
        self._frame: SharedScalarSlotData[int] = SharedScalarSlotData(int, 0)

    def write_slot(self, slot: int, value: CanData) -> None:
        self._yaw_angle_deg.write_slot(slot, float(value.yaw_angle_deg))
        self._handle_lever.write_slot(slot, value.lever_pressure)
        self._frame.write_slot(slot, int(value.frame))
        self._time.write_slot(slot, float(value.time))

    def borrow_slot(self, slot: int) -> CanData:
        return CanData(
            yaw_angle_deg=float(self._yaw_angle_deg.read_slot_value(slot)),
            lever_pressure=self._handle_lever.borrow_slot(slot),
            frame=int(self._frame.read_slot_value(slot)),
            time=float(self._time.read_slot_value(slot)),
        )

    def _close(self) -> None:
        self._yaw_angle_deg.close()
        self._handle_lever.close()
        self._time.close()
        self._frame.close()
        self._close_slot_controller()


class CameraMessage(SlotMessage[CameraData]):
    __slots__ = ("_frame", "_height", "_image", "_index", "_time", "_width")

    __MAX_IMAGE_SHAPE = (1080, 1920, 3)

    def __init__(self) -> None:
        super().__init__()
        self._index: SharedScalarSlotData[int] = SharedScalarSlotData(int, 0)
        self._frame: SharedScalarSlotData[int] = SharedScalarSlotData(int, 0)
        self._time: SharedScalarSlotData[float] = SharedScalarSlotData(float, 0.0)
        self._image: SharedArraySlotData[np.uint8] = SharedArraySlotData(
            self.__MAX_IMAGE_SHAPE, np.uint8
        )
        self._height: SharedScalarSlotData[int] = SharedScalarSlotData(int, 0)
        self._width: SharedScalarSlotData[int] = SharedScalarSlotData(int, 0)

    def write_slot(self, slot: int, value: CameraData) -> None:
        self._index.write_slot(slot, int(value.index))
        self._frame.write_slot(slot, int(value.frame))
        self._time.write_slot(slot, float(value.time))
        self._image.write_slot_slice(slot, value.image)
        self._height.write_slot(slot, int(value.image.shape[0]))
        self._width.write_slot(slot, int(value.image.shape[1]))

    def borrow_slot(self, slot: int) -> CameraData:
        height = self._height.read_slot_value(slot)
        width = self._width.read_slot_value(slot)

        return CameraData(
            index=int(self._index.read_slot_value(slot)),
            frame=int(self._frame.read_slot_value(slot)),
            time=float(self._time.read_slot_value(slot)),
            image=self._image.borrow_slot_slice(
                slot, (slice(0, height), slice(0, width), slice(None))
            ),
        )

    def _close(self) -> None:
        self._index.close()
        self._frame.close()
        self._time.close()
        self._image.close()
        self._height.close()
        self._width.close()
        self._close_slot_controller()
