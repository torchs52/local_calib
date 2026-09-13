from __future__ import annotations

from dataclasses import dataclass
from typing import Protocol

import numpy as np
from numpy.typing import NDArray

from argus_synchro.config.app_config import AppConfig
from argus_synchro.edge_det.base import EdgeDetectionResult
from argus_synchro.message.input_message import PcdData
from argus_synchro.process.message import SlotMessage
from argus_synchro.shared_data import SharedArraySlotData, SharedScalarSlotData


class ResolutionLike(Protocol):
    WIDTH: int
    HEIGHT: int


@dataclass(slots=True)
class AccumGroundPointsData:
    frame: int
    time: float
    point_cloud: NDArray[np.float64]


@dataclass(slots=True)
class AccumPointsData:
    frame: int
    time: float
    point_cloud: NDArray[np.float64]
    yaw_angle_deg: float


@dataclass(slots=True)
class DeltaYawData:
    frame: int
    delta_yaw: float


@dataclass(slots=True)
class RemovePointsData:
    frame: int
    time: float
    point_cloud: NDArray[np.float64]


@dataclass(slots=True)
class CameraDetectionsData:
    index: int
    frame: int
    time: float
    boxes: NDArray[np.float32]
    scores: NDArray[np.float32]
    classes: NDArray[np.int64]
    valid_detects: NDArray[np.int32]
    image: NDArray[np.uint8]


@dataclass(slots=True)
class CanAngleData:
    frame: int
    yaw_angle_deg: float


@dataclass(slots=True)
class CanLeverData:
    frame: int
    lever_pressure: NDArray[np.float16]


class PcdDet:
    MAX_TOTAL_EDGE_SIZE: int = 50  # 崖オブジェクトの上限


class CreateEdgeDetectionResultMessage(SlotMessage[EdgeDetectionResult]):
    __slots__ = (
        "_edge_length",
        "_edge_length_num",
        "_edge_lines",
        "_edge_lines_num",
        "_edge_points",
        "_edge_points_num",
        "_frame",
        "_time",
    )

    def __init__(self) -> None:
        super().__init__()
        self._frame: SharedScalarSlotData[int] = SharedScalarSlotData(int, 0)
        self._time: SharedScalarSlotData[float] = SharedScalarSlotData(float, 0.0)
        self._edge_points_num: SharedScalarSlotData[int] = SharedScalarSlotData(int, 0)
        self._edge_points: SharedArraySlotData[np.float64] = SharedArraySlotData(
            (12 * PcdDet.MAX_TOTAL_EDGE_SIZE, 3), np.float64
        )
        self._edge_lines_num: SharedScalarSlotData[int] = SharedScalarSlotData(int, 0)
        self._edge_lines: SharedArraySlotData[np.int32] = SharedArraySlotData(
            (12 * PcdDet.MAX_TOTAL_EDGE_SIZE, 2), np.int32
        )
        self._edge_length_num: SharedScalarSlotData[int] = SharedScalarSlotData(int, 0)
        self._edge_length: SharedArraySlotData[np.int32] = SharedArraySlotData(
            (PcdData.SIZE,), np.int32
        )

    def write_slot(self, slot: int, value: EdgeDetectionResult) -> None:
        self._frame.write_slot(slot, int(value.frame))
        self._time.write_slot(slot, float(value.time))
        max_edge_points = 12 * PcdDet.MAX_TOTAL_EDGE_SIZE
        edge_points_num = min(value.edge_points.shape[0], max_edge_points)
        self._edge_points_num.write_slot(slot, int(edge_points_num))
        self._edge_points.write_slot_slice(slot, value.edge_points[:edge_points_num])

        max_edge_lines = 12 * PcdDet.MAX_TOTAL_EDGE_SIZE
        edge_lines_num = min(value.edge_lines.shape[0], max_edge_lines)
        self._edge_lines_num.write_slot(slot, int(edge_lines_num))
        self._edge_lines.write_slot_slice(slot, value.edge_lines[:edge_lines_num])

        max_edge_length = PcdData.SIZE
        edge_length_num = min(value.edge_length.shape[0], max_edge_length)
        self._edge_length_num.write_slot(slot, int(edge_length_num))
        self._edge_length.write_slot_slice(slot, value.edge_length[:edge_length_num])

    def borrow_slot(self, slot: int) -> EdgeDetectionResult:
        return EdgeDetectionResult(
            frame=int(self._frame.read_slot_value(slot)),
            time=float(self._time.read_slot_value(slot)),
            edge_points=self._edge_points.borrow_slot_slice(
                slot,
                (
                    slice(0, int(self._edge_points_num.read_slot_value(slot))),
                    slice(None),
                ),
            ),
            edge_lines=self._edge_lines.borrow_slot_slice(
                slot,
                (
                    slice(0, int(self._edge_lines_num.read_slot_value(slot))),
                    slice(None),
                ),
            ),
            edge_length=self._edge_length.borrow_slot_slice(
                slot, (slice(0, int(self._edge_length_num.read_slot_value(slot))),)
            ),
        )

    def _close(self) -> None:
        self._frame.close()
        self._time.close()
        self._edge_points_num.close()
        self._edge_lines_num.close()
        self._edge_length_num.close()
        self._edge_points.close()
        self._edge_lines.close()
        self._edge_length.close()
        self._close_slot_controller()


class AccumPointsDataMessage(SlotMessage[AccumPointsData]):
    __slots__ = ("_frame", "_num", "_point_cloud", "_size", "_time", "_yaw_angle_deg")

    def __init__(self, app_config: AppConfig) -> None:
        super().__init__()
        self._size: int = PcdData.SIZE * app_config.Lidar.count
        self._frame: SharedScalarSlotData[int] = SharedScalarSlotData(int, 0)
        self._time: SharedScalarSlotData[float] = SharedScalarSlotData(float, 0.0)
        self._yaw_angle_deg: SharedScalarSlotData[float] = SharedScalarSlotData(
            float, 0.0
        )
        self._num: SharedScalarSlotData[int] = SharedScalarSlotData(int, 0)
        self._point_cloud: SharedArraySlotData[np.float64] = SharedArraySlotData(
            (self._size, 3), np.float64
        )

    def write_slot(self, slot: int, value: AccumPointsData) -> None:
        self._frame.write_slot(slot, int(value.frame))
        self._time.write_slot(slot, float(value.time))
        self._yaw_angle_deg.write_slot(slot, float(value.yaw_angle_deg))
        num = min(value.point_cloud.shape[0], self._size)
        self._num.write_slot(slot, int(num))
        self._point_cloud.write_slot_slice(slot, value.point_cloud[:num])

    def borrow_slot(self, slot: int) -> AccumPointsData:
        num = int(self._num.read_slot_value(slot))
        return AccumPointsData(
            frame=int(self._frame.read_slot_value(slot)),
            time=float(self._time.read_slot_value(slot)),
            point_cloud=self._point_cloud.borrow_slot_slice(
                slot, (slice(0, num), slice(None))
            ),
            yaw_angle_deg=float(self._yaw_angle_deg.read_slot_value(slot)),
        )

    def _close(self) -> None:
        self._frame.close()
        self._time.close()
        self._yaw_angle_deg.close()
        self._num.close()
        self._point_cloud.close()
        self._close_slot_controller()


class CamDet:
    MAX_TOTAL_SIZE: int = 50  # 検出オブジェクトの上限


class CameraDetectionsDataMessage(SlotMessage[CameraDetectionsData]):
    __slots__ = (
        "_boxes",
        "_boxes_dim0",
        "_boxes_dim1",
        "_boxes_dim2",
        "_boxes_max_shape",
        "_classes",
        "_classes_dim0",
        "_classes_dim1",
        "_classes_max_shape",
        "_frame",
        "_frames",
        "_frames_dim0",
        "_frames_dim1",
        "_frames_dim2",
        "_frames_dim3",
        "_frames_max_shape",
        "_index",
        "_scores",
        "_scores_dim0",
        "_scores_dim1",
        "_scores_max_shape",
        "_time",
        "_valid_detects",
        "_valid_detects_dim0",
        "_valid_detects_max_shape",
    )

    def __init__(self, camera_count: int, resolution: ResolutionLike) -> None:
        super().__init__()
        self._index: SharedScalarSlotData[int] = SharedScalarSlotData(int, 0)
        self._frame: SharedScalarSlotData[int] = SharedScalarSlotData(int, 0)
        self._time: SharedScalarSlotData[float] = SharedScalarSlotData(float, 0.0)
        self._boxes_max_shape = (camera_count, CamDet.MAX_TOTAL_SIZE, 4)
        self._scores_max_shape = (camera_count, CamDet.MAX_TOTAL_SIZE)
        self._classes_max_shape = (camera_count, CamDet.MAX_TOTAL_SIZE)
        self._valid_detects_max_shape = (camera_count,)
        self._frames_max_shape = (camera_count, resolution.HEIGHT, resolution.WIDTH, 3)
        self._boxes_dim0: SharedScalarSlotData[int] = SharedScalarSlotData(
            int, camera_count
        )
        self._boxes_dim1: SharedScalarSlotData[int] = SharedScalarSlotData(
            int, CamDet.MAX_TOTAL_SIZE
        )
        self._boxes_dim2: SharedScalarSlotData[int] = SharedScalarSlotData(int, 4)
        self._scores_dim0: SharedScalarSlotData[int] = SharedScalarSlotData(
            int, camera_count
        )
        self._scores_dim1: SharedScalarSlotData[int] = SharedScalarSlotData(
            int, CamDet.MAX_TOTAL_SIZE
        )
        self._classes_dim0: SharedScalarSlotData[int] = SharedScalarSlotData(
            int, camera_count
        )
        self._classes_dim1: SharedScalarSlotData[int] = SharedScalarSlotData(
            int, CamDet.MAX_TOTAL_SIZE
        )
        self._valid_detects_dim0: SharedScalarSlotData[int] = SharedScalarSlotData(
            int, camera_count
        )
        self._frames_dim0: SharedScalarSlotData[int] = SharedScalarSlotData(
            int, camera_count
        )
        self._frames_dim1: SharedScalarSlotData[int] = SharedScalarSlotData(
            int, resolution.HEIGHT
        )
        self._frames_dim2: SharedScalarSlotData[int] = SharedScalarSlotData(
            int, resolution.WIDTH
        )
        self._frames_dim3: SharedScalarSlotData[int] = SharedScalarSlotData(int, 3)
        self._boxes: SharedArraySlotData[np.float32] = SharedArraySlotData(
            (camera_count, CamDet.MAX_TOTAL_SIZE, 4), np.float32
        )
        self._scores: SharedArraySlotData[np.float32] = SharedArraySlotData(
            (camera_count, CamDet.MAX_TOTAL_SIZE), np.float32
        )
        self._classes: SharedArraySlotData[np.int64] = SharedArraySlotData(
            (camera_count, CamDet.MAX_TOTAL_SIZE), np.int64
        )
        self._valid_detects: SharedArraySlotData[np.int32] = SharedArraySlotData(
            (camera_count,), np.int32
        )
        self._frames: SharedArraySlotData[np.uint8] = SharedArraySlotData(
            (camera_count, resolution.HEIGHT, resolution.WIDTH, 3), np.uint8
        )

    def write_slot(self, slot: int, value: CameraDetectionsData) -> None:
        self._index.write_slot(slot, int(value.index))
        self._frame.write_slot(slot, int(value.frame))
        self._time.write_slot(slot, float(value.time))
        boxes_dim0 = min(value.boxes.shape[0], self._boxes_max_shape[0])
        boxes_dim1 = min(value.boxes.shape[1], self._boxes_max_shape[1])
        boxes_dim2 = min(value.boxes.shape[2], self._boxes_max_shape[2])
        self._boxes_dim0.write_slot(slot, int(boxes_dim0))
        self._boxes_dim1.write_slot(slot, int(boxes_dim1))
        self._boxes_dim2.write_slot(slot, int(boxes_dim2))
        self._boxes.write_slot_slice(
            slot, value.boxes[:boxes_dim0, :boxes_dim1, :boxes_dim2]
        )
        scores_dim0 = min(value.scores.shape[0], self._scores_max_shape[0])
        scores_dim1 = min(value.scores.shape[1], self._scores_max_shape[1])
        self._scores_dim0.write_slot(slot, int(scores_dim0))
        self._scores_dim1.write_slot(slot, int(scores_dim1))
        self._scores.write_slot_slice(slot, value.scores[:scores_dim0, :scores_dim1])
        classes_dim0 = min(value.classes.shape[0], self._classes_max_shape[0])
        classes_dim1 = min(value.classes.shape[1], self._classes_max_shape[1])
        self._classes_dim0.write_slot(slot, int(classes_dim0))
        self._classes_dim1.write_slot(slot, int(classes_dim1))
        self._classes.write_slot_slice(
            slot, value.classes[:classes_dim0, :classes_dim1]
        )
        valid_detects_dim0 = min(
            value.valid_detects.shape[0], self._valid_detects_max_shape[0]
        )
        self._valid_detects_dim0.write_slot(slot, int(valid_detects_dim0))
        self._valid_detects.write_slot_slice(
            slot, value.valid_detects[:valid_detects_dim0]
        )
        frames_dim0 = min(value.image.shape[0], self._frames_max_shape[0])
        frames_dim1 = min(value.image.shape[1], self._frames_max_shape[1])
        frames_dim2 = min(value.image.shape[2], self._frames_max_shape[2])
        frames_dim3 = min(value.image.shape[3], self._frames_max_shape[3])
        self._frames_dim0.write_slot(slot, int(frames_dim0))
        self._frames_dim1.write_slot(slot, int(frames_dim1))
        self._frames_dim2.write_slot(slot, int(frames_dim2))
        self._frames_dim3.write_slot(slot, int(frames_dim3))
        self._frames.write_slot_slice(
            slot, value.image[:frames_dim0, :frames_dim1, :frames_dim2, :frames_dim3]
        )

    def borrow_slot(self, slot: int) -> CameraDetectionsData:
        boxes_dim0 = int(self._boxes_dim0.read_slot_value(slot))
        boxes_dim1 = int(self._boxes_dim1.read_slot_value(slot))
        boxes_dim2 = int(self._boxes_dim2.read_slot_value(slot))
        scores_dim0 = int(self._scores_dim0.read_slot_value(slot))
        scores_dim1 = int(self._scores_dim1.read_slot_value(slot))
        classes_dim0 = int(self._classes_dim0.read_slot_value(slot))
        classes_dim1 = int(self._classes_dim1.read_slot_value(slot))
        valid_detects_dim0 = int(self._valid_detects_dim0.read_slot_value(slot))
        frames_dim0 = int(self._frames_dim0.read_slot_value(slot))
        frames_dim1 = int(self._frames_dim1.read_slot_value(slot))
        frames_dim2 = int(self._frames_dim2.read_slot_value(slot))
        frames_dim3 = int(self._frames_dim3.read_slot_value(slot))
        return CameraDetectionsData(
            index=int(self._index.read_slot_value(slot)),
            frame=int(self._frame.read_slot_value(slot)),
            time=float(self._time.read_slot_value(slot)),
            boxes=self._boxes.borrow_slot_slice(
                slot, (slice(0, boxes_dim0), slice(0, boxes_dim1), slice(0, boxes_dim2))
            ),
            scores=self._scores.borrow_slot_slice(
                slot, (slice(0, scores_dim0), slice(0, scores_dim1))
            ),
            classes=self._classes.borrow_slot_slice(
                slot, (slice(0, classes_dim0), slice(0, classes_dim1))
            ),
            valid_detects=self._valid_detects.borrow_slot_slice(
                slot, (slice(0, valid_detects_dim0),)
            ),
            image=self._frames.borrow_slot_slice(
                slot,
                (
                    slice(0, frames_dim0),
                    slice(0, frames_dim1),
                    slice(0, frames_dim2),
                    slice(0, frames_dim3),
                ),
            ),
        )

    def _close(self) -> None:
        self._index.close()
        self._frame.close()
        self._time.close()
        self._boxes_dim0.close()
        self._boxes_dim1.close()
        self._boxes_dim2.close()
        self._scores_dim0.close()
        self._scores_dim1.close()
        self._classes_dim0.close()
        self._classes_dim1.close()
        self._valid_detects_dim0.close()
        self._frames_dim0.close()
        self._frames_dim1.close()
        self._frames_dim2.close()
        self._frames_dim3.close()
        self._boxes.close()
        self._scores.close()
        self._classes.close()
        self._valid_detects.close()
        self._frames.close()
        self._close_slot_controller()


class CanAngleMessage(SlotMessage[CanAngleData]):
    __slots__ = ("_frame", "_yaw_angle_deg")

    def __init__(self) -> None:
        super().__init__()
        self._frame: SharedScalarSlotData[int] = SharedScalarSlotData(int, 0)
        self._yaw_angle_deg: SharedScalarSlotData[float] = SharedScalarSlotData(
            float, 0.0
        )

    def write_slot(self, slot: int, value: CanAngleData) -> None:
        self._frame.write_slot(slot, int(value.frame))
        self._yaw_angle_deg.write_slot(slot, float(value.yaw_angle_deg))

    def borrow_slot(self, slot: int) -> CanAngleData:
        return CanAngleData(
            frame=int(self._frame.read_slot_value(slot)),
            yaw_angle_deg=float(self._yaw_angle_deg.read_slot_value(slot)),
        )

    def _close(self) -> None:
        self._frame.close()
        self._yaw_angle_deg.close()
        self._close_slot_controller()


class CanLeverMessage(SlotMessage[CanLeverData]):
    __slots__ = ("_frame", "_lever_pressure")

    def __init__(self) -> None:
        super().__init__()
        self._frame: SharedScalarSlotData[int] = SharedScalarSlotData(int, 0)
        self._lever_pressure: SharedArraySlotData[np.float16] = SharedArraySlotData(
            (4,), np.float16
        )
        self._lever_pressure.view()[:] = np.array(
            [np.nan, np.nan, np.nan, np.nan], dtype=np.float16
        )

    def write_slot(self, slot: int, value: CanLeverData) -> None:
        self._frame.write_slot(slot, int(value.frame))
        self._lever_pressure.write_slot(slot, value.lever_pressure)

    def borrow_slot(self, slot: int) -> CanLeverData:
        return CanLeverData(
            frame=int(self._frame.read_slot_value(slot)),
            lever_pressure=self._lever_pressure.borrow_slot(slot),
        )

    def _close(self) -> None:
        self._frame.close()
        self._lever_pressure.close()
        self._close_slot_controller()
