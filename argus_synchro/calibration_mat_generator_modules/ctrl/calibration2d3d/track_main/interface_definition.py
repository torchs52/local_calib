from dataclasses import dataclass

import numpy as np
from numpy.typing import NDArray

# ===================  3D共通インターフェース  ===================
dtype3DArea_xxyyzz = tuple[
    tuple[float, float], tuple[float, float], tuple[float, float]
]

dtypeTupleBBox = tuple[NDArray[np.float64], NDArray[np.int32], NDArray[np.float64]]


@dataclass
class dtypeBBox3D:
    multi_points: NDArray[np.float64]  # (N,8)?
    multi_lines: NDArray[np.int32]  # (N,12)?
    multi_minmax: NDArray[np.float64]  # (N,6)?


def tupleBBox_to_dtypeBBox(
    integrated_bboxinfo: dtypeTupleBBox,
) -> dtypeBBox3D:
    return dtypeBBox3D(
        multi_points=integrated_bboxinfo[0],
        multi_lines=integrated_bboxinfo[1],
        multi_minmax=integrated_bboxinfo[2],
    )


dtypePreprocess3d = tuple[dtypeBBox3D, NDArray[np.float64]]  # bboxと点群

# ===================  2D / 3D追跡用インターフェース　（追跡情報、追跡履歴データ）  ===================

dtype_possible2dIDdata = dict[
    int, tuple[float, int, int, tuple[float, float]]
]  # 追跡bbox番号キー: 移動距離、タイムスタンプ最小、タイムスタンプ最大、xy最大
dtype_possible3dIDdata = dict[
    int, tuple[float, int, int, float]
]  # 追跡bbox番号キー: 移動距離、タイムスタンプ最小、タイムスタンプ最大、カメラ直下からの最小距離


class tracking2d_dataclass:  # TODO: 他にも追加する可能性あり
    def __init__(
        self,
        accum_track_length: float,
        final_xy: tuple[float, float],
        xymin: tuple[float, float],
        xymax: tuple[float, float],
        frame_ix_min: int,
        frame_ix_max: int,
        frame_ix_lastmove: int,
        frame_evval_min: float,
        frame_evval_max: float,
        workarea_count: int,
        is_alive: bool = False,  # 追跡後フィルタ処理で生き残ったか否か
        last_filter_applied_frame: int = -1,  # 追跡後フィルタ処理が最後に行われたフレームインデックス
        is_tracking_target: bool = False,  # 校正作業者判定にて作業者と判定された) -> None
    ):
        self.accum_track_length = accum_track_length
        self.final_xy = final_xy
        self.xymin = xymin
        self.xymax = xymax
        self.frame_ix_min = frame_ix_min
        self.frame_ix_max = frame_ix_max
        self.frame_ix_lastmove = frame_ix_lastmove
        self.frame_evval_min = frame_evval_min
        self.frame_evval_max = frame_evval_max
        self.workarea_count = workarea_count
        self.is_alive = is_alive
        self.last_filter_applied_frame = last_filter_applied_frame
        self.is_tracking_target = is_tracking_target


dtype_singleIDbbox2dlog = list[
    tuple[int, tuple[float, float, float, float]]
]  # (frame_ix, (x1, y1, x2, y2))
dtype_tracking2dIDmetadata = dict[
    int, tracking2d_dataclass
]  # 追跡IDに対する統計データ等々　主に追跡対象選定に使用
dtype_tracking2dIDbboxlog = dict[int, dtype_singleIDbbox2dlog]


class Tracking2dDataInterface:
    def __init__(
        self,
        trackingIDmetadata=dtype_tracking2dIDmetadata(),
        trackingIDbboxlog=dtype_tracking2dIDbboxlog(),
    ) -> None:
        self.trackingIDmetadata: dtype_tracking2dIDmetadata = trackingIDmetadata
        self.trackingIDbboxlog: dtype_tracking2dIDbboxlog = trackingIDbboxlog


class tracking3d_dataclass:  # TODO: 他にも追加する可能性あり
    def __init__(
        self,
        accum_track_length: float,
        final_xy: tuple[float, float],
        dist_from_camera_min: float,
        dist_from_camera_max: float,
        frame_ix_min: int,
        frame_ix_max: int,
        frame_ix_lastmove: int,
        frame_evval_min: float,
        frame_evval_max: float,
        workarea_count: float,
        is_alive: bool = False,  # 追跡後フィルタ処理で生き残ったか否か
        last_filter_applied_frame: int = -1,  # 追跡後フィルタ処理が最後に行われたフレームインデックス
        is_tracking_target: bool = False,  # 校正作業者判定にて作業者と判定された
    ) -> None:
        self.accum_track_length = accum_track_length
        self.final_xy = final_xy
        self.dist_from_camera_min = dist_from_camera_min
        self.dist_from_camera_max = dist_from_camera_max
        self.frame_ix_min = frame_ix_min
        self.frame_ix_max = frame_ix_max
        self.frame_ix_lastmove = frame_ix_lastmove
        self.frame_evval_min = frame_evval_min
        self.frame_evval_max = frame_evval_max
        self.workarea_count = workarea_count
        self.is_alive = is_alive
        self.last_filter_applied_frame = last_filter_applied_frame
        self.is_tracking_target = is_tracking_target


dtype_singleIDbbox3dlog = list[
    tuple[int, tuple[float, float, float, float]]
]  # (frame_ix, (x1, y1, x2, y2))
dtype_tracking3dIDmetadata = dict[
    int, tracking3d_dataclass
]  # 追跡IDに対する統計データ等々　主に追跡対象選定に使用
dtype_tracking3dIDbboxlog = dict[int, dtype_singleIDbbox3dlog]


class Tracking3dDataInterface:
    def __init__(
        self,
        trackingIDmetadata=dtype_tracking3dIDmetadata(),
        trackingIDbboxlog=dtype_tracking3dIDbboxlog(),
    ) -> None:
        self.trackingIDmetadata: dtype_tracking3dIDmetadata = trackingIDmetadata
        self.trackingIDbboxlog: dtype_tracking3dIDbboxlog = trackingIDbboxlog
