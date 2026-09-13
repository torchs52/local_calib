# 点群対応点抽出

import json
import pickle
import sys
from collections.abc import Iterable
from typing import Dict, Optional

import cv2
import numpy as np
from numpy.typing import NDArray
from supervision import Detections
from trackers import SORTTracker

from argus_synchro.calibration_mat_generator_modules.ctrl.calibration2d3d.track_main.interface_definition import (
    Tracking3dDataInterface,
    dtype_tracking3dIDbboxlog,
    dtype_tracking3dIDmetadata,
    tracking3d_dataclass,
)
from argus_synchro.calibration_mat_generator_modules.utils.debugdata_store import (
    debug_store,
)
from argus_synchro.calibration_mat_generator_modules.utils.GrayImageLUT import (
    GrayImageLUT,
)
from argus_synchro.calibration_mat_generator_modules.utils.NumpyMatrixLUT import (
    NumpyMatrixLUT,
)
from argus_synchro.calibration_mat_generator_modules.utils.utils3d import (
    set_xyz_range,
)
from argus_synchro.config.app_config_calibration import AppConfigCalibration
from argus_synchro.shared_app_config import SharedAppConfig


class bbox3d_mot_tracker_wrapper:
    def __init__(
        self,
        lost_track_buffer: int = 60,
        frame_rate: float = 10.0,
        track_activation_threshold: float = 0.25,
        minimum_consecutive_frames: int = 10,
        minimum_iou_threshold: float = 0.001,
        enable_bbox3d_overlap_merge: bool = False,
        bbox3d_overlap_merge_threshold: float = 0.7,
    ) -> None:
        # 追跡器の作成：パラメータは必要に応じて調整
        self.mot = SORTTracker(
            lost_track_buffer=lost_track_buffer,
            frame_rate=frame_rate,
            track_activation_threshold=track_activation_threshold,
            minimum_consecutive_frames=minimum_consecutive_frames,
            minimum_iou_threshold=minimum_iou_threshold,
        )
        self.enable_bbox3d_overlap_merge = enable_bbox3d_overlap_merge
        self.bbox3d_overlap_merge_threshold = bbox3d_overlap_merge_threshold
        self.last_bbox_multi_minmax: NDArray | None = None
        self.reset()

    def reset(self) -> None:
        self.mot.reset()
        self.last_bbox_multi_minmax = None

    @staticmethod
    def merge_overlapping_bbox3d_xy(
        bbox3d: NDArray,
        overlap_threshold: float,
    ) -> NDArray:
        bbox3d = np.asarray(bbox3d)
        if bbox3d.size == 0:
            return bbox3d.reshape(0, 6)
        if bbox3d.ndim != 2 or bbox3d.shape[1] != 6:
            raise ValueError(f"Expected shape (N,6), got {bbox3d.shape}")

        overlap_threshold = float(np.clip(overlap_threshold, 0.0, 1.0))
        xyxy = bbox3d[:, [0, 2, 1, 3]].astype(np.float64, copy=False)
        x1 = np.minimum(xyxy[:, 0], xyxy[:, 2])
        y1 = np.minimum(xyxy[:, 1], xyxy[:, 3])
        x2 = np.maximum(xyxy[:, 0], xyxy[:, 2])
        y2 = np.maximum(xyxy[:, 1], xyxy[:, 3])
        area = np.maximum(0.0, x2 - x1) * np.maximum(0.0, y2 - y1)

        used = np.zeros(len(bbox3d), dtype=bool)
        merged_bbox3d: list[list[float]] = []
        for start_ix in range(len(bbox3d)):
            if used[start_ix]:
                continue

            group_ixs = [start_ix]
            used[start_ix] = True
            cursor = 0
            while cursor < len(group_ixs):
                current_ix = group_ixs[cursor]
                cursor += 1

                inter_w = np.maximum(
                    0.0, np.minimum(x2[current_ix], x2) - np.maximum(x1[current_ix], x1)
                )
                inter_h = np.maximum(
                    0.0, np.minimum(y2[current_ix], y2) - np.maximum(y1[current_ix], y1)
                )
                inter_area = inter_w * inter_h
                min_area = np.minimum(area[current_ix], area)
                overlap_ratio = np.divide(
                    inter_area,
                    min_area,
                    out=np.zeros_like(inter_area),
                    where=min_area > 0,
                )
                linked_ixs = np.where(
                    (inter_area > 0.0) & (overlap_ratio >= overlap_threshold) & ~used
                )[0]
                used[linked_ixs] = True
                group_ixs.extend(linked_ixs.tolist())

            group_bbox3d = bbox3d[group_ixs]
            merged_bbox3d.append(
                [
                    float(np.min(group_bbox3d[:, 0])),
                    float(np.max(group_bbox3d[:, 1])),
                    float(np.min(group_bbox3d[:, 2])),
                    float(np.max(group_bbox3d[:, 3])),
                    float(np.min(group_bbox3d[:, 4])),
                    float(np.max(group_bbox3d[:, 5])),
                ]
            )

        return np.asarray(merged_bbox3d, dtype=bbox3d.dtype)

    @staticmethod
    def bbox3d_to_sv_detections(
        bbox3d: np.ndarray,
        *,
        default_class_id: int = 0,
        default_probability: float = 1,
        clip_xyxy: tuple[int, int] | None = None,  # (W, H)
        tracker_id: np.ndarray | None = None,  # 形状 (N,), dtype=int を想定
    ) -> Detections:
        """
        dbscan 3d bbox（ndarray: [xmin,xmax,ymin,ymax,zmin,zmax]）を supervision.Detections に変換。
        追加で tracker_id を受け取って Detections に格納可能。

        Parameters
        ----------
        bbox3d : np.ndarray
            shape=(N,6) で [xmin,xmax,ymin,ymax,zmin,zmax]
        default_class_id : Optional[int]
            全検出に付与するクラスID（人=0想定）。Noneなら class_id は付与しない。
        clip_xyxy : Optional[tuple[int,int]]
            画像幅高さ (W,H)。指定時は座標を [0..W-1/H-1] にクリップ。
        tracker_id : Optional[np.ndarray]
            形状 (N,) の整数配列。指定時は Detections.tracker_id に設定。

        Returns
        -------
        Detections
        """

        bbox3d = np.asarray(bbox3d)
        if bbox3d.size == 0:
            return Detections(
                xyxy=np.empty((0, 4), dtype=np.float32),
                confidence=np.empty((0,), dtype=np.float32),
                class_id=(
                    None if default_class_id is None else np.empty((0,), dtype=np.int32)
                ),
                tracker_id=(
                    None if tracker_id is None else np.empty((0,), dtype=np.int32)
                ),
            )

        if bbox3d.ndim != 2 or (bbox3d.shape[1] != 6 and bbox3d.size != 0):
            raise ValueError(f"Expected shape (N,6), got {bbox3d.shape}")

        bbox3d = bbox3d[:, :4]

        xyxy = bbox3d[:, [0, 2, 1, 3]].astype(
            np.float32, copy=False
        )  # xmin,xmax,ymin,ymax→xmin,ymin, xmax,ymax
        conf = np.ones(bbox3d.shape[0], dtype=np.float32) * default_probability

        # クリップ（任意）
        if clip_xyxy is not None:
            W, H = clip_xyxy
            xyxy[:, [0, 2]] = np.clip(xyxy[:, [0, 2]], 0, W - 1)
            xyxy[:, [1, 3]] = np.clip(xyxy[:, [1, 3]], 0, H - 1)

        # x1<=x2, y1<=y2 の整形
        x1, y1, x2, y2 = xyxy[:, 0], xyxy[:, 1], xyxy[:, 2], xyxy[:, 3]
        bad_x = x1 > x2
        bad_y = y1 > y2
        if np.any(bad_x):
            xyxy[bad_x, 0], xyxy[bad_x, 2] = x2[bad_x], x1[bad_x]
        if np.any(bad_y):
            xyxy[bad_y, 1], xyxy[bad_y, 3] = y2[bad_y], y1[bad_y]

        # class_id
        class_id = (
            None
            if default_class_id is None
            else np.full((xyxy.shape[0],), int(default_class_id), dtype=np.int32)
        )

        # tracker_id（任意）
        if tracker_id is not None:
            tracker_id = np.asarray(tracker_id)
            if tracker_id.shape[0] != xyxy.shape[0]:
                raise ValueError(
                    f"tracker_id length {tracker_id.shape[0]} must equal N={xyxy.shape[0]}"
                )
            tracker_id = tracker_id.astype(np.int32, copy=False)
        else:
            tracker_id = None

        return Detections(
            xyxy=xyxy, confidence=conf, class_id=class_id, tracker_id=tracker_id
        )

    @staticmethod
    def sv_detections_to_ndarray(
        dets: Detections,
        *,
        dtype: np.dtype = np.float64,
        class_filter: Iterable[int] | None = None,
        default_confidence: float = 1.0,
        include_tracker_id: bool = False,
        default_tracker_id: int = -1,
    ) -> NDArray:
        """
        supervision.Detections -> ndarray に変換。
        include_tracker_id=True の場合は 6列目に tracker_id を含めます。

        出力列:
        include_tracker_id=False: [x1, y1, x2, y2, prob]
        include_tracker_id=True : [x1, y1, x2, y2, prob, tracker_id]

        Parameters
        ----------
        dets : Detections
        dtype : np.dtype
        class_filter : Optional[Iterable[int]]
            指定した class_id のみ抽出（Noneで全件）。
        default_confidence : float
            dets.confidence が None/空の場合の既定値。
        include_tracker_id : bool
            True なら 6列目に tracker_id を付加。
        default_tracker_id : int
            tracker_id が None の場合に埋める既定値。

        Returns
        -------
        np.ndarray
            shape=(N, 5 or 6)
        """
        if dets is None:
            raise ValueError("dets is None")

        xyxy = np.asarray(getattr(dets, "xyxy", None))
        if xyxy is None:
            raise ValueError("Detections.xyxy is missing")
        if xyxy.size == 0:
            cols = 6 if include_tracker_id else 5
            return np.empty((0, cols), dtype=dtype)

        # クラスフィルタ
        mask = None
        if class_filter is not None and getattr(dets, "class_id", None) is not None:
            class_filter = set(int(c) for c in class_filter)
            class_id = np.asarray(dets.class_id)
            mask = np.array([cid in class_filter for cid in class_id], dtype=bool)
            xyxy = xyxy[mask]

        # confidence
        conf_arr = None
        if getattr(dets, "confidence", None) is not None:
            conf_raw = np.asarray(dets.confidence)
            conf_arr = conf_raw if mask is None else conf_raw[mask]
        if conf_arr is None:
            conf_arr = np.full((xyxy.shape[0],), float(default_confidence), dtype=dtype)

        xyxy = xyxy.astype(dtype, copy=False)
        conf_arr = conf_arr.astype(dtype, copy=False)

        if include_tracker_id:
            # tracker_id 取得
            tid_arr = None
            if getattr(dets, "tracker_id", None) is not None:
                tid_raw = np.asarray(dets.tracker_id)
                tid_arr = tid_raw if mask is None else tid_raw[mask]
            if tid_arr is None:
                tid_arr = np.full(
                    (xyxy.shape[0],), int(default_tracker_id), dtype=np.int32
                )
            else:
                tid_arr = tid_arr.astype(np.int32, copy=False)

            out = np.empty((xyxy.shape[0], 6), dtype=dtype)
            out[:, :4] = xyxy
            out[:, 4] = conf_arr
            # tracker_id は整数だが、全体の dtype に合わせる必要がある場合はキャスト
            out[:, 5] = tid_arr.astype(dtype, copy=False)
            return out
        out = np.empty((xyxy.shape[0], 5), dtype=dtype)
        out[:, :4] = xyxy
        out[:, 4] = conf_arr
        return out

    def update(self, bbox_multi_minmax: NDArray) -> NDArray:
        if self.enable_bbox3d_overlap_merge:
            bbox_multi_minmax = self.merge_overlapping_bbox3d_xy(
                bbox3d=bbox_multi_minmax,
                overlap_threshold=self.bbox3d_overlap_merge_threshold,
            )
        self.last_bbox_multi_minmax = bbox_multi_minmax.copy()
        bbox_sv = self.bbox3d_to_sv_detections(bbox3d=bbox_multi_minmax)
        tracks_detection = self.mot.update(bbox_sv)
        tracks = self.sv_detections_to_ndarray(
            tracks_detection, include_tracker_id=True
        )  # 返り値： xmin,ymin,xmax,ymax,conf,trackid
        # AppLogger.info("bbox2d_mot_tracker_wrapper",f"{tracks=}")
        return tracks

    def draw(
        self,
        frame_sortview: NDArray[np.uint8],
        tracks: NDArray,
        x_offset: float = 0,
        y_offset: float = 0,
    ):
        for res in tracks:
            tid = -1
            if len(res) == 5:
                x1, y1, x2, y2, prob = res
            elif len(res) == 6:
                x1, y1, x2, y2, prob, tid = res
            x1, y1, x2, y2 = map(
                int, [x1 + x_offset, y1 + y_offset, x2 + x_offset, y2 + y_offset]
            )
            cv2.rectangle(frame_sortview, (x1, y1), (x2, y2), (0, 200, 0), 2)
            cv2.putText(
                frame_sortview,
                f"ID {int(tid)}",
                (x1, y1 - 6),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 200, 0),
                2,
            )
        return frame_sortview


class proc3d_bboxtracker_recorder:
    def __init__(
        self, sac: SharedAppConfig, app_config_calib: AppConfigCalibration
    ) -> None:
        self.mot_tracker = bbox3d_mot_tracker_wrapper(
            lost_track_buffer=int(app_config_calib.calib2d3d.Proc3d.lost_track_buffer),
            frame_rate=app_config_calib.calib2d3d.Proc3d.tracking_frame_rate,
            track_activation_threshold=app_config_calib.calib2d3d.Proc3d.track_activation_threshold,
            minimum_consecutive_frames=int(
                app_config_calib.calib2d3d.Proc3d.minimum_consecutive_frames
            ),
            minimum_iou_threshold=app_config_calib.calib2d3d.Proc3d.minimum_iou_threshold,
            enable_bbox3d_overlap_merge=app_config_calib.calib2d3d.Proc3d.enable_bbox3d_overlap_merge,
            bbox3d_overlap_merge_threshold=app_config_calib.calib2d3d.Proc3d.bbox3d_overlap_merge_threshold,
        )
        self.reset()

        self.trackingID_data: dtype_tracking3dIDmetadata = {}
        self.trackingID_bboxlog: dtype_tracking3dIDbboxlog = {}

        self.app_config_calib = app_config_calib

        self.sac = sac
        camera_index = sac.read().CalibMode.cameraID

        self.evLUT3D = NumpyMatrixLUT(
            A_X=self.app_config_calib.calib2d3d.Proc3d.lid_valmat_coord_A_X[
                camera_index
            ],
            B_X=self.app_config_calib.calib2d3d.Proc3d.lid_valmat_coord_B_X[
                camera_index
            ],
            A_Y=self.app_config_calib.calib2d3d.Proc3d.lid_valmat_coord_A_Y[
                camera_index
            ],
            B_Y=self.app_config_calib.calib2d3d.Proc3d.lid_valmat_coord_B_Y[
                camera_index
            ],
            ARRAY_PATH=self.app_config_calib.calib2d3d.Proc3d.lid_valmat_path[
                camera_index
            ],
            DEFAULT_VALUE=self.app_config_calib.calib2d3d.Proc3d.lid_valmat_val_DEFAULT[
                camera_index
            ],
        )

        self.evLUT3D_workarea = GrayImageLUT(
            A_X=self.app_config_calib.calib2d3d.Proc3d.lid_workareadef_img_coord_A_X[
                camera_index
            ],
            B_X=self.app_config_calib.calib2d3d.Proc3d.lid_workareadef_img_coord_B_X[
                camera_index
            ],
            A_Y=self.app_config_calib.calib2d3d.Proc3d.lid_workareadef_img_coord_A_Y[
                camera_index
            ],
            B_Y=self.app_config_calib.calib2d3d.Proc3d.lid_workareadef_img_coord_B_Y[
                camera_index
            ],
            IMAGE_PATH=self.app_config_calib.calib2d3d.Proc3d.lid_workareadef_img_path[
                camera_index
            ],
            A_ETA=self.app_config_calib.calib2d3d.Proc3d.lid_workareadef_img_coord_A_ETA[
                camera_index
            ],
            B_ETA=self.app_config_calib.calib2d3d.Proc3d.lid_workareadef_img_coord_B_ETA[
                camera_index
            ],
            DEFAULT_VALUE=self.app_config_calib.calib2d3d.Proc3d.lid_workareadef_img_val_DEFAULT[
                camera_index
            ],
        )

        with open(
            app_config_calib.calib2d3d.CalcCorrespondence.optparam_initialvector,
            encoding="utf-8",
        ) as rtf:
            initial_vectors: list[dict[str, list[float]]] = json.load(rtf)

        self.camerapos: NDArray[np.float64] = np.array(
            initial_vectors[camera_index]["initial_tvec"]
        )

    def reset(self) -> None:
        self.mot_tracker.reset()
        self.last_tracker_result: NDArray | None = None
        self.lastframe_person_detected = False
        self.data_array_fbb_point_history: list[
            tuple[list[NDArray[np.float64]], int, list[float], NDArray[np.float64]]
        ] = []  # allpoints, timestamp, tracking_id, minmaxlist

    @staticmethod
    def _calc_L2norm(a: tuple[float, float], b: tuple[float, float]):
        return np.sqrt(np.sum((np.array(a) - np.array(b)) ** 2))

    def _update_trackinfo(self, frame_ix: int) -> None:
        if self.last_tracker_result is None:
            return

        for res in self.last_tracker_result:
            if len(res) == 6:
                x1, y1, x2, y2, prob, tracker_id = res
            else:
                return

            xc = (x1 + x2) / 2
            yc = (y1 + y2) / 2

            camera_to_bbcenter_dist = float(
                np.linalg.norm(
                    np.array([xc, yc])
                    - np.array([self.camerapos[0], self.camerapos[1]])
                )
            )
            is_workarea = 1 if self.evLUT3D_workarea.evaluate(xc, yc) > 0 else 0

            if tracker_id >= 0:
                if self.trackingID_data.get(tracker_id) is None:
                    self.trackingID_data[tracker_id] = tracking3d_dataclass(
                        accum_track_length=0,
                        dist_from_camera_max=camera_to_bbcenter_dist,
                        dist_from_camera_min=camera_to_bbcenter_dist,
                        final_xy=(xc, yc),
                        frame_ix_min=frame_ix,
                        frame_ix_max=frame_ix,
                        frame_ix_lastmove=frame_ix,
                        workarea_count=is_workarea,
                        frame_evval_max=self.evLUT3D.evaluate(xc, yc),
                        frame_evval_min=self.evLUT3D.evaluate(xc, yc),
                    )
                    self.trackingID_bboxlog[tracker_id] = []
                    self.trackingID_bboxlog[tracker_id].append(
                        (frame_ix, (x1, y1, x2, y2))
                    )
                else:
                    self.trackingID_data[tracker_id].dist_from_camera_max = min(
                        self.trackingID_data[tracker_id].dist_from_camera_max,
                        camera_to_bbcenter_dist,
                    )
                    self.trackingID_data[tracker_id].dist_from_camera_min = min(
                        self.trackingID_data[tracker_id].dist_from_camera_min,
                        camera_to_bbcenter_dist,
                    )

                    frame_movelen = self._calc_L2norm(
                        self.trackingID_data[tracker_id].final_xy, (xc, yc)
                    )
                    self.trackingID_data[tracker_id].accum_track_length += frame_movelen
                    self.trackingID_data[tracker_id].final_xy = (xc, yc)
                    self.trackingID_data[tracker_id].frame_ix_max = frame_ix
                    if frame_movelen > 1e-6:
                        self.trackingID_data[tracker_id].frame_ix_lastmove = frame_ix
                    self.trackingID_data[tracker_id].workarea_count += is_workarea
                    self.trackingID_bboxlog[tracker_id].append(
                        (frame_ix, (x1, y1, x2, y2))
                    )
                    self.trackingID_data[tracker_id].frame_evval_max = max(
                        self.trackingID_data[tracker_id].frame_evval_max,
                        self.evLUT3D.evaluate(xc, yc),
                    )
                    self.trackingID_data[tracker_id].frame_evval_min = min(
                        self.trackingID_data[tracker_id].frame_evval_min,
                        self.evLUT3D.evaluate(xc, yc),
                    )

    def print_trackinfo(self, file=sys.stdout) -> None:
        for k, d in self.trackingID_data.items():
            print(
                f"key {k} : {d.__dict__}, points:{self.trackingID_bboxlog[k]}",
                file=file,
            )

    def get_rawresults(
        self,
    ) -> Tracking3dDataInterface:
        return Tracking3dDataInterface(
            trackingIDmetadata=self.trackingID_data,
            trackingIDbboxlog=self.trackingID_bboxlog,
        )

    def save_results_debug(self, openflag: str = "wb") -> None:
        debug_store("tracker3d_db", self.trackingID_data)
        debug_store("tracker3d_ptlist", self.trackingID_bboxlog)

    def update(
        self, bbox_multi_minmax: NDArray, frame_ix: int, pcdframe: NDArray[np.float64]
    ):
        # SORT法追跡
        self.last_tracker_result = self.mot_tracker.update(
            bbox_multi_minmax=bbox_multi_minmax
        )
        # 追跡によるメタデータ更新
        self._update_trackinfo(frame_ix=frame_ix)

        self.data_array_fbb_point_history.append(  # 1フレームごとの点群・bbox記録
            (
                [
                    set_xyz_range(pcdframe, (x1L, x1H), (y1L, y1H), (z1L, z1H)).copy()
                    for (
                        x1L,
                        x1H,
                        y1L,
                        y1H,
                        z1L,
                        z1H,
                    ) in self.mot_tracker.last_bbox_multi_minmax
                ],
                frame_ix,
                [
                    track_id for track_id in self.last_tracker_result[:, 5]
                ],  # リスト要素番号からトラッキングIDを得るリスト
                self.mot_tracker.last_bbox_multi_minmax.copy(),
            )
        )

        return self.last_tracker_result

    def is_person_detected(self) -> bool:
        return self.lastframe_person_detected

    def draw_mot(self, frame_sortview: NDArray[np.uint8]) -> NDArray[np.uint8]:
        if self.last_tracker_result is None:
            return frame_sortview
        return self.mot_tracker.draw(
            frame_sortview=frame_sortview, tracks=self.last_tracker_result
        )

    def get_data_array_fbb_point_history(
        self,
    ) -> list[
        tuple[list[NDArray[np.float64]], int, list[float], NDArray[np.float64]]
    ]:  # allpoints, timestamp, tracking_id, minmaxlist:
        return self.data_array_fbb_point_history
