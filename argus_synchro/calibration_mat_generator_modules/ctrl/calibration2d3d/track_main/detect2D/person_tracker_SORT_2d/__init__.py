import sys
from collections.abc import Iterable
from typing import Optional

import cv2
import numpy as np
from numpy.typing import NDArray
from supervision import Detections
from trackers import SORTTracker

from argus_synchro.calibration_mat_generator_modules.ctrl.calibration2d3d.track_main.interface_definition import (
    dtype_tracking2dIDbboxlog,
    dtype_tracking2dIDmetadata,
    tracking2d_dataclass,
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
from argus_synchro.config.app_config_calibration import AppConfigCalibration


class bbox2d_mot_tracker_wrapper:
    def __init__(
        self,
        lost_track_buffer: int = 30,
        frame_rate: float = 10.0,
        track_activation_threshold: float = 0.25,
        minimum_consecutive_frames: int = 3,
        minimum_iou_threshold: float = 0.3,
    ) -> None:
        # 追跡器の作成：パラメータは必要に応じて調整
        self.mot = SORTTracker(
            lost_track_buffer=lost_track_buffer,
            frame_rate=frame_rate,
            track_activation_threshold=track_activation_threshold,
            minimum_consecutive_frames=minimum_consecutive_frames,
            minimum_iou_threshold=minimum_iou_threshold,
        )
        self.reset()

    def reset(self) -> None:
        self.mot.reset()

    @staticmethod
    def yolo_list_to_ndarray(
        yoloresult_whole: list[NDArray], image_h: int, image_w: int
    ):
        yolo_length = len(yoloresult_whole[0].reshape(-1, 4))
        person_found = False
        yoloresult_for_mot_list = []

        for result_ix in range(yolo_length):
            # AppLogger.info("yolo_list_to_ndarray", yoloresult_whole)
            coor = yoloresult_whole[0].reshape(-1, 4)[result_ix]
            prob = yoloresult_whole[1].reshape(-1)[result_ix]
            cls_id = yoloresult_whole[2].reshape(-1)[result_ix]
            if prob <= 0:
                continue
            # AppLogger.info("yolo_list_to_ndarray", f"{coor=},{prob=},{cls_id=},{result_ix}/{yolo_length}")

            # メインアプリ core - utils.py - draw_bbox 関数より編集
            bbox_ymin = coor[0] * image_h
            bbox_ymax = coor[2] * image_h
            bbox_xmin = coor[1] * image_w
            bbox_xmax = coor[3] * image_w

            if prob > 0 and cls_id == 0:
                yoloresult_for_mot_list.append(
                    [bbox_xmin, bbox_ymin, bbox_xmax, bbox_ymax, prob]
                )
                person_found = True

        if len(yoloresult_for_mot_list) == 0:
            dets = np.empty((0, 5), dtype=float)
        else:
            dets = np.array(yoloresult_for_mot_list)

        return dets, person_found

    @staticmethod
    def yolo_ndarray_to_sv_detections(
        yolo_nd: np.ndarray,
        *,
        default_class_id: int | None = 0,
        clip_xyxy: tuple[int, int] | None = None,  # (W, H)
        tracker_id: np.ndarray | None = None,  # 形状 (N,), dtype=int を想定
    ) -> Detections:
        """
        YOLO検出（ndarray: [x1,y1,x2,y2,prob]）を supervision.Detections に変換。
        追加で tracker_id を受け取って Detections に格納可能。

        Parameters
        ----------
        yolo_nd : np.ndarray
            shape=(N,5) で [x1,y1,x2,y2,prob]
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
        if yolo_nd is None:
            raise ValueError("yolo_nd is None")
        yolo_nd = np.asarray(yolo_nd)
        if yolo_nd.ndim != 2 or (yolo_nd.shape[1] != 5 and yolo_nd.size != 0):
            raise ValueError(f"Expected shape (N,5), got {yolo_nd.shape}")

        if yolo_nd.size == 0:
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

        xyxy = yolo_nd[:, :4].astype(np.float32, copy=False)
        conf = yolo_nd[:, 4].astype(np.float32, copy=False)

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

    def update(
        self, yoloresult_whole: list[NDArray], image_h: int, image_w: int, frame_ix: int
    ) -> tuple[NDArray, bool]:
        yolo_result_ndarray, person_detect = self.yolo_list_to_ndarray(
            yoloresult_whole=yoloresult_whole, image_h=image_h, image_w=image_w
        )
        tracks_detection = self.mot.update(
            self.yolo_ndarray_to_sv_detections(yolo_result_ndarray)
        )
        tracks = self.sv_detections_to_ndarray(
            tracks_detection, include_tracker_id=True
        )
        # AppLogger.info("bbox2d_mot_tracker_wrapper",f"{tracks=}")
        return tracks, person_detect

    def draw(self, frame_sortview: NDArray[np.uint8], tracks: NDArray):
        for res in tracks:
            tid = -1
            if len(res) == 5:
                x1, y1, x2, y2, prob = res
            elif len(res) == 6:
                x1, y1, x2, y2, prob, tid = res
            x_offset = 0
            y_offset = 0
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


class proc2d_bboxtracker_recorder:
    def __init__(
        self,
        app_config_calib: AppConfigCalibration,
        image_size_hw: tuple[int, int],
        onnx_model_path,
        camera_index: int,
    ) -> None:
        self.mot_tracker = bbox2d_mot_tracker_wrapper(
            lost_track_buffer=int(app_config_calib.calib2d3d.Proc2d.lost_track_buffer),
            frame_rate=app_config_calib.calib2d3d.Proc2d.tracking_frame_rate,
            track_activation_threshold=app_config_calib.calib2d3d.Proc2d.track_activation_threshold,
            minimum_consecutive_frames=int(
                app_config_calib.calib2d3d.Proc2d.minimum_consecutive_frames
            ),
            minimum_iou_threshold=app_config_calib.calib2d3d.Proc2d.minimum_iou_threshold,
        )
        self.app_config_calib = app_config_calib
        self.reset()

        self.image_size_hw = image_size_hw

        self.trackingID_data: dtype_tracking2dIDmetadata = {}
        self.trackingID_bboxlog: dtype_tracking2dIDbboxlog = {}

        self.evLUT2D = NumpyMatrixLUT(
            A_X=self.app_config_calib.calib2d3d.Proc2d.cam_valmat_coord_A_X[
                camera_index
            ],
            B_X=self.app_config_calib.calib2d3d.Proc2d.cam_valmat_coord_B_X[
                camera_index
            ],
            A_Y=self.app_config_calib.calib2d3d.Proc2d.cam_valmat_coord_A_Y[
                camera_index
            ],
            B_Y=self.app_config_calib.calib2d3d.Proc2d.cam_valmat_coord_B_Y[
                camera_index
            ],
            ARRAY_PATH=self.app_config_calib.calib2d3d.Proc2d.cam_valmat_path[
                camera_index
            ],
            DEFAULT_VALUE=self.app_config_calib.calib2d3d.Proc2d.cam_valmat_val_DEFAULT[
                camera_index
            ],
        )

        self.evLUT2D_workarea = GrayImageLUT(
            A_X=self.app_config_calib.calib2d3d.Proc2d.cam_workareadef_img_coord_A_X[
                camera_index
            ],
            B_X=self.app_config_calib.calib2d3d.Proc2d.cam_workareadef_img_coord_B_X[
                camera_index
            ],
            A_Y=self.app_config_calib.calib2d3d.Proc2d.cam_workareadef_img_coord_A_Y[
                camera_index
            ],
            B_Y=self.app_config_calib.calib2d3d.Proc2d.cam_workareadef_img_coord_B_Y[
                camera_index
            ],
            IMAGE_PATH=self.app_config_calib.calib2d3d.Proc2d.cam_workareadef_img_path[
                camera_index
            ],
            A_ETA=self.app_config_calib.calib2d3d.Proc2d.cam_workareadef_img_coord_A_ETA[
                camera_index
            ],
            B_ETA=self.app_config_calib.calib2d3d.Proc2d.cam_workareadef_img_coord_B_ETA[
                camera_index
            ],
            DEFAULT_VALUE=self.app_config_calib.calib2d3d.Proc2d.cam_workareadef_img_val_DEFAULT[
                camera_index
            ],
        )

    def reset(self) -> None:
        self.mot_tracker.reset()

        self.last_tracker_result: NDArray | None = None

        self.lastframe_person_detected = False

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
            xymin = min(x1, x2), min(y1, y2)
            xymax = max(x1, x2), max(y1, y2)
            is_workarea = 1 if self.evLUT2D_workarea.evaluate(xc, yc) else 0

            if tracker_id >= 0:
                if self.trackingID_data.get(tracker_id) is None:
                    self.trackingID_data[tracker_id] = tracking2d_dataclass(
                        accum_track_length=0,
                        xymax=xymax,
                        xymin=xymin,
                        final_xy=(xc, yc),
                        frame_ix_min=frame_ix,
                        frame_ix_max=frame_ix,
                        frame_ix_lastmove=frame_ix,
                        workarea_count=is_workarea,
                        frame_evval_max=self.evLUT2D.evaluate(xc, yc),
                        frame_evval_min=self.evLUT2D.evaluate(xc, yc),
                    )
                    self.trackingID_bboxlog[tracker_id] = []
                    self.trackingID_bboxlog[tracker_id].append(
                        (frame_ix, (x1, y1, x2, y2))
                    )
                else:
                    self.trackingID_data[tracker_id].xymin = (
                        min(self.trackingID_data[tracker_id].xymin[0], xymin[0]),
                        min(self.trackingID_data[tracker_id].xymin[1], xymin[1]),
                    )
                    self.trackingID_data[tracker_id].xymax = (
                        max(self.trackingID_data[tracker_id].xymax[0], xymax[0]),
                        max(self.trackingID_data[tracker_id].xymax[1], xymax[1]),
                    )

                    frame_movelen = self._calc_L2norm(
                        self.trackingID_data[tracker_id].final_xy, (xc, yc)
                    )
                    self.trackingID_data[tracker_id].accum_track_length += frame_movelen
                    self.trackingID_data[tracker_id].final_xy = (xc, yc)
                    self.trackingID_data[tracker_id].frame_ix_max = frame_ix
                    if frame_movelen > 1e-6:
                        self.trackingID_data[tracker_id].frame_ix_lastmove = frame_ix
                    self.trackingID_data[tracker_id].frame_evval_max = max(
                        self.trackingID_data[tracker_id].frame_evval_max,
                        self.evLUT2D.evaluate(xc, yc),
                    )
                    self.trackingID_data[tracker_id].frame_evval_min = min(
                        self.trackingID_data[tracker_id].frame_evval_min,
                        self.evLUT2D.evaluate(xc, yc),
                    )
                    self.trackingID_data[tracker_id].workarea_count += is_workarea
                    self.trackingID_bboxlog[tracker_id].append(
                        (frame_ix, (x1, y1, x2, y2))
                    )

    def print_trackinfo(self, file=sys.stdout) -> None:
        for k, d in self.trackingID_data.items():
            print(
                f"key {k} : {d.__dict__}, points:{self.trackingID_bboxlog[k]}",
                file=file,
            )

    def get_rawresults(
        self,
    ) -> tuple[dtype_tracking2dIDmetadata, dtype_tracking2dIDbboxlog]:
        return self.trackingID_data, self.trackingID_bboxlog

    def save_results_debug(self, openflag: str = "wb") -> None:
        debug_store("tracker2d_db", self.trackingID_data)
        debug_store("tracker2d_ptlist", self.trackingID_bboxlog)

    def update(self, yoloresult_whole: list[NDArray], frame_ix: int):
        self.last_tracker_result, self.lastframe_person_detected = (
            self.mot_tracker.update(
                yoloresult_whole=yoloresult_whole,
                image_w=self.image_size_hw[1],
                image_h=self.image_size_hw[0],
                frame_ix=frame_ix,
            )
        )
        self._update_trackinfo(frame_ix=frame_ix)

    def is_person_detected(self) -> bool:
        return self.lastframe_person_detected

    @staticmethod
    def draw_bboxes(frame, YOLOsingleresult_conv):
        # yolo.draw_bboxesで呼び出しているmedia.draw_bboxesより変更
        # YOLOsingleresult_convの中身： self.convert_bbox_imagecoordinate_info(yoloresult_whole, frame_ix, image_h, image_w)

        x1, x2, y1, y2, cls_id, _ = np.array(YOLOsingleresult_conv, dtype=np.int32)
        prob = YOLOsingleresult_conv[5]

        cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 255, 0), 2)

        # Draw text box
        bbox_text = f"person: {prob:.1%}"
        t_size = cv2.getTextSize(bbox_text, 0, 1, 1)[0]
        cv2.rectangle(
            frame,
            (x1, y1),
            (x1 + t_size[0], y1 - t_size[1]),
            (255, 255, 0),
            -1,
        )
        # Draw text
        cv2.putText(
            frame,
            bbox_text,
            (x1, y1),
            cv2.FONT_HERSHEY_SIMPLEX,
            1,
            (255, 255, 255),
            1,
            lineType=cv2.LINE_AA,
        )

        return frame

    @staticmethod
    def draw_detection(
        yoloresult_whole: list[NDArray],
        frame: NDArray[np.uint8],
        image_size_hw: tuple[int, int],
        file=sys.stdout,
    ):
        if yoloresult_whole is None:
            return None

        for result_ix in range(len(yoloresult_whole[0])):
            coor = yoloresult_whole[0][result_ix]
            prob = yoloresult_whole[1][result_ix]
            # cls_id = yoloresult_whole[2][result_ix]

            # メインアプリ core - utils.py - draw_bbox 関数より編集
            bbox_ymin = coor[0] * image_size_hw[0]
            bbox_ymax = coor[2] * image_size_hw[0]
            bbox_xmin = coor[1] * image_size_hw[1]
            bbox_xmax = coor[3] * image_size_hw[1]

            if prob > 0:
                # print(
                #    f"{result_ix=},{prob=},{cls_id=},{bbox_ymin=},{bbox_ymax=},{bbox_xmin=},{bbox_xmax=},",
                #    file=file,
                # )
                proc2d_bboxtracker_recorder.draw_bboxes(
                    frame=frame,
                    YOLOsingleresult_conv=np.array(
                        [
                            float(bbox_xmin),
                            float(bbox_xmax),
                            float(bbox_ymin),
                            float(bbox_ymax),
                            0,
                            float(prob),
                        ]
                    ),
                )

        return frame

    def draw_mot(self, frame_sortview: NDArray[np.uint8]) -> NDArray[np.uint8]:
        if self.last_tracker_result is None:
            return frame_sortview
        return self.mot_tracker.draw(
            frame_sortview=frame_sortview, tracks=self.last_tracker_result
        )
