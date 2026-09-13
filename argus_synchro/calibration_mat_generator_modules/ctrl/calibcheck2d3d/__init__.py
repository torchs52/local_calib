import copy
import pickle
import sys
from collections.abc import Callable
from os import path
from pathlib import Path
from time import sleep
from typing import TypedDict

import cv2
import numpy as np
from numpy.typing import NDArray

from argus_synchro.calibration_mat_generator_modules.ctrl.calibcheck2d3d import (
    calibcheck_detection_2d3d,
)

# デバッグ用パラメータ設定
from argus_synchro.calibration_mat_generator_modules.ctrl.calibcheck2d3d.debuginfo_and_functions import (
    conbine3d3d,
    conv_intarr,
    draw_multibbox,
    internal_make_BB,
    read_rtvec,
)
from argus_synchro.calibration_mat_generator_modules.ctrl.calibcheck2d3d.SceneDesc import (
    Scene,
)
from argus_synchro.calibration_mat_generator_modules.ctrl.calibcheck2d3d.YOLOadapter import (
    YOLODamoBatchAdapter,
)
from argus_synchro.calibration_mat_generator_modules.ctrl.calibration2d3d.track_main.detect2D.person_tracker_SORT_2d import (
    bbox2d_mot_tracker_wrapper,
)
from argus_synchro.calibration_mat_generator_modules.ctrl.calibration2d3d.track_main.detect3D.person_tracker_SORT_3d import (
    bbox3d_mot_tracker_wrapper,
)
from argus_synchro.calibration_mat_generator_modules.ctrl.calibration2d3d.track_main.interface_definition import (
    Tracking2dDataInterface,
    Tracking3dDataInterface,
    dtype_tracking2dIDbboxlog,
    dtype_tracking2dIDmetadata,
    dtype_tracking3dIDbboxlog,
    dtype_tracking3dIDmetadata,
    tracking2d_dataclass,
    tracking3d_dataclass,
)

# 型定義のみ
from argus_synchro.calibration_mat_generator_modules.ctrl.data_capture import (
    data_capture,
)
from argus_synchro.calibration_mat_generator_modules.ctrl.data_capture.datacapture_local import (
    datacapture_class,
)
from argus_synchro.calibration_mat_generator_modules.ctrl.data_capture.datacapture_local import (
    tools3Dcapture as t3dc,
)
from argus_synchro.calibration_mat_generator_modules.facade import (
    CalibrationCommonStatus,
    CalibrationUIGodot,
)
from argus_synchro.calibration_mat_generator_modules.utils import utils3d
from argus_synchro.calibration_mat_generator_modules.utils.debugdata_store import (
    debug_config,
    debug_force_snapshot,
    debug_store,
)
from argus_synchro.calibration_mat_generator_modules.utils.filter_static_objects import (
    filter_static_objects,
)
from argus_synchro.calibration_mat_generator_modules.utils.GrayImageLUT import (
    GrayImageLUT,
)
from argus_synchro.calibration_mat_generator_modules.utils.NumpyMatrixLUT import (
    NumpyMatrixLUT,
)

# ARGUSシステム制御関連
from argus_synchro.common import paths
from argus_synchro.common.app_logger import AppLogger, AppLoggerFactory
from argus_synchro.config.app_config import SceneDescriptionConf
from argus_synchro.config.app_config_calibration import (
    AppConfigCalibration,
    CalibCheck2d3dConf,
    DataCaptureConf,
)
from argus_synchro.device.camera.helper import CameraHelper
from argus_synchro.diagnosis.calibcheck2d3d_result_diagnosis import (
    CameraCalibCheckStatusDiagnosis,
)
from argus_synchro.diagnosis.error_diagnosis import ResultDiagnosis
from argus_synchro.message.calib_fifo_message import FIFOData
from argus_synchro.provider.image import Mcde7000UndistortImageProvider
from argus_synchro.shared_app_config import SharedAppConfig
from argus_synchro.shared_errors import SharedErrors, StateErrorDIndex
from argus_synchro.shared_excepts import SharedExcepts

_logger: AppLogger = AppLoggerFactory.from_name("calibcheck2d3d")

VIRTUAL_BBOX_XY_OFFSETS: tuple[tuple[float, float], ...] = (
    (0.0, 0.0),
    (-3.0, 0.0),
    (3.0, 0.0),
    (0.0, -3.0),
    (0.0, 3.0),
)
VIRTUAL_BBOX_CANDIDATE_NAMES: tuple[str, ...] = (
    "real",
    "x_minus",
    "x_plus",
    "y_minus",
    "y_plus",
)
BBOX_SHRINK_FACTOR: float = 1.0  # バウンディングボックスの縮小率。1.0で縮小なし、0.9で10%縮小、0.8で20%縮小など。
BBOX_CENTER_DIFF_RATIO_THRESHOLD: float = 0.7  # バウンディングボックスの中心点の差の割合の閾値。0.2で20%以内、0.1で10%以内など。


class VirtualBBoxDebugCounts(TypedDict):
    comparison_count: int
    virtual_bbox_win_count: int
    selected_candidate_counts: dict[str, int]


class EvaluationMetricDebug(TypedDict):
    numerator: int
    denominator: int
    strict_hit_rate: float
    legacy_numerator: float
    legacy_denominator: int
    legacy_like_score: float
    selected_score: float
    use_legacy_like_metric: bool


class TrackProximityWarning(TypedDict):
    track_id_a: int
    track_id_b: int
    common_frame_count: int
    close_frame_count: int
    close_frame_ratio: float
    min_center_distance_m: float
    closest_frame_ix: int
    close_frame_samples: list[int]


def log_register(app_logger_factory: AppLoggerFactory) -> None:
    app_logger_factory.append_logger(_logger)
    calibcheck_detection_2d3d.log_register(app_logger_factory)


class Scene_CalibCheck2d3d(Scene):
    def __init__(
        self,
        scene_conf: SceneDescriptionConf,
        app_config_calib: AppConfigCalibration,
        file_io_error_reporter: Callable[[str, str, Exception], None] | None = None,
    ) -> None:
        super().__init__(scene_conf)
        self.app_config_calib: AppConfigCalibration = app_config_calib
        self._image_width = app_config_calib.calibCheck2d3d.image_w
        self._image_height = app_config_calib.calibCheck2d3d.image_h

        # 魚眼カメラ歪み補正データの読み込み
        camera_intrinsics_path = (
            self.app_config_calib.calibCheck2d3d.camera_intrinsics_path
        )
        try:
            (
                _,
                _,
                _,
                _,
                self._ncm1,
            ) = CameraHelper.read_fisheye_param(camera_intrinsics_path)
        except (OSError, UnicodeError, ValueError, KeyError, TypeError) as error:
            if file_io_error_reporter is not None:
                file_io_error_reporter(
                    camera_intrinsics_path,
                    "read calibcheck2d3d camera intrinsics JSON",
                    error,
                )
            raise

    @classmethod
    def create_for_evaluation(
        cls,
        scene_conf: SceneDescriptionConf,
        camera_intrinsics: NDArray[np.float32],
        image_width: int,
        image_height: int,
    ) -> "Scene_CalibCheck2d3d":
        instance = cls.__new__(cls)
        Scene.__init__(instance, scene_conf)
        instance._ncm1 = camera_intrinsics
        instance._image_width = image_width
        instance._image_height = image_height
        return instance

    def integrate2d3d_calibcheck(
        self,
        rvec: NDArray[np.float64],
        tvec: NDArray[np.float64],
        boxpoints: NDArray[
            np.float64
        ],  # 3次元bb座標, 8点*3*(valid_detects[0]個)が入っている オリジナルはLS_pcd_det.boxes
        minmax3ds: NDArray[
            np.float64
        ],  # 3次元bbのminmax座標, (valid_detects[0],6)が入っている オリジナルはLS_pcd_det.minmax
        bbox2d: NDArray[np.float32],  # 2次元bb座標と言うことにしておく
        yolo_classes: NDArray[np.int32],  # yoloのクラスID
        n_clusters: int,  # クラスタ数
        bbox2d_detection_count: int,  # 2次元bb検出数
        method: str = "center",
    ) -> dict[int, str]:
        # 立体物と人検知の紐づけ処理
        """
        元の環境にて、人検知結果:
        LS_cam_det.boxes = copy.deepcopy(pred_bbox[0]) → box2ds
        LS_cam_det.scores = copy.deepcopy(pred_bbox[1])
        LS_cam_det.classes = copy.deepcopy(pred_bbox[2]) → yolo_classes
        LS_cam_det.valid_detects = copy.deepcopy(pred_bbox[3]) → bbox2d_detection_count
        点群バウンディングボックス作成
        LS_pcd_det.boxes = copy.deepcopy(multi_points) → box3ds
        LS_pcd_det.minmax = copy.deepcopy(multi_minmax) → minmax3ds
        LS_pcd_det.valid_detects = copy.deepcopy(valid_detect_num) → n_clusters

        """
        width: int = self._image_width
        height: int = self._image_height
        ncm1: NDArray[np.float32] = self._ncm1

        # integrated_retults_2d3d : 元は3カメラ共通の3DBBの属性リスト。今回はカメラごとに独立して結果を出したいので、ただの辞書で良い
        integrated_retults_2d3d: dict[int, str] = {}  # 3DBBの属性リスト

        box3ds_reproj: NDArray[np.float64] = np.zeros(
            (n_clusters * 8, 2), dtype=np.float64
        )  # 3次元bbの射影変換結果を格納する配列
        if n_clusters != 0:
            box3ds_reproj = cv2.projectPoints(
                np.array([boxpoints]),
                rvec,
                tvec,
                ncm1,
                np.zeros((1, 5)),
            )[0].squeeze(1)
            assert np.array([boxpoints]).shape[0] == 1

            extrinsic_matrix = np.hstack([cv2.Rodrigues(rvec)[0], tvec.reshape(3, 1)])
            homogeneous_points = np.hstack(
                [boxpoints, np.ones((boxpoints.shape[0], 1))]
            ).T
            camera_coordinate_pts = extrinsic_matrix @ homogeneous_points
            camera_coordin_z = camera_coordinate_pts[2]

            BBOX_VERTEX_POINTS = 8

            # box3ds_reproj: bbox8点分ずつ格納。前からn_clusters*8点分のみ有効（n_clusters*8以降は不定？）対応するz座標を8個ずつ見て1つでも<0なら8点全て除去する必要がある
            # 本当に除去してしまうとintegrated_retults_2d3d反映時のインデックスと整合が取れなくなるので-1e6に飛ばすことで対応。
            camera_coordin_z_bboxset = camera_coordin_z.reshape(-1, BBOX_VERTEX_POINTS)
            camera_coordin_z_bboxset_flag = np.all(camera_coordin_z_bboxset > 0, axis=1)
            box3ds_zfilter = np.repeat(
                camera_coordin_z_bboxset_flag, BBOX_VERTEX_POINTS
            )

            box3ds_reproj[box3ds_zfilter == 0] = -1e6

            # 改良版：人らしさの寸法ゲートを追加
            for i in range(int(bbox2d_detection_count)):  # 2dbbを順番にチェック
                if yolo_classes[i] == 0:  # 人である場合
                    box2d_single: NDArray[np.float32] = bbox2d[i]
                    index: int = self.get_human_3bb(
                        box2d_single,
                        int(width),
                        int(height),
                        box3ds_reproj,
                        n_clusters,
                        method,
                    )
                    if index >= 0:
                        # ---- 寸法ゲート（人らしさ）で最終確認：外れたら HUMAN を取り消す ----
                        if self.use_human_gate:
                            # 最後に人寸法ゲートで判断
                            if self.passes_human_size(minmax3ds[index]):
                                integrated_retults_2d3d[index] = "HUMAN"
                            else:
                                # 人寸法を外れるので、元の属性のまま（誤通知抑止）
                                # AppLogger.debug(f"Rejected HUMAN by size gate: idx={index}")
                                pass
                        else:
                            # 人寸法ゲートは使わず、そのまま追加
                            integrated_retults_2d3d[index] = "HUMAN"

        return integrated_retults_2d3d


class calibcheck2d_bboxtracker_recorder:
    def __init__(
        self,
        app_config_calib: AppConfigCalibration,
        image_size_hw: tuple[int, int],
        camera_index: int,
    ) -> None:
        proc2d_conf = app_config_calib.calib2d3d.Proc2d
        self.mot_tracker = bbox2d_mot_tracker_wrapper(
            lost_track_buffer=int(proc2d_conf.lost_track_buffer),
            frame_rate=proc2d_conf.tracking_frame_rate,
            track_activation_threshold=proc2d_conf.track_activation_threshold,
            minimum_consecutive_frames=int(proc2d_conf.minimum_consecutive_frames),
            minimum_iou_threshold=proc2d_conf.minimum_iou_threshold,
        )
        self.image_size_hw = image_size_hw
        self.trackingID_data: dtype_tracking2dIDmetadata = {}
        self.trackingID_bboxlog: dtype_tracking2dIDbboxlog = {}
        self.last_tracker_result: NDArray | None = None
        self.lastframe_person_detected = False
        self.evLUT2D = NumpyMatrixLUT(
            A_X=proc2d_conf.cam_valmat_coord_A_X[camera_index],
            B_X=proc2d_conf.cam_valmat_coord_B_X[camera_index],
            A_Y=proc2d_conf.cam_valmat_coord_A_Y[camera_index],
            B_Y=proc2d_conf.cam_valmat_coord_B_Y[camera_index],
            ARRAY_PATH=proc2d_conf.cam_valmat_path[camera_index],
            DEFAULT_VALUE=proc2d_conf.cam_valmat_val_DEFAULT[camera_index],
        )
        self.evLUT2D_workarea = GrayImageLUT(
            A_X=proc2d_conf.cam_workareadef_img_coord_A_X[camera_index],
            B_X=proc2d_conf.cam_workareadef_img_coord_B_X[camera_index],
            A_Y=proc2d_conf.cam_workareadef_img_coord_A_Y[camera_index],
            B_Y=proc2d_conf.cam_workareadef_img_coord_B_Y[camera_index],
            IMAGE_PATH=proc2d_conf.cam_workareadef_img_path[camera_index],
            A_ETA=proc2d_conf.cam_workareadef_img_coord_A_ETA[camera_index],
            B_ETA=proc2d_conf.cam_workareadef_img_coord_B_ETA[camera_index],
            DEFAULT_VALUE=proc2d_conf.cam_workareadef_img_val_DEFAULT[camera_index],
        )

    def reset(self) -> None:
        self.mot_tracker.reset()
        self.last_tracker_result = None
        self.lastframe_person_detected = False

    @staticmethod
    def _calc_L2norm(a: tuple[float, float], b: tuple[float, float]) -> float:
        return float(np.sqrt(np.sum((np.array(a) - np.array(b)) ** 2)))

    def _update_trackinfo(self, frame_ix: int) -> None:
        if self.last_tracker_result is None:
            return

        for x1, y1, x2, y2, _prob, tracker_id in self.last_tracker_result:
            xc, yc = (x1 + x2) / 2, (y1 + y2) / 2
            xymin = min(x1, x2), min(y1, y2)
            xymax = max(x1, x2), max(y1, y2)
            if tracker_id >= 0:
                is_workarea = 1 if self.evLUT2D_workarea.evaluate(xc, yc) else 0
                frame_evval = self.evLUT2D.evaluate(xc, yc)
                if tracker_id not in self.trackingID_data:
                    self.trackingID_data[tracker_id] = tracking2d_dataclass(
                        accum_track_length=0,
                        final_xy=(xc, yc),
                        xymin=xymin,
                        xymax=xymax,
                        frame_ix_min=frame_ix,
                        frame_ix_max=frame_ix,
                        frame_ix_lastmove=frame_ix,
                        frame_evval_min=frame_evval,
                        frame_evval_max=frame_evval,
                        workarea_count=is_workarea,
                        is_alive=True,
                        is_tracking_target=True,
                    )
                    self.trackingID_bboxlog[tracker_id] = []
                else:
                    metadata = self.trackingID_data[tracker_id]
                    metadata.xymin = (
                        min(metadata.xymin[0], xymin[0]),
                        min(metadata.xymin[1], xymin[1]),
                    )
                    metadata.xymax = (
                        max(metadata.xymax[0], xymax[0]),
                        max(metadata.xymax[1], xymax[1]),
                    )
                    frame_movelen = self._calc_L2norm(metadata.final_xy, (xc, yc))
                    metadata.accum_track_length += frame_movelen
                    metadata.final_xy = (xc, yc)
                    metadata.frame_ix_max = frame_ix
                    if frame_movelen > 1e-6:
                        metadata.frame_ix_lastmove = frame_ix
                    metadata.frame_evval_min = min(
                        metadata.frame_evval_min, frame_evval
                    )
                    metadata.frame_evval_max = max(
                        metadata.frame_evval_max, frame_evval
                    )
                    metadata.workarea_count += is_workarea
                self.trackingID_bboxlog[tracker_id].append((frame_ix, (x1, y1, x2, y2)))

    def print_trackinfo(self, file=sys.stdout) -> None:
        print(f"追跡結果 len: {len(self.trackingID_data)}", file=file)
        for k, d in self.trackingID_data.items():
            print(
                f"key {k} : {d.__dict__}, points:{self.trackingID_bboxlog[k]}",
                file=file,
            )

    def get_rawresults(
        self,
    ) -> Tracking2dDataInterface:
        return Tracking2dDataInterface(self.trackingID_data, self.trackingID_bboxlog)

    def update(self, yoloresult_whole: list[NDArray], frame_ix: int) -> None:
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
        x1, x2, y1, y2, _cls_id, _ = np.array(YOLOsingleresult_conv, dtype=np.int32)
        prob = YOLOsingleresult_conv[5]

        cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 255, 0), 2)

        # Draw text box
        bbox_text = f"person: {prob:.1%}"
        text_size = cv2.getTextSize(bbox_text, 0, 1, 1)[0]
        cv2.rectangle(
            frame,
            (x1, y1),
            (x1 + text_size[0], y1 - text_size[1]),
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
        del file
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
                calibcheck2d_bboxtracker_recorder.draw_bboxes(
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


class calibcheck3d_bboxtracker_recorder:
    def __init__(
        self,
        app_config_calib: AppConfigCalibration,
        camera_index: int,
    ) -> None:
        proc3d_conf = app_config_calib.calib2d3d.Proc3d
        self.mot_tracker = bbox3d_mot_tracker_wrapper(
            lost_track_buffer=int(proc3d_conf.lost_track_buffer),
            frame_rate=proc3d_conf.tracking_frame_rate,
            track_activation_threshold=proc3d_conf.track_activation_threshold,
            minimum_consecutive_frames=int(proc3d_conf.minimum_consecutive_frames),
            minimum_iou_threshold=proc3d_conf.minimum_iou_threshold,
            enable_bbox3d_overlap_merge=proc3d_conf.enable_bbox3d_overlap_merge,
            bbox3d_overlap_merge_threshold=proc3d_conf.bbox3d_overlap_merge_threshold,
        )
        self.trackingID_data: dtype_tracking3dIDmetadata = {}
        self.trackingID_bboxlog: dtype_tracking3dIDbboxlog = {}
        self.last_tracker_result: NDArray | None = None
        self.lastframe_person_detected = False
        self.data_array_fbb_point_history: list[
            tuple[list[NDArray[np.float64]], int, list[float], NDArray[np.float64]]
        ] = []  # allpoints, timestamp, tracking_id, minmaxlist
        self.evLUT3D = NumpyMatrixLUT(
            A_X=proc3d_conf.lid_valmat_coord_A_X[camera_index],
            B_X=proc3d_conf.lid_valmat_coord_B_X[camera_index],
            A_Y=proc3d_conf.lid_valmat_coord_A_Y[camera_index],
            B_Y=proc3d_conf.lid_valmat_coord_B_Y[camera_index],
            ARRAY_PATH=proc3d_conf.lid_valmat_path[camera_index],
            DEFAULT_VALUE=proc3d_conf.lid_valmat_val_DEFAULT[camera_index],
        )
        self.evLUT3D_workarea = GrayImageLUT(
            A_X=proc3d_conf.lid_workareadef_img_coord_A_X[camera_index],
            B_X=proc3d_conf.lid_workareadef_img_coord_B_X[camera_index],
            A_Y=proc3d_conf.lid_workareadef_img_coord_A_Y[camera_index],
            B_Y=proc3d_conf.lid_workareadef_img_coord_B_Y[camera_index],
            IMAGE_PATH=proc3d_conf.lid_workareadef_img_path[camera_index],
            A_ETA=proc3d_conf.lid_workareadef_img_coord_A_ETA[camera_index],
            B_ETA=proc3d_conf.lid_workareadef_img_coord_B_ETA[camera_index],
            DEFAULT_VALUE=proc3d_conf.lid_workareadef_img_val_DEFAULT[camera_index],
        )

    def reset(self) -> None:
        self.mot_tracker.reset()
        self.last_tracker_result = None
        self.lastframe_person_detected = False
        self.data_array_fbb_point_history = []

    @staticmethod
    def _calc_L2norm(a: tuple[float, float], b: tuple[float, float]) -> float:
        return float(np.linalg.norm(np.array(a) - np.array(b)))

    def _update_trackinfo(self, frame_ix: int) -> None:
        if self.last_tracker_result is None:
            return

        for result in self.last_tracker_result:
            if len(result) != 6:
                return

            x1, y1, x2, y2, _prob, tracker_id = result
            if tracker_id < 0:
                continue

            xc = (x1 + x2) / 2
            yc = (y1 + y2) / 2
            distance = float(np.hypot(xc, yc))
            is_workarea = 1 if self.evLUT3D_workarea.evaluate(xc, yc) > 0 else 0
            frame_evval = self.evLUT3D.evaluate(xc, yc)

            if tracker_id not in self.trackingID_data:
                self.trackingID_data[tracker_id] = tracking3d_dataclass(
                    accum_track_length=0,
                    final_xy=(xc, yc),
                    dist_from_camera_min=distance,
                    dist_from_camera_max=distance,
                    frame_ix_min=frame_ix,
                    frame_ix_max=frame_ix,
                    frame_ix_lastmove=frame_ix,
                    frame_evval_min=frame_evval,
                    frame_evval_max=frame_evval,
                    workarea_count=is_workarea,
                    is_alive=True,
                    is_tracking_target=True,
                )
                self.trackingID_bboxlog[tracker_id] = []
            else:
                metadata = self.trackingID_data[tracker_id]
                frame_movelen = self._calc_L2norm(metadata.final_xy, (xc, yc))
                metadata.accum_track_length += frame_movelen
                metadata.final_xy = (xc, yc)
                metadata.frame_ix_max = frame_ix
                if frame_movelen > 1e-6:
                    metadata.frame_ix_lastmove = frame_ix
                metadata.workarea_count += is_workarea
                metadata.dist_from_camera_min = min(
                    metadata.dist_from_camera_min, distance
                )
                metadata.dist_from_camera_max = max(
                    metadata.dist_from_camera_max, distance
                )
                metadata.frame_evval_min = min(metadata.frame_evval_min, frame_evval)
                metadata.frame_evval_max = max(metadata.frame_evval_max, frame_evval)

            self.trackingID_bboxlog[tracker_id].append((frame_ix, (x1, y1, x2, y2)))

    def print_trackinfo(self, file=sys.stdout) -> None:
        print(f"追跡結果 len: {len(self.trackingID_data)}", file=file)
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

    def update(
        self,
        bbox_multi_minmax: NDArray,
        frame_ix: int,
    ) -> None:
        # SORT法追跡
        self.last_tracker_result = self.mot_tracker.update(
            bbox_multi_minmax=bbox_multi_minmax
        )
        merged_bbox = self.mot_tracker.last_bbox_multi_minmax
        assert merged_bbox is not None
        # 追跡によるメタデータ更新
        self._update_trackinfo(frame_ix=frame_ix)
        self.data_array_fbb_point_history.append(
            (
                [np.zeros((0, 3), dtype=np.float32)],  # allpoints
                frame_ix,
                list(self.last_tracker_result[:, 5]),  # tracking_id
                merged_bbox.copy(),  # minmaxlist
            )
        )

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
    ) -> list[tuple[list[NDArray[np.float64]], int, list[float], NDArray[np.float64]]]:
        return self.data_array_fbb_point_history


class calibcheck2d3d:
    def _report_file_io_error_impl(
        self, path: str, operation: str, error: Exception
    ) -> None:
        file_io_error = self._ser.state_errors_D[StateErrorDIndex.FILE_IO_ERROR]
        result = file_io_error.errors_diagnosis(True)
        file_io_error.log_output(
            *result,
            StateErrorDIndex.FILE_IO_ERROR,
            path,
            operation,
            f"{type(error).__name__}: {error}",
        )

    def __init__(
        self,
        app_config_calib: AppConfigCalibration,
        sac: SharedAppConfig,
        app_logger_factory: AppLoggerFactory,
        shared_errors: SharedErrors,
    ) -> None:
        self._logger: AppLogger = app_logger_factory.register_from_type(self.__class__)
        self.sac = sac
        self.app_config_calib = app_config_calib
        self._ser: SharedErrors = shared_errors
        self._calibcheck_status_diagnosis = CameraCalibCheckStatusDiagnosis()

        debug_config(
            base_dir="debug_out",
            flush_interval_sec=0.5,
            snapshot_interval_sec=5.0,
            snapshot_on_close=True,
            fsync_snapshot=False,
            fsync_flush=False,
        )

        self._report_file_io_error: Callable[[str, str, Exception], None] = (
            self._report_file_io_error_impl
        )

        self.calibcheck2d3d_conf: CalibCheck2d3dConf = app_config_calib.calibCheck2d3d
        self.dataCapture_conf: DataCaptureConf = app_config_calib.dataCapture
        self._evaluation_camera_count = self.calibcheck2d3d_conf.camera_count
        self.FRAME_INFO_MAXLEN = self.calibcheck2d3d_conf.frame_info_maxlen
        self.THRESH_3DBBOX_COUNT_PER_FRAME = (
            self.calibcheck2d3d_conf.thresh_3dbbox_count_per_frame
        )
        self.THRESH_3DBBOX_COUNT_MEAN_RATIO = (
            self.calibcheck2d3d_conf.thresh_3dbbox_count_mean_ratio
        )
        self.THRESH_2DBBOX_COUNT_PER_FRAME = (
            self.calibcheck2d3d_conf.thresh_2dbbox_count_per_frame
        )
        self.THRESH_2DBBOX_COUNT_MEAN_RATIO = (
            self.calibcheck2d3d_conf.thresh_2dbbox_count_mean_ratio
        )
        self.THRESH_3DBBOX_TRACKING_IDCOUNT = (
            self.calibcheck2d3d_conf.thresh_3dbbox_tracking_idcount
        )
        self.THRESH_2DBBOX_TRACKING_IDCOUNT = (
            self.calibcheck2d3d_conf.thresh_2dbbox_tracking_idcount
        )
        self.WARN_3D_TRACK_PROXIMITY_ENABLED = True
        self.WARN_3D_TRACK_PROXIMITY_FAILS_VALIDATION = True
        self.WARN_3D_TRACK_CENTER_DISTANCE_M = 2.0
        self.WARN_3D_TRACK_MIN_CLOSE_FRAMES = 20
        self.WARN_3D_TRACK_MIN_CLOSE_RATIO = 0.2
        self._3d_track_proximity_warnings: list[TrackProximityWarning] = []
        self.yolo_adapter = YOLODamoBatchAdapter(
            app_config_calib=app_config_calib,
            app_logger_factory=app_logger_factory,
            shared_errors=shared_errors,
        )

        self.ud: Mcde7000UndistortImageProvider = Mcde7000UndistortImageProvider(
            camera_intrinsics_path=self.calibcheck2d3d_conf.camera_intrinsics_path,
            sys_width=self.calibcheck2d3d_conf.image_w,
            sys_height=self.calibcheck2d3d_conf.image_h,
        )
        self.width = self.calibcheck2d3d_conf.image_w
        self.height = self.calibcheck2d3d_conf.image_h
        self._evaluation_intrinsics = self.ud.ncm1
        self.EVAL_FRAME_STRIDE = self.calibcheck2d3d_conf.eval_frame_stride
        self.USE_LEGACY_LIKE_METRIC = self.calibcheck2d3d_conf.use_legacy_like_metric
        self.EVAL_ZVALUES = self.calibcheck2d3d_conf.eval_zvalues
        self.VIRTUAL_BBOX_XY_OFFSETS = VIRTUAL_BBOX_XY_OFFSETS
        self.VIRTUAL_BBOX_CANDIDATE_NAMES = VIRTUAL_BBOX_CANDIDATE_NAMES
        self._virtual_bbox_debug_counts: list[VirtualBBoxDebugCounts] = []
        self._evaluation_metric_debug: list[EvaluationMetricDebug] = []
        self.DEBUG_CALIBCHECK_ENABLED = (
            self.calibcheck2d3d_conf.debug_calibcheck_enabled
        )
        self.DEBUG_CAPTURE_UI_VIDEO_ENABLED = (
            self.calibcheck2d3d_conf.debug_capture_ui_video_enabled
        )
        self.DEBUG_CAPTURE_FRAME_TEXT_ENABLED = (
            self.calibcheck2d3d_conf.debug_capture_frame_text_enabled
        )
        self.DEBUG_VIDEO_FPS = self.calibcheck2d3d_conf.debug_video_fps
        self.DEBUG_VIDEO_PREFIX = self.calibcheck2d3d_conf.debug_video_prefix
        self.DEBUG_EVAL_PICKLE_PATH = self.calibcheck2d3d_conf.debug_eval_pickle_path
        self.DEBUG_EVAL_TRACE_ENABLED = (
            self.calibcheck2d3d_conf.debug_eval_trace_enabled
        )
        self.DEBUG_EVAL_TRACE_ALL_FRAMES = (
            self.calibcheck2d3d_conf.debug_eval_trace_all_frames
        )
        self.DEBUG_EVAL_TRACE_RANGE_START = (
            self.calibcheck2d3d_conf.debug_eval_trace_range_start
        )
        self.DEBUG_EVAL_TRACE_RANGE_END = (
            self.calibcheck2d3d_conf.debug_eval_trace_range_end
        )
        self.DEBUG_EVAL_TRACE_TARGET_FRAMES: set[int] = set()
        self._debug_video_writers: list[cv2.VideoWriter | None] = []
        self._debug_video_paths: list[str] = []
        self._debug_eval_info: dict[str, object] = {}
        self.scenedesc_calibcheck = Scene_CalibCheck2d3d.create_for_evaluation(
            scene_conf=self.sac.read().SceneDescription,
            camera_intrinsics=self._evaluation_intrinsics,
            image_width=self.width,
            image_height=self.height,
        )

        self.cap3d = t3dc.capture3d(
            app_config_calib=self.app_config_calib,
            sac=self.sac,
            app_logger_factory=app_logger_factory,
            file_io_error_reporter=self._report_file_io_error,
        )

        # 各クラスコンストラクタ呼び出し
        self.proccap = datacapture_class(
            app_config_calib=self.app_config_calib,
            sac=self.sac,
            app_logger_factory=app_logger_factory,
            shared_errors=self._ser,
        )

        # 点群事前処理：静止点群除去
        self.DATARANGE_XYZ = ((-10, 20), (-20, 20), (-2, 2))
        self.static_point_filter = filter_static_objects(
            1.0,
            *self.DATARANGE_XYZ,
        )
        self.pointfilter_lastadd = -1

        self.debug_index = 0
        self.monitor_data = {}
        self.verbose = not app_config_calib.default.print_disabled

    @classmethod
    def create_for_evaluation(
        cls,
        *,
        camera_extrinsics: list[NDArray[np.float64]],
        camera_intrinsics: NDArray[np.float64],
        image_width: int,
        image_height: int,
        scene_conf: SceneDescriptionConf,
        app_logger: AppLogger,
    ) -> "calibcheck2d3d":
        instance = cls.__new__(cls)
        instance._logger = app_logger
        instance.width = image_width
        instance.height = image_height
        instance._evaluation_camera_count = len(camera_extrinsics)
        instance._evaluation_intrinsics = np.array(camera_intrinsics, dtype=np.float32)
        instance.rtvec_mat = []
        for extrinsic in camera_extrinsics:
            rotation = extrinsic[:3, :3]
            translation = extrinsic[:3, 3].reshape(3, 1)
            instance.rtvec_mat.append(
                (cv2.Rodrigues(rotation)[0], translation, extrinsic)
            )
        instance.scenedesc_calibcheck = Scene_CalibCheck2d3d.create_for_evaluation(
            scene_conf=scene_conf,
            camera_intrinsics=instance._evaluation_intrinsics,
            image_width=image_width,
            image_height=image_height,
        )
        instance.EVAL_FRAME_STRIDE = 1
        instance.USE_LEGACY_LIKE_METRIC = True
        instance.THRESH_3DBBOX_COUNT_PER_FRAME = 1
        instance.THRESH_3DBBOX_COUNT_MEAN_RATIO = 0.1
        instance.THRESH_2DBBOX_COUNT_PER_FRAME = 1
        instance.THRESH_2DBBOX_COUNT_MEAN_RATIO = 0.1
        instance.THRESH_3DBBOX_TRACKING_IDCOUNT = 1
        instance.THRESH_2DBBOX_TRACKING_IDCOUNT = 1
        instance.WARN_3D_TRACK_PROXIMITY_ENABLED = True
        instance.WARN_3D_TRACK_PROXIMITY_FAILS_VALIDATION = False
        instance.WARN_3D_TRACK_CENTER_DISTANCE_M = 1.0
        instance.WARN_3D_TRACK_MIN_CLOSE_FRAMES = 5
        instance.WARN_3D_TRACK_MIN_CLOSE_RATIO = 0.2
        instance.DEBUG_CALIBCHECK_ENABLED = False
        instance.DEBUG_EVAL_TRACE_ENABLED = False
        instance.DEBUG_EVAL_TRACE_ALL_FRAMES = False
        instance.DEBUG_EVAL_TRACE_RANGE_START = 0
        instance.DEBUG_EVAL_TRACE_RANGE_END = -1
        instance.DEBUG_EVAL_TRACE_TARGET_FRAMES = set()
        instance.VIRTUAL_BBOX_XY_OFFSETS = VIRTUAL_BBOX_XY_OFFSETS
        instance.VIRTUAL_BBOX_CANDIDATE_NAMES = VIRTUAL_BBOX_CANDIDATE_NAMES
        instance._virtual_bbox_debug_counts = []
        instance._evaluation_metric_debug = []
        instance._3d_track_proximity_warnings = []
        return instance

    @property
    def virtual_bbox_debug_counts(self) -> list[VirtualBBoxDebugCounts]:
        return self._virtual_bbox_debug_counts

    @property
    def evaluation_metric_debug(self) -> list[EvaluationMetricDebug]:
        return self._evaluation_metric_debug

    @property
    def track_proximity_warnings(self) -> list[TrackProximityWarning]:
        return self._3d_track_proximity_warnings

    def input_settings(self) -> None:
        # TODO: 同期入力別プロセスのモジュールに入替

        self.trans_mat3D3D_eachlidar = []
        for path in self.calibcheck2d3d_conf.lidar_calib_files:
            self._logger.info(f"[input_settings] path:{path}")
            try:
                transform = np.loadtxt(path, delimiter=",")
            except (OSError, UnicodeError, ValueError) as error:
                self._report_file_io_error(
                    path, "read calibcheck2d3d LiDAR calibration CSV", error
                )
                raise
            self._logger.info(f"loadtxt: {transform}")
            self.trans_mat3D3D_eachlidar.append(transform)

        self.rtvec_mat = []
        for path in self.calibcheck2d3d_conf.camera_calib_files:
            try:
                rtvec = read_rtvec(
                    rvec_convmat_path=path,
                    new_axis_mode=self.calibcheck2d3d_conf.new_axis_mode,
                    points_inverted=True,
                )
            except (
                OSError,
                UnicodeError,
                ValueError,
                RuntimeError,
                EOFError,
            ) as error:
                self._report_file_io_error(
                    path, "read calibcheck2d3d camera calibration", error
                )
                raise
            self.rtvec_mat.append(rtvec)

    def _ensure_debug_video_writer(
        self,
        camera_ix: int,
        frame_bgr: NDArray[np.uint8],
    ) -> None:
        if not self.DEBUG_CALIBCHECK_ENABLED or not self.DEBUG_CAPTURE_UI_VIDEO_ENABLED:
            return
        if self._debug_video_writers[camera_ix] is not None:
            return

        height, width = frame_bgr.shape[:2]
        output_path = f"{self.DEBUG_VIDEO_PREFIX}{camera_ix}calibcheck.mp4"
        writer = cv2.VideoWriter(
            output_path,
            cv2.VideoWriter_fourcc(*"mp4v"),
            self.DEBUG_VIDEO_FPS,
            (width, height),
        )
        if not writer.isOpened():
            self._logger.warning(
                f"Debug video writer open failed: camera={camera_ix}, path={output_path}",
            )
            return

        self._debug_video_writers[camera_ix] = writer
        self._debug_video_paths[camera_ix] = output_path
        self._logger.info(
            f"Debug video writer opened: camera={camera_ix}, path={output_path}",
        )

    def _write_debug_video_frames(
        self,
        monitor: CalibrationUIGodot,
        frame_ix: int,
    ) -> None:
        if not self.DEBUG_CALIBCHECK_ENABLED or not self.DEBUG_CAPTURE_UI_VIDEO_ENABLED:
            return

        for camera_ix, frame in enumerate(monitor.cameradata):
            if frame is None:
                continue

            self._ensure_debug_video_writer(camera_ix, frame)
            writer = self._debug_video_writers[camera_ix]
            if writer is None:
                continue

            output_frame = frame.copy()
            if self.DEBUG_CAPTURE_FRAME_TEXT_ENABLED:
                cv2.putText(
                    output_frame,
                    f"frame={frame_ix}",
                    (20, 40),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    1.0,
                    (0, 255, 255),
                    2,
                    cv2.LINE_AA,
                )
            writer.write(output_frame)

    def _close_debug_video_writers(self) -> None:
        for camera_ix, writer in enumerate(self._debug_video_writers):
            if writer is None:
                continue
            writer.release()
            self._logger.info(
                f"Debug video writer closed: camera={camera_ix}, path={self._debug_video_paths[camera_ix]}",
            )
            self._debug_video_writers[camera_ix] = None

    def _dump_debug_eval_info(self) -> None:
        if not self.DEBUG_CALIBCHECK_ENABLED:
            return
        with open(self.DEBUG_EVAL_PICKLE_PATH, "wb") as output_file:
            pickle.dump(self._debug_eval_info, output_file)
        self._logger.info(
            f"Debug eval info dumped: {self.DEBUG_EVAL_PICKLE_PATH}",
        )

    def _dump_frame_info(self) -> None:
        if not self.DEBUG_CALIBCHECK_ENABLED:
            return
        with open("frame_info_dump.pickle", "wb") as output_file:
            pickle.dump(self.frame_info, output_file)

    def _should_trace_eval_frame(self, frame_ix: int) -> bool:
        if not self.DEBUG_CALIBCHECK_ENABLED or not self.DEBUG_EVAL_TRACE_ENABLED:
            return False
        if self.DEBUG_EVAL_TRACE_ALL_FRAMES:
            return True
        if (
            self.DEBUG_EVAL_TRACE_RANGE_START
            <= frame_ix
            <= self.DEBUG_EVAL_TRACE_RANGE_END
        ):
            return True
        return frame_ix in self.DEBUG_EVAL_TRACE_TARGET_FRAMES

    @staticmethod
    def _append_trace_event(
        eval_trace_by_camera: dict[int, dict[int, list[dict[str, object]]]],
        camera_ix: int,
        frame_ix: int,
        event: dict[str, object],
    ) -> None:
        if camera_ix not in eval_trace_by_camera:
            eval_trace_by_camera[camera_ix] = {}
        if frame_ix not in eval_trace_by_camera[camera_ix]:
            eval_trace_by_camera[camera_ix][frame_ix] = []
        eval_trace_by_camera[camera_ix][frame_ix].append(event)

    # データ取得・bbox記録

    def _sub_proc_3d_monitor_data(
        self,
        monitor: CalibrationUIGodot,
        pcd_limited: NDArray[np.float32],
        multi_points: NDArray[np.float32],
        multi_lines: NDArray[np.float32],
    ) -> None:
        if pcd_limited.shape[-1] == 4:
            monitor.set_points(
                pcd_limited[:, :3],
                monitor.convert_intensity_to_color(pcd_limited[:, 3]),
            )
        elif pcd_limited.shape[-1] == 3:
            monitor.set_points(
                pcd_limited,
                np.tile([0.2, 0.2, 0.2], (pcd_limited.shape[0], 1)),
            )
        else:
            monitor.set_points(
                np.zeros((0, 3), dtype=np.float32),
                np.zeros((0, 3), dtype=np.float32),
            )
        monitor.set_boxes(
            points_multipoints=multi_points,
            points_multi_lines=multi_lines,
        )

    def proc_lidar1f(
        self,
        lidar_datalist: list[tuple[NDArray[np.float64], int, float] | None],
        framecounter: int,
        monitor: CalibrationUIGodot,
    ) -> NDArray[np.float64]:
        pcd, _ts_lidar_raw = self.cap3d.read(
            lidar_datalist, dontread=False, adjust_coordinate_enable=False
        )
        self._logger.info(
            f"proc_lidar1f: pcd.shape={pcd.shape}, framecounter={framecounter}"
        )

        pcd[:, 1] = -pcd[:, 1]
        pcd[:, 2] = -pcd[:, 2]
        points = self._sub_detect_apply_static_point_filter(
            pcdframe=pcd,
            timestamp_pcd=framecounter,
        )
        threshold = -self.calibcheck2d3d_conf.z_threshold
        points = points[points[:, 2] < threshold]

        points_object = np.array(
            utils3d.np_to_pcd(points[:, :3]).voxel_down_sample(0.2).points
        )
        (multi_points, multi_lines, multi_minmax), pcd_limited, _db = internal_make_BB(
            points_object,
            x_range=self.DATARANGE_XYZ[0],
            y_range=self.DATARANGE_XYZ[1],
            z_range=self.DATARANGE_XYZ[2],
        )
        self._logger.info(
            f"proc_lidar1f: "
            f"pcd.shape={pcd.shape}, points.shape={points.shape}, points_object.shape={points_object.shape}, "
            f"pcd_limited.shape={pcd_limited.shape}, multi_points.shape={multi_points.shape}, "
            f"multi_lines.shape={multi_lines.shape}, multi_minmax.shape={multi_minmax.shape}, "
            f"framecounter={framecounter}"
        )
        self._sub_proc_3d_monitor_data(
            monitor=monitor,
            pcd_limited=pcd_limited,
            multi_points=multi_points,
            multi_lines=multi_lines,
        )
        return multi_minmax.reshape(-1, 6)  # array(N,6) 3D bboxのminmax情報を返す

    def _sub_proc_2d_monitor_data(
        self,
        monitor: CalibrationUIGodot,
        frame_cameras: list[NDArray[np.uint8] | None],
        yoloresult_whole_list: list[list[NDArray[np.float32]]],
    ) -> None:
        for camera_ix, frame in enumerate(frame_cameras):
            if frame is None:
                monitor.set_2Dbbox(camera_ix, np.zeros((0, 4), dtype=np.int32))
                continue

            yoloresult_whole = yoloresult_whole_list[camera_ix]
            frame, _ = draw_multibbox(frame, yoloresult_whole)
            bbox_for_ui: list[list[int]] = []
            image_h, image_w, _ = frame.shape
            for result_ix in range(int(yoloresult_whole[3])):
                coordinate = yoloresult_whole[0].reshape((-1, 4))[result_ix]
                bbox_for_ui.append(
                    [
                        int(coordinate[1] * image_w),
                        int(coordinate[0] * image_h),
                        int(coordinate[3] * image_w),
                        int(coordinate[2] * image_h),
                    ]
                )
            monitor.set_2Dbbox(
                camera_ix,
                np.array(bbox_for_ui, dtype=np.int32).reshape(-1, 4),
            )

    @staticmethod
    def _make_empty_yoloresult() -> list[NDArray[np.float32]]:
        # YOLO出力フォーマットに合わせて、検出0件の結果を返す。
        return [
            np.zeros((0, 4), dtype=np.float32),
            np.zeros((0,), dtype=np.float32),
            np.zeros((0,), dtype=np.float32),
            np.array(0, dtype=np.float32),
        ]

    def proc_camera1f(
        self,
        camera_datalist: list[tuple[NDArray[np.uint8], int, float] | None],
        framecounter: int,
        monitor: CalibrationUIGodot,
        sec: SharedExcepts,
    ) -> list[list[NDArray[np.float32]]]:
        del framecounter
        camera_count = self.dataCapture_conf.Camera.count
        frame_cameras: list[NDArray[np.uint8] | None] = [None] * camera_count
        for camera_ix, camera_data in enumerate(camera_datalist):
            if camera_data is None:
                self._logger.info(f"frame {camera_ix} is invalid, skip")
                continue
            if camera_ix >= camera_count:
                self._logger.warning(
                    f"camera index {camera_ix} is out of configured range "
                    f"{camera_count}, skip",
                )
                continue
            frame = self.ud.get_undistort_image(camera_data[0])
            frame_cameras[camera_ix] = frame
            monitor.set_image(camera_ix, frame)

        yoloresult_whole_list = self.yolo_adapter.predict_batch(
            sec=sec,
            frames=frame_cameras,
        )
        for camera_ix in range(camera_count):
            detected_count = int(yoloresult_whole_list[camera_ix][3])
            self._logger.info(
                f"YOLO detection results: {detected_count} objects detected "
                f"in camera {camera_ix}",
            )
        self._sub_proc_2d_monitor_data(
            monitor=monitor,
            frame_cameras=frame_cameras,
            yoloresult_whole_list=yoloresult_whole_list,
        )
        return yoloresult_whole_list

    def record_bbox1f(
        self,
        multi_minmax: NDArray[np.float64],
        yoloresult_whole_list: list[list[NDArray]],
    ) -> None:
        if len(self.frame_info) >= self.FRAME_INFO_MAXLEN:
            self.frame_info.pop(0)
        self.frame_info.append((yoloresult_whole_list, multi_minmax))

    def validate_3dbbox_log(self) -> bool:
        # 3D bboxログ: self.frame_info [:][1]部分。list長さがbbox個数なので、各フレームへの閾値、全フレームに渡っての比率閾値を計算。この部分は判定基準次第で変化。
        framecount_3dbbox_valid = 0
        for _, multi_minmax in self.frame_info:
            if (
                multi_minmax is not None
                and len(multi_minmax) >= self.THRESH_3DBBOX_COUNT_PER_FRAME
            ):
                framecount_3dbbox_valid += 1
        self._logger.info(
            f"validate_3dbbox_log: framecount_3dbbox_valid={framecount_3dbbox_valid}, total_frames={len(self.frame_info)}, ratio={framecount_3dbbox_valid / len(self.frame_info) if self.frame_info else 0.0}",
        )
        return (
            framecount_3dbbox_valid / len(self.frame_info)
            >= self.THRESH_3DBBOX_COUNT_MEAN_RATIO
            if self.frame_info
            else False
        )

    def validate_2dbbox_log(self) -> list[bool]:
        # 2D bboxログ: self.frame_info [:][0]部分。
        # カメラごとの結果のlistで、各カメラごとにYOLO検出結果フォーマット(float 4値)×要素数50、確信度×要素数50、クラス×要素数50のnumpy配列と、それに加えて末尾に有効個数が格納。
        # 各フレームへの閾値、全フレームに渡っての比率閾値を計算。この部分は判定基準次第で変化。

        # 多分これで行けるのだが自信がない。トレースバックで詳細情報を取得したい。
        # try:
        framecount_2dbbox_valid_percamera = [
            0 for _ in range(self._evaluation_camera_count)
        ]
        for yoloresult_whole_list, _ in self.frame_info:
            for camera_ix, yoloresult_whole in enumerate(yoloresult_whole_list):
                if (
                    yoloresult_whole is not None
                    and yoloresult_whole[3] >= self.THRESH_2DBBOX_COUNT_PER_FRAME
                ):
                    framecount_2dbbox_valid_percamera[camera_ix] += 1

        self._logger.info(
            f"validate_2dbbox_log: framecount_2dbbox_valid_percamera={framecount_2dbbox_valid_percamera}, total_frames={len(self.frame_info)}, ratio={[framecount_2dbbox_valid_percamera[camera_ix] / len(self.frame_info) if self.frame_info else 0.0 for camera_ix in range(self._evaluation_camera_count)]}",
        )
        return [
            framecount_2dbbox_valid_percamera[camera_ix]
            >= self.THRESH_2DBBOX_COUNT_MEAN_RATIO * len(self.frame_info)
            if self.frame_info
            else False
            for camera_ix in range(self._evaluation_camera_count)
        ]

    def validate_recorded_bbox_logs(self) -> list[int]:
        if self.validate_3dbbox_log() is False:
            return [2 for _ in range(self._evaluation_camera_count)]

        return [
            0 if validation_result else 3
            for validation_result in self.validate_2dbbox_log()
        ]

    def track_3dbbox(self) -> Tracking3dDataInterface:
        # BBox追跡、記録
        bbox_track_and_record_3d = calibcheck3d_bboxtracker_recorder(
            app_config_calib=self.app_config_calib,
            camera_index=self.sac.read().CalibMode.cameraID,
        )
        # 一旦全体を再スキャン もしかするとリアルタイム処理の方が早いかもしれない
        for frame_ix, (_, multi_minmax) in enumerate(self.frame_info):
            # 3D追跡 - bbox閾値判定
            bbox_track_and_record_3d.update(multi_minmax, frame_ix)

        # 判定をここに記載　今回は全部有効のままにして様子見
        return bbox_track_and_record_3d.get_rawresults()

    def select_3dbbox_tracking_results(
        self, tracking_3d_data_interface: Tracking3dDataInterface
    ) -> Tracking3dDataInterface:
        # 3D bboxトラッキング結果の選択(datainterfaceのプロパティで判定し、is_aliveをFalseにすることで「選択」（除外）を行う)

        # 閾値判定
        for id, metadata in tracking_3d_data_interface.trackingIDmetadata.items():
            frame_ix_length = metadata.frame_ix_max - metadata.frame_ix_min
            accum_track_length = metadata.accum_track_length
            workarea_count = metadata.workarea_count
            self._logger.info(
                f"select_3dbbox_tracking_results: id={id}, frame_ix_length={frame_ix_length}, accum_track_length={accum_track_length}, workarea_count={workarea_count}",
            )

            if frame_ix_length < 30:
                tracking_3d_data_interface.trackingIDmetadata[id].is_alive = False
            if accum_track_length < 3.0:
                tracking_3d_data_interface.trackingIDmetadata[id].is_alive = False
            if workarea_count < frame_ix_length * 0.1 or workarea_count < 30:
                tracking_3d_data_interface.trackingIDmetadata[id].is_alive = False

        return tracking_3d_data_interface

    def validate_3dbbox_tracking_results(
        self, tracking_3d_data_interface: Tracking3dDataInterface
    ) -> bool:
        # 3D bboxトラッキング結果の妥当性判定
        # ここでは、トラッキング結果の数が閾値を満たしているかどうかを判定する。必要に応じて、トラッキング結果の属性やスコアなども考慮することができる。
        self._3d_track_proximity_warnings = self._detect_close_3dbbox_tracks(
            tracking_3d_data_interface
        )
        if self._3d_track_proximity_warnings:
            self._logger.warning(
                f"Close 3D tracks detected: {self._3d_track_proximity_warnings}",
            )

        count = 0
        if (
            len(tracking_3d_data_interface.trackingIDmetadata)
            >= self.THRESH_3DBBOX_TRACKING_IDCOUNT
        ):
            for _id, metadata in tracking_3d_data_interface.trackingIDmetadata.items():
                if metadata.is_alive:
                    count += 1
            if count >= self.THRESH_3DBBOX_TRACKING_IDCOUNT:
                if (
                    self.WARN_3D_TRACK_PROXIMITY_FAILS_VALIDATION
                    and self._3d_track_proximity_warnings
                ):
                    self._logger.info(
                        "validate_3dbbox_tracking_results: close 3D tracks "
                        f"detected, warnings={self._3d_track_proximity_warnings}, "
                        "return False",
                    )
                    return False
                return True
        self._logger.info(
            f"validate_3dbbox_tracking_results: count={count}, threshold={self.THRESH_3DBBOX_TRACKING_IDCOUNT}, return False",
        )
        return False

    def _detect_close_3dbbox_tracks(
        self,
        tracking_3d_data_interface: Tracking3dDataInterface,
    ) -> list[TrackProximityWarning]:
        if not self.WARN_3D_TRACK_PROXIMITY_ENABLED:
            return []

        alive_track_ids = [
            track_id
            for track_id, metadata in tracking_3d_data_interface.trackingIDmetadata.items()
            if metadata.is_alive
        ]
        if len(alive_track_ids) < 2:
            return []

        centers_by_track_id: dict[int, dict[int, tuple[float, float]]] = {}
        for track_id in alive_track_ids:
            frame_centers: dict[int, tuple[float, float]] = {}
            for frame_ix, bbox_xy in tracking_3d_data_interface.trackingIDbboxlog.get(
                track_id, []
            ):
                x1, y1, x2, y2 = bbox_xy
                frame_centers[int(frame_ix)] = (
                    (float(x1) + float(x2)) / 2.0,
                    (float(y1) + float(y2)) / 2.0,
                )
            if frame_centers:
                centers_by_track_id[track_id] = frame_centers

        warnings: list[TrackProximityWarning] = []
        for track_index, track_id_a in enumerate(alive_track_ids):
            centers_a = centers_by_track_id.get(track_id_a)
            if centers_a is None:
                continue

            for track_id_b in alive_track_ids[track_index + 1 :]:
                centers_b = centers_by_track_id.get(track_id_b)
                if centers_b is None:
                    continue

                common_frame_ixs = sorted(set(centers_a) & set(centers_b))
                if not common_frame_ixs:
                    continue

                close_frame_ixs: list[int] = []
                min_center_distance_m = float("inf")
                closest_frame_ix = common_frame_ixs[0]
                for frame_ix in common_frame_ixs:
                    center_a = centers_a[frame_ix]
                    center_b = centers_b[frame_ix]
                    center_distance_m = float(
                        np.hypot(
                            center_a[0] - center_b[0],
                            center_a[1] - center_b[1],
                        )
                    )
                    if center_distance_m < min_center_distance_m:
                        min_center_distance_m = center_distance_m
                        closest_frame_ix = frame_ix
                    if center_distance_m <= self.WARN_3D_TRACK_CENTER_DISTANCE_M:
                        close_frame_ixs.append(frame_ix)

                close_frame_ratio = len(close_frame_ixs) / len(common_frame_ixs)
                if (
                    len(close_frame_ixs) >= self.WARN_3D_TRACK_MIN_CLOSE_FRAMES
                    and close_frame_ratio >= self.WARN_3D_TRACK_MIN_CLOSE_RATIO
                ):
                    warnings.append(
                        {
                            "track_id_a": int(track_id_a),
                            "track_id_b": int(track_id_b),
                            "common_frame_count": len(common_frame_ixs),
                            "close_frame_count": len(close_frame_ixs),
                            "close_frame_ratio": float(close_frame_ratio),
                            "min_center_distance_m": float(min_center_distance_m),
                            "closest_frame_ix": int(closest_frame_ix),
                            "close_frame_samples": [
                                int(frame_ix) for frame_ix in close_frame_ixs[:10]
                            ],
                        }
                    )

        return warnings

    def track_2dbbox(self) -> list[Tracking2dDataInterface]:
        bbox_track_and_record_2d_allcamera = [
            calibcheck2d_bboxtracker_recorder(
                app_config_calib=self.app_config_calib,
                image_size_hw=(
                    self.calibcheck2d3d_conf.image_h,
                    self.calibcheck2d3d_conf.image_w,
                ),
                camera_index=camera_ix,
            )
            for camera_ix in range(self._evaluation_camera_count)
        ]

        for frame_ix, (yoloresult_whole_list, _) in enumerate(self.frame_info):
            for camera_ix, yoloresult_whole in enumerate(yoloresult_whole_list):
                bbox_track_and_record_2d_allcamera[camera_ix].update(
                    yoloresult_whole, frame_ix
                )

        return [
            recorder.get_rawresults() for recorder in bbox_track_and_record_2d_allcamera
        ]

    def select_2dbbox_tracking_results(
        self, tracking_2d_data_interfaces: list[Tracking2dDataInterface]
    ) -> list[Tracking2dDataInterface]:
        for camera_ix, tracking_2d_data_interface in enumerate(
            tracking_2d_data_interfaces
        ):
            for id, metadata in tracking_2d_data_interface.trackingIDmetadata.items():
                frame_ix_length = metadata.frame_ix_max - metadata.frame_ix_min
                accum_track_length = metadata.accum_track_length
                self._logger.info(
                    f"select_2dbbox_tracking_results: camera_ix={camera_ix}, id={id}, frame_ix_length={frame_ix_length}, accum_track_length={accum_track_length}",
                )
                if frame_ix_length < 10:
                    tracking_2d_data_interfaces[camera_ix].trackingIDmetadata[
                        id
                    ].is_alive = False
                if accum_track_length < 50:
                    tracking_2d_data_interfaces[camera_ix].trackingIDmetadata[
                        id
                    ].is_alive = False

        return tracking_2d_data_interfaces

    def validate_2dbbox_tracking_results(
        self, tracking_2d_data_interfaces: list[Tracking2dDataInterface]
    ) -> list[bool]:
        # 2D bboxトラッキング結果の妥当性判定
        # ここでは、トラッキング結果の数が閾値を満たしているかどうかを判定する。必要に応じて、トラッキング結果の属性やスコアなども考慮することができる。

        results: list[bool] = []
        for tracking_2d_data_interface in tracking_2d_data_interfaces:
            count = sum(
                1
                for metadata in tracking_2d_data_interface.trackingIDmetadata.values()
                if metadata.is_alive
            )
            results.append(count >= self.THRESH_2DBBOX_TRACKING_IDCOUNT)
        return results

    def validate_tracked_bboxes(
        self,
        tracking_3d_data_interface: Tracking3dDataInterface,
        tracking_2d_data_interfaces: list[Tracking2dDataInterface],
    ) -> list[int]:
        tracking_3d_data_interface = self.select_3dbbox_tracking_results(
            tracking_3d_data_interface
        )
        if self.validate_3dbbox_tracking_results(tracking_3d_data_interface) is False:
            return [4 for _ in range(self._evaluation_camera_count)]

        tracking_2d_data_interfaces = self.select_2dbbox_tracking_results(
            tracking_2d_data_interfaces
        )
        return [
            0 if validation_result else 5
            for validation_result in self.validate_2dbbox_tracking_results(
                tracking_2d_data_interfaces
            )
        ]

    def project_3dbbox_core(
        self,
        bbox3d: NDArray[np.float32],
        camera_index: int,
        require_points_in_image: bool = False,
    ) -> NDArray[np.float32] | None:
        # 3D bboxをカメラ座標系に変換し、2D bboxに投影する。投影結果がカメラフレーム内に収まるかどうかを判定する。
        # require_points_in_image=True の場合は「8頂点すべてが画像内」を可視条件にする。
        camera_extrinsics = self.rtvec_mat[camera_index][2]  # 2: convmat
        camera_intrinsics = self._evaluation_intrinsics

        # bbox3dとbbox2dの重なりを計算するために、まずbbox3dを2Dに投影する
        bbox3d_corners = np.array(
            [
                [bbox3d[0], bbox3d[1], bbox3d[2]],  # min corner
                [bbox3d[0], bbox3d[1], bbox3d[5]],  # min x, min y, max z
                [bbox3d[0], bbox3d[4], bbox3d[2]],  # min x, max y, min z
                [bbox3d[0], bbox3d[4], bbox3d[5]],  # min x, max y, max z
                [bbox3d[3], bbox3d[1], bbox3d[2]],  # max x, min y, min z
                [bbox3d[3], bbox3d[1], bbox3d[5]],  # max x, min y, max z
                [bbox3d[3], bbox3d[4], bbox3d[2]],  # max x, max y, min z
                [bbox3d[3], bbox3d[4], bbox3d[5]],  # max corner
            ],
            dtype=np.float32,
        )
        # 3D bboxの8つの角をカメラ座標系に変換
        bbox3d_corners_homogeneous = np.hstack(
            (bbox3d_corners, np.ones((bbox3d_corners.shape[0], 1), dtype=np.float32))
        )  # (8, 4)
        bbox3d_corners_camera = (camera_extrinsics @ bbox3d_corners_homogeneous.T).T[
            :, :3
        ]
        if np.any(bbox3d_corners_camera[:, 2] <= 0):
            # カメラ前方にないbbox3dは、評価値をNoneにする。
            return None

        # 3D bboxの8つの角を画像平面に投影
        bbox3d_corners_image = (camera_intrinsics @ bbox3d_corners_camera.T).T
        bbox3d_corners_image = (
            bbox3d_corners_image[:, :2] / bbox3d_corners_image[:, 2:3]
        )
        if require_points_in_image:
            # 可視判定用: 8頂点すべてが画像内にある場合のみ有効とする。
            x_in = (bbox3d_corners_image[:, 0] >= 0.0) & (
                bbox3d_corners_image[:, 0] <= float(self.width - 1)
            )
            y_in = (bbox3d_corners_image[:, 1] >= 0.0) & (
                bbox3d_corners_image[:, 1] <= float(self.height - 1)
            )
            if not np.all(x_in & y_in):
                return None

        return np.array(
            [
                np.min(bbox3d_corners_image[:, 0]),
                np.min(bbox3d_corners_image[:, 1]),
                np.max(bbox3d_corners_image[:, 0]),
                np.max(bbox3d_corners_image[:, 1]),
            ],
            dtype=np.float32,
        )

    @staticmethod
    def _shrink_bbox2d(
        bbox2d: NDArray[np.float32], factor: float
    ) -> NDArray[np.float32]:
        # bbox format: (x1, y1, x2, y2)。中心を保ったまま縦横をfactor倍する。
        center_x = (bbox2d[0] + bbox2d[2]) / 2.0
        center_y = (bbox2d[1] + bbox2d[3]) / 2.0
        half_width = (bbox2d[2] - bbox2d[0]) * factor / 2.0
        half_height = (bbox2d[3] - bbox2d[1]) * factor / 2.0
        return np.array(
            [
                center_x - half_width,
                center_y - half_height,
                center_x + half_width,
                center_y + half_height,
            ],
            dtype=np.float32,
        )

    @classmethod
    def _passes_center_diff_gate(
        cls,
        bbox_a: NDArray[np.float32],
        bbox_b: NDArray[np.float32],
    ) -> bool:
        # 縮小後の縦横長さの平均でX/Y中心差を正規化し、そのユークリッド距離を閾値判定する。
        shrunk_a = cls._shrink_bbox2d(bbox_a, BBOX_SHRINK_FACTOR)
        shrunk_b = cls._shrink_bbox2d(bbox_b, BBOX_SHRINK_FACTOR)
        width_a, height_a = shrunk_a[2] - shrunk_a[0], shrunk_a[3] - shrunk_a[1]
        width_b, height_b = shrunk_b[2] - shrunk_b[0], shrunk_b[3] - shrunk_b[1]
        center_a = (
            (shrunk_a[0] + shrunk_a[2]) / 2.0,
            (shrunk_a[1] + shrunk_a[3]) / 2.0,
        )
        center_b = (
            (shrunk_b[0] + shrunk_b[2]) / 2.0,
            (shrunk_b[1] + shrunk_b[3]) / 2.0,
        )
        average_width = max((width_a + width_b) / 2.0, 1e-6)
        average_height = max((height_a + height_b) / 2.0, 1e-6)
        normalized_x = (center_a[0] - center_b[0]) / average_width
        normalized_y = (center_a[1] - center_b[1]) / average_height
        return (
            float(np.hypot(normalized_x, normalized_y))
            <= BBOX_CENTER_DIFF_RATIO_THRESHOLD
        )

    def evaluate_bbox_overlap_scenedesc(
        self,
        bbox3d: NDArray[np.float32],
        bbox2d: NDArray[np.float32],
        camera_index: int,
    ) -> float | None:
        # scenedescと同じ処理で、3D bboxが2D bboxにどれだけ重なっているかを評価する。scenedescのintegrate2d3d_calibcheck(校正評価用にオーバーライドしたもの)関数を呼び出す。
        # 元の評価用関数トップは共有メモリとの依存性があり呼べない。

        # 中心座標ずれゲート：縮小後bboxの中心差(正規化)が閾値超過なら、scenedescの結果によらず不一致とする。
        projected_bbox2d = self.project_3dbbox_core(bbox3d, camera_index)
        if projected_bbox2d is not None and not self._passes_center_diff_gate(
            projected_bbox2d, bbox2d
        ):
            return 0.0

        candidate_minmax_list: list[NDArray[np.float32]] = []
        candidate_corners_list: list[NDArray[np.float32]] = []
        for offset_x, offset_y in self.VIRTUAL_BBOX_XY_OFFSETS:
            # bbox3d: [minx, miny, minz, maxx, maxy, maxz]
            candidate_bbox3d = bbox3d.copy()
            candidate_bbox3d[[0, 3]] += offset_x
            candidate_bbox3d[[1, 4]] += offset_y
            # SceneDesc側のminmax形式: [x_min, x_max, y_min, y_max, z_min, z_max]
            candidate_minmax = np.array(
                [
                    candidate_bbox3d[0],
                    candidate_bbox3d[3],
                    candidate_bbox3d[1],
                    candidate_bbox3d[4],
                    candidate_bbox3d[2],
                    candidate_bbox3d[5],
                ],
                dtype=np.float32,
            )
            candidate_minmax_list.append(candidate_minmax)
            candidate_corners_list.append(
                np.array(
                    [
                        [candidate_minmax[0], candidate_minmax[2], candidate_minmax[4]],
                        [candidate_minmax[1], candidate_minmax[2], candidate_minmax[4]],
                        [candidate_minmax[0], candidate_minmax[3], candidate_minmax[4]],
                        [candidate_minmax[1], candidate_minmax[3], candidate_minmax[4]],
                        [candidate_minmax[0], candidate_minmax[2], candidate_minmax[5]],
                        [candidate_minmax[1], candidate_minmax[2], candidate_minmax[5]],
                        [candidate_minmax[0], candidate_minmax[3], candidate_minmax[5]],
                        [candidate_minmax[1], candidate_minmax[3], candidate_minmax[5]],
                    ],
                    dtype=np.float32,
                )
            )

        # YOLO形式変換 bbox2d: (x1, y1, x2, y2)[pixels] -> [image_h_min, image_w_min, image_h_max, image_w_max](0~1で正規化された位置)
        # YOLOパラメータについて: この関数が呼ばれている時点で人検知bboxであることは確定。
        # 実bboxを候補0、前後左右の仮想bboxを候補1以降として一括比較する。
        bbox_class = self.scenedesc_calibcheck.integrate2d3d_calibcheck(
            rvec=self.rtvec_mat[camera_index][0],  # 0: rvec
            tvec=self.rtvec_mat[camera_index][1],  # 1: tvec
            boxpoints=np.concatenate(candidate_corners_list),
            bbox2d=np.array(
                [
                    [
                        bbox2d[1] / self.height,
                        bbox2d[0] / self.width,
                        bbox2d[3] / self.height,
                        bbox2d[2] / self.width,
                    ]
                ],
                dtype=np.float32,
            ),
            minmax3ds=np.stack(candidate_minmax_list),
            yolo_classes=np.array([0], dtype=np.int32),
            n_clusters=len(self.VIRTUAL_BBOX_XY_OFFSETS),
            bbox2d_detection_count=1,
            method="center",
        )
        selected_candidate_index = next(iter(bbox_class), None)
        if camera_index < len(self._virtual_bbox_debug_counts):
            camera_counts = self._virtual_bbox_debug_counts[camera_index]
            camera_counts["comparison_count"] += 1
            if selected_candidate_index is not None:
                selected_name = self.VIRTUAL_BBOX_CANDIDATE_NAMES[
                    selected_candidate_index
                ]
                camera_counts["selected_candidate_counts"][selected_name] += 1
                if selected_candidate_index != 0:
                    camera_counts["virtual_bbox_win_count"] += 1
        return 1.0 if bbox_class.get(0) == "HUMAN" else 0.0

    @staticmethod
    def _has_positive_2d_intersection(
        bbox_a: NDArray[np.float32],
        bbox_b: NDArray[np.float32],
    ) -> bool:
        # bbox format: (x1, y1, x2, y2)
        ax1, ay1 = min(bbox_a[0], bbox_a[2]), min(bbox_a[1], bbox_a[3])
        ax2, ay2 = max(bbox_a[0], bbox_a[2]), max(bbox_a[1], bbox_a[3])
        bx1, by1 = min(bbox_b[0], bbox_b[2]), min(bbox_b[1], bbox_b[3])
        bx2, by2 = max(bbox_b[0], bbox_b[2]), max(bbox_b[1], bbox_b[3])
        return (
            min(ax2, bx2) - max(ax1, bx1) > 0.0 and min(ay2, by2) - max(ay1, by1) > 0.0
        )

    def evaluate_2d3d(
        self,
        tracking_3d_data_interface: Tracking3dDataInterface,
        tracking_2d_data_interfaces: list[Tracking2dDataInterface],
        zvalues: tuple[float, float],
    ) -> tuple[list[float], list[int]]:
        # 2D3D評価計算のメイン関数
        # 欠番フレームを埋めずに、各軌跡の実在フレームだけを扱う。
        # 各3Dフレームに対して2D軌跡の評価をOR集約し、同一フレームの重複Trueは1回として数える。

        mean_scores_per_camera = [0.0 for _ in range(self._evaluation_camera_count)]
        reason_camera_notvalid = [0 for _ in range(self._evaluation_camera_count)]
        overlap_debug_by_camera: dict[int, dict[str, object]] = {
            camera_ix: {"overlap_frame_ix_set": set(), "overlap_events": []}
            for camera_ix in range(self._evaluation_camera_count)
        }
        eval_trace_by_camera: dict[int, dict[int, list[dict[str, object]]]] = {}
        self._virtual_bbox_debug_counts = [
            {
                "comparison_count": 0,
                "virtual_bbox_win_count": 0,
                "selected_candidate_counts": dict.fromkeys(
                    self.VIRTUAL_BBOX_CANDIDATE_NAMES, 0
                ),
            }
            for _ in range(self._evaluation_camera_count)
        ]
        self._evaluation_metric_debug = []

        for camera_ix, tracking_2d_data_interface in enumerate(
            tracking_2d_data_interfaces
        ):
            # カメラ単位で、妥当性フラグと2種類のスコア集計器を初期化する。
            camera_has_inframe = False
            camera_has_timematch = False
            camera_has_valid = False
            camera_numerator = 0
            camera_denominator = 0
            camera_sampled_count = 0
            # 旧寄り指標用: 比較できたフレームのmax(score)を足し上げる。
            camera_legacy_numerator = 0.0
            camera_legacy_denominator = 0
            alive_2d_ids = [
                track_id
                for track_id, metadata in tracking_2d_data_interface.trackingIDmetadata.items()
                if metadata.is_alive
            ]

            for (
                track_id_3d,
                metadata_3d,
            ) in tracking_3d_data_interface.trackingIDmetadata.items():
                if not metadata_3d.is_alive:
                    continue
                bboxlog_3d = tracking_3d_data_interface.trackingIDbboxlog.get(
                    track_id_3d, []
                )
                if len(bboxlog_3d) == 0:
                    continue

                frame_ix_3d: list[int] = []
                bbox3d_full_list: list[NDArray[np.float32]] = []
                visible_mask: list[bool] = []
                projected_bbox2d_list: list[NDArray[np.float32] | None] = []
                for frame_ix, bbox3d_xy in bboxlog_3d:
                    bbox3d_full = np.array(
                        [
                            bbox3d_xy[0],
                            bbox3d_xy[1],
                            zvalues[0],
                            bbox3d_xy[2],
                            bbox3d_xy[3],
                            zvalues[1],
                        ],
                        dtype=np.float32,
                    )
                    projected_bbox2d = self.project_3dbbox_core(
                        bbox3d_full,
                        camera_ix,
                        require_points_in_image=True,
                    )
                    if self._should_trace_eval_frame(frame_ix):
                        self._append_trace_event(
                            eval_trace_by_camera,
                            camera_ix,
                            int(frame_ix),
                            {
                                "stage": "project_3d_to_2d",
                                "id3d": int(track_id_3d),
                                "bbox3d": bbox3d_full.tolist(),
                                "projected_bbox2d": (
                                    projected_bbox2d.tolist()
                                    if projected_bbox2d is not None
                                    else None
                                ),
                                "is_visible_in_image": projected_bbox2d is not None,
                            },
                        )
                    frame_ix_3d.append(frame_ix)
                    bbox3d_full_list.append(bbox3d_full)
                    projected_bbox2d_list.append(projected_bbox2d)
                    visible_mask.append(projected_bbox2d is not None)

                visible_frame_to_local_ix = {
                    frame_ix_3d[local_ix]: local_ix
                    for local_ix, is_visible in enumerate(visible_mask)
                    if is_visible
                }
                if len(visible_frame_to_local_ix) == 0:
                    continue

                visible_local_ixs = [
                    local_ix
                    for local_ix, is_visible in enumerate(visible_mask)
                    if is_visible
                ]
                if self.EVAL_FRAME_STRIDE > 1:
                    sampled_local_ixs = {
                        local_ix
                        for sample_index, local_ix in enumerate(visible_local_ixs)
                        if sample_index % self.EVAL_FRAME_STRIDE == 0
                    }
                    sampled_local_ixs.add(visible_local_ixs[-1])
                else:
                    sampled_local_ixs = set(visible_local_ixs)

                if len(sampled_local_ixs) == 0:
                    continue

                camera_has_inframe = True
                camera_denominator += len(sampled_local_ixs)
                camera_sampled_count += len(sampled_local_ixs)
                # strict指標用: フレームごとのヒット有無(OR集約)。
                hit_mask = [False for _ in frame_ix_3d]
                # legacy寄り指標用: 同一フレーム内で2D IDをまたいだ最大スコアを保持。
                legacy_frame_best_score: list[float | None] = [
                    None for _ in frame_ix_3d
                ]
                visible_frame_min = min(visible_frame_to_local_ix)
                visible_frame_max = max(visible_frame_to_local_ix)

                for track_id_2d in alive_2d_ids:
                    metadata_2d = tracking_2d_data_interface.trackingIDmetadata[
                        track_id_2d
                    ]
                    if (
                        metadata_2d.frame_ix_max < visible_frame_min
                        or metadata_2d.frame_ix_min > visible_frame_max
                    ):
                        continue
                    for (
                        frame_ix,
                        bbox2d,
                    ) in tracking_2d_data_interface.trackingIDbboxlog.get(
                        track_id_2d, []
                    ):
                        local_ix_3d = visible_frame_to_local_ix.get(frame_ix)
                        if local_ix_3d is None:
                            if self._should_trace_eval_frame(frame_ix):
                                self._append_trace_event(
                                    eval_trace_by_camera,
                                    camera_ix,
                                    int(frame_ix),
                                    {
                                        "stage": "match_2d_to_3d",
                                        "id3d": int(track_id_3d),
                                        "id2d": int(track_id_2d),
                                        "decision": "skip_no_visible_3d_frame",
                                        "bbox2d": np.array(
                                            bbox2d, dtype=np.float32
                                        ).tolist(),
                                    },
                                )
                            continue
                        camera_has_timematch = True
                        if local_ix_3d not in sampled_local_ixs:
                            if self._should_trace_eval_frame(frame_ix):
                                self._append_trace_event(
                                    eval_trace_by_camera,
                                    camera_ix,
                                    int(frame_ix),
                                    {
                                        "stage": "sampling",
                                        "id3d": int(track_id_3d),
                                        "id2d": int(track_id_2d),
                                        "decision": "skip_by_stride",
                                        "local_ix_3d": int(local_ix_3d),
                                    },
                                )
                            continue
                        if hit_mask[local_ix_3d]:
                            if self._should_trace_eval_frame(frame_ix):
                                self._append_trace_event(
                                    eval_trace_by_camera,
                                    camera_ix,
                                    int(frame_ix),
                                    {
                                        "stage": "hit_mask",
                                        "id3d": int(track_id_3d),
                                        "id2d": int(track_id_2d),
                                        "decision": "skip_already_hit",
                                        "local_ix_3d": int(local_ix_3d),
                                    },
                                )
                            continue
                        bbox3d_full = bbox3d_full_list[local_ix_3d]
                        projected_bbox2d = projected_bbox2d_list[local_ix_3d]
                        assert projected_bbox2d is not None
                        bbox2d_array = np.array(bbox2d, dtype=np.float32)
                        if BBOX_SHRINK_FACTOR < 1.0:
                            # 縮小後のbboxを使う場合は、bbox2dも縮小する。
                            bbox2d_array = self._shrink_bbox2d(
                                bbox2d_array, BBOX_SHRINK_FACTOR
                            )
                        has_overlap = self._has_positive_2d_intersection(
                            projected_bbox2d, bbox2d_array
                        )
                        if self._should_trace_eval_frame(frame_ix):
                            self._append_trace_event(
                                eval_trace_by_camera,
                                camera_ix,
                                int(frame_ix),
                                {
                                    "stage": "overlap_gate",
                                    "id3d": int(track_id_3d),
                                    "id2d": int(track_id_2d),
                                    "local_ix_3d": int(local_ix_3d),
                                    "bbox3d": bbox3d_full.tolist(),
                                    "bbox2d": bbox2d_array.tolist(),
                                    "projected_bbox2d": projected_bbox2d.tolist(),
                                    "has_overlap": bool(has_overlap),
                                },
                            )
                        if not has_overlap:
                            camera_has_valid = True
                            if legacy_frame_best_score[local_ix_3d] is None:
                                legacy_frame_best_score[local_ix_3d] = 0.0
                            if self._should_trace_eval_frame(frame_ix):
                                self._append_trace_event(
                                    eval_trace_by_camera,
                                    camera_ix,
                                    int(frame_ix),
                                    {
                                        "stage": "decision",
                                        "id3d": int(track_id_3d),
                                        "id2d": int(track_id_2d),
                                        "decision": "fast_reject_no_overlap",
                                    },
                                )
                            continue
                        if self.DEBUG_CALIBCHECK_ENABLED:
                            overlap_frame_ix_set = overlap_debug_by_camera[camera_ix][
                                "overlap_frame_ix_set"
                            ]
                            overlap_events = overlap_debug_by_camera[camera_ix][
                                "overlap_events"
                            ]
                            assert isinstance(overlap_frame_ix_set, set)
                            assert isinstance(overlap_events, list)
                            overlap_frame_ix_set.add(frame_ix)
                            overlap_events.append(
                                {
                                    "frame_ix": int(frame_ix),
                                    "id3d": int(track_id_3d),
                                    "id2d": int(track_id_2d),
                                    "bbox3d": bbox3d_full.tolist(),
                                    "bbox2d": bbox2d_array.tolist(),
                                    "bbox3d_projected_2d": projected_bbox2d.tolist(),
                                }
                            )
                        score = self.evaluate_bbox_overlap_scenedesc(
                            bbox3d_full,
                            bbox2d_array,
                            camera_ix,
                        )
                        if self._should_trace_eval_frame(frame_ix):
                            self._append_trace_event(
                                eval_trace_by_camera,
                                camera_ix,
                                int(frame_ix),
                                {
                                    "stage": "scenedesc_score",
                                    "id3d": int(track_id_3d),
                                    "id2d": int(track_id_2d),
                                    "score": None if score is None else float(score),
                                },
                            )
                        if score is None:
                            continue
                        camera_has_valid = True
                        if legacy_frame_best_score[local_ix_3d] is None:
                            legacy_frame_best_score[local_ix_3d] = score
                        else:
                            legacy_frame_best_score[local_ix_3d] = max(
                                legacy_frame_best_score[local_ix_3d], score
                            )
                        if score > 0.0:
                            hit_mask[local_ix_3d] = True
                            if self._should_trace_eval_frame(frame_ix):
                                self._append_trace_event(
                                    eval_trace_by_camera,
                                    camera_ix,
                                    int(frame_ix),
                                    {
                                        "stage": "hit_mask",
                                        "id3d": int(track_id_3d),
                                        "id2d": int(track_id_2d),
                                        "decision": "set_hit_true",
                                    },
                                )

                camera_numerator += sum(
                    1
                    for local_ix, is_hit in enumerate(hit_mask)
                    if is_hit and local_ix in sampled_local_ixs
                )
                camera_legacy_numerator += sum(
                    best_score
                    for local_ix, best_score in enumerate(legacy_frame_best_score)
                    if local_ix in sampled_local_ixs and best_score is not None
                )
                camera_legacy_denominator += sum(
                    1
                    for local_ix, best_score in enumerate(legacy_frame_best_score)
                    if local_ix in sampled_local_ixs and best_score is not None
                )

            if not camera_has_inframe:
                reason_camera_notvalid[camera_ix] = 9
            elif not camera_has_timematch:
                reason_camera_notvalid[camera_ix] = 10
            elif not camera_has_valid:
                reason_camera_notvalid[camera_ix] = 11
            elif self.USE_LEGACY_LIKE_METRIC:
                mean_scores_per_camera[camera_ix] = (
                    camera_legacy_numerator / camera_legacy_denominator
                    if camera_legacy_denominator > 0
                    else 0.0
                )
            else:
                mean_scores_per_camera[camera_ix] = (
                    camera_numerator / camera_denominator
                    if camera_denominator > 0
                    else 0.0
                )

            strict_hit_rate_score = (
                camera_numerator / camera_denominator if camera_denominator > 0 else 0.0
            )
            legacy_like_score = (
                camera_legacy_numerator / camera_legacy_denominator
                if camera_legacy_denominator > 0
                else 0.0
            )
            self._logger.info(
                f"evaluate_2d3d camera={camera_ix}: numerator={camera_numerator}, "
                f"denominator={camera_denominator}, "
                f"legacy_numerator={camera_legacy_numerator}, "
                f"legacy_denominator={camera_legacy_denominator}, "
                f"sampled_count={camera_sampled_count}, "
                f"strict_hit_rate={strict_hit_rate_score}, "
                f"legacy_like_score={legacy_like_score}, "
                f"score={mean_scores_per_camera[camera_ix]}, "
                f"use_legacy_like={self.USE_LEGACY_LIKE_METRIC}, "
                f"reason={reason_camera_notvalid[camera_ix]}",
            )
            self._evaluation_metric_debug.append(
                {
                    "numerator": camera_numerator,
                    "denominator": camera_denominator,
                    "strict_hit_rate": strict_hit_rate_score,
                    "legacy_numerator": camera_legacy_numerator,
                    "legacy_denominator": camera_legacy_denominator,
                    "legacy_like_score": legacy_like_score,
                    "selected_score": mean_scores_per_camera[camera_ix],
                    "use_legacy_like_metric": self.USE_LEGACY_LIKE_METRIC,
                }
            )
            self._logger.info(
                "evaluate_2d3d virtual bbox diagnostics "
                f"camera={camera_ix}: {self._virtual_bbox_debug_counts[camera_ix]}",
            )

        if self.DEBUG_CALIBCHECK_ENABLED:
            serializable_overlap: dict[int, dict[str, object]] = {}
            for camera_ix, camera_debug in overlap_debug_by_camera.items():
                overlap_frame_ix_set = camera_debug["overlap_frame_ix_set"]
                assert isinstance(overlap_frame_ix_set, set)
                serializable_overlap[camera_ix] = {
                    "overlap_frame_ix_list": sorted(overlap_frame_ix_set),
                    "overlap_events": camera_debug["overlap_events"],
                }
            self._debug_eval_info = {
                "overlap_debug_by_camera": serializable_overlap,
                "eval_trace_by_camera": eval_trace_by_camera,
                "eval_trace_config": {
                    "enabled": self.DEBUG_EVAL_TRACE_ENABLED,
                    "all_frames": self.DEBUG_EVAL_TRACE_ALL_FRAMES,
                    "range_start": int(self.DEBUG_EVAL_TRACE_RANGE_START),
                    "range_end": int(self.DEBUG_EVAL_TRACE_RANGE_END),
                    "target_frames": sorted(self.DEBUG_EVAL_TRACE_TARGET_FRAMES),
                },
                "video_paths": self._debug_video_paths,
                "virtual_bbox_debug_counts": self._virtual_bbox_debug_counts,
            }

        return mean_scores_per_camera, reason_camera_notvalid

    def judge_calibration_result(
        self, evaluation_statistics: list[float], threshold: float = 0.1
    ) -> list[bool]:
        return [score >= threshold for score in evaluation_statistics]

    def data_evaluation_process(self) -> tuple[list[int], list[bool]]:
        # 初期は全部OK 無効になったものは理由番号を入れて以降実行しない。
        # (1: 情報取得時エラー), 2: 3D bboxログが不正、3: 2D bboxログが不正、4: 3D bboxトラッキング結果が不正、5: 2D bboxトラッキング結果が不正
        # 6: 2D3D評価結果が不正、7: 評価統計計算が不正、8: 校正判定が不正
        self._logger.info("validate_recorded_bbox_logs called")
        reason_camera_notvalid = self.validate_recorded_bbox_logs()
        self._logger.info(
            f"validate_recorded_bbox_logs results: {reason_camera_notvalid}",
        )
        camera_evaluation_results = [
            False for _ in range(self._evaluation_camera_count)
        ]  # True: 精度OK False: 精度NG
        if all(reason_camera_notvalid):
            return reason_camera_notvalid, camera_evaluation_results

        self._logger.info("track_3dbbox called")
        tracking_3d_data_interface = self.track_3dbbox()
        self._logger.info("track_2dbbox called")
        tracking_2d_data_interfaces = self.track_2dbbox()
        self._logger.info("validate_tracked_bboxes called")
        tracking_reasons = self.validate_tracked_bboxes(
            tracking_3d_data_interface,
            tracking_2d_data_interfaces,
        )
        reason_camera_notvalid = [
            current_reason if current_reason else tracking_reasons[camera_ix]
            for camera_ix, current_reason in enumerate(reason_camera_notvalid)
        ]
        if all(reason_camera_notvalid):
            return reason_camera_notvalid, camera_evaluation_results

        if self.DEBUG_CALIBCHECK_ENABLED:
            debug_store("tracking_3d_data_interface", tracking_3d_data_interface)
            for camera_ix, tracking_2d_data_interface in enumerate(
                tracking_2d_data_interfaces
            ):
                debug_store(
                    f"tracking_2d_data_interface_{camera_ix}",
                    tracking_2d_data_interface,
                )
            debug_force_snapshot()

        evaluation_statistics, evaluation_reasons = self.evaluate_2d3d(
            tracking_3d_data_interface,
            tracking_2d_data_interfaces,
            zvalues=self.EVAL_ZVALUES,
        )
        self._logger.info(
            f"evaluate_2d3d called, {evaluation_statistics = }, "
            f"{evaluation_reasons = }",
        )
        if self.DEBUG_CALIBCHECK_ENABLED:
            self._debug_eval_info["evaluation_statistics"] = evaluation_statistics
            self._debug_eval_info["reason_camera_notvalid"] = evaluation_reasons
            self._debug_eval_info["camera_calib_files"] = [
                str(file_path)
                for file_path in self.calibcheck2d3d_conf.camera_calib_files
            ]
            self._debug_eval_info["camera_intrinsics_path"] = str(
                self.calibcheck2d3d_conf.camera_intrinsics_path
            )
            self._debug_eval_info["image_size"] = {
                "width": int(self.width),
                "height": int(self.height),
            }
            self._debug_eval_info["eval_zvalues"] = [
                float(self.EVAL_ZVALUES[0]),
                float(self.EVAL_ZVALUES[1]),
            ]
            self._debug_eval_info["new_axis_mode"] = bool(
                self.calibcheck2d3d_conf.new_axis_mode
            )
            self._debug_eval_info["tracking_3d_bboxlog"] = {
                int(track_id): [
                    (int(frame_ix), [float(value) for value in bbox2d])
                    for frame_ix, bbox2d in bboxlog
                ]
                for track_id, bboxlog in tracking_3d_data_interface.trackingIDbboxlog.items()
            }
            self._debug_eval_info["tracking_3d_metadata"] = {
                int(track_id): {
                    "is_alive": bool(metadata.is_alive),
                    "frame_ix_min": int(metadata.frame_ix_min),
                    "frame_ix_max": int(metadata.frame_ix_max),
                    "frame_ix_lastmove": int(metadata.frame_ix_lastmove),
                    "accum_track_length": float(metadata.accum_track_length),
                    "workarea_count": float(metadata.workarea_count),
                }
                for track_id, metadata in tracking_3d_data_interface.trackingIDmetadata.items()
            }
            self._debug_eval_info["tracking_3d_proximity_warnings"] = (
                self._3d_track_proximity_warnings
            )
            self._debug_eval_info["tracking_2d_bboxlog"] = {
                int(camera_ix): {
                    int(track_id): [
                        (int(frame_ix), [float(value) for value in bbox2d])
                        for frame_ix, bbox2d in bboxlog
                    ]
                    for track_id, bboxlog in tracking_2d_data_interface.trackingIDbboxlog.items()
                }
                for camera_ix, tracking_2d_data_interface in enumerate(
                    tracking_2d_data_interfaces
                )
            }
            self._dump_debug_eval_info()
        evaluated_results = self.judge_calibration_result(
            evaluation_statistics,
            threshold=self.calibcheck2d3d_conf.score_value_threshold,
        )
        self._logger.info("judge_calibration_result called")
        for camera_ix, current_reason in enumerate(reason_camera_notvalid):
            if current_reason == 0:
                reason_camera_notvalid[camera_ix] = evaluation_reasons[camera_ix]
                camera_evaluation_results[camera_ix] = evaluated_results[camera_ix]

        return reason_camera_notvalid, camera_evaluation_results

    def camera_evaluation_results_to_monitor(
        self,
        monitor: CalibrationUIGodot,
        reason_camera_notvalid: list[int],
        camera_evaluation_results: list[bool],
    ) -> None:
        for camera_ix, (reason, result) in enumerate(
            zip(reason_camera_notvalid, camera_evaluation_results, strict=False)
        ):
            status = self._calibcheck_status_diagnosis.diagnose(
                reason_code=reason,
                calibration_is_acceptable=result,
            )
            monitor.set_camera_calibcheck_status(camera_ix, status)

    def pre_app_loopmain(
        self,
        monitor: CalibrationUIGodot,
        sec: SharedExcepts,
        sac: SharedAppConfig,
    ) -> None:
        self.debug_index = 0
        self._logger.info(f"frame index: {self.debug_index}")

        self.input_settings()

        # CalibStatus:D1/D2 もう一度送信
        monitor.set_status_calibcommon(CalibrationCommonStatus.RUNNING)
        monitor.set_dummydata(
            enable_systemerrorflag=True,
            enable_errorflag=True,
            overwrite_checkresult=True,
            enable_yawangle=True,
        )
        monitor.transmit_setdata(
            sec=sec, ref_t=None, is_firstframe=True, mmap_erase_rest=True
        )  # GUI共有メモリ書き込み。未書き込みエリアを初期化する(開始時1回だけ）

        self.checked_points3d = []
        self.checked_points3d_score = []
        self.checked_points2d = []
        self.checked_points2d_score = []

        self.camera_scores_rawdata: list[list[float]] = [
            [] for _ in range(self.calibcheck2d3d_conf.camera_count)
        ]  # カメラごとのbbox評価値のリスト
        self.frame_info = []
        self._debug_video_writers = [
            None for _ in range(self.calibcheck2d3d_conf.camera_count)
        ]
        self._debug_video_paths = [
            "" for _ in range(self.calibcheck2d3d_conf.camera_count)
        ]
        self._debug_eval_info = {}
        self.read_count = 0

    def input_data_diagnosis(
        self,
        camera_datalist: list[tuple[NDArray[np.uint8], int, float] | None],
        lidar_datalist: list[tuple[NDArray[np.float64], int, float] | None],
        can_data: object,
    ) -> bool:
        invalid_data_input = self._ser.state_errors_D[
            StateErrorDIndex.INVALID_DATA_INPUT
        ]
        result, failsafe_result = invalid_data_input.errors_diagnosis(
            (camera_datalist, lidar_datalist, can_data)
        )
        invalid_data_input.log_output(
            result, failsafe_result, StateErrorDIndex.INVALID_DATA_INPUT
        )
        if result == ResultDiagnosis.DETECTION:
            return True

        # NOTE: 静的解析の指摘が出ているが、手前の不正データ入力の診断でNoneで無いことは担保出来ている
        images = tuple(camera_data[0] for camera_data in camera_datalist)
        min_xyz_columns = 3
        # NOTE: 静的解析の指摘が出ているが、手前の不正データ入力の診断でNoneで無いことは担保出来ている
        pcds_point_cloud = tuple(
            lidar_data[0][:, :min_xyz_columns] for lidar_data in lidar_datalist
        )
        array_shape_error = self._ser.state_errors_D[StateErrorDIndex.ARRAY_SHAPE_ERROR]
        result, failsafe_result = array_shape_error.errors_diagnosis(
            ("images", images),
            ("pcds_point_cloud", pcds_point_cloud),
        )
        array_shape_error.log_output(
            result, failsafe_result, StateErrorDIndex.ARRAY_SHAPE_ERROR
        )
        return result == ResultDiagnosis.DETECTION

    def _draw_evaluation_bboxes(
        self,
        monitor: CalibrationUIGodot,
        multi_minmax: NDArray[np.float64],
    ) -> None:
        edge_pairs = [
            (0, 1),
            (1, 3),
            (3, 2),
            (2, 0),
            (4, 5),
            (5, 7),
            (7, 6),
            (6, 4),
            (0, 4),
        ]

        for camera_ix, frame in enumerate(monitor.cameradata):
            if frame is None:
                continue

            for bbox3d in multi_minmax:
                vertices3d = np.array(
                    [
                        [bbox3d[0], bbox3d[2], bbox3d[4]],
                        [bbox3d[1], bbox3d[2], bbox3d[4]],
                        [bbox3d[0], bbox3d[3], bbox3d[4]],
                        [bbox3d[1], bbox3d[3], bbox3d[4]],
                        [bbox3d[0], bbox3d[2], bbox3d[5]],
                        [bbox3d[1], bbox3d[2], bbox3d[5]],
                        [bbox3d[0], bbox3d[3], bbox3d[5]],
                        [bbox3d[1], bbox3d[3], bbox3d[5]],
                    ],
                    dtype=np.float32,
                )
                bbox3d_2d = self.project_3dbbox_core(
                    np.array(
                        [
                            bbox3d[0],
                            bbox3d[2],
                            bbox3d[4],
                            bbox3d[1],
                            bbox3d[3],
                            bbox3d[5],
                        ],
                        dtype=np.float32,
                    ),
                    camera_ix,
                )
                if bbox3d_2d is not None:
                    vertices3d_reproj = cv2.projectPoints(
                        vertices3d,
                        self.rtvec_mat[camera_ix][0],
                        self.rtvec_mat[camera_ix][1],
                        self.ud.ncm1,
                        np.zeros((5, 1), dtype=np.float32),
                    )[0].reshape(-1, 2)
                    boxcolor = (0, 255, 0)
                    for point1_ix, point2_ix in edge_pairs:
                        cv2.line(
                            frame,
                            conv_intarr(vertices3d_reproj[point1_ix]),
                            conv_intarr(vertices3d_reproj[point2_ix]),
                            boxcolor,
                            2,
                        )

                eval_zmin, eval_zmax = self.EVAL_ZVALUES
                vertices3d_eval = np.array(
                    [
                        [bbox3d[0], bbox3d[2], eval_zmin],
                        [bbox3d[1], bbox3d[2], eval_zmin],
                        [bbox3d[0], bbox3d[3], eval_zmin],
                        [bbox3d[1], bbox3d[3], eval_zmin],
                        [bbox3d[0], bbox3d[2], eval_zmax],
                        [bbox3d[1], bbox3d[2], eval_zmax],
                        [bbox3d[0], bbox3d[3], eval_zmax],
                        [bbox3d[1], bbox3d[3], eval_zmax],
                    ],
                    dtype=np.float32,
                )
                bbox3d_2d_eval = self.project_3dbbox_core(
                    np.array(
                        [
                            bbox3d[0],
                            bbox3d[2],
                            eval_zmin,
                            bbox3d[1],
                            bbox3d[3],
                            eval_zmax,
                        ],
                        dtype=np.float32,
                    ),
                    camera_ix,
                )
                if bbox3d_2d_eval is not None:
                    vertices3d_eval_reproj = cv2.projectPoints(
                        vertices3d_eval,
                        self.rtvec_mat[camera_ix][0],
                        self.rtvec_mat[camera_ix][1],
                        self.ud.ncm1,
                        np.zeros((5, 1), dtype=np.float32),
                    )[0].reshape(-1, 2)
                    orange_color = (0, 165, 255)
                    for point1_ix, point2_ix in edge_pairs:
                        cv2.line(
                            frame,
                            conv_intarr(vertices3d_eval_reproj[point1_ix]),
                            conv_intarr(vertices3d_eval_reproj[point2_ix]),
                            orange_color,
                            2,
                        )

                light_blue_color = (255, 255, 0)
                for offset_x, offset_y in self.VIRTUAL_BBOX_XY_OFFSETS:
                    if offset_x == 0.0 and offset_y == 0.0:
                        continue

                    virtual_x_min = bbox3d[0] + offset_x
                    virtual_x_max = bbox3d[1] + offset_x
                    virtual_y_min = bbox3d[2] + offset_y
                    virtual_y_max = bbox3d[3] + offset_y
                    bbox3d_2d_virtual = self.project_3dbbox_core(
                        np.array(
                            [
                                virtual_x_min,
                                virtual_y_min,
                                eval_zmin,
                                virtual_x_max,
                                virtual_y_max,
                                eval_zmax,
                            ],
                            dtype=np.float32,
                        ),
                        camera_ix,
                    )
                    if bbox3d_2d_virtual is None:
                        continue

                    vertices3d_virtual = np.array(
                        [
                            [virtual_x_min, virtual_y_min, eval_zmin],
                            [virtual_x_max, virtual_y_min, eval_zmin],
                            [virtual_x_min, virtual_y_max, eval_zmin],
                            [virtual_x_max, virtual_y_max, eval_zmin],
                            [virtual_x_min, virtual_y_min, eval_zmax],
                            [virtual_x_max, virtual_y_min, eval_zmax],
                            [virtual_x_min, virtual_y_max, eval_zmax],
                            [virtual_x_max, virtual_y_max, eval_zmax],
                        ],
                        dtype=np.float32,
                    )
                    vertices3d_virtual_reproj = cv2.projectPoints(
                        vertices3d_virtual,
                        self.rtvec_mat[camera_ix][0],
                        self.rtvec_mat[camera_ix][1],
                        self.ud.ncm1,
                        np.zeros((5, 1), dtype=np.float32),
                    )[0].reshape(-1, 2)
                    for point1_ix, point2_ix in edge_pairs:
                        cv2.line(
                            frame,
                            conv_intarr(vertices3d_virtual_reproj[point1_ix]),
                            conv_intarr(vertices3d_virtual_reproj[point2_ix]),
                            light_blue_color,
                            2,
                        )

            monitor.cameradata[camera_ix] = frame

    def app_loopmain(
        self,
        fifo_data: FIFOData,
        monitor: CalibrationUIGodot,
        sec: SharedExcepts,
        sac: SharedAppConfig,
    ) -> bool:
        # try:
        # self._logger.info(f"frame index: {self.debug_index}")

        # self.input_settings()

        # # CalibStatus:D1/D2 もう一度送信
        # monitor.set_status_calibcommon(1)
        # monitor.set_dummydata(
        #     enable_systemerrorflag=True,
        #     enable_errorflag=True,
        #     overwrite_checkresult=True,
        #     enable_yawangle=True,
        # )
        # monitor.transmit_setdata(sec=sec, ref_t=None, is_firstframe=True)

        # self.checked_points3d = []
        # self.checked_points3d_score = []
        # self.checked_points2d = []
        # self.checked_points2d_score = []

        # self.camera_scores_rawdata: list[list[float]] = [
        #     [] for _ in range(self.calibcheck2d3d_conf.camera_count)
        # ]  # カメラごとのbbox評価値のリスト
        # try:
        #     readfailed_count = 0
        #     while (
        #         not sec.CalMatGen_ex.IsFinished.value
        #         and sac.read().CalibMode.isRunning2D3Dcheck
        #         and (not sac.read().CalibMode.start2D3DCheckCalc)
        #     ):
        # CalibStatus:D2 現状はstart2D3DCheckCalcが入り次第while loopから抜ける

        camera_datalist, lidar_datalist, can_data, framecounter = fifo_data
        if self.input_data_diagnosis(camera_datalist, lidar_datalist, can_data):
            return False
        monitor.set_yaw(can_data[0])

        self._logger.info(self.proc_lidar1f.__qualname__ + " called")
        multi_minmax = self.proc_lidar1f(lidar_datalist, framecounter, monitor)
        self._logger.info(self.proc_camera1f.__qualname__ + " called")
        yoloresult_whole_list = self.proc_camera1f(
            camera_datalist,
            framecounter,
            monitor,
            sec,
        )
        self._draw_evaluation_bboxes(monitor, multi_minmax)
        self.record_bbox1f(multi_minmax, yoloresult_whole_list)

        monitor.set_dummydata(
            enable_systemerrorflag=True,
            enable_errorflag=True,
            overwrite_checkresult=True,
            enable_yawangle=True,
        )
        self._write_debug_video_frames(monitor, self.debug_index)
        monitor.transmit_setdata(sec=sec, ref_t=self.debug_index)
        self.debug_index += 1
        return True

    # except KeyboardInterrupt as e:
    #     self._logger.info(f"{e}, calibcheck2d3d app_loopmain ended")
    # except Exception as ea:
    #     self._logger.error(
    #         f"app_loopmain: exception! {ea} - \n{traceback.format_exc()}"
    #     )
    #     monitor.set_errorcode_unexpected_exception(True)
    # finally:

    # # CalibStatus:D3
    # monitor.set_status_calibcommon(2)
    # monitor.set_dummydata(
    #     enable_systemerrorflag=True,
    #     enable_errorflag=True,
    #     overwrite_checkresult=True,
    #     enable_yawangle=True,
    # )
    # monitor.transmit_setdata(sec=sec, ref_t=None)
    # for camera_ix, camera_values in enumerate(self.camera_scores_rawdata):
    #     resultstr = "Unknown"
    #     camera_score = 0
    #     if (
    #         len(camera_values)
    #         >= self.calibcheck2d3d_conf.score_accept_count_threshold
    #     ):
    #         camera_score = np.median(camera_values)
    #         if (
    #             camera_score
    #             >= self.calibcheck2d3d_conf.score_value_threshold
    #         ):
    #             resultstr = "OK"
    #         else:
    #             resultstr = "NG"

    #     with open(
    #         self.calibcheck2d3d_conf.resultfiles[camera_ix], "w"
    #     ) as wf:
    #         print(resultstr, file=wf)
    #     self._logger.info(
    #         f"Camera{camera_ix} result: {resultstr}, camera_score:{camera_score}, score_count:{len(camera_values)}",
    #     )

    # with open(
    #     "argus_synchro/calibration_mat_generator_modules/temp/calibcheck2d3d_results.txt",
    #     "w",
    # ) as wf:
    #     print("checked_points3d", file=wf)
    #     for v in self.checked_points3d:
    #         print(v, file=wf)
    #     print("checked_points3d_score", file=wf)
    #     for v in self.checked_points3d_score:
    #         print(v, file=wf)
    #     print("checked_points2d", file=wf)
    #     for v in self.checked_points2d:
    #         print(v, file=wf)
    #     print("checked_points2d_score", file=wf)
    #     for v in self.checked_points2d_score:
    #         print(v, file=wf)

    # except Exception as ea:
    #     self._logger.error(
    #         f"app_loopmain (status D3~): exception! {ea} - \n{traceback.format_exc()}",
    #     )
    #     monitor.set_errorcode_unexpected_exception(True)

    # self.end_wait(sec, sac, monitor)

    def post_app_loopmain(
        self,
        monitor: CalibrationUIGodot,
        sec: SharedExcepts,
        sac: SharedAppConfig,
    ) -> None:
        # CalibStatus:D3
        monitor.set_status_calibcommon(CalibrationCommonStatus.CALCULATING)
        monitor.set_dummydata(
            enable_systemerrorflag=True,
            enable_errorflag=True,
            overwrite_checkresult=True,
            enable_yawangle=True,
        )
        reason_camera_notvalid, camera_evaluation_results = (
            self.data_evaluation_process()
        )
        self.camera_evaluation_results_to_monitor(
            monitor,
            reason_camera_notvalid,
            camera_evaluation_results,
        )
        monitor.transmit_setdata(sec=sec, ref_t=None)
        self._close_debug_video_writers()
        self._dump_frame_info()
        for camera_ix, result in enumerate(camera_evaluation_results):
            resultstr = (
                "Unknown"
                if reason_camera_notvalid[camera_ix]
                else ("OK" if result else "NG")
            )

            with open(self.calibcheck2d3d_conf.resultfiles[camera_ix], "w") as wf:
                print(resultstr, file=wf)
            self._logger.info(
                f"Camera{camera_ix} result: {resultstr}, reason:{reason_camera_notvalid[camera_ix]}",
            )

        result_path: Path = Path(
            path.join(
                self.app_config_calib.default.outputdir_root,
                "calibcheck2d3d_results.txt",
            ),
        )

        with open(
            result_path,
            "w",
        ) as wf:
            print("checked_points3d", file=wf)
            for v in self.checked_points3d:
                print(v, file=wf)
            print("checked_points3d_score", file=wf)
            for v in self.checked_points3d_score:
                print(v, file=wf)
            print("checked_points2d", file=wf)
            for v in self.checked_points2d:
                print(v, file=wf)
            print("checked_points2d_score", file=wf)
            for v in self.checked_points2d_score:
                print(v, file=wf)

    # except Exception as ea:
    #     self._logger.error(
    #         f"app_loopmain (status D3~): exception! {ea} - \n{traceback.format_exc()}",
    #     )
    #     monitor.set_errorcode_unexpected_exception(True)

    # self.end_wait(sec, sac, monitor)

    @classmethod
    def end_wait(
        cls,
        timercount: int,
        sec: SharedExcepts,
        sac: SharedAppConfig,
        monitor: CalibrationUIGodot,
    ) -> int:
        monitor.set_status_calibcommon(CalibrationCommonStatus.COMPLETED)
        monitor.set_dummydata(
            enable_systemerrorflag=True,
            enable_errorflag=True,
            overwrite_checkresult=True,
            enable_yawangle=True,
        )
        monitor.transmit_setdata(sec=sec, ref_t=None)

        timercount += 1

        if timercount > 10:
            timercount = 0
            _logger.info("========================")
            _logger.info("CalibCheck2d3d end")
            _logger.info("========================")

        sleep(0.1)
        return timercount

    @classmethod
    def send_end_wait(
        cls,
        sec: SharedExcepts,
        sac: SharedAppConfig,
        monitor: CalibrationUIGodot,
    ) -> None:
        monitor.set_status_calibcommon(CalibrationCommonStatus.INACTIVE)
        monitor.set_dummydata(
            enable_systemerrorflag=True,
            enable_errorflag=True,
            overwrite_checkresult=True,
            enable_yawangle=True,
        )
        monitor.transmit_setdata(sec=sec, ref_t=None)

    def dataproc(
        self,
        readresult_pop: FIFOData,
        monitor: CalibrationUIGodot,
        sec: SharedExcepts,
    ) -> bool:  # 継続可否を返す Falseで終了
        # readresults = self.proccap.read(data_capture_inst)
        comparemode = "max"
        # データ入力
        # 10回ごとに入力を受け付け
        self.read_count += 1
        if self.read_count % 10 != 0:
            return True

        # for _ in range(10):
        #     readresult_pop  # 同期センサデータ入力
        # if readresult_pop is None:
        #     return False

        # データ入力 - カメラ入力
        camera_datalist, lidar_datalist, can_data, framecounter = readresult_pop
        if self.input_data_diagnosis(
            camera_datalist,
            lidar_datalist,
            can_data,
        ):
            return False

        camera_count = self.dataCapture_conf.Camera.count
        frame_cameras: list[NDArray[np.uint8] | None] = [None] * camera_count
        for ix, camera_rawdatatuple in enumerate(camera_datalist):
            if camera_rawdatatuple is None:
                self._logger.info(f"frame {ix} is invalid, skip")
                continue
            if ix >= camera_count:
                self._logger.warning(
                    f"camera index {ix} is out of configured range {camera_count}, skip"
                )
                continue
            frame = camera_rawdatatuple[0]
            frame_cameras[ix] = self.ud.get_undistort_image(frame)

        yoloresult_whole_list = self.yolo_adapter.predict_batch(
            sec=sec,
            frames=frame_cameras,
        )

        for ix, frame in enumerate(frame_cameras):
            yoloresult_whole = yoloresult_whole_list[ix]
            if frame is None:
                monitor.set_2Dbbox(ix, np.empty((0, 4), dtype=np.int32))
                continue
            frame, _ = draw_multibbox(frame, yoloresult_whole)

            bbox_for_ui: list[list[int]] = []
            image_h, image_w, _ = frame.shape
            for result_ix in range(int(yoloresult_whole[3])):
                coor = yoloresult_whole[0].reshape((-1, 4))[result_ix]
                # prob = yoloresult_whole[1][result_ix]
                # cls_id = yoloresult_whole[2][result_ix]

                # メインアプリ core - utils.py - draw_bbox 関数より編集
                bbox_ymin = coor[0] * image_h
                bbox_ymax = coor[2] * image_h
                bbox_xmin = coor[1] * image_w
                bbox_xmax = coor[3] * image_w

                bbox_for_ui.append([bbox_xmin, bbox_ymin, bbox_xmax, bbox_ymax])

            monitor.set_2Dbbox(ix, np.array(bbox_for_ui, dtype=np.int32))

        # データ入力 - LiDAR入力
        lidar_data = [x[0] for x in lidar_datalist if x is not None]

        if len(lidar_data) == 0:
            return False

        for x in range(len(lidar_data)):
            lidar_data[x] = np.array(lidar_data[x], dtype=np.float32)
            # lidar_data[x][:, :3] *= np.ones((len(lidar_data[x]), 3), dtype=np.float32) * debuginfo_and_functions.point_scale

        pts = conbine3d3d(
            xyz_data=lidar_data, trans_mat3D3D_eachlidar=self.trans_mat3D3D_eachlidar
        )

        # pts[:, 3] = np.where(pts[:, 3] < 0, pts[:, 3] + 256, pts[:, 3])
        # pts = pts[pts[:, 3] > 0]
        pts[:, 1] = -pts[:, 1]
        pts[:, 2] = -pts[:, 2]  # yz反転
        points = self._sub_detect_apply_static_point_filter(
            pcdframe=pts,
            timestamp_pcd=framecounter,
        )
        del pts

        # 点群処理：　地面点群除去と点群クラスタリング

        th = self.calibcheck2d3d_conf.z_threshold
        points = points[points[:, 2] > th]

        pts_obj_lim = np.array(
            utils3d.np_to_pcd(points[:, :3]).voxel_down_sample(0.2).points
        )
        (multi_points, multi_lines, multi_minmax), pcd_limited, db = internal_make_BB(
            pts_obj_lim
        )
        self.record_bbox1f(multi_minmax, yoloresult_whole_list)

        # 評価

        # integrated_retults_2d3d_old = None
        evaluate_results = []  # integrated_retults_2d3d, box3ds_reproj_list をカメラ個数分
        integrated_retults_2d3d_allcamera = []
        for camera_ix in range(3):
            # YOLO推定結果を取得
            yoloresult_whole = yoloresult_whole_list[camera_ix]

            # YOLO結果と3D bboxを入力し3D bboxごとの評価結果を得る。
            evaluate_results.append(
                calibcheck_detection_2d3d.evaluate2d3d(
                    width=self.calibcheck2d3d_conf.image_w,
                    height=self.calibcheck2d3d_conf.image_h,
                    multi_points=multi_points,
                    linkmethod="iou",
                    yoloresult_whole=yoloresult_whole,
                    rvec=self.rtvec_mat[camera_ix][0],
                    tvec=self.rtvec_mat[camera_ix][1],
                    ncm1=self.ud.ncm1,
                    integrated_retults_2d3d_old=None,  # integrated_retults_2d3d_old
                    # ここでoldを指定するとこの3D bboxごとの属性リストに上書きする形で登録。カメラごとの結果を知りたい場合はnoneにして混ぜないようにする
                )
            )

            # 直前にappendした要素からintegrated_retults_2d3d（3D bboxごとの評価結果リスト）を取り出し、カメラごとに人のbboxの評価値を取得し記録する。
            integrated_retults_2d3d = evaluate_results[-1][0]
            for elem_ix, elem in enumerate(integrated_retults_2d3d):
                if elem[0] == "human":
                    self.camera_scores_rawdata[camera_ix].append(
                        elem[2]
                    )  # カメラごとの評価値を記録しておく（後で統計を取る

            # integrated_retults_2d3d_old = evaluate_results[-1][0]

            # 3カメラ分の結果を統合　カメラ0の結果を土台に、同じcluster_ixでよりスコアの高い結果を上書きする。　←統合してはいけない　【Todo】
            if camera_ix == 0:
                integrated_retults_2d3d_allcamera = copy.deepcopy(
                    integrated_retults_2d3d
                )
            else:
                for cluster_ix, Z in enumerate(integrated_retults_2d3d):
                    (category, bbox_ix, score, box3ds_reproj_box, box2d) = Z
                    if category == "human":
                        if comparemode == "min":
                            eval_result: bool = (
                                score < integrated_retults_2d3d_allcamera[cluster_ix][2]
                            )
                        else:
                            eval_result: bool = (
                                score > integrated_retults_2d3d_allcamera[cluster_ix][2]
                            )
                        if eval_result:
                            # 該当3dBBの情報を書き換え
                            integrated_retults_2d3d_allcamera[cluster_ix] = (
                                copy.deepcopy(Z)
                            )

        # 評価値をUI送信用に加工
        reason_camera_notvalid: list[int] = []
        camera_evaluation_results: list[bool] = []
        for camera_values in self.camera_scores_rawdata:
            reason = 1
            result = False
            if (
                len(camera_values)
                >= self.calibcheck2d3d_conf.score_accept_count_threshold
            ):
                reason = 0
                camera_score = np.median(camera_values)
                result = camera_score >= self.calibcheck2d3d_conf.score_value_threshold

            reason_camera_notvalid.append(reason)
            camera_evaluation_results.append(result)

        self.camera_evaluation_results_to_monitor(
            monitor,
            reason_camera_notvalid,
            camera_evaluation_results,
        )

        del yoloresult_whole
        del integrated_retults_2d3d

        # 以下描画用の処理

        integrated_retults_2d3d = integrated_retults_2d3d_allcamera
        for camera_ix in range(3):
            # yoloresult_whole = yoloresult_whole_list[camera_ix]
            frame = frame_cameras[camera_ix]
            if frame is None:
                continue
            box3ds_reproj = evaluate_results[camera_ix][1]

            for ix, (lines, attr) in enumerate(
                zip(
                    multi_lines.reshape(-1, 12, 2),
                    integrated_retults_2d3d,
                    strict=False,
                )
            ):
                if attr[0] == "human":
                    boxcolor = (0, 0, 255)
                    self.checked_points3d.append(
                        multi_points[int(ix * 8) : int(ix * 8) + 8].mean(axis=0)
                    )
                    self.checked_points3d_score.append(attr[2])
                    self.checked_points2d.append(attr[3].mean(axis=0))
                    self.checked_points2d_score.append(attr[2])
                    # print(f"ix: {ix}, human, {multi_points[int(ix/8):int(ix/8)+8]}, {multi_points[int(ix/8):int(ix/8)+8].mean(axis=0)}, score: {attr[2]}")

                else:
                    boxcolor = (10, 10, 10)

                for pt1ix, pt2ix in lines:
                    cv2.line(
                        frame,
                        conv_intarr(box3ds_reproj[pt1ix]),
                        conv_intarr(box3ds_reproj[pt2ix]),
                        boxcolor,
                        2,
                    )

                cv2.putText(
                    frame,
                    f"{ix}",
                    conv_intarr(box3ds_reproj[pt1ix]),
                    fontFace=cv2.FONT_HERSHEY_SIMPLEX,
                    fontScale=1.0,
                    color=boxcolor,
                    thickness=2,
                    lineType=cv2.LINE_4,
                )

            # human_scores = np.array(
            #    [x[2] for x in integrated_retults_2d3d if x[0] == "human"]
            # )

            # cv2.imshow(f"frame{camera_ix}", cv2.resize(frame, dsize=None, fx=0.25, fy=0.25))

            # monitor.put_data(
            #    "dataproc",
            #    f"detect2d_image{camera_ix}",
            #    cv2.resize(frame, dsize=None, fx=0.25, fy=0.25),
            # )
            monitor.set_image(camera_ix, frame)

        # monitor.put_data("dataproc", "detect3d_points_raw", (pcd_limited, 0))
        if pcd_limited.shape[-1] == 4:
            monitor.set_points(
                pcd_limited[:, :3],
                monitor.convert_intensity_to_color(pcd_limited[:, 3]),
            )
        elif pcd_limited.shape[-1] == 3:
            monitor.set_points(
                pcd_limited, np.tile([0.2, 0.2, 0.2], (pcd_limited.shape[0], 1))
            )
        else:
            monitor.set_points(
                np.zeros((0, 3), dtype=np.float32), np.zeros((0, 3), dtype=np.float32)
            )
        # monitor.put_data("dataproc", "detect3d_multipoints", multi_points)
        monitor.set_boxes(
            points_multipoints=multi_points, points_multi_lines=multi_lines
        )
        # monitor.put_data("dataproc", "detect3d_multi_lines", multi_lines)

        if len(self.checked_points3d) > 0:
            scores = np.array(
                self.checked_points3d_score
            )  # checked_points3dと同じ長さのスコア情報 max1
            # monitor.put_data(
            #    "dataproc", f"3dobj_{0}_pts", np.array(self.checked_points3d)
            # )
            monitor.set_cornerpoints(
                np.array(self.checked_points3d),
                np.outer(scores, [0, 1, 0]) + np.outer(1 - scores, [1, 0, 0]),
            )
            # monitor.put_data(
            #    "dataproc",
            #    f"3dobj_{0}_clr",
            #    ,
            # )

        monitor.set_dummydata(
            enable_systemerrorflag=True,
            enable_errorflag=True,
            overwrite_checkresult=True,
            enable_yawangle=True,
        )
        self._write_debug_video_frames(monitor, self.debug_index)
        monitor.transmit_setdata(sec=sec, ref_t=self.debug_index)

        self.debug_index += 1
        return True

    def _sub_detect_apply_static_point_filter(self, pcdframe, timestamp_pcd):
        if self.app_config_calib.calib2d3d.Proc3d.enable_static_point_filter:
            if (
                self.static_point_filter.filtersource_framecount
                <= self.app_config_calib.calib2d3d.Proc3d.static_point_filter_initlength
                or (timestamp_pcd - self.pointfilter_lastadd)
                > self.app_config_calib.calib2d3d.Proc3d.static_point_filter_refresh_period
            ):
                self.static_point_filter.add_single_voxel_map(frame=pcdframe)
                if (
                    self.static_point_filter.filtersource_framecount
                    >= self.app_config_calib.calib2d3d.Proc3d.static_point_filter_initlength
                ):
                    self.static_point_filter.apply_voxelfilter()
                if not self.app_config_calib.default.print_disabled:
                    self._logger.info(
                        f"pointfilter add, length: {self.static_point_filter.filtersource_framecount}",
                    )

            if (
                self.static_point_filter.filtersource_framecount
                >= self.app_config_calib.calib2d3d.Proc3d.static_point_filter_initlength
            ):
                pcdframe = self.static_point_filter.extract_moving_objects(pcdframe)
            else:
                self._logger.info("static_point_filter - stacking points")
        return pcdframe

    def error_reason_to_string(self, reason: int) -> str:
        reason_map = {
            0: "OK",
            1: "情報取得時エラー",
            2: "3D bboxログが不正",
            3: "2D bboxログが不正",
            4: "3D bboxトラッキング結果が不正/不足",
            5: "2D bboxトラッキング結果が不正/不足",
            6: "2D3D評価結果が不正",
            7: "評価統計計算が不正",
            8: "校正判定が不正",
            9: "カメラ内に対象物なし",
            10: "カメラとLiDARの時間同期なし",
            11: "カメラとLiDARの評価可能な対象物なし",
        }
        return reason_map.get(reason, f"Unknown reason code {reason}")

    def error_reason_to_ui_errornum(self, reason: int) -> int:
        reason_map = {
            1: 3,
            2: 3,
            3: 3,
            4: 5,
            5: 5,
            6: 5,
            7: 5,
            8: 5,
            9: 4,
            10: 3,
            11: 4,
        }
        return reason_map.get(reason, -1)
