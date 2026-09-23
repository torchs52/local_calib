import copy
from collections.abc import Callable
from time import sleep

import cv2
import numpy as np
from numpy.typing import NDArray

from argus_synchro.calibration_mat_generator_modules.ctrl.calibcheck2d3d import (
    calibcheck_detection_2d3d,
)
from argus_synchro.calibration_mat_generator_modules.ctrl.calibcheck2d3d.debug_artifacts import (
    append_trace_event,
    append_evaluation_debug_info,
    close_video_writers,
    conv_intarr,
    create_evaluation_debug_summary,
    draw_evaluation_bboxes,
    draw_multibbox,
    draw_projected_bbox_edges,
    dump_pickle,
    ensure_video_writer,
    internal_make_BB,
    make_bbox3d_vertices,
    should_trace_eval_frame,
    write_video_frames,
)
from argus_synchro.calibration_mat_generator_modules.utils.calibration_utils import (
    conbine3d3d,
    read_rtvec,
)
from argus_synchro.calibration_mat_generator_modules.ctrl.calibcheck2d3d.SceneDesc import (
    Scene,
)
from argus_synchro.calibration_mat_generator_modules.ctrl.calibcheck2d3d.scene_calibcheck2d3d import (
    Scene_CalibCheck2d3d,
)
from argus_synchro.calibration_mat_generator_modules.ctrl.calibcheck2d3d.YOLOadapter import (
    YOLODamoBatchAdapter,
)
from argus_synchro.calibration_mat_generator_modules.ctrl.calibcheck2d3d.evaluator import (
    EvaluationRuntime,
    create_evaluation_debug_state,
    create_evaluation_metric_debug,
    evaluate_bbox_overlap_scenedesc,
    evaluate_2d3d,
    has_positive_2d_intersection,
    judge_calibration_result,
    passes_center_diff_gate,
    project_3dbbox_core,
    project_3d_track_bboxes,
    run_data_evaluation_process,
    select_camera_evaluation_result,
    select_visible_evaluation_frames,
    shrink_bbox2d,
)
from argus_synchro.calibration_mat_generator_modules.ctrl.calibcheck2d3d.processor import (
    apply_static_point_filter,
    create_calibcheck_session_state,
    diagnose_input_data,
    load_calibration_settings,
    make_empty_yoloresult,
    process_camera_frame,
    process_calibcheck_frame,
    process_lidar_frame,
    publish_yolo_bboxes_to_ui,
    lidar_points_to_ui_data,
    yolo_result_to_ui_bboxes,
    EvaluationMetricDebug,
    TrackProximityWarning,
    VirtualBBoxDebugCounts,
)
from argus_synchro.calibration_mat_generator_modules.ctrl.calibcheck2d3d.reporting import (
    publish_calibcheck_lifecycle_status,
    error_reason_to_string,
    error_reason_to_ui_errornum,
    publish_camera_calibcheck_statuses,
    write_calibcheck_result_files,
    write_evaluation_point_debug_file,
)
from argus_synchro.calibration_mat_generator_modules.ctrl.calibcheck2d3d.tracker_recorder import (
    calibcheck2d_bboxtracker_recorder,
    calibcheck3d_bboxtracker_recorder,
)
from argus_synchro.calibration_mat_generator_modules.ctrl.calibcheck2d3d.tracker import (
    collect_bbox_log_observation,
    collect_tracking_observation,
    detect_close_3dbbox_tracks,
    record_bbox1f,
    select_2dbbox_tracking_results,
    select_3dbbox_tracking_results,
    track_2dbbox,
    track_3dbbox,
)
from argus_synchro.calibration_mat_generator_modules.ctrl.calibration2d3d.track_main.interface_definition import (
    Tracking2dDataInterface,
    Tracking3dDataInterface,
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
    BBoxLogObservation,
    CalibCheck2d3dDiagnosis,
    CalibCheckFrameObservation,
    CalibCheckFailureReason,
    CameraCalibCheckDiagnosisResult,
    TrackingObservation,
    calibcheck_reason_to_status,
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

DATAPROC_READ_INTERVAL: int = 3 # 以前10だった、nフレームおきに処理実行するパラメータ。5や3のように小さくする可能性あり。


def log_register(app_logger_factory: AppLoggerFactory) -> None:
    app_logger_factory.append_logger(_logger)
    calibcheck_detection_2d3d.log_register(app_logger_factory)


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
        self.sac: SharedAppConfig = sac
        self.app_config_calib: AppConfigCalibration = app_config_calib
        self._ser: SharedErrors = shared_errors
        debug_config(
            base_dir="debug_out",
            flush_interval_sec=0.5,
            snapshot_interval_sec=5.0,
            snapshot_on_close=True,
            fsync_snapshot=False,
            fsync_flush=False,
        )

        self.width = app_config_calib.calibCheck2d3d.image_w
        self.height = app_config_calib.calibCheck2d3d.image_h
        self._evaluation_camera_count = app_config_calib.calibCheck2d3d.camera_count
        # 収集ループから収集後評価まで、1回の校正チェックで同じ診断を共有する。
        self._calibcheck_diagnosis = CalibCheck2d3dDiagnosis(
            self._evaluation_camera_count,
            person_not_detected_sec=(
                app_config_calib.calibCheck2d3d.person_not_detected_sec
            ),
        )

        self._report_file_io_error: Callable[[str, str, Exception], None] = (
            self._report_file_io_error_impl
        )

        self.calibcheck2d3d_conf: CalibCheck2d3dConf = app_config_calib.calibCheck2d3d
        self.dataCapture_conf: DataCaptureConf = app_config_calib.dataCapture
        self.yolo_adapter = YOLODamoBatchAdapter(
            app_config_calib=app_config_calib,
            app_logger_factory=app_logger_factory,
            shared_errors=self._ser,
        )

        self.ud: Mcde7000UndistortImageProvider = Mcde7000UndistortImageProvider(
            camera_intrinsics_path=self.calibcheck2d3d_conf.camera_intrinsics_path,
            sys_width=self.calibcheck2d3d_conf.image_w,
            sys_height=self.calibcheck2d3d_conf.image_h,
        )
        self._evaluation_intrinsics = self.ud.ncm1

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

        self.scenedesc_calibcheck = Scene_CalibCheck2d3d.create_for_evaluation(
            scene_conf=self.sac.read().SceneDescription,
            camera_intrinsics=self._evaluation_intrinsics,
            image_width=self.width,
            image_height=self.height,
        )

        self.debug_index = 0
        self.monitor_data = {}
        self.verbose = not app_config_calib.default.print_disabled

        # 点群事前処理：静止点群除去
        self.DATARANGE_XYZ = ((-10, 20), (-20, 20), (-2, 2))
        self.static_point_filter = filter_static_objects(
            1.0,
            *self.DATARANGE_XYZ,
        )
        self.pointfilter_lastadd = -1

        # Load configuration from app_config_calib.calibCheck2d3d
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
        self.WARN_3D_TRACK_PROXIMITY_ENABLED: bool = True
        self.WARN_3D_TRACK_PROXIMITY_FAILS_VALIDATION: bool = True
        self.WARN_3D_TRACK_CENTER_DISTANCE_M: float = 2.0
        self.WARN_3D_TRACK_MIN_CLOSE_FRAMES: int = 20
        self.WARN_3D_TRACK_MIN_CLOSE_RATIO: float = 0.2
        self._3d_track_proximity_warnings: list[TrackProximityWarning] = []

        # 評価対象フレームの間引き幅。1なら全フレーム、2なら半分程度を評価。
        self.EVAL_FRAME_STRIDE: int = self.calibcheck2d3d_conf.eval_frame_stride
        # True: 旧寄り指標(比較可能フレーム平均)を採用 / False: 厳密hit-rateを採用。
        self.USE_LEGACY_LIKE_METRIC: bool = (
            self.calibcheck2d3d_conf.use_legacy_like_metric
        )

        # デバッグ記録設定（必要時のみ有効化し、通常運用のオーバーヘッドを抑える）
        self.DEBUG_CALIBCHECK_ENABLED: bool = (
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
        self.EVAL_ZVALUES = self.calibcheck2d3d_conf.eval_zvalues
        self.VIRTUAL_BBOX_XY_OFFSETS = VIRTUAL_BBOX_XY_OFFSETS
        self.VIRTUAL_BBOX_CANDIDATE_NAMES = VIRTUAL_BBOX_CANDIDATE_NAMES
        self._virtual_bbox_debug_counts: list[VirtualBBoxDebugCounts] = []
        self._evaluation_metric_debug: list[EvaluationMetricDebug] = []

        # 評価分岐トレース: 特定フレームの判定経路を詳細保存する。
        self.DEBUG_EVAL_TRACE_ENABLED: bool = (
            self.calibcheck2d3d_conf.debug_eval_trace_enabled
        )
        self.DEBUG_EVAL_TRACE_ALL_FRAMES: bool = (
            self.calibcheck2d3d_conf.debug_eval_trace_all_frames
        )
        self.DEBUG_EVAL_TRACE_RANGE_START: int = (
            self.calibcheck2d3d_conf.debug_eval_trace_range_start
        )
        self.DEBUG_EVAL_TRACE_RANGE_END: int = (
            self.calibcheck2d3d_conf.debug_eval_trace_range_end
        )
        self.DEBUG_EVAL_TRACE_TARGET_FRAMES: set[int] = set()

        self._debug_video_writers: list[cv2.VideoWriter | None] = []
        self._debug_video_paths: list[str] = []
        self._debug_eval_info: dict[str, object] = {}

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
        (
            self.trans_mat3D3D_eachlidar,
            self.rtvec_mat,
        ) = load_calibration_settings(
            lidar_calib_files=self.calibcheck2d3d_conf.lidar_calib_files,
            camera_calib_files=self.calibcheck2d3d_conf.camera_calib_files,
            get_new_axis_mode=lambda: self.calibcheck2d3d_conf.new_axis_mode,
            read_rtvec=read_rtvec,
            report_file_io_error=self._report_file_io_error,
            logger=self._logger,
        )

    def _ensure_debug_video_writer(
        self,
        camera_ix: int,
        frame_bgr: NDArray[np.uint8],
    ) -> None:
        if not self.DEBUG_CALIBCHECK_ENABLED or not self.DEBUG_CAPTURE_UI_VIDEO_ENABLED:
            return
        ensure_video_writer(
            camera_ix,
            frame_bgr,
            writers=self._debug_video_writers,
            paths=self._debug_video_paths,
            prefix=self.DEBUG_VIDEO_PREFIX,
            fps=self.DEBUG_VIDEO_FPS,
            logger=self._logger,
        )

    def _write_debug_video_frames(
        self,
        monitor: CalibrationUIGodot,
        frame_ix: int,
    ) -> None:
        if not self.DEBUG_CALIBCHECK_ENABLED or not self.DEBUG_CAPTURE_UI_VIDEO_ENABLED:
            return

        write_video_frames(
            monitor.cameradata,
            frame_ix=frame_ix,
            writers=self._debug_video_writers,
            paths=self._debug_video_paths,
            prefix=self.DEBUG_VIDEO_PREFIX,
            fps=self.DEBUG_VIDEO_FPS,
            add_frame_text=self.DEBUG_CAPTURE_FRAME_TEXT_ENABLED,
            logger=self._logger,
        )

    def _close_debug_video_writers(self) -> None:
        if not self._debug_video_writers:
            return
        close_video_writers(
            self._debug_video_writers,
            self._debug_video_paths,
            logger=self._logger,
        )

    def _dump_debug_eval_info(self) -> None:
        if not self.DEBUG_CALIBCHECK_ENABLED:
            return
        dump_pickle(self.DEBUG_EVAL_PICKLE_PATH, self._debug_eval_info)
        self._logger.info(
            f"Debug eval info dumped: {self.DEBUG_EVAL_PICKLE_PATH}",
        )

    def _dump_frame_info(self) -> None:
        if not self.DEBUG_CALIBCHECK_ENABLED:
            return
        dump_pickle("frame_info_dump.pickle", self.frame_info)

    def _should_trace_eval_frame(self, frame_ix: int) -> bool:
        return should_trace_eval_frame(
            frame_ix,
            debug_enabled=self.DEBUG_CALIBCHECK_ENABLED,
            trace_enabled=self.DEBUG_EVAL_TRACE_ENABLED,
            trace_all_frames=self.DEBUG_EVAL_TRACE_ALL_FRAMES,
            trace_range_start=self.DEBUG_EVAL_TRACE_RANGE_START,
            trace_range_end=self.DEBUG_EVAL_TRACE_RANGE_END,
            trace_target_frames=self.DEBUG_EVAL_TRACE_TARGET_FRAMES,
        )

    @staticmethod
    def _append_trace_event(
        eval_trace_by_camera: dict[int, dict[int, list[dict[str, object]]]],
        camera_ix: int,
        frame_ix: int,
        event: dict[str, object],
    ) -> None:
        append_trace_event(
            eval_trace_by_camera,
            camera_ix=camera_ix,
            frame_ix=frame_ix,
            event=event,
        )

    # データ取得・bbox記録

    def _sub_proc_3d_monitor_data(
        self,
        monitor: CalibrationUIGodot,
        pcd_limited: NDArray[np.float32],
        multi_points: NDArray[np.float32],
        multi_lines: NDArray[np.float32],
    ) -> None:
        points, colors = lidar_points_to_ui_data(
            pcd_limited,
            intensity_to_color=monitor.convert_intensity_to_color,
        )
        monitor.set_points(points, colors)
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
        return process_lidar_frame(
            lidar_datalist,
            framecounter=framecounter,
            read_point_cloud=self.cap3d.read,
            apply_static_filter=self._sub_detect_apply_static_point_filter,
            z_threshold=self.calibcheck2d3d_conf.z_threshold,
            data_range_xyz=self.DATARANGE_XYZ,
            voxel_downsample=lambda points: np.array(
                utils3d.np_to_pcd(points).voxel_down_sample(0.2).points
            ),
            make_bounding_boxes=internal_make_BB,
            publish_lidar_data=lambda pcd_limited, multi_points, multi_lines: (
                self._sub_proc_3d_monitor_data(
                    monitor=monitor,
                    pcd_limited=pcd_limited,
                    multi_points=multi_points,
                    multi_lines=multi_lines,
                )
            ),
            logger=self._logger,
        )

    def _sub_proc_2d_monitor_data(
        self,
        monitor: CalibrationUIGodot,
        frame_cameras: list[NDArray[np.uint8] | None],
        yoloresult_whole_list: list[list[NDArray[np.float32]]],
    ) -> None:
        publish_yolo_bboxes_to_ui(
            frame_cameras,
            yoloresult_whole_list,
            draw_bboxes=draw_multibbox,
            set_bboxes=monitor.set_2Dbbox,
        )

    @staticmethod
    def _make_empty_yoloresult() -> list[NDArray[np.float32]]:
        # YOLO出力フォーマットに合わせて、検出0件の結果を返す。
        return make_empty_yoloresult()

    def proc_camera1f(
        self,
        camera_datalist: list[tuple[NDArray[np.uint8], int, float] | None],
        framecounter: int,
        monitor: CalibrationUIGodot,
        sec: SharedExcepts,
    ) -> list[list[NDArray[np.float32]]]:
        del framecounter
        return process_camera_frame(
            camera_datalist,
            camera_count=self.dataCapture_conf.Camera.count,
            undistort_image=self.ud.get_undistort_image,
            predict_batch=self.yolo_adapter.predict_batch,
            sec=sec,
            publish_image=monitor.set_image,
            publish_detection_data=lambda frame_cameras, yoloresult_whole_list: (
                self._sub_proc_2d_monitor_data(
                    monitor=monitor,
                    frame_cameras=frame_cameras,
                    yoloresult_whole_list=yoloresult_whole_list,
                )
            ),
            logger=self._logger,
        )

    def record_bbox1f(
        self,
        multi_minmax: NDArray[np.float64],
        yoloresult_whole_list: list[list[NDArray]],
    ) -> None:
        record_bbox1f(
            self.frame_info,
            multi_minmax=multi_minmax,
            yoloresult_whole_list=yoloresult_whole_list,
            maximum_frame_count=self.FRAME_INFO_MAXLEN,
        )

    def collect_bbox_log_observation(self) -> BBoxLogObservation:
        return collect_bbox_log_observation(
            self.frame_info,
            camera_count=self._evaluation_camera_count,
            minimum_3d_bbox_count_per_frame=self.THRESH_3DBBOX_COUNT_PER_FRAME,
            minimum_3d_valid_frame_ratio=self.THRESH_3DBBOX_COUNT_MEAN_RATIO,
            minimum_2d_bbox_count_per_frame=self.THRESH_2DBBOX_COUNT_PER_FRAME,
            minimum_2d_valid_frame_ratio=self.THRESH_2DBBOX_COUNT_MEAN_RATIO,
        )

    def track_3dbbox(self) -> Tracking3dDataInterface:
        return track_3dbbox(
            self.frame_info,
            app_config_calib=self.app_config_calib,
            camera_index=self.sac.read().CalibMode.cameraID,
        )

    def select_3dbbox_tracking_results(
        self, tracking_3d_data_interface: Tracking3dDataInterface
    ) -> Tracking3dDataInterface:
        return select_3dbbox_tracking_results(
            tracking_3d_data_interface,
            logger=self._logger,
        )

    def _detect_close_3dbbox_tracks(
        self,
        tracking_3d_data_interface: Tracking3dDataInterface,
    ) -> list[TrackProximityWarning]:
        return detect_close_3dbbox_tracks(
            tracking_3d_data_interface,
            enabled=self.WARN_3D_TRACK_PROXIMITY_ENABLED,
            center_distance_m=self.WARN_3D_TRACK_CENTER_DISTANCE_M,
            min_close_frames=self.WARN_3D_TRACK_MIN_CLOSE_FRAMES,
            min_close_ratio=self.WARN_3D_TRACK_MIN_CLOSE_RATIO,
        )

    def track_2dbbox(self) -> list[Tracking2dDataInterface]:
        return track_2dbbox(
            self.frame_info,
            app_config_calib=self.app_config_calib,
            image_size_hw=(
                self.calibcheck2d3d_conf.image_h,
                self.calibcheck2d3d_conf.image_w,
            ),
            camera_count=self._evaluation_camera_count,
        )

    def select_2dbbox_tracking_results(
        self, tracking_2d_data_interfaces: list[Tracking2dDataInterface]
    ) -> list[Tracking2dDataInterface]:
        return select_2dbbox_tracking_results(
            tracking_2d_data_interfaces,
            logger=self._logger,
        )

    def collect_tracking_observation(
        self,
        tracking_3d_data_interface: Tracking3dDataInterface,
        tracking_2d_data_interfaces: list[Tracking2dDataInterface],
    ) -> TrackingObservation:
        observation, proximity_warnings = collect_tracking_observation(
            tracking_3d_data_interface,
            tracking_2d_data_interfaces,
            camera_count=self._evaluation_camera_count,
            minimum_3d_alive_track_count=self.THRESH_3DBBOX_TRACKING_IDCOUNT,
            minimum_2d_alive_track_count=self.THRESH_2DBBOX_TRACKING_IDCOUNT,
            proximity_enabled=self.WARN_3D_TRACK_PROXIMITY_ENABLED,
            proximity_center_distance_m=self.WARN_3D_TRACK_CENTER_DISTANCE_M,
            proximity_min_close_frames=self.WARN_3D_TRACK_MIN_CLOSE_FRAMES,
            proximity_min_close_ratio=self.WARN_3D_TRACK_MIN_CLOSE_RATIO,
            proximity_warning_fails_validation=(
                self.WARN_3D_TRACK_PROXIMITY_FAILS_VALIDATION
            ),
            logger=self._logger,
        )
        self._3d_track_proximity_warnings = proximity_warnings
        return observation

    def project_3dbbox_core(
        self,
        bbox3d: NDArray[np.float32],
        camera_index: int,
        require_points_in_image: bool = False,
    ) -> NDArray[np.float32] | None:
        return project_3dbbox_core(
            bbox3d,
            camera_extrinsics=self.rtvec_mat[camera_index][2],
            camera_intrinsics=self._evaluation_intrinsics,
            image_width=self.width,
            image_height=self.height,
            require_points_in_image=require_points_in_image,
        )

    @staticmethod
    def _shrink_bbox2d(
        bbox2d: NDArray[np.float32], factor: float
    ) -> NDArray[np.float32]:
        return shrink_bbox2d(bbox2d, factor)

    @classmethod
    def _passes_center_diff_gate(
        cls,
        bbox_a: NDArray[np.float32],
        bbox_b: NDArray[np.float32],
    ) -> bool:
        return passes_center_diff_gate(
            bbox_a,
            bbox_b,
            shrink_factor=BBOX_SHRINK_FACTOR,
            threshold=BBOX_CENTER_DIFF_RATIO_THRESHOLD,
        )

    def evaluate_bbox_overlap_scenedesc(
        self,
        bbox3d: NDArray[np.float32],
        bbox2d: NDArray[np.float32],
        camera_index: int,
    ) -> float | None:
        return evaluate_bbox_overlap_scenedesc(
            bbox3d,
            bbox2d,
            camera_index=camera_index,
            rvec=self.rtvec_mat[camera_index][0],
            tvec=self.rtvec_mat[camera_index][1],
            camera_extrinsics=self.rtvec_mat[camera_index][2],
            camera_intrinsics=self._evaluation_intrinsics,
            image_width=self.width,
            image_height=self.height,
            scene=self.scenedesc_calibcheck,
            virtual_bbox_xy_offsets=self.VIRTUAL_BBOX_XY_OFFSETS,
            virtual_bbox_candidate_names=self.VIRTUAL_BBOX_CANDIDATE_NAMES,
            virtual_bbox_debug_counts=self._virtual_bbox_debug_counts,
            bbox_shrink_factor=BBOX_SHRINK_FACTOR,
            center_diff_ratio_threshold=BBOX_CENTER_DIFF_RATIO_THRESHOLD,
        )

    @staticmethod
    def _has_positive_2d_intersection(
        bbox_a: NDArray[np.float32],
        bbox_b: NDArray[np.float32],
    ) -> bool:
        return has_positive_2d_intersection(bbox_a, bbox_b)

    def evaluate_2d3d(
        self,
        tracking_3d_data_interface: Tracking3dDataInterface,
        tracking_2d_data_interfaces: list[Tracking2dDataInterface],
        zvalues: tuple[float, float],
    ) -> tuple[list[float], list[int]]:
        return evaluate_2d3d(
            EvaluationRuntime(
                evaluate=self._evaluate_2d3d_impl,
                virtual_bbox_debug_counts=self._virtual_bbox_debug_counts,
                evaluation_metric_debug=self._evaluation_metric_debug,
                debug_eval_info=self._debug_eval_info,
            ),
            tracking_3d_data_interface,
            tracking_2d_data_interfaces,
            zvalues=zvalues,
        )

    def _evaluate_2d3d_impl(
        self,
        tracking_3d_data_interface: Tracking3dDataInterface,
        tracking_2d_data_interfaces: list[Tracking2dDataInterface],
        zvalues: tuple[float, float],
    ) -> tuple[list[float], list[int]]:
        # Delegate to extracted evaluator function
        from argus_synchro.calibration_mat_generator_modules.ctrl.calibcheck2d3d.evaluator import (
            _run_evaluation_impl_core,
        )
        return _run_evaluation_impl_core(
            tracking_3d_data_interface,
            tracking_2d_data_interfaces,
            zvalues=zvalues,
            camera_count=self._evaluation_camera_count,
            eval_frame_stride=self.EVAL_FRAME_STRIDE,
            use_legacy_like_metric=self.USE_LEGACY_LIKE_METRIC,
            debug_enabled=self.DEBUG_CALIBCHECK_ENABLED,
            bbox_shrink_factor=BBOX_SHRINK_FACTOR,
            virtual_bbox_candidate_names=self.VIRTUAL_BBOX_CANDIDATE_NAMES,
            project_3dbbox_core_fn=lambda bbox3d, camera_ix: self.project_3dbbox_core(
                bbox3d, camera_ix, require_points_in_image=True
            ),
            should_trace_eval_frame=self._should_trace_eval_frame,
            append_trace_event=self._append_trace_event,
            evaluate_bbox_overlap_scenedesc_fn=self.evaluate_bbox_overlap_scenedesc,
            shrink_bbox2d_fn=self._shrink_bbox2d,
            has_positive_2d_intersection_fn=self._has_positive_2d_intersection,
            virtual_bbox_debug_counts=self._virtual_bbox_debug_counts,
            evaluation_metric_debug=self._evaluation_metric_debug,
            debug_eval_info=self._debug_eval_info,
            logger=self._logger,
            debug_eval_trace_enabled=self.DEBUG_EVAL_TRACE_ENABLED,
            debug_eval_trace_all_frames=self.DEBUG_EVAL_TRACE_ALL_FRAMES,
            debug_eval_trace_range_start=self.DEBUG_EVAL_TRACE_RANGE_START,
            debug_eval_trace_range_end=self.DEBUG_EVAL_TRACE_RANGE_END,
            debug_eval_trace_target_frames=self.DEBUG_EVAL_TRACE_TARGET_FRAMES,
            debug_video_paths=self._debug_video_paths,
        )

    def judge_calibration_result(
        self, evaluation_statistics: list[float], threshold: float = 0.1
    ) -> list[bool]:
        return judge_calibration_result(
            evaluation_statistics,
            threshold=threshold,
        )

    def data_evaluation_process(self) -> list[CameraCalibCheckDiagnosisResult]:
        def save_debug_tracking(
            tracking_3d: Tracking3dDataInterface,
            tracking_2d: list[Tracking2dDataInterface],
        ) -> None:
            if not self.DEBUG_CALIBCHECK_ENABLED:
                return
            debug_store("tracking_3d_data_interface", tracking_3d)
            for camera_ix, tracking_2d_data_interface in enumerate(tracking_2d):
                debug_store(
                    f"tracking_2d_data_interface_{camera_ix}",
                    tracking_2d_data_interface,
                )
            debug_force_snapshot()

        def save_debug_evaluation(
            evaluation_statistics: list[float],
            evaluation_reasons: list[int],
            tracking_3d: Tracking3dDataInterface,
            tracking_2d: list[Tracking2dDataInterface],
        ) -> None:
            if not self.DEBUG_CALIBCHECK_ENABLED:
                return
            append_evaluation_debug_info(
                self._debug_eval_info,
                evaluation_statistics=evaluation_statistics,
                evaluation_reasons=evaluation_reasons,
                camera_calib_files=self.calibcheck2d3d_conf.camera_calib_files,
                camera_intrinsics_path=self.calibcheck2d3d_conf.camera_intrinsics_path,
                image_width=int(self.width),
                image_height=int(self.height),
                eval_zvalues=self.EVAL_ZVALUES,
                new_axis_mode=bool(self.calibcheck2d3d_conf.new_axis_mode),
                tracking_3d_data_interface=tracking_3d,
                tracking_2d_data_interfaces=tracking_2d,
                proximity_warnings=self._3d_track_proximity_warnings,
            )
            self._dump_debug_eval_info()

        # object.__new__で生成する既存テストなど、__init__を通らない経路も許容する。
        if not hasattr(self, "_calibcheck_diagnosis"):
            self._calibcheck_diagnosis = CalibCheck2d3dDiagnosis(
                self._evaluation_camera_count,
                person_not_detected_sec=getattr(
                    self.calibcheck2d3d_conf,
                    "person_not_detected_sec",
                    None,
                ),
            )
        return run_data_evaluation_process(
            camera_count=self._evaluation_camera_count,
            collect_bbox_log_observation=self.collect_bbox_log_observation,
            track_3dbbox=self.track_3dbbox,
            track_2dbbox=self.track_2dbbox,
            collect_tracking_observation=self.collect_tracking_observation,
            evaluate=lambda tracking_3d, tracking_2d: self.evaluate_2d3d(
                tracking_3d,
                tracking_2d,
                zvalues=self.EVAL_ZVALUES,
            ),
            get_evaluation_statistics=lambda: self._evaluation_metric_debug,
            judge=lambda statistics: self.judge_calibration_result(
                statistics,
                threshold=self.calibcheck2d3d_conf.score_value_threshold,
            ),
            minimum_sample_count=(
                self.calibcheck2d3d_conf.score_accept_count_threshold
            ),
            judgement_threshold=self.calibcheck2d3d_conf.score_value_threshold,
            save_debug_tracking=save_debug_tracking,
            save_debug_evaluation=save_debug_evaluation,
            logger=self._logger,
            diagnosis=self._calibcheck_diagnosis,
        )

    def camera_evaluation_results_to_monitor(
        self,
        monitor: CalibrationUIGodot,
        results: list[CameraCalibCheckDiagnosisResult],
    ) -> None:
        publish_camera_calibcheck_statuses(
            results,
            set_status=monitor.set_camera_calibcheck_status,
        )

    def pre_app_loopmain(
        self,
        monitor: CalibrationUIGodot,
        sec: SharedExcepts,
        sac: SharedAppConfig,
    ) -> None:
        self.debug_index = 0
        self._logger.info(f"frame index: {self.debug_index}")

        # 前回実行のfindingを次の校正チェックへ持ち越さない。
        if hasattr(self, "_calibcheck_diagnosis"):
            self._calibcheck_diagnosis.reset(
                self.calibcheck2d3d_conf.camera_count
            )
        else:
            self._calibcheck_diagnosis = CalibCheck2d3dDiagnosis(
                self.calibcheck2d3d_conf.camera_count,
                person_not_detected_sec=getattr(
                    self.calibcheck2d3d_conf,
                    "person_not_detected_sec",
                    None,
                ),
            )

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

        state = create_calibcheck_session_state(self.calibcheck2d3d_conf.camera_count)
        self.frame_info = state.frame_info  # 2D bbox(list) x camera, 3D bbox
        self.read_count = state.read_count
        self._debug_video_writers = state.debug_video_writers
        self._debug_video_paths = state.debug_video_paths
        self._debug_eval_info = state.debug_eval_info
        self.checked_points3d = state.checked_points3d
        self.checked_points3d_score = state.checked_points3d_score
        self.checked_points2d = state.checked_points2d
        self.checked_points2d_score = state.checked_points2d_score
        self.camera_scores_rawdata = state.camera_scores_rawdata  # カメラごとのbbox評価値のリスト

    def input_data_diagnosis(
        self,
        camera_datalist: list[tuple[NDArray[np.uint8], int, float] | None],
        lidar_datalist: list[tuple[NDArray[np.float64], int, float] | None],
        can_data: object,
    ) -> bool:
        return diagnose_input_data(
            camera_datalist,
            lidar_datalist,
            can_data,
            shared_errors=self._ser,
            invalid_data_input_index=StateErrorDIndex.INVALID_DATA_INPUT,
            array_shape_error_index=StateErrorDIndex.ARRAY_SHAPE_ERROR,
            detection_result=ResultDiagnosis.DETECTION,
        )

    def _draw_evaluation_bboxes(
        self,
        monitor: CalibrationUIGodot,
        multi_minmax: NDArray[np.float64],
    ) -> None:
        draw_evaluation_bboxes(
            monitor.cameradata,
            multi_minmax,
            camera_rtvecs=self.rtvec_mat,
            camera_intrinsics=self.ud.ncm1,
            eval_zvalues=self.EVAL_ZVALUES,
            virtual_bbox_xy_offsets=self.VIRTUAL_BBOX_XY_OFFSETS,
            project_bbox=self.project_3dbbox_core,
        )

    def app_loopmain(
        self,
        fifo_data: FIFOData,
        monitor: CalibrationUIGodot,
        sec: SharedExcepts,
        sac: SharedAppConfig,
    ) -> bool:
        def diagnose_input_and_record(
            camera_datalist: object,
            lidar_datalist: object,
            can_data: object,
        ) -> bool:
            # 既存のシステム入力診断結果を、校正チェック固有のreason 1にも記録する。
            is_invalid = self.input_data_diagnosis(
                camera_datalist,
                lidar_datalist,
                can_data,
            )
            if is_invalid:
                diagnosis = getattr(self, "_calibcheck_diagnosis", None)
                if diagnosis is not None:
                    diagnosis.diagnose_frame(
                        CalibCheckFrameObservation(system_input_error_detected=True)
                    )
            return is_invalid

        def process_camera_and_record(
            camera_datalist: object, framecounter: int
        ) -> list[list[NDArray[np.float32]]]:
            yolo_results = self.proc_camera1f(
                camera_datalist,
                framecounter,
                monitor,
                sec,
            )
            diagnosis = getattr(self, "_calibcheck_diagnosis", None)
            # __init__を通らない呼び出しでは、実際のカメラ数から診断を補完する。
            if diagnosis is None and isinstance(yolo_results, list) and yolo_results:
                diagnosis = CalibCheck2d3dDiagnosis(
                    len(yolo_results),
                    person_not_detected_sec=getattr(
                        self.calibcheck2d3d_conf,
                        "person_not_detected_sec",
                        None,
                    ),
                )
                self._calibcheck_diagnosis = diagnosis
            if diagnosis is not None and isinstance(yolo_results, list):
                try:
                    # proc_camera1fの各カメラ結果のindex 3は人物検出数。
                    # 連続未検出(reason 9)を将来有効化できるようフレームごとに蓄積する。
                    detection_counts = tuple(
                        int(camera_result[3]) for camera_result in yolo_results
                    )
                except (IndexError, TypeError, ValueError):
                    # テスト用差し替えなど、実運用の戻り値形式でない場合は診断しない。
                    detection_counts = ()
                if len(detection_counts) == diagnosis.camera_count:
                    diagnosis.diagnose_frame(
                        CalibCheckFrameObservation(
                            camera_detection_counts=detection_counts
                        )
                    )
            return yolo_results

        def transmit_frame(_framecounter: int) -> None:
            monitor.set_dummydata(
                enable_systemerrorflag=True,
                enable_errorflag=True,
                overwrite_checkresult=True,
                enable_yawangle=True,
            )
            monitor.transmit_setdata(sec=sec, ref_t=self.debug_index)
            self.debug_index += 1

        return process_calibcheck_frame(
            fifo_data,
            diagnose_input=diagnose_input_and_record,
            publish_yaw=monitor.set_yaw,
            process_lidar=lambda lidar_datalist, framecounter: self.proc_lidar1f(
                lidar_datalist, framecounter, monitor
            ),
            process_camera=process_camera_and_record,
            draw_evaluation_bboxes=lambda multi_minmax: self._draw_evaluation_bboxes(
                monitor, multi_minmax
            ),
            record_bboxes=self.record_bbox1f,
            write_debug_video=lambda _framecounter: self._write_debug_video_frames(
                monitor, self.debug_index
            ),
            transmit_frame=transmit_frame,
            logger=self._logger,
        )

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
        diagnosis_results = self.data_evaluation_process()
        self.camera_evaluation_results_to_monitor(
            monitor,
            diagnosis_results,
        )
        monitor.transmit_setdata(sec=sec, ref_t=None)
        self._close_debug_video_writers()
        self._dump_frame_info()
        write_calibcheck_result_files(
            self.calibcheck2d3d_conf.resultfiles,
            diagnosis_results,
            logger=self._logger,
        )
        write_evaluation_point_debug_file(
            self.app_config_calib.default.outputdir_root,
            checked_points3d=self.checked_points3d,
            checked_points3d_score=self.checked_points3d_score,
            checked_points2d=self.checked_points2d,
            checked_points2d_score=self.checked_points2d_score,
        )

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
        publish_calibcheck_lifecycle_status(
            monitor,
            sec=sec,
            status=CalibrationCommonStatus.COMPLETED,
        )

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
        publish_calibcheck_lifecycle_status(
            monitor,
            sec=sec,
            status=CalibrationCommonStatus.INACTIVE,
        )

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
        if self.read_count % DATAPROC_READ_INTERVAL != 0:
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

        diagnosis_results: list[CameraCalibCheckDiagnosisResult] = []
        for camera_index, (reason, result) in enumerate(
            zip(reason_camera_notvalid, camera_evaluation_results, strict=True)
        ):
            normalized_reason = CalibCheckFailureReason(reason)
            effective_result = result if normalized_reason == 0 else None
            diagnosis_results.append(
                CameraCalibCheckDiagnosisResult(
                    camera_index=camera_index,
                    status=calibcheck_reason_to_status(
                        normalized_reason,
                        calibration_is_acceptable=effective_result,
                    ),
                    primary_reason=normalized_reason,
                    all_reasons=(normalized_reason,) if normalized_reason else (),
                    calibration_is_acceptable=effective_result,
                )
            )
        self.camera_evaluation_results_to_monitor(monitor, diagnosis_results)

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
        proc3d_conf = self.app_config_calib.calib2d3d.Proc3d
        return apply_static_point_filter(
            pcdframe,
            static_point_filter=self.static_point_filter,
            timestamp_pcd=timestamp_pcd,
            pointfilter_lastadd=self.pointfilter_lastadd,
            enabled=proc3d_conf.enable_static_point_filter,
            initlength=proc3d_conf.static_point_filter_initlength,
            refresh_period=proc3d_conf.static_point_filter_refresh_period,
            print_disabled=self.app_config_calib.default.print_disabled,
            logger=self._logger,
        )

    def error_reason_to_string(self, reason: int) -> str:
        return error_reason_to_string(reason)

    def error_reason_to_ui_errornum(self, reason: int) -> int:
        return error_reason_to_ui_errornum(reason)
