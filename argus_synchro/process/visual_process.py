from __future__ import annotations

import time
import typing
from contextlib import ExitStack

import argus_synchro_lib.detect3d as d3
import numpy as np
from argus_synchro_lib.collision_detector import CoordMethod
from argus_synchro_lib.octotree import NodeEntity, OctoTree
from argus_synchro_lib.ui_interface import GeneralConf, UIIFConf
from argus_synchro_lib.visualizer import GodotUIVisualizer

import argus_synchro_lib
from argus_synchro import SubScrutinizer, cam_monitor
from argus_synchro.common import app_logger, paths
from argus_synchro.common.common import t_py_col_res
from argus_synchro.diagnosis.error_config import ErrorConfig
from argus_synchro.diagnosis.error_diagnosis import ResultDiagnosis
from argus_synchro.interface.collision_detection import (
    AbstractCollisionDetectCreator,
    CollisionDetectOnCreator,
)
from argus_synchro.interface.octotree_func import (
    OctoTreeFuncInterface,
    OctoTreeFuncOff,
    OctoTreeFuncOn,
)
from argus_synchro.message.input_message import PcdData
from argus_synchro.message.scrutinizer_message import (
    AccumPointsData,
    CameraDetectionsData,
)
from argus_synchro.process import ProcessBase
from argus_synchro.process.message import Consumer, MessageFlow
from argus_synchro.process.synchronizer import ProcessActivator
from argus_synchro.profiler import log_main, log_target
from argus_synchro.profiler.prof_fps import ProfFps
from argus_synchro.profiler.prof_mode import ProfCategory
from argus_synchro.profiler.target import log_target_area
from argus_synchro.py_octotree.detectable_points import (
    DetectableCylinderPointImmobile,
    DetectableCylinderPointMobile,
    get_detectable_z_range,
)
from argus_synchro.shared_errors import (
    ActionErrorIndex,
    ModuleErrorIndex,
    SharedErrors,
    StateErrorDIndex,
    StateErrorIndex,
)
from argus_synchro.shared_excepts import INVALID_TIMESTAMP
from argus_synchro.SystemMonitor import ProcessTimeMonitor
from argus_synchro.SystemMonitor import ProcessTimeMonitor as PTMonitor
from argus_synchro.tester import Tester

if typing.TYPE_CHECKING:
    from typing import Literal

    from argus_synchro_lib.machine_collision import MachineCollisionBase
    from numpy.typing import NDArray

    from argus_synchro.Camera import Camera
    from argus_synchro.config.app_config import AppConfig
    from argus_synchro.edge_det.base import EdgeDetectionResult
    from argus_synchro.shared_app_config import SharedAppConfig
    from argus_synchro.shared_excepts import (
        SharedExcepts,
        SharedScrutinizerExcept,
        SharedVisualizeExcept,
    )


class VisualProcess(ProcessBase):
    __slots__ = (
        "Scruti_ex",
        "_MonitoredTime",
        "_TimeMonitor",
        "_accum_points_input",
        "_app_config",
        "_bouding_box_data",
        "_cliff_inputs",
        "_collision_detect_creator",
        "_det_point_immobile_gen",
        "_det_point_mobile_gen",
        "_err_config",
        "_fps_prof",
        "_initial_offsets",
        "_l_machine_col",
        "_last_elapsed_ms",
        "_last_updated",
        "_machine_immobile_points_measure",
        "_machine_mobile_points_measure",
        "_monitor",
        "_octotree_func",
        "_octotree_obj",
        "_pre_frame",
        "_sac",
        "_scene_camera",
        "_sec_visu_ex",
        "_tester",
        "_visual_ui",
        "camera",
        "cluster2entity",
        "collision_clusters",
        "keep_cluster2entity_by_camera",
        "scene",
        "sec",
        "ser",
    )

    def __init__(
        self,
        sac: SharedAppConfig,
        sec: SharedExcepts,
        ser: SharedErrors,
        sec_visu_ex: SharedVisualizeExcept,
        accum_points_input: MessageFlow[AccumPointsData],
        cliff_input: MessageFlow[EdgeDetectionResult],
        bouding_box_inputs: MessageFlow[CameraDetectionsData],
        activator: ProcessActivator,
    ) -> None:
        super().__init__(sec_visu_ex, activator, "VisualProcess")
        self._sec_visu_ex = sec_visu_ex
        self._accum_points_input: MessageFlow[AccumPointsData] = self._subscribe(
            accum_points_input,
        )
        self._cliff_inputs: MessageFlow[EdgeDetectionResult] = self._subscribe(
            cliff_input
        )
        self._bouding_box_data: MessageFlow[CameraDetectionsData] = self._subscribe(
            bouding_box_inputs,
        )

        self.camera: list[Camera]
        self._scene_camera: list[argus_synchro_lib.dataclass.Camera]
        self.collision_clusters: t_py_col_res = {}
        self.cluster2entity: dict[int | None, NodeEntity] = {}
        self.keep_cluster2entity_by_camera: list[dict[int | None, NodeEntity]]
        self._MonitoredTime: float = 0
        self._pre_frame: int = 0
        self._fps_prof: ProfFps = ProfFps(self.__class__.__name__, console=True)
        self._tester: Tester = Tester()

        self.sec: SharedExcepts = sec
        self._sac: SharedAppConfig = sac
        self.Scruti_ex: SharedScrutinizerExcept = sec.Scruti_ex
        self._ser: SharedErrors = ser
        self._last_elapsed_ms: int = 0

        # _startupで初期化
        self._octotree_func: OctoTreeFuncInterface
        self._visual_ui: GodotUIVisualizer
        self._l_machine_col: list[MachineCollisionBase]
        self._machine_mobile_points_measure: NDArray[np.float64]
        self._machine_immobile_points_measure: NDArray[np.float64]
        self._det_point_mobile_gen: DetectableCylinderPointMobile
        self._det_point_immobile_gen: DetectableCylinderPointImmobile
        self._octotree_obj: OctoTree
        self._initial_offsets: tuple[float, float, float]
        self._collision_detect_creator: AbstractCollisionDetectCreator
        self._monitor: cam_monitor.Monitoring
        self._TimeMonitor: ProcessTimeMonitor.ProcessTimeMonitor
        self._err_config: ErrorConfig

    def _get_SceneDescription(self) -> argus_synchro_lib.scene.SceneDescriptionConf:
        SceneDescription = argus_synchro_lib.scene.SceneDescriptionConf()
        SceneDescription.coarse_lo = self._app_config.SceneDescription.coarse_lo
        SceneDescription.coarse_hi = self._app_config.SceneDescription.coarse_hi
        SceneDescription.k_min = self._app_config.SceneDescription.k_min
        SceneDescription.h_ref_px = self._app_config.SceneDescription.h_ref_px
        SceneDescription.lo_gain = self._app_config.SceneDescription.lo_gain
        SceneDescription.hi_gain = self._app_config.SceneDescription.hi_gain
        SceneDescription.lo_floor = self._app_config.SceneDescription.lo_floor
        SceneDescription.hi_ceil = self._app_config.SceneDescription.hi_ceil
        SceneDescription.vertical_w_iou = (
            self._app_config.SceneDescription.vertical_w_iou
        )
        SceneDescription.vertical_w_scale = (
            self._app_config.SceneDescription.vertical_w_scale
        )
        SceneDescription.vertical_w_phi = (
            self._app_config.SceneDescription.vertical_w_phi
        )
        SceneDescription.final_threshold = (
            self._app_config.SceneDescription.final_threshold
        )
        SceneDescription.use_human_gate = (
            self._app_config.SceneDescription.use_human_gate
        )
        SceneDescription.H_min = self._app_config.SceneDescription.H_min
        SceneDescription.H_max = self._app_config.SceneDescription.H_max
        SceneDescription.W_min = self._app_config.SceneDescription.W_min
        SceneDescription.W_max = self._app_config.SceneDescription.W_max
        SceneDescription.D_min = self._app_config.SceneDescription.D_min
        SceneDescription.D_max = self._app_config.SceneDescription.D_max
        SceneDescription.tall_ratio_min = (
            self._app_config.SceneDescription.tall_ratio_min
        )
        return SceneDescription

    def _config_load(self) -> None:
        self._app_config: AppConfig = self._sac.read()
        self._last_updated: int = self._sac.last_updated
        reduced_load_config = self._app_config.ReducedLoadMode
        self._ser.reduced_load_mode.configure(
            reduced_load_config.many_points_ratio,
            reduced_load_config.few_points_ratio,
        )
        self.scene = argus_synchro_lib.scene.Scene(
            self._get_SceneDescription(),
            lambda level, msg: self._logger.log(int(level), msg),
        )

    def _err_config_load(self) -> None:
        self._err_config = self._ser.shared_err_conf.read()

        # NOTE: このプロセスで実施する全ての診断クラスのupdateをここに追加していく
        self._ser.state_errors_A_C[StateErrorIndex.PROCESSING_SPEED_DEGRADED].update(
            self._err_config
        )
        self._ser.state_errors_A_C[
            StateErrorIndex.PROCESSING_SPEED_DEGRADATION_TREND
        ].update(self._err_config)
        self._ser.state_errors_D[StateErrorDIndex.INVALID_DATA_INPUT].update(
            self._err_config
        )
        self._ser.state_errors_D[StateErrorDIndex.ARRAY_SHAPE_ERROR].update(
            self._err_config
        )
        self._ser.state_errors_D[StateErrorDIndex.FILE_IO_ERROR].update(
            self._err_config
        )
        self._ser.action_errors_A_C[
            ActionErrorIndex.CRANE_MODEL_FILE_MISSING
        ].update(self._err_config)
        self._ser.action_errors_A_C[
            ActionErrorIndex.MMAP_READ_WRITE_ERROR
        ].update(self._err_config)
        self._ser.module_errors[ModuleErrorIndex.INTEGRATE_2D3D_MODULE_ERROR].update(
            self._err_config
        )
        self._ser.module_errors[ModuleErrorIndex.VISUAL_MODULE_ERROR].update(
            self._err_config
        )
        self._ser.module_errors[
            ModuleErrorIndex.COLLISION_JUDGMENT_MODULE_ERROR
        ].update(self._err_config)
        self._ser.module_errors[
            ModuleErrorIndex.OBJECT_3D_DETECTION_MODULE_ERROR
        ].update(self._err_config)

    def _report_file_io_error(self, path: str, error: Exception) -> None:
        file_io_error = self._ser.state_errors_D[StateErrorDIndex.FILE_IO_ERROR]
        result = file_io_error.errors_diagnosis(True)
        file_io_error.log_output(
            *result,
            StateErrorDIndex.FILE_IO_ERROR,
            path,
            "read class name text file",
            f"{type(error).__name__}: {error}",
        )

    def _recover_file_io_error(self) -> None:
        self._ser.state_errors_D[
            StateErrorDIndex.FILE_IO_ERROR
        ].errors_diagnosis(False)

    def _input_data_diagnosis(
        self,
        *shape_targets: tuple[str, object],
    ) -> bool:
        array_shape_error = self._ser.state_errors_D[StateErrorDIndex.ARRAY_SHAPE_ERROR]
        result, failsafe_result = array_shape_error.errors_diagnosis(*shape_targets)
        array_shape_error.log_output(
            result, failsafe_result, StateErrorDIndex.ARRAY_SHAPE_ERROR, self.name
        )
        return result == ResultDiagnosis.DETECTION

    def input_3d_data_diagnosis(
        self,
        accum_pcds: AccumPointsData,
        edge_result: EdgeDetectionResult,
    ) -> bool:
        invalid_data_input = self._ser.state_errors_D[
            StateErrorDIndex.INVALID_DATA_INPUT
        ]
        result, failsafe_result = invalid_data_input.errors_diagnosis(
            (accum_pcds, edge_result)
        )
        invalid_data_input.log_output(
            result, failsafe_result, StateErrorDIndex.INVALID_DATA_INPUT, self.name
        )
        if result == ResultDiagnosis.DETECTION:
            return True

        return self._input_data_diagnosis(
            ("accum_pcds_point_cloud", accum_pcds.point_cloud),
            ("edge_points", edge_result.edge_points),
            ("edge_lines", edge_result.edge_lines),
            ("edge_length", edge_result.edge_length),
        )

    def input_2d3d_data_diagnosis(
        self,
        bb_box: CameraDetectionsData,
        boxes: NDArray[np.float64],
        minmax: NDArray[np.float32],
        valid_detects: NDArray[np.int32],
    ) -> bool:
        invalid_data_input = self._ser.state_errors_D[
            StateErrorDIndex.INVALID_DATA_INPUT
        ]
        result, failsafe_result = invalid_data_input.errors_diagnosis(
            (bb_box, boxes, minmax, valid_detects)
        )
        invalid_data_input.log_output(
            result, failsafe_result, StateErrorDIndex.INVALID_DATA_INPUT, self.name
        )
        if result == ResultDiagnosis.DETECTION:
            return True

        return self._input_data_diagnosis(
            ("camera_detections_boxes", bb_box.boxes),
            ("camera_detections_scores", bb_box.scores),
            ("camera_detections_classes", bb_box.classes),
            ("camera_detections_valid_detects", bb_box.valid_detects),
            ("camera_detections_image", bb_box.image),
            ("visual_2d3d_boxes", boxes),
            ("minmax", minmax),
            ("valid_detects", valid_detects),
        )

    def input_ui_data_diagnosis(
        self,
        boxes: NDArray[np.float64],
        minmax: NDArray[np.float32],
        valid_detects: NDArray[np.int32],
        frames: list[NDArray[np.uint8]],
    ) -> bool:
        invalid_data_input = self._ser.state_errors_D[
            StateErrorDIndex.INVALID_DATA_INPUT
        ]
        result, failsafe_result = invalid_data_input.errors_diagnosis(
            (boxes, minmax, valid_detects, frames)
        )
        invalid_data_input.log_output(
            result, failsafe_result, StateErrorDIndex.INVALID_DATA_INPUT, self.name
        )
        if result == ResultDiagnosis.DETECTION:
            return True

        return self._input_data_diagnosis(
            ("visual_d3_boxes", boxes),
            ("minmax", minmax),
            ("valid_detects", valid_detects),
            ("images", tuple(frames)),
        )

    def _apply_parameters(self) -> None:
        # 再生成
        self.scene.update(
            self._get_SceneDescription(),
            lambda level, msg: self._logger.log(int(level), msg),
        )
        # update_module
        # VisualProcess(Octotree)
        if self._det_point_mobile_gen.is_required_only_mobile_value_update(
            self._app_config.machine.offset_rotate_center,
            self._app_config.CollisionDetection.key_num,
        ):
            self._det_point_mobile_gen.update_only_mobile_value(
                self._app_config.machine.offset_rotate_center,
                self._app_config.CollisionDetection.key_num,
                self._machine_mobile_points_measure,
            )

        if isinstance(self._collision_detect_creator, CollisionDetectOnCreator):
            self._collision_detect_creator.update(
                # *** 多分動くが出社時に確認 ***
                coord_method=CoordMethod.from_string(
                    self._app_config.CollisionDetection.coord_method
                )
            )

        # VisualProcess(CloudYolo)
        self._monitor.update(
            self._app_config.Monitor.show_cam, self._app_config.detect2d
        )
        ui_conf = UIIFConf(
            damp_out=self._app_config.UI_IF.damp_out,
            bbox_3d_num=self._app_config.UI_IF.bbox_3d_num,
            bbox_3d_distance=self._app_config.UI_IF.bbox_3d_distance,
            UI_mmap=self._app_config.UI_IF.UI_mmap,
            damp_mmap=self._app_config.UI_IF.damp_mmap,
            show_unk=self._app_config.UI_IF.show_unk,
            collision_depict_dist=self._app_config.UI_IF.collision_depict_dist,
            collision_attention_dist=self._app_config.UI_IF.collision_attention_dist,
            collision_warning_dist=self._app_config.UI_IF.collision_warning_dist,
            cliff_attention_dist=self._app_config.UI_IF.cliff_attention_dist,
            cliff_warning_dist=self._app_config.UI_IF.cliff_warning_dist,
            draw_bbox_3d=self._app_config.UI_IF.draw_bbox_3d,
            draw_collision=self._app_config.UI_IF.draw_collision,
        )
        general_conf = GeneralConf(
            in_factory=self._app_config.General.in_factory,
            operation_mode=self._app_config.General.operation_mode,
            has_external_guard=self._app_config.General.has_external_guard,
            external_guard_offset=self._app_config.General.external_guard_offset,
            ground_height=self._app_config.General.ground_height,
            ground_height_margin=self._app_config.General.ground_height_margin,
            rotation_radius=self._app_config.General.rotation_radius,
            initial_transform_file=self._app_config.General.initial_transform_file,
        )
        self._visual_ui.update(
            ui_conf,
            general_conf,
            lambda level, msg: self._logger.log(int(level), msg),
        )
        # update_octotree_module
        # VisualProcess(Octotree)
        self.update_octotree_module()

        # キャリブレーション設定変更時にカメラパラメータを再構築
        self._build_camera()

    def _build_camera(self) -> None:
        """sac から読み込んだ AppConfig を使って Camera を構築する。"""
        from argus_synchro.Camera import Camera, SyscamRes

        syscamres = SyscamRes(
            self._app_config.camera.sys_width, self._app_config.camera.sys_height
        )
        self.camera = [
            Camera(i, self._app_config.calibration, syscam_res=syscamres)
            for i in range(self._app_config.camera.count)
        ]
        self._scene_camera = [
            argus_synchro_lib.dataclass.Camera(
                c.index, c.width, c.height, c.rvec, c.tvec, c.ncm1, c.extrmat
            )
            for c in self.camera
        ]
        self.keep_cluster2entity_by_camera = [{} for _ in range(len(self.camera))]

    def _create_machine_points(self):
        diagnosis = self._ser.action_errors_A_C[
            ActionErrorIndex.CRANE_MODEL_FILE_MISSING
        ]
        try:
            return SubScrutinizer.create_machine_points(
                self._app_config.OctoTree.col_machine_dir,
                self._app_config.LiDARPosition,
                self._app_config.OctoTree.json_col_machine_file,
            )
        except Exception as error:
            is_target = diagnosis.excepts_diagnosis(error)
            diagnosis.log_output(
                is_target,
                False,
                ActionErrorIndex.CRANE_MODEL_FILE_MISSING,
                error,
            )
            raise

    def _create_visual_ui(self, ui_conf: UIIFConf) -> GodotUIVisualizer:
        try:
            return GodotUIVisualizer(
                ui_conf,
                self._app_config.Scrutinizer.s_frame,
                self._app_config.General.rotation_radius,
                self._app_config.camera.count,
                self._app_config.General.has_external_guard,
                self._app_config.General.external_guard_offset,
                str(paths.get_mmap_dir(self._directory_config, "status.mmap")),
                lambda level, msg: self._logger.log(int(level), msg),
            )
        except (OSError, ValueError, BufferError, RuntimeError) as error:
            diagnosis = self._ser.action_errors_A_C[
                ActionErrorIndex.MMAP_READ_WRITE_ERROR
            ]
            is_target = diagnosis.excepts_diagnosis(error)
            diagnosis.log_output(
                is_target,
                False,
                ActionErrorIndex.MMAP_READ_WRITE_ERROR,
                "Godot UI MMAP initialization failed",
                error,
            )
            raise

    def _startup(self) -> None:
        self._config_load()
        self._err_config_load()
        self.start_diagnosis()
        self._build_camera()

        if self._app_config.OctoTree.func_on:
            self._octotree_func = OctoTreeFuncOn(self._app_logger_factory)
        else:
            self._octotree_func = OctoTreeFuncOff()

        # YOLOの結果重畳
        ui_conf = UIIFConf(
            damp_out=self._app_config.UI_IF.damp_out,
            bbox_3d_num=self._app_config.UI_IF.bbox_3d_num,
            bbox_3d_distance=self._app_config.UI_IF.bbox_3d_distance,
            UI_mmap=self._app_config.UI_IF.UI_mmap,
            damp_mmap=self._app_config.UI_IF.damp_mmap,
            show_unk=self._app_config.UI_IF.show_unk,
            collision_depict_dist=self._app_config.UI_IF.collision_depict_dist,
            collision_attention_dist=self._app_config.UI_IF.collision_attention_dist,
            collision_warning_dist=self._app_config.UI_IF.collision_warning_dist,
            cliff_attention_dist=self._app_config.UI_IF.cliff_attention_dist,
            cliff_warning_dist=self._app_config.UI_IF.cliff_warning_dist,
            draw_bbox_3d=self._app_config.UI_IF.draw_bbox_3d,
            draw_collision=self._app_config.UI_IF.draw_collision,
        )
        self._visual_ui = self._create_visual_ui(ui_conf)

        (
            self._l_machine_col,
            self._machine_mobile_points_measure,
            self._machine_immobile_points_measure,
        ) = self._create_machine_points()

        self._det_point_mobile_gen, self._det_point_immobile_gen = (
            SubScrutinizer.initialize_detectable_point_generators(
                machine_mobile_points=self._machine_mobile_points_measure,
                machine_immobile_points=self._machine_immobile_points_measure,
                detectable_tree_depth=self._app_config.OctoTree.max_tree_depth
                - self._app_config.CollisionDetection.dialate_point_size,
                z_range=get_detectable_z_range(
                    self._app_config.General,
                    self._app_config.CollisionDetection,
                ),
                max_dist=self._app_config.CollisionDetection.max_dist,
                grid_intervals=self._app_config.CollisionDetection.grid_intervals,
                min_radius=self._app_config.CollisionDetection.min_radius,
                max_radius=self._app_config.CollisionDetection.max_radius,
                key_num=self._app_config.CollisionDetection.key_num,
                octotree_conf=self._app_config.OctoTree,
                dialate_point_size=self._app_config.CollisionDetection.dialate_point_size,
                offset_rotate_center=self._app_config.machine.offset_rotate_center,
            )
        )

        self._octotree_obj = SubScrutinizer.initialize_octotree(
            self._machine_immobile_points_measure,
            self._machine_mobile_points_measure,
            self._app_config.OctoTree.max_xyz,
            self._app_config.OctoTree.min_xyz,
            self._app_config.OctoTree.max_tree_depth,
            self._app_config.OctoTree.use_node_stats,
            self._app_config.CollisionDetection.dialate_point_size,
            origin_w2oct=(0.0, 0.0, 0.0),
        )
        self._octotree_obj = SubScrutinizer.put_immobile_points_to_octotree(
            octotree_obj=self._octotree_obj,
            machine_immobile_points_measure=self._machine_immobile_points_measure,
            machine_immobile_points_detect=self._det_point_immobile_gen.get_detectable_points(
                yaw_angle=None
            ),
            machine_center=self._app_config.machine.offset_rotate_center,
        )

        self._initial_offsets = (
            self._app_config.LiDARPosition.x_offset,
            self._app_config.LiDARPosition.y_offset,
            self._app_config.LiDARPosition.z_offset,
        )

        self.update_octotree_module()

        self._collision_detect_creator = SubScrutinizer.initialize_collision_detector(
            self._app_config.CollisionDetection.func_on,
            self._app_config.CollisionDetection.collision_detector_name,
            self._app_config.CollisionDetection.coord_method,
        )

        self._monitor = cam_monitor.Monitoring(
            show_cam=self._app_config.Monitor.show_cam,
            detect2d_conf=self._app_config.detect2d,
            app_logger_factory=self._app_logger_factory,
            file_io_error_callback=self._report_file_io_error,
            file_io_recovery_callback=self._recover_file_io_error,
        )

        self._TimeMonitor = ProcessTimeMonitor.ProcessTimeMonitor(
            fast_threshold_ms=self._app_config.Scrutinizer.fast_th_ms,
            slow_threshold_ms=self._app_config.Scrutinizer.slow_th_ms,
            short_que_length=self._app_config.Scrutinizer.short_que,
            long_que_length=self._app_config.Scrutinizer.long_que,
        )

        self._pre_frame = 0
        self._MonitoredTime: float = time.perf_counter()
        self._fps_prof.start()
        self.create_producer_and_consumer()
        # NOTE: 動作テスト用
        # self._tester.import_()

    def create_producer_and_consumer(self) -> None:
        self.accum_points_consumer: Consumer[AccumPointsData] = (
            self._accum_points_input.create_consumer()
        )
        self.bb_box_consumer: Consumer[CameraDetectionsData] = (
            self._bouding_box_data.create_consumer()
        )
        self.cliff_consumer: Consumer[EdgeDetectionResult] = (
            self._cliff_inputs.create_consumer()
        )

    def restart_completed(self) -> None:
        self.accum_points_consumer.restart_completed()
        self.bb_box_consumer.restart_completed()
        self.cliff_consumer.restart_completed()

    def _log_register(self) -> None:
        super()._log_register()
        self._sac.log_register(self._app_logger_factory)
        self.sec.log_register(self._app_logger_factory)
        self._ser.log_register(self._app_logger_factory)
        SubScrutinizer.log_register(self._app_logger_factory)

    def _shutdown(self) -> None:
        self.stop_diagnosis()
        self._visual_ui.close()
        self._fps_prof.export()
        # NOTE: 動作テスト用
        # self._tester.export()

    def start_diagnosis(self) -> None:
        self._sec_visu_ex.last_heartbeat.value = INVALID_TIMESTAMP
        self._sec_visu_ex.is_heartbeat_enabled.value = True

    def stop_diagnosis(self) -> None:
        self._sec_visu_ex.is_heartbeat_enabled.value = False

    def _start_restart(self) -> None:
        self._config_load()
        self._build_camera()

    @log_main()
    def _loop(self) -> None:
        try:
            while self.enable:
                if self._sac.last_updated > self._last_updated:
                    self._config_load()
                    self._apply_parameters()
                try:
                    if (
                        not self.accum_points_consumer.wait()
                        or not self.cliff_consumer.wait()
                    ):
                        continue

                    with ExitStack() as stack:
                        accum_pcds: AccumPointsData = stack.enter_context(
                            self.accum_points_consumer.consume()
                        )
                        edge_result: EdgeDetectionResult = stack.enter_context(
                            self.cliff_consumer.consume()
                        )

                        if accum_pcds.frame == self._pre_frame:
                            time.sleep(0.001)
                            continue
                        self._pre_frame = accum_pcds.frame

                        if self.input_3d_data_diagnosis(accum_pcds, edge_result):
                            continue

                        tmp: (
                            tuple[
                                NDArray[np.float64],
                                NDArray[np.float64],
                                NDArray[np.float32],
                                NDArray[np.int32],
                            ]
                            | None
                        ) = self._update_3d(accum_pcds, edge_result)

                        try:
                            # BoundingBoxの更新待機
                            if not self.bb_box_consumer.wait():
                                continue
                            # 入力処理(BoundingBox)
                            bb_box: CameraDetectionsData = stack.enter_context(
                                self.bb_box_consumer.consume()
                            )

                            if tmp is None or len(tmp) != 4:
                                continue
                            downsampled_accum_points: NDArray[np.float64] = tmp[0]
                            boxes: NDArray[np.float64] = tmp[1]
                            minmax: NDArray[np.float32] = tmp[2]
                            valid_detects: NDArray[np.int32] = tmp[3]
                            if self.input_2d3d_data_diagnosis(
                                bb_box,
                                boxes,
                                minmax,
                                valid_detects,
                            ):
                                continue

                            frames: list[NDArray[np.uint8]] = self._update_2d3d(
                                bb_box, boxes, minmax, valid_detects
                            )
                            if self.input_ui_data_diagnosis(
                                boxes,
                                minmax,
                                valid_detects,
                                frames,
                            ):
                                continue
                        except Exception as e:
                            is_state_error_d_exception = (
                                self._ser.is_state_error_d_exception(e, self._logger)
                            )
                            if not is_state_error_d_exception:
                                if self._ser.module_errors[
                                    ModuleErrorIndex.INTEGRATE_2D3D_MODULE_ERROR
                                ].excepts_diagnosis(e):
                                    self._ser.module_errors[
                                        ModuleErrorIndex.INTEGRATE_2D3D_MODULE_ERROR
                                    ].log_output(
                                        ResultDiagnosis.DETECTION,
                                        ResultDiagnosis.DETECTION,
                                        ModuleErrorIndex.INTEGRATE_2D3D_MODULE_ERROR,
                                        e,
                                    )
                                else:
                                    raise e

                        self._update(
                            accum_pcds,
                            downsampled_accum_points,
                            bb_box,
                            boxes,
                            minmax,
                            valid_detects,
                            frames,
                        )
                        self._sec_visu_ex.last_heartbeat.value = time.monotonic()

                except Exception as e:
                    is_state_error_d_exception = self._ser.is_state_error_d_exception(
                        e, self._logger
                    )
                    if not is_state_error_d_exception:
                        if self._ser.module_errors[
                            ModuleErrorIndex.VISUAL_MODULE_ERROR
                        ].excepts_diagnosis(e):
                            self._ser.module_errors[
                                ModuleErrorIndex.VISUAL_MODULE_ERROR
                            ].log_output(
                                ResultDiagnosis.DETECTION,
                                ResultDiagnosis.DETECTION,
                                ModuleErrorIndex.VISUAL_MODULE_ERROR,
                                e,
                            )
                        else:
                            raise e
        except KeyboardInterrupt:
            self._logger.info("KeyboardInterrupt を検知して終了")

        finally:
            self._logger.warning(
                "VisualProcess loop ended: process_activator=%s restart_required=%s "
                "flow_activators=%s",
                self._process_activator.value,
                self._process_activator.is_restart_required,
                [
                    self._accum_points_input.activator.value,
                    self._cliff_inputs.activator.value,
                    self._bouding_box_data.activator.value,
                ],
            )
            self.sec.Scruti_ex.IsFinished.value = True
            self._logger.info("終了条件に到達.")

    @log_target("描画", category=ProfCategory.Process)
    def _update_3d(
        self,
        accum_pcd_input: AccumPointsData,
        edge_result: EdgeDetectionResult,
    ) -> (
        tuple[
            NDArray[np.float64],
            NDArray[np.float64],
            NDArray[np.float64],
            NDArray[np.int32],
        ]
        | None
    ):
        try:
            self._fps_prof.enter()
            # remove redundant elements in octotree
            self._octotree_obj.erase_nodes_for_entities_noret(
                [
                    NodeEntity.UNK,
                    NodeEntity.OTHER,
                    NodeEntity.CLIFF,
                    NodeEntity.CRANE_MOBILE,
                    NodeEntity.CRANE_MOBILE_FOR_DET,
                    NodeEntity.HUMAN,
                    NodeEntity.LOW_3D,
                ]
            )
            # self._logger.info(
            #     "each points = %s",
            #     {key: len(val) for key, val in self._octotree_obj.entity_octonodes.items()},
            # )

            start_time: float = time.time()
            downsampled_accum_points, self._octotree_obj = (
                self._octotree_func.octotree_accum(
                    accum_pcd_input.point_cloud,
                    self._octotree_obj,
                    NodeEntity.OTHER,
                    self._app_config.OctoTree.clustering_tree_depth,
                )
            )

            self._logger.info(
                "八分木点群にかかった時間: %s%s",
                time.time() - start_time,
                f" number of points = {len(downsampled_accum_points)}",
            )

            # self._logger.info(
            #     "each points = %s",
            #     {key: len(val) for key, val in self._octotree_obj.entity_octonodes.items()},
            # )
            if (
                len(downsampled_accum_points) == 0
            ):  # 原因不明だが,空配列が返ってくることがある.(20240314_130128 ref_t=3267)
                self._logger.warning("クラスタリングのデータ数が0です")
                return None
            if len(downsampled_accum_points) > PcdData.SIZE:
                downsampled_accum_points: NDArray[np.float64] = (
                    downsampled_accum_points[: PcdData.SIZE]
                )
                self._logger.warning(
                    "クラスタリングのデータ数が、クラスタを記録できる最大数を超えているため、最大数に合わせます",
                )

            # 衝突判定
            boxes: NDArray[np.float64]
            minmax: NDArray[np.float64]
            valid_detects: NDArray[np.int32]
            labels: NDArray[np.int32]
            start_time = time.time()

            with log_target_area("main_accum", ProfCategory.Process, d3.main_accum):
                boxes, _, minmax, valid_detects, labels = d3.main_accum(
                    downsampled_accum_points,
                    self._app_config.DEFAULT.debug_log,
                    self._app_config.detect3d.eps,
                    self._app_config.detect3d.min_samples,
                    lambda level, msg: self._logger.log(int(level), msg),
                )

            clustering_labels: NDArray[np.int32] = labels[
                : len(downsampled_accum_points)
            ]
            self._octotree_func.clustering_result(
                self._octotree_obj,
                clustered_data=downsampled_accum_points,
                labels=clustering_labels[: len(downsampled_accum_points)],
                start_time=start_time,
                cluster_entity=NodeEntity.OTHER,
                cluster_fail_table={-1: NodeEntity.UNK},
            )

            ##### 機体回転 ####
            yaw_angle: float = np.deg2rad(accum_pcd_input.yaw_angle_deg)
            self._octotree_func.update_machine_mobile(
                machine_mobile_points_measure=self._machine_mobile_points_measure,
                machine_mobile_points_detect=self._det_point_mobile_gen.get_detectable_points(
                    yaw_angle=yaw_angle
                ),
                octotree_obj=self._octotree_obj,
                yaw_angle=yaw_angle,
            )
        except Exception as e:
            is_state_error_d_exception = self._ser.is_state_error_d_exception(
                e, self._logger
            )
            if not is_state_error_d_exception:
                if self._ser.module_errors[
                    ModuleErrorIndex.OBJECT_3D_DETECTION_MODULE_ERROR
                ].excepts_diagnosis(e):
                    self._ser.module_errors[
                        ModuleErrorIndex.OBJECT_3D_DETECTION_MODULE_ERROR
                    ].log_output(
                        ResultDiagnosis.DETECTION,
                        ResultDiagnosis.DETECTION,
                        ModuleErrorIndex.OBJECT_3D_DETECTION_MODULE_ERROR,
                        e,
                    )
                else:
                    raise e

        try:
            ########### 衝突判定 #############
            collision_clusters: t_py_col_res = (
                self._collision_detect_creator.collision_detection(
                    octotree_obj=self._octotree_obj,
                    app_config=self._app_config,
                )
            )

            if self._logger.is_enabled_for(app_logger.INFO):
                for key, octonodes in self._octotree_obj.entity_octonodes.items():
                    if key.entity in [
                        NodeEntity.OTHER,
                        NodeEntity.UNK,
                        NodeEntity.HUMAN,
                    ] and isinstance(octonodes, dict):
                        vox_coords = np.array(list(octonodes.keys()))

                        self._logger.debug(
                            "クラスタリングの最大最小:%s %s %s",
                            key.cluster_id,
                            vox_coords.min(axis=0),
                            vox_coords.max(axis=0),
                        )

            with log_target_area(
                "append_distance_info",
                ProfCategory.Process,
                self.scene.append_distance_info,
            ):
                self.collision_clusters = self.scene.append_distance_info(
                    collision_clusters,
                    minmax,
                )
            # self._logger.info("衝突判定結果: %s", self.collision_clusters)

            self._logger.info(
                "衝突判定にかかった時間: %f",
                time.time() - start_time,
            )

            # 実行結果を八分木に書き込む
            self._put_result_to_octree(self._octotree_obj, edge_result)
        except Exception as e:
            is_state_error_d_exception = self._ser.is_state_error_d_exception(
                e, self._logger
            )
            if not is_state_error_d_exception:
                if self._ser.module_errors[
                    ModuleErrorIndex.COLLISION_JUDGMENT_MODULE_ERROR
                ].excepts_diagnosis(e):
                    self._ser.module_errors[
                        ModuleErrorIndex.COLLISION_JUDGMENT_MODULE_ERROR
                    ].log_output(
                        ResultDiagnosis.DETECTION,
                        ResultDiagnosis.DETECTION,
                        ModuleErrorIndex.COLLISION_JUDGMENT_MODULE_ERROR,
                        e,
                    )
                else:
                    raise e
        return downsampled_accum_points, boxes, minmax, valid_detects

    @log_target("描画", category=ProfCategory.Process)
    def _update_2d3d(
        self,
        bb_box_inputs: CameraDetectionsData,
        boxes: NDArray[np.float64],
        minmax: NDArray[np.float32],
        valid_detects: NDArray[np.int32],
    ) -> list[NDArray[np.uint8]]:
        frames: list[NDArray[np.uint8]] = []
        self.keep_cluster2entity_by_camera = [
            {} for _ in range(self._app_config.camera.count)
        ]

        # for bb_box_data in bb_box_input:
        for i in range(self._app_config.camera.count):
            bb_box_data = argus_synchro_lib.dataclass.CameraDetectionData(
                bb_box_inputs.boxes[i],
                np.zeros((1, 1), np.float32),
                bb_box_inputs.classes[i].reshape(1, 50),
                bb_box_inputs.valid_detects[i],
            )

            with log_target_area(
                "integrate2d3d", ProfCategory.Process, self.scene.integrate2d3d
            ):
                camera_cluster2entity = self.scene.integrate2d3d(
                    self._scene_camera[i],
                    bb_box_data,
                    boxes,
                    minmax,
                    int(valid_detects[0]),
                    from_entity=NodeEntity.OTHER,
                )
                self.keep_cluster2entity_by_camera[i] = camera_cluster2entity

            frames.append(bb_box_inputs.image[i])

        with log_target_area(
            "aggregate2d3d_results",
            ProfCategory.Process,
            self.scene.aggregate2d3d_results,
        ):
            self._octotree_obj, self.cluster2entity = self.scene.aggregate2d3d_results(
                self.keep_cluster2entity_by_camera,
                self._octotree_obj,
                from_entity=NodeEntity.OTHER,
            )
        return frames

    @log_target("描画", category=ProfCategory.Process)
    def _update(
        self,
        accum_pcd_input: AccumPointsData,
        downsampled_accum_points: NDArray[np.float64],
        bb_box_inputs: CameraDetectionsData,
        boxes: NDArray[np.float64],
        minmax: NDArray[np.float32],
        valid_detects: NDArray[np.int32],
        frames: list[NDArray[np.uint8]],
    ) -> None:
        cpp_camera: list[argus_synchro_lib.dataclass.Camera] = self._scene_camera
        cpp_bbox_inputs: list[argus_synchro_lib.dataclass.CameraDetectionData] = [
            argus_synchro_lib.dataclass.CameraDetectionData(
                bb_box_inputs.boxes[i],
                bb_box_inputs.scores[i],
                bb_box_inputs.classes[i],
                bb_box_inputs.valid_detects[i],
            )
            for i in range(bb_box_inputs.valid_detects.shape[0])
        ]
        with log_target_area("summary", ProfCategory.Process, self._visual_ui.summary):
            self._visual_ui.summary(
                self.sec.Scruti_ex.IsSlow.value,
                boxes,
                minmax,
                valid_detects,
                self._octotree_obj,
                accum_pcd_input.yaw_angle_deg,
                frames,
                cpp_bbox_inputs,
                cpp_camera,
                self.collision_clusters,
                self.cluster2entity,
                accum_pcd_input.frame,
                self._app_config.OctoTree.max_tree_depth,
                self._app_config.CollisionDetection.dialate_point_size,
                self._app_config.OctoTree.func_on,
                self._app_config.CollisionDetection.func_on,
                self._app_config.Visualizer.display_octree,
                self._app_config.UI_IF.damp_out,
                self._last_elapsed_ms,
            )
        # self._logger.info(
        #     "each points = %s",
        #     {key: len(val) for key, val in self._octotree_obj.entity_octonodes.items()},
        # )
        self.sec.frame_number.value = accum_pcd_input.frame
        self.monitor_process_time(self.sec, self._TimeMonitor)

        # 負荷低減モード判定
        self._ser.reduced_load_mode.update_proc_speed(self.sec.Scruti_ex.IsSlow.value)
        self._ser.reduced_load_mode.update_state()
        self._logger.info(
            f"負荷低減モード: {'有効' if self._ser.reduced_load_mode.enabled else '無効'}"
        )

        self._fps_prof.prof(
            pcd_frame=accum_pcd_input.frame,
            camera_frame=bb_box_inputs.frame,
            pcd_s_time=accum_pcd_input.time,
            camera_s_time=bb_box_inputs.time,
            points=accum_pcd_input,
            downsample=downsampled_accum_points,
            octotree=self._octotree_obj,
            camera_detection=bb_box_inputs,
        )

        # NOTE: 動作テスト用
        # self._tester.test(
        #     accum_pcd_input.frame,
        #     accum_pcd_input,
        #     downsampled_accum_points,
        #     boxes,
        #     # lines,
        #     minmax,
        #     valid_detects,
        #     # labels,
        #     # edge_result,
        #     # obj_results,
        #     self._octotree_obj,
        #     angle_input_data.yaw_angle_deg,
        # )

    def update_octotree_module(self) -> None:
        """
        octotree関係のパラメータを更新する
        """

        # 更新する必要があるかを確認する。
        if self._det_point_mobile_gen.is_required_octotree_update(
            self._initial_offsets,
            self._app_config.OctoTree,
            self._app_config.CollisionDetection,
            self._app_config.LiDARPosition,
            self._app_config.General,
        ):
            self._initial_offsets = (
                self._app_config.LiDARPosition.x_offset,
                self._app_config.LiDARPosition.y_offset,
                self._app_config.LiDARPosition.z_offset,
            )
            self._det_point_mobile_gen.update_octotree_value(
                self._app_config.OctoTree,
                self._app_config.CollisionDetection,
                self._app_config.General,
                self._machine_mobile_points_measure,
            )
            self._det_point_immobile_gen.update_octotree_value(
                self._app_config.OctoTree,
                self._app_config.CollisionDetection,
                self._app_config.General,
                self._machine_mobile_points_measure,
            )
            self._octotree_obj = SubScrutinizer.initialize_octotree(
                self._machine_immobile_points_measure,
                self._machine_mobile_points_measure,
                self._app_config.OctoTree.max_xyz,
                self._app_config.OctoTree.min_xyz,
                self._app_config.OctoTree.max_tree_depth,
                self._app_config.OctoTree.use_node_stats,
                self._app_config.CollisionDetection.dialate_point_size,
                origin_w2oct=(0.0, 0.0, 0.0),
            )

            self._octotree_obj = SubScrutinizer.put_immobile_points_to_octotree(
                octotree_obj=self._octotree_obj,
                machine_immobile_points_measure=self._machine_immobile_points_measure,
                machine_immobile_points_detect=self._det_point_immobile_gen.get_detectable_points(
                    yaw_angle=None
                ),
                machine_center=self._app_config.machine.offset_rotate_center,
            )

    def monitor_process_time(
        self,
        sec: SharedExcepts,
        time_monitor: PTMonitor.ProcessTimeMonitor,
    ) -> None:
        # 処理時間監視[msec]
        now: float = time.perf_counter()
        elapsed_ms: float = (now - self._MonitoredTime) * 1000
        self._MonitoredTime: float = now
        self._last_elapsed_ms = int(elapsed_ms)
        process_time_status: PTMonitor.FrameResult = time_monitor.record_frame(
            elapsed_ms,
        )
        status: Literal["slow", "trend"] | None = process_time_status["status"]
        processing_speed_degraded = self._ser.state_errors_A_C[
            StateErrorIndex.PROCESSING_SPEED_DEGRADED
        ]
        processing_speed_degradation_trend = self._ser.state_errors_A_C[
            StateErrorIndex.PROCESSING_SPEED_DEGRADATION_TREND
        ]
        result = processing_speed_degraded.errors_diagnosis(status)
        processing_speed_degraded.log_output(
            *result, StateErrorIndex.PROCESSING_SPEED_DEGRADED, elapsed_ms
        )
        result = processing_speed_degradation_trend.errors_diagnosis(status)
        processing_speed_degradation_trend.log_output(
            *result, StateErrorIndex.PROCESSING_SPEED_DEGRADATION_TREND, elapsed_ms
        )

        if status == "slow":
            sec.Scruti_ex.IsSlow.value = 2
        elif status == "trend":
            sec.Scruti_ex.IsSlow.value = 1
        elif status is None:
            sec.Scruti_ex.IsSlow.value = 0
            self._logger.info(f"処理時間[msec] = {elapsed_ms},")

    def _put_result_to_octree(
        self, octree_obj: OctoTree, edge_result: EdgeDetectionResult
    ) -> None:
        # TODO: 崖はinsert_or_entity...関数内で初期化したいが、今はできない
        _ = octree_obj.erase_nodes_for_entities([NodeEntity.CLIFF])

        edge_ground_points, edge_labels = edge_result.get_edge_cluster()
        octree_obj.insert_or_entity_octonodes_with_labels(
            xyz=edge_ground_points,
            labels=edge_labels,
            is_order=True,
            entity=NodeEntity.CLIFF,
        )
