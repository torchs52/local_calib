from __future__ import annotations

import time
import typing
from collections import deque
from contextlib import ExitStack

import numpy as np
from argus_synchro_lib.octotree import NodeEntity, OctoTree

from argus_synchro import Registrate_LiDAR, SubScrutinizer
from argus_synchro.common.common import Err, Ok
from argus_synchro.crane_state_estimation import CraneStateEstimator
from argus_synchro.diagnosis.error_config import ErrorConfig
from argus_synchro.diagnosis.error_diagnosis import ResultDiagnosis
from argus_synchro.edge_det import create_edge_detection
from argus_synchro.edge_det.edge_det import EdgeDetectionPolar
from argus_synchro.interface.create_obj_cliff import (
    AppliedCreateObjNoOcclusion,
    AppliedCreateObjOcclusion,
    CreateObjCliffInterface,
    NotAppliedCreateObjCliff,
)
from argus_synchro.interface.filter_accum_points import (
    AccumAxisPointsCloud,
    FilterAccumPointsInterface,
    StaticAccumPoints,
)
from argus_synchro.interface.pcd_calib import MultiCalib, PCDCalibInterface, RCalib
from argus_synchro.interface.pcd_real_data import PCDDataInterface, PCDRealData
from argus_synchro.message.input_message import PointCloudData
from argus_synchro.message.scrutinizer_message import (
    AccumGroundPointsData,
    AccumPointsData,
    CanAngleData,
    CanLeverData,
    DeltaYawData,
    RemovePointsData,
)
from argus_synchro.process import ProcessBase
from argus_synchro.process.synchronizer import ProcessActivator
from argus_synchro.profiler import log_main, log_target
from argus_synchro.profiler.prof_fps import ProfFps
from argus_synchro.profiler.prof_mode import ProfCategory
from argus_synchro.py_octotree.detectable_points import get_detectable_z_range
from argus_synchro.shared_errors import (
    ActionErrorIndex,
    ModuleErrorIndex,
    SharedErrors,
    StateErrorDIndex,
)
from argus_synchro_lib import controller as octo_ctrl

if typing.TYPE_CHECKING:
    from argus_synchro_lib.machine_collision import MachineCollisionBase
    from numpy.typing import NDArray

    from argus_synchro.config.app_config import AppConfig
    from argus_synchro.edge_det.base import EdgeDetectionIF, EdgeDetectionResult
    from argus_synchro.process.message import Consumer, MessageFlow, Producer
    from argus_synchro.py_octotree import DetectableCylinderPointImmobile
    from argus_synchro.shared_app_config import SharedAppConfig
    from argus_synchro.shared_excepts import SharedScrutinizerExcept

from argus_synchro.shared_excepts import INVALID_TIMESTAMP


class PointsRefineProcess(ProcessBase):
    __slots__ = (
        "_accum_counter",
        "_accum_ground_dq",
        "_accum_output",
        "_accum_point_cloud",
        "_accum_points_dq",
        "_app_config",
        "_can_angle_input",
        "_can_lever_input",
        "_cliff_pcd_outputs",
        "_counter",
        "_crane_state_est",
        "_det_point_mobile_gen",
        "_display_detect_cliff",
        "_edge_detector",
        "_err_config",
        "_fps_prof",
        "_init_r",
        "_init_t",
        "_l_machine_col",
        "_last_accum_reduced_load_mode",
        "_last_updated",
        "_machine_immobile_points_measure",
        "_octotree_obj",
        "_pcd_inputs",
        "_pcd_proofreading",
        "_pcd_xyz",
        "_pre_frame",
        "_sac",
        "_ser",
        "yaw_pre_angle_deg",
    )

    def __init__(
        self,
        sac: SharedAppConfig,
        sec_scruti_ex: SharedScrutinizerExcept,
        ser: SharedErrors,
        pcd_inputs: tuple[MessageFlow[PointCloudData], ...],
        can_angle_input: MessageFlow[CanAngleData],
        can_lever_input: MessageFlow[CanLeverData],
        accum_output: MessageFlow[AccumPointsData],
        cliff_pcd_outputs: MessageFlow[EdgeDetectionResult],
        activator: ProcessActivator,
    ) -> None:
        super().__init__(sec_scruti_ex, activator, self.__class__.__name__)
        self._pcd_inputs: tuple[MessageFlow[PointCloudData], ...] = tuple(
            self._subscribe(pcd_input) for pcd_input in pcd_inputs
        )
        self._can_angle_input: MessageFlow[CanAngleData] = self._subscribe(
            can_angle_input,
        )
        self._can_lever_input: MessageFlow[CanLeverData] = self._subscribe(
            can_lever_input,
        )
        self._accum_output: MessageFlow[AccumPointsData] = self._subscribe(accum_output)
        self._cliff_pcd_outputs: MessageFlow[EdgeDetectionResult] = self._subscribe(
            cliff_pcd_outputs,
        )

        # init(remove)
        self.yaw_pre_angle_deg: int = 0

        # init(共通)
        self._pre_frame: int = 0
        self._sac: SharedAppConfig = sac
        self._fps_prof: ProfFps = ProfFps(self.__class__.__name__)
        self._ser: SharedErrors = ser

        self._init_r: NDArray[np.float64]
        self._init_t: NDArray[np.float64]
        self._app_config: AppConfig
        self._last_updated: int
        self._last_accum_reduced_load_mode: bool | None = None

        # _startupで初期化(3処理共通)
        self._err_config: ErrorConfig

        # _startupで初期化(remove)
        self._pcd_xyz: PCDDataInterface
        self._pcd_proofreading: PCDCalibInterface
        self._l_machine_col: list[MachineCollisionBase]
        self._det_point_mobile_gen: SubScrutinizer.DetectableCylinderPointMobile
        self._octotree_obj: OctoTree

        # _startupで初期化(accum)
        self._crane_state_est: CraneStateEstimator
        self._accum_point_cloud: FilterAccumPointsInterface

        # _startupで初期化(accum/collision_cliff共通)
        self._counter: NDArray[np.int32]
        self._accum_points_dq: deque[NDArray[np.float64]]
        self._accum_ground_dq: deque[NDArray[np.float64]]
        self._accum_counter: int

        # _startupで初期化(collision_cliff)
        self._machine_immobile_points_measure: NDArray[np.float64]
        self._edge_detector: EdgeDetectionIF
        self._display_detect_cliff: CreateObjCliffInterface

    def _config_load(self) -> None:
        self._app_config: AppConfig = self._sac.read()
        self._last_updated: int = self._sac.last_updated
        file_io_error = self._ser.state_errors_D[StateErrorDIndex.FILE_IO_ERROR]
        file_io_error.update(self._ser.shared_err_conf.read())
        csv_path = self._app_config.General.initial_transform_file
        try:
            self._init_r, self._init_t = SubScrutinizer.load_transform_csv(csv_path)
        except (OSError, UnicodeError, ValueError, TypeError) as error:
            result = file_io_error.errors_diagnosis(True)
            file_io_error.log_output(
                *result,
                StateErrorDIndex.FILE_IO_ERROR,
                csv_path,
                "read initial transform CSV",
                f"{type(error).__name__}: {error}",
            )
            raise
        file_io_error.errors_diagnosis(False)

    def _err_config_load(self) -> None:
        self._err_config = self._ser.shared_err_conf.read()

        # NOTE: このプロセスで実施する全ての診断クラスのupdateをここに追加していく
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
        self._ser.module_errors[ModuleErrorIndex.ACCUMULATION_MODULE_ERROR].update(
            self._err_config
        )
        self._ser.module_errors[ModuleErrorIndex.POINTS_REFINE_MODULE_ERROR].update(
            self._err_config
        )

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

    def input_remove_data_diagnosis(
        self,
        pcds: tuple[PointCloudData, ...],
        can_angle: CanAngleData,
    ) -> bool:
        invalid_data_input = self._ser.state_errors_D[
            StateErrorDIndex.INVALID_DATA_INPUT
        ]
        result, failsafe_result = invalid_data_input.errors_diagnosis((pcds, can_angle))
        invalid_data_input.log_output(
            result, failsafe_result, StateErrorDIndex.INVALID_DATA_INPUT, self.name
        )
        if result == ResultDiagnosis.DETECTION:
            return True

        pcds_point_cloud = tuple(pcd.point_cloud for pcd in pcds)
        return self._input_data_diagnosis(
            ("pcds_point_cloud", pcds_point_cloud),
        )

    def input_accum_data_diagnosis(
        self,
        removed_pcds: RemovePointsData,
        delta_yaw: DeltaYawData,
        can_lever: CanLeverData,
    ) -> bool:
        invalid_data_input = self._ser.state_errors_D[
            StateErrorDIndex.INVALID_DATA_INPUT
        ]
        result, failsafe_result = invalid_data_input.errors_diagnosis(
            (removed_pcds, delta_yaw, can_lever)
        )
        invalid_data_input.log_output(
            result, failsafe_result, StateErrorDIndex.INVALID_DATA_INPUT, self.name
        )
        if result == ResultDiagnosis.DETECTION:
            return True

        return self._input_data_diagnosis(
            ("removed_pcds_point_cloud", removed_pcds.point_cloud),
            ("lever_pressure", can_lever.lever_pressure),
        )

    def input_collision_cliff_data_diagnosis(
        self,
        accum_ground_pcd: AccumGroundPointsData,
    ) -> bool:
        invalid_data_input = self._ser.state_errors_D[
            StateErrorDIndex.INVALID_DATA_INPUT
        ]
        result, failsafe_result = invalid_data_input.errors_diagnosis(
            (accum_ground_pcd,)
        )
        invalid_data_input.log_output(
            result, failsafe_result, StateErrorDIndex.INVALID_DATA_INPUT, self.name
        )
        if result == ResultDiagnosis.DETECTION:
            return True

        return self._input_data_diagnosis(
            ("accum_ground_pcd_point_cloud", accum_ground_pcd.point_cloud),
        )

    def _log_register(self) -> None:
        super()._log_register()
        self._sac.log_register(self._app_logger_factory)
        self._ser.log_register(self._app_logger_factory)
        SubScrutinizer.log_register(self._app_logger_factory)
        Registrate_LiDAR.ndt.log_register(self._app_logger_factory)

    def _apply_parameters(self) -> None:
        # remove
        if self._app_config.calibration.calib_lidar2crane:
            self._pcd_proofreading = MultiCalib(
                self._app_config.Lidar,
                self._app_config.calibration,
            )  # Change_LiDAR
        else:
            self._pcd_proofreading = RCalib(self._app_config.calibration)
        if isinstance(self._pcd_xyz, PCDRealData):
            self._pcd_xyz.update(self._pcd_proofreading)

        # collision_cliff
        self._edge_detector.update(self._app_config.EdgeDetection)
        edge_det = self._edge_detector
        if self._app_config.EdgeDetection.is_applied:
            if isinstance(edge_det, EdgeDetectionPolar):
                self._display_detect_cliff = AppliedCreateObjOcclusion(
                    fwd_range=edge_det.fwd_range_cartesian,
                    side_range=edge_det.side_range_cartesian,
                    grid_size_cartesian=edge_det.grid_size_cartesian,
                    target_entity=NodeEntity.LOW_3D,
                    put_label=self._app_config.EdgeDetection.machine_occ_label,
                    app_logger_factory=self._app_logger_factory,
                )
            else:
                self._display_detect_cliff = AppliedCreateObjNoOcclusion(
                    self._app_logger_factory
                )
        else:
            self._display_detect_cliff = NotAppliedCreateObjCliff(
                self._app_logger_factory
            )

    def _startup_remove(self) -> None:
        machine_mobile_points_measure: NDArray[np.float64]
        machine_immobile_points_measure: NDArray[np.float64]
        (
            self._l_machine_col,
            machine_mobile_points_measure,
            machine_immobile_points_measure,
        ) = self._create_machine_points()

        if self._app_config.calibration.calib_lidar2crane:
            self._pcd_proofreading = MultiCalib(
                self._app_config.Lidar,
                self._app_config.calibration,
            )  # Change_LiDAR
        else:
            self._pcd_proofreading = RCalib(self._app_config.calibration)

        self._pcd_xyz = PCDRealData(self._pcd_proofreading)

        det_point_immobile_gen: DetectableCylinderPointImmobile
        self._det_point_mobile_gen, det_point_immobile_gen = (
            SubScrutinizer.initialize_detectable_point_generators(
                machine_mobile_points=machine_mobile_points_measure,
                machine_immobile_points=machine_immobile_points_measure,
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

        # 八分木インスタンスを生成
        self._octotree_obj = SubScrutinizer.initialize_octotree(
            machine_immobile_points_measure,
            machine_mobile_points_measure,
            self._app_config.OctoTree.max_xyz,
            self._app_config.OctoTree.min_xyz,
            self._app_config.OctoTree.max_tree_depth,
            self._app_config.OctoTree.use_node_stats,
            self._app_config.CollisionDetection.dialate_point_size,
            origin_w2oct=(0.0, 0.0, 0.0),
        )

        self._octotree_obj = SubScrutinizer.put_immobile_points_to_octotree(
            octotree_obj=self._octotree_obj,
            machine_immobile_points_measure=machine_immobile_points_measure,
            machine_immobile_points_detect=det_point_immobile_gen.get_detectable_points(
                yaw_angle=None
            ),
            machine_center=self._app_config.machine.offset_rotate_center,
        )

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

    def _startup_accum(self) -> None:
        self._crane_state_est = CraneStateEstimator(
            app_config=self._app_config,
        )

        if self._app_config.Accumulation.accum_point:
            self._accum_point_cloud = StaticAccumPoints()
        else:
            self._accum_point_cloud = AccumAxisPointsCloud()

        (
            self._counter,
            _,
            _,
            self._accum_points_dq,
            self._accum_ground_dq,
            self._accum_counter,
        ) = SubScrutinizer.initialize_accum_counter(
            self._app_config.LiDARGrid,
            self._app_config.Accumulation,
        )

    def _startup_collision_cliff(self) -> None:
        (
            _,
            machine_mobile_points_measure,
            self._machine_immobile_points_measure,
        ) = self._create_machine_points()

        result_edge_det = create_edge_detection(
            self._app_config,
            machine_mobile_points_measure,
        )

        # 崖検出インスタンスを作るのに失敗した場合は、空のメンバ変数がある想定で進めることにする
        match (
            self._app_config.EdgeDetection.is_applied,
            result_edge_det,
        ):
            case (_, Err(e)):
                # 崖検知インスタンスの作成に失敗した場合はlogger出力することにしている
                # 一先ずエラーの場合は、崖検出しない方向にしておいて、edge_detectorは初期化されない感じにする
                # TODO: 異常系をどうするか決めて必要なエラーハンドリングを行う
                self._logger.warning(
                    f"fail to create edge detector by e = {e}, and NotAppliedCreateObjCliff is applied."
                )
                self._display_detect_cliff = NotAppliedCreateObjCliff(
                    self._app_logger_factory
                )
            case (True, Ok(edge_det)) if isinstance(edge_det, EdgeDetectionPolar):
                self._edge_detector = edge_det
                self._display_detect_cliff = AppliedCreateObjOcclusion(
                    fwd_range=edge_det.fwd_range_cartesian,
                    side_range=edge_det.side_range_cartesian,
                    grid_size_cartesian=edge_det.grid_size_cartesian,
                    target_entity=NodeEntity.LOW_3D,
                    put_label=self._app_config.EdgeDetection.machine_occ_label,
                    app_logger_factory=self._app_logger_factory,
                )
            case (True, Ok(edge_det)):
                self._edge_detector = edge_det
                self._display_detect_cliff = AppliedCreateObjNoOcclusion(
                    self._app_logger_factory
                )
            case (False, Ok(edge_det)):
                self._edge_detector = edge_det
                self._display_detect_cliff = NotAppliedCreateObjCliff(
                    self._app_logger_factory
                )

    def _startup(self) -> None:
        self._config_load()
        self._err_config_load()
        self.start_diagnosis()
        self._startup_remove()
        self._startup_accum()
        self._startup_collision_cliff()

        self._pre_frame = 0
        self._fps_prof.start()
        self.create_producer_and_consumer()

    def create_producer_and_consumer(self) -> None:
        self.pcd_consumers: tuple[Consumer[PointCloudData], ...] = tuple(
            pcd.create_consumer() for pcd in self._pcd_inputs
        )
        self.can_angle_consumer: Consumer[CanAngleData] = (
            self._can_angle_input.create_consumer()
        )
        self.can_lever_consumer: Consumer[CanLeverData] = (
            self._can_lever_input.create_consumer()
        )

        self.accum_producer: Producer[AccumPointsData] = (
            self._accum_output.create_producer()
        )
        self.cliff_pcd_producer: Producer[EdgeDetectionResult] = (
            self._cliff_pcd_outputs.create_producer()
        )

    def restart_completed(self) -> None:
        for i in self.pcd_consumers:
            i.restart_completed()
        self.can_angle_consumer.restart_completed()
        self.can_lever_consumer.restart_completed()
        self.accum_producer.restart_completed()
        self.cliff_pcd_producer.restart_completed()

    def _start_restart(self) -> None:
        # TODO """必要に応じて実際にプロセスを落とさないで再起動で実行する処理を記載""" (NSW)
        pass

    def _shutdown(self) -> None:
        self.stop_diagnosis()
        self._fps_prof.export()

    def start_diagnosis(self) -> None:
        self._spe.last_heartbeat.value = INVALID_TIMESTAMP
        self._spe.is_heartbeat_enabled.value = True

    def stop_diagnosis(self) -> None:
        self._spe.is_heartbeat_enabled.value = False

    @log_main()
    def _loop(self) -> None:
        while self.enable:
            if self._sac.last_updated > self._last_updated:
                self._config_load()
                self._apply_parameters()
            try:
                if not self.accum_producer.wait() or not self.cliff_pcd_producer.wait():
                    continue

                if (
                    any(not i.wait() for i in self.pcd_consumers)
                    or not self.can_angle_consumer.wait()
                    or not self.can_lever_consumer.wait()
                ):
                    continue

                with ExitStack() as stack:
                    # 入力処理
                    pcds: tuple[PointCloudData, ...] = tuple(
                        stack.enter_context(c.consume()) for c in self.pcd_consumers
                    )
                    can_angle: CanAngleData = stack.enter_context(
                        self.can_angle_consumer.consume()
                    )
                    can_lever: CanLeverData = stack.enter_context(
                        self.can_lever_consumer.consume()
                    )

                    if pcds[0].frame == self._pre_frame:
                        time.sleep(0.001)
                        continue
                    self._pre_frame = pcds[0].frame
                    if self.input_remove_data_diagnosis(pcds, can_angle):
                        continue

                    # 実際の処理(remove)
                    output_remove: tuple[RemovePointsData, DeltaYawData] | None = (
                        self._update_remove(
                            pcds,
                            can_angle,
                        )
                    )
                    if output_remove is None or len(output_remove) != 2:
                        continue
                    try:
                        # 実際の処理(accum)
                        removed_pcds: RemovePointsData = output_remove[0]
                        delta_yaw: DeltaYawData = output_remove[1]
                        if self.input_accum_data_diagnosis(
                            removed_pcds,
                            delta_yaw,
                            can_lever,
                        ):
                            continue
                        output_accum: (
                            tuple[AccumPointsData, AccumGroundPointsData] | None
                        ) = self._update_accum(
                            removed_pcds,
                            delta_yaw,
                            can_lever,
                            can_angle,
                        )
                        if output_accum is None or len(output_accum) != 2:
                            continue
                    except Exception as e:
                        is_state_error_d_exception = (
                            self._ser.is_state_error_d_exception(e, self._logger)
                        )
                        if not is_state_error_d_exception:
                            if self._ser.module_errors[
                                ModuleErrorIndex.ACCUMULATION_MODULE_ERROR
                            ].excepts_diagnosis(e):
                                self._ser.module_errors[
                                    ModuleErrorIndex.ACCUMULATION_MODULE_ERROR
                                ].log_output(
                                    ResultDiagnosis.DETECTION,
                                    ResultDiagnosis.DETECTION,
                                    ModuleErrorIndex.ACCUMULATION_MODULE_ERROR,
                                    e,
                                )
                            else:
                                raise e

                    # 実際の処理(collision_cliff)
                    output_accum_points: AccumPointsData = output_accum[0]
                    accum_ground_pcd: AccumGroundPointsData = output_accum[1]
                    if self.input_collision_cliff_data_diagnosis(accum_ground_pcd):
                        continue
                    output_cliff: EdgeDetectionResult | None = (
                        self._update_collision_cliff(
                            accum_ground_pcd,
                        )
                    )
                    self._spe.last_heartbeat.value = time.monotonic()
                    if output_cliff is None:
                        continue

                # 出力処理
                self.accum_producer.produce(output_accum_points)
                self.cliff_pcd_producer.produce(output_cliff)
            except Exception as e:
                is_state_error_d_exception = self._ser.is_state_error_d_exception(
                    e, self._logger
                )
                if not is_state_error_d_exception:
                    if self._ser.module_errors[
                        ModuleErrorIndex.POINTS_REFINE_MODULE_ERROR
                    ].excepts_diagnosis(e):
                        self._ser.module_errors[
                            ModuleErrorIndex.POINTS_REFINE_MODULE_ERROR
                        ].log_output(
                            ResultDiagnosis.DETECTION,
                            ResultDiagnosis.DETECTION,
                            ModuleErrorIndex.POINTS_REFINE_MODULE_ERROR,
                            e,
                        )
                    else:
                        raise e

    @log_target("機体点除去", ProfCategory.Process)
    def _update_remove(
        self,
        pcd_input_data: tuple[PointCloudData, ...],
        can_angle_data: CanAngleData,
    ) -> tuple[RemovePointsData, DeltaYawData] | None:
        self._fps_prof.enter()

        combined_xyz: NDArray[np.float64] = self._pcd_xyz.get_pcd_data(
            pcd_input_data[0].frame,
            self._app_config,
            pcd_input_data,
        )

        initialize_xyz: NDArray[np.float64] = SubScrutinizer.apply_initial_transform(
            combined_xyz,
            self._init_r,
            self._init_t,
            in_place=True,
        )

        rd1, rd2, rd3 = map(
            float,
            self._octotree_obj.cell_interval * self._app_config.OctoTree.remove_dist,
        )
        xyz_machine_points_removed: NDArray[np.float64] = (
            octo_ctrl.remove_machine_points(
                pcd_points=initialize_xyz,
                l_machine_col=self._l_machine_col,
                remove_dist=(rd1, rd2, rd3),
                yaw_angle=np.deg2rad(can_angle_data.yaw_angle_deg),
            )
        )

        remove_points_data: RemovePointsData = RemovePointsData(
            pcd_input_data[0].frame,
            max(image.time for image in pcd_input_data),
            xyz_machine_points_removed,
        )

        delta_yaw: float = float(
            can_angle_data.yaw_angle_deg - self.yaw_pre_angle_deg,
        )
        self.yaw_pre_angle_deg = can_angle_data.yaw_angle_deg

        delta_yaw_data = DeltaYawData(can_angle_data.frame, delta_yaw)
        return remove_points_data, delta_yaw_data

    @log_target("蓄積", ProfCategory.Process)
    def _update_accum(
        self,
        pcd_input_data: RemovePointsData,
        delta_yaw_input: DeltaYawData,
        lever_input_data: CanLeverData,
        can_angle_data: CanAngleData,
    ) -> (
        tuple[
            AccumPointsData,
            AccumGroundPointsData,
        ]
        | None
    ):
        list_lever_pressure = lever_input_data.lever_pressure.tolist()
        crane_state: bool | None = self._crane_state_est.update(
            pressures=list_lever_pressure,
        )

        # 点群数を負荷低減モードの判定に使用するために更新する
        points_num: int = pcd_input_data.point_cloud.shape[0]
        self._ser.reduced_load_mode.update_pcd_nums(points_num)
        is_reduced_load_mode = self._ser.reduced_load_mode.enabled
        accum_frames_before = len(self._accum_points_dq)
        ground_frames_before = len(self._accum_ground_dq)

        accum_points: NDArray[np.float64] | None
        accum_ground_points: NDArray[np.float64] | None
        (
            self._counter,
            accum_points,
            accum_ground_points,
            self._accum_points_dq,
            self._accum_ground_dq,
            self._accum_counter,
        ) = self._accum_point_cloud.accum_point_cloud(
            pcd_input_data.point_cloud,
            self._counter,
            self._accum_points_dq,
            self._accum_ground_dq,
            self._accum_counter,
            delta_yaw_input.delta_yaw,
            self._app_config,
            crane_state,
            is_reduced_load_mode,
        )

        if is_reduced_load_mode != self._last_accum_reduced_load_mode:
            accumulation = self._app_config.Accumulation
            max_accumulated_frames = (
                accumulation.max_accumulated_frames_reduced_load
                if is_reduced_load_mode
                else accumulation.max_accumulated_frames
            )
            max_accumulated_frames_ground = (
                accumulation.max_accumulated_frames_ground_reduced_load
                if is_reduced_load_mode
                else accumulation.max_accumulated_frames_ground
            )
            self._logger.info(
                "蓄積バッファ切替: mode=%s, frame=%s, points=%s->%s (上限=%s), ground=%s->%s (上限=%s)",
                "reduced_load" if is_reduced_load_mode else "normal",
                pcd_input_data.frame,
                accum_frames_before,
                len(self._accum_points_dq),
                max_accumulated_frames,
                ground_frames_before,
                len(self._accum_ground_dq),
                max_accumulated_frames_ground,
            )
            self._last_accum_reduced_load_mode = is_reduced_load_mode

        if accum_points is None or accum_ground_points is None:
            return None

        accum_data = AccumPointsData(
            pcd_input_data.frame,
            pcd_input_data.time,
            accum_points,
            can_angle_data.yaw_angle_deg,
        )
        accum_ground_data = AccumGroundPointsData(
            pcd_input_data.frame,
            pcd_input_data.time,
            accum_ground_points,
        )

        return accum_data, accum_ground_data

    @log_target("崖検出", ProfCategory.Process)
    def _update_collision_cliff(
        self,
        accum_ground_inputs: AccumGroundPointsData,
    ) -> EdgeDetectionResult | None:
        # 地面点群を入れる
        self._display_detect_cliff.put_ground_points(
            ground_points=accum_ground_inputs.point_cloud,
            octree_obj=self._octotree_obj,
            target_entity=NodeEntity.LOW_3D,
        )

        _, edge_result = self._display_detect_cliff.create_obj_cliff(
            self._edge_detector,
            # accum_ground_inputs.point_cloud,
            self._octotree_obj,
            [NodeEntity.LOW_3D],
            self._app_config.EdgeDetection,
            self._app_config.General,
            # np.vstack(accum_points_with_ground_deq),
            # downsampled_accum_points,
        )
        edge_result.frame = accum_ground_inputs.frame
        edge_result.time = accum_ground_inputs.time

        self._fps_prof.prof(
            pcd_frame=accum_ground_inputs.frame,
            pcd_s_time=accum_ground_inputs.time,
            accum_ground_points=accum_ground_inputs,
        )

        return edge_result
