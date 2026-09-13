from __future__ import annotations

import time
from contextlib import ExitStack

from argus_synchro.config.app_config import AppConfig
from argus_synchro.diagnosis.error_config import ErrorConfig
from argus_synchro.diagnosis.error_diagnosis import ResultDiagnosis
from argus_synchro.message.input_message import CameraData, CanData, PointCloudData
from argus_synchro.message.scrutinizer_message import CanAngleData, CanLeverData
from argus_synchro.process import ProcessBase
from argus_synchro.process.message import Consumer, MessageFlow, Producer
from argus_synchro.process.synchronizer import ProcessActivator
from argus_synchro.profiler import log_main, log_target
from argus_synchro.profiler.prof_fps import ProfFps
from argus_synchro.profiler.prof_mode import ProfCategory
from argus_synchro.shared_app_config import SharedAppConfig
from argus_synchro.shared_errors import ModuleErrorIndex, SharedErrors, StateErrorDIndex
from argus_synchro.shared_excepts import INVALID_TIMESTAMP, SharedGetDataExcept


class GetDataProcess(ProcessBase):
    __slots__ = (
        "_app_config",
        "_camera_inputs",
        "_camera_outputs",
        "_can_angle_output",
        "_can_input",
        "_can_lever_output",
        "_end_frame",
        "_err_config",
        "_fps_prof",
        "_last_updated",
        "_pcd_inputs",
        "_pcd_outputs",
        "_ref_t",
        "_sac",
        "_sec_get_data",
        "_ser",
        "_target_frame_time_sec",
    )

    def __init__(
        self,
        sec_get_data: SharedGetDataExcept,
        sac: SharedAppConfig,
        ser: SharedErrors,
        camera_inputs: tuple[MessageFlow[CameraData], ...],
        pcd_inputs: tuple[MessageFlow[PointCloudData], ...],
        can_input: MessageFlow[CanData],
        camera_outputs: tuple[MessageFlow[CameraData], ...],
        pcd_outputs: tuple[MessageFlow[PointCloudData], ...],
        can_angle_outputs: MessageFlow[CanAngleData],
        can_lever_output: MessageFlow[CanLeverData],
        activator: ProcessActivator,
    ) -> None:
        super().__init__(sec_get_data, activator, "GetDataProcess")
        self._sec_get_data = sec_get_data
        self._camera_inputs: tuple[MessageFlow[CameraData], ...] = tuple(
            self._subscribe(camera) for camera in camera_inputs
        )
        self._pcd_inputs: tuple[MessageFlow[PointCloudData], ...] = tuple(
            self._subscribe(pcd) for pcd in pcd_inputs
        )
        self._can_input: MessageFlow[CanData] = self._subscribe(
            can_input,
        )
        self._camera_outputs: tuple[MessageFlow[CameraData], ...] = tuple(
            self._subscribe(camera) for camera in camera_outputs
        )
        self._pcd_outputs: tuple[MessageFlow[PointCloudData], ...] = tuple(
            self._subscribe(pcd_output) for pcd_output in pcd_outputs
        )
        self._can_angle_output: MessageFlow[CanAngleData] = self._subscribe(
            can_angle_outputs
        )
        self._can_lever_output: MessageFlow[CanLeverData] = self._subscribe(
            can_lever_output,
        )
        self._sac: SharedAppConfig = sac
        self._fps_prof: ProfFps = ProfFps(self.__class__.__name__)
        self._ser: SharedErrors = ser

        # _startupで初期化
        self._ref_t: int
        self._err_config: ErrorConfig

    def _config_load(self) -> None:
        self._app_config: AppConfig = self._sac.read()
        self._last_updated: int = self._sac.last_updated
        self._end_frame: int = self._app_config.Scrutinizer.e_frame
        self._target_frame_time_sec: float = (
            self._app_config.Scrutinizer.get_data_sleep_sec
        )
        if self._target_frame_time_sec < 0:
            raise ValueError("Scrutinizer.get_data_sleep_sec must be >= 0")

    def _err_config_load(self) -> None:
        self._err_config = self._ser.shared_err_conf.read()

        # NOTE: このプロセスで実施する全ての診断クラスのupdateをここに追加していく
        self._err_config = self._ser.shared_err_conf.read()
        self._ser.state_errors_D[StateErrorDIndex.INVALID_DATA_INPUT].update(
            self._err_config
        )
        self._ser.state_errors_D[StateErrorDIndex.ARRAY_SHAPE_ERROR].update(
            self._err_config
        )
        self._ser.module_errors[ModuleErrorIndex.GET_DATA_MODULE_ERROR].update(
            self._err_config
        )

    def _log_register(self) -> None:
        super()._log_register()
        self._sac.log_register(self._app_logger_factory)
        self._ser.log_register(self._app_logger_factory)

    def _apply_parameters(self) -> None:
        pass

    def _startup(self) -> None:
        self._config_load()
        self._err_config_load()
        self._ref_t = self._app_config.Scrutinizer.s_frame
        self.start_diagnosis()

        self._fps_prof.start()
        self.create_producer_and_consumer()
        self._err_config_load()

    def create_producer_and_consumer(self) -> None:
        self.camera_consumers: tuple[Consumer[CameraData], ...] = tuple(
            camera.create_consumer() for camera in self._camera_inputs
        )
        self.pcd_consumers: tuple[Consumer[PointCloudData], ...] = tuple(
            pcd.create_consumer() for pcd in self._pcd_inputs
        )
        self.can_consumer: Consumer[CanData] = self._can_input.create_consumer()

        self.camera_producers: tuple[Producer[CameraData], ...] = tuple(
            camera.create_producer() for camera in self._camera_outputs
        )
        self.pcd_producers: tuple[Producer[PointCloudData], ...] = tuple(
            pcd.create_producer() for pcd in self._pcd_outputs
        )

        self.can_angle_producer: Producer[CanAngleData] = (
            self._can_angle_output.create_producer()
        )
        self.can_lever_producer: Producer[CanLeverData] = (
            self._can_lever_output.create_producer()
        )

    def restart_completed(self) -> None:
        for i in self.camera_consumers:
            i.restart_completed()
        for i in self.pcd_consumers:
            i.restart_completed()
        self.can_consumer.restart_completed()
        for i in self.camera_producers:
            i.restart_completed()
        for i in self.pcd_producers:
            i.restart_completed()
        self.can_angle_producer.restart_completed()
        self.can_lever_producer.restart_completed()

    def _start_restart(self) -> None:
        pass

    def _shutdown(self) -> None:
        self.stop_diagnosis()
        self._fps_prof.export()

    def start_diagnosis(self) -> None:
        self._sec_get_data.last_heartbeat.value = INVALID_TIMESTAMP
        self._sec_get_data.is_heartbeat_enabled.value = True

    def stop_diagnosis(self) -> None:
        self._sec_get_data.is_heartbeat_enabled.value = False

    def _wait_for_next_frame(self, frame_begin: float) -> None:
        elapsed_sec = time.perf_counter() - frame_begin
        wait_sec = self._target_frame_time_sec - elapsed_sec
        if wait_sec > 0:
            time.sleep(wait_sec)

    def _advance_frame(self) -> None:
        self._ref_t += 1
        if self._ref_t <= self._end_frame:
            return
        if (
            self._app_config.DEFAULT.File_Input
            and self._app_config.Scrutinizer.file_input_loop
        ):
            self._ref_t = self._app_config.Scrutinizer.s_frame
            return
        self._unsubscribe()

    def input_data_diagnosis(
        self,
        pcds: tuple[PointCloudData, ...],
        candata: CanData,
        cameras: tuple[CameraData, ...],
    ) -> bool:
        invalid_data_input = self._ser.state_errors_D[
            StateErrorDIndex.INVALID_DATA_INPUT
        ]
        input_data = (pcds, candata, cameras)
        result, failsafe_result = invalid_data_input.errors_diagnosis(input_data)
        invalid_data_input.log_output(
            result, failsafe_result, StateErrorDIndex.INVALID_DATA_INPUT
        )
        if result == ResultDiagnosis.DETECTION:
            return True

        pcds_point_cloud = tuple(pcd.point_cloud for pcd in pcds)
        lever_pressure = candata.lever_pressure
        images = tuple(camera.image for camera in cameras)

        array_shape_error = self._ser.state_errors_D[StateErrorDIndex.ARRAY_SHAPE_ERROR]
        result, failsafe_result = array_shape_error.errors_diagnosis(
            ("pcds_point_cloud", pcds_point_cloud),
            ("lever_pressure", lever_pressure),
            ("images", images),
        )
        array_shape_error.log_output(
            result, failsafe_result, StateErrorDIndex.ARRAY_SHAPE_ERROR, self.name
        )
        return result == ResultDiagnosis.DETECTION

    @log_main()
    def _loop(self) -> None:
        stack = ExitStack()
        try:
            while self.enable:
                frame_begin = time.perf_counter()

                if self._ser.is_idle_mode:
                    self._logger.warning("Idle mode: アイドルモード中...")
                    time.sleep(1)
                    continue

                if self._sac.last_updated > self._last_updated:
                    self._config_load()
                    self._apply_parameters()

                try:
                    if (
                        any(not i.wait() for i in self.camera_producers)
                        or any(not i.wait() for i in self.pcd_producers)
                        or not self.can_angle_producer.wait()
                        or not self.can_lever_producer.wait()
                    ):
                        continue

                    stack.close()

                    if (
                        any(not c.wait() for c in self.camera_consumers)
                        or any(not i.wait() for i in self.pcd_consumers)
                        or not self.can_consumer.wait()
                    ):
                        continue

                    # 入力処理
                    pcds: tuple[PointCloudData, ...] = tuple(
                        stack.enter_context(c.consume()) for c in self.pcd_consumers
                    )
                    candata: CanData = stack.enter_context(self.can_consumer.consume())
                    cameras: tuple[CameraData, ...] = tuple(
                        stack.enter_context(c.consume()) for c in self.camera_consumers
                    )

                    if self.input_data_diagnosis(
                        pcds,
                        candata,
                        cameras,
                    ):
                        continue

                    # 実際の処理
                    output_data = self._update(
                        pcds,
                        candata,
                        cameras,
                    )
                    self._sec_get_data.last_heartbeat.value = time.monotonic()

                    # 出力処理
                    if output_data is None:
                        self._wait_for_next_frame(frame_begin)
                        continue

                    for pcd_producer, pcd in zip(
                        self.pcd_producers, output_data[0], strict=False
                    ):
                        pcd_producer.produce(pcd)

                    self.can_angle_producer.produce(output_data[1])
                    self.can_lever_producer.produce(output_data[2])

                    for camera_producer, camera in zip(
                        self.camera_producers,
                        output_data[3],
                        strict=False,
                    ):
                        camera_producer.produce(camera)

                    self._wait_for_next_frame(frame_begin)
                except Exception as e:
                    is_state_error_d_exception = self._ser.is_state_error_d_exception(
                        e, self._logger
                    )
                    if not is_state_error_d_exception:
                        if self._ser.module_errors[
                            ModuleErrorIndex.GET_DATA_MODULE_ERROR
                        ].excepts_diagnosis(e):
                            self._ser.module_errors[
                                ModuleErrorIndex.GET_DATA_MODULE_ERROR
                            ].log_output(
                                ResultDiagnosis.DETECTION,
                                ResultDiagnosis.DETECTION,
                                ModuleErrorIndex.GET_DATA_MODULE_ERROR,
                                e,
                            )
                        else:
                            raise e
        finally:
            stack.close()

    @log_target("データ取得", ProfCategory.Process)
    def _update(
        self,
        pcd_input_data: tuple[PointCloudData, ...],
        can_input_data: CanData,
        camera_input_data: tuple[CameraData, ...],
    ) -> (
        tuple[
            tuple[PointCloudData, ...],
            CanAngleData,
            CanLeverData,
            tuple[CameraData, ...],
        ]
        | None
    ):
        self._fps_prof.enter()

        pcd_out_data: tuple[PointCloudData, ...] = tuple(
            PointCloudData(self._ref_t, pcd.time, pcd.point_cloud)
            for pcd in pcd_input_data
        )

        angle_out_data = CanAngleData(self._ref_t, can_input_data.yaw_angle_deg)

        lever_out_data = CanLeverData(self._ref_t, can_input_data.lever_pressure)

        camera_out_data: tuple[CameraData, ...] = tuple(
            CameraData(
                image.index,
                self._ref_t,
                image.time,
                image.image,
            )
            for image in camera_input_data
        )

        self._fps_prof.prof(
            pcd_frame=self._ref_t,
            camera_frame=self._ref_t,
            pcd_s_time=max(pcd.time for pcd in pcd_input_data),
            camera_s_time=max(image.time for image in camera_input_data),
        )

        self._advance_frame()

        return (
            pcd_out_data,
            angle_out_data,
            lever_out_data,
            camera_out_data,
        )
