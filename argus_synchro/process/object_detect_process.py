from __future__ import annotations

import time
from contextlib import ExitStack
from typing import TYPE_CHECKING

import numpy as np
import psutil
from numpy.typing import NDArray

from argus_synchro.config.app_config import AppConfig
from argus_synchro.core.utils import FastResize
from argus_synchro.diagnosis.error_config import ErrorConfig
from argus_synchro.diagnosis.error_diagnosis import ResultDiagnosis
from argus_synchro.message.input_message import CameraData
from argus_synchro.message.scrutinizer_message import CameraDetectionsData
from argus_synchro.process.message import Consumer, MessageFlow, Producer
from argus_synchro.process.process import ProcessBase
from argus_synchro.process.synchronizer import ProcessActivator
from argus_synchro.profiler import log_main, log_target
from argus_synchro.profiler.prof_fps import ProfFps
from argus_synchro.profiler.prof_mode import ProfCategory
from argus_synchro.shared_app_config import SharedAppConfig
from argus_synchro.shared_errors import (
    ActionErrorIndex,
    ModuleErrorIndex,
    SharedErrors,
    StateErrorDIndex,
)
from argus_synchro.shared_excepts import (
    INVALID_TIMESTAMP,
    SharedExcepts,
    SharedScrutinizerExcept,
)

if TYPE_CHECKING:
    from argus_synchro.detect2d import ObjDetectionBase, ObjDetectionInterface
    from argus_synchro.provider.image import UndistortImageProvider


class ObjectDetectProcess(ProcessBase):
    __slots__ = (
        "_app_config",
        "_applied_detect2d",
        "_bouding_box_data",
        "_calib_conf",
        "_camera_conf",
        "_camera_inputs",
        "_detect2d",
        "_err_config",
        "_fps_prof",
        "_frames_buf",
        "_index",
        "_last_updated",
        "_pre_frame",
        "_provider",
        "_resize",
        "_sac",
        "_ser",
        "_undistort_backend_mode",
        "sec",
    )

    def __init__(
        self,
        index: int,
        sac: SharedAppConfig,
        sec: SharedExcepts,
        ser: SharedErrors,
        sec_scruti_ex: SharedScrutinizerExcept,
        camera_inputs: tuple[MessageFlow[CameraData], ...],
        bouding_box_data: MessageFlow[CameraDetectionsData],
        activator: ProcessActivator,
    ) -> None:
        super().__init__(sec_scruti_ex, activator, "ObjectDetectProcess")
        self._camera_inputs: tuple[MessageFlow[CameraData], ...] = tuple(
            self._subscribe(camera) for camera in camera_inputs
        )
        self._bouding_box_data: MessageFlow[CameraDetectionsData] = self._subscribe(
            bouding_box_data
        )
        self._index: int = index
        self._pre_frame: int = 0
        self.sec: SharedExcepts = sec
        self._sac: SharedAppConfig = sac
        self._ser: SharedErrors = ser
        self._fps_prof: ProfFps = ProfFps(self.__class__.__name__)
        self._provider: UndistortImageProvider
        self._undistort_backend_mode: str = "auto"

        # _startupで初期化
        self._applied_detect2d: ObjDetectionBase | None
        self._detect2d: ObjDetectionInterface
        self._frames_buf: NDArray[np.uint8]
        self._err_config: ErrorConfig

    def _config_load(self) -> None:
        self._app_config: AppConfig = self._sac.read()
        self._last_updated: int = self._sac.last_updated

    def _log_register(self) -> None:
        super()._log_register()
        self._sac.log_register(self._app_logger_factory)
        self.sec.log_register(self._app_logger_factory)
        self._ser.log_register(self._app_logger_factory)

    def _err_config_load(self) -> None:
        self._err_config = self._ser.shared_err_conf.read()

        # NOTE: このプロセスで実施する全ての診断クラスのupdateをここに追加していく
        self._ser.state_errors_D[StateErrorDIndex.INVALID_DATA_INPUT].update(
            self._err_config
        )
        self._ser.state_errors_D[StateErrorDIndex.ARRAY_SHAPE_ERROR].update(
            self._err_config
        )
        self._ser.state_errors_D[
            StateErrorDIndex.AI_INFERENCE_RESULT_ERROR
        ].update(self._err_config)
        self._ser.action_errors_A_C[ActionErrorIndex.AI_MODEL_LOAD_FAILED].update(
            self._err_config
        )
        self._ser.module_errors[
            ModuleErrorIndex.CAMERA_HUMAN_DETECTION_MODULE_ERROR
        ].update(self._err_config)

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

    def input_camera_data_diagnosis(
        self,
        cameras: tuple[CameraData, ...],
    ) -> bool:
        invalid_data_input = self._ser.state_errors_D[
            StateErrorDIndex.INVALID_DATA_INPUT
        ]
        result, failsafe_result = invalid_data_input.errors_diagnosis(cameras)
        invalid_data_input.log_output(
            result, failsafe_result, StateErrorDIndex.INVALID_DATA_INPUT, self.name
        )
        if result == ResultDiagnosis.DETECTION:
            return True

        images: tuple[NDArray[np.uint8], ...] = tuple(
            camera.image for camera in cameras
        )
        return self._input_data_diagnosis(("images", images))

    def input_detect2d_data_diagnosis(self) -> bool:
        invalid_data_input = self._ser.state_errors_D[
            StateErrorDIndex.INVALID_DATA_INPUT
        ]
        result, failsafe_result = invalid_data_input.errors_diagnosis(self._frames_buf)
        invalid_data_input.log_output(
            result, failsafe_result, StateErrorDIndex.INVALID_DATA_INPUT, self.name
        )
        if result == ResultDiagnosis.DETECTION:
            return True

        return self._input_data_diagnosis(("frames_buf", self._frames_buf))

    def _apply_parameters(self) -> None:
        if self._app_config.detect2d.is_applied:
            try:
                if self._applied_detect2d is None:
                    from argus_synchro.detect2d import Detect2dDamoYoloOnnx

                    process_instance = psutil.Process(self.pid)
                    affinity_cores: list[int] = process_instance.cpu_affinity()
                    self._logger.info("damo yolo is selected")
                    self._applied_detect2d = Detect2dDamoYoloOnnx(
                        conf_thresh=self._app_config.detect2d.conf_thresh,
                        nms_thresh=self._app_config.detect2d.nms_thresh,
                        onnx_model_path=self._app_config.detect2d.onnx_model_path,
                        batch_size=self._app_config.camera.count,
                        app_logger_factory=self._app_logger_factory,
                        affinity_cores=affinity_cores,
                    )
                self._detect2d = self._applied_detect2d
                self._applied_detect2d.update(self._app_config)
            except Exception as e:
                diagnosis = self._ser.action_errors_A_C[
                    ActionErrorIndex.AI_MODEL_LOAD_FAILED
                ]
                is_ce013 = diagnosis.excepts_diagnosis(e)

                from argus_synchro.detect2d import NotAppliedObjDetection

                self._detect2d = NotAppliedObjDetection()
                self._applied_detect2d = None
                if is_ce013:
                    diagnosis.log_output(
                        True, False, ActionErrorIndex.AI_MODEL_LOAD_FAILED, e
                    )
                else:
                    self._logger.warning(
                        f"AI model initialization warning (ignored): {e!r}"
                    )
        else:
            from argus_synchro.detect2d import NotAppliedObjDetection

            self._detect2d = NotAppliedObjDetection()

        # キャリブレーション設定変更時に歪み補正プロバイダーを再構築
        self._build_provider()

    def _build_provider(self) -> None:
        """sac から読み込んだ AppConfig を使って歪み補正プロバイダーを構築する。"""
        from argus_synchro.provider.image import create_mcde7000_undistort_provider

        self._camera_conf = self._app_config.camera
        self._calib_conf = self._app_config.calibration
        self._undistort_backend_mode = self._resolve_undistort_backend_mode()
        self._provider = create_mcde7000_undistort_provider(
            camera_intrinsics_path=self._calib_conf.fisheye_param_file,
            sys_width=self._camera_conf.sys_width,
            sys_height=self._camera_conf.sys_height,
            backend=self._undistort_backend_mode,
        )
        self._logger.info(
            "undistort backend mode=%s actual=%s",
            self._undistort_backend_mode,
            self._provider.backend_name,
        )

    def _resolve_undistort_backend_mode(self) -> str:
        mode = self._camera_conf.undistort_backend.strip().lower()
        if mode in {"auto", "cpu", "cuda"}:
            return mode
        self._logger.warning(
            "invalid camera.undistort_backend=%s. fallback to auto",
            mode,
        )
        return "auto"

    def _startup(self) -> None:
        self._config_load()
        self._err_config_load()
        self._build_provider()
        self.start_diagnosis()
        self._frames_buf = np.zeros(
            (
                self._camera_conf.count,
                self._camera_conf.sys_height,
                self._camera_conf.sys_width,
                3,
            ),
            np.uint8,
        )
        self._applied_detect2d = None
        self._apply_parameters()
        self._pre_frame = 0
        self._fps_prof.start()
        self.create_producer_and_consumer()
        self._resize = FastResize(
            width=self._app_config.camera.sys_width,
            height=self._app_config.camera.sys_height,
        )

    def _start_restart(self) -> None:
        self._config_load()
        self._build_provider()

    def create_producer_and_consumer(self) -> None:
        self.camera_consumers: tuple[Consumer[CameraData], ...] = tuple(
            camera.create_consumer() for camera in self._camera_inputs
        )

        self.bb_box_producers: Producer[CameraDetectionsData] = (
            self._bouding_box_data.create_producer()
        )

    def restart_completed(self) -> None:
        for i in self.camera_consumers:
            i.restart_completed()

        self.bb_box_producers.restart_completed()

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
                if not self.bb_box_producers.wait():
                    continue
                if any(not c.wait() for c in self.camera_consumers):
                    continue

                # 入力処理
                with ExitStack() as stack:
                    cameras: tuple[CameraData, ...] = tuple(
                        stack.enter_context(c.consume()) for c in self.camera_consumers
                    )

                    if cameras[0].frame == self._pre_frame:
                        time.sleep(0.001)
                        continue
                    self._pre_frame = cameras[0].frame
                    if self.input_camera_data_diagnosis(cameras):
                        continue

                    output_data: CameraDetectionsData | None = self._update(cameras)
                    self._spe.last_heartbeat.value = time.monotonic()

                    if output_data is None:
                        continue

                self.bb_box_producers.produce(output_data)
            except Exception as e:
                is_state_error_d_exception = self._ser.is_state_error_d_exception(
                    e, self._logger
                )
                if not is_state_error_d_exception:
                    if self._ser.module_errors[
                        ModuleErrorIndex.CAMERA_HUMAN_DETECTION_MODULE_ERROR
                    ].excepts_diagnosis(e):
                        self._ser.module_errors[
                            ModuleErrorIndex.CAMERA_HUMAN_DETECTION_MODULE_ERROR
                        ].log_output(
                            ResultDiagnosis.DETECTION,
                            ResultDiagnosis.DETECTION,
                            ModuleErrorIndex.CAMERA_HUMAN_DETECTION_MODULE_ERROR,
                            e,
                        )
                    else:
                        raise e

    def _ai_inference_result_content_diagnosis(
        self, camera_detect_data: CameraDetectionsData
    ) -> None:
        ai_inference_result_error = self._ser.state_errors_D[
            StateErrorDIndex.AI_INFERENCE_RESULT_ERROR
        ]
        result = ai_inference_result_error.errors_diagnosis(
            camera_detect_data.boxes,
            camera_detect_data.scores,
            camera_detect_data.valid_detects,
        )
        ai_inference_result_error.log_output(
            *result,
            StateErrorDIndex.AI_INFERENCE_RESULT_ERROR,
            self._index,
        )

    @log_target("画像処理", ProfCategory.Process)
    def _update(
        self,
        camera_input_data: tuple[CameraData, ...],
    ) -> CameraDetectionsData | None:
        self._fps_prof.enter()
        for i, camera_data in enumerate(camera_input_data):
            resized_image: NDArray[np.uint8] = self._resize.apply(camera_data.image)
            # 歪み補正
            self._provider.get_undistort_image(
                resized_image,
                dst=self._frames_buf[i],
            )

        if self.input_detect2d_data_diagnosis():
            return None
        try:
            camera_detect_data: CameraDetectionsData = self._detect2d.object_detect(
                self.sec, self._frames_buf
            )
        except Exception as error:
            from argus_synchro.detect2d import NotAppliedObjDetection

            if not self._ser.is_state_error_d_exception(error, self._logger):
                raise
            camera_detect_data = NotAppliedObjDetection().object_detect(
                self.sec, self._frames_buf
            )
        else:
            self._ai_inference_result_content_diagnosis(camera_detect_data)

        camera_detect_data.index = camera_input_data[0].index
        camera_detect_data.frame = camera_input_data[0].frame
        camera_detect_data.time = max(image.time for image in camera_input_data)

        self._fps_prof.prof(
            camera_frame=camera_detect_data.frame,
            camera_s_time=camera_detect_data.time,
        )

        return camera_detect_data
