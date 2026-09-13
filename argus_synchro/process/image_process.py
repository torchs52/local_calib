"""
プロセスの概要

* カメラ画像取得プロセス
    * プロセス1: カメラ1から画像を取得する (ファイル入力モード)
    * プロセス2: カメラ2から画像を取得する (ファイル入力モード)
    * プロセス3: カメラ3から画像を取得する (ファイル入力モード)
"""

from __future__ import annotations

import datetime
import time
from typing import TYPE_CHECKING, final

import numpy as np
from numpy.typing import NDArray

from argus_synchro.config.app_config import AppConfig, ScrutinizerConf
from argus_synchro.config.app_config_calibration import (
    AppConfigCalibration,
    DataCaptureConf,
    DefaultConf,
)
from argus_synchro.config.fileinput_pathselector import (
    video_filepath_loader,
)
from argus_synchro.diagnosis.error_config import ErrorConfig
from argus_synchro.diagnosis.error_diagnosis import ResultDiagnosis
from argus_synchro.message.input_message import CameraData
from argus_synchro.process import InputProcess, MessageFlow
from argus_synchro.process.message import Producer
from argus_synchro.process.operation_mode import OPERATION_MODE as OPM
from argus_synchro.process.synchronizer import ProcessActivator
from argus_synchro.profiler import log_main, log_target
from argus_synchro.profiler.prof_mode import ProfCategory
from argus_synchro.provider.clock import (
    DummyClockProvider,
    PerfCounterClockProvider,
    TimeClockProvider,
)
from argus_synchro.shared_app_config import SharedAppConfig, SharedAppConfigCalibration
from argus_synchro.shared_errors import (
    ModuleErrorIndex,
    SharedErrors,
    StateErrorDIndex,
    StateErrorIndex,
)
from argus_synchro.shared_excepts import INVALID_TIMESTAMP, SharedCAMExcept

if TYPE_CHECKING:
    from argus_synchro.provider.image import (
        CalibMcde7000FileImageProvider,
        ImageProvider,
    )
from argus_synchro.provider.image import (
    CalibMcde7000FileImageProvider,
    Mcde7000FileImageProvider,
)


@final
class CameraProviderProcess(InputProcess[CameraData]):
    """カメラ画像取得プロセス"""

    __slots__ = (
        "_app_config",
        "_clock_provider",
        "_end_frame",
        "_err_config",
        "_file_input",
        "_frame",
        "_index",
        "_last_updated",
        "_print_disabled",
        "_provider",
        "_sac",
        "_sac_calib",
        "_scrutinizer_conf",
        "_ser",
    )

    def __init__(
        self,
        index: int,
        sec_cam: SharedCAMExcept,
        sac_calib: SharedAppConfigCalibration,
        sac: SharedAppConfig,
        ser: SharedErrors,
        producer_flow: MessageFlow[CameraData],
        activator: ProcessActivator,
        name: str | None = None,
    ) -> None:
        super().__init__(producer_flow, sec_cam, activator, name)
        self._index: int = index
        self._sac_calib: SharedAppConfigCalibration = sac_calib
        self._sec_cam: SharedCAMExcept = sec_cam

        self._sac: SharedAppConfig = sac
        self._provider: ImageProvider
        self._ser: SharedErrors = ser
        self._consecutive_read_failure_count: int = 0
        self._timestamp: float | None = None

        # _startupで初期化
        self._err_config: ErrorConfig

    def _config_load(self) -> None:
        self._app_config: AppConfig = self._sac.read()
        self._app_config_calib = self._sac_calib.read()
        self._app_config_calib: AppConfigCalibration = self._app_config_calib
        self.DefaultConfig: DefaultConf = self._app_config_calib.default
        self.CameraConfig: DataCaptureConf.CameraConf = (
            self._app_config_calib.dataCapture.Camera
        )
        self._last_updated: int = self._sac.last_updated
        self._scrutinizer_conf: ScrutinizerConf = self._app_config.Scrutinizer
        self._print_disabled: bool = self._app_config_calib.default.print_disabled

        calib_mode: bool = bool(self._app_config.General.operation_mode == OPM.CALIB)
        if calib_mode:
            self._end_frame = self._app_config_calib.dataCapture.e_frame
            self._file_input: bool = self._app_config_calib.default.File_Input
        else:
            self._end_frame: int = self._app_config.Scrutinizer.e_frame
            self._file_input = self._app_config.DEFAULT.File_Input

    def _err_config_load(self) -> None:
        self._err_config = self._ser.shared_err_conf.read()

        # NOTE: このプロセスで実施する全ての診断クラスのupdateをここに追加していく
        self._ser.state_errors_A_C[
            StateErrorIndex.CAMERA0_COMM_QUALITY_DEGRADED + self._index
        ].update(self._err_config)
        self._ser.state_errors_A_C[
            StateErrorIndex.CAMERA0_COMM_QUALITY_ERROR + self._index
        ].update(self._err_config)
        self._ser.state_errors_A_C[
            StateErrorIndex.CAMERA0_INVALID_DATA + self._index
        ].update(self._err_config)
        self._ser.state_errors_D[StateErrorDIndex.CAMERA_DATA_MISSING].update(
            self._err_config
        )
        self._ser.state_errors_D[StateErrorDIndex.FILE_IO_ERROR].update(
            self._err_config
        )
        self._ser.module_errors[ModuleErrorIndex.CAMERA_MODULE_ERROR].update(
            self._err_config
        )

    def _change_file_name_index(self) -> None:
        """
        校正モードかつFileInputのときにFileを変更する。
        """
        calib_mode: bool = bool(self._app_config.General.operation_mode == OPM.CALIB)
        if calib_mode and self._app_config_calib.default.File_Input:
            camera_file_path: str = video_filepath_loader(
                sac=self._sac, cameraConf=self.CameraConfig
            )[self._index]
            assert isinstance(self._provider, CalibMcde7000FileImageProvider)
            self._provider.change_file_name_index(camera_file_path, self._frame)

    def _log_register(self) -> None:
        super()._log_register()
        self._sac.log_register(self._app_logger_factory)
        self._sac_calib.log_register(self._app_logger_factory)
        self._ser.log_register(self._app_logger_factory)

    def _apply_parameters(self) -> None:
        pass

    def _startup(self) -> None:
        self._config_load()
        self._err_config_load()
        calib_mode: bool = bool(self._app_config.General.operation_mode == OPM.CALIB)
        if calib_mode:
            # 校正モードと周辺監視モードでは開始のインデックスが異なるため
            self._frame = self._app_config_calib.dataCapture.s_frame - 1
        else:
            self._frame: int = self._app_config.Scrutinizer.s_frame
        self._change_device()
        self._change_clock_info()
        self.create_producer_and_consumer()

    def _start_restart(self) -> None:
        # TODO """必要に応じて実際にプロセスを落とさないで再起動で実行する処理を記載""" (NSW)
        self._config_load()
        calib_mode: bool = bool(self._app_config.General.operation_mode == OPM.CALIB)
        if calib_mode:
            self._frame = self._app_config_calib.dataCapture.s_frame - 1
        else:
            self._frame: int = self._app_config.Scrutinizer.s_frame
        self._change_file_name_index()
        self._clock_provider.reset_time()
        self.producer.require_restart()
        del self.producer

    def _restart_surround_file_input_if_needed(self) -> None:
        calib_mode = bool(self._app_config.General.operation_mode == OPM.CALIB)
        if (
            calib_mode
            or not self._file_input
            or not self._scrutinizer_conf.file_input_loop
            or self._frame <= self._end_frame
        ):
            return

        self._frame = self._scrutinizer_conf.s_frame
        camera_file_path = (
            self._scrutinizer_conf.v0_file,
            self._scrutinizer_conf.v1_file,
            self._scrutinizer_conf.v2_file,
        )[self._index]
        assert isinstance(self._provider, Mcde7000FileImageProvider)
        self._provider.change_file_name_index(camera_file_path, self._frame)

    def _change_clock_info(self) -> None:
        calib_mode: bool = bool(self._app_config.General.operation_mode == OPM.CALIB)
        if calib_mode:
            if self._file_input:
                basetime: datetime.datetime = datetime.datetime.strptime(
                    self.CameraConfig.videofile_basetime, "%Y-%m-%d_%H-%M-%S"
                )
                steptime: float = self.CameraConfig.videofile_steptime
                self._clock_provider = DummyClockProvider(
                    basetime, steptime, self._frame, self._end_frame
                )

            else:
                self._clock_provider = TimeClockProvider()
        else:
            self._clock_provider = PerfCounterClockProvider()

    def _change_device(self) -> None:
        use_shi_lib: bool = self._app_config.DEFAULT.use_shi_lib
        calib_mode: bool = bool(self._app_config.General.operation_mode == OPM.CALIB)
        if self._file_input:
            from argus_synchro.device.camera.mcde7000 import Mcde7000File

            # ファイル入力
            if calib_mode:
                camera_file_path: str = video_filepath_loader(
                    sac=self._sac, cameraConf=self.CameraConfig
                )[self._index]
            else:
                camera_file_path: str = [
                    self._scrutinizer_conf.v0_file,
                    self._scrutinizer_conf.v1_file,
                    self._scrutinizer_conf.v2_file,
                ][self._index]
            try:
                device = Mcde7000File(
                    self._index,
                    camera_file_path,
                    self._frame,
                    self._app_logger_factory,
                )
                # _startupはプロセス起動時に実行されるため、デバイスの初期化はここで行う
                device.init_capture()
            except (OSError, RuntimeError, ValueError, TypeError) as error:
                file_io_error = self._ser.state_errors_D[
                    StateErrorDIndex.FILE_IO_ERROR
                ]
                result = file_io_error.errors_diagnosis(True)
                file_io_error.log_output(
                    *result,
                    StateErrorDIndex.FILE_IO_ERROR,
                    camera_file_path,
                    "read file-input camera video",
                    f"{type(error).__name__}: {error}",
                )
                raise
            self._ser.state_errors_D[
                StateErrorDIndex.FILE_IO_ERROR
            ].errors_diagnosis(False)

            if calib_mode:
                from argus_synchro.provider.image import CalibMcde7000FileImageProvider

                self._provider = CalibMcde7000FileImageProvider(
                    self._index,
                    device,
                    self.CameraConfig,
                    self._app_config.camera.sys_width,
                    self._app_config.camera.sys_height,
                    self._app_logger_factory,
                )

            else:
                from argus_synchro.provider.image import Mcde7000FileImageProvider

                self._provider = Mcde7000FileImageProvider(
                    device,
                    self._app_config.camera.sys_width,
                    self._app_config.camera.sys_height,
                )
        elif use_shi_lib:
            from argus_synchro.device.camera.shi_lib_camera import CameraDevReader
            from argus_synchro.provider.image import ShiLibCameraDeviceImageProvider

            # SHIライブラリ使用
            device = CameraDevReader(
                self._index,
                self._app_config.camera.config_file,
                self._app_logger_factory,
            )
            device.init()
            self._provider = ShiLibCameraDeviceImageProvider(device)
        else:
            from argus_synchro.device.camera.mcde7000 import Mcde7000Device
            from argus_synchro.provider.image import (
                CalibMcde7000DeviceImageProvider,
                Mcde7000DeviceImageProvider,
            )

            # MCDE7000(実機)
            device = Mcde7000Device(
                self._index,
                self._app_config.camera,
                self._app_logger_factory,
            )
            device.init_capture()
            if calib_mode:
                self._provider: ImageProvider = CalibMcde7000DeviceImageProvider(
                    self._index,
                    device,
                    self.CameraConfig,
                    self._app_config.camera.sys_width,
                    self._app_config.camera.sys_height,
                    self._app_logger_factory,
                )
            else:
                self._provider: ImageProvider = Mcde7000DeviceImageProvider(
                    self._index,
                    device,
                    self._app_config.camera.sys_width,
                    self._app_config.camera.sys_height,
                    self._app_logger_factory,
                )

    def _shutdown(self) -> None:
        pass

    def create_producer_and_consumer(self) -> None:
        self.producer: Producer[CameraData] = self._producer_flow.create_producer()

    def restart_completed(self) -> None:
        self.producer.restart_completed()

    def _camera_healthy_check(self, image: NDArray[np.uint8] | None) -> None:
        now: float = time.perf_counter()
        comm_quality_degraded = self._ser.state_errors_A_C[
            StateErrorIndex.CAMERA0_COMM_QUALITY_DEGRADED + self._index
        ]
        result: tuple[ResultDiagnosis, ResultDiagnosis] = (
            comm_quality_degraded.errors_diagnosis(
                now,
                self._timestamp,
                self._consecutive_read_failure_count,
            )
        )
        comm_quality_degraded.log_output(
            *result,
            StateErrorIndex.CAMERA0_COMM_QUALITY_DEGRADED + self._index,
            self._index,
        )

        camera_comm_quality_error = self._ser.state_errors_A_C[
            StateErrorIndex.CAMERA0_COMM_QUALITY_ERROR + self._index
        ]
        result = camera_comm_quality_error.errors_diagnosis(
            now,
            self._timestamp,
            self._consecutive_read_failure_count,
        )
        camera_comm_quality_error.log_output(
            *result,
            StateErrorIndex.CAMERA0_COMM_QUALITY_ERROR + self._index,
            self._index,
        )

        if image is not None:
            camera_invalid_data = self._ser.state_errors_A_C[
                StateErrorIndex.CAMERA0_INVALID_DATA + self._index
            ]
            result = camera_invalid_data.errors_diagnosis(image)
            camera_invalid_data.log_output(
                *result,
                StateErrorIndex.CAMERA0_INVALID_DATA + self._index,
                self._index,
            )

            result = self._ser.state_errors_D[
                StateErrorDIndex.CAMERA_DATA_MISSING
            ].errors_diagnosis(self._timestamp, image)
            self._ser.state_errors_D[StateErrorDIndex.CAMERA_DATA_MISSING].log_output(
                *result, StateErrorDIndex.CAMERA_DATA_MISSING, self._index
            )

    @log_target("カメラ入力I/F", ProfCategory.Process)
    def _update(self) -> CameraData | None:
        camera_data: CameraData | None
        image: NDArray[np.uint8] | None = self._provider.get_image()
        self._ser.set_camera_connected(self._index, image is not None)

        now: float = time.perf_counter()
        if image is None:
            self._consecutive_read_failure_count += 1
            camera_data = None
        else:
            self._timestamp = now
            self._sec_cam.last_received.value = self._timestamp
            self._consecutive_read_failure_count = 0

            t: float = self._clock_provider.get_time()
            if self._clock_provider.isframeexceeded():
                self._unsubscribe()
            camera_data = CameraData(self._index, self._frame, t, image)
        self._sec_cam.last_heartbeat.value = now

        self._camera_healthy_check(image)
        return camera_data

    def start_diagnosis(self) -> None:
        self._sec_cam.last_heartbeat.value = INVALID_TIMESTAMP
        self._sec_cam.is_heartbeat_enabled.value = True
        self._sec_cam.last_received.value = INVALID_TIMESTAMP
        self._sec_cam.is_received_enabled.value = True

    def stop_diagnosis(self) -> None:
        self._sec_cam.is_heartbeat_enabled.value = False
        self._sec_cam.is_received_enabled.value = False

    @log_main()
    def _loop(self) -> None:
        self.start_diagnosis()
        while self.enable:
            if self._sac.last_updated > self._last_updated:
                self._config_load()
                self._apply_parameters()
            try:
                if not self.producer.wait():
                    continue
                self._restart_surround_file_input_if_needed()
                output_data = self._update()
                if output_data is None:
                    is_unsubscribe, image = self._provider.handle_no_input()
                    if is_unsubscribe:
                        self._unsubscribe()
                    if image is not None:
                        output_data = CameraData(
                            self._index, self._frame, time.perf_counter(), image
                        )

                if not self._print_disabled:
                    self._logger.info(
                        f"camera {self._index} read, ix {self._frame}, time:{datetime.datetime.fromtimestamp(time.time())}",
                    )
                self._frame += 1
                self.producer.produce(output_data)
            except Exception as e:
                is_state_error_d_exception = self._ser.is_state_error_d_exception(
                    e, self._logger
                )
                if not is_state_error_d_exception:
                    if self._ser.module_errors[
                        ModuleErrorIndex.CAMERA_MODULE_ERROR
                    ].excepts_diagnosis(e):
                        self._ser.module_errors[
                            ModuleErrorIndex.CAMERA_MODULE_ERROR
                        ].log_output(
                            ResultDiagnosis.DETECTION,
                            ResultDiagnosis.DETECTION,
                            ModuleErrorIndex.CAMERA_MODULE_ERROR,
                            e,
                            self._index,
                        )
                    else:
                        raise e
