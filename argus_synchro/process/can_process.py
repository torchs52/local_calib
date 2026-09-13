from __future__ import annotations

import datetime
import time
from dataclasses import replace
from typing import TYPE_CHECKING, final

import numpy as np

from argus_synchro.config.app_config import AppConfig
from argus_synchro.config.app_config_calibration import (
    AppConfigCalibration,
    DefaultConf,
)
from argus_synchro.diagnosis.error_config import ErrorConfig
from argus_synchro.diagnosis.error_diagnosis import ResultDiagnosis
from argus_synchro.message.input_message import CanData
from argus_synchro.process.message import MessageFlow, Producer
from argus_synchro.process.operation_mode import OPERATION_MODE as OPM
from argus_synchro.process.process import InputProcess
from argus_synchro.process.synchronizer import ProcessActivator
from argus_synchro.profiler import log_main, log_target
from argus_synchro.profiler.prof_mode import ProfCategory
from argus_synchro.provider.can_data import CanFileProvider, FixedYawCanDataProvider
from argus_synchro.provider.clock import (
    DummyClockProvider,
    PerfCounterClockProvider,
    TimeClockProvider,
)
from argus_synchro.shared_app_config import SharedAppConfig, SharedAppConfigCalibration
from argus_synchro.shared_errors import (
    ActionErrorIndex,
    ModuleErrorIndex,
    SharedErrors,
    StateErrorDIndex,
)
from argus_synchro.shared_excepts import INVALID_TIMESTAMP, SharedCANExcept

if TYPE_CHECKING:
    from argus_synchro.provider.can_data import CanDataProvider


@final
class CanDataProviderProcess(InputProcess[CanData]):
    __slots__ = (
        "_app_config",
        "_end_frame",
        "_err_config",
        "_index",
        "_last_updated",
        "_provider",
        "_sac",
        "_ser",
    )

    def __init__(
        self,
        index: int,
        sec_can: SharedCANExcept,
        sac_calib: SharedAppConfigCalibration,
        sac: SharedAppConfig,
        ser: SharedErrors,
        producer_flow: MessageFlow[CanData],
        activator: ProcessActivator,
        name: str | None = None,
    ) -> None:
        super().__init__(producer_flow, sec_can, activator, name)
        self._index: int = index
        self._sac_calib: SharedAppConfigCalibration = sac_calib

        # self.CameraConfig: DataCaptureConf.CameraConf = (
        #     self._app_config_calib.dataCapture.Camera
        # )

        self._sac: SharedAppConfig = sac
        self._provider: CanDataProvider
        self._ser: SharedErrors = ser
        self._sec_can: SharedCANExcept = sec_can
        self._err_config: ErrorConfig

    def _start_restart(self) -> None:
        # TODO """必要に応じて実際にプロセスを落とさないで再起動で実行する処理を記載""" (NSW)
        self._config_load()
        calib_mode: bool = bool(self._app_config.General.operation_mode == OPM.CALIB)
        if calib_mode:
            self._frame = self._app_config_calib.dataCapture.s_frame
        else:
            self._frame: int = self._app_config.Scrutinizer.s_frame
        calib_can = self._app_config_calib.dataCapture.Can
        if calib_mode and calib_can.is_fixed_yaw:
            self._provider = FixedYawCanDataProvider(calib_can.fixed_yaw_deg)
        elif isinstance(self._provider, FixedYawCanDataProvider):
            self._change_device()
        else:
            self._change_file_name_index()
        self._clockPprovider.reset_time()

        self.producer.require_restart()
        del self.producer

    def _restart_surround_file_input_if_needed(self) -> None:
        calib_mode = bool(self._app_config.General.operation_mode == OPM.CALIB)
        if (
            calib_mode
            or not self._app_config.DEFAULT.File_Input
            or not self._app_config.Scrutinizer.file_input_loop
            or self._frame <= self._end_frame
        ):
            return

        self._frame = self._app_config.Scrutinizer.s_frame
        self._change_file_input_file(self._app_config.CAN.c_file, self._frame)

    def _config_load(self) -> None:
        self._app_config: AppConfig = self._sac.read()
        self._last_updated: int = self._sac.last_updated
        self._app_config_calib: AppConfigCalibration = self._sac_calib.read()
        self.DefaultConfig: DefaultConf = self._app_config_calib.default
        calib_mode: bool = bool(self._app_config.General.operation_mode == OPM.CALIB)
        if calib_mode:
            self._end_frame = self._app_config_calib.dataCapture.e_frame
        else:
            self._end_frame: int = self._app_config.Scrutinizer.e_frame

    def _err_config_load(self) -> None:
        self._err_config = self._ser.shared_err_conf.read()

        # NOTE: このプロセスで実施する全ての診断クラスのupdateをここに追加していく
        self._ser.module_errors[ModuleErrorIndex.CAN_MODULE_ERROR].update(
            self._err_config
        )
        self._ser.state_errors_D[StateErrorDIndex.FILE_IO_ERROR].update(
            self._err_config
        )
        self._ser.action_errors_A_C[ActionErrorIndex.CONFIG_FILE_MISSING].update(
            self._err_config
        )

    def _change_file_name_index(self) -> None:
        """FileInputのCANファイルと開始frameを現在のモードへ合わせる。"""
        calib_mode: bool = bool(self._app_config.General.operation_mode == OPM.CALIB)
        calib_can = self._app_config_calib.dataCapture.Can
        if not self._app_config.DEFAULT.File_Input:
            return
        if calib_mode and calib_can.is_fixed_yaw:
            return
        if calib_mode:
            can_file_path = calib_can.c_file
        else:
            can_file_path = self._app_config.CAN.c_file
        if isinstance(self._provider, CanFileProvider):
            self._change_file_input_file(can_file_path, self._frame)

    def _report_file_io_error(self, file_path: str, error: Exception) -> None:
        file_io_error = self._ser.state_errors_D[StateErrorDIndex.FILE_IO_ERROR]
        result = file_io_error.errors_diagnosis(True)
        file_io_error.log_output(
            *result,
            StateErrorDIndex.FILE_IO_ERROR,
            file_path,
            "read file-input CAN CSV",
            f"{type(error).__name__}: {error}",
        )

    def _report_can_id_map_error(self, error: Exception) -> None:
        diagnosis = self._ser.action_errors_A_C[
            ActionErrorIndex.CONFIG_FILE_MISSING
        ]
        if diagnosis.excepts_diagnosis(error):
            diagnosis.log_output(
                True,
                False,
                ActionErrorIndex.CONFIG_FILE_MISSING,
                error,
            )

    def _change_file_input_file(self, file_path: str, index: int) -> None:
        assert isinstance(self._provider, CanFileProvider)
        try:
            self._provider.change_file_name_index(file_path, index)
        except (OSError, UnicodeError, ValueError, TypeError, KeyError) as error:
            self._report_file_io_error(file_path, error)
            raise
        self._ser.state_errors_D[
            StateErrorDIndex.FILE_IO_ERROR
        ].errors_diagnosis(False)

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
            self._frame = self._app_config_calib.dataCapture.s_frame
        else:
            self._frame: int = self._app_config.Scrutinizer.s_frame
        self._change_device()
        self._change_clock_info()
        self.create_producer_and_consumer()

    def create_producer_and_consumer(self) -> None:
        self.producer: Producer[CanData] = self._producer_flow.create_producer()

    def restart_completed(self) -> None:
        self.producer.restart_completed()

    def _shutdown(self) -> None:
        pass

    @log_target("CAN入力I/F", ProfCategory.Process)
    def _update(self) -> CanData | None:
        # CANデータ受信イベントが発生するまでここでロックされる
        yaw_angle_data, lever_pressure = self._provider.receive_can_data()
        t: float = self._clockPprovider.get_time()
        if self._clockPprovider.isframeexceeded():
            self._unsubscribe()
        now = time.perf_counter()
        self._sec_can.last_heartbeat.value = now
        return CanData(
            yaw_angle_deg=float(yaw_angle_data),
            lever_pressure=np.asarray(lever_pressure, dtype=np.float16),
            frame=self._frame,
            time=t,
        )

    def start_diagnosis(self) -> None:
        self._sec_can.last_heartbeat.value = INVALID_TIMESTAMP
        self._sec_can.is_heartbeat_enabled.value = True
        self._sec_can.last_received.value = INVALID_TIMESTAMP
        self._sec_can.is_received_enabled.value = True

    def stop_diagnosis(self) -> None:
        self._sec_can.is_heartbeat_enabled.value = False
        self._sec_can.is_received_enabled.value = False

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
                output_data: CanData | None = self._update()
                if output_data is None:
                    continue
                self._frame += 1
                self.producer.produce(output_data)
            except Exception as e:
                is_state_error_d_exception = self._ser.is_state_error_d_exception(
                    e, self._logger
                )
                if not is_state_error_d_exception:
                    if self._ser.module_errors[
                        ModuleErrorIndex.CAN_MODULE_ERROR
                    ].excepts_diagnosis(e):
                        self._ser.module_errors[
                            ModuleErrorIndex.CAN_MODULE_ERROR
                        ].log_output(
                            ResultDiagnosis.DETECTION,
                            ResultDiagnosis.DETECTION,
                            ModuleErrorIndex.CAN_MODULE_ERROR,
                            e,
                        )
                    else:
                        raise e

    def _change_clock_info(self) -> None:
        file_input: bool = self._app_config.DEFAULT.File_Input
        calib_mode: bool = bool(self._app_config.General.operation_mode == OPM.CALIB)
        if calib_mode:
            if file_input:
                # TODO CANの設定値必要。現在は、cameraの設定値をそのまま記載 (NSW)
                # basetime: datetime.datetime = datetime.datetime.strptime(
                #     self.CameraConfig.videofile_basetime, "%Y-%m-%d_%H-%M-%S"
                # )
                # steptime: float = self.CameraConfig.videofile_steptime
                basetime: datetime.datetime = datetime.datetime.strptime(
                    "2025-09-01_09-00-00", "%Y-%m-%d_%H-%M-%S"
                )
                steptime: float = 0.1
                self._clockPprovider = DummyClockProvider(
                    basetime, steptime, self._frame, self._end_frame
                )
            else:
                self._clockPprovider = TimeClockProvider()
        else:
            self._clockPprovider = PerfCounterClockProvider()

    def _change_device(self) -> None:
        file_input: bool = self._app_config.DEFAULT.File_Input
        use_shi_lib: bool = self._app_config.DEFAULT.use_shi_lib
        calib_mode: bool = bool(self._app_config.General.operation_mode == OPM.CALIB)
        calib_can = self._app_config_calib.dataCapture.Can

        if calib_mode and calib_can.is_fixed_yaw:
            self._provider = FixedYawCanDataProvider(calib_can.fixed_yaw_deg)
        elif file_input:
            from argus_synchro.device.can.can_receiver import CanFile, CanIdMapError
            from argus_synchro.provider.can_data import CanFileProvider

            can_config = self._app_config.CAN
            if calib_mode:
                can_config = replace(can_config, c_file=calib_can.c_file)
            try:
                device = CanFile(
                    can_config,
                    self._app_config.UI_IF.crane_model,
                    self._app_logger_factory,
                )
            except CanIdMapError as error:
                self._report_can_id_map_error(error)
                raise
            except (OSError, UnicodeError, ValueError, TypeError, KeyError) as error:
                self._report_file_io_error(self._app_config.CAN.c_file, error)
                raise
            self._ser.state_errors_D[
                StateErrorDIndex.FILE_IO_ERROR
            ].errors_diagnosis(False)
            self._provider = CanFileProvider(
                device,
                self._frame,
            )
        elif use_shi_lib:
            from argus_synchro.device.can.can_receiver import CanIdMapError
            from argus_synchro.device.can.shi_lib_can_receiver import ShiLibCan
            from argus_synchro.provider.can_data import ShiLibCanProvider

            try:
                device = ShiLibCan(
                    self._index,
                    self._app_config.CAN,
                    self._app_config.UI_IF.crane_model,
                    self._app_logger_factory,
                )
            except CanIdMapError as error:
                self._report_can_id_map_error(error)
                raise
            self._provider = ShiLibCanProvider(device)
        else:
            # 実機
            from argus_synchro.device.can.can_receiver import Can, CanIdMapError
            from argus_synchro.provider.can_data import CanReceiverProvider

            try:
                device = Can(
                    self._index,
                    self._app_config.CAN,
                    self._app_config.UI_IF.crane_model,
                    self._app_logger_factory,
                    self._ser,
                    self._sec_can,
                )
            except CanIdMapError as error:
                self._report_can_id_map_error(error)
                raise
            self._provider = CanReceiverProvider(device, self._ser)
