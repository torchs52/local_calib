from __future__ import annotations

import time
from contextlib import ExitStack

from argus_synchro.config.app_config import AppConfig
from argus_synchro.diagnosis.error_config import ErrorConfig
from argus_synchro.diagnosis.error_diagnosis import ResultDiagnosis
from argus_synchro.message.input_message import (
    ImuData,
)
from argus_synchro.process import ProcessBase
from argus_synchro.process.message import Consumer, MessageFlow
from argus_synchro.process.synchronizer import ProcessActivator
from argus_synchro.profiler import log_target
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
    SharedLidarShiftMonitorExcept,
)
from argus_synchro.SystemMonitor.LidarShiftMonitor import LidarShiftMonitor


class LidarShiftMonitorProcess(ProcessBase):
    __slots__ = (
        "_app_config",
        "_end_frame",
        "_err_config",
        "_imu_inputs",
        "_last_updated",
        "_lidmonitor",
        "_ref_t",
        "_sac",
        "_sec_lidar_sm",
        "_ser",
    )

    def __init__(
        self,
        sec_lidar_sm: SharedLidarShiftMonitorExcept,
        sac: SharedAppConfig,
        ser: SharedErrors,
        imu_inputs: tuple[MessageFlow[ImuData], ...],
        activator: ProcessActivator,
    ) -> None:
        super().__init__(sec_lidar_sm, activator, "LidarShiftMonitorProcess")
        self._imu_inputs: tuple[MessageFlow[ImuData], ...] = tuple(
            self._subscribe(camera) for camera in imu_inputs
        )
        self._sec_lidar_sm: SharedLidarShiftMonitorExcept = sec_lidar_sm
        self._sac: SharedAppConfig = sac
        self._ser: SharedErrors = ser

        # _startupで初期化
        self._ref_t: int
        self._err_config: ErrorConfig
        self._diagnosis_warmup_start: float | None = None
        self._last_heartbeat: float = time.perf_counter()

        self._req_start_diagnosis: bool = False
        self._req_stop_diagnosis: bool = False

    def _config_load(self) -> None:
        self._app_config: AppConfig = self._sac.read()
        self._last_updated: int = self._sac.last_updated

    def _err_config_load(self) -> None:
        self._err_config = self._ser.shared_err_conf.read()

        # NOTE: このプロセスで実施する全ての診断クラスのupdateをここに追加していく
        self._ser.state_errors_D[StateErrorDIndex.INVALID_DATA_INPUT].update(
            self._err_config
        )
        self._ser.state_errors_D[StateErrorDIndex.ARRAY_SHAPE_ERROR].update(
            self._err_config
        )
        self._ser.action_errors_A_C[
            ActionErrorIndex.LIDAR_POSITION_MISALIGNMENT_DETECTED
        ].update(self._err_config)
        self._ser.action_errors_A_C[
            ActionErrorIndex.SENSOR_CALIBRATION_REQUIRED
        ].update(self._err_config)
        self._ser.module_errors[
            ModuleErrorIndex.LIDAR_SHIFT_MONITOR_MODULE_ERROR
        ].update(self._err_config)

    def input_data_diagnosis(
        self,
        imu_inputs: tuple[ImuData, ...],
    ) -> bool:
        invalid_data_input = self._ser.state_errors_D[
            StateErrorDIndex.INVALID_DATA_INPUT
        ]
        input_data = imu_inputs
        result, failsafe_result = invalid_data_input.errors_diagnosis(input_data)
        invalid_data_input.log_output(
            result, failsafe_result, StateErrorDIndex.INVALID_DATA_INPUT
        )
        if result == ResultDiagnosis.DETECTION:
            return True

        imu_values = tuple(imu_input.imu for imu_input in imu_inputs)

        array_shape_error = self._ser.state_errors_D[StateErrorDIndex.ARRAY_SHAPE_ERROR]
        result, failsafe_result = array_shape_error.errors_diagnosis(
            ("imu_values", imu_values),
        )
        array_shape_error.log_output(
            result, failsafe_result, StateErrorDIndex.ARRAY_SHAPE_ERROR, self.name
        )
        return result == ResultDiagnosis.DETECTION

    def _log_register(self) -> None:
        super()._log_register()
        self._sac.log_register(self._app_logger_factory)
        self._ser.log_register(self._app_logger_factory)

    def _apply_parameters(self) -> None:
        self._lidmonitor = LidarShiftMonitor(
            calib_files=self._app_config.calibration.Lidar_calib_files,
            # Fast（瞬間ズレ）
            win_fast=self._app_config.LiDARShiftMonitor.win_fast,
            hold_fast=self._app_config.LiDARShiftMonitor.hold_fast,
            fast_abs=self._app_config.LiDARShiftMonitor.thr_fast,
            # Slow（定常ズレ：未実装なので値は何でもOK）
            win=self._app_config.LiDARShiftMonitor.win,
            hold=self._app_config.LiDARShiftMonitor.hold,
            slow_abs=self._app_config.LiDARShiftMonitor.thr_slow,
        )

    def _startup(self) -> None:
        self._config_load()
        self._err_config_load()
        self._apply_parameters()
        self._ref_t = self._app_config.Scrutinizer.s_frame
        self._sec_lidar_sm.load_has_not_calibrated()
        self.create_producer_and_consumer()

    def create_producer_and_consumer(self) -> None:
        self.imu_inputs_consumers: tuple[Consumer[ImuData], ...] = tuple(
            imu_input.create_consumer() for imu_input in self._imu_inputs
        )

    def restart_completed(self) -> None:
        for i in self.imu_inputs_consumers:
            i.restart_completed()

    def _start_restart(self) -> None:
        # TODO """必要に応じて実際にプロセスを落とさないで再起動で実行する処理を記載""" (NSW)
        pass

    def _shutdown(self) -> None:
        pass

    def start_diagnosis(self) -> None:
        self._sec_lidar_sm.last_heartbeat.value = INVALID_TIMESTAMP
        self._last_heartbeat = time.perf_counter()
        self._diagnosis_warmup_start = None
        self._sec_lidar_sm.is_heartbeat_enabled.value = False

    def stop_diagnosis(self) -> None:
        self._sec_lidar_sm.is_heartbeat_enabled.value = False
        self._diagnosis_warmup_start = None

    def _update_heartbeat(self, now: float) -> None:
        heartbeat_interval = now - self._last_heartbeat
        self._sec_lidar_sm.last_heartbeat.value = now
        self._last_heartbeat = now
        if self._sec_lidar_sm.is_heartbeat_enabled.value:
            return

        param = self._err_config.lidar_position_misalignment_not_responding
        if (
            self._diagnosis_warmup_start is None
            or heartbeat_interval > param.recovery_receive_interval_sec
        ):
            self._diagnosis_warmup_start = now
            return

        recovery_duration = max(
            param.error_recovery_confirm_duration_sec,
            param.failsafe_recovery_confirm_duration_sec,
        )
        if now - self._diagnosis_warmup_start >= recovery_duration:
            self._sec_lidar_sm.is_heartbeat_enabled.value = True

    # @log_main()
    def _loop(self) -> None:
        self.start_diagnosis()
        while self.enable:
            if self._sac.last_updated > self._last_updated:
                self._config_load()
                self._apply_parameters()
            try:
                if any(not c.wait() for c in self.imu_inputs_consumers):
                    continue

                # 入力処理
                with ExitStack() as stack:
                    imu_inputs: tuple[ImuData, ...] = tuple(
                        stack.enter_context(c.consume())
                        for c in self.imu_inputs_consumers
                    )

                    # 実際の処理
                    if self.input_data_diagnosis(imu_inputs):
                        continue
                    self._update(imu_inputs)
                    if self._req_start_diagnosis:
                        self._sec_lidar_sm.is_heartbeat_enabled.value = True
                        self._req_start_diagnosis = False
                        self._logger.warning("診断開始要求を処理完了")

            except Exception as e:
                is_state_error_d_exception = self._ser.is_state_error_d_exception(
                    e, self._logger
                )
                if not is_state_error_d_exception:
                    if self._ser.module_errors[
                        ModuleErrorIndex.LIDAR_SHIFT_MONITOR_MODULE_ERROR
                    ].excepts_diagnosis(e):
                        self._ser.module_errors[
                            ModuleErrorIndex.LIDAR_SHIFT_MONITOR_MODULE_ERROR
                        ].log_output(
                            ResultDiagnosis.DETECTION,
                            ResultDiagnosis.DETECTION,
                            ModuleErrorIndex.LIDAR_SHIFT_MONITOR_MODULE_ERROR,
                            e,
                        )
                    else:
                        raise e

    @log_target("LidarShiftMonitor", ProfCategory.Process)
    def _update(
        self,
        imu_input_data: tuple[ImuData, ...],
    ) -> None:
        self._update_heartbeat(time.perf_counter())
        self._ref_t += 1
        self._lidmonitor.detect_lidar_shift_from_k_samples(
            imu_input_data,
            self._sec_lidar_sm,
            self._app_config.LiDARShiftMonitor.num_sample_k,
            self._app_config.LiDARShiftMonitor.dt,
        )
        self._diag()

    def _diag(self) -> None:
        pid: int | None = self.pid
        if pid is None:
            # TODO: 「Lidar位置ズレ検出　未応答」用の更新をどこでやるか考える。ここか、updateの中か。
            self._logger.info("PID is None, skip diagnosis")
            return

        scr_diag = self._ser.action_errors_A_C[
            ActionErrorIndex.SENSOR_CALIBRATION_REQUIRED
        ]
        err, recover, _ = scr_diag.errors_diagnosis(self._sec_lidar_sm, pid)
        scr_diag.log_output(err, recover, ActionErrorIndex.SENSOR_CALIBRATION_REQUIRED)

        lpmd_diag = self._ser.action_errors_A_C[
            ActionErrorIndex.LIDAR_POSITION_MISALIGNMENT_DETECTED
        ]
        err, recover, _ = lpmd_diag.errors_diagnosis(self._sec_lidar_sm)
        lpmd_diag.log_output(
            err, recover, ActionErrorIndex.LIDAR_POSITION_MISALIGNMENT_DETECTED
        )
