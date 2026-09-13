from __future__ import annotations

# パフォーマンスプロファイリング（デバッグ用）
import cProfile
import json
import queue
import sys
import time
import traceback
import typing
from datetime import datetime as dt

from pandas.errors import EmptyDataError, ParserError
from argus_synchro_lib.visualizer import GodotUIVisualizer

import argus_synchro.calibration_mat_generator_modules.ctrl.calibcheck2d3d as calibcheck2d3d_log
from argus_synchro import SubScrutinizer
from argus_synchro.calibration_mat_generator_modules.ctrl import (
    calibration2d3d,
    calibration3d3d,
)
from argus_synchro.calibration_mat_generator_modules.ctrl.calibcheck2d3d import (
    calibcheck2d3d,
)

# 3d3d校正プログラムもここに統合
from argus_synchro.calibration_mat_generator_modules.ctrl.calibration2d3d import (
    calibration2d3d_class,
)
from argus_synchro.calibration_mat_generator_modules.ctrl.calibration3d3d import (
    calibration3d3d_class,
)
from argus_synchro.calibration_mat_generator_modules.facade import calib_godot_interface
from argus_synchro.common.common import t_py_col_res
from argus_synchro.diagnosis.error_diagnosis import ResultDiagnosis
from argus_synchro.interface.octotree_func import (
    OctoTreeFuncInterface,
)
from argus_synchro.message.calib_fifo_message import FIFOData
from argus_synchro.process import ProcessBase
from argus_synchro.process.message import Consumer, MessageFlow
from argus_synchro.profiler import log_main
from argus_synchro.profiler.prof_fps import ProfFps
from argus_synchro.shared_errors import (
    ActionErrorIndex,
    ModuleErrorIndex,
    SharedErrors,
    StateErrorDIndex,
)

if typing.TYPE_CHECKING:
    from argus_synchro.config.app_config import AppConfig
    from argus_synchro.shared_app_config import (
        SharedAppConfig,
        SharedAppConfigCalibration,
    )
    from argus_synchro.shared_excepts import (
        SharedExcepts,
    )
import datetime
import os

from argus_synchro import (
    calibration_mat_generator_modules,
    shared_app_config,
)

# 校正関連
from argus_synchro.calibration_mat_generator_modules.boss import boss

# from argus_synchro.calibration_mat_generator_modules.facade import FacadeUIManager
from argus_synchro.calibration_mat_generator_modules.facade import (
    CalibrationCommonStatus,
    CalibrationUIGodot,
)
from argus_synchro.config.app_config_calibration import AppConfigCalibration
from argus_synchro.process.synchronizer import ProcessActivator
from argus_synchro.shared_excepts import SharedCalMatGeneratorExcept
from argus_synchro.SystemMonitor.status_mmap import StatusMMAP


class CalibProcess(ProcessBase):
    __slots__ = (
        "_MonitoredTime",
        "_TimeMonitor",
        "_app_config",
        "_fps_prof",
        "_last_updated",
        "_sac",
        "_sac_calib",
        "_ser",
        "fifo_input",
        "sec",
    )

    def __init__(
        self,
        sac_calib: shared_app_config.SharedAppConfigCalibration,
        sac: SharedAppConfig,
        sec: SharedExcepts,
        ser: SharedErrors,
        sec_calib_ex: SharedCalMatGeneratorExcept,
        fifo_input: MessageFlow[FIFOData],
        activator: ProcessActivator,
        arglist: list[str] = sys.argv,
        inifilepath: str | None = None,
    ) -> None:
        super().__init__(sec_calib_ex, activator, "CalibProcess")
        self._fifo_input: MessageFlow[FIFOData] = self._subscribe(
            fifo_input,
        )

        self.collision_clusters: t_py_col_res = {}

        self._MonitoredTime: float = 0
        self._pre_frame: int = 0
        self._fps_prof: ProfFps = ProfFps(self.__class__.__name__, console=True)

        if inifilepath is None:
            inifilepath = sys.argv[1]
        assert os.path.isfile(inifilepath)
        self.arglist: list[str] = arglist
        self.inifilepath: str = inifilepath

        self.sec: SharedExcepts = sec
        self._sac: SharedAppConfig = sac
        self._sac_calib: SharedAppConfigCalibration = sac_calib
        self._ser: SharedErrors = ser
        self._config_load()

        # _startupで初期化
        self._octotree_func: OctoTreeFuncInterface
        self._visual_ui: GodotUIVisualizer

    def _config_load(self) -> None:
        self._app_config: AppConfig = self._sac.read()
        self._last_updated: int = self._sac.last_updated
        self.app_config_calib: AppConfigCalibration = self._sac_calib.read()

    def _apply_parameters(self) -> None:
        # 再生成
        pass

    def _err_config_load(self) -> None:
        self._err_config = self._ser.shared_err_conf.read()
        self._ser.state_errors_D[StateErrorDIndex.FILE_IO_ERROR].update(
            self._err_config
        )
        self._ser.state_errors_D[StateErrorDIndex.INVALID_DATA_INPUT].update(
            self._err_config
        )
        self._ser.state_errors_D[StateErrorDIndex.ARRAY_SHAPE_ERROR].update(
            self._err_config
        )
        self._ser.action_errors_A_C[ActionErrorIndex.AI_MODEL_LOAD_FAILED].update(
            self._err_config
        )
        self._ser.module_errors[ModuleErrorIndex.CALIBRATION_MODULE_ERROR].update(
            self._err_config
        )

    def _report_file_io_error(self, error: Exception, operation: str) -> None:
        file_io_error = self._ser.state_errors_D[StateErrorDIndex.FILE_IO_ERROR]
        result = file_io_error.errors_diagnosis(True)
        error_path = getattr(error, "filename", None) or self.inifilepath
        file_io_error.log_output(
            *result,
            StateErrorDIndex.FILE_IO_ERROR,
            str(error_path),
            operation,
            f"{type(error).__name__}: {error}",
        )

    def _startup(self) -> None:
        self._err_config_load()
        try:
            self.uimanager = CalibrationUIGodot(
                sac=self._sac,
                sec=self.sec,
                app_logger_factory=self._app_logger_factory,
                directory_config=self._directory_config,
            )
            _ = StatusMMAP(
                self._logger,
                create=False,
                directory_config=self._directory_config,
            )
            # mprof_handler = MachineProfile.MachineProfileHandler()
            self._logger.info(
                f"{datetime.datetime.now()} - calibration_mat_generator started"
            )
            admin_inst = calibration_mat_generator_modules.calibration2d3d_manager_class(
                sec=self.sec,
                ser=self._ser,
                arglist=[],
                shared_errors=self._ser,
                inifilepath=self.inifilepath,
                app_logger_factory=self._app_logger_factory,
                directory_config=self._directory_config,
            )
        except (OSError, UnicodeError, json.JSONDecodeError) as error:
            self._report_file_io_error(
                error,
                "read calibration auxiliary file during startup",
            )
            raise
        self.uimanager.apply_config(
            FacadeConfInst=admin_inst.get_calibconfig(sec=self.sec).facadeConf
        )

        self._pre_frame = 0
        self._MonitoredTime: float = time.time()
        self._fps_prof.start()

        self.check_allowexit(
            app_config_calib=self.app_config_calib
        )  # 一時ファイル削除のため空読み

        self.is_enable_profiler = self.app_config_calib.debug.is_enable_profiler
        # 校正モードのプロファイラ立ち上げ
        self.pr = None
        try:
            if self.is_enable_profiler:
                self.pr = cProfile.Profile()
                self.pr.enable()
        except Exception as error:
            self._logger.warning(
                f"cProfile enable failed: {error}; continue without profiler"
            )

        self.boss_inst = boss(
            app_config_calib=self.app_config_calib,
            app_logger_factory=self._app_logger_factory,
            shared_errors=self._ser,
        )
        # UIとの連携とそれに伴う2D3D・3D3D校正処理の起動・終了等をここで動作定義

        self.uimanager.initialize_internal_values(errorcode_pre=0)  # CalibStatus:A0
        self.uimanager.set_dummydata(enable_systemerrorflag=True, enable_errorflag=True)
        self.uimanager.transmit_setdata(sec=self.sec, ref_t=0, is_firstframe=True)
        # time.sleep(0.01)
        # self.uimanager.transmit_setdata(sec=sec, ref_t=None, is_firstframe=False, force_changepage=True)
        # time.sleep(0.01)
        # self.uimanager.transmit_setdata(sec=sec, ref_t=None, is_firstframe=False, force_changepage=True)

        # モードごとに初期設定をする
        if self.app_config_calib.debug.calib2d3d_fileend_autoexit and (
            (self._sac.read().CalibMode.isRunning3D3Dcalib)
            or (not self._sac.read().CalibMode.isRunning2D3Dcalib)
            or (self._sac.read().CalibMode.start2D3DCalibCalc)
            or (self._sac.read().CalibMode.isRunning2D3Dcheck)
            or (self._sac.read().CalibMode.start2D3DCheckCalc)
        ):
            self._logger.critical(
                "************  デバッグモード設定エラー（安全策としての停止）   ************",
            )
            errorstr = "calib2d3d_fileend_autoexitがTrueです。2D3D校正にて自動開始自動終了が指定されましたがモード設定状況が想定と異なります。中止。"
            self._logger.critical(errorstr)
            self._logger.critical(
                "************  デバッグモード設定エラー（安全策としての停止）   ************"
            )
            raise RuntimeError(errorstr)

        self.create_producer_and_consumer()

    def create_producer_and_consumer(self) -> None:
        self.fifo_consumer: Consumer[FIFOData] = self._fifo_input.create_consumer()

    def restart_completed(self) -> None:
        self.fifo_consumer.restart_completed()

    def _start_restart(self) -> None:
        # TODO """必要に応じて実際にプロセスを落とさないで再起動で実行する処理を記載""" (NSW)
        self.fifo_consumer.require_restart()
        del self.fifo_consumer

    def _log_register(self) -> None:
        super()._log_register()
        self._sac.log_register(self._app_logger_factory)
        self.sec.log_register(self._app_logger_factory)
        self._sac_calib.log_register(self._app_logger_factory)
        SubScrutinizer.log_register(self._app_logger_factory)
        calib_godot_interface.log_register(self._app_logger_factory)
        calibration3d3d.log_register(self._app_logger_factory)
        calibration2d3d.log_register(self._app_logger_factory)
        calibcheck2d3d_log.log_register(self._app_logger_factory)

    def _shutdown(self) -> None:
        self.sec.CalMatGen_ex.IsFinished.value = True
        self._logger.info("終了条件に到達.")
        self._fps_prof.export()

        self._logger.info("========================")
        self._logger.info("calibration2d3d_manager_class ended")
        self._logger.info("========================")

        assert self.uimanager is not None
        self.uimanager.set_end_calibration_flag(1)
        # self.uimanager.transmit_setdata(sec=sec, ref_t=None, is_firstframe=False)
        self.uimanager.transmit_setdata(
            sec=self.sec, ref_t=None, is_firstframe=False, force_changepage=True
        )
        time.sleep(0.01)
        self.uimanager.transmit_setdata(
            sec=self.sec, ref_t=None, is_firstframe=False, force_changepage=True
        )

        if self.app_config_calib.debug.calib2d3d_fileend_autoexit:
            # Exitを許可するフラグとしてファイルを作成（このプロセスは即終了するため伝える手段がない）
            with open(
                self.app_config_calib.debug.calib2d3d_fileend_autoexit_flagfile_path,
                mode="w",
            ) as wf:
                wf.write("")
            time.sleep(0.1)  # 念の為のファイル保存反映待ち
            self._sac.is_restart_required.value = True

        self.app_close()

    @log_main()
    def _loop(self) -> None:
        try:
            while self.enable:
                if self._sac.last_updated > self._last_updated:
                    self._config_load()
                    self._apply_parameters()

                is_running_3d3d = False
                try:
                    calib_mode = self._sac.read().CalibMode
                    is_running_3d3d = calib_mode.isRunning3D3Dcalib
                    if is_running_3d3d:
                        self.calib3d3d_app(
                            self.fifo_consumer,
                            self.uimanager,
                            self.sec,
                            self._sac,
                            self.app_config_calib,
                        )
                    elif self._sac.read().CalibMode.isRunning2D3Dcalib:
                        self.calib2d3d_app(
                            self.fifo_consumer,
                            self.uimanager,
                            self.sec,
                            self._sac,
                            self.app_config_calib,
                        )
                    elif self._sac.read().CalibMode.isRunning2D3Dcheck:
                        self.calibcheck2d3d_app(
                            self.fifo_consumer,
                            self.uimanager,
                            self.sec,
                            self._sac,
                            self.app_config_calib,
                        )
                    else:
                        self.wait_app(
                            self.fifo_consumer,
                            self.uimanager,
                            self.sec,
                            self._sac,
                            self.app_config_calib,
                        )
                except Exception as e:
                    if is_running_3d3d:
                        self._set_calib3d3d_error_state(self.uimanager)
                        self.uimanager.set_dummydata(
                            enable_systemerrorflag=True,
                            enable_errorflag=True,
                        )
                        self.uimanager.transmit_setdata(sec=self.sec, ref_t=None)
                    if self._ser.is_state_error_d_exception(e, self._logger):
                        continue
                    if isinstance(
                        e,
                        (
                            OSError,
                            UnicodeError,
                            json.JSONDecodeError,
                            EmptyDataError,
                            ParserError,
                            KeyError,
                        ),
                    ):
                        self._report_file_io_error(
                            e,
                            "read calibration auxiliary file",
                        )
                        raise
                    if self._ser.module_errors[
                        ModuleErrorIndex.CALIBRATION_MODULE_ERROR
                    ].excepts_diagnosis(e):
                        self._ser.module_errors[
                            ModuleErrorIndex.CALIBRATION_MODULE_ERROR
                        ].log_output(
                            ResultDiagnosis.DETECTION,
                            ResultDiagnosis.DETECTION,
                            ModuleErrorIndex.CALIBRATION_MODULE_ERROR,
                            e,
                        )
                    else:
                        raise e

        except KeyboardInterrupt:
            self._logger.info("KeyboardInterrupt を検知して終了")

    def app_close(self) -> None:
        try:
            if self.is_enable_profiler and self.pr is not None:
                self.pr.disable()
                profile_path = f"calib_profiler_results_{dt.now().strftime('%Y-%m-%d_%H-%M-%S_%f')}.prof"
                self.pr.dump_stats(profile_path)
        except Exception as error:
            self._logger.warning(
                f"cProfile disable or dump_stats failed: {error}; continue shutdown"
            )

        self._logger.info("close monitor")
        if self.uimanager is not None:
            self.uimanager.close()
        self._logger.info("close boss_inst")
        self.boss_inst.close()

    @staticmethod
    def _set_calib3d3d_error_state(monitor: CalibrationUIGodot) -> None:
        monitor.set_errors_calibcommon(2)
        monitor.set_errorcode_unexpected_exception(True)

    @staticmethod
    # 精度自動テスト用の終了可否判定 製品版への実装はおそらく危険？
    def check_allowexit(app_config_calib: AppConfigCalibration) -> bool:
        if os.path.isfile(
            app_config_calib.debug.calib2d3d_fileend_autoexit_flagfile_path
        ):
            os.remove(app_config_calib.debug.calib2d3d_fileend_autoexit_flagfile_path)
            if app_config_calib.debug.calib2d3d_fileend_autoexit:
                return True
        return False

    def calib3d3d_app(
        self,
        fifo_consumer: Consumer[FIFOData],
        monitor: CalibrationUIGodot,
        sec: SharedExcepts,
        sac: SharedAppConfig,
        app_config_calib: AppConfigCalibration,
    ):
        assert self.uimanager is not None

        # CalibStatus:B1
        self.uimanager.reset_internal_values(
            errorcode_pre=0,
            status_calibcommon=CalibrationCommonStatus.RUNNING,
            currentmode=1,
        )  # running:1, 3d3d:1
        self.uimanager.set_dummydata(enable_systemerrorflag=True, enable_errorflag=True)
        self.uimanager.transmit_setdata(sec=sec, ref_t=None, is_firstframe=True)
        try:
            self.boss_inst.pre_calib3d3d(
                monitor=self.uimanager,
                sec=sec,
                sac=sac,
                app_config_calib=self.app_config_calib,
            )
            self._logger.info("calib3d3d_app started")

            self._logger.info("app_loopmain begin")
            last_fifo_data: FIFOData | None = None
            app_loop_failed = False
            try:
                self.boss_inst.calibration3d3d_inst.pre_app_loopmain(
                    monitor=self.uimanager,
                    sec=self.sec,
                    sac=self._sac,
                    app_config_calib=self.app_config_calib,
                )
                # TODO: 下記構造検討　他のメソッドを下に追いやるか下記をどこかに格納するか？
                while self.enable:
                    if self._sac.last_updated > self._last_updated:
                        self._config_load()
                        self._apply_parameters()

                    # 入力処理(点群ほか)
                    try:
                        with fifo_consumer.consume() as fifo_data:
                            last_fifo_data = fifo_data
                            iscontinue = self.boss_inst.calibration3d3d_inst.app_loopmain(
                                fifo_data,
                                self.uimanager,
                                self.sec,
                                self._sac,
                                self.app_config_calib,
                                self.app_config_calib.filepath_io.Calib3d3dmat_lidars,
                            )
                    except queue.Empty:
                        time.sleep(0.1)
                        continue
                    if not iscontinue:
                        break
            except Exception as ea:
                self._logger.error(
                    f"app_loopmain: exception! {ea} - \n{traceback.format_exc()}"
                )
                self._set_calib3d3d_error_state(monitor)
                app_loop_failed = True
            if not app_loop_failed and last_fifo_data is not None:
                self.boss_inst.calibration3d3d_inst.post_app_loopmain(
                    last_fifo_data,
                    self.uimanager,
                    self.sec,
                    self._sac,
                    self.app_config_calib,
                    self.app_config_calib.filepath_io.Calib3d3dmat_lidars,
                )
            timercount = 0
            while self.enable:
                timercount = calibration3d3d_class.end_wait(
                    timercount=timercount, sec=sec, sac=sac, monitor=monitor
                )
            calibration3d3d_class.send_end_wait(sec=sec, sac=sac, monitor=monitor)
            self.boss_inst.post_calib3d3d(
                monitor=self.uimanager,
                sec=sec,
                sac=sac,
                app_config_calib=self.app_config_calib,
            )
            self.sec.Lidar_SM_ex.write_has_not_calibrated(False)

        except Exception as ea:
            self._logger.critical(
                f"calib3d3d_app exception: {ea}, traceback: \n{traceback.format_exc()}",
            )
            self._set_calib3d3d_error_state(monitor)
            timercount = 0
            while self.enable:
                timercount = calibration3d3d_class.end_wait(
                    timercount=timercount, sec=sec, sac=sac, monitor=monitor
                )
            calibration3d3d_class.send_end_wait(sec=sec, sac=sac, monitor=monitor)

    def calib2d3d_app(
        self,
        fifo_consumer: Consumer[FIFOData],
        monitor: CalibrationUIGodot,
        sec: SharedExcepts,
        sac: SharedAppConfig,
        app_config_calib: AppConfigCalibration,
    ) -> None:
        # CalibStatus:C1
        assert self.uimanager is not None
        self.uimanager.reset_internal_values(
            errorcode_pre=0,
            status_calibcommon=CalibrationCommonStatus.RUNNING,
            currentmode=2,
            currentcamera=sac.read().CalibMode.cameraID,
        )  # running:1, 2d3d:2
        self.uimanager.set_dummydata(enable_systemerrorflag=True, enable_errorflag=True)
        self.uimanager.transmit_setdata(sec=sec, ref_t=None, is_firstframe=True)

        try:
            self.boss_inst.pre_calib2d3d_app(
                monitor=self.uimanager,
                sec=self.sec,
                sac=self._sac,
                app_config_calib=self.app_config_calib,
            )
            try:
                self.boss_inst.calibration2d3d_inst.pre_app_loopmain(
                    monitor=self.uimanager,
                    sec=self.sec,
                    sac=self._sac,
                    resultmat_path=self.app_config_calib.filepath_io.Calib2d3dmat_cameras[
                        sac.read().CalibMode.cameraID
                    ],
                )
                # TODO: 下記構造検討　他のメソッドを下に追いやるか下記をどこかに格納するか？
                while self.enable:
                    if self._sac.last_updated > self._last_updated:
                        self._config_load()
                        self._apply_parameters()

                    try:
                        with fifo_consumer.consume() as fifo_data:
                            iscontinue = self.boss_inst.calibration2d3d_inst.app_loopmain(
                                fifo_data,
                                self.uimanager,
                                self.sec,
                                self._sac,
                                self.app_config_calib.filepath_io.Calib2d3dmat_cameras[
                                    sac.read().CalibMode.cameraID
                                ],
                            )
                    except queue.Empty:
                        time.sleep(0.1)
                        continue
                    if not iscontinue:
                        break

            except Exception as ea:
                self._logger.error(
                    f"app_loopmain: exception! {ea} - \n{traceback.format_exc()}"
                )
                monitor.set_errorcode_unexpected_exception(True)
            self.boss_inst.calibration2d3d_inst.post_app_loopmain(
                monitor=self.uimanager,
                sec=sec,
                sac=sac,
            )
            if not self.app_config_calib.debug.calib2d3d_fileend_autoexit:
                timercount = 0
                while self.enable:
                    timercount = calibration2d3d_class.end_wait(
                        timercount=timercount, sec=sec, sac=sac, monitor=monitor
                    )
                calibration2d3d_class.send_end_wait(sec=sec, sac=sac, monitor=monitor)
            self.boss_inst.post_calib2d3d_app()

        except Exception as ea:
            self._logger.critical(
                f"calib2d3d_app exception: {ea}, traceback: \n{traceback.format_exc()}",
            )
            monitor.set_errorcode_unexpected_exception(True)
            timercount = 0
            while self.enable:
                timercount = calibration2d3d_class.end_wait(
                    timercount=timercount, sec=sec, sac=sac, monitor=monitor
                )
            calibration2d3d_class.send_end_wait(sec=sec, sac=sac, monitor=monitor)
        if self.app_config_calib.debug.calib2d3d_fileend_autoexit:
            # 停止させる
            self._unsubscribe()

    def calibcheck2d3d_app(
        self,
        fifo_consumer: Consumer[FIFOData],
        monitor: CalibrationUIGodot,
        sec: SharedExcepts,
        sac: SharedAppConfig,
        app_config_calib: AppConfigCalibration,
    ):
        assert self.uimanager is not None
        # CalibStatus:D1
        self.uimanager.reset_internal_values(
            errorcode_pre=0,
            status_calibcommon=CalibrationCommonStatus.RUNNING,
            currentmode=3,
        )  # running:1, 2d3dcheck:3
        self.uimanager.set_dummydata(enable_systemerrorflag=True, enable_errorflag=True)
        self.uimanager.transmit_setdata(sec=sec, ref_t=None, is_firstframe=True)

        try:
            self.boss_inst.pre_calibcheck2d3d_app(
                monitor=self.uimanager,
                sec=self.sec,
                sac=self._sac,
                app_config_calib=self.app_config_calib,
            )
            try:
                self.boss_inst.calibcheck2d3d_inst.pre_app_loopmain(
                    monitor=self.uimanager,
                    sec=self.sec,
                    sac=self._sac,
                )
                try:
                    # TODO: 下記構造検討　他のメソッドを下に追いやるか下記をどこかに格納するか？
                    while self.enable and (not sac.read().CalibMode.start2D3DCheckCalc):
                        # CalibStatus:D2 現状はstart2D3DCheckCalcが入り次第while loopから抜ける
                        if self._sac.last_updated > self._last_updated:
                            self._config_load()
                            self._apply_parameters()

                        try:
                            with fifo_consumer.consume() as fifo_data:
                                iscontinue = (
                                    self.boss_inst.calibcheck2d3d_inst.app_loopmain(
                                        fifo_data,
                                        self.uimanager,
                                        self.sec,
                                        self._sac,
                                    )
                                )
                        except queue.Empty:
                            time.sleep(0.1)
                            continue

                        if not iscontinue:
                            break
                except KeyboardInterrupt as e:
                    self._logger.info(f"{e}, calibcheck2d3d app_loopmain ended")
                except Exception as ea:
                    self._logger.error(
                        f"app_loopmain: exception! {ea} - \n{traceback.format_exc()}"
                    )
                    monitor.set_errorcode_unexpected_exception(True)
                finally:
                    if self.enable and sac.read().CalibMode.start2D3DCheckCalc:
                        self.boss_inst.calibcheck2d3d_inst.post_app_loopmain(
                            monitor=self.uimanager,
                            sec=sec,
                            sac=sac,
                        )
                    else:
                        self._logger.info(
                            "calibcheck2d3d calculation skipped: "
                            "start2D3DCheckCalc was not requested"
                        )
            except Exception as ea:
                self._logger.error(
                    f"app_loopmain (status D3~): exception! {ea} - \n{traceback.format_exc()}",
                )
                monitor.set_errorcode_unexpected_exception(True)
            timercount = 0
            while self.enable:
                timercount = calibcheck2d3d.end_wait(
                    timercount=timercount,
                    monitor=self.uimanager,
                    sec=sec,
                    sac=sac,
                )
            calibcheck2d3d.send_end_wait(
                monitor=self.uimanager,
                sec=sec,
                sac=sac,
            )
            self.boss_inst.post_calibcheck2d3d_app()

        except Exception as ea:
            self._logger.critical(
                f"calibcheck2d3d_app exception: {ea}, traceback: \n{traceback.format_exc()}",
            )
            monitor.set_errorcode_unexpected_exception(True)
            timercount = 0
            while self.enable:
                timercount = calibcheck2d3d.end_wait(
                    timercount=timercount,
                    monitor=self.uimanager,
                    sec=sec,
                    sac=sac,
                )

            calibcheck2d3d.send_end_wait(
                monitor=self.uimanager,
                sec=sec,
                sac=sac,
            )

    def wait_app(
        self,
        fifo_consumer: Consumer[FIFOData],
        monitor: CalibrationUIGodot,
        sec: SharedExcepts,
        sac: SharedAppConfig,
        app_config_calib: AppConfigCalibration,
    ) -> None:
        assert self.uimanager is not None
        self.uimanager.clear_internal_values(
            status_calibcommon=CalibrationCommonStatus.INACTIVE
        )
        self.uimanager.set_dummydata(
            enable_systemerrorflag=True,
            enable_errorflag=True,
            enable_yawangle=True,
        )
        self.uimanager.transmit_setdata(sec=sec, ref_t=None)
        self.boss_inst.pre_waiting_sensorcapture_app(
            monitor=self.uimanager,
            sec=self.sec,
            sac=self._sac,
            app_config_calib=self.app_config_calib,
        )
        # CalibStatus:A1
        while self.enable:
            try:
                self.boss_inst.pre_waiting_sensorcapture_app(
                    monitor=self.uimanager,
                    sec=self.sec,
                    sac=self._sac,
                    app_config_calib=self.app_config_calib,
                )
                self.boss_inst.wait_app_inst.pre_app_loopmain()
                try:
                    while self.enable:
                        if self._sac.last_updated > self._last_updated:
                            self._config_load()
                            self._apply_parameters()

                        try:
                            with fifo_consumer.consume() as fifo_data:
                                self.boss_inst.wait_app_inst.app_loopmain(
                                    fifo_data,
                                    self.uimanager,
                                    self.sec,
                                    self._sac,
                                )
                        except queue.Empty:
                            time.sleep(0.1)
                            continue

                    self.boss_inst.wait_app_inst.post_app_loopmain()
                except Exception as e:
                    self._logger.error(
                        f"app_loopmain: exception! {e} - \n{traceback.format_exc()}"
                    )

                self.boss_inst.post_waiting_sensorcapture_app()
            except Exception as e:
                self._logger.error(f"wait: exception! {e} - \n{traceback.format_exc()}")
