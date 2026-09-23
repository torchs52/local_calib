import asyncio  # センサログファイル書き出し
import copy
import datetime
import faulthandler
import json
import os
import pickle
import signal
import time
import traceback
from collections.abc import Callable
from pathlib import Path
from typing import Any, Optional

import numpy as np
from numpy.typing import NDArray

from argus_synchro.calibration_mat_generator_modules.ctrl.calibration2d3d.calc_accuracy import (
    calc_accuracy_class,
    calib2d3d_matchecker,
)
from argus_synchro.calibration_mat_generator_modules.ctrl.calibration2d3d.calc_progress import (
    calc_progress_class,
    dtype_integrated_progress_info,
)
from argus_synchro.calibration_mat_generator_modules.ctrl.calibration2d3d.correspondence import (
    correspondence_class_loader,
)
from argus_synchro.calibration_mat_generator_modules.ctrl.calibration2d3d.track_main import (
    track_main_class,
)
from argus_synchro.calibration_mat_generator_modules.ctrl.calibration2d3d.track_main.detect2D import (
    detect2d_axis_faster,
)

# 型定義でのみ使用
from argus_synchro.calibration_mat_generator_modules.ctrl.data_capture import (
    data_capture,
)

# メイン処理
from argus_synchro.calibration_mat_generator_modules.ctrl.data_capture.datacapture_local import (
    datacapture_class,
)
from argus_synchro.calibration_mat_generator_modules.facade import (
    CalibrationCommonStatus,
    CalibrationUIGodot,
)
from argus_synchro.calibration_mat_generator_modules.utils.debugdata_store import (
    debug_config,
    debug_store,
)
from argus_synchro.calibration_mat_generator_modules.utils.NumpyMatrixLUT import (
    NumpyMatrixLUT,
)
from argus_synchro.common.app_logger import AppLogger, AppLoggerFactory
from argus_synchro.config.app_config_calibration import AppConfigCalibration
from argus_synchro.diagnosis.calib2d3d_result_diagnosis import (
    Calib2d3dDiagnosisSession,
    Calib2d3dErrorCommon,
    Calib2d3dFinalDiagnosis,
    Calib2d3dFinalObservation,
    CameraCalibrationStatus,
)
from argus_synchro.diagnosis.calib2d3d_runtime_diagnosis import (
    Calib2d3dFrameObservation,
    Calib2d3dRuntimeDiagnosis,
    Calib2d3dRuntimeDiagnosisConfig,
)
from argus_synchro.diagnosis.error_diagnosis import ResultDiagnosis
from argus_synchro.message.calib_fifo_message import FIFOData
from argus_synchro.shared_app_config import SharedAppConfig
from argus_synchro.shared_errors import SharedErrors, StateErrorDIndex
from argus_synchro.shared_excepts import SharedExcepts

MAX_MATRIX_INVALID_CALCULATION_ATTEMPTS = 3

_extract_ordered_data_logger: AppLogger = AppLoggerFactory.from_name(
    "_extract_ordered_data"
)
_logger: AppLogger = AppLoggerFactory.from_name("calibration2d3d_class")

MIN_FINAL_3DPOINTS_SIZE = 100
PROGRESS_RECALC_THRESHOLD = 0.5


def log_register(app_logger_factory: AppLoggerFactory) -> None:
    app_logger_factory.append_logger(_logger)
    app_logger_factory.append_logger(_extract_ordered_data_logger)
    detect2d_axis_faster.log_register(app_logger_factory)


class calibration2d3d_class:
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

    def _write_result_matrix(self, resultmat_path: str, transmat: Any) -> None:
        try:
            Path(resultmat_path).parent.mkdir(parents=True, exist_ok=True)
            np.savetxt(resultmat_path, transmat, delimiter=",")
        except OSError as error:
            self._report_file_io_error(
                resultmat_path,
                "write 2D-3D calibration result matrix CSV",
                error,
            )
            raise

    def __init__(
        self,
        app_config_calib: AppConfigCalibration,
        sac: SharedAppConfig,
        app_logger_factory: AppLoggerFactory,
        shared_errors: SharedErrors,
    ) -> None:
        _logger: AppLogger = app_logger_factory.register_from_type(self.__class__)
        self.sac = sac
        self.app_config_calib = app_config_calib
        camerasel = sac.read().CalibMode.cameraID

        # SIGUSR1を受け取ったらスタックトレースを出力
        faulthandler.register(
            signal.SIGUSR1
        )  # 指定したシグナルでスタックトレースを出力。
        faulthandler.enable()  # 致命的なエラー時に自動でスタックトレースを出力。

        self.app_config_calib: AppConfigCalibration = app_config_calib
        self._ser: SharedErrors = shared_errors
        self._report_file_io_error: Callable[[str, str, Exception], None] = (
            self._report_file_io_error_impl
        )
        self._diagnosis_session = Calib2d3dDiagnosisSession(camerasel)
        self._runtime_diagnosis = Calib2d3dRuntimeDiagnosis(
            self._make_runtime_diagnosis_config(camerasel)
        )
        self._final_diagnosis = Calib2d3dFinalDiagnosis()

        self.read_settingfile()

        self.verbose = not self.app_config_calib.default.print_disabled
        self.progress_score: float = (
            app_config_calib.calib2d3d.CalcProgress.progress_threshold
        )
        self.verbose: bool = not app_config_calib.default.print_disabled

        # 各クラスコンストラクタ呼び出し
        self.proccap = datacapture_class(
            app_config_calib=self.app_config_calib,
            sac=self.sac,
            app_logger_factory=app_logger_factory,
            shared_errors=self._ser,
        )
        self.track_main = track_main_class(
            sac=sac,
            app_config_calib=app_config_calib,
            Mc=self.proccap.get_cameramatrix(),
            camera_index=camerasel,
            app_logger_factory=app_logger_factory,
            file_io_error_reporter=self._report_file_io_error,
            shared_errors=self._ser,
        )
        self.proccorr = correspondence_class_loader(
            app_config_calib=app_config_calib,
            cameramatrix=self.proccap.get_cameramatrix(),
            camera_index=camerasel,
            app_logger_factory=app_logger_factory,
            file_io_error_reporter=self._report_file_io_error,
        )
        self.procprog = calc_progress_class(
            calib2d3d_CalcProgress=app_config_calib.calib2d3d.CalcProgress,
            camerasel=camerasel,
            verbose=self.verbose,
            app_logger_factory=app_logger_factory,
            file_io_error_reporter=self._report_file_io_error,
        )
        self.prog_settings = self.procprog.read_settings(camerasel)
        self.procaccr = calc_accuracy_class()

        self.procaccr.set_cameramatrix(self.proccap.get_cameramatrix())

        self.facade_index_offset: int | None = (
            None  # dataproc関数やfacadeなどで使用中。できれば正しいフレームインデックスにしたいが設計を見直す必要あり
        )
        self.monitor_data = {}
        self.finalized_fileend_autoexit = False
        self.current_progress_score = 0.0
        self.progress_mem = 0.0
        self.datasource_endflag = False
        self._matrix_invalid_calculation_attempts = 0

        self.lastts2d: int | None = None
        self.lastts3d: int | None = None
        debug_config(
            base_dir="debug_out",
            flush_interval_sec=0.5,
            snapshot_interval_sec=5.0,
            snapshot_on_close=True,
            fsync_snapshot=False,
            fsync_flush=False,
        )

        self.use_centerpoint_x_min = (
            self.app_config_calib.calib2d3d.CalcCorrespondence.use_centerpoint_x_min[
                camerasel
            ]
        )
        self.use_centerpoint_x_max = (
            self.app_config_calib.calib2d3d.CalcCorrespondence.use_centerpoint_x_max[
                camerasel
            ]
        )
        self.use_centerpoint_y_min = (
            self.app_config_calib.calib2d3d.CalcCorrespondence.use_centerpoint_y_min[
                camerasel
            ]
        )
        self.use_centerpoint_y_max = (
            self.app_config_calib.calib2d3d.CalcCorrespondence.use_centerpoint_y_max[
                camerasel
            ]
        )

        self.corner_rangefilter_x_min = (
            self.app_config_calib.calib2d3d.CalcCorrespondence.corner_rangefilter_x_min[
                camerasel
            ]
        )
        self.corner_rangefilter_x_max = (
            self.app_config_calib.calib2d3d.CalcCorrespondence.corner_rangefilter_x_max[
                camerasel
            ]
        )
        self.corner_rangefilter_y_min = (
            self.app_config_calib.calib2d3d.CalcCorrespondence.corner_rangefilter_y_min[
                camerasel
            ]
        )
        self.corner_rangefilter_y_max = (
            self.app_config_calib.calib2d3d.CalcCorrespondence.corner_rangefilter_y_max[
                camerasel
            ]
        )
        center3d_zratio_config = self.app_config_calib.calib2d3d.CalcCorrespondence
        self.center3d_zratio_LUT: NumpyMatrixLUT | None = None
        if center3d_zratio_config.center3d_zratio_path:
            try:
                self.center3d_zratio_LUT = NumpyMatrixLUT(
                    A_X=center3d_zratio_config.center3d_zratio_coord_A_X[camerasel],
                    B_X=center3d_zratio_config.center3d_zratio_coord_B_X[camerasel],
                    A_Y=center3d_zratio_config.center3d_zratio_coord_A_Y[camerasel],
                    B_Y=center3d_zratio_config.center3d_zratio_coord_B_Y[camerasel],
                    ARRAY_PATH=center3d_zratio_config.center3d_zratio_path[camerasel],
                    DEFAULT_VALUE=center3d_zratio_config.center3d_zratio_val_DEFAULT[
                        camerasel
                    ],
                )
            except FileNotFoundError as error:
                _logger.warning(
                    "Center-3D Z-ratio LUT unavailable; using legacy fallback: %r",
                    error,
                )
        self.center3d_apply_zratio_x_min = self.app_config_calib.calib2d3d.CalcCorrespondence.bbox_center3d_z_ratio_area_xmin[
            camerasel
        ]
        self.center3d_apply_zratio_x_max = self.app_config_calib.calib2d3d.CalcCorrespondence.bbox_center3d_z_ratio_area_xmax[
            camerasel
        ]
        self.center3d_apply_zratio_y_min = self.app_config_calib.calib2d3d.CalcCorrespondence.bbox_center3d_z_ratio_area_ymin[
            camerasel
        ]
        self.center3d_apply_zratio_y_max = self.app_config_calib.calib2d3d.CalcCorrespondence.bbox_center3d_z_ratio_area_ymax[
            camerasel
        ]

    def __delattr__(self, name: str) -> None:
        self._close()

    def _close(self) -> None:
        pass

    @staticmethod
    def save_pickle(fpath: str, data: Any) -> None:
        with open(fpath, "wb") as wbf:
            pickle.dump(data, wbf)

    def _save_pickle_with_diagnosis(self, fpath: str, data: Any) -> None:
        try:
            self.save_pickle(fpath, data)
        except OSError as error:
            self._report_file_io_error(
                fpath,
                "write 2D-3D sensor data pickle",
                error,
            )
            raise

    def fire_and_forget_writedata(self, fpath: str, data: Any) -> None:
        loop = asyncio.get_event_loop()
        loop.run_in_executor(None, self._save_pickle_with_diagnosis, fpath, data)
        try:
            if self.verbose:
                _logger.info(f"saved: {str(data)[:30]}")
            else:
                pass
        except Exception as e:
            _logger.info(f"{e} in fire_and_forget_writedata")

    def pre_app_loopmain(
        self,
        monitor: CalibrationUIGodot,
        sec: SharedExcepts,
        sac: SharedAppConfig,
        resultmat_path: str,
    ):
        _logger.info(f"entering app_loopmain, arg: {resultmat_path = }")

        self.loopcount = 0
        self.current_progress_score = 0.0
        self.debug_processingtime_record: list[float] = []
        self.starttime: datetime.datetime = datetime.datetime.now()
        self.datasource_endflag = False
        # カメラごとの校正開始時に、行列不正時の再計算回数も初期化する。
        self._matrix_invalid_calculation_attempts = 0

        # CalibStatus:C1/C2 もう一度送信
        monitor.set_status_calibcommon(CalibrationCommonStatus.RUNNING)
        monitor.set_dummydata(
            enable_systemerrorflag=True,
            enable_errorflag=True,
            overwrite_calibresult=True,
            enable_yawangle=True,
        )
        monitor.transmit_setdata(sec, None, is_firstframe=True, mmap_erase_rest=True)

        self.final_2dpoints = None
        self.final_3dpoints = None
        self.finalized_fileend_autoexit = False

        self.camera_id: int = sac.read().CalibMode.cameraID
        self._diagnosis_session.reset(self.camera_id)
        self._runtime_diagnosis = Calib2d3dRuntimeDiagnosis(
            self._make_runtime_diagnosis_config(self.camera_id)
        )

        _logger.info("clear all data in queues")
        self.progress_mem: float = 0.0

    def finalize_fileend_autoexit(
        self,
        monitor: CalibrationUIGodot,
        sec: SharedExcepts,
        sac: SharedAppConfig,
        resultmat_path: str,
    ) -> bool:
        del sac
        if self.finalized_fileend_autoexit:
            return True

        self.datasource_endflag = True
        self.finalized_fileend_autoexit = True
        missing = []
        if self.lastts2d is None:
            missing.append("lastts2d")
        if self.lastts3d is None:
            missing.append("lastts3d")
        if self.facade_index_offset is None:
            missing.append("facade_index_offset")
        if self.final_2dpoints is None or self.final_2dpoints.size == 0:
            missing.append("final_2dpoints")
        if (
            self.final_3dpoints is None
            or self.final_3dpoints.size <= MIN_FINAL_3DPOINTS_SIZE
        ):
            missing.append("final_3dpoints")
        if missing:
            _logger.error(
                "Data source end, but calibration points are insufficient: "
                f"missing={missing}, {self.lastts2d=}, {self.lastts3d=}, "
                f"{self.current_progress_score=}, {self.progress_score=}",
            )
            monitor.set_errorcode_unexpected_exception(True)
            return True

        ref_t = self.lastts2d - self.facade_index_offset
        _logger.info(
            "Data source end: running calibration calculation before auto exit. "
            f"{self.lastts2d=}, {self.lastts3d=}, "
            f"{self.current_progress_score=}, {self.progress_score=}",
        )
        monitor.set_calibration_ready(1)
        monitor.set_status_calibcommon(
            CalibrationCommonStatus.CALCULATING
        )  # CalibStatus:C4
        monitor.set_dummydata(
            enable_systemerrorflag=True,
            enable_errorflag=True,
            overwrite_calibresult=True,
            enable_yawangle=True,
        )
        monitor.transmit_setdata(sec=sec, ref_t=ref_t)

        transmat, accvalue = self.get_calibval(
            recalc_bbox_index=(self.progress_mem < PROGRESS_RECALC_THRESHOLD),
            frame_ix=self.lastts3d,
        )
        debug_store("transmat", transmat, self.lastts2d)
        monitor.set_dummydata(
            enable_systemerrorflag=True,
            enable_errorflag=True,
            overwrite_calibresult=True,
            enable_yawangle=True,
        )
        monitor.transmit_setdata(sec=sec, ref_t=ref_t)
        _logger.info(f"get_calibval: {transmat} accvalue: {accvalue}")
        reference_matrix_difference_invalid = (
            self._log_calibration_matrix_difference(transmat, accvalue)
        )

        camera_status = self._diagnose_final_calibration(
            transmat=transmat,
            accvalue=accvalue,
            reference_matrix_difference_invalid=(
                reference_matrix_difference_invalid
            ),
            monitor=monitor,
        )
        if camera_status is CameraCalibrationStatus.CALIBRATION_SUCCEEDED:
            if not self.app_config_calib.calib2d3d.CalcAccuracy.check_enable:
                _logger.info("** [CalcAccuracy]check_enable=False **")
            self._write_result_matrix(resultmat_path, transmat)
            _logger.info(
                f"Accuracy Check OK ({accvalue}), result written: {resultmat_path} end"
            )
        else:
            _logger.error(
                "Calibration result diagnosis failed: status=%s, accvalue=%s",
                int(camera_status),
                accvalue,
            )
            monitor.set_errorcode_unexpected_exception(True)
        self._update_errors_calibcommon(monitor)
        return True

    def _next_lidar_frame_is_missing(self, framecounter: int) -> bool:
        """ファイル入力の次フレームが揃っているか確認する。"""
        if not self.app_config_calib.default.File_Input:
            return False

        lidar_conf = self.app_config_calib.dataCapture.Lidar
        lidar_files = (
            lidar_conf.lidar_files_for_cam0calib,
            lidar_conf.lidar_files_for_cam1calib,
            lidar_conf.lidar_files_for_cam2calib,
        )[self.camera_id]
        next_frame = framecounter + 1
        missing_files = [
            f"{lidar_file}{next_frame:06d}.npy"
            for lidar_file in lidar_files
            if not Path(f"{lidar_file}{next_frame:06d}.npy").is_file()
        ]
        if missing_files:
            _logger.warning(
                "Next LiDAR frame is missing; treating it as data-source end: "
                f"frame={next_frame}, files={missing_files}"
            )
            return True
        return False

    def _log_calibration_matrix_difference(
        self, transmat: NDArray[np.float64], accvalue: float
    ) -> bool | None:
        """既存行列との差をログ出力し、閾値超過の有無を返す。

        参照値を読み出せない場合は、推測で不合格にせず ``None`` を返す。
        """
        try:
            with open(
                self.app_config_calib.calib2d3d.CalcCorrespondence.optparam_initialvector,
                encoding="utf-8",
            ) as reference_file:
                reference_parameters = json.load(reference_file)[self.camera_id]

            reference_rvec = np.asarray(reference_parameters["initial_rvec_rad"])
            reference_tvec = np.asarray(reference_parameters["initial_tvec"])
            reference_rvec, reference_tvec = calib2d3d_matchecker.conv_4x4mat_to_rtvec(
                calib2d3d_matchecker.conv_rtvec_to_mat(reference_rvec, reference_tvec)
            )
            result_rvec, result_tvec = calib2d3d_matchecker.conv_4x4mat_to_rtvec(
                transmat, x_inverted=True
            )
            rvec_ok, tvec_ok, rvec_diff_deg, tvec_diff = (
                calib2d3d_matchecker.check_diff(
                    reference_rvec,
                    reference_tvec,
                    result_rvec,
                    result_tvec,
                    rvec_threshold_deg=10.0,
                    tvec_threshold=0.5,
                    camerapos_mode=True,
                )
            )
            _logger.info(
                "Calibration matrix reference check: "
                f"{rvec_ok=}, {tvec_ok=}, {rvec_diff_deg=}, "
                f"{tvec_diff=}, {accvalue=}"
            )
            return not (bool(rvec_ok) and bool(tvec_ok))
        except (OSError, IndexError, KeyError, TypeError, ValueError) as error:
            _logger.warning("Calibration matrix reference check skipped: %r", error)
            return None

    def app_loopmain(
        self,
        fifo_data: FIFOData,
        monitor: CalibrationUIGodot,
        sec: SharedExcepts,
        sac: SharedAppConfig,
        resultmat_path: str,
    ) -> bool:
        # 今までWhile文の実行していた処理のみに変更した。
        # boolで継続可否を返す(Trueなら継続)
        # TODO: 下記構造検討　他のメソッドを下に追いやるか下記をどこかに格納するか？

        if self.app_config_calib.dataCapture.save_sensordata:
            os.makedirs(
                self.app_config_calib.dataCapture.save_sensordata_dir,
                exist_ok=True,
            )
            self.fire_and_forget_writedata(
                os.path.join(
                    self.app_config_calib.dataCapture.save_sensordata_dir,
                    f"sensordata_newdata_time_{self.lastts2d}.pickle",
                ),
                (
                    f"newdata_time,ix:{self.lastts2d}",
                    datetime.datetime.now(),
                ),
            )

        # CalibStatus:C2/C3

        if (
            self.current_progress_score <= self.progress_score
            and sac.read().CalibMode.start2D3DCalibCalc
        ):
            _logger.warning(
                f"progress_score {self.current_progress_score} <= {self.progress_score} but 'start2D3DCalibCalc is True. Ignored.",
            )

        monitor.set_yaw(fifo_data[2][0])
        monitor.set_dummydata(
            enable_systemerrorflag=True,
            enable_errorflag=True,
            enable_yawangle=True,
        )

        debug_allow_calibcalc_flag = False
        if self.dataproc(fifo_data, monitor, cameraID=self.camera_id) is False:
            _logger.warning("Data source end")
            self.datasource_endflag = True
            if self.app_config_calib.debug.calib2d3d_fileend_autoexit:
                debug_allow_calibcalc_flag = True
        elif (
            self.app_config_calib.debug.calib2d3d_fileend_autoexit
            and self._next_lidar_frame_is_missing(fifo_data[3])
        ):
            self.datasource_endflag = True
            debug_allow_calibcalc_flag = True

        last_persondetect_result = None  # self.get_last_singleyoloBB()
        # 2D bbox描画周りで一旦保留 画像上のbboxが出るか否かだけ変化。
        if last_persondetect_result is not None:
            # last_persondetect_result: np.array([float(bbox_xmin), float(bbox_xmax), float(bbox_ymin), float(bbox_ymax), float(cls_id), float(prob)])
            monitor.set_2Dbbox(
                self.camera_id,
                np.array(
                    [
                        last_persondetect_result[0],
                        last_persondetect_result[2],
                        last_persondetect_result[1],
                        last_persondetect_result[3],
                    ],
                    dtype=np.int32,
                ),
            )

            if self.app_config_calib.dataCapture.save_sensordata:
                self.fire_and_forget_writedata(
                    os.path.join(
                        self.app_config_calib.dataCapture.save_sensordata_dir,
                        f"sensordata_2dbbox_{self.lastts2d}.pickle",
                    ),
                    (
                        f"2d bbox, ix:{self.lastts2d}",
                        self.camera_id,
                        np.array(
                            [
                                last_persondetect_result[0],
                                last_persondetect_result[2],
                                last_persondetect_result[1],
                                last_persondetect_result[3],
                            ],
                            dtype=np.int32,
                        ),
                    ),
                )
        else:
            monitor.set_2Dbbox(self.camera_id, np.zeros((0, 4), dtype=np.int32))

        if self.loopcount < 10 and self.datasource_endflag is False:
            self.loopcount += 1
            monitor.set_dummydata(
                enable_systemerrorflag=True,
                enable_errorflag=True,
                overwrite_checkresult=True,
                enable_yawangle=True,
            )
            if self.facade_index_offset is None:
                if self.lastts2d is not None and self.lastts2d < 0:
                    self.facade_index_offset = self.lastts2d
                else:
                    self.facade_index_offset = 0
            monitor.transmit_setdata(
                sec=sec, ref_t=self.lastts2d - self.facade_index_offset
            )
            return True
        self.loopcount = 0

        if (
            self.lastts3d is None and self.datasource_endflag is False
        ):  # 最新点群タイムスタンプ読み込みがNoneはあり得ないが念の為
            return True

        corner2d, corner3d = self.get_corrpoints_fast(
            recalc_bbox_index=(self.progress_mem < PROGRESS_RECALC_THRESHOLD),
            frame_ix=self.lastts3d,
        )

        if self.app_config_calib.dataCapture.save_sensordata:
            self.fire_and_forget_writedata(
                os.path.join(
                    self.app_config_calib.dataCapture.save_sensordata_dir,
                    f"sensordata_cornerpoints_{self.lastts2d}.pickle",
                ),
                (f"cornerpoints, ix:{self.lastts2d}", corner2d, corner3d),
            )

        if self.verbose:
            if corner2d is not None:
                _logger.info("corner2d:{corner2d.shape}")
                if len(corner2d) > 0:
                    _logger.info(
                        f"({corner2d.min(axis=0)} - {corner2d.max(axis=0)})",
                    )
            else:
                _logger.info("corner2d: None")

        if corner3d is not None:
            if self.verbose:
                _logger.info(f"corner3d:{corner3d.shape}")
                if len(corner3d) > 0:
                    _logger.info(
                        f"corner3d diff: ({corner3d.min(axis=0)} - {corner3d.max(axis=0)})",
                    )

            if len(corner3d.shape) == 2 and len(corner3d) > 1:
                corner3d_view = copy.deepcopy(corner3d)
                corner3d_view[:, 1] = -corner3d_view[:, 1]
                corner3d_view[:, 2] = 0

                monitor.set_cornerpoints(
                    corner3d_view,
                    np.tile([0.2, 0.2, 0.2], (corner3d_view.shape[0], 1)),
                )
                monitor.put_data("app_loopmain", "corner2d_temp", corner2d)
                monitor.put_data("app_loopmain", "corner3d_temp", corner3d)
        else:
            _logger.info("corner3d: None")

        # monitor.put_data("app_loopmain", "corner3d_temp", corner3d)

        if corner2d is not None and corner3d is not None:
            if self.verbose:
                _logger.info(f"get_corrpoints_fast:{corner2d.shape},{corner3d.shape}")
            self.final_2dpoints = corner2d
            self.final_3dpoints = corner3d
            debug_store("corner2d_fast", corner2d, self.lastts2d)
            debug_store("corner3d_fast", corner3d, self.lastts2d)
        elif self.verbose:
            _logger.info(
                f"get_corrpoints_fast, corner2d, corner3d:{type(corner2d)} {type(corner3d)} "
                f"final_2dpoints={self.final_2dpoints.shape if isinstance(self.final_2dpoints, np.ndarray) else type(self.final_2dpoints)} "
                f"final_3dpoints={self.final_3dpoints.shape if isinstance(self.final_3dpoints, np.ndarray) else type(self.final_3dpoints)}"
            )
        if (self.final_3dpoints is not None) and (
            self.final_3dpoints.size > 100
        ):  # TODO:　仮の値 ini閾値等に変える
            (
                self.current_progress_score,
                progress_block_detail,
                progress_total_detail,
            ) = self.get_progress(corner3d=self.final_3dpoints)
            self.progress_mem = self.current_progress_score
            _logger.info(f"get_progress:{self.current_progress_score}")
            monitor.put_data("progress", "block_detail", progress_block_detail)

            blockprogress_status: list[int] = []
            subblockprogress_status: list[list[int]] = []

            for index, _1, _2, block_detail in progress_block_detail:
                bscore = block_detail[0]
                sub_detail: (
                    list[tuple[float, float, float, float, int, int, bool, float]]
                    | None
                ) = block_detail[1].get("debug_subgrid_satisfied_coordinates")

                blockprogress_status.append(1 if bscore >= 1 else 0)
                subblockprogress_status.append([])
                if sub_detail is not None:
                    for (
                        xmin,
                        ymin,
                        xmax,
                        ymax,
                        ix,
                        iy,
                        isSatisfied,
                        pointcount,
                    ) in sub_detail:
                        subblockprogress_status[-1].append(1 if isSatisfied else 0)
                else:
                    for _ in range(
                        9
                    ):  # TODO: マジックナンバー！！　サブブロックの数は必ず9なのか？検討が必要
                        subblockprogress_status[-1].append(0)

            # デバッグ用チェック
            if self.verbose:
                if len(blockprogress_status) != len(
                    self.prog_settings[0]
                ):  # setting_params_list
                    _logger.error(
                        f"{len(blockprogress_status) = } != {len(self.prog_settings[0]) = }",
                    )
                if (
                    len(subblockprogress_status) != len(self.prog_settings[0])
                ):  # TODO: マジックナンバー！！　サブブロックの数は必ず9なのか？検討が必要 （現在設定値で間接的に変化するはず）
                    _logger.error(
                        f"{len(subblockprogress_status) = } != {len(self.prog_settings[0]) = }",
                    )

            if self.app_config_calib.dataCapture.save_sensordata:
                self.fire_and_forget_writedata(
                    os.path.join(
                        self.app_config_calib.dataCapture.save_sensordata_dir,
                        f"sensordata_blockprogress_status_{self.lastts2d}.pickle",
                    ),
                    (
                        f"blockprogress_status,ix:{self.lastts2d}",
                        blockprogress_status,
                        subblockprogress_status,
                    ),
                )

            # validIDs = np.sort( [x["bid"] for x in self.prog_settings[0] if (x is not None and x["bid"] >= 0)] )
            # validIDgroups = [x["gid"] for x in self.prog_settings[0] if (x in validIDs)]
            # _logger.info(alidIDgroups = }")

            ixforgui_ordered_blockprogress_status = self._extract_ordered_data(
                self.prog_settings[0],
                np.array(blockprogress_status, dtype=np.float32),
            )

            if self.verbose:
                _logger.info(f"{subblockprogress_status = }")
            ixforgui_blkordered_subblockprogress_status = self._extract_ordered_data(
                self.prog_settings[0],
                np.array(subblockprogress_status, dtype=np.float32),
            )

            gui_blk_listeddict = sorted(
                [x for x in self.prog_settings[0] if x["gid"] >= 0],
                key=lambda x: x["gui_blkid"],
            )

            is_group_overwrite: bool = (
                self.app_config_calib.calib2d3d.CalcProgress.subblock_overwrite_src.lower()
                == "group"
            )
            is_block_overwrite: bool = (
                self.app_config_calib.calib2d3d.CalcProgress.subblock_overwrite_src.lower()
                == "block"
            )

            if is_group_overwrite:
                for gui_blkid in range(8):
                    if (
                        progress_total_detail["progress_idmap"][
                            gui_blk_listeddict[gui_blkid]["gid"]
                        ]
                        >= 1
                    ):
                        ixforgui_ordered_blockprogress_status[gui_blkid] = 1.0
            if is_block_overwrite or is_group_overwrite:
                for gui_blkid in range(8):
                    if ixforgui_ordered_blockprogress_status[gui_blkid] >= 1:
                        for sb_ix in range(9):
                            ixforgui_blkordered_subblockprogress_status[gui_blkid][
                                sb_ix
                            ] = 1

            subblock_progress_reordered = []
            for ixblk, sbres in enumerate(ixforgui_blkordered_subblockprogress_status):
                for gui_sb_ix, sb_ix in enumerate(
                    self.prog_settings[6]
                ):  # [6]: sb_gui_to_sbindex
                    # _logger.info(xblk = }, {sb_ix = } -> {gui_sb_ix}")
                    subblock_progress_reordered.append(sbres[sb_ix])

            monitor.set_blockprogress_status(
                blockprogress_status=ixforgui_ordered_blockprogress_status
            )
            monitor.set_subblockprogress_status(
                subblockprogress_status=np.array(
                    subblock_progress_reordered, dtype=np.float32
                )
            )

            monitor.put_data("progress", "total_detail", progress_total_detail)
            # monitor.put_data("app_loopmain", "progress_val", progress_score)
            monitor.set_progress(self.current_progress_score)

            endflag = False  # 下のwhileを超えて上のwhileを抜ける（2D3D校正を終了する）ためのフラグ
            datawait_loop_enable = True
            while (
                datawait_loop_enable
                and not sec.CalMatGen_ex.IsFinished.value
                and sac.read().CalibMode.isRunning2D3Dcalib
            ):  # do-while的使用方法　self.datasource_endflag=Falseなら１回で抜ける
                datawait_loop_enable = False
                if (
                    self.current_progress_score > self.progress_score
                    or debug_allow_calibcalc_flag
                ):
                    monitor.set_calibration_ready(1)  # CalibStatus:C2->C3 フラグ変化
                    monitor.set_dummydata(
                        enable_systemerrorflag=True,
                        enable_errorflag=True,
                        overwrite_calibresult=True,
                        enable_yawangle=True,
                    )
                    monitor.transmit_setdata(
                        sec, self.lastts2d - self.facade_index_offset
                    )

                    if debug_allow_calibcalc_flag:
                        endflag = self.finalize_fileend_autoexit(
                            monitor=monitor,
                            sec=sec,
                            sac=sac,
                            resultmat_path=resultmat_path,
                        )
                        break

                    if (
                        sac.read().CalibMode.start2D3DCalibCalc
                    ):  # TODO:　仮の値 ini閾値等に変える
                        monitor.set_status_calibcommon(
                            CalibrationCommonStatus.CALCULATING
                        )  # CalibStatus:C4
                        monitor.set_dummydata(
                            enable_systemerrorflag=True,
                            enable_errorflag=True,
                            overwrite_calibresult=True,
                            enable_yawangle=True,
                        )
                        monitor.transmit_setdata(
                            sec=sec, ref_t=self.lastts2d - self.facade_index_offset
                        )

                        transmat, accvalue = self.get_calibval(
                            recalc_bbox_index=(
                                self.progress_mem < PROGRESS_RECALC_THRESHOLD
                            ),
                            frame_ix=self.lastts3d,
                        )
                        debug_store("transmat", transmat, self.lastts2d)
                        # for key, value in self.monitor_data.items():
                        #    monitor.put_data("app_loopmain", key, value)

                        # monitor.set_cornerpoints(
                        #    self.final_3dpoints,
                        #    np.ones_like(self.final_3dpoints) * 0.5,
                        # )
                        monitor.set_dummydata(
                            enable_systemerrorflag=True,
                            enable_errorflag=True,
                            overwrite_calibresult=True,
                            enable_yawangle=True,
                        )
                        monitor.transmit_setdata(
                            sec, self.lastts2d - self.facade_index_offset
                        )
                        _logger.info(
                            f"get_calibval: {transmat} accvalue: {accvalue}",
                        )

                        reference_matrix_difference_invalid = (
                            self._log_calibration_matrix_difference(
                                transmat, accvalue
                            )
                        )

                        camera_status = self._diagnose_final_calibration(
                            transmat=transmat,
                            accvalue=accvalue,
                            reference_matrix_difference_invalid=(
                                reference_matrix_difference_invalid
                            ),
                            monitor=monitor,
                        )
                        if (
                            camera_status
                            is CameraCalibrationStatus.CALIBRATION_SUCCEEDED
                        ):
                            if not self.app_config_calib.calib2d3d.CalcAccuracy.check_enable:
                                _logger.info(
                                    "** [CalcAccuracy]check_enable=False **",
                                )
                            self._write_result_matrix(resultmat_path, transmat)
                            _logger.info(
                                f"Accuracy Check OK ({accvalue}), result written: {resultmat_path} end",
                            )
                            endflag = True

                        else:
                            _logger.error(
                                "calibration result diagnosis failed!! "
                                f"status={int(camera_status)}, "
                                f"{self.current_progress_score = }",
                            )
                            monitor.set_errorcode_unexpected_exception(True)
                            endflag = self._should_finish_after_final_diagnosis(
                                camera_status
                            )
                        break

                    if (
                        len(self.debug_processingtime_record) == 0
                    ):  # 周期タイマー代わりであり何でも良い
                        _logger.info(
                            f"progress_score {self.current_progress_score} > {self.progress_score}, calibration calculation is ready to run",
                        )
                        break

                    if self.datasource_endflag:
                        _logger.info("Data source end: waiting calc request...")
                        time.sleep(1)
                        monitor.set_dummydata(
                            enable_systemerrorflag=True,
                            enable_errorflag=True,
                            overwrite_calibresult=True,
                            enable_yawangle=True,
                        )
                        monitor.transmit_setdata(
                            sec=sec, ref_t=self.lastts2d - self.facade_index_offset
                        )
                        datawait_loop_enable = True
                        continue

                elif self.datasource_endflag:
                    _logger.info(
                        f"Data source end, but {self.current_progress_score = } > {self.progress_score = }",
                    )
                    time.sleep(2)
                    monitor.set_dummydata(
                        enable_systemerrorflag=True,
                        enable_errorflag=True,
                        overwrite_calibresult=True,
                        enable_yawangle=True,
                    )
                    monitor.transmit_setdata(
                        sec=sec, ref_t=self.lastts2d - self.facade_index_offset
                    )
                    datawait_loop_enable = True
                    continue

            if endflag:
                return False

        # monitor.view()
        monitor.set_dummydata(
            enable_systemerrorflag=True,
            enable_errorflag=True,
            overwrite_calibresult=True,
            enable_yawangle=True,
        )
        monitor.transmit_setdata(
            sec=sec, ref_t=self.lastts2d - self.facade_index_offset
        )

        elapsed = (datetime.datetime.now() - self.starttime).total_seconds()
        self.debug_processingtime_record.append(elapsed)
        self.starttime = datetime.datetime.now()

        if len(self.debug_processingtime_record) > 10:
            calcmat: NDArray[np.float32] = np.array(self.debug_processingtime_record)
            _logger.info(
                f"Calibration2D3D process working, average time: {calcmat.mean()}, σ={np.std(calcmat)}",
            )
            self.debug_processingtime_record = []
        return True

    # except Exception as ea:
    #     _logger.error(f"app_loopmain: exception! {ea} - \n{traceback.format_exc()}")
    #     monitor.set_errorcode_unexpected_exception(True)

    # ----  whileループここまで  ----

    def post_app_loopmain(
        self,
        monitor: CalibrationUIGodot,
        sec: SharedExcepts,
        sac: SharedAppConfig,
    ) -> None:
        # CalibStatus:C5
        if self.app_config_calib.debug.calib2d3d_fileend_autoexit:
            return

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
            overwrite_calibresult=True,
            enable_yawangle=True,
        )
        monitor.transmit_setdata(sec=sec, ref_t=None)

        timercount += 1

        if timercount > 10:
            timercount = 0
            _logger.info("========================")
            _logger.info("Calib2d3d end")
            _logger.info("========================")

        time.sleep(0.1)
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
            overwrite_calibresult=True,
            enable_yawangle=True,
        )
        monitor.transmit_setdata(sec=sec, ref_t=None)

    @staticmethod
    def _extract_ordered_data(
        dict_list: list[dict[str, Any]],
        data_array: NDArray[Any],
        output_log: bool = False,
    ) -> NDArray[Any]:
        if output_log:
            _extract_ordered_data_logger.info(f"{dict_list = } , {data_array = }")
        sorted_dicts = sorted(dict_list, key=lambda x: x["gui_blkid"])
        if output_log:
            _extract_ordered_data_logger.info(f"{sorted_dicts = }")
        result: list[Any] = []
        for lp_bid, entry in enumerate(sorted_dicts):
            bid = entry["bid"]
            if output_log:
                _extract_ordered_data_logger.info(f"{entry = } @ {lp_bid = }")
            if bid >= 0:
                result.append(data_array[bid])
        return np.array(result)

    def read_settingfile(self):
        self.accumulate_length = (
            self.app_config_calib.dataConverter2D3D.Lidar.accumulate_length
        )
        assert (
            self.accumulate_length % 2 == 1
        )  # accumulate_lengthは奇数（中心が整数であること）

        self.pcd_indexoffset = (self.accumulate_length - 1) / 2

    def _make_runtime_diagnosis_config(
        self, camera_id: int
    ) -> Calib2d3dRuntimeDiagnosisConfig:
        """アプリ設定を、校正計算から独立した診断設定へ変換する。"""

        config = self.app_config_calib.calib2d3d.Diagnosis
        return Calib2d3dRuntimeDiagnosisConfig(
            # 各フラグはINIへ公開せず、診断単位で独立して切り替えられる。
            enable_walking_range=True,
            enable_person_count=True,
            enable_poor_person_detection=True,
            enable_poor_tracking_3d=True,
            enable_poor_tracking_2d=True,
            enable_unsuitable_condition=True,
            enable_long_duration=True,
            enable_person_detection_impossible=True,
            enable_tracking_impossible=True,
            continuation_seconds=config.continuation_seconds,
            long_duration_seconds=config.long_duration_seconds,
            detection_rate_window_seconds=config.detection_rate_window_seconds,
            detection_rate_threshold=config.detection_rate_threshold,
            expected_person_count=config.expected_person_count,
            brightness_threshold=config.brightness_threshold,
            brightness_sample_stride=config.brightness_sample_stride,
            walking_area_enabled=config.walking_area_enabled,
            # 設定ではcamera0始まりのフラット座標列、ここでは計算用の4点を受け取る。
            walking_area_corners=config.walking_area_corners[camera_id],
        )

    def _update_errors_calibcommon(
        self,
        monitor: CalibrationUIGodot,
        status: Calib2d3dErrorCommon = Calib2d3dErrorCommon.DEFAULT,
    ) -> None:
        """本校正診断をMMAP送信用のUI状態へ反映する。"""
        result = self._diagnosis_session.diagnose_runtime(status)
        monitor.set_errors_calibcommon(int(result.common_error))
        if result.camera_status is not None:
            monitor.set_camera_calibration_status(
                camera_id=result.camera_id,
                value=int(result.camera_status),
            )

    def _diagnose_final_calibration(
        self,
        *,
        transmat: NDArray[np.float64],
        accvalue: float,
        reference_matrix_difference_invalid: bool | None,
        monitor: CalibrationUIGodot,
    ) -> CameraCalibrationStatus:
        """最終計算の観測値を診断へ渡し、カメラ別結果を設定する。"""

        common_error = self._diagnosis_session.current_result.common_error
        decision = self._final_diagnosis.diagnose(
            Calib2d3dFinalObservation(
                matrix=transmat,
                accuracy_value=accvalue,
                accuracy_threshold=(
                    self.app_config_calib.calib2d3d.CalcAccuracy.accvalue
                ),
                accuracy_check_enabled=(
                    self.app_config_calib.calib2d3d.CalcAccuracy.check_enable
                ),
                invalid_person_count=(
                    common_error
                    is Calib2d3dErrorCommon.WALKING_PERSON_COUNT_INVALID
                ),
                poor_tracking_3d=(
                    common_error is Calib2d3dErrorCommon.POOR_TRACKING_3D
                ),
                poor_tracking_2d=(
                    common_error is Calib2d3dErrorCommon.POOR_TRACKING_2D
                ),
                reference_matrix_difference_invalid=(
                    reference_matrix_difference_invalid
                ),
            )
        )
        result = self._diagnosis_session.diagnose_final(
            decision.status,
            details=decision.details,
        )
        assert result.camera_status is not None
        monitor.set_camera_calibration_status(
            camera_id=self.camera_id,
            value=int(result.camera_status),
        )
        return result.camera_status

    def _should_finish_after_final_diagnosis(
        self,
        camera_status: CameraCalibrationStatus,
    ) -> bool:
        """最終診断不合格後に結果画面へ進むかを決める。

        行列不正だけは一時的な計算失敗を考慮して初回を含む最大3回まで
        計算する。それ以外の不合格は再計算しても診断理由が変わらないため、
        現在のカメラ状態を保持したまま直ちに収集ループを終了する。
        """

        if camera_status is not CameraCalibrationStatus.CALIBRATION_MATRIX_INVALID:
            return True

        self._matrix_invalid_calculation_attempts += 1
        if (
            self._matrix_invalid_calculation_attempts
            < MAX_MATRIX_INVALID_CALCULATION_ATTEMPTS
        ):
            _logger.warning(
                "calibration matrix diagnosis failed; retry calculation "
                f"({self._matrix_invalid_calculation_attempts}/"
                f"{MAX_MATRIX_INVALID_CALCULATION_ATTEMPTS})",
            )
            return False

        _logger.error(
            "calibration matrix diagnosis failed after maximum calculation "
            f"attempts ({MAX_MATRIX_INVALID_CALCULATION_ATTEMPTS})",
        )
        return True

    def get_last_singleyoloBB(self) -> list[NDArray[np.float64]] | None:
        return self.track_main.get_last_singleyoloBB()

    def input_data_diagnosis(
        self,
        camera_datalist: list[tuple[NDArray[np.uint8], int, float]],
        lidar_datalist: list[tuple[NDArray[np.float32], int, float]],
        can_data: tuple[float, float],
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

        images = tuple(camera_data[0] for camera_data in camera_datalist)
        min_xyz_columns = 3
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

    @staticmethod
    def _active_tracking_id_count(metadata: dict[int, Any], timestamp: int) -> int:
        """現在フレームまで更新された追跡ID数を数える。"""

        return sum(
            int(getattr(track, "frame_ix_max", -1)) == int(timestamp)
            for track in metadata.values()
        )

    def detection_diagnosis(
        self,
        *,
        image: NDArray[np.uint8],
        monitor: CalibrationUIGodot,
    ) -> None:
        """検出・追跡結果を診断モジュールへ渡し、UI向け値を更新する。"""

        yolo_result = self.track_main.get_last_yoloBB()
        detection_2d_count = 0
        if yolo_result is not None and len(yolo_result) > 1:
            # bbox配列の深い添字ではなく、有効score数から2D検出人数を数える。
            scores = np.asarray(yolo_result[1])
            detection_2d_count = int(np.count_nonzero(scores > 0))

        process3d = self.track_main.get_monitor_data().get("process3d_frame", {})
        bbox_3d = np.asarray(process3d.get("multi_minmax", np.empty((0, 6))))
        if bbox_3d.ndim != 2 or bbox_3d.shape[-1] < 4:
            bbox_3d = np.empty((0, 6))
        # 3D bbox形式は(xmin, xmax, ymin, ymax, zmin, zmax)。歩行範囲にはXY中心を使う。
        bbox_3d_centers_xy = tuple(
            ((float(bbox[0]) + float(bbox[1])) / 2.0,
             (float(bbox[2]) + float(bbox[3])) / 2.0)
            for bbox in bbox_3d
        )

        tracking_2d = self.track_main.detect2d.get_tracking_results()
        tracking_3d = self.track_main.detect3d.get_tracking_results()
        tracking_2d_count = self._active_tracking_id_count(
            tracking_2d.trackingIDmetadata,
            -1 if self.lastts2d is None else self.lastts2d,
        )
        tracking_3d_count = self._active_tracking_id_count(
            tracking_3d.trackingIDmetadata,
            -1 if self.lastts3d is None else self.lastts3d,
        )

        decision = self._runtime_diagnosis.diagnose(
            Calib2d3dFrameObservation(
                image=image,
                detection_2d_count=detection_2d_count,
                bbox_3d_count=len(bbox_3d),
                tracking_2d_id_count=tracking_2d_count,
                tracking_3d_id_count=tracking_3d_count,
                bbox_3d_centers_xy=bbox_3d_centers_xy,
            )
        )
        result = self._diagnosis_session.diagnose_runtime(
            decision.common_error,
            phase=decision.phase,
            details=decision.details,
        )
        monitor.set_errors_calibcommon(int(result.common_error))
        if result.camera_status is not None:
            monitor.set_camera_calibration_status(
                camera_id=result.camera_id,
                value=int(result.camera_status),
            )

    def dataproc(
        self,
        readresult_pop: FIFOData,
        monitor: CalibrationUIGodot,
        cameraID: int,
    ) -> bool:  # 継続可否を返す Falseで終了
        camera_datalist, lidar_datalist, can_data, framecounter = readresult_pop
        if self.input_data_diagnosis(
            camera_datalist,
            lidar_datalist,
            can_data,
        ):
            return False

        readresults = self.proccap.read(readresult_pop)
        if readresults is not None:
            self.lastts2d = readresults[0][1]
            self.lastts3d = readresults[1][1]

        if self.app_config_calib.dataCapture.save_sensordata:
            self.fire_and_forget_writedata(
                os.path.join(
                    self.app_config_calib.dataCapture.save_sensordata_dir,
                    f"sensordata_rawdata_{self.lastts2d}.pickle",
                ),
                (
                    f"dataproc-self.proccap.read rawdata ix:{self.lastts2d}",
                    readresults,
                ),
            )

        if readresults is None:
            return False
        self.track_main.detect(indata=readresults)

        self.detection_diagnosis(image=readresults[0][0], monitor=monitor)

        monitor.set_image(cameraID, self.track_main.monitor_data[f"detect2d_image{0}"])

        # pointdata: NDArray[np.float32] = self.track_main.monitor_data["detect3d_points_raw"]
        pointdata: NDArray[np.float32] | None = None
        process3d_frame_datadict = self.track_main.get_monitor_data().get(
            "process3d_frame"
        )
        if process3d_frame_datadict is not None:
            pointdata = process3d_frame_datadict.get("limited_pts")

        if pointdata is not None and pointdata.shape[0] > 0:
            if pointdata.shape[-1] == 4:
                pts = pointdata[:, :3]
                pts = self.proccap.points_adjust_coord_calib2normal(pts, 0)
                pts[:, 1] = -pts[:, 1]
                pts[:, 2] = -pts[:, 2]

                monitor.set_points(
                    pts, monitor.convert_intensity_to_color(pointdata[:, 3])
                )
            elif pointdata.shape[-1] == 3:
                pts = pointdata
                pts = self.proccap.points_adjust_coord_calib2normal(pts, 0)
                pts[:, 1] = -pts[:, 1]
                pts[:, 2] = -pts[:, 2]

                monitor.set_points(pts, np.tile([0.2, 0.2, 0.2], (pts.shape[0], 1)))

        monitor.set_boxes(
            self.track_main.monitor_data["detect3d_multipoints"],
            self.track_main.monitor_data["detect3d_multi_lines"],
        )

        if self.app_config_calib.dataCapture.save_sensordata:
            self.fire_and_forget_writedata(
                os.path.join(
                    self.app_config_calib.dataCapture.save_sensordata_dir,
                    f"sensordata_process3d_frame_datadict_{self.lastts2d}.pickle",
                ),
                (
                    f"process3d_frame_datadict, ix:{self.lastts2d}",
                    process3d_frame_datadict,
                ),
            )
            self.fire_and_forget_writedata(
                os.path.join(
                    self.app_config_calib.dataCapture.save_sensordata_dir,
                    f"sensordata_monitor_data_{self.lastts2d}.pickle",
                ),
                (
                    f"self.track_main.monitor_data, ix:{self.lastts2d}",
                    self.track_main.monitor_data,
                ),
            )

        return True

    def get_corrpoints_fast(self, frame_ix: int, recalc_bbox_index: bool = True):
        result_index = 0  # TODO: 削除
        suffix = ""
        corner2d_set, corner3d_set = self.track_main.extract_fromcenter(
            recalc_bbox_index=recalc_bbox_index, frame_ix=frame_ix
        )  # 26/1/9時点でpoints_per_time=1固定
        if corner2d_set[0] is None or corner3d_set is None or corner3d_set[0] is None:
            return None, None
        corner2d, corner3d = self.proccorr.preproc(
            corner2d_set=corner2d_set,
            corner3d_set=corner3d_set,
            adjustfunc_coord_calib2normal=self.proccap.points_adjust_coord_calib2normal,
            adjustfunc_coord_normal2argus=self.proccap.points_adjust_coord_normal2argus,
            savename_suffix="p" + str(result_index) + suffix,
            points_per_time=1,
        )
        return corner2d, corner3d

    def get_progress(
        self, corner3d
    ) -> dtype_integrated_progress_info:  # この時点では人の軸を使った調整は不要
        progress_result = self.procprog.calc_progress(
            data3d=corner3d, points_per_time=1
        )
        self.track_main.set_progress(progress=progress_result[0])
        return progress_result

    def _get_result_single(
        self, corner2d_set, corner3d_set, suffix, points_per_time: int
    ):
        corner2d, corner3d = self.proccorr.preproc(
            corner2d_set=corner2d_set,
            corner3d_set=corner3d_set,
            adjustfunc_coord_calib2normal=self.proccap.points_adjust_coord_calib2normal,
            adjustfunc_coord_normal2argus=self.proccap.points_adjust_coord_normal2argus,
            savename_suffix=suffix,
            points_per_time=points_per_time,
        )
        return corner2d, corner3d

    def get_calibval(
        self, frame_ix: int, recalc_bbox_index: bool = True
    ):  # 旧get_result 人の軸使用の2D対応点抽出は仮の校正行列が無ければいけないので通常の対応点抽出→校正→軸使った対応点抽出→校正の順になる
        final_rtvec = None
        savefile_timecode = (
            f"{os.getpid()}_{datetime.datetime.now().strftime('%Y%m%d_%H')}"
        )
        # for result_index in range(self.calc3dcount):
        if True:
            # bbox中心対応点を取得
            corner2d_set_c, corner3d_set_c = self.track_main.extract_fromcenter(
                recalc_bbox_index=recalc_bbox_index, frame_ix=frame_ix
            )
            points_per_time = 1

            debug_store(key="corner2d_set_calib", value=corner2d_set_c)
            debug_store(key="corner3d_set_calib", value=corner3d_set_c)

            # タイムスタンプ同期
            corner2d_center, corner3d_center = self._get_result_single(
                corner2d_set=corner2d_set_c,
                corner3d_set=corner3d_set_c,
                suffix=savefile_timecode,
                points_per_time=points_per_time,
            )

            # 校正行列推定
            self.proccorr.estimate(
                corner2d_center, corner3d_center, centerF_or_axisT=False
            )
            if self.app_config_calib.debug.save_cornerlist_pickle:
                self.proccorr.save(
                    file_savedir=self.app_config_calib.default.outputdir_root,
                    name_suffix=savefile_timecode,
                )

            debug_store(key="corner2d_center", value=corner2d_center)
            debug_store(key="corner3d_center", value=corner3d_center)

            final_rtvec = self.proccorr.get_last_rtvec()
            debug_store(
                key="center_rtvec",
                value=self.proccorr.get_last_rtvec(),
            )

            # 人の軸推定を使った、画像bboxから頭・足対応点推定
            corner2d_set_axis, corner3d_set_axis = self.track_main.extract_withaxis(
                final_rtvec[0],
                final_rtvec[1],
                recalc_bbox_index=recalc_bbox_index,
                frame_ix=frame_ix,
            )
            points_per_time = 2  # 2固定：高精度計算

            debug_store(key="corner2d_set_head_and_foot", value=corner2d_set_axis)
            debug_store(key="corner3d_set_head_and_foot", value=corner3d_set_axis)

            # タイムスタンプ同期
            corner2d_axis, corner3d_axis = self._get_result_single(
                corner2d_set=corner2d_set_axis,
                corner3d_set=corner3d_set_axis,
                suffix=savefile_timecode + "_axis",
                points_per_time=points_per_time,
            )

            debug_store(key="corner2d_axis", value=corner2d_axis)
            debug_store(key="corner3d_axis", value=corner3d_axis)
            debug_store(
                key="axis_rtvec",
                value=self.proccorr.get_last_rtvec(),
            )

            # ここから、最終出力に向けて対応点の調整を行う。（全部bbox中央でも全部頭足でも精度は上がり切らない。混合する）

            # 範囲フィルター
            # 一度全部Trueのフィルターを作っておいて、モードに応じてFalseにする CY-XY相当
            corner_center_filter = corner3d_center[:, 0] == corner3d_center[:, 0]
            corner_axis_filter = corner3d_axis[:, 0] == corner3d_axis[:, 0]

            # モード別の対応
            # [C,X][Y,L,E,N] default:CY-XY
            if (
                self.app_config_calib.calib2d3d.CalcCorrespondence.corrpoint_mode.find(
                    "CN"
                )
                >= 0
            ):  # CN: bbox中心点は使用しないモード
                corner_center_filter = corner3d_center[:, 0] != corner3d_center[:, 0]
            elif (
                self.app_config_calib.calib2d3d.CalcCorrespondence.corrpoint_mode.find(
                    "CL"
                )
                >= 0
            ):  # CL: bbox中心点は指定範囲内を使用するモード
                corner_center_filter = (
                    (corner3d_center[:, 0] >= self.use_centerpoint_x_min)
                    & (corner3d_center[:, 0] <= self.use_centerpoint_x_max)
                    & (corner3d_center[:, 1] >= self.use_centerpoint_y_min)
                    & (corner3d_center[:, 1] <= self.use_centerpoint_y_max)
                )
            elif (
                self.app_config_calib.calib2d3d.CalcCorrespondence.corrpoint_mode.find(
                    "CE"
                )
                >= 0
            ):  # CE: bbox中心点は指定範囲外を使用するモード
                corner_center_filter = ~(
                    (corner3d_center[:, 0] >= self.use_centerpoint_x_min)
                    & (corner3d_center[:, 0] <= self.use_centerpoint_x_max)
                    & (corner3d_center[:, 1] >= self.use_centerpoint_y_min)
                    & (corner3d_center[:, 1] <= self.use_centerpoint_y_max)
                )

            if (
                self.app_config_calib.calib2d3d.CalcCorrespondence.corrpoint_mode.find(
                    "XN"
                )
                >= 0
            ):  # XN: bbox頭足点は使用しないモード
                corner_axis_filter = corner3d_axis[:, 0] != corner3d_axis[:, 0]
            elif (
                self.app_config_calib.calib2d3d.CalcCorrespondence.corrpoint_mode.find(
                    "XL"
                )
                >= 0
            ):  # XL: bbox頭足点は指定範囲内を使用するモード
                corner_axis_filter = (
                    (corner3d_axis[:, 0] >= self.use_centerpoint_x_min)
                    & (corner3d_axis[:, 0] <= self.use_centerpoint_x_max)
                    & (corner3d_axis[:, 1] >= self.use_centerpoint_y_min)
                    & (corner3d_axis[:, 1] <= self.use_centerpoint_y_max)
                )
            elif (
                self.app_config_calib.calib2d3d.CalcCorrespondence.corrpoint_mode.find(
                    "XE"
                )
                >= 0
            ):  # XE: bbox頭足点は指定範囲外を使用するモード
                corner_axis_filter = ~(
                    (corner3d_axis[:, 0] >= self.use_centerpoint_x_min)
                    & (corner3d_axis[:, 0] <= self.use_centerpoint_x_max)
                    & (corner3d_axis[:, 1] >= self.use_centerpoint_y_min)
                    & (corner3d_axis[:, 1] <= self.use_centerpoint_y_max)
                )

            # 3D bbox Z座標高さを補正する
            if self.app_config_calib.calib2d3d.CalcCorrespondence.enable_recalc_center3d_z:
                head_z: np.float64 = corner3d_axis[0, 2]
                foot_z: np.float64 = corner3d_axis[1, 2]

                if self.center3d_zratio_LUT is not None:
                    for i, c in enumerate(corner3d_center):
                        x, y, z = c
                        z_ratio = self.center3d_zratio_LUT.evaluate(x, y)
                        z = (head_z - foot_z) * z_ratio + foot_z
                        corner3d_center[i, 2] = z
                else:
                    center_area_filter = (
                        (corner3d_center[:, 0] >= self.center3d_apply_zratio_x_min)
                        & (corner3d_center[:, 0] <= self.center3d_apply_zratio_x_max)
                        & (corner3d_center[:, 1] >= self.center3d_apply_zratio_y_min)
                        & (corner3d_center[:, 1] <= self.center3d_apply_zratio_y_max)
                    )
                    corner3d_center[center_area_filter, 2] = (
                        (head_z - foot_z)
                        * self.app_config_calib.calib2d3d.CalcCorrespondence.bbox_center3d_z_ratio
                        + foot_z
                    )
                    corner3d_center[~center_area_filter, 2] = (
                        head_z - foot_z
                    ) * 0.5 + foot_z

            corner2d_final = np.concatenate(
                [
                    corner2d_center[corner_center_filter],
                    corner2d_axis[corner_axis_filter],
                ]
            )
            corner3d_final = np.concatenate(
                [
                    corner3d_center[corner_center_filter],
                    corner3d_axis[corner_axis_filter],
                ]
            )

            corner_range_filter = corner3d_final[:, 0] == corner3d_final[:, 0]
            if (
                self.app_config_calib.calib2d3d.CalcCorrespondence.corner_rangefilter_mode.find(
                    "N"
                )
                >= 0
            ):
                corner_range_filter = corner3d_final[:, 0] != corner3d_final[:, 0]
            elif (
                self.app_config_calib.calib2d3d.CalcCorrespondence.corner_rangefilter_mode.find(
                    "L"
                )
                >= 0
            ):
                corner_range_filter = (
                    (corner3d_final[:, 0] >= self.use_centerpoint_x_min)
                    & (corner3d_final[:, 0] <= self.use_centerpoint_x_max)
                    & (corner3d_final[:, 1] >= self.use_centerpoint_y_min)
                    & (corner3d_final[:, 1] <= self.use_centerpoint_y_max)
                )
            elif (
                self.app_config_calib.calib2d3d.CalcCorrespondence.corner_rangefilter_mode.find(
                    "E"
                )
                >= 0
            ):
                corner_range_filter = ~(
                    (corner3d_final[:, 0] >= self.corner_rangefilter_x_min)
                    & (corner3d_final[:, 0] <= self.corner_rangefilter_x_max)
                    & (corner3d_final[:, 1] >= self.corner_rangefilter_y_min)
                    & (corner3d_final[:, 1] <= self.corner_rangefilter_y_max)
                )

            corner2d_final = corner2d_final[corner_range_filter]
            corner3d_final = corner3d_final[corner_range_filter]

            debug_store(key="corner2d_final", value=corner2d_final)
            debug_store(key="corner3d_final", value=corner3d_final)

            debug_store(
                key="final_rtvec_axis",
                value=self.proccorr.get_last_rtvec(),
            )

            self.proccorr.estimate(
                corner2d_final, corner3d_final, centerF_or_axisT=True
            )
        if self.app_config_calib.debug.save_cornerlist_pickle:
            self.proccorr.save(
                file_savedir=self.app_config_calib.default.outputdir_root,
                name_suffix=savefile_timecode + "_axis",
            )

            # 本当は含めたいが上下反転が必要　暫定的にコメントアウト
            # self.monitor_data["corner2d"] = corner2d
            # self.monitor_data["corner3d"] = corner3d

        evalacc_value = self.procaccr.LOOCV_bytime(corner2d_final, corner3d_final)

        # self.track_main.save_debugdata_2dtracker(openflag="ab")

        return self.proccorr.get_rotation_mat(), evalacc_value

    def get_calibval_old(
        self, frame_ix: int, recalc_bbox_index: bool = True
    ):  # 旧get_result 人の軸使用の2D対応点抽出は仮の校正行列が無ければいけないので通常の対応点抽出→校正→軸使った対応点抽出→校正の順になる
        final_rtvec = None
        savefile_timecode = (
            f"{os.getpid()}_{datetime.datetime.now().strftime('%Y%m%d_%H')}"
        )
        # for result_index in range(self.calc3dcount):
        if True:
            corner2d_set_c, corner3d_set_c = self.track_main.extract_fromcenter(
                recalc_bbox_index=recalc_bbox_index, frame_ix=frame_ix
            )
            points_per_time = 1

            debug_store(key="corner2d_set_calib", value=corner2d_set_c)
            debug_store(key="corner3d_set_calib", value=corner3d_set_c)

            self._get_result_single(
                corner2d_set=corner2d_set_c,
                corner3d_set=corner3d_set_c,
                suffix=savefile_timecode,
                points_per_time=points_per_time,
            )

            final_rtvec = self.proccorr.get_last_rtvec()
            corner2d_set_axis, corner3d_set = self.track_main.extract_withaxis(
                final_rtvec[0],
                final_rtvec[1],
                recalc_bbox_index=recalc_bbox_index,
                frame_ix=frame_ix,
            )
            points_per_time = 2  # 2固定：高精度計算

            debug_store(key="final_rtvec", value=final_rtvec)

            debug_store(key="corner2d_set_head_and_foot", value=corner2d_set_axis)
            debug_store(key="corner3d_set_head_and_foot", value=corner3d_set)

            corner2d, corner3d, rtvec = self._get_result_single(
                corner2d_set=corner2d_set_axis,
                corner3d_set=corner3d_set,
                suffix=savefile_timecode + "_axis",
                points_per_time=points_per_time,
            )

            debug_store(key="corner2d_final", value=corner2d)
            debug_store(key="corner3d_final", value=corner3d)
            # 本当は含めたいが上下反転が必要　暫定的にコメントアウト
            # self.monitor_data["corner2d"] = corner2d
            # self.monitor_data["corner3d"] = corner3d

        evalacc_value = self.procaccr.LOOCV_bytime(corner2d, corner3d)

        # self.track_main.save_debugdata_2dtracker(openflag="ab")

        return self.proccorr.get_rotation_mat(), evalacc_value

    def release_resources(self):
        self.proccap.release()
