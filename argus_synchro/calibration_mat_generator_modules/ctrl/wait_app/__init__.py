"""
センサテスト・表示テストモード
UI向けに全フラグの受け渡しテスト機能を提供すると共に、各機能コーディング時のセンサ入力・データ出力のひな型を提供する。
"""

# from open3d.cpu.pybind.geometry import PointCloud
import time

# import open3d as o3d
import traceback  # 例外時のトレースバック取得用

import cv2
import numpy as np
from numpy.typing import NDArray

from argus_synchro.calibration_mat_generator_modules.ctrl.calibcheck2d3d.debuginfo_and_functions import (
    conbine3d3d,
    read_rtvec,
)
from argus_synchro.calibration_mat_generator_modules.ctrl.calibration3d3d import (
    pcd_to_np,
)

# 型定義でのみ使用
from argus_synchro.calibration_mat_generator_modules.ctrl.data_capture import (
    data_capture,
)

# メイン処理
from argus_synchro.calibration_mat_generator_modules.ctrl.data_capture.datacapture_local import (
    datacapture_class,
)
from argus_synchro.calibration_mat_generator_modules.facade import CalibrationUIGodot
from argus_synchro.calibration_mat_generator_modules.utils.utils3d import np_to_pcd

# ARGUSシステム制御関連
from argus_synchro.common.app_logger import AppLogger, AppLoggerFactory
from argus_synchro.config.app_config_calibration import (
    AppConfigCalibration,
    CalibCheck2d3dConf,
    DefaultConf,
)
from argus_synchro.config.fileinput_pathselector import (
    lidar_calib_filepath_loader,
)
from argus_synchro.diagnosis.error_diagnosis import ResultDiagnosis
from argus_synchro.message.calib_fifo_message import FIFOData
from argus_synchro.provider.image import Mcde7000UndistortImageProvider
from argus_synchro.shared_app_config import SharedAppConfig
from argus_synchro.shared_errors import SharedErrors, StateErrorDIndex
from argus_synchro.shared_excepts import SharedExcepts

ALIVE_LOG_INTERVAL_SEC = 5.0


class wait_app:
    def __init__(
        self,
        app_config_calib: AppConfigCalibration,
        sac: SharedAppConfig,
        app_logger_factory: AppLoggerFactory,
        shared_errors: SharedErrors,
    ) -> None:
        self.sac = sac
        self.app_config_calib = app_config_calib
        # 各クラスコンストラクタ呼び出し
        self._logger: AppLogger = app_logger_factory.register_from_type(self.__class__)
        self.calibcheck2d3d_conf: CalibCheck2d3dConf = app_config_calib.calibCheck2d3d
        self.DefaultConfig: DefaultConf = app_config_calib.default
        self._ser: SharedErrors = shared_errors

        self.proccap = datacapture_class(
            app_config_calib=self.app_config_calib,
            sac=self.sac,
            app_logger_factory=app_logger_factory,
            shared_errors=self._ser,
        )
        self.debug_index = 0

        self.ud: Mcde7000UndistortImageProvider = Mcde7000UndistortImageProvider(
            camera_intrinsics_path=self.calibcheck2d3d_conf.camera_intrinsics_path,
            sys_width=self.calibcheck2d3d_conf.image_w,
            sys_height=self.calibcheck2d3d_conf.image_h,
        )

        self.verbose: bool = not app_config_calib.default.print_disabled

    def __delattr__(self, name: str) -> None:
        self._close()

    def _close(self) -> None:
        pass

    def _report_file_io_error(
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

    def input_settings(self) -> None:
        # TODO: 同期入力別プロセスのモジュールに入替

        self.trans_mat3D3D_eachlidar = []
        for path in lidar_calib_filepath_loader(
            sac=self.sac, app_config_calib=self.app_config_calib
        ):
            self._logger.info(f"[input_settings] path:{path}")
            try:
                transform = np.loadtxt(path, delimiter=",")
            except (OSError, UnicodeError, ValueError) as error:
                self._report_file_io_error(
                    path, "read wait_app LiDAR calibration CSV", error
                )
                raise
            self._logger.info(f"loadtxt: {transform}")
            self.trans_mat3D3D_eachlidar.append(transform)

        self.rtvec_mat = []
        for path in self.calibcheck2d3d_conf.camera_calib_files:
            try:
                rtvec = read_rtvec(
                    rvec_convmat_path=path,
                    new_axis_mode=self.calibcheck2d3d_conf.new_axis_mode,
                    points_inverted=True,
                )
            except (
                OSError,
                UnicodeError,
                ValueError,
                RuntimeError,
                EOFError,
            ) as error:
                self._report_file_io_error(
                    path, "read wait_app camera calibration", error
                )
                raise
            self.rtvec_mat.append(rtvec)

    def pre_app_loopmain(self) -> None:
        self.input_settings()
        self._last_alive_log_mono = time.monotonic()
        self._logger.info("app_loopmain start")

    def app_loopmain(
        self,
        readresult_pop: FIFOData,
        monitor: CalibrationUIGodot,
        sec: SharedExcepts,
        sac: SharedAppConfig,
    ) -> None:
        # TODO: 下記構造検討　他のメソッドを下に追いやるか下記をどこかに格納するか？
        now = time.monotonic()
        if now - self._last_alive_log_mono >= ALIVE_LOG_INTERVAL_SEC:
            self._last_alive_log_mono = now
            self._logger.info("A1: No mode selected, waiting (alive)")
        self.dataproc(readresult_pop, monitor, sec)
        time.sleep(0.01)
        # データが最後まで到達したことは、入力系のプロセスが検知。
        # 入力が最後まで到達したときの処理は記載しない。

    # try:
    #     self.input_settings()
    #     while self.is_wait_selected(sec=sec, sac=sac):
    #         self.timercount_for_log += 1

    #         if self.timercount_for_log > 10:
    #             self.timercount_for_log = 0
    #             self._logger.info("========================")
    #             self._logger.info("A1: No mode selected, waiting...")
    #             self._logger.info("========================")
    #         if self.dataproc(data_capture_inst, monitor, sec) is False:
    #             self._logger.info("Data source end")
    #             break
    #         time.sleep(0.01)

    #     while self.is_wait_selected(sec=sec, sac=sac):
    #         timercount_for_log += 1

    #         if timercount_for_log > 10:
    #             timercount_for_log = 0
    #             self._logger.info("========================")
    #             self._logger.info(
    #                 "A1: No mode selected, waiting... (data source end)"
    #             )
    #             self._logger.info("========================")

    #         time.sleep(0.1)
    #         self.debug_index += 1
    #         monitor.set_dummydata(
    #             enable_systemerrorflag=True,
    #             enable_errorflag=True,
    #             overwrite_calibresult=True,
    #             enable_yawangle=True,
    #         )
    #         monitor.transmit_setdata(sec=sec, ref_t=self.debug_index)

    #     self._logger.info("app_loopmain end")
    # except Exception as e:
    #     self._logger.error(
    #         f"app_loopmain: exception! {e} - \n{traceback.format_exc()}"
    #     )
    # finally:
    #     pass

    def post_app_loopmain(self) -> None:
        self._logger.info("app_loopmain end")

    @staticmethod
    def is_wait_selected(sec: SharedExcepts, sac: SharedAppConfig) -> bool:
        if sec.CalMatGen_ex.IsFinished.value:
            return False
        if sac.read().General.operation_mode != 1:
            return False
        if (
            sac.read().CalibMode.isRunning3D3Dcalib
            or sac.read().CalibMode.isRunning2D3Dcalib
            or sac.read().CalibMode.isRunning2D3Dcheck
            or sac.read().CalibMode.isRunningInterfaceDebug
        ):
            return False
        return True

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

    def dataproc(
        self,
        readresult_pop: FIFOData,
        monitor: CalibrationUIGodot,
        sec: SharedExcepts,
    ) -> bool:  # 継続可否を返す Falseで終了
        # readresults = self.proccap.read(data_capture_inst)

        # if readresults is None:
        #    return False
        # else:

        # フォントの種類（Hershey系）
        font = cv2.FONT_HERSHEY_SIMPLEX
        # 文字の位置（左上の座標）
        position = (50, 100)
        # 文字の色（BGR形式：青）
        color = (255, 0, 0)
        # 文字のサイズ
        font_scale = 1.0
        # 線の太さ
        thickness = 2

        if True:
            camera_datalist, lidar_datalist, can_data, framecounter = readresult_pop
            if self.input_data_diagnosis(
                camera_datalist,
                lidar_datalist,
                can_data,
            ):
                return False

            for ix, x in enumerate(camera_datalist):
                if x is not None:
                    if self.verbose:
                        self._logger.info(f"{ix}: {x[0].shape}, {x[1]}, {x[2]}")
                    frame = x[0]

                    frame: NDArray[np.uint8] = self.ud.get_undistort_image(frame)
                    cv2.putText(
                        frame,
                        f"{x[1]}, {x[2]}",
                        position,
                        font,
                        font_scale,
                        color,
                        thickness,
                        cv2.LINE_AA,
                    )

                    if x[0].size > 0:
                        # monitor.put_data("dataproc", f"detect2d_image{ix}", x[0])
                        monitor.set_image(ix, frame)

            lidar_data = [x[0] for x in lidar_datalist if x is not None]
            if len(lidar_data) > 0:
                pts = conbine3d3d(
                    xyz_data=lidar_data,
                    trans_mat3D3D_eachlidar=self.trans_mat3D3D_eachlidar,
                )
                if self.verbose:
                    self._logger.info(f"before downsample, lidar points: {pts.shape}")
                lidar_points = pcd_to_np(np_to_pcd(pts[:, :3]).voxel_down_sample(0.1))
                if self.verbose:
                    self._logger.info(
                        f"after downsample, lidar points: {lidar_points.shape}"
                    )
                lidar_points[:, 1] = -lidar_points[:, 1]
                lidar_points[:, 2] = -lidar_points[:, 2]

                lidar_points = lidar_points[lidar_points[:, 2] > -0.4]

                if self.verbose:
                    self._logger.info(
                        f"after limitation, lidar points: {lidar_points.shape}"
                    )

                monitor.set_points(
                    lidar_points[:, :3],
                    np.tile([0.2, 0.2, 0.2], (lidar_points.shape[0], 1)),
                )

            # cv2.waitKey(1)

        monitor.set_dummydata(
            enable_systemerrorflag=True,
            enable_errorflag=True,
            enable_yawangle=True,
            overwrite_checkresult=True,
            overwrite_calibresult=True,
        )
        monitor.transmit_setdata(sec=sec, ref_t=self.debug_index)

        self.debug_index += 1
        return True
