# from open3d.cpu.pybind.geometry import PointCloud
import time
import traceback  # 例外時のトレースバック取得用

import numpy as np
import open3d as o3d
from numpy.typing import NDArray

from argus_synchro.calibration_mat_generator_modules.ctrl.calibration3d3d.calib_lidars import (
    calibrateLidars2Crane,
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
from argus_synchro.common.app_logger import AppLogger, AppLoggerFactory
from argus_synchro.config.app_config import AppConfig
from argus_synchro.config.app_config_calibration import AppConfigCalibration
from argus_synchro.diagnosis.error_diagnosis import ResultDiagnosis
from argus_synchro.message.calib_fifo_message import FIFOData

# from interface.sourse2target_point import NormalColor, PaintColorInterface, UnifromColor
# from lidar_registration.icp import registrateTwoPClouds
from argus_synchro.shared_app_config import SharedAppConfig
from argus_synchro.shared_errors import (
    ActionErrorIndex,
    SharedErrors,
    StateErrorDIndex,
)
from argus_synchro.shared_excepts import SharedExcepts

_logger: AppLogger = AppLoggerFactory.from_name("calibration3d3d_class")


def log_register(app_logger_factory: AppLoggerFactory) -> None:
    app_logger_factory.append_logger(_logger)


def np_to_pcd(numpy_file: NDArray[np.float32]) -> o3d.geometry.PointCloud:
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(numpy_file)
    return pcd


def pcd_to_np(pcd_file: o3d.geometry.PointCloud) -> NDArray[np.float32]:
    numpy_file = np.asarray(pcd_file.points)
    return numpy_file


class calibration3d3d_class:
    def __init__(
        self,
        app_config_calib: AppConfigCalibration,
        sac: SharedAppConfig,
        app_logger_factory: AppLoggerFactory,
        shared_errors: SharedErrors,
    ) -> None:
        self._app_logger_factory = app_logger_factory
        self._logger: AppLogger = app_logger_factory.register_from_type(self.__class__)
        self.sac = sac
        self.app_config_calib = app_config_calib
        self._ser: SharedErrors = shared_errors
        # 各クラスコンストラクタ呼び出し
        self.proccap = datacapture_class(
            app_config_calib=self.app_config_calib,
            sac=self.sac,
            app_logger_factory=self._app_logger_factory,
            shared_errors=self._ser,
        )
        self.debug_index = 0
        self.app_config_calib: AppConfigCalibration = app_config_calib

        self.visualize = False

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

    @staticmethod
    def _set_error_state(monitor: CalibrationUIGodot) -> None:
        monitor.set_errors_calibcommon(2)

    def pre_app_loopmain(
        self,
        monitor: CalibrationUIGodot,
        sec: SharedExcepts,
        sac: SharedAppConfig,
        app_config_calib: AppConfigCalibration,
    ):
        self._logger.info("app_loopmain running")
        # UI向けmmap 稼働状態=1 (状態B1, B2)
        monitor.set_status_calibcommon(CalibrationCommonStatus.RUNNING)
        monitor.set_dummydata(enable_systemerrorflag=True, enable_errorflag=True)
        monitor.transmit_setdata(sec, None, is_firstframe=True, mmap_erase_rest=True)

        # AppConfigの読み込み
        self.app_config: AppConfig = sac.read()

        self.lidarpoints = []

        self.validcount = 0

    def app_loopmain(
        self,
        readresult_pop: FIFOData,
        monitor: CalibrationUIGodot,
        sec: SharedExcepts,
        sac: SharedAppConfig,
        app_config_calib: AppConfigCalibration,
        resultmat_paths: list[str],
    ) -> bool:
        return self.dataproc(
            readresult_pop,
            monitor,
            sec,
            sac,
            app_config_calib,
            resultmat_paths,
        )

    def input_post_data_diagnosis(
        self,
        lidar_datalist: list[tuple[NDArray[np.float64], int, float]],
        can_data: tuple[float, float],
    ) -> bool:
        invalid_data_input = self._ser.state_errors_D[
            StateErrorDIndex.INVALID_DATA_INPUT
        ]
        result, failsafe_result = invalid_data_input.errors_diagnosis(
            (lidar_datalist, can_data)
        )
        invalid_data_input.log_output(
            result, failsafe_result, StateErrorDIndex.INVALID_DATA_INPUT
        )
        if result == ResultDiagnosis.DETECTION:
            return True

        min_xyz_columns = 3
        pcds_point_cloud = tuple(
            lidar_data[0][:, :min_xyz_columns] for lidar_data in lidar_datalist
        )
        array_shape_error = self._ser.state_errors_D[StateErrorDIndex.ARRAY_SHAPE_ERROR]
        result, failsafe_result = array_shape_error.errors_diagnosis(
            ("pcds_point_cloud", pcds_point_cloud),
        )
        array_shape_error.log_output(
            result, failsafe_result, StateErrorDIndex.ARRAY_SHAPE_ERROR
        )
        return result == ResultDiagnosis.DETECTION

    def input_capture_data_diagnosis(
        self,
        lidar_datalist: list[tuple[NDArray[np.float64], int, float]],
    ) -> bool:
        invalid_data_input = self._ser.state_errors_D[
            StateErrorDIndex.INVALID_DATA_INPUT
        ]
        result, failsafe_result = invalid_data_input.errors_diagnosis(lidar_datalist)
        invalid_data_input.log_output(
            result, failsafe_result, StateErrorDIndex.INVALID_DATA_INPUT
        )
        if result == ResultDiagnosis.DETECTION:
            return True

        min_xyz_columns = 3
        pcds_point_cloud = tuple(
            lidar_data[0][:, :min_xyz_columns] for lidar_data in lidar_datalist
        )
        array_shape_error = self._ser.state_errors_D[StateErrorDIndex.ARRAY_SHAPE_ERROR]
        result, failsafe_result = array_shape_error.errors_diagnosis(
            ("pcds_point_cloud", pcds_point_cloud),
        )
        array_shape_error.log_output(
            result, failsafe_result, StateErrorDIndex.ARRAY_SHAPE_ERROR
        )
        return result == ResultDiagnosis.DETECTION

    def post_app_loopmain(
        self,
        readresult_pop: FIFOData,
        monitor: CalibrationUIGodot,
        sec: SharedExcepts,
        sac: SharedAppConfig,
        app_config_calib: AppConfigCalibration,
        resultmat_paths: list[str],
    ):
        camera_datalist, lidar_datalist, can_data, framecounter = readresult_pop
        if self.input_post_data_diagnosis(lidar_datalist, can_data):
            self._set_error_state(monitor)
            return False
        # 撮ってきた点群を蓄積
        accum_points = []
        for pointframes_set in self.lidarpoints:
            for lid_ix, pts in enumerate(pointframes_set):
                if len(accum_points) <= lid_ix:
                    accum_points.append(np.zeros((0, pts.shape[-1]), dtype=pts.dtype))
                accum_points[lid_ix] = np.concatenate([accum_points[lid_ix], pts])
                self._logger.info(f"lidar {lid_ix} : accum_points {pts.shape}")

        angle_data = can_data[0]  # CAN入力実装済み
        monitor.set_yaw(angle_data)

        # UI向けmmap 稼働状態=2 #CalibStatus:B3
        monitor.set_status_calibcommon(CalibrationCommonStatus.CALCULATING)
        monitor.set_dummydata(enable_systemerrorflag=True, enable_errorflag=True)
        monitor.transmit_setdata(sec, None)

        pts0 = accum_points[0][:, :3]
        pts1 = accum_points[1][:, :3]

        if self.visualize:
            coord = o3d.geometry.TriangleMesh.create_coordinate_frame(size=3)
            o3d.visualization.draw_geometries([np_to_pcd(pts0), np_to_pcd(pts1), coord])

        # T_i2C =
        result_is_invalid = self.calib3d3d_once(
            lidar_pts=accum_points,
            angle_data=angle_data,
            resultmat_paths=resultmat_paths,
            app_config=self.app_config,
            app_config_calib=app_config_calib,
        )
        if result_is_invalid:
            self._set_error_state(monitor)

        self.debug_index += 1

        return False

    @classmethod
    def end_wait(
        cls,
        timercount: int,
        sec: SharedExcepts,
        sac: SharedAppConfig,
        monitor: CalibrationUIGodot,
    ) -> int:
        monitor.set_status_calibcommon(CalibrationCommonStatus.COMPLETED)
        monitor.set_dummydata(enable_systemerrorflag=True, enable_errorflag=True)
        monitor.transmit_setdata(sec=sec, ref_t=None)

        timercount += 1

        if timercount > 10:
            timercount = 0
            _logger.info("========================")
            _logger.info("Calib3d3d end")
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
        monitor.set_dummydata(enable_systemerrorflag=True, enable_errorflag=True)
        monitor.transmit_setdata(sec=sec, ref_t=None)

    def dataproc(
        self,
        readresult_pop: FIFOData,
        monitor: CalibrationUIGodot,
        sec: SharedExcepts,
        sac: SharedAppConfig,
        app_config_calib: AppConfigCalibration,
        resultmat_paths: list[str],
    ) -> bool:  # 継続可否を返す Falseで終了
        # フォントの種類（Hershey系）
        """font = cv2.FONT_HERSHEY_SIMPLEX
        # 文字の位置（左上の座標）
        position = (50, 100)
        # 文字の色（BGR形式：青）
        color = (255, 0, 0)
        # 文字のサイズ
        font_scale = 1.0
        # 線の太さ
        thickness = 2"""

        # while True:  # 点群を撮ってきて蓄積
        # CalibStatus:B2
        self._logger.info(f"capture {self.validcount}")

        if readresult_pop is None:
            self._set_error_state(monitor)
            return False

        camera_datalist, lidar_datalist, can_data, framecounter = readresult_pop
        if self.input_capture_data_diagnosis(lidar_datalist):
            self._set_error_state(monitor)
            return False
        # points_ts = 0
        both_valid = True
        frame_ptslist = []
        for ix, x in enumerate(lidar_datalist):
            if x is not None:
                if x[0] is not None and x[0].size > 0:
                    print(f"{ix}: {x[0].shape}, {x[1]}, {x[2]}")
                    # lidarpoints[-1].append(x[0])
                    frame_ptslist.append(x[0])
                    # points_ts = x[1]

                self._logger.info(
                    f"capture {self.validcount} - {ix} : shape{x[0].shape}"
                )
            else:
                self._logger.info(f"capture {self.validcount} - {ix} : None")
                both_valid = False
        if both_valid:
            self.lidarpoints.append(frame_ptslist)
            self.validcount += 1
            if self.validcount >= 30:
                return False
        return True

        # if len(points) > 0:
        #    monitor.put_data(
        #        "dataproc", "detect3d_points_raw", (np.vstack(points), points_ts)
        #    )

    def calib3d3d_once(
        self,
        lidar_pts: [list],
        angle_data: float,
        resultmat_paths: list[str],
        app_config: AppConfig,
        app_config_calib: AppConfigCalibration,
    ) -> bool:
        lidars_points_list = [lid_pts for lid_pts in lidar_pts]
        lidar_raw_pcd_list = [np_to_pcd(lid_pts[:, :3]) for lid_pts in lidar_pts]
        self._logger.info("entering calib3d3d_once")
        merged_pcd, lidar_pcds, T_i2C, T_i2L1 = calibrateLidars2Crane(
            lidar_np_list=lidars_points_list,  # [lidar1_np, lidar2_np, ..., lidarN_np]
            lidar_raw_pcd_list=lidar_raw_pcd_list,  # [lidar1_raw_pcd, ...]
            angle_data=angle_data,
            visualize=False,
            savepaths=resultmat_paths,
            app_config=app_config,
            calib_app_config=app_config_calib,
            file_io_error_reporter=self._report_file_io_error,
        )
        diagnosis = self._ser.action_errors_A_C[
            ActionErrorIndex.SENSOR_CALIB_DATA_INVALID
        ]
        validation_issues = diagnosis.diagnose_matrices(
            T_i2C,
            ActionErrorIndex.SENSOR_CALIB_DATA_INVALID,
            matrix_paths=resultmat_paths,
            reference_matrix_paths=(
                app_config_calib.Calib3d3d_CalibParams.lidars_calib_path
            ),
        )
        result_is_invalid = bool(validation_issues)
        if result_is_invalid:
            self._logger.error(
                "3D-3D calibration result validation failed: "
                f"issues={len(validation_issues)} first={validation_issues[0]}"
            )
        else:
            self._logger.info("3D-3D calibration result validation succeeded")
        self._logger.info("end calib3d3d_once")
        return result_is_invalid
