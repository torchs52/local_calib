# 画像対応点抽出

# import utils.utils3d as utils
import json
import math
from collections.abc import Callable

import numpy as np
import open3d as o3d
import pandas as pd
from numpy.typing import NDArray

from argus_synchro import calibration
from argus_synchro.calibration_mat_generator_modules.utils.debugdata_store import (
    debug_store,
)
from argus_synchro.common.app_logger import AppLogger, AppLoggerFactory
from argus_synchro.config.app_config_calibration import AppConfigCalibration
from argus_synchro.config.fileinput_pathselector import (
    lidar_calib_filepath_loader,
)
from argus_synchro.Registrate_LiDAR import crop_points
from argus_synchro.shared_app_config import SharedAppConfig


def np_to_pcd(numpy_file: NDArray[np.float32]) -> o3d.geometry.PointCloud:
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(numpy_file)
    return pcd


def pcd_to_np(pcd_file: o3d.geometry.PointCloud) -> NDArray[np.float32]:
    numpy_file = np.asarray(pcd_file.points)
    return numpy_file


def RotateMat_eulerangle(theta):
    R_x = np.array(
        [
            [1, 0, 0],
            [0, math.cos(theta[0]), -math.sin(theta[0])],
            [0, math.sin(theta[0]), math.cos(theta[0])],
        ]
    )
    R_y = np.array(
        [
            [math.cos(theta[1]), 0, math.sin(theta[1])],
            [0, 1, 0],
            [-math.sin(theta[1]), 0, math.cos(theta[1])],
        ]
    )
    R_z = np.array(
        [
            [math.cos(theta[2]), -math.sin(theta[2]), 0],
            [math.sin(theta[2]), math.cos(theta[2]), 0],
            [0, 0, 1],
        ]
    )
    R = np.dot(R_z, np.dot(R_y, R_x))
    return R


class capture3d:
    def __init__(
        self,
        app_config_calib: AppConfigCalibration,
        sac: SharedAppConfig,
        app_logger_factory: AppLoggerFactory,
        file_io_error_reporter: Callable[[str, str, Exception], None] | None = None,
    ) -> None:
        self._logger: AppLogger = app_logger_factory.register_from_type(self.__class__)
        # dataConverter2D3DConf: DataConverter2D3DConf, dataCaptureConf: DataCaptureConf, camerasel: int
        self.sac = sac
        self.app_config_calib = app_config_calib

        self.dataCaptureConf = app_config_calib.dataCapture
        self.dataConverter2D3DConf = app_config_calib.dataConverter2D3D
        camerasel = sac.read().CalibMode.cameraID

        rtvec_path = self.dataConverter2D3DConf.Lidar.calibration_coord_rtvec_jsonpath
        try:
            with open(rtvec_path, encoding="utf-8") as rtvec_definition:
                convvecs = json.load(rtvec_definition)
        except (OSError, UnicodeError, json.JSONDecodeError) as error:
            if file_io_error_reporter is not None:
                file_io_error_reporter(
                    rtvec_path,
                    "read LiDAR coordinate conversion JSON",
                    error,
                )
            raise

        self.convvec_t = np.array(convvecs[camerasel]["tvec"], dtype=np.float64)
        convvec_r_deg = np.array(convvecs[camerasel]["rvec"], dtype=np.float64)
        convvec_r_rad = convvec_r_deg / 180.0 * np.pi
        self.rmat = RotateMat_eulerangle(convvec_r_rad)
        if ((convvec_r_deg % 90) == 0).all():
            self.rmat = np.round(self.rmat)
        self.rev_rmat = self.rmat.T

        self.verbose = False  # TODO config追加
        self.is_finalframe_valid = True
        self.ref_t = None
        self.e_frame = None

        self.point_readfail_count = 0

        self.voxel_down_sample_voxelsize = (
            self.dataConverter2D3DConf.Lidar.voxel_down_sample_voxelsize
        )
        self.voxel_down_sample_enable_afteraccum = (
            self.dataConverter2D3DConf.Lidar.voxel_down_sample_enable_afteraccum
        )

        self.lidar_fifo = list()
        self.accumulate_length = self.dataConverter2D3DConf.Lidar.accumulate_length

        self.trans_mat3D3D = None
        self.trans_mat3D3D_eachlidar = []
        lidar_calib_paths = lidar_calib_filepath_loader(
            sac=self.sac,
            app_config_calib=self.app_config_calib,
        )
        for ix in range(2):
            # self.trans_mat3D3D_eachlidar.append( np.array(pd.read_csv(self.dataConverter2D3DConf.Lidar.lidar_calib_files[ix], header=None)) )
            lidar_calib_path = lidar_calib_paths[ix]
            try:
                transform = np.array(pd.read_csv(lidar_calib_path, header=None))
            except (OSError, UnicodeError, ValueError) as error:
                if file_io_error_reporter is not None:
                    file_io_error_reporter(
                        lidar_calib_path,
                        "read data_capture LiDAR calibration CSV",
                        error,
                    )
                raise
            self.trans_mat3D3D_eachlidar.append(transform)

    def release(self):
        pass

    def isOpened(self):
        point_readfail_count_threshold = 100

        if self.is_finalframe_valid:
            self.point_readfail_count = 0
        else:
            if self.point_readfail_count < point_readfail_count_threshold:
                self.point_readfail_count += 1

        return self.point_readfail_count < point_readfail_count_threshold

    def internal_lidar_accumulate(self, pcd):
        self.lidar_fifo.append(pcd)
        if len(self.lidar_fifo) > self.accumulate_length:
            del self.lidar_fifo[0]

        for idx in range(len(self.lidar_fifo)):
            frame = self.lidar_fifo[idx]
            if idx == 0:
                xyz = frame
            else:
                xyz = np.append(xyz, frame, axis=0)
        return xyz

    # @profile
    def read(
        self,
        data_lidars: list,
        dontread=False,
        adjust_coordinate_enable=True,
    ):
        if dontread:
            pass

        xyz_data = []
        timestamps = None
        for ix, data in enumerate(data_lidars):
            if data is None:
                xyz_data.append(np.zeros((0, 4)))
            else:
                xyz_data.append(data[0])
                if timestamps is None:
                    timestamps = data[1:]

        # 3D-3D校正行列適用
        debug_store("xyz_data_raw", value=xyz_data, index=-1)
        if self.trans_mat3D3D is not None:
            if len(xyz_data[1]) > 0:
                xyz_data[1][:, :3] = calibration.calib_lidar(
                    xyz_data[1], self.trans_mat3D3D
                )
        else:
            for ix in range(len(self.trans_mat3D3D_eachlidar)):
                if len(xyz_data[ix]) > 0:
                    xyz_data[ix][:, :3] = calibration.calib_lidar(
                        xyz_data[ix], self.trans_mat3D3D_eachlidar[ix]
                    )
        # ここで点群統合
        xyz = np.concatenate([xyz_data[0], xyz_data[1]], axis=0)

        if len(xyz) > 0:
            if self.verbose:
                self._logger.info(
                    self,
                    f"lidar : {xyz.shape = }, {np.max(xyz, axis=0) = }, {np.min(xyz, axis=0) = }",
                )

            # x_range = (-20,20)
            # y_range = (-20,20)
            # z_range = (-5,5)
            x_range = (
                self.dataCaptureConf.Lidar.capturerange_x_min,
                self.dataCaptureConf.Lidar.capturerange_x_max,
            )
            y_range = (
                self.dataCaptureConf.Lidar.capturerange_y_min,
                self.dataCaptureConf.Lidar.capturerange_y_max,
            )
            z_range = (
                self.dataCaptureConf.Lidar.capturerange_z_min,
                self.dataCaptureConf.Lidar.capturerange_z_max,
            )

            xyz = crop_points(xyz, x_range, y_range, z_range)
            xyz = pcd_to_np(
                np_to_pcd(xyz[:, :3]).voxel_down_sample(
                    self.voxel_down_sample_voxelsize
                )
            )
            if self.verbose:
                self._logger.info(
                    self,
                    f"after limit & downsample: {xyz.shape = }, {np.max(xyz, axis=0) = }, {np.min(xyz, axis=0) = }",
                )
        else:
            xyz = xyz[:, :3]

        # xyz = Adjust_Lidar_data(xyz_data)

        debug_store("xyz_frame", value=xyz, index=-1)

        # 点群蓄積
        if self.voxel_down_sample_enable_afteraccum:
            xyz_accum = pcd_to_np(
                np_to_pcd(self.internal_lidar_accumulate(xyz)).voxel_down_sample(
                    self.voxel_down_sample_voxelsize
                )
            )  # 望ましいのはこちら
        else:
            xyz_accum = self.internal_lidar_accumulate(xyz)

        if xyz_accum.size == 0:
            self.is_finalframe_valid = False
        else:
            self.is_finalframe_valid = True

        debug_store("xyz_accum", value=xyz_accum, index=-1)

        if self.verbose:
            self._logger.info(
                f"after accumulate and 3d3d transform, {xyz_accum.shape = }, {np.max(xyz_accum, axis=0) = }, {np.min(xyz_accum, axis=0) = }",
            )
        # 以前の会議から：3D3D校正・点群蓄積後に校正座標に変換
        if adjust_coordinate_enable:
            xyz_accum_pts = self.adjust_coordinate(xyz_accum, 0)
        else:
            xyz_accum_pts = xyz_accum

        return xyz_accum_pts, timestamps

    def adjust_coordinate(
        self,
        points3d: NDArray,
        convvec_index: int,
        rmat_accuracy_digit: int | None = None,
    ):
        if points3d.size == 0:
            return points3d

        assert points3d[0].size >= 3
        rmat = self.rmat  # 回転行列算出
        tvec = self.convvec_t

        res = points3d.copy()
        res[:, 0:3] = (rmat @ points3d[:, 0:3].T).T + tvec
        return res

    def adjust_rev_coordinate(self, points3d: NDArray, convvec_index: int):
        if points3d.size == 0:
            return points3d

        assert points3d[0].size >= 3
        rev_rmat = self.rev_rmat  # 回転行列算出
        # tvec = self.convvec_list_t_r[convvec_index][0]
        tvec = self.convvec_t
        res = points3d.copy()
        res[:, 0:3] = (rev_rmat @ (points3d[:, 0:3] - tvec).T).T
        return res

    def Adjust_Lidar_data(self, xyz):
        return xyz
