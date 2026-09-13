# 対応点→校正行列算出プログラム

# 方向性として、このディレクトリ以外への依存性は極力減らす。
# privateメンバが使えないためmainから呼ばないものは名前空間で区切る。

# 2D-3D対応点を読み込み変換行列出力

import os
from collections.abc import Callable
from typing import Any

import cv2
import numpy as np
from numpy.typing import NDArray

from argus_synchro.calibration_mat_generator_modules.ctrl.calibration2d3d.correspondence.pnp_estimation_opt_core import (
    solvePnP_opt_leastsq,
)
from argus_synchro.common.app_logger import AppLogger, AppLoggerFactory


class PnpEstimationCalculator:  # FusionSensorProcessor互換インターフェース　ただしfit計算時に初期値rvec/tvecが必要なことに注意 RANSACは使用しない
    dist_coeffs: NDArray[np.float64]
    camera_matrix: NDArray[np.float64]
    verbose: bool
    rotation_vector: NDArray[np.float64] | None
    rotation_matrix: NDArray[np.float64] | None
    translation_vector: NDArray[np.float64] | None

    def __init__(
        self,
        dist_coeffs: NDArray[np.float64],
        camera_matrix: NDArray[np.float64],
        app_logger_factory: AppLoggerFactory,
        lambda_center_r: NDArray[np.float64] | None = None,
        lambda_center_t: NDArray[np.float64] | None = None,
        lambda_axis_r: NDArray[np.float64] | None = None,
        lambda_axis_t: NDArray[np.float64] | None = None,
        normalize_imagesize: NDArray[np.int32] | None = None,
        verbose: bool = True,
        file_io_error_reporter: Callable[[str, str, Exception], None] | None = None,
    ):
        """複数センサ間の座標合わせに関連する処理をまとめたクラス
        共通して使う変数もあるように感じたので、クラスとして管理する
        """
        self._logger: AppLogger = app_logger_factory.register_from_type(self.__class__)
        self.dist_coeffs = dist_coeffs
        self.camera_matrix = camera_matrix
        self.verbose = verbose
        self.rotation_vector: NDArray[np.float64] | None = None
        self.rotation_matrix: NDArray[np.float64] | None = None
        self.translation_vector: NDArray[np.float64] | None = None

        self.lambda_center_r = lambda_center_r
        self.lambda_center_t = lambda_center_t
        self.lambda_axis_r = lambda_axis_r
        self.lambda_axis_t = lambda_axis_t
        self.normalize_imagesize = normalize_imagesize
        self.file_io_error_reporter = file_io_error_reporter

    def fit(
        self,
        corners_3d: NDArray[np.float64],
        corners_2d: NDArray[np.float64],
        initial_rvec_rad: NDArray[np.float64],
        initial_tvec: NDArray[np.float64],
        centerF_or_axisT: bool,
        debug_dict: dict[str, Any] | None = None,  # デバッグ情報受け取り用辞書
    ):
        corners_3d = corners_3d.copy()
        corners_2d = corners_2d.copy()

        if not np.issubdtype(corners_3d.dtype, np.floating):
            corners_3d = corners_3d.astype(np.float64)
        if not np.issubdtype(corners_2d.dtype, np.floating):
            corners_2d = corners_2d.astype(np.float64)

        self.corner3d_save = corners_3d
        self.corner2d_save = corners_2d
        """ 3Dデータを2Dデータに移すための変換行列を計算する関数
        """
        self._logger.info(f"{type(corners_2d) = }, {type(corners_3d) = }")
        self._logger.info(
            f"{corners_2d.shape = }, {corners_2d[:10] = }, {corners_3d.shape = }, {corners_3d[:10] = }",
        )

        assert len(corners_2d) == len(corners_3d), (
            f"{len(corners_2d) = }, {len(corners_3d) = }"
        )

        if not centerF_or_axisT:
            lambda_r = self.lambda_center_r
            lambda_t = self.lambda_center_t
        else:
            lambda_r = self.lambda_axis_r
            lambda_t = self.lambda_axis_t

        success, self.rotation_vector, self.translation_vector = solvePnP_opt_leastsq(
            objectPoints=corners_3d,
            imagePoints=corners_2d,
            cameraMatrix=self.camera_matrix,
            distCoeffs=self.dist_coeffs,
            rvec=initial_rvec_rad,
            tvec=initial_tvec,
            r_fixedtgt_flag=0b111,  # 3bit: bit0=x, bit1=y, bit2=z
            t_fixedtgt_flag=0,  # 同上
            r_target=np.array(initial_rvec_rad),  # (3,), ロドリゲス角度[rad]
            t_target=np.array(initial_tvec),  # (3,), 平行移動（PnPの単位に合わせる）
            lambda_r=lambda_r,  # (3,), 軸別ペナルティ強度（ピクセル相当への重み付け）
            lambda_t=lambda_t,  # (3,), 軸別ペナルティ強度
            normalize_imagesize=self.normalize_imagesize,
            debug_dict=debug_dict,
        )

        self.rotation_matrix = cv2.Rodrigues(self.rotation_vector)[0]
        self._logger.info(f"ransac PnP - success flag: {success}")

        return self.rotation_vector, self.translation_vector

    def get_rotation_mat(self) -> NDArray[np.float64] | None:
        """回転行列と並進ベクトルを合わせた4*4の行列を作って返す"""
        try:
            rotation_mat = np.zeros((4, 4))
            rotation_mat[:3, :3] = self.rotation_matrix
            rotation_mat[:3, 3] = self.translation_vector.transpose()
            rotation_mat[3, 3] = 1
            return rotation_mat
        except Exception as e:
            self._logger.error(f"at get_rotation_mat, Exception: {e}, continue")
            return None

    def save(
        self,
        savedir: str = "",
        suffix: str = "",
        postprocess_mat: NDArray[np.float64] | None = None,
    ):
        """校正結果を保存"""

        output_path = savedir
        try:
            assert self.rotation_matrix is not None
            assert self.rotation_vector is not None
            assert self.translation_vector is not None

            rotation_vector_filename = os.path.join(
                savedir, "rotation_vector" + suffix + ".npy"
            )
            translation_vector_filename = os.path.join(
                savedir, "translation_vector" + suffix + ".npy"
            )

            corner2dsave_filename = os.path.join(
                savedir, "2dcorrpoints_calib" + suffix + ".npy"
            )
            corner3dsave_filename = os.path.join(
                savedir, "3dcorrpoints_calib" + suffix + ".npy"
            )

            output_path = rotation_vector_filename
            np.save(rotation_vector_filename, np.array(self.rotation_vector))
            output_path = translation_vector_filename
            np.save(translation_vector_filename, np.array(self.translation_vector))

            output_path = corner2dsave_filename
            np.save(corner2dsave_filename, self.corner2d_save)
            output_path = corner3dsave_filename
            np.save(corner3dsave_filename, self.corner3d_save)

            rotation_mat_filename = os.path.join(
                savedir, "rotation_mat" + suffix + ".txt"
            )

            transmat = self.get_rotation_mat()
            if postprocess_mat is not None:
                transmat = transmat @ postprocess_mat

            output_path = rotation_mat_filename
            np.savetxt(rotation_mat_filename, transmat, delimiter=",")

        except Exception as e:
            if isinstance(e, OSError) and self.file_io_error_reporter is not None:
                self.file_io_error_reporter(
                    output_path,
                    "write 2D-3D PnP calibration artifact",
                    e,
                )
            self._logger.warning(f"Exception: {e} at PnpEstimationCalculator.save()")
