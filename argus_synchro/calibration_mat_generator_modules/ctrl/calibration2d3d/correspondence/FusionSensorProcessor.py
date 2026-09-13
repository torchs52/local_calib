# 対応点→校正行列算出プログラム

# 方向性として、このディレクトリ以外への依存性は極力減らす。
# privateメンバが使えないためmainから呼ばないものは名前空間で区切る。

# 2D-3D対応点を読み込み変換行列出力
# 構想基にファイルだけ作成したがクラス自体は手動校正プログラムのcalibration.py （→calibration2d3d_calccore.pyコピー）


import cv2
import numpy as np
from numpy.typing import NDArray

from argus_synchro.calibration_mat_generator_modules.utils.debugdata_store import (
    debug_store,
)
from argus_synchro.common.app_logger import AppLogger, AppLoggerFactory


class FusionSensorProcessor:
    dist_coeffs: np.ndarray
    camera_matrix: np.ndarray
    verbose: bool
    rotation_vector: np.ndarray
    rotation_matrix: np.ndarray
    translation_vector: np.ndarray

    def __init__(
        self,
        dist_coeffs: np.ndarray,
        camera_matrix: np.ndarray,
        app_logger_factory: AppLoggerFactory,
        verbose: bool = True,
    ):
        """複数センサ間の座標合わせに関連する処理をまとめたクラス
        共通して使う変数もあるように感じたので、クラスとして管理する
        """
        self._logger: AppLogger = app_logger_factory.register_from_type(self.__class__)
        self.dist_coeffs = dist_coeffs
        self.camera_matrix = camera_matrix
        self.verbose = verbose
        self.rotation_vector = None
        self.rotation_matrix = None
        self.translation_vector = None
        self.inliers = None

        self.rotation_vector_useallpoint = None
        self.translation_vector_useallpoint = None

    def fit(
        self,
        corners_3d: np.ndarray,
        corners_2d: np.ndarray,
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

        success, self.rotation_vector, self.translation_vector, self.inliers = (
            cv2.solvePnPRansac(
                objectPoints=corners_3d,
                imagePoints=corners_2d,
                cameraMatrix=self.camera_matrix,
                distCoeffs=self.dist_coeffs,
                flags=cv2.SOLVEPNP_ITERATIVE,
            )
        )
        self.rotation_matrix = cv2.Rodrigues(self.rotation_vector)[0]
        self._logger.info(f"ransac PnP - success flag: {success}")

        (
            success_useallpoint,
            self.rotation_vector_useallpoint,
            self.translation_vector_useallpoint,
        ) = cv2.solvePnP(
            objectPoints=corners_3d,
            imagePoints=corners_2d,
            cameraMatrix=self.camera_matrix,
            distCoeffs=self.dist_coeffs,
            flags=cv2.SOLVEPNP_ITERATIVE,
        )
        self._logger.info(f"Without ransac PnP - success flag: {success_useallpoint}")

        # self.inliers = np.array(range(len(corners_3d))) #inliersせっかく情報があるのに上書きしていたので修正　特記すべき背景を何か忘れている？

        if self.verbose:
            try:
                self.inliers = self.inliers.flatten()
                self._logger.info(
                    f"inliners: {self.inliers}, success: {success}",
                )

                # Compute re-projection error.
                corners_2d_reproj = self.internal_project_coordiinates(
                    np.array([corners_3d])
                )
                if corners_2d_reproj.shape != corners_2d.shape:
                    RuntimeError(
                        "Reprojection does not work"
                        "debug a function that creates points2D_repoj."
                    )

                rmse = self.internal_quantitative_score(
                    points_2d=corners_2d,
                    points_2d_projected=corners_2d_reproj,
                    inliers=self.inliers,
                )
                self._logger.info(
                    f"Re-projection error before LM refinement (RMSE) in px: {rmse!s}"
                )
            except Exception as e:
                self._logger.error(f"error : {e}")
        return self.rotation_vector, self.translation_vector

    def internal_quantitative_score(
        self,
        points_2d: np.ndarray,
        points_2d_projected: np.ndarray,
        inliers: np.ndarray = None,
    ) -> float:
        """lidarの基準点をカメラ座標に変換したものと、
        カメラの基準点を比較して、rmseを計算する関数
        Todo: 合致度判定の評価値として使えるように変更が必要
        """
        error = points_2d_projected - points_2d
        if inliers is not None:
            error = error[inliers]
        return np.sqrt(np.mean(error[:, 0] ** 2 + error[:, 1] ** 2))

    def get_lastfitresult(self):
        return self.rotation_vector, self.translation_vector

    def internal_project_coordiinates(
        self,
        points_3d: np.ndarray,
    ) -> NDArray:
        """3Dデータを2Dデータに変換する関数"""
        if not self.is_fitted():
            RuntimeError(
                "Calculating coordinate transform is necessary before project."
            )
            return None
        points2D_reproj = cv2.projectPoints(
            objectPoints=points_3d,
            rvec=self.rotation_vector,
            tvec=self.translation_vector,
            cameraMatrix=self.camera_matrix,
            distCoeffs=self.dist_coeffs,
        )[0].squeeze(1)

        return points2D_reproj

    def is_fitted(self) -> bool:
        """fitが行われているどうかの判定を行うメソッド
        fitが行われていればTrueを返す
        """
        return self.rotation_matrix is not None

    def get_rotation_mat(self) -> NDArray:
        """回転行列と並進ベクトルを合わせた4*4の行列を作って返す"""
        rotation_mat = np.zeros((4, 4))
        rotation_mat[:3, :3] = self.rotation_matrix
        rotation_mat[:3, 3] = self.translation_vector.transpose()
        rotation_mat[3, 3] = 1
        return rotation_mat

    def get_rotation_mat_useallpoint(self) -> NDArray:
        """回転行列と並進ベクトルを合わせた4*4の行列を作って返す RANSAC不使用"""
        rotation_mat = np.zeros((4, 4))
        rotation_mat[:3, :3] = cv2.Rodrigues(self.rotation_vector_useallpoint)[0]
        rotation_mat[:3, 3] = self.translation_vector_useallpoint.transpose()
        rotation_mat[3, 3] = 1
        return rotation_mat

    def save(
        self,
        savedir: str = "",
        suffix: str = "",
        postprocess_mat: NDArray[np.float64] | None = None,
    ):
        """校正結果を保存"""

        if self.rotation_vector is not None:
            debug_store("rotation_matrix", np.array(self.rotation_matrix))
            debug_store("rotation_vector", np.array(self.rotation_vector))
            debug_store("translation_vector", np.array(self.translation_vector))
            debug_store(
                "rotation_vector_useallpoint",
                np.array(self.rotation_vector_useallpoint),
            )
            debug_store(
                "translation_vector_useallpoint",
                np.array(self.translation_vector_useallpoint),
            )
            debug_store("2dcorrpoints_calib", self.corner2d_save)
            debug_store("3dcorrpoints_calib", self.corner3d_save)

            transmat = self.get_rotation_mat()
            if postprocess_mat is not None:
                transmat = transmat @ postprocess_mat

            debug_store("rotation_mat_final", transmat)
            debug_store("inliers", np.array(self.inliers))

        else:
            self._logger.warning(
                "Warning: FusionSensorProcessor.save() is called before calibration",
            )
