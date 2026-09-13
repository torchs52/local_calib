from __future__ import annotations

import time
from typing import Any

import numpy as np
import open3d as o3d
from numpy.typing import NDArray
from scipy.optimize import OptimizeResult, minimize

from argus_synchro.common.app_logger import AppLogger, AppLoggerFactory


class NDT:
    """
    NDT(Normal Distributions Transform)を実装するクラス
    """

    def __init__(self, grid_size: float) -> None:
        self._logger: AppLogger = AppLoggerFactory.from_type(self.__class__)
        self.grid_size: float = grid_size

    def create_ndt(
        self,
        point_cloud: NDArray[np.float64],
        grid_size: float,
    ) -> tuple[NDArray[Any], NDArray[np.float64]]:
        """
        グリッド内の各セルに対して点群の平均値および分散共分散行列の逆行列を計算し格納する関数

        Parameters:
        point_cloud (np.ndarray): 点群データの配列
        grid_size (float): グリッドセルのサイズ

        Returns:
        tuple: (グリッド内の各セルの平均値および逆行列を格納する配列, 点群の最小座標)
        """
        t0: float = time.time()
        min_coords: NDArray[np.float64] = np.min(point_cloud, axis=0)
        max_coords: NDArray[np.float64] = np.max(point_cloud, axis=0)
        grid_shape: NDArray[np.int64] = np.ceil(
            (max_coords - min_coords) / grid_size,
        ).astype(int)
        indices: NDArray[np.int64] = np.floor(
            (point_cloud - min_coords) / grid_size,
        ).astype(int)

        # 平均の計算
        ndt_grid_means: NDArray[np.float64] = np.zeros((*grid_shape, 3))
        counts: NDArray[np.int64] = np.zeros(grid_shape, dtype=int)
        np.add.at(ndt_grid_means, tuple(indices.T), point_cloud)
        np.add.at(counts, tuple(indices.T), 1)

        valid_cells = counts > 0
        ndt_grid_means[valid_cells] /= counts[valid_cells][:, None]

        # 分散共分散行列の計算を一括で実行
        diffs = point_cloud - ndt_grid_means[tuple(indices.T)]
        cov_accumulator: NDArray[np.float64] = np.zeros((*grid_shape, 3, 3))

        for i in range(len(point_cloud)):
            idx = tuple(indices[i])
            if counts[idx] > 1:
                cov_accumulator[idx] += np.outer(diffs[i], diffs[i])

        # 分散共分散行列の正規化と逆行列の計算
        ndt_grid_covs: NDArray[np.float64] = np.zeros((*grid_shape, 3, 3))
        ndt_grid_inv_covs: NDArray[np.float64] = np.empty((*grid_shape, 3, 3))
        for idx in np.ndindex(*grid_shape):
            if counts[idx] > 1:
                ndt_grid_covs[idx] = cov_accumulator[idx] / (counts[idx] - 1)
                ndt_grid_covs[idx] += np.eye(3) * 1e-6
                try:
                    ndt_grid_inv_covs[idx] = np.linalg.inv(ndt_grid_covs[idx])
                except np.linalg.LinAlgError:
                    ndt_grid_inv_covs[idx] = None
            else:
                ndt_grid_inv_covs[idx] = None

        # 平均値uと逆行列V^-1を格納
        ndt_grid: NDArray[Any] = np.empty(grid_shape, dtype=object)
        for idx in np.ndindex(*grid_shape):
            if counts[idx] > 1:
                ndt_grid[idx] = (ndt_grid_means[idx], ndt_grid_inv_covs[idx])
            else:
                ndt_grid[idx] = (None, None)

        self._logger.info("create_ndt: %f", time.time() - t0)
        return ndt_grid, min_coords

    @staticmethod
    def tukey_loss(
        residual: NDArray[np.float64],
        delta: float = 1.0,
    ) -> NDArray[np.float64]:
        """
        Tukey損失関数を計算する関数

        Parameters:
        residual (np.ndarray): 残差ベクトル
        delta (float): Tukey損失のパラメータ

        Returns:
        np.ndarray: 損失値
        """
        abs_residual: NDArray[np.float64] = np.abs(residual)
        condition = abs_residual <= delta
        loss: NDArray[np.float64] = np.zeros_like(residual)
        loss[condition] = delta**2 * (1 - (1 - (residual[condition] / delta) ** 2) ** 3)
        loss[~condition] = delta**2
        return loss

    def ndt_score(
        self,
        transform: NDArray[np.float64],
        source_points: NDArray[np.float64],
        delta: float = 10,
    ) -> float | np.float64:
        """
        変換に対するNDTスコアを計算する関数

        Parameters:
        transform (np.ndarray): 変換ベクトル
        source_points (np.ndarray): ソース点群
        delta (float): Tukey損失のパラメータ

        Returns:
        float: NDTスコア
        """
        cos_theta, sin_theta = np.cos(transform[2]), np.sin(transform[2])
        cos_phi, sin_phi = np.cos(transform[1]), np.sin(transform[1])
        cos_psi, sin_psi = np.cos(transform[0]), np.sin(transform[0])

        R: NDArray[np.float64] = np.array(
            [
                [
                    cos_theta * cos_phi,
                    cos_theta * sin_phi * sin_psi - sin_theta * cos_psi,
                    cos_theta * sin_phi * cos_psi + sin_theta * sin_psi,
                ],
                [
                    sin_theta * cos_phi,
                    sin_theta * sin_phi * sin_psi + cos_theta * cos_psi,
                    sin_theta * sin_phi * cos_psi - cos_theta * sin_psi,
                ],
                [-sin_phi, cos_phi * sin_psi, cos_phi * cos_psi],
            ],
        )

        transformed_points: NDArray[np.float64] = source_points @ R.T + transform[3:]

        cell_idx: NDArray[np.float64] = np.floor(
            (transformed_points - self.min_coords) / self.grid_size,
        ).astype(int)
        grid_shape = np.array(self.ndt_grid.shape[:3])
        valid_mask = np.all((cell_idx >= 0) & (cell_idx < grid_shape), axis=1)
        valid_cell_idx: NDArray[np.float64] = cell_idx[valid_mask]

        indices = tuple(valid_cell_idx.T)
        valid_cells = np.asarray(self.ndt_grid[indices], dtype=object)
        count_mask: NDArray[np.bool_] = np.array(
            [cell is not None and cell[0] is not None for cell in valid_cells],
            dtype=bool,
        )
        valid_cells = valid_cells[count_mask]
        valid_transformed_points: NDArray[np.float64] = transformed_points[valid_mask][
            count_mask
        ]

        if len(valid_cells) == 0:
            return np.inf

        means: NDArray[np.float64] = np.array([cell[0] for cell in valid_cells])
        cov_invs: NDArray[np.float64] = np.array([cell[1] for cell in valid_cells])
        d = valid_transformed_points - means
        residuals: NDArray[np.float64] = np.einsum("ij,ijk,ik->i", d, cov_invs, d)

        cost: np.float64 = np.sum(self.tukey_loss(residuals, delta))
        return cost

    def ndt_registration(
        self,
        source_points: NDArray[np.float64],
        target_points: NDArray[np.float64],
        yaw_angle: float,
        max_iter: int = 10,
        tol: float = 0.1,
    ) -> NDArray[np.float64]:
        """
        NDTに基づいて点群の位置合わせを行う関数

        Parameters:
        source_points (np.ndarray): ソース点群
        target_points (np.ndarray): ターゲット点群

        Returns:
        tuple: (変換された点群, 最終的な変換ベクトル)
        """
        # t0: float = time.time()

        self.ndt_grid, self.min_coords = self.create_ndt(target_points, self.grid_size)

        initial_transform: NDArray[np.float64] = np.zeros(6)
        if yaw_angle is not None:
            initial_transform[2] = yaw_angle

        result: OptimizeResult = minimize(
            self.ndt_score,
            initial_transform,
            args=(source_points,),
            method="BFGS",
            options={"maxiter": max_iter, "gtol": tol},
        )

        final_transform: NDArray[np.float64] = result.x

        rotation: NDArray[np.float64] = final_transform[:3]
        translation: NDArray[np.float64] = final_transform[3:]

        R: NDArray[np.float64] = np.array(
            [
                [
                    np.cos(rotation[2]) * np.cos(rotation[1]),
                    np.cos(rotation[2]) * np.sin(rotation[1]) * np.sin(rotation[0])
                    - np.sin(rotation[2]) * np.cos(rotation[0]),
                    np.cos(rotation[2]) * np.sin(rotation[1]) * np.cos(rotation[0])
                    + np.sin(rotation[2]) * np.sin(rotation[0]),
                ],
                [
                    np.sin(rotation[2]) * np.cos(rotation[1]),
                    np.sin(rotation[2]) * np.sin(rotation[1]) * np.sin(rotation[0])
                    + np.cos(rotation[2]) * np.cos(rotation[0]),
                    np.sin(rotation[2]) * np.sin(rotation[1]) * np.cos(rotation[0])
                    - np.cos(rotation[2]) * np.sin(rotation[0]),
                ],
                [
                    -np.sin(rotation[1]),
                    np.cos(rotation[1]) * np.sin(rotation[0]),
                    np.cos(rotation[1]) * np.cos(rotation[0]),
                ],
            ],
        )

        transformation_matrix: NDArray[np.float64] = np.eye(4)
        transformation_matrix[:3, :3] = R
        transformation_matrix[:3, 3] = translation

        return transformation_matrix

    @staticmethod
    def deg2rad(degrees: float) -> float:
        return degrees * (np.pi / 180)

    def log_register(self, app_logger_factory: AppLoggerFactory) -> None:
        app_logger_factory.append_logger(self._logger)


def np_to_pcd(numpy_file: NDArray[np.float64]) -> o3d.geometry.PointCloud:
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(numpy_file)
    return pcd
