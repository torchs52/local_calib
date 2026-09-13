import numpy as np
import open3d as o3d
from numpy.typing import NDArray

from argus_synchro.config.app_config_calibration import (
    Calib2d3dConf,
)


def Config_Proc3d_Datarange_loader(calib2d3d: Calib2d3dConf):
    return (
        (
            calib2d3d.Proc3d.datarange_x_min,
            calib2d3d.Proc3d.datarange_x_max,
        ),
        (
            calib2d3d.Proc3d.datarange_y_min,
            calib2d3d.Proc3d.datarange_y_max,
        ),
        (
            calib2d3d.Proc3d.datarange_z_min,
            calib2d3d.Proc3d.datarange_z_max,
        ),
    )


def scale_transform(
    data: np.ndarray,
    val_min: float = 0,
    val_max: float = 1,
    allclose: float = 0.001,
) -> NDArray:
    if (data.max() - data.min()) < allclose:
        return np.array([(val_max + val_min) / 2] * len(data))
    return (val_max - val_min) / (data.max() - data.min()) * (
        data - data.min()
    ) + val_min


def set_xyz_range(
    pcd_data: NDArray,
    x_range: tuple[float, float],
    y_range: tuple[float, float],
    z_range: tuple[float, float],
) -> NDArray:
    """LiDAR位置合わせ範囲の設定"""
    inrange = np.where(
        (pcd_data[:, 0] >= x_range[0])
        & (pcd_data[:, 0] <= x_range[1])
        & (pcd_data[:, 1] >= y_range[0])
        & (pcd_data[:, 1] <= y_range[1])
        & (pcd_data[:, 2] >= z_range[0])
        & (pcd_data[:, 2] <= z_range[1])
    )[0]
    return pcd_data[inrange]


def np_to_pcd(numpy_file) -> o3d.geometry.PointCloud:
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(numpy_file)
    return pcd


def pcd_to_np(pcd_file):
    numpy_file = np.asarray(pcd_file.points)
    return numpy_file


def bounding_box(numpy_pc, unique_labels, labels):
    # クラスごとに最小最大値座標とBB用のリストを作成
    multi_points = []
    multi_lines = []

    multi_minmax = []

    for idx in unique_labels:
        if idx == -1:
            continue
        # クラスタごとにインデックスを取得
        indices = np.where(labels == idx)
        # 取得したインデックスで点群を抜き出し
        extracted_pc = numpy_pc[indices]

        # 抜き出した点群から最小最大値の点を見つける
        x_max = np.max(extracted_pc[:, 0])
        x_min = np.min(extracted_pc[:, 0])
        y_max = np.max(extracted_pc[:, 1])
        y_min = np.min(extracted_pc[:, 1])
        z_max = np.max(extracted_pc[:, 2])
        z_min = np.min(extracted_pc[:, 2])

        # ８個のポイントを設定
        points = [
            [x_min, y_min, z_min],
            [x_max, y_min, z_min],
            [x_min, y_max, z_min],
            [x_max, y_max, z_min],
            [x_min, y_min, z_max],
            [x_max, y_min, z_max],
            [x_min, y_max, z_max],
            [x_max, y_max, z_max],
        ]

        multi_points.extend(points)

        lines = [
            [0 + 8 * idx, 1 + 8 * idx],
            [0 + 8 * idx, 2 + 8 * idx],
            [1 + 8 * idx, 3 + 8 * idx],
            [2 + 8 * idx, 3 + 8 * idx],
            [4 + 8 * idx, 5 + 8 * idx],
            [4 + 8 * idx, 6 + 8 * idx],
            [5 + 8 * idx, 7 + 8 * idx],
            [6 + 8 * idx, 7 + 8 * idx],
            [0 + 8 * idx, 4 + 8 * idx],
            [1 + 8 * idx, 5 + 8 * idx],
            [2 + 8 * idx, 6 + 8 * idx],
            [3 + 8 * idx, 7 + 8 * idx],
        ]

        multi_lines.extend(lines)

        minmax = [x_min, x_max, y_min, y_max, z_min, z_max]
        multi_minmax.append(minmax)

    multi_points = np.array(multi_points)
    multi_lines = np.array(multi_lines)
    multi_minmax = np.array(multi_minmax)
    # colors = [[0, 0, 0] for i in range(len(multi_lines))]
    """
    print(multi_points.shape)
    print(multi_lines.shape)
    print(multi_minmax.shape)
    """
    return multi_points, multi_lines, multi_minmax
