from __future__ import annotations

import numpy as np
import open3d as o3d
from numpy.typing import NDArray


def remove_pc_outside_of(
    data: NDArray[np.float64],
    x_dis_max: float | None = None,
    x_dis_min: float | None = None,
    y_dis_max: float | None = None,
    y_dis_min: float | None = None,
    z_dis_max: float | None = None,
    z_dis_min: float | None = None,
) -> NDArray[np.float64]:
    pc_data: NDArray[np.float64] = data
    if x_dis_min is not None and x_dis_max is not None:
        pc_data: NDArray[np.float64] = pc_data[
            np.where(
                (pc_data[:, 0] >= x_dis_min) & (pc_data[:, 0] < x_dis_max),
                True,
                False,
            )
        ]
    if y_dis_min is not None and y_dis_max is not None:
        pc_data = pc_data[
            np.where(
                (pc_data[:, 1] >= y_dis_min) & (pc_data[:, 1] < y_dis_max),
                True,
                False,
            )
        ]
    if z_dis_min is not None and z_dis_max is not None:
        pc_data = pc_data[
            np.where(
                (pc_data[:, 2] >= z_dis_min) & (pc_data[:, 2] < z_dis_max),
                True,
                False,
            )
        ]

    return pc_data


def np_to_pcd(
    numpy_file: NDArray[np.float32] | NDArray[np.float64],
) -> o3d.geometry.PointCloud:
    return o3d.geometry.PointCloud(o3d.utility.Vector3dVector(numpy_file))


def pcd_to_np(pcd_file: o3d.geometry.PointCloud) -> NDArray[np.float64]:
    numpy_file: NDArray[np.float64] = np.asarray(pcd_file.points)
    return numpy_file
