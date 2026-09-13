"""scx2000向けの機体除去クラスを作るための準備"""

from dataclasses import dataclass

import numpy as np
from numpy.typing import NDArray


def is_in_interval(
    points: NDArray,
    x_range: tuple[float, float] | None = None,
    y_range: tuple[float, float] | None = None,
    z_range: tuple[float, float] | None = None,
) -> NDArray:
    if len(points) == 0:
        raise ValueError("points is empty array")

    def _in_range(points: np.ndarray, range: tuple[float, float]) -> NDArray:
        return (range[0] < points) & (points < range[1])

    ind = np.array([True] * len(points))
    if x_range is not None:
        ind = ind & _in_range(points[:, 0], x_range)

    if y_range is not None:
        ind = ind & _in_range(points[:, 1], y_range)

    if z_range is not None:
        ind = ind & _in_range(points[:, 2], z_range)
    return ind


@dataclass
class CuboidRange:
    x_min: float
    x_max: float
    y_min: float
    y_max: float
    z_min: float
    z_max: float
    remove_minmax_range_ratio: tuple[
        tuple[float, float], tuple[float, float], tuple[float, float]
    ] = ((-1, 1), (-1, 1), (-1, 1))

    def check_pcd(
        self,
        target_points: NDArray,
        remove_dist: NDArray,
    ) -> NDArray:
        target_x = target_points[:, 0]
        target_y = target_points[:, 1]
        target_z = target_points[:, 2]

        remove_range_x_ratio, remove_range_y_ratio, remove_range_z_ratio = (
            self.remove_minmax_range_ratio
        )
        remove_x_from_ratio, remove_x_to_ratio = remove_range_x_ratio
        remove_y_from_ratio, remove_y_to_ratio = remove_range_y_ratio
        remove_z_from_ratio, remove_z_to_ratio = remove_range_z_ratio

        remove_ind = (
            ((self.x_min + remove_x_from_ratio * remove_dist[0]) <= target_x)
            & (target_x <= (self.x_max + remove_x_to_ratio * remove_dist[0]))
            & ((self.y_min + remove_y_from_ratio * remove_dist[1]) <= target_y)
            & (target_y <= (self.y_max + remove_y_to_ratio * remove_dist[1]))
            & ((self.z_min + remove_z_from_ratio * remove_dist[2]) <= target_z)
            & (target_z <= (self.z_max + remove_z_to_ratio * remove_dist[2]))
        )
        return remove_ind


def check_pcd_side(
    target_points: NDArray,
    normal_vec_2d: NDArray,
    target_x: tuple,
    target_y: tuple,
    target_z: tuple,
    point_on_line: NDArray,
) -> tuple[NDArray, NDArray]:
    inside_line_inds = ((target_points[:, :2] - point_on_line) @ normal_vec_2d) < 0

    in_cuboid_inds = is_in_interval(
        target_points,
        target_x,
        target_y,
        target_z,
    )

    # _target_points = target_points[in_cuboid_inds]
    remove_inds = inside_line_inds & in_cuboid_inds

    return remove_inds, in_cuboid_inds


@dataclass
class TriPillar:
    z_min: float
    z_max: float
    tri_points: list[tuple[float, float]]
    remove_offset_ratio: list[
        tuple[float, float]
    ]  # 基本的にremove_distの倍数での運用なので、remove_distに何倍掛けるかを設定値として持っておく
    remove_dist_z_ratio: float  # 基本的にremove_distの倍数での運用なので、remove_distに何倍掛けるかを設定値として持っておく
    normal_vec_is_reverse: bool = False

    def check_pcd(
        self,
        target_points: NDArray,
        remove_dist: NDArray,
    ) -> NDArray:
        p1 = (
            np.array(self.tri_points[0])
            + np.array(self.remove_offset_ratio[0]) * remove_dist[0]
        )
        p2 = (
            np.array(self.tri_points[1])
            + np.array(self.remove_offset_ratio[1]) * remove_dist[1]
        )

        target_x = (min([p1[0], p2[0]]), max([p1[0], p2[0]]))
        target_y = (min([p1[1], p2[1]]), max([p1[1], p2[1]]))
        target_z = (
            self.z_min - self.remove_dist_z_ratio * remove_dist[2],
            self.z_max + self.remove_dist_z_ratio * remove_dist[2],
        )

        p12 = p1 - p2
        remove_line_normal = np.array([-p12[1], p12[0]])
        remove_line_normal = remove_line_normal / np.linalg.norm(remove_line_normal)
        if self.normal_vec_is_reverse:
            remove_line_normal = -1 * remove_line_normal

        remove_inds, in_cuboid_inds = check_pcd_side(
            target_points,
            remove_line_normal,
            target_x,
            target_y,
            target_z,
            p1,
        )
        return remove_inds
