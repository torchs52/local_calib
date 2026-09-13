"""衝突判定評価用の格子を作ったり、格子をいい感じに可視化したりするのに用いるモジュール
基本的には、検証でだけ用いる
"""

import copy
import itertools

import numpy as np
import open3d as o3d
from numpy.typing import NDArray


def _concat_geometry(
    geometries: list[o3d.geometry.LineSet],
) -> o3d.geometry.LineSet:
    """同じ構造を持つopen3dのオブジェクトを繋げる
    Note: まとめた方が描画は早くなる
    """
    concated_geometry = copy.deepcopy(geometries[0])
    if len(geometries) == 1:
        return concated_geometry

    for geometry in geometries[1:]:
        concated_geometry += geometry
    return concated_geometry


def _create_grid_center_points(
    x_range: tuple[float, float],
    y_range: tuple[float, float],
    grid_size: tuple[float, float],
) -> NDArray[np.float64]:
    """格子の中心のxy座標を作る
    x_rangeの範囲でx座標は格子を作って, y_rangeの範囲でy座標は格子を作って, grid_sizeの間隔で作る
    格子の中心座標を返すため、最後に格子幅/2を加えている
    """
    # 格子のx,y座標の直積を得る
    points = np.array(
        list(
            itertools.product(
                np.arange(x_range[0], x_range[1] + grid_size[0], grid_size[0]),
                np.arange(y_range[0], y_range[1] + grid_size[1], grid_size[1]),
            ),
        ),
        dtype=np.float64,
    )

    # 格子の中心に移す
    return points + np.array(grid_size) / 2


def _create_rect_lineset(
    center_xy: tuple[float, float],
    length_xy: tuple[float, float],
    length_z: float,
    color: tuple[float, float, float] = (0, 0, 0),
) -> o3d.geometry.LineSet:
    """center_xyを中心とした長さlength_xyのlinesetを高さがlength_zの地点に作る
    色はデフォルトは黒
    """
    offset_xy = (length_xy[0] / 2, length_xy[1] / 2)

    points = np.array(
        [
            [center_xy[0] - offset_xy[0], center_xy[1] - offset_xy[1], length_z],
            [center_xy[0] + offset_xy[0], center_xy[1] - offset_xy[1], length_z],
            [center_xy[0] + offset_xy[0], center_xy[1] + offset_xy[1], length_z],
            [center_xy[0] - offset_xy[0], center_xy[1] + offset_xy[1], length_z],
        ],
    )

    inds = np.array(
        [
            [0, 1],
            [1, 2],
            [2, 3],
            [3, 0],
        ],
    )

    return o3d.geometry.LineSet(
        o3d.utility.Vector3dVector(points),
        o3d.utility.Vector2iVector(inds),
    ).paint_uniform_color(color)


def create_collision_mesh(
    machine_points: NDArray[np.float64],
    detect_range: tuple[float, float],
    grid_size: tuple[float, float],
    grid_height: float,
    grid_color: tuple[int, int, int] = (0, 0, 0),
) -> tuple[o3d.geometry.LineSet, NDArray[np.float64]]:
    """衝突判定の評価で用いる格子を作成する"""

    machine_min = machine_points.min(axis=0)
    machine_max = machine_points.max(axis=0)
    detect_x, detect_y = detect_range

    # カウンタウェイト前の格子の中心座標
    x_range = (machine_max[0], machine_max[0] + detect_x - grid_size[0])
    y_range = (machine_min[1] - detect_y, machine_max[1] + detect_y - grid_size[1])
    points_cw = _create_grid_center_points(x_range, y_range, grid_size)

    # 運転席前の格子の中心座標
    x_range = (machine_min[0] - detect_x, machine_min[0] - grid_size[0])
    y_range = (machine_min[1] - detect_y, machine_max[1] + detect_y - grid_size[1])
    points_op = _create_grid_center_points(x_range, y_range, grid_size)

    # 運転席から見て右側のクローラー隣の中心座標
    x_range = (machine_min[0], machine_max[0] - grid_size[0])
    y_range = (machine_max[1], machine_max[1] + detect_y - grid_size[1])
    points_clawer_right = _create_grid_center_points(x_range, y_range, grid_size)

    # 運転席から見て左側のクローラー隣の中心座標
    x_range = (machine_min[0], machine_max[0] - grid_size[0])
    y_range = (machine_min[1] - detect_y, machine_min[1] - grid_size[1])
    points_clawer_left = _create_grid_center_points(x_range, y_range, grid_size)

    grid_center_points = np.vstack(
        [
            points_cw,
            points_op,
            points_clawer_right,
            points_clawer_left,
        ],
    )

    o3d_grids = [
        _create_rect_lineset(center_xy, grid_size, grid_height, color=grid_color)
        for center_xy in grid_center_points
    ]

    o3d_grid = _concat_geometry(o3d_grids)

    return o3d_grid, grid_center_points


# def check_collision_grid_describable(
#     visual_ui: VisualizerInterface,
#     app_config: AppConfig,
# ) -> bool:
#     """衝突判定の格子を描画できるかどうか判定する"""
#     return (
#         isinstance(visual_ui, Open3DVisualizer)
#         and app_config.eval_collision.func_on
#         and not app_config.Visualizer.rotate_grid  # 格子が回転する場合も評価用の格子は使わないこととする
#     )
