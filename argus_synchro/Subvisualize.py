from __future__ import annotations

import numpy as np
import open3d as o3d
from argus_synchro_lib.octotree import NodeEntity
from numpy.typing import NDArray


def create_box_at_point(
    point: NDArray[np.float64], size: float = 1.0
) -> o3d.geometry.TriangleMesh:
    """
    点に対応する小さな箱を生成
    """
    box: o3d.geometry.TriangleMesh = o3d.geometry.TriangleMesh.create_box(
        width=size,
        height=size,
        depth=size,
    )
    box.translate(point)
    return box


# memberごとに色を決める役割モジュールが2か所にあるので変な実装だが、SHOW_POINTSオプションとの兼ね合いで、
# すべてをこちらに持ってくるのはすぐには難しい. あとで見直す.
def get_color_by_height(
    member: NodeEntity,
    z_value: float,
    min_z: float,
    max_z: float,
) -> list[float]:
    # 高さに基づいて色を決定(青から赤へ)
    t: float = (z_value - min_z) / (max_z - min_z)
    # return [t, 0.05, 1-t]
    if member == NodeEntity.OTHER:
        col: list[float] = [t, 0.3, 1 - t]
    else:
        ratio = 0.7
        col = [(1 - ratio) * t + ratio, 0, 0]
        # col = [0, 0, (1 - ratio) * t + ratio]
    return col


def update_boxes(
    member: NodeEntity,
    points: NDArray[np.float32] | NDArray[np.float64],
    box_size: float = 1.0,
) -> o3d.geometry.TriangleMesh:
    """
    新しい点群に基づいて各箱を再生成
    """
    boxes = o3d.geometry.TriangleMesh()
    colors: list[list[float]] = []
    min_z: float = -1.38
    max_z: float = 1.0

    for point in points:
        box: o3d.geometry.TriangleMesh = create_box_at_point(point, box_size)
        color: list[float] = get_color_by_height(member, point[2], min_z, max_z)
        for _ in range(len(box.vertices)):
            colors.append(color)  # 各頂点に色を適用
        boxes += box

    boxes.vertex_colors = o3d.utility.Vector3dVector(colors)
    return boxes
