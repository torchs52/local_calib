"""
LiDAR座標とピクセル座標の変換を行う処理が入ったモジュール
"""

import numpy as np

from argus_synchro.common.common import (
    NDPoint2f,
    NDPoint2i,
)
from argus_synchro.edge_det.const import DediscretizeMethod
from argus_synchro.edge_det.typedef import PxPyTup, XYTup


def grid_to_real(
    grid_coords_2d: NDPoint2i,
    grid_size: XYTup,
    grid_offset: PxPyTup = (0, 0),
    real_offset: XYTup = (0.0, 0.0),
    repr_method: DediscretizeMethod = DediscretizeMethod.MED,
) -> NDPoint2f:
    """格子上の座標を実数上の座標に変換する
    grid_coords p -> (p + grid_offset + repr_method_offset) * grid_size + real_offset

    :param grid_coords_2d: 格子座標
    :type grid_coords_2d: NDPoint2i
    :param grid_size: 格子サイズ
    :type grid_size: t_xy
    :param grid_offset: _description_, defaults to (0, 0)
    :type grid_offset: t_pxpy, optional
    :param real_offset: _description_, defaults to (0.0, 0.0)
    :type real_offset: t_xy, optional
    :param repr_method: 格子座標のどの位置で実数上の座標を表現するか, defaults to DediscretizeMethod.MED
    :type repr_method: DediscretizeMethod, optional
    :return: _description_
    :rtype: NDPoint2f
    """
    _grid_size = np.array(grid_size)[:2]
    _grid_offset = np.array(grid_offset, dtype=float)[:2]
    _real_offset = np.array(real_offset)[:2]
    match repr_method:
        case DediscretizeMethod.MED:
            _grid_offset += np.ones(len(grid_size)) * 0.5
        case DediscretizeMethod.MIN:
            _grid_offset += np.zeros(len(grid_size))
        case DediscretizeMethod.MAX:
            _grid_offset += np.ones(len(grid_size)) * 1.0
    return (grid_coords_2d + _grid_offset) * _grid_size + _real_offset


def real_to_grid(
    real_coords_2d: NDPoint2f,
    grid_size: XYTup,
    real_offset: XYTup = (0.0, 0.0),
    grid_offset: PxPyTup = (0, 0),
) -> NDPoint2i:
    """実数上の座標を格子状の座標に変換する
    real_coords_2d p -> floor(p - real_offset) / grid_size - grid_offset

    :param real_coords_2d: _description_
    :type real_coords_2d: NDPoint2f
    :param grid_size: _description_
    :type grid_size: t_xy
    :param real_offset: _description_, defaults to (0.0, 0.0)
    :type real_offset: t_xy, optional
    :param grid_offset: _description_, defaults to (0, 0)
    :type grid_offset: t_pxpy, optional
    :return: _description_
    :rtype: NDPoint2i
    """
    grid_coords_2d = np.floor(
        (real_coords_2d - np.array(real_offset)[:2]) / np.array(grid_size)
    ).astype(int)
    return grid_coords_2d - np.array(grid_offset)[:2]


def polar_to_cartesian(
    real_polar_coords_2d: NDPoint2f,
    from_polar_origin: XYTup,
) -> NDPoint2f:
    """from_polar_originを中心とした極座標上の点をデカルト座標上の点に変換する
    real_polar_coords_2d (r, t) -> (r cos(t), r sin(t)) + from_polar_origin

    :param real_polar_coords_2d: _description_
    :type real_polar_coords_2d: NDPoint2f
    :param from_polar_origin: _description_
    :type from_polar_origin: t_xy
    :return: _description_
    :rtype: NDPoint2f
    """
    radius_coord = real_polar_coords_2d[:, 0]
    angle_coord = real_polar_coords_2d[:, 1]

    x_coord = radius_coord * np.cos(angle_coord)
    y_coord = radius_coord * np.sin(angle_coord)

    return np.stack((x_coord, y_coord), axis=1) + np.array(from_polar_origin)[:2]


def cartesian_to_polar(
    real_cartesian_coords_2d: NDPoint2f,
    to_polar_origin: XYTup,
) -> NDPoint2f:
    """to_polar_origin中心でデカルト座標を極座標に変換する
    0列目がraidus, 1列目がangleとなるようにn*2の行列を返す
    p=(x,y) -> (dist(p, to_polar_origin), atan2(p - to_polar_origin)

    :param real_cartesian_coords_2d: _description_
    :type real_cartesian_coords_2d: NDPoint2f
    :param to_polar_origin: _description_
    :type to_polar_origin: t_xy
    :return: _description_
    :rtype: NDPoint2f
    """
    _real_cartesian_coords_2d = real_cartesian_coords_2d - np.array(to_polar_origin)[:2]
    radius_coord = np.sqrt(((_real_cartesian_coords_2d[:, :2]) ** 2).sum(axis=1))

    angle_coord = np.arctan2(
        _real_cartesian_coords_2d[:, 1], _real_cartesian_coords_2d[:, 0]
    )
    # angle_coord = angle_coord % (2 * np.pi) # この剰余計算は行わず関数を呼ぶ側で適宜オフセットを行う

    return np.stack((radius_coord, angle_coord), axis=1)


def polar_grid_to_grid(
    from_grid_coords_2d: NDPoint2i,
    from_origin: XYTup,
    to_origin: XYTup,
    grid_size: XYTup,
    grid_offset: PxPyTup = (0, 0),
    real_offset: XYTup = (0.0, 0.0),
) -> NDPoint2i:
    """ある点を中心とする極座標を使った格子座標上の点を別の点を中心とする極座標を使った格子座標上の点で表すための変換関数
    格子極座標 -> 実極座標 -> デカルト座標 -> 他の実極座標 -> 他の格子極座標

    :param from_grid_coords_2d: _description_
    :type from_grid_coords_2d: NDPoint2i
    :param from_origin: _description_
    :type from_origin: t_xy
    :param to_origin: _description_
    :type to_origin: t_xy
    :param grid_size: _description_
    :type grid_size: t_xy
    :param grid_offset: _description_, defaults to (0, 0)
    :type grid_offset: t_pxpy, optional
    :param real_offset: _description_, defaults to (0.0, 0.0)
    :type real_offset: t_xy, optional
    :return: _description_
    :rtype: NDPoint2i
    """

    # こうしないと座標を戻せなかったので、このオフセットを設定, real_offset[0]はこれで良いか未検証で、real_offset[0]を0以外にする場合は変換が合っているか確認する
    reverse_real_offset = (-real_offset[0], -real_offset[1])

    from_real_coords = grid_to_real(
        from_grid_coords_2d, grid_size, grid_offset, real_offset
    )

    cartesian_coords = polar_to_cartesian(from_real_coords, from_origin)

    to_polar_coords = cartesian_to_polar(cartesian_coords, to_origin)
    return real_to_grid(to_polar_coords, grid_size, reverse_real_offset, grid_offset)


def polar_grid_to_lidar_coord(
    polar_grid_2d: NDPoint2i,
    grid_size: XYTup,
    grid_offset: PxPyTup,
    real_offset: XYTup,
    origin: XYTup,
    repr_method: DediscretizeMethod = DediscretizeMethod.MED,
) -> NDPoint2f:
    """格子極座標をデカルト座標に変換する
    格子極座標 -> 実極座標 -> デカルト座標

    :param polar_grid_2d: _description_
    :type polar_grid_2d: NDPoint2i
    :param grid_size: _description_
    :type grid_size: t_xy
    :param grid_offset: _description_
    :type grid_offset: t_pxpy
    :param real_offset: _description_
    :type real_offset: t_xy
    :param origin: _description_
    :type origin: t_xy
    :param repr_method: _description_, defaults to DediscretizeMethod.MED
    :type repr_method: DediscretizeMethod, optional
    :return: _description_
    :rtype: NDPoint2f
    """
    real_polar_coords = grid_to_real(
        polar_grid_2d, grid_size, grid_offset, real_offset, repr_method
    )
    return polar_to_cartesian(real_polar_coords, origin)
