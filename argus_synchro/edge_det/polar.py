"""
極座標を使った鳥瞰図に関連する処理が入っているモジュール
"""

from collections import defaultdict

import cv2
import numpy as np
from argus_synchro_lib.edge_det import (
    AggName,
    octotree2bev,
    scale_value,
)
from argus_synchro_lib.octotree import NodeEntity, OctoTree

from argus_synchro.common.common import (
    Err,
    NDImage,
    NDPoint2f,
    NDPoint2i,
    NDPoint3f,
    NDSeries,
    NDSeriesF,
    Ok,
    Point2f,
    Point2i,
    Point3f,
    Range,
    RangeF,
    Result,
    Size,
    SizeF,
    is_in_interval,
)
from argus_synchro.edge_det.common import apply_morphology_close, thresh_based_edge2bin
from argus_synchro.edge_det.const import BevCoord
from argus_synchro.edge_det.typedef import (
    EdgeDetError,
    EdgeDetLogicError,
    PxPyTup,
    XYTup,
    XYZTup,
)


def _polar_discrete(
    points: NDPoint2f,
    grid_size: tuple[float, float],
    discrete_origin: tuple[float, float] = (0, -np.pi),
) -> tuple[NDPoint2i, NDPoint2i]:
    """デカルト座標を極座標の格子座標に変換する
    points (x, y)を極座標表示して(r, t)に変換, それを離散化して(hat{r}, hat{t})にしている
    (r, t) から (hat{r}, hat{t})への変換は
    hat{r} = floor((r - o_r) / g_r), o_r: オフセット, g_r:格子サイズ


    :param points: 2次元の点の位置
    :type points: NDPoint2f
    :param grid_size: 格子サイズ
    :type grid_size: tuple[float, float]
    :param discrete_origin: 極座標から格子座標への変換のオフセット, defaults to (0, -np.pi)
    :type discrete_origin: tuple[float, float], optional
    :return: 格子座標
    :rtype: tuple[NDPoint2i, NDPoint2i]
    """
    bev_radius = np.sqrt(((points[:, :2]) ** 2).sum(axis=1))
    bev_angle = np.arctan2(points[:, 1], points[:, 0])

    bev_radius = bev_radius - discrete_origin[0]
    bev_angle = bev_angle - discrete_origin[1]

    grid_radius = np.floor(bev_radius / grid_size[0]).astype(int)
    grid_angle = np.floor(bev_angle / grid_size[1]).astype(int)
    return grid_radius, grid_angle


def _proc_agg(val: list[float], agg_name: AggName) -> Result[float, EdgeDetError]:
    """listの要素を集約して一つにする

    :param val: 集約される値の集合
    :type val: list[float]
    :param agg_name: 集約方法
    :type agg_name: AggName
    :return: 集約結果かエラー結果が入ったResult型
    :rtype: Result[float, EdgeDetError]
    """
    match agg_name:
        case AggName.MAX:
            return Ok(max(val))
        case AggName.MEAN:
            return Ok(np.array(val).mean())
        case AggName.MIN:
            return Ok(min(val))
        case AggName.LAST:
            return Ok(val[-1])
        case _:
            return Err(
                EdgeDetLogicError(
                    f"agg_nameに想定されていない列挙値が入っています: agg_name={agg_name}"
                )
            )


def py_octotree2bev(
    octree_obj: OctoTree,
    fwd_range: RangeF,
    side_range: RangeF,
    grid_size: RangeF,
    bev_shape: Range,
    target_entities: list[NodeEntity],
    bev_coord: BevCoord = BevCoord.POLAR,
    agg_name: AggName = AggName.MAX,
    coord_origin: Point3f = (0, 0, 0),
    discrete_origin: tuple[float, float] = (0, -np.pi),
    scaled: bool = True,
    min_scale_z: float = -1.88,
    max_scale_z: float = -0.88,
    min_bev_val: float = 0.0,
    max_bev_val: float = 255.0,
    nan_fill_value: float = 0.0,
) -> Result[NDImage, EdgeDetError]:
    """八分木の与えられたNodeEntityの点群を入力として鳥瞰図を生成する関数

    :param octree_obj: 八分木インスタンス
    :type octree_obj: OctoTree
    :param fwd_range: _description_
    :type fwd_range: RangeF
    :param side_range: _description_
    :type side_range: RangeF
    :param grid_size: _description_
    :type grid_size: RangeF
    :param bev_shape: _description_
    :type bev_shape: Range
    :param target_entities: _description_
    :type target_entities: list[NodeEntity]
    :param bev_coord: _description_, defaults to BevCoord.Polar
    :type bev_coord: BevCoord, optional
    :param agg_name: _description_, defaults to AggName.MAX
    :type agg_name: AggName, optional
    :param coord_origin: _description_, defaults to (0, 0, 0)
    :type coord_origin: Point3f, optional
    :param discrete_origin: _description_, defaults to (0, -np.pi)
    :type discrete_origin: tuple[float, float], optional
    :param scaled: _description_, defaults to True
    :type scaled: bool, optional
    :param min_scale_z: _description_, defaults to -1.88
    :type min_scale_z: float, optional
    :param max_scale_z: _description_, defaults to -0.88
    :type max_scale_z: float, optional
    :param min_bev_val: _description_, defaults to 0.0
    :type min_bev_val: float, optional
    :param max_bev_val: _description_, defaults to 255.0
    :type max_bev_val: float, optional
    :param nan_fill_value: _description_, defaults to 0.0
    :type nan_fill_value: float, optional
    :return: 正常に終了すればNDImage, 失敗すればEdgeDetErrorを返す
    :rtype: Result[NDImage, EdgeDetError]
    """
    # 点群の範囲を絞る
    target_points = octree_obj.get_np_from_entity_octonodes_by_chunk(target_entities)
    # np.save("target_points_snap.npy", target_points)
    detect_ind = is_in_interval(target_points, fwd_range, side_range)
    # detect_ind = is_in_interval(target_points, fwd_range, side_range)
    bev_points = target_points[detect_ind]

    # 鳥瞰図の原点を定める
    bev_points = bev_points - np.array(coord_origin)

    # 点群を離散化
    match bev_coord:
        case BevCoord.POLAR:
            grid_xs, grid_ys = _polar_discrete(
                points=bev_points,
                grid_size=grid_size,
                discrete_origin=discrete_origin,
            )
        case BevCoord.CARTESIAN:
            raise NotImplementedError("C++では実装済みなので、省略")

    # 各格子点の高さ情報を貯める
    bev_zs = bev_points[:, 2]
    bev_zs = scale_value(bev_zs, min_scale_z, max_scale_z, min_bev_val, max_bev_val)
    grid_pos: dict[tuple[int, int], list[float]] = defaultdict(list)
    for grid_x, grid_y, z_val in zip(grid_xs, grid_ys, bev_zs, strict=False):
        grid_pos[(grid_x, grid_y)].append(z_val)

    # 各格子に集約を掛ける
    polar_bev = np.zeros(bev_shape, dtype=np.uint8)
    for pixel_pos, z_val in grid_pos.items():
        res = _proc_agg(z_val, agg_name)
        if isinstance(res, Ok):
            polar_bev[(pixel_pos[0], pixel_pos[1])] = res.value
        else:
            return res

    return Ok(polar_bev)


def calc_rect_max_dist(
    min_pos: RangeF,
    max_pos: RangeF,
    origin: RangeF | Point3f,
) -> float:
    """min_pos, max_posで与えられる矩形に対して、原点をoriginとしたときの4隅の最大距離を計算する
    4隅と原点までの距離を計算して、その最大値を出力

    :param min_pos: _description_
    :type min_pos: RangeF
    :param max_pos: _description_
    :type max_pos: RangeF
    :param origin: _description_
    :type origin: RangeF | Point3f
    :return: _description_
    :rtype: float
    """
    rect_pos = np.array(
        [
            (min_pos[0] - origin[0], min_pos[1] - origin[1]),
            (min_pos[0] - origin[0], max_pos[1] - origin[1]),
            (max_pos[0] - origin[0], min_pos[1] - origin[1]),
            (max_pos[0] - origin[0], max_pos[1] - origin[1]),
        ]
    )
    return max(np.sqrt((rect_pos**2).sum(axis=1)))


def get_around_machine(
    octree_obj: OctoTree,
    fwd_range: RangeF,
    side_range: RangeF,
    grid_size: Point2f,
    node_entities: list[NodeEntity],
    group_center: Size,
    ground_height: float,
    n_morphology: int = 2,
    bin_th: int = 50,
) -> NDPoint3f:
    """機体周辺の死角位置の(x,y,z)を取得する

    :param octree_obj: _description_
    :type octree_obj: OctoTree
    :param fwd_range: _description_
    :type fwd_range: RangeF
    :param side_range: _description_
    :type side_range: RangeF
    :param grid_size: _description_
    :type grid_size: Point2f
    :param node_entities: _description_
    :type node_entities: list[NodeEntity]
    :param group_center: _description_
    :type group_center: Size
    :param ground_height: _description_
    :type ground_height: float
    :param n_morphology: _description_, defaults to 2
    :type n_morphology: int, optional
    :param bin_th: _description_, defaults to 50
    :type bin_th: int, optional
    :return: _description_
    :rtype: NDPoint3f
    """

    # 鳥観図を作成する
    # TODO: octotree2bevを修正する必要がある, 暫定的に(0,0)を入れてエラーを回避
    cartesian_bev = octotree2bev(
        octree_obj,
        fwd_range,
        side_range,
        (0, 0),
        (0, 0),
        node_entities,
        bev_coord=CppBevCoord.CARTESIAN,
    )
    cartesian_bev = apply_morphology_close(cartesian_bev, 2)

    # 地面側が0になるので、反転させる, TODO: 鳥観図の相対値に応じたbin_thになっていて、例えば点群が立体物で埋め尽くされていると意図した動作をするか怪しい
    bin_bev = thresh_based_edge2bin(cartesian_bev, min_th=bin_th)
    bin_bev = (255 - bin_bev).astype(np.uint8)
    _, labels = cv2.connectedComponents(bin_bev)

    # group_centerと同じラベリング部分を取り出す = 機体周りと同じグループの画素を取り出す
    target_pixel_x, target_pixel_y = np.where(labels == labels[group_center])

    # fwd_range, side_range, grid_sizeの対応に注意しながら画素をLiDAR座標に変換
    return np.array(
        [
            target_pixel_x * grid_size[1] + fwd_range[0],
            target_pixel_y * grid_size[0] + side_range[0],
            np.ones_like(target_pixel_x) * ground_height,
        ]
    ).T


def calc_max_radius_in_angle(
    target_angle: float,
    x_range: RangeF,
    y_range: RangeF,
    origin: Point2f,
    angle_close_eps: float = 0.1,
) -> Result[tuple[float, NDSeriesF], EdgeDetError]:
    """原点をorigin, 検出範囲をx_range, y_rangeで囲まれる矩形としたときに、target_angleに対する矩形までの距離を計算する関数
    矩形までの距離が分かることで与えられたtarget_angleの最大検出範囲が分かる

    :param target_angle: 計算したい角度
    :type target_angle: float
    :param x_range: x軸の検出範囲
    :type x_range: RangeF
    :param y_range: y軸の検出範囲
    :type y_range: RangeF
    :param origin: 極座標の原点
    :type origin: Point2f
    :param angle_close_eps: _description_, defaults to 0.1
    :type angle_close_eps: float, optional
    :return: _description_
    :rtype: Result[tuple[float, NDSeriesF], EdgeDetError]
    """

    close_eps = 1e-5
    # 角度を[0, 2pi]の範囲に標準化する
    norm_angle = target_angle % (2 * np.pi)

    intersec_rect_point: NDPoint2f
    # 0, pi/2, pi, 3pi/2, 2piの時に交わる点は別途計算
    if np.abs(norm_angle - 0) < close_eps or np.abs(norm_angle - 2 * np.pi) < close_eps:
        intersec_rect_point = np.array([(x_range[1], origin[1])])
    elif np.abs(norm_angle - np.pi / 2) < close_eps:
        intersec_rect_point = np.array([(origin[0], y_range[1])])
    elif np.abs(norm_angle - np.pi) < close_eps:
        intersec_rect_point = np.array([(x_range[0], origin[1])])
    elif np.abs(norm_angle - 3 * np.pi / 2) < close_eps:
        intersec_rect_point = np.array([(origin[0], y_range[0])])
    else:
        # 検出範囲の矩形のどこと交わるかを計算
        intersec_rect_candidate_points = (
            ((y_range[0] - origin[1]) / np.tan(norm_angle) + origin[0], y_range[0]),
            ((y_range[1] - origin[1]) / np.tan(norm_angle) + origin[0], y_range[1]),
            (x_range[0], np.tan(norm_angle) * (x_range[0] - origin[0]) + origin[1]),
            (x_range[1], np.tan(norm_angle) * (x_range[1] - origin[0]) + origin[1]),
        )

        _intersec_rect_points: list[Point2f] = []
        if (
            x_range[0] <= intersec_rect_candidate_points[0][0]
            and intersec_rect_candidate_points[0][0] <= x_range[1]
        ):
            _intersec_rect_points.append(intersec_rect_candidate_points[0])

        if (
            x_range[0] <= intersec_rect_candidate_points[1][0]
            and intersec_rect_candidate_points[1][0] <= x_range[1]
        ):
            _intersec_rect_points.append(intersec_rect_candidate_points[1])

        if (
            y_range[0] <= intersec_rect_candidate_points[2][1]
            and intersec_rect_candidate_points[2][1] <= y_range[1]
        ):
            _intersec_rect_points.append(intersec_rect_candidate_points[2])

        if (
            y_range[0] <= intersec_rect_candidate_points[3][1]
            and intersec_rect_candidate_points[3][1] <= y_range[1]
        ):
            _intersec_rect_points.append(intersec_rect_candidate_points[3])
        intersec_rect_points = np.array(_intersec_rect_points)

        # arctanで角度が復元できる方を採用
        reconst_angle = np.arctan2(
            intersec_rect_points[:, 1] - origin[1],
            intersec_rect_points[:, 0] - origin[0],
        ) % (2 * np.pi)
        intersec_rect_point = intersec_rect_points[
            np.abs(reconst_angle - norm_angle) < angle_close_eps
        ]

    if len(intersec_rect_point) != 1:
        return Err(
            EdgeDetLogicError(
                f"想定外の交点があるので、要検証, target_angle={target_angle}, x_range={x_range}, y_range={y_range}, origin={origin}, intersec_rect_point={intersec_rect_point}"
            )
        )

    dist: float = np.linalg.norm(intersec_rect_point - origin[:2])
    return Ok((dist, intersec_rect_point.squeeze()))


def postproc_for_polar(
    polar_bev: cv2.typing.MatLike,
    min_radius_grid_offset: int,
    front_radius: float,
    polar_grid: tuple[float, float],
    front_kernel_size: tuple[int, int],
    rear_kernel_size: tuple[int, int],
) -> cv2.typing.MatLike:
    """鳥観図を作った後の後処理関数
    動径方向が小さい箇所と大きい箇所で別々の画像処理をしている

    :param polar_bev: _description_
    :type polar_bev: cv2.typing.MatLike
    :param min_radius_grid_offset: _description_
    :type min_radius_grid_offset: int
    :param front_radius: _description_
    :type front_radius: float
    :param polar_grid: _description_
    :type polar_grid: tuple[float, float]
    :param front_kernel_size: _description_
    :type front_kernel_size: tuple[int, int]
    :param rear_kernel_size: _description_
    :type rear_kernel_size: tuple[int, int]
    :return: _description_
    :rtype: cv2.typing.MatLike
    """
    grid_radius, _ = polar_grid

    front_grid = int(np.floor(front_radius / grid_radius))
    _polar_bev = polar_bev[min_radius_grid_offset:]
    return np.vstack(
        [
            apply_morphology_close(_polar_bev[:front_grid], front_kernel_size),
            apply_morphology_close(_polar_bev[front_grid:], rear_kernel_size),
        ]
    )


def preproc_edge(img: cv2.typing.MatLike, n_ite: int = 2) -> cv2.typing.MatLike:
    """エッジ検出を行う前の前処理を行って、その結果を返す, エッジ検出の結果にバイラテラルフィルタを何回か適用する

    :param img: _description_
    :type img: cv2.typing.MatLike
    :param n_ite: _description_, defaults to 2
    :type n_ite: int, optional
    :return: _description_
    :rtype: cv2.typing.MatLike
    """
    bi = cv2.bilateralFilter(img, 10, 20, 20)
    for _ in range(n_ite):
        bi = cv2.bilateralFilter(img, 10, 20, 20)

    return bi


def polarpixel_to_lidar_coord(
    polarpixel: NDPoint2i,
    grid_size: XYTup,
    grid_offset: PxPyTup,
    w_offset: XYTup,
    w_origin: XYTup,
) -> NDPoint2f:
    """
    w_origin周りの極座標の離散座標をLiDAR座標の実座標に変換する関数

    :param polarpixel: 極座標の離散座標が入ったndarray
    :type polarpixel: NDArray
    :param grid_size: 極座標における離散幅, (radius, angleの順)
    :type grid_size: t_xy
    :param grid_offset: 離散座標において載っているオフセット
    :type grid_offset: t_pxpy
    :param w_offset: w_origin中心の実座標において載っているオフセット
    :type w_offset: t_xy
    :param w_origin: LiDAR座標における極座標中心の位置
    :type w_origin: t_xy
    :return: LiDAR座標における極座標上の点の位置
    :rtype: NDArray[Any]
    """
    # 離散座標からw_origin原点の実極座標に写す
    polar_real_coords = (polarpixel[:, :2] + np.array(grid_offset)) * np.array(
        grid_size
    ) + np.array(w_offset)
    radius_real_coords = polar_real_coords[:, 0]
    # 角度は[0, 2pi]で表すことにする
    angle_real_coords = polar_real_coords[:, 1] % (2 * np.pi)

    # LiDAR原点に写す
    lidar_coords_2d = (
        np.hstack(
            [
                (radius_real_coords * np.cos(angle_real_coords))[:, np.newaxis],
                (radius_real_coords * np.sin(angle_real_coords))[:, np.newaxis],
            ]
        )
        + np.array(w_origin)[:2]
    )

    return lidar_coords_2d


def calc_max_radius_each_theta(
    origin: XYZTup,
    x_range: RangeF,
    y_range: RangeF,
    grid_size_polar: SizeF,
    real_offset: Point2f,
    grid_offset: Point2i,
) -> Result[NDSeries, EdgeDetError]:
    """各角度に対して最大の検出範囲を計算して、
    鳥観図の各角度に対する動径方向の検出範囲を計算する

    :param origin: _description_
    :type origin: t_xyz
    :param x_range: _description_
    :type x_range: RangeF
    :param y_range: _description_
    :type y_range: RangeF
    :param grid_size_polar: _description_
    :type grid_size_polar: SizeF
    :param real_offset: _description_
    :type real_offset: Point2f
    :param grid_offset: _description_
    :type grid_offset: Point2i
    :return: _description_
    :rtype: Result[NDSeries, EdgeDetError]
    """
    origin_xy = (origin[0], origin[1])
    max_w_radiuses: list[float] = []
    # 角度の格子位置に対して動径方向の最大検出範囲を計算
    for grid_angle in np.arange(-np.pi, np.pi, grid_size_polar[1]):
        res = calc_max_radius_in_angle(
            target_angle=grid_angle,
            x_range=x_range,
            y_range=y_range,
            origin=origin_xy,
        )
        match res:
            case Ok((val, _)):
                max_w_radiuses.append(val)
            case Err(e):
                return Err(e)

    # 得られた各角度に対する動径方向の最大値を格子座標に変換する
    np_max_w_radiuses: NDSeriesF = np.array(max_w_radiuses)
    np_max_grid_w_radiuses: NDSeries = (
        np.floor((np_max_w_radiuses - real_offset[0]) / grid_size_polar[0]).astype(int)
        - grid_offset[0]
    )

    return Ok(np_max_grid_w_radiuses)
