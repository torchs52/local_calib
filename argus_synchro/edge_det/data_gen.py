"""
崖検出の評価データを作成するためのモジュール
"""

import numpy as np

from argus_synchro.common.common import (
    NDPoint3f,
    NDSeries,
    NDSeriesB,
    RangeF,
    is_in_interval,
    rotate_y,
)

from .typedef import Range3D


def is_in_line_boundary(
    points: NDPoint3f,
    pos_from: NDPoint3f,
    pos_to: NDPoint3f,
    column_order: list[int] = [1, 0, 2],
    reverse: bool = False,
) -> NDSeriesB:
    """
    直線境界に応じて除去対象となる点を選択する
    pos_from, pos_toによる線分を作ってその前後で削除

    :param points: 除去対象を選ばれる点群
    :type points: np.ndarray
    :param pos_from: 説明
    :type pos_from: np.ndarray
    :param pos_to: 説明
    :type pos_to: np.ndarray
    :param column_order: 説明
    :type column_order: list
    :param reverse: 説明
    :type reverse: bool
    :return: 説明
    :rtype: ndarray[Any, Any]
    """
    _pos_from = pos_from[column_order]
    _pos_to = pos_to[column_order]
    _points = points[:, column_order]

    # 直線の傾きと切片を計算
    slope = (_pos_from[1] - _pos_to[1]) / (_pos_from[0] - _pos_to[0])
    intercept = (_pos_to[1] * _pos_from[0] - _pos_from[1] * _pos_to[0]) / (
        _pos_from[0] - _pos_to[0]
    )

    if reverse:
        target_ind = _points[:, 1] < (slope * _points[:, 0] + intercept)
    else:
        target_ind = _points[:, 1] >= (slope * _points[:, 0] + intercept)

    return is_in_interval(_points, x_range=(_pos_from[0], _pos_to[0])) & target_ind


def calc_target_by_line_boundary(
    points: NDPoint3f,
    line_pos: NDPoint3f,
    column_order: list[int] = [1, 0, 2],
    reverse: bool = False,
) -> tuple[NDSeriesB, NDSeriesB]:
    """
    line_pos同士をつなげる形で境界を作り、内側か外側の点を削除する

    :param points: 対象となる点群
    :type points: np.ndarray
    :param line_pos:
    :type line_pos: np.ndarray
    :param column_order: 説明
    :type column_order: list
    :param reverse: 説明
    :type reverse: bool
    :return: 説明
    :rtype: tuple[ndarray[Any, Any], ndarray[Any, Any]]
    """
    line_ind = np.hstack(
        [
            np.arange(len(line_pos))[:-1, np.newaxis],
            np.arange(len(line_pos))[1:, np.newaxis],
        ]
    )

    target_ind = np.array([False] * len(points))
    for ind in line_ind:
        pos_from = line_pos[ind[0]]
        pos_to = line_pos[ind[1]]
        target_ind = target_ind | is_in_line_boundary(
            points, pos_from, pos_to, column_order, reverse
        )

    return line_ind, target_ind


def create_sheer_cliff(
    points: NDPoint3f,
    sheer_cliff_range: list[Range3D],
) -> tuple[NDPoint3f, NDPoint3f]:
    """断崖絶壁の崖をpointsから作成する, sheer_cliff_rangeに該当しないものが点として残る

    :param points: _description_
    :type points: NDPoint3f
    :param sheer_cliff_range:
    :type sheer_cliff_range: list[Range3D]
    :return: _description_
    :rtype: tuple[NDPoint3f, NDPoint3f]
    """

    target_ind_sheer_cliff = np.array(
        [is_in_interval(points, *one_range) for one_range in sheer_cliff_range]
    ).any(axis=0)

    return points[~target_ind_sheer_cliff], target_ind_sheer_cliff


def create_slope_cliff(
    points: NDPoint3f,
    x_range: tuple[float, float],
    y_range: tuple[float, float],
    rot_center: NDSeries,
    rot_deg: float,
) -> tuple[NDPoint3f, NDPoint3f]:
    """傾斜のある崖を作る, x_range, y_rangeに該当する箇所の点群をrot_centerを中心にy軸周りにrot_degだけ回転させる

    :param points: _description_
    :type points: NDPoint3f
    :param x_range: _description_
    :type x_range: tuple[float, float]
    :param y_range: _description_
    :type y_range: tuple[float, float]
    :param rot_center: _description_
    :type rot_center: NDSeries
    :param rot_deg: 単位: degree
    :type rot_deg: float
    :return: _description_
    :rtype: tuple[NDPoint3f, NDPoint3f]
    """
    target_ind_sheer_cliff = is_in_interval(points, x_range, y_range)
    target_ground_points = points[target_ind_sheer_cliff]
    target_ground_points_rot = (target_ground_points - rot_center) @ rotate_y(
        np.deg2rad(rot_deg)
    ) + rot_center

    return (
        np.vstack([points[~target_ind_sheer_cliff], target_ground_points_rot]),
        target_ground_points_rot,
    )


def create_step_cliff(
    points: NDPoint3f,
    x_range: RangeF,
    y_range: RangeF,
    step_height: float = 0.5,
    noise_scale: float = 0,
) -> tuple[NDPoint3f, NDPoint3f]:
    """段差型の崖を生成する, x_range, y_rangeの範囲の地面点群が、step_heightだけ下になることで地面が作られる

    :param points: _description_
    :type points: NDPoint3f
    :param x_range: _description_
    :type x_range: RangeF
    :param y_range: _description_
    :type y_range: RangeF
    :param step_height: _description_, defaults to 0.5
    :type step_height: float, optional
    :param noise_scale: 段差位置の点群にノイズを加える, 0より大きい値を与えると高さ方向にノイズが載る
    :type noise_scale: float, optional
    :return: _description_
    :rtype: tuple[NDPoint3f, NDPoint3f]
    """
    target_ind_sheer_cliff = is_in_interval(points, x_range, y_range)
    target_points = points[target_ind_sheer_cliff]
    target_points[:, 2] = target_points[:, 2] - step_height

    if noise_scale > 0:
        target_points[:, 2] += np.random.normal(
            scale=noise_scale, size=len(target_points)
        )

    return (
        np.vstack([points[~target_ind_sheer_cliff], target_points]),
        target_points,
    )
