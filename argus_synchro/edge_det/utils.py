"""モジュール内の共通の関数などを置いておくモジュール"""

import numpy as np

from argus_synchro.common.common import (
    NDImage,
    NDPoint2iArray,
    NDPoint3fArray,
    NDSeries,
)


def calc_edge_length_piecewise(points: NDPoint3fArray) -> float:
    """
    pointsを順番に結んでいった時の総距離を計算する

    :param points: 結んでいく点
    :type points: NDArray
    :return: 総距離, 点の数が1以下の場合0を返す
    :rtype: float
    """
    if len(points) <= 1:
        return 0
    return np.linalg.norm(points[1:] - points[:-1], axis=1).sum()


def get_empty_points_lines_length() -> tuple[
    NDPoint3fArray,
    NDPoint2iArray,
    NDSeries,
]:
    """空の点群と空の線分, 空のクラスタを返す関数

    :return: _description_
    :rtype: tuple[ NDPoint3fArray, NDPoint2iArray, NDSeries, ]
    """
    return (
        np.empty((0, 3), np.float64),
        np.empty((0, 2), np.int32),
        np.empty((0,), np.int32),
    )


def scale_to_255(pixel_values: NDSeries, min_value: float, max_value: float) -> NDImage:
    """
    ピクセル値を0から255の範囲にスケーリングする。
    パラメータ:
        pixel_values: スケーリングするピクセル値の配列
        min: スケーリングのための最小値
        max: スケーリングのための最大値

    戻り値:
        スケーリングされたピクセル値の配列
    """
    # 最小値と最大値の間で正規化, min:0, max:1に線形変換している
    normalized = (pixel_values - min_value) / (max_value - min_value)
    # 0から255の範囲にスケーリング,
    scaled = np.clip(normalized * 255, 0, 255).astype(np.uint8)
    return scaled
