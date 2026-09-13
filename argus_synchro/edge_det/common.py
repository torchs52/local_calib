"""
崖検出の処理の中で共通して出てくる処理を入れておくモジュール
"""

import cv2
import numpy as np

from argus_synchro.common.common import (
    NDImage,
    NDSeries,
)
from argus_synchro.edge_det.const import BinMethod


def apply_morphology_close(
    target_image: cv2.typing.MatLike,
    kernel_size: int | tuple[int, int] = 5,
) -> cv2.typing.MatLike:
    """
    OpenCVのMORPH_CLOSEしているだけだが、頻繁に呼んでいるので関数化
    特にkernel_sizeが指定できて、大きく物体の輪郭を滑らかにしたりできる

    :param target_image: 対象画像
    :type target_image: cv2.typing.MatLike
    :param kernel_size: カーネルサイズ
    :type kernel_size: int | tuple[int, int]
    :return: 滑らかにした結果の画像
    :rtype: MatLike

    """
    if isinstance(kernel_size, int):
        _kernel = np.ones((kernel_size, kernel_size), np.uint8)
    else:
        _kernel = np.ones(kernel_size, np.uint8)
    # kernel = np.ones((kernel_size, kernel_size), np.uint8)
    return cv2.morphologyEx(target_image, cv2.MORPH_CLOSE, _kernel)


def min_max(x: NDSeries, axis: int | None = None) -> NDSeries:
    """[0,1]正規化

    :param x: 正規化対象
    :type x: NDSeries
    :param axis: 正規化する方向, defaults to None
    :type axis: int | None, optional
    :return: 正規化後のx
    :rtype: NDSeries
    """
    min = x.min(axis=axis, keepdims=True)
    max = x.max(axis=axis, keepdims=True)
    if np.allclose(max, min):
        # Remark: maxとminが近い場合は、一旦全部0にしているが、他の値にするのでも良いと思う
        return x - min
    return (x - min) / (max - min)


def thresh_based_edge2bin(
    edge_img: cv2.typing.MatLike,
    min_th: int = 150,
    bin_method: BinMethod = BinMethod.REL,
) -> NDImage:
    """閾値ベースでエッジ画像を二値画像に変換する
    画像の値の相対値を見る(BinMethod.REL)か、絶対値を見るか(BinMethod.ABS)で二値化方法は異なる

    :param edge_img: エッジ画像
    :type edge_img: cv2.typing.MatLike
    :param min_th: 二値化の閾値, defaults to 150
    :type min_th: int, optional
    :param bin_method: 二値化の方法, defaults to BinMethod.REL
    :type bin_method: BinMethod, optional
    :return: 二値化された画像
    :rtype: NDImage
    """

    match bin_method:
        case BinMethod.REL:
            # floatでないと二値化できないので、float変換している
            z_normalized_img = (min_max(edge_img) * 255).astype(float)
            _, binalized_img = cv2.threshold(
                z_normalized_img, min_th, 255, cv2.THRESH_BINARY
            )
        case BinMethod.ABS:
            binalized_img = (edge_img > min_th).astype(np.uint8) * 255

    # ラベリング
    kernel = np.ones((3, 3), np.uint8)
    # kernel = np.ones((5, 5), np.uint8)
    return cv2.morphologyEx(
        binalized_img,
        cv2.MORPH_CLOSE,
        kernel,
    ).astype(np.uint8)
