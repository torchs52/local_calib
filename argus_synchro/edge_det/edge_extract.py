"""
崖検出の中でエッジ抽出を行う処理が入ったモジュール
"""

import cv2
import numpy as np

from argus_synchro.edge_det.const import ComplementMissingMethod, EdgeFilterType


def bev_masking(
    high_low_img: cv2.typing.MatLike,
    polar_bev: cv2.typing.MatLike,
    completed_value: int = 1,
    completing_value: int = 0,
    complement_method: ComplementMissingMethod = ComplementMissingMethod.MEDIAN,
    is_copy: bool = True,
) -> cv2.typing.MatLike:
    """
    鳥観図で値が大きい領域をそうでない領域の値で補完して、補完後の鳥瞰図を返す

    :param high_low_img: 鳥観図の各位置の値の大きさを表現したNDArray, 基本的に{-1, 0, 1}を値として持つような行列になっている想定
    :type high_low_img: NDArray
    :param polar_bev: 鳥観図のNDArray
    :type polar_bev: NDArray
    :param completed_value: high_low_imgのどの値を持つ領域を補完するか
    :type completed_value: int
    :param completing_value: high_low_imgのどの値を持つ領域で補完するか
    :type completing_value: int
    :param complement_method: どういった方法で鳥瞰図から補完するか
    :type complement_method: ComplementMissingMethod
    :param is_copy: 戻り値はpolar_bevと別のメモリを参照したものにするか
    :type is_copy: bool
    :return: 値が大きい領域を補完した鳥瞰図
    :rtype: Any
    """

    _polar_bev = polar_bev.copy() if is_copy else polar_bev

    completing_row_col = np.where(high_low_img == completing_value)
    completing_vals = polar_bev[completing_row_col[0], completing_row_col[1]]
    match complement_method:
        case ComplementMissingMethod.MEDIAN:
            completed_val = np.median(completing_vals)
        case ComplementMissingMethod.MEAN:
            completed_val = np.mean(completing_vals)
        case ComplementMissingMethod.MIN:
            completed_val = np.min(completing_vals)
        case ComplementMissingMethod.MAX:
            completed_val = np.max(completing_vals)

    completed_row_col = np.where(high_low_img == completed_value)
    _polar_bev[completed_row_col[0], completed_row_col[1]] = completed_val

    return _polar_bev


def extract_edge(
    img: cv2.typing.MatLike,
    im_filter: EdgeFilterType,
    *,
    ksize: int | None = None,
    dx: int = 0,
    dy: int = 1,
    is_scharr: bool = False,
) -> cv2.typing.MatLike:
    """画像からエッジ画像を生成する

    :param img: エッジ画像を作る元の画像
    :type img: cv2.typing.MatLike
    :param im_filter: filterの種類
    :type im_filter: EdgeFilterType
    :param ksize: SOBEL, LAPLACIANのパラメータ, カーネルサイズ, defaults to None
    :type ksize: int | None, optional
    :param dx: SOBELのdx, defaults to 0
    :type dx: int, optional
    :param dy: SOBELのdy, defaults to 1
    :type dy: int, optional
    :return: エッジ画像
    :rtype: cv2.typing.MatLike
    """
    _ksize = 3 if ksize is None else ksize

    def _fixed_ksize_sobel(
        img: cv2.typing.MatLike, dx: int, dy: int
    ) -> cv2.typing.MatLike:
        return cv2.Sobel(img, cv2.CV_32F, dx, dy, ksize=_ksize)

    def _fixed_sharr(img: cv2.typing.MatLike, dx: int, dy: int) -> cv2.typing.MatLike:
        return cv2.Scharr(img, cv2.CV_32F, dx, dy)

    # Scharrフィルタでエッジフィルタをするかどうかの判定
    deriv_filter = _fixed_sharr if is_scharr and _ksize == 3 else _fixed_ksize_sobel

    match im_filter:
        case EdgeFilterType.SOBEL:
            return deriv_filter(img, dx, dy)
        case EdgeFilterType.SOBEL_X_FORWARD:
            return -1 * deriv_filter(img, 0, 1)
        case EdgeFilterType.SOBEL_X_BACKWARD:
            return deriv_filter(img, 0, 1)
        case EdgeFilterType.SOBEL_Y_FORWARD:
            return -1 * deriv_filter(img, 1, 0)
        case EdgeFilterType.SOBEL_Y_BACKWARD:
            return deriv_filter(img, 1, 0)
        case EdgeFilterType.LAPLACIAN:
            return cv2.Laplacian(img, cv2.CV_64F, ksize=_ksize)
        case EdgeFilterType.DoG:
            return apply_DoG_filter(img)


def apply_DoG_filter(
    img: cv2.typing.MatLike,
    size: int = 3,
    sigma: float = 2.0,
    k: float = 1.3,
    gamma: float = 1,
) -> cv2.typing.MatLike:
    """Difference of Gaussian filterによるフィルタ処理を行う

    :param img: _description_
    :type img: cv2.typing.MatLike
    :param size: _description_, defaults to 3
    :type size: int, optional
    :param sigma: _description_, defaults to 2.0
    :type sigma: float, optional
    :param k: _description_, defaults to 1.3
    :type k: float, optional
    :param gamma: _description_, defaults to 1
    :type gamma: float, optional
    :return: _description_
    :rtype: cv2.typing.MatLike
    """
    g1 = cv2.GaussianBlur(img, (size, size), sigma)
    g2 = cv2.GaussianBlur(img, (size, size), sigma * k)
    return g1 - gamma * g2
