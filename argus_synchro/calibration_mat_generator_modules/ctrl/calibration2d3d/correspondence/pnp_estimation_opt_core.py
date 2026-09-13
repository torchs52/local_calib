import copy
from typing import Any, Optional

import cv2
import numpy as np
from numpy.typing import NDArray
from scipy.optimize import least_squares

glob_optcount = 0


def pnp_residuals(
    x: NDArray[np.float64],
    object_points: NDArray[np.float64],  # (N,3), float32/float64
    image_points: NDArray[np.float64],  # (N,2), float32/float64
    camera_matrix: NDArray[np.float64],  # (3,3)
    dist_coeffs: NDArray[np.float64] | None,  # (k,), None でも可
    # ペナルティ設定
    r_fixedtgt_flag: int = 0,  # 3bit: bit0=x, bit1=y, bit2=z
    t_fixedtgt_flag: int = 0,  # 同上
    r_target: NDArray[np.float64] | None = None,  # (3,), ロドリゲス角度[rad]
    t_target: NDArray[np.float64] | None = None,  # (3,), 平行移動[m]
    lambda_r: NDArray[np.float64]
    | None = None,  # (3,), 軸別ペナルティ強度（ピクセル相当への重み付け）
    lambda_t: NDArray[np.float64] | None = None,  # (3,), 軸別ペナルティ強度
    # オプション
    normalize_imagesize=Optional[
        NDArray[np.int32]
    ],  # 画像サイズに応じたスケール調整 Noneで無効化
) -> NDArray[np.float64]:
    global glob_optcount
    """
    戻り値：1次元 residual ベクトル
    前半：再投影誤差（各2D点の x/y 誤差を1つにまとめる or 軸別で2つ入れる）
    後半：rvec/tvecの軸別ペナルティ残差（フラグに応じてのみ追加）
    """

    if normalize_imagesize is None:
        normalize_imagesize = np.ones(2)
    else:
        normalize_imagesize = np.array(normalize_imagesize)

    # パラメータ分解
    rvec = x[:3].reshape(3, 1)
    tvec = x[3:].reshape(3, 1)

    # 再投影
    projected, _ = cv2.projectPoints(
        object_points, rvec, tvec, camera_matrix, dist_coeffs
    )
    projected = projected.reshape(-1, 2)

    # ここでは x/y の2軸それぞれの residual を入れます（ベクトル長は 2N）
    # （各点のx誤差とy誤差を別々の要素にしておくと、数値的により情報量が増えます）
    reproj_residuals = (
        (projected - image_points) / normalize_imagesize
    ).ravel()  # shape=(2N,)

    residuals = [reproj_residuals]

    # ペナルティのためのデフォルト設定
    if r_target is None:
        r_target = np.zeros(3, dtype=float)
    if t_target is None:
        t_target = np.zeros(3, dtype=float)
    if lambda_r is None:
        lambda_r = np.zeros(3, dtype=float)  # 0ならペナルティ無し相当
    if lambda_t is None:
        lambda_t = np.zeros(3, dtype=float)

    # rvec / tvec の軸別ペナルティ残差を追加
    rvec_flat = rvec.flatten()
    tvec_flat = tvec.flatten()

    # ビットが立っている軸だけ残差を追加
    scale_factor = np.sqrt(2 * len(object_points))  # 2Nはx/y誤差分
    for i, bit in enumerate([1, 2, 4]):  # x->1, y->2, z->4
        if (r_fixedtgt_flag & bit) != 0:
            # 係数 * 差分 を residual に追加（二乗は least_squares 側で行われる）
            residuals.append(lambda_r[i] * (rvec_flat[i] - r_target[i]) * scale_factor)
        # 立っていない場合は追加しない（=ペナルティ無し）

    for i, bit in enumerate([1, 2, 4]):
        if (t_fixedtgt_flag & bit) != 0:
            residuals.append(lambda_t[i] * (tvec_flat[i] - t_target[i]) * scale_factor)

    glob_optcount += 1

    # 1次元ベクトルに結合
    residuals_vec = np.concatenate(
        [r if isinstance(r, np.ndarray) else np.array([r]) for r in residuals]
    )
    return residuals_vec


"""r_fixedtgt_flag=0,      # 3bit: bit0=x, bit1=y, bit2=z
    t_fixedtgt_flag=0,      # 同上
    r_target=None,          # (3,), ロドリゲス角度[rad]
    t_target=None,          # (3,), 平行移動（PnPの単位に合わせる）
    lambda_r=None,          # (3,), 軸別ペナルティ強度（ピクセル相当への重み付け）
    lambda_t=None,          # (3,), 軸別ペナルティ強度
    # オプション
    normalize_pixel=True,   # 画像サイズに応じたスケール調整をするなら拡張用"""


def solvePnP_opt_leastsq(
    objectPoints: NDArray[np.float64],
    imagePoints: NDArray[np.float64],
    cameraMatrix: NDArray[np.float64],
    distCoeffs: NDArray[np.float64],
    rvec: NDArray[np.float64],
    tvec: NDArray[np.float64],  # rvec/tvec:初期値。
    # ペナルティ設定
    r_fixedtgt_flag: int = 0,  # 3bit: bit0=x, bit1=y, bit2=z
    t_fixedtgt_flag: int = 0,  # 同上
    r_target: NDArray[np.float64] | None = None,  # (3,), ロドリゲス角度[rad]
    t_target: NDArray[np.float64]
    | None = None,  # (3,), 平行移動（PnPの単位に合わせる）
    lambda_r: NDArray[np.float64]
    | None = None,  # (3,), 軸別ペナルティ強度（ピクセル相当への重み付け）
    lambda_t: NDArray[np.float64] | None = None,  # (3,), 軸別ペナルティ強度
    # オプション
    normalize_imagesize: NDArray[np.int32]
    | None = None,  # 画像サイズに応じたスケール調整 Noneで無効化、(2,): 画像サイズ(imagePoints座標と同じxy順序)
    debug_dict: dict[str, Any] | None = None,
) -> tuple[bool, NDArray[np.float64], NDArray[np.float64]]:
    init_params = np.hstack([rvec.flatten(), tvec.flatten()])

    # 最適化
    result = least_squares(
        pnp_residuals,
        init_params,
        args=(
            objectPoints,
            imagePoints,
            cameraMatrix,
            distCoeffs,
            r_fixedtgt_flag,
            t_fixedtgt_flag,
            r_target,
            t_target,
            lambda_r,
            lambda_t,
            normalize_imagesize,
        ),
    )
    optimized_rvec = result.x[:3].reshape(3, 1)
    optimized_tvec = result.x[3:].reshape(3, 1)
    if debug_dict is not None:
        debug_dict["least_squares_result"] = copy.copy(result)
    return True, optimized_rvec, optimized_tvec
