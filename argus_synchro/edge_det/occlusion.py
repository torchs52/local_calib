"""
オクルージョン対策に関する処理を入れておくモジュール
"""

import cv2
import numpy as np

from argus_synchro.common.common import NDPoint2i, NDSeries, is_in_interval
from argus_synchro.edge_det.transform import polar_grid_to_grid
from argus_synchro.edge_det.typedef import PxPyTup, XYTup


def mask_img_by_value(
    img: cv2.typing.MatLike,
    low_value: float = 50,
    high_value: float = 200,
) -> cv2.typing.MatLike:
    """
    画像の値を基準としてマスクを出力する
    low_valueより小さいものは-1, high_valueより大きいものは1として、それ以外は0にする

    :param img: 鳥観図
    :type img: cv2.typing.MatLike
    :param low_value: 値が小さいと判定される閾値
    :type low_value: float
    :param high_value: 値が大きいと判定される閾値
    :type high_value: float
    :return: 値が大きい部分は1, 小さい部分は-1, それ以外は0が入ったimgと同じ大きさの行列
    :rtype: MatLike
    """
    return np.where(
        img < low_value,
        -1,
        np.where(img > high_value, 1, 0),
    )


def mask_img_by_gradient(
    filtered_img: cv2.typing.MatLike, grad_strength: float = 30
) -> cv2.typing.MatLike:
    """
    画像の勾配に基づいて、各ピクセルの勾配方向をマスクとして出力する

    この関数は、入力された2次元配列（画像の勾配を表す）の各ピクセルに対してマスク処理を行う
    勾配値がgrad_strength以下のピクセルは-1としてマークされ、強い負の勾配を示す
    勾配値がgrad_strength以上のピクセルは1としてマークされ、強い正の勾配を示す
    その他のピクセルは0としてマークされ、勾配が弱いまたは無いことを示す

    パラメータ:
    - filtered_img: 勾配値の2次元numpy配列

    戻り値:
    - masked_img: 勾配方向に基づいて、各要素が-1、0、または1になっている2次元numpy配列
    """
    masked_img = np.where(
        filtered_img <= -1 * grad_strength,
        -1,
        np.where(filtered_img >= grad_strength, 1, 0),
    )
    return masked_img


def check_occlusion_by_col(
    masked_img: cv2.typing.MatLike,
    max_radius: NDSeries,
    row_range: int = 3,
    n_happen: int = 1,
) -> cv2.typing.MatLike:
    """
    画像の勾配方向マスクに基づいてオクルージョンをチェックする

    この関数は、マスクされた画像の各ピクセルをスキャンし、行方向の上下指定されたピクセル範囲内に異なる符号のピクセルが存在するかどうかをチェックする
    そのようなピクセルが見つかった場合、オクルージョンとみなされ、出力配列の対応する列はTrueとしてマークする

    パラメータ:
    - masked_img: 勾配方向マスクを表す-1、0、または1の各要素を持つ2次元numpy配列
    - max_radius: 各列の最大検出範囲
    - row_range: 行方向のチェック範囲(ピクセル数)
    - n_happen: 1つの列に対して、何回該当のピクセルが見つかるとTrue判定するか

    戻り値:
    - output: オクルージョンを示すTrueの位置を持つ1次元のブール型numpy配列
    """
    rows, cols = masked_img.shape
    output = np.zeros(cols, dtype=bool)

    for col in range(cols):
        max_row = max_radius[col] + 1

        for row in range(max_row):
            center = masked_img[row, col]
            if center == 0:
                continue

            # 中心ピクセルを囲むウィンドウを取得
            cnt = 0
            end = min(row + row_range + 1, max_row)
            for r in range(row, end):
                if masked_img[r, col] * center == -1:
                    cnt += 1
                    if cnt >= n_happen:
                        output[col] = True
                        break

            if output[col]:
                break
    return output


def check_occlusion(
    masked_img: cv2.typing.MatLike,
    pixel_range: int = 3,
) -> cv2.typing.MatLike:
    """
    画像の勾配方向マスクに基づいてオクルージョンをチェックする

    この関数は、マスクされた画像の各ピクセルをスキャンし、y軸方向の上下指定されたピクセル範囲内に異なる符号のピクセルが存在するかどうかをチェックする
    そのようなピクセルが見つかった場合、オクルージョンとみなされ、出力配列の対応する位置はTrueとしてマークする

    パラメータ:
    - masked_img: 勾配方向マスクを表す-1、0、または1の各要素を持つ2次元numpy配列
    - pixel_range: y軸方向のチェック範囲(ピクセル数)

    戻り値:
    - output: オクルージョンを示すTrueの位置を持つ2次元のブール型numpy配列
    """
    arr = masked_img
    output = np.full(arr.shape, False, dtype=bool)
    height, width = arr.shape
    # for y in range(height):
    for y in range(pixel_range, height - pixel_range):
        for x in range(pixel_range, width):
            if arr[y, x] == 0:
                continue
            # 中心ピクセルを囲むウィンドウを取得
            window = arr[max(0, y - pixel_range) : min(height, y + pixel_range + 1), x]
            # 中心ピクセルと異なる符号を持つピクセルが存在するかチェック
            if np.any(window * arr[y, x] == -1):
                output[y, x] = True
    return output


def apply_occlusion_mask(filtered_img: cv2.typing.MatLike) -> cv2.typing.MatLike:
    """
    勾配方向マスクに基づいて画像にオクルージョンマスクを適用する

    この関数はまず入力画像に勾配方向マスクを適用し、次にオクルージョンをチェックする
    オクルージョンと識別されたピクセルは出力画像で0に設定される

    パラメータ:
    - filtered_img: 処理対象の2次元numpy配列の画像

    戻り値:
    - result_img: オクルージョンに基づいて更新された画像を表す2次元numpy配列
    """
    masked_img = mask_img_by_gradient(filtered_img)
    occlusion_mask = check_occlusion(masked_img)
    return np.where(occlusion_mask, 0, masked_img)


def check_occlusion_from_other_origin(
    masked_img: cv2.typing.MatLike,
    max_radius: NDSeries,
    other_origin: XYTup,
    origin: XYTup,
    grid_size: XYTup,
    grid_offset: PxPyTup = (0, 0),
    real_offset: XYTup = (0.0, 0.0),
    focus_range: int = 3,
    n_happen: int = 1,
) -> list[NDPoint2i]:
    """
    画像の勾配方向マスクに基づいてオクルージョンをチェックする

    この関数は、マスクされた画像の各ピクセルをスキャンし、あるother_originを原点とする極座標上の行方向の中で異なる符号のピクセルが存在するかどうかをチェックする
    そのようなピクセルが見つかった場合はオクルージョンとして、それに対応するoriginを原点とする極座標上の位置をリストに入れる

    パラメータ:
    - masked_img: 勾配方向マスクを表す-1、0、または1の各要素を持つ2次元numpy配列
    - max_radius: originの極座標における各角度に対する最大の動径方向の値
    - other_origin: オクルージョン検出を行うときの原点の位置 = あるLiDARの点群の座標系における位置
    - grid_size: 極座標の格子サイズ
    - grid_offset: 格子座標から実座標に変換する手前でどれだけオフセットを載せるか
    - real_offset: 実座標に変換後どれだけオフセットを載せるか
    - row_range: 行方向のチェック範囲(ピクセル数)
    - n_happen: 1つの列に対して、何回該当のピクセルが見つかるとTrue判定するか

    戻り値:
    - output: originの鳥瞰図におけるオクルージョンに該当する格子座標のリスト, リストの各要素がother_originにおいて角度を固定して動径方向に変化した時の位置になっていて、リストの各要素の最後の要素がオクルージョンの境界になっているはず
    """
    rows, cols = masked_img.shape

    occ_area: list[NDPoint2i] = []
    for theta in range(cols):
        # あるLiDAR中心における, (radius, theta)の格子座標を取得
        lidar_grid_coords = np.stack(
            (np.arange(rows), np.ones(rows) * theta),
            axis=1,
        ).astype(int)

        # あるLiDAR中心における格子座標を実座標に変換する
        target_grid_coords = polar_grid_to_grid(
            from_grid_coords_2d=lidar_grid_coords,
            from_origin=other_origin,
            to_origin=origin,
            grid_size=grid_size,
            grid_offset=grid_offset,
            real_offset=real_offset,
        )

        # 範囲内の格子座標に絞る
        detect_inds = is_in_interval(target_grid_coords, (0, rows), (0, cols))
        target_grid_coords_detect = target_grid_coords[detect_inds]

        # radiusに対する検出範囲内の点群だけ取り出す
        in_radius_inds = (
            target_grid_coords_detect[:, 0]
            < max_radius[target_grid_coords_detect[:, 1]]
        )
        target_grid_coords_in_radius = target_grid_coords_detect[in_radius_inds]

        strips_in_src_origin = masked_img[
            target_grid_coords_in_radius[:, 0], target_grid_coords_in_radius[:, 1]
        ]

        # あるLiDAR座標の角度thetaにおける画素値の一覧がstrips_in_src_originに入っているので、これを使ったオクルージョン判定を行う
        n_focus = len(strips_in_src_origin)
        for ind in range(n_focus):
            center = strips_in_src_origin[ind]
            if center == 0:
                continue

            # 中心ピクセルを囲むウィンドウを取得
            cnt = 0
            end = min(ind + focus_range + 1, n_focus)
            for r in range(ind, end):
                if strips_in_src_origin[r] * center == -1:
                    cnt += 1
                    if cnt >= n_happen:
                        occ_area.append(target_grid_coords_in_radius)
                        break

            if cnt >= n_happen:
                break
    return occ_area
