from __future__ import annotations

from typing import Any

import numpy as np
from numpy.typing import NDArray

from argus_synchro.config.app_config import SceneDescriptionConf


def calc_iou(
    ax_min: float,
    ay_min: float,
    ax_max: float,
    ay_max: float,
    bx_min: float,
    by_min: float,
    bx_max: float,
    by_max: float,
) -> tuple[float, float, float, float]:
    """
    aとbのiouを計算する関数
    """
    a_area = (ax_max - ax_min) * (ay_max - ay_min)
    b_area = (bx_max - bx_min) * (by_max - by_min)

    if (a_area == 0) and (b_area == 0):
        return 0, 0, 0, 0

    abx_min = max(ax_min, bx_min)
    aby_min = max(ay_min, by_min)
    abx_max = min(ax_max, bx_max)
    aby_max = min(ay_max, by_max)

    intersect = max(0, abx_max - abx_min) * max(0, aby_max - aby_min)
    iou = intersect / (a_area + b_area - intersect)

    return iou, intersect, a_area, b_area


# calibcheck2d3d用のシーン判定クラス
class Scene:
    """calibcheck2d3d評価用。2D-3D対応付けと人サイズゲート機能を提供。

    注意: 本クラスは暫定実装。後でargus_synchro_lib.scene.Sceneとの統合が必要。
    """

    def __init__(self, scene_conf: SceneDescriptionConf) -> None:
        # メンバ変数としてパラメータを定義
        self.coarse_lo: float = scene_conf.coarse_lo
        self.coarse_hi: float = scene_conf.coarse_hi
        self.k_min: float = scene_conf.k_min
        self.h_ref_px: int = scene_conf.h_ref_px
        self.lo_gain: float = scene_conf.lo_gain
        self.hi_gain: float = scene_conf.hi_gain
        self.lo_floor: float = scene_conf.lo_floor
        self.hi_ceil: float = scene_conf.hi_ceil
        self.vertical_w_iou: float = scene_conf.vertical_w_iou
        self.vertical_w_scale: float = scene_conf.vertical_w_scale
        self.vertical_w_phi: float = scene_conf.vertical_w_phi
        self.final_threshold: float = scene_conf.final_threshold
        self.use_human_gate: bool = scene_conf.use_human_gate
        self.H_min: float = scene_conf.H_min
        self.H_max: float = scene_conf.H_max
        self.W_min: float = scene_conf.W_min
        self.W_max: float = scene_conf.W_max
        self.D_min: float = scene_conf.D_min
        self.D_max: float = scene_conf.D_max
        self.tall_ratio_min: float = scene_conf.tall_ratio_min

    # -----------------------------------------------------------
    # 共通補助指標：縦方向の一致度を評価（俯瞰配置で前後の距離一致を重く評価する）
    #   - 3D投影BBox(8点)の縦範囲と、YOLO人BBoxの縦範囲を比較
    #   - ① 縦IoU（1D）… 前後の重なり度合い（大きいほど良い）
    #   - ② 縦スケール整合 … 縦の大きさ比 |h3d/h2d - 1|（小さいほど良い）
    #   - ③ 縦ベクトルの向き差 … 姿勢の見え方の差（rad、小さいほど良い）
    # 戻り値: (vIoU, height_ratio, phi)
    # ------------------------------------------------------------
    def vertical_consistency_scores(
        self,
        box2d: NDArray[np.float32],
        width: int,
        height: int,
        proj8: NDArray[np.float32],  # shape (8,2) の1クラスタ分
    ) -> tuple[float, float, float]:
        # YOLO人BBox（正規化→px）
        u1, v1 = float(box2d[1] * width), float(box2d[0] * height)
        u2, v2 = float(box2d[3] * width), float(box2d[2] * height)
        h2d = max(1.0, v2 - v1)

        # 3D投影BBox（8点）から2D縦範囲・縦方向ベクトルを作る
        ys = proj8[:, 1]
        vmin3d, vmax3d = float(np.min(ys)), float(np.max(ys))
        h3d = max(1.0, vmax3d - vmin3d)

        # 1D-vertical IoU（縦方向の重なり）
        inter = max(0.0, min(v2, vmax3d) - max(v1, vmin3d))
        den = (v2 - v1) + (vmax3d - vmin3d) - inter
        vIoU = (inter / den) if den > 0 else 0.0

        # 縦スケール整合（1に近いほど良い）
        height_ratio = abs((h3d / h2d) - 1.0)

        # 縦ベクトルの向き差：
        #   3D投影BBoxの「上面中心→下面中心」を2Dベクトルに
        top_c2d = np.mean(proj8[0:4, :], axis=0)
        bot_c2d = np.mean(proj8[4:8, :], axis=0)
        v2d_j = bot_c2d - top_c2d
        if np.linalg.norm(v2d_j) < 1e-6:
            phi = np.pi / 2  # ほぼ無効な場合は90度相当として扱う
        else:
            # 画像の“鉛直”は (0, +h2d) とみなす（ロールがあるなら重力の画像投影に差し替え可）
            v2d_h = np.array([0.0, h2d], dtype=np.float32)
            a = v2d_j / np.linalg.norm(v2d_j)
            b = v2d_h / np.linalg.norm(v2d_h)
            cosang = float(np.clip(np.dot(a, b), -1.0, 1.0))
            phi = float(np.arccos(cosang))  # [rad] 小さいほど縦方向の見え方が近い

        return (
            vIoU,
            float(height_ratio),
            float(phi),
        )

    # ------------------------------------------------------------
    # 共通ヘルパ：方式固有の base コストに“縦の一致”ペナルティを加える
    #   cost = base + w1*(1-vIoU) + w2*height_ratio + w3*phi
    # ------------------------------------------------------------
    def augment_cost_with_verticals(
        self,
        base_cost: float,
        vIoU: float,
        height_ratio: float,
        phi_rad: float,
        w1: float | None = None,
        w2: float | None = None,
        w3: float | None = None,
    ) -> float:
        w1 = self.vertical_w_iou
        w2 = self.vertical_w_scale
        w3 = self.vertical_w_phi
        return float(base_cost + w1 * (1.0 - vIoU) + w2 * height_ratio + w3 * phi_rad)

    # ------------------------------------------------------------
    # 共通ヘルパ：軽量プリゲート（候補の早刈り）
    #   - h3d/h2d が極端にズレるものはそもそも評価しない
    #   - 遠方・小BBOXでは緩める（画像高さpxに依存して自動調整）
    # ------------------------------------------------------------
    def passes_coarse_height_gate(
        self,
        box2d: NDArray[np.float32],
        width: int,
        height: int,
        proj8: NDArray[np.float32],
        lo: float | None = None,
        hi: float | None = None,
    ) -> bool:
        # 2D人BBox高さ(px)
        u1, v1 = float(box2d[1] * width), float(box2d[0] * height)
        u2, v2 = float(box2d[3] * width), float(box2d[2] * height)
        h2d = max(1.0, v2 - v1)

        # 3D投影BBoxの縦サイズ(px)
        ys = proj8[:, 1]
        h3d = max(1.0, float(np.max(ys) - np.min(ys)))
        r = h3d / h2d

        # [SceneDesc] から取得（無ければ既定値）
        if lo is None:
            lo = self.coarse_lo
        if hi is None:
            hi = self.coarse_hi

        # 遠方・小BBoxで厳しすぎないよう緩和係数を適用（k∈[k_min,1]）
        k_min = self.k_min
        h_ref = self.h_ref_px
        lo_gain = self.lo_gain
        hi_gain = self.hi_gain
        lo_floor = self.lo_floor
        hi_ceil = self.hi_ceil

        k = max(k_min, min(1.0, h2d / h_ref))  # 小さい箱ほど k が小さい
        lo_eff = max(lo_floor, lo * k * lo_gain)
        hi_eff = min(hi_ceil, hi / k * hi_gain)
        return lo_eff <= r <= hi_eff

    # ------------------------------------------------------------
    # 共通ヘルパ：本ゲート（誤対応の強制抑止）※関数は残置（呼出しは無効化可能）
    #   - 縦IoU / 縦スケール整合 / 縦向き
    # ------------------------------------------------------------
    def passes_vertical_hard_gate(
        self,
        vIoU: float,
        height_ratio: float,
        phi_rad: float,
        *,
        box_h_px: float | None = None,
        vIoU_min: float = 0.30,
        height_ratio_max: float = 0.60,
        phi_max_deg: float = 8.0,
    ) -> bool:
        # 必要なら「小さい箱に優しい」適応も利用可（現状は呼び出し側で未使用）
        if box_h_px is not None:
            k = max(0.3, min(1.0, box_h_px / 80.0))
            vIoU_min = max(0.05, vIoU_min * k)
            height_ratio_max = min(1.50, height_ratio_max / k)
            phi_max_deg = min(40.0, phi_max_deg + (1.0 - k) * 32.0)

        return (
            (vIoU >= vIoU_min)
            and (height_ratio <= height_ratio_max)
            and (phi_rad <= np.deg2rad(phi_max_deg))
        )

    # ------------------------------------------------------------
    # 方式ディスパッチ
    # ------------------------------------------------------------
    def get_human_3bb(
        self,
        box2d: NDArray[np.float32],
        width: int,
        height: int,
        box3ds: NDArray[Any],
        num_3d: int,
        method: str = "center",
    ) -> int:
        """
        box2d:YoloのBB、[image_h_min, image_w_min, image_h_max, image_w_max](0~1で正規化された位置)
        width:画像の幅
        height:画像の高さ
        box3ds:立体物数*[x, y]*8点が1列に並んでいる
        cat3ds:立体物のクラス、0:人, それ以外も判別はされている
        num_3d:3dbbの数
        返り値:box2dに最も近いbox3dのインデックス、条件に合うものがない場合は-1
        """
        if method == "center":
            return self.correspondence_by_center(box2d, width, height, box3ds, num_3d)
        elif method == "iou":
            return self.correspondence_by_iou(box2d, width, height, box3ds, num_3d)
        elif method == "endpoints":
            return self.correspondence_by_endpoints(
                box2d, width, height, box3ds, num_3d
            )
        else:
            raise ValueError(
                f"method should be 'center' or 'iou', current method = {method}"
            )

    # ------------------------------------------------------------
    # 手法１：端点距離
    # ------------------------------------------------------------
    def correspondence_by_endpoints(
        self,
        box2d: NDArray[np.float32],
        width: int,
        height: int,
        box3ds: NDArray[Any],
        num_3d: int,
    ) -> int:
        """
        2dと3dのbounding boxの端点の近さで選ぶ
        """
        box2d_min: NDArray[np.float64] = np.array((box2d[1] * width, box2d[0] * height))
        box2d_max: NDArray[np.float64] = np.array((box2d[3] * width, box2d[2] * height))
        best_idx, best_cost = -1, 1e9
        for j in range(num_3d):
            box_3d_in_2d = box3ds[(j * 8 + 4) : (j * 8 + 8)]
            box3d_max = box_3d_in_2d.max(axis=0)
            box3d_min = box_3d_in_2d.min(axis=0)
            base = 0.5 * (
                np.linalg.norm(box3d_min - box2d_min)
                + np.linalg.norm(box3d_max - box2d_max)
            )
            # 縦整合のペナルティを加点
            proj8 = np.zeros((8, 2), dtype=np.float32)
            for k in range(8):
                proj8[k, 0] = box3ds[j * 8 + k][0]
                proj8[k, 1] = box3ds[j * 8 + k][1]
            vIoU, hratio, phi = self.vertical_consistency_scores(
                box2d, width, height, proj8
            )
            cost = self.augment_cost_with_verticals(base, vIoU, hratio, phi)
            if cost < best_cost:
                best_cost, best_idx = cost, j
        return best_idx

    # ------------------------------------------------------------
    # 手法２：IoUベース
    # ------------------------------------------------------------
    def correspondence_by_iou(
        self,
        box2d: NDArray[np.float32],
        width: int,
        height: int,
        box3ds: NDArray[Any],
        num_3d: int,
    ) -> int:
        """
        2D IoU 最大のものを選ぶ方式。
        俯瞰配置での前後取り違え/近距離誤吸着を抑制するため、
        - 粗い縦スケールゲート
        - （必要なら）縦の強制ゲート
        - 上記をペナルティ加点して総合コスト化
        """
        if num_3d == 0 or len(box3ds) == 0:
            return -1

        best_idx = -1
        best_cost = 1e9

        # 人BBox（px）
        box2d_xy = (
            box2d[1] * width,
            box2d[0] * height,
            box2d[3] * width,
            box2d[2] * height,
        )

        for j in range(num_3d):
            proj8 = np.zeros((8, 2), dtype=np.float32)
            for k in range(8):
                proj8[k, 0] = box3ds[j * 8 + k][0]
                proj8[k, 1] = box3ds[j * 8 + k][1]

            # 早刈り
            if not self.passes_coarse_height_gate(box2d, width, height, proj8):
                continue

            vIoU, hratio, phi = self.vertical_consistency_scores(
                box2d, width, height, proj8
            )

            # （必要なら）強制ゲートを戻せる（デフォルトは使わない）
            # box_h_px = float((box2d[2] - box2d[0]) * height)
            # if not self.passes_vertical_hard_gate(vIoU, hratio, phi, box_h_px=box_h_px):
            #     continue

            # 3D投影BBoxの2D矩形（下面4点で安定）
            max_pos = proj8[4:8, :].max(axis=0)
            min_pos = proj8[4:8, :].min(axis=0)
            box3d_xy = (min_pos[0], min_pos[1], max_pos[0], max_pos[1])

            iou = calc_iou(*box2d_xy, *box3d_xy)[0]
            base = 1.0 - float(iou)  # baseコストは (1 - IoU)

            cost = self.augment_cost_with_verticals(base, vIoU, hratio, phi)

            # 任意：最終保険ゲート（設定にあれば有効）
            final_th = self.final_threshold
            if cost >= final_th:
                continue

            if cost < best_cost:
                best_cost = cost
                best_idx = j

        return best_idx

    # ------------------------------------------------------------
    # 手法３：重心（下辺中心）ベース
    # ------------------------------------------------------------
    def correspondence_by_center(
        self,
        box2d: NDArray[np.float32],
        width: int,
        height: int,
        box3ds: NDArray[Any],
        num_3d: int,
    ) -> int:
        """
        重心（下辺中心）距離ベースで2Dと3Dを対応付け。
        俯瞰配置での前後取り違え/近距離誤吸着を抑えるため、
        - 粗い縦スケールゲート
        - （必要なら）縦の強制ゲート
        - 上記をペナルティ加点して総合コスト化
        """
        if num_3d == 0 or len(box3ds) == 0:
            return -1

        best_idx = -1
        best_cost = 1e9

        # 人BBoxの下辺中心（接地点相当, px）
        bottom_x_2d = (box2d[3] + box2d[1]) * width * 0.5
        bottom_y_2d = box2d[2] * height
        bottom_2d = np.array([bottom_x_2d, bottom_y_2d], dtype=np.float32)

        for j in range(num_3d):
            proj8 = np.zeros((8, 2), dtype=np.float32)
            for k in range(8):
                proj8[k, 0] = box3ds[j * 8 + k][0]
                proj8[k, 1] = box3ds[j * 8 + k][1]

            # 早刈り
            if not self.passes_coarse_height_gate(box2d, width, height, proj8):
                continue

            # 縦方向のサイズで前チェック。厳しすぎる場合はコメントアウトでもいい
            vIoU, hratio, phi = self.vertical_consistency_scores(
                box2d, width, height, proj8
            )
            # box_h_px = float((box2d[2] - box2d[0]) * height)
            # if not Scene._passes_vertical_hard_gate(vIoU, hratio, phi, box_h_px=box_h_px):
            #     continue

            # 3D側の「下面中心」（px）：median の方が遠方ノイズに強い
            bottom_c2d = np.median(proj8[4:8, :], axis=0)

            # baseコスト：下辺中心と下面中心の距離（px）
            base = float(np.linalg.norm(bottom_2d - bottom_c2d))

            cost = self.augment_cost_with_verticals(base, vIoU, hratio, phi)

            # 任意：最終保険ゲート（設定にあれば有効）
            # final_th = self.final_threshold
            # if cost >= final_th:
            #     continue

            if cost < best_cost:
                best_cost = cost
                best_idx = j

        return best_idx

    def passes_human_size(self, minmax_tuple) -> bool:
        """人サイズゲート。3D bboxの寸法が人らしいかを判定。

        Args:
            minmax_tuple: (x_min, x_max, y_min, y_max, z_min, z_max)

        Returns:
            bool: 人サイズ範囲内ならTrue
        """
        try:
            x_min, x_max, y_min, y_max, z_min, z_max = minmax_tuple
        except Exception:
            return True  # 情報が無いときは通す（必要ならログ）
        H = float(z_max - z_min)  # 身長方向
        W = float(x_max - x_min)
        D = float(y_max - y_min)

        # 身長チェック
        if not (self.H_min <= H <= self.H_max):
            return False

        # 幅（W, D）チェック
        if not (self.W_min <= W <= self.W_max) or not (self.D_min <= D <= self.D_max):
            return False

        # 縦長比（人らしさ）チェック
        if (H / max(W, D, 1e-6)) < self.tall_ratio_min:
            return False

        return True
