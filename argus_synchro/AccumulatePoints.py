from __future__ import annotations

from collections import deque

import numpy as np
from numpy.typing import NDArray

from argus_synchro.config.app_config import (
    AccumulationConf,
    GeneralConf,
    LidarGridConf,
)
from argus_synchro.point_processing import utils


def filter_lidar_points(
    xyz: NDArray[np.float64],
    lidar_grid: LidarGridConf,
    general_conf: GeneralConf,
) -> tuple[NDArray[np.float64], NDArray[np.float64]]:
    """
    1. XY 範囲 (`fwd_min/max`, `side_min/max`) によるマスクを作成する。
    2. 範囲内点群に対して `ground_height + ground_height_margin` で上下分割する。
    3. 上側点群と下側点群の配列を返す。
        Returns:
      - points_mid:  zが-0.95以上かつ500m以下の点群
      - points_low:  zが-0.95以下の点群
    """

    # 1) X/Yフィルタ([-15,15]範囲内)
    mask_xy: NDArray[np.bool_] = (
        (xyz[:, 0] >= lidar_grid.fwd_min)
        & (xyz[:, 0] <= lidar_grid.fwd_max)
        & (xyz[:, 1] >= lidar_grid.side_min)
        & (xyz[:, 1] <= lidar_grid.side_max)
    )
    xyz_xy: NDArray[np.float64] = xyz[mask_xy]

    # 2) Z軸でマスクを2つ作成
    mask_upper: NDArray[np.bool_] = xyz_xy[:, 2] > (
        general_conf.ground_height + general_conf.ground_height_margin
    )
    mask_lower: NDArray[np.bool_] = ~mask_upper

    points_upper: NDArray[np.float64] = xyz_xy[mask_upper]
    points_lower: NDArray[np.float64] = xyz_xy[mask_lower]

    return points_upper, points_lower


def lidar_to_grid_map(
    points: NDArray[np.float64],
    grid_size: tuple[float, float],
    side_range: tuple[float, float],
    fwd_range: tuple[float, float],
) -> NDArray[np.uint8]:
    x_lidar: NDArray[np.float64] = points[:, 0]
    y_lidar: NDArray[np.float64] = points[:, 1]

    # LiDAR 座標をグリッドマップ座標に変換
    # グリッドマップ座標を範囲内にシフト
    x_img: NDArray[np.int32] = (
        (y_lidar / grid_size[0]) - (side_range[0] / grid_size[0])
    ).astype(np.int32)
    y_img: NDArray[np.int32] = (
        (x_lidar / grid_size[1]) - (fwd_range[0] / grid_size[1])
    ).astype(np.int32)

    # グリッドマップの初期化
    x_max: int = int(np.ceil((side_range[1] - side_range[0]) / grid_size[0]))
    y_max: int = int(np.ceil((fwd_range[1] - fwd_range[0]) / grid_size[1]))
    grid_map: NDArray[np.uint8] = np.zeros((y_max, x_max), dtype=np.uint8)

    # 範囲内の座標に対してのみグリッドマップを更新
    valid_indices: NDArray[np.bool_] = (
        (x_img >= 0) & (x_img < x_max) & (y_img >= 0) & (y_img < y_max)
    )
    grid_map[y_img[valid_indices], x_img[valid_indices]] = 1

    return grid_map


def compare_in_grid_space_vectorized(
    grid_map: NDArray[np.float64],
    original_points: NDArray[np.float64],
    grid_size: tuple[float, float],
    side_range: tuple[float, float],
    fwd_range: tuple[float, float],
) -> NDArray[np.float64]:
    x_lidar: NDArray[np.float64] = original_points[:, 0]
    y_lidar: NDArray[np.float64] = original_points[:, 1]

    # 座標変換
    x_img: NDArray[np.int32] = np.floor(
        (y_lidar - side_range[0]) / grid_size[0],
    ).astype(np.int32)
    y_img: NDArray[np.int32] = np.floor((x_lidar - fwd_range[0]) / grid_size[1]).astype(
        np.int32,
    )

    # グリッドマップの範囲内にある点のみを処理
    valid_mask: NDArray[np.bool_] = (
        (x_img >= 0)
        & (x_img < grid_map.shape[1])
        & (y_img >= 0)
        & (y_img < grid_map.shape[0])
    )

    x_img_valid: NDArray[np.int32] = x_img[valid_mask]
    y_img_valid: NDArray[np.int32] = y_img[valid_mask]
    valid_points: NDArray[np.float64] = original_points[valid_mask]

    occupied_mask: NDArray[np.bool_] = grid_map[y_img_valid, x_img_valid] == 1
    matched_points: NDArray[np.float64] = valid_points[occupied_mask]

    return matched_points


def update_counters_and_probabilities(
    grid_map: NDArray[np.uint8],
    counter: NDArray[np.int32],
    accumulation: AccumulationConf,
) -> tuple[NDArray[np.float64], NDArray[np.int32]]:
    """
    grid_mapとcounterを更新して、占有されたセルを示すprob_presentを計算する

    Parameters:
    grid_map (np.ndarray): グリッドマップのデータを持つ2D配列
    counter (np.ndarray): 各セルのカウンターを持つ2D配列

    Returns:
    np.ndarray: 占有されたセルを示す2D配列
    np.ndarray: 更新されたcounter配列
    """
    # measurementが1のセルに対する処理
    mask: NDArray[np.bool_] = grid_map == 1

    # もしカウンターが負値のときに点群が計測された場合はcounterをリセット
    counter[mask & (counter < 0)] = 0
    counter[mask] += accumulation.increment_accum_counter

    # measurementが1でないセルに対する処理
    not_mask: NDArray[np.bool_] = ~mask
    counter[not_mask & (counter > accumulation.accum_counter_max_cap)] = (
        accumulation.accum_counter_max_cap
    )
    counter[not_mask & (counter > accumulation.decrement_speed_threshold)] -= (
        accumulation.decrement_accum_counter
    )
    counter[not_mask & (counter <= accumulation.decrement_speed_threshold)] -= (
        accumulation.decrement_accum_counter_slow
    )

    # counterが10以上の場合、prob_presentを1に設定
    prob_present: NDArray[np.float64] = np.zeros_like(counter, dtype=np.float64)
    prob_present[counter >= accumulation.prob_present_threshold] = 1.0

    # counterが-10未満の場合は0にリセット
    counter[counter < accumulation.accum_counter_lower_reset_threshold] = 0

    return prob_present, counter


def transform_np(
    pts: NDArray[np.float64],
    inv_t: NDArray[np.float64],
) -> NDArray[np.float64]:
    n: int = pts.shape[0]
    hom: NDArray[np.float64] = np.hstack(
        (pts, np.ones((n, 1), dtype=np.float64)),
    )
    transformed: NDArray[np.float64] = (hom @ inv_t.T)[:, :3]
    return transformed


EYE = np.eye(4)


def trim_accumulated_frames(
    accumulated_frames: deque[NDArray[np.float64]],
    max_frames: int,
) -> None:
    if max_frames <= 0:
        raise ValueError("max accumulated frames must be greater than zero")
    while len(accumulated_frames) > max_frames:
        accumulated_frames.popleft()


def accumulate_point(
    xyz: NDArray[np.float64],
    counter: NDArray[np.int32],
    accum_points_dq: deque[NDArray[np.float64]],
    accum_ground_dq: deque[NDArray[np.float64]],
    lidar_grid: LidarGridConf,
    accumulation: AccumulationConf,
    general_conf: GeneralConf,
    is_edge_detection_applied: bool,
    trans_mat: NDArray[np.float64] = EYE,
    accum_counter: int = -1,
    is_reduced_load_mode: bool = False,
) -> tuple[
    NDArray[np.int32],  # counter
    NDArray[np.float64],  # accumulated_points
    NDArray[np.float64],  # accumulated_ground_points
    deque[NDArray[np.float64]],  # accum_points_dq
    deque[NDArray[np.float64]],  # accum_ground_dq
    int,  # accum_counter
]:
    # ------------------------------------------------------------------
    # 1. 最新点群を立体物と地面点群に分割
    # ------------------------------------------------------------------
    # accum_counterの初期値=-1
    accum_counter += 1

    # 負荷低減モードの切替を毎フレーム反映し、保持する履歴数を動的に変更する。
    if is_reduced_load_mode:
        max_accumulated_frames = accumulation.max_accumulated_frames_reduced_load
        max_accumulated_frames_ground = (
            accumulation.max_accumulated_frames_ground_reduced_load
        )
    else:
        max_accumulated_frames = accumulation.max_accumulated_frames
        max_accumulated_frames_ground = accumulation.max_accumulated_frames_ground

    # 上限縮小時は、以降の処理で参照する前に古い履歴から破棄する。
    trim_accumulated_frames(accum_points_dq, max_accumulated_frames)
    trim_accumulated_frames(accum_ground_dq, max_accumulated_frames_ground)

    non_ground_pts: NDArray[np.float64]
    ground_pts: NDArray[np.float64]
    non_ground_pts, ground_pts = filter_lidar_points(
        xyz,
        lidar_grid,
        general_conf,
    )

    # ------------------------------------------------------------------
    # 2. 蓄積した過去Nフレーム分の点群を最新LiDAR座標系へ変換
    # ------------------------------------------------------------------
    if accum_counter > 0:
        inv_T: NDArray[np.float64] = np.linalg.inv(trans_mat).astype(np.float64)
        for i, arr in enumerate(accum_points_dq):
            accum_points_dq[i] = transform_np(arr, inv_T)
        for i, arr in enumerate(accum_ground_dq):
            accum_ground_dq[i] = transform_np(arr, inv_T)

    # ------------------------------------------------------------------
    # 3. グリッドマップ → 占有確率更新
    # ------------------------------------------------------------------
    grid_map: NDArray[np.uint8] = lidar_to_grid_map(
        points=non_ground_pts,
        grid_size=(lidar_grid.grid_size, lidar_grid.grid_size),
        side_range=(lidar_grid.side_min, lidar_grid.side_max),
        fwd_range=(lidar_grid.fwd_min, lidar_grid.fwd_max),
    )

    prob_present: NDArray[np.float64]
    prob_present, counter = update_counters_and_probabilities(
        grid_map,
        counter,
        accumulation,
    )

    # ------------------------------------------------------------------
    # 4. 安定セルに含まれる「過去Nフレームの点を抽出
    #    (現時点ではdequeへまだ最新フレーム点群を入れていない)
    # ------------------------------------------------------------------
    if len(accum_points_dq) == 0:
        extracted_points: NDArray[np.float64] = np.empty((0, 3), dtype=np.float64)
    else:
        accum_points_all: NDArray[np.float64] = (
            accum_points_dq[0]
            if len(accum_points_dq) == 1
            else np.vstack(accum_points_dq)
        )
        extracted_points = compare_in_grid_space_vectorized(
            grid_map=prob_present,
            original_points=accum_points_all,
            grid_size=(lidar_grid.grid_size, lidar_grid.grid_size),
            side_range=(lidar_grid.side_min, lidar_grid.side_max),
            fwd_range=(lidar_grid.fwd_min, lidar_grid.fwd_max),
        )

    # ------------------------------------------------------------------
    # 5. 安定化点群を生成(最新点群 + 安定化点群)
    # ------------------------------------------------------------------

    ## 立体物点群
    accumulated_points_wo_ground: NDArray[np.float64] = np.append(
        non_ground_pts,
        extracted_points,
    ).reshape(-1, 3)

    if is_reduced_load_mode:
        # 負荷低減モード
        voxel_size_for_accumulated_points: float = (
            accumulation.voxel_size_for_accumulated_points_reduced_load
        )
        voxel_size_for_accumulated_ground_points: float = (
            accumulation.voxel_size_for_accumulated_ground_points_reduced_load
        )
    else:
        # 通常モード
        voxel_size_for_accumulated_points: float = (
            accumulation.voxel_size_for_accumulated_points
        )
        voxel_size_for_accumulated_ground_points: float = (
            accumulation.voxel_size_for_accumulated_ground_points
        )

    accumulated_points_wo_ground: NDArray[np.float64] = utils.pcd_to_np(
        utils.np_to_pcd(accumulated_points_wo_ground).voxel_down_sample(
            voxel_size_for_accumulated_points,
        ),
    )

    ## 地面点群
    if is_edge_detection_applied:
        if len(accum_ground_dq) == 0:
            ground_fused: NDArray[np.float64] = ground_pts
        else:
            ground_fused = np.vstack((*accum_ground_dq, ground_pts))

        accumulated_ground_points: NDArray[np.float64] = utils.pcd_to_np(
            utils.np_to_pcd(ground_fused).voxel_down_sample(
                voxel_size_for_accumulated_ground_points,
            ),
        )
    else:
        accumulated_ground_points = np.empty((0, 3), dtype=np.float64)

    # ------------------------------------------------------------------
    # 6. deque(FIFO)
    # ------------------------------------------------------------------
    accum_points_dq.append(
        utils.pcd_to_np(
            utils.np_to_pcd(non_ground_pts).voxel_down_sample(
                voxel_size_for_accumulated_points,
            ),
        ),
    )
    if is_edge_detection_applied:
        accum_ground_dq.append(
            utils.pcd_to_np(
                utils.np_to_pcd(ground_pts).voxel_down_sample(
                    voxel_size_for_accumulated_ground_points,
                ),
            ),
        )

    # dequeの物理上限より小さいモード別上限を、append後も維持する。
    trim_accumulated_frames(accum_points_dq, max_accumulated_frames)
    trim_accumulated_frames(accum_ground_dq, max_accumulated_frames_ground)

    return (
        counter,
        accumulated_points_wo_ground,
        accumulated_ground_points,
        accum_points_dq,
        accum_ground_dq,
        accum_counter,
    )
