from __future__ import annotations

from collections.abc import Callable
from pathlib import Path

import numpy as np
import open3d as o3d
import pandas as pd
from numpy.typing import NDArray

import argus_synchro.SubScrutinizer as sub
from argus_synchro import calibration
from argus_synchro.calibration_mat_generator_modules.ctrl.calibration3d3d.simulate_lidar_points import (
    simulate_crane_pts,
)
from argus_synchro.config.app_config import AppConfig
from argus_synchro.config.app_config_calibration import AppConfigCalibration
from argus_synchro.lidar_registration import ndt2d
from argus_synchro.lidar_registration.icp import registrate_two_pclouds
from argus_synchro.lidar_registration.ndt2d import (
    NDT2D,
    NDT2DParams,
    preprocess_xy,
    se2_from_params,
    se2_to_se3,
    voxel_downsample_xy,
)
from argus_synchro.point_processing import utils


def visualize_preprocess_xy(src_xy: np.ndarray, tgt_xy: np.ndarray, z0: float = 0.0):
    """
    src_xy, tgt_xy: (N,2)  ※preprocess_xyの出力
    z0: 表示用の固定Z（鳥瞰なら0でOK）
    """

    def xy_to_pcd(xy: np.ndarray, color):
        xy = np.asarray(xy, dtype=np.float64)
        xyz = np.c_[xy, np.full((xy.shape[0],), z0, dtype=np.float64)]
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(xyz)
        pcd.paint_uniform_color(color)
        return pcd

    src_pcd = xy_to_pcd(src_xy, [1, 0, 0])  # red
    tgt_pcd = xy_to_pcd(tgt_xy, [0, 1, 0])  # green

    if False:
        o3d.visualization.draw_geometries(
            [src_pcd, tgt_pcd], window_name="preprocess_xy (XY projected)"
        )


def compute_init_trans_by_ndt2d(
    src_pts: np.ndarray,
    tgt_pts: np.ndarray,
    T0: np.ndarray,
    param: NDT2DParams,
    range_min: float = 0.5,
    range_max: float = 40.0,
    ground_height: float = -1.368,
    ds_voxel_xy: float = 0.0,  # 2D点の間引き
    yaw_offset_list: list[float] = [
        -15,
        -12.5,
        -10,
        -7.5,
        -5,
        -2.5,
        0,
        2.5,
        5,
        7.5,
        10,
        12.5,
        15,
    ],
) -> np.ndarray:
    """
    2DNDT(tx, ty, yaw)でinit_transを返す（4x4, Z回転＋XY並進のみ）
    """

    T0 = np.asarray(T0, dtype=np.float64)
    if T0.shape != (4, 4):
        raise ValueError("T0 must be (4,4)")

    # 地面除去
    # LiDARは上下反転してつけられているため、地面方向が正方向、上空が負方向なことに注意
    # z_dis_min= -999にして、上空方向の点群はすべて取得させる
    # z_dis_max = -1*ground_height -0.5 settings.iniでground_heightの値はLiDARが上下正しくつけられたときの値が設定されている
    # そのため、-1をかけることで、上下を反転させる。その上で、マージンを持たせるために 0.5を引く
    # ground_heightは機種ごとに設定されているので、機種が変わっても問題なく動くはず
    src_pts = utils.remove_pc_outside_of(
        src_pts, z_dis_min=-999, z_dis_max=-1 * ground_height - 0.5
    )
    tgt_pts = utils.remove_pc_outside_of(
        tgt_pts, z_dis_min=-999, z_dis_max=-1 * ground_height - 0.5
    )

    # 鳥瞰図へ投影
    src_xy = preprocess_xy(src_pts, r_min=range_min, r_max=range_max)
    tgt_xy = preprocess_xy(tgt_pts, r_min=range_min, r_max=range_max)

    # TODO: デバッグ用 本番では消す
    # visualize_preprocess_xy(src_xy, tgt_xy, z0=0.0)

    src_xy = voxel_downsample_xy(src_xy, ds_voxel_xy)
    tgt_xy = voxel_downsample_xy(tgt_xy, ds_voxel_xy)

    # 初期行列T0から、2Dの初期(tx,ty,yaw)を取り出す
    yaw0 = float(np.arctan2(T0[1, 0], T0[0, 0]))
    tx0 = float(T0[0, 3])
    ty0 = float(T0[1, 3])
    init_T2 = se2_from_params(tx0, ty0, yaw0)

    # 2D NDTパラメータ
    ndt2 = NDT2D(param)
    ndt2.fit(tgt_xy)

    # yaw探索（degオフセット）
    # LiDAR設置角度が大幅にずれてる場合の対策として導入
    yaw_offsets_deg = np.array(yaw_offset_list, dtype=np.float64)
    yaw_offsets = np.deg2rad(yaw_offsets_deg)

    init_list = []
    for dy in yaw_offsets:
        init_list.append(se2_from_params(tx0, ty0, yaw0 + float(dy)))

    # スコア最小の初期行列を採用する
    best_T2 = init_T2
    best_score = np.inf
    best_inliers = -1
    min_inliers = 50

    for itr, T2_init in enumerate(init_list):
        res = ndt2.register(src_xy, T2_init)

        # 破綻候補は除外
        if res.inliers < min_inliers:
            continue

        # 第一基準：スコア最小
        # 同点近傍のときだけ第二基準でinliers大を採用
        if (res.final_score < best_score) or (
            np.isclose(res.final_score, best_score) and res.inliers > best_inliers
        ):
            best_score = res.final_score
            best_inliers = res.inliers
            best_T2 = res.T2

    # SE2 -> SE3
    T3 = se2_to_se3(best_T2)
    # z成分は変化しないはずなので差し替え
    T3[2, 3] = float(T0[2, 3])

    return T3


def _split_and_thin(
    points: np.ndarray,
    z_height: float = 0.5,
    voxel_size: float = 1.0,
) -> np.ndarray:
    """
    Z座標で点群を2分割し、低層部のみダウンサンプリングして再結合する。
    """
    above = points[points[:, 2] < z_height][:, :3]
    below = points[points[:, 2] >= z_height][:, :3]
    below_thin = utils.pcd_to_np(utils.np_to_pcd(below).voxel_down_sample(voxel_size))
    return np.vstack((above, below_thin))


def translate_points(
    points: np.ndarray,
    trans_mat: np.ndarray,
) -> o3d.geometry.PointCloud:
    """
    np.ndarray の点群に 4×4 変換行列を適用して Open3D PointCloud を返す。
    """
    return utils.np_to_pcd(points).transform(trans_mat)


def _write_calibration_csv(
    savepath: str,
    transform: np.ndarray,
    file_io_error_reporter: Callable[[str, str, Exception], None] | None,
) -> None:
    try:
        Path(savepath).parent.mkdir(parents=True, exist_ok=True)
        pd.DataFrame(transform).to_csv(savepath, header=False, index=False)
    except OSError as error:
        if file_io_error_reporter is not None:
            file_io_error_reporter(
                savepath,
                "write 3D-3D calibration result matrix CSV",
                error,
            )
        raise


def calibrateLidars2Crane(
    lidar_np_list: list[np.ndarray],
    lidar_raw_pcd_list: list[o3d.geometry.PointCloud],
    savepaths: list[str],
    app_config: AppConfig,
    calib_app_config: AppConfigCalibration,
    angle_data: float = 0,
    visualize: bool = False,
    file_io_error_reporter: Callable[[str, str, Exception], None] | None = None,
) -> tuple[
    o3d.geometry.PointCloud,  # 統合点群 (Crane 座標系) P₁₂…N
    list[o3d.geometry.PointCloud],  # 各 LiDAR の点群 (Crane 座標系)
    list[np.ndarray],  # T_i→Crane (len = N)
    list[np.ndarray],  # T_i→LiDAR1 (len = N)  ※LiDAR1 は I
]:
    """
    N 台の LiDAR を 1→2→…→N の順で統合し、その後クレーン CAD と位置合わせする。

    Parameters
    ----------
    lidar_np_list : list[np.ndarray]
        LiDAR i の生点群 (N_i×3)。順番は登録順 (LiDAR1, LiDAR2, …)。
    lidar_raw_pcd_list : list[o3d.geometry.PointCloud]
        可視化用の Open3D PointCloud。`lidar_np_list` と同じ順序。
    z_height, voxel_size : float
        `_split_and_thin` のパラメータ。
    visualize : bool
        True なら最終結果を可視化。

    Returns
    -------
    merged_pcd_crane : o3d.geometry.PointCloud
        統合された点群 (Crane 座標系)。
    lidar_pcds_crane : list[o3d.geometry.PointCloud]
        LiDAR i の個別点群 (Crane 座標系)。
    T_i2Crane_list : list[np.ndarray]
        LiDAR i → Crane の 4×4 行列。
    T_i2L1_list : list[np.ndarray]
        LiDAR i → LiDAR1 の 4×4 行列 (参照用)。LiDAR1 は恒等行列。
    """

    voxel_size_for_ground_pts = (
        calib_app_config.Calib3d3d_CalibParams.voxel_size_for_ground_pts
    )
    voxel_size_for_calib = calib_app_config.Calib3d3d_CalibParams.voxel_size_for_calib

    init_R: NDArray[np.float64]
    init_t: NDArray[np.float64]
    init_R, init_t = sub.load_transform_csv(app_config.General.initial_transform_file)

    # 1. 各LiDARを設計図からクレーン座標系に移動させる
    # そうすることで、各LiDARの座標も近づくので, init_transは単位行列で良くなる
    num_lidars = app_config.Lidar.count
    ideal_calib_path = calib_app_config.Calib3d3d_CalibParams.lidars_calib_path

    for i in range(num_lidars):
        try:
            lidar_np_list[i] = calibration.old_calib_lidar(
                lidar_np_list[i], ideal_calib_path[i]
            )
        except (OSError, UnicodeError, ValueError) as error:
            if file_io_error_reporter is not None:
                file_io_error_reporter(
                    ideal_calib_path[i], "read 3D-3D ideal calibration CSV", error
                )
            raise

    # AppLogger.info("calibrateLidars2Crane", "make LiDARi2LiDAR1_list")

    # シミュレーション点群の生成
    sim_pts, hits_per_sensor = simulate_crane_pts(
        lidar_pos=ideal_calib_path,
        angle_data=angle_data,
        app_config=app_config,
        calib_app_config=calib_app_config,
        file_io_error_reporter=file_io_error_reporter,
    )

    # 2. LiDAR0基準で逐次登録
    merged_points = lidar_np_list[0].copy()
    # LiDAR-iから基準LiDARへの変換行列をここに格納する。基準LiDARから基準LiDARへの変換行列も使うので、先に単位行列として格納
    LiDARi2LiDAR1_list: list[np.ndarray] = [np.eye(4)]

    # 理想行列で変換済みなので単位行列
    init_trans = np.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])

    param_lid2lid = ndt2d.NDT2DParams(
        voxel_size=calib_app_config.Calib3d3d_NdtParams.voxel_size,
        min_points_per_voxel=calib_app_config.Calib3d3d_NdtParams.min_points_per_voxel,
        neighbor_top_k=calib_app_config.Calib3d3d_NdtParams.neighbor_top_k,
        neighbor_maha2_gate=calib_app_config.Calib3d3d_NdtParams.neighbor_maha2_gate,
        weight_temperature=calib_app_config.Calib3d3d_NdtParams.weight_temperature,
        geom_sigma=calib_app_config.Calib3d3d_NdtParams.geom_sigma,
        geom_gate=calib_app_config.Calib3d3d_NdtParams.geom_gate,
        max_iters=calib_app_config.Calib3d3d_NdtParams.max_iters,
        lm_lambda_init=calib_app_config.Calib3d3d_NdtParams.lm_lambda_init,
        step_clip_trans=calib_app_config.Calib3d3d_NdtParams.step_clip_trans,
        step_clip_yaw=np.deg2rad(calib_app_config.Calib3d3d_NdtParams.step_clip_yaw),
    )

    # LiDAR-LiDAR間の位置合わせを実施する
    # LiDAR0を基準LiDARとして、LiDAR1, LiDAR2,..., LiDARNを一つずつマージする
    for i in range(1, num_lidars):
        # AppLogger.info("calibrateLidars2Crane", f"for {i}")
        src_pts = lidar_np_list[i]

        init_trans_ndt = compute_init_trans_by_ndt2d(
            src_pts=src_pts,
            tgt_pts=merged_points,
            T0=init_trans,
            param=param_lid2lid,
            range_min=calib_app_config.Calib3d3d_NdtParams.range_min,
            range_max=calib_app_config.Calib3d3d_NdtParams.range_max,
            ground_height=app_config.General.ground_height,
            ds_voxel_xy=calib_app_config.Calib3d3d_NdtParams.ds_voxel_xy,
            yaw_offset_list=calib_app_config.Calib3d3d_NdtParams.yaw_offset_list,
        )

        transformed_pair, T_pair = registrate_two_pclouds(
            src_pts,
            merged_points,
            voxel_size=voxel_size_for_calib,
            init_trans=init_trans_ndt,
            thr_radius=calib_app_config.Calib3d3d_CalibParams.thr_radius_L2L,
            icp_type="plane2plane",
            visualize=visualize,
        )
        src_transformed_pcd = transformed_pair
        T_i2L1 = T_pair

        merged_points = np.vstack((merged_points, utils.pcd_to_np(src_transformed_pcd)))
        LiDARi2LiDAR1_list.append(T_i2L1)

    # 高さ閾値(z_height)でLiDAR点群を分けて、閾値よりも低い点群に対してダウンサンプルを実施
    # これは地面点群に含まれている情報量が少なく、重要視したくないため実施

    z_height = (
        -1 * app_config.General.ground_height
        - calib_app_config.Calib3d3d_NdtParams.thinning_margin
    )
    merged_points = _split_and_thin(merged_points, z_height, voxel_size_for_ground_pts)

    # LiDAR座標系にクレーン点群を回転させる
    sim_pts = sub.apply_initial_transform(xyz=sim_pts, r=init_R, t=init_t)
    sim_pts_thin = _split_and_thin(sim_pts, z_height, voxel_size_for_ground_pts)

    transformed_pair, T_merge2Crane = registrate_two_pclouds(
        merged_points,
        sim_pts_thin,
        voxel_size=voxel_size_for_calib,
        init_trans=init_trans,
        thr_radius=calib_app_config.Calib3d3d_CalibParams.thr_radius_L2C,
        icp_type="plane2plane",
        visualize=visualize,
    )

    # 各LiDARのCrane座標系点群・行列
    lidar_pcds_crane: list[o3d.geometry.PointCloud] = []
    LiDARi2Crane_list: list[np.ndarray] = []
    for i, (raw_pcd_i, T_i2L1) in enumerate(
        zip(lidar_raw_pcd_list, LiDARi2LiDAR1_list, strict=False)
    ):
        try:
            ideal_transform = pd.read_csv(
                ideal_calib_path[i], header=None
            ).values
        except (
            OSError,
            UnicodeError,
            pd.errors.EmptyDataError,
            pd.errors.ParserError,
        ) as error:
            if file_io_error_reporter is not None:
                file_io_error_reporter(
                    ideal_calib_path[i], "read 3D-3D ideal calibration CSV", error
                )
            raise
        T_i2Crane = T_merge2Crane @ T_i2L1 @ ideal_transform
        LiDARi2Crane_list.append(T_i2Crane)
        lidar_pcds_crane.append(translate_points(utils.pcd_to_np(raw_pcd_i), T_i2Crane))

    # ファイル保存
    for idx, T in enumerate(LiDARi2Crane_list):
        _write_calibration_csv(savepaths[idx], T, file_io_error_reporter)

    # 可視化
    merged_pcd_crane = utils.np_to_pcd(sim_pts)
    merged_pcd_crane.paint_uniform_color([0, 0, 0])
    if not visualize:
        colors = [
            [1, 0, 0],
            [0, 1, 0],
            [0, 0, 1],
            [1, 1, 0],
            [1, 0, 1],
            [0, 1, 1],
            [0.5, 0.5, 0.5],
        ]
        for pcd_i, col in zip(
            lidar_pcds_crane, colors * (num_lidars // len(colors) + 1), strict=False
        ):
            if hasattr(pcd_i, "paint_uniform_color"):
                pcd_i.paint_uniform_color(col)
        coord = o3d.geometry.TriangleMesh.create_coordinate_frame(size=3)
        if visualize:
            o3d.visualization.draw_geometries(
                [merged_pcd_crane, *lidar_pcds_crane, coord],
                window_name="LiDAR1…N & Crane",
            )

    return merged_pcd_crane, lidar_pcds_crane, LiDARi2Crane_list, LiDARi2LiDAR1_list
