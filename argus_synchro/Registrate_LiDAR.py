from __future__ import annotations

import numpy as np
import open3d as o3d
from numpy.typing import NDArray

from argus_synchro.config.app_config import AccumulationConf
from argus_synchro.lidar_registration.multi_scale_icp import multi_scale_icp_cpu
from argus_synchro.lidar_registration.ndt import NDT
from argus_synchro.point_processing import utils

grid_size = 5
ndt = NDT(grid_size=grid_size)


EYE = np.eye(4)
USE_CUDA = o3d.core.cuda.is_available()


def crop_points(
    xyz: NDArray[np.float64],
    x_range: tuple[float, float] = (-15, 15),
    y_range: tuple[float, float] = (-15, 15),
    z_range: tuple[float, float] = (-1.5, 1),
) -> NDArray[np.float64]:
    """registrationに用いる点群の範囲を制限する"""
    mask: NDArray[np.bool_] = (
        (xyz[:, 0] >= x_range[0])
        & (xyz[:, 0] < x_range[1])
        & (xyz[:, 1] >= y_range[0])
        & (xyz[:, 1] < y_range[1])
        & (xyz[:, 2] >= z_range[0])
        & (xyz[:, 2] < z_range[1])
    )
    return xyz[mask]


# def icp(
#     target: PointCloud,
#     source: PointCloud,
#     threshold: float,
#     trans_init: NDArray[np.float64],
# ) -> registration.RegistrationResult:
#     return o3d.pipelines.registration.registration_icp(
#         source,
#         target,
#         threshold,
#         trans_init,
#         o3d.pipelines.registration.TransformationEstimationPointToPlane(),
#     )


def preprocess_point_cloud(
    pcd_down: o3d.geometry.PointCloud,
    voxel_size: float,
) -> tuple[o3d.geometry.PointCloud, o3d.pipelines.registration.Feature]:
    # AppLogger.info(":: Downsample with a voxel size %.3f." % voxel_size)
    pcd_down = pcd_down.voxel_down_sample(voxel_size)

    radius_normal: float = voxel_size * 2
    # AppLogger.info(":: Estimate normal with search radius %.3f." % radius_normal)
    pcd_down.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30),
    )

    radius_feature: float = voxel_size * 5
    # AppLogger.info(":: Compute FPFH feature with search radius %.3f." % radius_feature)
    pcd_fpfh: o3d.pipelines.registration.Feature = (
        o3d.pipelines.registration.compute_fpfh_feature(
            pcd_down,
            o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=100),
        )
    )
    return pcd_down, pcd_fpfh


def calculate_features(
    voxel_size: float,
    source: o3d.geometry.PointCloud,
    target: o3d.geometry.PointCloud,
) -> tuple[
    o3d.geometry.PointCloud,
    o3d.geometry.PointCloud,
    o3d.geometry.PointCloud,
    o3d.geometry.PointCloud,
    o3d.pipelines.registration.Feature,
    o3d.pipelines.registration.Feature,
]:
    # AppLogger.info(":: Load two point clouds and disturb initial pose.")
    # trans_init = np.asarray([[0.0, 0.0, 1.0, 0.0], [1.0, 0.0, 0.0, 0.0],
    #                         [0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]])
    # source.transform(trans_init)
    # draw_registration_result(source, target, np.identity(4))

    source_down, source_fpfh = preprocess_point_cloud(source, voxel_size)
    target_down, target_fpfh = preprocess_point_cloud(target, voxel_size)
    return source, target, source_down, target_down, source_fpfh, target_fpfh


def refine_registration_icp_p2plane(
    source: o3d.geometry.PointCloud,
    target: o3d.geometry.PointCloud,
    thr_radius: float,
    init_matrix: NDArray[np.float64],
) -> o3d.pipelines.registration.RegistrationResult:
    distance_threshold: float = thr_radius
    result: o3d.pipelines.registration.RegistrationResult = (
        o3d.pipelines.registration.registration_icp(
            source,
            target,
            distance_threshold,
            init_matrix,
            o3d.pipelines.registration.TransformationEstimationPointToPlane(),
        )
    )
    return result


def registrate_two_pclouds(
    input_xyz: NDArray[np.float64],
    target_in: NDArray[np.float64],
    accumulation: AccumulationConf,
    init_source_to_target: o3d.core.Tensor,
    delta_yaw: float = 0.0,
    visualize: bool = False,
) -> tuple[
        NDArray[np.float64],
        o3d.core.Tensor,
    ]:
    """
    2つの点群(sourceとtarget)の位置合わせを行い、sourceをtargetに合わせる変換行列を返す

    Parameters:
        input_xyz (np.ndarray): source点群データ、形状は (N, 3)
        target (np.ndarray): target点群データ、形状は (M, 3)
        voxel_size (float, optional): ダウンサンプリング時のボクセルサイズ。デフォルトは0.25
        method (str, optional): 使用するレジストレーション手法 ("icp", "multi_icp", または "ndt")。デフォルトは "ndt"
        delta_yaw (Optional[float], optional): NDTで使用する回転角度(度数)。デフォルトは0.0
        visualize (bool, optional): Trueの場合、レジストレーション結果を可視化する。デフォルトはFalse

    Returns:
        np.ndarray: sourceをtargetに合わせる4x4の変換行列
    """

    xyz: NDArray[np.float64] = np.array(input_xyz, copy=True)
    source_range: float = accumulation.registration_range
    target_range: float = accumulation.registration_range

    xyz: NDArray[np.float64] = crop_points(
        xyz,
        x_range=[-source_range, source_range],
        y_range=[-source_range, source_range],
        z_range=[-3, 3],
    )
    source: o3d.geometry.PointCloud = utils.np_to_pcd(xyz).voxel_down_sample(
        accumulation.voxel_down_sample,
    )

    # source点群、ターゲット点群のダウンサンプリング、法線計算、FPFH特徴量を計算
    target: NDArray[np.float64] = np.asarray(target_in).reshape(-1, 3)
    target = crop_points(
        target,
        x_range=(-target_range, target_range),
        y_range=(-target_range, target_range),
        z_range=(-3, 3),
    )
    target: o3d.geometry.PointCloud = utils.np_to_pcd(target).voxel_down_sample(
        accumulation.voxel_down_sample,
    )

    if accumulation.registration_methods == "icp":
        thr_radius: list[float] = accumulation.correspondence_distances
        source, target, source_down, target_down, _source_fpfh, _target_fpfh = (
            calculate_features(accumulation.voxel_down_sample, source, target)
        )

        init_matrix: NDArray[np.float64] = np.array(EYE, copy=True)

        # ICPによる微調整
        for scale in range(len(thr_radius)):
            if scale > 0:
                init_matrix = result_icp.transformation

            # init_matrix = np.identity(4)
            result_icp: o3d.pipelines.registration.RegistrationResult = (
                refine_registration_icp_p2plane(
                    source_down,
                    target_down,
                    thr_radius[scale],
                    init_matrix,
                )
            )

        final_transform: NDArray[np.float64] = np.eye(4)
        final_transform[:3, :3] = np.asarray(result_icp.transformation)[:3, :3]
        final_transform[:3, 3] = np.asarray(result_icp.transformation)[:3, 3]
        final_transform.reshape(4, 4)

        if visualize:
            target: NDArray[np.float64] = utils.pcd_to_np(target)
            source: NDArray[np.float64] = utils.pcd_to_np(source)

    elif accumulation.registration_methods == "multi_icp":
        result_icp = multi_scale_icp_cpu(
            source,
            target,
            accumulation.voxel_size_for_multi_icp,
            accumulation.correspondence_distances,
            init_source_to_target,
        )
        # 次回初期変換用に、今回の結果を保持
        init_source_to_target = result_icp.transformation
        final_transform = np.array(EYE, copy=True)
        final_transform[:3, :3] = o3d.core.Tensor.numpy(result_icp.transformation)[
            :3,
            :3,
        ]
        final_transform[:3, 3] = o3d.core.Tensor.numpy(result_icp.transformation)[:3, 3]
        final_transform.reshape(4, 4)

        if visualize:
            target: NDArray[np.float64] = utils.pcd_to_np(target)
            source: NDArray[np.float64] = utils.pcd_to_np(source)

    elif accumulation.registration_methods == "ndt":
        target: NDArray[np.float64] = utils.pcd_to_np(target)
        source: NDArray[np.float64] = utils.pcd_to_np(source)
        final_transform = ndt.ndt_registration(
            source_points=source,
            target_points=target,
            yaw_angle=delta_yaw,
        )

    else:
        msg = (
            "Method should be one of: 'icp', 'multi_icp', or 'ndt'. Check settings.ini"
        )
        raise ValueError(msg)

    if visualize:
        # source点群をtarget点群座標へ変換
        source_transformed = (
            utils.np_to_pcd(source)
            .transform(final_transform)
            .paint_uniform_color([1, 0, 0])
        )
        o3d.visualization.draw_geometries(
            geometry_list=[
                source_transformed,
                utils.np_to_pcd(target).paint_uniform_color([0, 0, 0]),
            ],
            window_name=f"{input_xyz.shape}",
        )

    return final_transform, init_source_to_target
