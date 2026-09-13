from __future__ import annotations

from typing import Any

import numpy as np
import open3d as o3d
from numpy.typing import NDArray
from tqdm import tqdm


def check_input_array_type(
    array: NDArray[Any] | o3d.geometry.PointCloud,
) -> o3d.geometry.PointCloud:
    # A が NumPy array かどうかをチェック
    if isinstance(array, np.ndarray):
        # NumPy array を Open3D の Point Cloud に変換
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(array)
        return pcd
    if isinstance(array, o3d.geometry.PointCloud):
        # A が既に Open3D の Point Cloud ならそのまま返す
        return array
    msg = "入力は NumPy array または Open3D の Point Cloud 形式である必要があります。"
    raise TypeError(msg)


def preprocess_point_cloud(
    pcd_down: o3d.geometry.PointCloud,
    voxel_size: float,
) -> tuple[o3d.geometry.PointCloud, o3d.pipelines.registration.Feature]:
    # print(":: Downsample with a voxel size %.3f." % voxel_size)
    pcd_down = pcd_down.voxel_down_sample(voxel_size)

    radius_normal = voxel_size * 2
    # print(":: Estimate normal with search radius %.3f." % radius_normal)
    pcd_down.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30),
    )

    radius_feature = voxel_size * 5
    # print(":: Compute FPFH feature with search radius %.3f." % radius_feature)
    pcd_fpfh = o3d.pipelines.registration.compute_fpfh_feature(
        pcd_down,
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=100),
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
    # print(":: Load two point clouds and disturb initial pose.")
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
    distance_threshold = thr_radius
    return o3d.pipelines.registration.registration_icp(
        source,
        target,
        distance_threshold,
        init_matrix,
        o3d.pipelines.registration.TransformationEstimationPointToPlane(),
    )


def refine_registration_icp_plane2plane(
    source: PointCloud,
    target: PointCloud,
    thr_radius: float,
    init_matrix: NDArray[np.float64],
) -> registration.RegistrationResult:
    distance_threshold = thr_radius
    result = o3d.pipelines.registration.registration_generalized_icp(
        source,
        target,
        distance_threshold,
        init_matrix,
        o3d.pipelines.registration.TransformationEstimationForGeneralizedICP(),
    )
    return result


# TODO registrateTwoPCloudsから名前を変更しているため、変更したことを反映する必要あり
def registrate_two_pclouds(
    source: NDArray[Any],
    target: NDArray[Any],
    voxel_size: float,
    init_trans: NDArray[Any],
    thr_radius: list,
    icp_type: str,
    visualize: bool = True,
) -> tuple[o3d.geometry.PointCloud, NDArray[np.float64]]:
    """2つの点群sourceとtargetの位置合わせを行う。srouceをtarget点群座標に合わせる

    Args:
        source (numpy): 点群1
        target (numpy): 点群2
        voxel_size (double): ダウンサンプルサイズ
        MAKE_GRID (bool, optional): Trueなら地面点群を格子状に変換する. Defaults to True.
        RANSAC (bool, optional): Trueなら初期変換行列としてRANSACの結果を使用する。Falseなら行列はnp.eye(4). Defaults to False.
        visualize (bool, optional): Trueなら変換結果を表示する. Defaults to False.

    Returns:
        source_transformed: 変換された点群
        result_icp.transformation: 得られた変換行列
    """

    pcd_source: o3d.geometry.PointCloud = check_input_array_type(source)
    pcd_target: o3d.geometry.PointCloud = check_input_array_type(target)
    if visualize:
        o3d.visualization.draw_geometries(
            geometry_list=[pcd_source, pcd_target], window_name="raw source and target"
        )

    # source点群、ターゲット点群のダウンサンプリング、法線計算、FPFH特徴量を計算
    pcd_source, pcd_target, source_down, target_down, source_fpfh, target_fpfh = (
        calculate_features(voxel_size, pcd_source, pcd_target)
    )

    init_matrix = init_trans

    # ICPによる微調整
    for scale in tqdm(range(len(thr_radius))):
        if icp_type == "plane2plane":
            result_icp = refine_registration_icp_plane2plane(
                source_down,
                target_down,
                thr_radius[scale],
                init_matrix,
            )

        elif icp_type == "point2plane":
            result_icp = refine_registration_icp_p2plane(
                source_down,
                target_down,
                thr_radius[scale],
                init_matrix,
            )

        init_matrix = result_icp.transformation

    # source点群をtarget点群座標へ変換
    source_transformed: PointCloud = pcd_source.transform(
        result_icp.transformation,
    )
    if visualize:
        pcd_target.paint_uniform_color([0, 0, 0])
        o3d.visualization.draw_geometries(
            geometry_list=[source_transformed, pcd_target],
            window_name="Registration result",
        )

    return source_transformed, result_icp.transformation
