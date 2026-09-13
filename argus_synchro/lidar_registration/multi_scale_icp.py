from __future__ import annotations

import numpy as np
import open3d as o3d


def _geometry_to_tensor_pcd(pcd: o3d.geometry.PointCloud) -> o3d.t.geometry.PointCloud:
    points = np.asarray(pcd.points, dtype=np.float64)
    if not points.flags.c_contiguous:
        points = np.ascontiguousarray(points)
    return o3d.t.geometry.PointCloud(o3d.core.Tensor(points))


def multi_scale_icp(
    t_source: o3d.geometry.PointCloud,
    t_target: o3d.geometry.PointCloud,
    voxel_sizes_multi: list[float],
    corr_distances: list[float],
    init_source_to_target: o3d.core.Tensor,
) -> o3d.t.pipelines.registration.RegistrationResult:
    """
    source と target の点群に対してマルチスケール ICP を実行し
    変換行列やメトリクスを含む ``RegistrationResult`` を返す。

    Parameters
    ----------
    t_source : PointCloud
        ソース点群(open3d.geometry.PointCloud)
    t_target : PointCloud
        ターゲット点群(open3d.geometry.PointCloud)
    voxel_sizes_multi :list[float]
        各スケールで用いるダウンサンプリング・ボクセルサイズ
    corr_distances :list[float]
        各スケールにおける最大対応距離
    init_source_to_target : o3d.core.Tensor
        ソースからターゲットへの初期変換行列

    Returns
    -------
    o3d.t.pipelines.registration.RegistrationResult
        ``transformation`` 行列や ``fitness``・``inlier_rmse`` を含む結果
    """
    # マルチスケールICPのボクセルサイズを定義
    voxel_sizes = o3d.utility.DoubleVector(voxel_sizes_multi)

    # マルチスケールICPの収束基準のリスト
    criteria_list: list[o3d.t.pipelines.registration.ICPConvergenceCriteria] = [
        o3d.t.pipelines.registration.ICPConvergenceCriteria(
            relative_fitness=0.0001,
            relative_rmse=0.0001,
            max_iteration=10,
        ),
        o3d.t.pipelines.registration.ICPConvergenceCriteria(0.00001, 0.00001, 15),
        o3d.t.pipelines.registration.ICPConvergenceCriteria(0.000001, 0.000001, 10),
    ]

    # 各スケールの最大対応距離
    max_correspondence_distances = o3d.utility.DoubleVector(corr_distances)

    # 外れ値除去のための推定方法とロバストカーネル
    estimation = o3d.t.pipelines.registration.TransformationEstimationPointToPlane()

    # イテレーションごとのメトリクスを保存するコールバック関数
    """
    callback_after_iteration = lambda loss_log_map: AppLogger.info(
        "Iteration Index: {}, Scale Index: {}, Scale Iteration Index: {}, Fitness: {}, Inlier RMSE: {}".format(
            loss_log_map["iteration_index"].item(),
            loss_log_map["scale_index"].item(),
            loss_log_map["scale_iteration_index"].item(),
            loss_log_map["fitness"].item(),
            loss_log_map["inlier_rmse"].item(),
        ),
    )
    """

    # マルチスケールICPを実行
    registration_ms_icp: o3d.t.pipelines.registration.RegistrationResult = (
        o3d.t.pipelines.registration.multi_scale_icp(
            t_source,
            t_target,
            voxel_sizes,
            criteria_list,
            max_correspondence_distances,
            init_source_to_target,
            estimation,
            # callback_after_iteration,
        )
    )

    return registration_ms_icp


def multi_scale_icp_cpu(
    source: o3d.geometry.PointCloud,
    target: o3d.geometry.PointCloud,
    voxel_sizes_multi: list[float],
    corr_distances: list[float],
    init_source_to_target: o3d.core.Tensor,
) -> o3d.t.pipelines.registration.RegistrationResult:

    # レガシー点群をテンソルベースの点群に変換
    t_source: o3d.t.geometry.PointCloud = _geometry_to_tensor_pcd(source)
    t_target: o3d.t.geometry.PointCloud = _geometry_to_tensor_pcd(target)
    t_target.estimate_normals()

    return multi_scale_icp(
        t_source,
        t_target,
        voxel_sizes_multi,
        corr_distances,
        init_source_to_target,
    )


def multi_scale_icp_cuda(
    source: o3d.geometry.PointCloud,
    target: o3d.geometry.PointCloud,
    voxel_sizes_multi: list[float],
    corr_distances: list[float],
    init_source_to_target: o3d.core.Tensor,
) -> o3d.t.pipelines.registration.RegistrationResult:

    # レガシー点群をテンソルベースの点群に変換
    t_source: o3d.t.geometry.PointCloud = _geometry_to_tensor_pcd(source)
    t_target: o3d.t.geometry.PointCloud = _geometry_to_tensor_pcd(target)


    # 点群をGPUに移動
    t_source = t_source.cuda(0)
    t_target = t_target.cuda(0)
    init_source_to_target: o3d.core.Tensor = init_source_to_target.cuda(0)

    t_target.estimate_normals()


    return multi_scale_icp(
        t_source,
        t_target,
        voxel_sizes_multi,
        corr_distances,
        init_source_to_target,
    )
