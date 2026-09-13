from __future__ import annotations

import os
from collections import deque
from collections.abc import Sequence

import numpy as np
import open3d as o3d
import pandas as pd
from argus_synchro_lib.collision_detector import CoordMethod
from argus_synchro_lib.machine_collision import (
    MachineCollisionBase,
    create_machine_collision_list,
)
from argus_synchro_lib.machine_collision import MachineConf as CppMachineConf
from argus_synchro_lib.octotree import NodeEntity, OctoTree
from numpy.typing import NDArray

from argus_synchro.AccumulatePoints import accumulate_point
from argus_synchro.common.app_logger import AppLogger, AppLoggerFactory
from argus_synchro.common.common import Point3f
from argus_synchro.config.app_config import (
    AccumulationConf,
    GeneralConf,
    LidarGridConf,
    LidarPositionConf,
    OctoTreeConf,
)
from argus_synchro.config.machine_collision import load_machine_info
from argus_synchro.experiments.conv import py_machine_info_to_cpp
from argus_synchro.interface.collision_detection import (
    AbstractCollisionDetectCreator,
    CollisionDetectLayerCreator,
    CollisionDetectNeighborCreator,
    CollisionDetectOffCreator,
)
from argus_synchro.py_octotree import (
    DetectableCylinderPointImmobile,
    DetectableCylinderPointMobile,
)
from argus_synchro.Registrate_LiDAR import registrate_two_pclouds

_logger_pcd_data: AppLogger = AppLoggerFactory.from_name("PCDData")
_logger_static_accum_points: AppLogger = AppLoggerFactory.from_name("StaticAccumPoints")


def log_register(app_logger_factory: AppLoggerFactory) -> None:
    app_logger_factory.append_logger(_logger_pcd_data)
    app_logger_factory.append_logger(_logger_static_accum_points)


def connect_path(*arg: str) -> str:
    """複数のディレクトリをつなげて返す
    ディレクトリの区切りは、スラッシュで統合
    """
    return os.sep.join(arg).replace("\\", "/")


def create_machine_points(
    machine_dir: str,
    lidarposition: LidarPositionConf,
    json_file: str | None = None,
    l_col_machine_conf: list[CppMachineConf] | None = None,
) -> tuple[list[MachineCollisionBase], NDArray[np.float64], NDArray[np.float64]]:
    """
    機体除去, 衝突判定用の機体データを生成

    :param machine_dir: 機体形状情報が入ったディレクトリ
    :type machine_dir: str
    :param lidarposition: settings.iniのLiDARPositionセクションのプログラム上の実体
    :type lidarposition: LidarPositionConf
    :param json_file: 機体形状情報が入ったjsonファイル, Noneの場合、l_col_machine_confが使われる
    :type json_file: str | None
    :param l_col_machine_conf: json_file相当のインスタンスであるMachineConfのリスト、json_fileとl_col_machine_confがNoneの場合例外を投げる
    :type l_col_machine_conf: list[CppMachineConf] | None
    :return: 機体形状情報, 可動部の機体点群, 非可動部の機体点群のtuple
    :rtype: tuple[list[Any], NDArray[float64], NDArray[float64]]
    """
    # json_file, l_col_machine_confのNoneの組み合わせに対する条件分岐
    match (json_file, l_col_machine_conf):
        case (json_file, _) if json_file is not None:
            # json_fileがNoneでなければとりあえずjsonを読み込んでlist[CppMachineConf]を作る
            py_machine_confs = load_machine_info(connect_path(machine_dir, json_file))
            _l_col_machine_conf = py_machine_info_to_cpp(py_machine_confs)
        case (None, l_col_machine_conf) if l_col_machine_conf is not None:
            # json_fileがNoneで、l_col_machine_confがNoneでなければ、l_col_machine_confを流用する
            _l_col_machine_conf = l_col_machine_conf
        case _:
            # 両方Noneであればcreate_machine_collision_listが呼べないので例外を投げる
            raise ValueError(
                "json_file, l_col_machine_confのどちらかはNoneでないことが必要です"
            )

    # 機体除去, 衝突判定用の機体データを生成
    l_machine_col: list[MachineCollisionBase] = create_machine_collision_list(
        file_dir=machine_dir,
        initial_offsets=(
            lidarposition.x_offset,
            lidarposition.y_offset,
            lidarposition.z_offset,
        ),
        l_col_machine_conf=_l_col_machine_conf,
    )
    # 旋回で動く機体点群を取り出す, machine_infoのis_mobileがTrueの場合可動部
    machine_mobile_points: NDArray[np.float64] = np.vstack(
        [
            machine_col_parts.machine_pcd_points
            for machine_col_parts in l_machine_col
            if machine_col_parts.machine_info.is_mobile
        ],
    )
    # 旋回で動かない機体点群を取り出す, machine_infoのis_mobileがFalseの場合非可動部
    machine_immobile_points: NDArray[np.float64] = np.vstack(
        [
            machine_col_parts.machine_pcd_points
            for machine_col_parts in l_machine_col
            if not machine_col_parts.machine_info.is_mobile
        ],
    )

    return l_machine_col, machine_mobile_points, machine_immobile_points


def initialize_detectable_point_generators(
    machine_mobile_points: NDArray[np.float64],
    machine_immobile_points: NDArray[np.float64],
    detectable_tree_depth: int,
    octotree_conf: OctoTreeConf,
    dialate_point_size: int,
    offset_rotate_center: Point3f,
    z_range: tuple[float, float],
    max_dist: float = 4.0,
    grid_intervals: tuple[int, int, int] = (20, 80, 10),
    min_radius: float = 0.1,
    max_radius: float = 7.0,
    key_num: int = 8,
) -> tuple[DetectableCylinderPointMobile, DetectableCylinderPointImmobile]:
    """
    接触可能性探索に必要な機体点群を生成する

    :param machine_mobile_points: 可動部の機体点群
    :type machine_mobile_points: NDArray[np.float64]
    :param machine_immobile_points: 非可動部の機体点群
    :type machine_immobile_points: NDArray[np.float64]
    :param detectable_tree_depth: 接触可能性探索を行う階層
    :type detectable_tree_depth: int
    :param octotree_conf: settings.iniにおけるOctoTreeセクションに対応するプログラム上の実体
    :type octotree_conf: OctoTreeConf
    :param dialate_point_size: 接触可能性探索で広げる大きさ
    :type dialate_point_size: int
    :param offset_rotate_center: 実座標から旋回中心までの並進量
    :type offset_rotate_center: Point3f
    :param z_range: 接触可能性探索として新しく制する点群の高さの幅
    :type z_range: tuple[float, float]
    :param max_dist: 接触可能性探索として新しく生成する点群の機体点群からの最大距離
    :type max_dist: float
    :param grid_intervals: 新しく生成する点群の各軸に対する点数
    :type grid_intervals: tuple[int, int, int]
    :param min_radius: 動径方向に生成される点の初期値
    :type min_radius: float
    :param max_radius: 動径方向に生成される点の終値
    :type max_radius: float
    :param key_num: 何個の旋回角に対して、接触可能性探索に用いる点群を生成するか
    :type key_num: int
    :return: 接触可能性探索の点群を可動部/非可動部のそれぞれで生成するインスタンス, 旋回角を入力にget_detectable_pointsを呼ぶと該当する接触可能性探索用の点群が得られる
    :rtype: tuple[DetectableCylinderPointMobile, DetectableCylinderPointImmobile]
    """
    det_point_mobile_generator = DetectableCylinderPointMobile(
        detectable_tree_depth=detectable_tree_depth,
        octotree_conf=octotree_conf,
        dialate_point_size=dialate_point_size,
        offset_rotate_center=offset_rotate_center,
        z_range=z_range,
        max_dist=max_dist,
        grid_intervals=grid_intervals,
        min_radius=min_radius,
        max_radius=max_radius,
        key_num=key_num,
    )
    det_point_mobile_generator.create_detectable_points(
        machine_points=machine_mobile_points,
    )

    det_point_immobile_generator = DetectableCylinderPointImmobile(
        detectable_tree_depth=detectable_tree_depth,
        octotree_conf=octotree_conf,
        dialate_point_size=dialate_point_size,
        z_range=z_range,
        max_dist=max_dist,
        grid_intervals=grid_intervals,
        min_radius=min_radius,
        max_radius=max_radius,
    )
    det_point_immobile_generator.create_detectable_points(
        machine_points=machine_immobile_points,
    )

    return det_point_mobile_generator, det_point_immobile_generator


def calc_cuboid_based_translation(
    machine_points: NDArray[np.float64],
    max_xyz: NDArray[np.float64],
    min_xyz: NDArray[np.float64],
    col_tree_depth: int,
) -> tuple[float, float, float]:
    """機体を直方体近似した時に、八分木を用いた衝突可能性の判定で、判定範囲をなるべく広げるような並進を与える計算を行う関数
    + 入力:
        1. machine_points: 機体点群
        2. max_xyz: 八分木の最大xyz
        3. min_xyz: 八分木の最小xyz
        4. col_tree_depth: 衝突判定を行う八分木の階層
    + 出力:
        機体を直方体近似した時に衝突可能性の判定範囲をなるべく広げるような並進

    + 背景:
        八分木を用いた衝突可能性判定で、判定範囲を広げるための並進がどうすれば知りたくて、色々試した結果、
        機体が直方体であるとき、八分木の衝突可能性判定を行うセルの幅の余りの半分をズレ量として補正すれば、機体の直方体の8隅はどの方向からも広い範囲の検知ができることが分かったため、
        その処理を行っている

    """
    # 衝突可能性判定を行うときのセルのサイズ
    col_cell_interval: NDArray[np.int32] = (max_xyz - min_xyz) / 2**col_tree_depth

    # 機体の幅を計算
    machine_size: NDArray[np.int32] = machine_points.max(axis=0) - machine_points.min(
        axis=0,
    )

    # 機体の幅とセルの幅から、並進の補正量を計算
    cell_auxiliary = (machine_size % col_cell_interval) / 2

    # 直方体の8隅を計算、補正量のほかに、8隅のどれか一つを衝突可能性判定のセル上に移す必要がある
    # TODO: 現状は原点に衝突可能性判定のセルの境界があるので原点に移す並進になっているが、非対称になったりして原点に境界が位置しなくなった場合も考慮した並進にしたほうが良い
    cuboid_trans_vec = (machine_points.min(axis=0) + cell_auxiliary) % col_cell_interval
    return tuple(cuboid_trans_vec)


# def get_octotree_instances(
#    machine_immobile_points_measure: NDArray[np.float64],
#    machine_mobile_points_measure: NDArray[np.float64],
#    machine_immobile_points_detect: NDArray[np.float64],
#    max_xyz: list[float],
#    min_xyz: list[float],
#    max_tree_depth: int,
#    use_node_stats: bool,
#    dialate_point_size: int,
#    quantile: float | None = None,
#    origin_w2oct: tuple[float, float, float] | None = None,
# ) -> tuple[OctoTree, OctoTree, OctoTree, OctoTree, OctoTree]:
#    np_max_xyz: NDArray[np.float64] = np.array(max_xyz)
#    np_min_xyz: NDArray[np.float64] = np.array(min_xyz)
#
#    if origin_w2oct:
#        _origin_w2oct: tuple[float, float, float] = origin_w2oct
#    else:
#        _origin_w2oct = calc_cuboid_based_translation(
#            machine_points=np.vstack(
#                [machine_immobile_points_measure, machine_mobile_points_measure],
#            ),
#            max_xyz=np_max_xyz,
#            min_xyz=np_min_xyz,
#            col_tree_depth=max_tree_depth - dialate_point_size,
#        )
#
#    octotree_obj_pcd = OctoTree(
#        max_xyz=np_max_xyz,
#        min_xyz=np_min_xyz,
#        max_tree_depth=max_tree_depth,
#        use_node_stats=use_node_stats,
#        quantile=quantile,
#        origin_w2oct=np.array(_origin_w2oct),
#    )
#    octotree_obj_machine_mobile_measure = OctoTree(
#        # xyz=machine_mobile_points,
#        max_xyz=np_max_xyz,
#        min_xyz=np_min_xyz,
#        max_tree_depth=max_tree_depth,
#        use_node_stats=use_node_stats,
#        quantile=quantile,
#        origin_w2oct=np.array(_origin_w2oct),
#    )
#    octotree_obj_machine_immobile_measure = OctoTree(
#        xyz=machine_immobile_points_measure,
#        max_xyz=np_max_xyz,
#        min_xyz=np_min_xyz,
#        max_tree_depth=max_tree_depth,
#        use_node_stats=use_node_stats,
#        quantile=quantile,
#        origin_w2oct=np.array(_origin_w2oct),
#    )
#
#    octotree_obj_machine_mobile_detect = OctoTree(
#        # xyz=machine_mobile_points,
#        max_xyz=np_max_xyz,
#        min_xyz=np_min_xyz,
#        max_tree_depth=max_tree_depth,
#        use_node_stats=use_node_stats,
#        quantile=quantile,
#        origin_w2oct=np.array(_origin_w2oct),
#    )
#    octotree_obj_machine_immobile_detect = OctoTree(
#        xyz=machine_immobile_points_detect,
#        max_xyz=np_max_xyz,
#        min_xyz=np_min_xyz,
#        max_tree_depth=max_tree_depth,
#        use_node_stats=use_node_stats,
#        quantile=quantile,
#        origin_w2oct=np.array(_origin_w2oct),
#    )
#
#    return (
#        octotree_obj_pcd,
#        octotree_obj_machine_mobile_measure,
#        octotree_obj_machine_immobile_measure,
#        octotree_obj_machine_mobile_detect,
#        octotree_obj_machine_immobile_detect,
#    )


def initialize_octotree(
    machine_immobile_points_measure: NDArray[np.float64] | None,
    machine_mobile_points_measure: NDArray[np.float64] | None,
    max_xyz: list[float],
    min_xyz: list[float],
    max_tree_depth: int,
    use_node_stats: bool,
    dialate_point_size: int,
    quantile: float | None = None,
    origin_w2oct: tuple[float, float, float] | None = None,
) -> OctoTree:
    """
    八分木インスタンスを作る

    :param machine_immobile_points_measure: 最短部位計算に用いる機体点群で非可動部, Noneの場合、八分木の原点計算を行わない
    :type machine_immobile_points_measure: NDArray[np.float64]
    :param machine_mobile_points_measure: 最短部位計算に用いる機体点群で可動部, 八分木の原点を並進させる場合に用いる, Noneの場合、八分木の原点計算を行わない
    :type machine_mobile_points_measure: NDArray[np.float64]
    :param max_xyz: 八分木に格納する最大のxyz座標
    :type max_xyz: list[float]
    :param min_xyz: 八分木に格納する最小のxyz座標
    :type min_xyz: list[float]
    :param max_tree_depth: 八分木の最大深さ
    :type max_tree_depth: int
    :param use_node_stats: 最短部位計算に統計量を用いるかどうか, Trueの場合用いる
    :type use_node_stats: bool
    :param dialate_point_size: 接触可能性探索で点群をどれだけ広げるかを制御するパラメータ, 八分木の原点を並進させる場合に用いる
    :type dialate_point_size: int
    :param quantile: 最短部位計算にquantileを用いる場合のquantileの値, Noneの場合用いない
    :type quantile: float | None
    :param origin_w2oct: 八分木の原点座標, 値がNoneの場合、機体点群の位置から計算が行われる
    :type origin_w2oct: tuple[float, float, float] | None
    :return: 生成された八分木インスタンス
    :rtype: OctoTree
    """
    np_max_xyz: NDArray[np.int32] = np.array(max_xyz)
    np_min_xyz: NDArray[np.int32] = np.array(min_xyz)

    # 引数の状況に応じた八分木の原点を決める処理を行う
    match (
        machine_immobile_points_measure,
        machine_mobile_points_measure,
        origin_w2oct,
    ):
        case (_, _, origin_w2oct) if origin_w2oct is not None:
            # origin_w2octがnullでなければ、それを採用
            _origin_w2oct = origin_w2oct
        case (immobile_points, mobile_points, None) if (
            immobile_points is not None and mobile_points is not None
        ):
            # origin_w2octがnullでも機体点群がnullでなければ、機体点群から原点計算
            _origin_w2oct = calc_cuboid_based_translation(
                machine_points=np.vstack(
                    [immobile_points, mobile_points],
                ),
                max_xyz=np_max_xyz,
                min_xyz=np_min_xyz,
                col_tree_depth=max_tree_depth - dialate_point_size,
            )
        case _:
            # 機体点群のどちらかがnullなら諦める
            raise ValueError(
                "machine_immobile_points_measure, machine_mobile_points_measure, origin_w2octの組み合わせ的に八分木の原点を与えられないです:"
                f"(machine_immobile_points_measure, machine_mobile_points_measure, origin_w2oct)={(machine_immobile_points_measure, machine_mobile_points_measure, origin_w2oct)}"
            )

    # 八分木生成
    return OctoTree(
        max_xyz=np.array(np_max_xyz),
        min_xyz=np.array(np_min_xyz),
        max_tree_depth=max_tree_depth,
        use_node_stats=use_node_stats,
        quantile=quantile,
        origin_w2oct=np.array(_origin_w2oct),
    )


def put_immobile_points_to_octotree(
    octotree_obj: OctoTree,
    machine_immobile_points_measure: NDArray[np.float64],
    machine_immobile_points_detect: NDArray[np.float64],
    machine_center: Point3f = (0.0, 0.0, 0.0),
) -> OctoTree:
    """
    八分木にフレーム毎に変化のないデータを入れる

    :param octotree_obj: 説明
    :type octotree_obj: OctoTree
    :param machine_immobile_points_measure: 最短部位計算も用いる非可動部の機体点群
    :type machine_immobile_points_measure: NDArray[np.float64]
    :param machine_immobile_points_detect: 接触可能性探索に用いる非可動部の機体点群
    :type machine_immobile_points_detect: NDArray[np.float64]
    :param machine_center: 付加物取り付け時の機体点群
    :type machine_center: Point3f
    :return: データを入れた後の八分木インスタンス
    :rtype: Any
    """

    # 変更しない属性をentity_octonodesに入れる
    ## 最短部位計算の機体点群
    octotree_obj.insert_or_entity_octonodes(
        xyz=machine_immobile_points_measure,
        entity=NodeEntity.CRANE_IMMOBILE,
        entity_replace=True,
        is_order=False,
    )
    ## 接触可能性探索の機体点群
    octotree_obj.insert_or_entity_octonodes(
        xyz=machine_immobile_points_detect,
        entity=NodeEntity.CRANE_IMMOBILE_FOR_DET,
        entity_replace=True,
        is_order=False,
    )
    # くも足用の機体点群
    machine_rotate_center = np.array(machine_center)[np.newaxis, :]
    octotree_obj.insert_or_entity_octonodes(
        xyz=machine_rotate_center,
        entity=NodeEntity.CRANE_EXTERNAL_GUARD,
        entity_replace=True,
        is_order=False,
    )

    return octotree_obj


def initialize_collision_detector(
    func_on: bool,
    col_det_name: str,
    coord_method: str,
) -> AbstractCollisionDetectCreator:
    """
    collision_detectorの初期化
    必要な引数の数も多くないので、今のところAppConfigを直に引数にしない作りにしている

    :param func_on: 衝突判定の実行有無
    :type func_on: bool
    :param col_det_name: 衝突判定インスタンスの文字列
    :type col_det_name: str
    :param coord_method: 最短部位計算の方法の文字列
    :type coord_method: str
    :return: 説明
    :rtype: AbstractCollisionDetectCreator
    """
    coord_method = CoordMethod.from_string(coord_method)
    match (
        func_on,
        col_det_name,
    ):
        case (True, "LayerBasedCollisionDetector"):
            return CollisionDetectLayerCreator(coord_method=coord_method)
        case (True, "LayerBasedCollisionDetector"):
            return CollisionDetectNeighborCreator(coord_method=coord_method)
        case (True, x):
            raise ValueError(
                f"collision_detector_name = is not assumed value., collision_detector_name = {x}"
            )
        case (False, _):
            return CollisionDetectOffCreator()


def initialize_accum_counter(
    lidargrid: LidarGridConf,
    accum_conf: AccumulationConf,
) -> tuple[
    NDArray[np.int32],  # counter
    NDArray[np.float64],  # accumulated_points_wo_ground
    NDArray[np.float64],  # accumulated_ground_points
    deque[NDArray[np.float64]],  # accum_points_dq
    deque[NDArray[np.float64]],  # accum_ground_dq
    int,  # accum_counter
]:
    # グリッドパラメーターからグリッドの初期値を決定
    x_max: int = int(
        np.ceil((lidargrid.side_max - lidargrid.side_min) / lidargrid.grid_size)
    )
    y_max: int = int(
        np.ceil((lidargrid.fwd_max - lidargrid.fwd_min) / lidargrid.grid_size)
    )

    # counterを初期化
    counter: NDArray[np.int32] = np.zeros(
        (y_max, x_max),
        dtype=np.int32,
    )

    accum_points: NDArray[np.float64] = np.empty(0, dtype=np.float64)
    accum_points_w_ground: NDArray[np.float64] = np.empty(0, dtype=np.float64)

    # 初期化
    accum_points_dq: deque[NDArray[np.float64]] = deque(
        maxlen=max(
            accum_conf.max_accumulated_frames,
            accum_conf.max_accumulated_frames_reduced_load,
        ),
    )
    accum_ground_dq: deque[NDArray[np.float64]] = deque(
        maxlen=max(
            accum_conf.max_accumulated_frames_ground,
            accum_conf.max_accumulated_frames_ground_reduced_load,
        ),
    )
    accum_counter: int = -1

    return (
        counter,
        accum_points,
        accum_points_w_ground,
        accum_points_dq,
        accum_ground_dq,
        accum_counter,
    )


def exe_accumulation(
    xyz: NDArray[np.float64],
    counter: NDArray[np.int32],
    accum_points: deque[NDArray[np.float64]],
    accum_ground_points: deque[NDArray[np.float64]],
    init_source_to_target: o3d.core.Tensor,
    accum_counter: int,
    delta_yaw: float,
    accumulation: AccumulationConf,
    lidargrid: LidarGridConf,
    general_info: GeneralConf,
    is_edge_detection_applied: bool,
    crane_state: bool | None,
    is_reduced_load_mode: bool = False,
) -> tuple[
    NDArray[np.int32],  # counter
    NDArray[np.float64] | None,  # accumulated_points_wo_ground
    NDArray[np.float64] | None,  # accumulated_ground_points
    deque[NDArray[np.float64]],  # accum_points_dq
    deque[NDArray[np.float64]],  # accum_ground_dq
    int,  # accum_counter
    o3d.core.Tensor,  # init_source_to_target
]:
    # 起動から10フレーム分は愚直に蓄積する
    if accum_counter < accumulation.num_skip_registration_frames:
        return accumulate_point(
            xyz,
            counter,
            accum_points,
            accum_ground_points,
            lidargrid,
            accumulation,
            general_info,
            is_edge_detection_applied,
            accum_counter=accum_counter,
            is_reduced_load_mode=is_reduced_load_mode,
        ), init_source_to_target

    if xyz.shape[0] < accumulation.thr_skip_points:
        # たまに極端に点群が少ないフレームがあるので、その場合はスキップ
        _logger_static_accum_points.info(
            f"取得された点群数: {xyz.shape[0]} が少ないのでaccumulationをスキップします。",
        )
        return (
            (
                counter,
                None,
                None,
                accum_points,
                accum_ground_points,
                accum_counter,
            ),
            init_source_to_target,
        )
    crane_state = False
    if crane_state is False:
        """クレーンが動いていない場合は、前後フレームの旋回量のみで回転させる
        """
        theta: float = np.deg2rad(delta_yaw)
        c: float
        s: float
        c, s = np.cos(theta), np.sin(theta)

        trans_mat: NDArray[np.float64] = np.array(
            [
                [c, -s, 0.0, 0.0],  # 回転のみ (並進 0)
                [s, c, 0.0, 0.0],
                [0.0, 0.0, 1.0, 0.0],
                [0.0, 0.0, 0.0, 1.0],
            ],
            dtype=np.float64,
        )

    else:
        """クレーンが動いている場合、もしくは、状態不明な場合はICPで回転と並進を計算
        """
        # crane_state is true or Noneのとき
        # 結合点群と観測された点群のレジストレーションを行う
        all_chunks: list[NDArray[np.float64]] = [*accum_points, *accum_ground_points]
        accum_points_with_ground: NDArray[np.float64] = (
            np.vstack(all_chunks) if all_chunks else np.empty((0, 3), dtype=np.float64)
        )
        trans_mat, init_source_to_target = registrate_two_pclouds(
            input_xyz=xyz,
            target_in=accum_points_with_ground,
            accumulation=accumulation,
            init_source_to_target=init_source_to_target,
            delta_yaw=delta_yaw,
            visualize=False,
        )

    return accumulate_point(
        xyz,
        counter,
        accum_points,
        accum_ground_points,
        lidargrid,
        accumulation,
        general_info,
        is_edge_detection_applied,
        trans_mat,
        accum_counter,
        is_reduced_load_mode=is_reduced_load_mode,
    ), init_source_to_target


def get_angle_data(can_df: pd.DataFrame, is_old: bool) -> pd.Series:
    if is_old:
        angle_data: pd.Series = can_df["o_msg"]
    else:
        angle_data: pd.Series = can_df["n_msg"]

    return angle_data


def transform_points(
    xyz: NDArray[np.float64],
    r: NDArray[np.float64],
    t: NDArray[np.float64] | None = None,
    in_place: bool = False,
) -> NDArray[np.float64]:
    """剛体変換(R, t)を3次元点群に適用する

    #     Parameters
    #     ----------
    #     xyz : np.ndarray
    #         (N, 3)の配列。各行(x, y, z)座標
    #     R : np.ndarray
    #         3x3回転行列。
    #     t : np.ndarray | None
    #         並進ベクトル。Noneの場合は並進しない
    #     in_place : bool, default ``False``
    #         Trueの場合はxyzを直接書き換え、Falseの場合はコピーを作成

    #     Returns
    #     -------
    #     np.ndarray
    #         変換後の点群配列
    #"""
    if xyz.shape[-1] != 3:
        msg = "xyzの次元数が(N, 3)ではありません"
        raise ValueError(msg)
    if r.shape != (3, 3):
        msg = "Rが3x3行列ではありません"
        raise ValueError(msg)
    if t is not None and t.shape != (3,):
        msg = "tが3次元ベクトルではありません"
        raise ValueError(msg)

    pts: NDArray[np.float64] = xyz if in_place else xyz.copy()
    original_shape: tuple[int, ...] = pts.shape
    pts2d: NDArray[np.float64] = pts.reshape(-1, 3)

    # 回転
    pts2d[:] = (r @ pts2d.T).T
    # 並進
    if t is not None:
        pts2d += t

    return pts.reshape(original_shape)


def rotation_matrix_from_euler(
    angles: Sequence[float] | NDArray[np.float64],
    degrees: bool = True,
) -> NDArray[np.float64]:
    """XYZオイラー角から回転行列を生成する

    Parameters
    ----------
    angles:
        *(rx, ry, rz)*。各軸 (X, Y, Z) 周りの回転角。
    degrees:
        ``True``にすると角度を度数法として解釈(内部でラジアンへ変換)

    Returns
    -------
        3x3回転行列
    """
    rx, ry, rz = angles
    if degrees:
        rx, ry, rz = np.deg2rad([rx, ry, rz])

    cx, sx = np.cos(rx), np.sin(rx)
    cy, sy = np.cos(ry), np.sin(ry)
    cz, sz = np.cos(rz), np.sin(rz)

    # Rz -> Ry -> Rx
    r_z: NDArray[np.float64] = np.array(
        [[cz, -sz, 0.0], [sz, cz, 0.0], [0.0, 0.0, 1.0]]
    )
    r_y: NDArray[np.float64] = np.array(
        [[cy, 0.0, sy], [0.0, 1.0, 0.0], [-sy, 0.0, cy]]
    )
    r_x: NDArray[np.float64] = np.array(
        [[1.0, 0.0, 0.0], [0.0, cx, -sx], [0.0, sx, cx]]
    )

    return r_z @ r_y @ r_x


def load_transform_csv(
    csv_path: str,
    degrees: bool = True,
) -> tuple[NDArray[np.float64], NDArray[np.float64]]:
    """CSVから回転・並進パラメータを読み込む

    CSVヘッダー:
    x_rot, y_rot, z_rot, x_trans, y_trans, z_trans
    """

    # ヘッダー行を読み込み、列名 -> インデックスをマップ
    with open(csv_path, encoding="utf-8") as fh:
        header: list[str] = fh.readline().strip().lower().split(",")

    required: list[str] = [
        "x_rot",
        "y_rot",
        "z_rot",
        "x_trans",
        "y_trans",
        "z_trans",
    ]

    if not all(col in header for col in required):
        raise ValueError("CSVには次の列が必要です: " + ", ".join(required))

    # 必要な列だけ読み込む
    idx: list[int] = [header.index(col) for col in required]
    data: NDArray[np.float64] = np.genfromtxt(
        csv_path,
        delimiter=",",
        skip_header=1,
        usecols=idx,
        max_rows=1,
    )

    if data.size != 6:
        msg = "CSVから6つの変換パラメータを取得できませんでした"
        raise ValueError(msg)

    rot: NDArray[np.float64] = data[:3]
    trans: NDArray[np.float64] = data[3:]

    r: NDArray[np.float64] = rotation_matrix_from_euler(rot, degrees=degrees)
    t: NDArray[np.float64] = trans.astype(float)
    return r, t


def apply_initial_transform(
    xyz: NDArray[np.float64],
    r: NDArray[np.float64],
    t: NDArray[np.float64],
    in_place: bool = False,
) -> NDArray[np.float64]:
    """CSVから読み込んだ剛体変換を点群に適用"""
    return transform_points(xyz, r, t, in_place=in_place)
