"""接触可能性探索で用いる機体点群データを作成するモジュール"""

from __future__ import annotations

from abc import ABCMeta, abstractmethod

import numpy as np
import scipy.spatial.distance as sp_dist
from argus_synchro_lib.octotree import NodeEntity
from numpy.typing import NDArray
from sortedcontainers import SortedDict

import argus_synchro.common.common as com
from argus_synchro import SubScrutinizer
from argus_synchro.check_update import is_required_initial_offsets_update
from argus_synchro.common.common import NDPoint3f, Point3f, RangeF
from argus_synchro.config.app_config import (
    CollisionDetectionConf,
    GeneralConf,
    LidarPositionConf,
    OctoTreeConf,
)


def get_detectable_z_range(
    general_conf: GeneralConf,
    col_det_conf: CollisionDetectionConf,
) -> tuple[float, float]:
    """接触可能性探索の高さ方向の範囲計算を行う関数
    色々な所でハードコードされていたので、ここに集約
    """
    # 地面の高さからdetectable_ground_offsetだけ上に上げる
    _det_point_ground_base = (
        general_conf.ground_height + col_det_conf.detectable_ground_offset
    )
    return (
        _det_point_ground_base,
        _det_point_ground_base + col_det_conf.detectable_height,
    )


def _circle_modulo(rad: float) -> float:
    """ラジアンの定義域を[0, 2pi)にするための処理, ラジアンを扱う処理で定義域を気にしたい場合に何度も必要なので、関数化"""
    return rad % (2 * np.pi)


def _rad2circle(theta: float) -> complex:
    """thetaを複素平面上の単位円に移す処理, thetaの近さを評価する上で用いる"""
    return np.exp(theta * 1j)


def _truncate_eval_data(
    eval_points: NDArray[np.float64],
    machine_points: NDArray[np.float64],
    min_dist: float,
    max_dist: float,
) -> NDArray[np.float64]:
    """eval_pointsに対して、machine_pointsから距離がmin_distからmax_distとなるものだけ取り出す"""
    dist_eval_machine_min: float = sp_dist.cdist(eval_points, machine_points).min(
        axis=1,
    )
    ind: int = (min_dist <= dist_eval_machine_min) & (dist_eval_machine_min <= max_dist)
    truncated_eval_points: NDArray[np.float64] = eval_points[ind]

    return truncated_eval_points


def create_eval_data_rect(
    machine_points: NDArray[np.float64],
    dist_from_center_to_vertex: float = 7.0,
    max_height: float = 2.5,
    z_range: RangeF = (0, 2.0),
    grid_interval: int = 30,
    min_dist: float = 0,
    max_dist: float = 1.0,
) -> NDArray[np.float64]:
    """機体点群回りに評価用のデータを生成する
    機体中心から4隅が等間隔になる点を直方体の4隅としてデータを生成する
    """

    machine_x, machine_y, machine_z = machine_points.mean(axis=0).tolist()
    machine_z: float = machine_points.min(axis=0)[2]
    vertex_dist: float = 1 / np.sqrt(2) * dist_from_center_to_vertex

    # 評価候補の点を作成する
    # 高さの設定
    _z_range: tuple[float, float] = (
        z_range if z_range else (0 + machine_z, max_height + machine_z)
    )
    eval_x, eval_y, eval_z = np.meshgrid(
        np.linspace(
            -vertex_dist + machine_x,
            vertex_dist + machine_x,
            num=grid_interval,
        ),
        np.linspace(
            -vertex_dist + machine_y,
            vertex_dist + machine_y,
            num=grid_interval,
        ),
        np.linspace(_z_range[0], _z_range[1], num=grid_interval),
    )
    eval_points: NDArray[np.float64] = np.hstack(
        [
            eval_x.flatten()[:, np.newaxis],
            eval_y.flatten()[:, np.newaxis],
            eval_z.flatten()[:, np.newaxis],
        ],
    )

    return _truncate_eval_data(eval_points, machine_points, min_dist, max_dist)


def create_eval_data_cylinder(
    machine_points: NDArray[np.float64],
    max_height: float = 2.5,
    z_range: tuple[float, float] = (0, 2.0),
    grid_intervals: tuple[int, int, int] = (30, 30, 30),
    min_radius: float = 0,
    max_radius: float = 1.0,
    min_dist: float = 0,
    max_dist: float = 1.0,
) -> NDArray[np.float64]:
    """機体点群回りに評価用のデータを生成する
    機体中心から円柱状にデータを生成する
    機体からmin_dist以下, max_dist以上のデータは除外される
    """

    machine_x, machine_y, machine_z = machine_points.mean(axis=0).tolist()
    machine_z: float = machine_points.min(axis=0)[2]
    # 高さの設定
    _z_range: tuple[float, float] = (
        z_range if z_range else (0 + machine_z, max_height + machine_z)
    )

    # 極座標としてgrid
    eval_radius, eval_theta, eval_z = np.meshgrid(
        np.linspace(min_radius, max_radius, num=grid_intervals[0]),
        np.linspace(0, 2 * np.pi, num=grid_intervals[1]),
        np.linspace(_z_range[0], _z_range[1], num=grid_intervals[2]),
    )

    eval_radius: NDArray[np.float64] = eval_radius.flatten()
    eval_theta: NDArray[np.float64] = eval_theta.flatten()
    eval_z: NDArray[np.float64] = eval_z.flatten()

    eval_points: NDArray[np.float64] = np.hstack(
        [
            (eval_radius * np.cos(eval_theta) + machine_x)[:, np.newaxis],
            (eval_radius * np.sin(eval_theta) + machine_y)[:, np.newaxis],
            eval_z[:, np.newaxis],
        ],
    )

    return _truncate_eval_data(eval_points, machine_points, min_dist, max_dist)


def _get_closest_radian_on_unit_circle(
    sorted_dict: SortedDict,
    target_theta: float,
) -> float:
    """sorted_dictのkeyでtarget_thetaに単位円上で一番近いものを選ぶ"""
    return min(
        sorted_dict.keys(),
        key=lambda key_theta: abs(_rad2circle(key_theta) - _rad2circle(target_theta)),
    )


class DetectableCylinderPointBase(metaclass=ABCMeta):
    detectable_tree_depth: int  # 接触可能性探索を行う木の深さ
    max_dist: float  # 機体のどの点からもmax_dist以上離れている場合は追加しない
    z_range: tuple[float, float]  # 接触可能性探索の追加点群のz座標の範囲
    grid_intervals: tuple[int, int, int]  # 格子数(動径方向, 角度, z座標)の格子数
    min_radius: float  # 追加点群動径方向の最小値
    max_radius: float  # 追加点群動径方向の最大値
    max_xyz: list[float]  # 八分木初期化initialize_octotreeで用いるmax_xyz
    min_xyz: list[float]  # 八分木初期化initialize_octotreeで用いるmax_xyz
    max_tree_depth: int  # 八分木初期化initialize_octotreeで用いるmax_tree_depth
    use_node_stats: bool  # 八分木初期化initialize_octotreeで用いるuse_node_stats
    dialate_point_size: int  # 八分木初期化initialize_octotreeで用いるdialate_point_size

    def __init__(
        self,
        detectable_tree_depth: int,
        octotree_conf: OctoTreeConf,
        dialate_point_size: int,
        z_range: RangeF,
        max_dist: float = 4.0,
        grid_intervals: tuple[int, int, int] = (20, 80, 10),
        min_radius: float = 0.1,
        max_radius: float = 7.0,
    ) -> None:
        """
        接触可能性探索の対象となる機体点群を制御するためのクラス
        特に円柱型で機体点群に接触可能性探索の点群を追加する
        可動部と非可動部で追加の点群を作る部分と取り出す部分が異なるので、基底クラスを作って、それぞれで基底クラスを継承する形で使用する

        処理の流れとしては、以下の1.を最初に行っておいて、各フレームで2.を行っている
        1. create_detectable_pointsで旋回していない場合の点群の初期化を行う
        2. get_detectable_pointsで与えられた旋回角における接触可能性探索の点群を生成する

        Note: 基本的にLayerBasedな衝突判定を行う前提でこのクラスは作られていて、NeighborBasedで衝突判定を行う場合は意図した箇所の衝突可能性探索が行われない可能性がある
         -> LayerBasedで衝突可能性探索を行うことがデフォルトになってから作ったクラスのため、デフォルト側だけ作ってある状況, NeighborBased用のクラスを作る場合は、get_detectable_pointsで非可動部と可動部で機体点群と同じ点群を返すことになると思う

        - 入力:
            - detectable_tree_depth: 接触可能性探索を行う木の深さ
            - z_range: 接触可能性探索の追加点群のz座標の範囲
            - max_dist: float  # 機体のどの点からもmax_dist以上離れている場合は追加しない
            - grid_intervals: 格子数, (動径方向, 角度, z座標)の格子数
            - min_radius, max_radius: 機体からの最小/最大半径, 機体からmin_radius, max_radiusの近さの点が接触可能性探索の点群として追加される
        """
        self.detectable_tree_depth = detectable_tree_depth
        self.max_dist = max_dist
        self.z_range = z_range
        self.grid_intervals = grid_intervals
        self.min_radius = min_radius
        self.max_radius = max_radius
        self.max_xyz: list[float] = octotree_conf.max_xyz
        self.min_xyz: list[float] = octotree_conf.min_xyz
        self.max_tree_depth: int = octotree_conf.max_tree_depth
        self.use_node_stats: bool = octotree_conf.use_node_stats
        self.dialate_point_size: int = dialate_point_size

    def create_cylinder_base_detect_points(
        self,
        machine_points: NDArray[np.float64],
    ) -> NDArray[np.float64]:
        """
        衝突可能性として、検出したい範囲の点群を生成する

        :param self: 説明
        :param machine_points: 元々の機体点群
        :type machine_points: NDArray[np.float64]
        :return: 説明
        :rtype: NDArray[float64]
        """
        # 八分木インスタンスを生成する
        pcd_octree = SubScrutinizer.initialize_octotree(
            None,
            None,
            self.max_xyz,
            self.min_xyz,
            self.max_tree_depth,
            self.use_node_stats,
            self.dialate_point_size,
            None,
            origin_w2oct=(0, 0, 0),
        )

        # 機体点群の半径max_radiusまでの円柱を作成し、機体との最短距離がmax_dist以下のものに絞る
        detect_target_points: NDArray[np.float64] = create_eval_data_cylinder(
            machine_points=machine_points,
            z_range=self.z_range,
            grid_intervals=self.grid_intervals,
            min_radius=self.min_radius,
            max_radius=self.max_radius,
            max_dist=self.max_dist,
        )

        # 旋回後の機体点群と衝突可能性を判定したい点群を八分木に入れる
        pcd_octree.insert_or_entity_octonodes(
            np.vstack([detect_target_points, machine_points]),
            entity=NodeEntity.CRANE,
        )

        # 八分木に入れた点群が衝突可能性に用いる単位の階層のどのセルに属するか計算し、実座標でそれを表現する
        np_machine_detect_points: NDArray[np.float64] = (
            pcd_octree.get_np_from_entity_octonodes_by_chunk(
                [NodeEntity.CRANE],
                tree_depth=self.detectable_tree_depth,
            )
        )

        return np_machine_detect_points

    def update_octotree_value(
        self,
        octotree_conf: OctoTreeConf,
        col_det_conf: CollisionDetectionConf,
        general_conf: GeneralConf,
        machine_points: NDArray[np.float64],
    ) -> None:
        """
        AppConfigの更新に伴うパラメータの更新

        :param self: 説明
        :param octotree_conf: 説明
        :type octotree_conf: OctoTreeConf
        :param col_det_conf: 説明
        :type col_det_conf: CollisionDetectionConf
        :param general_conf: 説明
        :type general_conf: GeneralConf
        :param machine_points: 説明
        :type machine_points: NDArray[np.float64]
        """
        self.use_node_stats = octotree_conf.use_node_stats
        self.max_dist = col_det_conf.max_dist
        self.detectable_tree_depth = (
            octotree_conf.max_tree_depth - col_det_conf.dialate_point_size
        )
        self.z_range = get_detectable_z_range(general_conf, col_det_conf)
        self.grid_intervals = col_det_conf.grid_intervals
        self.min_radius = col_det_conf.min_radius
        self.max_radius = col_det_conf.max_radius
        self.dialate_point_size = col_det_conf.dialate_point_size

        # 変更された設定値に応じて、接触可能性探索の点群も更新
        self.create_detectable_points(
            machine_points=machine_points,
        )

    @abstractmethod
    def create_detectable_points(
        self,
        machine_points: NDArray[np.float64],
    ) -> DetectableCylinderPointBase:
        msg = "接触可能性探索の点群を生成するメソッドは実装する必要があります"
        raise NotImplementedError(msg)

    @abstractmethod
    def get_detectable_points(
        self,
        yaw_angle: float | None = None,
    ) -> NDPoint3f:
        msg = "接触可能性探索の点群として必要な点群の取り方は実装する必要があります"
        raise NotImplementedError(msg)


class DetectableCylinderPointImmobile(DetectableCylinderPointBase):
    detectable_points: (
        NDArray[np.float64] | None
    )  # 衝突可能性探索の点, nullの場合衝突可能性探索の点が作られていない状態を表している

    def __init__(
        self,
        detectable_tree_depth: int,
        octotree_conf: OctoTreeConf,
        dialate_point_size: int,
        z_range: RangeF,
        max_dist: float = 4.0,
        grid_intervals: tuple[int, int, int] = (20, 80, 10),
        min_radius: float = 0.1,
        max_radius: float = 7.0,
    ) -> None:
        """
        非可動部分の接触可能性探索に用いる点群を生成するクラス

        :param detectable_tree_depth: 接触可能性探索を行う木の深さ
        :type detectable_tree_depth: int
        :param octotree_conf: settings.iniにおけるOctoTreeセクションのプログラム上の実体
        :type octotree_conf: OctoTreeConf
        :param dialate_point_size: 接触可能性探索を行う場合に、一番下の階層から何個上の階層で行うかを表す整数
        :type dialate_point_size: int
        :param z_range: 接触可能性探索として追加点群の高さの範囲を表すtuple, min, maxの順で入っている事を想定
        :type z_range: tuple
        :param max_dist: 接触可能性探索としてどこまで機体の外の点を持っておくかを制御する変数
        :type max_dist: float
        :param grid_intervals: 接触可能性探索として格子状に各軸でどれだけの数の人工的な点を作るかを表すtuple, (動径方向, 角度方向, 高さ方向)の順で入っている
        :type grid_intervals: tuple[int, int, int]
        :param min_radius: 人工的な点の動径方向の最小値
        :type min_radius: float
        :param max_radius: 人工的な点の動径方向の最大値
        :type max_radius: float
        """
        super().__init__(
            detectable_tree_depth,
            octotree_conf,
            dialate_point_size,
            z_range,
            max_dist,
            grid_intervals,
            min_radius,
            max_radius,
        )
        self.detectable_points = None

    def create_detectable_points(
        self,
        machine_points: NDArray[np.float64],
    ) -> DetectableCylinderPointBase:
        """非可動部の接触可能性探索の対象となる点群を作る
        機体中心から円柱状にデータを作って、必要な範囲の点をdectable_pointsに入れている
        """
        self.detectable_points = self.create_cylinder_base_detect_points(machine_points)
        return self

    def get_detectable_points(
        self,
        yaw_angle: float | None = None,
    ) -> NDPoint3f:
        """非可動部の接触可能性探索の対象点群を返す, yaw_angleと関係なく該当点群を返す"""
        if self.detectable_points is None:
            return np.array([])
        # Remark: 空の行列を返すので良いかは、確認したほうが良いかも
        return self.detectable_points


class DetectableCylinderPointMobile(DetectableCylinderPointBase):
    key_num: int
    yaw2machine_mobile_detectable_points: (
        dict[float, NDPoint3f] | None
    )  # 旋回角に対する接触可能性探索の点群, sorted_dictになっていて、nullでない場合与えられた旋回角に最も近い点群を返す

    def __init__(
        self,
        detectable_tree_depth: int,
        octotree_conf: OctoTreeConf,
        dialate_point_size: int,
        offset_rotate_center: Point3f,
        z_range: RangeF,
        max_dist: float = 4.0,
        grid_intervals: tuple[int, int, int] = (20, 80, 10),
        min_radius: float = 0.1,
        max_radius: float = 7.0,
        key_num: int = 8,
    ) -> None:
        """
        可動部分の接触可能性探索に用いる点群を生成するクラス
        いくつかの旋回角に対する接触可能性探索の点群を持っておいて、
        与えられた旋回角度に最も近いものを取ってくるような事を行う

        :param detectable_tree_depth: 接触可能性探索を行う木の深さ
        :type detectable_tree_depth: int
        :param octotree_conf: settings.iniにおけるOctoTreeセクションのプログラム上の実体
        :type octotree_conf: OctoTreeConf
        :param dialate_point_size: 接触可能性探索を行う場合に、一番下の階層から何個上の階層で行うかを表す整数
        :type dialate_point_size: int
        :param z_range: 接触可能性探索として追加点群の高さの範囲を表すtuple, min, maxの順で入っている事を想定
        :type z_range: tuple
        :param max_dist: 接触可能性探索としてどこまで機体の外の点を持っておくかを制御する変数
        :type max_dist: float
        :param grid_intervals: 接触可能性探索として格子状に各軸でどれだけの数の人工的な点を作るかを表すtuple, (動径方向, 角度方向, 高さ方向)の順で入っている
        :type grid_intervals: tuple[int, int, int]
        :param min_radius: 人工的な点の動径方向の最小値
        :type min_radius: float
        :param max_radius: 人工的な点の動径方向の最大値
        :type max_radius: float
        :param key_num: 何個の旋回角での接触可能性探索の点群を持っておくかを表す
        :type key_num: int
        """
        super().__init__(
            detectable_tree_depth,
            octotree_conf,
            dialate_point_size,
            z_range,
            max_dist,
            grid_intervals,
            min_radius,
            max_radius,
        )
        self.key_num: int = key_num
        self.yaw2machine_mobile_detect_points = None
        self.offset_rotate_center = np.array(offset_rotate_center)

    def create_detectable_points(
        self,
        machine_points: NDArray[np.float64],
    ) -> DetectableCylinderPointBase:
        """可動部の接触可能性探索の対象となる点群を作る
        可動部は、いくつかの旋回角に対して、その旋回角における機体点群の検出範囲をカバーするように点群を持つので、
        それをyaw2machine_mobile_detect_poitnsというハッシュテーブルで保持する
        """

        # いくつかの旋回角に対する検出範囲をカバーする点群を作る
        yaw2machine_mobile_detect_points: dict[float, NDArray[np.float64]] = {
            # 近いyaw_angleを取る都合上、keyの定義域を[0, 2pi)にしておきたいので、余りを計算
            _circle_modulo(yaw_angle): self.create_cylinder_base_detect_points(
                machine_points=com.rotate_machine(
                    machine_points,
                    -yaw_angle,
                    self.offset_rotate_center,
                ),
            )
            for yaw_angle in np.linspace(0, 2 * np.pi, num=self.key_num, endpoint=False)
        }

        self.yaw2machine_mobile_detect_points = SortedDict(
            yaw2machine_mobile_detect_points,
        )
        return self

    def get_detectable_points(
        self,
        yaw_angle: float | None = None,
    ) -> NDPoint3f:
        """非可動部の接触可能性探索の対象点群を返す,
        yaw_angleに近いkeyを持つyaw2machine_mobile_detect_pointsのvalueを返す"""

        if self.yaw2machine_mobile_detect_points is None or yaw_angle is None:
            return np.array([])

        # 与えられたyaw_angleに単位円上で最も近いyaw_angleのkeyを取り出す
        closest_yaw_angle: float = _get_closest_radian_on_unit_circle(
            self.yaw2machine_mobile_detect_points,
            yaw_angle,
        )
        return self.yaw2machine_mobile_detect_points[closest_yaw_angle]

    def update_only_mobile_value(
        self,
        offset_rotate_center: Point3f,
        key_num: int,
        machine_points: NDArray[np.float64],
    ) -> None:
        """
        可動部依存の部分を更新
        keyの数と旋回中心の変更と、それに応じたyaw2machine_mobile_detect_points更新

        :param self: 説明
        :param offset_rotate_center: 説明
        :type offset_rotate_center: Point3f
        :param key_num: 説明
        :type key_num: int
        :param machine_points: 説明
        :type machine_points: NDArray[np.float64]
        """
        self.offset_rotate_center = np.array(offset_rotate_center)
        self.key_num = key_num
        self.create_detectable_points(
            machine_points=machine_points,
        )

    def is_required_only_mobile_value_update(
        self,
        offset_rotate_center: Point3f,
        key_num: int,
    ) -> bool:
        """
        可動部だけ更新インスタンス変数の更新が必要か判定する
        """
        return (
            not np.array_equal(
                self.offset_rotate_center,
                offset_rotate_center,
            )
            or self.key_num != key_num
        )

    def is_required_octotree_update(
        self,
        initial_offset: tuple[float, float, float],
        octotree_conf: OctoTreeConf,
        col_det_conf: CollisionDetectionConf,
        lidar_pos_conf: LidarPositionConf,
        general_conf: GeneralConf,
    ) -> bool:
        """
        AppConfigの更新に伴うパラメータの更新

        :param self: 説明
        :param initial_offset: 説明
        :type initial_offset: tuple[float, float, float]
        :param octotree_conf: 説明
        :type octotree_conf: OctoTreeConf
        :param col_det_conf: 説明
        :type col_det_conf: CollisionDetectionConf
        :param lidar_pos_conf: 説明
        :type lidar_pos_conf: LidarPositionConf
        :param general_conf: 説明
        :type general_conf: GeneralConf
        :return: 説明
        :rtype: bool
        """
        return (
            self.use_node_stats != octotree_conf.use_node_stats
            or self.max_dist != col_det_conf.max_dist
            or self.z_range != get_detectable_z_range(general_conf, col_det_conf)
            or self.grid_intervals != col_det_conf.grid_intervals
            or self.min_radius != col_det_conf.min_radius
            or self.max_radius != col_det_conf.max_radius
            or self.dialate_point_size != col_det_conf.dialate_point_size
            or is_required_initial_offsets_update(initial_offset, lidar_pos_conf)
        )
