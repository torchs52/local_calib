from __future__ import annotations

import time
from abc import ABC, abstractmethod
from collections.abc import Mapping

import argus_synchro_lib.controller as octo_ctrl
import numpy as np
from argus_synchro_lib.octotree import NodeEntity, OctoTree
from numpy.typing import NDArray

from argus_synchro.common.app_logger import AppLogger, AppLoggerFactory


class OctoTreeFuncInterface(ABC):
    """八分木を使うかどうかで、必要な点群をどのように取り出すかを規定しているインターフェース"""

    @abstractmethod
    def update_machine_mobile(
        self,
        machine_mobile_points_measure: NDArray[np.float64],
        machine_mobile_points_detect: NDArray[np.float64],
        octotree_obj: OctoTree,
        yaw_angle: float,
        measure_entity: NodeEntity = NodeEntity.CRANE_MOBILE,
        detect_entity: NodeEntity = NodeEntity.CRANE_MOBILE_FOR_DET,
    ) -> OctoTree:
        """
        八分木を使っている場合、機体点群の可動部を回転させて八分木インスタンスを更新するメソッド

        :param machine_mobile_points_measure: 最短部位計算に用いる可動部の機体点群
        :type machine_mobile_points_measure:
        :param machine_mobile_points_detect: 接触可能性探索に用いる可動部の機体点群
        :type machine_mobile_points_detect: NDArray
        :param octotree_obj: 八分木インスタンス
        :type octotree_obj: octotree.octotree.OctoTree
        :param yaw_angle: CAN準拠の旋回角[rad]
        :type yaw_angle: float
        :param measure_entity: 最短部位計算に用いる可動部の機体点群を入れるNodeEntity
        :type measure_entity: NodeEntity
        :param detect_entity: 接触可能性探索に用いる可動部の機体点群を入れるNodeEntity
        :type detect_entity: NodeEntity
        :return: 説明
        :rtype: OctoTree
        """

    @abstractmethod
    def octotree_accum(
        self,
        accum_points: NDArray[np.float64],
        octotree_obj_pcd: OctoTree,
        target_entity: NodeEntity = NodeEntity.OTHER,
        point_depth: int | None = None,
    ) -> tuple[NDArray[np.float64], OctoTree]:
        """
        八分木を使っている場合、LiDARの蓄積点群を八分木に入れて、八分木に入れた点群と八分木インスタンスを返すメソッド

        :param self: 説明
        :param accum_points: LiDARの蓄積点群
        :type accum_points: NDArray[np.float64]
        :param octotree_obj_pcd: 八分木インスタンス
        :type octotree_obj_pcd: octotree.octotree.OctoTree
        :param target_entity: LiDARの蓄積点群を入れておくNodeEntity
        :type target_entity: NodeEntity
        :param point_depth: クラスタリングで取り出す八分木の深さ,Noneの場合、一番深い階層
        :type point_depth: int | None
        :return: 説明
        :rtype: tuple[Any, OctoTree] クラスタリングに用いるデータ(n,3), 八分木インスタンス
        """

    @abstractmethod
    def clustering_result(
        self,
        octotree_obj_pcd: OctoTree,
        clustered_data: NDArray[np.float64],
        labels: NDArray[np.int32],
        start_time: float,
        cluster_entity: NodeEntity = NodeEntity.OTHER,
        cluster_fail_table: Mapping[int | None, NodeEntity] | None = {
            -1: NodeEntity.UNK
        },
    ) -> OctoTree:
        """
        八分木を使う場合、クラスタリング結果を入れるメソッド

        :param self: 説明
        :param octotree_obj_pcd: 八分木インスタンス
        :type octotree_obj_pcd: OctoTree
        :param clustered_data: クラスタリングに用いたデータ (n,3)行列
        :type clustered_data: NDArray[np.float64]
        :param labels: クラスタ結果(n,)次元ベクトル
        :type labels: NDArray[np.int32]
        :param start_time: 計算開始時刻
        :type start_time: float
        :param cluster_entity: クラスタリング結果を入れるNodeEntity
        :type cluster_entity: NodeEntity
        :param cluster_fail_table: クラスタリングできなかったラベルを入れておくNodeEntityの対応表
        :type cluster_fail_table: NodeEntity
        :return: 説明
        :rtype: OctoTree データを入れた八分木インスタンス
        """


class OctoTreeFuncOn(OctoTreeFuncInterface):
    """
    八分木を使ってデータの格納やクラスタリングを行う場合に使うクラス
    """

    __slots__ = ("_logger",)

    def __init__(self, app_logger_factory: AppLoggerFactory) -> None:
        self._logger: AppLogger = app_logger_factory.register_from_type(self.__class__)

    def update_machine_mobile(
        self,
        machine_mobile_points_measure: NDArray[np.float64],
        machine_mobile_points_detect: NDArray[np.float64],
        octotree_obj: OctoTree,
        yaw_angle: float,
        measure_entity: NodeEntity = NodeEntity.CRANE_MOBILE,
        detect_entity: NodeEntity = NodeEntity.CRANE_MOBILE_FOR_DET,
    ) -> OctoTree:
        """
        旋回によって変化する機体点群を更新する

        :param self: 説明
        :param machine_mobile_points_measure: 説明
        :type machine_mobile_points_measure: NDArray[np.float64]
        :param machine_mobile_points_detect: 説明
        :type machine_mobile_points_detect: NDArray[np.float64]
        :param octotree_obj: 説明
        :type octotree_obj: OctoTree
        :param yaw_angle: can準拠の旋回角度
        :type yaw_angle: float
        :param measure_entity: 説明
        :type measure_entity: NodeEntity
        :param detect_entity: 説明
        :type detect_entity: NodeEntity
        :return: 説明
        :rtype: OctoTree
        """
        # 最短部位計算に用いる機体点群を回転させる
        octo_ctrl.update_movable_entity(
            octotree_obj,
            machine_mobile_points_measure,
            measure_entity,
            True,
            0,
            0,
            -yaw_angle,
        )

        # 接触可能性探索に用いる機体点群は既に回転済みのものを渡すので
        # 直接該当するNodeEntityに点群を入れる
        octotree_obj.insert_or_entity_octonodes(
            machine_mobile_points_detect,
            detect_entity,
            True,
        )
        return octotree_obj

    def octotree_accum(
        self,
        accum_points: NDArray[np.float64],
        octotree_obj_pcd: OctoTree,
        target_entity: NodeEntity = NodeEntity.OTHER,
        point_depth: int | None = None,
    ) -> tuple[NDArray[np.float64], OctoTree]:
        """
        八分木にLiDAR点群を入れてクラスタリングに用いるデータを出力する

        :param self: 説明
        :param accum_points: LiDARの蓄積点群
        :type accum_points: NDArray[np.float64]
        :param octotree_obj_pcd: 八分木インスタンス
        :type octotree_obj_pcd: octotree.octotree.OctoTree
        :param target_entity: LiDARの蓄積点群を入れておくNodeEntity
        :type target_entity: NodeEntity
        :param point_depth: クラスタリングで取り出す八分木の深さ,Noneの場合、一番深い階層
        :type point_depth: int | None
        :return: 説明
        :rtype: tuple[Any, OctoTree] クラスタリングに用いるデータ(n,3), 八分木インスタンス
        """
        return octo_ctrl.octotree_accum_points(
            accum_points,
            octotree_obj_pcd,
            target_entity,
            point_depth,
        )

    def clustering_result(
        self,
        octotree_obj_pcd: OctoTree,
        clustered_data: NDArray[np.float64],
        labels: NDArray[np.int32],
        start_time: float,
        cluster_entity: NodeEntity = NodeEntity.OTHER,
        cluster_fail_table: Mapping[int | None, NodeEntity] | None = {
            -1: NodeEntity.UNK
        },
    ) -> OctoTree:
        """
        クラスタリング結果を八分木に詰める
        クラスタ失敗以外はcluster_entityに入れて、
        クラスタ失敗は(cluster_fail_entity, -1)に入れる

        :param self: 説明
        :param octotree_obj_pcd: 説明
        :type octotree_obj_pcd: OctoTree
        :param clustered_data: 説明
        :type clustered_data: NDArray[np.float64]
        :param labels: 説明
        :type labels: NDArray[np.int32]
        :param start_time: 説明
        :type start_time: float
        :param cluster_entity: 説明
        :type cluster_entity: NodeEntity
        :param cluster_fail_table: 説明
        :type cluster_fail_table: Mapping[int, NodeEntity] | None
        :return: 説明
        :rtype: OctoTree
        """
        # クラスタリング結果をcluster_entityに格納する
        octotree_obj_pcd.insert_labeles_and_move_in_octonodes(
            clustered_data=clustered_data,
            labels=labels,
            cluster_entity=cluster_entity,
        )

        # cluster_fail_tableの設定があれば(cluster_fail_table.keys(), cluster_entity)をkeyに持つ部分はクラスタリングが失敗しているので、
        # cluster_fail_tableに結果を移す
        if cluster_fail_table is not None:
            octotree_obj_pcd.replace_entities_in_octonodes(
                cluster_entity, cluster_fail_table
            )
        self._logger.info("八分木ラベルにかかった時間: %f", time.time() - start_time)
        return octotree_obj_pcd


class OctoTreeFuncOff(OctoTreeFuncInterface):
    """
    八分木を用いずデータを保持する場合に呼ばれるクラス
    """

    def update_machine_mobile(
        self,
        machine_mobile_points_measure: NDArray[np.float64],
        machine_mobile_points_detect: NDArray[np.float64],
        octotree_obj: OctoTree,
        yaw_angle: float,
        measure_entity: NodeEntity = NodeEntity.CRANE_MOBILE,
        detect_entity: NodeEntity = NodeEntity.CRANE_MOBILE_FOR_DET,
    ) -> OctoTree:
        """
        八分木にデータを入れないのでそのまま八分木インスタンスを返す

        """
        return octotree_obj

    def octotree_accum(
        self,
        accum_points: NDArray[np.float64],
        octotree_obj_pcd: OctoTree,
        target_entity: NodeEntity = NodeEntity.OTHER,
        point_depth: int | None = None,
    ) -> tuple[
        NDArray[np.float64],
        OctoTree,
    ]:
        """
        八分木にデータを入れない場合、accum_pointsがクラスタリングに用いるデータなので、
        accum_poitnsを返す

        """
        func_off_downsampled_accum_points = accum_points
        return func_off_downsampled_accum_points, octotree_obj_pcd

    def clustering_result(
        self,
        octotree_obj_pcd: OctoTree,
        clustered_data: NDArray[np.float64],
        labels: NDArray[np.int32],
        start_time: float,
        cluster_entity: NodeEntity = NodeEntity.OTHER,
        cluster_fail_table: Mapping[int | None, NodeEntity] | None = {
            -1: NodeEntity.UNK
        },
    ) -> OctoTree:
        """
        八分木に入れない場合は、クラスタリング結果は何も行われない

        """
        return octotree_obj_pcd
