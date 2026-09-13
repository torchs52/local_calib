from __future__ import annotations

from abc import ABC, abstractmethod

import numpy as np
from argus_synchro_lib.octotree import NodeEntity, OctoTree
from numpy.typing import NDArray

from argus_synchro.common.app_logger import AppLogger, AppLoggerFactory
from argus_synchro.config.app_config import EdgeDetectionConf, GeneralConf
from argus_synchro.edge_det import (
    EdgeDetectionIF,
    EdgeDetectionResult,
    get_around_machine,
)


class CreateObjCliffInterface(ABC):
    __slots__ = ("_logger",)

    def __init__(self, app_logger_factory: AppLoggerFactory) -> None:
        self._logger: AppLogger = app_logger_factory.register_from_type(self.__class__)

    @abstractmethod
    def put_ground_points(
        self,
        ground_points: NDArray[np.float64],
        octree_obj: OctoTree,
        target_entity: NodeEntity,
    ) -> OctoTree:
        """
        地面点群を入れるメソッド, 崖検知がonの場合は地面点群を8分木に入れるが、offの場合は何もしない

        :param self: 説明
        :param ground_points: 地面点群
        :type ground_points: NDArray
        :param octree_obj: 8分木インスタンス
        :type octree_obj: OctoTree
        :param target_entity: 崖検知を入れる属性
        :type target_entity: NodeEntity
        :return: 引数で使った八分木インスタンス
        :rtype: OctoTree
        """

    @abstractmethod
    def create_obj_cliff(
        self,
        edge_detect: EdgeDetectionIF,
        octree_obj: OctoTree,
        ground_entities: list[NodeEntity],
        edge_conf: EdgeDetectionConf,
        general_conf: GeneralConf,
    ) -> tuple[OctoTree, EdgeDetectionResult]:
        """
        崖検知インスタンスを基に、崖検知を行って、結果をEdgeDetectionResult型で返す
        :param self: 説明
        :param edge_detect: 崖検知インスタンス
        :type edge_detect: EdgeDetectionIF
        :param octree_obj: 地面点群などが入った八分木インスタンス
        :type octree_obj: OctoTree
        :param ground_entities: 地面点群が入っているNodeEntityのリスト
        :type ground_entities: list[NodeEntity]
        :param edge_conf: settings.iniのEdgeDetectionセクションのプログラム上の実体
        :type edge_conf: EdgeDetectionConf
        :param general_conf: settings.iniのGeneralセクションのプログラム上の実体
        :type general_conf: GeneralConf
        :return: 説明
        :rtype: tuple[OctoTree, EdgeDetectionResult] 八分木インスタンスと崖検知の結果が入ったインスタンス, 崖検知をしない場合は空のエッジ点などが入っている
        """


class AppliedCreateObjCliff(CreateObjCliffInterface, ABC):
    """
    崖検知を実行する場合に呼ばれるクラス
    新しいバージョンの崖検知では機体周辺の死角を除去するような処理を入れていて、死角除去がないような崖検知も含められるようにするため、階層を分けた
    """

    @abstractmethod
    def add_machine_occ(
        self,
        octotree_obj: OctoTree,
        edge_conf: EdgeDetectionConf,
        general_conf: GeneralConf,
    ) -> OctoTree:
        """
        機体周辺の死角に該当する位置を特定して、死角に地面点群を入れるメソッド
        この機能を実装していないメソッドにおいては何もしないメソッド

        :param self: 説明
        :param octotree_obj: 八分木インスタンス
        :type octotree_obj: OctoTree
        :param edge_conf: settings.iniのEdgeDetectionセクションのプログラム上の実体
        :type edge_conf: EdgeDetectionConf
        :param general_conf: settings.iniのGeneralセクションのプログラム上の実体
        :type general_conf: GeneralConf
        :return: 説明 死角位置に地面点群を入れた八分木インスタンス
        :rtype: OctoTree
        """

    def put_ground_points(
        self,
        ground_points: NDArray[np.float64],
        octree_obj: OctoTree,
        target_entity: NodeEntity,
    ) -> OctoTree:
        """
        地面点群を八分木インスタンスに入れる

        :param self: 説明
        :param ground_points: 説明
        :type ground_points: NDArray
        :param octree_obj: 説明
        :type octree_obj: OctoTree
        :param target_entity: 説明
        :type target_entity: NodeEntity
        """
        octree_obj.insert_or_entity_octonodes(
            ground_points,
            target_entity,
            entity_replace=True,
        )
        return octree_obj

    def create_obj_cliff(
        self,
        edge_detect: EdgeDetectionIF,
        octree_obj: OctoTree,
        ground_entities: list[NodeEntity],
        # ground_points: NDArray[np.float64],
        edge_conf: EdgeDetectionConf,
        general_conf: GeneralConf,
    ) -> tuple[OctoTree, EdgeDetectionResult]:
        """
        機体周りの死角除去が必要な場合は、それを行いながら結果を崖検知を行って、結果を返す

        :param self: 説明
        :param edge_detect: 説明
        :type edge_detect: EdgeDetectionIF
        :param octree_obj: 説明
        :type octree_obj: OctoTree
        :param ground_entities: 説明
        :type ground_entities: list[NodeEntity]
        :param edge_conf: 説明
        :type edge_conf: EdgeDetectionConf
        :param general_conf: 説明
        :type general_conf: GeneralConf
        :return: 説明
        :rtype: tuple[OctoTree, EdgeDetectionResult]
        """
        # 機体周りの死角除去が必要な場合は行う
        self.add_machine_occ(
            octotree_obj=octree_obj,
            edge_conf=edge_conf,
            general_conf=general_conf,
        )

        # 崖検知の実行
        edge_result = edge_detect.main(
            octree_obj,
            ground_entities,
            edge_conf,
            general_conf,
        )
        self._logger.info(f"崖点 = {len(edge_result.get_edge_points_on_ground())},")
        ## 実行結果を八分木に書き込む
        # self._put_result_to_octree(octree_obj, edge_result)

        return octree_obj, edge_result


class AppliedCreateObjNoOcclusion(AppliedCreateObjCliff):
    """
    機体周りの死角を地面点群で補わない崖検知を行う場合のクラス
    機体周りの死角を地面点群で補う処理そのものを継承するのも変な感じなので、
    死角を地面点群で補うインターフェースを作って、それを呼ぶようにしても良いかも

    例:
    class AddMachineOcc(ABC):
        add_machine_occ(...):

    class AddMachineOccOn(AddMachineOcc):
        ...
    class AddMachineOccOff(AddMachineOcc):
        ...

    class AppliedCreateObj(...):
        adder: AddMachineOcc
        ...
        adder.add_machine_occ(...)
    """

    def add_machine_occ(
        self,
        octotree_obj: OctoTree,
        edge_conf: EdgeDetectionConf,
        general_conf: GeneralConf,
    ) -> OctoTree:
        return octotree_obj


class AppliedCreateObjOcclusion(AppliedCreateObjCliff):
    """
    機体周りの死角を地面点群で補うような崖検知を行う場合のクラス
    """

    machine_pos: tuple[int, int]  # 鳥観図で機体が位置している画像上の位置
    side_range: tuple[float, float]  # デカルト座標の鳥瞰図におけるy座標の範囲
    fwd_range: tuple[float, float]  # デカルト座標の鳥瞰図におけるx座標の範囲
    target_entity: NodeEntity  # 仮想の地面点群を入れる対象のNodeEntity
    put_label: int  # どのラベルに仮想の地面点群を入れるか

    def __init__(
        self,
        fwd_range: tuple[float, float],
        side_range: tuple[float, float],
        grid_size_cartesian: tuple[float, float],
        target_entity: NodeEntity,
        put_label: int,
        app_logger_factory: AppLoggerFactory,
    ) -> None:
        super().__init__(app_logger_factory)
        self.fwd_range = fwd_range
        self.side_range = side_range
        self.target_entity = target_entity
        self.put_label = put_label

        # 鳥観図のy方向は中点, x方向は原点に機体が存在するとして計算を行う
        y_pixel = np.floor(
            (side_range[1] - side_range[0]) / grid_size_cartesian[1] / 2
        ).astype(int)
        self.machine_pos = (0, y_pixel)

    def add_machine_occ(
        self,
        octotree_obj: OctoTree,
        edge_conf: EdgeDetectionConf,
        general_conf: GeneralConf,
    ) -> OctoTree:
        """
        機体周辺の死角部分を検出して
        検出した部分の地面位置に仮想点群を作って、
        それをput_labelのラベル番号で地面点群のNodeEntityに入れる

        :param self: 説明
        :param octotree_obj: 説明
        :type octotree_obj: OctoTree
        :param edge_conf: 説明
        :type edge_conf: EdgeDetectionConf
        :param general_conf: 説明
        :type general_conf: GeneralConf
        :return: 説明
        :rtype: OctoTree
        """

        # 機体周辺の死角部分の地面点群を生成する
        machine_occ_points = get_around_machine(
            octree_obj=octotree_obj,
            fwd_range=self.fwd_range,
            side_range=self.side_range,
            grid_size=edge_conf.grid_size,
            node_entities=[self.target_entity],
            group_center=self.machine_pos,
            ground_height=general_conf.ground_height,
        )

        # 機体の死角部分をput_label, put_entityで八分木に格納する
        octotree_obj.insert_or_entity_octonodes_with_labels(
            xyz=machine_occ_points,
            labels=np.array([self.put_label] * len(machine_occ_points), dtype=np.int32),
            entity=self.target_entity,
            entity_replace=False,
            is_order=False,
        )
        return octotree_obj


class NotAppliedCreateObjCliff(CreateObjCliffInterface):
    def put_ground_points(
        self,
        ground_points: NDArray[np.float64],
        octree_obj: OctoTree,
        target_entity: NodeEntity,
    ) -> OctoTree:
        """
        地面点群を八分木インスタンスに入れる, このクラスでは何もしない

        :param self: 説明
        :param ground_points: 説明
        :type ground_points: NDArray
        :param octree_obj: 説明
        :type octree_obj: OctoTree
        :param target_entity: 説明
        :type target_entity: NodeEntity
        """
        return octree_obj

    def create_obj_cliff(
        self,
        edge_detect: EdgeDetectionIF,
        octree_obj: OctoTree,
        ground_entities: list[NodeEntity],
        # ground_points: NDArray[np.float64],
        edge_conf: EdgeDetectionConf,
        general_conf: GeneralConf,
    ) -> tuple[OctoTree, EdgeDetectionResult]:
        """
        空のエッジ点などを作って、EdgeDetectionResultを生成して、それを返す

        :param self: 説明
        :param edge_detect: 説明
        :type edge_detect: EdgeDetectionIF
        :param octree_obj: 説明
        :type octree_obj: OctoTree
        :param ground_entities: 説明
        :type ground_entities: list[NodeEntity]
        :param edge_conf: 説明
        :type edge_conf: EdgeDetectionConf
        :param general_conf: 説明
        :type general_conf: GeneralConf
        :return: 説明
        :rtype: tuple[OctoTree, EdgeDetectionResult]
        """
        edge_points: NDArray[np.float64] = np.empty((0, 3), np.float64)
        edge_lines: NDArray[np.int32] = np.empty((0, 2), np.int32)
        edge_cluster_vertices: NDArray[np.int32] = np.empty((0,), np.int32)
        edge_result = EdgeDetectionResult(
            0, 0, edge_points, edge_lines, edge_cluster_vertices
        )

        self._logger.info("崖検出は実行されず")
        return octree_obj, edge_result
