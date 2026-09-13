"""衝突判定関連のインターフェース"""

import typing
from abc import ABC, abstractmethod

import argus_synchro_lib.controller as octo_ctrl
import numpy as np
from argus_synchro_lib.collision_detector import (
    CoordMethod,
    LayerBasedCollisionDetector,
    NeighborBasedCollisionDetector,
)
from argus_synchro_lib.controller import (
    OctotreeCollisionConfig,
    OctotreeCollisionConfigBuilder,
)
from argus_synchro_lib.octotree import NodeEntity, OctoTree

from argus_synchro.common.common import t_py_col_res
from argus_synchro.config.app_config import AppConfig

ColDetector: typing.TypeAlias = (
    NeighborBasedCollisionDetector | LayerBasedCollisionDetector
)


class AbstractCollisionDetectCreator(ABC):
    """衝突判定処理を行うインターフェース
    衝突判定の処理フローは、
    1. 衝突判定を行うかどうか
    2. 衝突判定を行う場合、どういった方法で衝突判定を行うか
    が設定値によって変わるので、それぞれに対して実装クラスを作っている
    """

    @abstractmethod
    def collision_detection(
        self,
        octotree_obj: OctoTree,
        app_config: AppConfig,
    ) -> t_py_col_res:
        """実装先のクラスで衝突判定を行って結果をt_py_col_resの形式で返す"""
        raise NotImplementedError("衝突判定の処理を行う関数を実装する必要があります")


class CollisionDetectOnCreator(AbstractCollisionDetectCreator):
    __slots__ = ("collision_detector", "conf_builder")
    collision_detector: ColDetector
    conf_builder: OctotreeCollisionConfigBuilder

    def __init__(
        self,
        collision_detector: ColDetector,
        dest_entity: NodeEntity = NodeEntity.OTHER,
    ) -> None:
        """衝突判定を行う場合に継承されるクラス
        衝突判定を実施する場合、衝突判定クラスと衝突判定処理の引数を生成する必要はあるので、それらを属性として持っているクラス

        処理フローは、衝突判定に用いる引数は、値が分かったタイミングでビルダークラスに渡して、衝突判定を行う手前で引数一覧のConfigクラスを生成して、衝突判定を行うという流れになっている
        + 入力
            1. collision_detector: 衝突判定を行うインスタンス
            2. dest_entity: 衝突判定の対象となる点群が入っているNodeEntity
        """
        self.collision_detector = collision_detector
        self.conf_builder = OctotreeCollisionConfigBuilder(dest_entity=dest_entity)

    def build_collision_detect_config(
        self,
        octotree_obj: OctoTree,
        app_config: AppConfig,
    ) -> OctotreeCollisionConfig:
        """必要な引数を基に、衝突判定で用いる引数を作る
        くも足かどうかで、引数が変わっていて、それに応じた引数のインスタンスを作成している
        + 入力:
            1. octotree_obj: 八分木インスタンス
            2. app_config: settings.iniのプログラム上の実体

        + 出力:
            OctotreeCollisionConfig: 衝突判定に用いる引数のインスタンス

        """
        self.conf_builder = self.conf_builder.setOctotree(octotree_obj)

        if app_config.General.has_external_guard:
            # くも足モード場合の設定
            return (
                self.conf_builder.setSrcDetectEntities(
                    [NodeEntity.CRANE_EXTERNAL_GUARD]
                )
                .setSrcMeasureEntities([NodeEntity.CRANE_EXTERNAL_GUARD])
                .setDistanceThreshold(None)
                .setDialatePointSize(app_config.OctoTree.max_tree_depth)
                .setDetectWindow(None)
                .build()
            )

        # 普通の設定
        return (
            self.conf_builder.setSrcDetectEntities(
                [NodeEntity.CRANE_IMMOBILE_FOR_DET, NodeEntity.CRANE_MOBILE_FOR_DET]
            )
            .setSrcMeasureEntities([NodeEntity.CRANE_IMMOBILE, NodeEntity.CRANE_MOBILE])
            .setDistanceThreshold(app_config.CollisionDetection.distance_threshold)
            .setDialatePointSize(app_config.CollisionDetection.dialate_point_size)
            .setDetectWindow(
                np.array(app_config.CollisionDetection.detect_focus_range)
                if app_config.CollisionDetection.detect_focus_range
                else None
            )
            .build()
        )

    def collision_detection(
        self,
        octotree_obj: OctoTree,
        app_config: AppConfig,
    ) -> t_py_col_res:
        """
        衝突判定を行う, 実装メソッド

        :param self:
        :param octotree_obj: 八分木インスタンス, dest_entityに該当するentity_octonodesが入っているはず
        :type octotree_obj: OctoTree
        :param app_config: settings.iniのプログラム上の実体
        :type app_config: AppConfig
        :return: Python上で衝突判定の結果として持っておくtuple型, t_py_col_resが何のaliasは定義を参照
        :rtype: t_py_col_res
        """
        col_conf = self.build_collision_detect_config(
            octotree_obj,
            app_config,
        )
        collision_clusters = octo_ctrl.octotree_collision_detection_entities(
            collision_detector=self.collision_detector,
            cfg=col_conf,
        )
        return octo_ctrl.cluster_col_map_to_py(collision_clusters)

    def update(self, coord_method: CoordMethod) -> None:
        """
        設定値が変更した際に呼ばれるメソッド
        最短部位の計算方法が変更するだけと考えてcoord_methodだけ受け取るようになっている

        :param self:
        :param coord_method: 最短部位の計算方法を表すenum
        :type coord_method: CoordMethod
        """
        self.collision_detector.coord_method = coord_method


class CollisionDetectNeighborCreator(CollisionDetectOnCreator):
    def __init__(
        self, coord_method: CoordMethod, dest_entity: NodeEntity = NodeEntity.OTHER
    ) -> None:
        """NeighborBasedで衝突判定を行うクラス"""
        super().__init__(
            NeighborBasedCollisionDetector(coord_method=coord_method),
            dest_entity=dest_entity,
        )


class CollisionDetectLayerCreator(CollisionDetectOnCreator):
    def __init__(
        self, coord_method: CoordMethod, dest_entity: NodeEntity = NodeEntity.OTHER
    ) -> None:
        """LayerBasedで衝突判定を行うクラス"""
        super().__init__(
            LayerBasedCollisionDetector(coord_method=coord_method),
            dest_entity=dest_entity,
        )


class CollisionDetectOffCreator(AbstractCollisionDetectCreator):
    def __init__(self) -> None:
        """
        衝突判定を行わない場合に使われるクラス

        :param self: 説明
        """

    def collision_detection(
        self,
        octotree_obj: OctoTree,
        app_config: AppConfig,
    ) -> t_py_col_res:
        """
        衝突判定を行うメソッド
        衝突判定を行わない場合は、空の辞書を返せばよいので、空の辞書を返している
        """
        return {}
