"""debug for cliff detection"""

from abc import ABC, abstractmethod
from typing import Self

import open3d as o3d

from argus_synchro.experiments.debug_vis.obj_gen import (
    Open3dObjGenerator,
)
from argus_synchro.experiments.debug_vis.params import DebugVisUpdateParams


class Open3dDebugVisualizer(ABC):
    """Open3dを用いてデバッグ用のオブジェクトをOpen3dに追加する時に必要な処理を記述したインターフェース

    initializeでデバッグで表示したいオブジェクトの初期化を行って、
    updateでデバッグで表示したいオブジェクトの更新を行う

    Visualizer側では、オブジェクトそのものが欲しいので、更新結果のopen3dのオブジェクを取り出すメソッドを使ってデバッグオブジェクトを反映していく
    """

    o3d_obj_gens: list[Open3dObjGenerator] | None

    def __init__(self, o3d_obj_gens: list[Open3dObjGenerator] | None = None) -> None:
        self.o3d_obj_gens = o3d_obj_gens

    @abstractmethod
    def update(self, update_params: DebugVisUpdateParams) -> Self:
        """o3d_obj_gensの各要素をupdate_paramsを用いて更新する"""
        raise NotImplementedError("update method should be implemented")

    def is_active(self) -> bool:
        """o3d_obj_gensがVisualizerとやりとりできる状態か判定する"""
        return self.o3d_obj_gens is not None

    def get_objs(self) -> list[o3d.geometry.Geometry]:
        """o3d_obj_gensをopen3dのオブジェクトに変換してリストで返す
        返せるものがなけば、空のリストを返す
        """
        if self.o3d_obj_gens:
            return [
                obj_gen.o3d_obj
                for obj_gen in self.o3d_obj_gens
                if obj_gen.o3d_obj is not None
            ]
        return []


class O3DCliffDebugVisualizer(Open3dDebugVisualizer):
    """崖検出のデバッグ時に用いるオブジェクトの生成と更新を制御するクラス"""

    def __init__(self, o3d_obj_gens: list[Open3dObjGenerator] | None = None) -> None:
        super().__init__(o3d_obj_gens)

    def update(self, update_params: DebugVisUpdateParams) -> Self:
        if self.is_active():
            for o3d_obj in self.o3d_obj_gens:
                o3d_obj.update(update_params)
        return self


class InactiveDebugVisualizer(Open3dDebugVisualizer):
    """デバッグの表示を行わない場合の処理を制御するクラス"""

    def __init__(self, o3d_obj_gens: list[Open3dObjGenerator] | None = None) -> None:
        self.o3d_obj_gens = None

    def update(self, update_params: DebugVisUpdateParams) -> Self:
        return self
