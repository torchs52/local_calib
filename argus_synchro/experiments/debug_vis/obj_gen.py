from abc import ABC, abstractmethod
from dataclasses import dataclass

import numpy as np
import open3d as o3d
from argus_synchro_lib.octotree import NodeEntity

import argus_synchro.Subvisualize as sub_vis
from argus_synchro.config.app_config import VisualizerConf
from argus_synchro.experiments.debug_vis.params import DebugVisUpdateParams


@dataclass
class Open3dObjGenerator(ABC):
    """Open3dのVisualizer上のオブジェクトの挙動を制御するためのインタフェース
    各オブジェクトはVisualizerと
    add_geometry、update_geometryを行うことでやり取りをしているので、
    その際に必要な入力を作るための処理を記述する

    o3d_objがVisualizerとやり取りをするオブジェクトで、
    コンストラクタ時点のo3d_objをadd_geometryして
    update時のo3d_objをupdate_geometryして使う想定
    """

    o3d_obj: o3d.geometry.Geometry | None = None

    @abstractmethod
    def update(self, update_params: DebugVisUpdateParams) -> None:
        """update_paramsに入っている情報を使ってo3d_objの内部状態を更新する"""
        raise NotImplementedError("update method should be implemented")


class Open3dMeshGenerator(Open3dObjGenerator):
    def __init__(self) -> None:
        """TriangleMeshを使った生成器の抽象クラス"""
        self.o3d_obj = o3d.geometry.TriangleMesh()

    def _mesh_update(self, target_mesh: o3d.geometry.TriangleMesh) -> None:
        """o3d_objをtarget_meshの情報で更新する"""
        if self.o3d_obj:
            self.o3d_obj.vertices = target_mesh.vertices
            self.o3d_obj.triangles = target_mesh.triangles
            self.o3d_obj.vertex_colors = target_mesh.vertex_colors


class Open3dDetectAreaGenerator(Open3dMeshGenerator):
    def __init__(self) -> None:
        """検出範囲に該当するopen3dのオブジェクト"""
        super().__init__()

    def update(self, update_params: DebugVisUpdateParams) -> None:
        """検出範囲を更新する"""
        if self.o3d_obj and update_params.detect_area:
            # コンストラにタ時にo3d_objはTriangleMeshで初期化しているので
            # TriangleMeshのフィールドは使えるはず
            self._mesh_update(update_params.detect_area)


class Open3dGroundPCDGenerator(Open3dObjGenerator):
    OBJ_COLORS = (0.4, 0.4, 0.4)

    def __init__(self) -> None:
        """地面点群に該当するopen3dのオブジェクト"""
        self.o3d_obj = o3d.geometry.PointCloud()

    def update(self, update_params: DebugVisUpdateParams) -> None:
        """地面点群情報を更新する"""
        if self.o3d_obj and update_params.cpp_ground_points:
            # コンストラにタ時にo3d_objはPointCloudで初期化しているので
            # PointCloudのフィールドは使えるはず
            self.o3d_obj.points = update_params.cpp_ground_points
            self.o3d_obj.paint_uniform_color(Open3dGroundPCDGenerator.OBJ_COLORS)


class Open3dBirdEyeViewMeshGenerator(Open3dMeshGenerator):
    def __init__(self) -> None:
        """崖検出の鳥瞰図を生成するためのクラス"""
        super().__init__()

    def update(self, update_params: DebugVisUpdateParams) -> None:
        if update_params.bev_mesh:
            self._mesh_update(update_params.bev_mesh)


class Open3dEdgeDetMeshGenerator(Open3dMeshGenerator):
    def __init__(self) -> None:
        """エッジ検出の結果を扱うクラス"""
        super().__init__()

    def update(self, update_params: DebugVisUpdateParams) -> None:
        if update_params.edge_det_mesh:
            self._mesh_update(update_params.edge_det_mesh)


class Open3dEdgeDetOcculudedMeshGenerator(Open3dMeshGenerator):
    def __init__(self) -> None:
        """オクルージョン対策後のエッジ検出の結果を扱うクラス"""
        super().__init__()

    def update(self, update_params: DebugVisUpdateParams) -> None:
        if update_params.edge_det_occuluded_mesh:
            self._mesh_update(update_params.edge_det_occuluded_mesh)


class Open3dCliffLineGenerator(Open3dObjGenerator):
    OBJ_COLOR = (1, 1, 0)

    def __init__(self) -> None:
        """崖検出の結果得られた線分を表示するクラス"""
        self.o3d_obj = o3d.geometry.LineSet()

    def update(self, update_params: DebugVisUpdateParams) -> None:
        if update_params.edge_line:
            self.o3d_obj.points = update_params.edge_line.points
            self.o3d_obj.lines = update_params.edge_line.lines
            self.o3d_obj.paint_uniform_color(Open3dCliffLineGenerator.OBJ_COLOR)


class Open3dMeshUpper(Open3dMeshGenerator):
    box_size: float

    INITIAL_MEMBER = NodeEntity.OTHER

    def __init__(self, vis_conf: VisualizerConf) -> None:
        """非地面点群をTriangleMeshで表示するクラス"""
        self.box_size = vis_conf.box_size
        self.o3d_obj = sub_vis.update_boxes(
            Open3dMeshUpper.INITIAL_MEMBER, np.empty((0, 3), float), self.box_size
        )

    def update(self, update_params: DebugVisUpdateParams) -> None:
        if update_params.octree_pcd:
            octree_pcd = update_params.octree_pcd
            combined_mesh = o3d.geometry.TriangleMesh()
            for entity, color in Open3dPcdUpper.NODE2COLOR.items():
                member_points = octree_pcd.get_np_from_entity_octonodes_by_chunk(
                    [entity]
                )
                if len(member_points) == 0:
                    continue

                box_mesh = sub_vis.update_boxes(
                    entity,
                    member_points,
                    self.box_size,
                )
                combined_mesh += box_mesh

            self._mesh_update(combined_mesh)


class Open3dRawPcd(Open3dObjGenerator):
    def __init__(self) -> None:
        self.o3d_obj = o3d.geometry.PointCloud()

    def update(self, update_params: DebugVisUpdateParams) -> None:
        if update_params.cpp_raw_pcd:
            self.o3d_obj.points = update_params.cpp_raw_pcd


class Open3dPcdUpper(Open3dObjGenerator):
    NODE2COLOR: dict[NodeEntity, list[float]] = {
        NodeEntity.UNK: [0.3, 0.3, 0.3],  # Gray
        NodeEntity.HUMAN: [0.8, 0.2, 0],  # Red
        # NodeEntity.CRANE: [0, 0, 0.8],  # Blue
        # NodeEntity.CLIFF: [1, 1, 0],  # Yellow
        # NodeEntity.HIGH_3D: [1, 0, 1],  # Magenta
        # NodeEntity.LOW_3D: [0, 1, 1],  # Cyan
        NodeEntity.OTHER: [0.3, 0.3, 0.3],  # Gray
    }

    def __init__(self) -> None:
        """非地面点群をPoint Cloudで表示するクラス"""
        self.o3d_obj = o3d.geometry.PointCloud()

    def update(self, update_params: DebugVisUpdateParams) -> None:
        if update_params.octree_pcd:
            octree_pcd = update_params.octree_pcd
            combined_pcd = o3d.geometry.PointCloud()
            for entity, color in Open3dPcdUpper.NODE2COLOR.items():
                member_points = octree_pcd.get_np_from_entity_octonodes_by_chunk(
                    [entity]
                )
                if len(member_points) == 0:
                    continue

                combined_pcd += o3d.geometry.PointCloud(
                    points=o3d.utility.Vector3dVector(member_points)
                ).paint_uniform_color(color)

            self.o3d_obj.points = combined_pcd.points
            self.o3d_obj.colors = combined_pcd.colors
