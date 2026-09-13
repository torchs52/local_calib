from dataclasses import dataclass

import open3d as o3d
from argus_synchro_lib.octotree import OctoTree


@dataclass
class DebugVisUpdateParams:
    """update処理に必要な引数をまとめたクラス
    基本的にNoneをデフォルト値にしておいて、Noneの挙動はupdateで別途定義することで対応している
    """

    cpp_ground_points: o3d.utility.Vector3dVector | None = None
    detect_area: o3d.geometry.TriangleMesh | None = None
    bev_mesh: o3d.geometry.TriangleMesh | None = None
    edge_det_mesh: o3d.geometry.TriangleMesh | None = None
    edge_det_occuluded_mesh: o3d.geometry.TriangleMesh | None = None
    edge_line: o3d.geometry.LineSet | None = None
    octree_pcd: OctoTree | None = None
    cpp_raw_pcd: o3d.utility.Vector3dVector | None = None
