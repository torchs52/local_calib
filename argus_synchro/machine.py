"""機体情報に関するモジュール
機体の設計情報から作られる機体のメッシュクラスやメッシュリストの生成を行う関数が入っている
Remark: open3dによる可視化で用いたモジュールなので、要らないはず
"""

from __future__ import annotations

import os
from abc import ABC, abstractmethod
from configparser import ConfigParser, ExtendedInterpolation
from typing import Self

import numpy as np
import open3d as o3d
from numpy.typing import NDArray

from argus_synchro.common import common as com
from argus_synchro.config.app_config import AppConfig
from argus_synchro.config.machine_vis import VisMachineConf, load_vis_machine_info


class MachinePartsBase(ABC):
    def __init__(self, machine_info: VisMachineConf) -> None:
        """機体クラスの一般部分"""
        self.machine_info: VisMachineConf = machine_info
        self.offsets_initial: tuple[float, float, float] = machine_info.offsets
        self.reverse_initial: tuple[float, float, float] = machine_info.reverse
        self.machine_points_npy: NDArray[np.float64] = np.empty(())

    @staticmethod
    def move_and_adjust_crane_position(
        machine_points_npy: NDArray[np.float64],
        offsets: tuple[float, ...],
        reverse: tuple[bool, bool, bool],
    ) -> NDArray[np.float64]:
        """UI上のクレーン位置や向きを変える"""
        np_crane = machine_points_npy.copy()
        M = np_crane.shape[1]

        for m in range(M):
            np_crane[:, m] = np_crane[:, m] + offsets[m]
            if reverse[m]:
                np_crane[:, m] = -1 * np_crane[:, m]
        return np_crane

    @abstractmethod
    def initialize_machine_status(
        self,
        common_offsets: tuple[float, float, float],
    ) -> MachinePartsBase: ...

    def __str__(self) -> str:
        return self.machine_info.file_base


class MachineMeshParts(MachinePartsBase):
    def __init__(self, machine_info: VisMachineConf, filename: str) -> None:
        """クレーンのパーツを表すクラス
        機体の情報を読み込んで、機体点群を出したり、機体のメッシュを出したりするクラス

        引数:
            machine_info: 機体のデータを表すクラス
            filename: 機体データのファイルパスの文字列
        """
        super().__init__(
            machine_info=machine_info,
        )

        # クレーンは剛体で形が変化することはないので、形を読み込む部分までをコンストラクタで行う
        crane3d_mesh: o3d.geometry.TriangleMesh = o3d.io.read_triangle_mesh(filename)
        machine_points_npy = np.asarray(crane3d_mesh.vertices) / 1000

        self.machine_points_npy = machine_points_npy
        self.machine_mesh = crane3d_mesh

    def initialize_machine_status(
        self,
        common_offsets: tuple[float, float, float],
    ) -> MachineMeshParts:
        """必要な並進などを行って点群とメッシュの最初の状態を設定する
        Remark: この関数を実行すると、フィールドの点群, メッシュも変化するので、フィールドのメッシュは読み込んだものを保持した場合は、関数の修正が必要

        引数:
            common_offsets: 機体の全パーツに対して共通して適用されるオフセット

        戻り値:
            自分自身をreturnする
        """
        offsets = tuple(
            map(lambda x, y: x + y, common_offsets, self.machine_info.offsets),
        )

        # np.ndarrayを並進させる
        updated_vertices = self.move_and_adjust_crane_position(
            machine_points_npy=self.machine_points_npy,
            offsets=offsets,
            reverse=self.machine_info.reverse,
        )
        self.update_vertex(updated_vertices=updated_vertices)

        # open3d関連のメッシュに対するその他の設定
        self.machine_mesh.paint_uniform_color(list(self.machine_info.color))
        self.machine_mesh.compute_vertex_normals()
        return self

    def update_vertex(
        self,
        updated_vertices: NDArray[np.float64] | o3d.utility.Vector3dVector,
    ) -> Self:
        """機体のverticesを更新する
        更新によって、open3dのverticesと、numpyで持っているverticesが更新される
        自分自身を返すようにして、必要であれば、呼んだメソッドから直接get_machine_meshを呼べるようにしている
        """
        if isinstance(updated_vertices, np.ndarray):
            self.machine_mesh.vertices = o3d.utility.Vector3dVector(updated_vertices)
            self.machine_points_npy = updated_vertices
        else:
            self.machine_mesh.vertices = updated_vertices
            self.machine_points_npy = np.asarray(updated_vertices, dtype=np.float64)
        return self

    def get_machine_mesh(self) -> o3d.geometry.TriangleMesh:
        return self.machine_mesh

    def get_machine_vertices(self) -> o3d.utility.Vector3dVector:
        return self.machine_mesh.vertices


def create_machine_lists(
    crane_dir: str,
    json_vis_machine_file: str,
    initial_offsets: com.Point3f,
) -> list[MachineMeshParts]:
    """crane_dir内の機体のcad情報を基に、機体のメッシュなどを持つMachinePartsクラスのリストを作って、それを返す

    引数:
        crane_dir: 機体のcad情報が入っているディレクトリ
        initial_offsets: 機体の初期オフセット座標

    戻り値:
        l_machine_parts: 初期のオフセットが行われた機体パーツのリスト

    """
    vis_machine_info = load_vis_machine_info(
        os.path.join(crane_dir, json_vis_machine_file),
    )
    return [
        MachineMeshParts(
            machine_info=one_machine_info,
            filename=os.path.join(crane_dir, one_machine_info.file_base),
        ).initialize_machine_status(common_offsets=initial_offsets)
        for one_machine_info in vis_machine_info
    ]


if __name__ == "__main__":
    # 各種設定ファイル
    app_ini = ConfigParser(interpolation=ExtendedInterpolation())
    app_ini.read("./config/settings.ini", "UTF-8")
    app_config = AppConfig(app_ini)
    common_offsets = (
        app_config.LiDARPosition.x_offset,
        app_config.LiDARPosition.y_offset,
        app_config.LiDARPosition.z_offset,
    )
    crane_dir = app_config.machine.vis_machine_dir
    json_file = app_config.machine.json_vis_machine_file

    vis = o3d.visualization.Visualizer()
    vis.create_window(
        width=1280 * 2,  # 幅
        height=720 * 2,  # 高さ
    )

    l_machine_parts = create_machine_lists(
        crane_dir=crane_dir,
        json_vis_machine_file="vis_machine_info.jsonc",
        initial_offsets=common_offsets,
    )

    for machine_parts in l_machine_parts:
        vis.add_geometry(machine_parts.get_machine_mesh())
    vis.run()
    vis.destroy_window()
