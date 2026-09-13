"""崖検出で基本となるクラスを定義するモジュール"""

from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import TYPE_CHECKING

import numpy as np
import open3d as o3d
from argus_synchro_lib.octotree import NodeEntity, OctoTree

from argus_synchro.common.common import t_np_float
from argus_synchro.config import app_config as app_config_module
from argus_synchro.edge_det.const import ScanDirType
from argus_synchro.edge_det.typedef import EdgeDetError, XYTup, XYZTup
from argus_synchro.edge_det.utils import get_empty_points_lines_length, scale_to_255

if TYPE_CHECKING:
    from cv2.typing import MatLike
    from numpy.typing import NDArray


@dataclass(slots=True)
class EdgeDetectionResult:
    frame: int
    time: float
    """崖検出の結果を入れておくデータクラス

    属性:
    - edge_points: 崖のエッジのxyz座標
    - edge_lines: エッジ同士の繋がり
    - edge_length: エッジの長さ
    """

    edge_points: NDArray[np.float64]
    edge_lines: NDArray[np.int32]
    edge_length: NDArray[np.int32]

    def get_edge_points_on_ground(self) -> t_np_float:
        """edge_pointsは
        高さに対して、短冊の始点と終点の座標を持っている結果
        [(x_1, y_1, z_1_lowest), (x_1, y_1, z_1_hightest), ...]
        という形で座標を保持しているので、highest側を除く形で点を取得するメソッド
        """
        return self.edge_points[::2, :]

    def get_edge_cluster(self) -> tuple[t_np_float, NDArray[np.int32]]:
        """edge_points, edge_lengthから地面位置の点を得して、
        その点が属するクラスタを紐づけて結果を返す
        """
        edge_points_ground = self.get_edge_points_on_ground()
        edge_cluster: NDArray[np.int32] = np.zeros(len(edge_points_ground), np.int32)
        ind = 0
        for cluster_id, offset in enumerate(self.edge_length):
            edge_cluster[ind : ind + offset] = cluster_id
            ind += offset
        return edge_points_ground, edge_cluster


class EdgeDetectionIF(ABC):
    """崖検出の外部機能との入出力方法を規定するためのインターフェース,
    Scrutinizerから呼び出す崖検出のクラスはこのクラスを実装しているものとする"""

    @abstractmethod
    def main(
        self,
        octree_obj: OctoTree,
        ground_entities: list[NodeEntity],
        edge_conf: app_config_module.EdgeDetectionConf,
        general_conf: app_config_module.GeneralConf,
    ) -> EdgeDetectionResult | EdgeDetError:
        """
        崖検出のメイン処理の入出力を規定する部分
        崖検出結果のインスタンスを返す

        :param self: 説明
        :param octree_obj: 八分木インスタンス
        :type octree_obj: OctoTree
        :param ground_entities: 崖検出に用いるNodeEntityのリスト
        :type ground_entities: list[NodeEntity]
        :param edge_conf: settings.iniにおけるEdgeDetectionセクションの実体
        :type edge_conf: app_config_module.EdgeDetectionConf
        :param general_conf: settings.iniにおけるGeneralセクションの実体
        :type general_conf: app_config_module.GeneralConf
        :return: 崖検出結果のインスタンス, 失敗時はEdgeDetErrorを返す
        :rtype: EdgeDetectionResult | EdgeDetError
        """
        raise NotImplementedError(
            "崖検出をScrutinizerと連結するクラスは、この入出力になるメソッドを実装する必要があります"
        )

    @abstractmethod
    def update(self, edgedetection: app_config_module.EdgeDetectionConf) -> None:
        """
        パラメータの更新
        TODO: 単純にedgedetectionのパラメータをインスタンス変数に代入しているだけではないので、崖検出インスタンスを作るcreate_edge_detectionを呼べるようにしたほうが良い気がする

        :param self: 説明
        :param edgedetection: settings.iniにおけるEdgeDetectionセクションの実体
        :type edgedetection: app_config_module.EdgeDetectionConf
        """
        raise NotImplementedError(
            "崖検出をScrutinizerと連結するクラスは、この入出力になるメソッドを実装する必要があります"
        )

    @abstractmethod
    def create_detect_area(self, plane_depth: float = 0.1) -> o3d.geometry.TriangleMesh:
        """
        崖検出の検出範囲を薄い平面のopen3dで返す

        :param self: 説明
        :param plane_depth: 平面の薄さ
        :type plane_depth: float
        :return: 説明
        :rtype: TriangleMesh
        """
        raise NotImplementedError("implementation of create_detect_area is needed.")


class BorderDetector:
    """EdgeDetectionとMultiEdgeDetectionで空間の範囲に対して処理をするものをまとめたクラス
    EdgeDetectionやMultiEdgeDetectionで使いたいが、継承するものとも違うように感じたので、クラスとして作っておいて、
    各クラスが参照する形でクラスを作成した
    """

    scan_direction: ScanDirType
    side_range: XYTup
    fwd_range: XYTup
    side_length: float
    fwd_length: float
    grid_size: XYTup
    height_strips: float

    def __init__(
        self,
        scan_direction: ScanDirType,
        side_range: XYTup,
        fwd_range: XYTup,
        side_length: float,
        fwd_length: float,
        grid_size: XYTup,
        height_strips: float,
    ) -> None:
        """
        エッジの範囲を取り出す処理をまとめたクラス
        EdgeDetection, MultiEdgeDetectionで使っているが、EdgeDetectionPolarでは使っていないので、いずれ消した方が良い
        REMARK: 最新はborder.BorderExtractorを使っている

        :param self: 説明
        :param scan_direction: 説明
        :type scan_direction: ScanDirType
        :param side_range: 説明
        :type side_range: t_xy
        :param fwd_range: 説明
        :type fwd_range: t_xy
        :param side_length: 説明
        :type side_length: float
        :param fwd_length: 説明
        :type fwd_length: float
        :param grid_size: 説明
        :type grid_size: t_xy
        :param height_strips: 説明
        :type height_strips: float
        """
        self.scan_direction = scan_direction
        self.side_range = side_range
        self.fwd_range = fwd_range
        self.side_length = side_length
        self.fwd_length = fwd_length
        self.grid_size = grid_size
        self.height_strips = height_strips

    def _edge_search_by_label_x(
        self,
        chosen_label: filter[int],
        labeled_img: MatLike,
        z_offset: float,
        is_inverse: bool = False,
    ) -> tuple[list[XYZTup], list[int]]:
        """ラベリング画像であるlabeled_imgに対して、各ラベル毎に各x座標の最小のy座標を見つける"""
        edges_oneside: list[XYZTup] = []
        length_edge: list[int] = []
        y_range = abs(self.fwd_length)
        y_max = int(y_range / self.grid_size[1])
        search_range = range(y_max - 1, -1, -1) if is_inverse else range(y_max)

        for label_no in chosen_label:
            # ラベルに一致するx座標のインデックスを取得
            x_coords: NDArray[np.intp] = np.where(labeled_img == label_no)[1]
            unique_x_coords: NDArray[np.intp] = np.unique(
                x_coords,
            )  # 同じx座標は一度見れば良い（x方向のエッジだけみたいから）

            _length_edge = 0  # ユニークなx座標の数をエッジの長さとして記録
            for x in unique_x_coords:
                # x座標を走査し、ラベルに一致する最初のx座標を見つける
                # for y in range(y_max - 1, -1, -1):
                for y in search_range:
                    if labeled_img[y, x] == label_no:
                        # 対応するZ座標を計算
                        # z_val = im[y-1, x] + z_offset
                        z_val = z_offset
                        # エッジ座標をリストに追加（y, x, zの順）
                        edges_oneside.append((y, x, z_val))
                        _length_edge += 1
                        break  # 最初の一致点を見つけたら、そのy座標でのループを終了
            length_edge.append(_length_edge)
        return edges_oneside, length_edge

    def _edge_search_by_label_y(
        self,
        chosen_label: filter[int],
        labeled_img: MatLike,
        z_offset: float,
        is_inverse: bool = False,
    ) -> tuple[list[XYZTup], list[int]]:
        """ラベリング画像であるlabeled_imgに対して、各ラベル毎に各y座標の最小のx座標を見つける"""
        edges_oneside: list[XYZTup] = []
        length_edge: list[int] = []
        x_range = abs(self.side_length)
        x_max = int(x_range / self.grid_size[0])

        # reverseの場合は、xが大きい側から走査している
        search_range = range(x_max - 1, -1, -1) if is_inverse else range(x_max)

        # for idx in range(1, labels):  # ラベル0（背景）を除く各ラベルに対してループ
        for label_no in chosen_label:
            # ラベルに一致するy座標のインデックスを取得
            y_coords = np.where(labeled_img == label_no)[0]
            unique_y_coords = np.unique(
                y_coords,
            )  # 同じy座標は一度見れば良い（x方向のエッジだけみたいから）

            _length_edge = 0  # ユニークなx座標の数をエッジの長さとして記録
            for y in unique_y_coords:
                # x座標を小さい方から走査し、ラベルに一致する最初のx座標を見つける
                for x in search_range:
                    if labeled_img[y, x] == label_no:
                        # 対応するZ座標を計算
                        # z_val = im[y, x] + z_offset
                        z_val = z_offset
                        # エッジ座標をリストに追加（y, x, zの順）
                        edges_oneside.append((y, x, z_val))
                        _length_edge += 1
                        break  # 最初の一致点を見つけたら、そのy座標でのループを終了
            length_edge.append(_length_edge)

        return edges_oneside, length_edge

    def border_detection(
        self,
        labeled_imgs: MatLike,
        z_offset: float,
        chosen_label: list[int],
    ) -> tuple[NDArray[np.float64], list[int]]:
        """
        ラベリング画像から操作方向に基づいて、3d座標を計算する

        パラメータ:
            labeled_img (numpy.ndarray): エッジ検出で得られた二値画像をラベリングした結果の画像
            z_offset (float): Z座標のオフセット値。
            chosen_label (list): labeled_imgの中でedge_onesideの計算を行うラベル番号のリスト

        戻り値:
            (tuple):
                - edges_oneside (numpy.ndarray): 検出された輪郭の3D座標。
                - length_edge (list): 各輪郭の長さ。

        """
        edges_oneside: list[XYZTup] = []
        length_edge: list[int] = []

        # ラベル番号0は、黒い画像部分になるので、除外
        _chosen_label: filter[int] = filter(lambda elem: elem != 0, chosen_label)

        # 探索する方向に応じて崖検出を行う
        if self.scan_direction == ScanDirType.PLUS_X:
            edges_oneside, length_edge = self._edge_search_by_label_y(
                _chosen_label,
                labeled_imgs,
                z_offset,
                is_inverse=False,
            )
        elif self.scan_direction == ScanDirType.PLUS_Y:
            edges_oneside, length_edge = self._edge_search_by_label_x(
                _chosen_label,
                labeled_imgs,
                z_offset,
                is_inverse=False,
            )
        elif self.scan_direction == ScanDirType.MINUS_X:
            edges_oneside, length_edge = self._edge_search_by_label_y(
                _chosen_label,
                labeled_imgs,
                z_offset,
                is_inverse=True,
            )
        elif self.scan_direction == ScanDirType.MINUS_Y:
            edges_oneside, length_edge = self._edge_search_by_label_x(
                _chosen_label,
                labeled_imgs,
                z_offset,
                is_inverse=True,
            )
        else:
            raise NotImplementedError("他の場合はまだ未実装")

        return np.array(edges_oneside), length_edge

    def convert_2DBoader_to_3D(
        self,
        lidar_edges: tuple[
            NDArray[np.float64], NDArray[np.float64], NDArray[np.float64]
        ],
        length_edge: list[int],
    ) -> tuple[NDArray[np.float64], NDArray[np.int32], NDArray[np.int32]]:
        """
        2Dの境界線座標から3DのLineSetオブジェクトを生成する。

        この関数は、BEV画像から抽出された2Dの境界線座標を3D空間にマッピングし、それらの座標を用いてLineSetオブジェクトを生成する。各境界線は、3D空間内の線分として表現される。

        パラメータ:
            edges_oneside (numpy.ndarray): 2Dの境界線座標を含む配列。
            length_edge (list of int): 各境界線の長さを表すリスト。
            side_range (tuple of float): BEV画像のX軸に対する範囲 (min_x, max_x)。
            fwd_range (tuple of float): BEV画像のY軸に対する範囲 (min_y, max_y)。
            resolution (float): BEV画像の解像度。

        戻り値:
            o3d.geometry.LineSet: 生成された3D LineSetオブジェクト。

        処理の流れ:
            1. 2D座標を3D座標に変換。
            2. 変換された座標を用いて複数の線分を生成。
            3. これらの線分を結合してLineSetオブジェクトを作成。
        """
        if len(lidar_edges[0]) == 0:
            # 崖が存在しない場合は空のLineSetを返す
            return get_empty_points_lines_length()
        (edges_x, edges_y, edges_z) = lidar_edges

        multi_points = []
        multi_lines = []
        pc_len = 0
        for _, edge_len in enumerate(length_edge):
            # 各エッジの3D座標
            x_pc = edges_x[pc_len : pc_len + edge_len]
            y_pc = edges_y[pc_len : pc_len + edge_len]
            z_pc = edges_z[pc_len : pc_len + edge_len]

            # 3D座標と線の作成
            obj_len = len(multi_points)
            for i in range(edge_len):
                points = [
                    [x_pc[i], y_pc[i], z_pc[i]],
                    [x_pc[i], y_pc[i], z_pc[i] + self.height_strips],
                ]
                multi_points.extend(points)

                if i < edge_len - 1:
                    lines = [
                        [obj_len + 2 * i, obj_len + 1 + 2 * i],
                        [obj_len + 2 * i, obj_len + 2 + 2 * i],
                        [obj_len + 1 + 2 * i, obj_len + 3 + 2 * i],
                        [obj_len + 2 + 2 * i, obj_len + 3 + 2 * i],
                    ]
                    multi_lines.extend(lines)

            pc_len += edge_len

        return np.array(multi_points), np.array(multi_lines), np.array(length_edge)

    def pixel2lidar(
        self,
        pixel_val: NDArray[np.float64],
        z_val: NDArray[np.float64] | float,
    ) -> tuple[NDArray[np.float64], NDArray[np.float64], NDArray[np.float64]]:
        """与えられた画素部分に対応する点群上の座標を返す関数, 高さはz_valで与えられる"""

        # x_centerを使っていたが、変更していて、これで問題ないか要確認
        lidar_pos: NDArray[np.float64] = np.zeros((len(pixel_val), 3), dtype=np.float64)
        lidar_pos[:, 0] = pixel_val[:, 0] * self.grid_size[1] + self.fwd_range[0]
        lidar_pos[:, 1] = pixel_val[:, 1] * self.grid_size[0] + self.side_range[0]
        lidar_pos[:, 2] = z_val

        return lidar_pos[:, 0].astype(np.float64), lidar_pos[:, 1], lidar_pos[:, 2]

    def lidar2pixel(
        self,
        lidar_points: NDArray[np.float64],
        min_z: float = -1.88,
        max_z: float = -0.88,
        is_scaled: bool = True,
    ) -> NDArray[np.uint8]:
        """LiDAR点群を解像度に合わせて、BEVに変換する
        lidar_pointsは絞った点群として、与えられたlidar_pointsをBEVに変換するだけの関数
        インスタンス変数を使ったりしないので、クラスメソッドにする
        """

        x_lidar: NDArray[np.float64] = lidar_points[:, 0]
        y_lidar: NDArray[np.float64] = lidar_points[:, 1]
        z_lidar: NDArray[np.float64] = lidar_points[:, 2]

        # LiDAR 座標をグリッドマップ座標に変換
        x_img: NDArray[np.int32] = np.floor(
            (y_lidar - self.side_range[0]) / self.grid_size[0]
        ).astype(np.int32)
        y_img: NDArray[np.int32] = np.floor(
            (x_lidar - self.fwd_range[0]) / self.grid_size[1]
        ).astype(np.int32)

        # グリッドマップの初期化
        x_max = int(np.ceil(self.side_length / self.grid_size[0]))
        y_max = int(np.ceil(self.fwd_length / self.grid_size[1]))

        grid_map: NDArray[np.uint8] = np.zeros([y_max, x_max], dtype=np.uint8)
        if is_scaled:
            grid_map[y_img, x_img] = scale_to_255(
                z_lidar, min_value=min_z, max_value=max_z
            )
        else:
            grid_map[y_img, x_img] = z_lidar
        return grid_map
