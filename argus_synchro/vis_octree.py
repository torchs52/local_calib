"""八分木に関連する表示を司るモジュール"""

from __future__ import annotations

import itertools

import numpy as np
import open3d as o3d
from argus_synchro_lib.octotree import NodeEntity, OctoTree
from numpy.typing import NDArray


def create_unit_bbox_by_lineset(
    color_list: tuple[float, float, float] | None = None,
    trans_vec: NDArray[np.float64] | None = None,
    scale: NDArray[np.float64] | None = None,
) -> o3d.geometry.LineSet:
    """trans_vecを起点として、xyz方向にscaleの長さを持たせたboxをcolor_listの色で生成する関数
    + 入力:
        1. color_list: boxの色, Noneの場合黒色になる
        2. trans_vec: 原点からどれだけ並進させるか, Noneの場合原点を起点にboxを作る
        3. scale: 各辺xyz方向にどれだけ伸ばすか, Noneの場合長さ1のboxを作る

    + 出力:
        設定値に基づいたLineSet

    """
    # 点と辺の設定
    unit_points = np.array(
        [
            [0, 0, 0],
            [1, 0, 0],
            [1, 1, 0],
            [0, 1, 0],
            [0, 0, 1],
            [1, 0, 1],
            [1, 1, 1],
            [0, 1, 1],
        ],
    )
    point_indices = [
        [0, 1],
        [1, 2],
        [2, 3],
        [3, 0],
        [0, 4],
        [1, 5],
        [2, 6],
        [3, 7],
        [4, 5],
        [5, 6],
        [6, 7],
        [7, 4],
    ]

    # scale, trans_vecに沿った並進と縮小拡大を行う
    if scale is not None:
        unit_points = scale * unit_points

    if trans_vec is not None:
        unit_points = unit_points + trans_vec

    unit_lineset = o3d.geometry.LineSet(
        points=o3d.utility.Vector3dVector(unit_points),
        lines=o3d.utility.Vector2iVector(point_indices),
    )

    # 色の変更
    if color_list:
        unit_lineset.paint_uniform_color(color_list)
    else:
        unit_lineset.paint_uniform_color([0, 0, 0])
    return unit_lineset


def create_octree_grid(
    w_max_range: NDArray[np.float64],
    w_min_range: NDArray[np.float64],
    vis_tree_depth: int = 2,
    grid_color: tuple[int, int, int] = (1, 0, 0),
) -> o3d.geometry.LineSet:
    """
    八分木の範囲において与えられた深さの八分木相当のセルのリストを作成する

    + 入力:
        1. w_max_range: 八分木の実座標での最大値
        2. w_min_range: 八分木の実座標での最小値
        3. vis_tree_depth: 表示したい八分木の深さ
        4. grid_color: 何の色でセルを作るかを決めるためのリスト
    + 出力:
        bboxes: 八分木のセル毎のbounding boxが入ったLineSet
    """
    # 何個下の階層までを表示するか
    vis_cell_size: NDArray[np.float64] = (w_max_range - w_min_range) / (
        2**vis_tree_depth
    )
    bboxes = o3d.geometry.LineSet()
    for vox_ind_x, vox_ind_y, vox_ind_z in itertools.product(
        range(2**vis_tree_depth),
        range(2**vis_tree_depth),
        range(2**vis_tree_depth),
    ):
        # bboxの始点計算
        trans_vec: NDArray[np.float64] = (
            w_min_range + np.array([vox_ind_x, vox_ind_y, vox_ind_z]) * vis_cell_size
        )
        # create_unit_bbox(

        # 始点からセルサイズに応じたbboxを作る
        bboxes += create_unit_bbox_by_lineset(
            color_list=grid_color,
            trans_vec=trans_vec,
            scale=vis_cell_size,
        )
    return bboxes


def create_grid_in_vox(
    vox_points: NDArray[np.int64],
    w_max_range: NDArray[np.float64],
    w_min_range: NDArray[np.float64],
    vis_tree_depth: int = 2,
    grid_color: tuple[float, float, float] = (1, 0, 0),
    scale_jitter: float = 0.95,
) -> o3d.geometry.LineSet:
    """
    八分木の範囲において与えられた深さの八分木相当のセルのリストを作成する

    + 入力:
        1. vox_points: 表示したい八分木の離散座標
        2. w_max_range: 八分木の実座標での最大値
        3. w_min_range: 八分木の実座標での最小値
        4. vis_tree_depth: 表示したい八分木の深さ
        5. grid_color: 何の色でセルを作るかを決めるためのリスト
        6. scale_jitter: 八分木の幅を微小に変動させるための実数, 重なり合う部分を分かるようにするための変数で1より小さい値を設定すると、隣の八分木と重ならなくなる
    + 出力:
        bboxes: 八分木のセル毎のbounding boxが入ったLineSet
    """
    # 何個下の階層までを表示するか
    vis_cell_size = (w_max_range - w_min_range) / (2**vis_tree_depth)
    trans_vecs: NDArray[np.int64] = w_min_range + vox_points * vis_cell_size

    if scale_jitter < 1:
        scale = vis_cell_size * scale_jitter
        # (vis_cell_size / max(vis_cell_size) * scale_jitter) * vis_cell_size
    else:
        scale = vis_cell_size
    # print(scale)

    bboxes = o3d.geometry.LineSet()
    for trans_vec in trans_vecs:
        bboxes += create_unit_bbox_by_lineset(
            color_list=grid_color,
            trans_vec=trans_vec,
            scale=scale,
        )
    return bboxes


def _split_intersection(
    np_source: NDArray[np.int32],
    np_dest: NDArray[np.int32],
) -> tuple[NDArray[np.int64], NDArray[np.int64], NDArray[np.int64]]:
    """sourceとdestの共通部分とそれ以外を分ける計算"""
    # 重なりを計算
    source_set: set[tuple[int]] = set(map(tuple, np_source))
    dest_set: set[tuple[int]] = set(map(tuple, np_dest))
    intersection_set: set[tuple[int]] = source_set & dest_set

    # 機体と被っている点群
    np_intersection: NDArray[np.int64] = np.array(intersection_set)
    np_source_only: NDArray[np.int64] = np.array(source_set - intersection_set)
    np_dest_only: NDArray[np.int64] = np.array(dest_set - intersection_set)

    return (np_intersection, np_source_only, np_dest_only)


def create_bboxes_existing_cell_by_entity(
    vis_tree_depth: int,
    octotree_obj: OctoTree,
    w_max_range: NDArray[np.float64],
    w_min_range: NDArray[np.float64],
    src_entities: list[NodeEntity] = [
        NodeEntity.CRANE_IMMOBILE_FOR_DET,
        NodeEntity.CRANE_MOBILE_FOR_DET,
    ],
    dest_entities: list[NodeEntity] = [
        NodeEntity.UNK,
        NodeEntity.OTHER,
        NodeEntity.HUMAN,
    ],
    intersection_grid_color: tuple[float, float, float] = (1, 0, 0),
    pcd_grid_color: tuple[float, float, float] = (0, 1, 0),
    machine_grid_color: tuple[float, float, float] = (0, 0, 1),
    scale_jitter: float = 0.99,
) -> tuple[o3d.geometry.LineSet, o3d.geometry.LineSet, o3d.geometry.LineSet]:
    vox_points_src = octotree_obj.get_vox_from_entity_octonodes_by_chunk(
        src_entities,
        tree_depth=vis_tree_depth,
    )

    vox_points_dest = octotree_obj.get_vox_from_entity_octonodes_by_chunk(
        dest_entities,
        tree_depth=vis_tree_depth,
    )

    vox_points_intersection, vox_points_machine_only, vox_points_pcd_only = (
        _split_intersection(np_source=vox_points_src, np_dest=vox_points_dest)
    )

    # 該当する八分木が存在すれば八分木セルを生成する
    # LiDAR点群のみから生成される八分木の一覧を生成する
    bboxes_vox_pcd = (
        create_grid_in_vox(
            vox_points_pcd_only,
            w_max_range=w_max_range,
            w_min_range=w_min_range,
            vis_tree_depth=vis_tree_depth,
            grid_color=pcd_grid_color,
            scale_jitter=scale_jitter,
        )
        if len(vox_points_pcd_only) > 0
        else o3d.geometry.LineSet()
    )
    # 機体点群のみから生成される八分木の一覧を生成する
    bboxes_vox_machine = (
        create_grid_in_vox(
            vox_points_machine_only,
            w_max_range=w_max_range,
            w_min_range=w_min_range,
            vis_tree_depth=vis_tree_depth,
            grid_color=machine_grid_color,
            scale_jitter=scale_jitter,
        )
        if len(vox_points_machine_only) > 0
        else o3d.geometry.LineSet()
    )
    # LiDAR点群と機体点群の共通部分から生成される八分木の一覧を生成する
    bboxes_vox_intersection: o3d.geometry.LineSet = (
        create_grid_in_vox(
            vox_points_intersection,
            w_max_range=w_max_range,
            w_min_range=w_min_range,
            vis_tree_depth=vis_tree_depth,
            grid_color=intersection_grid_color,
            scale_jitter=scale_jitter,
        )
        if len(vox_points_intersection) > 0
        else o3d.geometry.LineSet()
    )

    return (bboxes_vox_pcd, bboxes_vox_machine, bboxes_vox_intersection)
