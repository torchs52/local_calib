import argus_synchro_lib.controller as octo_ctrl
import numpy as np
import pytest
from argus_synchro_lib.octotree import NodeClusterKey, NodeEntity, OctoTree
from numpy.typing import NDArray

import argus_synchro.SubScrutinizer as SubScrt
from argus_synchro.common.app_logger import AppLoggerFactory
from argus_synchro.config.app_config import AppConfig
from argus_synchro.interface.octotree_func import OctoTreeFuncOn


def vox_coords_to_w_med_coords(
    vox_coords: NDArray[np.int64],
    min_xyz: NDArray[np.float64],
    cell_interval: NDArray[np.float64],
) -> NDArray[np.float64]:
    """
    離散座標をLiDAR座標に変換する関数
    これは正しい前提でテストを行う
    """
    # 離散座標を格子の最小値の座標に写す
    min_coords = vox_coords * cell_interval + min_xyz

    # 最小値の座標に0.5*格子サイズすると、中心座標にになる
    return min_coords + 0.5 * cell_interval


def rotate_yaw(yaw_angle: float) -> NDArray[np.float64]:
    """
    z軸周りにyaw_angleだけ回転させた時の回転行列を返す
    これは正しい前提でテストを行う

    :param yaw_angle: 説明
    :type yaw_angle: float
    :return: 説明
    :rtype: NDArray[float64]
    """
    cos = np.cos(yaw_angle)
    sin = np.sin(yaw_angle)
    return np.array(
        [
            [cos, -sin, 0.0],
            [sin, cos, 0.0],
            [0.0, 0.0, 1.0],
        ]
    )


def test_octotree_insert_lidar(app_config: AppConfig, octotree_obj: OctoTree) -> None:
    """
    サンプルの点群を八分木に入れて、クラスタリングデータを生成する部分までをテストする
    1. データが意図したkeyに入るか
    2. 不要な重複を除外してデータを入れられるか
    3. 入れたデータをクラスタリングデータとして取り出せるか
    をテストする

    :param app_config: settings.iniの実体
    """
    # octotree = create_octotree(app_config.OctoTree)
    # セルサイズは (0.1171875, 0.1171875, 0.0390625)
    test_data = np.array(
        [
            [0.05, 0.05, 0.02],
            [0.17, 0.05, 0.02],
            [0.17, 0.17, 0.02],
            [0.17, 0.17, 0.06],
            # 重複データ of (0.05, 0.05, 0.02)
            [0.11, 0.11, 0.039],
            # 重複データ of (0.05, 0.05, 0.02)
            [0.01, 0.01, 0.01],
            # 重複データ of (0.17, 0.05, 0.02)
            [0.21, 0.11, 0.039],
            # 重複データ of (0.17, 0.05, 0.02)
            [0.12, 0.01, 0.01],
        ]
    )

    octotree_obj.insert_or_entity_octonodes(
        test_data,
        NodeEntity.OTHER,
        entity_replace=True,
    )

    may_inserted_key = NodeClusterKey(NodeEntity.OTHER, None)
    assert may_inserted_key in octotree_obj.entity_octonodes
    assert len(octotree_obj.entity_octonodes[may_inserted_key]) == 4

    clustering_data = octotree_obj.get_clustering_data_by_entity(NodeEntity.OTHER)
    may_obtained_data = vox_coords_to_w_med_coords(
        vox_coords=np.array(
            list(octotree_obj.entity_octonodes[may_inserted_key].keys())
        ),
        min_xyz=octotree_obj.min_xyz,
        cell_interval=octotree_obj.cell_interval,
    )
    assert np.allclose(clustering_data, may_obtained_data)


def test_octotree_insert_labels(
    app_config: AppConfig,
    octotree_obj: OctoTree,
    app_logger_factory: AppLoggerFactory,
) -> None:
    """
    八分木にクラスタリング結果を入れて、意図した変更が行われるかテスト
    ダミーのクラスタリング結果をクラスタリングデータに紐づけて、八分木に入れると該当するkeyにデータが入ることをテストする

    :param app_config: 説明
    """
    # octotree = create_octotree(app_config.OctoTree)
    octotree_func = OctoTreeFuncOn(app_logger_factory)
    test_data = np.array(
        [
            [0.05, 0.05, 0.02],
            [0.17, 0.05, 0.02],
            [0.17, 0.17, 0.02],
            [0.17, 0.17, 0.06],
            # 重複データ of (0.05, 0.05, 0.02)
            [0.11, 0.11, 0.039],
            # 重複データ of (0.05, 0.05, 0.02)
            [0.01, 0.01, 0.01],
            # 重複データ of (0.17, 0.05, 0.02)
            [0.21, 0.11, 0.039],
            # 重複データ of (0.17, 0.05, 0.02)
            [0.12, 0.01, 0.01],
        ]
    )

    clustering_data, _ = octotree_func.octotree_accum(
        accum_points=test_data,
        octotree_obj_pcd=octotree_obj,
        target_entity=NodeEntity.OTHER,
    )

    assert len(clustering_data) == 4

    # クラスタ部分は飛ばして、以下のようになっているとする
    # 最後の要素はクラスタリングを失敗している
    labels = np.array([0, 0, 1, -1])

    # クラスタリング結果を入れる
    octotree_func.clustering_result(
        octotree_obj_pcd=octotree_obj,
        clustered_data=clustering_data,
        labels=labels,
        start_time=0,
        cluster_entity=NodeEntity.OTHER,
        cluster_fail_table={-1: NodeEntity.UNK},
    )

    # TEST: クラスタリングのkeyが意図した通りか
    target_clustered_keys = set(octotree_obj.entity_octonodes.keys())
    maybe_clustered_keys = set(
        [
            NodeClusterKey(NodeEntity.UNK, -1),
            NodeClusterKey(NodeEntity.OTHER, 0),
            NodeClusterKey(NodeEntity.OTHER, 1),
        ]
    )
    assert target_clustered_keys == maybe_clustered_keys

    # TEST: 各keyに格納されているデータの数が意図した通りか
    target_n_points_per_cluster = {
        key: len(val) for key, val in octotree_obj.entity_octonodes.items()
    }
    maybe_n_points_per_cluster = {
        NodeClusterKey(NodeEntity.UNK, -1): 1,
        NodeClusterKey(NodeEntity.OTHER, 0): 2,
        NodeClusterKey(NodeEntity.OTHER, 1): 1,
    }
    assert target_n_points_per_cluster == maybe_n_points_per_cluster


def test_octotree_remove_entity(app_config: AppConfig, octotree_obj: OctoTree) -> None:
    """
    八分木に入っているデータが正しく削除されるかテストする

    複数のNodeEntityのデータを削除して、データが残っていないことをテストする

    :param app_config: 説明
    """
    # octotree = create_octotree(app_config.OctoTree)
    test_data = np.array(
        [
            [0.05, 0.05, 0.02],
            [0.17, 0.05, 0.02],
            [0.17, 0.17, 0.02],
            [0.17, 0.17, 0.06],
            # 重複データ of (0.05, 0.05, 0.02)
            [0.11, 0.11, 0.039],
            # 重複データ of (0.05, 0.05, 0.02)
            [0.01, 0.01, 0.01],
            # 重複データ of (0.17, 0.05, 0.02)
            [0.21, 0.11, 0.039],
            # 重複データ of (0.17, 0.05, 0.02)
            [0.12, 0.01, 0.01],
        ]
    )

    octotree_obj.insert_or_entity_octonodes(
        test_data,
        NodeEntity.OTHER,
        entity_replace=True,
    )
    octotree_obj.insert_or_entity_octonodes(
        test_data,
        NodeEntity.CRANE_IMMOBILE,
        entity_replace=True,
    )

    octotree_obj.erase_nodes_for_entities_noret(
        [NodeEntity.OTHER, NodeEntity.CRANE_IMMOBILE]
    )

    assert len(octotree_obj.entity_octonodes) == 0


def test_octotree_insert_empty(
    app_config: AppConfig,
    octotree_obj: OctoTree,
    app_logger_factory: AppLoggerFactory,
) -> None:
    """
    空の点群を八分木に入れて、意図した動作するかテストする
    :param app_config: 説明
    """
    # octotree = create_octotree(app_config.OctoTree)
    octotree_func = OctoTreeFuncOn(app_logger_factory)
    test_data = np.empty((0, 3))

    # 空のデータを八分木に入れる
    clustering_data, _ = octotree_func.octotree_accum(
        accum_points=test_data,
        octotree_obj_pcd=octotree_obj,
        target_entity=NodeEntity.OTHER,
    )

    # TEST: 空の点群を入力しても動くか
    assert (
        len(octotree_obj.entity_octonodes[NodeClusterKey(NodeEntity.OTHER, None)]) == 0
    )

    # TEST: 空のクラスタリングデータをを取り出せるか
    assert len(clustering_data) == 0

    octotree_func.clustering_result(
        octotree_obj_pcd=octotree_obj,
        clustered_data=clustering_data,
        labels=np.empty(0),
        start_time=0,
        cluster_entity=NodeEntity.OTHER,
        cluster_fail_table={-1: NodeEntity.UNK},
    )

    # TEST: 空の結果を八分木に入れると、OTHERをentityに持つkeyが存在しないか
    assert (
        len(
            list(
                filter(
                    lambda key: key.entity == NodeEntity.OTHER,
                    octotree_obj.entity_octonodes.keys(),
                )
            )
        )
        == 0
    )


@pytest.mark.parametrize(
    "can_yaw_angle",
    [
        0.0,
        np.pi / 3,
        0.12345,
        2.1 * np.pi,
    ],
)
def test_octotree_insert_machine_mobile(
    app_config: AppConfig,
    octotree_obj: OctoTree,
    can_yaw_angle: float,
) -> None:
    yaw_angle = -1 * can_yaw_angle
    (
        _,
        machine_mobile_points_measure,
        _,
    ) = SubScrt.create_machine_points(
        app_config.OctoTree.col_machine_dir,
        app_config.LiDARPosition,
        app_config.OctoTree.json_col_machine_file,
    )

    # octotree = create_octotree(app_config.OctoTree)

    # 最短部位計算用いる機体点群をアプリと同じで入れる
    octo_ctrl.update_movable_entity(
        octotree_obj=octotree_obj,
        octotree_points=machine_mobile_points_measure,
        transfered_entity=NodeEntity.CRANE_MOBILE,
        entity_replace=True,
        roll_angle=0,
        pitch_angle=0,
        yaw_angle=yaw_angle,
    )

    # このテストで用いないNodeEntity上に期待されるデータを入れる
    maybe_rot_points = machine_mobile_points_measure @ rotate_yaw(yaw_angle).T
    octotree_obj.insert_or_entity_octonodes(
        xyz=maybe_rot_points,
        entity=NodeEntity.OTHER,
        entity_replace=True,
    )

    # NDArrayで比較すると順番を保持するのが大変なので、八分木に入れてある離散座標が同じかどうかで判定する
    target_rot_vox_coords = set(
        octotree_obj.entity_octonodes[NodeClusterKey(NodeEntity.OTHER, None)].keys()
    )
    maybe_rot_vox_coords = set(
        octotree_obj.entity_octonodes[NodeClusterKey(NodeEntity.OTHER, None)].keys()
    )

    # TEST: アプリ上で行われる処理で旋回されるデータが想定コードと同じか比較する
    assert target_rot_vox_coords == maybe_rot_vox_coords
