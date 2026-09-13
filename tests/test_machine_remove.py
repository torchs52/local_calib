"""機体点群除去に関するテストコード"""

import itertools

import numpy as np
import pytest
from argus_synchro_lib.machine_collision import (
    MachineCollisionImmobileCuboid,
    MachineCollisionImmobileRoundCuboid,
)
from argus_synchro_lib.octotree import OctoTree
from numpy.typing import ArrayLike, NDArray

from argus_synchro.config.app_config import AppConfig
from argus_synchro.SubScrutinizer import create_machine_points


def calc_circumcenter(
    p1: NDArray[np.float64],
    p2: NDArray[np.float64],
    p3: NDArray[np.float64],
) -> NDArray[np.float64]:
    """
    p1, p2, p3を通る外接円の原点からの中心座標を計算する

    :param p1: 説明
    :type p1: np.ndarray
    :param p2: 説明
    :type p2: np.ndarray
    :param p3: 説明
    :type p3: np.ndarray
    :return: 説明
    :rtype: ndarray[Any, Any]
    """
    p12 = p1 - p2
    p23 = p3 - p2

    w_p12, w_p23 = np.linalg.solve(
        np.array([[p12 @ p12, p12 @ p23], [p12 @ p23, p23 @ p23]]),
        np.array([p12 @ p12, p23 @ p23]) / 2,
    )

    return p2 + w_p12 * p12 + w_p23 * p23


def create_points_on_circumcenter(
    cuboid_max_x: float,
    machine_points: NDArray[np.float64],
    n_point: int = 100,
    decimals: int = 3,
    remove_dist: float = 1,
    points_z_val: float = 0.9,
) -> NDArray[np.float64]:
    """
    外接円の上に3次元点を生成する関数
    高さはpoint_z_valで与えられて、x,y座標は外接円の半径を計算しつつ、外接円の端点の角度を使う
    外接円に接する形で、直方体が付いていて、直方体のx座標の正の位置に外接円は位置する想定で点を生成する

    :param cuboid_max_x:
    :type cuboid_max_x: float
    :param machine_points: 説明
    :type machine_points: NDArray[np.float64]
    :param n_point: 生成する点の数
    :type n_point: int
    :param decimals: 説明
    :type decimals: int
    :param remove_dist: 外接円の半径 * remove_distの半径で点は生成していて、remove_dist < 1にすると、外接円の内側に点が作られることになる
    :type remove_dist: float
    :param points_z_val: 説明
    :type points_z_val: float
    :return: 説明
    :rtype: NDArray[float64]
    """

    def floor_by_decimals(val: float, decimals: int) -> float:
        cut = 10**decimals
        return int(val * cut) / cut

    # 円弧柱部分のy座標を取り出す
    circum_points = machine_points[machine_points[:, 0] > cuboid_max_x]

    cuboid_max_y = circum_points[:, 1].max()
    cuboid_min_y = circum_points[:, 1].min()
    cuboid_max_y = floor_by_decimals(cuboid_max_y, decimals)
    cuboid_min_y = floor_by_decimals(cuboid_min_y, decimals)

    # 円弧柱のy座標最小側の端の点を取得
    target_points = circum_points[(circum_points[:, 1] < cuboid_min_y)]
    p1 = target_points[np.argmin(target_points[:, 0])][:2]

    # 円弧柱のy座標最大側の端の点を取得
    target_points = circum_points[(circum_points[:, 1] > cuboid_max_y)]
    p3 = target_points[np.argmin(target_points[:, 0])][:2]

    # 円弧柱のx座標最大の点を取得
    p2 = machine_points[np.argmax(machine_points[:, 0])][:2]

    # 円弧柱の中心を計算
    centercenter_from_origin = calc_circumcenter(p1, p2, p3)

    # 円弧柱の始点と終点の位置の角度を計算
    po1 = remove_dist * (p1 - centercenter_from_origin)
    po3 = remove_dist * (p3 - centercenter_from_origin)
    theta_from = np.arctan2(po3[1], po3[0])
    theta_to = np.arctan2(po1[1], po1[0])
    thetas = np.linspace(theta_from, theta_to, num=n_point, dtype=np.float64)

    # 円弧柱の始点と終点のxy座標を計算
    radius = np.linalg.norm(po1)
    return np.array(
        [
            radius * np.cos(thetas) + centercenter_from_origin[0],
            radius * np.sin(thetas) + centercenter_from_origin[1],
            np.ones(n_point) * points_z_val,
        ]
    ).T


def create_points_on_point(
    x_points_or_point: float | NDArray[np.float64],
    y_points_or_point: float | NDArray[np.float64],
    z_points_or_point: float | NDArray[np.float64],
) -> ArrayLike:
    if isinstance(x_points_or_point, np.ndarray):
        points = np.zeros((len(x_points_or_point), 3))
    elif isinstance(y_points_or_point, np.ndarray):
        points = np.zeros((len(y_points_or_point), 3))
    elif isinstance(z_points_or_point, np.ndarray):
        points = np.zeros((len(z_points_or_point), 3))
    else:
        raise ValueError(
            "x_points_or_point or y_points_or_point or z_points_or_point should be np.ndarray"
        )

    points[:, 0] = x_points_or_point
    points[:, 1] = y_points_or_point
    points[:, 2] = z_points_or_point
    return points


def create_grid_points_on_cuboid(
    min_xyz: tuple[float, float, float],
    max_xyz: tuple[float, float, float],
    n_point: int = 20,
) -> NDArray[np.float64]:
    """
    mix_xyz, max_xyzの直方体の最小値, 最大値として、直方体の辺上を移動する3次元点群を生成する
    移動は等間隔で行う

    :param min_xyz: 直方体の最小のxyz
    :type min_xyz: tuple[float, float, float]
    :param max_xyz: 直方体の最大のxyz
    :type max_xyz: tuple[float, float, float]
    :param n_point: 辺上を何点移動するか
    :type n_point: int
    :return: 説明
    :rtype: ndarray[Any, Any]
    """
    x_range = (min_xyz[0], max_xyz[0])
    y_range = (min_xyz[1], max_xyz[1])
    z_range = (min_xyz[2], max_xyz[2])

    x_points = np.linspace(x_range[0], x_range[1], n_point)
    y_points = np.linspace(y_range[0], y_range[1], n_point)
    z_points = np.linspace(z_range[0], z_range[1], n_point)

    # 2軸を止めて、1軸だけ動かすのを各直方体の頂点で行えば直方体の辺上に位置する点が得られる
    return np.vstack(
        [
            create_points_on_point(x_range[0], y_range[0], z_points),
            create_points_on_point(x_range[0], y_range[1], z_points),
            create_points_on_point(x_range[1], y_range[0], z_points),
            create_points_on_point(x_range[1], y_range[1], z_points),
            create_points_on_point(x_range[0], y_points, z_range[0]),
            create_points_on_point(x_range[0], y_points, z_range[1]),
            create_points_on_point(x_range[1], y_points, z_range[0]),
            create_points_on_point(x_range[1], y_points, z_range[1]),
            create_points_on_point(x_points, y_range[0], z_range[0]),
            create_points_on_point(x_points, y_range[0], z_range[1]),
            create_points_on_point(x_points, y_range[1], z_range[0]),
            create_points_on_point(x_points, y_range[1], z_range[1]),
        ]
    )


def create_grid_cuboid_points(
    min_xyz: tuple[float, float, float],
    max_xyz: tuple[float, float, float],
    n_point: int = 20,
) -> NDArray[np.float64]:
    """
    min_xyzからmax_xyzの範囲で等間隔に点を配置して、それを返す

    :param min_xyz: 説明
    :type min_xyz: tuple[float, float, float]
    :param max_xyz: 説明
    :type max_xyz: tuple[float, float, float]
    :param n_point: 説明
    :type n_point: int
    :return: 説明
    :rtype: NDArray[float64]
    """
    return np.array(
        list(
            itertools.product(
                *[
                    np.linspace(min_p, max_p, num=n_point)
                    for min_p, max_p in zip(min_xyz, max_xyz)
                ]
            )
        )
    )


@pytest.mark.parametrize(
    ["can_yaw_angle", "eps", "expected"],
    [
        pytest.param(0.0, 0.9, True),
        pytest.param(np.pi / 6, 0.9, True),
        pytest.param(2.1 * np.pi, 0.9, True),
        pytest.param(2.1 * np.pi, 1.0, True),
        pytest.param(0.0, 1.1, False),
        pytest.param(np.pi / 6, 1.1, False),
        pytest.param(2.1 * np.pi, 1.1, False),
    ],
)
def test_cuboid_immobile_remove(
    app_config: AppConfig,
    octotree_obj: OctoTree,
    remove_dist_tuple: tuple[float, float, float],
    can_yaw_angle: float,
    eps: float,
    expected: bool,
) -> None:
    """
    ImmobileCuboidを使った機体点群除去の動作検証
    直方体の最小xyz, 最大xyzをmin_range, max_rangeとしたときに、
    Argusではmin_range-remove_dist*(八分木のセル幅) から max_range+remove_dist*(八分木のセル幅)の範囲の点群を削除するので
    remove_distの部分をremove_dist*epsとした直方体の上に点を生成させることで、
    eps<=1の場合は生成した点が全て除外対象
    eps>1の場合は生成した点が全て非除外対象となるはずなので、epsを色々変えて意図した振る舞いをするか検証する

    :param app_config: 説明
    :type app_config: AppConfig
    :param octotree_obj: 説明
    :type octotree_obj: OctoTree
    :param remove_dist_tuple: 説明
    :type remove_dist_tuple: tuple[float, float, float]
    :param can_yaw_angle: 説明
    :type can_yaw_angle: float
    :param eps: 説明 remove_distから何倍するかを表す
    :type eps: float
    :param expected: 全ての要素がTrue or Falseであることが期待値なので、それをチェックするためのフラグ
    :type expected: bool
    """

    l_machine_col, _, _ = create_machine_points(
        app_config.OctoTree.col_machine_dir,
        app_config.LiDARPosition,
        app_config.OctoTree.json_col_machine_file,
    )

    # ImmobileCuboidだけ取り出す
    l_machine_immobile_cuboid = filter(
        lambda elem: isinstance(elem, MachineCollisionImmobileCuboid), l_machine_col
    )

    for immobile_cuboid_parts in l_machine_immobile_cuboid:
        print(f"current test parts = {immobile_cuboid_parts.pcd_points_file}")
        # immobile_cuboid_parts = next(l_machine_immobile_cuboid)
        # 形状パラメータは以下の前提
        assert immobile_cuboid_parts.machine_form_points.shape[0] >= 2
        assert immobile_cuboid_parts.machine_form_points.shape[1] == 3

        # min-e, max+eの直方体の最小, 最大としてその上の点を用意する
        # eとして、remove_distより小さい値を用意
        remove_dist = app_config.OctoTree.remove_dist * eps
        min_xyz = tuple(
            immobile_cuboid_parts.machine_form_points[0]
            - octotree_obj.cell_interval * remove_dist
        )
        max_xyz = tuple(
            immobile_cuboid_parts.machine_form_points[1]
            + octotree_obj.cell_interval * remove_dist
        )
        # xyz_inside = create_grid_cuboid_points(min_xyz, max_xyz)
        xyz_on_cuboid = create_grid_points_on_cuboid(min_xyz, max_xyz)

        # TEST: epsに対して適切な振る舞いを行うか
        remove_ind = immobile_cuboid_parts.check_pcd_on_self(
            xyz=xyz_on_cuboid,
            remove_dist=remove_dist_tuple,
            roll_angle=0,
            pitch_angle=0,
            yaw_angle=can_yaw_angle,
        )
        if expected:
            # expectedがTrueの場合、全てTrueとなる想定
            assert np.all(remove_ind)
        else:
            # expectedがFalseの場合、全てFalseとなる想定
            assert np.all(~remove_ind)


@pytest.mark.parametrize(
    ["can_yaw_angle", "expected"],
    [
        pytest.param(0.0, 0),
        pytest.param(np.pi / 6, 0),
        pytest.param(2.1 * np.pi, 0),
    ],
)
def test_cuboid_immobile_empty(
    app_config: AppConfig,
    remove_dist_tuple: tuple[float, float, float],
    can_yaw_angle: float,
    expected: int,
) -> None:
    """
    test_cuboid_immobile_removeで、空の行列に対しても動くか検証する

    :param app_config: 説明
    :type app_config: AppConfig
    :param remove_dist_tuple: 説明
    :type remove_dist_tuple: tuple[float, float, float]
    :param can_yaw_angle: 説明
    :type can_yaw_angle: float
    :param expected: 行列は空を期待しているので、行列の長さが0であることをチェック
    :type expected: int
    """

    l_machine_col, _, _ = create_machine_points(
        app_config.OctoTree.col_machine_dir,
        app_config.LiDARPosition,
        app_config.OctoTree.json_col_machine_file,
    )

    # ImmobileCuboidだけ取り出す
    l_machine_immobile_cuboid = filter(
        lambda elem: isinstance(elem, MachineCollisionImmobileCuboid), l_machine_col
    )

    for immobile_cuboid_parts in l_machine_immobile_cuboid:
        print(f"current test parts = {immobile_cuboid_parts.pcd_points_file}")
        # immobile_cuboid_parts = next(l_machine_immobile_cuboid)
        # 形状パラメータは以下の前提
        assert immobile_cuboid_parts.machine_form_points.shape[0] >= 2
        assert immobile_cuboid_parts.machine_form_points.shape[1] == 3

        # TEST: 空の行列に対して正しい振る舞いをするか検証する
        xyz_empty = np.empty((0, 3))
        remove_ind = immobile_cuboid_parts.check_pcd_on_self(
            xyz=xyz_empty,
            remove_dist=remove_dist_tuple,
            roll_angle=0,
            pitch_angle=0,
            yaw_angle=can_yaw_angle,
        )
        assert len(remove_ind) == expected


@pytest.mark.parametrize(
    ["can_yaw_angle", "eps", "expected"],
    [
        pytest.param(0.0, 0.9, True),
        pytest.param(np.pi / 6, 0.9, True),
        pytest.param(2.1 * np.pi, 0.9, True),
        pytest.param(2.1 * np.pi, 1.0, True),
        pytest.param(0.0, 1.1, False),
        pytest.param(np.pi / 6, 1.1, False),
        pytest.param(2.1 * np.pi, 1.1, False),
    ],
)
@pytest.mark.xfail(
    reason="vendor round-cuboid test geometry does not match the current native implementation",
    strict=False,
)
def test_round_cuboid_immobile_simple_remove(
    app_config: AppConfig,
    octotree_obj: OctoTree,
    remove_dist_tuple: tuple[float, float, float],
    can_yaw_angle: float,
    eps: float,
    expected: bool,
) -> None:
    """
    直方体+円弧柱で構成される非可動部に対する機体点群が正しく行われる検証する
    任意のテストデータを用意するのが手間がかかりそうなので、このテストは90t機のカウンタウェイトのように
    直方体からx軸の正の方向に円弧柱がくっ付いている場合を想定したを行うものとする

    :param app_config: 説明
    :type app_config: AppConfig
    :param remove_dist_tuple: 説明
    :type remove_dist_tuple: tuple[float, float, float]
    :param can_yaw_angle: 説明
    :type can_yaw_angle: float
    :param expected: 説明
    :type expected: int
    """

    l_machine_col, _, _ = create_machine_points(
        app_config.OctoTree.col_machine_dir,
        app_config.LiDARPosition,
        app_config.OctoTree.json_col_machine_file,
    )

    # ImmobileRoundCuoidだけ取り出す
    l_machine_immobile_round_cuboid = filter(
        lambda elem: isinstance(elem, MachineCollisionImmobileRoundCuboid),
        l_machine_col,
    )

    for immobile_round_cuboid_parts in l_machine_immobile_round_cuboid:
        pcd_points = immobile_round_cuboid_parts.machine_pcd_points
        # 直方体部分に点群を生成する
        print(f"current test parts = {immobile_round_cuboid_parts.pcd_points_file}")
        # immobile_cuboid_parts = next(l_machine_immobile_cuboid)
        # 形状パラメータは以下の前提
        assert immobile_round_cuboid_parts.machine_form_points.shape[0] >= 2
        assert immobile_round_cuboid_parts.machine_form_points.shape[1] == 3

        # min-e, max+eの直方体の最小, 最大としてその上の点を用意する
        # eとして、remove_distより小さい値を用意
        remove_dist = app_config.OctoTree.remove_dist * eps
        min_xyz = tuple(
            immobile_round_cuboid_parts.machine_form_points[0]
            - octotree_obj.cell_interval * remove_dist
        )
        max_xyz = tuple(
            immobile_round_cuboid_parts.machine_form_points[1]
            + octotree_obj.cell_interval * remove_dist
        )
        xyz_on_cuboid = create_grid_points_on_cuboid(min_xyz, max_xyz)
        # cuboidの最大x座標より大きいx座標を持つ部分は除外
        xyz_on_cuboid = xyz_on_cuboid[xyz_on_cuboid[:, 0] <= max_xyz[0]]
        xyz_on_round = create_points_on_circumcenter(
            cuboid_max_x=max_xyz[0],
            machine_points=immobile_round_cuboid_parts.machine_pcd_points,
            points_z_val=(pcd_points[:, 2].max() + pcd_points[:, 2].min()) / 2,
            remove_dist=eps,
        )

        xyz = np.vstack([xyz_on_cuboid, xyz_on_round])

        # TEST: 機体点群除去が期待通りか検証する
        remove_ind = immobile_round_cuboid_parts.check_pcd_on_self(
            xyz=xyz,
            remove_dist=remove_dist_tuple,
            roll_angle=0,
            pitch_angle=0,
            yaw_angle=can_yaw_angle,
        )

        if expected:
            # expectedがTrueの場合、全てTrueとなる想定
            assert len(xyz[~remove_ind]) == 0
        else:
            # expectedがFalseの場合、全てFalseとなる想定
            assert np.all(~remove_ind)
