"""SCX2000の機体点群の選定を行うときに作った関数を置いておくモジュール"""

from typing import Any

import numpy as np
import pandas as pd
from numpy.typing import NDArray

from argus_synchro.common.common import is_in_interval


def groupby_argmin(df: pd.DataFrame, key: list[Any], value: Any) -> pd.DataFrame:
    return df.loc[df.groupby(key)[value].idxmin(), :]


def groupby_argmax(df: pd.DataFrame, key: list[Any], value: Any) -> pd.DataFrame:
    return df.loc[df.groupby(key)[value].idxmax(), :]


def pipe_pcd_pd_to_np(
    df: pd.DataFrame,
    pcd_columns: list[str] = ["x", "y", "z"],
) -> NDArray[np.float64]:
    """
    pandas.pipeに渡す関数として使用する想定の関数
    DataFrameからxyz座標に該当する列を取り出して、np.ndarrayに変換する関数

    :param df: 説明
    :type df: pd.DataFrame
    :param pcd_columns: 説明
    :type pcd_columns: list
    :return: 説明
    :rtype: Any
    """
    return df.reindex(columns=pcd_columns).values


def select_upper_points_v5(
    df_upper_points: pd.DataFrame,
    cell_size: tuple[float, float, float] = (0.24, 0.24, 0.08),
    diff_z_th: float = 0.7,
    min_z_th: float = 0.05,
    min_x_th: float = 0.05,
    max_x_th: float = 0.05,
    min_y_th: float = 0.05,
    max_y_th: float = 0.05,
    frac_circum: float = 0.25,
    frac_is_small_z: float = 0.25,
    random_state: int = 20250710,
) -> pd.DataFrame:
    """
    上部旋回体のupper_parts部分を取り出す関数
    x,yの最大/最小のものやzが小さいものを選んでいる

    :param df_upper_points: 説明
    :type df_upper_points: pd.DataFrame
    :param cell_size: 説明
    :type cell_size: tuple
    :param diff_z_th: is_small_zの閾値
    :type diff_z_th: float
    :param min_z_th: is_underの閾値
    :type min_z_th: float
    :param min_x_th: is_min_xの閾値
    :type min_x_th: float
    :param max_x_th: is_max_xの閾値
    :type max_x_th: float
    :param min_y_th: is_min_yの閾値
    :type min_y_th: float
    :param max_y_th: is_max_yの閾値
    :type max_y_th: float
    :param frac_circum: 説明
    :type frac_circum: float
    :param frac_is_small_z: 説明
    :type frac_is_small_z: float
    :param random_state: 説明
    :type random_state: int
    :return: 説明
    :rtype: DataFrame
    """
    df_points_info = (
        df_upper_points.assign(
            vox_x=lambda df: (df.x // cell_size[0]).astype(int),
            vox_y=lambda df: (df.y // cell_size[1]).astype(int),
            vox_z=lambda df: (df.z // cell_size[2]).astype(int),
            diff_min_z=lambda df: df.z - df.z.min(),
            diff_min_x=lambda df: df.x.max() - df.x,
            diff_max_x=lambda df: df.x - df.x.min(),
            diff_min_y=lambda df: df.y.max() - df.y,
            diff_max_y=lambda df: df.y - df.y.min(),
        )
        .pipe(
            # zが小さいものを選ぶ
            lambda df: pd.concat(
                [
                    df[df.diff_min_z < min_z_th]
                    .sample(frac=frac_circum, random_state=random_state)
                    .assign(is_under=1),
                    df.assign(is_under=0),
                ],
                ignore_index=True,
            ).drop_duplicates(["x", "y", "z"])
        )
        .pipe(
            # xが小さいものを選ぶ
            lambda df: pd.concat(
                [
                    df[df.diff_min_x < min_x_th]
                    .sample(frac=frac_circum, random_state=random_state)
                    .assign(is_min_x=1),
                    df.assign(is_min_x=0),
                ],
                ignore_index=True,
            ).drop_duplicates(["x", "y", "z"])
        )
        .pipe(
            # xが大きいものを選ぶ
            lambda df: pd.concat(
                [
                    df[df.diff_max_x < max_x_th]
                    .sample(frac=frac_circum, random_state=random_state)
                    .assign(is_max_x=1),
                    df.assign(is_max_x=0),
                ],
                ignore_index=True,
            ).drop_duplicates(["x", "y", "z"])
        )
        .pipe(
            # yが小さいものを選ぶ
            lambda df: pd.concat(
                [
                    df[df.diff_min_y < min_y_th]
                    .sample(frac=frac_circum, random_state=random_state)
                    .assign(is_min_y=1),
                    df.assign(is_min_y=0),
                ],
                ignore_index=True,
            ).drop_duplicates(["x", "y", "z"])
        )
        .pipe(
            # yが大きいものを選ぶ
            lambda df: pd.concat(
                [
                    df[df.diff_max_y < max_y_th]
                    .sample(frac=frac_circum, random_state=random_state)
                    .assign(is_max_y=1),
                    df.assign(is_max_y=0),
                ],
                ignore_index=True,
            ).drop_duplicates(["x", "y", "z"])
        )
        .pipe(
            # zが小さいものを適当な離散幅の中から一つずつ選ぶ
            lambda df: pd.concat(
                [
                    df[df.diff_min_z < diff_z_th]
                    .groupby(["vox_x", "vox_y", "vox_z"])
                    .first()
                    .reset_index()
                    .sample(frac=frac_is_small_z, random_state=random_state)
                    .assign(is_small_z=1),
                    df.assign(is_small_z=0),
                ],
                ignore_index=True,
            ).drop_duplicates(["x", "y", "z"])
        )
        .assign(
            is_first=lambda df: (
                (df.is_small_z == 1)
                | (df.is_min_x == 1)
                | (df.is_max_x == 1)
                | (df.is_min_y == 1)
                | (df.is_max_y == 1)
                | (df.is_under == 1)
            )
        )
    )

    return df_points_info


def select_immobile_machine_points(
    machine_points: NDArray[np.float64],
    cell_size: tuple[float, float, float] = (0.12, 0.12, 0.4),
    x_th: float = 0.1,
    y_th: float = 0.1,
    z_th: float = 0.1,
    frac: float | list[float] = 1.0,
    key_to_agg_x: list[str] = ["vox_y"],
    key_to_agg_y: list[str] = ["vox_x"],
    key_to_agg_z: list[str] = ["vox_x", "vox_y"],
    random_state: int = 20251021,
) -> pd.DataFrame:
    """
    要確認

    :param machine_points: 説明
    :type machine_points: np.ndarray
    :param cell_size: 説明
    :type cell_size: tuple
    :param x_th: 説明
    :type x_th: float
    :param y_th: 説明
    :type y_th: float
    :param z_th: 説明
    :type z_th: float
    :param frac: 説明
    :type frac: float | list[float]
    :param key_to_agg_x: 説明
    :type key_to_agg_x: list[str]
    :param key_to_agg_y: 説明
    :type key_to_agg_y: list[str]
    :param key_to_agg_z: 説明
    :type key_to_agg_z: list[str]
    :param random_state: 説明
    :type random_state: int
    :return: 説明
    :rtype: DataFrame
    """

    _fracs = frac if isinstance(frac, list) else [frac] * 3
    df_machine_points = (
        pd.DataFrame(machine_points, columns=["x", "y", "z"])
        .assign(
            vox_x=lambda df: (df.x // cell_size[0]).astype(int),
            vox_y=lambda df: (df.y // cell_size[1]).astype(int),
            vox_z=lambda df: (df.z // cell_size[2]).astype(int),
        )
        .assign(
            diff_min_x=lambda df: df.x - df.x.min(),
            diff_max_x=lambda df: df.x.max() - df.x,
            diff_min_y=lambda df: df.y - df.y.min(),
            diff_max_y=lambda df: df.y.max() - df.y,
            diff_min_z=lambda df: df.z - df.z.min(),
        )
        .pipe(
            # xが小さい小さものを選ぶ
            # ボクセル範囲内でxの最小値を与える点を取得
            lambda df: pd.concat(
                [
                    groupby_argmin(df.query(f"diff_min_x < {x_th}"), key_to_agg_x, "x")
                    .sample(frac=_fracs[0], random_state=random_state)
                    .assign(is_min_x_in_vox=True),
                    df.assign(is_min_x_in_vox=False),
                ]
            ).drop_duplicates(["x", "y", "z"])
        )
        .pipe(
            # ボクセル範囲内でxの最大値を与える点を取得
            lambda df: pd.concat(
                [
                    groupby_argmax(df.query(f"diff_max_x < {x_th}"), key_to_agg_x, "x")
                    .sample(frac=_fracs[0], random_state=random_state)
                    .assign(is_max_x_in_vox=True),
                    df.assign(is_max_x_in_vox=False),
                ]
            ).drop_duplicates(["x", "y", "z"])
        )
        .pipe(
            # ボクセル範囲内でyの最小値を与える点を取得
            lambda df: pd.concat(
                [
                    groupby_argmin(df.query(f"diff_min_y < {y_th}"), key_to_agg_y, "y")
                    .sample(frac=_fracs[1], random_state=random_state)
                    .assign(is_max_y_in_vox=True),
                    df.assign(is_min_y_in_vox=False),
                ]
            ).drop_duplicates(["x", "y", "z"])
        )
        .pipe(
            # ボクセル範囲内でyの最大値を与える点を取得
            lambda df: pd.concat(
                [
                    groupby_argmax(df.query(f"diff_max_y < {y_th}"), key_to_agg_y, "y")
                    .sample(frac=_fracs[1], random_state=random_state)
                    .assign(is_max_y_in_vox=True),
                    df.assign(is_max_y_in_vox=False),
                ]
            ).drop_duplicates(["x", "y", "z"])
        )
        .pipe(
            # ボクセル範囲内でzの最小値を与える点を取得
            lambda df: pd.concat(
                [
                    groupby_argmin(df.query(f"diff_min_z < {z_th}"), key_to_agg_z, "z")
                    .sample(frac=_fracs[2], random_state=random_state)
                    .assign(is_min_z_in_vox=True),
                    df.assign(is_min_z_in_vox=False),
                ]
            ).drop_duplicates(["x", "y", "z"])
        )
    )

    return df_machine_points


def get_perimeter(
    machine_points: NDArray[np.float64],
    z_val: float,
    center_xy: NDArray[np.float64],
    z_th: float = 0.1,
    r_th: float = 1.0,
    t_boundary_th: float = 0.1,
    r_boundary_th: float = 0.1,
    vox_size: tuple[float, float, float] = (0.06, 0.06, 0.02),
) -> pd.DataFrame:
    """
    machine_pointsの外周を取り出す
    machine_pointsを極座標で扱って、極座標の外周を取り出すことで、外周を取り出している

    :param machine_points: 説明
    :type machine_points: NDArray[np.float64]
    :param z_val: 説明
    :type z_val: float
    :param center_xy: 説明
    :type center_xy: NDArray[np.float64]
    :param z_th: 説明
    :type z_th: float
    :param r_th: 半径がr_thより大きいを取り出す
    :type r_th: float
    :param t_boundary_th: 説明
    :type t_boundary_th: float
    :param r_boundary_th: 説明
    :type r_boundary_th: float
    :param vox_size: 説明
    :type vox_size: tuple[float, float, float]
    :return: 説明
    :rtype: DataFrame
    """

    # zで範囲指定
    target_ind = is_in_interval(machine_points, z_range=(z_val - z_th, z_val + z_th))
    target_points = machine_points[target_ind]

    # 中心を決めて極座標計算
    centered_points = target_points[:, :2] - center_xy
    dist = np.sqrt((centered_points**2).sum(axis=1))
    angle = np.arctan2(centered_points[:, 1], centered_points[:, 0])
    cylindar_points = np.vstack(
        [dist[np.newaxis, :], angle[np.newaxis, :], target_points[:, 2]]
    ).T

    df_machine_points = (
        pd.DataFrame(cylindar_points, columns=["radius", "theta", "z"])
        .assign(x=target_points[:, 0], y=target_points[:, 1])
        .assign(
            diff_max_radius=lambda df: df.radius.max() - df.radius,
            diff_max_theta=lambda df: df.theta.max() - df.theta,
            diff_min_radius=lambda df: df.radius - df.radius.min(),
            diff_min_theta=lambda df: df.theta - df.theta.min(),
        )
        .assign(
            is_max_radius=lambda df: df.diff_max_radius < r_boundary_th,
            is_min_radius=lambda df: df.diff_min_radius < r_boundary_th,
            is_max_theta=lambda df: df.diff_max_theta < t_boundary_th,
            is_min_theta=lambda df: df.diff_min_theta < t_boundary_th,
        )
        .assign(
            vox_r=lambda df: (df.radius // vox_size[0]).astype(int),
            vox_t=lambda df: (df.theta // vox_size[1]).astype(int),
            vox_z=lambda df: (df.z // vox_size[2]).astype(int),
        )
        .pipe(
            # 半径がr_thより大きい中で、同じくらいのthetaを持つもの中で、半径が最も大きいものを取り出す
            lambda df: pd.concat(
                [
                    groupby_argmax(
                        df.query(f"radius > {r_th}"), ["vox_t"], "vox_r"
                    ).assign(is_max_r_in_theta=True),
                    df.assign(is_max_r_in_theta=False),
                ]
            ).drop_duplicates(["radius", "theta", "z"])
        )
        .assign(
            is_selected=lambda df: (
                df.is_max_r_in_theta | df.is_max_theta | df.is_min_theta
            )
        )
    )

    return df_machine_points


def select_cw(
    machine_points_cw: NDArray[np.float64],
    quantiles: NDArray[np.float64],
    center_xy: NDArray[np.float64],
    min_z_th: float = 0.1,
    min_x_th: float = 0.1,
    max_x_th: float = 0.1,
    cell_size: tuple[float, float, float] = (0.24, 0.24, 0.08),
    frac: float = 0.25,
    random_state: int = 20251020,
) -> pd.DataFrame:
    """
    カウンタウェイトから点群を取り出す関数
    下面, xが大きい/小さい部分, カウンタウェイトの外側外周部分から取り出す

    :param machine_points_cw: 説明
    :type machine_points_cw: NDArray[np.float64]
    :param quantiles: 説明
    :type quantiles: NDArray[np.float64]
    :param center_xy: 説明
    :type center_xy: NDArray[np.float64]
    :param min_z_th: 説明
    :type min_z_th: float
    :param min_x_th: 説明
    :type min_x_th: float
    :param max_x_th: 説明
    :type max_x_th: float
    :param cell_size: 説明
    :type cell_size: tuple[float, float, float]
    :param frac: 説明
    :type frac: float
    :param random_state: 説明
    :type random_state: int
    :return: 説明
    :rtype: DataFrame
    """
    # カウンタウェイトの外周部分を取り出す
    df_perimeter = pd.concat(
        get_perimeter(
            machine_points_cw,
            quantile,
            center_xy,
        )
        for quantile in quantiles
    ).reindex(
        columns=["x", "y", "z", "is_max_r_in_theta", "is_max_theta", "is_min_theta"]
    )

    # カウンタウェイトの下面部分を取り出す
    df_under = (
        pd.DataFrame(machine_points_cw, columns=["x", "y", "z"])
        .assign(
            vox_x=lambda df: (df.x // cell_size[0]).astype(int),
            vox_y=lambda df: (df.y // cell_size[1]).astype(int),
            vox_z=lambda df: (df.z // cell_size[2]).astype(int),
            diff_min_z=lambda df: df.z - df.z.min(),
            diff_max_x=lambda df: df.x.max() - df.x,
            diff_min_x=lambda df: df.x - df.x.min(),
            diff_max_y=lambda df: df.y.max() - df.y,
            diff_min_y=lambda df: df.y - df.y.min(),
        )
        .pipe(
            # 下面に当たる部分からランダムサンプリング
            lambda df: pd.concat(
                [
                    df[df.diff_min_z < min_z_th]
                    .sample(frac=frac, random_state=random_state)
                    .assign(is_min_z=True),
                    df.assign(is_min_z=False),
                ],
                ignore_index=True,
            ).drop_duplicates(["x", "y", "z"])
        )
        .pipe(
            # 下面でない部分で、xが小さい部分から取り出す
            lambda df: pd.concat(
                [
                    df[(df.diff_min_z >= min_z_th) & (df.diff_min_x < min_x_th)]
                    .sample(frac=frac, random_state=random_state)
                    .assign(is_min_x=True),
                    df.assign(is_min_x=False),
                ],
                ignore_index=True,
            ).drop_duplicates(["x", "y", "z"])
        )
        .pipe(
            # 下面でない部分で、xが大きい部分から取り出す
            lambda df: pd.concat(
                [
                    df[(df.diff_min_z >= min_z_th) & (df.diff_max_x < max_x_th)]
                    .sample(frac=frac, random_state=random_state)
                    .assign(is_max_x=True),
                    df.assign(is_max_x=False),
                ],
                ignore_index=True,
            ).drop_duplicates(["x", "y", "z"])
        )
        .reindex(columns=["x", "y", "z", "is_min_z", "is_min_x", "is_max_x"])
    )

    return pd.merge(df_under, df_perimeter, how="left", on=["x", "y", "z"])


def get_perimeter_v2(
    machine_points: NDArray[np.float64],
    z_val: float,
    center_xy: NDArray[np.float64],
    z_th: float = 0.05,
    max_r_th: float = 0.05,
    t_boundary_th: float = 0.1,
    r_boundary_th: float = 0.1,
    vox_size: tuple[float, float, float] = (0.09, 0.09, 0.03),
) -> pd.DataFrame:
    """
    machine_pointsの外周を取り出す
    machine_pointsを極座標で扱って、極座標の外周を取り出すことで、外周を取り出している

    同じ離散角で最大の半径を持つものに近くて、diff_bandがなるべく小さいものを取ってくるような事を新しくしている
    :param machine_points: 説明
    :type machine_points: NDArray[np.float64]
    :param z_val: 説明
    :type z_val: float
    :param center_xy: 説明
    :type center_xy: NDArray[np.float64]
    :param z_th: 説明
    :type z_th: float
    :param max_r_th: 説明
    :type max_r_th: float
    :param t_boundary_th: 説明
    :type t_boundary_th: float
    :param r_boundary_th: 説明
    :type r_boundary_th: float
    :param vox_size: 説明
    :type vox_size: tuple[float, float, float]
    :return: 説明
    :rtype: DataFrame
    """

    target_ind = is_in_interval(machine_points, z_range=(z_val - z_th, z_val + z_th))
    target_points = machine_points[target_ind]
    centered_points = target_points[:, :2] - center_xy

    dist = np.sqrt((centered_points**2).sum(axis=1))
    angle = np.arctan2(centered_points[:, 1], centered_points[:, 0])
    cylindar_points = np.vstack(
        [dist[np.newaxis, :], angle[np.newaxis, :], target_points[:, 2]]
    ).T

    df_machine_points = (
        pd.DataFrame(cylindar_points, columns=["radius", "theta", "z"])
        .assign(x=target_points[:, 0], y=target_points[:, 1])
        .assign(
            diff_max_radius=lambda df: df.radius.max() - df.radius,
            diff_max_theta=lambda df: df.theta.max() - df.theta,
            diff_min_radius=lambda df: df.radius - df.radius.min(),
            diff_min_theta=lambda df: df.theta - df.theta.min(),
            diff_band=lambda df: np.abs(df.z - z_val),
        )
        .assign(
            is_max_radius=lambda df: df.diff_max_radius < r_boundary_th,
            is_min_radius=lambda df: df.diff_min_radius < r_boundary_th,
            is_max_theta=lambda df: df.diff_max_theta < t_boundary_th,
            is_min_theta=lambda df: df.diff_min_theta < t_boundary_th,
        )
        .assign(
            vox_r=lambda df: (df.radius // vox_size[0]).astype(int),
            vox_t=lambda df: (df.theta // vox_size[1]).astype(int),
            vox_z=lambda df: (df.z // vox_size[2]).astype(int),
        )
        .assign(max_r_in_theta=lambda df: df.groupby(["vox_t"]).radius.transform("max"))
        .assign(diff_max_r_in_theta=lambda df: df.max_r_in_theta - df.radius)
        .pipe(
            lambda df: pd.concat(
                [
                    # 同じ離散角で最大の半径を持つものに近くて、diff_bandがなるべく小さいものを取ってくる
                    groupby_argmin(
                        df.query(f"diff_max_r_in_theta < {max_r_th}"),
                        ["vox_t"],
                        "diff_band",
                    ).assign(on_band_near_max_r=True),
                    df.assign(on_band_near_max_r=False),
                ]
            ).drop_duplicates(["radius", "theta", "z"])
        )
        .assign(
            is_selected=lambda df: (
                df.on_band_near_max_r | df.is_max_theta | df.is_min_theta
            )
        )
    )

    return df_machine_points


def select_cw_v2(
    machine_points_cw: np.ndarray,
    quantiles: np.ndarray,
    center_xy: np.ndarray,
    min_z_th: float = 0.1,
    min_x_th: float = 0.1,
    max_x_th: float = 0.1,
    cell_size: tuple = (0.24, 0.24, 0.08),
    frac: float | list[float] = 0.25,
    random_state: int = 20251020,
) -> pd.DataFrame:
    """
    カウンタウェイト部分から点を取り出す関数 v2
    外周を取り出す部分以外もfracをリストする観点が異なる

    :param machine_points_cw: 説明
    :type machine_points_cw: np.ndarray
    :param quantiles: 説明
    :type quantiles: np.ndarray
    :param center_xy: 説明
    :type center_xy: np.ndarray
    :param min_z_th: 説明
    :type min_z_th: float
    :param min_x_th: 説明
    :type min_x_th: float
    :param max_x_th: 説明
    :type max_x_th: float
    :param cell_size: 説明
    :type cell_size: tuple
    :param frac: 説明
    :type frac: float | list[float]
    :param random_state: 説明
    :type random_state: int
    :return: 説明
    :rtype: DataFrame

    """
    # カウンタウェイトの外周部分を取り出す
    _fracs = frac if isinstance(frac, list) else [frac] * 3
    df_perimeter = pd.concat(
        get_perimeter_v2(
            machine_points_cw,
            quantile,
            center_xy,
        )
        for quantile in quantiles
    ).reindex(
        columns=["x", "y", "z", "on_band_near_max_r", "is_max_theta", "is_min_theta"]
    )

    # カウンタウェイトの下面部分を取り出す
    df_under = (
        pd.DataFrame(machine_points_cw, columns=["x", "y", "z"])
        .assign(
            vox_x=lambda df: (df.x // cell_size[0]).astype(int),
            vox_y=lambda df: (df.y // cell_size[1]).astype(int),
            vox_z=lambda df: (df.z // cell_size[2]).astype(int),
            diff_min_z=lambda df: df.z - df.z.min(),
            diff_max_x=lambda df: df.x.max() - df.x,
            diff_min_x=lambda df: df.x - df.x.min(),
            diff_max_y=lambda df: df.y.max() - df.y,
            diff_min_y=lambda df: df.y - df.y.min(),
        )
        .pipe(
            # 下面の点を取り出す
            lambda df: pd.concat(
                [
                    df[df.diff_min_z < min_z_th]
                    .sample(frac=_fracs[2], random_state=random_state)
                    .assign(is_min_z=True),
                    df.assign(is_min_z=False),
                ],
                ignore_index=True,
            ).drop_duplicates(["x", "y", "z"])
        )
        .pipe(
            # 下面以外でxが小さい部分を取り出す
            lambda df: pd.concat(
                [
                    df[(df.diff_min_z >= min_z_th) & (df.diff_min_x < min_x_th)]
                    .sample(frac=_fracs[0], random_state=random_state)
                    .assign(is_min_x=True),
                    df.assign(is_min_x=False),
                ],
                ignore_index=True,
            ).drop_duplicates(["x", "y", "z"])
        )
        .pipe(
            # 下面以外でxが大きい部分を取り出す
            lambda df: pd.concat(
                [
                    df[(df.diff_min_z >= min_z_th) & (df.diff_max_x < max_x_th)]
                    .sample(frac=_fracs[0], random_state=random_state)
                    .assign(is_max_x=True),
                    df.assign(is_max_x=False),
                ],
                ignore_index=True,
            ).drop_duplicates(["x", "y", "z"])
        )
        .reindex(columns=["x", "y", "z", "is_min_z", "is_min_x", "is_max_x"])
    )

    # 外周のデータとそれ以外のデータをjoin
    return pd.merge(df_under, df_perimeter, how="left", on=["x", "y", "z"])


def select_mobile_machine_points_in_boundary(
    df_xyz: pd.DataFrame,
    cell_size: tuple[float, float, float] = (0.12, 0.12, 0.4),
    x_th: float = 0.1,
    y_th: float = 0.1,
    z_th: float = 0.1,
    frac: float | list[float] = 1.0,
    key_to_agg_x: list[str] = ["vox_y"],
    key_to_agg_y: list[str] = ["vox_x"],
    key_to_agg_z: list[str] = ["vox_x", "vox_y"],
    random_state: int = 20251021,
) -> pd.DataFrame:
    """
    下部走行体の下面、上面、側面から点群を間引いて取得する関数
    対応する入力が複数機体を合わせた点群なので、どの部位がどの部位か管理しやくするため、np.ndarrayを渡すよりも
    DataFrameを渡す方が扱いやすいので、入力はDataFrameにしている

    x,y,zの最大/最小に近い部分をx_th, y_th, z_thの閾値で取り出して、
    取り出した中で、key_agg_to_[xyz]をkeyとして、同じkeyの中の最大/最小を取り出すようなことをしている

    処理の例: pipeやassignは読んだDataFrameを返すような関数であることに注意して
    以下の処理は、自分自身dfという名前にして、
    1. diff_min_xが閾値x_thより小さいものを選んできて(query部分)、
    2. key_to_agg_xに該当する列の同じ値になっているもののそれぞれでxという列名が最小のものを取ってきて(groupy_argmin部分)
    3. _fracs[0]の割合でランダムサンプリングして(sample部分)
    4. 残ったDataFrameにis_min_x_in_voxという列を作りつつ、Trueにして(assign部分)
    5. 全てのDataFrameにもis_min_x_in_voxという列を作って、Falseにして(assign部分)
    6. 4と5のDataFrameを合わせつつ、(x,y,z)が同じものは最初のもの(=is_min_x_in_voxがTrueのもの)を優先して(drop_duplicates部分)、
    その結果を返す関数(このpipe関数の戻り値はDataFrame)

        .pipe(
            lambda df: pd.concat(
                [
                    groupby_argmin(df.query(f"diff_min_x < {x_th}"), key_to_agg_x, "x")
                    .sample(frac=_fracs[0], random_state=random_state)
                    .assign(is_min_x_in_vox=True),
                    df.assign(is_min_x_in_vox=False),
                ]
            ).drop_duplicates(["x", "y", "z"])
        )

    :param df_xyz: [x, y, z]を列に持つDataFrame
    :type df_xyz: pd.DataFrame
    :param cell_size: 離散幅
    :type cell_size: tuple[float, float, float]
    :param x_th: xに対する最小/最大の閾値
    :type x_th: float
    :param y_th: yに対する最小/最大の閾値
    :type y_th: float
    :param z_th: zに対する最小/最大の閾値
    :type z_th: float
    :param frac: 説明
    :type frac: float | list[float]
    :param key_to_agg_x: xの値の最大や最小を見るkey, 同じkeyの中で最大/最小のxの値を見つける
    :type key_to_agg_x: list[str]
    :param key_to_agg_y: yの値の最大や最小を見るkey, 同じkeyの中で最大/最小のyの値を見つける
    :type key_to_agg_y: list[str]
    :param key_to_agg_z: zの値の最大や最小を見るkey, 同じkeyの中で最大/最小のzの値を見つける
    :type key_to_agg_z: list[str]
    :param random_state: 説明
    :type random_state: int
    :return: 説明
    :rtype: DataFrame
    """
    _fracs = frac if isinstance(frac, list) else [frac] * 3
    df_machine_points = (
        df_xyz.assign(
            vox_x=lambda df: (df.x // cell_size[0]).astype(int),
            vox_y=lambda df: (df.y // cell_size[1]).astype(int),
            vox_z=lambda df: (df.z // cell_size[2]).astype(int),
        )
        .assign(
            diff_min_x=lambda df: df.x - df.x.min(),
            diff_max_x=lambda df: df.x.max() - df.x,
            diff_min_y=lambda df: df.y - df.y.min(),
            diff_max_y=lambda df: df.y.max() - df.y,
            diff_max_z=lambda df: df.z.max() - df.z,
            diff_min_z=lambda df: df.z - df.z.min(),
        )
        .pipe(
            # ボクセル範囲内でxの最小値を与える点を取得
            lambda df: pd.concat(
                [
                    groupby_argmin(df.query(f"diff_min_x < {x_th}"), key_to_agg_x, "x")
                    .sample(frac=_fracs[0], random_state=random_state)
                    .assign(is_min_x_in_vox=True),
                    df.assign(is_min_x_in_vox=False),
                ]
            ).drop_duplicates(["x", "y", "z"])
        )
        .pipe(
            # ボクセル範囲内でxの最大値を与える点を取得
            lambda df: pd.concat(
                [
                    groupby_argmax(df.query(f"diff_max_x < {x_th}"), key_to_agg_x, "x")
                    .sample(frac=_fracs[0], random_state=random_state)
                    .assign(is_max_x_in_vox=True),
                    df.assign(is_max_x_in_vox=False),
                ]
            ).drop_duplicates(["x", "y", "z"])
        )
        .pipe(
            # ボクセル範囲内でyの最小値を与える点を取得
            lambda df: pd.concat(
                [
                    groupby_argmin(df.query(f"diff_min_y < {y_th}"), key_to_agg_y, "y")
                    .sample(frac=_fracs[1], random_state=random_state)
                    .assign(is_max_y_in_vox=True),
                    df.assign(is_min_y_in_vox=False),
                ]
            ).drop_duplicates(["x", "y", "z"])
        )
        .pipe(
            # ボクセル範囲内でyの最大値を与える点を取得
            lambda df: pd.concat(
                [
                    groupby_argmax(df.query(f"diff_max_y < {y_th}"), key_to_agg_y, "y")
                    .sample(frac=_fracs[1], random_state=random_state)
                    .assign(is_max_y_in_vox=True),
                    df.assign(is_max_y_in_vox=False),
                ]
            ).drop_duplicates(["x", "y", "z"])
        )
        .pipe(
            # ボクセル範囲内でzの最小値を与える点を取得
            lambda df: pd.concat(
                [
                    groupby_argmin(df.query(f"diff_min_z < {z_th}"), key_to_agg_z, "z")
                    .sample(frac=_fracs[2], random_state=random_state)
                    .assign(is_min_z_in_vox=True),
                    df.assign(is_min_z_in_vox=False),
                ]
            ).drop_duplicates(["x", "y", "z"])
        )
        .pipe(
            # ボクセル範囲内でzの最大値を与える点を取得
            lambda df: pd.concat(
                [
                    groupby_argmax(df.query(f"diff_max_z < {z_th}"), key_to_agg_z, "z")
                    .sample(frac=_fracs[2], random_state=random_state)
                    .assign(is_max_z_in_vox=True),
                    df.assign(is_max_z_in_vox=False),
                ]
            ).drop_duplicates(["x", "y", "z"])
        )
    )

    return df_machine_points
