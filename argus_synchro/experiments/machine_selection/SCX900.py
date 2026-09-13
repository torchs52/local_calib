"""90t機の点群を選択する際に用いた関数を入れているモジュール
点群の選択は基本的に、(x,y,z)がある条件を満たす場合、is_firstというフラグを付けて、is_firstというフラグが付いている点群を選びつつ、
目標の点数(n_target_pointsという引数が主に対応しているはず)に達していなければ、他の部分から満遍なく点を取っている

"""

from glob import glob

import numpy as np
import pandas as pd
from numpy.typing import NDArray
from sklearn.cluster import AgglomerativeClustering


def pd2np(df_xyz: pd.DataFrame) -> NDArray[np.float64]:
    """
    df_xyzのx,y,zカラムを取り出して、3次元のnp.ndarrayに変換する関数
    何度も呼んでいるのでヘルパー関数にした
    x, y, zという名前の列がない場合は、ない列がnp.nanになると思われるが、今のところx,y,zの列がない状況で呼ばないので確認せずに用いる

    :param df_xyz: 説明
    :type df_xyz: pd.DataFrame
    :return: 説明
    :rtype: NDArray[float64]

    """
    return df_xyz.reindex(columns=["x", "y", "z"]).values


def select_points_first_second(
    df_xyz: pd.DataFrame,
    n_target_points: int,
    is_first_column: str = "is_first",
) -> tuple[NDArray[np.float64], NDArray[np.float64]]:
    """
    is_firstに該当する点を取り出して、n_target_pointsに達するだけ点がなければ、
    それ以外の点群の中でクラスタリングを行い、各クラスタから一つを選ぶ

    :param df_xyz: 対象となる点群が入ったDataFrame, (x,y,z, @is_first_column)という列を持っている
    :type df_xyz: pd.DataFrame
    :param n_target_points: 取り出したい点の数
    :type n_target_points: int
    :param is_first_column: is_first_columnであることを表す列
    :type is_first_column: str
    :return: 説明
    :rtype: tuple[NDArray[float64], NDArray[float64]]
    """
    # 一番重要な点を抜き出す
    first_points = (
        df_xyz[df_xyz[is_first_column]].reindex(columns=["x", "y", "z"]).values
    )

    # 目標の点数に足りているかチェック
    diff_n = n_target_points - len(first_points)
    if diff_n <= 0:
        return (first_points, np.empty((0, 3)))

    # それ以外の点を抜き出す
    df_other = df_xyz[~df_xyz[is_first_column]]
    clf = AgglomerativeClustering(n_clusters=diff_n, linkage="ward")
    labels = clf.fit_predict(df_other.reindex(columns=["x", "y", "z"]).values)

    second_points = (
        df_other.assign(label=labels).groupby("label")[["x", "y", "z"]].first().values
    )

    return first_points, second_points


def np_downsample(
    points: NDArray[np.float64],
    frac: float,
    random_state: int = -1,
) -> NDArray[np.float64]:
    """
    ランダムサンプリングを行う

    :param points: 対象対象と点群
    :type points: NDArray[np.float64]
    :param frac: 何割削減するか
    :type frac: float
    :param random_state: 乱種の種, 0以下の場合乱数初期化しない
    :type random_state: int
    :return: 説明
    :rtype: NDArray[float64]

    """
    if random_state > 0:
        np.random.seed(random_state)

    n_points = len(points)
    choice_ind = np.random.choice(
        np.arange(n_points), int(n_points * frac), replace=False
    )

    return points[choice_ind]


def select_upper_points_v4(
    upper_points: NDArray[np.float64],
    n_target_points: int,
    cell_size: tuple[float, float, float] = (0.24, 0.24, 0.08),
    diff_z_th: float = 0.7,
    min_z_th: float = 0.05,
    frac: float = 0.25,
    random_state: int = 20250710,
) -> tuple[NDArray[np.float64], NDArray[np.float64]]:
    """
    カウンタウェイトを除く上部旋回体の点群選択, 上部旋回体の下面付近の点を選びに行っている

    :param upper_points: 説明
    :type upper_points: NDArray[np.float64]
    :param n_target_points: 説明
    :type n_target_points: int
    :param cell_size: 格子サイズ
    :type cell_size: tuple[float, float, float]
    :param diff_z_th: 下面の点の閾値
    :type diff_z_th: float
    :param min_z_th: voxelのzの閾値
    :type min_z_th: float
    :param frac: 下面の間引きの割合
    :type frac: float
    :param random_state: 説明
    :type random_state: int
    :return: 説明
    :rtype: tuple[NDArray[float64], NDArray[float64]]
    """

    df_points_info = (
        pd.DataFrame(upper_points, columns=["x", "y", "z"])
        .assign(
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
            # 下面側の点を選択しつつ、ランダムサンプリング
            lambda df: pd.concat(
                [
                    df[df.diff_min_z < min_z_th]
                    .sample(frac=frac, random_state=random_state)
                    .assign(is_under=1),
                    df.assign(is_under=0),
                ],
                ignore_index=True,
            ).drop_duplicates(["x", "y", "z"])
        )
        .pipe(
            # 下面で適当な格子の中の点が1つは選択する
            lambda df: pd.concat(
                [
                    df[df.diff_min_z < diff_z_th]
                    .groupby(["vox_x", "vox_y", "vox_z"])
                    .first()
                    .reset_index()
                    .assign(is_small_z=1),
                    df.assign(is_small_z=0),
                ],
                ignore_index=True,
            ).drop_duplicates(["x", "y", "z"])
        )
        .assign(
            # 下面側の点を選択
            is_first=lambda df: (df.is_small_z == 1) | (df.is_under == 1)
        )
    )

    return select_points_first_second(df_points_info, n_target_points, "is_first")


def select_cw_points_v2(
    cw_points: NDArray[np.float64],
    n_target_points: int,
    cell_size_under: tuple[float, float, float] = (0.36, 0.36, 0.12),
    cell_size_surface: tuple[float, float, float] = (0.12, 0.12, 0.24),
    cell_size_side: tuple[float, float, float] = (0.12, 0.12, 0.24),
    diff_y_th: float = 0.1,
) -> tuple[NDArray[np.float64], NDArray[np.float64]]:
    """
    カウンタウェイトの点群cw_pointsから基準に合わせて点群を取り出す

    :param cw_points: カウンタウェイトの機体点群
    :type cw_points: _NDFloat
    :param n_target_points: 説明
    :type n_target_points: int
    :param cell_size_under: 下面の離散幅
    :type cell_size_under: _FloatTup
    :param cell_size_surface: カウンタウェイトの丸みがある部分の離散幅
    :type cell_size_surface: _FloatTup
    :param cell_size_side: 側面の離散幅
    :type cell_size_side: _FloatTup
    :param diff_y_th: 側面の閾値
    :type diff_y_th: float
    :return: 説明
    :rtype: tuple[NDArray[float64], NDArray[float64]]
    """
    df_points_info = (
        pd.DataFrame(cw_points, columns=["x", "y", "z"])
        .assign(
            vox_x_under=lambda df: (df.x // cell_size_under[0]).astype(int),
            vox_y_under=lambda df: (df.y // cell_size_under[1]).astype(int),
            vox_z_under=lambda df: (df.z // cell_size_under[2]).astype(int),
            vox_x_surface=lambda df: (df.x // cell_size_surface[0]).astype(int),
            vox_y_surface=lambda df: (df.y // cell_size_surface[1]).astype(int),
            vox_z_surface=lambda df: (df.z // cell_size_surface[2]).astype(int),
            vox_x_side=lambda df: (df.x // cell_size_side[0]).astype(int),
            vox_y_side=lambda df: (df.y // cell_size_side[1]).astype(int),
            vox_z_side=lambda df: (df.z // cell_size_side[2]).astype(int),
            min_z=lambda df: df.z.min(),
            med_z=lambda df: df.z.median(),
            diff_max_y=lambda df: df.y.max() - df.y,
            diff_min_y=lambda df: df.y - df.y.min(),
            diff_min_z=lambda df: df.z - df.z.min(),
        )
        .pipe(
            # カウンタウェイトの丸みがある部分を取り出す
            lambda df: pd.concat(
                [
                    df.loc[
                        df.groupby(["vox_y_surface", "vox_z_surface"]).x.idxmax()
                    ].assign(is_highest_x=1),
                    df.assign(is_highest_x=0),
                ]
            )
        )
        .drop_duplicates(["x", "y", "z"])
        .pipe(
            # カウンタウェイトの下面を選ぶ
            lambda df: pd.concat(
                [
                    df.loc[
                        df.groupby(["vox_x_under", "vox_y_under"]).z.idxmin()
                    ].assign(is_lowest_z=1),
                    df.assign(is_lowest_z=0),
                ]
            )
        )
        .drop_duplicates(["x", "y", "z"])
        .pipe(
            # カウンタウェイトの側面を選ぶ
            lambda df: pd.concat(
                [
                    df.loc[
                        df[df.diff_max_y < diff_y_th]
                        .groupby(["vox_x_side", "vox_z_side"])
                        .y.idxmax()
                    ].assign(is_max_y=1),
                    df.assign(is_max_y=0),
                ]
            )
        )
        .drop_duplicates(["x", "y", "z"])
        .pipe(
            # カウンタウェイトの側面を選ぶ
            lambda df: pd.concat(
                [
                    df.loc[
                        df[df.diff_min_y < diff_y_th]
                        .groupby(["vox_x_side", "vox_z_side"])
                        .y.idxmin()
                    ].assign(is_min_y=1),
                    df.assign(is_min_y=0),
                ]
            )
        )
        .drop_duplicates(["x", "y", "z"])
        .assign(
            # 選んだ基準のどれかを満たすものをis_firstに採用
            is_first=lambda df: (
                (df.is_highest_x == 1)
                | (df.is_lowest_z == 1)
                | (df.is_max_y == 1)
                | (df.is_min_y == 1)
            )
        )
    )

    return select_points_first_second(df_points_info, n_target_points, "is_first")


def select_cw_points_on_lines(
    cw_points: NDArray[np.float64],
    line_cell_size: tuple[float, float, float] = (0.01, 0.01, 0.1),
    z_th: float = 0.1,
    exclusion_file_regex: str = "./data/argus_synchro_cw_group*_exclusion.npy",
) -> NDArray[np.float64]:
    """
    上部旋回体のカウンタウェイト部分を取り出す関数
    カウンタウェイトをある高さに合わせて切り出した点を作る

    :param cw_points: 説明
    :type cw_points: NDArray[np.float64]
    :param line_cell_size: 説明
    :type line_cell_size: tuple[float, float, float]
    :param z_th: 説明
    :type z_th: float
    :param exclusion_file_regex: 説明
    :type exclusion_file_regex: str
    :return: 説明
    :rtype: NDArray[float64]

    """

    used_quantiles = [0, 0.25, 0.5, 0.75]
    height_quantiles = np.quantile(cw_points[:, 2], used_quantiles)

    _third_points = cw_points[
        np.array(
            [
                ((cw_points[:, 2] - quantile) >= 0)
                & ((cw_points[:, 2] - quantile) < z_th)
                for quantile in height_quantiles
            ]
        ).any(axis=0)
    ]

    df_third_points = (
        pd.DataFrame(_third_points, columns=["x", "y", "z"])
        .assign(
            vox_x=lambda df: (df.x // line_cell_size[0]).astype(int),
            vox_y=lambda df: (df.y // line_cell_size[1]).astype(int),
            vox_z=lambda df: (df.z // line_cell_size[2]).astype(int),
        )
        .assign(
            # 取り出したい高さに該当する点かどうか
            height_group=lambda df: np.argmin(
                np.abs(df.z.values[:, np.newaxis] - height_quantiles), axis=1
            )
        )
        .pipe(
            # 取り出したい高さの中でzが最大のものを、同じ格子の中から一つ取り出す
            lambda df: pd.concat(
                [
                    df.loc[
                        df.groupby(["vox_x", "vox_y", "height_group"]).z.idxmax()
                    ].assign(is_chosen=1),
                    df.assign(is_chosen=0),
                ]
            )
        )
        .drop_duplicates(["x", "y", "z"])
    )

    # 上記の処理だと取り除けない点がある場合、exclusion_filesから点を取り出して、除外
    exclusion_points = [np.load(filename) for filename in glob(exclusion_file_regex)]
    third_points = pd2np(df_third_points.query("is_chosen == 1"))
    if len(exclusion_points) == 0:
        return third_points

    exclusion_points = np.vstack(exclusion_points)
    third_points = third_points[
        ~(
            np.sqrt(
                (
                    (
                        third_points[:, np.newaxis, :]
                        - exclusion_points[np.newaxis, :, :]
                    )
                    ** 2
                ).sum(axis=2)
            )
            < 0.01
        ).any(axis=1)
    ]

    return third_points


def select_senkai_chushin_points_v2(
    senkai_chushin_points: NDArray[np.float64],
    n_target_points: int,
    cell_size: tuple[float, float, float] = (0.1, 0.1, 0.035),
    diff_x_th: float = 0.1,
    frac: float = 0.5,
    random_state: int = 20250708,
) -> tuple[NDArray[np.float64], NDArray[np.float64]]:
    """
    下部走行体の旋回中心部分から選ぶ
    旋回中心はx方向の側面だけあれば良いので、それらを選ぶような処理

    :param senkai_chushin_points: 説明
    :type senkai_chushin_points: NDArray[np.float64]
    :param n_target_points: 説明
    :type n_target_points: int
    :param cell_size: 格子サイズ
    :type cell_size: tuple[float, float, float]
    :param diff_x_th: 最大, 最小のxにどれだけ近いと選ぶかの閾値
    :type diff_x_th: float
    :param frac: 説明
    :type frac: float
    :param random_state: 説明
    :type random_state: int
    :return: 説明
    :rtype: tuple[NDArray[float64], NDArray[float64]]
    """
    df_points_info = (
        pd.DataFrame(senkai_chushin_points, columns=["x", "y", "z"])
        .assign(
            vox_x=lambda df: (df.x // cell_size[0]).astype(int),
            vox_y=lambda df: (df.y // cell_size[1]).astype(int),
            vox_z=lambda df: (df.z // cell_size[2]).astype(int),
            diff_max_x=lambda df: df.x.max() - df.x,
            diff_min_x=lambda df: df.x - df.x.min(),
        )
        .pipe(
            # x最大に近い点を選びつつ、離散座標(y,z)が同じものをrandom samplingしつつ1つだけ選ぶ
            lambda df: pd.concat(
                [
                    df.loc[
                        df[df.diff_max_x < diff_x_th]
                        .groupby(["vox_y", "vox_z"])
                        .x.idxmax()
                    ]
                    .sample(frac=frac, random_state=random_state)
                    .assign(is_max_x=1),
                    df.assign(is_max_x=0),
                ]
            )
        )
        .drop_duplicates(["x", "y", "z"])
        .pipe(
            # x最小に近い点を選びつつ、離散座標(y,z)が同じものをrandom samplingしつつ1つだけ選ぶ
            lambda df: pd.concat(
                [
                    df.loc[
                        df[df.diff_min_x < diff_x_th]
                        .groupby(["vox_y", "vox_z"])
                        .x.idxmin()
                    ]
                    .sample(frac=frac, random_state=random_state)
                    .assign(is_min_x=1),
                    df.assign(is_min_x=0),
                ]
            )
        )
        .drop_duplicates(["x", "y", "z"])
        .assign(is_first=lambda df: (df.is_max_x == 1) | (df.is_min_x == 1))
    )

    return select_points_first_second(df_points_info, n_target_points, "is_first")


def select_front_right_L_ji_points_v2(
    front_right_L_ji_points: NDArray[np.float64],
    n_target_points: int,
    cell_size: tuple[float, float, float] = (0.1, 0.1, 0.035),
    diff_y_th: float = 0.1,
    diff_x_th: float = 0.1,
    frac: float = 0.5,
    random_state: int = 20250708,
) -> tuple[NDArray[np.float64], NDArray[np.float64]]:
    """
    下部走行体のキャブ側のL字の右側の点を選ぶ
    x最小, y最小/最大らへんの点を選ぶ

    :param front_right_L_ji_points: 説明
    :type front_right_L_ji_points: NDArray[np.float64]
    :param n_target_points: 説明
    :type n_target_points: int
    :param cell_size: 説明
    :type cell_size: tuple[float, float, float]
    :param diff_y_th: yの閾値
    :type diff_y_th: float
    :param diff_x_th: 説明
    :type diff_x_th: xの閾値
    :param frac: 説明
    :type frac: float
    :param random_state: 説明
    :type random_state: int
    :return: 説明
    :rtype: tuple[NDArray[float64], NDArray[float64]]

    """
    df_points_info = (
        pd.DataFrame(front_right_L_ji_points, columns=["x", "y", "z"])
        .assign(
            vox_x=lambda df: (df.x // cell_size[0]).astype(int),
            vox_y=lambda df: (df.y // cell_size[1]).astype(int),
            vox_z=lambda df: (df.z // cell_size[2]).astype(int),
            diff_max_x=lambda df: df.x.max() - df.x,
            diff_min_x=lambda df: df.x - df.x.min(),
            diff_max_y=lambda df: df.y.max() - df.y,
            diff_min_y=lambda df: df.y - df.y.min(),
        )
        .pipe(
            lambda df: pd.concat(
                [
                    df.loc[
                        df[df.diff_min_y < diff_y_th]
                        .groupby(["vox_x", "vox_z"])
                        .y.idxmin()
                    ]
                    .sample(frac=frac, random_state=random_state)
                    .assign(is_min_y=1),
                    df.assign(is_min_y=0),
                ]
            )
        )
        .drop_duplicates(["x", "y", "z"])
        .pipe(
            lambda df: pd.concat(
                [
                    df.loc[
                        df[df.diff_max_y < diff_y_th]
                        .groupby(["vox_x", "vox_z"])
                        .y.idxmax()
                    ]
                    .sample(frac=frac, random_state=random_state)
                    .assign(is_max_y=1),
                    df.assign(is_max_y=0),
                ]
            )
        )
        .drop_duplicates(["x", "y", "z"])
        .pipe(
            lambda df: pd.concat(
                [
                    df.loc[
                        df[df.diff_min_x < diff_x_th]
                        .groupby(["vox_y", "vox_z"])
                        .x.idxmin()
                    ]
                    .sample(frac=frac, random_state=random_state)
                    .assign(is_min_x=1),
                    df.assign(is_min_x=0),
                ]
            )
        )
        .drop_duplicates(["x", "y", "z"])
        .assign(
            is_first=lambda df: (
                (df.is_min_x == 1) | (df.is_max_y == 1) | (df.is_min_y == 1)
            )
        )
    )

    return select_points_first_second(df_points_info, n_target_points, "is_first")


def select_front_left_L_ji_points_v2(
    front_left_L_ji_points: NDArray[np.float64],
    n_target_points: int,
    cell_size: tuple[float, float, float] = (0.1, 0.1, 0.035),
    diff_y_th: float = 0.1,
    diff_x_th: float = 0.1,
    frac: float = 0.5,
    random_state: int = 20250708,
) -> tuple[NDArray[np.float64], NDArray[np.float64]]:
    """
    下部走行体のキャブ側のL字の左側の点を選ぶ
    x最小, y最小/最大らへんの点を選ぶ

    :param front_left_L_ji_points: 説明
    :type front_left_L_ji_points: NDArray[np.float64]
    :param n_target_points: 説明
    :type n_target_points: int
    :param cell_size: 説明
    :type cell_size: tuple[float, float, float]
    :param diff_y_th: 説明
    :type diff_y_th: float
    :param diff_x_th: 説明
    :type diff_x_th: float
    :param frac: 説明
    :type frac: float
    :param random_state: 説明
    :type random_state: int
    :return: 説明
    :rtype: tuple[NDArray[float64], NDArray[float64]]


    """
    df_points_info = (
        pd.DataFrame(front_left_L_ji_points, columns=["x", "y", "z"])
        .assign(
            vox_x=lambda df: (df.x // cell_size[0]).astype(int),
            vox_y=lambda df: (df.y // cell_size[1]).astype(int),
            vox_z=lambda df: (df.z // cell_size[2]).astype(int),
            diff_max_x=lambda df: df.x.max() - df.x,
            diff_min_x=lambda df: df.x - df.x.min(),
            diff_max_y=lambda df: df.y.max() - df.y,
            diff_min_y=lambda df: df.y - df.y.min(),
        )
        .pipe(
            lambda df: pd.concat(
                [
                    df.loc[
                        df[df.diff_min_y < diff_y_th]
                        .groupby(["vox_x", "vox_z"])
                        .y.idxmin()
                    ]
                    .sample(frac=frac, random_state=random_state)
                    .assign(is_min_y=1),
                    df.assign(is_min_y=0),
                ]
            )
        )
        .drop_duplicates(["x", "y", "z"])
        .pipe(
            lambda df: pd.concat(
                [
                    df.loc[
                        df[df.diff_max_y < diff_y_th]
                        .groupby(["vox_x", "vox_z"])
                        .y.idxmax()
                    ]
                    .sample(frac=frac, random_state=random_state)
                    .assign(is_max_y=1),
                    df.assign(is_max_y=0),
                ]
            )
        )
        .drop_duplicates(["x", "y", "z"])
        .pipe(
            lambda df: pd.concat(
                [
                    df.loc[
                        df[df.diff_min_x < diff_x_th]
                        .groupby(["vox_y", "vox_z"])
                        .x.idxmin()
                    ]
                    .sample(frac=frac, random_state=random_state)
                    .assign(is_min_x=1),
                    df.assign(is_min_x=0),
                ]
            )
        )
        .drop_duplicates(["x", "y", "z"])
        .assign(
            is_first=lambda df: (
                (df.is_min_x == 1) | (df.is_max_y == 1) | (df.is_min_y == 1)
            )
        )
    )

    return select_points_first_second(df_points_info, n_target_points, "is_first")


def select_back_right_L_ji_points_v2(
    back_right_L_ji_points: NDArray[np.float64],
    n_target_points: int,
    cell_size: tuple[float, float, float] = (0.1, 0.1, 0.035),
    diff_y_th: float = 0.1,
    diff_x_th: float = 0.1,
    frac: float = 0.5,
    random_state: int = 20250708,
) -> tuple[NDArray[np.float64], NDArray[np.float64]]:
    """
    下部走行体のキャブ側のL字の右側の点を選ぶ
    x最大, y最小/最大らへんの点を選ぶ

    :param back_right_L_ji_points: 説明
    :type back_right_L_ji_points: NDArray[np.float64]
    :param n_target_points: 説明
    :type n_target_points: int
    :param cell_size: 説明
    :type cell_size: tuple[float, float, float]
    :param diff_y_th: 説明
    :type diff_y_th: float
    :param diff_x_th: 説明
    :type diff_x_th: float
    :param frac: 説明
    :type frac: float
    :param random_state: 説明
    :type random_state: int
    :return: 説明
    :rtype: tuple[NDArray[float64], NDArray[float64]]
    """
    df_points_info = (
        pd.DataFrame(back_right_L_ji_points, columns=["x", "y", "z"])
        .assign(
            vox_x=lambda df: (df.x // cell_size[0]).astype(int),
            vox_y=lambda df: (df.y // cell_size[1]).astype(int),
            vox_z=lambda df: (df.z // cell_size[2]).astype(int),
            diff_max_x=lambda df: df.x.max() - df.x,
            diff_min_x=lambda df: df.x - df.x.min(),
            diff_max_y=lambda df: df.y.max() - df.y,
            diff_min_y=lambda df: df.y - df.y.min(),
        )
        .pipe(
            lambda df: pd.concat(
                [
                    df.loc[
                        df[df.diff_min_y < diff_y_th]
                        .groupby(["vox_x", "vox_z"])
                        .y.idxmin()
                    ]
                    .sample(frac=frac, random_state=random_state)
                    .assign(is_min_y=1),
                    df.assign(is_min_y=0),
                ]
            )
        )
        .drop_duplicates(["x", "y", "z"])
        .pipe(
            lambda df: pd.concat(
                [
                    df.loc[
                        df[df.diff_max_y < diff_y_th]
                        .groupby(["vox_x", "vox_z"])
                        .y.idxmax()
                    ]
                    .sample(frac=frac, random_state=random_state)
                    .assign(is_max_y=1),
                    df.assign(is_max_y=0),
                ]
            )
        )
        .drop_duplicates(["x", "y", "z"])
        .pipe(
            lambda df: pd.concat(
                [
                    df.loc[
                        df[df.diff_max_x < diff_x_th]
                        .groupby(["vox_y", "vox_z"])
                        .x.idxmin()
                    ]
                    .sample(frac=frac, random_state=random_state)
                    .assign(is_min_x=1),
                    df.assign(is_min_x=0),
                ]
            )
        )
        .drop_duplicates(["x", "y", "z"])
        .assign(
            is_first=lambda df: (
                (df.is_min_x == 1) | (df.is_max_y == 1) | (df.is_min_y == 1)
            )
        )
    )

    return select_points_first_second(df_points_info, n_target_points, "is_first")


def select_back_left_L_ji_points_v2(
    back_left_L_ji_points: NDArray[np.float64],
    n_target_points: int,
    cell_size: tuple[float, float, float] = (0.1, 0.1, 0.035),
    diff_y_th: float = 0.1,
    diff_x_th: float = 0.1,
    frac: float = 0.75,
    random_state: int = 20250708,
) -> tuple[NDArray[np.float64], NDArray[np.float64]]:
    """
    下部走行体のキャブ側のL字の右側の点を選ぶ
    x最大, y最小/最大らへんの点を選ぶ

    :param back_left_L_ji_points: 説明
    :type back_left_L_ji_points: NDArray[np.float64]
    :param n_target_points: 説明
    :type n_target_points: int
    :param cell_size: 説明
    :type cell_size: tuple[float, float, float]
    :param diff_y_th: 説明
    :type diff_y_th: float
    :param diff_x_th: 説明
    :type diff_x_th: float
    :param frac: 説明
    :type frac: float
    :param random_state: 説明
    :type random_state: int
    :return: 説明
    :rtype: tuple[NDArray[float64], NDArray[float64]]
    """
    df_points_info = (
        pd.DataFrame(back_left_L_ji_points, columns=["x", "y", "z"])
        .assign(
            vox_x=lambda df: (df.x // cell_size[0]).astype(int),
            vox_y=lambda df: (df.y // cell_size[1]).astype(int),
            vox_z=lambda df: (df.z // cell_size[2]).astype(int),
            diff_max_x=lambda df: df.x.max() - df.x,
            diff_min_x=lambda df: df.x - df.x.min(),
            diff_max_y=lambda df: df.y.max() - df.y,
            diff_min_y=lambda df: df.y - df.y.min(),
        )
        .pipe(
            lambda df: pd.concat(
                [
                    df.loc[
                        df[df.diff_min_y < diff_y_th]
                        .groupby(["vox_x", "vox_z"])
                        .y.idxmin()
                    ]
                    .sample(frac=frac, random_state=random_state)
                    .assign(is_min_y=1),
                    df.assign(is_min_y=0),
                ]
            )
        )
        .drop_duplicates(["x", "y", "z"])
        .pipe(
            lambda df: pd.concat(
                [
                    df.loc[
                        df[df.diff_max_y < diff_y_th]
                        .groupby(["vox_x", "vox_z"])
                        .y.idxmax()
                    ]
                    .sample(frac=frac, random_state=random_state)
                    .assign(is_max_y=1),
                    df.assign(is_max_y=0),
                ]
            )
        )
        .drop_duplicates(["x", "y", "z"])
        .pipe(
            lambda df: pd.concat(
                [
                    df.loc[
                        df[df.diff_max_x < diff_x_th]
                        .groupby(["vox_y", "vox_z"])
                        .x.idxmin()
                    ]
                    .sample(frac=frac, random_state=random_state)
                    .assign(is_min_x=1),
                    df.assign(is_min_x=0),
                ]
            )
        )
        .drop_duplicates(["x", "y", "z"])
        .assign(
            is_first=lambda df: (
                (df.is_min_x == 1) | (df.is_max_y == 1) | (df.is_min_y == 1)
            )
        )
    )

    return select_points_first_second(df_points_info, n_target_points, "is_first")


def select_crawler_right_points_v2(
    crawler_right_points: NDArray[np.float64],
    n_target_points: int,
    cell_size: tuple[float, float, float] = (0.24, 0.24, 0.08),
    cell_size_over: tuple[float, float, float] = (0.12, 0.12, 0.04),
    except_y_th_max: float = 0.1,
    except_y_th_other: float = 0.3,
    z_th: float = 0.3,
    x_th: float = 0.15,
    min_y_th: float = 0.2,
    max_y_th: float = 0.4,
    random_state: int = 20250708,
    frac: float = 0.5,
) -> tuple[NDArray[np.float64], NDArray[np.float64]]:
    """
    右側のクローラー点群の選択
    x_max, x_min, y_min, y_max, z_maxな点を取り出す

    :param crawler_right_points: 説明
    :type crawler_right_points: NDArray[np.float64]
    :param n_target_points: 説明
    :type n_target_points: int
    :param cell_size: 説明
    :type cell_size: tuple[float, float, float]
    :param cell_size_over: 説明
    :type cell_size_over: tuple[float, float, float]
    :param except_y_th_max: 説明
    :type except_y_th_max: float
    :param except_y_th_other: 説明
    :type except_y_th_other: float
    :param z_th: 説明
    :type z_th: float
    :param x_th: 説明
    :type x_th: float
    :param min_y_th: 説明
    :type min_y_th: float
    :param max_y_th: 説明
    :type max_y_th: float
    :param random_state: 説明
    :type random_state: int
    :param frac: 説明
    :type frac: float
    :return: 説明
    :rtype: tuple[NDArray[float64], NDArray[float64]]
    """
    df_points_info = (
        pd.DataFrame(crawler_right_points, columns=["x", "y", "z"])
        .assign(
            vox_x=lambda df: (df.x // cell_size[0]).astype(int),
            vox_y=lambda df: (df.y // cell_size[1]).astype(int),
            vox_z=lambda df: (df.z // cell_size[2]).astype(int),
            vox_x_over=lambda df: (df.x // cell_size_over[0]).astype(int),
            vox_y_over=lambda df: (df.y // cell_size_over[1]).astype(int),
            vox_z_over=lambda df: (df.z // cell_size_over[2]).astype(int),
            diff_max_x=lambda df: df.x.max() - df.x,
            diff_min_x=lambda df: df.x - df.x.min(),
            diff_max_y=lambda df: df.y.max() - df.y,
            diff_min_y=lambda df: df.y - df.y.min(),
            diff_max_z=lambda df: df.z.max() - df.z,
        )
        .pipe(
            # zが大きい点を選ぶ
            lambda df: pd.concat(
                [
                    df.loc[
                        df[(df.diff_max_z < z_th) & (df.diff_max_y > except_y_th_other)]
                        .groupby(["vox_x_over", "vox_y_over"])
                        .z.idxmax()
                    ].assign(is_max_z=1),
                    df.assign(is_max_z=0),
                ]
            )
        )
        .drop_duplicates(["x", "y", "z"])
        .pipe(
            # xが大きい点を選ぶ
            lambda df: pd.concat(
                [
                    df.loc[
                        df[(df.diff_max_x < x_th) & (df.diff_max_y > except_y_th_other)]
                        .groupby(["vox_y", "vox_z"])
                        .x.idxmax()
                    ].assign(is_max_x=1),
                    df.assign(is_max_x=0),
                ]
            )
        )
        .drop_duplicates(["x", "y", "z"])
        .pipe(
            # xが小さい点を選ぶ
            lambda df: pd.concat(
                [
                    df.loc[
                        df[(df.diff_min_x < x_th) & (df.diff_max_y > except_y_th_other)]
                        .groupby(["vox_y", "vox_z"])
                        .x.idxmax()
                    ].assign(is_min_x=1),
                    df.assign(is_min_x=0),
                ]
            )
        )
        .drop_duplicates(["x", "y", "z"])
        .pipe(
            # yが小さい点を選ぶ
            lambda df: pd.concat(
                [
                    df.loc[
                        df[(df.diff_min_y < min_y_th)]
                        .groupby(["vox_x", "vox_z"])
                        .y.idxmin()
                    ]
                    .sample(frac=frac, random_state=random_state)
                    .assign(is_min_random_y=1),
                    df.assign(is_min_random_y=0),
                ]
            )
        )
        .drop_duplicates(["x", "y", "z"])
        .pipe(
            # yが高い点を選ぶ
            lambda df: pd.concat(
                [
                    df.loc[
                        df[
                            (except_y_th_max < df.diff_max_y)
                            & (df.diff_max_y < max_y_th)
                        ]
                        .groupby(["vox_x", "vox_z"])
                        .y.idxmin()
                    ]
                    .sample(frac=frac, random_state=random_state)
                    .assign(is_max_random_y=1),
                    df.assign(is_max_random_y=0),
                ]
            )
        )
        .drop_duplicates(["x", "y", "z"])
        .assign(
            is_first=lambda df: (
                (df.is_max_z == 1)
                | (df.is_max_x == 1)
                | (df.is_min_x == 1)
                | (df.is_min_random_y == 1)
                | (df.is_max_random_y == 1)
            )
        )
    )

    return select_points_first_second(df_points_info, n_target_points, "is_first")


def select_crawler_right_on_lines(
    crawler_right_points: NDArray[np.float64],
    line_cell_size: tuple[float, float, float] = (0.05, 0.05, 0.1),
    z_th: float = 0.075,
    exclusion_file_regex: str = "./data/argus_synchro_crawler_right_*_exclusion.npy",
) -> NDArray[np.float64]:
    """
    右側のクローラー点群の高さが一定のものを取り出す

    :param crawler_right_points: 説明
    :type crawler_right_points: NDArray[np.float64]
    :param line_cell_size: 説明
    :type line_cell_size: tuple[float, float, float]
    :param z_th: 説明
    :type z_th: float
    :param exclusion_file_regex: 説明
    :type exclusion_file_regex: str
    :return: 説明
    :rtype: NDArray[float64]
    """
    used_quantile = np.quantile(crawler_right_points[:, 2], 0.4)
    line_target_cond = "diff_y_max > 0.2 or diff_x_max < 1.5 or diff_x_min < 0.1"
    surrounding_cond = (
        "(min_x + 0.075 <= x <= max_x - 0.075) and (min_y + 0.2 <= y <= max_y - 0.3)"
    )

    _third_points = crawler_right_points[
        np.abs(crawler_right_points[:, 2] - used_quantile) < z_th
    ]

    df_third_points = (
        pd.DataFrame(_third_points, columns=["x", "y", "z"])
        .assign(
            vox_x=lambda df: (df.x // line_cell_size[0]).astype(int),
            vox_y=lambda df: (df.y // line_cell_size[1]).astype(int),
            vox_z=lambda df: (df.z // line_cell_size[2]).astype(int),
            diff_y_max=lambda df: df.y.max() - df.y,
            diff_x_max=lambda df: df.x.max() - df.x,
            diff_x_min=lambda df: df.x - df.x.min(),
        )
        .query(line_target_cond)
        .assign(
            # 機体の真ん中の部位は消したい
            min_y=lambda df: df.y.min(),
            max_y=lambda df: df.y.max(),
            min_x=lambda df: df.x.min(),
            max_x=lambda df: df.x.max(),
        )
        .pipe(
            lambda df: pd.concat(
                [
                    df.query(surrounding_cond).assign(in_rect=1),
                    df.assign(in_rect=0),
                ]
            ).drop_duplicates(["x", "y", "z"])
        )
        .pipe(
            lambda df: pd.concat(
                [
                    df.loc[df.groupby(["vox_x", "vox_y"]).z.idxmax()].assign(
                        is_max_z=1
                    ),
                    df.assign(is_max_z=0),
                ]
            ).drop_duplicates(["x", "y", "z"])
        )
        .assign(is_chosen=lambda df: (df.in_rect == 0) & (df.is_max_z == 1))
    )

    # Remark: 除外処理をしているが、returnには貢献してないので、貢献するようにするか検討したほうが良い
    # 上記の処理だと取り除けない点がある場合、exclusion_filesから点を取り出して、除外
    exclusion_points = np.vstack(
        [np.load(filename) for filename in glob(exclusion_file_regex)]
    )

    third_points = pd2np(df_third_points.query("is_chosen == 1"))
    third_points = third_points[
        ~(
            np.sqrt(
                (
                    (
                        third_points[:, np.newaxis, :]
                        - exclusion_points[np.newaxis, :, :]
                    )
                    ** 2
                ).sum(axis=2)
            )
            < 0.01
        ).any(axis=1)
    ]

    # is_rectでなく, is_max_zであるものを選ぶ
    third_points = pd2np(df_third_points.query("in_rect == 0 and is_max_z == 1"))

    return third_points


def select_crawler_left_points(
    crawler_left_points: NDArray[np.float64],
    n_target_points: int,
    cell_size: tuple[float, float, float] = (0.24, 0.24, 0.08),
    cell_size_over: tuple[float, float, float] = (0.12, 0.12, 0.04),
    except_y_th_min: float = 0.2,
    except_y_th_other: float = 0.3,
    z_th: float = 0.3,
    x_th: float = 0.15,
    max_y_th: float = 0.2,
    min_y_th: float = 0.4,
    random_state: int = 20250708,
    frac: float = 0.5,
) -> tuple[NDArray[np.float64], NDArray[np.float64]]:
    """
    左側のクローラー点群の選択
    x_max, x_min, y_min, y_max, z_maxな点を取り出す

    :param crawler_left_points: 説明
    :type crawler_left_points: NDArray[np.float64]
    :param n_target_points: 説明
    :type n_target_points: int
    :param cell_size: 説明
    :type cell_size: tuple[float, float, float]
    :param cell_size_over: 説明
    :type cell_size_over: tuple[float, float, float]
    :param except_y_th_min: 説明
    :type except_y_th_min: float
    :param except_y_th_other: 説明
    :type except_y_th_other: float
    :param z_th: 説明
    :type z_th: float
    :param x_th: 説明
    :type x_th: float
    :param max_y_th: 説明
    :type max_y_th: float
    :param min_y_th: 説明
    :type min_y_th: float
    :param random_state: 説明
    :type random_state: int
    :param frac: 説明
    :type frac: float
    :return: 説明
    :rtype: tuple[NDArray[float64], NDArray[float64]]
    """
    df_points_info = (
        pd.DataFrame(crawler_left_points, columns=["x", "y", "z"])
        .assign(
            vox_x=lambda df: (df.x // cell_size[0]).astype(int),
            vox_y=lambda df: (df.y // cell_size[1]).astype(int),
            vox_z=lambda df: (df.z // cell_size[2]).astype(int),
            vox_x_over=lambda df: (df.x // cell_size_over[0]).astype(int),
            vox_y_over=lambda df: (df.y // cell_size_over[1]).astype(int),
            vox_z_over=lambda df: (df.z // cell_size_over[2]).astype(int),
            diff_max_x=lambda df: df.x.max() - df.x,
            diff_min_x=lambda df: df.x - df.x.min(),
            diff_max_y=lambda df: df.y.max() - df.y,
            diff_min_y=lambda df: df.y - df.y.min(),
            diff_max_z=lambda df: df.z.max() - df.z,
        )
        .pipe(
            lambda df: pd.concat(
                [
                    df.loc[
                        df[(df.diff_max_z < z_th) & (df.diff_min_y > except_y_th_other)]
                        .groupby(["vox_x_over", "vox_y_over"])
                        .z.idxmax()
                    ].assign(is_max_z=1),
                    df.assign(is_max_z=0),
                ]
            )
        )
        .drop_duplicates(["x", "y", "z"])
        .pipe(
            lambda df: pd.concat(
                [
                    df.loc[
                        df[(df.diff_max_x < x_th) & (df.diff_min_y > except_y_th_other)]
                        .groupby(["vox_y", "vox_z"])
                        .x.idxmax()
                    ].assign(is_max_x=1),
                    df.assign(is_max_x=0),
                ]
            )
        )
        .drop_duplicates(["x", "y", "z"])
        .pipe(
            lambda df: pd.concat(
                [
                    df.loc[
                        df[(df.diff_min_x < x_th) & (df.diff_min_y > except_y_th_other)]
                        .groupby(["vox_y", "vox_z"])
                        .x.idxmax()
                    ].assign(is_min_x=1),
                    df.assign(is_min_x=0),
                ]
            )
        )
        .drop_duplicates(["x", "y", "z"])
        .pipe(
            lambda df: pd.concat(
                [
                    df.loc[
                        df[(df.diff_max_y < max_y_th)]
                        .groupby(["vox_x", "vox_z"])
                        .y.idxmin()
                    ]
                    .sample(frac=frac, random_state=random_state)
                    .assign(is_max_random_y=1),
                    df.assign(is_max_random_y=0),
                ]
            )
        )
        .drop_duplicates(["x", "y", "z"])
        .pipe(
            lambda df: pd.concat(
                [
                    df.loc[
                        df[
                            (except_y_th_min < df.diff_min_y)
                            & (df.diff_min_y < min_y_th)
                        ]
                        .groupby(["vox_x", "vox_z"])
                        .y.idxmin()
                    ]
                    .sample(frac=frac, random_state=random_state)
                    .assign(is_min_random_y=1),
                    df.assign(is_min_random_y=0),
                ]
            )
        )
        .drop_duplicates(["x", "y", "z"])
        .assign(
            is_first=lambda df: (
                (df.is_max_z == 1)
                | (df.is_max_x == 1)
                | (df.is_min_x == 1)
                | (df.is_min_random_y == 1)
                | (df.is_max_random_y == 1)
            )
        )
    )

    return select_points_first_second(df_points_info, n_target_points, "is_first")


def select_crawler_left_on_lines(
    crawler_left_points: NDArray[np.float64],
    line_cell_size: tuple[float, float, float] = (0.05, 0.05, 0.1),
    z_th: float = 0.075,
    exclusion_file_regex: str = "./data/argus_synchro_crawler_left_*_exclusion.npy",
) -> NDArray[np.float64]:
    """
    左側のクローラー点群の高さが一定のものを取り出す

    :param crawler_left_points: 説明
    :type crawler_left_points: NDArray[np.float64]
    :param line_cell_size: 説明
    :type line_cell_size: tuple[float, float, float]
    :param z_th: 説明
    :type z_th: float
    :param exclusion_file_regex: 説明
    :type exclusion_file_regex: str
    :return: 説明
    :rtype: NDArray[float64]
    """
    used_quantile = np.quantile(crawler_left_points[:, 2], 0.35)
    line_target_cond = "diff_y_min > 0.3 or diff_x_max < 1.5 or diff_x_min < 0.1"
    surrounding_cond = (
        "(min_x + 0.075 <= x <= max_x - 0.075) and (min_y + 0.3 <= y <= max_y - 0.2)"
    )

    _third_points = crawler_left_points[
        np.abs(crawler_left_points[:, 2] - used_quantile) < z_th
    ]

    df_third_points = (
        pd.DataFrame(_third_points, columns=["x", "y", "z"])
        .assign(
            vox_x=lambda df: (df.x // line_cell_size[0]).astype(int),
            vox_y=lambda df: (df.y // line_cell_size[1]).astype(int),
            vox_z=lambda df: (df.z // line_cell_size[2]).astype(int),
            diff_y_min=lambda df: (
                df.y - df.y.min()
            ),  # diff_y_maxが小さい部分は、L字を指す部分が該当する
            diff_x_max=lambda df: df.x.max() - df.x,
            diff_x_min=lambda df: df.x - df.x.min(),
        )
        .query(line_target_cond)
        .assign(
            # 機体の真ん中の部位は消したい
            min_y=lambda df: df.y.min(),
            max_y=lambda df: df.y.max(),
            min_x=lambda df: df.x.min(),
            max_x=lambda df: df.x.max(),
        )
        .pipe(
            lambda df: pd.concat(
                [
                    df.query(surrounding_cond).assign(in_rect=1),
                    df.assign(in_rect=0),
                ]
            ).drop_duplicates(["x", "y", "z"])
        )
        .pipe(
            lambda df: pd.concat(
                [
                    df.loc[df.groupby(["vox_x", "vox_y"]).z.idxmax()].assign(
                        is_max_z=1
                    ),
                    df.assign(is_max_z=0),
                ]
            ).drop_duplicates(["x", "y", "z"])
        )
        .assign(is_chosen=lambda df: (df.in_rect == 0) & (df.is_max_z == 1))
    )

    # 上記の処理だと取り除けない点がある場合、exclusion_filesから点を取り出して、除外
    exclusion_points = [np.load(filename) for filename in glob(exclusion_file_regex)]
    third_points = pd2np(df_third_points.query("is_chosen == 1"))
    if len(exclusion_points) == 0:
        return third_points
    exclusion_points = np.vstack(exclusion_points)

    third_points = third_points[
        ~(
            np.sqrt(
                (
                    (
                        third_points[:, np.newaxis, :]
                        - exclusion_points[np.newaxis, :, :]
                    )
                    ** 2
                ).sum(axis=2)
            )
            < 0.01
        ).any(axis=1)
    ]

    return third_points
