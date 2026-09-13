#!/usr/bin/env python

# # 機体形状のデータを作るためのnotebook
# + いずれpythonファイルに変換するが、最初はnotebookで生成する

# # 初期設定

# ## ライブラリ読み込み

# In[2]:


import os
import sys
from collections.abc import Callable

import numpy as np
import pandas as pd

sys.path.append(os.path.abspath("./"))
from config.machine_collision import MachineConf, load_machine_info

# ## 関数読み込み


def read_saved_points(file_path):
    points = np.genfromtxt(file_path, delimiter=" ")
    return points


def connect_path(*arg) -> str:
    return os.sep.join(arg).replace("\\", "/")


def create_machine_cuboid_simple(
    file_dir: str,
    col_machine_info: list[MachineConf],
    custom_reverse: tuple[bool, bool, bool] | None = None,
):
    """単純に各機体部位のxyzの最大値と最小値を取取り出して書き込む"""
    # col_machine_info = conf_col.machine_info

    for machine_parts in col_machine_info:
        input_file = connect_path(file_dir, machine_parts.pcd_points_file)
        output_file = connect_path(
            file_dir,
            os.path.splitext(machine_parts.pcd_points_file)[0] + "_cuboid_points.csv",
        )
        _reverse = (
            np.array(custom_reverse)
            if custom_reverse
            else np.array(machine_parts.reverse)
        )

        print(f"machine_parts={machine_parts}, reverse={_reverse}")

        machine_points = read_saved_points(input_file)[:, :3] * (
            -1 * (2 * _reverse.astype(int) - 1)
        ).astype(int)

        form_points = np.vstack(
            [
                machine_points.min(axis=0),
                machine_points.max(axis=0),
            ]
        )
        np.savetxt(output_file, form_points)


def create_machine_cuboid_rigorus(
    file_dir: str,
    col_machine_info: list[MachineConf],
    custom_reverse: tuple[bool, bool, bool] | None = None,
):
    """機体部位の最小値を原点に並進させた時に一番離れている点と最小値を選ぶ"""
    # col_machine_info = conf_col.machine_info

    for machine_parts in col_machine_info:
        input_file = connect_path(file_dir, machine_parts.pcd_points_file)
        output_file = connect_path(
            file_dir,
            os.path.splitext(machine_parts.pcd_points_file)[0] + "_cuboid_points.csv",
        )
        _reverse = (
            np.array(custom_reverse)
            if custom_reverse
            else np.array(machine_parts.reverse)
        )

        print(f"machine_parts={machine_parts}, reverse={_reverse}")

        machine_points = read_saved_points(input_file)[:, :3] * (
            -1 * (2 * _reverse.astype(int) - 1)
        ).astype(int)

        # xyz座標の最小値分だけ並進すればすべての点が非負で、原点からの距離の最小と最大が、直方体の隅になる
        machine_dist = np.sqrt(
            ((machine_points - machine_points.min(axis=0)) ** 2).sum(axis=1),
        )

        form_points = np.vstack(
            [
                machine_points[np.argmin(machine_dist)],
                machine_points[np.argmax(machine_dist)],
            ],
        )
        np.savetxt(output_file, form_points)


def create_machine_curound(
    file_dir: str,
    col_machine_info: list[MachineConf],
    feature_file: str = "CW_座標_車体点群除去用.csv",
):
    cw_for_remove_machine_points = f"{file_dir}/{feature_file}"

    assert os.path.exists(file_dir), f"file_dir={file_dir}が見つかりません。"

    # # カウンタウェイトの形状で必要な点を取り出す
    # + 直方体を表す, 最小, 最大座標の後は、6点ずつ弧柱を表現する点を作る
    cw_ind = 1
    cuboid_indices = ["Pa", "Pb", "Pd", "Pc", "Pf", "Pg", "Ph", "Pi"]

    round_indices = [
        ["Pd", "Pk", "Pe", "Pi", "Pm", "Pj"],
        ["Pe", "Pl", "Pc", "Pj", "Pn", "Ph"],
    ]

    df_cw_form_points = (
        pd.read_csv(cw_for_remove_machine_points)
        .rename(columns={"Unnamed: 0": "index"})
        .set_index("index")
        .assign(
            x=lambda df: (
                df.x
                * (
                    -1
                    * (
                        2 * np.array(col_machine_info[cw_ind].reverse).astype(int) - 1
                    ).astype(
                        int,
                    )
                )[0]
            ),
            y=lambda df: (
                df.y
                * (
                    -1
                    * (
                        2 * np.array(col_machine_info[cw_ind].reverse).astype(int) - 1
                    ).astype(
                        int,
                    )
                )[1]
            ),
            z=lambda df: (
                df.z
                * (
                    -1
                    * (
                        2 * np.array(col_machine_info[cw_ind].reverse).astype(int) - 1
                    ).astype(
                        int,
                    )
                )[2]
            ),
        )
    )

    df_cw_cuboid_points = (df_cw_form_points).loc[cuboid_indices]
    np_cw_form_points = np.vstack(
        [
            df_cw_cuboid_points.iloc[
                np.argmin(
                    ((df_cw_cuboid_points - df_cw_form_points.min(axis=0)) ** 2).sum(
                        axis=1,
                    ),
                )
            ],
            df_cw_cuboid_points.iloc[
                np.argmax(
                    ((df_cw_cuboid_points - df_cw_form_points.min(axis=0)) ** 2).sum(
                        axis=1,
                    ),
                )
            ],
        ]
        + [df_cw_form_points.loc[round_ind] for round_ind in round_indices],
    )

    np.savetxt(
        connect_path(
            file_dir,
            os.path.splitext(col_machine_info[cw_ind].pcd_points_file)[0]
            + "_curound_points.csv",
        ),
        np_cw_form_points,
    )


def create_machine_form_main(
    file_dir: str,
    col_machine_info: list[MachineConf],
    funcs: list[Callable] = [create_machine_cuboid_rigorus],
    **func_params,
) -> None:
    for func in funcs:
        func(file_dir, col_machine_info, **func_params)

    # cw_for_remove_machine_points = (
    #    "./config/crane3d/collision_detection/CW_座標_車体点群除去用.csv"
    # )

    # assert os.path.exists(file_dir), f"file_dir={file_dir}が見つかりません。"

    ## col_machine_info = conf_col.machine_info

    # machine_points = [
    #    o3d.utility.Vector3dVector(
    #        read_saved_points(connect_path(file_dir, machine_parts.pcd_points_file))[
    #            :, :3
    #        ],
    #    )
    #    for machine_parts in col_machine_info
    # ]

    # pcds = []
    # for machine_parts in machine_points:
    #    pcd = o3d.geometry.PointCloud()
    #    pcd.points = machine_parts
    #    pcd.paint_uniform_color([0, 0, 0])
    #    pcds.append(pcd)

    # for machine_parts in col_machine_info:
    #    input_file = connect_path(file_dir, machine_parts.pcd_points_file)
    #    output_file = connect_path(
    #        file_dir,
    #        os.path.splitext(machine_parts.pcd_points_file)[0] + "_cuboid_points.csv",
    #    )
    #    machine_points = read_saved_points(input_file)[:, :3] * (
    #        -1 * (2 * np.array(machine_parts.reverse).astype(int) - 1)
    #    ).astype(int)

    #    # xyz座標の最小値分だけ並進すればすべての点が非負で、原点からの距離の最小と最大が、直方体の隅になる
    #    machine_dist = np.sqrt(
    #        ((machine_points - machine_points.min(axis=0)) ** 2).sum(axis=1),
    #    )

    #    form_points = np.vstack(
    #        [
    #            machine_points[np.argmin(machine_dist)],
    #            machine_points[np.argmax(machine_dist)],
    #        ],
    #    )
    #    np.savetxt(output_file, form_points)

    ## # カウンタウェイトの形状で必要な点を取り出す
    ## + 直方体を表す, 最小, 最大座標の後は、6点ずつ弧柱を表現する点を作る
    # cw_ind = 1
    # cuboid_indices = ["Pa", "Pb", "Pd", "Pc", "Pf", "Pg", "Ph", "Pi"]

    # round_indices = [
    #    ["Pd", "Pk", "Pe", "Pi", "Pm", "Pj"],
    #    ["Pe", "Pl", "Pc", "Pj", "Pn", "Ph"],
    # ]

    # df_cw_form_points = (
    #    pd.read_csv(cw_for_remove_machine_points)
    #    .rename(columns={"Unnamed: 0": "index"})
    #    .set_index("index")
    #    .assign(
    #        x=lambda df: df.x
    #        * (
    #            -1
    #            * (
    #                2 * np.array(col_machine_info[cw_ind].reverse).astype(int) - 1
    #            ).astype(
    #                int,
    #            )
    #        )[0],
    #        y=lambda df: df.y
    #        * (
    #            -1
    #            * (
    #                2 * np.array(col_machine_info[cw_ind].reverse).astype(int) - 1
    #            ).astype(
    #                int,
    #            )
    #        )[1],
    #        z=lambda df: df.z
    #        * (
    #            -1
    #            * (
    #                2 * np.array(col_machine_info[cw_ind].reverse).astype(int) - 1
    #            ).astype(
    #                int,
    #            )
    #        )[2],
    #    )
    # )

    # df_cw_cuboid_points = (df_cw_form_points).loc[cuboid_indices]
    # np_cw_form_points = np.vstack(
    #    [
    #        df_cw_cuboid_points.iloc[
    #            np.argmin(
    #                ((df_cw_cuboid_points - df_cw_form_points.min(axis=0)) ** 2).sum(
    #                    axis=1,
    #                ),
    #            )
    #        ],
    #        df_cw_cuboid_points.iloc[
    #            np.argmax(
    #                ((df_cw_cuboid_points - df_cw_form_points.min(axis=0)) ** 2).sum(
    #                    axis=1,
    #                ),
    #            )
    #        ],
    #    ]
    #    + [df_cw_form_points.loc[round_ind] for round_ind in round_indices],
    # )

    # np.savetxt(
    #    connect_path(
    #        file_dir,
    #        os.path.splitext(col_machine_info[cw_ind].pcd_points_file)[0]
    #        + "_curound_points.csv",
    #    ),
    #    np_cw_form_points,
    # )


if __name__ == "__main__":
    file_dir = "./config/crane3d/collision_detection/SCX2000-3/weighted"
    json_file = "col_machine_info.jsonc"
    py_machine_confs = load_machine_info(connect_path(file_dir, json_file))
    custom_reverse = [True, True, False]
    create_machine_form_main(
        file_dir,
        py_machine_confs,
        funcs=[create_machine_cuboid_simple],
        custom_reverse=custom_reverse,
    )
