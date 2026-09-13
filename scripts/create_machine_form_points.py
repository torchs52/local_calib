#!/usr/bin/env python

# # 機体点群除去用の機体形状情報を生成するスクリプト


# ## ライブラリ読み込み

import os
import sys
from abc import ABC, abstractmethod

import numpy as np
import pandas as pd

from argus_synchro.config.machine_collision import MachineConf, load_machine_info

# ## 関数読み込み


def read_saved_points(file_path):
    points = np.genfromtxt(file_path, delimiter=" ")
    return points


def connect_path(*arg) -> str:
    return os.sep.join(arg).replace("\\", "/")


class CreateMachineFormInterface(ABC):
    """
    機体形状を作るインターフェース
    """

    @abstractmethod
    def create_machine_form(
        self,
        file_dir: str,
        col_machine_info: list[MachineConf],
        custom_reverse: tuple[bool, bool, bool] | None = None,
    ) -> None:
        """
        機体点群除去に用いる機体の形状情報を作る関数
        このInterfaceを継承するクラスで実装する

        :param file_dir: 説明
        :type file_dir: str
        :param col_machine_info: 説明
        :type col_machine_info: list[MachineConf]
        :param custom_reverse: 説明
        :type custom_reverse: tuple[bool, bool, bool] | None
        """


class CreateCuboidSimple(CreateMachineFormInterface):
    """
    直方体の機体点群除去に必要な情報作るためのクラス
    特にreverse判定を自由に扱いたかったので、custom_reverseに応じて機体点群のxyz座標を反転させたりできるようにしている
    """

    def create_machine_form(
        self,
        file_dir: str,
        col_machine_info: list[MachineConf],
        custom_reverse: tuple[bool, bool, bool] | None = None,
    ) -> None:
        for machine_parts in col_machine_info:
            input_file = connect_path(file_dir, machine_parts.pcd_points_file)
            output_file = connect_path(
                file_dir,
                os.path.splitext(machine_parts.pcd_points_file)[0]
                + "_cuboid_points.csv",
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


class CreateCuround(CreateMachineFormInterface):
    """
    直方体+円弧柱の機体点群除去に必要な情報を作るためのクラス
    情報を作るためにfeature_file_nameに該当するcsvファイルから情報を取り出したりするので、コンストラクタで事前に必要な情報を定義している
    """

    cw_ind: int | None
    cuboid_indices: list[str]
    round_indices: list[list[str]]
    feature_file_name: str

    def __init__(
        self,
        feature_file_name: str,
        cw_ind: int | None = 1,
        cuboid_indices: list[str] = ["Pa", "Pb", "Pd", "Pc", "Pf", "Pg", "Ph", "Pi"],
        round_indices: list[list[str]] = [
            ["Pd", "Pk", "Pe", "Pi", "Pm", "Pj"],
            ["Pe", "Pl", "Pc", "Pj", "Pn", "Ph"],
        ],
        # feature_file_name: str = "CW_座標_車体点群除去用.csv",
    ) -> None:
        self.cw_ind = cw_ind
        self.cuboid_indices = cuboid_indices
        self.round_indices = round_indices
        self.feature_file_name = feature_file_name

    def create_machine_form(
        self,
        file_dir: str,
        col_machine_info: list[MachineConf],
        custom_reverse: tuple[bool, bool, bool] | None = None,
    ) -> None:
        if self.cw_ind is None:
            return

        # cw_for_remove_machine_points = f"{file_dir}/{self.feature_file_name}"
        cw_for_remove_machine_points = self.feature_file_name
        target_machine_conf = col_machine_info[self.cw_ind]

        assert os.path.exists(file_dir), f"file_dir={file_dir}が見つかりません。"

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
                            2 * np.array(target_machine_conf.reverse).astype(int) - 1
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
                            2 * np.array(target_machine_conf.reverse).astype(int) - 1
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
                            2 * np.array(target_machine_conf.reverse).astype(int) - 1
                        ).astype(
                            int,
                        )
                    )[2]
                ),
            )
        )

        df_cw_cuboid_points = (df_cw_form_points).loc[self.cuboid_indices]
        np_cw_form_points = np.vstack(
            [
                df_cw_cuboid_points.iloc[
                    np.argmin(
                        (
                            (df_cw_cuboid_points - df_cw_form_points.min(axis=0)) ** 2
                        ).sum(
                            axis=1,
                        ),
                    )
                ],
                df_cw_cuboid_points.iloc[
                    np.argmax(
                        (
                            (df_cw_cuboid_points - df_cw_form_points.min(axis=0)) ** 2
                        ).sum(
                            axis=1,
                        ),
                    )
                ],
            ]
            + [df_cw_form_points.loc[round_ind] for round_ind in self.round_indices],
        )

        np.savetxt(
            connect_path(
                file_dir,
                os.path.splitext(target_machine_conf.pcd_points_file)[0]
                + "_curound_points.csv",
            ),
            np_cw_form_points,
        )


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
    cw_ind: int = 1,
    cuboid_indices: list[str] = ["Pa", "Pb", "Pd", "Pc", "Pf", "Pg", "Ph", "Pi"],
    round_indices: list[list[str]] = [
        ["Pd", "Pk", "Pe", "Pi", "Pm", "Pj"],
        ["Pe", "Pl", "Pc", "Pj", "Pn", "Ph"],
    ],
    feature_file: str = "CW_座標_車体点群除去用.csv",
):
    cw_for_remove_machine_points = f"{file_dir}/{feature_file}"

    assert os.path.exists(file_dir), f"file_dir={file_dir}が見つかりません。"

    # # カウンタウェイトの形状で必要な点を取り出す
    # + 直方体を表す, 最小, 最大座標の後は、6点ずつ弧柱を表現する点を作る
    # cw_ind = 1
    # cuboid_indices = ["Pa", "Pb", "Pd", "Pc", "Pf", "Pg", "Ph", "Pi"]

    # round_indices = [
    #    ["Pd", "Pk", "Pe", "Pi", "Pm", "Pj"],
    #    ["Pe", "Pl", "Pc", "Pj", "Pn", "Ph"],
    # ]

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
    create_list: list[CreateMachineFormInterface],
    custom_reverse: tuple[bool, bool, bool] | None = None,
    # funcs: list[Callable] = [create_machine_cuboid_rigorus],
) -> None:
    for elem in create_list:
        elem.create_machine_form(
            file_dir=file_dir,
            col_machine_info=col_machine_info,
            custom_reverse=custom_reverse,
        )
    # for func in funcs:
    #    func(file_dir, col_machine_info, **func_params)


DEFAULT_MODE = "withoutCuround"

if __name__ == "__main__":
    mode = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_MODE
    print(f"mode = {mode}")

    # 200t機の設定の例
    # file_dir = "./config/crane3d/collision_detection/SCX2000-3/test"
    # json_file = "col_machine_info.jsonc"
    # py_machine_confs = load_machine_info(connect_path(file_dir, json_file))
    # custom_reverse = (True, True, False)

    # 90t機の設定の例
    file_dir = "./config/crane3d/collision_detection/SCX900-3/test"
    json_file = "col_machine_info.jsonc"
    py_machine_confs = load_machine_info(connect_path(file_dir, json_file))
    custom_reverse = (True, True, False)
    match mode:
        case "withoutCuround":
            create_list: list[CreateMachineFormInterface] = [CreateCuboidSimple()]
        case "withCuround":
            # CuboidとCuroundを作る場合
            create_list: list[CreateMachineFormInterface] = [
                CreateCuboidSimple(),
                CreateCuround(
                    cw_ind=1,
                    feature_file_name="./scripts/CW_座標_車体点群除去用.csv",
                ),
            ]
        case x:
            raise ValueError(
                f"mode = {mode}, この値はWithoutCuroundかWithCuroundしか想定していないです"
            )

    create_machine_form_main(
        file_dir=file_dir,
        col_machine_info=py_machine_confs,
        create_list=create_list,
        custom_reverse=custom_reverse,
    )
