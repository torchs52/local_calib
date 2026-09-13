"""機体除去, 衝突判定で用いる機体情報に関するクラスが入っているモジュール
C++側に多くは移行しているが、jsonからMachineConfを作る部分がC++に移行できておらず
json -> list[Python側のMachineConf] -> list[C++側のMachineConf]という変換を行っている
"""

import os
from dataclasses import dataclass

import argus_synchro.common.common as com


@dataclass
class MachineConf:
    """機体除去, 衝突判定で用いる機体情報が入ったdataclass"""

    pcd_points_file: str  # 読み込むファイルを特定するためのファイル名, 同じディレクトリに同名が存在すると意図したファイルを呼び出せなくなるので、同名が存在しないように設定する
    load_order: int  # 読み込む順番, 読み込む順が早いものから順に任意の整数を設定すれば、その順に並び替えられる
    instance_name: str  # 呼び出すクラス名, クローラーと上部旋回体、カウンタウェイトなどで使うクラスが異なるので、どのクラスを呼び出すかを指定するための文字列
    offsets: tuple[float, float, float] = (0, 0, 0)
    reverse: tuple[bool, bool, bool] = (False, True, False)
    form_points_pattern: str = "cuboid_points"  # 機体除去で用いる形状点群ファイル名のパターン, ${pcd_points_fileから拡張子を除いた文字列} + ${form_points_patter}.csv をloadする

    # 旋回可能かどうか, Trueの場合、旋回可能な部位であることを表す
    is_mobile: bool = False

    def get_form_points_filename(self, filename: str) -> str:
        """形状点群が入ったファイルを取得する"""
        return os.path.splitext(filename)[0] + f"_{self.form_points_pattern}.csv"


def load_machine_info(json_machine_info: str) -> list[MachineConf]:
    """
    MachineConfの引数のリストが入ったjsonを読み込んでMachineConfのリストを生成する関数

    :param json_machine_info: MachineConfの引数が入ったjson
    :type json_machine_info: str
    :return: 説明
    :rtype: list[MachineConf]
    """

    raw_machine_info = com.read_jsonc(json_machine_info)

    # load_orderの順に並べてリストにして返す
    return sorted(
        [MachineConf(**elem) for elem in raw_machine_info],
        key=lambda elem: elem.load_order,
    )
