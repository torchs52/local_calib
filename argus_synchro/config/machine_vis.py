"""可視化用の機体の設定値が入っているモジュール"""

from dataclasses import dataclass

import argus_synchro.common.common as com


@dataclass
class VisMachineConf:
    file_base: str  # 読み込むファイルを特定するためのファイル名, 同じディレクトリに同名が存在すると意図したファイルを呼び出せなくなるので、同名が存在しないように設定する
    load_order: int  # 読み込む順番, 読み込む順が早いものから順に任意の整数を設定すれば、その順に並び替えられる
    offsets: tuple[float, float, float] = (0, 0, 0)
    color: tuple[float, float, float] = (0.4, 0.4, 0.4)
    reverse: tuple[bool, bool, bool] = (True, True, False)

    # 該当部位が旋回可能かどうかを判定, Trueの場合回転できる部位, Falseの場合回転できない部位
    # 基本的に下部走行体がTrueになるはずでデフォルトはFalseにしている
    rotatable: bool = False


def load_vis_machine_info(json_vis_machine_file: str) -> list[VisMachineConf]:
    """
    open3d用のVisMachineConfのリストが入ったjsonを読み込んでVisMachineConfのリストを生成する関数

    :param json_vis_machine_info: VisMachineConfの引数が入ったjson
    :type json_vis_machine_file: str
    :return: 説明
    :rtype: list[MachineConf]
    """
    raw_machine_info = com.read_jsonc(json_vis_machine_file)

    return sorted(
        [VisMachineConf(**elem) for elem in raw_machine_info],
        key=lambda elem: elem.load_order,
    )
