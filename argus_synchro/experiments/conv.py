"""C++に変換された型や関数などで、Python側から呼びたい場合に用いるモジュール

経緯:
Python側の機能を完全に実装されていなくて、C++のライブラリを呼んだ時に、欲しい変数が作れない状況が発生していて、C++にそれを実装するのも手間なので、
Python側からいい感じにC++のライブラリを使えるようにしたくなったので作成

方針:
基本的に、C++ライブラリを適宜修正するが、修正するよりもPython側のモジュールを上手く活用すれば修正せずに済んで、処理性能を落とさないようなものに用いる
"""

from argus_synchro_lib.machine_collision import MachineConf as CppMachineConf

from argus_synchro.config.machine_collision import MachineConf as pyMachineConf


def py_machine_info_to_cpp(machine_info: list[pyMachineConf]) -> list[CppMachineConf]:
    """Python側で実装されたmachine_infoからC++のmachine_infoに変換する関数
    現状のライブラリ上は、コンパイルしなおさないと、machine_infoの付け替えができなくて、

    pybindにmachine_info_lightningを登録するのが上手くいかなかったので、Python側で作ったmachine_infoをc++のmachineに変換する
    """
    return [
        CppMachineConf(
            pcd_points_file=conf.pcd_points_file,
            load_order=conf.load_order,
            instance_name=conf.instance_name,
            offsets=conf.offsets,
            reverse=conf.reverse,
            is_mobile=conf.is_mobile,
            form_points_pattern=conf.form_points_pattern,
        )
        for conf in machine_info
    ]
