import os
import re
import sys
from configparser import ConfigParser, ExtendedInterpolation

import pandas as pd


def find_db_patterns_first(text) -> None | tuple[int, int]:
    # 正規表現パターンを定義
    pattern = r"\(db\:[a-zA-Z0-9_]+\)"

    # 一致するすべてのパターンを見つける
    matches: re.Match[str] | None = re.search(pattern, text)
    if matches is None:
        return None
    return matches.span()


def ini_replace_argnum(ini: ConfigParser, arglist: list = sys.argv, verb=False):
    for i, s in reversed(
        list(enumerate(arglist))
    ):  # 昇順だと1,2,3という数字の時に11,21,31等が置き換わってしまう。間に合わせの対応のためiniの%番号の値と引数の数が合わないと異常な動作をする
        if verb:
            print("Replace", i, " to ", s)
        for x in ini:
            if str(type(ini[x])) == "<class 'configparser.SectionProxy'>":
                for y in ini[x]:
                    if type(ini[x][y]) is str:
                        if ini[x][y].find("%" + str(i)) >= 0:
                            origstr = ini[x][y]
                            ini[x][y] = ini[x][y].replace("%" + str(i), s)
                            if verb:
                                print(x, ">", y, ":", origstr, " -> ", ini[x][y])
    return ini


def dbread_multiformat(db_path, db_type):  # -> None | Any | Any:
    db = None
    for method, filetype, argdict in [
        (pd.read_csv, "csv", {"index_col": 0}),
        (pd.read_excel, "excel", {"index_col": 0}),
        (pd.read_pickle, "pickle", {}),
    ]:
        if db_type == filetype:
            db = method(db_path, **argdict)
    return db


def ini_replace_db(ini: ConfigParser, db_series, verb=False):
    for x in ini:
        if str(type(ini[x])) == "<class 'configparser.SectionProxy'>":
            for y in ini[x]:
                if type(ini[x][y]) is str:
                    looplimit = 5
                    while looplimit > 0:
                        looplimit -= 1
                        pos: None | tuple[int, int] = find_db_patterns_first(ini[x][y])
                        if pos is None:
                            break

                        origstr = ini[x][y]
                        db_colkey = origstr[pos[0] + len("(db:") : pos[1] - 1]

                        ini[x][y] = (
                            origstr[: pos[0]] + db_series[db_colkey] + origstr[pos[1] :]
                        )

                        if verb:
                            print(x, ">", y, ":", origstr, " -> ", ini[x][y])
                    if looplimit == 0:
                        raise RuntimeError(
                            f"read_settings : db代入上限回数を超えました: [{x}][{y}] {ini[x][y]}"
                        )
    return ini


def ini_read_and_replace(
    arglist: list[str] = sys.argv, configpath: str | None = None, verb: bool = False
) -> ConfigParser:
    if configpath is None:
        configpath = arglist[1].replace('"', "").strip()

    ini = ConfigParser(interpolation=ExtendedInterpolation())

    if os.path.isfile(configpath) is False:
        raise FileNotFoundError("config file " + configpath + "is not found")

    ini.read(configpath, "UTF-8")  # 旧：'./config/settings.ini'

    for key, value in list(ini.defaults().items()):
        ini["DEFAULT"][key] = value.replace('"', "").replace("'", "").strip()

    for section in ini.sections():
        for key, value in list(ini._sections[section].items()):
            if isinstance(value, str):
                ini[section][key] = value.replace('"', "").replace("'", "").strip()

    ini: ConfigParser = ini_replace_argnum(ini, arglist, verb=verb)

    if ini.has_section("FROM_DATABASE"):
        db_path: str = ini["FROM_DATABASE"]["db_path"]
        db_type: str = ini["FROM_DATABASE"]["db_type"]
        db_key: str = ini["FROM_DATABASE"]["db_key"]

        db_series = dbread_multiformat(db_path, db_type).loc[db_key]

        ini_replace_db(ini=ini, db_series=db_series, verb=verb)
    return ini


if __name__ == "__main__":
    print(
        sys.argv
    )  # 引数は[pythonファイル名,引数1,2,...]と続く。arglistにsys.argvを入れることを想定。
    ini = ini_read_and_replace(
        arglist=[
            "hogehoge.py",
            "実験データ管理リストsample.xlsx",
            "mov007",
            "(db:raw_v_adp)",
        ],
        configpath="test_config.ini",
        verb=True,
    )
    ini.write(
        open("test_config_out.ini", mode="w", encoding="UTF-8")
    )  # ConfigParserはwriteでiniを書き出せる。引数やDB読み込みで結局何の値が使われたのか書き出しておくと良い。
