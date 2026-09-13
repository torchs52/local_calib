"""モジュール内で用いられている定数"""

from enum import IntEnum, auto


class ComplementMissingMethod(IntEnum):
    """欠損値の補完方法に関する列挙型

    :param IntEnum: _description_
    :type IntEnum: _type_
    """

    MEDIAN = auto()  # 中央値で補完
    MEAN = auto()  # 平均値で補完
    MIN = auto()  # 最小値で補完
    MAX = auto()  # 最大値で補完


class DediscretizeMethod(IntEnum):
    """離散値を実数値にどのように変換するかを管理するためのenum

    :param IntEnum: _description_
    :type IntEnum: _type_
    """

    MED = auto()  # 中心座標を代表値にする場合
    MIN = auto()  # 格子の各座標の最小値
    MAX = auto()  # 格子の各座標の最大値


class BinMethod(IntEnum):
    """二値化の方法を表現するためのenum

    :param IntEnum: _description_
    :type IntEnum: _type_
    """

    ABS = auto()  # 画像の値が一定値以上かどうかで二値を決める方法
    REL = auto()  # 画像の値を正規化して、その中で一定値以上かどうかで二値を決める方法


class BevCoord(IntEnum):
    """鳥観図の作り方に関するenum型

    :param IntEnum: _description_
    :type IntEnum: _type_
    """

    CARTESIAN = auto()  # デカルト座標で鳥観図を作る
    POLAR = auto()  # 極座標で鳥観図を作る


class EdgeFilterType(IntEnum):
    """微分フィルタとして想定しているものを表すenum型,
    EdgeFilter["SOBEL_X_FORWARD"]で文字列からプログラム上の意味に落としやすいし、
    紐づけができるし、許容している処理が明確になるので、
    一応設定しておく

    :param IntEnum: _description_
    :type IntEnum: _type_
    """

    SOBEL_X_FORWARD = auto()
    SOBEL_Y_FORWARD = auto()
    SOBEL_X_BACKWARD = auto()
    SOBEL_Y_BACKWARD = auto()
    SOBEL = auto()
    LAPLACIAN = auto()
    DoG = auto()


class ScanDirType(IntEnum):
    """走査する方向に関するenum型, 一応設定

    :param IntEnum: _description_
    :type IntEnum: _type_
    """

    PLUS_X = auto()  # Xが増える方向に走査
    MINUS_X = auto()  # Xが減る方向に走査
    PLUS_Y = auto()  # Yが増える方向に走査
    MINUS_Y = auto()  # Yが減る方向に走査


class FuncMode(IntEnum):
    """機能をどのモードで動作させるか規定する
    参照している部分はあるが、新しい崖検出で使わなくなった

    :param IntEnum: _description_
    :type IntEnum: _type_
    """

    PRODUCT = auto()
    DEBUG = auto()


# 検出領域の色, Open3d描画用
DETECT_MESH_COLOR = (0.5, 0.0, 0.0)

# settings.iniの地面の高さからどれだけオフセットさせるかを規定する定数
DEBUG_GROUND_HEIGHT_OFFSET = -0.2
