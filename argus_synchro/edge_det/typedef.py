"""モジュール内で共通で用いる型alias"""

from typing import NamedTuple, TypeAlias

from argus_synchro.common.common import RangeF
from argus_synchro.common.error import ArgusSeverityDError

# lidar点群を表現する型
XYZTup: TypeAlias = tuple[float, float, float]

# lidar点群のxy座標を表現する型
XYTup: TypeAlias = tuple[float, float]

# 画像のxy座標を表現する型
PxPyTup: TypeAlias = tuple[int, int]

# 複数ラベルのエッジを表現する型
MultiEdge: TypeAlias = list[list[tuple[float, float, float]]]

# 複数ラベルのエッジのindexを表現する型
MultiLine: TypeAlias = list[list[tuple[int, int]]]


class Range3D(NamedTuple):
    """x, y, zの範囲を保持するtuple, Noneの場合は、その範囲は見ない想定

    :param NamedTuple: _description_
    :type NamedTuple: _type_
    """

    x_range: RangeF | None
    y_range: RangeF | None
    z_range: RangeF | None


class XYMinMax(NamedTuple):
    """x, yの最小, 最大が入ったtuple

    :param NamedTuple: _description_
    :type NamedTuple: _type_
    """

    min_x: float
    max_x: float
    min_y: float
    max_y: float


class EdgeDetLogicError(ArgusSeverityDError):
    """崖検出のプログラム的に想定した通りに動いていない場合に投げるエラー
    崖検出が動かないことに対する致命度に応じて継承するエラーは変更する

    :param ArgusSeverityDError: _description_
    :type ArgusSeverityDError: _type_
    """


class EdgeDetInvalidArgumenError(ArgusSeverityDError):
    """崖検出の設定ファイルが不正な値の場合に投げるエラー
    例えば、文字列系の設定ファイルの場合、任意の文字列を設定できるため、想定外の文字列を入力すると投げる

    :param ArgusSeverityDError: _description_
    :type ArgusSeverityDError: _type_
    """


# 崖検出で発生する例外をまとめるためのAlias, Result[T, EdgeDetError]で、関数内で発生する複数種類のエラーを扱えるようにするためのもの
EdgeDetError: TypeAlias = EdgeDetLogicError | EdgeDetInvalidArgumenError
