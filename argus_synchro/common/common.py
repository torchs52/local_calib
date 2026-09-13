from __future__ import annotations

import json
import re
from collections.abc import Callable
from dataclasses import dataclass
from typing import Any, Generic, TypeAlias, TypeVar, final

import cv2
import numpy as np
from argus_synchro_lib.octotree import OctoNode
from numpy.typing import NDArray

Point2i: TypeAlias = tuple[int, int]
"""2次元座標(x,y)"""

Point2f: TypeAlias = tuple[float, float]
"""2次元座標(x,y)"""

Point3i: TypeAlias = tuple[int, int, int]
"""3次元座標(x,y,z)"""

Point3f: TypeAlias = tuple[float, float, float]
"""3次元座標(x,y,z)"""

Size: TypeAlias = tuple[int, int]
"""サイズ(width, height)"""

SizeF: TypeAlias = tuple[float, float]
"""サイズ(width, height)"""

Range: TypeAlias = tuple[int, int]
"""1次元範囲(start, end)"""

RangeF: TypeAlias = tuple[float, float]
"""1次元範囲(start, end)"""

Point2iRange: TypeAlias = tuple[Point2i, Point2i]
"""2次元範囲(start, end)"""

Point2fRange: TypeAlias = tuple[Point2f, Point2f]
"""2次元範囲(start, end)"""

Point3iRange: TypeAlias = tuple[Point3i, Point3i]
"""3次元範囲(start, end)"""

Point3fRange: TypeAlias = tuple[Point3f, Point3f]
"""3次元範囲(start, end)"""

Matrix33i: TypeAlias = tuple[Point3i, Point3i, Point3i]
"""3x3行列の整数型"""

Matrix33f: TypeAlias = tuple[Point3f, Point3f, Point3f]
"""3x3行列の浮動小数点型"""

NDPoint2i: TypeAlias = NDArray[np.int32]
"""2次元座標のndarray型"""

NDPoint2f: TypeAlias = NDArray[np.float64]
"""2次元座標のndarray型"""

NDPoint3i: TypeAlias = NDArray[np.int32]
"""3次元座標のndarray型"""

NDPoint3f: TypeAlias = NDArray[np.float64]
"""3次元座標のndarray型"""

NDMatrix33i: TypeAlias = NDArray[np.int32]
"""3x3行列の整数型ndarray"""

NDMatrix33f: TypeAlias = NDArray[np.float64]
"""3x3行列の浮動小数点型ndarray"""


NDSeries: TypeAlias = NDArray[np.int32]
"""1次元ベクトルの整数型ndarray"""

NDSeriesF: TypeAlias = NDArray[np.float64]
"""1次元ベクトルの浮動小数点型ndarray"""

NDSeriesB: TypeAlias = NDArray[np.bool_]
"""1次元ベクトルのbool値のndarray"""

NDPoint2iArray: TypeAlias = NDArray[np.int32]
"""2次元座標のndarray型"""

NDPoint2fArray: TypeAlias = NDArray[np.float64]
"""2次元座標の浮動小数点型ndarray"""

NDPoint3fArray: TypeAlias = NDArray[np.float64]
"""3次元座標の浮動小数点型ndarray"""

Pixel: TypeAlias = Point2i
"""2次元座標(x,y)"""

NDImage: TypeAlias = NDArray[np.uint8]
"""画像データのndarray型"""

NDImageF: TypeAlias = NDArray[np.float64]
"""画像データでnp.float64で保持する場合の型, np.uint8だと不十分な場合に用いる"""

t_point_tuple: TypeAlias = tuple[float, float, float]

# float型のndarray型のtypealias, 頻出しているので作った
t_np_float: TypeAlias = NDArray[np.float64]

# uint型のndarray型のtypealias, 頻出しているので作った
t_np_uint: TypeAlias = NDArray[np.uint32]


# 衝突判定の結果としてScrutinizer上で扱う型, key側は良いが、value側はdataclassとかで扱ったほうが良いかも
t_py_col_res: TypeAlias = dict[
    int | None,
    tuple[OctoNode, OctoNode, t_point_tuple, t_point_tuple, float, float | None],
]


def put_text_with_background(
    img: cv2.typing.MatLike,
    text: str,
    org: Pixel,
    font: int,
    size: float,
    color: cv2.typing.Scalar,
    thickness: int,
    background_color: cv2.typing.Scalar,
) -> cv2.typing.MatLike:
    # 背景を描く
    (width, height), _ = cv2.getTextSize(text, font, size, thickness)
    top_left_point: Pixel = (org[0], org[1] - height)
    bottom_right_point: Pixel = (org[0] + width, org[1])
    img = cv2.rectangle(img, top_left_point, bottom_right_point, background_color, -1)

    # textを書く
    return cv2.putText(img, text, org, font, size, color, thickness)


def rotate_y(theta: float) -> cv2.typing.Matx33d:
    """y軸周りにthetaだけ回転する行列を作成する"""
    return np.array(
        [
            [np.cos(theta), 0, np.sin(theta)],
            [0, 1, 0],
            [-np.sin(theta), 0, np.cos(theta)],
        ],
    )


def rotate_z(theta: float) -> cv2.typing.Matx33d:
    """z軸周りにthetaだけ回転する行列を作成する"""
    return np.array(
        [
            [np.cos(theta), -np.sin(theta), 0],
            [np.sin(theta), np.cos(theta), 0],
            [0, 0, 1],
        ],
    )


# 旋回中心のオフセット
# _l2c_trans_vec = np.array(app_config.machine.offset_rotate_center)


def rotate_machine(
    points: NDPoint3fArray,
    yaw_angle: float,
    l2c_trans_vec: NDArray[np.float64],
) -> NDPoint3fArray:
    """
    旋回中心でpointsをyaw_angleだけ旋回させて、元に戻す
    """
    return (points - l2c_trans_vec) @ rotate_z(yaw_angle).T + l2c_trans_vec


def shift_tuple(
    _arr: tuple[float, float, float],
    shift_arr: list[float],
) -> NDArray[np.float64]:
    # _arr = list(_arr)
    np_arr: NDArray[np.float64] = np.array(_arr)
    np_shift_arr: NDArray[np.float64] = np.array(shift_arr)
    out_arr = np_arr + np_shift_arr
    # _arr = tuple(_arr)

    return out_arr


def shift_array(
    _arr: NDArray[np.float32],
    shift_arr: list[float],
    size: tuple[int, int],
) -> NDArray[np.float32]:
    offset_array: NDArray[np.float32] = np.tile(shift_arr, size)
    # _arr = list(_arr)
    out_arr = _arr + offset_array
    # _arr = tuple(_arr)

    return out_arr


JS_COMMENT_PATTERN = R"/\*[\s\S]*?\*/|//.*"


def _read_str(_text: str) -> dict[str, Any] | list[dict[str, Any]]:
    """jsonc文字列を辞書にする"""
    sub_text = re.sub(JS_COMMENT_PATTERN, "", _text)
    return json.loads(sub_text)


def read_jsonc(
    filepath: str,
    encoding: str = "utf-8",
) -> dict[str, Any] | list[dict[str, Any]]:
    """jsonc形式ファイルを辞書にする
    普通にjsonにコメントを含んでいる場合、json.loadは上手くいかなかったはずなので別途実装している
    """
    with open(filepath, encoding=encoding) as f:
        text = f.read()
    return _read_str(text)


"""使わないかもしれないですが、Result型も用意
関数の戻り値にResult[正常系の型, 異常系の型]みたいな型を作って、関数を呼ぶ側が異常系をどう扱うか判断できるようにしているような型になっている:
例: 以下の関数があった時に、関数の定義を見れば、ゼロ割で例外を投げることが分かって、関数を作る側は異常系の扱いに関与しなくてよくて、関数を使う側が異常系をどう扱うか決めればよくなる
def divide(numerator: float, denom: float) -> Result[float, ZeroDivideError]:
    if denom == 0:
        return Err(ZeroDivideError("denom is 0"))
    return Ok(numerator / denom)

res = divide(3, 0)
match res:
    case Ok(val):
        print(f"value is {val}")
    case Err(e):
        raise e
"""

T = TypeVar("T")
U = TypeVar("U")
E = TypeVar("E")
F = TypeVar("F")


@final
@dataclass(frozen=True, slots=True)
class Ok(Generic[T]):
    value: T


@final
@dataclass(frozen=True, slots=True)
class Err(Generic[E]):
    error: E


Result: TypeAlias = Ok[T] | Err[E]


def unwrap(result: Result[T, E]) -> T:
    """
    result型がOk型ならそれに内包される値を返して、Err型ならErr型がExceptionを継承していればそれに沿った例外を投げて、そうじゃなければRuntimeExceptionを投げる

    result1: Result = Ok(1)
    unwrap(result1) => 1を返す

    result1: Result = Err(1)
    unwrap(result1) => RuntimeErrorを投げる

    :param result: 説明
    :type result: Result[T, E]
    :return: 説明
    :rtype: T
    """
    if isinstance(result, Ok):
        return result.value
    raise RuntimeError(f"called unwrap on Err: {result.error}")


def map_(result: Result[T, E], f: Callable[[T], U]) -> Result[U, E]:
    """
    Okの値だけ変換する
    result1 : Result = Ok(1)
    map_(result1, lambda x: x * 2) => Ok(2)を返す
    result1: Result = Err(1)
    map_(result1, lambda x: x * 2) => Err(1)を返す

    :param result: 説明
    :type result: Result[T, E]
    :param f: 説明
    :type f: Callable[[T], U]
    :return: 説明
    :rtype: Result[U, E]
    """
    if isinstance(result, Ok):
        return Ok(f(result.value))
    return result


def map_err(result: Result[T, E], f: Callable[[E], F]) -> Result[T, F]:
    """
    Errの値だけ変換する
    result1 : Result = Ok(1)
    map_(result1, lambda x: x * 2) => Ok(1)を返す
    result1: Result = Err(1)
    map_(result1, lambda x: x * 2) => Err(2)を返す

    :param result: 説明
    :type result: Result[T, E]
    :param f: 説明
    :type f: Callable[[E], F]
    :return: 説明
    :rtype: Result[T, F]
    """
    if isinstance(result, Err):
        return Err(f(result.error))
    return result


def and_then(
    result: Result[T, E],
    f: Callable[[T], Result[U, E]],
) -> Result[U, E]:
    if isinstance(result, Ok):
        return f(result.value)
    return result


def is_in_interval(
    points: NDPoint3f | NDPoint3i,
    x_range: RangeF | None = None,
    y_range: RangeF | None = None,
    z_range: RangeF | None = None,
) -> NDSeriesB:
    """pointsの各x,y,z座標がx_range, y_range, z_rangeに入っているか判定して、boolのarrayを返す"""
    if len(points) == 0:
        raise ValueError("pointsが空を想定したコードになっていないのでエラー送出")

    def _in_range(points: NDSeries | NDSeriesF, range: RangeF) -> NDSeriesB:
        return (range[0] < points) & (points < range[1])

    ind = np.array([True] * len(points))
    if x_range is not None:
        ind = ind & _in_range(points[:, 0], x_range)

    if y_range is not None:
        ind = ind & _in_range(points[:, 1], y_range)

    if z_range is not None:
        ind = ind & _in_range(points[:, 2], z_range)
    return ind
