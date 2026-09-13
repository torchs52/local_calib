from __future__ import annotations
import collections.abc
import numpy
import numpy.typing
import typing
__all__: list[str] = ['bounding_box', 'dbscan', 'main_accum']
def bounding_box(mtx_pc: typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[m, n]"], unique_labels: typing.Annotated[numpy.typing.ArrayLike, numpy.int32, "[m, 1]"], labels: typing.Annotated[numpy.typing.ArrayLike, numpy.int32, "[m, 1]"]) -> tuple[typing.Annotated[numpy.typing.NDArray[numpy.float64], "[m, n]"], typing.Annotated[numpy.typing.NDArray[numpy.float64], "[m, n]"], typing.Annotated[numpy.typing.NDArray[numpy.float64], "[m, n]"]]:
    ...
def dbscan(matrix_pc: typing.Annotated[numpy.typing.NDArray[numpy.float64], "[m, n]", "flags.f_contiguous"], eps: typing.SupportsFloat | typing.SupportsIndex, min_samples: typing.SupportsInt | typing.SupportsIndex) -> typing.Annotated[numpy.typing.NDArray[numpy.int32], "[m, 1]"]:
    ...
def main_accum(xyz: typing.Annotated[numpy.typing.NDArray[numpy.float64], "[m, n]", "flags.f_contiguous"], debug_log: str, eps: typing.SupportsFloat | typing.SupportsIndex, min_samples: typing.SupportsInt | typing.SupportsIndex, logfunc: collections.abc.Callable[[typing.SupportsInt | typing.SupportsIndex, str], None]) -> tuple[typing.Annotated[numpy.typing.NDArray[numpy.float64], "[m, n]"], typing.Annotated[numpy.typing.NDArray[numpy.float64], "[m, n]"], typing.Annotated[numpy.typing.NDArray[numpy.float64], "[m, n]"], typing.Annotated[numpy.typing.NDArray[numpy.int32], "[m, 1]"], typing.Annotated[numpy.typing.NDArray[numpy.int32], "[m, 1]"]]:
    ...
