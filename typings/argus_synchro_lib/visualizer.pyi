from __future__ import annotations
import argus_synchro_lib.dataclass
import argus_synchro_lib.octotree
import argus_synchro_lib.ui_interface
import collections.abc
import numpy
import numpy.typing
import typing
__all__: list[str] = ['GodotUIVisualizer']
class GodotUIVisualizer:
    def __init__(self, ui_if_config: argus_synchro_lib.ui_interface.UIIFConf, s_frame: typing.SupportsInt | typing.SupportsIndex, rotation_radius: typing.SupportsFloat | typing.SupportsIndex, camera_num: typing.SupportsInt | typing.SupportsIndex, has_external_guard: bool, external_guard_offset: typing.SupportsFloat | typing.SupportsIndex, status_mmap_path: str, logfunc: collections.abc.Callable[[typing.SupportsInt | typing.SupportsIndex, str], None]) -> None:
        ...
    def close(self) -> None:
        ...
    def summary(self, arg0: typing.SupportsInt | typing.SupportsIndex, arg1: typing.Annotated[numpy.typing.NDArray[numpy.float64], "[m, n]", "flags.f_contiguous"], arg2: typing.Annotated[numpy.typing.NDArray[numpy.float32], "[m, n]", "flags.f_contiguous"], arg3: typing.Annotated[numpy.typing.NDArray[numpy.int32], "[m, 1]"], arg4: argus_synchro_lib.octotree.OctoTree, arg5: typing.SupportsInt | typing.SupportsIndex, arg6: collections.abc.Sequence[typing.Annotated[numpy.typing.ArrayLike, numpy.uint8]], arg7: collections.abc.Sequence[argus_synchro_lib.dataclass.CameraDetectionData], arg8: collections.abc.Sequence[argus_synchro_lib.dataclass.Camera], arg9: collections.abc.Mapping[typing.SupportsInt | typing.SupportsIndex | None, tuple[argus_synchro_lib.octotree.OctoNode, argus_synchro_lib.octotree.OctoNode, tuple[typing.SupportsFloat | typing.SupportsIndex, typing.SupportsFloat | typing.SupportsIndex, typing.SupportsFloat | typing.SupportsIndex], tuple[typing.SupportsFloat | typing.SupportsIndex, typing.SupportsFloat | typing.SupportsIndex, typing.SupportsFloat | typing.SupportsIndex], typing.SupportsFloat | typing.SupportsIndex, typing.SupportsFloat | typing.SupportsIndex | None]], arg10: collections.abc.Mapping[typing.SupportsInt | typing.SupportsIndex, argus_synchro_lib.octotree.NodeEntity], arg11: typing.SupportsInt | typing.SupportsIndex, arg12: typing.SupportsInt | typing.SupportsIndex, arg13: typing.SupportsInt | typing.SupportsIndex, arg14: bool, arg15: bool, arg16: bool, arg17: bool, arg18: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...
    def update(self, ui_if_config: argus_synchro_lib.ui_interface.UIIFConf, general_config: argus_synchro_lib.ui_interface.GeneralConf, logfunc: collections.abc.Callable[[typing.SupportsInt | typing.SupportsIndex, str], None]) -> None:
        ...
