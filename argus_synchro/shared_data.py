from __future__ import annotations

import math
import multiprocessing as mp
from collections.abc import Iterable
from multiprocessing.shared_memory import SharedMemory
from multiprocessing.sharedctypes import Synchronized
from typing import Any, Generic, TypeVar, final

import numpy as np
from numpy.typing import NDArray

from argus_synchro.core.closable import Closable

_type_to_typecode: dict[type[bool | int | float], str] = {
    bool: "b",
    int: "l",
    float: "d",
}

DT = TypeVar("DT", bound=np.generic)
ST = TypeVar("ST", bool, int, float)
SLOT_COUNT = 3


@final
class SharedArrayData(Closable, Generic[DT]):
    """
    配列用共有データクラス

    * SharedNumpyをベースとして、Genericで型の補完を強化している
    * メインプロセスでインスタンスを生成し、子プロセスの引数に入れる
    """

    def __init__(
        self,
        shape: Iterable[int],
        dtype: np.dtype[DT] | type[DT],
    ) -> None:
        super().__init__()
        self._shape: tuple[int, ...] = tuple(shape)
        self._dtype: np.dtype[DT] = (
            dtype if isinstance(dtype, np.dtype) else dtype().dtype
        )
        self._bytesize: int = math.prod(shape) * self._dtype.itemsize
        # NOTE: Python3.11環境ではmmapよりSharedMemoryの方が速かった
        self._shm = SharedMemory(create=True, size=self._bytesize)
        self._name: str = self._shm.name
        self._data: NDArray[DT] = np.ndarray(
            shape=self._shape,
            dtype=self._dtype,
            buffer=self._shm.buf,
        )

    def __getstate__(self) -> dict[str, Any]:
        """プロセス間で共有される時に、共有しないデータを削除する"""
        state: dict[str, Any] = self.__dict__.copy()
        del state["_data"]
        return state

    def __setstate__(self, d: dict[str, Any]) -> None:
        """プロセス間で共有されるときに削除したデータを、プロセスように生成する"""
        self.__dict__: dict[str, Any] = d
        self._data = np.ndarray(
            shape=self._shape,
            dtype=self._dtype,
            buffer=self._shm.buf,
        )

    def read(self) -> NDArray[DT]:
        return self._data.copy()

    def read_slice(self, slices: tuple[slice, ...]) -> NDArray[DT]:
        return self._data[slices].copy()

    def view(self) -> NDArray[DT]:
        return self._data

    def borrow(self) -> NDArray[DT]:
        out = self._data.view()
        out.setflags(write=False)
        return out

    def borrow_slice(self, slices: tuple[slice, ...]) -> NDArray[DT]:
        out = self._data[slices]
        out.setflags(write=False)
        return out

    def read_slot_value(self, slot: int) -> DT:
        return self._data[slot]

    def write(self, value: NDArray[DT]) -> None:
        np.copyto(self._data, value)

    def write_slice(self, value: NDArray[DT]) -> None:
        slices = tuple(slice(0, size) for size in value.shape)
        np.copyto(self._data[slices], value)

    def write_slot(self, slot: int, value: NDArray[DT] | DT | bool | float) -> None:
        target = self._data[slot]
        if isinstance(target, np.ndarray):
            np.copyto(target, value)
            return
        self._data[slot] = value

    def write_slot_slice(self, slot: int, value: NDArray[DT]) -> None:
        slices = (slot, *(slice(0, size) for size in value.shape))
        np.copyto(self._data[slices], value)

    def borrow_slot(self, slot: int) -> NDArray[DT]:
        out = self._data[slot]
        out.setflags(write=False)
        return out

    def borrow_slot_slice(
        self,
        slot: int,
        slices: tuple[slice, ...],
    ) -> NDArray[DT]:
        out = self._data[(slot, *slices)]
        out.setflags(write=False)
        return out

    def _close(self) -> None:
        try:
            self._shm.close()
            self._shm.unlink()
        except FileNotFoundError:
            pass


def create_shared_single_data(initial_value: ST) -> Synchronized[ST]:
    return mp.Value(_type_to_typecode[type(initial_value)], initial_value, lock=False)


@final
class SharedScalarSlotData(Closable, Generic[ST]):
    """固定3スロットの単一値共有データ"""

    __slots__ = ("_slots",)

    def __init__(self, value_type: type[ST], initial_value: ST) -> None:
        super().__init__()
        if value_type not in _type_to_typecode:
            raise TypeError(f"unsupported value type: {value_type}")
        self._slots: tuple[Synchronized[ST], ...] = tuple(
            mp.Value(_type_to_typecode[value_type], initial_value, lock=False)
            for _ in range(SLOT_COUNT)
        )

    def read_slot_value(self, slot: int) -> ST:
        return self._slots[slot].value

    def write_slot(self, slot: int, value: ST) -> None:
        self._slots[slot].value = value

    def fill(self, value: ST) -> None:
        for slot in self._slots:
            slot.value = value

    def _close(self) -> None:
        pass


@final
class SharedArraySlotData(Closable, Generic[DT]):
    """固定3スロットの配列共有データ"""

    __slots__ = ("_data",)

    def __init__(
        self,
        shape: Iterable[int],
        dtype: np.dtype[DT] | type[DT],
    ) -> None:
        super().__init__()
        self._data: SharedArrayData[DT] = SharedArrayData((SLOT_COUNT, *shape), dtype)

    def view(self) -> NDArray[DT]:
        return self._data.view()

    def write_slot(self, slot: int, value: NDArray[DT]) -> None:
        self._data.write_slot(slot, value)

    def write_slot_slice(self, slot: int, value: NDArray[DT]) -> None:
        self._data.write_slot_slice(slot, value)

    def borrow_slot(self, slot: int) -> NDArray[DT]:
        return self._data.borrow_slot(slot)

    def borrow_slot_slice(
        self,
        slot: int,
        slices: tuple[slice, ...],
    ) -> NDArray[DT]:
        return self._data.borrow_slot_slice(slot, slices)

    def _close(self) -> None:
        self._data.close()
