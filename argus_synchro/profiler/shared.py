from __future__ import annotations

import contextlib
import mmap
import os
import pickle
from types import TracebackType
from typing import Self

from argus_synchro.profiler.prof_info import ProfInfo


def getname() -> str:
    return "e6c28f7d-53f6-4214-add4-8f3d691b8e0d.pkl"


class ProfSharedWriter:
    """プロファイリング共有情報の書き込みクラス"""

    def __enter__(self) -> Self:
        return self

    def __exit__(
        self,
        type_: type[BaseException] | None,
        value: BaseException | None,
        trace: TracebackType | None,
    ) -> None:
        with contextlib.suppress(OSError):
            os.remove(getname())

    def set(self, info: ProfInfo) -> None:
        """共有情報を設定する

        Args:
            info (ProfInfo): プロファイリング共有情報
        """
        with open(getname(), "w+b") as f:
            pickle.dump(info, f)


class ProfSharedReader:
    """共有情報の読み込みクラス"""

    __loaded = False
    __info = ProfInfo()

    @classmethod
    def __load(cls) -> None:
        if not os.path.isfile(getname()):
            cls.__loaded = True
            return
        with open(getname(), "r+b") as f, mmap.mmap(f.fileno(), 0) as mm:
            cls.__info: ProfInfo = pickle.load(mm)
            mm.seek(0)
            pickle.dump(cls.__info, mm)
        cls.__loaded = True

    @classmethod
    def get(cls) -> ProfInfo:
        """プロファイリング共有情報を取得する"""
        if not cls.__loaded:
            cls.__load()
        return cls.__info
