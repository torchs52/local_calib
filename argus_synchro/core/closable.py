from __future__ import annotations

from abc import ABC, abstractmethod
from typing import Self, final


class IClosable(ABC):
    """
    closeの必要のあるクラスのインターフェース
    """

    @abstractmethod
    def close(self) -> None: ...


class Closable(IClosable, ABC):
    """
    closeの必要のあるクラスの抽象クラス

    * Disposeパターンをベースとしたcloseメソッドが実装済み
    * 抽象メソッド_closeにクローズ処理を実装する
    """

    def __init__(self) -> None:
        self._is_closed = False

    @abstractmethod
    def _close(self) -> None:
        """クローズ処理を実装する"""
        ...

    def add_to(self, closables: CompositeClosable) -> Self:
        """
        CompositeClosableによる一括クローズに追加する

        Sample:
        ```
        closables = CompositeClosable()
        data1 = Data().add_to(closables)
        data2 = Data().add_to(closables)
        data3 = Data().add_to(closable)
        closables.close()
        ```
        """
        closables.append(self)
        return self

    def close(self) -> None:
        """
        Disposeパターンをベースとして、一度クローズしたものは再クローズされないようにする
        """
        if self._is_closed:
            return
        self._close()
        self._is_closed = True


@final
class CompositeClosable(IClosable):
    """
    一括クローズを実施するクラス
    Compositeパターンをベースとしているため、クラス構造(木構造)のままcloseを定義できる
    """

    def __init__(self) -> None:
        self._is_closed = False
        self._items: list[IClosable] = []

    def append(self, value: IClosable) -> None:
        """クローズするリストにClosableを追加する
        CompositeClosableも指定可能
        """
        self._items.append(value)

    def close(self) -> None:
        """
        Disposeパターンをベースとして、一度クローズしたものは再クローズされないようにする
        """
        if self._is_closed:
            return
        for item in self._items:
            item.close()
        self._is_closed = True
