from __future__ import annotations

import multiprocessing
from abc import ABC, abstractmethod
from multiprocessing.sharedctypes import Synchronized
from multiprocessing.synchronize import Event
from typing import final


class ProcessActivator:
    """
    プロセス固有の有効状態管理クラス

    * プロセスの有効状態を制御する場合に使用する

    例:
    * 全プロセスに同じインスタンスを共有し、例外発生による全プロセスの終了に使用
    * プロセスごとにインスタンスを生成し、メインプロセスからのプロセス正常終了に使用
    """

    __slots__ = ("_enable", "_is_restart_required")

    def __init__(self) -> None:
        self._enable: Synchronized[bool] = multiprocessing.Value("b", True, lock=False)
        self._is_restart_required: Synchronized[bool] = multiprocessing.Value(
            "b", False, lock=False
        )

    @property
    def value(self) -> bool:
        return self._enable.value

    @property
    def is_restart_required(self) -> bool:
        return self._is_restart_required.value

    def enable(self) -> None:
        self._enable.value = True

    def disable(self) -> None:
        self._enable.value = False

    def require_restart(self) -> None:
        """restartフラグを立ててrestartを促す"""
        self._is_restart_required.value = True

    def restart_completed(self) -> None:
        """restart完了後に、restartフラグを下げて、
        次のrestartが行われないようにする。"""
        self._is_restart_required.value = False


class MessageSynchronizer(ABC):
    """
    プロセス間のメッセージ送信同期クラス

    * MessageFlowクラスが同期形式ごとにインスタンスを生成する
    """

    __slots__ = ("_enable",)

    def __init__(self, enable: ProcessActivator) -> None:
        self._enable: ProcessActivator = enable

    @abstractmethod
    def wait_consume(self) -> bool:
        """
        メッセージ読み込み待機

        * 無効状態が返却されたらcomplete_readは実行しない

        Returns:
            bool: 有効状態
        """
        ...

    @abstractmethod
    def completed_consume(self) -> None:
        """メッセージ読み込み完了"""
        ...

    @abstractmethod
    def wait_produce(self) -> bool:
        """
        メッセージ書き込み待機

        * 無効状態が返却されたらcomplete_readは実行しない

        Returns:
            bool: 有効状態
        """
        ...

    @abstractmethod
    def completed_produce(self) -> None:
        """メッセージ書き込み完了"""
        ...

    @abstractmethod
    def stop(self) -> None:
        """待機停止"""
        ...

    @abstractmethod
    def start(self) -> None:
        """stopで停止した後に再度起動する場合は実行する。"""
        ...

    @abstractmethod
    def require_restart(self) -> None:
        """再起動が必要な時に実行する。"""
        ...

    @abstractmethod
    def restart_completed(self) -> None:
        """再起動が完了した後に実行する。"""
        ...


@final
class ProduceSynchronizer(MessageSynchronizer):
    """
    同期データ書き込み

    * Produce、Consumeのどちらも同期する
    * Produceされていなければ、Consume待機する
    * Consumeされていなければ、Produceを待機する
    """

    __slots__ = ("_consume_ready", "_produce_ready")

    def __init__(self, enable: ProcessActivator) -> None:
        super().__init__(enable=enable)
        self._consume_ready: Event = multiprocessing.Event()
        self._produce_ready: Event = multiprocessing.Event()
        self._produce_ready.set()

    def wait_consume(self) -> bool:
        if not self._enable.value or self._enable.is_restart_required:
            return False
        self._consume_ready.wait()
        return self._enable.value and not self._enable.is_restart_required

    def completed_consume(self) -> None:
        if self._enable.value:
            self._consume_ready.clear()
            self._produce_ready.set()

    def wait_produce(self) -> bool:
        if not self._enable.value or self._enable.is_restart_required:
            return False
        self._produce_ready.wait()
        return self._enable.value and not self._enable.is_restart_required

    def completed_produce(self) -> None:
        if self._enable.value:
            self._produce_ready.clear()
            self._consume_ready.set()

    def stop(self) -> None:
        self._enable.disable()
        self._consume_ready.set()
        self._produce_ready.set()

    def start(self) -> None:
        self._enable.enable()
        self._consume_ready.clear()
        self._produce_ready.set()

    def require_restart(self) -> None:
        self._enable.require_restart()
        self._consume_ready.set()
        self._produce_ready.set()

    def restart_completed(self) -> None:
        self._enable.restart_completed()
        self._consume_ready.clear()
        self._produce_ready.set()


@final
class PassSynchronizer(MessageSynchronizer):
    """
    非同期データ読み込み

    * 同期なし
    * 同じデータを何度も読み込む可能性がある
    """

    def __init__(self, enable: ProcessActivator) -> None:
        super().__init__(enable)

    def wait_consume(self) -> bool:
        return self._enable.value and not self._enable.is_restart_required

    def completed_consume(self) -> None:
        pass

    def wait_produce(self) -> bool:
        return self._enable.value and not self._enable.is_restart_required

    def completed_produce(self) -> None:
        pass

    def stop(self) -> None:
        self._enable.disable()

    def start(self) -> None:
        self._enable.enable()

    def require_restart(self) -> None:
        self._enable.require_restart()

    def restart_completed(self) -> None:
        self._enable.restart_completed()
