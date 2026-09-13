from __future__ import annotations

import enum
import time
from abc import ABC, abstractmethod
from multiprocessing.sharedctypes import Synchronized
from typing import Generic, Protocol, TypeVar, final, runtime_checkable

import numpy as np
from argus_synchro_lib.spsc_slot_controller import SpscSlotController

from argus_synchro.core.closable import Closable
from argus_synchro.process.synchronizer import (
    MessageSynchronizer,
    PassSynchronizer,
    ProcessActivator,
    ProduceSynchronizer,
)
from argus_synchro.profiler import ProfCategory, log_target
from argus_synchro.shared_data import (
    SLOT_COUNT,
    SharedArrayData,
    create_shared_single_data,
)

T = TypeVar("T")


class SyncType(enum.Enum):
    """メッセージ同期形式"""

    SYNC_FRAME = enum.auto()
    """1フレーム同期 (1 produce : 1 consume)"""
    LATEST = enum.auto()
    """最新のみ取得 (途中フレームは破棄)"""


@runtime_checkable
class HasQSize(Protocol):
    """qsize関数の存在判定用クラス"""

    def qsize(self) -> int: ...


class Message(Closable, ABC, Generic[T]):
    """プロセス間メッセージの基底クラス"""

    @abstractmethod
    def write(self, value: T) -> None:
        """書き込み処理"""
        ...

    @abstractmethod
    def read(self) -> T:
        """読み込み処理"""
        ...

    def set_sync_type(self, sync_type: SyncType) -> None:
        _ = sync_type

    def has_new_data(self, last_seq: int) -> bool:
        _ = last_seq
        return True

    def borrow(self, last_seq: int) -> tuple[T, int]:
        _ = last_seq
        return self.read(), -1

    def release(self) -> None:
        pass


class SlotMessage(Message[T], ABC):
    """
    スロット式共有メモリメッセージの基底

    * 各メッセージ実装は write_slot / borrow_slot を実装する
    * スロット公開/取得/解放は SpscSlotController が行う
    """

    __slots__ = (
        "_borrowed_slot",
        "_slot_controller",
        "_slot_controller_state",
        "_slot_count",
        "_sync_type",
    )

    def __init__(self) -> None:
        super().__init__()
        self._slot_count: int = SLOT_COUNT
        self._sync_type: SyncType = SyncType.LATEST
        self._borrowed_slot: int = -1
        self._slot_controller_state: SharedArrayData[np.int64] = SharedArrayData(
            (4,), np.int64
        )
        state = self._slot_controller_state.view()
        state[:] = np.array([-1, -1, -1, -1], dtype=np.int64)
        self._slot_controller = SpscSlotController(state, self._slot_count)

    @abstractmethod
    def write_slot(self, slot: int, value: T) -> None: ...

    @abstractmethod
    def borrow_slot(self, slot: int) -> T: ...

    def set_sync_type(self, sync_type: SyncType) -> None:
        self._sync_type = sync_type

    def write(self, value: T) -> None:
        if self._sync_type == SyncType.SYNC_FRAME:
            slot = self._slot_controller.reserve_write_slot_sync()
        else:
            slot = self._slot_controller.reserve_write_slot_latest()
        self.write_slot(slot, value)
        self._slot_controller.publish(slot)

    def read(self) -> T:
        value, _ = self.borrow(-1)
        return value

    def has_new_data(self, last_seq: int) -> bool:
        return self._slot_controller.published_seq() > last_seq

    def borrow(self, last_seq: int) -> tuple[T, int]:
        if self._sync_type == SyncType.SYNC_FRAME:
            slot, seq = self._slot_controller.acquire_read_slot_sync()
        else:
            slot, seq = self._slot_controller.acquire_read_slot_latest(last_seq)
        if slot < 0:
            raise RuntimeError("read slot is unavailable")
        self._borrowed_slot = slot
        return self.borrow_slot(slot), seq

    def release(self) -> None:
        if self._borrowed_slot < 0:
            return
        self._slot_controller.release_read_slot(self._borrowed_slot)
        self._borrowed_slot = -1

    def _close_slot_controller(self) -> None:
        self._slot_controller_state.close()

    @staticmethod
    def _collect_slots(obj: object) -> list[str]:
        names: list[str] = []
        for cls in type(obj).mro():
            slots = cls.__dict__.get("__slots__")
            if slots is None:
                continue
            if isinstance(slots, str):
                names.append(slots)
            else:
                names.extend(slots)
        return names

    def __getstate__(self) -> dict[str, object]:
        state: dict[str, object] = {}
        if hasattr(self, "__dict__"):
            state.update(self.__dict__)
        for name in self._collect_slots(self):
            if name in {"__dict__", "__weakref__", "_slot_controller"}:
                continue
            if hasattr(self, name):
                state[name] = getattr(self, name)
        return state

    def __setstate__(self, state: dict[str, object]) -> None:
        for key, value in state.items():
            setattr(self, key, value)
        if not hasattr(self, "_slot_count"):
            self._slot_count = SLOT_COUNT
        if not hasattr(self, "_borrowed_slot"):
            self._borrowed_slot = -1
        self._slot_controller = SpscSlotController(
            self._slot_controller_state.view(),
            self._slot_count,
        )


class BorrowedMessage(Generic[T]):
    __slots__ = ("_is_sync_frame", "_message", "_synchronizer", "data")

    def __init__(
        self,
        data: T,
        message: Message[T],
        synchronizer: MessageSynchronizer,
        *,
        is_sync_frame: bool,
    ) -> None:
        self.data = data
        self._message = message
        self._synchronizer = synchronizer
        self._is_sync_frame = is_sync_frame

    def __enter__(self) -> T:
        return self.data

    def __exit__(self, exc_type: object, exc: object, tb: object) -> bool:
        self._message.release()
        if self._is_sync_frame:
            self._synchronizer.completed_consume()
        return False


class Producer(ABC, Generic[T]):
    """
    データ書き込みを行うクラス
    """

    __slots__ = (
        "_is_created_producer",
        "_message",
        "_sync_type",
        "_synchronizer",
    )

    def __init__(
        self,
        message: Message[T],
        synchronizer: MessageSynchronizer,
        sync_type: SyncType,
        is_created_producer: Synchronized[bool],
    ) -> None:
        super().__init__()
        self._message: Message[T] = message
        self._synchronizer: MessageSynchronizer = synchronizer
        self._sync_type: SyncType = sync_type
        # NOTE __del__やcloseでのみ、この値をFalseに変更
        self._is_created_producer: Synchronized[bool] = is_created_producer

    def __del__(self) -> None:
        self._is_created_producer.value = False

    @log_target("Producer.wait", ProfCategory.Message)
    def wait(self) -> bool:
        return self._synchronizer.wait_produce()

    @log_target("Producer.produce", ProfCategory.Message)
    def produce(self, value: T) -> None:
        self._message.write(value)
        if self._sync_type == SyncType.SYNC_FRAME:
            self._synchronizer.completed_produce()

    def stop(self) -> None:
        self._synchronizer.stop()

    def start(self) -> None:
        self._synchronizer.start()

    def require_restart(self) -> None:
        self._synchronizer.require_restart()

    def restart_completed(self) -> None:
        self._synchronizer.restart_completed()


class Consumer(ABC, Generic[T]):
    """
    データ読み込みを行うクラス
    """

    __slots__ = (
        "_is_created_consumer",
        "_last_seq",
        "_message",
        "_sync_type",
        "_synchronizer",
    )

    def __init__(
        self,
        message: Message[T],
        synchronizer: MessageSynchronizer,
        sync_type: SyncType,
        is_created_consumer: Synchronized[bool],
    ) -> None:
        super().__init__()
        self._message: Message[T] = message
        self._synchronizer: MessageSynchronizer = synchronizer
        self._sync_type: SyncType = sync_type
        self._last_seq: int = -1
        # NOTE __del__やcloseでのみ、この値をFalseに変更
        self._is_created_consumer: Synchronized[bool] = is_created_consumer

    def __del__(self) -> None:
        self._is_created_consumer.value = False

    @log_target("Consumer.wait", ProfCategory.Message)
    def wait(self) -> bool:
        if self._sync_type == SyncType.SYNC_FRAME:
            return self._synchronizer.wait_consume()

        while self._synchronizer.wait_consume():
            if self._message.has_new_data(self._last_seq):
                return True
            time.sleep(0.0005)
        return False

    @log_target("Consumer.consume", ProfCategory.Message)
    def consume(self) -> BorrowedMessage[T]:
        data, seq = self._message.borrow(self._last_seq)
        if seq >= 0:
            self._last_seq = seq
        return BorrowedMessage(
            data=data,
            message=self._message,
            synchronizer=self._synchronizer,
            is_sync_frame=self._sync_type == SyncType.SYNC_FRAME,
        )

    def stop(self) -> None:
        self._synchronizer.stop()

    def start(self) -> None:
        self._synchronizer.start()

    def require_restart(self) -> None:
        self._synchronizer.require_restart()

    def restart_completed(self) -> None:
        self._synchronizer.restart_completed()


@final
class MessageFlow(Closable, ABC, Generic[T]):
    """
    プロセス間メッセージのフロー(Producer, Consumer)を生成するクラス
    """

    __slots__ = (
        "_is_created_consumer",
        "_is_created_producer",
        "_message",
        "_sync_type",
        "activator",
        "synchronizer",
    )

    def __init__(
        self,
        message: Message[T],
        activator: ProcessActivator,
        sync_type: SyncType = SyncType.LATEST,
    ) -> None:
        super().__init__()
        self._message: Message[T] = message
        self._sync_type: SyncType = sync_type
        self._message.set_sync_type(sync_type)

        # 特定のプロセスが終了したら、全てのプロセスを終了する
        self.activator: ProcessActivator = activator
        self.synchronizer: MessageSynchronizer
        if sync_type == SyncType.SYNC_FRAME:
            self.synchronizer = ProduceSynchronizer(self.activator)
        else:
            self.synchronizer = PassSynchronizer(self.activator)
        self._is_created_producer: Synchronized[bool] = create_shared_single_data(False)
        self._is_created_consumer: Synchronized[bool] = create_shared_single_data(False)

    def create_producer(self) -> Producer[T]:
        """
        Producer(データ生成側)を作成する
        """
        if self._is_created_producer.value:
            raise RuntimeError
        producer: Producer[T] = Producer[T](
            message=self._message,
            synchronizer=self.synchronizer,
            sync_type=self._sync_type,
            is_created_producer=self._is_created_producer,
        )
        self._is_created_producer.value = True
        return producer

    def create_consumer(self) -> Consumer[T]:
        """
        Consumer(データ消費側)を生成する
        """
        if self._is_created_consumer.value:
            raise RuntimeError
        consumer: Consumer[T] = Consumer[T](
            message=self._message,
            synchronizer=self.synchronizer,
            sync_type=self._sync_type,
            is_created_consumer=self._is_created_consumer,
        )
        self._is_created_consumer.value = True
        return consumer

    def qsize(self) -> int | None:
        """qsizeを返す\n
        messageにqsize関数がなかったら、Noneを返す"""
        if isinstance(self._message, HasQSize):
            return self._message.qsize()
        return None

    def _close(self) -> None:
        self.synchronizer.stop()
        self._message.close()
