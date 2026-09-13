import multiprocessing
import time
from queue import Empty
from typing import TypeAlias, final

import numpy as np
from numpy.typing import NDArray

from argus_synchro.process.message import Message

# 型エイリアスの定義
FIFOData: TypeAlias = tuple[
    list[tuple[NDArray[np.uint8], int, float]],
    list[tuple[NDArray[np.float64], int, float]],
    tuple[float, float],
    int,
]


@final
class CalibFIFOMessage(Message[FIFOData]):
    """Use mp.Queue"""

    __slots__ = ("_queue",)

    def __init__(self) -> None:
        super().__init__()

        self._queue: multiprocessing.Queue[FIFOData] = multiprocessing.Queue()

    def write(
        self,
        value: FIFOData,
    ) -> None:
        self._queue.put(value)

    def read(
        self,
    ) -> FIFOData:
        return self._queue.get_nowait()

    def qsize(self) -> int:
        """queueのsizeを取得"""
        return self._queue.qsize()

    def _close(self) -> None:
        while True:
            try:
                self._queue.get_nowait()
            except Empty:
                break
        self._queue.close()

    def prepare_close(self, wait_timeout: float = 0.2) -> None:
        # self.loopEnable.value = False

        # self._logger.info(f"camera {ix}: closing")
        # if wait_for_worker:
        #     deadline = time.time() + wait_timeout
        #     while self.working_task_flags[ix] and time.time() < deadline:
        #         self._logger.info(f"camera {ix}: read end waiting")
        #         time.sleep(0.1)

        # ★ キューは Empty まで捨て切る（センチネル含む）
        dropped = 0
        deadline = time.time() + wait_timeout
        while time.time() < deadline:
            try:
                self._queue.get_nowait()
                dropped += 1
            except Empty:
                break
        # self._logger.info(f"camera {ix}: queue emptied (dropped={dropped})")
