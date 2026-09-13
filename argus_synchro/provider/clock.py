from __future__ import annotations

import datetime
import time
from abc import ABC, abstractmethod
from typing import final


class ClockProvider(ABC):
    """基準とするClockを決めるProvider"""

    @abstractmethod
    def get_time(self) -> float: ...

    @abstractmethod
    def reset_time(self) -> None: ...

    @abstractmethod
    def isframeexceeded(self) -> bool: ...


@final
class DummyClockProvider(ClockProvider):
    """校正モードFile入力用のDummy Clock"""

    def __init__(
        self,
        basetime: datetime.datetime,
        steptime: float,
        start_frame: int,
        end_frame: int,
    ) -> None:
        self._basetime: datetime.datetime = basetime
        self._steptime: float = steptime
        self._start_frame: int = start_frame
        self._timestamp_ix: int = start_frame
        self._end_frame = end_frame

    def get_time(self) -> float:
        ret_time = (
            self._basetime
            + datetime.timedelta(seconds=self._steptime * self._timestamp_ix)
        ).timestamp()
        self._timestamp_ix += 1
        return ret_time

    def reset_time(self) -> None:
        self._timestamp_ix = self._start_frame

    def isframeexceeded(self) -> bool:
        return self._end_frame < self._timestamp_ix


@final
class TimeClockProvider(ClockProvider):
    """校正モードSensor入力用のClock"""

    def __init__(self) -> None:
        pass

    def get_time(self) -> float:
        return time.time()

    def isframeexceeded(self) -> bool:
        return False

    def reset_time(self) -> None:
        pass


@final
class PerfCounterClockProvider(ClockProvider):
    """周辺監視モード用のClock"""

    def __init__(self) -> None:
        pass

    def get_time(self) -> float:
        return time.perf_counter()

    def isframeexceeded(self) -> bool:
        return False

    def reset_time(self) -> None:
        pass
