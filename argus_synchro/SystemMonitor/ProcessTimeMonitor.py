import collections
from collections import deque
from typing import Literal, TypedDict


class FrameResult(TypedDict):
    status: Literal["slow", "trend"] | None
    elapsed: float
    avg_short: float
    avg_long: float
    delay_count_trend: int
    delay_count_slow: int


class ProcessTimeMonitor:
    def __init__(
        self,
        fast_threshold_ms: float,
        slow_threshold_ms: float,
        short_que_length: int = 10,
        long_que_length: int = 100,
        short_long_ratio: float = 1.2,
    ) -> None:
        self.fast_threshold: float = fast_threshold_ms
        self.slow_threshold: float = slow_threshold_ms
        self.short_long_ratio: float = short_long_ratio

        self.short_term: deque[float] = collections.deque(maxlen=short_que_length)
        self.long_term: deque[float] = collections.deque(maxlen=long_que_length)

        self.trend_counter: int = 0
        self.slow_counter: int = 0
        # いきなり状態遷移しないためのカウンタ(5フレーム連続で発生してから正式に検出等)
        self.delay_frame_limit: int = 5

    def record_frame(self, elapsed_ms: float) -> FrameResult:
        self.short_term.append(elapsed_ms)
        self.long_term.append(elapsed_ms)

        status: Literal["slow", "trend"] | None = None
        short_avg = sum(self.short_term) / len(self.short_term)
        long_avg = sum(self.long_term) / len(self.long_term)

        # 下限未満の処理時間:問題なし 全カウンタリセット
        if elapsed_ms < self.fast_threshold:
            self.trend_counter = 0
            self.slow_counter = 0

        # 上限時間を超えるスローダウン　カウントして連続性を確認
        elif elapsed_ms > self.slow_threshold:
            self.slow_counter += 1
            self.trend_counter = 0  # trendとslowは排他的

            if self.slow_counter >= self.delay_frame_limit:
                status = "slow"
                self.slow_counter -= 1  # 警告後に１減らす

        #  速度低下のトレンド発生(直近の短期平均が長期平均より遅い)
        elif (
            # 充分データは揃っている
            len(self.short_term) == self.short_term.maxlen
            and len(self.long_term) == self.long_term.maxlen
        ):
            if short_avg > long_avg * self.short_long_ratio:
                self.trend_counter += 1
                self.slow_counter = 0  # trendとslowは排他的
            else:
                self.trend_counter = 0

            if self.trend_counter >= self.delay_frame_limit:
                status = "trend"
                self.trend_counter -= 1

        return {
            "status": status,
            "elapsed": elapsed_ms,
            "avg_short": short_avg,
            "avg_long": long_avg,
            "delay_count_trend": self.trend_counter,
            "delay_count_slow": self.slow_counter,
        }
