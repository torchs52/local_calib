import time
from collections import deque

from argus_synchro.config.app_config import AppConfig


class CraneStateEstimator:
    def __init__(
        self,
        app_config: AppConfig,
        lever_keys: list[str] = ("rf", "rb", "lf", "lb"),
    ) -> None:
        """
        CraneStateEstimatorは、4種類のクレーンレバー圧力入力から
        走行状態(Moving/Stopped/Unknown)を推定

        Parameters
        ----------
        window_sec:float
            スライディングウィンドウ長[秒]
        delta_db:float
            デッドバンド幅(圧力単位)。|p|<delta_dbは0とみなす
        p_on:float
            平均圧力がこの値を超えたらON(走行中)判定閾値
        p_off:float
            平均圧力がこの値を下回ったらOFF(停止)判定閾値
        lever_keys:list of str
            レバー識別キーの順序。データ入力リストのインデックスと対応
        """
        # パラメータ
        # AppConfigでパラメータを置き換え
        self.window_sec: float = app_config.StateEstimator.window_sec
        self.delta_db: float = app_config.StateEstimator.delta_db
        self.p_on: float = app_config.StateEstimator.p_on
        self.p_off: float = app_config.StateEstimator.p_off
        self.lever_keys: list[str] = lever_keys

        # バーごとの時刻付きバッファと状態フラグを初期化
        self.buffers: dict[str, deque[tuple[float, float]]] = {
            key: deque() for key in lever_keys
        }
        # 状態はTrue/False/None(None=Unknown)
        self.states: dict[str, bool | None] = dict.fromkeys(lever_keys)
        self.isMoving: bool | None = None

    def update(
        self,
        pressures: list[float] | None,
        timestamp: float | None = None,
    ) -> bool | None:
        """
        圧力リストを受け取り、内部状態を更新して走行中フラグを返す

        Parameters
        ----------
        pressures:list of float or None
            レバー圧力リスト[rf,rb,lf,lb]。空またはNoneの場合はUnknown
        timestamp:float,optional
            サンプル時刻[秒]。省略時はtime.time()を使用

        Returns
        -------
        bool | None
            True:Moving(走行中)
            False:Stopped(停止中)
            None:Unknown(データ不足または入力なし)
        """
        # 入力なしチェック
        if len(pressures) < len(self.lever_keys):
            for key in self.lever_keys:
                self.states[key] = None
            self.isMoving = None
            return self.isMoving

        now: float = timestamp if timestamp is not None else time.time()

        # 各レバーについて処理
        for key, p_raw in zip(self.lever_keys, pressures, strict=False):
            key: str
            p_raw: float

            buf: deque[tuple[float, float]] = self.buffers[key]

            # 1)デッドバンド処理
            p: float = 0.0 if abs(p_raw) < self.delta_db else p_raw

            # 2)スライディングウィンドウ更新
            buf.append((now, p))
            while buf and now - buf[0][0] > self.window_sec:
                buf.popleft()

            # データがない場合はUnknown
            if not buf:
                self.states[key] = None
                continue

            # 3)窓内平均圧力計算
            avg_p: float = sum(val for _, val in buf) / len(buf)

            # 4)ヒステリシス判定
            prev = self.states[key]
            if prev is None:
                # 初回データ時は閾値のみで判定
                if avg_p > self.p_on:
                    self.states[key] = True
                elif avg_p < self.p_off:
                    self.states[key] = False
                else:
                    self.states[key] = None
            # 通常のヒステリシス判定
            elif not prev and avg_p > self.p_on:
                self.states[key] = True
            elif prev and avg_p < self.p_off:
                self.states[key] = False
                # しきい値間は状態維持

        # 全体判定:いずれかTrueならTrue、すべてFalseならFalse、その他はNone
        if any(state is True for state in self.states.values()):
            self.isMoving = True
        elif all(state is False for state in self.states.values()):
            self.isMoving = False
        else:
            self.isMoving = None

        return self.isMoving

    def reset(self) -> None:
        """
        内部バッファと状態をすべてクリアし、再初期化
        """
        for key in self.lever_keys:
            self.buffers[key].clear()
            self.states[key] = None
        self.isMoving = None
