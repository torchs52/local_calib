from __future__ import annotations

import os
from multiprocessing.sharedctypes import Synchronized

from argus_synchro.common.app_logger import AppLogger, AppLoggerFactory
from argus_synchro.shared_data import create_shared_single_data

PROC_SPEED_NORMAL: int = 0
PROC_SPEED_SLOWDOWN_TREND: int = 1
PROC_SPEED_SLOW: int = 2
DEFAULT_POINT_CAPACITY: int = 40000
DEFAULT_MANY_POINTS_RATIO: float = 0.4
DEFAULT_FEW_POINTS_RATIO: float = 0.3
FORCE_ENABLED_ENV: str = "ARGUS_FORCE_REDUCED_LOAD_MODE"


class ReducedLoadMode:
    """負荷低減モードクラス"""

    PCD_ENABLE_THRESHOLD: int = 5
    THERMAL_ENABLE_THRESHOLD: int = 5
    DISABLE_THRESHOLD: int = 5

    def __init__(self) -> None:
        self._logger: AppLogger = AppLoggerFactory.from_type(self.__class__)
        self._force_enabled = os.getenv(FORCE_ENABLED_ENV) == "1"
        self._force_enabled_logged = False
        self.many_points_threshold = int(
            DEFAULT_POINT_CAPACITY * DEFAULT_MANY_POINTS_RATIO
        )
        self.few_points_threshold = int(
            DEFAULT_POINT_CAPACITY * DEFAULT_FEW_POINTS_RATIO
        )

        self._enabled: Synchronized[bool] = create_shared_single_data(False)
        self._proc_speed: Synchronized[int] = create_shared_single_data(
            PROC_SPEED_NORMAL
        )
        self._pcd_nums: Synchronized[int] = create_shared_single_data(0)
        self._is_thermal_throttling: Synchronized[bool] = create_shared_single_data(
            False
        )
        self._pcd_enable_counter: Synchronized[int] = create_shared_single_data(0)
        self._thermal_enable_counter: Synchronized[int] = create_shared_single_data(0)
        self._disable_counter: Synchronized[int] = create_shared_single_data(0)

    def log_register(self, app_logger_factory: AppLoggerFactory) -> None:
        self._app_logger_factory: AppLoggerFactory = app_logger_factory
        app_logger_factory.append_logger(self._logger)

    def configure(
        self,
        many_points_ratio: float,
        few_points_ratio: float,
        point_capacity: int = DEFAULT_POINT_CAPACITY,
    ) -> None:
        if not 0.0 <= few_points_ratio < many_points_ratio <= 1.0:
            raise ValueError(
                "reduced-load ratios must satisfy "
                "0 <= few_points_ratio < many_points_ratio <= 1"
            )
        if point_capacity <= 0:
            raise ValueError("point_capacity must be greater than zero")
        self.many_points_threshold = int(point_capacity * many_points_ratio)
        self.few_points_threshold = int(point_capacity * few_points_ratio)

    def update_proc_speed(self, proc_speed: int) -> None:
        self._proc_speed.value = proc_speed

    def update_pcd_nums(self, pcd_nums: int) -> None:
        self._pcd_nums.value = pcd_nums

    def update_is_thermal_throttling(self, is_thermal_throttling: bool) -> None:
        self._is_thermal_throttling.value = is_thermal_throttling

    @property
    def enabled(self) -> bool:
        """負荷低減モードの有効状態"""
        return self._enabled.value

    def update_state(self) -> None:
        """負荷低減モードの状態を更新するメソッド"""

        if self._force_enabled:
            self._enabled.value = True
            if not self._force_enabled_logged:
                self._logger.warning(
                    f"{FORCE_ENABLED_ENV}=1により負荷低減モードを強制有効化します。"
                )
                self._force_enabled_logged = True
            return

        # 負荷低減モードの開始条件の判定
        if not self._enabled.value and self._should_enable():
            self._enabled.value = True
            self._logger.warning("負荷低減モードを開始します。")
            return

        # 負荷低減モードの終了条件の判定
        if self._enabled.value and self._should_disable():
            self._enabled.value = False
            self._logger.warning("負荷低減モードを終了します。")
            return

    def _should_enable(self) -> bool:
        """負荷低減モードの開始条件を判定するメソッド"""
        # 処理速度が落ちているかの判定
        is_slow: bool = self._proc_speed.value == PROC_SPEED_SLOW
        # 点群数が多い環境に連続して存在するかの判定
        is_many_points: bool = self._pcd_nums.value > self.many_points_threshold
        # ECUがサーマルスロットリング状態かの判定
        is_thermal_throttling: bool = self._is_thermal_throttling.value

        # 条件①: 「処理速度が落ちている場合」かつ「異常に点群が多い環境に連続して存在する場合」
        if is_slow and is_many_points:
            self._pcd_enable_counter.value += 1
            self._logger.warning(
                f"入力点群数が多く処理速度が落ちている状態が{self._pcd_enable_counter.value}フレーム継続しています。点群数: {self._pcd_nums.value} 点"
            )
        else:
            self._pcd_enable_counter.value = 0

        # 条件②: 「処理速度が落ちている場合」かつ「ECUがサーマルスロットリング状態」
        if is_slow and is_thermal_throttling:
            self._thermal_enable_counter.value += 1
            self._logger.warning(
                f"サーマルスロットリング状態により処理速度が落ちている状態が、{self._thermal_enable_counter.value}フレーム継続しています。"
            )
        else:
            self._thermal_enable_counter.value = 0

        should_enable: bool = (
            self._pcd_enable_counter.value >= self.PCD_ENABLE_THRESHOLD
            or self._thermal_enable_counter.value >= self.THERMAL_ENABLE_THRESHOLD
        )
        if should_enable:
            self._pcd_enable_counter.value = 0
            self._thermal_enable_counter.value = 0

        return should_enable

    def _should_disable(self) -> bool:
        """負荷低減モードの終了条件を判定するメソッド"""
        # 処理速度が通常範囲かの判定
        is_not_slow: bool = self._proc_speed.value == PROC_SPEED_NORMAL
        # 点群数が多い環境に連続して存在しないかの判定
        is_not_many_points: bool = self._pcd_nums.value < self.few_points_threshold
        # ECUがサーマルスロットリング状態でないかの判定
        is_not_thermal_throttling: bool = not self._is_thermal_throttling.value

        if is_not_slow and is_not_many_points and is_not_thermal_throttling:
            self._disable_counter.value += 1
        else:
            self._disable_counter.value = 0

        should_disable: bool = self._disable_counter.value >= self.DISABLE_THRESHOLD
        if should_disable:
            self._disable_counter.value = 0

        return should_disable
