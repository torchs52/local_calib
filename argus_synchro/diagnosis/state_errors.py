from __future__ import annotations

import typing
from collections import deque
from typing import cast

import numpy as np
from numpy.typing import NDArray

import argus_synchro.diagnosis.error_config as err_conf
from argus_synchro.diagnosis.error_diagnosis import (
    DiagnosisRuntimePolicy,
    StateErrorDiagnosisA,
    StateErrorDiagnosisB,
    StateErrorDiagnosisC,
)

if typing.TYPE_CHECKING:
    from typing import Literal


def _sensor_index(args: tuple[object, ...]) -> int:
    if len(args) != 1 or not isinstance(args[0], int):
        raise ValueError("args must be (index,)")
    return args[0]


class LidarNConnectionErrorDiagnosis(StateErrorDiagnosisA):
    """LIDARX_CONNECTION_ERROR: LidarX接続エラー"""

    def __init__(self, runtime_policy: DiagnosisRuntimePolicy | None = None) -> None:
        super().__init__(runtime_policy)
        self.param: err_conf.LidarNConnectionErrorParameters
        self.clear()

    def clear(self) -> None:
        self._previous_heartbeat: float | None = None
        self._recovery_prev_timestamp = -1.0
        self._fail_safe_prev_timestamp = -1.0
        self._error_recovery_start_time: float | None = None
        self._fail_safe_recovery_start_time: float | None = None

    def update(self, error_config: err_conf.ErrorConfig) -> None:
        self.param = error_config.lidar_n_connection_error
        self.is_enabled = self.param.is_enabled

    def _parse_args(self, *args: object) -> tuple[float, float]:
        if len(args) != 2 or not all(isinstance(value, float) for value in args):
            raise ValueError("args must be (now, last_heartbeat) as float")
        return args[0], args[1]  # type: ignore[return-value]

    def detect_error(self, *args: object) -> bool:
        now, last_heartbeat = self._parse_args(*args)
        # メンテナンスモード中は対象機器の停止を新規検出しない。
        if self.is_suppressed_in_maintenance:
            return False
        if (
            self._previous_heartbeat is None
            or self._previous_heartbeat < 0.0
            or last_heartbeat < 0.0
        ):
            self._previous_heartbeat = last_heartbeat
            return False
        is_error = (
            last_heartbeat - self._previous_heartbeat >= self.param.error_threshold_sec
            or now - last_heartbeat >= self.param.error_threshold_sec
        )
        self._previous_heartbeat = last_heartbeat
        if is_error:
            self.is_error.value = True
            self.is_fail_safe.value = True
            return True
        return False

    def detect_recovery_error(self, *args: object) -> bool:
        now, last_heartbeat = self._parse_args(*args)
        if last_heartbeat < 0.0:
            return False
        interval = last_heartbeat - self._recovery_prev_timestamp
        self._recovery_prev_timestamp = last_heartbeat
        if interval < 0.0:
            return False
        if (
            interval > self.param.recovery_receive_interval_sec
            or now - last_heartbeat > self.param.recovery_receive_interval_sec
        ):
            self._error_recovery_start_time = None
            return False
        if self._error_recovery_start_time is None:
            self._error_recovery_start_time = last_heartbeat
        elif (
            last_heartbeat - self._error_recovery_start_time
            >= self.param.error_recovery_confirm_duration_sec
        ):
            self._error_recovery_start_time = None
            self.is_error.value = False
            return True
        return False

    def detect_recovery_fail_safe(self, *args: object) -> bool:
        now, last_heartbeat = self._parse_args(*args)
        if last_heartbeat < 0.0:
            return False
        interval = last_heartbeat - self._fail_safe_prev_timestamp
        self._fail_safe_prev_timestamp = last_heartbeat
        if interval < 0.0:
            return False
        if (
            interval > self.param.recovery_receive_interval_sec
            or now - last_heartbeat > self.param.recovery_receive_interval_sec
        ):
            self._fail_safe_recovery_start_time = None
            return False
        if self._fail_safe_recovery_start_time is None:
            self._fail_safe_recovery_start_time = last_heartbeat
        elif (
            last_heartbeat - self._fail_safe_recovery_start_time
            >= self.param.failsafe_recovery_confirm_duration_sec
        ):
            self._fail_safe_recovery_start_time = None
            self.is_fail_safe.value = False
            return True
        return False

    def _error_log_output(self, err_idx: int, *args: object) -> None:
        index = _sensor_index(args)
        self._logger.error(
            self.get_error_no(err_idx) + f": Lidar[{index}] connection error detected."
        )

    def _recover_log_output(self, err_idx: int, *args: object) -> None:
        index = _sensor_index(args)
        self._logger.info(
            self.get_error_no(err_idx) + f": Lidar[{index}] connection error recovered."
        )

    def _fail_safe_recover_log_output(self, err_idx: int, *args: object) -> None:
        index = _sensor_index(args)
        self._logger.info(
            self.get_error_no(err_idx)
            + f": Lidar[{index}] connection error recovered from failsafe."
        )


class CameraNConnectionErrorDiagnosis(StateErrorDiagnosisB):
    """CAMERAN_CONNECTION_ERROR: カメラN接続エラー"""

    def __init__(self) -> None:
        super().__init__()
        self._recovery_start_time: float | None = None
        self._previous_heartbeat: float | None = None
        self._recovery_prev_timestamp = 0.0
        self._fail_safe_prev_timestamp = 0.0
        self._error_recovery_start_time: float | None = None
        self._fail_safe_recovery_start_time: float | None = None
        self.param: err_conf.CameraNConnectionErrorParameters

    def _parse_args(self, *args: object) -> tuple[float, float]:
        """
        args[0] (float) : 現在時刻now
        args[1] (float) : 最後の取得時刻last_heartbeat
        """
        if len(args) != 2:
            raise ValueError("args must be (now, last_heartbeat)")
        now = args[0]
        last_heartbeat = args[1]

        if not isinstance(now, float) or not isinstance(last_heartbeat, float):
            raise ValueError("args must be float, float")

        return float(now), float(last_heartbeat)

    def clear(self) -> None:
        """
        診断用のインスタンス変数を初期化する
        """
        self._recovery_start_time = None
        self._previous_heartbeat = None
        self._recovery_prev_timestamp = 0.0
        self._fail_safe_prev_timestamp = 0.0
        self._error_recovery_start_time = None
        self._fail_safe_recovery_start_time = None

    def update(self, err_conf: err_conf.ErrorConfig) -> None:
        self.param: err_conf.CameraNConnectionErrorParameters = (
            err_conf.camera_n_connection_error
        )
        self.is_enabled = self.param.is_enabled

    def detect_error(self, *args: object) -> bool:
        """
        最後の取得時刻が、現在から規定秒より前の場合にエラー(True)。
        """
        now, last_heartbeat = self._parse_args(*args)
        if self._previous_heartbeat is None or last_heartbeat < 0.0:
            self._previous_heartbeat = last_heartbeat
            return False

        # 前回のハートビートから規定秒以上経過している、または最後のハートビートから規定秒以上経過している場合にエラーとする
        is_error = (
            last_heartbeat - self._previous_heartbeat >= self.param.error_threshold_sec
            or now - last_heartbeat >= self.param.error_threshold_sec
        )

        self._previous_heartbeat = last_heartbeat

        if is_error:
            self.is_error.value = True
            self.is_fail_safe.value = True
            return True
        return False

    def detect_recovery_error(self, *args: object) -> bool:
        now, last_heartbeat = self._parse_args(*args)
        if last_heartbeat < 0.0:
            return False

        interval = last_heartbeat - self._recovery_prev_timestamp
        self._recovery_prev_timestamp = last_heartbeat
        if (
            interval > self.param.recovery_receive_interval_sec
            or now - last_heartbeat > self.param.recovery_receive_interval_sec
        ):
            self._error_recovery_start_time = None
            return False

        if self._error_recovery_start_time is None:
            self._error_recovery_start_time = last_heartbeat
        elif (
            last_heartbeat - self._error_recovery_start_time
            >= self.param.error_recovery_confirm_duration_sec
        ):
            self._error_recovery_start_time = None
            self.is_error.value = False
            return True
        return False

    def detect_recovery_fail_safe(self, *args: object) -> bool:
        now, last_heartbeat = self._parse_args(*args)
        if last_heartbeat < 0.0:
            return False

        interval = last_heartbeat - self._fail_safe_prev_timestamp
        self._fail_safe_prev_timestamp = last_heartbeat
        if (
            interval > self.param.recovery_receive_interval_sec
            or now - last_heartbeat > self.param.recovery_receive_interval_sec
        ):
            self._fail_safe_recovery_start_time = None
            return False

        if self._fail_safe_recovery_start_time is None:
            self._fail_safe_recovery_start_time = last_heartbeat
        elif (
            last_heartbeat - self._fail_safe_recovery_start_time
            >= self.param.failsafe_recovery_confirm_duration_sec
        ):
            self._fail_safe_recovery_start_time = None
            self.is_fail_safe.value = False
            return True
        return False

    def _error_log_output(self, err_idx: int, *args: object) -> None:
        if len(args) != 1:
            raise ValueError("args must be (index,)")
        if isinstance(args[0], int):
            index: int = args[0]
        else:
            raise ValueError("args[0] must be int")
        self._logger.warning(
            self.get_error_no(err_idx)
            + f": Camera {index} is not connected. Please check the connection status."
        )

    def _recover_log_output(self, err_idx: int, *args: object) -> None:
        if len(args) != 1:
            raise ValueError("args must be (index,)")
        if isinstance(args[0], int):
            index: int = args[0]
        else:
            raise ValueError("args[0] must be int")
        self._logger.warning(
            self.get_error_no(err_idx)
            + f": CAM[{index}] connection heartbeat has recovered."
        )

    def _fail_safe_recover_log_output(self, err_idx: int, *args: object) -> None:
        if len(args) != 1:
            raise ValueError("args must be (index,)")
        if isinstance(args[0], int):
            index: int = args[0]
        else:
            raise ValueError("args[0] must be int")
        self._logger.warning(
            self.get_error_no(err_idx)
            + f": CAM[{index}] connection heartbeat has recovered from failsafe."
        )


class CanConnectionErrorDiagnosis(StateErrorDiagnosisA):
    """CAN_CONNECTION_ERROR: CAN接続エラー"""

    def __init__(self, runtime_policy: DiagnosisRuntimePolicy | None = None) -> None:
        super().__init__(runtime_policy)
        self._previous_heartbeat: float | None = None
        self._recovery_prev_timestamp = 0.0
        self._fail_safe_prev_timestamp = 0.0
        self._error_recovery_start_time: float | None = None
        self._fail_safe_recovery_start_time: float | None = None
        self.param: err_conf.CanConnectionErrorParameters

    def clear(self) -> None:
        self._previous_heartbeat = None
        self._recovery_prev_timestamp = 0.0
        self._fail_safe_prev_timestamp = 0.0
        self._error_recovery_start_time = None
        self._fail_safe_recovery_start_time = None

    def update(self, err_conf: err_conf.ErrorConfig) -> None:
        self.param = err_conf.can_connection_error
        self.is_enabled = self.param.is_enabled

    def _parse_args(self, *args: object) -> tuple[float, float]:
        """
        args[0] (float) : 現在時刻now
        args[1] (float) : 最後の取得時刻last_heartbeat
        """
        if len(args) != 2:
            raise ValueError("args must be (now, last_heartbeat)")

        now = args[0]
        last_heartbeat = args[1]

        if not isinstance(now, float) or not isinstance(last_heartbeat, float):
            raise ValueError("args must be float, float")

        return float(now), float(last_heartbeat)

    def detect_error(self, *args: object) -> bool:
        """
        最後の取得時刻が、現在から規定秒より前の場合にエラー(True)。
        """
        now, last_heartbeat = self._parse_args(*args)
        # メンテナンスモード中は対象機器の停止を新規検出しない。
        if self.is_suppressed_in_maintenance:
            return False
        if self._previous_heartbeat is None or last_heartbeat < 0.0:
            self._previous_heartbeat = last_heartbeat
            return False

        # 前回のハートビートから規定秒以上経過している、または最後のハートビートから規定秒以上経過している場合にエラーとする
        is_error = (
            last_heartbeat - self._previous_heartbeat >= self.param.error_threshold_sec
            or now - last_heartbeat >= self.param.error_threshold_sec
        )

        self._previous_heartbeat = last_heartbeat

        if is_error:
            self.is_error.value = True
            self.is_fail_safe.value = True
            self.is_idle.value = True
            return True
        return False

    def detect_recovery_error(self, *args: object) -> bool:
        now, last_heartbeat = self._parse_args(*args)
        if last_heartbeat < 0.0:
            return False

        interval = last_heartbeat - self._recovery_prev_timestamp
        self._recovery_prev_timestamp = last_heartbeat
        if (
            interval > self.param.recovery_receive_interval_sec
            or now - last_heartbeat > self.param.recovery_receive_interval_sec
        ):
            self._error_recovery_start_time = None
            return False

        if self._error_recovery_start_time is None:
            self._error_recovery_start_time = last_heartbeat
        elif (
            last_heartbeat - self._error_recovery_start_time
            >= self.param.error_recovery_confirm_duration_sec
        ):
            self._error_recovery_start_time = None
            self.is_error.value = False
            return True
        return False

    def detect_recovery_fail_safe(self, *args: object) -> bool:
        now, last_heartbeat = self._parse_args(*args)
        if last_heartbeat < 0.0:
            return False

        interval = last_heartbeat - self._fail_safe_prev_timestamp
        self._fail_safe_prev_timestamp = last_heartbeat
        if (
            interval > self.param.recovery_receive_interval_sec
            or now - last_heartbeat > self.param.recovery_receive_interval_sec
        ):
            self._fail_safe_recovery_start_time = None
            return False

        if self._fail_safe_recovery_start_time is None:
            self._fail_safe_recovery_start_time = last_heartbeat
        elif (
            last_heartbeat - self._fail_safe_recovery_start_time
            >= self.param.failsafe_recovery_confirm_duration_sec
        ):
            self._fail_safe_recovery_start_time = None
            self.is_fail_safe.value = False
            self.is_idle.value = False
            return True
        return False

    def _error_log_output(self, err_idx: int, *args: object) -> None:
        self._logger.error(
            self.get_error_no(err_idx)
            + ": CAN is not connected. Please check the connection status."
        )

    def _recover_log_output(self, err_idx: int, *args: object) -> None:
        self._logger.warning(
            self.get_error_no(err_idx) + ": CAN heartbeat has recovered."
        )

    def _fail_safe_recover_log_output(self, err_idx: int, *args: object) -> None:
        self._logger.warning(
            self.get_error_no(err_idx) + ": CAN heartbeat has recovered from failsafe."
        )


class LidarNCommQualityDegradedDiagnosis(StateErrorDiagnosisC):
    """LIDARN_COMM_QUALITY_DEGRADED: LidarN通信品質低下"""

    def __init__(self) -> None:
        super().__init__()
        self._error_start_time: float | None = None
        self._recovery_start_time: float | None = None
        self._fail_safe_recovery_start_time: float | None = None
        self.param: err_conf.LidarCommQualityDegradedParameters

    def _parse_args(self, *args: object) -> tuple[float, float]:
        if len(args) != 2:
            raise ValueError("args must be (now, last_quality_degraded)")
        now, last_degraded = args
        if not isinstance(now, float) or not isinstance(last_degraded, float):
            raise ValueError("args must be float, float")
        return now, last_degraded

    def update(self, error_config: err_conf.ErrorConfig) -> None:
        self.param = error_config.lidar_n_comm_quality_degraded
        self.is_enabled = self.param.is_enabled

    def _is_degraded(self, now: float, last_degraded: float) -> bool:
        return (
            last_degraded > 0.0
            and now - last_degraded < self.param.recent_event_threshold_sec
        )

    def detect_error(self, *args: object) -> bool:
        now, last_degraded = self._parse_args(*args)
        if not self._is_degraded(now, last_degraded):
            self._error_start_time = None
            return False
        if self._error_start_time is None:
            self._error_start_time = now
        if now - self._error_start_time >= self.param.error_confirm_duration_sec:
            self.is_error.value = True
            self.is_fail_safe.value = True
            return True
        return False

    def detect_recovery_error(self, *args: object) -> bool:
        now, last_degraded = self._parse_args(*args)
        if self._is_degraded(now, last_degraded):
            self._recovery_start_time = None
            return False
        if self._recovery_start_time is None:
            self._recovery_start_time = now
        if now - self._recovery_start_time >= self.param.recovery_confirm_duration_sec:
            self._recovery_start_time = None
            self.is_error.value = False
            return True
        return False

    def detect_recovery_fail_safe(self, *args: object) -> bool:
        now, last_degraded = self._parse_args(*args)
        if self._is_degraded(now, last_degraded):
            self._fail_safe_recovery_start_time = None
            return False
        if self._fail_safe_recovery_start_time is None:
            self._fail_safe_recovery_start_time = now
        if (
            now - self._fail_safe_recovery_start_time
            >= self.param.recovery_confirm_duration_sec
        ):
            self._fail_safe_recovery_start_time = None
            self.is_fail_safe.value = False
            return True
        return False

    def _error_log_output(self, err_idx: int, *args: object) -> None:
        index = _sensor_index(args)
        self._logger.info(
            self.get_error_no(err_idx)
            + f": Lidar[{index}] comm quality degraded detected."
        )

    def _recover_log_output(self, err_idx: int, *args: object) -> None:
        index = _sensor_index(args)
        self._logger.info(
            self.get_error_no(err_idx)
            + f": Lidar[{index}] comm quality degraded recovered."
        )


class CameraNCommQualityDegradedDiagnosis(StateErrorDiagnosisC):
    """CAMERAN_COMM_QUALITY_DEGRADED: カメラN通信品質低下"""

    def __init__(self) -> None:
        super().__init__()
        self._error_recovery_count: int = 0
        self._fail_safe_recovery_count: int = 0
        self._initial_diagnosis_time: float | None = None

    def _parse_args(self, *args: object) -> tuple[float, float | None, int]:
        """
        args[0] (float) : 現在時刻now
        args[1] (float) : 最後の取得時刻last_heartbeat
        args[2] (int) : 連続取得失敗回数read_failure_cou以下
        """
        if len(args) != 3:
            raise ValueError("args must be (now, last_heartbeat, read_failure_count)")
        now = args[0]
        timestamp = args[1]
        read_failure_count = args[2]

        if (
            not isinstance(now, float)
            or (timestamp is not None and not isinstance(timestamp, float))
            or not isinstance(read_failure_count, int)
        ):
            raise ValueError("args must be float, float, int")

        return float(now), timestamp, int(read_failure_count)

    def update(self, err_conf: err_conf.ErrorConfig) -> None:
        self.param: err_conf.CameraCommQualityDegradedParameters = (
            err_conf.camera_n_comm_quality_degraded
        )
        self.is_enabled = self.param.is_enabled

    def detect_error(self, *args: object) -> bool:
        now, timestamp, read_failure_count = self._parse_args(*args)
        if self._initial_diagnosis_time is None:
            self._initial_diagnosis_time = now

        if timestamp is None:
            timestamp = self._initial_diagnosis_time

        if (
            now - timestamp >= self.param.error_threshold_sec
            or read_failure_count >= self.param.error_threshold_count
        ):
            self.is_error.value = True
            self.is_fail_safe.value = True
            return True
        return False

    def detect_recovery_error(self, *args: object) -> bool:
        now, timestamp, failed_count = self._parse_args(*args)
        if timestamp is None:
            return False
        recovery_receive_interval = (
            now - timestamp <= self.param.error_recovery_receive_interval_sec
        )

        if failed_count == 0:
            self._error_recovery_count += 1
        else:
            self._error_recovery_count = 0
            return False
        recovery_threshold_count = (
            self._error_recovery_count >= self.param.error_recovery_threshold_count
        )

        if recovery_receive_interval and recovery_threshold_count:
            self._error_recovery_count = 0
            self.is_error.value = False
            return True

        return False

    def detect_recovery_fail_safe(self, *args: object) -> bool:
        now, timestamp, failed_count = self._parse_args(*args)
        if timestamp is None:
            return False
        recovery_receive_interval = (
            now - timestamp <= self.param.fail_safe_recovery_receive_interval_sec
        )

        if failed_count == 0:
            self._fail_safe_recovery_count += 1
        else:
            self._fail_safe_recovery_count = 0
            return False
        fail_safe_recovery_threshold_count = (
            self._fail_safe_recovery_count
            >= self.param.fail_safe_recovery_threshold_count
        )

        if recovery_receive_interval and fail_safe_recovery_threshold_count:
            self._fail_safe_recovery_count = 0
            self.is_fail_safe.value = False
            return True

        return False

    def _error_log_output(self, err_idx: int, *args: object) -> None:
        if len(args) != 1:
            raise ValueError("args must be (index,)")
        if isinstance(args[0], int):
            index: int = args[0]
        else:
            raise ValueError("args[0] must be int")
        self._logger.warning(
            self.get_error_no(err_idx)
            + f": Camera {index} communication quality is degraded. If this occurs frequently, restarting may help."
        )

    def _recover_log_output(self, err_idx: int, *args: object) -> None:
        if len(args) != 1:
            raise ValueError("args must be (index,)")
        if isinstance(args[0], int):
            index: int = args[0]
        else:
            raise ValueError("args[0] must be int")
        self._logger.warning(
            self.get_error_no(err_idx)
            + f": Camera {index} communication quality has recovered."
        )

    def _fail_safe_recover_log_output(self, err_idx: int, *args: object) -> None:
        if len(args) != 1:
            raise ValueError("args must be (index,)")
        if isinstance(args[0], int):
            index: int = args[0]
        else:
            raise ValueError("args[0] must be int")
        self._logger.warning(
            self.get_error_no(err_idx)
            + f": Camera {index} communication quality has recovered from fail-safe mode."
        )


class LidarNCommQualityErrorDiagnosis(StateErrorDiagnosisB):
    """LIDAR_N_COMM_QUALITY_ERROR: LidarN通信品質エラー"""

    def __init__(self) -> None:
        super().__init__()
        self._error_start_time: float | None = None
        self._recovery_start_time: float | None = None
        self._fail_safe_recovery_start_time: float | None = None
        self.param: err_conf.LidarCommQualityErrorParameters

    def _parse_args(self, *args: object) -> tuple[float, bool]:
        if len(args) != 2:
            raise ValueError("args must be (now, is_quality_degraded)")
        now, is_quality_degraded = args
        if not isinstance(now, float) or not isinstance(is_quality_degraded, bool):
            raise ValueError("args must be float, bool")
        return now, is_quality_degraded

    def update(self, error_config: err_conf.ErrorConfig) -> None:
        self.param = error_config.lidar_n_comm_quality_error
        self.is_enabled = self.param.is_enabled

    def detect_error(self, *args: object) -> bool:
        now, is_quality_degraded = self._parse_args(*args)
        if not is_quality_degraded:
            self._error_start_time = None
            return False
        if self._error_start_time is None:
            self._error_start_time = now
        if now - self._error_start_time >= self.param.error_confirm_duration_sec:
            self.is_error.value = True
            self.is_fail_safe.value = True
            return True
        return False

    def detect_recovery_error(self, *args: object) -> bool:
        now, is_quality_degraded = self._parse_args(*args)
        if is_quality_degraded:
            self._recovery_start_time = None
            return False
        if self._recovery_start_time is None:
            self._recovery_start_time = now
        if now - self._recovery_start_time >= self.param.recovery_confirm_duration_sec:
            self._recovery_start_time = None
            self.is_error.value = False
            return True
        return False

    def detect_recovery_fail_safe(self, *args: object) -> bool:
        now, is_quality_degraded = self._parse_args(*args)
        if is_quality_degraded:
            self._fail_safe_recovery_start_time = None
            return False
        if self._fail_safe_recovery_start_time is None:
            self._fail_safe_recovery_start_time = now
        if (
            now - self._fail_safe_recovery_start_time
            >= self.param.failsafe_recovery_confirm_duration_sec
        ):
            self._fail_safe_recovery_start_time = None
            self.is_fail_safe.value = False
            return True
        return False

    def _error_log_output(self, err_idx: int, *args: object) -> None:
        index = _sensor_index(args)
        self._logger.warning(
            self.get_error_no(err_idx)
            + f": Lidar[{index}] comm quality error detected."
        )

    def _recover_log_output(self, err_idx: int, *args: object) -> None:
        index = _sensor_index(args)
        self._logger.info(
            self.get_error_no(err_idx)
            + f": Lidar[{index}] comm quality error recovered."
        )

    def _fail_safe_recover_log_output(self, err_idx: int, *args: object) -> None:
        index = _sensor_index(args)
        self._logger.info(
            self.get_error_no(err_idx)
            + f": Lidar[{index}] comm quality error recovered from failsafe."
        )


class CameraNCommQualityErrorDiagnosis(StateErrorDiagnosisB):
    """CAMERA_N_COMM_QUALITY_ERROR: カメラN通信品質エラー"""

    def __init__(self) -> None:
        super().__init__()
        self._read_history: deque[tuple[float, bool]] = deque()
        self._failure_count: int
        self._failure_rate: float = 0.0
        self._recovery_error_start_time: float | None = None
        self._recovery_fail_safe_start_time: float | None = None
        self._comm_quality_recovery_start_time: float | None = None
        self._comm_quality_fail_safe_start_time: float | None = None
        self._read_success_count_recovery_fail_safe: int = 0
        self._read_success_count_recovery_error: int = 0
        self._initial_diagnosis_time: float | None = None
        self.param: err_conf.CameraCommQualityErrorParameters

    def _parse_args(self, *args: object) -> tuple[float, float | None, int]:
        """
        args[0] (float) : 現在時刻now
        args[1] (float | None) : 最後の取得時刻timestamp
        args[2] (int) : 連続取得失敗回数read_failure_count
        """
        if len(args) != 3:
            raise ValueError("args must be (now, timestamp, read_failure_count)")
        now = args[0]
        timestamp = args[1]
        read_failure_count = args[2]

        if (
            not isinstance(now, float)
            or (timestamp is not None and not isinstance(timestamp, float))
            or not isinstance(read_failure_count, int)
        ):
            raise ValueError("args must be float, float | None, int")

        return (
            float(now),
            timestamp,
            int(read_failure_count),
        )

    def update(self, err_conf: err_conf.ErrorConfig) -> None:
        self.param = err_conf.camera_n_comm_quality_error
        self.is_enabled = self.param.is_enabled

    def detect_error(self, *args: object) -> bool:
        now, timestamp, read_failure_count = self._parse_args(*args)
        if self._initial_diagnosis_time is None:
            self._initial_diagnosis_time = now

        if timestamp is None:
            timestamp = self._initial_diagnosis_time

        # 最近の取得失敗履歴を更新し、古い履歴を削除する
        read_failed = read_failure_count > 0
        self._read_history.append((now, read_failed))
        while (
            self._read_history
            and now - self._read_history[0][0] > self.param.read_error_rate_window_sec
        ):
            self._read_history.popleft()

        # 最近の取得失敗率を計算する
        total_count = len(self._read_history)
        self._failure_count = sum(
            1 for _, is_failure in self._read_history if is_failure
        )
        self._failure_rate = (
            self._failure_count / total_count if total_count > 0 else 0.0
        )

        if (
            now - timestamp > self.param.read_error_threshold_sec
            or read_failure_count >= self.param.read_error_threshold_count
            or self._failure_rate >= self.param.read_error_rate_threshold
        ):
            self.is_error.value = True
            self.is_fail_safe.value = True
            return True

        return False

    def detect_recovery_error(self, *args: object) -> bool:
        now, timestamp, read_failure_count = self._parse_args(*args)
        if timestamp is None:
            return False

        is_read_error_recovery_confirmed = False
        if now - timestamp <= self.param.recovery_receive_interval_sec:
            if self._recovery_error_start_time is None:
                self._recovery_error_start_time = now
            else:
                is_read_error_recovery_confirmed = (
                    now - self._recovery_error_start_time
                    >= self.param.read_error_recovery_confirm_duration_sec
                )
        else:
            self._recovery_error_start_time = None

        if read_failure_count != 0:
            self._comm_quality_recovery_start_time = None
            self._read_success_count_recovery_error = 0
        else:
            self._read_success_count_recovery_error += 1

        read_error_recovery_threshold_count = (
            self._read_success_count_recovery_error
            >= self.param.read_error_recovery_threshold_count
        )

        if self._comm_quality_recovery_start_time is None:
            self._comm_quality_recovery_start_time = now

        latest_success_rate = (
            now - self._comm_quality_recovery_start_time
            > self.param.error_rate_recovery_confirm_duration_sec
            and self._failure_count == 0
        )

        if (
            is_read_error_recovery_confirmed
            and read_error_recovery_threshold_count
            and latest_success_rate
        ):
            self._recovery_error_start_time = None
            self.is_error.value = False
            return True

        return False

    def detect_recovery_fail_safe(self, *args: object) -> bool:
        now, timestamp, read_failure_count = self._parse_args(*args)
        if timestamp is None:
            return False

        is_fail_safe_recovery_confirmed = False
        if now - timestamp <= self.param.fail_safe_recovery_receive_interval_sec:
            if self._recovery_fail_safe_start_time is None:
                self._recovery_fail_safe_start_time = now
            else:
                is_fail_safe_recovery_confirmed = (
                    now - self._recovery_fail_safe_start_time
                    >= self.param.fail_safe_recovery_confirm_duration_sec
                )
        else:
            self._recovery_fail_safe_start_time = None

        if read_failure_count != 0:
            self._comm_quality_fail_safe_start_time = None
            self._read_success_count_recovery_fail_safe = 0
        else:
            self._read_success_count_recovery_fail_safe += 1

        fail_safe_recovery_threshold_count = (
            self._read_success_count_recovery_fail_safe
            >= self.param.fail_safe_recovery_threshold_count
        )

        if self._comm_quality_fail_safe_start_time is None:
            self._comm_quality_fail_safe_start_time = now

        latest_success_rate = (
            now - self._comm_quality_fail_safe_start_time
            > self.param.fail_safe_rate_recovery_confirm_duration_sec
            and self._failure_count == 0
        )

        if (
            is_fail_safe_recovery_confirmed
            and fail_safe_recovery_threshold_count
            and latest_success_rate
        ):
            self._recovery_fail_safe_start_time = None
            self.is_fail_safe.value = False
            return True

        return False

    def _error_log_output(self, err_idx: int, *args: object) -> None:
        if len(args) != 1:
            raise ValueError("args must be (index,)")
        if isinstance(args[0], int):
            index = args[0]
        else:
            raise ValueError("args[0] must be int")
        self._logger.warning(
            self.get_error_no(err_idx)
            + f": Camera {index} communication quality degradation persists. If this occurs frequently, restarting may help."
        )

    def _recover_log_output(self, err_idx: int, *args: object) -> None:
        if len(args) != 1:
            raise ValueError("args must be (index,)")
        if isinstance(args[0], int):
            index = args[0]
        else:
            raise ValueError("args[0] must be int")
        self._logger.warning(
            self.get_error_no(err_idx)
            + f": Camera {index} communication quality has recovered."
        )

    def _fail_safe_recover_log_output(self, err_idx: int, *args: object) -> None:
        if len(args) != 1:
            raise ValueError("args must be (index,)")
        if isinstance(args[0], int):
            index = args[0]
        else:
            raise ValueError("args[0] must be int")
        self._logger.warning(
            self.get_error_no(err_idx)
            + f": Camera {index} communication quality has recovered from fail-safe mode."
        )


class LidarNInvalidDataDiagnosis(StateErrorDiagnosisB):
    """LIDARN_INVALID_DATA: LidarNデータ不正"""

    def __init__(self) -> None:
        super().__init__()
        self._error_start_time: float | None = None
        self._recovery_start_time: float | None = None
        self._fail_safe_recovery_start_time: float | None = None
        self.param: err_conf.LidarInvalidDataParameters

    def _parse_args(self, *args: object) -> tuple[float, float]:
        if len(args) != 2:
            raise ValueError("args must be (now, invalid_ratio)")
        now, invalid_ratio = args
        if not isinstance(now, float) or not isinstance(invalid_ratio, float):
            raise ValueError("args must be float, float")
        return now, invalid_ratio

    def update(self, error_config: err_conf.ErrorConfig) -> None:
        self.param = error_config.lidar_n_invalid_data
        self.is_enabled = self.param.is_enabled

    def detect_error(self, *args: object) -> bool:
        now, invalid_ratio = self._parse_args(*args)
        if invalid_ratio < self.param.invalid_ratio_threshold:
            self._error_start_time = None
            return False
        if self._error_start_time is None:
            self._error_start_time = now
        if now - self._error_start_time >= self.param.error_confirm_duration_sec:
            self.is_error.value = True
            self.is_fail_safe.value = True
            return True
        return False

    def detect_recovery_error(self, *args: object) -> bool:
        now, invalid_ratio = self._parse_args(*args)
        if invalid_ratio >= self.param.recovery_ratio_threshold:
            self._recovery_start_time = None
            return False
        if self._recovery_start_time is None:
            self._recovery_start_time = now
        if now - self._recovery_start_time >= self.param.recovery_confirm_duration_sec:
            self._recovery_start_time = None
            self.is_error.value = False
            return True
        return False

    def detect_recovery_fail_safe(self, *args: object) -> bool:
        now, invalid_ratio = self._parse_args(*args)
        if invalid_ratio >= self.param.recovery_ratio_threshold:
            self._fail_safe_recovery_start_time = None
            return False
        if self._fail_safe_recovery_start_time is None:
            self._fail_safe_recovery_start_time = now
        if (
            now - self._fail_safe_recovery_start_time
            >= self.param.failsafe_recovery_confirm_duration_sec
        ):
            self._fail_safe_recovery_start_time = None
            self.is_fail_safe.value = False
            return True
        return False

    def _error_log_output(self, err_idx: int, *args: object) -> None:
        index = _sensor_index(args)
        self._logger.warning(
            self.get_error_no(err_idx) + f": Lidar[{index}] invalid data detected."
        )

    def _recover_log_output(self, err_idx: int, *args: object) -> None:
        index = _sensor_index(args)
        self._logger.info(
            self.get_error_no(err_idx) + f": Lidar[{index}] invalid data recovered."
        )

    def _fail_safe_recover_log_output(self, err_idx: int, *args: object) -> None:
        index = _sensor_index(args)
        self._logger.info(
            self.get_error_no(err_idx)
            + f": Lidar[{index}] invalid data recovered from failsafe."
        )


class CameraNInvalidDataDiagnosis(StateErrorDiagnosisB):
    """CAMERAN_INVALID_DATA: カメラNデータ不正"""

    def __init__(self) -> None:
        super().__init__()
        self._black_count: int = 0
        self._non_black_frame_count: int = 0
        self.param: err_conf.CameraInvalidDataParameters

    def update(self, err_conf: err_conf.ErrorConfig) -> None:
        self.param = err_conf.camera_n_invalid_data
        self.is_enabled = self.param.is_enabled

    def _parse_args(self, *args: object) -> NDArray[np.uint8]:
        """
        args[0] (NDArray[np.uint8]) : 画像データimage
        """
        if len(args) != 1:
            raise ValueError("args must be (image)")
        image = args[0]

        if not isinstance(image, np.ndarray) or image.dtype != np.uint8:
            raise ValueError("args must be NDArray[np.uint8]")

        return cast(NDArray[np.uint8], image)

    def detect_error(self, *args: object) -> bool:
        """
        真っ黒な画像が error_threshold_frames 続いたらエラー(True)。
        """
        image = self._parse_args(*args)
        if not image.any():
            self._black_count += 1
            self._non_black_frame_count = 0
        else:
            self._black_count = 0
            self._non_black_frame_count += 1

        is_error = self._black_count >= self.param.error_threshold_frames

        if is_error:
            self.is_error.value = True
            self.is_fail_safe.value = True
            return True

        return False

    def detect_recovery_error(self, *args: object) -> bool:
        is_recovered = (
            self._non_black_frame_count >= self.param.error_recovery_frame_threshold
        )

        if is_recovered:
            self.is_error.value = False
            return True

        return False

    def detect_recovery_fail_safe(self, *args: object) -> bool:
        is_recovered = (
            self._non_black_frame_count >= self.param.fail_safe_recovery_frame_threshold
        )

        if is_recovered:
            self.is_fail_safe.value = False
            return True

        return False

    def _error_log_output(self, err_idx: int, *args: object) -> None:
        if len(args) != 1:
            raise ValueError("args must be (index,)")
        if isinstance(args[0], int):
            index: int = args[0]
        else:
            raise ValueError("args[0] must be int")
        self._logger.warning(
            self.get_error_no(err_idx)
            + f": Invalid camera data has been detected continuously on Camera {index}. Please check the sensor connection status. If this occurs frequently, restarting may help."
        )

    def _recover_log_output(self, err_idx: int, *args: object) -> None:
        if len(args) != 1:
            raise ValueError("args must be (index,)")
        if isinstance(args[0], int):
            index: int = args[0]
        else:
            raise ValueError("args[0] must be int")
        self._logger.warning(
            self.get_error_no(err_idx) + f": CAM[{index}] image data has recovered."
        )

    def _fail_safe_recover_log_output(self, err_idx: int, *args: object) -> None:
        if len(args) != 1:
            raise ValueError("args must be (index,)")
        if isinstance(args[0], int):
            index: int = args[0]
        else:
            raise ValueError("args[0] must be int")
        self._logger.warning(
            self.get_error_no(err_idx)
            + f": CAM[{index}] image data has recovered from failsafe."
        )


class YawAngleInfoErrorDiagnosis(StateErrorDiagnosisA):
    """YAW_ANGLE_INFO_ERROR: 旋回角情報エラー"""

    def __init__(self, runtime_policy: DiagnosisRuntimePolicy | None = None) -> None:
        super().__init__(runtime_policy)
        self._initial_diagnosis_time: float | None = None
        self._last_received_time: float | None = None
        self._error_receive_recovery_start_time: float | None = None
        self._fail_safe_receive_recovery_start_time: float | None = None
        self.param: err_conf.YawAngleInfoErrorParameters

    def update(self, err_conf: err_conf.ErrorConfig) -> None:
        self.param = err_conf.yaw_angle_info_error
        self.is_enabled = self.param.is_enabled

    def _parse_args(self, *args: object) -> tuple[float | None, float]:
        """
        args[0] (float): CANデータ取得時刻 timestamp
        args[1] (float): 現在時刻 now
        """
        if len(args) != 2:
            raise ValueError("args must be (timestamp, now)")

        timestamp = args[0]
        now = args[1]

        if (
            timestamp is not None and not isinstance(timestamp, float)
        ) or not isinstance(now, float):
            raise ValueError("args must be (timestamp: float | None, now: float)")

        return timestamp, now

    def detect_error(self, *args: object) -> bool:
        timestamp, now = self._parse_args(*args)
        # メンテナンスモード中は旋回角異常を新規検出しない。
        if self.is_suppressed_in_maintenance:
            return False

        if self._initial_diagnosis_time is None:
            self._initial_diagnosis_time = now

        if timestamp is None:
            timestamp = self._initial_diagnosis_time

        if now - timestamp >= self.param.error_duration_sec:
            self.is_error.value = True
            self.is_fail_safe.value = True
            self.is_idle.value = True
            return True

        return False

    def detect_recovery_error(self, *args: object) -> bool:
        timestamp, now = self._parse_args(*args)

        if timestamp is None:
            return False

        if now - timestamp <= self.param.recovery_receive_interval_sec:
            if self._error_receive_recovery_start_time is None:
                self._error_receive_recovery_start_time = now
        else:
            self._error_receive_recovery_start_time = None
            return False

        if (
            now - self._error_receive_recovery_start_time
            >= self.param.error_recovery_duration_sec
        ):
            self.is_error.value = False
            self._error_receive_recovery_start_time = None
            return True

        return False

    def detect_recovery_fail_safe(self, *args: object) -> bool:
        timestamp, now = self._parse_args(*args)

        if timestamp is None:
            return False

        if now - timestamp <= self.param.fail_safe_recovery_receive_interval_sec:
            if self._fail_safe_receive_recovery_start_time is None:
                self._fail_safe_receive_recovery_start_time = now
        else:
            self._fail_safe_receive_recovery_start_time = None
            return False

        if (
            now - self._fail_safe_receive_recovery_start_time
            >= self.param.fail_safe_recovery_duration_sec
        ):
            self.is_fail_safe.value = False
            self.is_idle.value = False
            self._fail_safe_receive_recovery_start_time = None
            return True

        return False

    def _error_log_output(self, err_idx: int, *args: object) -> None:
        self._logger.error(
            self.get_error_no(err_idx) + ": Unable to acquire yaw angle information."
        )

    def _recover_log_output(self, err_idx: int, *args: object) -> None:
        self._logger.warning(
            self.get_error_no(err_idx) + ": Yaw angle CAN data has recovered."
        )

    def _fail_safe_recover_log_output(self, err_idx: int, *args: object) -> None:
        self._logger.warning(
            self.get_error_no(err_idx)
            + ": Yaw angle CAN data has recovered from failsafe."
        )


class CanCommQualityDegradedDiagnosis(StateErrorDiagnosisC):
    """CAN_COMM_QUALITY_DEGRADED: CAN通信品質低下"""

    def __init__(self) -> None:
        super().__init__()
        self._initial_diagnosis_time_by_canid: dict[str, float | None] = {}
        self._last_failed_count_by_canid: dict[str, int] = {}
        self._error_recovery_count_by_canid: dict[str, int] = {}
        self._fail_safe_recovery_count_by_canid: dict[str, int] = {}
        self.can_last_update_time: dict[str, float | None] = {}
        self.param: err_conf.CanCommQualityDegradedParameters

    def _normalize_can_id(self, can_id: str) -> str:
        return can_id.upper().lstrip("0X")

    def _parse_args(self, *args: object) -> tuple[str, int, float | None, float]:
        """
        args[0] (str): CAN ID can_id
        args[1] (int): 連続取得失敗回数 failed_count
        args[2] (float): CANデータ取得時刻 timestamp
        args[3] (float): 現在時刻 now
        """
        if len(args) != 4:
            raise ValueError("args must be (can_id, failed_count, timestamp, now)")

        can_id = args[0]
        failed_count = args[1]
        timestamp = args[2]
        now = args[3]

        if (
            not isinstance(can_id, str)
            or not isinstance(failed_count, int)
            or (timestamp is not None and not isinstance(timestamp, float))
            or not isinstance(now, float)
        ):
            raise ValueError("args must be str, int, float | None, float")

        return (
            str(can_id),
            int(failed_count),
            timestamp,
            float(now),
        )

    def update(self, err_conf: err_conf.ErrorConfig) -> None:
        # 診断対象のCANIDを登録し、CANIDごとの状態を初期化する。
        self.param = err_conf.can_comm_quality_degraded
        monitored_can_ids = (
            self.param.angle_can_id,
            self.param.lever_can_id,
        )
        self.can_last_update_time = dict.fromkeys(monitored_can_ids, None)
        self._initial_diagnosis_time_by_canid = dict.fromkeys(monitored_can_ids, None)
        self._last_failed_count_by_canid = dict.fromkeys(monitored_can_ids, 0)
        self._error_recovery_count_by_canid = dict.fromkeys(monitored_can_ids, 0)
        self._fail_safe_recovery_count_by_canid = dict.fromkeys(monitored_can_ids, 0)

        self.is_enabled = self.param.is_enabled

    def _update_input_state(
        self, can_id: str, failed_count: int, timestamp: float | None, now: float
    ) -> None:
        # CAN IDを特定できないタイムアウトは、個別CAN IDの状態へ反映しない。
        if can_id == "":
            return

        if can_id not in self.can_last_update_time:
            return

        if failed_count == 0:
            # failed_countは累積値として保存せず、成功イベントを受けたら診断側の連続失敗カウンタを明示的にリセットする。
            self._last_failed_count_by_canid[can_id] = 0
        else:
            # 失敗イベントを1回として数える。入力されたfailed_countの値そのものを加算しないことで、診断呼出し単位の連続回数になる。
            self._last_failed_count_by_canid[can_id] += 1

        if self._initial_diagnosis_time_by_canid[can_id] is None:
            self._initial_diagnosis_time_by_canid[can_id] = now

        if timestamp is not None:
            # timestampは受信したCANIDに対応するスロットだけ更新する。
            self.can_last_update_time[can_id] = timestamp
        elif self.can_last_update_time[can_id] is None:
            # 初回からtimestampがない場合は、診断開始時刻を仮の最終更新時刻として使い、未受信状態が時間経過で検出できるようにする。
            self.can_last_update_time[can_id] = self._initial_diagnosis_time_by_canid[
                can_id
            ]

    def detect_error(self, *args: object) -> bool:
        """
        角度CANとレバーCANを個別に監視する。どちらか一方でも
        timestampの停止または連続失敗回数の超過が発生した時点でエラー
        """
        can_id, failed_count, timestamp, now = self._parse_args(*args)
        can_id_str = self._normalize_can_id(can_id)
        self._update_input_state(can_id_str, failed_count, timestamp, now)

        is_any_error = False
        for target_can_id, stored_timestamp in self.can_last_update_time.items():
            latest_timestamp = stored_timestamp
            if latest_timestamp is None:
                first_now = self._initial_diagnosis_time_by_canid[target_can_id]
                if first_now is None:
                    self._initial_diagnosis_time_by_canid[target_can_id] = now
                    first_now = now
                latest_timestamp = first_now

            is_timeout = (
                now - latest_timestamp >= self.param.error_threshold_sec[target_can_id]
            )
            is_count_over = (
                self._last_failed_count_by_canid[target_can_id]
                >= self.param.error_threshold_count[target_can_id]
            )

            if is_timeout or is_count_over:
                is_any_error = True
                break

        if is_any_error:
            self.is_error.value = True
            self.is_fail_safe.value = True
            return True
        return False

    def _detect_recovery_common(
        self, now: float, can_id: str, is_fail_safe: bool
    ) -> bool:
        if can_id not in self.can_last_update_time:
            return False

        if is_fail_safe:
            recovery_count_by_canid = self._fail_safe_recovery_count_by_canid
            receive_interval_by_canid = (
                self.param.fail_safe_recovery_receive_interval_sec
            )
            recovery_threshold_by_canid = self.param.fail_safe_recovery_threshold_count
        else:
            recovery_count_by_canid = self._error_recovery_count_by_canid
            receive_interval_by_canid = self.param.recovery_receive_interval_sec
            recovery_threshold_by_canid = self.param.recovery_threshold_count

        is_all_recovered = True
        for target_can_id, latest_timestamp in self.can_last_update_time.items():
            if latest_timestamp is None:
                # 一度も更新されていないCANIDがある場合、全CANID復帰条件は満たせない。
                is_all_recovered = False
                break

            is_receive_interval_ok = (
                now - latest_timestamp <= receive_interval_by_canid[target_can_id]
            )

            if not is_receive_interval_ok:
                recovery_count_by_canid[target_can_id] = 0
                is_all_recovered = False
                continue

            if (
                target_can_id == can_id
                and self._last_failed_count_by_canid[target_can_id] == 0
            ):
                recovery_count_by_canid[target_can_id] += 1
            elif target_can_id == can_id:
                recovery_count_by_canid[target_can_id] = 0
                is_all_recovered = False
                continue

            is_above_recovery_threshold = (
                recovery_count_by_canid[target_can_id]
                >= recovery_threshold_by_canid[target_can_id]
            )

            if not (is_receive_interval_ok and is_above_recovery_threshold):
                # timestampの更新間隔と連続成功回数は、両方成立する必要がある。
                is_all_recovered = False

        if not is_all_recovered:
            return False

        for target_can_id in recovery_count_by_canid:
            recovery_count_by_canid[target_can_id] = 0

        if is_fail_safe:
            self.is_fail_safe.value = False
        else:
            self.is_error.value = False
        return True

    def detect_recovery_error(self, *args: object) -> bool:
        # 通常エラー復帰用の状態を使って共通復帰判定を行う。
        can_id, _, _, now = self._parse_args(*args)
        can_id_str = self._normalize_can_id(can_id)
        return self._detect_recovery_common(now, can_id_str, is_fail_safe=False)

    def detect_recovery_fail_safe(self, *args: object) -> bool:
        # フェールセーフ復帰用の状態を使って共通復帰判定を行う。
        can_id, _, _, now = self._parse_args(*args)
        can_id_str = self._normalize_can_id(can_id)
        return self._detect_recovery_common(now, can_id_str, is_fail_safe=True)

    def _error_log_output(self, err_idx: int, *args: object) -> None:
        self._logger.warning(
            self.get_error_no(err_idx)
            + ": CAN signal communication quality is degraded. If this occurs frequently, restarting may help."
        )

    def _recover_log_output(self, err_idx: int, *args: object) -> None:
        self._logger.warning(
            self.get_error_no(err_idx)
            + ": CAN communication quality degradation has recovered."
        )

    def _fail_safe_recover_log_output(self, err_idx: int, *args: object) -> None:
        self._logger.warning(
            self.get_error_no(err_idx)
            + ": CAN communication quality degradation has recovered from failsafe."
        )


class CanCommQualityErrorDiagnosis(StateErrorDiagnosisB):
    """CAN_COMM_QUALITY_ERROR: CAN通信品質エラー"""

    def __init__(self) -> None:
        super().__init__()
        self._read_history_by_canid: dict[str, deque[tuple[float, bool]]] = {}
        self._failure_count_by_canid: dict[str, int] = {}
        self._failure_rate_by_canid: dict[str, float] = {}
        self._recovery_error_start_time_by_canid: dict[str, float | None] = {}
        self._recovery_fail_safe_start_time_by_canid: dict[str, float | None] = {}
        self._comm_quality_recovery_start_time_by_canid: dict[str, float | None] = {}
        self._comm_quality_fail_safe_start_time_by_canid: dict[str, float | None] = {}
        self._read_success_count_recovery_fail_safe_by_canid: dict[str, int] = {}
        self._read_success_count_recovery_error_by_canid: dict[str, int] = {}
        self._initial_diagnosis_time_by_canid: dict[str, float | None] = {}
        self._last_read_failure_streak_by_canid: dict[str, int] = {}
        self.can_last_update_time: dict[str, float | None] = {}
        self.param: err_conf.CanCommQualityErrorParameters

    def _normalize_can_id(self, can_id: str) -> str:
        return can_id.upper().lstrip("0X")

    def update(self, err_conf: err_conf.ErrorConfig) -> None:
        # 診断対象のCANIDを登録し、CANIDごとの状態を初期化する。

        self.param = err_conf.can_comm_quality_error
        monitored_can_ids = (
            self.param.angle_can_id,
            self.param.lever_can_id,
        )
        self.can_last_update_time = dict.fromkeys(monitored_can_ids, None)
        self._initial_diagnosis_time_by_canid = dict.fromkeys(monitored_can_ids, None)
        self._last_read_failure_streak_by_canid = dict.fromkeys(monitored_can_ids, 0)
        self._read_history_by_canid = {can_id: deque() for can_id in monitored_can_ids}
        self._failure_count_by_canid = dict.fromkeys(monitored_can_ids, 0)
        self._failure_rate_by_canid = dict.fromkeys(monitored_can_ids, 0.0)
        self._recovery_error_start_time_by_canid = dict.fromkeys(
            monitored_can_ids, None
        )
        self._recovery_fail_safe_start_time_by_canid = dict.fromkeys(
            monitored_can_ids, None
        )
        self._comm_quality_recovery_start_time_by_canid = dict.fromkeys(
            monitored_can_ids, None
        )
        self._comm_quality_fail_safe_start_time_by_canid = dict.fromkeys(
            monitored_can_ids, None
        )
        self._read_success_count_recovery_error_by_canid = dict.fromkeys(
            monitored_can_ids, 0
        )
        self._read_success_count_recovery_fail_safe_by_canid = dict.fromkeys(
            monitored_can_ids, 0
        )

        self.is_enabled = self.param.is_enabled

    def _parse_args(self, *args: object) -> tuple[str, int, float | None, float]:
        # CANID、今回の失敗通知、timestamp、現在時刻を検証して取り出す。
        """
        args[0] (str): CAN ID can_id
        args[1] (int): 連続取得失敗回数 failed_count
        args[2] (float): CANデータ取得時刻 timestamp
        args[3] (float): 現在時刻 now
        """
        if len(args) != 4:
            raise ValueError("args must be (can_id, failed_count, timestamp, now)")

        can_id = args[0]
        failed_count = args[1]
        timestamp = args[2]
        now = args[3]

        if (
            not isinstance(can_id, str)
            or not isinstance(failed_count, int)
            or (timestamp is not None and not isinstance(timestamp, float))
            or not isinstance(now, float)
        ):
            raise ValueError("args must be str, int, float | None, float")

        return (
            str(can_id),
            int(failed_count),
            timestamp,
            float(now),
        )

    def _update_input_state(
        self,
        can_id: str,
        read_failure_streak: int,
        timestamp: float | None,
        now: float,
    ) -> None:
        # 受信はCANID単位で発生するため、timestampと連続失敗状態を別CANIDへコピーしない。
        # CAN IDを特定できないタイムアウトは、個別CAN IDの状態へ反映しない。
        if can_id == "":
            return

        if can_id not in self.can_last_update_time:
            return

        if read_failure_streak == 0:
            # 成功イベントは連続失敗状態を断ち切る。
            self._last_read_failure_streak_by_canid[can_id] = 0
        else:
            # 入力値は累積回数ではなく、今回の受信処理が成功したかどうかを示す通知として扱う。
            self._last_read_failure_streak_by_canid[can_id] += 1

        if self._initial_diagnosis_time_by_canid[can_id] is None:
            self._initial_diagnosis_time_by_canid[can_id] = now

        if timestamp is not None:
            # timestampは今回通知されたCANIDの履歴だけ更新する。
            self.can_last_update_time[can_id] = timestamp
        elif self.can_last_update_time[can_id] is None:
            # 初回timestampがない場合の基準時刻を保存し、未受信タイムアウトの計測開始点を確定する。
            self.can_last_update_time[can_id] = self._initial_diagnosis_time_by_canid[
                can_id
            ]

    def _refresh_failure_rate(
        self, can_id: str, read_failure_streak: int, now: float
    ) -> None:
        # 対象CANIDの読込結果だけを時間履歴へ追加し、設定された時間窓より古いサンプルを捨てる。
        read_history = self._read_history_by_canid[can_id]
        read_history.append((now, read_failure_streak > 0))

        while read_history and (
            now - read_history[0][0] > self.param.read_error_rate_window_sec[can_id]
        ):
            read_history.popleft()

        total_count = len(read_history)
        failure_count = sum(1 for _, is_failure in read_history if is_failure)
        failure_rate = failure_count / total_count if total_count > 0 else 0.0

        self._failure_count_by_canid[can_id] = failure_count
        self._failure_rate_by_canid[can_id] = failure_rate

    def detect_error(self, *args: object) -> bool:
        """
        エラーはどれか1つのCANIDが長時間停止、連続失敗、または時間窓内の
        失敗率超過になった場合にエラー
        """
        can_id, read_failure_streak, timestamp, now = self._parse_args(*args)
        can_id_str = self._normalize_can_id(can_id)
        self._update_input_state(can_id_str, read_failure_streak, timestamp, now)
        if can_id_str in self._read_history_by_canid:
            self._refresh_failure_rate(can_id_str, read_failure_streak, now)

        is_any_error = False
        for target_can_id, stored_timestamp in self.can_last_update_time.items():
            latest_timestamp = stored_timestamp
            if latest_timestamp is None:
                first_now = self._initial_diagnosis_time_by_canid[target_can_id]
                if first_now is None:
                    self._initial_diagnosis_time_by_canid[target_can_id] = now
                    first_now = now
                latest_timestamp = first_now

            is_timeout = (
                now - latest_timestamp
                >= self.param.read_error_threshold_sec[target_can_id]
            )
            is_count_over = (
                self._last_read_failure_streak_by_canid[target_can_id]
                >= self.param.read_error_threshold_count[target_can_id]
            )
            is_rate_over = (
                self._failure_rate_by_canid[target_can_id]
                >= self.param.read_error_rate_threshold[target_can_id]
            )
            if is_timeout or is_count_over or is_rate_over:
                is_any_error = True
                break

        if is_any_error:
            self.is_error.value = True
            self.is_fail_safe.value = True
            return True

        return False

    def _detect_recovery_common(
        self, now: float, can_id: str, is_fail_safe: bool
    ) -> bool:
        if can_id not in self.can_last_update_time:
            return False

        if is_fail_safe:
            recovery_start_by_canid = self._recovery_fail_safe_start_time_by_canid
            comm_quality_start_by_canid = (
                self._comm_quality_fail_safe_start_time_by_canid
            )
            success_count_by_canid = (
                self._read_success_count_recovery_fail_safe_by_canid
            )
            receive_interval_by_canid = (
                self.param.fail_safe_recovery_receive_interval_sec
            )
            recovery_confirm_duration_by_canid = (
                self.param.fail_safe_recovery_confirm_duration_sec
            )
            recovery_threshold_count_by_canid = (
                self.param.fail_safe_recovery_threshold_count
            )
            rate_recovery_duration_by_canid = (
                self.param.fail_safe_rate_recovery_confirm_duration_sec
            )
        else:
            recovery_start_by_canid = self._recovery_error_start_time_by_canid
            comm_quality_start_by_canid = (
                self._comm_quality_recovery_start_time_by_canid
            )
            success_count_by_canid = self._read_success_count_recovery_error_by_canid
            receive_interval_by_canid = self.param.recovery_receive_interval_sec
            recovery_confirm_duration_by_canid = (
                self.param.read_error_recovery_confirm_duration_sec
            )
            recovery_threshold_count_by_canid = (
                self.param.read_error_recovery_threshold_count
            )
            rate_recovery_duration_by_canid = (
                self.param.error_rate_recovery_confirm_duration_sec
            )

        is_all_recovered = True
        for target_can_id, latest_timestamp in self.can_last_update_time.items():
            if latest_timestamp is None:
                # 対象CANIDの更新実績がない場合、復帰確認を開始できない。
                is_all_recovered = False
                continue

            is_receive_recovery_confirmed = False
            if now - latest_timestamp <= receive_interval_by_canid[target_can_id]:
                # timestampが許容間隔内なら、確認継続時間の計測を開始・継続する。
                if recovery_start_by_canid[target_can_id] is None:
                    recovery_start_by_canid[target_can_id] = now
                else:
                    is_receive_recovery_confirmed = (
                        now - recovery_start_by_canid[target_can_id]
                        >= recovery_confirm_duration_by_canid[target_can_id]
                    )
            else:
                # 更新間隔が空いた時点で、そのCANIDのtimestamp復帰確認をリセットする。
                recovery_start_by_canid[target_can_id] = None
                is_all_recovered = False
                continue

            if (
                target_can_id == can_id
                and self._last_read_failure_streak_by_canid[target_can_id] != 0
            ):
                # 現在入力されたCANIDが失敗した場合、連続成功数と100%成功確認を破棄。
                comm_quality_start_by_canid[target_can_id] = None
                success_count_by_canid[target_can_id] = 0
                is_all_recovered = False
                continue

            if target_can_id == can_id:
                success_count_by_canid[target_can_id] += 1
            is_read_count_recovery_confirmed = (
                success_count_by_canid[target_can_id]
                >= recovery_threshold_count_by_canid[target_can_id]
            )

            if comm_quality_start_by_canid[target_can_id] is None:
                # 失敗率100%確認の計測開始時刻。以後、時間窓の成功状態を確認する。
                comm_quality_start_by_canid[target_can_id] = now

            is_rate_recovery_confirmed = (
                now - comm_quality_start_by_canid[target_can_id]
                > rate_recovery_duration_by_canid[target_can_id]
                and self._failure_count_by_canid[target_can_id] == 0
            )

            if not (
                is_receive_recovery_confirmed
                and is_read_count_recovery_confirmed
                and is_rate_recovery_confirmed
            ):
                # timestamp間隔、連続成功回数、時間窓内100%成功を満たしたら復帰。
                is_all_recovered = False

        if not is_all_recovered:
            return False

        for target_can_id in recovery_start_by_canid:
            recovery_start_by_canid[target_can_id] = None
        for target_can_id in comm_quality_start_by_canid:
            comm_quality_start_by_canid[target_can_id] = None
        for target_can_id in success_count_by_canid:
            success_count_by_canid[target_can_id] = 0

        if is_fail_safe:
            self.is_fail_safe.value = False
        else:
            self.is_error.value = False
        return True

    def detect_recovery_error(self, *args: object) -> bool:
        # 通常エラー復帰用の状態を使って共通復帰判定を行う。
        can_id, _, _, now = self._parse_args(*args)
        can_id_str = self._normalize_can_id(can_id)
        return self._detect_recovery_common(now, can_id_str, is_fail_safe=False)

    def detect_recovery_fail_safe(self, *args: object) -> bool:
        # フェールセーフ復帰用の状態を使って共通復帰判定を行う。
        can_id, _, _, now = self._parse_args(*args)
        can_id_str = self._normalize_can_id(can_id)
        return self._detect_recovery_common(now, can_id_str, is_fail_safe=True)

    def _error_log_output(self, err_idx: int, *args: object) -> None:
        self._logger.warning(
            self.get_error_no(err_idx)
            + ": CAN signal communication quality degradation persists. Restarting may help."
        )

    def _recover_log_output(self, err_idx: int, *args: object) -> None:
        self._logger.warning(
            self.get_error_no(err_idx) + ": CAN communication quality has recovered."
        )

    def _fail_safe_recover_log_output(self, err_idx: int, *args: object) -> None:
        self._logger.warning(
            self.get_error_no(err_idx)
            + ": CAN communication quality has recovered from failsafe."
        )


class CanInvalidDataDiagnosis(StateErrorDiagnosisB):
    """CAN_INVALID_DATA_DIAGNOSIS: CANデータ不正"""

    def __init__(self) -> None:
        super().__init__()
        self._error_recovery_start_time: dict[str, float | None] = {}
        self._fail_safe_recovery_start_time: dict[str, float | None] = {}
        self.param: err_conf.CanInvalidDataParameters

    def update(self, err_conf: err_conf.ErrorConfig) -> None:
        self.param = err_conf.can_invalid_data

        # 辞書のキーから監視対象のCAN IDを取得し、値をNoneに初期化する
        self._error_recovery_start_time = {
            can_id.upper().lstrip("0X"): None for can_id in self.param.params_by_canid
        }
        self._fail_safe_recovery_start_time = {
            can_id.upper().lstrip("0X"): None for can_id in self.param.params_by_canid
        }

        self.is_enabled = self.param.is_enabled

    def _parse_args(self, *args: object) -> tuple[str, str, float]:
        """
        args[0] (str): CAN IDcan_id
        args[1] (str): CANデータdata
        args[2] (float): 現在時刻now
        """
        if len(args) != 3:
            raise ValueError("args must be (can_id, data, now)")
        can_id = args[0]
        data = args[1]
        now = args[2]

        if (
            not isinstance(can_id, str)
            or not isinstance(data, str)
            or not isinstance(now, float)
        ):
            raise ValueError("args must be str, str, float")

        return str(can_id), str(data), float(now)

    def detect_error(self, *args: object) -> bool:
        can_id, data, _ = self._parse_args(*args)
        can_id_str: str = can_id.upper().lstrip("0X")

        config = self.param.params_by_canid.get(can_id_str)
        if config is None:
            return False

        if len(data) < config.required_length:
            self.is_error.value = True
            self.is_fail_safe.value = True
            return True
        return False

    def detect_recovery_error(self, *args: object) -> bool:
        can_id, data, now = self._parse_args(*args)
        can_id_str: str = can_id.upper().lstrip("0X")
        config = self.param.params_by_canid.get(can_id_str)
        if config is None:
            return False

        if len(data) < config.required_length:
            self._error_recovery_start_time[can_id_str] = None
            return False
        if self._error_recovery_start_time[can_id_str] is None:
            self._error_recovery_start_time[can_id_str] = now
            return False

        for target_can_id, start_time in self._error_recovery_start_time.items():
            target_config = self.param.params_by_canid[target_can_id]
            if (
                start_time is None
                or (now - start_time)
                < target_config.error_recovery_confirm_duration_sec
            ):
                return False

        for target_can_id in self._error_recovery_start_time:
            self._error_recovery_start_time[target_can_id] = None

        self.is_error.value = False
        return True

    def detect_recovery_fail_safe(self, *args: object) -> bool:
        can_id, data, now = self._parse_args(*args)
        can_id_str: str = can_id.upper().lstrip("0X")
        config = self.param.params_by_canid.get(can_id_str)
        if config is None:
            return False

        if len(data) < config.required_length:
            self._fail_safe_recovery_start_time[can_id_str] = None
            return False
        if self._fail_safe_recovery_start_time[can_id_str] is None:
            self._fail_safe_recovery_start_time[can_id_str] = now
            return False

        for target_can_id, start_time in self._fail_safe_recovery_start_time.items():
            target_config = self.param.params_by_canid[target_can_id]
            if (
                start_time is None
                or (now - start_time)
                < target_config.failsafe_recovery_confirm_duration_sec
            ):
                return False

        for target_can_id in self._fail_safe_recovery_start_time:
            self._fail_safe_recovery_start_time[target_can_id] = None

        self.is_fail_safe.value = False
        return True

    def _error_log_output(self, err_idx: int, *args: object) -> None:
        self._logger.warning(
            self.get_error_no(err_idx)
            + ": Invalid CAN data has been detected continuously. Please check the CAN connection status. If this occurs frequently, restarting may help."
        )

    def _recover_log_output(self, err_idx: int, *args: object) -> None:
        self._logger.warning(
            self.get_error_no(err_idx) + ": CAN data length has recovered."
        )

    def _fail_safe_recover_log_output(self, err_idx: int, *args: object) -> None:
        self._logger.warning(
            self.get_error_no(err_idx)
            + ": CAN data length has recovered from failsafe."
        )


class StorageSpaceLowDiagnosis(StateErrorDiagnosisB):
    """STORAGE_SPACE_LOW: ストレージ残容量低下"""

    def __init__(self) -> None:
        super().__init__()
        self._error_start_time: float | None = None
        self._error_recovery_start_time: float | None = None
        self._fail_safe_recovery_start_time: float | None = None
        self._is_below_threshold: bool = False
        self.param: err_conf.StorageSpaceLowParameters

    def update(self, err_conf: err_conf.ErrorConfig) -> None:
        self.param = err_conf.storage_space_low
        self.is_enabled = self.param.is_enabled

    def _parse_args(self, *args: object) -> tuple[float, float, float]:
        """
        args[0] (float): eMMC残量disk_root_avail[GB]
        args[1] (float): SSD残量disk_data_avail[GB]
        args[2] (float): 現在時刻now
        """
        if len(args) != 3:
            raise ValueError("args must be (disk_root_avail, disk_data_avail, now)")

        disk_root_avail = args[0]
        disk_data_avail = args[1]
        now = args[2]
        if (
            not isinstance(disk_root_avail, float)
            or not isinstance(disk_data_avail, float)
            or not isinstance(now, float)
        ):
            raise ValueError("args must be float, float, float")

        return float(disk_root_avail), float(disk_data_avail), float(now)

    def detect_error(self, *args: object) -> bool:
        """
        いずれかのストレージ残容量が閾値未満の時間を計測し、継続時間を超えたらエラー(True)。
        """
        disk_root_avail, disk_data_avail, now = self._parse_args(*args)

        is_below_threshold = (
            disk_root_avail < self.param.error_threshold_gb
            or disk_data_avail < self.param.error_threshold_gb
        )

        # 残量が閾値未満の場合は、エラー開始時間をリセットしてFalseを返す
        if not is_below_threshold:
            self._error_start_time = None
            return False

        # 閾値未満の場合は、エラー開始時間を記録
        if self._error_start_time is None:
            self._error_start_time = now
            return False

        # 閾値未満の状態が継続した場合、エラー(True)を返す
        if now - self._error_start_time >= self.param.error_duration_sec:
            self.is_error.value = True
            self.is_fail_safe.value = True
            return True

        return False

    def detect_recovery_error(self, *args: object) -> bool:
        disk_root_avail, disk_data_avail, now = self._parse_args(*args)

        # 残量が閾値未満の場合は、エラー復帰開始時間をリセットしてFalseを返す
        is_above_threshold = (
            disk_root_avail >= self.param.error_recovery_threshold_gb
            and disk_data_avail >= self.param.error_recovery_threshold_gb
        )
        if not is_above_threshold:
            self._error_recovery_start_time = None
            return False

        # 閾値以上の場合は、エラー復帰開始時間を記録
        if self._error_recovery_start_time is None:
            self._error_recovery_start_time = now
            return False

        # 閾値以上の状態が継続した場合、エラー復帰(True)
        if (
            now - self._error_recovery_start_time
            >= self.param.error_recovery_duration_sec
        ):
            self._error_recovery_start_time = None
            self.is_error.value = False
            return True

        return False

    def detect_recovery_fail_safe(self, *args: object) -> bool:
        disk_root_avail, disk_data_avail, now = self._parse_args(*args)

        # 残量が閾値未満の場合は、フェールセーフ復帰開始時間をリセットしてFalseを返す
        is_above_threshold = (
            disk_root_avail >= self.param.fail_safe_recovery_threshold_gb
            and disk_data_avail >= self.param.fail_safe_recovery_threshold_gb
        )

        if not is_above_threshold:
            self._fail_safe_recovery_start_time = None
            return False

        # 閾値以上の場合は、フェールセーフ復帰開始時間を記録
        if self._fail_safe_recovery_start_time is None:
            self._fail_safe_recovery_start_time = now
            return False

        # 閾値以上の状態が継続した場合、フェールセーフ復帰(True)
        if (
            now - self._fail_safe_recovery_start_time
            >= self.param.fail_safe_recovery_duration_sec
        ):
            self._fail_safe_recovery_start_time = None
            self.is_fail_safe.value = False
            return True

        return False

    def _error_log_output(self, err_idx: int, *args: object) -> None:
        self._logger.warning(
            self.get_error_no(err_idx)
            + ": Storage space is low. Please free up disk space on root or data volume."
        )

    def _recover_log_output(self, err_idx: int, *args: object) -> None:
        self._logger.warning(
            self.get_error_no(err_idx) + ": Storage space availability has recovered."
        )

    def _fail_safe_recover_log_output(self, err_idx: int, *args: object) -> None:
        self._logger.warning(
            self.get_error_no(err_idx)
            + ": Storage space availability has recovered from failsafe."
        )


class ProcessingSpeedDegradedDiagnosis(StateErrorDiagnosisC):
    """PROCESSING_SPEED_DEGRADED: 処理速度低下"""

    def __init__(self) -> None:
        super().__init__()
        self.param: err_conf.ProcessingSpeedDegradedParameters

    def update(self, err_conf: err_conf.ErrorConfig) -> None:
        self.param = err_conf.processing_speed_degraded
        self.is_enabled = self.param.is_enabled

    def _parse_args(self, *args: object) -> Literal["slow", "trend"] | None:
        """
        args[0] (Literal["slow", "trend"]): status
        """
        if len(args) != 1:
            raise ValueError("args must be (status)")
        status = args[0]
        if status not in ("slow", "trend", None):
            raise ValueError("args must be 'slow', 'trend' or None")

        return status

    def detect_error(self, *args: object) -> bool:
        """
        statusが"slow"の場合にエラー(True)。
        """
        status = self._parse_args(*args)
        if status == "slow":
            self.is_error.value = True
            self.is_fail_safe.value = True
            return True

        return False

    def detect_recovery_error(self, *args: object) -> bool:
        status = self._parse_args(*args)
        if status != "slow":
            self.is_error.value = False
            return True
        return False

    def detect_recovery_fail_safe(self, *args: object) -> bool:
        status = self._parse_args(*args)
        if status != "slow":
            self.is_fail_safe.value = False
            return True
        return False

    def _error_log_output(self, err_idx: int, *args: object) -> None:
        if len(args) != 1:
            raise ValueError("args must be (elapsed_ms)")

        elapsed_ms = args[0]
        if not isinstance(elapsed_ms, (int, float)):
            raise ValueError("elapsed_ms must be int or float")

        self._logger.warning(
            self.get_error_no(err_idx)
            + f": 処理速度が低下しています。処理時間[msec] = {elapsed_ms},"
        )

    def _recover_log_output(self, err_idx: int, *args: object) -> None:
        super()._recover_log_output(err_idx, *args)
        self._logger.warning(
            self.get_error_no(err_idx) + ": 処理速度低下: 復帰しました。"
        )

    def _fail_safe_recover_log_output(self, err_idx: int, *args: object) -> None:
        self._logger.warning(
            self.get_error_no(err_idx) + ": 処理速度低下: 復帰しました。"
        )


class ProcessingSpeedDegradationTrendDiagnosis(StateErrorDiagnosisB):
    """PROCESSING_SPEED_DEGRADATION_TREND: 処理速度低下トレンド"""

    def __init__(self) -> None:
        super().__init__()
        self.param: err_conf.ProcessingSpeedDegradationTrendParameters

    def update(self, err_conf: err_conf.ErrorConfig) -> None:
        self.param = err_conf.processing_speed_degradation_trend
        self.is_enabled = err_conf.processing_speed_degradation_trend.is_enabled

    def _parse_args(self, *args: object) -> Literal["slow", "trend"] | None:
        if len(args) != 1:
            raise ValueError("args must be (status)")
        status = args[0]
        if status not in ("slow", "trend", None):
            raise ValueError("status must be 'slow', 'trend' or None")

        return status

    def detect_error(self, *args: object) -> bool:
        """
        statusが"trend"の場合にエラー(True)。
        """
        status = self._parse_args(*args)
        if status == "trend":
            self.is_error.value = True
            self.is_fail_safe.value = True
            return True

        return False

    def detect_recovery_error(self, *args: object) -> bool:
        status = self._parse_args(*args)
        if status != "trend":
            self.is_error.value = False
            return True
        return False

    def detect_recovery_fail_safe(self, *args: object) -> bool:
        status = self._parse_args(*args)
        if status != "trend":
            self.is_fail_safe.value = False
            return True
        return False

    def _error_log_output(self, err_idx: int, *args: object) -> None:
        if len(args) != 1:
            raise ValueError("args must be (elapsed_ms)")

        elapsed_ms = args[0]
        if not isinstance(elapsed_ms, (int, float)):
            raise ValueError("elapsed_ms must be int or float")

        self._logger.warning(
            self.get_error_no(err_idx)
            + f": 処理速度が低下傾向にあります。処理時間[msec] = {elapsed_ms},"
        )

    def _recover_log_output(self, err_idx: int, *args: object) -> None:
        super()._recover_log_output(err_idx, *args)
        self._logger.warning(
            self.get_error_no(err_idx) + ": 処理速度低下トレンド: 復帰しました。"
        )

    def _fail_safe_recover_log_output(self, err_idx: int, *args: object) -> None:
        self._logger.warning(
            self.get_error_no(err_idx) + ": 処理速度低下トレンド: 復帰しました。"
        )


class OutOfMemoryDiagnosis(StateErrorDiagnosisC):
    """OUT_OF_MEMORY: メモリ不足"""

    def __init__(self) -> None:
        super().__init__()
        self._error_start_time: float | None = None
        self._error_recovery_start_time: float | None = None
        self._fail_safe_recovery_start_time: float | None = None
        self.param: err_conf.OutOfMemoryParameters

    def update(self, err_conf: err_conf.ErrorConfig) -> None:
        self.param = err_conf.out_of_memory
        self.is_enabled = self.param.is_enabled

    def _parse_args(self, *args: object) -> tuple[float, float, float]:
        """
        args[0] : 使用中メモリ ram_used_mb
        args[1] : 総メモリ ram_total_mb
        args[2] : 現在時刻 now
        """
        if len(args) != 3:
            raise ValueError("args must be (ram_used_mb, ram_total_mb, now)")
        ram_used_mb = args[0]
        ram_total_mb = args[1]
        now = args[2]

        if (
            not isinstance(ram_used_mb, int)
            or not isinstance(ram_total_mb, int)
            or not isinstance(now, float)
        ):
            raise ValueError("args must be int, int, float")

        return ram_used_mb, ram_total_mb, now

    def detect_error(self, *args: object) -> bool:
        """
        閾値NG状態が閾値秒以上継続した時点でエラー(True)。
        """
        ram_used_mb, ram_total_mb, now = self._parse_args(*args)
        measured: float = ram_total_mb - ram_used_mb
        is_low_memory: bool = measured < self.param.threshold_mb
        if not is_low_memory:
            self._error_start_time = None
            return False

        # メモリ不足が始まった時刻を記録する
        if self._error_start_time is None:
            self._error_start_time = now
            return False

        if now - self._error_start_time >= self.param.error_duration_sec:
            self.is_error.value = True
            self.is_fail_safe.value = True
            return True

        return False

    def detect_recovery_error(self, *args: object) -> bool:

        ram_used_mb, ram_total_mb, now = self._parse_args(*args)
        measured: float = ram_total_mb - ram_used_mb
        is_enough_memory: bool = measured >= self.param.threshold_mb
        if not is_enough_memory:
            self._error_recovery_start_time = None
            return False

        if self._error_recovery_start_time is None:
            self._error_recovery_start_time = now
            return False

        if (
            now - self._error_recovery_start_time
            >= self.param.error_recovery_duration_sec
        ):
            self.is_error.value = False
            return True

        return False

    def detect_recovery_fail_safe(self, *args: object) -> bool:

        ram_used_mb, ram_total_mb, now = self._parse_args(*args)
        measured: float = ram_total_mb - ram_used_mb
        is_enough_memory: bool = measured >= self.param.threshold_mb
        if not is_enough_memory:
            self._fail_safe_recovery_start_time = None
            return False

        if self._fail_safe_recovery_start_time is None:
            self._fail_safe_recovery_start_time = now
            return False

        if (
            now - self._fail_safe_recovery_start_time
            >= self.param.fail_safe_recovery_duration_sec
        ):
            self.is_fail_safe.value = False
            return True

        return False

    def _error_log_output(self, err_idx: int, *args: object) -> None:
        self._logger.warning(
            self.get_error_no(err_idx)
            + ": メモリ不足: メモリが不足しています。再起動で改善する場合があります。"
        )

    def _recover_log_output(self, err_idx: int, *args: object) -> None:
        self._logger.warning(
            self.get_error_no(err_idx) + ": メモリ不足: メモリ不足が回復しました"
        )

    def _fail_safe_recover_log_output(self, err_idx: int, *args: object) -> None:
        self._logger.warning(
            self.get_error_no(err_idx) + ": メモリ不足: メモリ不足が回復しました"
        )


class GpuPerformanceDegradedDiagnosis(StateErrorDiagnosisC):
    """GPU_PERFORMANCE_DEGRADED: GPU性能低下"""

    def __init__(self) -> None:
        super().__init__()
        self.param: err_conf.GpuPerformanceDegradedParameters
        self._error_start_time: float | None = None
        self._recovery_start_time: float | None = None
        self._fail_safe_recovery_start_time: float | None = None

    def update(self, err_conf: err_conf.ErrorConfig) -> None:
        self.param = err_conf.gpu_performance_degraded
        self.is_enabled = self.param.is_enabled

    def _parse_args(self, *args: object) -> tuple[float, int]:
        """
        args[0] (float) : 現在時刻now
        args[1] (int): throttling_gpu GPUサーマルスロットリング状態
        """
        if len(args) != 2:
            raise ValueError("args must be (now, throttling_gpu)")

        now = args[0]
        throttling_gpu = args[1]
        if isinstance(now, float) and isinstance(throttling_gpu, int):
            return now, throttling_gpu

        raise ValueError("now must be float and throttling_gpu must be int")

    def detect_error(self, *args: object) -> bool:
        now, throttling_gpu = self._parse_args(*args)
        if throttling_gpu > 0:
            if self._error_start_time is None:
                self._error_start_time = now
            elif (now - self._error_start_time) >= self.param.error_duration_sec:
                self.is_error.value = True
                self.is_fail_safe.value = True
                return True
        return False

    def detect_recovery_error(self, *args: object) -> bool:
        now, throttling_gpu = self._parse_args(*args)
        if throttling_gpu == 0:
            if self._recovery_start_time is None:
                self._recovery_start_time = now
            elif (
                now - self._recovery_start_time
            ) >= self.param.error_recovery_duration_sec:
                self.is_error.value = False
                return True
        return False

    def detect_recovery_fail_safe(self, *args: object) -> bool:
        now, throttling_gpu = self._parse_args(*args)
        if throttling_gpu == 0:
            if self._fail_safe_recovery_start_time is None:
                self._fail_safe_recovery_start_time = now
            elif (
                now - self._fail_safe_recovery_start_time
            ) >= self.param.fail_safe_recovery_duration_sec:
                self.is_fail_safe.value = False
                return True
        return False

    def _error_log_output(self, err_idx: int, *args: object) -> None:
        self._logger.warning(
            self.get_error_no(err_idx)
            + ": GPU性能が低下しています。再起動で改善する場合があります。"
        )

    def _recover_log_output(self, err_idx: int, *args: object) -> None:
        self._logger.warning(self.get_error_no(err_idx) + ": GPU性能が回復しました。")

    def _fail_safe_recover_log_output(self, err_idx: int, *args: object) -> None:
        self._logger.warning(self.get_error_no(err_idx) + ": GPU性能が回復しました。")


class MonitorProcessNotRespondingDiagnosis(StateErrorDiagnosisA):
    """MONITOR_PROCESS_NOT_RESPONDING: Monitorプロセス未応答"""

    def __init__(self, runtime_policy: DiagnosisRuntimePolicy | None = None) -> None:
        super().__init__(runtime_policy)
        self._previous_heartbeat: float | None = None
        self._last_time: float | None = None
        self._heartbeat_tolerance_sec: float = 0.1
        self.param: err_conf.MonitorProcessNotRespondingParameters

    def _parse_args(self, *args: object) -> tuple[float, float | None]:
        """
        args[0] (float) : 現在時刻now
        args[1] (float) : 最後の取得時刻last_heartbeat
        """
        if len(args) != 2:
            raise ValueError("args must be (now, last_heartbeat)")
        now = args[0]
        last_heartbeat = args[1]

        if not isinstance(now, float) or (
            last_heartbeat is not None and not isinstance(last_heartbeat, float)
        ):
            raise ValueError("args must be float, float, int")
        return float(now), last_heartbeat

    def update(self, err_conf: err_conf.ErrorConfig) -> None:
        self.param: err_conf.MonitorProcessNotRespondingParameters = (
            err_conf.monitor_process_not_responding
        )
        self.is_enabled = self.param.is_enabled

    def detect_error(self, *args: object) -> bool:
        """
        最後の取得時刻が、現在から規定秒より前の場合にエラー(True)。
        """
        now, last_heartbeat = self._parse_args(*args)
        # メンテナンスモード中は監視プロセス停止を新規検出しない。
        if self.is_suppressed_in_maintenance:
            return False
        previous_heartbeat = self._previous_heartbeat

        is_stale = False
        is_jump_error = False

        if previous_heartbeat is None:
            # 初期値None。None継続は未更新として扱う。
            is_stale = last_heartbeat is None
        elif last_heartbeat is None:
            # 直前値があるのにNoneは未更新として扱う。
            is_stale = True
        else:
            heartbeat_delta = last_heartbeat - previous_heartbeat
            is_stale = abs(heartbeat_delta) < self._heartbeat_tolerance_sec
            # 前回値から閾値秒以上進んでいたら更新遅延としてエラー。
            is_jump_error = heartbeat_delta >= self.param.error_threshold_sec

        if is_stale:
            if self._last_time is None:
                self._last_time = now
            elif now - self._last_time >= self.param.error_threshold_sec:
                self.is_error.value = True
                self.is_fail_safe.value = True
                self.is_idle.value = True
                if last_heartbeat is not None:
                    self._previous_heartbeat = last_heartbeat
                return True
        else:
            self._last_time = None

        if is_jump_error:
            self.is_error.value = True
            self.is_fail_safe.value = True
            self.is_idle.value = True
            self._previous_heartbeat = last_heartbeat
            return True

        if last_heartbeat is not None:
            self._previous_heartbeat = last_heartbeat

        return False

    def detect_recovery_error(self, *args: object) -> bool:
        # 復帰条件が無いため、常にFalseを返す
        return False

    def detect_recovery_fail_safe(self, *args: object) -> bool:
        # 復帰条件が無いため、常にFalseを返す
        return False

    def _error_log_output(self, err_idx: int, *args: object) -> None:
        self._logger.error(
            self.get_error_no(err_idx)
            + ": No response from the system monitoring process. Restarting may help."
        )


class StatusInfoNotUpdatedDiagnosis(StateErrorDiagnosisA):
    """STATUS_INFO_NOT_UPDATED: ステータス情報　未更新"""

    def __init__(self) -> None:
        super().__init__()

    def detect_error(self, *args: object) -> bool:
        return False

    def detect_recovery_error(self, *args: object) -> bool:
        return True

    def detect_recovery_fail_safe(self, *args: object) -> bool:
        return True


class SurroundMonitorModuleNotRespondingDiagnosis(StateErrorDiagnosisA):
    """SURROUND_MONITOR_MODULE_NOT_RESPONDING: 周辺監視モジュール 未応答"""

    def __init__(self, runtime_policy: DiagnosisRuntimePolicy | None = None) -> None:
        super().__init__(runtime_policy)
        self._last_max_elapsed_sec: float = 0.0
        self.param: err_conf.SurroundMonitorModuleNotRespondingParameters

    def clear(self) -> None:
        self._last_max_elapsed_sec = 0.0

    def _parse_args(self, *args: object) -> list[float]:
        if len(args) != 1 or not isinstance(args[0], (list, tuple)):
            raise ValueError("args must be (elapsed_list,)")
        elapsed_list = args[0]
        if not elapsed_list:
            raise ValueError("elapsed_list must not be empty")
        if not all(isinstance(elapsed, float) for elapsed in elapsed_list):
            raise ValueError("elapsed_list element must be float")
        return list(elapsed_list)

    def _evaluate_elapsed(self, *args: object) -> bool:
        elapsed_list = self._parse_args(*args)
        active_elapsed_list = [elapsed for elapsed in elapsed_list if elapsed >= 0.0]
        if not active_elapsed_list:
            self._last_max_elapsed_sec = 0.0
            return False
        self._last_max_elapsed_sec = max(active_elapsed_list)
        return any(
            elapsed > self.param.error_threshold_sec
            for elapsed in active_elapsed_list
        )

    def update(self, err_conf: err_conf.ErrorConfig) -> None:
        self.param = err_conf.surround_monitor_module_not_responding
        self.is_enabled = self.param.is_enabled

    def detect_error(self, *args: object) -> bool:
        is_any_not_responding = self._evaluate_elapsed(*args)
        # メンテナンスモード中は周辺監視モジュール停止を新規検出しない。
        if self.is_suppressed_in_maintenance:
            return False
        if is_any_not_responding:
            self.is_error.value = True
            self.is_fail_safe.value = True
            return True
        return False

    def detect_recovery_error(self, *args: object) -> bool:
        if self._evaluate_elapsed(*args):
            return False
        if self.is_error.value:
            self.is_error.value = False
            return True
        return False

    def detect_recovery_fail_safe(self, *args: object) -> bool:
        if self._evaluate_elapsed(*args):
            return False
        if self.is_fail_safe.value:
            self.is_fail_safe.value = False
            return True
        return False

    def _error_log_output(self, err_idx: int, *args: object) -> None:
        self._logger.error(
            self.get_error_no(err_idx)
            + ": SURROUND_MONITOR_MODULE_NOT_RESPONDING detected: max_elapsed=%.3f sec",
            self._last_max_elapsed_sec,
        )

    def _recover_log_output(self, err_idx: int, *args: object) -> None:
        self._logger.info(
            self.get_error_no(err_idx)
            + ": SURROUND_MONITOR_MODULE_NOT_RESPONDING recovered"
        )

    def _fail_safe_recover_log_output(self, err_idx: int, *args: object) -> None:
        self._logger.info(
            self.get_error_no(err_idx)
            + ": SURROUND_MONITOR_MODULE_NOT_RESPONDING failsafe recovered"
        )


class LidarPositionMisalignmentNotRespondingDiagnosis(StateErrorDiagnosisB):
    """LIDAR_POSITION_MISALIGNMENT_NOT_RESPONDING: Lidar位置ズレ検出 未応答"""

    def __init__(self) -> None:
        super().__init__()
        self._error_recovery_start_time: float | None = None
        self._fail_safe_recovery_start_time: float | None = None
        self._previous_heartbeat: float | None = None
        self._recovery_prev_timestamp = 0.0
        self._fail_safe_prev_timestamp = 0.0
        self.param: err_conf.LidarPositionMisalignmentNotRespondingParameters

    def clear(self) -> None:
        self._previous_heartbeat = None
        self._recovery_prev_timestamp = 0.0
        self._fail_safe_prev_timestamp = 0.0
        self._error_recovery_start_time = None
        self._fail_safe_recovery_start_time = None

    def _parse_args(self, *args: object) -> tuple[float, float]:
        """
        args[0] (float) : 現在時刻now
        args[1] (float) : 最後の取得時刻last_heartbeat
        """
        if len(args) != 2:
            raise ValueError("args must be (now, last_heartbeat)")
        now = args[0]
        last_heartbeat = args[1]

        if not isinstance(now, float) or not isinstance(last_heartbeat, float):
            raise ValueError("args must be float, float")

        return float(now), float(last_heartbeat)

    def update(self, err_conf: err_conf.ErrorConfig) -> None:
        self.param = err_conf.lidar_position_misalignment_not_responding
        self.is_enabled = self.param.is_enabled

    def detect_error(self, *args: object) -> bool:
        """
        最後の取得時刻が、現在から規定秒より前の場合にエラー(True)。
        """
        now, last_heartbeat = self._parse_args(*args)
        if self._previous_heartbeat is None or last_heartbeat < 0.0:
            self._previous_heartbeat = last_heartbeat
            return False

        # 前回のハートビートから規定秒以上経過している、または最後のハートビートから規定秒以上経過している場合にエラーとする
        is_error = (
            last_heartbeat - self._previous_heartbeat >= self.param.error_threshold_sec
            or now - last_heartbeat >= self.param.error_threshold_sec
        )

        self._previous_heartbeat = last_heartbeat

        if is_error:
            self.is_error.value = True
            self.is_fail_safe.value = True
            return True
        return False

    def detect_recovery_error(self, *args: object) -> bool:
        now, last_heartbeat = self._parse_args(*args)
        if last_heartbeat < 0.0:
            return False

        interval = last_heartbeat - self._recovery_prev_timestamp
        self._recovery_prev_timestamp = last_heartbeat
        if (
            interval > self.param.recovery_receive_interval_sec
            or now - last_heartbeat > self.param.recovery_receive_interval_sec
        ):
            self._error_recovery_start_time = None
            return False

        if self._error_recovery_start_time is None:
            self._error_recovery_start_time = last_heartbeat
        elif (
            last_heartbeat - self._error_recovery_start_time
            >= self.param.error_recovery_confirm_duration_sec
        ):
            self._error_recovery_start_time = None
            self.is_error.value = False
            return True
        return False

    def detect_recovery_fail_safe(self, *args: object) -> bool:
        now, last_heartbeat = self._parse_args(*args)
        if last_heartbeat < 0.0:
            return False

        interval = last_heartbeat - self._fail_safe_prev_timestamp
        self._fail_safe_prev_timestamp = last_heartbeat
        if (
            interval > self.param.recovery_receive_interval_sec
            or now - last_heartbeat > self.param.recovery_receive_interval_sec
        ):
            self._fail_safe_recovery_start_time = None
            return False

        if self._fail_safe_recovery_start_time is None:
            self._fail_safe_recovery_start_time = last_heartbeat
        elif (
            last_heartbeat - self._fail_safe_recovery_start_time
            >= self.param.failsafe_recovery_confirm_duration_sec
        ):
            self._fail_safe_recovery_start_time = None
            self.is_fail_safe.value = False
            return True
        return False

    def _error_log_output(self, err_idx: int, *args: object) -> None:
        self._logger.warning(
            self.get_error_no(err_idx)
            + f": Lidar position misalignment detection process has not responded for {self.param.error_threshold_sec} seconds. Restarting may help."
        )

    def _recover_log_output(self, err_idx: int, *args: object) -> None:
        self._logger.warning(
            self.get_error_no(err_idx)
            + ": Lidar position misalignment detection process has recovered."
        )


class ApplicationManagerNotRespondingDiagnosis(StateErrorDiagnosisA):
    """APPLICATION_MANAGER_NOT_RESPONDING: アプリケーションマネージャー未応答"""

    def __init__(self, runtime_policy: DiagnosisRuntimePolicy | None = None) -> None:
        super().__init__(runtime_policy)
        self._previous_heartbeat: float | None = None
        self._last_time: float | None = None
        self._heartbeat_tolerance_sec: float = 0.01
        self._detected_in_current_cycle: bool = False
        self.param: err_conf.ApplicationManagerNotRespondingParameters

    def clear(self) -> None:
        self._previous_heartbeat = None
        self._last_time = None
        self._detected_in_current_cycle = False

    def _parse_args(self, *args: object) -> tuple[float, float | None]:
        if len(args) != 2:
            raise ValueError("args must be (now, last_heartbeat)")
        now = args[0]
        last_heartbeat = args[1]
        if not isinstance(now, float) or (
            last_heartbeat is not None and not isinstance(last_heartbeat, float)
        ):
            raise ValueError("args must be float, float | None")
        return now, last_heartbeat

    def update(self, err_conf: err_conf.ErrorConfig) -> None:
        self.param = err_conf.application_manager_not_responding
        self.is_enabled = self.param.is_enabled

    def detect_error(self, *args: object) -> bool:
        now, last_heartbeat = self._parse_args(*args)
        self._detected_in_current_cycle = False
        # メンテナンスモード中はAppManager停止を新規検出しない。
        if self.is_suppressed_in_maintenance:
            return False
        previous_heartbeat = self._previous_heartbeat

        if last_heartbeat is None or last_heartbeat <= 0.0:
            is_stale = True
            is_jump_error = False
        elif previous_heartbeat is None or previous_heartbeat <= 0.0:
            self._previous_heartbeat = last_heartbeat
            self._last_time = None
            return False
        else:
            heartbeat_delta = last_heartbeat - previous_heartbeat
            is_stale = abs(heartbeat_delta) < self._heartbeat_tolerance_sec
            is_jump_error = heartbeat_delta <= -self.param.error_threshold_sec

        if is_stale:
            if self._last_time is None:
                self._last_time = now
            elif now - self._last_time >= self.param.error_threshold_sec:
                self.is_error.value = True
                self.is_fail_safe.value = True
                self._detected_in_current_cycle = True
                if last_heartbeat is not None:
                    self._previous_heartbeat = last_heartbeat
                return True
        else:
            self._last_time = None

        if is_jump_error:
            self.is_error.value = True
            self.is_fail_safe.value = True
            self._detected_in_current_cycle = True
            self._previous_heartbeat = last_heartbeat
            return True

        if last_heartbeat is not None:
            self._previous_heartbeat = last_heartbeat
        return False

    def detect_recovery_error(self, *args: object) -> bool:
        if self._detected_in_current_cycle:
            return False
        now, last_heartbeat = self._parse_args(*args)
        if last_heartbeat is None:
            return False
        if now - last_heartbeat <= self.param.error_threshold_sec and self.is_error.value:
            self.is_error.value = False
            return True
        return False

    def detect_recovery_fail_safe(self, *args: object) -> bool:
        if self._detected_in_current_cycle:
            return False
        now, last_heartbeat = self._parse_args(*args)
        if last_heartbeat is None:
            return False
        if (
            now - last_heartbeat <= self.param.error_threshold_sec
            and self.is_fail_safe.value
        ):
            self.is_fail_safe.value = False
            return True
        return False

    def _error_log_output(self, err_idx: int, *args: object) -> None:
        self._logger.error(
            self.get_error_no(err_idx) + ": APPLICATION_MANAGER_NOT_RESPONDING detected"
        )

    def _recover_log_output(self, err_idx: int, *args: object) -> None:
        self._logger.info(
            self.get_error_no(err_idx)
            + ": APPLICATION_MANAGER_NOT_RESPONDING recovered"
        )

    def _fail_safe_recover_log_output(self, err_idx: int, *args: object) -> None:
        self._logger.info(
            self.get_error_no(err_idx)
            + ": APPLICATION_MANAGER_NOT_RESPONDING failsafe recovered"
        )


class ImuNConnectionErrorDiagnosis(StateErrorDiagnosisB):
    """IMU_N_CONNECTION_ERROR: imuN接続エラー"""

    def __init__(self) -> None:
        super().__init__()
        self.param: err_conf.ImuNConnectionErrorParameters
        self.clear()

    def clear(self) -> None:
        self._previous_heartbeat: float | None = None
        self._recovery_prev_timestamp = -1.0
        self._fail_safe_prev_timestamp = -1.0
        self._error_recovery_start_time: float | None = None
        self._fail_safe_recovery_start_time: float | None = None

    def update(self, error_config: err_conf.ErrorConfig) -> None:
        self.param = error_config.imu_n_connection_error
        self.is_enabled = self.param.is_enabled

    def _parse_args(self, *args: object) -> tuple[float, float]:
        if len(args) != 2 or not all(isinstance(value, float) for value in args):
            raise ValueError("args must be (now, last_heartbeat) as float")
        return args[0], args[1]  # type: ignore[return-value]

    def detect_error(self, *args: object) -> bool:
        now, last_heartbeat = self._parse_args(*args)
        if self._previous_heartbeat is None or last_heartbeat < 0.0:
            self._previous_heartbeat = last_heartbeat
            return False
        is_error = (
            last_heartbeat - self._previous_heartbeat >= self.param.error_threshold_sec
            or now - last_heartbeat >= self.param.error_threshold_sec
        )
        self._previous_heartbeat = last_heartbeat
        if is_error:
            self.is_error.value = True
            self.is_fail_safe.value = True
            return True
        return False

    def detect_recovery_error(self, *args: object) -> bool:
        now, last_heartbeat = self._parse_args(*args)
        if last_heartbeat < 0.0:
            return False
        interval = last_heartbeat - self._recovery_prev_timestamp
        self._recovery_prev_timestamp = last_heartbeat
        if (
            interval > self.param.recovery_receive_interval_sec
            or now - last_heartbeat > self.param.recovery_receive_interval_sec
        ):
            self._error_recovery_start_time = None
            return False
        if self._error_recovery_start_time is None:
            self._error_recovery_start_time = last_heartbeat
        elif (
            last_heartbeat - self._error_recovery_start_time
            >= self.param.error_recovery_confirm_duration_sec
        ):
            self._error_recovery_start_time = None
            self.is_error.value = False
            return True
        return False

    def detect_recovery_fail_safe(self, *args: object) -> bool:
        now, last_heartbeat = self._parse_args(*args)
        if last_heartbeat < 0.0:
            return False
        interval = last_heartbeat - self._fail_safe_prev_timestamp
        self._fail_safe_prev_timestamp = last_heartbeat
        if (
            interval > self.param.recovery_receive_interval_sec
            or now - last_heartbeat > self.param.recovery_receive_interval_sec
        ):
            self._fail_safe_recovery_start_time = None
            return False
        if self._fail_safe_recovery_start_time is None:
            self._fail_safe_recovery_start_time = last_heartbeat
        elif (
            last_heartbeat - self._fail_safe_recovery_start_time
            >= self.param.failsafe_recovery_confirm_duration_sec
        ):
            self._fail_safe_recovery_start_time = None
            self.is_fail_safe.value = False
            return True
        return False

    def _error_log_output(self, err_idx: int, *args: object) -> None:
        index = _sensor_index(args)
        self._logger.warning(
            self.get_error_no(err_idx) + f": IMU[{index}] connection error detected."
        )

    def _recover_log_output(self, err_idx: int, *args: object) -> None:
        index = _sensor_index(args)
        self._logger.info(
            self.get_error_no(err_idx) + f": IMU[{index}] connection error recovered."
        )

    def _fail_safe_recover_log_output(self, err_idx: int, *args: object) -> None:
        index = _sensor_index(args)
        self._logger.info(
            self.get_error_no(err_idx)
            + f": IMU[{index}] connection error recovered from failsafe."
        )


class LogOutputStoppedDiagnosis(StateErrorDiagnosisC):
    """LOG_OUTPUT_STOPPED: ログ出力停止"""

    def __init__(self) -> None:
        super().__init__()
        self._error_recovery_start_time: float | None = None
        self._fail_safe_recovery_start_time: float | None = None
        self.param: err_conf.LogOutputStoppedParameters

    def _parse_args(self, *args: object) -> tuple[float, float]:
        if len(args) != 2:
            raise ValueError("args must be (last_update_mono, now_mono)")
        last_update_mono = args[0]
        now_mono = args[1]
        if not isinstance(last_update_mono, float) or not isinstance(now_mono, float):
            raise ValueError("args must be float, float")
        return last_update_mono, now_mono

    def update(self, err_conf: err_conf.ErrorConfig) -> None:
        self.param = err_conf.log_output_stopped
        self.is_enabled = self.param.is_enabled

    def detect_error(self, *args: object) -> bool:
        last_update_mono, now_mono = self._parse_args(*args)
        elapsed = max(now_mono - last_update_mono, 0.0)
        if elapsed >= self.param.error_threshold_sec:
            self.is_error.value = True
            self.is_fail_safe.value = True
            return True
        return False

    def detect_recovery_error(self, *args: object) -> bool:
        last_update_mono, now_mono = self._parse_args(*args)
        elapsed = max(now_mono - last_update_mono, 0.0)
        if elapsed > self.param.recovery_receive_interval_sec:
            self._error_recovery_start_time = None
            return False
        if self._error_recovery_start_time is None:
            self._error_recovery_start_time = now_mono
            return False
        if (
            now_mono - self._error_recovery_start_time
            >= self.param.error_recovery_confirm_duration_sec
        ):
            self._error_recovery_start_time = None
            self.is_error.value = False
            return True
        return False

    def detect_recovery_fail_safe(self, *args: object) -> bool:
        last_update_mono, now_mono = self._parse_args(*args)
        elapsed = max(now_mono - last_update_mono, 0.0)
        if elapsed > self.param.recovery_receive_interval_sec:
            self._fail_safe_recovery_start_time = None
            return False
        if self._fail_safe_recovery_start_time is None:
            self._fail_safe_recovery_start_time = now_mono
            return False
        if (
            now_mono - self._fail_safe_recovery_start_time
            >= self.param.failsafe_recovery_confirm_duration_sec
        ):
            self._fail_safe_recovery_start_time = None
            self.is_fail_safe.value = False
            return True
        return False

    def _error_log_output(self, err_idx: int, *args: object) -> None:
        self._logger.error(
            self.get_error_no(err_idx)
            + ": LOG_OUTPUT_STOPPED detected: file log output has stopped"
        )

    def _recover_log_output(self, err_idx: int, *args: object) -> None:
        self._logger.info(
            self.get_error_no(err_idx)
            + ": LOG_OUTPUT_STOPPED recovered: file log output resumed"
        )

    def _fail_safe_recover_log_output(self, err_idx: int, *args: object) -> None:
        self._logger.info(
            self.get_error_no(err_idx) + ": LOG_OUTPUT_STOPPED failsafe recovered"
        )


class InternalTemperatureRiseDiagnosis(StateErrorDiagnosisB):
    """INTERNAL_TEMPERATURE_RISE: 内部温度上昇"""

    INVALID_TEMPERATURE: float = -273.15  # 絶対零度以下の温度は無効値とみなす

    def __init__(self) -> None:
        super().__init__()
        self.param: err_conf.TemperatureRiseTrendContinuesParameters
        _tj: deque[tuple[float, float]] = deque()
        _tc: deque[tuple[float, float]] = deque()
        _tg: deque[tuple[float, float]] = deque()
        self._temp_deques: tuple[deque[tuple[float, float]], ...] = (
            _tj,
            _tc,
            _tg,
        )
        self.moving_averages: list[float] = [
            self.INVALID_TEMPERATURE,
            self.INVALID_TEMPERATURE,
            self.INVALID_TEMPERATURE,
        ]
        self._is_average_ready = False

    def update(self, err_conf: err_conf.ErrorConfig) -> None:
        self.param = err_conf.temperature_rise_trend_continues
        self.is_enabled = self.param.is_enabled

    def _parse_args(
        self, *args: object
    ) -> tuple[float, tuple[float | None, float | None, float | None]]:
        """
        args[0] (float) : 現在時刻now
        args[1] (float) : TJ温度tj
        args[2] (float) : TC温度tc
        args[3] (float) : TG温度tg
        # args[4] (float) : TS温度ts
        """
        if len(args) != 4:
            raise ValueError("args must be (now, tj, tc, tg)")
        now = args[0]
        tj = args[1]
        tc = args[2]
        tg = args[3]

        if (
            not isinstance(now, float)
            or not isinstance(tj, float | None)
            or not isinstance(tc, float | None)
            or not isinstance(tg, float | None)
        ):
            raise ValueError(
                "args must be float, float | None, float | None, float | None"
            )

        return float(now), (tj, tc, tg)

    def _calc_moving_average(
        self, now: float, temp: float, temp_deque: deque[tuple[float, float]]
    ) -> float | None:
        """
        移動平均の算出
        サンプル時間以上のデータが溜まっていれば移動平均値を返す
        """
        if temp < self.INVALID_TEMPERATURE:
            if len(temp_deque) == 0:
                return None
        else:
            temp_deque.append((now, temp))
        while now - temp_deque[0][0] > self.param.moving_avg_window_sec:
            temp_deque.popleft()
            self._is_average_ready = True

        if self._is_average_ready and len(temp_deque) >= self.param.least_sample_count:
            ma = sum(t for _, t in temp_deque) / len(temp_deque)
        else:
            ma = None

        return ma

    def detect_error(self, *args: object) -> bool:
        now, temp = self._parse_args(*args)
        results: list[bool] = []
        for i, t in enumerate(temp):
            if t is None or t < self.INVALID_TEMPERATURE:
                continue
            ma: float | None = self._calc_moving_average(now, t, self._temp_deques[i])
            if ma is None:
                self.moving_averages[i] = self.INVALID_TEMPERATURE
                return False
            results.append(ma > self.param.error_threshold_degree)
            self.moving_averages[i] = ma
        # デバッグ用に移動平均を出力
        self._logger.debug(f"Moving averages: {self.moving_averages}")

        if any(results):
            self.is_error.value = True
            self.is_fail_safe.value = True
            return True
        return False

    def detect_recovery_error(self, *args: object) -> bool:
        now, temp = self._parse_args(*args)
        results: list[bool] = []
        for i, t in enumerate(temp):
            if t is None or t < self.INVALID_TEMPERATURE:
                continue
            ma: float | None = self._calc_moving_average(now, t, self._temp_deques[i])
            if ma is None:
                self.moving_averages[i] = self.INVALID_TEMPERATURE
                return False
            results.append(ma <= self.param.error_threshold_degree)
            self.moving_averages[i] = ma

        if all(results):
            self.is_error.value = False
            return True
        return False

    def detect_recovery_fail_safe(self, *args: object) -> bool:
        now, temp = self._parse_args(*args)
        results: list[bool] = []
        for i, t in enumerate(temp):
            if t is None or t < self.INVALID_TEMPERATURE:
                continue
            ma: float | None = self._calc_moving_average(now, t, self._temp_deques[i])
            if ma is None:
                self.moving_averages[i] = self.INVALID_TEMPERATURE
                return False
            results.append(ma <= self.param.error_threshold_degree)
            self.moving_averages[i] = ma

        if all(results):
            self.is_fail_safe.value = False
            return True
        return False

    def _error_log_output(self, err_idx: int, *args: object) -> None:
        self._logger.warning(
            self.get_error_no(err_idx)
            + f": 内部温度上昇: {f'tj:{self.moving_averages[0]:.1f}℃,tc:{self.moving_averages[1]:.1f}℃,tg:{self.moving_averages[2]:.1f}℃'}"
        )

    def _recover_log_output(self, err_idx: int, *args: object) -> None:
        self._logger.warning(
            self.get_error_no(err_idx)
            + f": 内部温度が回復: {f'tj:{self.moving_averages[0]:.1f}℃,tc:{self.moving_averages[1]:.1f}℃,tg:{self.moving_averages[2]:.1f}℃'}"
        )

    def _fail_safe_recover_log_output(self, err_idx: int, *args: object) -> None:
        self._logger.warning(
            self.get_error_no(err_idx)
            + f": 内部温度が回復: {f'tj:{self.moving_averages[0]:.1f}℃,tc:{self.moving_averages[1]:.1f}℃,tg:{self.moving_averages[2]:.1f}℃'}"
        )


class TemperatureSensorAbnormalDiagnosis(StateErrorDiagnosisB):
    """TEMPERATURE_SENSOR_ABNORMAL: 温度センサ異常"""

    INVALID_TEMPERATURE: float = -273.15  # 絶対零度以下の温度は無効値とみなす

    def __init__(self) -> None:
        super().__init__()
        self.param: err_conf.TemperatureSensorAbnormalParameters
        self._start_time: float | None = None
        self._error_recovery_start_time: float | None = None
        self._fail_safe_recovery_start_time: float | None = None

    def update(self, err_conf: err_conf.ErrorConfig) -> None:
        self.param = err_conf.temperature_sensor_abnormal
        self.is_enabled = self.param.is_enabled

    def _parse_args(self, *args: object) -> tuple[float, tuple[object, object, object]]:
        """
        args[0] (float) : 現在時刻now
        args[1] (float) : TJ温度tj
        args[2] (float) : TC温度tc
        args[3] (float) : TG温度tg
        """
        if len(args) != 4:
            raise ValueError("args must be (now, tj, tc, tg)")
        now = args[0]
        tj = args[1]
        tc = args[2]
        tg = args[3]

        if not isinstance(now, float):
            raise ValueError("args must be float, object, object, object")

        return float(now), (tj, tc, tg)

    def detect_error(self, *args: object) -> bool:
        now, temp = self._parse_args(*args)
        if all(
            isinstance(t, (int, float)) and float(t) > self.INVALID_TEMPERATURE
            for t in temp
        ):
            self._start_time = None
            return False

        if self._start_time is None:
            self._start_time = now
        elif now - self._start_time >= self.param.error_threshold_sec:
            self.is_error.value = True
            self.is_fail_safe.value = True
            return True
        return False

    def detect_recovery_error(self, *args: object) -> bool:
        now, temp = self._parse_args(*args)
        if all(
            isinstance(t, (int, float)) and float(t) > self.INVALID_TEMPERATURE
            for t in temp
        ):
            if self._error_recovery_start_time is None:
                self._error_recovery_start_time = now
            elif (
                now - self._error_recovery_start_time
                >= self.param.error_recovery_duration_sec
            ):
                self.is_error.value = False
                return True
            return False

        self._error_recovery_start_time = None
        return False

    def detect_recovery_fail_safe(self, *args: object) -> bool:
        now, temp = self._parse_args(*args)
        if all(
            isinstance(t, (int, float)) and float(t) > self.INVALID_TEMPERATURE
            for t in temp
        ):
            if self._fail_safe_recovery_start_time is None:
                self._fail_safe_recovery_start_time = now
            elif (
                now - self._fail_safe_recovery_start_time
                >= self.param.fail_safe_recovery_duration_sec
            ):
                self.is_fail_safe.value = False
                return True
            return False

        self._fail_safe_recovery_start_time = None
        return False

    def _error_log_output(self, err_idx: int, *args: object) -> None:
        self._logger.warning(
            self.get_error_no(err_idx)
            + ": 温度センサ異常: 温度センサから温度が取得できません"
        )

    def _recover_log_output(self, err_idx: int, *args: object) -> None:
        self._logger.warning(
            self.get_error_no(err_idx)
            + ": 温度センサ異常: 復帰しました。温度センサから温度が取得できるようになりました。"
        )

    def _fail_safe_recover_log_output(self, err_idx: int, *args: object) -> None:
        self._logger.warning(
            self.get_error_no(err_idx)
            + ": 温度センサ異常: 復帰しました。温度センサから温度が取得できるようになりました。"
        )


class TemperatureRiseTrendContinuesDiagnosis(StateErrorDiagnosisA):
    """TEMPERATURE_RISE_TREND_CONTINUES: 温度上昇傾向の継続"""

    INVALID_TEMPERATURE: float = -273.15  # 絶対零度以下の温度は無効値とみなす

    def __init__(self) -> None:
        super().__init__()
        self.param: err_conf.TemperatureRiseTrendContinuesParameters
        _tj: deque[tuple[float, float]] = deque()
        _tc: deque[tuple[float, float]] = deque()
        _tg: deque[tuple[float, float]] = deque()
        self._temp_deques: tuple[deque[tuple[float, float]], ...] = (
            _tj,
            _tc,
            _tg,
        )
        self.moving_averages: list[float] = [
            self.INVALID_TEMPERATURE,
            self.INVALID_TEMPERATURE,
            self.INVALID_TEMPERATURE,
        ]
        self._is_average_ready = False

    def update(self, err_conf: err_conf.ErrorConfig) -> None:
        self.param = err_conf.temperature_rise_trend_continues
        self.is_enabled = self.param.is_enabled

    def _parse_args(
        self, *args: object
    ) -> tuple[float, tuple[float | None, float | None, float | None]]:
        """
        args[0] (float) : 現在時刻now
        args[1] (float) : TJ温度tj
        args[2] (float) : TC温度tc
        args[3] (float) : TG温度tg
        # args[4] (float) : TS温度ts
        """
        if len(args) != 4:
            raise ValueError("args must be (now, tj, tc, tg)")
        now = args[0]
        tj = args[1]
        tc = args[2]
        tg = args[3]

        if (
            not isinstance(now, float)
            or not isinstance(tj, float | None)
            or not isinstance(tc, float | None)
            or not isinstance(tg, float | None)
        ):
            raise ValueError("args must be float, float, float, float")

        return float(now), (tj, tc, tg)

    def _calc_moving_average(
        self, now: float, temp: float, temp_deque: deque[tuple[float, float]]
    ) -> float | None:
        """
        移動平均の算出
        サンプル時間以上のデータが溜まっていれば移動平均値を返す
        """
        if temp < self.INVALID_TEMPERATURE:
            if len(temp_deque) == 0:
                return None
        else:
            temp_deque.append((now, temp))
        while now - temp_deque[0][0] > self.param.moving_avg_window_sec:
            temp_deque.popleft()
            self._is_average_ready = True

        if self._is_average_ready and len(temp_deque) >= self.param.least_sample_count:
            ma = sum(t for _, t in temp_deque) / len(temp_deque)
        else:
            ma = None

        return ma

    def detect_error(self, *args: object) -> bool:
        now, temp = self._parse_args(*args)
        results: list[bool] = []
        for i, t in enumerate(temp):
            if t is None or t < self.INVALID_TEMPERATURE:
                continue
            ma: float | None = self._calc_moving_average(now, t, self._temp_deques[i])
            if ma is None:
                self.moving_averages[i] = self.INVALID_TEMPERATURE
                return False
            results.append(ma > self.param.error_threshold_degree)
            self.moving_averages[i] = ma
        # デバッグ用に移動平均を出力
        self._logger.debug(f"Moving averages: {self.moving_averages}")

        if any(results):
            self.is_error.value = True
            self.is_fail_safe.value = True
            self.is_idle.value = True
            return True
        return False

    def detect_recovery_error(self, *args: object) -> bool:
        # 復帰条件が無いため、常にFalseを返す
        return False

    def detect_recovery_fail_safe(self, *args: object) -> bool:
        # 復帰条件が無いため、常にFalseを返す
        return False

    def _error_log_output(self, err_idx: int, *args: object) -> None:
        self._logger.error(
            self.get_error_no(err_idx)
            + f": 温度上昇傾向の継続: {f'tj:{self.moving_averages[0]:.1f}℃,tc:{self.moving_averages[1]:.1f}℃,tg:{self.moving_averages[2]:.1f}℃'}"
        )


class CalibHumanDetectionFailureDiagnosis(StateErrorDiagnosisB):
    """CALIB_HUMAN_DETECTION_FAILURE: 校正時: 人検知不良"""

    def __init__(self) -> None:
        super().__init__()

    def detect_error(self, *args: object) -> bool:
        return False

    def detect_recovery_error(self, *args: object) -> bool:
        return True

    def detect_recovery_fail_safe(self, *args: object) -> bool:
        return True


class CalibHumanTrackingFailureDiagnosis(StateErrorDiagnosisB):
    """CALIB_HUMAN_TRACKING_FAILURE: 校正時: 人検知追跡不良"""

    def __init__(self) -> None:
        super().__init__()

    def detect_error(self, *args: object) -> bool:
        return False

    def detect_recovery_error(self, *args: object) -> bool:
        return True

    def detect_recovery_fail_safe(self, *args: object) -> bool:
        return True


class ReserveStateABC(StateErrorDiagnosisC):
    def __init__(self) -> None:
        super().__init__()

    def detect_error(self, *args: object) -> bool:
        return False

    def detect_recovery_error(self, *args: object) -> bool:
        return True

    def detect_recovery_fail_safe(self, *args: object) -> bool:
        return True
