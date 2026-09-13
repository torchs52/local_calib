from __future__ import annotations

import subprocess
import time
from collections import deque
from collections.abc import Callable
from enum import Enum, StrEnum, auto
from pathlib import Path
from typing import cast

import numpy as np
from numpy.typing import NDArray

import argus_synchro.diagnosis.error_config as err_conf
from argus_synchro.diagnosis.error_diagnosis import (
    StateErrorDiagnosisD,
)
from argus_synchro.process.process import ProcessBase
from argus_synchro.shared_data import create_shared_single_data


class LidarDataMissing(StateErrorDiagnosisD):
    """LIDAR_DATA_MISSING: Lidarデータ欠落"""

    def __init__(self) -> None:
        super().__init__()
        self.param: err_conf.LidarDataMissingParameters

    def update(self, err_conf: err_conf.ErrorConfig) -> None:
        self.param = err_conf.lidar_data_missing
        self.is_enabled = self.param.is_enabled

    def _parse_args(self, *args: object) -> tuple[int, bool]:
        if len(args) != 2:
            raise ValueError("args must be (point_count, is_connection_error)")

        point_count = args[0]
        is_connection_error = args[1]
        if not isinstance(point_count, int):
            raise ValueError("args[0] must be int")
        if not isinstance(is_connection_error, bool):
            raise ValueError("args[1] must be bool")
        return point_count, is_connection_error

    def detect_error(self, *args: object) -> bool:
        point_count, is_connection_error = self._parse_args(*args)
        if is_connection_error:
            return False
        return 0 < point_count < self.param.min_point_count

    def _error_log_output(self, err_idx: int, *args: object) -> None:
        if len(args) != 1:
            raise ValueError("args must be (index,)")
        if not isinstance(args[0], int):
            raise ValueError("args[0] must be int")
        index = args[0]
        self._logger.info(
            self.get_error_no(err_idx)
            + f": Lidarデータ欠落: Lidar{index}の蓄積点数が閾値を下回りました。"
        )


class AiInferenceResultError(StateErrorDiagnosisD):
    """AI推論結果異常（ログのみ）"""

    def __init__(self) -> None:
        super().__init__()
        self.param: err_conf.AiInferenceResultErrorParameters

    def update(self, err_conf: err_conf.ErrorConfig) -> None:
        self.param = err_conf.ai_inference_result_error
        self.is_enabled = self.param.is_enabled

    def excepts_diagnosis(self, e: Exception) -> bool:
        if not self.is_enabled:
            return False
        return not isinstance(e, (KeyboardInterrupt, SystemExit))

    def _parse_args(
        self, *args: object
    ) -> tuple[NDArray[np.float32], NDArray[np.float32], NDArray[np.int32]]:
        if len(args) != 3:
            raise ValueError("args must be (boxes, scores, valid_detects)")
        boxes, scores, valid_detects = args
        if not isinstance(boxes, np.ndarray):
            raise ValueError("args[0] must be NDArray")
        if not isinstance(scores, np.ndarray):
            raise ValueError("args[1] must be NDArray")
        if not isinstance(valid_detects, np.ndarray):
            raise ValueError("args[2] must be NDArray")
        return (
            cast(NDArray[np.float32], boxes),
            cast(NDArray[np.float32], scores),
            cast(NDArray[np.int32], valid_detects),
        )

    def detect_error(self, *args: object) -> bool:
        boxes, scores, valid_detects = self._parse_args(*args)
        if not np.all(np.isfinite(boxes)):
            return True
        if not np.all(np.isfinite(scores)):
            return True
        if np.any((scores < self.param.score_min) | (scores > self.param.score_max)):
            return True
        return bool(
            np.any(valid_detects < 0) or np.any(valid_detects > boxes.shape[1])
        )

    def _error_log_output(self, err_idx: int, *args: object) -> None:
        if len(args) != 1:
            raise ValueError("args must be (index_or_exception,)")
        if isinstance(args[0], Exception):
            error = args[0]
            self._logger.warning(
                f"D-level exception: {type(error).__name__}: {error}"
            )
            return
        if not isinstance(args[0], int):
            raise ValueError("args[0] must be int or Exception")
        self._logger.warning(
            self.get_error_no(err_idx)
            + f": AI推論結果異常: カメラ{args[0]}の推論結果にNaN/Inf/範囲外スコアを検出しました。"
        )


class CameraDataMissing(StateErrorDiagnosisD):
    """CAMERA_DATA_MISSING: カメラデータ欠落"""

    def __init__(self) -> None:
        super().__init__()
        self._read_history: deque[tuple[float, bool]] = deque()
        self._black_count_in_window: int = 0
        self.param: err_conf.CameraDataMissingParameters

    def update(self, err_conf: err_conf.ErrorConfig) -> None:
        self.param = err_conf.camera_data_missing
        self.is_enabled = self.param.is_enabled

    def _parse_args(self, *args: object) -> tuple[float, NDArray[np.uint8]]:
        """
        args[0] (float): 現在時刻timestamp
        args[1] (NDArray[np.uint8]) : 画像データimage
        """
        if len(args) != 2:
            raise ValueError("args must be (timestamp, image)")

        timestamp = args[0]
        image = args[1]

        if not isinstance(timestamp, float):
            raise ValueError("args[0] must be float")
        if not isinstance(image, np.ndarray) or image.dtype != np.uint8:
            raise ValueError("args[1] must be NDArray[np.uint8]")

        return float(timestamp), cast(NDArray[np.uint8], image)

    def detect_error(self, *args: object) -> bool:
        """
        直近black_frame_rate_window_sec秒のフレームのうち、
        真っ黒フレームの割合がblack_frame_rate_threshold以上でエラー(True)。
        """
        timestamp, image = self._parse_args(*args)
        is_black = not image.any()
        history = self._read_history
        history.append((timestamp, is_black))
        if is_black:
            self._black_count_in_window += 1

        window_sec = self.param.black_frame_rate_window_sec
        threshold = self.param.black_frame_rate_threshold
        cutoff = timestamp - window_sec
        while history and history[0][0] < cutoff:
            _, removed_is_black = history.popleft()
            if removed_is_black:
                self._black_count_in_window -= 1

        total_count = len(history)
        black_rate = self._black_count_in_window / total_count

        return black_rate >= threshold

    def _error_log_output(self, err_idx: int, *args: object) -> None:
        if len(args) != 1:
            raise ValueError("args must be (index,)")
        if isinstance(args[0], int):
            index: int = args[0]
        else:
            raise ValueError("args[0] must be int")
        self._logger.warning(
            f"カメラデータ欠落: カメラ{index}からの映像に欠落が発生しています。"
        )


class MemoryLeakDetected(StateErrorDiagnosisD):
    """MEMORY_LEAK_DETECTED: メモリリーク検出"""

    def __init__(self) -> None:
        super().__init__()
        self._used_mb_samples: deque[float] = deque()
        self._window_start_time: float | None = None
        self._baseline_min_used_mb: float | None = None
        self.param: err_conf.MemoryLeakDetectedParameters

    def update(self, err_conf: err_conf.ErrorConfig) -> None:
        self.param = err_conf.memory_leak_detected
        self.is_enabled = self.param.is_enabled

    def _parse_args(self, *args: object) -> tuple[float, float]:
        """
        args[0] (int): 使用メモリ量 ram_used_mb
        args[1] (float): 現在時刻 now
        """
        if len(args) != 2:
            raise ValueError("args must be (ram_used_mb, now)")

        ram_used_mb = args[0]
        now = args[1]
        if not isinstance(ram_used_mb, int) or not isinstance(now, float):
            raise ValueError("args must be int, float")

        return int(ram_used_mb), float(now)

    def detect_error(self, *args: object) -> bool:
        """
        一定時間ごとに使用メモリ最小値を評価し、
        最小値が基準より規定倍以上なら True。
        """
        ram_used_mb, now = self._parse_args(*args)

        if self._window_start_time is None:
            self._window_start_time = now

        # window_sec間の使用メモリ量を蓄積
        self._used_mb_samples.append(ram_used_mb)
        elapsed = now - self._window_start_time
        if elapsed < self.param.window_sec:
            return False

        window_min_used_mb = min(self._used_mb_samples)

        # 初回ウィンドウの最小値は基準値として保持、最小値が基準値の規定倍以上ならエラー
        is_error = False
        if self._baseline_min_used_mb is None:
            self._baseline_min_used_mb = window_min_used_mb
        elif (
            len(self._used_mb_samples) >= self.param.min_samples
            and window_min_used_mb
            >= self._baseline_min_used_mb * self.param.leak_ratio_threshold
        ):
            is_error = True

        # 履歴をクリアして次のウィンドウの計測を開始
        self._used_mb_samples.clear()
        self._window_start_time = now

        return is_error

    def _error_log_output(self, err_idx: int, *args: object) -> None:
        self._logger.warning("メモリリーク検出: メモリリークが検出されました。")


class OtherHardwareError(StateErrorDiagnosisD):
    """OTHER_HARDWARE_ERROR: その他のエラー（ハードウェア）"""

    class HardwareType(StrEnum):
        """ハードウェアの種類"""

        TOUCHSCREEN = "ID_INPUT_TOUCHSCREEN=1"
        KEYBOARD = "ID_INPUT_KEYBOARD=1"
        MOUSE = "ID_INPUT_MOUSE=1"
        SPATIAL_CONTROLLER = "SPATIAL_CONTROLLER_IS_NO_ID"

    class ConnectionStatus(Enum):
        """接続状態"""

        CONNECTED = auto()
        NOT_CONNECTED = auto()
        NOT_CHECKED = auto()

    def __init__(self) -> None:
        super().__init__()
        self.param: err_conf.OtherHardwareErrorParameters
        self._is_connected_funcs: dict[
            OtherHardwareError.HardwareType,
            Callable[[OtherHardwareError.HardwareType], bool],
        ] = {
            self.HardwareType.TOUCHSCREEN: self._general_device_is_connected,
            self.HardwareType.KEYBOARD: self._general_device_is_connected,
            self.HardwareType.MOUSE: self._general_device_is_connected,
            self.HardwareType.SPATIAL_CONTROLLER: self._spatial_controller_is_connected,
        }
        self._devices_is_connected: dict[
            OtherHardwareError.HardwareType, OtherHardwareError.ConnectionStatus
        ] = {
            self.HardwareType.TOUCHSCREEN: self.ConnectionStatus.NOT_CHECKED,
            self.HardwareType.KEYBOARD: self.ConnectionStatus.NOT_CHECKED,
            self.HardwareType.MOUSE: self.ConnectionStatus.NOT_CHECKED,
            self.HardwareType.SPATIAL_CONTROLLER: self.ConnectionStatus.NOT_CHECKED,
        }

    def update(self, err_conf: err_conf.ErrorConfig) -> None:
        self.param = err_conf.other_hardware_error
        self.is_enabled = self.param.is_enabled

    def _general_device_is_connected(
        self, hardware_type: OtherHardwareError.HardwareType
    ) -> bool:
        """udevadmを使用して、指定されたハードウェアタイプが接続されているかどうかを確認する"""

        for event_dev in Path("/dev/input").glob("event*"):
            result: subprocess.CompletedProcess[str] = subprocess.run(
                [
                    "udevadm",
                    "info",
                    "--query=property",
                    f"--name={event_dev}",
                ],
                capture_output=True,
                text=True,
                check=False,
            )
            self._logger.debug(result.stdout)

            if hardware_type.value in result.stdout:
                self._devices_is_connected[hardware_type] = (
                    self.ConnectionStatus.CONNECTED
                )
                return True

        self._devices_is_connected[hardware_type] = self.ConnectionStatus.NOT_CONNECTED
        return False

    def _spatial_controller_is_connected(
        self, hardware_type: OtherHardwareError.HardwareType
    ) -> bool:
        """3Dマウスの接続判定"""
        # TODO(NSW): 3Dマウスの接続判定を実装する必要があります。現状では常にFalseを返すようにしています。

        self._devices_is_connected[hardware_type] = self.ConnectionStatus.NOT_CONNECTED
        return False

    def detect_error(self, *args: object) -> bool:
        """
        ハードウェアの種類がHardwareTypeに含まれていればTrue
        """
        result: list[bool] = []
        for hard in self.param.hardware_types:
            try:
                hardware_type: OtherHardwareError.HardwareType = self.HardwareType[hard]
            except KeyError:
                self._logger.warning(f"Unsupported hardware type: {hard}")
                continue
            result.append(self._is_connected_funcs[hardware_type](hardware_type))

        return not all(result)

    def _error_log_output(self, err_idx: int, *args: object) -> None:
        is_not_connected: tuple[str, ...] = tuple(
            hard.name
            for hard, status in self._devices_is_connected.items()
            if status == self.ConnectionStatus.NOT_CONNECTED
        )
        self._logger.warning(
            f"その他のエラー（ハードウェア）: {is_not_connected}が接続されていません。"
        )


class InvalidDataInput(StateErrorDiagnosisD):
    """INVALID_DATA_INPUT: 不正データ入力"""

    def __init__(self) -> None:
        super().__init__()
        self.param: err_conf.InvalidDataInputParameters

    def update(self, err_conf: err_conf.ErrorConfig) -> None:
        self.param = err_conf.invalid_data_input
        self.is_enabled = self.param.is_enabled

    def detect_error(self, *args: object) -> bool:
        """
        input_dataの要素が1つでもNoneならTrue
        input_dataの要素がtupleであれば再帰的に判定する
        """
        input_data = args
        if input_data is None:
            return True
        for item in input_data:
            if item is None:
                return True
            if isinstance(item, tuple) and self.detect_error(*item):
                return True

        return False

    def _error_log_output(self, err_idx: int, *args: object) -> None:
        self._logger.warning("不正データ入力: Invalid data entry.")


class NumericAnomalyException(StateErrorDiagnosisD):
    """NUMERIC_ANOMALY_EXCEPTION: 不正演算（NaN／ゼロ除算等）"""

    def __init__(self) -> None:
        super().__init__()

    def excepts_diagnosis(self, e: Exception) -> bool:
        # Exceptionの種類判定　-> 0割のExceptionならTrue
        return isinstance(e, ZeroDivisionError)

    def _error_log_output(self, err_idx: int, *args: object) -> None:
        if len(args) != 1:
            raise ValueError("args must be Exception")
        if isinstance(args[0], Exception):
            e: Exception = args[0]
        else:
            raise ValueError("args[0] must be Exception")
        self._logger.warning(
            f"不正演算（NaN／ゼロ除算等）: Numeric anomaly exception: {e}.\n",
            exc_info=True,
        )


class ArrayShapeError(StateErrorDiagnosisD):
    """ARRAY_SHAPE_ERROR: 配列形状エラー"""

    def __init__(self) -> None:
        super().__init__()
        self._invalid_array: tuple[str, tuple[int, ...], tuple[int, ...]] | None = None
        self.param: err_conf.ArrayShapeErrorParameters

    def update(self, err_conf: err_conf.ErrorConfig) -> None:
        self.param = err_conf.array_shape_error
        self.is_enabled = self.param.is_enabled

    def _parse_args(self, *args: object) -> tuple[tuple[str, NDArray], ...]:
        """
        argsは (配列名, 配列) の組を可変長で受け取る。
        配列が tuple[NDArray, ...] の場合は、同じ配列名を各NDArrayへ展開する。
        """
        array_name_and_data: list[tuple[str, NDArray]] = []
        pair_size = 2
        for arg in args:
            if not isinstance(arg, tuple) or len(arg) != pair_size:
                raise ValueError("args must be tuples of (array_name, array_data)")

            array_name_obj, array_data = arg
            if not isinstance(array_name_obj, str):
                raise TypeError("array_name must be str")
            array_name = array_name_obj

            if isinstance(array_data, np.ndarray):
                array_name_and_data.append((array_name, array_data))
                continue

            if isinstance(array_data, tuple):
                for item in array_data:
                    if not isinstance(item, np.ndarray):
                        raise TypeError("array_data tuple must contain only NDArray")
                    array_name_and_data.append((array_name, item))
                continue

            raise TypeError("array_data must be NDArray or tuple[NDArray, ...]")

        return tuple(array_name_and_data)

    def _is_shape_match(
        self, actual_shape: tuple[int, ...], expected_shape: tuple[int, ...]
    ) -> bool:
        # shapeが期待値と一致しない場合Trueを返す
        if len(actual_shape) != len(expected_shape):
            return True
        return not all(
            e in (-1, a) for a, e in zip(actual_shape, expected_shape, strict=True)
        )

    def detect_error(self, *args: object) -> bool:
        """
        引数は (配列名, 配列) の組を可変長で受け取り、
        期待shapeと一致しない場合にTrueを返す。
        """
        self._invalid_array = None
        expected_shapes = self.param.expected_shape_by_array_name
        parsed_args = self._parse_args(*args)
        for array_name, array_data in parsed_args:
            expected_shape = expected_shapes.get(array_name)
            if expected_shape is not None and self._is_shape_match(
                array_data.shape, expected_shape
            ):
                # shapeが期待値と一致しない場合、配列名,実際の形状,期待形状を保持
                self._invalid_array = (array_name, array_data.shape, expected_shape)
                return True

        return False

    def _error_log_output(self, err_idx: int, *args: object) -> None:
        self._logger.warning(
            f"The array provided does not match the expected format. array_name:{self._invalid_array[0]}, actual_shape:{self._invalid_array[1]}, expected_shape:{self._invalid_array[2]}"
        )


class ProcessForcedTermination(StateErrorDiagnosisD):
    """PROCESS_FORCED_TERMINATION: プロセス強制終了"""

    def __init__(self) -> None:
        super().__init__()
        self.param: err_conf.ProcessForcedTerminationParameters

    def update(self, err_conf: err_conf.ErrorConfig) -> None:
        self.param = err_conf.process_forced_termination
        self.is_enabled = self.param.is_enabled

    def _args_parse(self, *args: object) -> tuple[list[ProcessBase], list[ProcessBase]]:
        if len(args) != 2:
            raise ValueError("args must be (termed_processes, killed_processes)")
        termed: object = args[0]
        killed: object = args[1]
        if not isinstance(termed, list) or not isinstance(killed, list):
            raise ValueError("args[0] and args[1] must be list")

        for process in termed:
            if not isinstance(process, ProcessBase):
                raise ValueError("args[0] must be list of ProcessBase")
        for process in killed:
            if not isinstance(process, ProcessBase):
                raise ValueError("args[1] must be list of ProcessBase")

        return cast(list[ProcessBase], termed), cast(list[ProcessBase], killed)

    def detect_error(self, *args: object) -> bool:
        """
        強制終了を実施した場合はTrue
        """
        try_term_processes, try_kill_processes = self._args_parse(*args)

        return len(try_term_processes) > 0 or len(try_kill_processes) > 0

    def _error_log_output(self, err_idx: int, *args: object) -> None:
        try_term_processes, try_kill_processes = self._args_parse(*args)
        term_proc_names: list[str] = [process.name for process in try_term_processes]
        kill_proc_names: list[str] = [process.name for process in try_kill_processes]
        term_msg = (
            f"Terminate process: {term_proc_names}"
            if len(term_proc_names) > 0
            else "Terminate process: なし"
        )
        kill_msg = (
            f"Kill process: {kill_proc_names}"
            if len(kill_proc_names) > 0
            else "Kill process: なし"
        )
        self._logger.warning(f"プロセス強制終了: {term_msg}, {kill_msg}")


class FileIoError(StateErrorDiagnosisD):
    """FILE_IO_ERROR: 重要設定以外のファイル読み込みエラー"""

    def __init__(self) -> None:
        super().__init__()
        self.param: err_conf.FileIoErrorParameters

    def update(self, err_conf: err_conf.ErrorConfig) -> None:
        self.param = err_conf.file_io_error
        self.is_enabled = self.param.is_enabled

    def detect_error(self, *args: object) -> bool:
        if len(args) != 1 or not isinstance(args[0], bool):
            raise ValueError("args must be (has_file_io_error,)")
        return args[0]

    def _error_log_output(self, err_idx: int, *args: object) -> None:
        if len(args) != 3 or not all(isinstance(value, str) for value in args):
            raise ValueError("args must be (path, operation, error_detail)")
        path, operation, error_detail = args
        self._logger.warning(
            self.get_error_no(err_idx)
            + f": ファイルI/Oエラー: operation={operation}, path={path}, "
            + f"error={error_detail}"
        )


class LogCompressionFailure(StateErrorDiagnosisD):
    """LOG_COMPRESSION_FAILURE: ログローテーション後のgzip圧縮失敗"""

    def __init__(self) -> None:
        super().__init__()
        self._event_count = create_shared_single_data(0)
        self._processed_event_count = 0
        self.param: err_conf.LogCompressionFailureParameters

    def update(self, err_conf: err_conf.ErrorConfig) -> None:
        self.param = err_conf.log_compression_failure
        self.is_enabled = self.param.is_enabled

    def report_event(self, _error: Exception | None = None) -> None:
        self._event_count.value += 1

    def detect_error(self, *args: object) -> bool:
        if self._event_count.value == self._processed_event_count:
            return False
        self._processed_event_count = self._event_count.value
        return True

    def _error_log_output(self, err_idx: int, *args: object) -> None:
        self._logger.warning(
            self.get_error_no(err_idx) + ": ログローテーション後の圧縮に失敗しました。"
        )


class LogTimeReversal(StateErrorDiagnosisD):
    """LOG_TIME_REVERSAL: ログレコード時刻の逆転"""

    def __init__(self) -> None:
        super().__init__()
        self._event_count = create_shared_single_data(0)
        self._processed_event_count = 0
        self.param: err_conf.LogTimeReversalParameters

    def update(self, err_conf: err_conf.ErrorConfig) -> None:
        self.param = err_conf.log_time_reversal
        self.is_enabled = self.param.is_enabled

    def report_record_time(self, previous_time: float, current_time: float) -> None:
        if current_time < previous_time - self.param.allowed_backward_sec:
            self._event_count.value += 1

    def detect_error(self, *args: object) -> bool:
        if self._event_count.value == self._processed_event_count:
            return False
        self._processed_event_count = self._event_count.value
        return True

    def _error_log_output(self, err_idx: int, *args: object) -> None:
        self._logger.warning(
            self.get_error_no(err_idx) + ": ログ記録時刻の逆転を検出しました。"
        )


class LidarModuleError(StateErrorDiagnosisD):
    """LIDAR_MODULE_ERROR: LiDARモジュールエラー"""

    def update(self, err_conf: err_conf.ErrorConfig) -> None:
        self.param = err_conf.lidar_module_error
        self.is_enabled = self.param.is_enabled
        self._ongoing_log_interval_sec = self.param.ongoing_log_interval_sec

    def excepts_diagnosis(self, e: Exception) -> bool:
        return not isinstance(e, KeyboardInterrupt)

    def _error_log_output(self, err_idx: int, *args: object) -> None:
        if len(args) != 2:
            raise ValueError("args must be Exception, index")
        if isinstance(args[0], Exception) and isinstance(args[1], int):
            e: Exception = args[0]
            index: int = args[1]
        else:
            raise ValueError("args[0] must be Exception and args[1] must be int")
        should_log, include_traceback = self._should_log(e, time.monotonic())
        if not should_log:
            return
        self._logger.warning(
            f"LiDAR{index}モジュールエラー: {type(e).__name__}: {e}",
            exc_info=include_traceback,
        )


class CameraModuleError(StateErrorDiagnosisD):
    """CAMERA_MODULE_ERROR: カメラモジュールエラー"""

    def update(self, err_conf: err_conf.ErrorConfig) -> None:
        self.param = err_conf.camera_module_error
        self.is_enabled = self.param.is_enabled
        self._ongoing_log_interval_sec = self.param.ongoing_log_interval_sec

    def excepts_diagnosis(self, e: Exception) -> bool:
        return not isinstance(e, KeyboardInterrupt)

    def _error_log_output(self, err_idx: int, *args: object) -> None:
        if len(args) != 2:
            raise ValueError("args must be Exception, index")
        if isinstance(args[0], Exception) and isinstance(args[1], int):
            e: Exception = args[0]
            index: int = args[1]
        else:
            raise ValueError("args[0] must be Exception and args[1] must be int")
        should_log, include_traceback = self._should_log(e, time.monotonic())
        if not should_log:
            return
        self._logger.warning(
            f"カメラ{index}モジュールエラー: {type(e).__name__}: {e}",
            exc_info=include_traceback,
        )


class AccumulationModuleError(StateErrorDiagnosisD):
    """ACCUMULATION_MODULE_ERROR: 蓄積モジュールエラー"""

    def update(self, err_conf: err_conf.ErrorConfig) -> None:
        self.param = err_conf.storage_module_error
        self.is_enabled = self.param.is_enabled
        self._ongoing_log_interval_sec = self.param.ongoing_log_interval_sec

    def excepts_diagnosis(self, e: Exception) -> bool:
        return not isinstance(e, KeyboardInterrupt)

    def _error_log_output(self, err_idx: int, *args: object) -> None:
        if len(args) != 1:
            raise ValueError("args must be Exception")
        if isinstance(args[0], Exception):
            e: Exception = args[0]
        else:
            raise ValueError("args[0] must be Exception")
        should_log, include_traceback = self._should_log(e, time.monotonic())
        if not should_log:
            return
        self._logger.warning(
            f"蓄積モジュールエラー: {type(e).__name__}: {e}",
            exc_info=include_traceback,
        )


class CanModuleError(StateErrorDiagnosisD):
    """CAN_MODULE_ERROR: CANモジュールエラー"""

    def update(self, err_conf: err_conf.ErrorConfig) -> None:
        self.param = err_conf.can_module_error
        self.is_enabled = self.param.is_enabled
        self._ongoing_log_interval_sec = self.param.ongoing_log_interval_sec

    def excepts_diagnosis(self, e: Exception) -> bool:
        return not isinstance(e, KeyboardInterrupt)

    def _error_log_output(self, err_idx: int, *args: object) -> None:
        if len(args) != 1:
            raise ValueError("args must be Exception")
        if isinstance(args[0], Exception):
            e: Exception = args[0]
        else:
            raise ValueError("args[0] must be Exception")
        should_log, include_traceback = self._should_log(e, time.monotonic())
        if not should_log:
            return
        self._logger.warning(
            f"CANモジュールエラー: {type(e).__name__}: {e}",
            exc_info=include_traceback,
        )


class Integrate2d3dModuleError(StateErrorDiagnosisD):
    """INTEGRATE_2D3D_MODULE_ERROR: 2D-3D紐づけモジュールエラー"""

    def update(self, err_conf: err_conf.ErrorConfig) -> None:
        self.param = err_conf.linkage_2d3d_module_error
        self.is_enabled = self.param.is_enabled
        self._ongoing_log_interval_sec = self.param.ongoing_log_interval_sec

    def excepts_diagnosis(self, e: Exception) -> bool:
        return not isinstance(e, KeyboardInterrupt)

    def _error_log_output(self, err_idx: int, *args: object) -> None:
        if len(args) != 1:
            raise ValueError("args must be Exception")
        if isinstance(args[0], Exception):
            e: Exception = args[0]
        else:
            raise ValueError("args[0] must be Exception")
        should_log, include_traceback = self._should_log(e, time.monotonic())
        if not should_log:
            return
        self._logger.warning(
            f"2D-3D紐づけモジュールエラー: {type(e).__name__}: {e}",
            exc_info=include_traceback,
        )


class Object3DDetectionModuleError(StateErrorDiagnosisD):
    """OBJECT_3D_DETECTION_MODULE_ERROR: 3D物体検知モジュールエラー"""

    def update(self, err_conf: err_conf.ErrorConfig) -> None:
        self.param = err_conf.object3_d_detection_module_error
        self.is_enabled = self.param.is_enabled
        self._ongoing_log_interval_sec = self.param.ongoing_log_interval_sec

    def excepts_diagnosis(self, e: Exception) -> bool:
        return not isinstance(e, KeyboardInterrupt)

    def _error_log_output(self, err_idx: int, *args: object) -> None:
        if len(args) != 1:
            raise ValueError("args must be Exception")
        if isinstance(args[0], Exception):
            e: Exception = args[0]
        else:
            raise ValueError("args[0] must be Exception")
        should_log, include_traceback = self._should_log(e, time.monotonic())
        if not should_log:
            return
        self._logger.warning(
            f"3D物体検知モジュールエラー: {type(e).__name__}: {e}",
            exc_info=include_traceback,
        )


class CameraHumanDetectionModuleError(StateErrorDiagnosisD):
    """CAMERA_HUMAN_DETECTION_MODULE_ERROR: カメラ人検知モジュールエラー"""

    def update(self, err_conf: err_conf.ErrorConfig) -> None:
        self.param = err_conf.camera_human_detection_module_error
        self.is_enabled = self.param.is_enabled
        self._ongoing_log_interval_sec = self.param.ongoing_log_interval_sec

    def excepts_diagnosis(self, e: Exception) -> bool:
        return not isinstance(e, KeyboardInterrupt)

    def _error_log_output(self, err_idx: int, *args: object) -> None:
        if len(args) != 1:
            raise ValueError("args must be Exception")
        if isinstance(args[0], Exception):
            e: Exception = args[0]
        else:
            raise ValueError("args[0] must be Exception")
        should_log, include_traceback = self._should_log(e, time.monotonic())
        if not should_log:
            return
        self._logger.warning(
            f"カメラ人検知モジュールエラー: {type(e).__name__}: {e}",
            exc_info=include_traceback,
        )


class CollisionJudgmentModuleError(StateErrorDiagnosisD):
    """COLLISION_JUDGMENT_MODULE_ERROR: 衝突判定モジュールエラー"""

    def update(self, err_conf: err_conf.ErrorConfig) -> None:
        self.param = err_conf.collision_judgment_module_error
        self.is_enabled = self.param.is_enabled
        self._ongoing_log_interval_sec = self.param.ongoing_log_interval_sec

    def excepts_diagnosis(self, e: Exception) -> bool:
        return not isinstance(e, KeyboardInterrupt)

    def _error_log_output(self, err_idx: int, *args: object) -> None:
        if len(args) != 1:
            raise ValueError("args must be Exception")
        if isinstance(args[0], Exception):
            e: Exception = args[0]
        else:
            raise ValueError("args[0] must be Exception")
        should_log, include_traceback = self._should_log(e, time.monotonic())
        if not should_log:
            return
        self._logger.warning(
            f"衝突判定モジュールエラー: {type(e).__name__}: {e}",
            exc_info=include_traceback,
        )


class CalibrationModuleError(StateErrorDiagnosisD):
    """CALIBRATION_MODULE_ERROR: 校正モジュールエラー"""

    def update(self, err_conf: err_conf.ErrorConfig) -> None:
        self.param = err_conf.calibration_module_error
        self.is_enabled = self.param.is_enabled
        self._ongoing_log_interval_sec = self.param.ongoing_log_interval_sec

    def excepts_diagnosis(self, e: Exception) -> bool:
        return not isinstance(e, KeyboardInterrupt)

    def _error_log_output(self, err_idx: int, *args: object) -> None:
        if len(args) != 1:
            raise ValueError("args must be Exception")
        if isinstance(args[0], Exception):
            e: Exception = args[0]
        else:
            raise ValueError("args[0] must be Exception")
        should_log, include_traceback = self._should_log(e, time.monotonic())
        if not should_log:
            return
        self._logger.warning(
            f"校正モジュールエラー: {type(e).__name__}: {e}",
            exc_info=include_traceback,
        )


class VisualModuleError(StateErrorDiagnosisD):
    """VISUAL_MODULE_ERROR: VisualProcessモジュールエラー"""

    def update(self, err_conf: err_conf.ErrorConfig) -> None:
        self.param = err_conf.visual_module_error
        self.is_enabled = self.param.is_enabled
        self._ongoing_log_interval_sec = self.param.ongoing_log_interval_sec

    def excepts_diagnosis(self, e: Exception) -> bool:
        return not isinstance(e, KeyboardInterrupt)

    def _error_log_output(self, err_idx: int, *args: object) -> None:
        if len(args) != 1:
            raise ValueError("args must be Exception")
        if isinstance(args[0], Exception):
            e: Exception = args[0]
        else:
            raise ValueError("args[0] must be Exception")
        should_log, include_traceback = self._should_log(e, time.monotonic())
        if not should_log:
            return
        self._logger.warning(
            f"VisualProcessモジュールエラー: {type(e).__name__}: {e}",
            exc_info=include_traceback,
        )


class PointsRefineModuleError(StateErrorDiagnosisD):
    """POINTS_REFINE_MODULE_ERROR: PointsRefineモジュールエラー"""

    def update(self, err_conf: err_conf.ErrorConfig) -> None:
        self.param = err_conf.points_refine_module_error
        self.is_enabled = self.param.is_enabled
        self._ongoing_log_interval_sec = self.param.ongoing_log_interval_sec

    def excepts_diagnosis(self, e: Exception) -> bool:
        return not isinstance(e, KeyboardInterrupt)

    def _error_log_output(self, err_idx: int, *args: object) -> None:
        if len(args) != 1:
            raise ValueError("args must be Exception")
        if isinstance(args[0], Exception):
            e: Exception = args[0]
        else:
            raise ValueError("args[0] must be Exception")
        should_log, include_traceback = self._should_log(e, time.monotonic())
        if not should_log:
            return
        self._logger.warning(
            f"PointsRefineモジュールエラー: {type(e).__name__}: {e}",
            exc_info=include_traceback,
        )


class MainModuleError(StateErrorDiagnosisD):
    """MAIN_MODULE_ERROR: MainProcessモジュールエラー"""

    def update(self, err_conf: err_conf.ErrorConfig) -> None:
        self.param = err_conf.main_module_error
        self.is_enabled = self.param.is_enabled
        self._ongoing_log_interval_sec = self.param.ongoing_log_interval_sec

    def excepts_diagnosis(self, e: Exception) -> bool:
        return not isinstance(e, KeyboardInterrupt)

    def _error_log_output(self, err_idx: int, *args: object) -> None:
        if len(args) != 1:
            raise ValueError("args must be Exception")
        if isinstance(args[0], Exception):
            e: Exception = args[0]
        else:
            raise ValueError("args[0] must be Exception")
        should_log, include_traceback = self._should_log(e, time.monotonic())
        if not should_log:
            return
        self._logger.warning(
            f"MainProcessモジュールエラー: {type(e).__name__}: {e}",
            exc_info=include_traceback,
        )


class AppManagerModuleError(StateErrorDiagnosisD):
    """APP_MANAGER_MODULE_ERROR: アプリケーションマネージャーモジュールエラー"""

    def update(self, err_conf: err_conf.ErrorConfig) -> None:
        self.param = err_conf.app_manager_module_error
        self.is_enabled = self.param.is_enabled
        self._ongoing_log_interval_sec = self.param.ongoing_log_interval_sec

    def excepts_diagnosis(self, e: Exception) -> bool:
        return not isinstance(e, KeyboardInterrupt)

    def _error_log_output(self, err_idx: int, *args: object) -> None:
        if len(args) != 1:
            raise ValueError("args must be Exception")
        if isinstance(args[0], Exception):
            e: Exception = args[0]
        else:
            raise ValueError("args[0] must be Exception")
        should_log, include_traceback = self._should_log(e, time.monotonic())
        if not should_log:
            return
        self._logger.warning(
            f"アプリケーションマネージャーモジュールエラー: {type(e).__name__}: {e}",
            exc_info=include_traceback,
        )


class ImuModuleError(StateErrorDiagnosisD):
    """IMU_MODULE_ERROR: IMUモジュールエラー"""

    def update(self, err_conf: err_conf.ErrorConfig) -> None:
        self.param = err_conf.imu_module_error
        self.is_enabled = self.param.is_enabled
        self._ongoing_log_interval_sec = self.param.ongoing_log_interval_sec

    def excepts_diagnosis(self, e: Exception) -> bool:
        return not isinstance(e, KeyboardInterrupt)

    def _error_log_output(self, err_idx: int, *args: object) -> None:
        if len(args) != 2:
            raise ValueError("args must be Exception, index")
        if isinstance(args[0], Exception) and isinstance(args[1], int):
            e: Exception = args[0]
            index: int = args[1]
        else:
            raise ValueError("args[0] must be Exception and args[1] must be int")
        should_log, include_traceback = self._should_log(e, time.monotonic())
        if not should_log:
            return
        self._logger.warning(
            f"IMU{index}モジュールエラー: {type(e).__name__}: {e}",
            exc_info=include_traceback,
        )


class LidarShiftMonitorModuleError(StateErrorDiagnosisD):
    """LIDAR_SHIFT_MONITOR_MODULE_ERROR: LiDARシフトモニタモジュールエラー"""

    def update(self, err_conf: err_conf.ErrorConfig) -> None:
        self.param = err_conf.lidar_shift_monitor_module_error
        self.is_enabled = self.param.is_enabled
        self._ongoing_log_interval_sec = self.param.ongoing_log_interval_sec

    def excepts_diagnosis(self, e: Exception) -> bool:
        return not isinstance(e, KeyboardInterrupt)

    def _error_log_output(self, err_idx: int, *args: object) -> None:
        if len(args) != 1:
            raise ValueError("args must be Exception")
        if isinstance(args[0], Exception):
            e: Exception = args[0]
        else:
            raise ValueError("args[0] must be Exception")
        should_log, include_traceback = self._should_log(e, time.monotonic())
        if not should_log:
            return
        self._logger.warning(
            f"LiDARシフトモニタモジュールエラー: {type(e).__name__}: {e}",
            exc_info=include_traceback,
        )


class GetDataModuleError(StateErrorDiagnosisD):
    """GET_DATA_MODULE_ERROR: データ取得モジュールエラー"""

    def update(self, err_conf: err_conf.ErrorConfig) -> None:
        self.param = err_conf.get_data_module_error
        self.is_enabled = self.param.is_enabled
        self._ongoing_log_interval_sec = self.param.ongoing_log_interval_sec

    def excepts_diagnosis(self, e: Exception) -> bool:
        return not isinstance(e, KeyboardInterrupt)

    def _error_log_output(self, err_idx: int, *args: object) -> None:
        if len(args) != 1:
            raise ValueError("args must be Exception")
        if isinstance(args[0], Exception):
            e: Exception = args[0]
        else:
            raise ValueError("args[0] must be Exception")
        should_log, include_traceback = self._should_log(e, time.monotonic())
        if not should_log:
            return
        self._logger.warning(
            f"データ取得モジュールエラー: {type(e).__name__}: {e}",
            exc_info=include_traceback,
        )
