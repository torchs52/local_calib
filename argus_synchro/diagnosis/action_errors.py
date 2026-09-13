from __future__ import annotations

import json
import time
from configparser import (
    DuplicateOptionError,
    DuplicateSectionError,
    MissingSectionHeaderError,
    NoOptionError,
    NoSectionError,
    ParsingError,
)
from typing import cast

import numpy as np
from numpy.typing import NDArray

import argus_synchro.diagnosis.error_config as err_conf
from argus_synchro.config.app_config import CalibrationConf
from argus_synchro.diagnosis.calib_matrix_validator import (
    CameraCalibValidationIssue,
    CameraCalibValidator,
    LidarCalibValidationIssue,
    LidarCalibValidator,
)
from argus_synchro.diagnosis.error_diagnosis import (
    ActionErrorDiagnosisA,
    ActionErrorDiagnosisB,
    ActionErrorDiagnosisC,
    DiagnosisRuntimePolicy,
)
from argus_synchro.shared_excepts import SharedLidarShiftMonitorExcept


class LidarPositionMisalignmentDetectedDiagnosis(ActionErrorDiagnosisA):
    """LIDAR_POSITION_MISALIGNMENT_DETECTED: LiDAR位置ズレ検出"""

    def __init__(self, runtime_policy: DiagnosisRuntimePolicy | None = None) -> None:
        super().__init__(runtime_policy)

    def update(self, err_conf: err_conf.ErrorConfig) -> None:
        self.param: err_conf.LidarPositionMisalignmentDetectedParameters = (
            err_conf.lidar_position_misalignment_detected
        )
        self.is_enabled = self.param.is_enabled

    def _parse_args(self, *args: object) -> SharedLidarShiftMonitorExcept:
        """
        args[0] (SharedLidarShiftMonitorExcept) :
        """
        if len(args) != 1:
            raise ValueError("args must be (sec_lidar_sm,)")
        sec_lidar_sm = args[0]

        if not isinstance(sec_lidar_sm, SharedLidarShiftMonitorExcept):
            raise ValueError("args must be (SharedLidarShiftMonitorExcept,)")

        return sec_lidar_sm

    def detect_error(self, *args: object) -> bool:
        sec_lidar_sm: SharedLidarShiftMonitorExcept = self._parse_args(*args)
        # サービス作業中は位置ずれを新規エラーとして扱わない。
        if self.is_suppressed_in_maintenance:
            return False
        if sec_lidar_sm.has_not_calibrated:
            # NOTE: 校正未実施フラグが立っている場合は、LiDAR位置ズレ検出の診断を行わない
            return False

        if sec_lidar_sm.is_shifted_fast.value or sec_lidar_sm.is_shifted_slow.value:
            self.increment_counter()
            self.is_idle.value = True
            self.is_fail_safe.value = True
            sec_lidar_sm.write_has_not_calibrated(True)
            return True

        return False

    def detect_recovery_error(self, *args: object) -> bool:
        # 復帰条件が無いため、常にFalseを返す
        return False

    def detect_recovery_fail_safe(self, *args: object) -> bool:
        # 復帰条件が無いため、常にFalseを返す
        return False

    def log_output(self, err: bool, recover: bool, err_idx: int, *args: object) -> None:
        if err:
            self._error_log_output(err_idx, *args)

    def _error_log_output(self, err_idx: int, *args: object) -> None:
        self._logger.error(
            self.get_error_no(err_idx)
            + ": LIDAR_POSITION_MISALIGNMENT_DETECTED: LiDAR位置ズレ検出. LiDARの位置がずれていないかを確認してください。 "
        )


class SensorCalibrationRequiredDiagnosis(ActionErrorDiagnosisA):
    """SENSOR_CALIBRATION_REQUIRED: 要センサ校正"""

    def __init__(self, runtime_policy: DiagnosisRuntimePolicy | None = None) -> None:
        super().__init__(runtime_policy)
        self._last_detect_error_pid: int = 0
        self._last_detect_recovery_fail_safe_pid: int = 0

    def update(self, err_conf: err_conf.ErrorConfig) -> None:
        self.param: err_conf.SensorCalibrationRequiredParameters = (
            err_conf.sensor_calibration_required
        )
        self.is_enabled = self.param.is_enabled

    def _parse_args(self, *args: object) -> tuple[SharedLidarShiftMonitorExcept, int]:
        """
        args[0] (SharedLidarShiftMonitorExcept) :
        args[1] (int) : 実行プロセスID
        """
        if len(args) != 2:
            raise ValueError("args must be (sec_lidar_sm, pid)")
        sec_lidar_sm = args[0]
        pid = args[1]

        if not isinstance(sec_lidar_sm, SharedLidarShiftMonitorExcept):
            raise ValueError("args must be (SharedLidarShiftMonitorExcept, pid)")

        if not isinstance(pid, int):
            raise ValueError("args must be (SharedLidarShiftMonitorExcept, int)")

        return sec_lidar_sm, pid

    def detect_error(self, *args: object) -> bool:
        # NOTE: この診断はプロセス起動毎に1回のみ行う

        sec_lidar_sm, pid = self._parse_args(*args)
        # サービス作業中は校正要求を新規エラーとして扱わない。
        if self.is_suppressed_in_maintenance:
            return False
        if pid == self._last_detect_error_pid:
            return False
        self._last_detect_error_pid = pid

        if sec_lidar_sm.has_not_calibrated:
            self.increment_counter()
            self.is_idle.value = True
            self.is_fail_safe.value = True
            return True
        return False

    def detect_recovery_error(self, *args: object) -> bool:
        # 復帰条件が無いため、常にFalseを返す
        return False

    def detect_recovery_fail_safe(self, *args: object) -> bool:
        # NOTE: この診断はプロセス起動毎に1回のみ行う
        sec_lidar_sm, pid = self._parse_args(*args)
        if pid == self._last_detect_recovery_fail_safe_pid:
            return False
        self._last_detect_recovery_fail_safe_pid = pid
        if not sec_lidar_sm.has_not_calibrated:
            self.is_idle.value = False
            self.is_fail_safe.value = False
            return True
        return False

    def log_output(self, err: bool, recover: bool, err_idx: int, *args: object) -> None:
        if err:
            self._error_log_output(err_idx)
        elif recover:
            self._fail_safe_recover_log_output(err_idx)

    def _error_log_output(self, err_idx: int, *args: object) -> None:
        self._logger.error(
            self.get_error_no(err_idx)
            + ": 要センサ校正: LiDARの校正を実施してください。"
        )

    def _fail_safe_recover_log_output(self, err_idx: int, *args: object) -> None:
        self._logger.warning(
            self.get_error_no(err_idx)
            + ": 要センサ校正: LiDARの校正完了を確認しました。"
        )


class ModelInfoMismatchDiagnosis(ActionErrorDiagnosisA):
    """MODEL_INFO_MISMATCH: 機種情報不一致"""

    def __init__(self) -> None:
        super().__init__()

    def detect_error(self, *args: object) -> bool:
        return False

    def detect_recovery_error(self, *args: object) -> bool:
        return True

    def detect_recovery_fail_safe(self, *args: object) -> bool:
        return True


class CraneModelFileMissingDiagnosis(ActionErrorDiagnosisA):
    """CRANE_MODEL_FILE_MISSING: 機体モデルファイル欠損/破損"""

    def __init__(self) -> None:
        super().__init__()
        self.param: err_conf.CraneModelFileMissingParameters

    def update(self, err_conf: err_conf.ErrorConfig) -> None:
        self.param = err_conf.crane_model_file_missing
        self.is_enabled = self.param.is_enabled

    def excepts_diagnosis(self, e: Exception) -> bool:
        if not self.is_enabled:
            return False
        is_target = isinstance(
            e,
            (
                FileNotFoundError,  # JSON/CSV ファイルが存在しない
                PermissionError,  # 読み取り権限なし
                IsADirectoryError,  # パス先がディレクトリ
                NotADirectoryError,  # パスの一部がディレクトリでない
                OSError,  # デバイス・I/O エラー
                UnicodeDecodeError,  # エンコーディング不正
                json.JSONDecodeError,  # JSON/JSONC 構文破損
                ValueError,  # 必須フィールド欠落・型不正
                KeyError,  # 必須キーが存在しない
                RuntimeError,  # C++ 側の CSV 読み込み失敗
            ),
        )
        if is_target:
            self.increment_counter()
        return is_target

    def detect_error(self, *args: object) -> bool:
        return False

    def detect_recovery_error(self, *args: object) -> bool:
        return True

    def detect_recovery_fail_safe(self, *args: object) -> bool:
        return True

    def log_output(self, err: bool, recover: bool, err_idx: int, *args: object) -> None:
        if err:
            self._error_log_output(err_idx, *args)

    def _error_log_output(self, err_idx: int, *args: object) -> None:
        if len(args) != 1 or not isinstance(args[0], Exception):
            raise ValueError("args must be (Exception,)")
        self._logger.error(
            self.get_error_no(err_idx)
            + f": CRANE_MODEL_FILE_MISSING: {args[0]!r}",
            exc_info=True,
        )


class ConfigFileMissingDiagnosis(ActionErrorDiagnosisA):
    """CONFIG_FILE_MISSING: 設定ファイル欠損/破損"""

    def __init__(self) -> None:
        super().__init__()
        self.param: err_conf.ConfigFileMissingParameters
        self._last_counted_mono: float = 0.0
        self._last_counted_signature: tuple[str, str] | None = None
        self._counter_throttle_sec: float = 1.0

    def update(self, err_conf: err_conf.ErrorConfig) -> None:
        self.param = err_conf.config_file_missing
        self.is_enabled = self.param.is_enabled

    def _io_error(self, e: Exception) -> bool:
        return bool(
            isinstance(
                e,
                (
                    FileNotFoundError,  # パスが存在しない。
                    PermissionError,  # 読み取り権限が無い。
                    IsADirectoryError,  # ファイルじゃなくディレクトリだった
                    NotADirectoryError,  # パスの一部がディレクトリじゃなかった
                    OSError,  # デバイス・I/O エラー、パス長、ファイルシステム不調など
                ),
            )
        )

    def _unicode_error(self, e: Exception) -> bool:
        return bool(
            isinstance(
                e,
                (
                    UnicodeDecodeError,  # バイナリだった
                    UnicodeError,  # 文字コード異常全般
                ),
            )
        )

    def _config_parse_error(self, e: Exception) -> bool:
        return bool(
            isinstance(
                e,
                (
                    NoOptionError,  # 要素が存在しない
                    NoSectionError,  # セクションが存在しない
                    MissingSectionHeaderError,  # セクションヘッダ欠損
                    ParsingError,  # ini構文エラー
                    DuplicateOptionError,  # option重複
                    DuplicateSectionError,  # section重複
                    ValueError,  # getboolean/getint などの型変換エラー
                ),
            )
        )

    def _should_increment_counter(self, e: Exception) -> bool:
        now_mono = time.monotonic()
        error_message = str(e)
        signature = (
            type(e).__name__,
            error_message.splitlines()[0] if error_message else "",
        )
        elapsed = now_mono - self._last_counted_mono

        if (
            self._last_counted_signature == signature
            and elapsed < self._counter_throttle_sec
        ):
            return False

        self._last_counted_mono = now_mono
        self._last_counted_signature = signature
        return True

    def excepts_diagnosis(self, e: Exception) -> bool:
        if not self.is_enabled:
            return False
        ret: bool = False
        ret = self._io_error(e) or self._unicode_error(e) or self._config_parse_error(e)
        if ret and self._should_increment_counter(e):
            self.increment_counter()
        return ret

    def detect_error(self, *args: object) -> bool:
        # 未使用
        return False

    def detect_recovery_error(self, *args: object) -> bool:
        # 未使用
        return True

    def detect_recovery_fail_safe(self, *args: object) -> bool:
        # 未使用
        return True

    def log_output(self, err: bool, recover: bool, err_idx: int, *args: object) -> None:
        if err:
            self._error_log_output(err_idx, *args)

    def _error_log_output(self, err_idx: int, *args: object) -> None:
        if len(args) != 1 or not isinstance(args[0], Exception):
            raise ValueError("args must be (Exception,)")
        error = args[0]
        error_message = str(error)
        self._logger.error(
            self.get_error_no(err_idx)
            + ": CONFIG_FILE_MISSING: "
            + f"{type(error).__name__}: "
            + (error_message.splitlines()[0] if error_message else ""),
            exc_info=True,
        )


class SensorCalibDataInvalidDiagnosis(ActionErrorDiagnosisB):
    """SENSOR_CALIB_DATA_INVALID: センサ校正データ不正"""

    def __init__(self) -> None:
        super().__init__()
        self.param = err_conf.SensorCalibDataInvalidParameters()

    def update(self, err_conf: err_conf.ErrorConfig) -> None:
        self.param = err_conf.sensor_calib_data_invalid
        self.is_enabled = self.param.is_enabled

    def validate_calibration_matrices(
        self,
        calibration_conf: CalibrationConf,
        lidar2crane_reference_paths: list[str] | None = None,
    ) -> tuple[LidarCalibValidationIssue, ...]:
        if not self.is_enabled:
            return ()
        issues = LidarCalibValidator(self.param).validate(
            calibration_conf,
            lidar2crane_reference_paths=lidar2crane_reference_paths,
        )
        if issues:
            self.increment_counter()
        return issues

    def diagnose_calibration_matrices(
        self,
        calibration_conf: CalibrationConf,
        err_idx: int,
        lidar2crane_reference_paths: list[str] | None = None,
    ) -> tuple[LidarCalibValidationIssue, ...]:
        issues = self.validate_calibration_matrices(
            calibration_conf,
            lidar2crane_reference_paths=lidar2crane_reference_paths,
        )
        self.log_output(bool(issues), False, err_idx, issues)
        return issues

    def validate_matrices(
        self,
        matrices: list[NDArray[np.float64]] | NDArray[np.float64],
        matrix_paths: list[str] | None = None,
    ) -> tuple[LidarCalibValidationIssue, ...]:
        if not self.is_enabled:
            return ()
        issues = tuple(
            LidarCalibValidator(self.param).validate_matrices(
                matrices,
                matrix_paths=matrix_paths,
            )
        )
        if issues:
            self.increment_counter()
        return issues

    def diagnose_matrices(
        self,
        matrices: list[NDArray[np.float64]] | NDArray[np.float64],
        err_idx: int,
        matrix_paths: list[str] | None = None,
        reference_matrix_paths: list[str] | None = None,
    ) -> tuple[LidarCalibValidationIssue, ...]:
        if not self.is_enabled:
            return ()
        issues = tuple(
            LidarCalibValidator(self.param).validate_matrices(
                matrices,
                matrix_paths=matrix_paths,
                reference_matrix_paths=reference_matrix_paths,
            )
        )
        if issues:
            self.increment_counter()
        self.log_output(bool(issues), False, err_idx, issues)
        return issues

    def excepts_diagnosis(self, e: Exception) -> bool:
        is_target = isinstance(e, (OSError, UnicodeError, ValueError))
        if is_target:
            self.increment_counter()
        return is_target

    def detect_error(self, *args: object) -> bool:
        return False

    def detect_recovery_error(self, *args: object) -> bool:
        return True

    def detect_recovery_fail_safe(self, *args: object) -> bool:
        return True

    def log_output(self, err: bool, recover: bool, err_idx: int, *args: object) -> None:
        if err:
            self._error_log_output(err_idx, *args)

    def _error_log_output(self, err_idx: int, *args: object) -> None:
        if len(args) == 1 and isinstance(args[0], Exception):
            error = args[0]
            self._logger.error(
                self.get_error_no(err_idx)
                + ": SENSOR_CALIB_DATA_INVALID: validation failed: "
                + f"{type(error).__name__}: {error}",
                exc_info=True,
            )
            return
        if len(args) != 1 or not isinstance(args[0], tuple):
            raise ValueError(
                "args must be (Exception,) or "
                "(tuple[LidarCalibValidationIssue, ...],)"
            )
        issue_args = cast(tuple[object, ...], args[0])
        if not issue_args or not all(
            isinstance(issue, LidarCalibValidationIssue) for issue in issue_args
        ):
            raise ValueError("args must be (tuple[LidarCalibValidationIssue, ...],)")
        issues = cast(tuple[LidarCalibValidationIssue, ...], issue_args)
        first = issues[0]
        self._logger.error(
            self.get_error_no(err_idx)
            + ": SENSOR_CALIB_DATA_INVALID: "
            + f"kind={first.matrix_kind} path={first.matrix_path} "
            + f"detail={first.detail} issues={len(issues)}"
        )


class CameraXCalibDataInvalidDiagnosis(ActionErrorDiagnosisB):
    """CAMERA_N_CALIB_DATA_INVALID: カメラN校正データ不正"""

    def __init__(self) -> None:
        super().__init__()
        self.param = err_conf.CameraNCalibDataInvalidParameters()

    def update(self, err_conf: err_conf.ErrorConfig) -> None:
        self.param = err_conf.camera_n_calib_data_invalid
        self.is_enabled = self.param.is_enabled

    def validate_calibration_data(
        self, camera_index: int, calibration_conf: CalibrationConf
    ) -> tuple[CameraCalibValidationIssue, ...]:
        if not self.is_enabled:
            return ()
        issues = CameraCalibValidator(self.param).validate(
            camera_index, calibration_conf
        )
        if issues:
            self.increment_counter()
        return issues

    def diagnose_calibration_data(
        self,
        camera_index: int,
        calibration_conf: CalibrationConf,
        err_idx: int,
    ) -> tuple[CameraCalibValidationIssue, ...]:
        issues = self.validate_calibration_data(camera_index, calibration_conf)
        self.log_output(bool(issues), False, err_idx, issues)
        return issues

    def excepts_diagnosis(self, e: Exception) -> bool:
        is_target = isinstance(e, (OSError, UnicodeError, ValueError))
        if is_target:
            self.increment_counter()
        return is_target

    def detect_error(self, *args: object) -> bool:
        return False

    def detect_recovery_error(self, *args: object) -> bool:
        return True

    def detect_recovery_fail_safe(self, *args: object) -> bool:
        return True

    def log_output(self, err: bool, recover: bool, err_idx: int, *args: object) -> None:
        if err:
            self._error_log_output(err_idx, *args)

    def _error_log_output(self, err_idx: int, *args: object) -> None:
        if len(args) == 1 and isinstance(args[0], Exception):
            error = args[0]
            self._logger.error(
                self.get_error_no(err_idx)
                + ": CAMERA_CALIB_DATA_INVALID: validation failed: "
                + f"{type(error).__name__}: {error}",
                exc_info=True,
            )
            return
        if len(args) != 1 or not isinstance(args[0], tuple):
            raise ValueError(
                "args must be (Exception,) or "
                "(tuple[CameraCalibValidationIssue, ...],)"
            )
        issue_args = cast(tuple[object, ...], args[0])
        if not issue_args or not all(
            isinstance(issue, CameraCalibValidationIssue) for issue in issue_args
        ):
            raise ValueError("args must be (tuple[CameraCalibValidationIssue, ...],)")
        issues = cast(tuple[CameraCalibValidationIssue, ...], issue_args)
        first = issues[0]
        self._logger.error(
            self.get_error_no(err_idx)
            + ": CAMERA_CALIB_DATA_INVALID: "
            + f"camera={first.camera_index} kind={first.kind} path={first.path} "
            + f"detail={first.detail} issues={len(issues)}"
        )


class MmapReadWriteErrorDiagnosis(ActionErrorDiagnosisB):
    """MMAP_READ_WRITE_ERROR: MMAP read/writeエラー"""

    def __init__(self) -> None:
        super().__init__()
        self.param: err_conf.MmapReadWriteErrorParameters

    def update(self, err_conf: err_conf.ErrorConfig) -> None:
        self.param = err_conf.mmap_read_write_error
        self.is_enabled = self.param.is_enabled

    def excepts_diagnosis(self, e: Exception) -> bool:
        if not self.is_enabled:
            return False
        is_target = isinstance(
            e,
            (
                OSError,  # ファイルシステム障害・ディスク容量不足 (mmap.error を含む)
                ValueError,  # mmap クローズ/無効状態でのアクセス・範囲外
                BufferError,  # バッファ競合
                RuntimeError,  # C++ 側 (ErrorMMapWriter) の内部エラー
            ),
        )
        if is_target:
            self.increment_counter()
        return is_target

    def detect_error(self, *args: object) -> bool:
        return False

    def detect_recovery_error(self, *args: object) -> bool:
        return True

    def detect_recovery_fail_safe(self, *args: object) -> bool:
        return True

    def log_output(self, err: bool, recover: bool, err_idx: int, *args: object) -> None:
        if err:
            self._error_log_output(err_idx, *args)

    def _error_log_output(self, err_idx: int, *args: object) -> None:
        if (
            len(args) != 2
            or not isinstance(args[0], str)
            or not isinstance(args[1], Exception)
        ):
            raise ValueError("args must be (operation, Exception)")
        self._logger.error(
            self.get_error_no(err_idx)
            + f": MMAP_READ_WRITE_ERROR: {args[0]}: {args[1]!r}",
            exc_info=True,
        )


class RebootLoopDetectedDiagnosis(ActionErrorDiagnosisA):
    """REBOOT_LOOP_DETECTED: 再起動ループ検出"""

    def __init__(self, runtime_policy: DiagnosisRuntimePolicy | None = None) -> None:
        super().__init__(runtime_policy)
        self.param: err_conf.RebootLoopDetectedParameters

    def update(self, err_conf: err_conf.ErrorConfig) -> None:
        self.param = err_conf.reboot_loop_detected
        self.is_enabled = self.param.is_enabled

    def _parse_args(self, *args: object) -> list[float]:
        if len(args) != 1 or not isinstance(args[0], list):
            raise ValueError("args must be (boot_times,)")
        boot_times = cast(list[object], args[0])
        if not all(isinstance(boot_time, float) for boot_time in boot_times):
            raise ValueError("boot_times must be list[float]")
        return cast(list[float], boot_times)

    def detect_error(self, *args: object) -> bool:
        boot_times = self._parse_args(*args)
        # メンテナンス作業に伴う再起動は新規エラーとして扱わない。
        if self.is_suppressed_in_maintenance:
            return False
        if len(boot_times) < self.param.required_boot_count:
            return False
        # 直近N回の起動がW秒以内なら再起動ループとする。Nは初回起動を含む起動記録数。
        recent_boot_times = boot_times[: self.param.required_boot_count]
        return max(recent_boot_times) - min(recent_boot_times) <= self.param.window_sec

    def detect_recovery_error(self, *args: object) -> bool:
        return True

    def detect_recovery_fail_safe(self, *args: object) -> bool:
        return True

    def log_output(self, err: bool, recover: bool, err_idx: int, *args: object) -> None:
        if err:
            self._error_log_output(err_idx, *args)

    def _error_log_output(self, err_idx: int, *args: object) -> None:
        boot_times = self._parse_args(*args)
        recent_boot_times = boot_times[: self.param.required_boot_count]
        observed_span_sec = max(recent_boot_times) - min(recent_boot_times)
        self._logger.warning(
            self.get_error_no(err_idx)
            + ": 再起動ループを検出しました: "
            + f"boot_count={self.param.required_boot_count}, "
            + f"window_sec={self.param.window_sec:.1f}, "
            + f"observed_span_sec={observed_span_sec:.1f}"
        )


class AiModelLoadFailed(ActionErrorDiagnosisB):
    """AI_MODEL_LOAD_FAILED: AIモデルロード失敗/破損"""

    def __init__(self) -> None:
        super().__init__()
        self.param: err_conf.AiModelLoadFailedParameters

    def update(self, err_conf: err_conf.ErrorConfig) -> None:
        self.param = err_conf.ai_model_load_failed
        self.is_enabled = self.param.is_enabled

    def excepts_diagnosis(self, e: Exception) -> bool:
        if not self.is_enabled:
            return False
        is_target = isinstance(
            e,
            (
                FileNotFoundError,  # パスが存在しない。
                PermissionError,  # 読み取り権限が無い。
                OSError,  # デバイス・I/O エラー、パス長、ファイルシステム不調など
                RuntimeError,  # 実行時エラー
                ValueError,  # 値エラー
                ImportError,  # インポートエラー
                ModuleNotFoundError,  # モジュールが見つからない
            ),
        )
        if is_target:
            self.increment_counter()
        return is_target

    def detect_error(self, *args: object) -> bool:
        return False

    def detect_recovery_error(self, *args: object) -> bool:
        return True

    def detect_recovery_fail_safe(self, *args: object) -> bool:
        return True

    def log_output(self, err: bool, recover: bool, err_idx: int, *args: object) -> None:
        if err:
            self._error_log_output(err_idx, *args)

    def _error_log_output(self, err_idx: int, *args: object) -> None:
        if len(args) != 1 or not isinstance(args[0], Exception):
            raise ValueError("args must be (Exception,)")
        self._logger.error(
            self.get_error_no(err_idx)
            + f": AI model initialization failed: {args[0]!r}"
        )


class OperationModeTransitionErrorDiagnosis(ActionErrorDiagnosisB):
    """OPERATION_MODE_TRANSITION_ERROR: 動作モード遷移エラー"""

    def __init__(self) -> None:
        super().__init__()

    def detect_error(self, *args: object) -> bool:
        return False

    def excepts_diagnosis(self, e: Exception) -> bool:
        ret = isinstance(e, RuntimeError)
        if ret:
            self.increment_counter()
        return ret

    def detect_recovery_error(self, *args: object) -> bool:
        return True

    def detect_recovery_fail_safe(self, *args: object) -> bool:
        return True

    def log_output(self, err: bool, recover: bool, err_idx: int, *args: object) -> None:
        if err:
            self._error_log_output(err_idx)

    def _error_log_output(self, err_idx: int, *args: object) -> None:
        self._logger.warning(
            self.get_error_no(err_idx)
            + ": 動作モード遷移エラー:動作モード遷移中にエラーが発生しました。再起動します。セットアップ実行中の場合は、初めからやり直して下さい。"
        )


class LogFileIoErrorDiagnosis(ActionErrorDiagnosisB):
    """LOG_FILE_IO_ERROR: ログファイルI/Oエラー"""

    def __init__(self) -> None:
        super().__init__()
        self.param: err_conf.LogFileIoErrorParameters
        self._last_counted_mono: float = 0.0
        self._last_counted_signature: tuple[str, str] | None = None
        self._counter_throttle_sec: float = 1.0

    def update(self, err_conf: err_conf.ErrorConfig) -> None:
        self.param = err_conf.log_file_io_error
        self.is_enabled = self.param.is_enabled

    def _should_increment_counter(self, e: Exception) -> bool:
        now_mono = time.monotonic()
        signature = (type(e).__name__, str(e).splitlines()[0] if str(e) else "")
        elapsed = now_mono - self._last_counted_mono
        if (
            self._last_counted_signature == signature
            and elapsed < self._counter_throttle_sec
        ):
            return False
        self._last_counted_mono = now_mono
        self._last_counted_signature = signature
        return True

    def excepts_diagnosis(self, e: Exception) -> bool:
        if not self.is_enabled:
            return False
        if isinstance(
            e,
            (
                OSError,  # ディスク容量不足・権限エラー・デバイスI/Oエラー (PermissionError/FileNotFoundError含む)
            ),
        ):
            if self._should_increment_counter(e):
                self.increment_counter()
            return True
        return False

    def detect_error(self, *args: object) -> bool:
        return False

    def detect_recovery_error(self, *args: object) -> bool:
        return True

    def detect_recovery_fail_safe(self, *args: object) -> bool:
        return True


class ReserveActionABC(ActionErrorDiagnosisC):
    def __init__(self) -> None:
        super().__init__()

    def detect_error(self, *args: object) -> bool:
        return False

    def detect_recovery_error(self, *args: object) -> bool:
        return True

    def detect_recovery_fail_safe(self, *args: object) -> bool:
        return True


class ProcessStartupErrorDiagnosis(ActionErrorDiagnosisB):
    """PROCESS_STARTUP_ERROR: プロセス起動エラー"""

    def __init__(self) -> None:
        super().__init__()

    def detect_error(self, *args: object) -> bool:
        return False

    def excepts_diagnosis(self, e: Exception) -> bool:
        ret = isinstance(e, RuntimeError)
        if ret:
            self.increment_counter()
        return ret

    def detect_recovery_error(self, *args: object) -> bool:
        return True

    def detect_recovery_fail_safe(self, *args: object) -> bool:
        return True

    def log_output(self, err: bool, recover: bool, err_idx: int, *args: object) -> None:
        if err:
            self._error_log_output(err_idx)

    def _error_log_output(self, err_idx: int, *args: object) -> None:
        self._logger.warning(
            self.get_error_no(err_idx)
            + ": プロセス起動エラー:プロセス起動中にエラーが発生しました。再起動します。セットアップ実行中の場合は、初めからやり直して下さい。"
        )
