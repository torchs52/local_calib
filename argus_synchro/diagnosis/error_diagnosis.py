from __future__ import annotations

from abc import ABC, abstractmethod
from enum import IntEnum, auto
from multiprocessing.sharedctypes import Synchronized

from argus_synchro.common.app_logger import AppLogger, AppLoggerFactory
from argus_synchro.diagnosis.error_config import ErrorConfig
from argus_synchro.shared_data import create_shared_single_data


class DiagnosisRuntimePolicy:
    """診断のメンテナンスモード制御をプロセス間で共有する。

    ``in_factory`` は既存設定キー名だが、工場と市場でのサービス作業を含む
    製品仕様上の正式名称は「メンテナンスモード」である。
    """

    def __init__(self, *, in_factory: bool = False) -> None:
        self._in_factory: Synchronized[bool] = create_shared_single_data(in_factory)

    @property
    def in_factory(self) -> bool:
        return bool(self._in_factory.value)

    def update_in_factory(self, in_factory: bool) -> None:
        self._in_factory.value = in_factory


class ResultDiagnosis(IntEnum):
    """
    診断結果
    """

    NORMAL = auto()
    """正常状態継続"""
    DETECTION = auto()
    """エラー検出"""
    KEEPING = auto()
    """エラー継続"""
    RECOVERY = auto()
    """正常復帰"""

    @classmethod
    def get_result(cls, before: bool, detect: bool, recover: bool) -> ResultDiagnosis:
        result: ResultDiagnosis
        if not before:
            result = ResultDiagnosis.DETECTION if detect else ResultDiagnosis.NORMAL
        else:
            result = ResultDiagnosis.RECOVERY if recover else ResultDiagnosis.KEEPING
        return result


class StateErrorDiagnosisBase(ABC):
    def __init__(self, runtime_policy: DiagnosisRuntimePolicy | None = None) -> None:
        super().__init__()
        self._logger: AppLogger = AppLoggerFactory.from_type(self.__class__)
        self.is_enabled: bool = False
        self._runtime_policy = runtime_policy

    @property
    def is_suppressed_in_maintenance(self) -> bool:
        """メンテナンスモード中に対象診断の新規検出を抑制する。"""
        return self._runtime_policy is not None and self._runtime_policy.in_factory

    @abstractmethod
    def excepts_diagnosis(self, e: Exception) -> bool:
        """
        例外検出
        """
        ...

    def update(self, err_conf: ErrorConfig) -> None:
        """
        判定用しきい値の更新
        """
        pass

    def log_register(self, app_logger_factory: AppLoggerFactory) -> None:
        self._app_logger_factory: AppLoggerFactory = app_logger_factory
        app_logger_factory.append_logger(self._logger)

    def log_output(
        self,
        err_result: ResultDiagnosis,
        failsafe_result: ResultDiagnosis,
        err_idx: int,
        *args: object,
    ) -> None:
        """
        エラーログ出力
        """
        if err_result == ResultDiagnosis.DETECTION:
            self._error_log_output(err_idx, *args)
        elif err_result == ResultDiagnosis.RECOVERY:
            self._recover_log_output(err_idx, *args)
        if failsafe_result == ResultDiagnosis.RECOVERY:
            self._fail_safe_recover_log_output(err_idx, *args)

    def _error_log_output(self, err_idx: int, *args: object) -> None:
        """
        エラー検知時ログ出力
        """
        return

    def _recover_log_output(self, err_idx: int, *args: object) -> None:
        """
        復帰ログ出力
        """
        return

    def _fail_safe_recover_log_output(self, err_idx: int, *args: object) -> None:
        """
        フェイルセーフ復帰ログ出力
        """
        return

    def clear(self) -> None:
        """
        診断用のインスタンス変数を初期化する
        """
        pass

    @classmethod
    def get_error_no(cls, index: int) -> str:
        """
        エラー番号取得
        """
        return "SE" + str(index + 1).zfill(3)


class StateErrorDiagnosisA(StateErrorDiagnosisBase):
    def __init__(self, runtime_policy: DiagnosisRuntimePolicy | None = None) -> None:
        super().__init__(runtime_policy)
        self.is_error: Synchronized[bool] = create_shared_single_data(False)
        """
        エラーフラグ
        """
        self.is_fail_safe: Synchronized[bool] = create_shared_single_data(False)
        """
        エラー時動作フラグ
        """
        self.is_idle: Synchronized[bool] = create_shared_single_data(False)
        """
        アイドル状態フラグ
        """

    def excepts_diagnosis(self, e: Exception) -> bool:
        raise NotImplementedError("必要なら実装")

    def errors_diagnosis(
        self, *args: object
    ) -> tuple[ResultDiagnosis, ResultDiagnosis]:
        """
        エラー診断
        """
        if self.is_enabled is False:
            # 診断無効
            return ResultDiagnosis.NORMAL, ResultDiagnosis.NORMAL

        before_err: bool = self.is_error.value
        before_failsafe: bool = self.is_fail_safe.value

        err: bool = self.detect_error(*args)
        recover_fs: bool = self.detect_recovery_fail_safe(*args)
        recover_err: bool = self.detect_recovery_error(*args)

        if (err and recover_fs) or (err and recover_err):
            # エラー検出時の処理
            raise RuntimeError("エラー検出とエラー復帰が同時に発生しています。")

        result_err: ResultDiagnosis = ResultDiagnosis.get_result(
            before_err, err, recover_err
        )
        result_failsafe: ResultDiagnosis = ResultDiagnosis.get_result(
            before_failsafe, err, recover_fs
        )
        return result_err, result_failsafe

    def reset_error(self) -> None:
        self.is_error.value = False

    @abstractmethod
    def detect_error(self, *args: object) -> bool:
        """
        エラー検知
        """
        ...

    @abstractmethod
    def detect_recovery_error(self, *args: object) -> bool:
        """
        正常復帰判定
        """
        ...

    @abstractmethod
    def detect_recovery_fail_safe(self, *args: object) -> bool:
        """
        エラー時動作復帰判定
        """
        ...


class StateErrorDiagnosisB(StateErrorDiagnosisBase):
    def __init__(self) -> None:
        super().__init__()
        self.is_error: Synchronized[bool] = create_shared_single_data(False)
        self.is_fail_safe: Synchronized[bool] = create_shared_single_data(False)

    def errors_diagnosis(
        self, *args: object
    ) -> tuple[ResultDiagnosis, ResultDiagnosis]:
        """
        エラー診断
        """
        if self.is_enabled is False:
            # 診断無効
            return ResultDiagnosis.NORMAL, ResultDiagnosis.NORMAL

        before_err: bool = self.is_error.value
        before_failsafe: bool = self.is_fail_safe.value

        err: bool = self.detect_error(*args)
        failsafe: bool = self.detect_recovery_fail_safe(*args)
        recover: bool = self.detect_recovery_error(*args)

        if (err and failsafe) or (err and recover):
            # エラー検出時の処理
            raise RuntimeError("エラー検出とエラー復帰が同時に発生しています。")

        result_err: ResultDiagnosis = ResultDiagnosis.get_result(
            before_err, err, recover
        )
        result_failsafe: ResultDiagnosis = ResultDiagnosis.get_result(
            before_failsafe, err, failsafe
        )
        return result_err, result_failsafe

    def reset_error(self) -> None:
        self.is_error.value = False

    def excepts_diagnosis(self, e: Exception) -> bool:
        raise NotImplementedError("必要なら実装")

    @abstractmethod
    def detect_error(self, *args: object) -> bool:
        """
        エラー検知
        """
        ...

    @abstractmethod
    def detect_recovery_error(self, *args: object) -> bool:
        """
        正常復帰判定
        """
        ...

    @abstractmethod
    def detect_recovery_fail_safe(self, *args: object) -> bool:
        """
        エラー時動作復帰判定
        """
        ...


class StateErrorDiagnosisC(StateErrorDiagnosisBase):
    def __init__(self) -> None:
        super().__init__()
        self.is_error: Synchronized[bool] = create_shared_single_data(False)
        self.is_fail_safe: Synchronized[bool] = create_shared_single_data(False)

    def errors_diagnosis(
        self, *args: object
    ) -> tuple[ResultDiagnosis, ResultDiagnosis]:
        """
        エラー診断
        """
        if self.is_enabled is False:
            # 診断無効
            return ResultDiagnosis.NORMAL, ResultDiagnosis.NORMAL

        before_err: bool = self.is_error.value
        before_failsafe: bool = self.is_fail_safe.value

        err: bool = self.detect_error(*args)
        failsafe: bool = self.detect_recovery_fail_safe(*args)
        recover: bool = self.detect_recovery_error(*args)

        if (err and failsafe) or (err and recover):
            # エラー検出時の処理
            raise RuntimeError("エラー検出とエラー復帰が同時に発生しています。")

        result_err: ResultDiagnosis = ResultDiagnosis.get_result(
            before_err, err, recover
        )
        result_failsafe: ResultDiagnosis = ResultDiagnosis.get_result(
            before_failsafe, err, failsafe
        )
        return result_err, result_failsafe

    def reset_error(self) -> None:
        self.is_error.value = False

    def excepts_diagnosis(self, e: Exception) -> bool:
        raise NotImplementedError("必要なら実装")

    @abstractmethod
    def detect_error(self, *args: object) -> bool:
        """
        エラー検知
        """
        ...

    @abstractmethod
    def detect_recovery_error(self, *args: object) -> bool:
        """
        正常復帰判定
        """
        ...

    @abstractmethod
    def detect_recovery_fail_safe(self, *args: object) -> bool:
        """
        エラー時動作復帰判定
        """
        ...


class StateErrorDiagnosisD(StateErrorDiagnosisBase):
    def __init__(self) -> None:
        super().__init__()
        self._was_detected: bool = False
        self._last_signature: tuple[str, str] | None = None
        self._last_log_mono: float | None = None
        self._ongoing_log_interval_sec: float = 60.0

    def _should_log(self, e: Exception, now_mono: float) -> tuple[bool, bool]:
        signature = (type(e).__name__, str(e).splitlines()[0] if str(e) else "")
        include_traceback = self._last_signature != signature
        if include_traceback:
            self._last_signature = signature
            self._last_log_mono = now_mono
            return True, True
        if (
            self._last_log_mono is None
            or now_mono - self._last_log_mono >= self._ongoing_log_interval_sec
        ):
            self._last_log_mono = now_mono
            return True, False
        return False, False

    def reset_error(self) -> None:
        self._was_detected = False

    def errors_diagnosis(
        self, *args: object
    ) -> tuple[ResultDiagnosis, ResultDiagnosis]:
        """
        エラー診断（検出/復帰のエッジでのみDETECTION/RECOVERYを返し、継続中の重複ログを防ぐ）
        """
        no_error: tuple[ResultDiagnosis, ResultDiagnosis] = (
            ResultDiagnosis.NORMAL,
            ResultDiagnosis.NORMAL,
        )
        if self.is_enabled is False:
            # 診断無効
            return no_error

        is_detected: bool = self.detect_error(*args)
        result = ResultDiagnosis.get_result(
            self._was_detected,
            is_detected,
            not is_detected,
        )
        self._was_detected = is_detected
        return result, ResultDiagnosis.NORMAL

    def excepts_diagnosis(self, e: Exception) -> bool:
        raise NotImplementedError("必要なら実装")

    def detect_error(self, *args: object) -> bool:
        """
        エラー検知
        """
        raise NotImplementedError("必要なら実装")


class ActionErrorDiagnosisBase(ABC):
    def __init__(self, runtime_policy: DiagnosisRuntimePolicy | None = None) -> None:
        super().__init__()
        self._logger: AppLogger = AppLoggerFactory.from_type(self.__class__)
        self._runtime_policy = runtime_policy
        self.is_enabled: bool = False
        self.err_cnt: Synchronized[int] = create_shared_single_data(0)
        """
        エラーカウンタ
        """
        self.is_fail_safe: Synchronized[bool] = create_shared_single_data(False)
        """
        エラー時動作フラグ
        """
        self.diag_param: dict[str, object] = {}
        self.is_enabled: bool = False

    @property
    def is_suppressed_in_maintenance(self) -> bool:
        """メンテナンスモード中に対象診断の新規検出を抑制する。"""
        return self._runtime_policy is not None and self._runtime_policy.in_factory

    @abstractmethod
    def detect_error(self, *args: object) -> bool:
        """
        エラー検知
        """
        ...

    @abstractmethod
    def detect_recovery_error(self, *args: object) -> bool:
        """
        正常復帰判定
        """
        ...

    @abstractmethod
    def detect_recovery_fail_safe(self, *args: object) -> bool:
        """
        エラー時動作復帰判定
        """
        ...

    def update(self, err_conf: ErrorConfig) -> None:
        """
        判定用しきい値の更新
        """
        pass

    def reset_error(self) -> None:
        self.is_fail_safe.value = False

    def increment_counter(self, num_bits: int = 8) -> None:
        max_v = (1 << num_bits) - 1
        incremented = (self.err_cnt.value & max_v) + 1
        self.err_cnt.value = (incremented & max_v) | (incremented >> num_bits)

    def errors_diagnosis(self, *args: object) -> tuple[bool, bool, bool]:
        """
        エラー診断
        """
        if self.is_enabled is False:
            # 診断無効
            return False, False, False

        err = self.detect_error(*args)
        failsafe = self.detect_recovery_fail_safe(*args)
        recover = self.detect_recovery_error(*args)
        return err, failsafe, recover

    @abstractmethod
    def excepts_diagnosis(self, e: Exception) -> bool: ...

    def log_register(self, app_logger_factory: AppLoggerFactory) -> None:
        self._app_logger_factory: AppLoggerFactory = app_logger_factory
        app_logger_factory.append_logger(self._logger)

    def log_output(self, err: bool, recover: bool, err_idx: int, *args: object) -> None:
        """
        エラーログ出力
        """
        raise NotImplementedError("必要なら実装")

    def _error_log_output(self, err_idx: int, *args: object) -> None:
        """
        エラーログ出力
        """
        raise NotImplementedError("必要なら実装")

    def _recover_log_output(self, err_idx: int, *args: object) -> None:
        """
        復帰ログ出力
        """
        raise NotImplementedError("必要なら実装")

    def _fail_safe_recover_log_output(self, err_idx: int, *args: object) -> None:
        """
        フェイルセーフ復帰ログ出力
        """
        raise NotImplementedError("必要なら実装")

    @classmethod
    def get_error_no(cls, index: int) -> str:
        """
        エラー番号取得
        """
        return "CE" + str(index + 1).zfill(3)


class ActionErrorDiagnosisA(ActionErrorDiagnosisBase):
    def __init__(self, runtime_policy: DiagnosisRuntimePolicy | None = None) -> None:
        super().__init__(runtime_policy)
        self.is_idle: Synchronized[bool] = create_shared_single_data(False)
        """
        アイドル状態フラグ
        """

    def errors_diagnosis(self, *args: object) -> tuple[bool, bool, bool]:
        if self.is_enabled is False:
            # 診断無効
            return False, False, False
        err = self.detect_error(*args)
        failsafe = self.detect_recovery_fail_safe(*args)
        recover = self.detect_recovery_error(*args)
        return err, failsafe, recover

    def excepts_diagnosis(self, e: Exception) -> bool:
        raise NotImplementedError("必要なら実装")


class ActionErrorDiagnosisB(ActionErrorDiagnosisBase):
    def __init__(self) -> None:
        super().__init__()

    def errors_diagnosis(self, *args: object) -> tuple[bool, bool, bool]:
        if self.is_enabled is False:
            # 診断無効
            return False, False, False
        err = self.detect_error(*args)
        failsafe = self.detect_recovery_fail_safe(*args)
        recover = self.detect_recovery_error(*args)
        return err, failsafe, recover

    def excepts_diagnosis(self, e: Exception) -> bool:
        raise NotImplementedError("必要なら実装")


class ActionErrorDiagnosisC(ActionErrorDiagnosisBase):
    def __init__(self) -> None:
        super().__init__()

    def errors_diagnosis(self, *args: object) -> tuple[bool, bool, bool]:
        if self.is_enabled is False:
            # 診断無効
            return False, False, False
        err = self.detect_error(*args)
        failsafe = self.detect_recovery_fail_safe(*args)
        recover = self.detect_recovery_error(*args)
        return err, failsafe, recover

    def excepts_diagnosis(self, e: Exception) -> bool:
        raise NotImplementedError("必要なら実装")
