"""2D-3D本校正固有のUI診断コードと診断セッション。

このモジュールが扱う値は、重要度AからDのシステムエラーとは別枠である。
校正処理から得た診断結果を保持し、校正UIへ公開するコードへ変換する。
画像処理、追跡、校正計算、MMAP操作、処理終了制御は担当しない。
"""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass, field
from enum import Enum, IntEnum, auto

import numpy as np
from numpy.typing import NDArray


class Calib2d3dErrorCommon(IntEnum):
    """2D-3D校正のerrors_calibcommonに設定する値。必ずいずれか1つを設定する。"""

    # 0から15は警告メッセージを表示するが、校正処理の継続は可能。
    DEFAULT = 0  # デフォルト
    CALIBRATION_SUCCEEDED = 1  # 校正成功
    WALKING_RANGE_INVALID = 2  # 歩行範囲が不適切
    WALKING_PERSON_COUNT_INVALID = 3  # 歩行人数が不適切
    POOR_PERSON_DETECTION = 4  # 人検知不良
    POOR_TRACKING_3D = 5  # 3D側の追跡不具合
    POOR_TRACKING_2D = 6  # 2D側の追跡不具合
    UNSUITABLE_CONDITION = 7  # 撮影条件が校正に不適切(周囲が暗いなど)
    RESERVED_8 = 8  # 予約
    RESERVED_9 = 9  # 予約
    RESERVED_10 = 10  # 予約
    RESERVED_11 = 11  # 予約
    RESERVED_12 = 12  # 予約
    RESERVED_13 = 13  # 予約
    RESERVED_14 = 14  # 予約
    RESERVED_15 = 15  # 予約

    # 16から31はUIが校正をエラー完了状態へ導く理由。
    LONG_DURATION_CALIBRATION = 16  # 長時間の校正作業
    PERSON_DETECTION_IMPOSSIBLE = 17  # 人検知不能
    TRACKING_IMPOSSIBLE = 18  # 追跡不能
    RESERVED_19 = 19  # 予約
    RESERVED_20 = 20  # 予約
    RESERVED_21 = 21  # 予約
    RESERVED_22 = 22  # 予約
    RESERVED_23 = 23  # 予約
    RESERVED_24 = 24  # 予約
    RESERVED_25 = 25  # 予約
    RESERVED_26 = 26  # 予約
    RESERVED_27 = 27  # 予約
    RESERVED_28 = 28  # 予約
    RESERVED_29 = 29  # 予約
    RESERVED_30 = 30  # 予約
    RESERVED_31 = 31  # 予約


class CameraCalibrationStatus(IntEnum):
    """カメラ別の2D-3D校正結果としてMMAPへ設定する値。必ずいずれか1つを設定する。"""

    DEFAULT = 0  # デフォルト(計算結果が出る前)
    CALIBRATION_SUCCEEDED = 1  # 校正成功
    POOR_SENSOR_CORRESPONDENCE = 2  # 複数センサの対応付け不良
    WALKING_PERSON_COUNT_INVALID = 3  # 歩行人数が不適切
    POOR_TRACKING_3D = 4  # 3D側の追跡不具合
    POOR_TRACKING_2D = 5  # 2D側の追跡不具合
    CALIBRATION_MATRIX_INVALID = 6  # 校正マトリクスデータ不正
    # 実行中の重大エラーにより校正を完了せず中断。
    # 詳細理由は同時に通知するCalib2d3dErrorCommonの16から31を参照する。
    CALIBRATION_ABORTED = 7


@dataclass(frozen=True, slots=True)
class Calib2d3dFinalDiagnosisConfig:
    """校正行列に対する最終診断を個別に無効化できる設定。"""

    enable_matrix_shape: bool = True
    enable_matrix_finite: bool = True
    enable_accuracy: bool = True
    # 参照行列との差分判定は実装済みだが、閾値確定まで既定では使用しない。
    enable_reference_matrix_difference: bool = False
    expected_matrix_shape: tuple[int, int] = (4, 4)


@dataclass(frozen=True, slots=True)
class Calib2d3dFinalObservation:
    """最終状態の決定に必要な校正処理の観測値。"""

    matrix: NDArray[np.floating]
    accuracy_value: float
    accuracy_threshold: float
    accuracy_check_enabled: bool
    poor_sensor_correspondence: bool = False
    invalid_person_count: bool = False
    poor_tracking_3d: bool = False
    poor_tracking_2d: bool = False
    reference_matrix_difference_invalid: bool | None = None


@dataclass(frozen=True, slots=True)
class Calib2d3dFinalDecision:
    """最終計算結果から決めたカメラ別状態と判定根拠。"""

    status: CameraCalibrationStatus
    details: dict[str, object]


class Calib2d3dFinalDiagnosis:
    """生成行列を正としてカメラ別の最終校正結果を決める。"""

    def __init__(self, config: Calib2d3dFinalDiagnosisConfig | None = None) -> None:
        self.config = config or Calib2d3dFinalDiagnosisConfig()

    def diagnose(
        self, observation: Calib2d3dFinalObservation
    ) -> Calib2d3dFinalDecision:
        matrix = np.asarray(observation.matrix)
        shape_valid = matrix.shape == self.config.expected_matrix_shape
        finite = bool(np.all(np.isfinite(matrix)))
        accuracy_valid = (
            # check_enable=False時に無条件成功とする既存契約を維持する。
            not observation.accuracy_check_enabled
            or observation.accuracy_value < observation.accuracy_threshold
        )
        details: dict[str, object] = {
            "matrix_shape": matrix.shape,
            "expected_matrix_shape": self.config.expected_matrix_shape,
            "matrix_finite": finite,
            "accuracy_value": observation.accuracy_value,
            "accuracy_threshold": observation.accuracy_threshold,
            "accuracy_check_enabled": observation.accuracy_check_enabled,
            "reference_matrix_difference_invalid": (
                observation.reference_matrix_difference_invalid
            ),
            # 途中の警告は成功を妨げないが、行列不正時の原因調査用に残す。
            "poor_sensor_correspondence": observation.poor_sensor_correspondence,
            "invalid_person_count": observation.invalid_person_count,
            "poor_tracking_3d": observation.poor_tracking_3d,
            "poor_tracking_2d": observation.poor_tracking_2d,
        }

        # 最終結果は生成行列を正とする。収集中の0～15の警告が残っていても、
        # 行列・精度・有効化された既存行列との差が正常なら校正成功とする。
        matrix_invalid = (
            (self.config.enable_matrix_shape and not shape_valid)
            or (self.config.enable_matrix_finite and not finite)
            or (self.config.enable_accuracy and not accuracy_valid)
            or (
                self.config.enable_reference_matrix_difference
                and observation.reference_matrix_difference_invalid is True
            )
        )
        if matrix_invalid:
            return Calib2d3dFinalDecision(
                CameraCalibrationStatus.CALIBRATION_MATRIX_INVALID, details
            )
        return Calib2d3dFinalDecision(
            CameraCalibrationStatus.CALIBRATION_SUCCEEDED, details
        )


_FATAL_COMMON_ERROR_MIN = Calib2d3dErrorCommon.LONG_DURATION_CALIBRATION
_FATAL_COMMON_ERROR_MAX = Calib2d3dErrorCommon.RESERVED_31

_COMMON_ERROR_LABELS: Mapping[Calib2d3dErrorCommon, str] = {
    Calib2d3dErrorCommon.CALIBRATION_SUCCEEDED: "校正成功",
    Calib2d3dErrorCommon.WALKING_RANGE_INVALID: "歩行範囲が不適切",
    Calib2d3dErrorCommon.WALKING_PERSON_COUNT_INVALID: "歩行人数が不適切",
    Calib2d3dErrorCommon.POOR_PERSON_DETECTION: "人検知不良",
    Calib2d3dErrorCommon.POOR_TRACKING_3D: "3D側の追跡不具合",
    Calib2d3dErrorCommon.POOR_TRACKING_2D: "2D側の追跡不具合",
    Calib2d3dErrorCommon.UNSUITABLE_CONDITION: "撮影条件が校正に不適切",
    Calib2d3dErrorCommon.LONG_DURATION_CALIBRATION: "長時間の校正作業",
    Calib2d3dErrorCommon.PERSON_DETECTION_IMPOSSIBLE: "人検知不能",
    Calib2d3dErrorCommon.TRACKING_IMPOSSIBLE: "追跡不能",
}

_CAMERA_STATUS_LABELS: Mapping[CameraCalibrationStatus, str] = {
    CameraCalibrationStatus.CALIBRATION_SUCCEEDED: "校正成功",
    CameraCalibrationStatus.POOR_SENSOR_CORRESPONDENCE: ("複数センサの対応付け不良"),
    CameraCalibrationStatus.WALKING_PERSON_COUNT_INVALID: "歩行人数が不適切",
    CameraCalibrationStatus.POOR_TRACKING_3D: "3D側の追跡不具合",
    CameraCalibrationStatus.POOR_TRACKING_2D: "2D側の追跡不具合",
    CameraCalibrationStatus.CALIBRATION_MATRIX_INVALID: ("校正マトリクスデータ不正"),
    CameraCalibrationStatus.CALIBRATION_ABORTED: "実行中の重大エラーによる中断",
}


def normalize_calib2d3d_error_common(
    value: int | Calib2d3dErrorCommon,
) -> Calib2d3dErrorCommon:
    """整数を正式な本校正共通診断コードへ変換する。"""

    try:
        return Calib2d3dErrorCommon(value)
    except ValueError as error:
        raise ValueError(f"invalid 2D-3D calibration common error: {value}") from error


def normalize_camera_calibration_status(
    value: int | CameraCalibrationStatus,
) -> CameraCalibrationStatus:
    """整数を正式なカメラ別校正結果へ変換する。"""

    try:
        return CameraCalibrationStatus(value)
    except ValueError as error:
        raise ValueError(f"invalid camera calibration status: {value}") from error


def is_fatal_calib2d3d_error(
    value: int | Calib2d3dErrorCommon,
) -> bool:
    """UIがエラー完了へ遷移する16から31のコードかを返す。"""

    status = normalize_calib2d3d_error_common(value)
    return _FATAL_COMMON_ERROR_MIN <= status <= _FATAL_COMMON_ERROR_MAX


class Calib2d3dDiagnosisPhase(Enum):
    """診断所見を検出した本校正の処理段階。"""

    FRAME_INPUT = auto()
    PERSON_DETECTION = auto()
    TRACKING_3D = auto()
    TRACKING_2D = auto()
    PROGRESS = auto()
    FINAL_CALCULATION = auto()
    MATRIX_VALIDATION = auto()


@dataclass(frozen=True, slots=True)
class Calib2d3dFinding:
    """後続のログ出力に使用できる一つの診断所見。"""

    camera_id: int
    phase: Calib2d3dDiagnosisPhase
    message: str
    common_error: Calib2d3dErrorCommon | None = None
    camera_status: CameraCalibrationStatus | None = None
    details: Mapping[str, object] = field(default_factory=dict)


@dataclass(frozen=True, slots=True)
class Calib2d3dDiagnosisResult:
    """1カメラの校正セッションからUIへ公開する診断結果。"""

    camera_id: int
    common_error: Calib2d3dErrorCommon
    camera_status: CameraCalibrationStatus | None
    findings: tuple[Calib2d3dFinding, ...] = ()


class Calib2d3dResultDiagnosis:
    """収集中に公開する共通診断コードを保持する。

    16から31はUIが読み取るまで消えないよう、セッション終了までラッチする。
    0から15は呼び出し側が渡した現在の診断結果へ更新できる。
    """

    def __init__(self) -> None:
        self.reset()

    def reset(self) -> None:
        self._current = Calib2d3dErrorCommon.DEFAULT
        self._fatal: Calib2d3dErrorCommon | None = None

    def diagnose(
        self,
        status: int | Calib2d3dErrorCommon = Calib2d3dErrorCommon.DEFAULT,
    ) -> Calib2d3dErrorCommon:
        normalized = normalize_calib2d3d_error_common(status)
        if self._fatal is not None:
            return self._fatal
        self._current = normalized
        if is_fatal_calib2d3d_error(normalized):
            self._fatal = normalized
        return self._current

    @property
    def current(self) -> Calib2d3dErrorCommon:
        return self._fatal if self._fatal is not None else self._current

    @property
    def fatal_error(self) -> Calib2d3dErrorCommon | None:
        return self._fatal


class CameraCalibrationStatusDiagnosis:
    """カメラ単位の最終校正結果を保持する。"""

    def __init__(self) -> None:
        self.reset()

    def reset(self) -> None:
        self._current = CameraCalibrationStatus.DEFAULT

    def diagnose(
        self,
        status: int | CameraCalibrationStatus = CameraCalibrationStatus.DEFAULT,
        *,
        common_error: int | Calib2d3dErrorCommon = Calib2d3dErrorCommon.DEFAULT,
    ) -> CameraCalibrationStatus:
        if is_fatal_calib2d3d_error(common_error):
            self._current = CameraCalibrationStatus.CALIBRATION_ABORTED
            return self._current
        self._current = normalize_camera_calibration_status(status)
        return self._current

    @property
    def current(self) -> CameraCalibrationStatus:
        return self._current


class Calib2d3dDiagnosisSession:
    """カメラ1台・本校正1回に対応する診断セッション。"""

    def __init__(self, camera_id: int) -> None:
        self._common_diagnosis = Calib2d3dResultDiagnosis()
        self._camera_diagnosis = CameraCalibrationStatusDiagnosis()
        self.reset(camera_id)

    def reset(self, camera_id: int) -> None:
        if camera_id < 0:
            raise ValueError("camera_id must not be negative")
        self.camera_id = camera_id
        self._common_diagnosis.reset()
        self._camera_diagnosis.reset()
        self.findings: list[Calib2d3dFinding] = []

    def _record_finding(
        self,
        *,
        phase: Calib2d3dDiagnosisPhase,
        common_error: Calib2d3dErrorCommon | None = None,
        camera_status: CameraCalibrationStatus | None = None,
        message: str | None = None,
        details: Mapping[str, object] | None = None,
    ) -> None:
        if common_error in (None, Calib2d3dErrorCommon.DEFAULT) and camera_status in (
            None,
            CameraCalibrationStatus.DEFAULT,
        ):
            return
        if any(
            finding.phase is phase
            and finding.common_error is common_error
            and finding.camera_status is camera_status
            for finding in self.findings
        ):
            return
        default_message = ""
        if common_error is not None:
            default_message = _COMMON_ERROR_LABELS.get(
                common_error, f"共通診断コード {int(common_error)}"
            )
        elif camera_status is not None:
            default_message = _CAMERA_STATUS_LABELS.get(
                camera_status, f"カメラ校正状態 {int(camera_status)}"
            )
        self.findings.append(
            Calib2d3dFinding(
                camera_id=self.camera_id,
                phase=phase,
                message=default_message if message is None else message,
                common_error=common_error,
                camera_status=camera_status,
                details={} if details is None else dict(details),
            )
        )

    def diagnose_runtime(
        self,
        status: int | Calib2d3dErrorCommon = Calib2d3dErrorCommon.DEFAULT,
        *,
        phase: Calib2d3dDiagnosisPhase = Calib2d3dDiagnosisPhase.FRAME_INPUT,
        message: str | None = None,
        details: Mapping[str, object] | None = None,
    ) -> Calib2d3dDiagnosisResult:
        """収集中の診断を記録し、MMAPへ公開する値を返す。"""

        common_error = self._common_diagnosis.diagnose(status)
        self._record_finding(
            phase=phase,
            common_error=normalize_calib2d3d_error_common(status),
            message=message,
            details=details,
        )
        camera_status = None
        if is_fatal_calib2d3d_error(common_error):
            camera_status = self._camera_diagnosis.diagnose(common_error=common_error)
        return Calib2d3dDiagnosisResult(
            camera_id=self.camera_id,
            common_error=common_error,
            camera_status=camera_status,
            findings=tuple(self.findings),
        )

    def diagnose_final(
        self,
        status: int | CameraCalibrationStatus,
        *,
        phase: Calib2d3dDiagnosisPhase = (Calib2d3dDiagnosisPhase.FINAL_CALCULATION),
        message: str | None = None,
        details: Mapping[str, object] | None = None,
    ) -> Calib2d3dDiagnosisResult:
        """最終計算結果を診断し、重大エラーがあれば中断を優先する。"""

        common_error = self._common_diagnosis.current
        camera_status = self._camera_diagnosis.diagnose(
            status,
            common_error=common_error,
        )
        self._record_finding(
            phase=phase,
            camera_status=camera_status,
            message=message,
            details=details,
        )
        return Calib2d3dDiagnosisResult(
            camera_id=self.camera_id,
            common_error=common_error,
            camera_status=camera_status,
            findings=tuple(self.findings),
        )

    @property
    def current_result(self) -> Calib2d3dDiagnosisResult:
        return Calib2d3dDiagnosisResult(
            camera_id=self.camera_id,
            common_error=self._common_diagnosis.current,
            camera_status=self._camera_diagnosis.current,
            findings=tuple(self.findings),
        )
