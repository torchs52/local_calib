"""2D-3D校正要否チェック固有の診断定義と診断セッション"""

from __future__ import annotations

import math
from collections.abc import Callable, Mapping, Sequence
from dataclasses import dataclass, field
from enum import Enum, IntEnum, auto
from time import monotonic
from typing import ClassVar


class CalibCheckFailureReason(IntEnum):
    """校正要否を判定できない理由値はUI・結果ログとの既存契約を維持する"""

    NONE = 0
    SENSOR_DATA_INVALID = 1
    BBOX3D_LOG_INVALID = 2
    BBOX2D_LOG_INVALID = 3
    TRACKING3D_INVALID = 4
    TRACKING2D_INVALID = 5
    EVALUATION_RESULT_INVALID = 6
    EVALUATION_STATISTICS_INVALID = 7
    CALIBRATION_JUDGEMENT_INVALID = 8
    CAMERA_PERSON_NOT_DETECTED = 9
    SENSOR_TIME_SYNC_INVALID = 10
    MATCHED_PERSON_NOT_DETECTED = 11


class CameraCalibCheckStatus(IntEnum):
    """set_camera_calibcheck_statusに設定する値"""

    CALIBRATION_NOT_REQUIRED = 0
    FORBIDDEN = 1
    CALIBRATION_REQUIRED = 2
    UNKNOWN_INSUFFICIENT_DATA = 3
    PERSON_NOT_DETECTED = 4
    POOR_PERSON_DETECTION = 5
    POOR_SENSOR_DATA = 6
    POOR_EVALUATION_RESULT = 7


# 校正要否チェック失敗理由
CALIBCHECK_REASON_LABELS: Mapping[CalibCheckFailureReason, str] = {
    CalibCheckFailureReason.NONE: "OK",
    CalibCheckFailureReason.SENSOR_DATA_INVALID: "情報取得時エラー",
    CalibCheckFailureReason.BBOX3D_LOG_INVALID: "3D bboxログが不正",
    CalibCheckFailureReason.BBOX2D_LOG_INVALID: "2D bboxログが不正",
    CalibCheckFailureReason.TRACKING3D_INVALID: "3D bboxトラッキング結果が不正/不足",
    CalibCheckFailureReason.TRACKING2D_INVALID: "2D bboxトラッキング結果が不正/不足",
    CalibCheckFailureReason.EVALUATION_RESULT_INVALID: "2D3D評価結果が不正",
    CalibCheckFailureReason.EVALUATION_STATISTICS_INVALID: "評価統計計算が不正",
    CalibCheckFailureReason.CALIBRATION_JUDGEMENT_INVALID: "校正判定が不正",
    CalibCheckFailureReason.CAMERA_PERSON_NOT_DETECTED: "カメラ内に対象物なし",
    CalibCheckFailureReason.SENSOR_TIME_SYNC_INVALID: "カメラとLiDARの時間同期なし",
    CalibCheckFailureReason.MATCHED_PERSON_NOT_DETECTED: (
        "カメラとLiDARの評価可能な対象物なし"
    ),
}

# 校正要否チェック失敗理由からステータスへのマッピング
CALIBCHECK_REASON_STATUS_MAP: Mapping[
    CalibCheckFailureReason, CameraCalibCheckStatus
] = {
    CalibCheckFailureReason.SENSOR_DATA_INVALID: (
        CameraCalibCheckStatus.POOR_SENSOR_DATA
    ),
    CalibCheckFailureReason.BBOX3D_LOG_INVALID: (
        CameraCalibCheckStatus.UNKNOWN_INSUFFICIENT_DATA
    ),
    CalibCheckFailureReason.BBOX2D_LOG_INVALID: (
        CameraCalibCheckStatus.UNKNOWN_INSUFFICIENT_DATA
    ),
    CalibCheckFailureReason.TRACKING3D_INVALID: (
        CameraCalibCheckStatus.POOR_PERSON_DETECTION
    ),
    CalibCheckFailureReason.TRACKING2D_INVALID: (
        CameraCalibCheckStatus.POOR_PERSON_DETECTION
    ),
    CalibCheckFailureReason.EVALUATION_RESULT_INVALID: (
        CameraCalibCheckStatus.POOR_EVALUATION_RESULT
    ),
    CalibCheckFailureReason.EVALUATION_STATISTICS_INVALID: (
        CameraCalibCheckStatus.POOR_EVALUATION_RESULT
    ),
    CalibCheckFailureReason.CALIBRATION_JUDGEMENT_INVALID: (
        CameraCalibCheckStatus.POOR_EVALUATION_RESULT
    ),
    CalibCheckFailureReason.CAMERA_PERSON_NOT_DETECTED: (
        CameraCalibCheckStatus.PERSON_NOT_DETECTED
    ),
    CalibCheckFailureReason.SENSOR_TIME_SYNC_INVALID: (
        CameraCalibCheckStatus.UNKNOWN_INSUFFICIENT_DATA
    ),
    CalibCheckFailureReason.MATCHED_PERSON_NOT_DETECTED: (
        CameraCalibCheckStatus.PERSON_NOT_DETECTED
    ),
}

# MMAPに書き込み可能なステータス
_WRITABLE_STATUSES = frozenset(
    {
        CameraCalibCheckStatus.CALIBRATION_NOT_REQUIRED,
        CameraCalibCheckStatus.CALIBRATION_REQUIRED,
        CameraCalibCheckStatus.UNKNOWN_INSUFFICIENT_DATA,
        CameraCalibCheckStatus.PERSON_NOT_DETECTED,
        CameraCalibCheckStatus.POOR_PERSON_DETECTION,
        CameraCalibCheckStatus.POOR_SENSOR_DATA,
        CameraCalibCheckStatus.POOR_EVALUATION_RESULT,
    }
)


def validate_camera_calibcheck_status(
    value: int | CameraCalibCheckStatus,
) -> CameraCalibCheckStatus:
    """MMAPへ書き込み可能なcalibcheckステータスへ変換する"""
    try:
        status = CameraCalibCheckStatus(value)
    except ValueError as error:
        raise ValueError(f"invalid camera calibcheck status: {value}") from error
    if status not in _WRITABLE_STATUSES:
        raise ValueError(f"camera calibcheck status {int(status)} is not writable")
    return status


def normalize_calibcheck_reason(
    value: int | CalibCheckFailureReason,
) -> CalibCheckFailureReason:
    """整数を正式なreasonへ変換し、未定義値を明示的に拒否する"""
    try:
        return CalibCheckFailureReason(value)
    except ValueError as error:
        raise ValueError(f"invalid calibcheck failure reason: {value}") from error


def calibcheck_reason_to_string(reason: int | CalibCheckFailureReason) -> str:
    """reasonを表示・ログ用文言へ変換する"""
    try:
        normalized = normalize_calibcheck_reason(reason)
    except ValueError:
        return f"Unknown reason code {reason}"
    return CALIBCHECK_REASON_LABELS[normalized]


def calibcheck_reason_to_status(
    reason: int | CalibCheckFailureReason,
    *,
    calibration_is_acceptable: bool | None,
) -> CameraCalibCheckStatus:
    """詳細reasonと校正評価結果をUI向けステータスへ集約する"""
    try:
        normalized = normalize_calibcheck_reason(reason)
    except ValueError:
        return CameraCalibCheckStatus.UNKNOWN_INSUFFICIENT_DATA
    if normalized is not CalibCheckFailureReason.NONE:
        return CALIBCHECK_REASON_STATUS_MAP[normalized]
    if calibration_is_acceptable is None:
        return CameraCalibCheckStatus.UNKNOWN_INSUFFICIENT_DATA
    if calibration_is_acceptable:
        return CameraCalibCheckStatus.CALIBRATION_NOT_REQUIRED
    return CameraCalibCheckStatus.CALIBRATION_REQUIRED


class CalibCheckDiagnosisPhase(Enum):
    """診断理由を検出した処理段階"""

    FRAME_INPUT = auto()
    BBOX_LOG = auto()
    TRACKING = auto()
    EVALUATION = auto()
    STATISTICS = auto()
    JUDGEMENT = auto()


@dataclass(frozen=True, slots=True)
class CalibCheckFinding:
    """診断セッション中に検出した一つの不良理由"""

    reason: CalibCheckFailureReason
    camera_index: int | None
    phase: CalibCheckDiagnosisPhase
    message: str
    details: Mapping[str, object] = field(default_factory=dict)


@dataclass(frozen=True, slots=True)
class CameraCalibCheckDiagnosisResult:
    """カメラ別の最終診断結果"""

    camera_index: int
    status: CameraCalibCheckStatus
    primary_reason: CalibCheckFailureReason
    all_reasons: tuple[CalibCheckFailureReason, ...]
    calibration_is_acceptable: bool | None


@dataclass(frozen=True, slots=True)
class CalibCheckFrameObservation:
    """1フレームの入力・検出・同期状態"""

    system_input_error_detected: bool = False
    camera_detection_counts: tuple[int, ...] = ()
    synchronization_valid: tuple[bool, ...] = ()


@dataclass(frozen=True, slots=True)
class BBoxLogObservation:
    """reason 2/3の判定に使う2D/3D bboxログ統計と閾値"""

    # 収集期間中にbboxログへ記録した総フレーム数
    total_frame_count: int
    # 1フレーム当たりの必要数を満たす3D bboxが記録されたフレーム数
    valid_3d_frame_count: int
    # 1フレーム当たりの必要数を満たす2D人物bboxが記録されたカメラ別フレーム数
    valid_2d_frame_counts: tuple[int, ...]
    # 3D bboxを含む有効フレームとみなすために必要な1フレーム当たりの最小bbox数
    minimum_3d_bbox_count_per_frame: int
    # 3D bboxログ全体を有効とみなすために必要な有効フレーム比率
    minimum_3d_valid_frame_ratio: float
    # 2D bboxを含む有効フレームとみなすために必要な1フレーム当たりの最小人物bbox数
    minimum_2d_bbox_count_per_frame: int
    # カメラ別2D bboxログを有効とみなすために必要な有効フレーム比率
    minimum_2d_valid_frame_ratio: float

    @property
    def valid_3d_frame_ratio(self) -> float:
        """全記録フレームに対する3D bbox有効フレームの割合"""
        if self.total_frame_count == 0:
            return 0.0
        return self.valid_3d_frame_count / self.total_frame_count

    @property
    def valid_2d_frame_ratios(self) -> tuple[float, ...]:
        """全記録フレームに対するカメラ別2D bbox有効フレームの割合"""
        if self.total_frame_count == 0:
            return tuple(0.0 for _ in self.valid_2d_frame_counts)
        return tuple(
            count / self.total_frame_count for count in self.valid_2d_frame_counts
        )


@dataclass(frozen=True, slots=True)
class TrackingObservation:
    """reason 4/5の判定に使う選別後の追跡件数と閾値"""

    # 3D trackの選別前の総数
    total_3d_track_count: int
    # 追跡期間、移動量、作業領域などの条件を満たした3D track数
    alive_3d_track_count: int
    # カメラごとの2D track選別前総数
    total_2d_track_counts: tuple[int, ...]
    # 追跡期間や画像上の移動量などの条件を満たしたカメラ別2D track数
    alive_2d_track_counts: tuple[int, ...]
    # 3D追跡結果を有効とするために必要な最小alive track数
    minimum_3d_alive_track_count: int
    # 2D追跡結果を有効とするために必要なカメラごとの最小alive track数
    minimum_2d_alive_track_count: int
    # 同時刻・近接位置に存在し、追跡の混同が疑われる3D trackペア数
    proximity_warning_count: int
    # 近接警告をreason 4の成立条件として扱うかどうか
    proximity_warning_fails_validation: bool


@dataclass(frozen=True, slots=True)
class EvaluationTimeSyncStatistics:
    """reason 10の判定証跡となる、追跡フレーム番号の対応状況"""

    # 対象カメラの画像内へ3D bboxを投影できた、重複しないフレーム数
    visible_3d_frame_count: int
    # 対象カメラでaliveな2D人物trackが存在した、重複しないフレーム数
    tracked_2d_frame_count: int
    # 投影可能な3D trackと2D人物trackが同じフレーム番号に存在した数
    common_frame_count: int
    # 投影可能な3Dフレーム番号の最小値と最大値存在しない場合はNone
    visible_3d_frame_range: tuple[int, int] | None
    # aliveな2D人物trackのフレーム番号の最小値と最大値存在しない場合はNone
    tracked_2d_frame_range: tuple[int, int] | None


@dataclass(frozen=True, slots=True)
class EvaluationMatchingStatistics:
    """reason 11の判定証跡となる、投影・対応評価の段階別件数"""

    # aliveな3D trackが持つ、重複しない追跡フレーム数
    tracked_3d_frame_count: int
    # 対象カメラの画像内へ3D bboxを投影できたフレーム数
    visible_3d_frame_count: int
    # 対象カメラでaliveな2D人物trackが存在したフレーム数
    tracked_2d_frame_count: int
    # 投影可能な3D trackと2D人物trackが同じフレーム番号に存在した数
    common_frame_count: int
    # 共通フレームのうち、間引き後に2D-3D比較候補となった件数
    comparison_candidate_count: int
    # 2D bboxと投影した3D bboxが画像上で重なった比較候補数
    overlap_candidate_count: int
    # Scene評価がNoneではないscoreを返した件数
    score_returned_count: int
    # 非重複による有効な0点も含め、統計へ入力できた比較件数
    valid_comparison_count: int


@dataclass(frozen=True, slots=True)
class EvaluationObservation:
    """reason 6の判定に使う、投影・対応評価の最終出力"""

    scores: tuple[float, ...]
    reasons: tuple[CalibCheckFailureReason, ...]
    time_sync_statistics: tuple[EvaluationTimeSyncStatistics, ...] = ()
    matching_statistics: tuple[EvaluationMatchingStatistics, ...] = ()


@dataclass(frozen=True, slots=True)
class EvaluationStatistics:
    """reason 7の判定に使う、1カメラ分の評価統計の内訳"""

    # strict hit-rateで一致と判定された評価フレーム数
    numerator: int
    # strict hit-rateの母数となる、画像内へ投影可能な評価フレーム数
    denominator: int
    # numerator / denominatorで求めた厳密な一致率
    strict_hit_rate: float
    # legacy式で各比較フレームから集計したscoreの合計
    legacy_numerator: float
    # legacy式で有効なscoreを得られた比較フレーム数
    legacy_denominator: int
    # legacy_numerator / legacy_denominatorで求めたscore
    legacy_like_score: float
    # 設定に従ってstrict式またはlegacy式から選択した最終score
    selected_score: float
    # Trueならlegacy_like_score、Falseならstrict_hit_rateを採用する
    use_legacy_like_metric: bool

    @property
    def selected_sample_count(self) -> int:
        """実際に選択した評価式の標本数を返す"""
        if self.use_legacy_like_metric:
            return self.legacy_denominator
        return self.denominator


@dataclass(frozen=True, slots=True)
class StatisticsObservation:
    """カメラ別評価統計と、成立性を判断するための条件"""

    statistics: tuple[EvaluationStatistics, ...]
    evaluation_reasons: tuple[CalibCheckFailureReason, ...]
    minimum_sample_count: int


@dataclass(frozen=True, slots=True)
class JudgementObservation:
    """reason 8の判定に使う、最終的な校正要否演算の入出力"""

    results: tuple[bool | None, ...]
    scores: tuple[float, ...]
    evaluation_reasons: tuple[CalibCheckFailureReason, ...]
    threshold: float


@dataclass(slots=True)
class CalibCheckFrameDiagnosisState:
    """データ収集中にフレームをまたいで保持する状態"""

    total_frames: int
    invalid_input_frames: int
    camera_observed_frames: list[int]
    camera_person_detected_frames: list[int]
    camera_total_person_detections: list[int]
    person_observation_started_at: list[float | None]
    person_observation_updated_at: list[float | None]
    consecutive_person_missing: list[int]
    max_consecutive_person_missing: list[int]
    sync_observed_frames: list[int]
    sync_invalid_frames: list[int]
    consecutive_sync_invalid: list[int]


class CalibCheck2d3dDiagnosis:
    """1回のcalibcheck2d3d実行に対応する診断セッション"""

    # 複数のfindingが成立した場合も、呼び出し順に依存せず代表理由を決める
    # camera_index=Noneのfindingは全カメラへ適用する
    _REASON_PRIORITY: ClassVar[tuple[CalibCheckFailureReason, ...]] = (
        CalibCheckFailureReason.SENSOR_DATA_INVALID,
        CalibCheckFailureReason.SENSOR_TIME_SYNC_INVALID,
        CalibCheckFailureReason.CAMERA_PERSON_NOT_DETECTED,
        CalibCheckFailureReason.BBOX3D_LOG_INVALID,
        CalibCheckFailureReason.BBOX2D_LOG_INVALID,
        CalibCheckFailureReason.TRACKING3D_INVALID,
        CalibCheckFailureReason.TRACKING2D_INVALID,
        CalibCheckFailureReason.MATCHED_PERSON_NOT_DETECTED,
        CalibCheckFailureReason.EVALUATION_RESULT_INVALID,
        CalibCheckFailureReason.EVALUATION_STATISTICS_INVALID,
        CalibCheckFailureReason.CALIBRATION_JUDGEMENT_INVALID,
    )

    def __init__(
        self,
        camera_count: int,
        *,
        person_not_detected_sec: float | None = None,
        sync_invalid_frame_threshold: int | None = None,
        elapsed_time: Callable[[], float] = monotonic,
    ) -> None:
        if person_not_detected_sec is not None and person_not_detected_sec <= 0:
            raise ValueError("person_not_detected_sec must be positive")
        self.person_not_detected_sec = person_not_detected_sec
        self.sync_invalid_frame_threshold = sync_invalid_frame_threshold
        # 壁時計やセンサー絶対時刻の変更に影響されない経過時間だけを使用する
        self._elapsed_time = elapsed_time
        self.reset(camera_count)

    def reset(self, camera_count: int) -> None:
        if camera_count <= 0:
            raise ValueError("camera_count must be positive")
        self.camera_count = camera_count
        self.findings: list[CalibCheckFinding] = []
        self.frame_state = CalibCheckFrameDiagnosisState(
            total_frames=0,
            invalid_input_frames=0,
            camera_observed_frames=[0 for _ in range(camera_count)],
            camera_person_detected_frames=[0 for _ in range(camera_count)],
            camera_total_person_detections=[0 for _ in range(camera_count)],
            person_observation_started_at=[None for _ in range(camera_count)],
            person_observation_updated_at=[None for _ in range(camera_count)],
            consecutive_person_missing=[0 for _ in range(camera_count)],
            max_consecutive_person_missing=[0 for _ in range(camera_count)],
            sync_observed_frames=[0 for _ in range(camera_count)],
            sync_invalid_frames=[0 for _ in range(camera_count)],
            consecutive_sync_invalid=[0 for _ in range(camera_count)],
        )

    def _record(
        self,
        reason: CalibCheckFailureReason,
        *,
        camera_index: int | None,
        phase: CalibCheckDiagnosisPhase,
        details: Mapping[str, object] | None = None,
    ) -> None:
        if reason is CalibCheckFailureReason.NONE:
            return
        if camera_index is not None and not 0 <= camera_index < self.camera_count:
            raise IndexError(f"camera index out of range: {camera_index}")
        # 同一カメラ・同一理由は一度だけ保持し、長時間の収集でも増え続けないようにする
        if any(
            finding.reason is reason and finding.camera_index == camera_index
            for finding in self.findings
        ):
            return
        self.findings.append(
            CalibCheckFinding(
                reason=reason,
                camera_index=camera_index,
                phase=phase,
                message=CALIBCHECK_REASON_LABELS[reason],
                details={} if details is None else details,
            )
        )

    def record_reason(
        self,
        reason: int | CalibCheckFailureReason,
        *,
        camera_index: int | None,
        phase: CalibCheckDiagnosisPhase,
        details: Mapping[str, object] | None = None,
    ) -> None:
        """既存処理から段階的に移行するための型付きreason記録入口"""
        self._record(
            normalize_calibcheck_reason(reason),
            camera_index=camera_index,
            phase=phase,
            details=details,
        )

    def diagnose_frame(self, observation: CalibCheckFrameObservation) -> None:
        """reason 1/9/10をフレーム単位で診断・蓄積する"""
        if (
            observation.camera_detection_counts
            and len(observation.camera_detection_counts) != self.camera_count
        ):
            raise ValueError("camera_detection_counts length must match camera_count")
        if (
            observation.synchronization_valid
            and len(observation.synchronization_valid) != self.camera_count
        ):
            raise ValueError("synchronization_valid length must match camera_count")

        self.frame_state.total_frames += 1
        if observation.system_input_error_detected:
            self.frame_state.invalid_input_frames += 1
            self._record(
                CalibCheckFailureReason.SENSOR_DATA_INVALID,
                camera_index=None,
                phase=CalibCheckDiagnosisPhase.FRAME_INPUT,
            )

        detection_observed_at = (
            self._elapsed_time() if observation.camera_detection_counts else None
        )
        for camera_index, detection_count in enumerate(
            observation.camera_detection_counts
        ):
            if detection_count < 0:
                raise ValueError("camera detection count must not be negative")
            self.frame_state.camera_observed_frames[camera_index] += 1
            self.frame_state.camera_total_person_detections[camera_index] += (
                detection_count
            )
            if self.frame_state.person_observation_started_at[camera_index] is None:
                self.frame_state.person_observation_started_at[camera_index] = (
                    detection_observed_at
                )
            self.frame_state.person_observation_updated_at[camera_index] = (
                detection_observed_at
            )
            if detection_count > 0:
                self.frame_state.camera_person_detected_frames[camera_index] += 1
                self.frame_state.consecutive_person_missing[camera_index] = 0
            else:
                self.frame_state.consecutive_person_missing[camera_index] += 1
                self.frame_state.max_consecutive_person_missing[camera_index] = max(
                    self.frame_state.max_consecutive_person_missing[camera_index],
                    self.frame_state.consecutive_person_missing[camera_index],
                )

        for camera_index, is_valid in enumerate(observation.synchronization_valid):
            self.frame_state.sync_observed_frames[camera_index] += 1
            if is_valid:
                self.frame_state.consecutive_sync_invalid[camera_index] = 0
            else:
                self.frame_state.sync_invalid_frames[camera_index] += 1
                self.frame_state.consecutive_sync_invalid[camera_index] += 1
            threshold = self.sync_invalid_frame_threshold
            if (
                threshold is not None
                and threshold > 0
                and (
                    self.frame_state.consecutive_sync_invalid[camera_index] >= threshold
                )
            ):
                self._record(
                    CalibCheckFailureReason.SENSOR_TIME_SYNC_INVALID,
                    camera_index=camera_index,
                    phase=CalibCheckDiagnosisPhase.FRAME_INPUT,
                    details={
                        "consecutive_invalid_frames": (
                            self.frame_state.consecutive_sync_invalid[camera_index]
                        ),
                        "invalid_frame_threshold": threshold,
                        "observed_frames": self.frame_state.sync_observed_frames[
                            camera_index
                        ],
                        "invalid_frames": self.frame_state.sync_invalid_frames[
                            camera_index
                        ],
                    },
                )

    def diagnose_person_detection(self) -> None:
        """十分な経過時間で人物を一度も検出していないカメラをreason 9とする"""
        threshold = self.person_not_detected_sec
        if threshold is None:
            return
        for camera_index in range(self.camera_count):
            started_at = self.frame_state.person_observation_started_at[camera_index]
            updated_at = self.frame_state.person_observation_updated_at[camera_index]
            if started_at is None or updated_at is None:
                continue
            observation_seconds = max(0.0, updated_at - started_at)
            if (
                observation_seconds >= threshold
                and self.frame_state.camera_person_detected_frames[camera_index] == 0
            ):
                self._record(
                    CalibCheckFailureReason.CAMERA_PERSON_NOT_DETECTED,
                    camera_index=camera_index,
                    phase=CalibCheckDiagnosisPhase.FRAME_INPUT,
                    details={
                        "observation_seconds": observation_seconds,
                        "observed_frames": self.frame_state.camera_observed_frames[
                            camera_index
                        ],
                        "total_person_detections": 0,
                        "max_consecutive_missing_frames": (
                            self.frame_state.max_consecutive_person_missing[
                                camera_index
                            ]
                        ),
                    },
                )

    def diagnose_bbox_logs(self, observation: BBoxLogObservation) -> tuple[bool, ...]:
        """bboxログ統計からreason 2/3を判定し、カメラ別の評価可否を返す"""
        if len(observation.valid_2d_frame_counts) != self.camera_count:
            raise ValueError("valid_2d_frame_counts length must match camera_count")
        if observation.total_frame_count < 0:
            raise ValueError("total_frame_count must not be negative")
        # 検討:この辺の設定範囲はもう少し整理する.
        if observation.minimum_3d_bbox_count_per_frame <= 0:
            raise ValueError("minimum_3d_bbox_count_per_frame must be positive")
        if observation.minimum_2d_bbox_count_per_frame <= 0:
            raise ValueError("minimum_2d_bbox_count_per_frame must be positive")
        if not 0.0 <= observation.minimum_3d_valid_frame_ratio <= 1.0:
            raise ValueError("minimum_3d_valid_frame_ratio must be between 0 and 1")
        if not 0.0 <= observation.minimum_2d_valid_frame_ratio <= 1.0:
            raise ValueError("minimum_2d_valid_frame_ratio must be between 0 and 1")
        valid_frame_counts = (
            observation.valid_3d_frame_count,
            *observation.valid_2d_frame_counts,
        )
        if any(
            count < 0 or count > observation.total_frame_count
            for count in valid_frame_counts
        ):
            raise ValueError(
                "valid bbox frame counts must be between 0 and total_frame_count"
            )

        total_frames = observation.total_frame_count
        valid_3d_ratio = observation.valid_3d_frame_ratio
        valid_3d = (
            total_frames > 0
            and valid_3d_ratio >= observation.minimum_3d_valid_frame_ratio
        )
        if not valid_3d:
            self._record(
                CalibCheckFailureReason.BBOX3D_LOG_INVALID,
                camera_index=None,
                phase=CalibCheckDiagnosisPhase.BBOX_LOG,
                details={
                    "total_frames": total_frames,
                    "valid_frames": observation.valid_3d_frame_count,
                    "valid_frame_ratio": valid_3d_ratio,
                    "minimum_bbox_count_per_frame": (
                        observation.minimum_3d_bbox_count_per_frame
                    ),
                    "minimum_valid_frame_ratio": (
                        observation.minimum_3d_valid_frame_ratio
                    ),
                },
            )

        valid_2d_by_camera: list[bool] = []
        for camera_index, (valid_frame_count, valid_frame_ratio) in enumerate(
            zip(
                observation.valid_2d_frame_counts,
                observation.valid_2d_frame_ratios,
                strict=True,
            )
        ):
            is_valid = (
                total_frames > 0
                and valid_frame_ratio >= observation.minimum_2d_valid_frame_ratio
            )
            valid_2d_by_camera.append(is_valid)
            if not is_valid:
                self._record(
                    CalibCheckFailureReason.BBOX2D_LOG_INVALID,
                    camera_index=camera_index,
                    phase=CalibCheckDiagnosisPhase.BBOX_LOG,
                    details={
                        "total_frames": total_frames,
                        "valid_frames": valid_frame_count,
                        "valid_frame_ratio": valid_frame_ratio,
                        "minimum_bbox_count_per_frame": (
                            observation.minimum_2d_bbox_count_per_frame
                        ),
                        "minimum_valid_frame_ratio": (
                            observation.minimum_2d_valid_frame_ratio
                        ),
                    },
                )
        if not valid_3d:
            return tuple(False for _ in range(self.camera_count))
        return tuple(valid_2d_by_camera)

    def diagnose_tracking(self, observation: TrackingObservation) -> tuple[bool, ...]:
        """選別後の追跡件数からreason 4/5を判定し、カメラ別評価可否を返す"""
        if len(observation.total_2d_track_counts) != self.camera_count:
            raise ValueError("total_2d_track_counts length must match camera_count")
        if len(observation.alive_2d_track_counts) != self.camera_count:
            raise ValueError("alive_2d_track_counts length must match camera_count")
        # 検討:この辺の設定範囲はもう少し整理する.
        if observation.minimum_3d_alive_track_count <= 0:
            raise ValueError("minimum_3d_alive_track_count must be positive")
        if observation.minimum_2d_alive_track_count <= 0:
            raise ValueError("minimum_2d_alive_track_count must be positive")
        if observation.proximity_warning_count < 0:
            raise ValueError("proximity_warning_count must not be negative")
        track_count_pairs = (
            (observation.total_3d_track_count, observation.alive_3d_track_count),
            *zip(
                observation.total_2d_track_counts,
                observation.alive_2d_track_counts,
                strict=True,
            ),
        )
        if any(
            total_count < 0 or alive_count < 0 or alive_count > total_count
            for total_count, alive_count in track_count_pairs
        ):
            raise ValueError(
                "alive track counts must be between 0 and total track counts"
            )

        proximity_invalid = (
            observation.proximity_warning_fails_validation
            and observation.proximity_warning_count > 0
        )
        valid_3d = (
            observation.alive_3d_track_count >= observation.minimum_3d_alive_track_count
            and not proximity_invalid
        )
        if not valid_3d:
            self._record(
                CalibCheckFailureReason.TRACKING3D_INVALID,
                camera_index=None,
                phase=CalibCheckDiagnosisPhase.TRACKING,
                details={
                    "total_tracks": observation.total_3d_track_count,
                    "alive_tracks": observation.alive_3d_track_count,
                    "minimum_alive_tracks": (observation.minimum_3d_alive_track_count),
                    "proximity_warning_count": observation.proximity_warning_count,
                    "proximity_warning_fails_validation": (
                        observation.proximity_warning_fails_validation
                    ),
                },
            )

        valid_2d_by_camera: list[bool] = []
        for camera_index, (total_count, alive_count) in enumerate(
            zip(
                observation.total_2d_track_counts,
                observation.alive_2d_track_counts,
                strict=True,
            )
        ):
            is_valid = alive_count >= observation.minimum_2d_alive_track_count
            valid_2d_by_camera.append(is_valid)
            if not is_valid:
                self._record(
                    CalibCheckFailureReason.TRACKING2D_INVALID,
                    camera_index=camera_index,
                    phase=CalibCheckDiagnosisPhase.TRACKING,
                    details={
                        "total_tracks": total_count,
                        "alive_tracks": alive_count,
                        "minimum_alive_tracks": (
                            observation.minimum_2d_alive_track_count
                        ),
                    },
                )
        if not valid_3d:
            return tuple(False for _ in range(self.camera_count))
        return tuple(valid_2d_by_camera)

    def diagnose_evaluation(self, observation: EvaluationObservation) -> None:
        """reason 6と、評価処理が明示したreason 10/11を記録する"""
        if (
            len(observation.scores) != self.camera_count
            or len(observation.reasons) != self.camera_count
        ):
            raise ValueError("evaluation observation length must match camera_count")
        if (
            observation.time_sync_statistics
            and len(observation.time_sync_statistics) != self.camera_count
        ):
            raise ValueError("time sync statistics length must match camera_count")
        if (
            observation.matching_statistics
            and len(observation.matching_statistics) != self.camera_count
        ):
            raise ValueError("matching statistics length must match camera_count")
        for camera_index, (score, reason) in enumerate(
            zip(observation.scores, observation.reasons, strict=True)
        ):
            score_is_valid = (
                not isinstance(score, bool)
                and isinstance(score, (int, float))
                and math.isfinite(score)
                and 0.0 <= score <= 1.0
            )
            # reason 10/11時に返す0点も範囲内なので、評価不能理由の有無にかかわらず
            # score自体の出力インターフェースを確認する
            if not score_is_valid:
                self._record(
                    CalibCheckFailureReason.EVALUATION_RESULT_INVALID,
                    camera_index=camera_index,
                    phase=CalibCheckDiagnosisPhase.EVALUATION,
                    details={
                        "score": score,
                        "expected_range": "0.0 <= score <= 1.0",
                    },
                )
            if reason is CalibCheckFailureReason.SENSOR_TIME_SYNC_INVALID:
                details: Mapping[str, object] | None = None
                if observation.time_sync_statistics:
                    sync_statistics = observation.time_sync_statistics[camera_index]
                    details = {
                        "visible_3d_frame_count": (
                            sync_statistics.visible_3d_frame_count
                        ),
                        "tracked_2d_frame_count": (
                            sync_statistics.tracked_2d_frame_count
                        ),
                        "common_frame_count": sync_statistics.common_frame_count,
                        "visible_3d_frame_range": (
                            sync_statistics.visible_3d_frame_range
                        ),
                        "tracked_2d_frame_range": (
                            sync_statistics.tracked_2d_frame_range
                        ),
                    }
                self._record(
                    reason,
                    camera_index=camera_index,
                    phase=CalibCheckDiagnosisPhase.EVALUATION,
                    details=details,
                )
            elif reason is CalibCheckFailureReason.MATCHED_PERSON_NOT_DETECTED:
                details = None
                if observation.matching_statistics:
                    matching = observation.matching_statistics[camera_index]
                    if matching.visible_3d_frame_count == 0:
                        failure_stage = "no_visible_3d_projection"
                    elif matching.tracked_2d_frame_count == 0:
                        failure_stage = "no_tracked_2d_person"
                    else:
                        failure_stage = "no_valid_2d3d_comparison"
                    details = {
                        "failure_stage": failure_stage,
                        "tracked_3d_frame_count": matching.tracked_3d_frame_count,
                        "visible_3d_frame_count": matching.visible_3d_frame_count,
                        "tracked_2d_frame_count": matching.tracked_2d_frame_count,
                        "common_frame_count": matching.common_frame_count,
                        "comparison_candidate_count": (
                            matching.comparison_candidate_count
                        ),
                        "overlap_candidate_count": (matching.overlap_candidate_count),
                        "score_returned_count": matching.score_returned_count,
                        "valid_comparison_count": matching.valid_comparison_count,
                    }
                self._record(
                    reason,
                    camera_index=camera_index,
                    phase=CalibCheckDiagnosisPhase.EVALUATION,
                    details=details,
                )
            elif reason is not CalibCheckFailureReason.NONE:
                self._record(
                    reason,
                    camera_index=camera_index,
                    phase=CalibCheckDiagnosisPhase.EVALUATION,
                )

    def diagnose_statistics(self, observation: StatisticsObservation) -> None:
        """reason 7として、標本不足または評価統計の内部不整合を記録する"""
        if (
            len(observation.statistics) != self.camera_count
            or len(observation.evaluation_reasons) != self.camera_count
        ):
            raise ValueError("statistics observation length must match camera_count")
        if observation.minimum_sample_count <= 0:
            raise ValueError("minimum_sample_count must be positive")

        for camera_index, (statistics, evaluation_reason) in enumerate(
            zip(
                observation.statistics,
                observation.evaluation_reasons,
                strict=True,
            )
        ):
            # reason 10/11は統計以前に評価対象が成立していないため、
            # その標本不足をreason 7として重複記録しない
            if evaluation_reason is not CalibCheckFailureReason.NONE:
                continue
            strict_counts_valid = (
                statistics.denominator >= 0
                and 0 <= statistics.numerator <= statistics.denominator
            )
            legacy_counts_valid = (
                statistics.legacy_denominator >= 0
                and 0.0
                <= statistics.legacy_numerator
                <= float(statistics.legacy_denominator)
            )
            values_are_finite = all(
                math.isfinite(value)
                for value in (
                    statistics.strict_hit_rate,
                    statistics.legacy_numerator,
                    statistics.legacy_like_score,
                    statistics.selected_score,
                )
            )
            expected_selected_score = (
                statistics.legacy_like_score
                if statistics.use_legacy_like_metric
                else statistics.strict_hit_rate
            )
            selected_score_is_consistent = values_are_finite and math.isclose(
                statistics.selected_score,
                expected_selected_score,
                rel_tol=1e-9,
                abs_tol=1e-12,
            )
            sample_count_is_sufficient = (
                statistics.selected_sample_count >= observation.minimum_sample_count
            )
            if not (
                strict_counts_valid
                and legacy_counts_valid
                and values_are_finite
                and selected_score_is_consistent
                and sample_count_is_sufficient
            ):
                self._record(
                    CalibCheckFailureReason.EVALUATION_STATISTICS_INVALID,
                    camera_index=camera_index,
                    phase=CalibCheckDiagnosisPhase.STATISTICS,
                    details={
                        "numerator": statistics.numerator,
                        "denominator": statistics.denominator,
                        "strict_hit_rate": statistics.strict_hit_rate,
                        "legacy_numerator": statistics.legacy_numerator,
                        "legacy_denominator": statistics.legacy_denominator,
                        "legacy_like_score": statistics.legacy_like_score,
                        "selected_score": statistics.selected_score,
                        "use_legacy_like_metric": statistics.use_legacy_like_metric,
                        "selected_sample_count": statistics.selected_sample_count,
                        "minimum_sample_count": observation.minimum_sample_count,
                        "strict_counts_valid": strict_counts_valid,
                        "legacy_counts_valid": legacy_counts_valid,
                        "values_are_finite": values_are_finite,
                        "selected_score_is_consistent": selected_score_is_consistent,
                        "sample_count_is_sufficient": sample_count_is_sufficient,
                    },
                )

    def diagnose_judgement(self, observation: JudgementObservation) -> None:
        """reason 8として、最終判定の型・閾値・演算結果の不整合を記録する"""
        if (
            len(observation.results) != self.camera_count
            or len(observation.scores) != self.camera_count
            or len(observation.evaluation_reasons) != self.camera_count
        ):
            raise ValueError("judgement results length must match camera_count")
        threshold_is_valid = (
            not isinstance(observation.threshold, bool)
            and isinstance(observation.threshold, (int, float))
            and math.isfinite(observation.threshold)
            and 0.0 <= observation.threshold <= 1.0
        )
        for camera_index, (result, score, evaluation_reason) in enumerate(
            zip(
                observation.results,
                observation.scores,
                observation.evaluation_reasons,
                strict=True,
            )
        ):
            if evaluation_reason is not CalibCheckFailureReason.NONE:
                continue
            score_is_valid = (
                not isinstance(score, bool)
                and isinstance(score, (int, float))
                and math.isfinite(score)
                and 0.0 <= score <= 1.0
            )
            # score不正はreason 6の責務なので、この段階では判定不整合を重ねない
            if not score_is_valid:
                continue
            result_is_bool = isinstance(result, bool)
            result_is_consistent = (
                threshold_is_valid
                and result_is_bool
                and result == (score >= observation.threshold)
            )
            if not result_is_consistent:
                self._record(
                    CalibCheckFailureReason.CALIBRATION_JUDGEMENT_INVALID,
                    camera_index=camera_index,
                    phase=CalibCheckDiagnosisPhase.JUDGEMENT,
                    details={
                        "result": result,
                        "score": score,
                        "threshold": observation.threshold,
                        "threshold_is_valid": threshold_is_valid,
                        "result_is_bool": result_is_bool,
                        "expected_result": (
                            score >= observation.threshold
                            if threshold_is_valid
                            else None
                        ),
                    },
                )

    def _reasons_for_camera(
        self, camera_index: int
    ) -> tuple[CalibCheckFailureReason, ...]:
        found = {
            finding.reason
            for finding in self.findings
            if finding.camera_index is None or finding.camera_index == camera_index
        }
        return tuple(reason for reason in self._REASON_PRIORITY if reason in found)

    def finalize(
        self, calibration_results: Sequence[bool | None]
    ) -> list[CameraCalibCheckDiagnosisResult]:
        if len(calibration_results) != self.camera_count:
            raise ValueError("calibration_results length must match camera_count")
        # reason 9は一時的な未検出では確定せず、セッション全体の観測実績で判断する
        self.diagnose_person_detection()
        results: list[CameraCalibCheckDiagnosisResult] = []
        for camera_index, calibration_is_acceptable in enumerate(calibration_results):
            # 共通findingとカメラ固有findingを合わせ、優先表から代表理由を選ぶ
            reasons = self._reasons_for_camera(camera_index)
            primary_reason = reasons[0] if reasons else CalibCheckFailureReason.NONE
            effective_result = (
                calibration_is_acceptable
                if primary_reason is CalibCheckFailureReason.NONE
                else None
            )
            results.append(
                CameraCalibCheckDiagnosisResult(
                    camera_index=camera_index,
                    status=calibcheck_reason_to_status(
                        primary_reason,
                        calibration_is_acceptable=effective_result,
                    ),
                    primary_reason=primary_reason,
                    all_reasons=reasons,
                    calibration_is_acceptable=effective_result,
                )
            )
        return results
