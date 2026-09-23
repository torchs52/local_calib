"""
2D-3D本校正の収集中に行う診断.

画像処理や追跡器そのものには手を加えず、呼び出し側から渡された1フレーム分の
観測値と時系列履歴だけを使って ``errors_calibcommon`` の候補を決定する.
"""

from __future__ import annotations

from collections import deque
from collections.abc import Callable
from dataclasses import dataclass
from time import monotonic

import numpy as np
from numpy.typing import NDArray

from argus_synchro.diagnosis.calib2d3d_result_diagnosis import (
    Calib2d3dDiagnosisPhase,
    Calib2d3dErrorCommon,
)

WALKING_AREA_CORNER_COUNT = 4
MINIMUM_POLYGON_POINT_COUNT = 3


@dataclass(frozen=True, slots=True)
class Calib2d3dRuntimeDiagnosisConfig:
    """収集中診断にだけ使用する設定値."""

    # UIコードごとに独立して診断を止められるよう、共通の一括フラグにはしない.
    enable_walking_range: bool = True
    enable_person_count: bool = True
    enable_poor_person_detection: bool = True
    enable_poor_tracking_3d: bool = True
    enable_poor_tracking_2d: bool = True
    enable_unsuitable_condition: bool = True
    enable_long_duration: bool = True
    enable_person_detection_impossible: bool = True
    enable_tracking_impossible: bool = True
    continuation_seconds: float = 20.0
    long_duration_seconds: float = 600.0
    detection_rate_window_seconds: float = 20.0
    detection_rate_threshold: float = 0.2
    expected_person_count: int = 1
    brightness_threshold: float = 20.0
    brightness_sample_stride: int = 16
    walking_area_enabled: bool = False
    walking_area_corners: tuple[tuple[float, float], ...] = ()

    def __post_init__(self) -> None:
        if self.continuation_seconds <= 0:
            raise ValueError("continuation_seconds must be positive")
        if self.long_duration_seconds <= 0:
            raise ValueError("long_duration_seconds must be positive")
        if self.detection_rate_window_seconds <= 0:
            raise ValueError("detection_rate_window_seconds must be positive")
        if not 0.0 <= self.detection_rate_threshold <= 1.0:
            raise ValueError("detection_rate_threshold must be between 0 and 1")
        if self.expected_person_count <= 0:
            raise ValueError("expected_person_count must be positive")
        if self.brightness_sample_stride <= 0:
            raise ValueError("brightness_sample_stride must be positive")
        if (
            self.walking_area_enabled
            and len(self.walking_area_corners) != WALKING_AREA_CORNER_COUNT
        ):
            raise ValueError("walking_area_corners must contain four points")


@dataclass(frozen=True, slots=True)
class Calib2d3dFrameObservation:
    """校正処理から診断へ渡す、加工済みの1フレーム観測値."""

    image: NDArray[np.uint8] | None
    detection_2d_count: int
    bbox_3d_count: int
    tracking_2d_id_count: int
    tracking_3d_id_count: int
    bbox_3d_centers_xy: tuple[tuple[float, float], ...] = ()


@dataclass(frozen=True, slots=True)
class Calib2d3dRuntimeDecision:
    """実行中診断が選んだUIコードと根拠."""

    common_error: Calib2d3dErrorCommon
    phase: Calib2d3dDiagnosisPhase
    details: dict[str, object]


def calculate_sampled_brightness(
    image: NDArray[np.uint8] | None,
    *,
    stride: int,
) -> float | None:
    """ヒストグラムを作らず、間引いた画素の平均輝度を返す."""

    if image is None or image.size == 0:
        return None
    sampled = image[::stride, ::stride]
    if sampled.size == 0:
        return None
    return float(np.mean(sampled, dtype=np.float64))


def point_is_strictly_inside_polygon(
    point: tuple[float, float],
    polygon: tuple[tuple[float, float], ...],
) -> bool:
    """境界を含めずに点が多角形内部にあるかを返す."""

    if len(polygon) < MINIMUM_POLYGON_POINT_COUNT:
        return False
    px, py = point
    inside = False
    tolerance = 1e-9
    for index, (x1, y1) in enumerate(polygon):
        x2, y2 = polygon[(index + 1) % len(polygon)]
        cross = (px - x1) * (y2 - y1) - (py - y1) * (x2 - x1)
        if (
            abs(cross) <= tolerance
            and min(x1, x2) - tolerance <= px <= max(x1, x2) + tolerance
            and min(y1, y2) - tolerance <= py <= max(y1, y2) + tolerance
        ):
            return False
        if (y1 > py) != (y2 > py):
            intersection_x = (x2 - x1) * (py - y1) / (y2 - y1) + x1
            if px < intersection_x:
                inside = not inside
    return inside


class Calib2d3dRuntimeDiagnosis:
    """フレーム観測を蓄積し、実行中診断コードを決める."""

    def __init__(
        self,
        config: Calib2d3dRuntimeDiagnosisConfig,
        *,
        clock: Callable[[], float] = monotonic,
    ) -> None:
        self.config = config
        self._clock = clock
        self.reset()

    def reset(self) -> None:
        now = self._clock()
        # app_loopmain開始時刻.既存の処理時間計測用starttimeとは共有しない.
        self._started_at = now
        self._condition_started_at: dict[str, float] = {}
        self._detection_samples: deque[tuple[float, bool]] = deque()
        self._tracking_2d_samples: deque[tuple[float, bool]] = deque()
        self._tracking_3d_samples: deque[tuple[float, bool]] = deque()

    def _continued(self, key: str, condition: bool, now: float) -> bool:
        # 正常へ戻った時点で開始時刻を捨てるため、次の異常は0秒から数え直す.
        if not condition:
            self._condition_started_at.pop(key, None)
            return False
        started_at = self._condition_started_at.setdefault(key, now)
        return now - started_at >= self.config.continuation_seconds

    def _append_rate_sample(
        self,
        samples: deque[tuple[float, bool]],
        *,
        now: float,
        value: bool,
    ) -> float | None:
        samples.append((now, value))
        cutoff = now - self.config.detection_rate_window_seconds
        while samples and samples[0][0] < cutoff:
            samples.popleft()
        # 開始直後の少数フレームだけで低検出率と判断しない.
        if now - self._started_at < self.config.detection_rate_window_seconds:
            return None
        return sum(sample_value for _, sample_value in samples) / max(len(samples), 1)

    def diagnose(
        self,
        observation: Calib2d3dFrameObservation,
        *,
        now: float | None = None,
    ) -> Calib2d3dRuntimeDecision:
        now = self._clock() if now is None else now
        detected_2d = observation.detection_2d_count > 0
        detected_3d = observation.bbox_3d_count > 0
        tracked_2d = observation.tracking_2d_id_count > 0
        tracked_3d = observation.tracking_3d_id_count > 0

        detection_rate = self._append_rate_sample(
            self._detection_samples, now=now, value=detected_2d
        )
        tracking_2d_rate = self._append_rate_sample(
            self._tracking_2d_samples,
            now=now,
            # 未検出は追跡器の責任にせず、人検知診断側で扱う.
            value=(not detected_2d) or tracked_2d,
        )
        tracking_3d_rate = self._append_rate_sample(
            self._tracking_3d_samples,
            now=now,
            # 3D bboxがないフレームは追跡不良に数えず、人検知診断側で扱う.
            value=(not detected_3d) or tracked_3d,
        )

        # 人数の主判定には追跡ID数を使い、現時点では2D追跡ID数を採用する.
        # 2D検出数、3D bbox数、3D追跡ID数も observation に保持し、仕様確定後に
        # 判定へ追加できるようにしている.
        # 今後、2D検出数や3D情報、座標情報を組み合わせてより精度の高い人数判定を行う可能性あり.
        person_count = observation.tracking_2d_id_count
        person_count_invalid = (
            person_count > 0 and person_count != self.config.expected_person_count
        )
        # 1つでも範囲外の3D bbox中心があれば、そのフレームは範囲外とする.
        walking_range_invalid = self.config.walking_area_enabled and any(
            not point_is_strictly_inside_polygon(
                center, self.config.walking_area_corners
            )
            for center in observation.bbox_3d_centers_xy
        )
        brightness = calculate_sampled_brightness(
            observation.image,
            stride=self.config.brightness_sample_stride,
        )
        too_dark = (
            brightness is not None and brightness < self.config.brightness_threshold
        )

        no_detection = self._continued(
            "no_detection",
            self.config.enable_person_detection_impossible and not detected_2d,
            now,
        )
        no_tracking_2d = self._continued(
            "no_tracking_2d",
            self.config.enable_tracking_impossible and detected_2d and not tracked_2d,
            now,
        )
        no_tracking_3d = self._continued(
            "no_tracking_3d",
            self.config.enable_tracking_impossible and detected_3d and not tracked_3d,
            now,
        )

        details: dict[str, object] = {
            "elapsed_seconds": now - self._started_at,
            "detection_2d_count": observation.detection_2d_count,
            "bbox_3d_count": observation.bbox_3d_count,
            "tracking_2d_id_count": observation.tracking_2d_id_count,
            "tracking_3d_id_count": observation.tracking_3d_id_count,
            "detection_rate": detection_rate,
            "tracking_2d_rate": tracking_2d_rate,
            "tracking_3d_rate": tracking_3d_rate,
            "brightness": brightness,
        }

        # 16-18はUIをエラー完了へ導くため、継続可能な警告より先に判定する.
        if (
            self.config.enable_long_duration
            and now - self._started_at >= self.config.long_duration_seconds
        ):
            return Calib2d3dRuntimeDecision(
                Calib2d3dErrorCommon.LONG_DURATION_CALIBRATION,
                Calib2d3dDiagnosisPhase.FRAME_INPUT,
                details,
            )
        if no_detection:
            return Calib2d3dRuntimeDecision(
                Calib2d3dErrorCommon.PERSON_DETECTION_IMPOSSIBLE,
                Calib2d3dDiagnosisPhase.PERSON_DETECTION,
                details,
            )
        if no_tracking_3d or no_tracking_2d:
            return Calib2d3dRuntimeDecision(
                Calib2d3dErrorCommon.TRACKING_IMPOSSIBLE,
                (
                    Calib2d3dDiagnosisPhase.TRACKING_3D
                    if no_tracking_3d
                    else Calib2d3dDiagnosisPhase.TRACKING_2D
                ),
                details,
            )
        # 2-7は同時に1コードしか送れない.仕様済みの5優先を含め、以下の順で選ぶ.
        if self._continued(
            "walking_range",
            self.config.enable_walking_range and walking_range_invalid,
            now,
        ):
            return Calib2d3dRuntimeDecision(
                Calib2d3dErrorCommon.WALKING_RANGE_INVALID,
                Calib2d3dDiagnosisPhase.PROGRESS,
                details,
            )
        if self._continued(
            "person_count",
            self.config.enable_person_count and person_count_invalid,
            now,
        ):
            return Calib2d3dRuntimeDecision(
                Calib2d3dErrorCommon.WALKING_PERSON_COUNT_INVALID,
                Calib2d3dDiagnosisPhase.PERSON_DETECTION,
                details,
            )
        if (
            self.config.enable_poor_person_detection
            and detection_rate is not None
            and detection_rate < self.config.detection_rate_threshold
        ):
            return Calib2d3dRuntimeDecision(
                Calib2d3dErrorCommon.POOR_PERSON_DETECTION,
                Calib2d3dDiagnosisPhase.PERSON_DETECTION,
                details,
            )
        if (
            self.config.enable_poor_tracking_3d
            and tracking_3d_rate is not None
            and tracking_3d_rate < self.config.detection_rate_threshold
        ):
            return Calib2d3dRuntimeDecision(
                Calib2d3dErrorCommon.POOR_TRACKING_3D,
                Calib2d3dDiagnosisPhase.TRACKING_3D,
                details,
            )
        if (
            self.config.enable_poor_tracking_2d
            and tracking_2d_rate is not None
            and tracking_2d_rate < self.config.detection_rate_threshold
        ):
            return Calib2d3dRuntimeDecision(
                Calib2d3dErrorCommon.POOR_TRACKING_2D,
                Calib2d3dDiagnosisPhase.TRACKING_2D,
                details,
            )
        if self._continued(
            "too_dark",
            self.config.enable_unsuitable_condition and too_dark,
            now,
        ):
            return Calib2d3dRuntimeDecision(
                Calib2d3dErrorCommon.UNSUITABLE_CONDITION,
                Calib2d3dDiagnosisPhase.FRAME_INPUT,
                details,
            )
        return Calib2d3dRuntimeDecision(
            Calib2d3dErrorCommon.DEFAULT,
            Calib2d3dDiagnosisPhase.FRAME_INPUT,
            details,
        )
