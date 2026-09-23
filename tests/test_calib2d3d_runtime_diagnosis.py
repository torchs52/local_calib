from __future__ import annotations

import numpy as np

from argus_synchro.diagnosis.calib2d3d_result_diagnosis import (
    Calib2d3dErrorCommon,
)
from argus_synchro.diagnosis.calib2d3d_runtime_diagnosis import (
    Calib2d3dFrameObservation,
    Calib2d3dRuntimeDiagnosis,
    Calib2d3dRuntimeDiagnosisConfig,
    calculate_sampled_brightness,
    point_is_strictly_inside_polygon,
)

SQUARE = ((0.0, 0.0), (10.0, 0.0), (10.0, 10.0), (0.0, 10.0))


def _observation(
    *,
    image_value: int = 100,
    detection_2d_count: int = 1,
    bbox_3d_count: int = 1,
    tracking_2d_id_count: int = 1,
    tracking_3d_id_count: int = 1,
    center: tuple[float, float] = (5.0, 5.0),
) -> Calib2d3dFrameObservation:
    return Calib2d3dFrameObservation(
        image=np.full((8, 8, 3), image_value, dtype=np.uint8),
        detection_2d_count=detection_2d_count,
        bbox_3d_count=bbox_3d_count,
        tracking_2d_id_count=tracking_2d_id_count,
        tracking_3d_id_count=tracking_3d_id_count,
        bbox_3d_centers_xy=(center,) if bbox_3d_count else (),
    )


def test_polygon_boundary_is_outside() -> None:
    assert point_is_strictly_inside_polygon((5.0, 5.0), SQUARE)
    assert not point_is_strictly_inside_polygon((0.0, 5.0), SQUARE)
    assert not point_is_strictly_inside_polygon((12.0, 5.0), SQUARE)


def test_sampled_brightness_uses_mean_without_histogram() -> None:
    brightness = 12
    image = np.full((16, 16, 3), brightness, dtype=np.uint8)
    assert calculate_sampled_brightness(image, stride=4) == float(brightness)


def test_walking_range_requires_20_seconds_and_clears_immediately() -> None:
    diagnosis = Calib2d3dRuntimeDiagnosis(
        Calib2d3dRuntimeDiagnosisConfig(
            walking_area_enabled=True,
            walking_area_corners=SQUARE,
        ),
        clock=lambda: 0.0,
    )
    outside = _observation(center=(12.0, 5.0))

    assert diagnosis.diagnose(outside, now=0.0).common_error == 0
    assert diagnosis.diagnose(outside, now=19.9).common_error == 0
    assert (
        diagnosis.diagnose(outside, now=20.0).common_error
        is Calib2d3dErrorCommon.WALKING_RANGE_INVALID
    )
    assert diagnosis.diagnose(_observation(), now=20.1).common_error == 0


def test_disabled_walking_range_diagnosis_never_fires() -> None:
    diagnosis = Calib2d3dRuntimeDiagnosis(
        Calib2d3dRuntimeDiagnosisConfig(
            enable_walking_range=False,
            walking_area_enabled=True,
            walking_area_corners=SQUARE,
        ),
        clock=lambda: 0.0,
    )
    outside = _observation(center=(12.0, 5.0))
    diagnosis.diagnose(outside, now=0.0)
    assert diagnosis.diagnose(outside, now=30.0).common_error == 0


def test_person_count_uses_tracking_ids_after_20_seconds() -> None:
    diagnosis = Calib2d3dRuntimeDiagnosis(
        Calib2d3dRuntimeDiagnosisConfig(), clock=lambda: 0.0
    )
    two_people = _observation(
        detection_2d_count=2,
        bbox_3d_count=2,
        tracking_2d_id_count=2,
        tracking_3d_id_count=2,
    )
    diagnosis.diagnose(two_people, now=0.0)
    assert (
        diagnosis.diagnose(two_people, now=20.0).common_error
        is Calib2d3dErrorCommon.WALKING_PERSON_COUNT_INVALID
    )


def test_continuous_no_detection_becomes_fatal_after_20_seconds() -> None:
    diagnosis = Calib2d3dRuntimeDiagnosis(
        Calib2d3dRuntimeDiagnosisConfig(), clock=lambda: 0.0
    )
    missing = _observation(
        detection_2d_count=0,
        bbox_3d_count=0,
        tracking_2d_id_count=0,
        tracking_3d_id_count=0,
    )
    diagnosis.diagnose(missing, now=0.0)
    assert (
        diagnosis.diagnose(missing, now=20.0).common_error
        is Calib2d3dErrorCommon.PERSON_DETECTION_IMPOSSIBLE
    )


def test_low_detection_rate_uses_safe_provisional_threshold() -> None:
    diagnosis = Calib2d3dRuntimeDiagnosis(
        Calib2d3dRuntimeDiagnosisConfig(
            enable_person_detection_impossible=False,
        ),
        clock=lambda: 0.0,
    )
    diagnosis.diagnose(_observation(), now=0.0)
    missing = _observation(
        detection_2d_count=0,
        bbox_3d_count=0,
        tracking_2d_id_count=0,
        tracking_3d_id_count=0,
    )
    for second in range(1, 21):
        result = diagnosis.diagnose(missing, now=float(second))
    assert result.common_error is Calib2d3dErrorCommon.POOR_PERSON_DETECTION


def test_long_duration_is_independently_switchable() -> None:
    enabled = Calib2d3dRuntimeDiagnosis(
        Calib2d3dRuntimeDiagnosisConfig(), clock=lambda: 0.0
    )
    assert (
        enabled.diagnose(_observation(), now=600.0).common_error
        is Calib2d3dErrorCommon.LONG_DURATION_CALIBRATION
    )

    disabled = Calib2d3dRuntimeDiagnosis(
        Calib2d3dRuntimeDiagnosisConfig(enable_long_duration=False),
        clock=lambda: 0.0,
    )
    assert disabled.diagnose(_observation(), now=600.0).common_error == 0
