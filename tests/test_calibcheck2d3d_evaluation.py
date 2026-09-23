# ruff: noqa: SLF001

from __future__ import annotations

from typing import Any, cast

import numpy as np

from argus_synchro.calibration_mat_generator_modules.ctrl.calibcheck2d3d import (
    Scene_CalibCheck2d3d,
    calibcheck2d3d,
)
from argus_synchro.calibration_mat_generator_modules.ctrl.calibcheck2d3d.evaluator import (
    select_visible_evaluation_frames,
)
from argus_synchro.calibration_mat_generator_modules.ctrl.calibration2d3d.track_main.interface_definition import (
    Tracking2dDataInterface,
    Tracking3dDataInterface,
    tracking2d_dataclass,
    tracking3d_dataclass,
)
from argus_synchro.config.app_config import SceneDescriptionConf
from argus_synchro.diagnosis.calibcheck2d3d_result_diagnosis import (
    BBoxLogObservation,
    CalibCheck2d3dDiagnosis,
    CalibCheckFailureReason,
    CalibCheckFrameObservation,
    TrackingObservation,
)


class _LoggerStub:
    def __init__(self) -> None:
        self.messages: list[str] = []

    def info(self, message: str) -> None:
        self.messages.append(message)


def _controller(camera_count: int = 1) -> calibcheck2d3d:
    controller = cast(calibcheck2d3d, object.__new__(calibcheck2d3d))
    controller._logger = cast(Any, _LoggerStub())
    controller._evaluation_camera_count = camera_count
    controller.width = 100
    controller.height = 100
    controller._evaluation_intrinsics = np.array(
        [[100.0, 0.0, 50.0], [0.0, 100.0, 50.0], [0.0, 0.0, 1.0]],
        dtype=np.float32,
    )
    controller.rtvec_mat = [
        (
            np.zeros((3, 1), dtype=np.float64),
            np.zeros((3, 1), dtype=np.float64),
            np.eye(4, dtype=np.float64),
        )
        for _ in range(camera_count)
    ]
    controller.EVAL_FRAME_STRIDE = 1
    controller.USE_LEGACY_LIKE_METRIC = True
    controller.VIRTUAL_BBOX_CANDIDATE_NAMES = (
        "real",
        "x_minus",
        "x_plus",
        "y_minus",
        "y_plus",
    )
    controller.DEBUG_CALIBCHECK_ENABLED = False
    controller.DEBUG_EVAL_TRACE_ENABLED = False
    controller.DEBUG_EVAL_TRACE_ALL_FRAMES = False
    controller.DEBUG_EVAL_TRACE_RANGE_START = 0
    controller.DEBUG_EVAL_TRACE_RANGE_END = -1
    controller.DEBUG_EVAL_TRACE_TARGET_FRAMES = set()
    controller._debug_video_paths = []
    controller._debug_eval_info = {}
    controller._virtual_bbox_debug_counts = []
    controller._evaluation_metric_debug = []
    return controller


def _tracking_3d(
    frame_ix: int, bbox: tuple[float, float, float, float]
) -> Tracking3dDataInterface:
    metadata = tracking3d_dataclass(
        accum_track_length=3.0,
        final_xy=(0.0, 0.0),
        dist_from_camera_min=1.0,
        dist_from_camera_max=2.0,
        frame_ix_min=frame_ix,
        frame_ix_max=frame_ix,
        frame_ix_lastmove=frame_ix,
        frame_evval_min=0.0,
        frame_evval_max=1.0,
        workarea_count=30,
        is_alive=True,
    )
    return Tracking3dDataInterface({1: metadata}, {1: [(frame_ix, bbox)]})


def _tracking_2d(
    frame_ix: int, bbox: tuple[float, float, float, float]
) -> Tracking2dDataInterface:
    metadata = tracking2d_dataclass(
        accum_track_length=50.0,
        final_xy=(0.0, 0.0),
        xymin=(bbox[0], bbox[1]),
        xymax=(bbox[2], bbox[3]),
        frame_ix_min=frame_ix,
        frame_ix_max=frame_ix,
        frame_ix_lastmove=frame_ix,
        frame_evval_min=0.0,
        frame_evval_max=1.0,
        workarea_count=10,
        is_alive=True,
    )
    return Tracking2dDataInterface({1: metadata}, {1: [(frame_ix, bbox)]})


def test_evaluate_2d3d_reports_reason_11_when_3d_is_out_of_view() -> None:
    controller = _controller()

    scores, reasons = controller.evaluate_2d3d(
        _tracking_3d(0, (10.0, -1.0, 12.0, 1.0)),
        [_tracking_2d(0, (30.0, 30.0, 70.0, 70.0))],
        zvalues=(5.0, 10.0),
    )

    assert scores == [0.0]
    assert reasons == [11]
    matching_statistics = controller.evaluation_metric_debug[0]
    assert matching_statistics["tracked_3d_frame_count"] == 1
    assert matching_statistics["visible_3d_frame_count"] == 0
    assert matching_statistics["valid_comparison_count"] == 0


def test_evaluate_2d3d_reports_reason_10_without_common_frame() -> None:
    controller = _controller()

    scores, reasons = controller.evaluate_2d3d(
        _tracking_3d(0, (-1.0, -1.0, 1.0, 1.0)),
        [_tracking_2d(1, (30.0, 30.0, 70.0, 70.0))],
        zvalues=(5.0, 10.0),
    )

    assert scores == [0.0]
    assert reasons == [10]
    sync_statistics = controller.evaluation_metric_debug[0]
    assert sync_statistics["visible_3d_frame_count"] == 1
    assert sync_statistics["tracked_2d_frame_count"] == 1
    assert sync_statistics["common_frame_count"] == 0
    assert sync_statistics["visible_3d_frame_range"] == (0, 0)
    assert sync_statistics["tracked_2d_frame_range"] == (1, 1)


def test_evaluate_2d3d_does_not_report_reason_10_without_2d_tracks() -> None:
    controller = _controller()

    scores, reasons = controller.evaluate_2d3d(
        _tracking_3d(0, (-1.0, -1.0, 1.0, 1.0)),
        [Tracking2dDataInterface({}, {})],
        zvalues=(5.0, 10.0),
    )

    assert scores == [0.0]
    assert reasons == [11]
    sync_statistics = controller.evaluation_metric_debug[0]
    assert sync_statistics["visible_3d_frame_count"] == 1
    assert sync_statistics["tracked_2d_frame_count"] == 0
    assert sync_statistics["common_frame_count"] == 0


def test_evaluate_2d3d_scores_common_overlapping_frame() -> None:
    controller = _controller()
    controller.evaluate_bbox_overlap_scenedesc = lambda *_args: 1.0

    scores, reasons = controller.evaluate_2d3d(
        _tracking_3d(0, (-1.0, -1.0, 1.0, 1.0)),
        [_tracking_2d(0, (30.0, 30.0, 70.0, 70.0))],
        zvalues=(5.0, 10.0),
    )

    assert scores == [1.0]
    assert reasons == [0]
    assert controller.judge_calibration_result(scores, threshold=0.5) == [True]


def test_evaluate_2d3d_records_shi_debug_metrics_when_enabled() -> None:
    controller = _controller()
    controller.DEBUG_CALIBCHECK_ENABLED = True
    controller.DEBUG_EVAL_TRACE_ENABLED = True
    controller.DEBUG_EVAL_TRACE_ALL_FRAMES = True
    controller.evaluate_bbox_overlap_scenedesc = lambda *_args: 1.0

    scores, reasons = controller.evaluate_2d3d(
        _tracking_3d(0, (-1.0, -1.0, 1.0, 1.0)),
        [_tracking_2d(0, (30.0, 30.0, 70.0, 70.0))],
        zvalues=(5.0, 10.0),
    )

    assert scores == [1.0]
    assert reasons == [0]
    assert controller.evaluation_metric_debug[0]["selected_score"] == 1.0
    assert controller._debug_eval_info["eval_trace_by_camera"][0][0][0][
        "stage"
    ] == "project_3d_to_2d"


def test_evaluate_2d3d_reports_reason_11_without_valid_score() -> None:
    controller = _controller()
    controller.evaluate_bbox_overlap_scenedesc = cast(Any, lambda *_args: None)

    scores, reasons = controller.evaluate_2d3d(
        _tracking_3d(0, (-1.0, -1.0, 1.0, 1.0)),
        [_tracking_2d(0, (30.0, 30.0, 70.0, 70.0))],
        zvalues=(5.0, 10.0),
    )

    assert scores == [0.0]
    assert reasons == [11]
    matching_statistics = controller.evaluation_metric_debug[0]
    assert matching_statistics["common_frame_count"] == 1
    assert matching_statistics["comparison_candidate_count"] == 1
    assert matching_statistics["overlap_candidate_count"] == 1
    assert matching_statistics["score_returned_count"] == 0
    assert matching_statistics["valid_comparison_count"] == 0


def test_visible_frame_sampling_keeps_last_visible_frame() -> None:
    frame_to_local_ix, sampled_local_ixs = select_visible_evaluation_frames(
        [10, 11, 12, 13, 14],
        [False, True, True, False, True],
        stride=2,
    )

    assert frame_to_local_ix == {11: 1, 12: 2, 14: 4}
    assert sampled_local_ixs == {1, 4}


def test_evaluate_2d3d_keeps_per_camera_reasons_and_metrics_independent() -> None:
    controller = _controller(camera_count=2)
    controller.evaluate_bbox_overlap_scenedesc = lambda *_args: 1.0

    scores, reasons = controller.evaluate_2d3d(
        _tracking_3d(0, (-1.0, -1.0, 1.0, 1.0)),
        [
            _tracking_2d(0, (30.0, 30.0, 70.0, 70.0)),
            _tracking_2d(1, (30.0, 30.0, 70.0, 70.0)),
        ],
        zvalues=(5.0, 10.0),
    )

    assert scores == [1.0, 0.0]
    assert reasons == [0, 10]
    assert [metric["selected_score"] for metric in controller.evaluation_metric_debug] == [
        1.0,
        0.0,
    ]
    assert [metric["denominator"] for metric in controller.evaluation_metric_debug] == [
        1,
        1,
    ]


def test_bbox_overlap_evaluation_uses_real_scene_matching() -> None:
    controller = _controller()
    controller.VIRTUAL_BBOX_XY_OFFSETS = ((0.0, 0.0),)
    controller.scenedesc_calibcheck = Scene_CalibCheck2d3d.create_for_evaluation(
        scene_conf=SceneDescriptionConf(
            coarse_lo=0.01,
            coarse_hi=100.0,
            k_min=0.3,
            h_ref_px=80,
            lo_gain=1.0,
            hi_gain=1.0,
            lo_floor=0.01,
            hi_ceil=100.0,
            vertical_w_iou=0.0,
            vertical_w_scale=0.0,
            vertical_w_phi=0.0,
            final_threshold=1000.0,
            use_human_gate=False,
            H_min=1.2,
            H_max=2.2,
            W_min=0.2,
            W_max=1.0,
            D_min=0.2,
            D_max=1.0,
            tall_ratio_min=1.5,
        ),
        camera_intrinsics=controller._evaluation_intrinsics,
        image_width=controller.width,
        image_height=controller.height,
    )

    score = controller.evaluate_bbox_overlap_scenedesc(
        np.array([-1.0, -1.0, 5.0, 1.0, 1.0, 6.0], dtype=np.float32),
        np.array([30.0, 30.0, 70.0, 70.0], dtype=np.float32),
        camera_index=0,
    )

    assert score == 1.0


def test_data_evaluation_preserves_earlier_per_camera_reason() -> None:
    controller = _controller(camera_count=2)
    controller.calibcheck2d3d_conf = cast(
        Any,
        type(
            "Config",
            (),
            {"score_value_threshold": 0.5, "score_accept_count_threshold": 1},
        )(),
    )
    controller.EVAL_ZVALUES = (5.0, 10.0)
    tracking_3d = _tracking_3d(0, (-1.0, -1.0, 1.0, 1.0))
    tracking_2d = [
        _tracking_2d(0, (30.0, 30.0, 70.0, 70.0)),
        _tracking_2d(0, (30.0, 30.0, 70.0, 70.0)),
    ]
    controller.collect_bbox_log_observation = lambda: BBoxLogObservation(
        total_frame_count=10,
        valid_3d_frame_count=10,
        valid_2d_frame_counts=(0, 10),
        minimum_3d_bbox_count_per_frame=1,
        minimum_3d_valid_frame_ratio=0.5,
        minimum_2d_bbox_count_per_frame=1,
        minimum_2d_valid_frame_ratio=0.5,
    )
    controller.track_3dbbox = lambda: tracking_3d
    controller.track_2dbbox = lambda: tracking_2d
    controller.collect_tracking_observation = lambda *_args: TrackingObservation(
        total_3d_track_count=1,
        alive_3d_track_count=1,
        total_2d_track_counts=(1, 1),
        alive_2d_track_counts=(1, 1),
        minimum_3d_alive_track_count=1,
        minimum_2d_alive_track_count=1,
        proximity_warning_count=0,
        proximity_warning_fails_validation=True,
    )
    controller.evaluate_2d3d = lambda *_args, **_kwargs: ([1.0, 1.0], [0, 0])
    controller._evaluation_metric_debug = [
        {
            "numerator": 1,
            "denominator": 1,
            "strict_hit_rate": 1.0,
            "legacy_numerator": 1.0,
            "legacy_denominator": 1,
            "legacy_like_score": 1.0,
            "selected_score": 1.0,
            "use_legacy_like_metric": False,
            "visible_3d_frame_count": 1,
            "tracked_2d_frame_count": 1,
            "common_frame_count": 1,
            "visible_3d_frame_range": (0, 0),
            "tracked_2d_frame_range": (0, 0),
            "tracked_3d_frame_count": 1,
            "comparison_candidate_count": 1,
            "overlap_candidate_count": 1,
            "score_returned_count": 1,
            "valid_comparison_count": 1,
        }
        for _ in range(2)
    ]

    results = controller.data_evaluation_process()

    assert [int(result.primary_reason) for result in results] == [3, 0]
    assert [result.calibration_is_acceptable for result in results] == [None, True]
    assert not any(
        "bbox log diagnosis failed: reason=2" in message
        for message in controller._logger.messages
    )
    assert any(
        "bbox log diagnosis failed: reason=3, camera_index=0" in message
        and "'valid_frame_ratio': 0.0" in message
        and "'minimum_valid_frame_ratio': 0.5" in message
        for message in controller._logger.messages
    )
    assert not any(
        "tracking diagnosis failed" in message
        for message in controller._logger.messages
    )


def test_data_evaluation_keeps_reason_9_when_camera_has_no_2d_data() -> None:
    controller = _controller(camera_count=2)
    controller.calibcheck2d3d_conf = cast(
        Any,
        type(
            "Config",
            (),
            {"score_value_threshold": 0.5, "score_accept_count_threshold": 1},
        )(),
    )
    controller.EVAL_ZVALUES = (5.0, 10.0)
    elapsed_times = iter((0.0, 11.0))
    diagnosis = CalibCheck2d3dDiagnosis(
        camera_count=2,
        person_not_detected_sec=10.0,
        elapsed_time=lambda: next(elapsed_times),
    )
    diagnosis.diagnose_frame(
        CalibCheckFrameObservation(camera_detection_counts=(0, 1))
    )
    diagnosis.diagnose_frame(
        CalibCheckFrameObservation(camera_detection_counts=(0, 1))
    )
    controller._calibcheck_diagnosis = diagnosis
    tracking_3d = _tracking_3d(0, (-1.0, -1.0, 1.0, 1.0))
    tracking_2d = [
        Tracking2dDataInterface({}, {}),
        _tracking_2d(0, (30.0, 30.0, 70.0, 70.0)),
    ]
    controller.collect_bbox_log_observation = lambda: BBoxLogObservation(
        total_frame_count=10,
        valid_3d_frame_count=10,
        valid_2d_frame_counts=(0, 10),
        minimum_3d_bbox_count_per_frame=1,
        minimum_3d_valid_frame_ratio=0.5,
        minimum_2d_bbox_count_per_frame=1,
        minimum_2d_valid_frame_ratio=0.5,
    )
    controller.track_3dbbox = lambda: tracking_3d
    controller.track_2dbbox = lambda: tracking_2d
    controller.collect_tracking_observation = lambda *_args: TrackingObservation(
        total_3d_track_count=1,
        alive_3d_track_count=1,
        total_2d_track_counts=(0, 1),
        alive_2d_track_counts=(0, 1),
        minimum_3d_alive_track_count=1,
        minimum_2d_alive_track_count=1,
        proximity_warning_count=0,
        proximity_warning_fails_validation=True,
    )
    # camera 0を含む全カメラをevaluateしても、先行診断が不良な
    # camera 0に後段のreason 10を追加しないことを確認する。
    controller.evaluate_2d3d = lambda *_args, **_kwargs: ([0.0, 1.0], [10, 0])
    controller._evaluation_metric_debug = [
        {
            "numerator": 0 if camera_index == 0 else 1,
            "denominator": 0 if camera_index == 0 else 1,
            "strict_hit_rate": 0.0 if camera_index == 0 else 1.0,
            "legacy_numerator": 0.0 if camera_index == 0 else 1.0,
            "legacy_denominator": 0 if camera_index == 0 else 1,
            "legacy_like_score": 0.0 if camera_index == 0 else 1.0,
            "selected_score": 0.0 if camera_index == 0 else 1.0,
            "use_legacy_like_metric": False,
            "visible_3d_frame_count": 1,
            "tracked_2d_frame_count": camera_index,
            "common_frame_count": camera_index,
            "visible_3d_frame_range": (0, 0),
            "tracked_2d_frame_range": None if camera_index == 0 else (0, 0),
            "tracked_3d_frame_count": 1,
            "comparison_candidate_count": camera_index,
            "overlap_candidate_count": camera_index,
            "score_returned_count": camera_index,
            "valid_comparison_count": camera_index,
        }
        for camera_index in range(2)
    ]

    results = controller.data_evaluation_process()

    assert results[0].primary_reason is CalibCheckFailureReason.CAMERA_PERSON_NOT_DETECTED
    assert CalibCheckFailureReason.SENSOR_TIME_SYNC_INVALID not in results[0].all_reasons
    assert CalibCheckFailureReason.MATCHED_PERSON_NOT_DETECTED not in results[0].all_reasons
    assert results[1].primary_reason is CalibCheckFailureReason.NONE
