from collections.abc import Callable
from dataclasses import dataclass

import numpy as np
from numpy.typing import NDArray

from argus_synchro.calibration_mat_generator_modules.ctrl.calibcheck2d3d.processor import (
    EvaluationMetricDebug,
    VirtualBBoxDebugCounts,
)
from argus_synchro.calibration_mat_generator_modules.ctrl.calibration2d3d.track_main.interface_definition import (
    Tracking2dDataInterface,
    Tracking3dDataInterface,
)
from argus_synchro.diagnosis.calibcheck2d3d_result_diagnosis import (
    BBoxLogObservation,
    CalibCheck2d3dDiagnosis,
    CalibCheckFailureReason,
    CameraCalibCheckDiagnosisResult,
    EvaluationMatchingStatistics,
    EvaluationObservation,
    EvaluationStatistics,
    EvaluationTimeSyncStatistics,
    JudgementObservation,
    StatisticsObservation,
    TrackingObservation,
    normalize_calibcheck_reason,
)


@dataclass
class EvaluationRuntime:
    evaluate: Callable[
        [Tracking3dDataInterface, list[Tracking2dDataInterface], tuple[float, float]],
        tuple[list[float], list[int]],
    ]
    virtual_bbox_debug_counts: list[VirtualBBoxDebugCounts]
    evaluation_metric_debug: list[EvaluationMetricDebug]
    debug_eval_info: dict[str, object]


@dataclass
class Projected3dTrack:
    frame_indexes: list[int]
    bboxes3d: list[NDArray[np.float32]]
    projected_bboxes2d: list[NDArray[np.float32] | None]
    visible_mask: list[bool]


def project_3dbbox_core(
    bbox3d: NDArray[np.float32],
    *,
    camera_extrinsics: NDArray[np.float64],
    camera_intrinsics: NDArray[np.float32],
    image_width: int,
    image_height: int,
    require_points_in_image: bool = False,
) -> NDArray[np.float32] | None:
    # 3D bbox の 8 頂点をカメラ座標系へ変換して画像平面に投影する。
    bbox3d_corners = np.array(
        [
            [bbox3d[0], bbox3d[1], bbox3d[2]],
            [bbox3d[0], bbox3d[1], bbox3d[5]],
            [bbox3d[0], bbox3d[4], bbox3d[2]],
            [bbox3d[0], bbox3d[4], bbox3d[5]],
            [bbox3d[3], bbox3d[1], bbox3d[2]],
            [bbox3d[3], bbox3d[1], bbox3d[5]],
            [bbox3d[3], bbox3d[4], bbox3d[2]],
            [bbox3d[3], bbox3d[4], bbox3d[5]],
        ],
        dtype=np.float32,
    )
    bbox3d_corners_homogeneous = np.hstack(
        (bbox3d_corners, np.ones((bbox3d_corners.shape[0], 1), dtype=np.float32))
    )
    bbox3d_corners_camera = (
        camera_extrinsics @ bbox3d_corners_homogeneous.T
    ).T[:, :3]
    if np.any(bbox3d_corners_camera[:, 2] <= 0):
        # 1 頂点でもカメラ前方にない bbox は評価対象にしない。
        return None

    bbox3d_corners_image = (camera_intrinsics @ bbox3d_corners_camera.T).T
    bbox3d_corners_image = (
        bbox3d_corners_image[:, :2] / bbox3d_corners_image[:, 2:3]
    )
    if require_points_in_image:
        # 可視判定では、8 頂点すべてが画像内にあることを要求する。
        x_in = (bbox3d_corners_image[:, 0] >= 0.0) & (
            bbox3d_corners_image[:, 0] <= float(image_width - 1)
        )
        y_in = (bbox3d_corners_image[:, 1] >= 0.0) & (
            bbox3d_corners_image[:, 1] <= float(image_height - 1)
        )
        if not np.all(x_in & y_in):
            return None

    return np.array(
        [
            np.min(bbox3d_corners_image[:, 0]),
            np.min(bbox3d_corners_image[:, 1]),
            np.max(bbox3d_corners_image[:, 0]),
            np.max(bbox3d_corners_image[:, 1]),
        ],
        dtype=np.float32,
    )


def shrink_bbox2d(
    bbox2d: NDArray[np.float32], factor: float
) -> NDArray[np.float32]:
    # bbox format は (x1, y1, x2, y2)。中心を保ったまま縦横を factor 倍する。
    center_x = (bbox2d[0] + bbox2d[2]) / 2.0
    center_y = (bbox2d[1] + bbox2d[3]) / 2.0
    half_width = (bbox2d[2] - bbox2d[0]) * factor / 2.0
    half_height = (bbox2d[3] - bbox2d[1]) * factor / 2.0
    return np.array(
        [
            center_x - half_width,
            center_y - half_height,
            center_x + half_width,
            center_y + half_height,
        ],
        dtype=np.float32,
    )


def passes_center_diff_gate(
    bbox_a: NDArray[np.float32],
    bbox_b: NDArray[np.float32],
    *,
    shrink_factor: float,
    threshold: float,
) -> bool:
    # 縮小後の縦横長さの平均で中心差を正規化し、ユークリッド距離を閾値判定する。
    shrunk_a = shrink_bbox2d(bbox_a, shrink_factor)
    shrunk_b = shrink_bbox2d(bbox_b, shrink_factor)
    width_a, height_a = shrunk_a[2] - shrunk_a[0], shrunk_a[3] - shrunk_a[1]
    width_b, height_b = shrunk_b[2] - shrunk_b[0], shrunk_b[3] - shrunk_b[1]
    center_a = ((shrunk_a[0] + shrunk_a[2]) / 2.0, (shrunk_a[1] + shrunk_a[3]) / 2.0)
    center_b = ((shrunk_b[0] + shrunk_b[2]) / 2.0, (shrunk_b[1] + shrunk_b[3]) / 2.0)
    average_width = max((width_a + width_b) / 2.0, 1e-6)
    average_height = max((height_a + height_b) / 2.0, 1e-6)
    normalized_x = (center_a[0] - center_b[0]) / average_width
    normalized_y = (center_a[1] - center_b[1]) / average_height
    return float(np.hypot(normalized_x, normalized_y)) <= threshold


def has_positive_2d_intersection(
    bbox_a: NDArray[np.float32], bbox_b: NDArray[np.float32]
) -> bool:
    ax1, ay1 = min(bbox_a[0], bbox_a[2]), min(bbox_a[1], bbox_a[3])
    ax2, ay2 = max(bbox_a[0], bbox_a[2]), max(bbox_a[1], bbox_a[3])
    bx1, by1 = min(bbox_b[0], bbox_b[2]), min(bbox_b[1], bbox_b[3])
    bx2, by2 = max(bbox_b[0], bbox_b[2]), max(bbox_b[1], bbox_b[3])
    return min(ax2, bx2) - max(ax1, bx1) > 0.0 and min(ay2, by2) - max(ay1, by1) > 0.0


def evaluate_bbox_overlap_scenedesc(
    bbox3d: NDArray[np.float32],
    bbox2d: NDArray[np.float32],
    *,
    camera_index: int,
    rvec: NDArray[np.float64],
    tvec: NDArray[np.float64],
    camera_extrinsics: NDArray[np.float64],
    camera_intrinsics: NDArray[np.float32],
    image_width: int,
    image_height: int,
    scene: object,
    virtual_bbox_xy_offsets: tuple[tuple[float, float], ...],
    virtual_bbox_candidate_names: tuple[str, ...],
    virtual_bbox_debug_counts: list[VirtualBBoxDebugCounts],
    bbox_shrink_factor: float,
    center_diff_ratio_threshold: float,
) -> float:
    # 実 bbox と XY オフセットを持つ仮想 bbox を一括で Scene に渡し、対応付けを評価する。
    projected_bbox2d = project_3dbbox_core(
        bbox3d,
        camera_extrinsics=camera_extrinsics,
        camera_intrinsics=camera_intrinsics,
        image_width=image_width,
        image_height=image_height,
    )
    if projected_bbox2d is not None and not passes_center_diff_gate(
        projected_bbox2d,
        bbox2d,
        shrink_factor=bbox_shrink_factor,
        threshold=center_diff_ratio_threshold,
    ):
        return 0.0

    candidate_minmax_list: list[NDArray[np.float32]] = []
    candidate_corners_list: list[NDArray[np.float32]] = []
    for offset_x, offset_y in virtual_bbox_xy_offsets:
        # evaluator の min/max 形式から Scene 側の [xmin, xmax, ymin, ymax, zmin, zmax] へ変換する。
        candidate_bbox3d = bbox3d.copy()
        candidate_bbox3d[[0, 3]] += offset_x
        candidate_bbox3d[[1, 4]] += offset_y
        candidate_minmax = np.array(
            [
                candidate_bbox3d[0],
                candidate_bbox3d[3],
                candidate_bbox3d[1],
                candidate_bbox3d[4],
                candidate_bbox3d[2],
                candidate_bbox3d[5],
            ],
            dtype=np.float32,
        )
        candidate_minmax_list.append(candidate_minmax)
        candidate_corners_list.append(
            np.array(
                [
                    [candidate_minmax[0], candidate_minmax[2], candidate_minmax[4]],
                    [candidate_minmax[1], candidate_minmax[2], candidate_minmax[4]],
                    [candidate_minmax[0], candidate_minmax[3], candidate_minmax[4]],
                    [candidate_minmax[1], candidate_minmax[3], candidate_minmax[4]],
                    [candidate_minmax[0], candidate_minmax[2], candidate_minmax[5]],
                    [candidate_minmax[1], candidate_minmax[2], candidate_minmax[5]],
                    [candidate_minmax[0], candidate_minmax[3], candidate_minmax[5]],
                    [candidate_minmax[1], candidate_minmax[3], candidate_minmax[5]],
                ],
                dtype=np.float32,
            )
        )

    bbox_class = scene.integrate2d3d_calibcheck(
        # YOLO bbox は pixels から [ymin, xmin, ymax, xmax] の正規化形式へ変換する。
        rvec=rvec,
        tvec=tvec,
        boxpoints=np.concatenate(candidate_corners_list),
        bbox2d=np.array(
            [[
                bbox2d[1] / image_height,
                bbox2d[0] / image_width,
                bbox2d[3] / image_height,
                bbox2d[2] / image_width,
            ]],
            dtype=np.float32,
        ),
        minmax3ds=np.stack(candidate_minmax_list),
        yolo_classes=np.array([0], dtype=np.int32),
        n_clusters=len(virtual_bbox_xy_offsets),
        bbox2d_detection_count=1,
        method="center",
    )
    selected_candidate_index = next(iter(bbox_class), None)
    if camera_index < len(virtual_bbox_debug_counts):
        camera_counts = virtual_bbox_debug_counts[camera_index]
        camera_counts["comparison_count"] += 1
        if selected_candidate_index is not None:
            selected_name = virtual_bbox_candidate_names[selected_candidate_index]
            camera_counts["selected_candidate_counts"][selected_name] += 1
            if selected_candidate_index != 0:
                camera_counts["virtual_bbox_win_count"] += 1
    return 1.0 if bbox_class.get(0) == "HUMAN" else 0.0


def judge_calibration_result(
    evaluation_statistics: list[float], *, threshold: float
) -> list[bool]:
    return [score >= threshold for score in evaluation_statistics]


def select_camera_evaluation_result(
    *,
    has_inframe_3d: bool,
    has_2d_tracking_frames: bool,
    has_time_match: bool,
    has_valid_comparison: bool,
    strict_hit_rate: float,
    legacy_like_score: float,
    use_legacy_like_metric: bool,
) -> tuple[float, int]:
    """カメラ別scoreと評価不能理由を選ぶ。

    reason 10は2D/3Dの両方に追跡フレームがある場合の共通フレームなし、
    11は画像内へ投影可能な3D対象、2D追跡対象、または有効scoreを得る比較なしを
    表す。reason 9は収集ループの人物検出実績から診断する。
    """
    if not has_inframe_3d:
        return 0.0, CalibCheckFailureReason.MATCHED_PERSON_NOT_DETECTED
    # 2D追跡フレーム自体がない場合は同期不良ではない。
    # 両側にフレームがある場合だけ、共通フレームの有無をreason 10で評価する。
    if not has_2d_tracking_frames:
        return 0.0, CalibCheckFailureReason.MATCHED_PERSON_NOT_DETECTED
    if not has_time_match:
        return 0.0, CalibCheckFailureReason.SENSOR_TIME_SYNC_INVALID
    if not has_valid_comparison:
        return 0.0, CalibCheckFailureReason.MATCHED_PERSON_NOT_DETECTED
    if use_legacy_like_metric:
        return legacy_like_score, CalibCheckFailureReason.NONE
    return strict_hit_rate, CalibCheckFailureReason.NONE


def create_evaluation_debug_state(
    *,
    camera_count: int,
    virtual_bbox_candidate_names: tuple[str, ...],
) -> tuple[list[VirtualBBoxDebugCounts], list[EvaluationMetricDebug]]:
    """1 回の評価サイクル用に、カメラ別の診断情報を新規作成する。"""
    return (
        [
            {
                "comparison_count": 0,
                "virtual_bbox_win_count": 0,
                "selected_candidate_counts": dict.fromkeys(
                    virtual_bbox_candidate_names, 0
                ),
            }
            for _ in range(camera_count)
        ],
        [],
    )


def create_evaluation_metric_debug(
    *,
    numerator: int,
    denominator: int,
    strict_hit_rate: float,
    legacy_numerator: float,
    legacy_denominator: int,
    legacy_like_score: float,
    selected_score: float,
    use_legacy_like_metric: bool,
    visible_3d_frame_indexes: set[int],
    tracked_2d_frame_indexes: set[int],
    common_frame_indexes: set[int],
    tracked_3d_frame_indexes: set[int],
    comparison_candidate_count: int,
    overlap_candidate_count: int,
    score_returned_count: int,
    valid_comparison_count: int,
) -> EvaluationMetricDebug:
    """現地評価時に比較できるよう、両方の score 算出式を保持する。"""
    return {
        "numerator": numerator,
        "denominator": denominator,
        "strict_hit_rate": strict_hit_rate,
        "legacy_numerator": legacy_numerator,
        "legacy_denominator": legacy_denominator,
        "legacy_like_score": legacy_like_score,
        "selected_score": selected_score,
        "use_legacy_like_metric": use_legacy_like_metric,
        "visible_3d_frame_count": len(visible_3d_frame_indexes),
        "tracked_2d_frame_count": len(tracked_2d_frame_indexes),
        "common_frame_count": len(common_frame_indexes),
        "visible_3d_frame_range": (
            (min(visible_3d_frame_indexes), max(visible_3d_frame_indexes))
            if visible_3d_frame_indexes
            else None
        ),
        "tracked_2d_frame_range": (
            (min(tracked_2d_frame_indexes), max(tracked_2d_frame_indexes))
            if tracked_2d_frame_indexes
            else None
        ),
        "tracked_3d_frame_count": len(tracked_3d_frame_indexes),
        "comparison_candidate_count": comparison_candidate_count,
        "overlap_candidate_count": overlap_candidate_count,
        "score_returned_count": score_returned_count,
        "valid_comparison_count": valid_comparison_count,
    }


def select_visible_evaluation_frames(
    frame_indexes: list[int],
    visible_mask: list[bool],
    *,
    stride: int,
) -> tuple[dict[int, int], set[int]]:
    """可視 3D フレームだけを評価対象にし、必要時は一定間隔でサンプリングする。

    stride が 1 より大きい場合も、最後の可視フレームは必ず評価に含める。
    """
    visible_frame_to_local_ix = {
        frame_indexes[local_ix]: local_ix
        for local_ix, is_visible in enumerate(visible_mask)
        if is_visible
    }
    visible_local_ixs = [
        local_ix for local_ix, is_visible in enumerate(visible_mask) if is_visible
    ]
    if stride > 1:
        sampled_local_ixs = {
            local_ix
            for index, local_ix in enumerate(visible_local_ixs)
            if index % stride == 0
        }
        if visible_local_ixs:
            sampled_local_ixs.add(visible_local_ixs[-1])
    else:
        sampled_local_ixs = set(visible_local_ixs)
    return visible_frame_to_local_ix, sampled_local_ixs


def project_3d_track_bboxes(
    bboxlog_3d: list[tuple[int, tuple[float, float, float, float]]],
    *,
    track_id_3d: int,
    camera_ix: int,
    zvalues: tuple[float, float],
    project_bbox: Callable[[NDArray[np.float32], int], NDArray[np.float32] | None],
    should_trace_frame: Callable[[int], bool],
    append_trace: Callable[[int, int, dict[str, object]], None],
) -> Projected3dTrack:
    """3D track の実在フレームを投影し、画像内にあるかを同じ順序で記録する。"""
    frame_indexes: list[int] = []
    bboxes3d: list[NDArray[np.float32]] = []
    projected_bboxes2d: list[NDArray[np.float32] | None] = []
    visible_mask: list[bool] = []
    for frame_ix, bbox3d_xy in bboxlog_3d:
        bbox3d_full = np.array(
            [
                bbox3d_xy[0],
                bbox3d_xy[1],
                zvalues[0],
                bbox3d_xy[2],
                bbox3d_xy[3],
                zvalues[1],
            ],
            dtype=np.float32,
        )
        projected_bbox2d = project_bbox(bbox3d_full, camera_ix)
        if should_trace_frame(frame_ix):
            append_trace(
                camera_ix,
                int(frame_ix),
                {
                    "stage": "project_3d_to_2d",
                    "id3d": int(track_id_3d),
                    "bbox3d": bbox3d_full.tolist(),
                    "projected_bbox2d": (
                        projected_bbox2d.tolist()
                        if projected_bbox2d is not None
                        else None
                    ),
                    "is_visible_in_image": projected_bbox2d is not None,
                },
            )
        frame_indexes.append(frame_ix)
        bboxes3d.append(bbox3d_full)
        projected_bboxes2d.append(projected_bbox2d)
        visible_mask.append(projected_bbox2d is not None)
    return Projected3dTrack(
        frame_indexes=frame_indexes,
        bboxes3d=bboxes3d,
        projected_bboxes2d=projected_bboxes2d,
        visible_mask=visible_mask,
    )


def _run_evaluation_impl_core(
    tracking_3d_data_interface: Tracking3dDataInterface,
    tracking_2d_data_interfaces: list[Tracking2dDataInterface],
    *,
    zvalues: tuple[float, float],
    camera_count: int,
    eval_frame_stride: int,
    use_legacy_like_metric: bool,
    debug_enabled: bool,
    bbox_shrink_factor: float,
    virtual_bbox_candidate_names: list[str],
    project_3dbbox_core_fn: Callable,
    should_trace_eval_frame: Callable,
    append_trace_event: Callable,
    evaluate_bbox_overlap_scenedesc_fn: Callable,
    shrink_bbox2d_fn: Callable,
    has_positive_2d_intersection_fn: Callable,
    virtual_bbox_debug_counts: list[VirtualBBoxDebugCounts],
    evaluation_metric_debug: list[EvaluationMetricDebug],
    debug_eval_info: dict,
    logger: object,
    debug_eval_trace_enabled: bool,
    debug_eval_trace_all_frames: bool,
    debug_eval_trace_range_start: int,
    debug_eval_trace_range_end: int,
    debug_eval_trace_target_frames: list[int],
    debug_video_paths: list[str],
) -> tuple[list[float], list[int]]:
    """2D3D評価ループで3D軌跡を各カメラの2D軌跡と照合し、スコアを集計する。

    Extracted from facade._evaluate_2d3d_impl() with all state passed via callbacks.
    """
    mean_scores_per_camera = [0.0 for _ in range(camera_count)]
    reason_camera_notvalid = [0 for _ in range(camera_count)]
    overlap_debug_by_camera: dict[int, dict[str, object]] = {
        camera_ix: {"overlap_frame_ix_set": set(), "overlap_events": []}
        for camera_ix in range(camera_count)
    }
    eval_trace_by_camera: dict[int, dict[int, list[dict[str, object]]]] = {}
    (
        virtual_bbox_debug_counts_result,
        evaluation_metric_debug_result,
    ) = create_evaluation_debug_state(
        camera_count=camera_count,
        virtual_bbox_candidate_names=virtual_bbox_candidate_names,
    )
    virtual_bbox_debug_counts.clear()
    virtual_bbox_debug_counts.extend(virtual_bbox_debug_counts_result)
    evaluation_metric_debug.clear()
    evaluation_metric_debug.extend(evaluation_metric_debug_result)

    for camera_ix, tracking_2d_data_interface in enumerate(
        tracking_2d_data_interfaces
    ):
        camera_has_inframe = False
        camera_has_timematch = False
        camera_has_valid = False
        camera_numerator = 0
        camera_denominator = 0
        camera_sampled_count = 0
        camera_legacy_numerator = 0.0
        camera_legacy_denominator = 0
        visible_3d_frame_indexes: set[int] = set()
        tracked_2d_frame_indexes: set[int] = set()
        common_frame_indexes: set[int] = set()
        tracked_3d_frame_indexes: set[int] = set()
        comparison_candidate_count = 0
        overlap_candidate_count = 0
        score_returned_count = 0
        valid_comparison_count = 0
        alive_2d_ids = [
            track_id
            for track_id, metadata in tracking_2d_data_interface.trackingIDmetadata.items()
            if metadata.is_alive
        ]
        for track_id_2d in alive_2d_ids:
            tracked_2d_frame_indexes.update(
                frame_ix
                for frame_ix, _bbox in tracking_2d_data_interface.trackingIDbboxlog.get(
                    track_id_2d, []
                )
            )

        for (
            track_id_3d,
            metadata_3d,
        ) in tracking_3d_data_interface.trackingIDmetadata.items():
            if not metadata_3d.is_alive:
                continue
            bboxlog_3d = tracking_3d_data_interface.trackingIDbboxlog.get(
                track_id_3d, []
            )
            if len(bboxlog_3d) == 0:
                continue

            projected_track = project_3d_track_bboxes(
                bboxlog_3d,
                track_id_3d=int(track_id_3d),
                camera_ix=camera_ix,
                zvalues=zvalues,
                project_bbox=lambda bbox3d, target_camera_ix, _fn=project_3dbbox_core_fn: (
                    _fn(bbox3d, target_camera_ix)
                ),
                should_trace_frame=should_trace_eval_frame,
                append_trace=lambda trace_camera_ix, frame_ix, event, _ec=eval_trace_by_camera, _fn=append_trace_event: (
                    _fn(_ec, trace_camera_ix, frame_ix, event)
                ),
            )
            tracked_3d_frame_indexes.update(projected_track.frame_indexes)

            (
                visible_frame_to_local_ix,
                sampled_visible_local_ix_set,
            ) = select_visible_evaluation_frames(
                projected_track.frame_indexes,
                projected_track.visible_mask,
                stride=eval_frame_stride,
            )
            visible_count = len(visible_frame_to_local_ix)
            visible_3d_frame_indexes.update(visible_frame_to_local_ix)
            if visible_count == 0:
                continue

            sampled_visible_count = len(sampled_visible_local_ix_set)
            if sampled_visible_count == 0:
                continue

            camera_has_inframe = True
            camera_denominator += sampled_visible_count
            camera_sampled_count += sampled_visible_count

            hit_mask = [False for _ in projected_track.frame_indexes]
            legacy_frame_best_score: list[float | None] = [
                None for _ in projected_track.frame_indexes
            ]

            visible_frame_min = min(visible_frame_to_local_ix.keys())
            visible_frame_max = max(visible_frame_to_local_ix.keys())

            for track_id_2d in alive_2d_ids:
                metadata_2d = tracking_2d_data_interface.trackingIDmetadata[
                    track_id_2d
                ]
                if (
                    metadata_2d.frame_ix_max < visible_frame_min
                    or metadata_2d.frame_ix_min > visible_frame_max
                ):
                    continue
                for (
                    frame_ix,
                    bbox2d,
                ) in tracking_2d_data_interface.trackingIDbboxlog.get(
                    track_id_2d, []
                ):
                    local_ix_3d = visible_frame_to_local_ix.get(frame_ix)
                    if local_ix_3d is None:
                        if should_trace_eval_frame(frame_ix):
                            append_trace_event(
                                eval_trace_by_camera,
                                camera_ix,
                                int(frame_ix),
                                {
                                    "stage": "match_2d_to_3d",
                                    "id3d": int(track_id_3d),
                                    "id2d": int(track_id_2d),
                                    "decision": "skip_no_visible_3d_frame",
                                    "bbox2d": np.array(
                                        bbox2d, dtype=np.float32
                                    ).tolist(),
                                },
                            )
                        continue
                    camera_has_timematch = True
                    common_frame_indexes.add(frame_ix)
                    if local_ix_3d not in sampled_visible_local_ix_set:
                        if should_trace_eval_frame(frame_ix):
                            append_trace_event(
                                eval_trace_by_camera,
                                camera_ix,
                                int(frame_ix),
                                {
                                    "stage": "sampling",
                                    "id3d": int(track_id_3d),
                                    "id2d": int(track_id_2d),
                                    "decision": "skip_by_stride",
                                    "local_ix_3d": int(local_ix_3d),
                                },
                            )
                        continue
                    comparison_candidate_count += 1
                    if hit_mask[local_ix_3d]:
                        if should_trace_eval_frame(frame_ix):
                            append_trace_event(
                                eval_trace_by_camera,
                                camera_ix,
                                int(frame_ix),
                                {
                                    "stage": "hit_mask",
                                    "id3d": int(track_id_3d),
                                    "id2d": int(track_id_2d),
                                    "decision": "skip_already_hit",
                                    "local_ix_3d": int(local_ix_3d),
                                },
                            )
                        continue
                    bbox3d_full = projected_track.bboxes3d[local_ix_3d]
                    projected_bbox2d = projected_track.projected_bboxes2d[
                        local_ix_3d
                    ]
                    assert projected_bbox2d is not None
                    bbox2d_array = np.array(bbox2d, dtype=np.float32)
                    if bbox_shrink_factor < 1.0:
                        bbox2d_array = shrink_bbox2d_fn(
                            bbox2d_array, bbox_shrink_factor
                        )
                    has_overlap = has_positive_2d_intersection_fn(
                        projected_bbox2d, bbox2d_array
                    )
                    if should_trace_eval_frame(frame_ix):
                        append_trace_event(
                            eval_trace_by_camera,
                            camera_ix,
                            int(frame_ix),
                            {
                                "stage": "overlap_gate",
                                "id3d": int(track_id_3d),
                                "id2d": int(track_id_2d),
                                "local_ix_3d": int(local_ix_3d),
                                "bbox3d": bbox3d_full.tolist(),
                                "bbox2d": bbox2d_array.tolist(),
                                "projected_bbox2d": projected_bbox2d.tolist(),
                                "has_overlap": bool(has_overlap),
                            },
                        )
                    if not has_overlap:
                        camera_has_valid = True
                        valid_comparison_count += 1
                        if legacy_frame_best_score[local_ix_3d] is None:
                            legacy_frame_best_score[local_ix_3d] = 0.0
                        if should_trace_eval_frame(frame_ix):
                            append_trace_event(
                                eval_trace_by_camera,
                                camera_ix,
                                int(frame_ix),
                                {
                                    "stage": "decision",
                                    "id3d": int(track_id_3d),
                                    "id2d": int(track_id_2d),
                                    "decision": "fast_reject_no_overlap",
                                },
                            )
                        continue
                    overlap_candidate_count += 1
                    if debug_enabled:
                        overlap_frame_ix_set = overlap_debug_by_camera[camera_ix][
                            "overlap_frame_ix_set"
                        ]
                        overlap_events = overlap_debug_by_camera[camera_ix][
                            "overlap_events"
                        ]
                        assert isinstance(overlap_frame_ix_set, set)
                        assert isinstance(overlap_events, list)
                        overlap_frame_ix_set.add(frame_ix)
                        overlap_events.append(
                            {
                                "frame_ix": int(frame_ix),
                                "id3d": int(track_id_3d),
                                "id2d": int(track_id_2d),
                                "bbox3d": bbox3d_full.tolist(),
                                "bbox2d": bbox2d_array.tolist(),
                                "bbox3d_projected_2d": projected_bbox2d.tolist(),
                            }
                        )
                    score = evaluate_bbox_overlap_scenedesc_fn(
                        bbox3d_full,
                        bbox2d_array,
                        camera_ix,
                    )
                    if should_trace_eval_frame(frame_ix):
                        append_trace_event(
                            eval_trace_by_camera,
                            camera_ix,
                            int(frame_ix),
                            {
                                "stage": "scenedesc_score",
                                "id3d": int(track_id_3d),
                                "id2d": int(track_id_2d),
                                "score": None if score is None else float(score),
                            },
                        )
                    if score is None:
                        continue
                    score_returned_count += 1
                    valid_comparison_count += 1
                    camera_has_valid = True
                    if legacy_frame_best_score[local_ix_3d] is None:
                        legacy_frame_best_score[local_ix_3d] = score
                    else:
                        legacy_frame_best_score[local_ix_3d] = max(
                            legacy_frame_best_score[local_ix_3d], score
                        )
                    if score > 0.0:
                        hit_mask[local_ix_3d] = True
                        if should_trace_eval_frame(frame_ix):
                            append_trace_event(
                                eval_trace_by_camera,
                                camera_ix,
                                int(frame_ix),
                                {
                                    "stage": "hit_mask",
                                    "id3d": int(track_id_3d),
                                    "id2d": int(track_id_2d),
                                    "decision": "set_hit_true",
                                },
                            )

            camera_numerator += sum(
                1
                for local_ix, is_hit in enumerate(hit_mask)
                if is_hit and local_ix in sampled_visible_local_ix_set
            )
            camera_legacy_numerator += sum(
                best_score
                for local_ix, best_score in enumerate(legacy_frame_best_score)
                if local_ix in sampled_visible_local_ix_set
                and best_score is not None
            )
            camera_legacy_denominator += sum(
                1
                for local_ix, best_score in enumerate(legacy_frame_best_score)
                if local_ix in sampled_visible_local_ix_set
                and best_score is not None
            )

        legacy_like_score = (
            camera_legacy_numerator / camera_legacy_denominator
            if camera_legacy_denominator > 0
            else 0.0
        )
        strict_hit_rate_score = (
            camera_numerator / camera_denominator if camera_denominator > 0 else 0.0
        )

        (
            mean_scores_per_camera[camera_ix],
            reason_camera_notvalid[camera_ix],
        ) = select_camera_evaluation_result(
            has_inframe_3d=camera_has_inframe,
            has_2d_tracking_frames=bool(tracked_2d_frame_indexes),
            has_time_match=camera_has_timematch,
            has_valid_comparison=camera_has_valid,
            strict_hit_rate=strict_hit_rate_score,
            legacy_like_score=legacy_like_score,
            use_legacy_like_metric=use_legacy_like_metric,
        )

        logger.info(
            f"evaluate_2d3d camera={camera_ix}: numerator={camera_numerator}, "
            f"denominator={camera_denominator}, "
            f"legacy_numerator={camera_legacy_numerator}, "
            f"legacy_denominator={camera_legacy_denominator}, "
            f"sampled_count={camera_sampled_count}, "
            f"strict_hit_rate={strict_hit_rate_score}, "
            f"legacy_like_score={legacy_like_score}, "
            f"score={mean_scores_per_camera[camera_ix]}, "
            f"use_legacy_like={use_legacy_like_metric}, "
            f"reason={reason_camera_notvalid[camera_ix]}",
        )
        evaluation_metric_debug.append(
            create_evaluation_metric_debug(
                numerator=camera_numerator,
                denominator=camera_denominator,
                strict_hit_rate=strict_hit_rate_score,
                legacy_numerator=camera_legacy_numerator,
                legacy_denominator=camera_legacy_denominator,
                legacy_like_score=legacy_like_score,
                selected_score=mean_scores_per_camera[camera_ix],
                use_legacy_like_metric=use_legacy_like_metric,
                visible_3d_frame_indexes=visible_3d_frame_indexes,
                tracked_2d_frame_indexes=tracked_2d_frame_indexes,
                common_frame_indexes=common_frame_indexes,
                tracked_3d_frame_indexes=tracked_3d_frame_indexes,
                comparison_candidate_count=comparison_candidate_count,
                overlap_candidate_count=overlap_candidate_count,
                score_returned_count=score_returned_count,
                valid_comparison_count=valid_comparison_count,
            )
        )
        logger.info(
            "evaluate_2d3d virtual bbox diagnostics "
            f"camera={camera_ix}: {virtual_bbox_debug_counts[camera_ix]}",
        )

    if debug_enabled:
        from argus_synchro.calibration_mat_generator_modules.ctrl.calibcheck2d3d.debug_artifacts import (
            create_evaluation_debug_summary,
        )
        debug_eval_info.update(
            create_evaluation_debug_summary(
                overlap_debug_by_camera,
                eval_trace_by_camera,
                trace_enabled=debug_eval_trace_enabled,
                trace_all_frames=debug_eval_trace_all_frames,
                trace_range_start=debug_eval_trace_range_start,
                trace_range_end=debug_eval_trace_range_end,
                trace_target_frames=debug_eval_trace_target_frames,
                video_paths=debug_video_paths,
                virtual_bbox_debug_counts=virtual_bbox_debug_counts,
            )
        )

    return mean_scores_per_camera, reason_camera_notvalid


def evaluate_2d3d(
    runtime: EvaluationRuntime,
    tracking_3d_data_interface: Tracking3dDataInterface,
    tracking_2d_data_interfaces: list[Tracking2dDataInterface],
    *,
    zvalues: tuple[float, float],
) -> tuple[list[float], list[int]]:
    return runtime.evaluate(
        tracking_3d_data_interface,
        tracking_2d_data_interfaces,
        zvalues=zvalues,
    )


def run_data_evaluation_process(
    *,
    camera_count: int,
    collect_bbox_log_observation: Callable[[], BBoxLogObservation],
    track_3dbbox: Callable[[], Tracking3dDataInterface],
    track_2dbbox: Callable[[], list[Tracking2dDataInterface]],
    collect_tracking_observation: Callable[
        [Tracking3dDataInterface, list[Tracking2dDataInterface]],
        TrackingObservation,
    ],
    evaluate: Callable[
        [Tracking3dDataInterface, list[Tracking2dDataInterface]],
        tuple[list[float], list[int]],
    ],
    get_evaluation_statistics: Callable[[], list[EvaluationMetricDebug]],
    judge: Callable[[list[float]], list[bool]],
    minimum_sample_count: int,
    judgement_threshold: float,
    save_debug_tracking: Callable[
        [Tracking3dDataInterface, list[Tracking2dDataInterface]], None
    ],
    save_debug_evaluation: Callable[
        [list[float], list[int], Tracking3dDataInterface, list[Tracking2dDataInterface]],
        None,
    ],
    logger: object,
    diagnosis: CalibCheck2d3dDiagnosis | None = None,
) -> list[CameraCalibCheckDiagnosisResult]:
    """ログ検証から最終判定までの評価段階を、既存の順序で実行する。"""
    # 初期は全部OK。無効になったカメラへ reason を入れ、後段評価で上書きしない。
    # 2/3 は bboxログ、4/5 は tracker、9/10/11 は 2D-3D 評価の理由を表す。
    # app_loopmainで蓄積した診断があれば同じセッションへ追記する。単体利用時は
    # ここでセッションを生成し、従来どおり収集後処理だけでも動作できるようにする。
    if diagnosis is not None:
        diagnosis_session = diagnosis
    else:
        diagnosis_session = CalibCheck2d3dDiagnosis(camera_count)

    def finalize_results(
        results: list[bool | None],
    ) -> list[CameraCalibCheckDiagnosisResult]:
        # 全findingから、カメラ別の代表理由、UI status、校正判定を確定する。
        return diagnosis_session.finalize(results)

    logger.info("collect_bbox_log_observation called")
    bbox_log_observation = collect_bbox_log_observation()
    camera_results = [False for _ in range(camera_count)]
    # 3Dログは全カメラ共通、2Dログはカメラ別として診断モジュールが
    # 件数・比率の閾値を評価し、reason 2/3と後続処理の評価可否を決める。
    finding_count_before_bbox_diagnosis = len(diagnosis_session.findings)
    bbox_log_validity = diagnosis_session.diagnose_bbox_logs(bbox_log_observation)
    # 正常時のログ量を増やさず、reason 2/3が新たに成立した場合だけ、
    # 従来ログ相当のフレーム数、比率、閾値を出力する。
    for finding in diagnosis_session.findings[finding_count_before_bbox_diagnosis:]:
        logger.info(
            "bbox log diagnosis failed: "
            f"reason={int(finding.reason)}, camera_index={finding.camera_index}, "
            f"details={dict(finding.details)}",
        )
    if not any(bbox_log_validity):
        return finalize_results(camera_results)

    logger.info("track_3dbbox called")
    tracking_3d = track_3dbbox()
    logger.info("track_2dbbox called")
    tracking_2d = track_2dbbox()
    logger.info("collect_tracking_observation called")
    tracking_observation = collect_tracking_observation(tracking_3d, tracking_2d)
    # bboxログと同様に、3D追跡の不成立(reason 4)は全カメラへ、
    # 2D追跡の不成立(reason 5)は該当カメラへ反映する。
    finding_count_before_tracking_diagnosis = len(diagnosis_session.findings)
    tracking_validity = diagnosis_session.diagnose_tracking(tracking_observation)
    # 正常時のログ量を増やさず、reason 4/5が成立した場合だけ件数と閾値を出す。
    for finding in diagnosis_session.findings[
        finding_count_before_tracking_diagnosis:
    ]:
        logger.info(
            "tracking diagnosis failed: "
            f"reason={int(finding.reason)}, camera_index={finding.camera_index}, "
            f"details={dict(finding.details)}",
        )
    can_evaluate_by_camera = [
        bbox_valid and tracking_valid
        for bbox_valid, tracking_valid in zip(
            bbox_log_validity, tracking_validity, strict=True
        )
    ]
    if not any(can_evaluate_by_camera):
        return finalize_results(camera_results)

    save_debug_tracking(tracking_3d, tracking_2d)
    evaluation_statistics, evaluation_reasons = evaluate(tracking_3d, tracking_2d)
    logger.info(
        f"evaluate_2d3d called, {evaluation_statistics = }, "
        f"{evaluation_reasons = }",
    )
    save_debug_evaluation(
        evaluation_statistics,
        evaluation_reasons,
        tracking_3d,
        tracking_2d,
    )
    normalized_evaluation_reasons = tuple(
        normalize_calibcheck_reason(reason) for reason in evaluation_reasons
    )
    # bboxログまたは追跡の段階で評価不能と判定済みのカメラには、
    # 後段の評価結果からreason 10/11を重ねて記録しない。これにより、
    # 人物未検出で2D追跡が0件のカメラはreason 9を代表理由として保てる。
    diagnosable_evaluation_reasons = tuple(
        reason if can_evaluate else CalibCheckFailureReason.NONE
        for reason, can_evaluate in zip(
            normalized_evaluation_reasons,
            can_evaluate_by_camera,
            strict=True,
        )
    )
    # evaluateが返すカメラ別reason 10/11と、評価値そのものの異常(reason 6)
    # を診断セッションへ渡す。整数reasonはここで正式なenumへ正規化する。
    finding_count_before_evaluation_diagnosis = len(diagnosis_session.findings)
    diagnosis_session.diagnose_evaluation(
        EvaluationObservation(
            scores=tuple(evaluation_statistics),
            reasons=diagnosable_evaluation_reasons,
            time_sync_statistics=tuple(
                EvaluationTimeSyncStatistics(
                    visible_3d_frame_count=metric["visible_3d_frame_count"],
                    tracked_2d_frame_count=metric["tracked_2d_frame_count"],
                    common_frame_count=metric["common_frame_count"],
                    visible_3d_frame_range=metric["visible_3d_frame_range"],
                    tracked_2d_frame_range=metric["tracked_2d_frame_range"],
                )
                for metric in get_evaluation_statistics()
            ),
            matching_statistics=tuple(
                EvaluationMatchingStatistics(
                    tracked_3d_frame_count=metric["tracked_3d_frame_count"],
                    visible_3d_frame_count=metric["visible_3d_frame_count"],
                    tracked_2d_frame_count=metric["tracked_2d_frame_count"],
                    common_frame_count=metric["common_frame_count"],
                    comparison_candidate_count=(
                        metric["comparison_candidate_count"]
                    ),
                    overlap_candidate_count=metric["overlap_candidate_count"],
                    score_returned_count=metric["score_returned_count"],
                    valid_comparison_count=metric["valid_comparison_count"],
                )
                for metric in get_evaluation_statistics()
            ),
        )
    )
    for finding in diagnosis_session.findings[
        finding_count_before_evaluation_diagnosis:
    ]:
        if finding.reason in {
            CalibCheckFailureReason.EVALUATION_RESULT_INVALID,
            CalibCheckFailureReason.SENSOR_TIME_SYNC_INVALID,
            CalibCheckFailureReason.MATCHED_PERSON_NOT_DETECTED,
        }:
            logger.info(
                "evaluation diagnosis failed: "
                f"reason={int(finding.reason)}, camera_index={finding.camera_index}, "
                f"details={dict(finding.details)}",
            )

    # 評価処理が保持した分子・分母を使い、選択中の評価式について標本数と
    # 内部整合性を確認する。reason 10/11のカメラは統計診断の対象外とする。
    metric_debug = get_evaluation_statistics()
    statistics = tuple(
        EvaluationStatistics(
            numerator=metric["numerator"],
            denominator=metric["denominator"],
            strict_hit_rate=metric["strict_hit_rate"],
            legacy_numerator=metric["legacy_numerator"],
            legacy_denominator=metric["legacy_denominator"],
            legacy_like_score=metric["legacy_like_score"],
            selected_score=metric["selected_score"],
            use_legacy_like_metric=metric["use_legacy_like_metric"],
        )
        for metric in metric_debug
    )
    finding_count_before_statistics_diagnosis = len(diagnosis_session.findings)
    diagnosis_session.diagnose_statistics(
        StatisticsObservation(
            statistics=statistics,
            evaluation_reasons=normalized_evaluation_reasons,
            minimum_sample_count=minimum_sample_count,
        )
    )
    for finding in diagnosis_session.findings[
        finding_count_before_statistics_diagnosis:
    ]:
        logger.info(
            "statistics diagnosis failed: "
            f"reason={int(finding.reason)}, camera_index={finding.camera_index}, "
            f"details={dict(finding.details)}",
        )
    evaluated_results = judge(evaluation_statistics)
    # 判定結果の型に加え、scoreと閾値から再計算した結果との一致も確認する。
    finding_count_before_judgement_diagnosis = len(diagnosis_session.findings)
    diagnosis_session.diagnose_judgement(
        JudgementObservation(
            results=tuple(evaluated_results),
            scores=tuple(evaluation_statistics),
            evaluation_reasons=normalized_evaluation_reasons,
            threshold=judgement_threshold,
        )
    )
    for finding in diagnosis_session.findings[
        finding_count_before_judgement_diagnosis:
    ]:
        logger.info(
            "judgement diagnosis failed: "
            f"reason={int(finding.reason)}, camera_index={finding.camera_index}, "
            f"details={dict(finding.details)}",
        )
    logger.info("judge_calibration_result called")
    # 従来と同じく、ログ・追跡段階で無効になったカメラは評価結果で上書きしない。
    for camera_ix, can_apply_evaluation in enumerate(can_evaluate_by_camera):
        if can_apply_evaluation:
            camera_results[camera_ix] = evaluated_results[camera_ix]
    return finalize_results(camera_results)
