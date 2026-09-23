import numpy as np
from numpy.typing import NDArray

from argus_synchro.calibration_mat_generator_modules.ctrl.calibcheck2d3d.tracker_recorder import (
    calibcheck2d_bboxtracker_recorder,
    calibcheck3d_bboxtracker_recorder,
)
from argus_synchro.calibration_mat_generator_modules.ctrl.calibration2d3d.track_main.interface_definition import (
    Tracking2dDataInterface,
)
from argus_synchro.calibration_mat_generator_modules.ctrl.calibration2d3d.track_main.interface_definition import (
    Tracking3dDataInterface,
)
from argus_synchro.calibration_mat_generator_modules.ctrl.calibcheck2d3d.processor import (
    TrackProximityWarning,
)
from argus_synchro.config.app_config_calibration import AppConfigCalibration
from argus_synchro.diagnosis.calibcheck2d3d_result_diagnosis import (
    BBoxLogObservation,
    TrackingObservation,
)


def record_bbox1f(
    frame_info: list[tuple[list[list[NDArray]], NDArray[np.float64]]],
    *,
    multi_minmax: NDArray[np.float64],
    yoloresult_whole_list: list[list[NDArray]],
    maximum_frame_count: int,
) -> None:
    # 評価は後段で全フレームを再走査するため、記録量は設定された末尾だけに制限する。
    if len(frame_info) >= maximum_frame_count:
        frame_info.pop(0)
    frame_info.append((yoloresult_whole_list, multi_minmax))


def collect_bbox_log_observation(
    frame_info: list[tuple[list[list[NDArray]], NDArray[np.float64]]],
    *,
    camera_count: int,
    minimum_3d_bbox_count_per_frame: int,
    minimum_3d_valid_frame_ratio: float,
    minimum_2d_bbox_count_per_frame: int,
    minimum_2d_valid_frame_ratio: float,
) -> BBoxLogObservation:
    """bboxログを走査し、reasonを含まない診断用統計を生成する。"""
    valid_3d_frame_count = 0
    valid_2d_frame_counts = [0 for _ in range(camera_count)]

    for yolo_results, bboxes_3d in frame_info:
        if bboxes_3d is not None and len(bboxes_3d) >= minimum_3d_bbox_count_per_frame:
            valid_3d_frame_count += 1

        # カメラ結果が不足しているフレームは、不足したカメラについて無効と数える。
        for camera_index in range(camera_count):
            if camera_index >= len(yolo_results):
                continue
            yolo_result = yolo_results[camera_index]
            if yolo_result is None or len(yolo_result) <= 3:
                continue
            try:
                detection_count = int(yolo_result[3])
            except (TypeError, ValueError):
                continue
            if detection_count >= minimum_2d_bbox_count_per_frame:
                valid_2d_frame_counts[camera_index] += 1

    return BBoxLogObservation(
        total_frame_count=len(frame_info),
        valid_3d_frame_count=valid_3d_frame_count,
        valid_2d_frame_counts=tuple(valid_2d_frame_counts),
        minimum_3d_bbox_count_per_frame=minimum_3d_bbox_count_per_frame,
        minimum_3d_valid_frame_ratio=minimum_3d_valid_frame_ratio,
        minimum_2d_bbox_count_per_frame=minimum_2d_bbox_count_per_frame,
        minimum_2d_valid_frame_ratio=minimum_2d_valid_frame_ratio,
    )


def track_3dbbox(
    frame_info: list[tuple[list[list[NDArray]], NDArray[np.float64]]],
    *,
    app_config_calib: AppConfigCalibration,
    camera_index: int,
) -> Tracking3dDataInterface:
    # 一旦全記録フレームを再走査する。リアルタイム追跡への変更は別途検討する。
    recorder = calibcheck3d_bboxtracker_recorder(
        app_config_calib=app_config_calib,
        camera_index=camera_index,
    )
    for frame_ix, (_, multi_minmax) in enumerate(frame_info):
        recorder.update(multi_minmax, frame_ix)
    return recorder.get_rawresults()


def track_2dbbox(
    frame_info: list[tuple[list[list[NDArray]], NDArray[np.float64]]],
    *,
    app_config_calib: AppConfigCalibration,
    image_size_hw: tuple[int, int],
    camera_count: int,
) -> list[Tracking2dDataInterface]:
    # 2D tracker はカメラごとに独立した recorder を持ち、同じ frame index で再走査する。
    recorders = [
        calibcheck2d_bboxtracker_recorder(
            app_config_calib=app_config_calib,
            image_size_hw=image_size_hw,
            camera_index=camera_index,
        )
        for camera_index in range(camera_count)
    ]
    for frame_ix, (yoloresult_whole_list, _) in enumerate(frame_info):
        for camera_index, yoloresult_whole in enumerate(yoloresult_whole_list):
            recorders[camera_index].update(yoloresult_whole, frame_ix)
    return [recorder.get_rawresults() for recorder in recorders]


def select_3dbbox_tracking_results(
    tracking_3d_data_interface: Tracking3dDataInterface,
    *,
    logger: object,
) -> Tracking3dDataInterface:
    # is_alive を False にして、追跡結果から除外する。
    for track_id, metadata in tracking_3d_data_interface.trackingIDmetadata.items():
        frame_ix_length = metadata.frame_ix_max - metadata.frame_ix_min
        accum_track_length = metadata.accum_track_length
        workarea_count = metadata.workarea_count
        logger.info(
            "select_3dbbox_tracking_results: "
            f"id={track_id}, frame_ix_length={frame_ix_length}, "
            f"accum_track_length={accum_track_length}, "
            f"workarea_count={workarea_count}",
        )
        if frame_ix_length < 30:
            metadata.is_alive = False
        if accum_track_length < 3.0:
            metadata.is_alive = False
        if workarea_count < frame_ix_length * 0.1 or workarea_count < 30:
            metadata.is_alive = False
    return tracking_3d_data_interface


def select_2dbbox_tracking_results(
    tracking_2d_data_interfaces: list[Tracking2dDataInterface],
    *,
    logger: object,
) -> list[Tracking2dDataInterface]:
    # is_alive を False にして、カメラごとの追跡結果から除外する。
    for camera_ix, tracking_2d_data_interface in enumerate(
        tracking_2d_data_interfaces
    ):
        for track_id, metadata in tracking_2d_data_interface.trackingIDmetadata.items():
            frame_ix_length = metadata.frame_ix_max - metadata.frame_ix_min
            accum_track_length = metadata.accum_track_length
            logger.info(
                "select_2dbbox_tracking_results: "
                f"camera_ix={camera_ix}, id={track_id}, "
                f"frame_ix_length={frame_ix_length}, "
                f"accum_track_length={accum_track_length}",
            )
            if frame_ix_length < 10:
                metadata.is_alive = False
            if accum_track_length < 50:
                metadata.is_alive = False
    return tracking_2d_data_interfaces


def collect_tracking_observation(
    tracking_3d_data_interface: Tracking3dDataInterface,
    tracking_2d_data_interfaces: list[Tracking2dDataInterface],
    *,
    camera_count: int,
    minimum_3d_alive_track_count: int,
    minimum_2d_alive_track_count: int,
    proximity_enabled: bool,
    proximity_center_distance_m: float,
    proximity_min_close_frames: int,
    proximity_min_close_ratio: float,
    proximity_warning_fails_validation: bool,
    logger: object,
) -> tuple[TrackingObservation, list[TrackProximityWarning]]:
    """追跡結果を選別し、reasonを含まない診断用統計を生成する。"""
    selected_3d = select_3dbbox_tracking_results(
        tracking_3d_data_interface,
        logger=logger,
    )
    proximity_warnings = detect_close_3dbbox_tracks(
        selected_3d,
        enabled=proximity_enabled,
        center_distance_m=proximity_center_distance_m,
        min_close_frames=proximity_min_close_frames,
        min_close_ratio=proximity_min_close_ratio,
    )
    if proximity_warnings:
        logger.warning(f"Close 3D tracks detected: {proximity_warnings}")
    selected_2d = select_2dbbox_tracking_results(
        tracking_2d_data_interfaces,
        logger=logger,
    )
    if len(selected_2d) != camera_count:
        raise ValueError("2D tracking result count must match camera_count")

    observation = TrackingObservation(
        total_3d_track_count=len(selected_3d.trackingIDmetadata),
        alive_3d_track_count=sum(
            metadata.is_alive
            for metadata in selected_3d.trackingIDmetadata.values()
        ),
        total_2d_track_counts=tuple(
            len(tracking.trackingIDmetadata) for tracking in selected_2d
        ),
        alive_2d_track_counts=tuple(
            sum(metadata.is_alive for metadata in tracking.trackingIDmetadata.values())
            for tracking in selected_2d
        ),
        minimum_3d_alive_track_count=minimum_3d_alive_track_count,
        minimum_2d_alive_track_count=minimum_2d_alive_track_count,
        proximity_warning_count=len(proximity_warnings),
        proximity_warning_fails_validation=proximity_warning_fails_validation,
    )
    return observation, proximity_warnings


def detect_close_3dbbox_tracks(
    tracking_3d_data_interface: Tracking3dDataInterface,
    *,
    enabled: bool,
    center_distance_m: float,
    min_close_frames: int,
    min_close_ratio: float,
) -> list[TrackProximityWarning]:
    # 同一フレームに出現する生存 3D track の中心距離から、重複追跡の疑いを検出する。
    if not enabled:
        return []

    alive_track_ids = [
        track_id
        for track_id, metadata in tracking_3d_data_interface.trackingIDmetadata.items()
        if metadata.is_alive
    ]
    if len(alive_track_ids) < 2:
        return []

    centers_by_track_id: dict[int, dict[int, tuple[float, float]]] = {}
    for track_id in alive_track_ids:
        frame_centers: dict[int, tuple[float, float]] = {}
        for frame_ix, bbox_xy in tracking_3d_data_interface.trackingIDbboxlog.get(
            track_id, []
        ):
            x1, y1, x2, y2 = bbox_xy
            frame_centers[int(frame_ix)] = (
                (float(x1) + float(x2)) / 2.0,
                (float(y1) + float(y2)) / 2.0,
            )
        if frame_centers:
            centers_by_track_id[track_id] = frame_centers

    warnings: list[TrackProximityWarning] = []
    for track_index, track_id_a in enumerate(alive_track_ids):
        centers_a = centers_by_track_id.get(track_id_a)
        if centers_a is None:
            continue

        for track_id_b in alive_track_ids[track_index + 1 :]:
            centers_b = centers_by_track_id.get(track_id_b)
            if centers_b is None:
                continue

            common_frame_ixs = sorted(set(centers_a.keys()) & set(centers_b.keys()))
            if not common_frame_ixs:
                continue

            close_frame_ixs: list[int] = []
            min_center_distance_m = float("inf")
            closest_frame_ix = common_frame_ixs[0]
            for frame_ix in common_frame_ixs:
                center_a = centers_a[frame_ix]
                center_b = centers_b[frame_ix]
                center_distance = float(
                    np.hypot(
                        center_a[0] - center_b[0],
                        center_a[1] - center_b[1],
                    )
                )
                if center_distance < min_center_distance_m:
                    min_center_distance_m = center_distance
                    closest_frame_ix = frame_ix
                if center_distance <= center_distance_m:
                    close_frame_ixs.append(frame_ix)

            close_frame_ratio = len(close_frame_ixs) / len(common_frame_ixs)
            if (
                len(close_frame_ixs) >= min_close_frames
                and close_frame_ratio >= min_close_ratio
            ):
                warnings.append(
                    {
                        "track_id_a": int(track_id_a),
                        "track_id_b": int(track_id_b),
                        "common_frame_count": len(common_frame_ixs),
                        "close_frame_count": len(close_frame_ixs),
                        "close_frame_ratio": float(close_frame_ratio),
                        "min_center_distance_m": float(min_center_distance_m),
                        "closest_frame_ix": int(closest_frame_ix),
                        "close_frame_samples": [
                            int(frame_ix) for frame_ix in close_frame_ixs[:10]
                        ],
                    }
                )

    return warnings
