from collections.abc import Callable
from dataclasses import dataclass
from typing import TypedDict

import numpy as np
from numpy.typing import NDArray


class VirtualBBoxDebugCounts(TypedDict):
    comparison_count: int
    virtual_bbox_win_count: int
    selected_candidate_counts: dict[str, int]


class EvaluationMetricDebug(TypedDict):
    numerator: int
    denominator: int
    strict_hit_rate: float
    legacy_numerator: float
    legacy_denominator: int
    legacy_like_score: float
    selected_score: float
    use_legacy_like_metric: bool
    visible_3d_frame_count: int
    tracked_2d_frame_count: int
    common_frame_count: int
    visible_3d_frame_range: tuple[int, int] | None
    tracked_2d_frame_range: tuple[int, int] | None
    tracked_3d_frame_count: int
    comparison_candidate_count: int
    overlap_candidate_count: int
    score_returned_count: int
    valid_comparison_count: int


class TrackProximityWarning(TypedDict):
    track_id_a: int
    track_id_b: int
    common_frame_count: int
    close_frame_count: int
    close_frame_ratio: float
    min_center_distance_m: float
    closest_frame_ix: int
    close_frame_samples: list[int]


@dataclass
class CalibCheckSessionState:
    frame_info: list[tuple[list[list[NDArray[np.float32]]], NDArray[np.float32]]]
    read_count: int
    debug_video_writers: list[object | None]
    debug_video_paths: list[str]
    debug_eval_info: dict[str, object]
    checked_points3d: list[object]
    checked_points3d_score: list[object]
    checked_points2d: list[object]
    checked_points2d_score: list[object]
    camera_scores_rawdata: list[list[float]]


def create_calibcheck_session_state(camera_count: int) -> CalibCheckSessionState:
    """評価開始時に蓄積・debug・カメラ別 score の状態を初期化する。"""
    return CalibCheckSessionState(
        frame_info=[],
        read_count=0,
        debug_video_writers=[None for _ in range(camera_count)],
        debug_video_paths=["" for _ in range(camera_count)],
        debug_eval_info={},
        checked_points3d=[],
        checked_points3d_score=[],
        checked_points2d=[],
        checked_points2d_score=[],
        camera_scores_rawdata=[[] for _ in range(camera_count)],
    )


def load_calibration_settings(
    *,
    lidar_calib_files: list[str],
    camera_calib_files: list[str],
    get_new_axis_mode: Callable[[], bool],
    read_rtvec: Callable[..., tuple[NDArray, NDArray, NDArray]],
    report_file_io_error: Callable[[str, str, Exception], None],
    logger: object,
) -> tuple[list[NDArray], list[tuple[NDArray, NDArray, NDArray]]]:
    """フレーム処理と最終評価の両方で使う変換行列を読み込む。"""
    lidar_transforms: list[NDArray] = []
    for file_path in lidar_calib_files:
        logger.info(f"[input_settings] path:{file_path}")
        try:
            transform = np.loadtxt(file_path, delimiter=",")
        except (OSError, UnicodeError, ValueError) as error:
            report_file_io_error(
                file_path, "read calibcheck2d3d LiDAR calibration CSV", error
            )
            raise
        logger.info(f"loadtxt: {transform}")
        lidar_transforms.append(transform)

    camera_rtvecs: list[tuple[NDArray, NDArray, NDArray]] = []
    for file_path in camera_calib_files:
        try:
            rtvec = read_rtvec(
                rvec_convmat_path=file_path,
                new_axis_mode=get_new_axis_mode(),
                points_inverted=True,
            )
        except (OSError, UnicodeError, ValueError, RuntimeError, EOFError) as error:
            report_file_io_error(
                file_path, "read calibcheck2d3d camera calibration", error
            )
            raise
        camera_rtvecs.append(rtvec)
    return lidar_transforms, camera_rtvecs


def apply_static_point_filter(
    pcdframe: NDArray,
    *,
    static_point_filter: object,
    timestamp_pcd: float,
    pointfilter_lastadd: float,
    enabled: bool,
    initlength: int,
    refresh_period: float,
    print_disabled: bool,
    logger: object,
) -> NDArray:
    """既存の校正モード用静的点フィルタのライフサイクルを適用する。

    `pointfilter_lastadd` は controller が所有しており、この処理経路では更新されない。
    そのため、この関数では意図的に読み取り専用として扱う。
    """
    if not enabled:
        return pcdframe
    if (
        static_point_filter.filtersource_framecount <= initlength
        or timestamp_pcd - pointfilter_lastadd > refresh_period
    ):
        static_point_filter.add_single_voxel_map(frame=pcdframe)
        if static_point_filter.filtersource_framecount >= initlength:
            static_point_filter.apply_voxelfilter()
        if not print_disabled:
            logger.info(
                "pointfilter add, "
                f"length: {static_point_filter.filtersource_framecount}",
            )
    if static_point_filter.filtersource_framecount >= initlength:
        return static_point_filter.extract_moving_objects(pcdframe)
    logger.info("static_point_filter - stacking points")
    return pcdframe


def make_empty_yoloresult() -> list[NDArray[np.float32]]:
    """検出がないカメラ向けに、4 配列からなる YOLO 出力形式を返す。"""
    return [
        np.zeros((0, 4), dtype=np.float32),
        np.zeros((0,), dtype=np.float32),
        np.zeros((0,), dtype=np.float32),
        np.array(0, dtype=np.float32),
    ]


def diagnose_input_data(
    camera_datalist: list[tuple[NDArray[np.uint8], int, float] | None],
    lidar_datalist: list[tuple[NDArray[np.float64], int, float] | None],
    can_data: object,
    *,
    shared_errors: object,
    invalid_data_input_index: object,
    array_shape_error_index: object,
    detection_result: object,
) -> bool:
    """UI/MMAP を更新する前に、このフレームを棄却すべきかを返す。

    最初の診断で各要素が None でないことを担保してから、画像と点群の shape を確認する。
    そのため後段では意図的に index 0 へアクセスする。
    """
    invalid_data_input = shared_errors.state_errors_D[invalid_data_input_index]
    result, failsafe_result = invalid_data_input.errors_diagnosis(
        (camera_datalist, lidar_datalist, can_data)
    )
    invalid_data_input.log_output(result, failsafe_result, invalid_data_input_index)
    if result == detection_result:
        return True

    images = tuple(camera_data[0] for camera_data in camera_datalist)
    min_xyz_columns = 3
    pcds_point_cloud = tuple(
        lidar_data[0][:, :min_xyz_columns] for lidar_data in lidar_datalist
    )
    array_shape_error = shared_errors.state_errors_D[array_shape_error_index]
    result, failsafe_result = array_shape_error.errors_diagnosis(
        ("images", images),
        ("pcds_point_cloud", pcds_point_cloud),
    )
    array_shape_error.log_output(result, failsafe_result, array_shape_error_index)
    return result == detection_result


def process_lidar_frame(
    lidar_datalist: list[tuple[NDArray[np.float64], int, float] | None],
    *,
    framecounter: int,
    read_point_cloud: Callable[..., tuple[NDArray, object]],
    apply_static_filter: Callable[..., NDArray],
    z_threshold: float,
    data_range_xyz: tuple[tuple[float, float], tuple[float, float], tuple[float, float]],
    voxel_downsample: Callable[[NDArray], NDArray],
    make_bounding_boxes: Callable[..., tuple[tuple[NDArray, NDArray, NDArray], NDArray, object]],
    publish_lidar_data: Callable[[NDArray, NDArray, NDArray], None],
    logger: object,
) -> NDArray:
    """1 フレームの LiDAR 点群から 3D bbox の min/max 配列を作る。

    点群の Y/Z 反転、静的物体除去、高さでの絞り込み、voxel downsample、クラスタリング
    の順序は既存の校正診断アルゴリズムに合わせる。MMAP 向けの点群・bbox 更新は callback
    に分離し、計算結果と同じデータを facade が公開する。
    """
    pcd, _ts_lidar_raw = read_point_cloud(
        lidar_datalist, dontread=False, adjust_coordinate_enable=False
    )
    logger.info(f"proc_lidar1f: pcd.shape={pcd.shape}, framecounter={framecounter}")

    pcd[:, 1] = -pcd[:, 1]
    pcd[:, 2] = -pcd[:, 2]
    points = apply_static_filter(pcdframe=pcd, timestamp_pcd=framecounter)
    points = points[points[:, 2] < -z_threshold]

    points_object = voxel_downsample(points[:, :3])
    (multi_points, multi_lines, multi_minmax), pcd_limited, _db = make_bounding_boxes(
        points_object,
        x_range=data_range_xyz[0],
        y_range=data_range_xyz[1],
        z_range=data_range_xyz[2],
    )
    logger.info(
        "proc_lidar1f: "
        f"pcd.shape={pcd.shape}, points.shape={points.shape}, "
        f"points_object.shape={points_object.shape}, "
        f"pcd_limited.shape={pcd_limited.shape}, "
        f"multi_points.shape={multi_points.shape}, "
        f"multi_lines.shape={multi_lines.shape}, "
        f"multi_minmax.shape={multi_minmax.shape}, "
        f"framecounter={framecounter}"
    )
    publish_lidar_data(pcd_limited, multi_points, multi_lines)
    return multi_minmax.reshape(-1, 6)


def process_camera_frame(
    camera_datalist: list[tuple[NDArray[np.uint8], int, float] | None],
    *,
    camera_count: int,
    undistort_image: Callable[[NDArray[np.uint8]], NDArray[np.uint8]],
    predict_batch: Callable[..., list[list[NDArray[np.float32]]]],
    sec: object,
    publish_image: Callable[[int, NDArray[np.uint8]], None],
    publish_detection_data: Callable[
        [list[NDArray[np.uint8] | None], list[list[NDArray[np.float32]]]], None
    ],
    logger: object,
) -> list[list[NDArray[np.float32]]]:
    """1 フレーム分の画像を補正して YOLO 推論し、UI 公開用の結果を渡す。

    無効なカメラ入力は空のまま YOLO adapter に渡す。補正画像は推論前に UI へ公開し、
    bbox 座標は推論後に公開することで、既存の MMAP 更新順を維持する。
    """
    frame_cameras: list[NDArray[np.uint8] | None] = [None] * camera_count
    for camera_ix, camera_data in enumerate(camera_datalist):
        if camera_data is None:
            logger.info(f"frame {camera_ix} is invalid, skip")
            continue
        if camera_ix >= camera_count:
            logger.warning(
                f"camera index {camera_ix} is out of configured range "
                f"{camera_count}, skip",
            )
            continue
        frame = undistort_image(camera_data[0])
        frame_cameras[camera_ix] = frame
        publish_image(camera_ix, frame)

    yoloresult_whole_list = predict_batch(sec=sec, frames=frame_cameras)
    for camera_ix in range(camera_count):
        detected_count = int(yoloresult_whole_list[camera_ix][3])
        logger.info(
            f"YOLO detection results: {detected_count} objects detected "
            f"in camera {camera_ix}",
        )
    publish_detection_data(frame_cameras, yoloresult_whole_list)
    return yoloresult_whole_list


def yolo_result_to_ui_bboxes(
    yoloresult_whole: list[NDArray[np.float32]],
    *,
    image_height: int,
    image_width: int,
) -> NDArray[np.int32]:
    """YOLO の [ymin, xmin, ymax, xmax] 正規化座標を UI 用ピクセル座標へ変換する。"""
    bbox_for_ui: list[list[int]] = []
    for result_ix in range(int(yoloresult_whole[3])):
        coordinate = yoloresult_whole[0].reshape((-1, 4))[result_ix]
        bbox_for_ui.append(
            [
                int(coordinate[1] * image_width),
                int(coordinate[0] * image_height),
                int(coordinate[3] * image_width),
                int(coordinate[2] * image_height),
            ]
        )
    return np.array(bbox_for_ui, dtype=np.int32).reshape(-1, 4)


def lidar_points_to_ui_data(
    pcd_limited: NDArray[np.float32],
    *,
    intensity_to_color: Callable[[NDArray], NDArray],
) -> tuple[NDArray, NDArray]:
    """LiDAR 点群を UI/MMAP 公開用の座標・色情報へ変換する。

    intensity 列がある点群は既存の色変換を使い、XYZ のみなら固定色、想定外の
    次元なら空配列を返す。
    """
    if pcd_limited.shape[-1] == 4:
        return pcd_limited[:, :3], intensity_to_color(pcd_limited[:, 3])
    if pcd_limited.shape[-1] == 3:
        return pcd_limited, np.tile([0.2, 0.2, 0.2], (pcd_limited.shape[0], 1))
    return np.zeros((0, 3), dtype=np.float32), np.zeros((0, 3), dtype=np.float32)


def publish_yolo_bboxes_to_ui(
    frame_cameras: list[NDArray[np.uint8] | None],
    yoloresult_whole_list: list[list[NDArray[np.float32]]],
    *,
    draw_bboxes: Callable[
        [NDArray[np.uint8], list[NDArray[np.float32]]], tuple[NDArray[np.uint8], object]
    ],
    set_bboxes: Callable[[int, NDArray[np.int32]], None],
) -> None:
    """カメラ別 YOLO 結果を debug 描画し、UI/MMAP 用の bbox 座標として公開する。"""
    for camera_ix, frame in enumerate(frame_cameras):
        if frame is None:
            set_bboxes(camera_ix, np.zeros((0, 4), dtype=np.int32))
            continue
        yoloresult_whole = yoloresult_whole_list[camera_ix]
        frame, _ = draw_bboxes(frame, yoloresult_whole)
        image_h, image_w, _ = frame.shape
        set_bboxes(
            camera_ix,
            yolo_result_to_ui_bboxes(
                yoloresult_whole,
                image_height=image_h,
                image_width=image_w,
            ),
        )


def process_calibcheck_frame(
    fifo_data: tuple[object, object, object, int],
    *,
    diagnose_input: Callable[[object, object, object], bool],
    publish_yaw: Callable[[float], None],
    process_lidar: Callable[[object, int], NDArray[np.float64]],
    process_camera: Callable[[object, int], list[list[NDArray[np.float32]]]],
    draw_evaluation_bboxes: Callable[[NDArray[np.float64]], None],
    record_bboxes: Callable[[NDArray[np.float64], list[list[NDArray[np.float32]]]], None],
    write_debug_video: Callable[[int], None],
    transmit_frame: Callable[[int], None],
    logger: object,
) -> bool:
    """通常の 1 フレーム処理を既存の UI/MMAP 公開順で実行する。"""
    camera_datalist, lidar_datalist, can_data, framecounter = fifo_data
    if diagnose_input(camera_datalist, lidar_datalist, can_data):
        return False
    publish_yaw(can_data[0])
    logger.info(process_lidar.__qualname__ + " called")
    multi_minmax = process_lidar(lidar_datalist, framecounter)
    logger.info(process_camera.__qualname__ + " called")
    yoloresult_whole_list = process_camera(camera_datalist, framecounter)
    draw_evaluation_bboxes(multi_minmax)
    record_bboxes(multi_minmax, yoloresult_whole_list)
    write_debug_video(framecounter)
    transmit_frame(framecounter)
    return True
