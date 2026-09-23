import pickle

import cv2
import numpy as np
from numpy.typing import NDArray

from argus_synchro.calibration_mat_generator_modules.ctrl.calibration2d3d.track_main.interface_definition import (
    Tracking2dDataInterface,
    Tracking3dDataInterface,
)
from argus_synchro.calibration_mat_generator_modules.ctrl.calibcheck2d3d.processor import (
    TrackProximityWarning,
)


def dump_pickle(path: str, value: object) -> None:
    """評価判定に関与しないデバッグ artifact を pickle として保存する。"""
    with open(path, "wb") as output_file:
        pickle.dump(value, output_file)


def should_trace_eval_frame(
    frame_ix: int,
    *,
    debug_enabled: bool,
    trace_enabled: bool,
    trace_all_frames: bool,
    trace_range_start: int,
    trace_range_end: int,
    trace_target_frames: set[int],
) -> bool:
    if not debug_enabled or not trace_enabled:
        return False
    if trace_all_frames:
        return True
    if trace_range_start <= frame_ix <= trace_range_end:
        return True
    return frame_ix in trace_target_frames


def append_trace_event(
    eval_trace_by_camera: dict[int, dict[int, list[dict[str, object]]]],
    *,
    camera_ix: int,
    frame_ix: int,
    event: dict[str, object],
) -> None:
    eval_trace_by_camera.setdefault(camera_ix, {}).setdefault(frame_ix, []).append(
        event
    )


def ensure_video_writer(
    camera_ix: int,
    frame_bgr: NDArray[np.uint8],
    *,
    writers: list[object | None],
    paths: list[str],
    prefix: str,
    fps: float,
    logger: object,
) -> None:
    """カメラごとの debug 動画 writer を初回だけ開く。"""
    if writers[camera_ix] is not None:
        return
    height, width = frame_bgr.shape[:2]
    output_path = f"{prefix}{camera_ix}calibcheck.mp4"
    writer = cv2.VideoWriter(
        output_path,
        cv2.VideoWriter_fourcc(*"mp4v"),
        fps,
        (width, height),
    )
    if not writer.isOpened():
        logger.warning(
            f"Debug video writer open failed: camera={camera_ix}, path={output_path}",
        )
        return
    writers[camera_ix] = writer
    paths[camera_ix] = output_path
    logger.info(f"Debug video writer opened: camera={camera_ix}, path={output_path}")


def write_video_frames(
    frames: list[NDArray[np.uint8] | None],
    *,
    frame_ix: int,
    writers: list[object | None],
    paths: list[str],
    prefix: str,
    fps: float,
    add_frame_text: bool,
    logger: object,
) -> None:
    """MMAP 送信前の表示用画像をコピーして debug 動画へ記録する。"""
    for camera_ix, frame in enumerate(frames):
        if frame is None:
            continue
        ensure_video_writer(
            camera_ix,
            frame,
            writers=writers,
            paths=paths,
            prefix=prefix,
            fps=fps,
            logger=logger,
        )
        writer = writers[camera_ix]
        if writer is None:
            continue
        output_frame = frame.copy()
        if add_frame_text:
            cv2.putText(
                output_frame,
                f"frame={frame_ix}",
                (20, 40),
                cv2.FONT_HERSHEY_SIMPLEX,
                1.0,
                (0, 255, 255),
                2,
                cv2.LINE_AA,
            )
        writer.write(output_frame)


def close_video_writers(
    writers: list[object | None], paths: list[str], *, logger: object
) -> None:
    """開いている debug 動画 writer を終了し、次回用に空へ戻す。"""
    for camera_ix, writer in enumerate(writers):
        if writer is None:
            continue
        writer.release()
        logger.info(
            f"Debug video writer closed: camera={camera_ix}, path={paths[camera_ix]}",
        )
        writers[camera_ix] = None


def append_evaluation_debug_info(
    debug_eval_info: dict[str, object],
    *,
    evaluation_statistics: list[float],
    evaluation_reasons: list[int],
    camera_calib_files: list[object],
    camera_intrinsics_path: object,
    image_width: int,
    image_height: int,
    eval_zvalues: tuple[float, float],
    new_axis_mode: bool,
    tracking_3d_data_interface: Tracking3dDataInterface,
    tracking_2d_data_interfaces: list[Tracking2dDataInterface],
    proximity_warnings: list[TrackProximityWarning],
) -> None:
    """評価後の追跡・設定情報を、後で解析できる debug artifact へ追記する。"""
    debug_eval_info["evaluation_statistics"] = evaluation_statistics
    debug_eval_info["reason_camera_notvalid"] = evaluation_reasons
    debug_eval_info["camera_calib_files"] = [
        str(file_path) for file_path in camera_calib_files
    ]
    debug_eval_info["camera_intrinsics_path"] = str(camera_intrinsics_path)
    debug_eval_info["image_size"] = {"width": image_width, "height": image_height}
    debug_eval_info["eval_zvalues"] = [float(eval_zvalues[0]), float(eval_zvalues[1])]
    debug_eval_info["new_axis_mode"] = new_axis_mode
    debug_eval_info["tracking_3d_bboxlog"] = {
        int(track_id): [
            (int(frame_ix), [float(value) for value in bbox2d])
            for frame_ix, bbox2d in bboxlog
        ]
        for track_id, bboxlog in tracking_3d_data_interface.trackingIDbboxlog.items()
    }
    debug_eval_info["tracking_3d_metadata"] = {
        int(track_id): {
            "is_alive": bool(metadata.is_alive),
            "frame_ix_min": int(metadata.frame_ix_min),
            "frame_ix_max": int(metadata.frame_ix_max),
            "frame_ix_lastmove": int(metadata.frame_ix_lastmove),
            "accum_track_length": float(metadata.accum_track_length),
            "workarea_count": float(metadata.workarea_count),
        }
        for track_id, metadata in tracking_3d_data_interface.trackingIDmetadata.items()
    }
    debug_eval_info["tracking_3d_proximity_warnings"] = proximity_warnings
    debug_eval_info["tracking_2d_bboxlog"] = {
        int(camera_ix): {
            int(track_id): [
                (int(frame_ix), [float(value) for value in bbox2d])
                for frame_ix, bbox2d in bboxlog
            ]
            for track_id, bboxlog in tracking_2d_data_interface.trackingIDbboxlog.items()
        }
        for camera_ix, tracking_2d_data_interface in enumerate(
            tracking_2d_data_interfaces
        )
    }


def create_evaluation_debug_summary(
    overlap_debug_by_camera: dict[int, dict[str, object]],
    eval_trace_by_camera: dict[int, dict[int, list[dict[str, object]]]],
    *,
    trace_enabled: bool,
    trace_all_frames: bool,
    trace_range_start: int,
    trace_range_end: int,
    trace_target_frames: set[int],
    video_paths: list[str],
    virtual_bbox_debug_counts: object,
) -> dict[str, object]:
    """評価 trace と overlap 情報を pickle 可能な debug summary に変換する。"""
    serializable_overlap: dict[int, dict[str, object]] = {}
    for camera_ix, camera_debug in overlap_debug_by_camera.items():
        overlap_frame_ix_set = camera_debug["overlap_frame_ix_set"]
        assert isinstance(overlap_frame_ix_set, set)
        serializable_overlap[camera_ix] = {
            "overlap_frame_ix_list": sorted(overlap_frame_ix_set),
            "overlap_events": camera_debug["overlap_events"],
        }
    return {
        "overlap_debug_by_camera": serializable_overlap,
        "eval_trace_by_camera": eval_trace_by_camera,
        "eval_trace_config": {
            "enabled": trace_enabled,
            "all_frames": trace_all_frames,
            "range_start": int(trace_range_start),
            "range_end": int(trace_range_end),
            "target_frames": sorted(trace_target_frames),
        },
        "video_paths": video_paths,
        "virtual_bbox_debug_counts": virtual_bbox_debug_counts,
    }


def draw_projected_bbox_edges(
    frame: NDArray[np.uint8],
    vertices3d: NDArray[np.float32],
    *,
    rvec: NDArray,
    tvec: NDArray,
    camera_intrinsics: NDArray,
    color: tuple[int, int, int],
    edge_pairs: list[tuple[int, int]],
) -> None:
    """デバッグ用に 3D bbox の 8 頂点を画像へ投影し、辺を描画する。"""
    vertices3d_reproj = cv2.projectPoints(
        vertices3d,
        rvec,
        tvec,
        camera_intrinsics,
        np.zeros((5, 1), dtype=np.float32),
    )[0].reshape(-1, 2)
    for point1_ix, point2_ix in edge_pairs:
        cv2.line(
            frame,
            [int(value) for value in vertices3d_reproj[point1_ix]],
            [int(value) for value in vertices3d_reproj[point2_ix]],
            color,
            2,
        )


def make_bbox3d_vertices(
    x_min: float,
    x_max: float,
    y_min: float,
    y_max: float,
    z_min: float,
    z_max: float,
) -> NDArray[np.float32]:
    """debug 描画用に min/max 形式の 3D bbox を既存の 8 頂点順へ変換する。"""
    return np.array(
        [
            [x_min, y_min, z_min],
            [x_max, y_min, z_min],
            [x_min, y_max, z_min],
            [x_max, y_max, z_min],
            [x_min, y_min, z_max],
            [x_max, y_min, z_max],
            [x_min, y_max, z_max],
            [x_max, y_max, z_max],
        ],
        dtype=np.float32,
    )


def draw_evaluation_bboxes(
    frames: list[NDArray[np.uint8] | None],
    multi_minmax: NDArray[np.float64],
    *,
    camera_rtvecs: list[tuple[NDArray, NDArray, NDArray]],
    camera_intrinsics: NDArray,
    eval_zvalues: tuple[float, float],
    virtual_bbox_xy_offsets: tuple[tuple[float, float], ...],
    project_bbox: object,
) -> None:
    """開発用に実測・評価用・仮想 3D bbox をカメラ画像へ重畳する。

    製品 UI の描画に使う情報は別途 MMAP 経由で渡す。この関数は core 側での
    デバッグ表示だけを更新し、評価値や bbox 記録には関与しない。
    """
    edge_pairs = [
        (0, 1),
        (1, 3),
        (3, 2),
        (2, 0),
        (4, 5),
        (5, 7),
        (7, 6),
        (6, 4),
        (0, 4),
    ]
    eval_zmin, eval_zmax = eval_zvalues
    for camera_ix, frame in enumerate(frames):
        if frame is None:
            continue
        rvec, tvec, _camera_extrinsics = camera_rtvecs[camera_ix]
        for bbox3d in multi_minmax:
            vertices3d = make_bbox3d_vertices(
                bbox3d[0], bbox3d[1], bbox3d[2], bbox3d[3], bbox3d[4], bbox3d[5]
            )
            projected_bbox = project_bbox(
                np.array(
                    [
                        bbox3d[0], bbox3d[2], bbox3d[4],
                        bbox3d[1], bbox3d[3], bbox3d[5],
                    ],
                    dtype=np.float32,
                ),
                camera_ix,
            )
            if projected_bbox is not None:
                draw_projected_bbox_edges(
                    frame,
                    vertices3d,
                    rvec=rvec,
                    tvec=tvec,
                    camera_intrinsics=camera_intrinsics,
                    color=(0, 255, 0),
                    edge_pairs=edge_pairs,
                )

            vertices3d_eval = make_bbox3d_vertices(
                bbox3d[0], bbox3d[1], bbox3d[2], bbox3d[3], eval_zmin, eval_zmax
            )
            projected_eval_bbox = project_bbox(
                np.array(
                    [
                        bbox3d[0], bbox3d[2], eval_zmin,
                        bbox3d[1], bbox3d[3], eval_zmax,
                    ],
                    dtype=np.float32,
                ),
                camera_ix,
            )
            if projected_eval_bbox is not None:
                draw_projected_bbox_edges(
                    frame,
                    vertices3d_eval,
                    rvec=rvec,
                    tvec=tvec,
                    camera_intrinsics=camera_intrinsics,
                    color=(0, 165, 255),
                    edge_pairs=edge_pairs,
                )

            for offset_x, offset_y in virtual_bbox_xy_offsets:
                if offset_x == 0.0 and offset_y == 0.0:
                    continue
                virtual_x_min = bbox3d[0] + offset_x
                virtual_x_max = bbox3d[1] + offset_x
                virtual_y_min = bbox3d[2] + offset_y
                virtual_y_max = bbox3d[3] + offset_y
                projected_virtual_bbox = project_bbox(
                    np.array(
                        [
                            virtual_x_min, virtual_y_min, eval_zmin,
                            virtual_x_max, virtual_y_max, eval_zmax,
                        ],
                        dtype=np.float32,
                    ),
                    camera_ix,
                )
                if projected_virtual_bbox is None:
                    continue
                draw_projected_bbox_edges(
                    frame,
                    make_bbox3d_vertices(
                        virtual_x_min,
                        virtual_x_max,
                        virtual_y_min,
                        virtual_y_max,
                        eval_zmin,
                        eval_zmax,
                    ),
                    rvec=rvec,
                    tvec=tvec,
                    camera_intrinsics=camera_intrinsics,
                    color=(255, 255, 0),
                    edge_pairs=edge_pairs,
                )


# ===== 描画・可視化ユーティリティ（debuginfo_and_functions.py から統合）=====

import matplotlib.pyplot as plt
import open3d as o3d
from sklearn.cluster import DBSCAN

from argus_synchro.calibration_mat_generator_modules.utils import utils3d
from argus_synchro.calibration_mat_generator_modules.utils.utils3d import (
    scale_transform,
    set_xyz_range,
)
from argus_synchro.common.app_logger import AppLogger, AppLoggerFactory


def internal_make_BB(
    pcd,
    x_range: tuple[float, float] = (-10, 10),
    y_range: tuple[float, float] = (-10, 10),
    z_range: tuple[float, float] = (-10, 10),
):
    """3D点群からBounding Boxを生成する。DBSCAN クラスタリングを使用。"""
    data_array = set_xyz_range(
        pcd_data=pcd,
        x_range=x_range,
        y_range=y_range,
        z_range=z_range,
    )

    if data_array.shape[0] < 10:
        return (
            (np.zeros((0, 3)), np.zeros((0, 3)), np.zeros((0, 3))),
            data_array,
            None,
        )

    db = DBSCAN(eps=0.5, min_samples=10).fit(data_array)
    labels = db.labels_
    return utils3d.bounding_box(data_array, np.unique(labels), labels), data_array, db


def draw_singlebbox(frame, YOLOsingleresult_conv):
    """単一の YOLO bbox を画像に描画する。"""
    yolo_length = 1

    for _ in range(yolo_length):
        x1, x2, y1, y2, _, _ = np.array(YOLOsingleresult_conv, dtype=np.int32)
        prob = YOLOsingleresult_conv[5]

        cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 255, 0), 2)

        bbox_text = f"person: {prob:.1%}"
        t_size = cv2.getTextSize(bbox_text, 0, 1, 1)[0]
        cv2.rectangle(
            frame,
            (x1, y1),
            (x1 + t_size[0], y1 - t_size[1]),
            (255, 255, 0),
            -1,
        )
        cv2.putText(
            frame,
            bbox_text,
            (x1, y1),
            cv2.FONT_HERSHEY_SIMPLEX,
            1,
            (255, 255, 255),
            1,
            lineType=cv2.LINE_AA,
        )

    return frame


def conv_intarr(itr):
    """イテレータの各要素を int に変換。"""
    return [int(x) for x in itr]


def draw_multibbox(frame, yoloresult_whole, image_h: int = 1080, image_w: int = 1920):
    """複数の YOLO bbox を画像に描画する。"""
    resultlist = []
    for result_ix in range(int(yoloresult_whole[3])):
        coor = yoloresult_whole[0].reshape((-1, 4))[result_ix]
        prob = yoloresult_whole[1].reshape((-1, 1))[result_ix]
        cls_id = yoloresult_whole[2].reshape((-1, 1))[result_ix]

        assert prob.size == 1
        assert cls_id.size == 1

        bbox_ymin = coor[0] * image_h
        bbox_ymax = coor[2] * image_h
        bbox_xmin = coor[1] * image_w
        bbox_xmax = coor[3] * image_w
        single_result = np.array(
            [
                float(bbox_xmin),
                float(bbox_xmax),
                float(bbox_ymin),
                float(bbox_ymax),
                float(cls_id[0]),
                float(prob[0]),
            ]
        )
        resultlist.append(single_result)

        frame = draw_singlebbox(frame, single_result)
    return frame, resultlist


class vis_viewer:
    """Open3D を使用した 3D可視化ビューア。"""

    def __init__(
        self,
        app_logger_factory: AppLoggerFactory,
    ) -> None:
        self._logger: AppLogger = app_logger_factory.register_from_type(self.__class__)
        self.vis = None
        self.showobj = []

    def __delattr__(self, name: str) -> None:
        self.closevis()

    def openvis(self):
        """可視化ウィンドウを開く。"""
        if self.vis is None:
            self._logger.info("*** vis open ***")
            self.vis = o3d.visualization.Visualizer()
            self.vis.create_window(
                width=500,
                height=500,
                left=0,
                top=0,
            )
            self.firsttimeflag = True

    def closevis(self):
        """可視化ウィンドウを閉じる。"""
        if self.vis is not None:
            self.vis.destroy_window()
        self.vis = None

    def add_obj(self, obj):
        """オブジェクトを表示対象に追加。"""
        self.showobj.append(obj)

    def get_color(self, colorref_single_vec):
        """スカラー値を色にマップ。"""
        if colorref_single_vec.size == 0:
            return np.zeros((0, 3))
        cmap_plt = plt.get_cmap("bwr")
        return cmap_plt(scale_transform(colorref_single_vec, val_min=0, val_max=1))[
            :, :3
        ]

    def add_points(
        self,
        points: np.ndarray | None,
        colors: np.ndarray | None = None,
        downsample_size: int | None = None,
    ):
        """点群を追加。"""
        points_o3d = o3d.geometry.PointCloud()
        if points is None or points.size == 0:
            return
        try:
            points_o3d.points = o3d.utility.Vector3dVector(points)
        except Exception as e:
            self._logger.info(
                f"points - Exception: {e}, exception type: {type(e)}, data type: {type(points)}",
            )
            self._logger.info(f"data shape: {points.shape}")
            self._logger.info(f"data contents: {points}")
            raise e
        if colors is not None:
            try:
                points_o3d.colors = o3d.utility.Vector3dVector(colors)
            except Exception as e:
                self._logger.info(
                    f"colors - Exception: {e}, exception type: {type(e)}, data type: {type(colors)}",
                )
                self._logger.info(f"data shape: {colors.shape}")
                self._logger.info(f"data contents: {colors}")
                raise e
        if downsample_size is not None:
            points_o3d.voxel_down_sample(downsample_size)
        self.showobj.append(points_o3d)

    def add_lineset(self, points, lines):
        """線セットを追加。"""
        line_set = o3d.geometry.LineSet(
            points=o3d.utility.Vector3dVector(points),
            lines=o3d.utility.Vector2iVector(lines),
        )
        self.showobj.append(line_set)

    def add_coordinate_frame(self, size=5.0):
        """座標軸フレームを追加。"""
        self.showobj.append(o3d.geometry.TriangleMesh.create_coordinate_frame(size))

    def show(self, repeatcount=1, clear_data=False):
        """可視化を表示。repeatcount <= 0で無限待機。"""
        self.openvis()

        self.vis.clear_geometries()
        if self.firsttimeflag:
            self.firsttimeflag = False
            for obj in self.showobj:
                self.vis.add_geometry(obj)
        else:
            for obj in self.showobj:
                self.vis.add_geometry(obj, reset_bounding_box=False)

        if repeatcount > 0:
            for _ in range(repeatcount):
                keep_running = self.vis.poll_events()
                self.vis.update_renderer()
                if not keep_running:
                    break
        else:
            self.vis.run()

        if clear_data:
            self.showobj = []

    def reset_view(self):
        """ビュー角度をリセット。"""
        self.openvis()
        self.vis.reset_view_point()