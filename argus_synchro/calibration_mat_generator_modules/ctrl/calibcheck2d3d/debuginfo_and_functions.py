import os

import cv2
import matplotlib.pyplot as plt
import numpy as np
import open3d as o3d
from numpy.typing import NDArray
from sklearn.cluster import DBSCAN

from argus_synchro.calibration_mat_generator_modules.utils import utils3d
from argus_synchro.calibration_mat_generator_modules.utils.utils3d import (
    scale_transform,
    set_xyz_range,
)
from argus_synchro.common.app_logger import AppLogger, AppLoggerFactory


def conbine3d3d(xyz_data: list, trans_mat3D3D_eachlidar: list):
    for ix in range(len(trans_mat3D3D_eachlidar)):
        xyz_data[ix][:, :3] = (
            np.dot(xyz_data[ix][:, :3], trans_mat3D3D_eachlidar[ix][:3, :3].T)
            + trans_mat3D3D_eachlidar[ix][:3, 3]
        )
    return np.concatenate([xyz_data[0], xyz_data[1]], axis=0)


def internal_make_BB(
    pcd,
    x_range: tuple[float, float] = (-10, 10),
    y_range: tuple[float, float] = (-10, 10),
    z_range: tuple[float, float] = (-10, 10),
):
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


def read_rtvec_with_inputcheck(
    rvec_convmat_path: str = "",
    new_axis_mode: bool | None = None,
    points_inverted: bool = True,
):
    if os.path.isfile(rvec_convmat_path) is False:
        rvec_convmat_path = (
            input("rvec npy path or rotation_mat csv/txt(conversion matrix):")
            .replace('"', "")
            .strip()
        )

    if rvec_convmat_path.find(".csv") == -1 and rvec_convmat_path.find(".txt") == -1:
        tvec_path = input("tvec npy path:").replace('"', "").strip()
    else:
        tvec_path = None

    if new_axis_mode is None:
        new_axis_mode = input("New axis mode?(up-side-down) Y/n").lower() != "n"

    read_rtvec(
        rvec_convmat_path=rvec_convmat_path,
        new_axis_mode=new_axis_mode,
        points_inverted=points_inverted,
        tvec_path=tvec_path,
    )


def read_rtvec(
    rvec_convmat_path: str,
    new_axis_mode: bool,
    points_inverted: bool,
    tvec_path: str | None = None,
) -> tuple[NDArray[np.float32], NDArray[np.float32], NDArray[np.float32]]:
    # points_inverted=False(本来の逆): new_axis_mode==Trueの時反転
    # points_inverted=True(本来の形): new_axis_mode==Falseの時反転

    if os.path.isfile(rvec_convmat_path) is False:
        raise RuntimeError(f"{rvec_convmat_path} : 読み取り失敗")

    if tvec_path is not None:
        rvec = np.load(rvec_convmat_path, allow_pickle=True)
        tvec = np.load(tvec_path, allow_pickle=True)

    else:
        convmat = np.loadtxt(rvec_convmat_path, delimiter=",")
        rvec = cv2.Rodrigues(convmat[:3, :3])[0]
        tvec = convmat[:3, 3]

    invertflag = new_axis_mode ^ points_inverted

    convmat = np.eye(4)
    convmat[:3, 3] = tvec.reshape(3)
    convmat[:3, :3] = cv2.Rodrigues(rvec)[0]
    Rxinv = np.array(
        [
            [1.0, 0.0, 0.0, 0.0],
            [0.0, -1.0, 0.0, 0.0],
            [0.0, 0.0, -1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ]
    )  # X軸回りに反転。180°回転に関してはinvも同じ

    if invertflag:
        convmat = convmat @ Rxinv

    rvec = cv2.Rodrigues(convmat[:3, :3])[0]
    tvec = convmat[:3, 3]
    return rvec, tvec, convmat


def draw_singlebbox(frame, YOLOsingleresult_conv):
    # return self.yolo.draw_bboxes(frame,yoloresult)
    # yolo.draw_bboxesで呼び出しているmedia.draw_bboxesより変更

    # yolo_length = self.calc_yoloresult_length(yoloresult_whole)
    yolo_length = 1  # 後で柔軟に対応できるよう修正

    for _ in range(yolo_length):
        # YOLOsingleresult_conv = self.convert_bbox_imagecoordinate_info(yoloresult_whole, frame_ix, image_h, image_w)

        x1, x2, y1, y2, _, _ = np.array(YOLOsingleresult_conv, dtype=np.int32)
        prob = YOLOsingleresult_conv[5]

        cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 255, 0), 2)

        # Draw text box
        bbox_text = f"person: {prob:.1%}"
        t_size = cv2.getTextSize(bbox_text, 0, 1, 1)[0]
        cv2.rectangle(
            frame,
            (x1, y1),
            (x1 + t_size[0], y1 - t_size[1]),
            (255, 255, 0),
            -1,
        )
        # Draw text
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
    return [int(x) for x in itr]


def draw_multibbox(frame, yoloresult_whole, image_h: int = 1080, image_w: int = 1920):
    resultlist = []
    for result_ix in range(int(yoloresult_whole[3])):
        coor = yoloresult_whole[0].reshape((-1, 4))[result_ix]
        prob = yoloresult_whole[1].reshape((-1, 1))[result_ix]
        cls_id = yoloresult_whole[2].reshape((-1, 1))[result_ix]

        assert prob.size == 1
        assert cls_id.size == 1

        # メインアプリ core - utils.py - draw_bbox 関数より編集
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
        if self.vis is None:
            self._logger.info("*** vis open ***")
            self.vis = o3d.visualization.Visualizer()
            self.vis.create_window(
                width=500,  # 幅
                height=500,  # 高さ
                left=0,  # 表示位置(左)
                top=0,  # 表示位置(上)
                # width =3840,            # 幅
                # height=2160,            # 高さ
            )
            self.firsttimeflag = True

    def closevis(self):
        if self.vis is not None:
            self.vis.destroy_window()
        self.vis = None

    def add_obj(self, obj):
        self.showobj.append(obj)

    def get_color(self, colorref_single_vec):
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
        line_set = o3d.geometry.LineSet(
            points=o3d.utility.Vector3dVector(points),
            lines=o3d.utility.Vector2iVector(lines),
        )
        self.showobj.append(line_set)

    def add_coordinate_frame(self, size=5.0):
        self.showobj.append(o3d.geometry.TriangleMesh.create_coordinate_frame(size))

    def show(self, repeatcount=1, clear_data=False):
        """
        repeatcount <= 0で無限待機
        """
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
        self.openvis()
        self.vis.reset_view_point()
