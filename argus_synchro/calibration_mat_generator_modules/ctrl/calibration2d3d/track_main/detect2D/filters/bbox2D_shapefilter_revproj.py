import json

import cv2
import numpy as np
from numpy.typing import NDArray

from argus_synchro.calibration_mat_generator_modules.ctrl.calibration2d3d.track_main.detect2D.filters.rev_projection import (
    intersect_pixel_ray_with_world_plane,
)
from argus_synchro.config.app_config_calibration import AppConfigCalibration


def make_singlepoint_personshape(
    person_height: float,
    person_head_width: float,
    person_foot_width_x: float = 0,
    person_foot_width_y: float = 0,
):
    points: list[list[float]] = [[0.0, 0.0, 0.0]]
    if person_foot_width_x > 0 or person_foot_width_y > 0:
        points.append([person_foot_width_x / 2, person_foot_width_y / 2, 0])
        points.append([-person_foot_width_x / 2, -person_foot_width_y / 2, 0])

    points += [
        [
            person_head_width / 2 * np.cos(a / 4 * 2 * np.pi),
            person_head_width / 2 * np.sin(a / 4 * 2 * np.pi),
            person_height,
        ]
        for a in range(4)
    ]
    return np.array(points)


def initial_rtvec_loader(app_config_calib: AppConfigCalibration, camera_index: int):
    with open(
        app_config_calib.calib2d3d.CalcCorrespondence.optparam_initialvector,
        encoding="utf-8",
    ) as rtf:
        initial_vectors: list[dict[str, list[float]]] = json.load(rtf)

    initial_rvec_rad: NDArray[np.float64] = np.array(
        initial_vectors[camera_index]["initial_rvec_rad"]
    )
    R = cv2.Rodrigues(initial_rvec_rad)[0]
    R = R @ np.array(
        [
            [
                1.0,
                0.0,
                0.0,
            ],
            [
                0.0,
                -1.0,
                0.0,
            ],
            [
                0.0,
                0.0,
                -1.0,
            ],
        ]
    )
    initial_rvec_rad = cv2.Rodrigues(R)[0]

    initial_tvec: NDArray[np.float64] = np.array(
        initial_vectors[camera_index]["initial_tvec"]
    )

    Rmat: NDArray[np.float64] = R
    return initial_rvec_rad, initial_tvec, Rmat


"""
bbox_filter
カメラの内部・外部パラメータから、地面に立つ人が画像上に現れた際のバウンディングボックスの画像上のピクセル面積、縦横比を推定する。
"""


class bbox2D_shapefilter:
    def __init__(
        self,
        Mc: NDArray[np.float64],
        app_config_calib: AppConfigCalibration,
        camera_index: int,
    ) -> None:
        self.enable_bbox2D_shapefilter = (
            app_config_calib.calib2d3d.Proc2d.enable_bbox2D_shapefilter
        )

        self.Mc = Mc
        self.image_size_hw = (
            app_config_calib.dataCapture.Camera.sys_height,
            app_config_calib.dataCapture.Camera.sys_width,
        )
        self.rvec, self.tvec, self.Rmat = initial_rtvec_loader(
            app_config_calib=app_config_calib, camera_index=camera_index
        )

        self.extrmat: NDArray[np.float64] = np.zeros((4, 4))
        self.extrmat[0:3, 0:3] = cv2.Rodrigues(self.rvec)[0]
        self.extrmat[0:3, 3] = self.tvec.T
        self.extrmat[3, 3] = 1

        if camera_index == 0:
            self.area_range_x = (3.5 - 6, 3.5 + 6, 30)
            self.area_range_y = (-2.2 - 6, -2.2, 30)
            self.z_ground = -1.3
            self.large_personspec = (2.2, 0.6, 0.6, 0)
            self.small_personspec = (1.0, 0.2, 0, 0)
        if camera_index == 1:
            self.area_range_x = (4.3, 4.3 + 6, 30)
            self.area_range_y = (-6.0, 6.0, 30)
            self.z_ground = -1.3
            self.large_personspec = (2.2, 0.6, 0, 0.6)
            self.small_personspec = (1.0, 0.2, 0, 0)
        if camera_index == 2:
            self.area_range_x = (3.5 - 6, 3.5 + 6, 30)
            self.area_range_y = (2.2, 2.2 + 6, 30)
            self.z_ground = -1.3
            self.large_personspec = (2.2, 0.6, 0.6, 0)
            self.small_personspec = (1.0, 0.2, 0, 0)

        self.plane_abcd = (0.0, 0.0, -1.0, self.z_ground + 1.65 / 2)
        self.compare_mode = "xymaxmin"

    def _estimate_bboxsize(  # TODO: 現状一点ずつ推定しているがベクトル化したい
        self,
        xc: float,
        yc: float,
        person_height: float,
        person_head_width: float,
        person_foot_width_x: float = 0,
        person_foot_width_y: float = 0,
    ) -> (
        tuple[
            NDArray[np.float64],
            NDArray[np.float64],
            NDArray[np.float64],
            NDArray[np.bool_],
            NDArray[np.float64],
            NDArray[np.float64],
        ]
        | None
    ):
        # min
        personpoint: NDArray[np.float64] = make_singlepoint_personshape(
            person_height,
            person_head_width,
            person_foot_width_x,
            person_foot_width_y,
        )
        res = intersect_pixel_ray_with_world_plane(
            pixel_xy=(xc, yc),
            K=self.Mc,
            R=self.Rmat,
            t=self.tvec,
            plane_abcd=self.plane_abcd,
        )
        if res is None:
            return None  # 計算できない場合（視線と平面が平行の場合：視線が地平線に向いている場合に該当）

        footpoint = res[0]
        is_front = res[1]
        pos_offset = np.array([*footpoint[:2], self.z_ground])

        pts_img_min = cv2.projectPoints(
            personpoint.reshape(-1, 3) + pos_offset,
            rvec=self.rvec,
            tvec=self.tvec,
            cameraMatrix=self.Mc,
            distCoeffs=np.zeros(5, dtype=np.float64),
        )[0]

        mn = pts_img_min.reshape((-1, 2)).min(axis=0)  # [xmin, ymin]
        mx = pts_img_min.reshape((-1, 2)).max(axis=0)  # [xmax, ymax]
        bbox = np.stack([mn, mx], axis=0)  # [[xmin,ymin],[xmax,ymax]]

        # bbox_c = np.mean(bbox, axis=1)
        bbox_diffx: NDArray[np.float64] = np.abs(mx[0] - mn[0])
        bbox_diffy: NDArray[np.float64] = np.abs(mx[1] - mn[1])
        bbox_a: NDArray[np.float64] = bbox_diffx * bbox_diffy
        bbox_yxr: NDArray[np.float64] = bbox_diffy / bbox_diffx

        return (
            bbox_a,
            bbox_yxr,
            bbox,
            is_front,
            bbox_diffx,
            bbox_diffy,
        )

    def _estimate_minmax_bboxsize(  # ここが以前と異なる
        self,
        searchpts: NDArray[np.float64] | None,
        allbbox_area: NDArray[np.float64],
    ) -> (
        tuple[
            NDArray[np.float64],
            NDArray[np.float64],
            NDArray[np.float64],
            NDArray[np.float64],
            NDArray[np.float64],
            NDArray[np.float64],
            NDArray[np.bool_],
            NDArray[np.bool_],
            NDArray[np.float64],
            NDArray[np.float64],
            NDArray[np.float64],
            NDArray[np.float64],
        ]
        | None
    ):
        """
        Docstring for _estimate_minmax_bboxsize

        :param searchpts: bbox中央座標 (-1,2)float行列想定。
        :type searchpts: Optional[NDArray[np.float64]] YOLO結果そのまま入れること想定。もしNoneやサイズゼロならNoneを返す。
        :return: Description YOLO結果がNoneならNone。異なるならサイズ8のTuple（最小最大面積、最小最大時y/x比、最小最大時bbox、最小最大カメラ前方フラグ）
        :rtype: tuple[NDArray[float64], NDArray[float64], NDArray[float64], NDArray[float64], NDArray[float64], NDArray[float64], NDArray[bool_], NDArray[bool_]] | None
        """

        if searchpts is None or len(searchpts) == 0:
            return None

        results_min_area = []
        results_max_area = []
        results_min_yxratio = []
        results_max_yxratio = []
        results_max_raw = []
        results_min_raw = []
        results_min_is_front = []
        results_max_is_front = []
        results_min_is_front = []
        results_max_is_front = []
        results_min_bbox_diffx = []
        results_min_bbox_diffy = []
        results_max_bbox_diffx = []
        results_max_bbox_diffy = []

        for (xc, yc), area in zip(searchpts, allbbox_area, strict=False):
            if area == 0:  # for calc speed, put dummy data for invalid bbox
                results_min_raw.append(np.array([[0, 0], [0, 0]], dtype=np.float64))
                results_min_area.append(100)
                results_min_yxratio.append(1)
                results_min_is_front.append(False)
                results_max_raw.append(np.array([[0, 0], [0, 0]], dtype=np.float64))
                results_max_area.append(200)
                results_max_yxratio.append(1.1)
                results_max_is_front.append(False)

                results_min_bbox_diffx.append(0)
                results_min_bbox_diffy.append(0)
                results_max_bbox_diffx.append(0)
                results_max_bbox_diffy.append(0)

            else:
                # min
                res = self._estimate_bboxsize(
                    xc=xc,
                    yc=yc,
                    person_height=self.small_personspec[0],
                    person_head_width=self.small_personspec[1],
                    person_foot_width_x=self.small_personspec[2],
                    person_foot_width_y=self.small_personspec[3],
                )
                if res is None:
                    res = (
                        1e10,
                        0,
                        np.array([[0, 0], [0, 0]], dtype=np.float64),
                        False,
                        0,
                        0,
                    )

                (
                    bbox_a,
                    bbox_yxr,
                    bbox,
                    is_front,
                    bbox_diffx,
                    bbox_diffy,
                ) = res
                results_min_raw.append(bbox)
                results_min_area.append(bbox_a)
                results_min_yxratio.append(bbox_yxr)
                results_min_is_front.append(is_front)
                results_min_bbox_diffx.append(bbox_diffx)
                results_min_bbox_diffy.append(bbox_diffy)

                # max
                res = self._estimate_bboxsize(
                    xc=xc,
                    yc=yc,
                    person_height=self.large_personspec[0],
                    person_head_width=self.large_personspec[1],
                    person_foot_width_x=self.large_personspec[2],
                    person_foot_width_y=self.large_personspec[3],
                )
                if res is None:
                    res = (
                        1e10,
                        0,
                        np.array([[0, 0], [0, 0]], dtype=np.float64),
                        False,
                        0,
                        0,
                    )

                (
                    bbox_a,
                    bbox_yxr,
                    bbox,
                    is_front,
                    bbox_diffx,
                    bbox_diffy,
                ) = res

                results_max_raw.append(bbox)
                results_max_area.append(bbox_a)
                results_max_yxratio.append(bbox_yxr)
                results_max_is_front.append(is_front)
                results_max_bbox_diffx.append(bbox_diffx)
                results_max_bbox_diffy.append(bbox_diffy)

        return (
            np.array(results_min_area, dtype=np.float64),
            np.array(results_max_area, dtype=np.float64),
            np.array(results_min_yxratio, dtype=np.float64),
            np.array(results_max_yxratio, dtype=np.float64),
            np.array(results_min_raw, dtype=np.float64),
            np.array(results_max_raw, dtype=np.float64),
            np.array(results_min_is_front, dtype=np.bool_),
            np.array(results_max_is_front, dtype=np.bool_),
            np.array(results_min_bbox_diffx, dtype=np.float64),
            np.array(results_min_bbox_diffy, dtype=np.float64),
            np.array(results_max_bbox_diffx, dtype=np.float64),
            np.array(results_max_bbox_diffy, dtype=np.float64),
        )

    def _is_correct_bboxsize(
        self, bbox_coordinate: NDArray[np.float64]
    ):  # -> NDArray[np.bool_]:
        """
        bboxサイズ判定

        :param self: Description
        :param bbox_coordinate: Description
        :type bbox_coordinate: NDArray[np.float64]

        返り値: is_inrange: 最大・最小範囲内か,
        allbbox_center:全bbox中心,
        allbbox_area: 全bbox面積,
        res_pts: 全最大最小bbox推定結果,
        mean_adiff: 推定した最大最小bbox面積の平均
        """

        # bbox_ymin = coor[0] * image_h
        # bbox_ymax = coor[2] * image_h
        # bbox_xmin = coor[1] * image_w
        # bbox_xmax = coor[3] * image_w
        allbbox_center = (bbox_coordinate[:, [1, 0]] + bbox_coordinate[:, [3, 2]]) / 2
        allbbox_dx = (
            bbox_coordinate[:, 3] - bbox_coordinate[:, 1]
        ) * self.image_size_hw[1]
        allbbox_dy = (
            bbox_coordinate[:, 2] - bbox_coordinate[:, 0]
        ) * self.image_size_hw[0]
        allbbox_area = allbbox_dx * allbbox_dy
        allbbox_center[:, 0] *= self.image_size_hw[1]
        allbbox_center[:, 1] *= self.image_size_hw[0]
        res_pts = self._estimate_minmax_bboxsize(
            searchpts=allbbox_center, allbbox_area=allbbox_area
        )  # bbox位置に対する推定bboxサイズ max/min
        if res_pts is None:
            return None

        vS, vL, rS, rL, bS, bL, fS, fL, bdSx, bdSy, bdLx, bdLy = (
            res_pts  # reproj版か否かで出力形式が違うことに注意
        )

        adiff = vL - vS
        mean_adiff = np.mean(np.max(adiff))
        area_max_corrected = vL  # TODO: 要検討
        area_min_corrected = vS  # TODO: 要検討

        if self.compare_mode == "area":
            is_inrange = (area_min_corrected < allbbox_area) & (
                allbbox_area < area_max_corrected
            )
        elif self.compare_mode == "xymaxmin":
            is_inrange = (
                (bdSx < allbbox_dx)
                & (allbbox_dx < bdLx)
                & (bdSy < allbbox_dy)
                & (allbbox_dy < bdLy)
            )

        else:
            raise RuntimeError(f"Undefined compare mode: {self.compare_mode}")

        return is_inrange, allbbox_center, allbbox_area, res_pts, mean_adiff

    def filter_bbox(
        self,
        allframe_bbox: list[NDArray[np.float64]] | None,
    ) -> list[NDArray[np.float64]] | None:
        """
        allframe_bbox: [bboxes, scores, class_ids, valid_detections]
        """

        if (
            allframe_bbox is None
            or len(allframe_bbox) == 0
            or not self.enable_bbox2D_shapefilter
        ):
            return allframe_bbox

        origshapes = [x.shape for x in allframe_bbox]

        compare_results = self._is_correct_bboxsize(
            bbox_coordinate=allframe_bbox[0].reshape((-1, 4))
        )

        if compare_results is None:
            return None

        inrange_mask = compare_results[0]
        allframe_bbox[1] = np.where(
            inrange_mask, allframe_bbox[1].reshape(-1), 0
        ).reshape(origshapes[1])  # bbox削除はリスクがあるため信頼度を0に書き換え

        return allframe_bbox
