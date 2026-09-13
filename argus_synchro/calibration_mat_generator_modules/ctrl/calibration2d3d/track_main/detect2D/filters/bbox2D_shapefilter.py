import json

import cv2
import numpy as np
from numpy.typing import NDArray
from scipy.interpolate import griddata

from argus_synchro.config.app_config_calibration import AppConfigCalibration


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
    return initial_rvec_rad, initial_tvec


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
        self.Mc = Mc
        self.image_size_hw = (
            app_config_calib.dataCapture.Camera.sys_height,
            app_config_calib.dataCapture.Camera.sys_width,
        )
        self.rvec, self.tvec = initial_rtvec_loader(
            app_config_calib=app_config_calib, camera_index=camera_index
        )

        if camera_index == 0:
            self.area_range_x = (3.5 - 6, 3.5 + 6, 30)
            self.area_range_y = (-2.2 - 6, -2.2, 30)
            self.z_ground = -1.3
        if camera_index == 1:
            self.area_range_x = (4.3, 4.3 + 6, 30)
            self.area_range_y = (-6.0, 6.0, 30)
            self.z_ground = -1.3
        if camera_index == 2:
            self.area_range_x = (3.5 - 6, 3.5 + 6, 30)
            self.area_range_y = (2.2, 2.2 + 6, 30)
            self.z_ground = -1.3

        # self.diff_w = (0.3, 0.8)
        self.diff_w = (
            0.01,
            0.8,
        )  # min,maxのペア 予測bboxを8点の投影点に外接するよう出している為、人が斜めに映る時本来の人につくbboxより大きくなるため極細として対応
        self.diff_d = (0.01, 0.8)
        self.diff_h = (1.45, 1.90)

        (
            self.min_refpts,
            self.min_bbox_areas,
            self.min_yxratio,
            self.max_refpts,
            self.max_bbox_areas,
            self.max_yxratio,
            min_bbox,
            max_bbox,
            min_valid_verts,
            min_valid_imgpoints,
            max_valid_verts,
            max_valid_imgpoints,
        ) = self._prepare_refmatset()

        self.debuglog_filename: str | None = None

    def _generate_cuboid_vertices(
        self,
        bbox_size: tuple[float, float, float],
        dtype=np.float64,
        return_flat: bool = False,
    ) -> NDArray[np.float64]:
        print(f"_generate_cuboid_vertices() arg: {bbox_size}")
        xmin, xmax, nx = self.area_range_x
        ymin, ymax, ny = self.area_range_y
        z_ground = self.z_ground
        diff_x, diff_y, diff_z = bbox_size
        """
        指定範囲で (nx, ny) 個のスキャン点 (x,y) を作り、それぞれを中心とする
        x: ±diff_x/2, y: ±diff_y/2, z: {0, diff_z} の直方体8頂点を返す。

        Parameters
        ----------
        xmin, xmax : float
            x の範囲
        nx : int
            x 方向の分割数（スキャン点個数, 端を含む）
        ymin, ymax : float
            y の範囲
        ny : int
            y 方向の分割数（スキャン点個数, 端を含む）
        diff_x, diff_y, diff_z : float
            直方体の各軸寸法（>0）
            ※ 各スキャン点の (x,y) を中心に配置。z は底面 0、上面 diff_z。
        dtype : np.dtype
            出力の dtype
        return_flat : bool
            True の場合、(N*8, 3) にフラット化して返す。False の場合、(N, 8, 3)。

        Returns
        -------
        verts : np.ndarray
            形状 (N, 8, 3) または (N*8, 3) の頂点座標配列。
            N = nx * ny
            頂点順序（C-order）は以下：
            z=0 面:  (x-, y-), (x+, y-), (x-, y+), (x+, y+)
            z=diff:  (x-, y-), (x+, y-), (x-, y+), (x+, y+)
        """
        # --- 入力検証 ---
        if nx < 1 or ny < 1:
            raise ValueError("nx, ny は 1 以上を指定してください。")
        if not (xmax >= xmin and ymax >= ymin):
            raise ValueError(
                "範囲が不正です（xmax >= xmin, ymax >= ymin を満たす必要があります）。"
            )
        if diff_x <= 0 or diff_y <= 0 or diff_z <= 0:
            raise ValueError("diff_x, diff_y, diff_z は正の値を指定してください。")

        # --- スキャン点中心 (x,y) を作成 ---
        x_centers = np.linspace(xmin, xmax, nx, dtype=dtype)
        y_centers = np.linspace(ymin, ymax, ny, dtype=dtype)
        Xc, Yc = np.meshgrid(x_centers, y_centers, indexing="xy")  # (ny, nx)

        centers_xy: NDArray[np.float64] = np.stack(
            [Xc.ravel(), Yc.ravel()], axis=1
        )  # (N, 2)
        N = centers_xy.shape[0]

        # --- 頂点のオフセット（x±, y±, z∈{0, diff_z} の全 8 組）---
        x_off = np.array([-0.5 * diff_x, +0.5 * diff_x], dtype=dtype)
        y_off = np.array([-0.5 * diff_y, +0.5 * diff_y], dtype=dtype)
        z_vals = np.array([z_ground, z_ground + diff_z], dtype=dtype)

        # meshgrid で 8 組 (2×2×2) を作る → (2,2,2,3) → (8,3)
        offs = np.stack(
            np.meshgrid(x_off, y_off, z_vals, indexing="xy"), axis=-1
        )  # (..., 3)
        xy_offs = offs[..., :2].reshape(-1, 2)  # (8, 2)
        z_combo = offs[..., 2].reshape(-1)  # (8,)

        # --- ブロードキャストで一括生成 ---
        verts_xy = centers_xy[:, None, :] + xy_offs[None, :, :]  # (N, 8, 2)
        verts_z = np.broadcast_to(z_combo[None, :, None], (N, 8, 1))  # (N, 8, 1)
        verts = np.concatenate([verts_xy, verts_z], axis=-1)  # (N, 8, 3)

        if return_flat:
            return verts.reshape(-1, 3)
        return verts

    @staticmethod
    def _calc_imgpoints_single(
        verts: NDArray[np.float64],
        rvec: NDArray[np.float64],
        tvec: NDArray[np.float64],
        Mc: NDArray[np.float64],
    ):
        verts3d = verts.reshape(-1, 3)
        imgpoints, _ = cv2.projectPoints(
            objectPoints=verts3d,
            rvec=rvec,
            tvec=tvec,
            cameraMatrix=Mc,
            distCoeffs=np.zeros(5),
        )
        extrmat: NDArray[np.float64] = np.zeros((4, 4))
        extrmat[0:3, 0:3] = cv2.Rodrigues(rvec)[0]
        extrmat[0:3, 3] = tvec.T
        extrmat[3, 3] = 1
        homogeneous_points = np.hstack([verts3d, np.ones((verts3d.shape[0], 1))]).T
        camera_coordinate_pts = extrmat @ homogeneous_points
        camera_coordin_z = camera_coordinate_pts[2]

        return verts, imgpoints.reshape(-1, 8, 2), camera_coordin_z.reshape(-1, 8)

    def _prepare_refmat_single(self, bbox_size: tuple[float, float, float]):
        verts = self._generate_cuboid_vertices(bbox_size=bbox_size)
        verts, imgpoints, camera_coordin_z = self._calc_imgpoints_single(
            verts=verts, rvec=self.rvec, tvec=self.tvec, Mc=self.Mc
        )

        valid_verts = verts[np.all(camera_coordin_z > 0, axis=1)]
        valid_imgpoints = imgpoints[np.all(camera_coordin_z > 0, axis=1)]

        bbox = np.stack(
            [valid_imgpoints.min(axis=1), valid_imgpoints.max(axis=1)], axis=1
        )
        bbox_c = np.mean(bbox, axis=1)
        bbox_diffx = np.abs(bbox[:, 1, 0] - bbox[:, 0, 0])
        bbox_diffy = np.abs(bbox[:, 1, 1] - bbox[:, 0, 1])
        bbox_a = bbox_diffx * bbox_diffy
        bbox_yxr = bbox_diffy / bbox_diffx

        refpts = bbox_c

        return refpts, bbox_a, bbox_yxr, bbox, valid_verts, valid_imgpoints

    def _prepare_refmatset(self):
        (
            min_refpts,
            min_bbox_areas,
            min_yxratio,
            min_bbox,
            min_valid_verts,
            min_valid_imgpoints,
        ) = self._prepare_refmat_single(
            bbox_size=(self.diff_d[0], self.diff_w[0], self.diff_h[0])
        )
        (
            max_refpts,
            max_bbox_areas,
            max_yxratio,
            max_bbox,
            max_valid_verts,
            max_valid_imgpoints,
        ) = self._prepare_refmat_single(
            bbox_size=(self.diff_d[1], self.diff_w[1], self.diff_h[1])
        )

        return (
            min_refpts,
            min_bbox_areas,
            min_yxratio,
            max_refpts,
            max_bbox_areas,
            max_yxratio,
            min_bbox,
            max_bbox,
            min_valid_verts,
            min_valid_imgpoints,
            max_valid_verts,
            max_valid_imgpoints,
        )

    def _estimate_minmax_bboxsize(
        self, searchpts: NDArray[np.float64] | None
    ) -> (
        tuple[
            NDArray[np.float64],
            NDArray[np.float64],
            NDArray[np.float64],
            NDArray[np.float64],
        ]
        | None
    ):
        if searchpts is None or len(searchpts) == 0:
            return None

        min_interpolated_areadata = griddata(
            points=self.min_refpts,
            values=self.min_bbox_areas,
            xi=searchpts,
            method="linear",
        )
        nanval_filter = np.isnan(min_interpolated_areadata)
        nan_coordinates = searchpts[nanval_filter]
        min_interpolated_areadata[nanval_filter] = griddata(
            points=self.min_refpts,
            values=self.min_bbox_areas,
            xi=nan_coordinates,
            method="nearest",
        )

        min_interpolated_yxratio = griddata(
            points=self.min_refpts,
            values=self.min_yxratio,
            xi=searchpts,
            method="linear",
        )
        nanval_filter = np.isnan(min_interpolated_yxratio)
        nan_coordinates = searchpts[nanval_filter]
        min_interpolated_yxratio[nanval_filter] = griddata(
            points=self.min_refpts,
            values=self.min_yxratio,
            xi=nan_coordinates,
            method="nearest",
        )

        max_interpolated_areadata = griddata(
            points=self.max_refpts,
            values=self.max_bbox_areas,
            xi=searchpts,
            method="linear",
        )
        nanval_filter = np.isnan(max_interpolated_areadata)
        nan_coordinates = searchpts[nanval_filter]
        max_interpolated_areadata[nanval_filter] = griddata(
            points=self.max_refpts,
            values=self.max_bbox_areas,
            xi=nan_coordinates,
            method="nearest",
        )

        max_interpolated_yxratio = griddata(
            points=self.max_refpts,
            values=self.max_yxratio,
            xi=searchpts,
            method="linear",
        )
        nanval_filter = np.isnan(max_interpolated_yxratio)
        nan_coordinates = searchpts[nanval_filter]
        max_interpolated_yxratio[nanval_filter] = griddata(
            points=self.max_refpts,
            values=self.max_yxratio,
            xi=nan_coordinates,
            method="nearest",
        )

        return (
            min_interpolated_areadata,
            min_interpolated_yxratio,
            max_interpolated_areadata,
            max_interpolated_yxratio,
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
            searchpts=allbbox_center
        )  # bbox位置に対する推定bboxサイズ max/min
        if res_pts is None:
            return None

        adiff = res_pts[2] - res_pts[0]
        mean_adiff = np.mean(np.max(adiff))
        area_max_corrected = res_pts[2]  # TODO: 要検討
        area_min_corrected = res_pts[0]  # TODO: 要検討

        is_inrange = (area_min_corrected < allbbox_area) & (
            allbbox_area < area_max_corrected
        )

        return is_inrange, allbbox_center, allbbox_area, res_pts, mean_adiff

    def filter_bbox(
        self,
        allframe_bbox: list[NDArray[np.float64]] | None,
    ) -> list[NDArray[np.float64]] | None:
        if allframe_bbox is None or len(allframe_bbox) == 0:
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
