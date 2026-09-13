import glob
import json
import os
from pathlib import Path

import cv2
import numpy as np
from cv2.typing import MatLike
from numpy.typing import NDArray


class CameraHelper:
    @staticmethod
    def undistort_image(
        image: MatLike,
        height: int,
        width: int,
        map1: MatLike,
        map2: MatLike,
        dst: NDArray[np.uint8] | None = None,
    ) -> MatLike:
        if height != image.shape[0] or width != image.shape[1]:
            image = cv2.resize(image, (width, height))

        # __init__にて作成した魚眼補正用変換行列を使用し魚眼カメラ画像を歪み補正
        remap_kwargs = {
            "src": image,
            "map1": map1,
            "map2": map2,
            "interpolation": cv2.INTER_LINEAR,
            "borderMode": cv2.BORDER_CONSTANT,
        }
        if dst is None:
            return cv2.remap(**remap_kwargs)

        cv2.remap(dst=dst, **remap_kwargs)
        return dst

    @staticmethod
    def get_lidar_camera_calib_para(
        fisheye_param_file: str,
        list_files_pattern: str,
        center_rotate: tuple[float, float, float],
        center_shift: tuple[float, float, float],
    ) -> tuple[
        NDArray[np.float32],
        NDArray[np.float32],
        int,
        int,
        NDArray[np.float32],
        MatLike,
        NDArray[np.float64],
        NDArray[np.float64],
    ]:
        """カメラ-Lidar間校正関連パラメータ"""

        # 魚眼カメラ歪み補正データの読み込み
        cm, dm, width, height, ncm1 = CameraHelper.read_fisheye_param(
            fisheye_param_file,
        )

        # 最新ファイルを自動的に選ぶ
        list_of_files = glob.glob(list_files_pattern)
        latest_file = max(list_of_files, key=os.path.getmtime)
        rt_matrix_cam2lid: NDArray[np.float64] = np.loadtxt(
            latest_file,
            delimiter=",",
            dtype=np.float64,
        )
        rt_matrix_cam2lid.reshape(4, 4)
        r: NDArray[np.float64] = rt_matrix_cam2lid[0:3, 0:3]
        rvec_os2cam, _ = cv2.Rodrigues(r)
        tvec_os2cam: NDArray[np.float64] = np.array(
            [rt_matrix_cam2lid[0, 3], rt_matrix_cam2lid[1, 3], rt_matrix_cam2lid[2, 3]],
            dtype=np.float64,
        )

        invtvec: NDArray[np.float64] = CameraHelper.calc_focalpoint_coordn(
            rt_matrix_cam2lid,
            center_rotate,
            center_shift,
        )

        return cm, dm, width, height, ncm1, rvec_os2cam, tvec_os2cam, invtvec

    @staticmethod
    def read_fisheye_param(
        path: str,
    ) -> tuple[
        NDArray[np.float32],
        NDArray[np.float32],
        int,
        int,
        NDArray[np.float32],
    ]:
        with Path(path).open() as json_open:
            json_load = json.load(json_open)

            # fisheye関連はfloat32で渡す必要あり
            cm: NDArray[np.float32] = np.array(json_load["camera_matrix"]["data"])
            cm = cm.reshape(
                [
                    json_load["camera_matrix"]["rows"],
                    json_load["camera_matrix"]["cols"],
                ],
            )
            dm: NDArray[np.float32] = np.array(
                [json_load["k1"], json_load["k2"], json_load["k3"], json_load["k4"]],
            )
            width: int = json_load["image_width"]
            height: int = json_load["image_height"]

            ncm1: NDArray[np.float32] = np.array(
                json_load["new_camera_matrix_alpha1"]["data"],
            )
            ncm1 = ncm1.reshape(
                [
                    json_load["new_camera_matrix_alpha1"]["rows"],
                    json_load["new_camera_matrix_alpha1"]["cols"],
                ],
            )

        return cm, dm, width, height, ncm1

    @staticmethod
    def calc_focalpoint_coordn(
        tg_g: NDArray[np.float64],
        center_rotate: tuple[float, float, float],
        center_shift: tuple[float, float, float],
    ) -> NDArray[np.float64]:
        """カメラ位置の計算"""
        centerrotate: NDArray[np.float64] = np.array(center_rotate)
        centerrotete_mat = cv2.Rodrigues(centerrotate / 180.0 * np.pi)[0]
        centershift: NDArray[np.float64] = np.array(center_shift)
        r: NDArray[np.float64] = tg_g[:3, :3]
        t: NDArray[np.float64] = (tg_g[:3, 3]).T
        invtvec: NDArray[np.float64] = (
            np.matmul(r.T, -t) @ centerrotete_mat + centershift
        )
        return invtvec

    @staticmethod
    def apply_ratio(
        cm: NDArray[np.float32],
        ncm1: NDArray[np.float32],
        camera_size: tuple[int, int],
        para_size: tuple[int, int],
    ) -> tuple[NDArray[np.float32], NDArray[np.float32]]:
        """1280x720対応 カメラ内部パラメータ (z軸1以外) にW,Hの比率を適用"""
        intrinsics_coeff_x: float = float(camera_size[0]) / float(para_size[0])
        intrinsics_coeff_y: float = float(camera_size[1]) / float(para_size[1])

        cm[0, :] = cm[0, :] * intrinsics_coeff_x
        cm[1, :] = cm[1, :] * intrinsics_coeff_y

        ncm1[0, :] = ncm1[0, :] * intrinsics_coeff_x
        ncm1[1, :] = ncm1[1, :] * intrinsics_coeff_y
        return cm, ncm1

    @staticmethod
    def init_undistort_rectify_map(
        cm: NDArray[np.float32],
        dm: NDArray[np.float32],
        ncm1: NDArray[np.float32],
        size: tuple[int, int],
    ) -> tuple[MatLike, MatLike]:
        return cv2.fisheye.initUndistortRectifyMap(
            K=cm,
            D=dm,
            R=np.eye(3),
            P=ncm1,
            size=size,
            m1type=cv2.CV_16SC2,
        )
