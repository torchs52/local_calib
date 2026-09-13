import glob
import json
import os

import cv2
import numpy as np
from numpy.typing import NDArray

from argus_synchro.config.app_config import CalibrationConf


def calib_lidar(
    pts: NDArray[np.float64],
    transform_values: NDArray[np.float64],
) -> NDArray[np.float64]:
    """
    numpy 点群を読み込み、
    CSVの4×4変換行列を適用してキャリブレーション済みの点群を返す

    Parameters
    ----------
    pts : (N,3) array
        生点群
    transform_values : (4,4) array
        変換行列を事前に読み込んだ値

    Returns
    -------
    (N,3) array
        キャリブレーション後の点群
    """
    rot = transform_values[:3, :3]
    trans = transform_values[:3, 3]
    pts_xyz = np.ascontiguousarray(pts[:, :3])
    return (pts_xyz @ rot.T) + trans


def old_calib_lidar(
    pts: NDArray[np.float64],
    transform_file: str,
) -> NDArray[np.float64]:
    """
    numpy 点群を読み込み、
    CSVの4×4変換行列を適用してキャリブレーション済みの点群を返す

    Parameters
    ----------
    pts : (N,3) array
        生点群
    transform_file : str
        4×4行列がヘッダなしCSVで格納されたファイルパス

    Returns
    -------
    (N,3) array
        キャリブレーション後の点群
    """
    import pandas as pd

    # 1) 行列読み込み
    values: NDArray[np.float64] = pd.read_csv(
        transform_file,
        header=None,
    ).values  # shape (4,4)

    # 2) 同次座標系への拡張
    n = pts.shape[0]
    ones = np.ones((n, 1), dtype=pts.dtype)
    hom_pts = np.hstack((pts[:, :3], ones))  # (n,4)

    # 3) 変換行列適用
    hom_trans = hom_pts @ values.T  # (n,4)

    # 4) XYZに戻してreturn
    return hom_trans[:, :3]


# motecカメラのパラメータ読み込み
def read_fisheye_param(
    path: str,
) -> tuple[NDArray[np.float32], NDArray[np.float32], int, int, NDArray[np.float32]]:
    json_open = open(path)
    json_load = json.load(json_open)

    # fisheye関連はfloat32で渡す必要あり
    cm: NDArray[np.float32] = np.array(json_load["camera_matrix"]["data"])
    cm = cm.reshape(
        [json_load["camera_matrix"]["rows"], json_load["camera_matrix"]["cols"]],
    )
    dm: NDArray[np.float32] = np.array(
        [json_load["k1"], json_load["k2"], json_load["k3"], json_load["k4"]],
    )
    W: int = json_load["image_width"]
    H: int = json_load["image_height"]

    ncm1: NDArray[np.float32] = np.array(json_load["new_camera_matrix_alpha1"]["data"])
    ncm1 = ncm1.reshape(
        [
            json_load["new_camera_matrix_alpha1"]["rows"],
            json_load["new_camera_matrix_alpha1"]["cols"],
        ],
    )
    json_open.close()

    return cm, dm, W, H, ncm1


# カメラ位置の計算
def calc_focalpoint_coordn(
    tgm: NDArray[np.float64],
    calibration_conf: CalibrationConf,
) -> NDArray[np.float64]:
    centerrotate = np.array(calibration_conf.center_rotate)
    centerrotete_mat = cv2.Rodrigues(centerrotate / 180.0 * np.pi)[0]
    centershift = np.array(calibration_conf.center_shift)
    R = tgm[:3, :3]
    t = (tgm[:3, 3]).T
    # invtvec = np.matmul(R.T, -t) #本番はこちら
    return np.matmul(R.T, -t) @ centerrotete_mat + centershift


def get_lidar_camera_calib_para(
    cam_index: int,
    calibration_conf: CalibrationConf,
) -> tuple[
    NDArray[np.float32],
    NDArray[np.float32],
    int,
    int,
    NDArray[np.float32],
    cv2.typing.MatLike,
    NDArray[np.float64],
    NDArray[np.float64],
]:
    ############################################
    # カメラ-Lidar間校正関連パラメータ
    ############################################

    # カメラパラメータ読み込み
    # 前回実験時のフォルダにあった適当なファイル
    # mtx = np.load(calibration_conf.mtx_file)#fisheyeでは使わない
    # dist = np.load(calibration_conf.dist_file)#fisheyeでは使わない
    # 魚眼カメラ歪み補正データの読み込み
    cm, dm, W, H, ncm1 = read_fisheye_param(calibration_conf.fisheye_param_file)

    # 最新ファイルを自動的に選ぶ
    # list_of_files=glob.glob(calibration_conf.list_files)
    list_files: dict[int, str] = {
        0: calibration_conf.list_0_files,
        1: calibration_conf.list_1_files,
        2: calibration_conf.list_2_files,
    }
    list_of_files = glob.glob(list_files[cam_index])
    latest_file = max(list_of_files, key=os.path.getmtime)
    rt_matrix_cam2lid = np.loadtxt(latest_file, delimiter=",", dtype=np.float64)
    rt_matrix_cam2lid.reshape(4, 4)
    R = rt_matrix_cam2lid[0:3, 0:3]
    rvec_os2cam, _ = cv2.Rodrigues(R)
    tvec_os2cam = np.array(
        [rt_matrix_cam2lid[0, 3], rt_matrix_cam2lid[1, 3], rt_matrix_cam2lid[2, 3]],
        dtype=np.float64,
    )

    invtvec = calc_focalpoint_coordn(rt_matrix_cam2lid, calibration_conf)

    return cm, dm, W, H, ncm1, rvec_os2cam, tvec_os2cam, invtvec
