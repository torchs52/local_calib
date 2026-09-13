# カメラ校正行列の差分チェックスクリプト

import numpy as np
import cv2


def conv_rtvec_to_mat(rvec, tvec):
    # rvecとtvecから4x4の変換行列を作成する
    R, _ = cv2.Rodrigues(rvec)
    mat = np.eye(4)
    mat[:3, :3] = R
    mat[:3, 3] = tvec
    return mat


def conv_4x4mat_to_rtvec(mat, x_inverted=False):
    # 4x4の変換行列からrvecとtvecを抽出する。x_invertedがTrueの場合はX軸回りに反転した行列を使用する(新旧変換)
    rvec = np.zeros(3)
    tvec = np.zeros(3)

    Rxinv = np.array(
        [
            [1.0, 0.0, 0.0, 0.0],
            [0.0, -1.0, 0.0, 0.0],
            [0.0, 0.0, -1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ]
    )  # X軸回りに反転。180°回転に関してはinvも同じ

    if x_inverted:
        mat = mat @ Rxinv
    tvec[:] = mat[:3, 3]
    R = mat[:3, :3]

    rvec, _ = cv2.Rodrigues(R)
    rvec = rvec.flatten()
    return rvec, tvec


# opencvのrvec, tvecは世界→カメラの変換を表す。ここではカメラの位置と設置角度を求めたい
def conv_rtvec_to_camera_pose(rvec, tvec):
    R, _ = cv2.Rodrigues(rvec)
    R = R.T
    tvec = -R @ tvec
    rvec, _ = cv2.Rodrigues(R)
    rvec = rvec.flatten()
    return R, tvec, rvec


# rvecとtvecの差分をチェックする。thresholdを差の絶対値が超えていればFalse。
# camerapos_modeがTrueの場合はカメラの位置と設置角度で比較し、Falseの場合はそのままrvecとtvecの差分で比較する
def check_diff(
    rvec1,
    tvec1,
    rvec2,
    tvec2,
    rvec_threshold_deg,
    tvec_threshold,
    camerapos_mode,
    verbose=False,
):
    # カメラの位置と設置角度で比較する場合は、rvecとtvecをカメラ座標系に変換する。「実際にカメラがどこに付いていると推定されたか」ベースでの比較になる
    if camerapos_mode:
        _, tvec1, rvec1 = conv_rtvec_to_camera_pose(rvec1, tvec1)
        _, tvec2, rvec2 = conv_rtvec_to_camera_pose(rvec2, tvec2)
    rvec_diff = rvec2 - rvec1
    tvec_diff = tvec2 - tvec1
    rvec_diff_deg = rvec_diff / np.pi * 180
    rvec_ok = np.all(np.abs(rvec_diff_deg) <= rvec_threshold_deg)
    tvec_ok = np.all(np.abs(tvec_diff) <= tvec_threshold)
    return rvec_ok, tvec_ok, rvec_diff_deg, tvec_diff