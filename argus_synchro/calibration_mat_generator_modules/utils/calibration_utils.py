"""校正用ユーティリティ関数: 3D座標変換・キャリブレーション行列I/O"""

import os

import cv2
import numpy as np
from numpy.typing import NDArray


def conbine3d3d(xyz_data: list, trans_mat3D3D_eachlidar: list):
    """複数の3D座標系を変換行列で統合する。"""
    for ix in range(len(trans_mat3D3D_eachlidar)):
        xyz_data[ix][:, :3] = (
            np.dot(xyz_data[ix][:, :3], trans_mat3D3D_eachlidar[ix][:3, :3].T)
            + trans_mat3D3D_eachlidar[ix][:3, 3]
        )
    return np.concatenate([xyz_data[0], xyz_data[1]], axis=0)


def read_rtvec_with_inputcheck(
    rvec_convmat_path: str = "",
    new_axis_mode: bool | None = None,
    points_inverted: bool = True,
):
    """対話的にキャリブレーション行列の入力パスを確認して読込。"""
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
    """カメラ座標系のキャリブレーション行列（回転・並進）を読込。
    
    Args:
        rvec_convmat_path: 回転行列またはコンバージョン行列のパス
        new_axis_mode: True=座標軸反転モード
        points_inverted: True=座標が既に反転済み
        tvec_path: 並進ベクトルのパス（rvecがnpyの場合必須）
    
    Returns:
        (rvec, tvec, convmat)
    """
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
