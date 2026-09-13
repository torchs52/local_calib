from __future__ import annotations

import numpy as np
from numpy.typing import NDArray


def read_lidar_file(
    ref_t: int,
    dpath: list[str],
    tot_frames: int,
) -> list[NDArray[np.float64]]:
    """
    評価モードでLiDARデータを読み込む
    """
    xyz_data: list[NDArray[np.float64]] = []
    frame_numbers: list[str] = [f"{ref_t + idx:06d}" for idx in range(tot_frames)]

    for lidar_path in dpath:
        # 構築された各フレームのファイルパスをリスト化
        lidar_files: list[str] = [f"{lidar_path}{frame}.npy" for frame in frame_numbers]

        # 各ファイルからデータを読み込み、リストに格納
        lidar_frames: list[NDArray[np.float64]] = [
            np.load(file, allow_pickle=True) for file in lidar_files
        ]

        # フレームが1つの場合はそのまま、複数の場合は結合
        if tot_frames == 1:
            xyz: NDArray[np.float64] = lidar_frames[0]
        else:
            xyz: NDArray[np.float64] = np.concatenate(lidar_frames, axis=0)

        xyz_data.append(xyz)

    return xyz_data


def eval_read_lidar_file(
    ref_t: int,
    tot_frames: int,
    eval_data: list[str],
) -> list[NDArray[np.float64]]:
    """
    評価モードでLiDARデータを読み込む
    """
    xyz_data: list[NDArray[np.float64]] = []
    frame_numbers: list[str] = [f"{ref_t + idx:06d}" for idx in range(tot_frames)]

    for lidar_path in eval_data:
        # 構築された各フレームのファイルパスをリスト化
        lidar_files: list[str] = [f"{lidar_path}{frame}.npy" for frame in frame_numbers]

        # 各ファイルからデータを読み込み、リストに格納
        lidar_frames: list[NDArray[np.float64]] = [
            np.load(file, allow_pickle=True) for file in lidar_files
        ]

        # フレームが1つの場合はそのまま、複数の場合は結合
        if tot_frames == 1:
            xyz: NDArray[np.float64] = lidar_frames[0]
        else:
            xyz: NDArray[np.float64] = np.concatenate(lidar_frames, axis=0)

        xyz_data.append(xyz)

    return xyz_data
