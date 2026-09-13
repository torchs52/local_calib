import numpy as np
from numpy.typing import NDArray


class filter_static_objects:
    def __init__(
        self,
        voxel_size: float,
        x_range: tuple[float, float],
        y_range: tuple[float, float],
        z_range: tuple[float, float],
    ) -> None:
        # self._logger.warning(self, "ボクセル設定値ハードコーディング")
        # パラメータ設定
        self.voxel_size = voxel_size
        self.x_range = x_range
        self.y_range = y_range
        self.z_range = z_range

        # ボクセルグリッドのサイズ
        self.grid_shape = (
            int((self.x_range[1] - self.x_range[0]) / self.voxel_size),
            int((self.y_range[1] - self.y_range[0]) / self.voxel_size),
            int((self.z_range[1] - self.z_range[0]) / self.voxel_size),
        )

        self.filtersource_framecount = 0

        self.voxel_counts = np.zeros(self.grid_shape, dtype=np.float32)

    def frame_to_voxel_indices(self, frame: NDArray[np.float32]):
        """
        点群フレームをボクセルインデックスに変換（高速行列演算）
        """
        coords = frame[:, :3]
        mask = (
            (coords[:, 0] >= self.x_range[0])
            & (coords[:, 0] < self.x_range[1])
            & (coords[:, 1] >= self.y_range[0])
            & (coords[:, 1] < self.y_range[1])
            & (coords[:, 2] >= self.z_range[0])
            & (coords[:, 2] < self.z_range[1])
        )
        filtered_frame = frame[mask]
        filtered_coords = filtered_frame[:, :3]
        voxel_indices = np.floor(
            (filtered_coords - [self.x_range[0], self.y_range[0], self.z_range[0]])
            / self.voxel_size
        ).astype(int)
        return voxel_indices, filtered_frame

    def add_single_voxel_map(self, frame: NDArray[np.float32]):
        self.filtersource_framecount += 1
        voxel_indices, _ = self.frame_to_voxel_indices(frame)
        voxel_map = np.zeros(self.grid_shape, dtype=np.float32)
        voxel_map[voxel_indices[:, 0], voxel_indices[:, 1], voxel_indices[:, 2]] = 1.0
        self.voxel_counts += voxel_map

    def apply_voxelfilter(self, threshold_ratio: float = 1.0 / 3):
        if self.filtersource_framecount == 0:
            print("warning: self.filtersource_framecount is zero. cannot apply filter")
            return

        # self.voxel_counts /= float(self.filtersource_framecount)
        # self.filtersource_framecount = 1
        self.static_mask = (
            self.voxel_counts / float(self.filtersource_framecount)
        ) >= threshold_ratio

    def make_voxel_filtermap_multiframes(
        self, frames: list[NDArray[np.float32]], threshold_ratio: float
    ):
        # 各フレームのボクセルマップ作成
        for frame in frames:
            self.add_single_voxel_map(frame)

        self.apply_voxelfilter(threshold_ratio=threshold_ratio)

    def extract_moving_objects(self, frame: NDArray[np.float32]):
        """
        Mフレームの点群から動体点群を抽出
        """
        voxel_indices, filtered_frame = self.frame_to_voxel_indices(frame)
        is_moving = ~self.static_mask[
            voxel_indices[:, 0], voxel_indices[:, 1], voxel_indices[:, 2]
        ]
        moving_points = filtered_frame[is_moving]

        return moving_points

    # 使用例（framesは(N, 4)のnumpy配列のリスト）
    # frames = [frame1, frame2, ..., frameM]
    # moving_objects = detect_moving_objects(frames)
