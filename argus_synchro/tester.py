import pickle
from pathlib import Path
from typing import Any

import numpy as np
from argus_synchro_lib.octotree import OctoTree
from numpy.typing import NDArray

from argus_synchro.message.scrutinizer_message import AccumPointsData

# テスト時設定
# [UI_IF]
# godot_ui = True
# [Scrutinizer]
# s_frame = 1
# e_frame = 30000
# [detect2d]
# isApplied = False


class Tester:
    def __init__(self, end: int = 100) -> None:
        self._end: int = end
        self._count: int = 0
        self._pre_frame = 0
        self._sizes: list[tuple[int | tuple[int, ...], ...]] = []
        self._data: list[tuple[Any, ...]] = []

    def record(
        self,
        frame: int,
        points: AccumPointsData,
        downsample: NDArray[np.float64],
        boxes: NDArray[np.float64],
        # lines: NDArray[np.float32],
        minmax: NDArray[np.float32],
        valid_detects: NDArray[np.int32],
        # clustering_labels: NDArray[np.int32],
        # edge_result: EdgeDetectionResult,
        # disp_pcd: tuple[Vector3dVector, Vector2iVector, Vector3dVector],
        octotree_obj_pcd: OctoTree,
        angle_deg: int,
    ) -> None:
        assert (frame - 1) == self._pre_frame
        self._sizes.append(
            (
                frame,
                points.point_cloud.shape,
                downsample.shape,
                boxes.shape,
                # lines.shape,
                minmax.shape,
                valid_detects.shape,
                # clustering_labels.shape,
                # edge_result.edge_points.shape,
                # edge_result.edge_lines.shape,
                # edge_result.edge_length.shape,
                # np.array(disp_pcd[0]).shape,
                # np.array(disp_pcd[1]).shape,
                # np.array(disp_pcd[2]).shape,
                sum(
                    len(value) for _, value in octotree_obj_pcd.entity_octonodes.items()
                )
                if octotree_obj_pcd.entity_octonodes is not None
                else 0,
                angle_deg,
            )
        )
        self._data.append(
            (
                points.point_cloud[:3].tolist(),
                downsample[:3].tolist(),
                boxes[:3].tolist(),
                # lines[:3].tolist(),
                minmax[:3].tolist(),
                valid_detects[0],
                # clustering_labels[:3].tolist(),
                # edge_result.edge_points[:3].tolist(),
                # edge_result.edge_lines[:3].tolist(),
                # edge_result.edge_length[:3].tolist(),
                # np.array(disp_pcd[0]).tolist(),
                # np.array(disp_pcd[1]).tolist(),
                # np.array(disp_pcd[2]).tolist(),
                angle_deg,
            )
        )
        print(f"{frame}/{self._end}")
        self._pre_frame: int = frame
        self._count += 1
        assert self._count < self._end

    def test(
        self,
        frame: int,
        points: AccumPointsData,
        downsample: NDArray[np.float64],
        boxes: NDArray[np.float64],
        # lines: NDArray[np.float32],
        minmax: NDArray[np.float32],
        valid_detects: NDArray[np.int32],
        # clustering_labels: NDArray[np.int32],
        # edge_result: EdgeDetectionResult,
        # disp_pcd: tuple[Vector3dVector, Vector2iVector, Vector3dVector],
        octotree_obj_pcd: OctoTree,
        angle_deg: int,
    ) -> None:
        sizes = (
            frame,
            points.point_cloud.shape,
            downsample.shape,
            boxes.shape,
            # lines.shape,
            minmax.shape,
            valid_detects.shape,
            # clustering_labels.shape,
            # edge_result.edge_points.shape,
            # edge_result.edge_lines.shape,
            # edge_result.edge_length.shape,
            # np.array(disp_pcd[0]).shape,
            # np.array(disp_pcd[1]).shape,
            # np.array(disp_pcd[2]).shape,
            sum(len(value) for _, value in octotree_obj_pcd.entity_octonodes.items())
            if octotree_obj_pcd.entity_octonodes is not None
            else 0,
            angle_deg,
        )
        data = (
            points.point_cloud[:3].tolist(),
            downsample[:3].tolist(),
            boxes[:3].tolist(),
            # lines[:3].tolist(),
            minmax[:3].tolist(),
            valid_detects[0],
            # clustering_labels[:3].tolist(),
            # edge_result.edge_points[:3].tolist(),
            # edge_result.edge_lines[:3].tolist(),
            # edge_result.edge_length[:3].tolist(),
            # np.array(disp_pcd[0]).tolist(),
            # np.array(disp_pcd[1]).tolist(),
            # np.array(disp_pcd[2]).tolist(),
            angle_deg,
        )
        for i, (expected, actual) in enumerate(
            zip(self._sizes[self._count], sizes, strict=True)
        ):
            try:
                assert expected == actual
            except AssertionError:
                print(frame, i)
                print(expected)
                print(actual)
                raise
        for i, (expected, actual) in enumerate(
            zip(self._data[self._count], data, strict=True)
        ):
            try:
                assert expected == actual
            except AssertionError:
                print(frame, i)
                print(expected)
                print(actual)
                raise
        print(f"{frame}/{self._end}")
        self._count += 1
        assert self._count < self._end

    def import_(self) -> None:
        out_dir = Path("result")
        path: Path = out_dir / "test.pkl"
        with path.open("rb") as f:
            data: Tester = pickle.load(f)
        self._sizes = data._sizes
        self._data = data._data

    def export(self) -> None:
        out_dir = Path("result")
        out_dir.mkdir(parents=True, exist_ok=True)
        path: Path = out_dir / "test.pkl"
        with path.open("wb") as f:
            pickle.dump(self, f)
