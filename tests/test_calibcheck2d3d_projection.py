# ruff: noqa: SLF001

from __future__ import annotations

from types import SimpleNamespace
from typing import Any, cast

import numpy as np
import pytest

from argus_synchro.calibration_mat_generator_modules.ctrl import (
    calibcheck2d3d as module,
)
from argus_synchro.calibration_mat_generator_modules.ctrl.calibcheck2d3d import (
    calibcheck2d3d,
    debuginfo_and_functions,
)


def _controller() -> calibcheck2d3d:
    controller = cast(calibcheck2d3d, object.__new__(calibcheck2d3d))
    controller.width = 100
    controller.height = 100
    controller._evaluation_intrinsics = np.array(
        [[100.0, 0.0, 50.0], [0.0, 100.0, 50.0], [0.0, 0.0, 1.0]],
        dtype=np.float32,
    )
    controller.rtvec_mat = [
        (
            np.zeros((3, 1), dtype=np.float64),
            np.zeros((3, 1), dtype=np.float64),
            np.eye(4, dtype=np.float64),
        )
    ]
    return controller


def test_project_3dbbox_returns_expected_image_bounds() -> None:
    projected = _controller().project_3dbbox_core(
        np.array([-1.0, -1.0, 5.0, 1.0, 1.0, 10.0], dtype=np.float32),
        camera_index=0,
        require_points_in_image=True,
    )

    np.testing.assert_allclose(projected, [30.0, 30.0, 70.0, 70.0])


def test_project_3dbbox_rejects_bbox_behind_camera() -> None:
    projected = _controller().project_3dbbox_core(
        np.array([-1.0, -1.0, -2.0, 1.0, 1.0, -1.0], dtype=np.float32),
        camera_index=0,
    )

    assert projected is None


def test_project_3dbbox_rejects_bbox_outside_image_when_required() -> None:
    projected = _controller().project_3dbbox_core(
        np.array([10.0, -1.0, 5.0, 12.0, 1.0, 10.0], dtype=np.float32),
        camera_index=0,
        require_points_in_image=True,
    )

    assert projected is None


def test_bbox_geometry_gates_match_shi_behavior() -> None:
    first = np.array([10.0, 10.0, 30.0, 30.0], dtype=np.float32)
    overlapping = np.array([19.0, 19.0, 39.0, 39.0], dtype=np.float32)
    distant = np.array([60.0, 60.0, 80.0, 80.0], dtype=np.float32)

    assert calibcheck2d3d._has_positive_2d_intersection(first, overlapping)
    assert not calibcheck2d3d._has_positive_2d_intersection(first, distant)
    assert calibcheck2d3d._passes_center_diff_gate(first, overlapping)
    assert not calibcheck2d3d._passes_center_diff_gate(first, distant)


def test_proc_lidar_uses_dev_capture3d_preprocessing(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    controller = cast(calibcheck2d3d, object.__new__(calibcheck2d3d))
    captured_points = np.array([[1.0, -2.0, -3.0], [2.0, -3.0, -4.0]], dtype=np.float64)
    read_calls: list[tuple[object, bool, bool]] = []

    def read(
        lidar_data: object,
        *,
        dontread: bool,
        adjust_coordinate_enable: bool,
    ) -> tuple[np.ndarray, None]:
        read_calls.append((lidar_data, dontread, adjust_coordinate_enable))
        return captured_points.copy(), None

    controller.cap3d = SimpleNamespace(read=read)
    controller.calibcheck2d3d_conf = SimpleNamespace(z_threshold=-10.0)
    controller.DATARANGE_XYZ = ((-10, 20), (-20, 20), (-2, 2))
    controller._logger = SimpleNamespace(info=lambda _message: None)
    filtered_inputs: list[np.ndarray] = []
    controller._sub_detect_apply_static_point_filter = lambda pcdframe, timestamp_pcd: (
        filtered_inputs.append(pcdframe.copy()) or pcdframe
    )
    controller._sub_proc_3d_monitor_data = lambda **_kwargs: None
    monkeypatch.setattr(
        module,
        "internal_make_BB",
        lambda _points, **_ranges: (
            (
                np.empty((0, 3), dtype=np.float32),
                np.empty((0, 2), dtype=np.float32),
                np.empty((0, 6), dtype=np.float64),
            ),
            np.empty((0, 3), dtype=np.float32),
            None,
        ),
    )
    monkeypatch.setattr(
        module.utils3d,
        "np_to_pcd",
        lambda points: SimpleNamespace(
            voxel_down_sample=lambda _size: SimpleNamespace(points=points)
        ),
    )
    lidar_datalist = [(np.ones((1, 4), dtype=np.float64), 1, 0.1)]

    result = controller.proc_lidar1f(
        cast(Any, lidar_datalist),
        framecounter=12,
        monitor=cast(Any, object()),
    )

    assert read_calls == [(lidar_datalist, False, False)]
    np.testing.assert_array_equal(
        filtered_inputs[0],
        np.array([[1.0, 2.0, 3.0], [2.0, 3.0, 4.0]], dtype=np.float64),
    )
    assert result.shape == (0, 6)


def test_internal_make_bb_uses_requested_data_ranges(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    requested_ranges: list[
        tuple[tuple[float, float], tuple[float, float], tuple[float, float]]
    ] = []

    def set_xyz_range(
        *,
        pcd_data: np.ndarray,
        x_range: tuple[float, float],
        y_range: tuple[float, float],
        z_range: tuple[float, float],
    ) -> np.ndarray:
        requested_ranges.append((x_range, y_range, z_range))
        return pcd_data

    monkeypatch.setattr(debuginfo_and_functions, "set_xyz_range", set_xyz_range)
    points = np.empty((0, 3), dtype=np.float32)

    debuginfo_and_functions.internal_make_BB(
        points,
        x_range=(-10, 20),
        y_range=(-20, 20),
        z_range=(-2, 2),
    )

    assert requested_ranges == [((-10, 20), (-20, 20), (-2, 2))]
