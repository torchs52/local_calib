# ruff: noqa: SLF001

from __future__ import annotations

from typing import cast

import numpy as np

from argus_synchro.calibration_mat_generator_modules.ctrl.calibcheck2d3d import (
    calibcheck2d3d,
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
