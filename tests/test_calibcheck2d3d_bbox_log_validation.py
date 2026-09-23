# ruff: noqa: PLR2004, SLF001

from __future__ import annotations

from types import SimpleNamespace
from typing import Any, cast

import numpy as np
import pytest

import argus_synchro.calibration_mat_generator_modules.ctrl.calibcheck2d3d as calibcheck_module
from argus_synchro.calibration_mat_generator_modules.ctrl.calibcheck2d3d.processor import (
    lidar_points_to_ui_data,
    yolo_result_to_ui_bboxes,
)
from argus_synchro.calibration_mat_generator_modules.ctrl.calibcheck2d3d import (
    calibcheck2d3d,
)
from argus_synchro.diagnosis.calibcheck2d3d_result_diagnosis import (
    CalibCheck2d3dDiagnosis,
    CalibCheckFailureReason,
)


class _LoggerStub:
    def info(self, message: str) -> None:
        del message


def _controller() -> calibcheck2d3d:
    controller = cast(calibcheck2d3d, object.__new__(calibcheck2d3d))
    controller._logger = cast(Any, _LoggerStub())
    controller._evaluation_camera_count = 3
    controller.FRAME_INFO_MAXLEN = 10
    controller.THRESH_3DBBOX_COUNT_PER_FRAME = 1
    controller.THRESH_3DBBOX_COUNT_MEAN_RATIO = 0.5
    controller.THRESH_2DBBOX_COUNT_PER_FRAME = 1
    controller.THRESH_2DBBOX_COUNT_MEAN_RATIO = 0.5
    controller.frame_info = []
    return controller


def _yoloresult(valid_count: int) -> list[np.ndarray]:
    return [
        np.zeros((1, 4), dtype=np.float32),
        np.zeros(1, dtype=np.float32),
        np.zeros(1, dtype=np.float32),
        np.array(valid_count, dtype=np.int32),
    ]


def test_bbox_log_validation_reports_missing_3d_for_all_cameras() -> None:
    controller = _controller()
    controller.record_bbox1f(
        np.zeros((0, 6), dtype=np.float64),
        [_yoloresult(1), _yoloresult(1), _yoloresult(1)],
    )

    diagnosis = CalibCheck2d3dDiagnosis(camera_count=3)
    validity = diagnosis.diagnose_bbox_logs(
        controller.collect_bbox_log_observation()
    )

    assert validity == (False, False, False)
    assert [result.primary_reason for result in diagnosis.finalize([None] * 3)] == [
        CalibCheckFailureReason.BBOX3D_LOG_INVALID,
        CalibCheckFailureReason.BBOX3D_LOG_INVALID,
        CalibCheckFailureReason.BBOX3D_LOG_INVALID,
    ]


def test_bbox_log_validation_reports_missing_2d_per_camera() -> None:
    controller = _controller()
    controller.record_bbox1f(
        np.ones((1, 6), dtype=np.float64),
        [_yoloresult(1), _yoloresult(0), _yoloresult(1)],
    )

    diagnosis = CalibCheck2d3dDiagnosis(camera_count=3)
    validity = diagnosis.diagnose_bbox_logs(
        controller.collect_bbox_log_observation()
    )

    assert validity == (True, False, True)
    assert [result.primary_reason for result in diagnosis.finalize([True] * 3)] == [
        CalibCheckFailureReason.NONE,
        CalibCheckFailureReason.BBOX2D_LOG_INVALID,
        CalibCheckFailureReason.NONE,
    ]


def test_bbox_log_recording_keeps_configured_tail() -> None:
    controller = _controller()
    controller.FRAME_INFO_MAXLEN = 2
    for valid_count in range(3):
        controller.record_bbox1f(
            np.ones((1, 6), dtype=np.float64),
            [_yoloresult(valid_count)] * 3,
        )

    assert len(controller.frame_info) == 2
    assert int(controller.frame_info[0][0][0][3]) == 1
    assert int(controller.frame_info[1][0][0][3]) == 2


def test_yolo_result_is_converted_to_ui_pixel_bbox_coordinates() -> None:
    yoloresult = [
        np.array([[0.1, 0.2, 0.5, 0.8]], dtype=np.float32),
        np.array([0.9], dtype=np.float32),
        np.array([0], dtype=np.float32),
        np.array(1, dtype=np.int32),
    ]

    bboxes = yolo_result_to_ui_bboxes(
        yoloresult,
        image_height=100,
        image_width=200,
    )

    np.testing.assert_array_equal(bboxes, [[40, 10, 160, 50]])


def test_lidar_points_are_prepared_for_ui_by_available_columns() -> None:
    intensity_points = np.array([[1.0, 2.0, 3.0, 0.7]], dtype=np.float32)
    points, colors = lidar_points_to_ui_data(
        intensity_points,
        intensity_to_color=lambda intensity: intensity.reshape(-1, 1) * 2,
    )
    np.testing.assert_array_equal(points, [[1.0, 2.0, 3.0]])
    np.testing.assert_allclose(colors, [[1.4]])

    xyz_points = np.array([[4.0, 5.0, 6.0]], dtype=np.float32)
    points, colors = lidar_points_to_ui_data(
        xyz_points,
        intensity_to_color=lambda _: pytest.fail("intensity is not available"),
    )
    np.testing.assert_array_equal(points, xyz_points)
    np.testing.assert_allclose(colors, [[0.2, 0.2, 0.2]])


def test_reason_mappings_match_shi_contract() -> None:
    controller = _controller()

    assert controller.error_reason_to_string(11) == "カメラとLiDARの評価可能な対象物なし"
    assert [controller.error_reason_to_ui_errornum(reason) for reason in range(1, 12)] == [
        6,
        3,
        3,
        5,
        5,
        7,
        7,
        7,
        4,
        3,
        4,
    ]


def test_input_settings_reports_lidar_calibration_read_error(monkeypatch) -> None:
    controller = _controller()
    controller.calibcheck2d3d_conf = SimpleNamespace(
        lidar_calib_files=["missing-lidar.csv"],
        camera_calib_files=[],
    )
    reports: list[tuple[str, str, Exception]] = []
    controller._report_file_io_error = (
        lambda file_path, operation, error: reports.append(
            (file_path, operation, error)
        )
    )

    error = OSError("missing calibration")

    def raise_read_error(*_args, **_kwargs):
        raise error

    monkeypatch.setattr(np, "loadtxt", raise_read_error)

    with pytest.raises(OSError, match="missing calibration"):
        controller.input_settings()

    assert reports == [
        (
            "missing-lidar.csv",
            "read calibcheck2d3d LiDAR calibration CSV",
            error,
        )
    ]


def test_static_point_filter_is_applied_when_enabled() -> None:
    controller = _controller()
    controller.app_config_calib = SimpleNamespace(
        calib2d3d=SimpleNamespace(
            Proc3d=SimpleNamespace(
                enable_static_point_filter=True,
                static_point_filter_initlength=1,
                static_point_filter_refresh_period=10,
            )
        ),
        default=SimpleNamespace(print_disabled=True),
    )
    filtered = np.array([[1.0, 2.0, 3.0]], dtype=np.float32)

    class _StaticFilterStub:
        filtersource_framecount = 0

        def add_single_voxel_map(self, frame) -> None:
            del frame
            self.filtersource_framecount += 1

        def apply_voxelfilter(self) -> None:
            return None

        def extract_moving_objects(self, frame):
            del frame
            return filtered

    controller.static_point_filter = _StaticFilterStub()
    controller.pointfilter_lastadd = -1

    result = controller._sub_detect_apply_static_point_filter(
        np.zeros((1, 3), dtype=np.float32),
        timestamp_pcd=0,
    )

    assert result is filtered
    assert controller.static_point_filter.filtersource_framecount == 1


def test_input_settings_reports_camera_calibration_read_error(monkeypatch) -> None:
    controller = _controller()
    controller.calibcheck2d3d_conf = SimpleNamespace(
        lidar_calib_files=[],
        camera_calib_files=["missing-camera.csv"],
        new_axis_mode=True,
    )
    reports: list[tuple[str, str, Exception]] = []
    controller._report_file_io_error = (
        lambda file_path, operation, error: reports.append(
            (file_path, operation, error)
        )
    )
    error = RuntimeError("missing calibration")

    def raise_read_error(**_kwargs):
        raise error

    monkeypatch.setattr(calibcheck_module, "read_rtvec", raise_read_error)

    with pytest.raises(RuntimeError, match="missing calibration"):
        controller.input_settings()

    assert reports == [
        (
            "missing-camera.csv",
            "read calibcheck2d3d camera calibration",
            error,
        )
    ]
