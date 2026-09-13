# ruff: noqa: SLF001

from __future__ import annotations

from pathlib import Path
from types import SimpleNamespace
from typing import Any, cast

import cv2
import numpy as np
import pytest

from argus_synchro.calibration_mat_generator_modules.ctrl.calibcheck2d3d import (
    calibcheck2d3d,
)
from argus_synchro.calibration_mat_generator_modules.facade import CalibrationUIGodot
from argus_synchro.diagnosis.calibcheck2d3d_result_diagnosis import (
    CameraCalibCheckStatusDiagnosis,
)
from argus_synchro.shared_excepts import SharedExcepts


class _MonitorStub:
    def __init__(self) -> None:
        self.events: list[tuple[str, object]] = []
        self.cameradata: list[np.ndarray | None] = []

    def set_status_calibcommon(self, value: int) -> None:
        self.events.append(("common", value))

    def set_dummydata(self, **kwargs: bool) -> None:
        self.events.append(("dummy", kwargs))

    def set_camera_calibcheck_status(self, camera_id: int, value: int) -> None:
        self.events.append(("camera", (camera_id, int(value))))

    def set_yaw(self, value: float) -> None:
        self.events.append(("yaw", value))

    def transmit_setdata(self, **kwargs: object) -> None:
        self.events.append(("transmit", kwargs))


class _LoggerStub:
    def info(self, message: str) -> None:
        del message


def test_post_uses_tracking_reasons_before_transmit(tmp_path: Path) -> None:
    controller = cast(calibcheck2d3d, object.__new__(calibcheck2d3d))
    controller._logger = cast(Any, _LoggerStub())
    controller._calibcheck_status_diagnosis = CameraCalibCheckStatusDiagnosis()
    controller._evaluation_camera_count = 3
    controller.camera_scores_rawdata = [[1.0], [1.0], [1.0]]
    controller.checked_points3d = []
    controller.checked_points3d_score = []
    controller.checked_points2d = []
    controller.checked_points2d_score = []
    controller.DEBUG_CALIBCHECK_ENABLED = False
    controller._debug_video_writers = []
    controller._debug_eval_info = {}
    controller.calibcheck2d3d_conf = SimpleNamespace(
        score_accept_count_threshold=1,
        score_value_threshold=0.5,
        resultfiles=[str(tmp_path / f"camera-{index}.txt") for index in range(3)],
    )
    controller.app_config_calib = SimpleNamespace(
        default=SimpleNamespace(outputdir_root=str(tmp_path))
    )
    controller.data_evaluation_process = lambda: ([4, 5, 0], [False, False, True])
    monitor = _MonitorStub()

    controller.post_app_loopmain(
        cast(CalibrationUIGodot, monitor),
        cast(SharedExcepts, object()),
        cast(Any, object()),
    )

    camera_events = [event for event in monitor.events if event[0] == "camera"]
    assert camera_events == [
        ("camera", (0, 5)),
        ("camera", (1, 5)),
        ("camera", (2, 0)),
    ]
    assert monitor.events.index(camera_events[-1]) < next(
        index for index, event in enumerate(monitor.events) if event[0] == "transmit"
    )


def test_app_loop_collects_frame_before_transmit() -> None:
    controller = cast(calibcheck2d3d, object.__new__(calibcheck2d3d))
    controller._logger = cast(Any, _LoggerStub())
    controller.debug_index = 4
    calls: list[str] = []
    controller.input_data_diagnosis = lambda *_args: False
    controller.proc_lidar1f = lambda *_args: calls.append("lidar") or "bbox3d"
    controller.proc_camera1f = lambda *_args: calls.append("camera") or "bbox2d"
    controller._draw_evaluation_bboxes = lambda *_args: calls.append("draw3d")
    controller.record_bbox1f = lambda *_args: calls.append("record")
    controller._write_debug_video_frames = lambda *_args: calls.append("video")
    monitor = _MonitorStub()
    sec = cast(SharedExcepts, object())

    result = controller.app_loopmain(
        cast(Any, ([], [], (0, 0.0), 123)),
        cast(CalibrationUIGodot, monitor),
        sec,
        cast(Any, object()),
    )

    assert result is True
    assert calls == ["lidar", "camera", "draw3d", "record", "video"]
    assert controller.debug_index == 5
    assert monitor.events[-1] == (
        "transmit",
        {"sec": sec, "ref_t": 4},
    )


def test_draw_evaluation_bboxes_renders_real_and_virtual_candidates(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    controller = cast(calibcheck2d3d, object.__new__(calibcheck2d3d))
    controller.EVAL_ZVALUES = (-1.0, 2.0)
    controller.VIRTUAL_BBOX_XY_OFFSETS = (
        (0.0, 0.0),
        (-3.0, 0.0),
        (3.0, 0.0),
        (0.0, -3.0),
        (0.0, 3.0),
    )
    controller.rtvec_mat = [
        (
            np.zeros((3, 1), dtype=np.float64),
            np.zeros((3, 1), dtype=np.float64),
            np.eye(4, dtype=np.float64),
        )
    ]
    controller.ud = SimpleNamespace(ncm1=np.eye(3, dtype=np.float32))
    controller.project_3dbbox_core = lambda *_args: np.array(
        [10.0, 10.0, 20.0, 20.0], dtype=np.float32
    )
    monitor = _MonitorStub()
    monitor.cameradata = [np.zeros((100, 100, 3), dtype=np.uint8)]
    projected_candidates: list[np.ndarray] = []
    line_colors: list[tuple[int, int, int]] = []

    def project_points(points: np.ndarray, *_args: object) -> tuple[np.ndarray, None]:
        projected_candidates.append(points.copy())
        projected = np.arange(16, dtype=np.float32).reshape(8, 1, 2)
        return projected, None

    def draw_line(
        _frame: np.ndarray,
        _pt1: list[int],
        _pt2: list[int],
        color: tuple[int, int, int],
        _thickness: int,
    ) -> None:
        line_colors.append(color)

    monkeypatch.setattr(cv2, "projectPoints", project_points)
    monkeypatch.setattr(cv2, "line", draw_line)

    controller._draw_evaluation_bboxes(
        cast(CalibrationUIGodot, monitor),
        np.array([[1.0, 2.0, 3.0, 4.0, -0.5, 1.5]], dtype=np.float64),
    )

    assert len(projected_candidates) == 6
    assert line_colors.count((0, 255, 0)) == 9
    assert line_colors.count((0, 165, 255)) == 9
    assert line_colors.count((255, 255, 0)) == 36
    assert [candidate[:, 0].min() for candidate in projected_candidates[2:]] == [
        -2.0,
        4.0,
        1.0,
        1.0,
    ]


def test_draw_evaluation_bboxes_uses_each_camera_extrinsics(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    controller = cast(calibcheck2d3d, object.__new__(calibcheck2d3d))
    controller.EVAL_ZVALUES = (-1.0, 2.0)
    controller.VIRTUAL_BBOX_XY_OFFSETS = (
        (0.0, 0.0),
        (-3.0, 0.0),
        (3.0, 0.0),
        (0.0, -3.0),
        (0.0, 3.0),
    )
    camera_rvecs = [
        np.full((3, 1), camera_ix, dtype=np.float64) for camera_ix in range(2)
    ]
    controller.rtvec_mat = [
        (
            camera_rvec,
            np.zeros((3, 1), dtype=np.float64),
            np.eye(4, dtype=np.float64),
        )
        for camera_rvec in camera_rvecs
    ]
    controller.ud = SimpleNamespace(ncm1=np.eye(3, dtype=np.float32))
    projected_camera_indices: list[int] = []
    controller.project_3dbbox_core = lambda _bbox, camera_ix: (
        projected_camera_indices.append(camera_ix)
        or np.array([10.0, 10.0, 20.0, 20.0], dtype=np.float32)
    )
    monitor = _MonitorStub()
    monitor.cameradata = [
        np.zeros((100, 100, 3), dtype=np.uint8),
        np.zeros((100, 100, 3), dtype=np.uint8),
    ]
    projected_rvecs: list[np.ndarray] = []

    def project_points(
        _points: np.ndarray,
        rvec: np.ndarray,
        *_args: object,
    ) -> tuple[np.ndarray, None]:
        projected_rvecs.append(rvec.copy())
        projected = np.arange(16, dtype=np.float32).reshape(8, 1, 2)
        return projected, None

    monkeypatch.setattr(cv2, "projectPoints", project_points)
    monkeypatch.setattr(cv2, "line", lambda *_args: None)

    controller._draw_evaluation_bboxes(
        cast(CalibrationUIGodot, monitor),
        np.array([[1.0, 2.0, 3.0, 4.0, -0.5, 1.5]], dtype=np.float64),
    )

    assert projected_camera_indices == [0] * 6 + [1] * 6
    assert all(np.array_equal(rvec, camera_rvecs[0]) for rvec in projected_rvecs[:6])
    assert all(np.array_equal(rvec, camera_rvecs[1]) for rvec in projected_rvecs[6:])
