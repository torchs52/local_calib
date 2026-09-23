# ruff: noqa: SLF001

from __future__ import annotations

from typing import Any, cast

import numpy as np

import argus_synchro.calibration_mat_generator_modules.ctrl.calibcheck2d3d.tracker as tracker_module
from argus_synchro.calibration_mat_generator_modules.ctrl.calibcheck2d3d import (
    calibcheck2d3d,
    calibcheck2d_bboxtracker_recorder,
    calibcheck3d_bboxtracker_recorder,
)
from argus_synchro.calibration_mat_generator_modules.ctrl.calibration2d3d.track_main.interface_definition import (
    Tracking2dDataInterface,
    Tracking3dDataInterface,
    tracking2d_dataclass,
    tracking3d_dataclass,
)
from argus_synchro.diagnosis.calibcheck2d3d_result_diagnosis import (
    CalibCheck2d3dDiagnosis,
    CalibCheckFailureReason,
)


class _LoggerStub:
    def info(self, message: str) -> None:
        del message

    def warning(self, message: str) -> None:
        del message


class _SequenceLut:
    def __init__(self, values: list[float]) -> None:
        self._values = iter(values)

    def evaluate(self, x: float, y: float) -> float:
        del x, y
        return next(self._values)


class _TrackerStub:
    def __init__(self) -> None:
        self._results = iter(
            (
                (np.array([[2, 4, 6, 8, 0.9, 7]], dtype=float), True),
                (np.array([[1, 3, 9, 10, 0.9, 7]], dtype=float), False),
            )
        )
        self.reset_called = False

    def update(self, **kwargs: object) -> tuple[np.ndarray, bool]:
        del kwargs
        return next(self._results)

    def reset(self) -> None:
        self.reset_called = True


class _Tracker3dStub:
    def __init__(self) -> None:
        self._results = iter(
            (
                np.array([[2, 4, 6, 8, 1.0, 7]], dtype=float),
                np.array([[4, 6, 10, 12, 1.0, 7]], dtype=float),
            )
        )
        self.last_bbox_multi_minmax: np.ndarray | None = None
        self.reset_called = False

    def update(self, **kwargs: object) -> np.ndarray:
        del kwargs
        result = next(self._results)
        self.last_bbox_multi_minmax = result
        return result

    def reset(self) -> None:
        self.reset_called = True


def _controller() -> calibcheck2d3d:
    controller = cast(calibcheck2d3d, object.__new__(calibcheck2d3d))
    controller._logger = cast(Any, _LoggerStub())
    controller._evaluation_camera_count = 3
    controller.THRESH_3DBBOX_TRACKING_IDCOUNT = 1
    controller.THRESH_2DBBOX_TRACKING_IDCOUNT = 1
    controller.WARN_3D_TRACK_PROXIMITY_ENABLED = True
    controller.WARN_3D_TRACK_PROXIMITY_FAILS_VALIDATION = False
    controller.WARN_3D_TRACK_CENTER_DISTANCE_M = 1.0
    controller.WARN_3D_TRACK_MIN_CLOSE_FRAMES = 5
    controller.WARN_3D_TRACK_MIN_CLOSE_RATIO = 0.2
    controller._3d_track_proximity_warnings = []
    return controller


def test_2d_recorder_preserves_shi_tracking_statistics() -> None:
    recorder = cast(
        calibcheck2d_bboxtracker_recorder,
        object.__new__(calibcheck2d_bboxtracker_recorder),
    )
    tracker = _TrackerStub()
    recorder.mot_tracker = cast(Any, tracker)
    recorder.image_size_hw = (720, 1280)
    recorder.trackingID_data = {}
    recorder.trackingID_bboxlog = {}
    recorder.last_tracker_result = None
    recorder.lastframe_person_detected = False
    recorder.evLUT2D = cast(Any, _SequenceLut([0.8, 0.2]))
    recorder.evLUT2D_workarea = cast(Any, _SequenceLut([1.0, 0.0]))

    recorder.update([], frame_ix=10)
    assert recorder.is_person_detected() is True
    recorder.update([], frame_ix=11)

    metadata = recorder.trackingID_data[7]
    assert recorder.is_person_detected() is False
    assert metadata.xymin == (1.0, 3.0)
    assert metadata.xymax == (9.0, 10.0)
    assert metadata.frame_evval_min == 0.2
    assert metadata.frame_evval_max == 0.8
    assert metadata.workarea_count == 1
    assert metadata.frame_ix_lastmove == 11

    recorder.reset()
    assert tracker.reset_called is True
    assert recorder.last_tracker_result is None
    assert recorder.is_person_detected() is False


def test_3d_recorder_preserves_shi_evaluation_statistics() -> None:
    recorder = cast(
        calibcheck3d_bboxtracker_recorder,
        object.__new__(calibcheck3d_bboxtracker_recorder),
    )
    tracker = _Tracker3dStub()
    recorder.mot_tracker = cast(Any, tracker)
    recorder.trackingID_data = {}
    recorder.trackingID_bboxlog = {}
    recorder.last_tracker_result = None
    recorder.lastframe_person_detected = False
    recorder.data_array_fbb_point_history = []
    recorder.evLUT3D = cast(Any, _SequenceLut([0.9, 0.3]))
    recorder.evLUT3D_workarea = cast(Any, _SequenceLut([1.0, 0.0]))

    recorder.update(np.empty((0, 6)), frame_ix=10)
    recorder.update(np.empty((0, 6)), frame_ix=11)

    metadata = recorder.trackingID_data[7]
    assert metadata.frame_evval_min == 0.3
    assert metadata.frame_evval_max == 0.9
    assert metadata.workarea_count == 1
    assert metadata.frame_ix_lastmove == 11
    assert metadata.dist_from_camera_min < metadata.dist_from_camera_max
    assert len(recorder.get_data_array_fbb_point_history()) == 2

    recorder.reset()
    assert tracker.reset_called is True
    assert recorder.last_tracker_result is None
    assert recorder.is_person_detected() is False


def test_track_3d_rescans_recorded_frames_in_order(monkeypatch) -> None:
    updates: list[tuple[np.ndarray, int]] = []
    result = Tracking3dDataInterface({}, {})

    class _RecorderStub:
        def __init__(self, *, app_config_calib: object, camera_index: int) -> None:
            assert app_config_calib == "config"
            assert camera_index == 2

        def update(self, multi_minmax: np.ndarray, frame_ix: int) -> None:
            updates.append((multi_minmax, frame_ix))

        def get_rawresults(self) -> Tracking3dDataInterface:
            return result

    monkeypatch.setattr(
        tracker_module,
        "calibcheck3d_bboxtracker_recorder",
        _RecorderStub,
    )
    first = np.array([[1.0] * 6])
    second = np.array([[2.0] * 6])

    tracked = tracker_module.track_3dbbox(
        [([], first), ([], second)],
        app_config_calib=cast(Any, "config"),
        camera_index=2,
    )

    assert tracked is result
    assert [frame_ix for _, frame_ix in updates] == [0, 1]
    assert updates[0][0] is first
    assert updates[1][0] is second


def test_track_2d_rescans_each_camera_with_configured_image_size(monkeypatch) -> None:
    creations: list[tuple[int, tuple[int, int]]] = []
    updates: list[tuple[int, object, int]] = []
    results = [Tracking2dDataInterface({}, {}), Tracking2dDataInterface({}, {})]

    class _RecorderStub:
        def __init__(
            self,
            *,
            app_config_calib: object,
            image_size_hw: tuple[int, int],
            camera_index: int,
        ) -> None:
            assert app_config_calib == "config"
            self.camera_index = camera_index
            creations.append((camera_index, image_size_hw))

        def update(self, yoloresult_whole: object, frame_ix: int) -> None:
            updates.append((self.camera_index, yoloresult_whole, frame_ix))

        def get_rawresults(self) -> Tracking2dDataInterface:
            return results[self.camera_index]

    monkeypatch.setattr(
        tracker_module,
        "calibcheck2d_bboxtracker_recorder",
        _RecorderStub,
    )
    first = ["camera-0-frame-0", "camera-1-frame-0"]
    second = ["camera-0-frame-1", "camera-1-frame-1"]

    tracked = tracker_module.track_2dbbox(
        [(first, np.empty((0, 6))), (second, np.empty((0, 6)))],
        app_config_calib=cast(Any, "config"),
        image_size_hw=(720, 1280),
        camera_count=2,
    )

    assert tracked == results
    assert creations == [(0, (720, 1280)), (1, (720, 1280))]
    assert updates == [
        (0, "camera-0-frame-0", 0),
        (1, "camera-1-frame-0", 0),
        (0, "camera-0-frame-1", 1),
        (1, "camera-1-frame-1", 1),
    ]


def _tracking_3d(
    *, frames: int, movement: float, workarea_count: int
) -> Tracking3dDataInterface:
    metadata = tracking3d_dataclass(
        accum_track_length=movement,
        final_xy=(movement, 0.0),
        dist_from_camera_min=1.0,
        dist_from_camera_max=2.0,
        frame_ix_min=0,
        frame_ix_max=frames,
        frame_ix_lastmove=frames,
        frame_evval_min=0.0,
        frame_evval_max=1.0,
        workarea_count=workarea_count,
        is_alive=True,
    )
    return Tracking3dDataInterface({1: metadata}, {1: []})


def _tracking_2d(*, frames: int, movement: float) -> Tracking2dDataInterface:
    metadata = tracking2d_dataclass(
        accum_track_length=movement,
        final_xy=(movement, 0.0),
        xymin=(0.0, 0.0),
        xymax=(movement, 1.0),
        frame_ix_min=0,
        frame_ix_max=frames,
        frame_ix_lastmove=frames,
        frame_evval_min=0.0,
        frame_evval_max=1.0,
        workarea_count=frames,
        is_alive=True,
    )
    return Tracking2dDataInterface({1: metadata}, {1: []})


def test_tracking_validation_reports_reason_4_for_short_3d_track() -> None:
    controller = _controller()

    observation = controller.collect_tracking_observation(
        _tracking_3d(frames=29, movement=3.0, workarea_count=30),
        [_tracking_2d(frames=10, movement=50.0) for _ in range(3)],
    )
    diagnosis = CalibCheck2d3dDiagnosis(camera_count=3)

    assert diagnosis.diagnose_tracking(observation) == (False, False, False)
    assert [result.primary_reason for result in diagnosis.finalize([None] * 3)] == [
        CalibCheckFailureReason.TRACKING3D_INVALID,
        CalibCheckFailureReason.TRACKING3D_INVALID,
        CalibCheckFailureReason.TRACKING3D_INVALID,
    ]
    assert diagnosis.findings[0].details == {
        "total_tracks": 1,
        "alive_tracks": 0,
        "minimum_alive_tracks": 1,
        "proximity_warning_count": 0,
        "proximity_warning_fails_validation": False,
    }


def test_tracking_validation_reports_reason_5_per_camera() -> None:
    controller = _controller()

    observation = controller.collect_tracking_observation(
        _tracking_3d(frames=30, movement=3.0, workarea_count=30),
        [
            _tracking_2d(frames=10, movement=50.0),
            _tracking_2d(frames=9, movement=50.0),
            _tracking_2d(frames=10, movement=49.0),
        ],
    )
    diagnosis = CalibCheck2d3dDiagnosis(camera_count=3)

    assert diagnosis.diagnose_tracking(observation) == (True, False, False)
    assert [result.primary_reason for result in diagnosis.finalize([True] * 3)] == [
        CalibCheckFailureReason.NONE,
        CalibCheckFailureReason.TRACKING2D_INVALID,
        CalibCheckFailureReason.TRACKING2D_INVALID,
    ]
    assert diagnosis.findings[0].details == {
        "total_tracks": 1,
        "alive_tracks": 0,
        "minimum_alive_tracks": 1,
    }


def test_3d_tracking_validation_rejects_tracks_that_remain_too_close() -> None:
    controller = _controller()
    controller.WARN_3D_TRACK_PROXIMITY_FAILS_VALIDATION = True
    first = _tracking_3d(frames=30, movement=3.0, workarea_count=30)
    second_metadata = tracking3d_dataclass(
        accum_track_length=3.0,
        final_xy=(3.0, 0.5),
        dist_from_camera_min=1.0,
        dist_from_camera_max=2.0,
        frame_ix_min=0,
        frame_ix_max=30,
        frame_ix_lastmove=30,
        frame_evval_min=0.0,
        frame_evval_max=1.0,
        workarea_count=30,
        is_alive=True,
    )
    first.trackingIDmetadata[2] = second_metadata
    first.trackingIDbboxlog = {
        1: [(frame_ix, (0.0, 0.0, 1.0, 1.0)) for frame_ix in range(5)],
        2: [(frame_ix, (0.5, 0.0, 1.5, 1.0)) for frame_ix in range(5)],
    }

    observation = controller.collect_tracking_observation(
        first,
        [_tracking_2d(frames=10, movement=50.0) for _ in range(3)],
    )
    diagnosis = CalibCheck2d3dDiagnosis(camera_count=3)

    assert diagnosis.diagnose_tracking(observation) == (False, False, False)
    assert controller._3d_track_proximity_warnings[0]["close_frame_count"] == 5
