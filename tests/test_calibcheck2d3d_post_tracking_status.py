# ruff: noqa: SLF001

from __future__ import annotations

from pathlib import Path
from types import SimpleNamespace
from typing import Any, cast

from argus_synchro.calibration_mat_generator_modules.ctrl.calibcheck2d3d import (
    calibcheck2d3d,
)
from argus_synchro.calibration_mat_generator_modules.facade import CalibrationUIGodot
from argus_synchro.calibration_mat_generator_modules.ctrl.calibcheck2d3d.reporting import (
    write_calibcheck_result_files,
)
from argus_synchro.diagnosis.calibcheck2d3d_result_diagnosis import (
    CalibCheck2d3dDiagnosis,
    CalibCheckDiagnosisPhase,
    CalibCheckFailureReason,
)
from argus_synchro.shared_excepts import SharedExcepts


class _MonitorStub:
    def __init__(self) -> None:
        self.events: list[tuple[str, object]] = []

    def set_status_calibcommon(self, value: int) -> None:
        self.events.append(("common", value))

    def set_dummydata(self, **kwargs: bool) -> None:
        self.events.append(("dummy", kwargs))

    def set_yaw(self, value: float) -> None:
        self.events.append(("yaw", value))

    def set_camera_calibcheck_status(self, camera_id: int, value: int) -> None:
        self.events.append(("camera", (camera_id, int(value))))

    def transmit_setdata(self, **kwargs: object) -> None:
        self.events.append(("transmit", kwargs))


class _LoggerStub:
    def info(self, message: str) -> None:
        del message


def test_pre_sets_running_before_initial_mmap_transmit() -> None:
    controller = cast(calibcheck2d3d, object.__new__(calibcheck2d3d))
    controller._logger = cast(Any, _LoggerStub())
    controller.calibcheck2d3d_conf = SimpleNamespace(camera_count=2)
    controller.input_settings = lambda: None
    monitor = _MonitorStub()
    sec = cast(SharedExcepts, object())

    controller.pre_app_loopmain(
        cast(CalibrationUIGodot, monitor),
        sec,
        cast(Any, object()),
    )

    assert monitor.events == [
        ("common", 1),
        (
            "dummy",
            {
                "enable_systemerrorflag": True,
                "enable_errorflag": True,
                "overwrite_checkresult": True,
                "enable_yawangle": True,
            },
        ),
        (
            "transmit",
            {
                "sec": sec,
                "ref_t": None,
                "is_firstframe": True,
                "mmap_erase_rest": True,
            },
        ),
    ]
    assert controller.debug_index == 0
    assert controller.frame_info == []
    assert controller.read_count == 0
    assert controller._debug_video_writers == [None, None]


def test_post_uses_tracking_reasons_before_transmit(tmp_path: Path) -> None:
    controller = cast(calibcheck2d3d, object.__new__(calibcheck2d3d))
    controller._logger = cast(Any, _LoggerStub())
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
    diagnosis = CalibCheck2d3dDiagnosis(camera_count=3)
    diagnosis.record_reason(
        CalibCheckFailureReason.TRACKING3D_INVALID,
        camera_index=0,
        phase=CalibCheckDiagnosisPhase.TRACKING,
    )
    diagnosis.record_reason(
        CalibCheckFailureReason.TRACKING2D_INVALID,
        camera_index=1,
        phase=CalibCheckDiagnosisPhase.TRACKING,
    )
    controller.data_evaluation_process = lambda: diagnosis.finalize(
        [False, False, True]
    )
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
    controller._draw_evaluation_bboxes = lambda *_args: calls.append("draw")
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
    assert calls == ["lidar", "camera", "draw", "record", "video"]
    assert controller.debug_index == 5
    assert monitor.events[-1] == (
        "transmit",
        {"sec": sec, "ref_t": 4},
    )


def test_result_files_preserve_camera_specific_unknown_ok_and_ng(tmp_path: Path) -> None:
    resultfiles = [str(tmp_path / f"camera-{camera_ix}.txt") for camera_ix in range(3)]
    diagnosis = CalibCheck2d3dDiagnosis(camera_count=3)
    diagnosis.record_reason(
        CalibCheckFailureReason.TRACKING3D_INVALID,
        camera_index=0,
        phase=CalibCheckDiagnosisPhase.TRACKING,
    )

    write_calibcheck_result_files(
        resultfiles,
        diagnosis.finalize([True, True, False]),
        logger=_LoggerStub(),
    )

    assert [Path(resultfile).read_text().strip() for resultfile in resultfiles] == [
        "Unknown",
        "OK",
        "NG",
    ]


def test_send_end_wait_publishes_inactive_to_mmap() -> None:
    monitor = _MonitorStub()
    sec = cast(SharedExcepts, object())

    calibcheck2d3d.send_end_wait(sec, cast(Any, object()), cast(CalibrationUIGodot, monitor))

    assert monitor.events == [
        ("common", 0),
        (
            "dummy",
            {
                "enable_systemerrorflag": True,
                "enable_errorflag": True,
                "overwrite_checkresult": True,
                "enable_yawangle": True,
            },
        ),
        ("transmit", {"sec": sec, "ref_t": None}),
    ]
