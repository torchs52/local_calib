from __future__ import annotations

import importlib
from types import SimpleNamespace
from unittest.mock import MagicMock

from argus_synchro.calibration_mat_generator_modules.ctrl.data_capture.datacapture_local import (
    datacapture_class,
)
from argus_synchro.calibration_mat_generator_modules.ctrl.data_capture.lidar_capture import (
    MultiLidarWorker,
)
from argus_synchro.calibration_mat_generator_modules.ctrl.data_capture.lidar_capture.lidar_capture_tool import (
    read_singleLidar_file,
)
from argus_synchro.diagnosis.error_diagnosis import ResultDiagnosis
from argus_synchro.shared_errors import StateErrorDIndex


class _RecordingDiagnosis:
    def __init__(self) -> None:
        self.diagnosis_calls: list[tuple[object, ...]] = []
        self.log_calls: list[tuple[object, ...]] = []

    def errors_diagnosis(
        self, *args: object
    ) -> tuple[ResultDiagnosis, ResultDiagnosis]:
        self.diagnosis_calls.append(args)
        return ResultDiagnosis.DETECTION, ResultDiagnosis.NORMAL

    def log_output(self, *args: object) -> None:
        self.log_calls.append(args)


def _shared_errors() -> tuple[SimpleNamespace, _RecordingDiagnosis]:
    diagnosis = _RecordingDiagnosis()
    state_errors_d: list[object | None] = [None] * (
        StateErrorDIndex.FILE_IO_ERROR + 1
    )
    state_errors_d[StateErrorDIndex.FILE_IO_ERROR] = diagnosis
    return SimpleNamespace(state_errors_D=state_errors_d), diagnosis


def test_data_capture_passes_shared_errors_to_lidar_manager(monkeypatch) -> None:
    data_capture_module = importlib.import_module(
        "argus_synchro.calibration_mat_generator_modules.ctrl.data_capture"
    )
    camera_manager = MagicMock()
    lidar_manager = MagicMock()
    monkeypatch.setattr(data_capture_module, "MultiCameraManager", camera_manager)
    monkeypatch.setattr(data_capture_module, "MultiLidarManager", lidar_manager)
    monkeypatch.setattr(data_capture_module, "sensor_sync_filter", MagicMock())
    logger_factory = MagicMock()
    shared_errors = MagicMock()
    default_config = SimpleNamespace(print_disabled=True)
    capture_config = SimpleNamespace(sync_type=0)

    capture = data_capture_module.data_capture(
        DefaultConfig=default_config,
        DataCaptureConfig=capture_config,
        sec=MagicMock(),
        sac=MagicMock(),
        verbose=False,
        app_logger_factory=logger_factory,
        shared_errors=shared_errors,
    )

    assert "shared_errors" not in camera_manager.call_args.kwargs
    assert lidar_manager.call_args.kwargs["shared_errors"] is shared_errors
    capture.force_close()


def test_missing_lidar_npy_reports_file_io_error_details(tmp_path) -> None:
    reports: list[tuple[str, str, Exception]] = []
    name_prefix = str(tmp_path / "lidar_")

    result = read_singleLidar_file(
        12,
        name_prefix,
        file_io_error_reporter=lambda path, operation, error: reports.append(
            (path, operation, error)
        ),
    )

    assert result is None
    assert reports[0][:2] == (
        f"{name_prefix}000012.npy",
        "read data_capture LiDAR point NPY",
    )
    assert isinstance(reports[0][2], OSError)


def test_lidar_worker_reports_importance_d_file_io_error() -> None:
    shared_errors, diagnosis = _shared_errors()
    worker = object.__new__(MultiLidarWorker)
    worker._ser = shared_errors

    worker._report_file_io_error(
        "/data/lidar_000012.npy",
        "read data_capture LiDAR point NPY",
        OSError("read failed"),
    )

    assert diagnosis.diagnosis_calls == [(True,)]
    assert diagnosis.log_calls == [
        (
            ResultDiagnosis.DETECTION,
            ResultDiagnosis.NORMAL,
            StateErrorDIndex.FILE_IO_ERROR,
            "/data/lidar_000012.npy",
            "read data_capture LiDAR point NPY",
            "OSError: read failed",
        )
    ]


def test_local_capture_reports_importance_d_file_io_error() -> None:
    shared_errors, diagnosis = _shared_errors()
    capture = object.__new__(datacapture_class)
    capture._ser = shared_errors

    capture._report_file_io_error(
        "/config/lidar_coordinate.json",
        "read LiDAR coordinate conversion JSON",
        ValueError("invalid data"),
    )

    assert diagnosis.diagnosis_calls == [(True,)]
    assert diagnosis.log_calls == [
        (
            ResultDiagnosis.DETECTION,
            ResultDiagnosis.NORMAL,
            StateErrorDIndex.FILE_IO_ERROR,
            "/config/lidar_coordinate.json",
            "read LiDAR coordinate conversion JSON",
            "ValueError: invalid data",
        )
    ]