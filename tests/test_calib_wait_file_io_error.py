# ruff: noqa: SLF001

from types import SimpleNamespace
from unittest.mock import MagicMock

import pytest

from argus_synchro.calibration_mat_generator_modules.ctrl import (
    calibcheck2d3d,
)
from argus_synchro.calibration_mat_generator_modules.ctrl import (
    wait_app as wait_module,
)
from argus_synchro.calibration_mat_generator_modules.ctrl.calibcheck2d3d import (
    calibcheck2d3d as calibcheck_class,
)
from argus_synchro.shared_errors import StateErrorDIndex


def _wait_process(camera_paths: list[str]) -> tuple[object, MagicMock]:
    diagnosis = MagicMock()
    diagnosis.errors_diagnosis.return_value = (True, False)
    process = object.__new__(wait_module.wait_app)
    process.sac = MagicMock()
    process.app_config_calib = MagicMock()
    process.calibcheck2d3d_conf = SimpleNamespace(
        camera_calib_files=camera_paths,
        new_axis_mode=False,
    )
    process._logger = MagicMock()
    process._ser = SimpleNamespace(
        state_errors_D={StateErrorDIndex.FILE_IO_ERROR: diagnosis}
    )
    return process, diagnosis


def test_wait_app_reports_camera_calibration_read_error_once(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    path = "camera0transmat.csv"
    process, diagnosis = _wait_process([path])
    monkeypatch.setattr(wait_module, "lidar_calib_filepath_loader", lambda **_: [])
    monkeypatch.setattr(
        wait_module,
        "read_rtvec",
        MagicMock(side_effect=EOFError("truncated numpy file")),
    )

    with pytest.raises(EOFError, match="truncated numpy file"):
        process.input_settings()

    diagnosis.errors_diagnosis.assert_called_once_with(True)
    diagnosis.log_output.assert_called_once_with(
        True,
        False,
        StateErrorDIndex.FILE_IO_ERROR,
        path,
        "read wait_app camera calibration",
        "EOFError: truncated numpy file",
    )


def test_wait_app_reports_lidar_calibration_read_error_once(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    path = "lidar2crane_trans_mat_0.csv"
    error = OSError("permission denied")
    process, diagnosis = _wait_process([])
    loadtxt = MagicMock(side_effect=error)
    monkeypatch.setattr(
        wait_module, "lidar_calib_filepath_loader", lambda **_: [path]
    )
    monkeypatch.setattr(wait_module.np, "loadtxt", loadtxt)

    with pytest.raises(OSError, match="permission denied"):
        process.input_settings()

    loadtxt.assert_called_once_with(path, delimiter=",")
    diagnosis.errors_diagnosis.assert_called_once_with(True)
    diagnosis.log_output.assert_called_once_with(
        True,
        False,
        StateErrorDIndex.FILE_IO_ERROR,
        path,
        "read wait_app LiDAR calibration CSV",
        "OSError: permission denied",
    )


def test_calibcheck_reports_eof_error_once(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    path = "camera0transmat.csv"
    error = EOFError("truncated numpy file")
    process = object.__new__(calibcheck_class)
    process.calibcheck2d3d_conf = SimpleNamespace(
        lidar_calib_files=[],
        camera_calib_files=[path],
        new_axis_mode=False,
    )
    process._report_file_io_error = MagicMock()
    monkeypatch.setattr(
        calibcheck2d3d,
        "read_rtvec",
        MagicMock(side_effect=error),
    )

    with pytest.raises(EOFError, match="truncated numpy file"):
        process.input_settings()

    process._report_file_io_error.assert_called_once_with(
        path,
        "read calibcheck2d3d camera calibration",
        error,
    )
