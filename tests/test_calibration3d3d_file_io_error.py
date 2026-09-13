from pathlib import Path
from types import SimpleNamespace
from unittest.mock import MagicMock

import numpy as np
import pytest

from argus_synchro.calibration_mat_generator_modules.ctrl.calibration3d3d import (
    calibration3d3d_class,
)
from argus_synchro.calibration_mat_generator_modules.ctrl.calibration3d3d.calib_lidars import (
    _write_calibration_csv,
)
from argus_synchro.calibration_mat_generator_modules.ctrl.calibration3d3d.simulate_lidar_points import (
    _read_angles_deg,
    _read_mat4_csv,
    load_crane_profiles,
)
from argus_synchro.diagnosis.error_diagnosis import ResultDiagnosis
from argus_synchro.shared_errors import StateErrorDIndex


@pytest.mark.parametrize(
    ("reader", "operation"),
    (
        (_read_mat4_csv, "read 3D-3D matrix CSV"),
        (load_crane_profiles, "read 3D-3D crane profile JSON"),
    ),
)
def test_3d3d_file_reader_reports_and_reraises(
    tmp_path: Path, reader, operation: str
) -> None:
    missing_path = tmp_path / "missing"
    reports: list[tuple[str, str, Exception]] = []

    with pytest.raises(OSError):
        reader(
            missing_path,
            file_io_error_reporter=lambda path, action, error: reports.append(
                (path, action, error)
            ),
        )

    assert reports[0][:2] == (str(missing_path), operation)
    assert isinstance(reports[0][2], OSError)


def test_3d3d_angle_reader_reports_and_reraises(tmp_path: Path) -> None:
    missing_path = tmp_path / "missing.csv"
    reports: list[tuple[str, str, Exception]] = []

    with pytest.raises(OSError):
        _read_angles_deg(
            missing_path,
            None,
            None,
            100,
            lambda path, operation, error: reports.append((path, operation, error)),
        )

    assert reports[0][:2] == (str(missing_path), "read 3D-3D angle CSV")
    assert isinstance(reports[0][2], OSError)


def test_calibration3d3d_file_io_reporter_uses_shared_diagnosis() -> None:
    diagnosis = MagicMock()
    diagnosis.errors_diagnosis.return_value = (
        ResultDiagnosis.DETECTION,
        ResultDiagnosis.NORMAL,
    )
    calibration = object.__new__(calibration3d3d_class)
    state_errors = [None] * (StateErrorDIndex.FILE_IO_ERROR + 1)
    state_errors[StateErrorDIndex.FILE_IO_ERROR] = diagnosis
    calibration._ser = SimpleNamespace(state_errors_D=state_errors)

    calibration._report_file_io_error(
        "/config/lidar.csv", "read 3D-3D matrix CSV", OSError("read failed")
    )

    diagnosis.errors_diagnosis.assert_called_once_with(True)
    diagnosis.log_output.assert_called_once_with(
        ResultDiagnosis.DETECTION,
        ResultDiagnosis.NORMAL,
        StateErrorDIndex.FILE_IO_ERROR,
        "/config/lidar.csv",
        "read 3D-3D matrix CSV",
        "OSError: read failed",
    )


def test_3d3d_result_write_error_is_reported_and_reraised(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    reports: list[tuple[str, str, Exception]] = []
    output_path = tmp_path / "result.csv"
    write_error = OSError("write failed")
    monkeypatch.setattr(
        "argus_synchro.calibration_mat_generator_modules.ctrl.calibration3d3d.calib_lidars.pd.DataFrame.to_csv",
        MagicMock(side_effect=write_error),
    )

    with pytest.raises(OSError, match="write failed"):
        _write_calibration_csv(
            str(output_path),
            np.eye(4),
            lambda path, operation, error: reports.append(
                (path, operation, error)
            ),
        )

    assert reports == [
        (
            str(output_path),
            "write 3D-3D calibration result matrix CSV",
            write_error,
        )
    ]