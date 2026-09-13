import json
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import MagicMock

import numpy as np

from argus_synchro.diagnosis.action_errors import CameraXCalibDataInvalidDiagnosis
from argus_synchro.diagnosis.error_config import CameraNCalibDataInvalidParameters
from argus_synchro.shared_errors import ActionErrorIndex


def _write_fisheye(path: Path) -> None:
    path.write_text(
        json.dumps(
            {
                "image_width": 1920,
                "image_height": 1080,
                "camera_matrix": {"rows": 3, "cols": 3, "data": np.eye(3).ravel().tolist()},
                "new_camera_matrix_alpha1": {
                    "rows": 3,
                    "cols": 3,
                    "data": np.eye(3).ravel().tolist(),
                },
                "k1": 0.0,
                "k2": 0.0,
                "k3": 0.0,
                "k4": 0.0,
            }
        ),
        encoding="utf-8",
    )


def _diagnosis() -> CameraXCalibDataInvalidDiagnosis:
    diagnosis = CameraXCalibDataInvalidDiagnosis()
    diagnosis.param = CameraNCalibDataInvalidParameters()
    diagnosis.is_enabled = True
    return diagnosis


def test_camera_calibration_accepts_valid_intrinsics_and_extrinsic(tmp_path: Path) -> None:
    fisheye = tmp_path / "fisheye.json"
    extrinsic = tmp_path / "camera0.csv"
    _write_fisheye(fisheye)
    np.savetxt(extrinsic, np.eye(4), delimiter=",")
    calibration = SimpleNamespace(
        fisheye_param_file=str(fisheye),
        list_0_files=str(extrinsic),
        list_1_files="",
        list_2_files="",
    )

    assert _diagnosis().validate_calibration_data(0, calibration) == ()


def test_camera_calibration_reports_invalid_intrinsics_and_extrinsic(tmp_path: Path) -> None:
    fisheye = tmp_path / "fisheye.json"
    extrinsic = tmp_path / "camera0.csv"
    _write_fisheye(fisheye)
    payload = json.loads(fisheye.read_text(encoding="utf-8"))
    payload["camera_matrix"]["data"][0] = float("nan")
    fisheye.write_text(json.dumps(payload), encoding="utf-8")
    np.savetxt(extrinsic, np.eye(3), delimiter=",")
    calibration = SimpleNamespace(
        fisheye_param_file=str(fisheye),
        list_0_files=str(extrinsic),
        list_1_files="",
        list_2_files="",
    )

    issues = _diagnosis().validate_calibration_data(0, calibration)

    assert {issue.kind for issue in issues} == {
        "fisheye_camera_matrix",
        "camera_lidar_matrix",
    }


def test_camera_calibration_diagnosis_owns_camera_error_log(tmp_path: Path) -> None:
    diagnosis = _diagnosis()
    logger = MagicMock()
    diagnosis._logger = logger  # noqa: SLF001
    calibration = SimpleNamespace(
        fisheye_param_file=str(tmp_path / "missing.json"),
        list_0_files=str(tmp_path / "missing.csv"),
        list_1_files="",
        list_2_files="",
    )

    issues = diagnosis.diagnose_calibration_data(
        0, calibration, ActionErrorIndex.CAMERA0_CALIB_DATA_INVALID
    )

    expected_issue_count = 2
    assert len(issues) == expected_issue_count
    assert diagnosis.err_cnt.value == 1
    assert logger.error.call_args.args[0].startswith(
        "CE007: CAMERA_CALIB_DATA_INVALID: camera=0"
    )


def test_camera_calibration_diagnosis_logs_validation_exception() -> None:
    diagnosis = _diagnosis()
    diagnosis._logger = MagicMock()
    error = FileNotFoundError("camera0 calibration disappeared")

    assert diagnosis.excepts_diagnosis(error) is True
    diagnosis.log_output(
        True,
        False,
        ActionErrorIndex.CAMERA0_CALIB_DATA_INVALID,
        error,
    )

    assert diagnosis.err_cnt.value == 1
    diagnosis._logger.error.assert_called_once_with(
        "CE007: CAMERA_CALIB_DATA_INVALID: validation failed: "
        "FileNotFoundError: camera0 calibration disappeared",
        exc_info=True,
    )


def test_camera3_slot_reports_missing_configuration() -> None:
    calibration = SimpleNamespace(
        fisheye_param_file="missing.json",
        list_0_files="",
        list_1_files="",
        list_2_files="",
    )

    issues = _diagnosis().validate_calibration_data(3, calibration)

    assert any("unsupported camera index: 3" in issue.detail for issue in issues)
