from pathlib import Path
from types import SimpleNamespace
from unittest.mock import MagicMock

import numpy as np

from argus_synchro.diagnosis.action_errors import SensorCalibDataInvalidDiagnosis
from argus_synchro.diagnosis.error_config import SensorCalibDataInvalidParameters
from argus_synchro.shared_errors import ActionErrorIndex


def _calibration_conf(both_lidars: Path, lidar_files: list[Path]) -> SimpleNamespace:
    return SimpleNamespace(
        BothLidars=str(both_lidars),
        Lidar_calib_files=[str(path) for path in lidar_files],
    )


def _diagnosis() -> SensorCalibDataInvalidDiagnosis:
    diagnosis = SensorCalibDataInvalidDiagnosis()
    diagnosis.param = SensorCalibDataInvalidParameters()
    diagnosis.param.check_lidar2lidar = True
    diagnosis.is_enabled = True
    return diagnosis


def test_lidar2lidar_check_is_disabled_by_default(tmp_path: Path) -> None:
    diagnosis = SensorCalibDataInvalidDiagnosis()
    diagnosis.is_enabled = True

    issues = diagnosis.validate_calibration_matrices(
        _calibration_conf(tmp_path / "missing.csv", [])  # type: ignore[arg-type]
    )

    assert issues == ()


def test_valid_lidar_calibration_matrices_have_no_issue(tmp_path: Path) -> None:
    both_lidars = tmp_path / "both.csv"
    lidar0 = tmp_path / "lidar0.csv"
    np.savetxt(both_lidars, np.eye(4), delimiter=",")
    np.savetxt(lidar0, np.eye(4), delimiter=",")
    diagnosis = _diagnosis()

    issues = diagnosis.validate_calibration_matrices(
        _calibration_conf(both_lidars, [lidar0])  # type: ignore[arg-type]
    )

    assert issues == ()
    assert diagnosis.err_cnt.value == 0


def test_all_missing_lidar_calibration_matrices_are_reported(tmp_path: Path) -> None:
    diagnosis = _diagnosis()

    issues = diagnosis.validate_calibration_matrices(
        _calibration_conf(
            tmp_path / "both.csv",
            [tmp_path / "lidar0.csv", tmp_path / "lidar1.csv"],
        )  # type: ignore[arg-type]
    )

    assert len(issues) == 3
    assert [issue.matrix_kind for issue in issues] == [
        "lidar2lidar",
        "lidar2crane",
        "lidar2crane",
    ]
    assert diagnosis.err_cnt.value == 1


def test_malformed_shape_and_non_finite_values_are_reported(
    tmp_path: Path,
) -> None:
    malformed = tmp_path / "malformed.csv"
    wrong_shape = tmp_path / "wrong_shape.csv"
    non_finite = tmp_path / "non_finite.csv"
    malformed.write_text("not,a,matrix\n", encoding="utf-8")
    np.savetxt(wrong_shape, np.eye(3), delimiter=",")
    matrix = np.eye(4)
    matrix[0, 0] = np.nan
    np.savetxt(non_finite, matrix, delimiter=",")
    diagnosis = _diagnosis()

    issues = diagnosis.validate_calibration_matrices(
        _calibration_conf(malformed, [wrong_shape, non_finite])  # type: ignore[arg-type]
    )

    assert len(issues) == 3
    assert "failed to load CSV" in issues[0].detail
    assert "matrix shape must be (4, 4)" in issues[1].detail
    assert issues[2].detail == "matrix contains non-finite values"


def test_disabled_diagnosis_does_not_read_files(tmp_path: Path) -> None:
    diagnosis = _diagnosis()
    diagnosis.is_enabled = False

    issues = diagnosis.validate_calibration_matrices(
        _calibration_conf(tmp_path / "missing.csv", [])  # type: ignore[arg-type]
    )

    assert issues == ()
    assert diagnosis.err_cnt.value == 0


def test_sensor_calibration_diagnosis_owns_ce006_log(tmp_path: Path) -> None:
    diagnosis = _diagnosis()
    diagnosis._logger = MagicMock()
    issues = diagnosis.validate_calibration_matrices(
        _calibration_conf(tmp_path / "both.csv", [])  # type: ignore[arg-type]
    )

    diagnosis.log_output(
        True,
        False,
        ActionErrorIndex.SENSOR_CALIB_DATA_INVALID,
        issues,
    )

    diagnosis._logger.error.assert_called_once_with(
        "CE006: SENSOR_CALIB_DATA_INVALID: "
        f"kind=lidar2lidar path={tmp_path / 'both.csv'} "
        "detail=failed to load CSV: FileNotFoundError: "
        f"{tmp_path / 'both.csv'} not found. issues=1"
    )


def test_sensor_calibration_diagnosis_logs_validation_exception() -> None:
    diagnosis = _diagnosis()
    diagnosis._logger = MagicMock()
    error = ValueError(
        "lidar2crane_reference_paths must match Lidar_calib_files length"
    )

    assert diagnosis.excepts_diagnosis(error) is True
    diagnosis.log_output(
        True,
        False,
        ActionErrorIndex.SENSOR_CALIB_DATA_INVALID,
        error,
    )

    assert diagnosis.err_cnt.value == 1
    diagnosis._logger.error.assert_called_once_with(
        "CE006: SENSOR_CALIB_DATA_INVALID: validation failed: "
        "ValueError: lidar2crane_reference_paths must match "
        "Lidar_calib_files length",
        exc_info=True,
    )