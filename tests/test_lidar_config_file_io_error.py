# ruff: noqa: SLF001

import json
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import MagicMock

import pytest

from argus_synchro.diagnosis.error_diagnosis import ResultDiagnosis
from argus_synchro.process.imu_process import ImuProviderProcess
from argus_synchro.process.points_process import PointsProviderProcess
from argus_synchro.shared_errors import ModuleErrorIndex, StateErrorDIndex


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


def _process(
    process_type: type[ImuProviderProcess] | type[PointsProviderProcess],
    config_path: Path,
) -> tuple[ImuProviderProcess | PointsProviderProcess, _RecordingDiagnosis]:
    diagnosis = _RecordingDiagnosis()
    process = object.__new__(process_type)
    process._app_config = SimpleNamespace(Lidar=SimpleNamespace(path=str(config_path)))
    process._ser = SimpleNamespace(
        state_errors_D={StateErrorDIndex.FILE_IO_ERROR: diagnosis}
    )
    return process, diagnosis


@pytest.mark.parametrize("process_type", (ImuProviderProcess, PointsProviderProcess))
def test_lidar_config_read_error_is_diagnosed_and_reraised(
    process_type: type[ImuProviderProcess] | type[PointsProviderProcess],
    tmp_path: Path,
) -> None:
    config_path = tmp_path / "config_lidars.json"
    config_path.write_text("{", encoding="utf-8")
    process, diagnosis = _process(process_type, config_path)

    with pytest.raises(json.JSONDecodeError):
        process._read_lidar_config()

    assert diagnosis.diagnosis_calls == [(True,)]
    assert diagnosis.log_calls[0][:5] == (
        ResultDiagnosis.DETECTION,
        ResultDiagnosis.NORMAL,
        StateErrorDIndex.FILE_IO_ERROR,
        str(config_path),
        "read config_lidars.json",
    )
    assert str(diagnosis.log_calls[0][5]).startswith("JSONDecodeError:")


@pytest.mark.parametrize("process_type", (ImuProviderProcess, PointsProviderProcess))
def test_lidar_config_read_success_updates_recovery_state(
    process_type: type[ImuProviderProcess] | type[PointsProviderProcess],
    tmp_path: Path,
) -> None:
    config_path = tmp_path / "config_lidars.json"
    config_path.write_text('{"MID3601": {}}', encoding="utf-8")
    process, diagnosis = _process(process_type, config_path)

    process._read_lidar_config()

    assert diagnosis.diagnosis_calls == [(False,)]
    assert diagnosis.log_calls == []
    assert process._lidar_config_index_map == {0: "MID3601"}


def test_imu_err_config_load_updates_file_io_diagnosis() -> None:
    error_config = object()
    file_io_error = MagicMock()
    imu_module_error = MagicMock()
    process = object.__new__(ImuProviderProcess)
    process._ser = SimpleNamespace(
        shared_err_conf=SimpleNamespace(read=lambda: error_config),
        state_errors_D={StateErrorDIndex.FILE_IO_ERROR: file_io_error},
        module_errors={ModuleErrorIndex.IMU_MODULE_ERROR: imu_module_error},
    )

    process._err_config_load()

    file_io_error.update.assert_called_once_with(error_config)
    imu_module_error.update.assert_called_once_with(error_config)
