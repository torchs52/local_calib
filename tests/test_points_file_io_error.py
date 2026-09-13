# pyright: reportAttributeAccessIssue=false, reportPrivateUsage=false

from types import SimpleNamespace

import pytest

from argus_synchro.diagnosis.error_diagnosis import ResultDiagnosis
from argus_synchro.process.points_process import PointsProviderProcess
from argus_synchro.shared_errors import StateErrorDIndex


class _FailingProvider:
    def get_accum_points(self) -> None:
        raise OSError("LiDAR input failed")


class _SuccessfulProvider:
    def get_accum_points(self) -> None:
        return None


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


def _process(file_input: bool) -> tuple[PointsProviderProcess, _RecordingDiagnosis]:
    diagnosis = _RecordingDiagnosis()
    state_errors_d: list[object | None] = [None] * (
        StateErrorDIndex.FILE_IO_ERROR + 1
    )
    state_errors_d[StateErrorDIndex.FILE_IO_ERROR] = diagnosis

    process = object.__new__(PointsProviderProcess)
    process._provider = _FailingProvider()
    process._file_input = file_input
    process._index = 1
    process._app_config = SimpleNamespace(
        Lidar=SimpleNamespace(
            lidar0_file="lidar0_",
            lidar1_file="lidar1_",
            lidar2_file="lidar2_",
            lidar3_file="lidar3_",
            lidar4_file="lidar4_",
            lidar5_file="lidar5_",
        )
    )
    process._ser = SimpleNamespace(state_errors_D=state_errors_d)
    return process, diagnosis


def test_file_input_read_error_is_diagnosed_and_reraised() -> None:
    process, diagnosis = _process(file_input=True)

    with pytest.raises(OSError, match="LiDAR input failed"):
        process._update()

    assert diagnosis.diagnosis_calls == [(True,)]
    assert diagnosis.log_calls == [
        (
            ResultDiagnosis.DETECTION,
            ResultDiagnosis.NORMAL,
            StateErrorDIndex.FILE_IO_ERROR,
            "lidar1_",
            "read file-input LiDAR point cloud",
            "OSError: LiDAR input failed",
        )
    ]


def test_sensor_read_error_is_not_classified_as_file_io() -> None:
    process, diagnosis = _process(file_input=False)

    with pytest.raises(OSError, match="LiDAR input failed"):
        process._update()

    assert diagnosis.diagnosis_calls == []
    assert diagnosis.log_calls == []


def test_successful_file_input_read_updates_recovery_state() -> None:
    process, diagnosis = _process(file_input=True)
    process._provider = _SuccessfulProvider()
    process._ser.set_lidar_connected = lambda *_args: None

    assert process._update() is None

    assert diagnosis.diagnosis_calls == [(False,)]
    assert diagnosis.log_calls == []