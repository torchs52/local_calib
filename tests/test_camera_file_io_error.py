# pyright: reportAttributeAccessIssue=false, reportPrivateUsage=false

from types import SimpleNamespace

import pytest

from argus_synchro.device.camera import mcde7000
from argus_synchro.diagnosis.error_diagnosis import ResultDiagnosis
from argus_synchro.process.image_process import CameraProviderProcess
from argus_synchro.process.operation_mode import OPERATION_MODE as OPM
from argus_synchro.provider import image as image_provider
from argus_synchro.shared_errors import StateErrorDIndex


class _FailingFileDevice:
    def __init__(self, *args: object, **kwargs: object) -> None:
        raise OSError("camera video open failed")


class _InitFailingFileDevice:
    def __init__(self, *args: object, **kwargs: object) -> None:
        pass

    def init_capture(self) -> None:
        raise RuntimeError("camera video init failed")


class _SuccessfulFileDevice:
    def __init__(self, *args: object, **kwargs: object) -> None:
        pass

    def init_capture(self) -> None:
        pass


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


def _process() -> tuple[CameraProviderProcess, _RecordingDiagnosis]:
    diagnosis = _RecordingDiagnosis()
    state_errors_d: list[object | None] = [None] * (
        StateErrorDIndex.FILE_IO_ERROR + 1
    )
    state_errors_d[StateErrorDIndex.FILE_IO_ERROR] = diagnosis

    process = object.__new__(CameraProviderProcess)
    process._file_input = True
    process._index = 0
    process._frame = 12
    process._app_config = SimpleNamespace(
        DEFAULT=SimpleNamespace(use_shi_lib=False),
        General=SimpleNamespace(operation_mode=OPM.SCRUT),
        camera=SimpleNamespace(sys_width=1920, sys_height=1080),
    )
    process._scrutinizer_conf = SimpleNamespace(
        v0_file="camera0.mp4",
        v1_file="camera1.mp4",
        v2_file="camera2.mp4",
    )
    process._ser = SimpleNamespace(state_errors_D=state_errors_d)
    process._app_logger_factory = SimpleNamespace()
    return process, diagnosis


def test_file_input_open_error_is_diagnosed_and_reraised(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setattr(mcde7000, "Mcde7000File", _FailingFileDevice)
    process, diagnosis = _process()

    with pytest.raises(OSError, match="camera video open failed"):
        process._change_device()

    assert diagnosis.diagnosis_calls == [(True,)]
    assert diagnosis.log_calls == [
        (
            ResultDiagnosis.DETECTION,
            ResultDiagnosis.NORMAL,
            StateErrorDIndex.FILE_IO_ERROR,
            "camera0.mp4",
            "read file-input camera video",
            "OSError: camera video open failed",
        )
    ]


def test_file_input_init_error_is_diagnosed_and_reraised(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setattr(mcde7000, "Mcde7000File", _InitFailingFileDevice)
    process, diagnosis = _process()

    with pytest.raises(RuntimeError, match="camera video init failed"):
        process._change_device()

    assert diagnosis.diagnosis_calls == [(True,)]
    assert diagnosis.log_calls == [
        (
            ResultDiagnosis.DETECTION,
            ResultDiagnosis.NORMAL,
            StateErrorDIndex.FILE_IO_ERROR,
            "camera0.mp4",
            "read file-input camera video",
            "RuntimeError: camera video init failed",
        )
    ]


def test_successful_file_input_init_updates_recovery_state(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setattr(mcde7000, "Mcde7000File", _SuccessfulFileDevice)
    monkeypatch.setattr(
        image_provider,
        "Mcde7000FileImageProvider",
        lambda *_args, **_kwargs: SimpleNamespace(),
    )
    process, diagnosis = _process()

    process._change_device()

    assert diagnosis.diagnosis_calls == [(False,)]
    assert diagnosis.log_calls == []