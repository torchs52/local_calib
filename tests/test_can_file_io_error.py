# pyright: reportAttributeAccessIssue=false

from types import SimpleNamespace
from unittest.mock import MagicMock

import numpy as np
import pytest

from argus_synchro.config.app_config import CANConf
from argus_synchro.device.can import can_receiver
from argus_synchro.diagnosis.error_diagnosis import ResultDiagnosis
from argus_synchro.process import can_process
from argus_synchro.process.can_process import CanDataProviderProcess
from argus_synchro.provider import can_data
from argus_synchro.shared_errors import (
    ActionErrorIndex,
    ModuleErrorIndex,
    StateErrorDIndex,
)


class _FailingCanFile:
    def __init__(self, *args: object, **kwargs: object) -> None:
        raise OSError("CAN input CSV open failed")


class _SuccessfulCanFile:
    def __init__(self, *args: object, **kwargs: object) -> None:
        pass


class _FailingCanMapFile:
    def __init__(self, *args: object, **kwargs: object) -> None:
        raise can_receiver.CanIdMapError(
            "config/can_id_map.csv",
            FileNotFoundError("map missing"),
        )


class _FailingProvider:
    def change_file_name_index(self, file_path: str, index: int) -> None:
        raise ValueError("CAN input CSV parse failed")


class _SuccessfulProvider:
    def change_file_name_index(self, file_path: str, index: int) -> None:
        pass


class _PathRecordingProvider:
    def __init__(self) -> None:
        self.calls: list[tuple[str, int]] = []

    def change_file_name_index(self, file_path: str, index: int) -> None:
        self.calls.append((file_path, index))


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

    def excepts_diagnosis(self, error: Exception) -> bool:
        self.diagnosis_calls.append((error,))
        return True


def _process() -> tuple[CanDataProviderProcess, _RecordingDiagnosis]:
    diagnosis = _RecordingDiagnosis()
    state_errors_d: list[object | None] = [None] * (
        StateErrorDIndex.FILE_IO_ERROR + 1
    )
    state_errors_d[StateErrorDIndex.FILE_IO_ERROR] = diagnosis

    process = object.__new__(CanDataProviderProcess)
    process._app_config = SimpleNamespace(
        DEFAULT=SimpleNamespace(File_Input=True, use_shi_lib=False),
        General=SimpleNamespace(operation_mode=0),
        UI_IF=SimpleNamespace(crane_model="SCX900-3"),
        CAN=CANConf(
            config_file="config/can.json",
            IsOld=False,
            interpretation=1,
            yaw_offset_deg=0.0,
            c_file="can_input.csv",
            can_id_map_file="config/can_id_map.csv",
        ),
        Scrutinizer=SimpleNamespace(s_frame=0),
    )
    process._app_config_calib = SimpleNamespace(
        dataCapture=SimpleNamespace(
            Can=SimpleNamespace(
                is_fixed_yaw=False,
                c_file="calibration_can.csv",
                fixed_yaw_deg=12.5,
            )
        )
    )
    process._frame = 0
    process._ser = SimpleNamespace(
        action_errors_A_C={
            ActionErrorIndex.CONFIG_FILE_MISSING: _RecordingDiagnosis()
        },
        state_errors_D=state_errors_d,
    )
    process._app_logger_factory = SimpleNamespace()
    return process, diagnosis


def test_can_err_config_load_updates_config_file_diagnosis() -> None:
    error_config = object()
    config_file_missing = MagicMock()
    can_module_error = MagicMock()
    file_io_error = MagicMock()
    process = object.__new__(CanDataProviderProcess)
    process._ser = SimpleNamespace(
        shared_err_conf=SimpleNamespace(read=lambda: error_config),
        action_errors_A_C={
            ActionErrorIndex.CONFIG_FILE_MISSING: config_file_missing
        },
        state_errors_D={StateErrorDIndex.FILE_IO_ERROR: file_io_error},
        module_errors={ModuleErrorIndex.CAN_MODULE_ERROR: can_module_error},
    )

    process._err_config_load()

    config_file_missing.update.assert_called_once_with(error_config)


def test_file_input_init_error_is_diagnosed_and_reraised(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setattr(can_receiver, "CanFile", _FailingCanFile)
    process, diagnosis = _process()

    with pytest.raises(OSError, match="CAN input CSV open failed"):
        process._change_device()

    assert diagnosis.diagnosis_calls == [(True,)]
    assert diagnosis.log_calls == [
        (
            ResultDiagnosis.DETECTION,
            ResultDiagnosis.NORMAL,
            StateErrorDIndex.FILE_IO_ERROR,
            "can_input.csv",
            "read file-input CAN CSV",
            "OSError: CAN input CSV open failed",
        )
    ]


def test_successful_file_input_init_updates_recovery_state(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setattr(can_receiver, "CanFile", _SuccessfulCanFile)
    monkeypatch.setattr(
        can_data,
        "CanFileProvider",
        lambda *_args, **_kwargs: SimpleNamespace(),
    )
    process, diagnosis = _process()

    process._change_device()

    assert diagnosis.diagnosis_calls == [(False,)]
    assert diagnosis.log_calls == []


def test_calibration_file_input_uses_calibration_can_file(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    received_configs: list[object] = []

    class _RecordingCanFile:
        def __init__(self, config: object, *_args: object) -> None:
            received_configs.append(config)

    monkeypatch.setattr(can_receiver, "CanFile", _RecordingCanFile)
    monkeypatch.setattr(
        can_data,
        "CanFileProvider",
        lambda *_args, **_kwargs: SimpleNamespace(),
    )
    process, _ = _process()
    process._app_config.General.operation_mode = 1

    process._change_device()

    assert len(received_configs) == 1
    assert received_configs[0].c_file == "calibration_can.csv"


@pytest.mark.parametrize("file_input", (False, True))
def test_calibration_fixed_yaw_does_not_open_can_device(
    monkeypatch: pytest.MonkeyPatch,
    file_input: bool,
) -> None:
    can_file = MagicMock()
    monkeypatch.setattr(can_receiver, "CanFile", can_file)
    process, _ = _process()
    process._app_config.DEFAULT.File_Input = file_input
    process._app_config.General.operation_mode = 1
    process._app_config_calib.dataCapture.Can.is_fixed_yaw = True

    process._change_device()

    can_file.assert_not_called()
    yaw, lever = process._provider.receive_can_data()
    assert yaw == 12.5
    assert np.isnan(lever).all()


@pytest.mark.parametrize(
    ("operation_mode", "expected_path"),
    ((0, "can_input.csv"), (1, "calibration_can.csv")),
)
def test_file_input_restart_selects_can_file_for_current_mode(
    monkeypatch: pytest.MonkeyPatch,
    operation_mode: int,
    expected_path: str,
) -> None:
    monkeypatch.setattr(can_process, "CanFileProvider", _PathRecordingProvider)
    process, _ = _process()
    process._app_config.General.operation_mode = operation_mode
    process._frame = 25
    provider = _PathRecordingProvider()
    process._provider = provider

    process._change_file_name_index()

    assert provider.calls == [(expected_path, 25)]


def test_can_id_map_error_records_map_path_as_ce005(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setattr(can_receiver, "CanFile", _FailingCanMapFile)
    process, file_io_diagnosis = _process()
    config_diagnosis = process._ser.action_errors_A_C[
        ActionErrorIndex.CONFIG_FILE_MISSING
    ]

    with pytest.raises(
        can_receiver.CanIdMapError,
        match=r"path=config/can_id_map\.csv",
    ):
        process._change_device()

    diagnosed_error = config_diagnosis.diagnosis_calls[0][0]
    assert isinstance(diagnosed_error, can_receiver.CanIdMapError)
    assert diagnosed_error.file_path == "config/can_id_map.csv"
    assert config_diagnosis.log_calls[0][0:3] == (
        True,
        False,
        ActionErrorIndex.CONFIG_FILE_MISSING,
    )
    assert config_diagnosis.log_calls[0][3] is diagnosed_error
    assert file_io_diagnosis.diagnosis_calls == []
    assert file_io_diagnosis.log_calls == []


def test_file_input_reload_error_is_diagnosed_and_reraised(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setattr(can_process, "CanFileProvider", _FailingProvider)
    process, diagnosis = _process()
    process._provider = _FailingProvider()

    with pytest.raises(ValueError, match="CAN input CSV parse failed"):
        process._change_file_input_file("next_can.csv", 10)

    assert diagnosis.diagnosis_calls == [(True,)]
    assert diagnosis.log_calls == [
        (
            ResultDiagnosis.DETECTION,
            ResultDiagnosis.NORMAL,
            StateErrorDIndex.FILE_IO_ERROR,
            "next_can.csv",
            "read file-input CAN CSV",
            "ValueError: CAN input CSV parse failed",
        )
    ]


def test_successful_file_input_reload_updates_recovery_state(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setattr(can_process, "CanFileProvider", _SuccessfulProvider)
    process, diagnosis = _process()
    process._provider = _SuccessfulProvider()

    process._change_file_input_file("next_can.csv", 10)

    assert diagnosis.diagnosis_calls == [(False,)]
    assert diagnosis.log_calls == []
