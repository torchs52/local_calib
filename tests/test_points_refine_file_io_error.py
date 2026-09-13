# ruff: noqa: SLF001

from types import SimpleNamespace
from unittest.mock import MagicMock

import pytest

from argus_synchro.diagnosis.error_diagnosis import ResultDiagnosis
from argus_synchro.process.points_refine_process import PointsRefineProcess
from argus_synchro.shared_errors import (
    ActionErrorIndex,
    ModuleErrorIndex,
    StateErrorDIndex,
)


def _make_process() -> tuple[PointsRefineProcess, MagicMock]:
    error_config = object()
    file_io_error = MagicMock()
    file_io_error.errors_diagnosis.return_value = (
        ResultDiagnosis.DETECTION,
        ResultDiagnosis.NORMAL,
    )
    process = object.__new__(PointsRefineProcess)
    process._sac = SimpleNamespace(
        read=lambda: SimpleNamespace(
            General=SimpleNamespace(initial_transform_file="initial.csv")
        ),
        last_updated=1,
    )
    process._ser = SimpleNamespace(
        shared_err_conf=SimpleNamespace(read=lambda: error_config),
        state_errors_D={StateErrorDIndex.FILE_IO_ERROR: file_io_error},
    )
    return process, file_io_error


def test_initial_transform_csv_error_is_diagnosed_and_reraised(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    process, file_io_error = _make_process()
    error = ValueError("invalid transform")
    monkeypatch.setattr(
        "argus_synchro.process.points_refine_process.SubScrutinizer.load_transform_csv",
        MagicMock(side_effect=error),
    )

    with pytest.raises(ValueError, match="invalid transform"):
        process._config_load()

    file_io_error.errors_diagnosis.assert_called_once_with(True)
    assert file_io_error.log_output.call_args.args == (
        ResultDiagnosis.DETECTION,
        ResultDiagnosis.NORMAL,
        StateErrorDIndex.FILE_IO_ERROR,
        "initial.csv",
        "read initial transform CSV",
        "ValueError: invalid transform",
    )


def test_initial_transform_csv_success_updates_recovery_state(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    process, file_io_error = _make_process()
    transform = (object(), object())
    monkeypatch.setattr(
        "argus_synchro.process.points_refine_process.SubScrutinizer.load_transform_csv",
        MagicMock(return_value=transform),
    )

    process._config_load()

    assert (process._init_r, process._init_t) == transform
    file_io_error.errors_diagnosis.assert_called_once_with(False)
    file_io_error.log_output.assert_not_called()


def test_points_refine_err_config_load_updates_file_io_diagnosis() -> None:
    error_config = object()
    invalid_data_input = MagicMock()
    array_shape_error = MagicMock()
    file_io_error = MagicMock()
    crane_model_file_missing = MagicMock()
    accumulation_module_error = MagicMock()
    points_refine_module_error = MagicMock()
    process = object.__new__(PointsRefineProcess)
    process._ser = SimpleNamespace(
        shared_err_conf=SimpleNamespace(read=lambda: error_config),
        state_errors_D={
            StateErrorDIndex.INVALID_DATA_INPUT: invalid_data_input,
            StateErrorDIndex.ARRAY_SHAPE_ERROR: array_shape_error,
            StateErrorDIndex.FILE_IO_ERROR: file_io_error,
        },
        action_errors_A_C={
            ActionErrorIndex.CRANE_MODEL_FILE_MISSING: crane_model_file_missing
        },
        module_errors={
            ModuleErrorIndex.ACCUMULATION_MODULE_ERROR: accumulation_module_error,
            ModuleErrorIndex.POINTS_REFINE_MODULE_ERROR: points_refine_module_error,
        },
    )

    process._err_config_load()

    file_io_error.update.assert_called_once_with(error_config)