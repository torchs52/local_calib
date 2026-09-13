from __future__ import annotations

import json
from dataclasses import fields, is_dataclass
from pathlib import Path

import pytest

from argus_synchro.diagnosis.error_config import ErrorConfig
from argus_synchro.shared_err_config import SharedErrorConfig

INITIAL_ERROR_THRESHOLD_SEC = 10.0
UPDATED_ERROR_THRESHOLD_SEC = 20.0
INITIAL_REQUIRED_LENGTH = 40
UPDATED_REQUIRED_LENGTH = 80


def test_repository_error_config_covers_all_parameter_fields() -> None:
    json_path = Path(__file__).parents[1] / "config" / "error_config.json"
    json_data = json.loads(json_path.read_text(encoding="utf-8"))
    error_config = ErrorConfig()

    assert set(json_data) == set(vars(error_config))
    for name, parameter_data in json_data.items():
        parameters = getattr(error_config, name)
        if is_dataclass(parameters):
            assert set(parameter_data) == {
                field.name for field in fields(parameters)
            }, name


def _write_error_config_json(
    json_path: Path,
    *,
    error_threshold_sec: float,
    required_length: int,
) -> None:
    test_data = {
        "camera_n_connection_error": {
            "is_enabled": False,
            "error_threshold_sec": error_threshold_sec,
            "error_recovery_confirm_duration_sec": 2.0,
            "failsafe_recovery_confirm_duration_sec": 3.0,
            "recovery_receive_interval_sec": 1.0,
        },
        "can_invalid_data": {
            "is_enabled": True,
            "params_by_canid": {
                "18FFD1D1": {
                    "required_length": required_length,
                    "error_recovery_confirm_duration_sec": 4.0,
                    "failsafe_recovery_confirm_duration_sec": 5.0,
                }
            },
        },
    }
    json_path.write_text(json.dumps(test_data), encoding="utf-8")


def test_shared_error_config_reads_initial_json(
    tmp_path: Path,
) -> None:
    json_path = tmp_path / "error_config.json"
    _write_error_config_json(
        json_path,
        error_threshold_sec=INITIAL_ERROR_THRESHOLD_SEC,
        required_length=INITIAL_REQUIRED_LENGTH,
    )

    shared_error_config = SharedErrorConfig(json_path)
    try:
        loaded = shared_error_config.read()
        assert loaded.camera_n_connection_error.is_enabled is False
        assert (
            loaded.camera_n_connection_error.error_threshold_sec
            == INITIAL_ERROR_THRESHOLD_SEC
        )
        assert (
            loaded.can_invalid_data.params_by_canid["18FFD1D1"].required_length
            == INITIAL_REQUIRED_LENGTH
        )
    finally:
        shared_error_config.close()


def test_shared_error_config_write_reflects_updated_json(
    tmp_path: Path,
) -> None:
    json_path = tmp_path / "error_config.json"
    _write_error_config_json(
        json_path,
        error_threshold_sec=INITIAL_ERROR_THRESHOLD_SEC,
        required_length=INITIAL_REQUIRED_LENGTH,
    )

    shared_error_config = SharedErrorConfig(json_path)
    try:
        previous_last_updated = shared_error_config.last_updated
        _write_error_config_json(
            json_path,
            error_threshold_sec=UPDATED_ERROR_THRESHOLD_SEC,
            required_length=UPDATED_REQUIRED_LENGTH,
        )

        shared_error_config.write()
        loaded = shared_error_config.read()

        assert (
            loaded.camera_n_connection_error.error_threshold_sec
            == UPDATED_ERROR_THRESHOLD_SEC
        )
        assert (
            loaded.can_invalid_data.params_by_canid["18FFD1D1"].required_length
            == UPDATED_REQUIRED_LENGTH
        )
        assert shared_error_config.last_updated > previous_last_updated
    finally:
        shared_error_config.close()


def test_shared_error_config_raises_if_json_file_missing(
    tmp_path: Path,
) -> None:
    missing_json_path = tmp_path / "missing_error_config.json"
    with pytest.raises(FileNotFoundError):
        SharedErrorConfig(missing_json_path)
