from configparser import ConfigParser, NoSectionError
from pathlib import Path

import pytest

from argus_synchro.common import paths
from argus_synchro.config.settings_validation import (
    ConfigValidationError,
    validate_settings,
)
from argus_synchro.diagnosis.action_errors import ConfigFileMissingDiagnosis
from argus_synchro.diagnosis.error_config import ErrorConfig
from argus_synchro.shared_app_config import SharedAppConfig


@pytest.mark.parametrize("operation_mode", ("-1", "2"))
def test_validate_settings_rejects_out_of_range_value(operation_mode: str) -> None:
    ini = ConfigParser()
    ini.read_dict({"General": {"operation_mode": operation_mode}})

    with pytest.raises(
        ConfigValidationError,
        match=r"\[General\] operation_mode=.*outside the allowed range",
    ):
        validate_settings(ini)


def test_validate_settings_rejects_invalid_type() -> None:
    ini = ConfigParser()
    ini.read_dict({"camera": {"count": "three"}})

    with pytest.raises(
        ConfigValidationError,
        match=r"\[camera\] count is invalid",
    ):
        validate_settings(ini)


def test_validate_settings_normalizes_value_to_allowed_range() -> None:
    ini = ConfigParser()
    ini.read_dict(
        {
            "ConfigValidation": {"invalid_value_policy": "normalize"},
            "camera": {"count": "8"},
        }
    )

    validate_settings(ini)

    assert ini.getint("camera", "count") == 4


def test_directory_config_load_validates_settings(tmp_path: Path) -> None:
    (tmp_path / "settings.ini").write_text(
        "[DEFAULT]\n"
        "data_dir = .\n"
        "[General]\n"
        "operation_mode = 2\n",
        encoding="utf-8",
    )
    directory_config = paths.DirectoryConfig(
        config_dir=tmp_path,
        log_dir=tmp_path,
        mmap_dir=tmp_path,
    )

    with pytest.raises(ConfigValidationError):
        paths.load_directory_config_from_ini(directory_config)


def test_broken_section_name_is_classified_as_ce005(tmp_path: Path) -> None:
    settings = Path("config/settings.ini").read_text(encoding="utf-8")
    (tmp_path / "settings.ini").write_text(
        settings.replace("[General]", "[BrokenGeneral]", 1),
        encoding="utf-8",
    )
    directory_config = paths.DirectoryConfig(
        config_dir=tmp_path,
        log_dir=tmp_path,
        mmap_dir=tmp_path,
    )

    with pytest.raises(NoSectionError) as error_info:
        SharedAppConfig(directory_config)

    diagnosis = ConfigFileMissingDiagnosis()
    diagnosis.update(ErrorConfig())
    assert diagnosis.excepts_diagnosis(error_info.value) is True
    assert diagnosis.err_cnt.value == 1