from pathlib import Path
from unittest.mock import MagicMock

from argus_synchro.common import paths
from argus_synchro.machine_profile import MachineProfileHandler


def test_model_specific_calibration_config_is_applied(tmp_path: Path) -> None:
    (tmp_path / "settings.ini").write_text(
        "[UI_IF]\ncrane_model = SCX900-3\n",
        encoding="utf-8",
    )
    (tmp_path / "calib_settings.ini").write_text(
        "[DataCapture_Lidar]\ncount = 1\n"
        "[CalibCheck2d3d]\nonnx_model_path = old.onnx\n",
        encoding="utf-8",
    )
    (tmp_path / "SCX900-3_calib_settings.ini").write_text(
        "[DataCapture_Lidar]\ncount = 2\n"
        "[CalibCheck2d3d]\nonnx_model_path = new.onnx\n"
        "unknown_key = ignored\n",
        encoding="utf-8",
    )
    directory_config = paths.DirectoryConfig(tmp_path, tmp_path, tmp_path)
    handler = MachineProfileHandler(MagicMock(), directory_config)

    handler.apply_model_specific_calib_config()

    result = (tmp_path / "calib_settings.ini").read_text(encoding="utf-8")
    assert "count = 2" in result
    assert "onnx_model_path = new.onnx" in result
    assert "unknown_key" not in result


def test_model_specific_calibration_path_uses_settings_crane_model(
    tmp_path: Path,
) -> None:
    (tmp_path / "settings.ini").write_text(
        "[UI_IF]\ncrane_model = SCX700-3\n",
        encoding="utf-8",
    )
    expected = tmp_path / "SCX700-3_calib_settings.ini"
    expected.touch()
    directory_config = paths.DirectoryConfig(tmp_path, tmp_path, tmp_path)

    assert (
        MachineProfileHandler.get_model_specific_calib_config_file_path(
            directory_config
        )
        == expected
    )