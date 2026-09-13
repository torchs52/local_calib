from pathlib import Path

from argus_synchro.common import paths
from argus_synchro.config.app_config_calibration import AppConfigCalibration
from argus_synchro.shared_app_config import SharedAppConfig

from directory_config_helper import dev_directory_config


def test_settings_debug_log_uses_cli_log_dir() -> None:
    dc = dev_directory_config()
    repo_root = Path.cwd()
    directory_config, _ = paths.load_directory_config_from_ini(dc)
    sac = SharedAppConfig(directory_config)
    app_config = sac.read()

    expected_log = str(paths.normalize_path("./log/argus3d_core.log", repo_root))
    assert app_config.DEFAULT.debug_log == expected_log
    assert not hasattr(app_config.AppManager, "logmode")
    assert not hasattr(app_config.AppManager, "logtime")
    assert not hasattr(app_config.AppManager, "log_dir")


def test_settings_cli_log_dir_overrides_default() -> None:
    repo_root = Path.cwd()
    directory_config = paths.DirectoryConfig(
        config_dir=paths.normalize_path("./config", repo_root),
        log_dir=paths.normalize_path("/var/log/argus3d", repo_root),
        mmap_dir=paths.DEFAULT_MMAP_DIR,
    )
    directory_config, _ = paths.load_directory_config_from_ini(directory_config)
    sac = SharedAppConfig(directory_config)
    app_config = sac.read()

    assert app_config.DEFAULT.debug_log == "/var/log/argus3d/argus3d_core.log"


def test_calib_outputdir_root_uses_cli_log_dir() -> None:
    app_config = AppConfigCalibration(
        configpath="config/calib_settings.ini",
        arglist=[],
        directory_config=dev_directory_config(),
    )
    expected_root = str(paths.normalize_path("./log/tmpCalib", Path.cwd()))
    assert app_config.default.outputdir_root == expected_root


def test_calib_outputdir_root_cli_log_dir_overrides_default() -> None:
    repo_root = Path.cwd()
    directory_config = paths.DirectoryConfig(
        config_dir=paths.normalize_path("./config", repo_root),
        log_dir=paths.normalize_path("/var/log/argus3d", repo_root),
        mmap_dir=paths.DEFAULT_MMAP_DIR,
    )
    directory_config, _ = paths.load_directory_config_from_ini(directory_config)
    app_config = AppConfigCalibration(
        configpath="config/calib_settings.ini",
        arglist=[],
        directory_config=directory_config,
    )
    assert app_config.default.outputdir_root == "/var/log/argus3d/tmpCalib"
    assert app_config.filepath_io.Calib3d3dmat_lidars[0] == (
        "/var/log/argus3d/tmpCalib/3d-3d/lidar2crane_trans_mat_0.csv"
    )
