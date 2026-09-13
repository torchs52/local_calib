# from configparser import ConfigParser, ExtendedInterpolation 勝手に使ってはいけない


from argus_synchro import (
    calibration_mat_generator_modules,
)
from argus_synchro.common import paths
from argus_synchro.common.app_logger import AppLogger, AppLoggerFactory
from argus_synchro.config.app_config_calibration import AppConfigCalibration


class Calib_Mat_Generator:
    def __init__(self, app_logger_factory: AppLoggerFactory) -> None:
        self._app_logger_factory = app_logger_factory
        self._logger: AppLogger = app_logger_factory.register_from_type(self.__class__)

    @staticmethod
    # 精度自動テスト用の終了可否判定
    def allow_exit(
        calib_settings_ini_path: str,
        directory_config: paths.DirectoryConfig = paths.DEFAULT_DIRECTORY_CONFIG,
    ) -> bool:
        app_config_calib = AppConfigCalibration(
            configpath=calib_settings_ini_path,
            arglist=[],
            verb=True,
            directory_config=directory_config,
        )
        return calibration_mat_generator_modules.calibration2d3d_manager_class.check_allowexit(
            app_config_calib=app_config_calib
        )
