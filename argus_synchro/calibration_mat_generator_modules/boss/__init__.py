"""
校正全体の管理クラス(boss)。校正処理で必要なリソースは全てここに入っている。
Adminクラスで制御を受け、各メソッドを実行する。
"""

import traceback  # 例外時のトレースバック取得用
from typing import cast

# ARGUSシステム制御関連
from argus_synchro.calibration_mat_generator_modules.ctrl.calibcheck2d3d import (
    calibcheck2d3d,
)

# 3d3d校正プログラムもここに統合
from argus_synchro.calibration_mat_generator_modules.ctrl.calibration2d3d import (
    calibration2d3d_class,
)
from argus_synchro.calibration_mat_generator_modules.ctrl.calibration3d3d import (
    calibration3d3d_class,
)
from argus_synchro.calibration_mat_generator_modules.ctrl.data_capture import (
    data_capture,
)

# 待機時のセンサー・カメラ出力用（暫定）
from argus_synchro.calibration_mat_generator_modules.ctrl.wait_app import wait_app

# 型定義でのみ使用
from argus_synchro.calibration_mat_generator_modules.facade import CalibrationUIGodot

# ARGUSシステム制御関連
from argus_synchro.common.app_logger import AppLogger, AppLoggerFactory
from argus_synchro.config.app_config_calibration import AppConfigCalibration
from argus_synchro.shared_app_config import SharedAppConfig
from argus_synchro.shared_errors import SharedErrors
from argus_synchro.shared_excepts import SharedExcepts


class boss:
    def __init__(
        self,
        app_config_calib: AppConfigCalibration,
        app_logger_factory: AppLoggerFactory,
        shared_errors: SharedErrors,
    ):
        self._app_logger_factory: AppLoggerFactory = app_logger_factory
        self._logger: AppLogger = app_logger_factory.register_from_type(self.__class__)
        # memo: 管轄が曖昧になるのでmonitorはこのクラスの変数として持たないこと
        self.app_config_calib: AppConfigCalibration = app_config_calib
        self._ser: SharedErrors = shared_errors

        self.sensor_reboot = (
            self.app_config_calib.default.File_Input
        )  # センサ入力時は逐次再起動はしない　ファイル入力時は再起動で暫定対応
        self.verbose: bool = not self.app_config_calib.default.print_disabled

    def update_settings(self, app_config_calib: AppConfigCalibration):
        self.app_config_calib: AppConfigCalibration = app_config_calib

    def __delattr__(self, name: str) -> None:
        self.close()

    def pre_calib3d3d(
        self,
        monitor: CalibrationUIGodot,
        sec: SharedExcepts,
        sac: SharedAppConfig,
        app_config_calib: AppConfigCalibration,
    ) -> None:
        self.update_settings(app_config_calib=app_config_calib)
        self.calibration3d3d_inst = calibration3d3d_class(
            app_config_calib=app_config_calib,
            sac=sac,
            app_logger_factory=self._app_logger_factory,
            shared_errors=self._ser,
        )

    def post_calib3d3d(
        self,
        monitor: CalibrationUIGodot,
        sec: SharedExcepts,
        sac: SharedAppConfig,
        app_config_calib: AppConfigCalibration,
    ) -> None:
        self._logger.info("calib3d3d_app finished")
        del self.calibration3d3d_inst

    def pre_calib2d3d_app(
        self,
        monitor: CalibrationUIGodot,
        sec: SharedExcepts,
        sac: SharedAppConfig,
        app_config_calib: AppConfigCalibration,
    ):
        self.update_settings(app_config_calib=app_config_calib)

        self._logger.info("calib2d3d_app started")
        self._logger.info(
            f"{sac.read().CalibMode.cameraID = }, monitor.set_currentcamera"
        )
        monitor.set_currentcamera(sac.read().CalibMode.cameraID)
        self.calibration2d3d_inst = calibration2d3d_class(
            app_config_calib=app_config_calib,
            sac=sac,
            app_logger_factory=self._app_logger_factory,
            shared_errors=self._ser,
        )

    def post_calib2d3d_app(
        self,
    ):
        self._logger.info("calib2d3d_app finished")
        del self.calibration2d3d_inst

    def pre_calibcheck2d3d_app(
        self,
        monitor: CalibrationUIGodot,
        sec: SharedExcepts,
        sac: SharedAppConfig,
        app_config_calib: AppConfigCalibration,
    ) -> None:
        self._logger.info("calibcheck2d3d_app started")
        self.update_settings(app_config_calib=app_config_calib)

        self.calibcheck2d3d_inst = calibcheck2d3d(
            app_config_calib=app_config_calib,
            sac=sac,
            app_logger_factory=self._app_logger_factory,
            shared_errors=self._ser,
        )

    def post_calibcheck2d3d_app(self) -> None:
        self._logger.info("calibcheck2d3d_app finished")
        del self.calibcheck2d3d_inst

    def pre_waiting_sensorcapture_app(
        self,
        monitor: CalibrationUIGodot,
        sec: SharedExcepts,
        sac: SharedAppConfig,
        app_config_calib: AppConfigCalibration,
    ) -> None:
        self._logger.info("wait started")
        self.wait_app_inst = wait_app(
            self.app_config_calib,
            sac=sac,
            app_logger_factory=self._app_logger_factory,
            shared_errors=self._ser,
        )

    def post_waiting_sensorcapture_app(self) -> None:
        self._logger.info("wait finished")

    def close(self) -> None:
        pass
