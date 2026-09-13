"""
プログラム全体の管理クラス(admin)。下位bossクラス各メソッドを呼び出しプログラム動作の制御を行う。

bossクラスインスタンスとfacadeクラスインスタンスを持ち、UIからの指令をbossに伝達する。
config等設定・引数の反映管理や例外処理等はこのモジュールの管轄。
"""

# パフォーマンスプロファイリング（デバッグ用）
import cProfile
import os
import sys
import traceback
from datetime import datetime as dt
from pathlib import Path

# 校正関連
from argus_synchro.calibration_mat_generator_modules.boss import boss

# from argus_synchro.calibration_mat_generator_modules.facade import FacadeUIManager
from argus_synchro.calibration_mat_generator_modules.facade import CalibrationUIGodot
from argus_synchro.calibration_mat_generator_modules.utils.debugdata_store import (  # デバッグ用情報記録クラス
    debug_apply_blacklist,
)
from argus_synchro.common import paths

# ARGUSシステム制御関連
from argus_synchro.common.app_logger import AppLogger, AppLoggerFactory
from argus_synchro.config.app_config_calibration import AppConfigCalibration
from argus_synchro.shared_errors import SharedErrors
from argus_synchro.shared_excepts import SharedExcepts


class calibration2d3d_manager_class:
    def __init__(
        self,
        sec: SharedExcepts,
        ser: SharedErrors,
        app_logger_factory: AppLoggerFactory,
        shared_errors: SharedErrors,
        arglist: list[str] = sys.argv,
        inifilepath: str | None = None,
        directory_config: paths.DirectoryConfig = paths.DEFAULT_DIRECTORY_CONFIG,
    ) -> None:
        self._directory_config = directory_config
        self.is_enable_profiler = False  # finally/closeで使う関係上先に宣言しないとconfig読み込み例外落ち時にログで混乱の原因になる
        self.is_closed = False
        self._app_logger_factory = app_logger_factory
        self._logger: AppLogger = app_logger_factory.register_from_type(self.__class__)
        if inifilepath is None:
            inifilepath = sys.argv[1]
        assert os.path.isfile(inifilepath)
        self.arglist: list[str] = arglist
        self.inifilepath: str = inifilepath
        self.uimanager: CalibrationUIGodot | None = None
        self._ser: SharedErrors = shared_errors
        self.get_calibconfig(sec=sec)  # 内部で反映

        self.check_allowexit(
            app_config_calib=self.app_config_calib
        )  # 一時ファイル削除のため空読み

        self.is_enable_profiler = self.app_config_calib.debug.is_enable_profiler
        # 校正モードのプロファイラ立ち上げ
        self.pr = None
        try:
            if self.is_enable_profiler:
                self.pr = cProfile.Profile()
                self.pr.enable()
                self._logger.info("cProfile.Profile started!")
        except Exception as error:
            self._logger.warning(
                f"cProfile enable failed: {error}; continue without profiler"
            )

        self.boss_inst = boss(
            app_config_calib=self.app_config_calib,
            app_logger_factory=self._app_logger_factory,
            shared_errors=self._ser,
        )
        config_dir: Path = paths.get_config_dir(
            self._directory_config, "calibration_mat_generator_modules"
        )
        debugdata_store_blacklist_path = str(
            paths.normalize_path("debugdata_store_blacklist.txt", config_dir)
        )

        debug_apply_blacklist(
            debugdata_store_blacklist_path,
            deny_all=(not self.app_config_calib.debug.enable_debugdata_store),
        )

    def get_calibconfig_s(
        self, inifilepath: str, arglist: list[str]
    ) -> AppConfigCalibration:
        return AppConfigCalibration(
            configpath=inifilepath,
            arglist=arglist,
            verb=True,
            directory_config=self._directory_config,
        )

    def get_calibconfig(
        self,
        sec: SharedExcepts,
    ) -> AppConfigCalibration:
        try:
            self.app_config_calib = self.get_calibconfig_s(
                inifilepath=self.inifilepath, arglist=self.arglist
            )
            return self.app_config_calib
        except Exception as ea:
            self._logger.error(
                f"calibration2d3d_manager_class - get_calibconfig exception: {ea}, traceback: \n{traceback.format_exc()}\n"
                "config構文が誤っていませんか？",
            )
            if self.uimanager is not None:
                self.uimanager.set_errorcode_unexpected_exception(True)
                self.uimanager.transmit_setdata(
                    sec=sec, ref_t=None, is_firstframe=False
                )
            raise ea

    def _app_init(self, arglist) -> None:
        pass

    def __del__(self) -> None:
        self.app_close()

    def app_close(self):
        self._logger.info("app_close() called")
        if not self.is_closed:
            try:
                if self.is_enable_profiler and self.pr is not None:
                    self.pr.disable()
                    profile_path = f"./calib_profiler_results_{dt.now().strftime('%Y-%m-%d_%H-%M-%S_%f')}.prof"
                    self.pr.dump_stats(profile_path)
                    self._logger.info(
                        f"cProfile.Profile closed, calib profiler file saved: {profile_path}"
                    )
            except Exception as error:
                self._logger.warning(
                    f"cProfile disable or dump_stats failed: {error}; continue shutdown"
                )

            self._logger.info("close monitor")
            if self.uimanager is not None:
                self.uimanager.close()
            self._logger.info("close boss_inst")
            self.boss_inst.close()
            self.is_closed = True

    @staticmethod
    # 精度自動テスト用の終了可否判定 製品版への実装はおそらく危険？
    def check_allowexit(app_config_calib: AppConfigCalibration) -> bool:
        if os.path.isfile(
            app_config_calib.debug.calib2d3d_fileend_autoexit_flagfile_path
        ):
            os.remove(app_config_calib.debug.calib2d3d_fileend_autoexit_flagfile_path)
            if app_config_calib.debug.calib2d3d_fileend_autoexit:
                return True
        return False
