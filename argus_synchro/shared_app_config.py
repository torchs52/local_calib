from __future__ import annotations

import configparser
import contextlib
import mmap
import multiprocessing as mp
import os
import pickle
import sys
import tempfile
import time
import traceback
from multiprocessing.sharedctypes import Synchronized
from multiprocessing.synchronize import RLock
from typing import Any

from argus_synchro.check_restart import check_restart_is_required
from argus_synchro.common import paths
from argus_synchro.common.app_logger import AppLogger, AppLoggerFactory
from argus_synchro.config.app_config import AppConfig
from argus_synchro.config.app_config_calibration import AppConfigCalibration
from argus_synchro.process.operation_mode import OPERATION_MODE as OPM
from argus_synchro.shared_data import create_shared_single_data
from argus_synchro.shared_excepts import SharedExcepts


class SharedAppConfig:
    def __init__(
        self,
        directory_config: paths.DirectoryConfig = paths.DEFAULT_DIRECTORY_CONFIG,
    ) -> None:
        self._logger: AppLogger = AppLoggerFactory.from_type(self.__class__)
        self._directory_config, app_ini = paths.load_directory_config_from_ini(
            directory_config
        )
        self.current_appconfig = AppConfig(app_ini, self._directory_config)
        tmp_dump = pickle.dumps(self.current_appconfig)

        fd, self._name = tempfile.mkstemp()
        os.ftruncate(fd, len(tmp_dump))
        # pickleファイルを共有メモリに乗せる。
        self._mm = mmap.mmap(fd, 0)
        os.close(fd)
        self._mm[:] = tmp_dump
        # 共有メモリにあるデータを_appconfigに(byteから変換して)代入する。
        self._mm.seek(0)
        # pickle fileのメモリサイズ
        self.size: Synchronized[int] = create_shared_single_data(len(tmp_dump))
        self._lock: RLock = mp.RLock()
        self._last_updated: Synchronized[int] = mp.Value("Q", lock=False)
        self._last_updated.value = time.monotonic_ns()
        # 再起動処理検知フラグ
        self.is_restart_required: Synchronized[bool] = create_shared_single_data(False)

    @property
    def directory_config(self) -> paths.DirectoryConfig:
        return self._directory_config

    def log_register(self, app_logger_factory: AppLoggerFactory) -> None:
        self._app_logger_factory: AppLoggerFactory = app_logger_factory
        app_logger_factory.append_logger(self._logger)

    @property
    def last_updated(self) -> int:
        return self._last_updated.value

    def __getstate__(self) -> dict[str, Any]:
        """
        サブプロセスに送るときに、mmapを消す
        """
        state = self.__dict__.copy()
        del state["_mm"]
        return state

    def __setstate__(self, d: dict[str, Any]) -> None:
        """
        サブプロセス内でmmapを作る
        """
        self.__dict__ = d
        with open(self._name, "r+b") as f:
            self._mm = mmap.mmap(f.fileno(), 0)
            self._mm.seek(0)

    def write(self, sec: SharedExcepts) -> None:
        """
        設定ファイルの変更を共有メモリに乗せる。
        """
        # ロックを掛けてmmapの変更を反映
        try:
            with self._lock:
                self._directory_config, app_ini = paths.load_directory_config_from_ini(
                    self._directory_config
                )
                # 共有メモリに反映
                new_app_config = AppConfig(app_ini, self._directory_config)
                self._mm.seek(0)
                if check_restart_is_required(
                    old=self.current_appconfig,
                    new=new_app_config,
                ):
                    self.is_restart_required.value = True
                    return
                # モード切替(校正モード になったらscrutinizerを終了させる.)
                # ここでこのように実施するのが良いのかは要検討.
                # TODO: 要修正　Mainで対応 (NSW)
                if new_app_config.General.operation_mode != OPM.CALIB:
                    sec.reset_operation_mode_scrut_ex()
                    sec.CalMatGen_ex.IsFinished.value = True

                self.current_appconfig = new_app_config
                # 文字列を変更したとき、サイズが変わるため、代入する前にmmapをresize
                tmp_dump = pickle.dumps(new_app_config)
                self._mm.resize(len(tmp_dump))
                self._mm[:] = tmp_dump
                self.size.value = len(tmp_dump)
                self._last_updated.value = time.monotonic_ns()

        except (configparser.NoOptionError, ValueError):
            self._logger.warning("app_configは更新されませんでした。")
            raise

    def read(self) -> AppConfig:
        """
        共有メモリにあるデータをappconfigに(byteから変換して)代入する。
        """
        with self._lock:
            self._mm.seek(0)
            self._mm.resize(self.size.value)
            new_app_config: AppConfig = pickle.loads(self._mm)
        return new_app_config

    def close(self) -> None:
        """
        共有メモリ破棄
        """
        if not self._mm.closed:
            self._mm.close()
        if os.path.isfile(self._name):
            os.remove(self._name)

    def __del__(self) -> None:
        with contextlib.suppress(BaseException):
            self.close()


class SharedAppConfigCalibration:
    def __init__(
        self,
        arglist: list[str] = sys.argv,
        inifilepath: str | None = None,
        directory_config: paths.DirectoryConfig = paths.DEFAULT_DIRECTORY_CONFIG,
    ) -> None:
        self._directory_config = directory_config
        self._logger: AppLogger = AppLoggerFactory.from_type(self.__class__)
        # 設定ファイルから読み取ってapp_iniに代入する。
        if inifilepath is None:
            inifilepath = sys.argv[1]
        assert os.path.isfile(inifilepath)
        self.arglist: list[str] = arglist
        self.inifilepath: str = inifilepath

        self.current_appconfig: AppConfigCalibration = self.get_calibconfig()
        tmp_dump = pickle.dumps(self.current_appconfig)

        fd, self._name = tempfile.mkstemp()
        os.ftruncate(fd, len(tmp_dump))
        # pickleファイルを共有メモリに乗せる。
        self._mm = mmap.mmap(fd, 0)
        os.close(fd)
        self._mm[:] = tmp_dump
        # 共有メモリにあるデータを_appconfigに(byteから変換して)代入する。
        self._mm.seek(0)
        # pickle fileのメモリサイズ
        self.size: Synchronized[int] = create_shared_single_data(len(tmp_dump))
        self._lock: RLock = mp.RLock()
        self._last_updated: Synchronized[int] = mp.Value("Q", lock=False)
        self._last_updated.value = time.monotonic_ns()
        # 再起動処理検知フラグ
        self.is_restart_required: Synchronized[bool] = create_shared_single_data(False)

    def get_calibconfig_s(
        self, inifilepath: str, arglist: list[str]
    ) -> AppConfigCalibration:
        return AppConfigCalibration(
            configpath=inifilepath,
            arglist=arglist,
            verb=True,
            directory_config=self._directory_config,
        )

    def get_calibconfig(self) -> AppConfigCalibration:
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
            # TODO エラー通知 (NSW)
            # if self.uimanager is not None:
            #     self.uimanager.set_errorcode_unexpected_exception(True)
            #     self.uimanager.transmit_setdata(
            #         sec=sec, ref_t=None, is_firstframe=False
            #     )
            raise ea

    def log_register(self, app_logger_factory: AppLoggerFactory) -> None:
        self._app_logger_factory: AppLoggerFactory = app_logger_factory
        app_logger_factory.append_logger(self._logger)

    @property
    def last_updated(self) -> int:
        return self._last_updated.value

    def __getstate__(self) -> dict[str, Any]:
        """
        サブプロセスに送るときに、mmapを消す
        """
        state = self.__dict__.copy()
        del state["_mm"]
        return state

    def __setstate__(self, d: dict[str, Any]) -> None:
        """
        サブプロセス内でmmapを作る
        """
        self.__dict__ = d
        with open(self._name, "r+b") as f:
            self._mm = mmap.mmap(f.fileno(), 0)
            self._mm.seek(0)

    def write(self, sec: SharedExcepts) -> None:
        """
        設定ファイルの変更を共有メモリに乗せる。
        """
        # ロックを掛けてmmapの変更を反映
        try:
            with self._lock:
                # 共有メモリに反映
                new_app_config_calib: AppConfigCalibration = self.get_calibconfig()
                self._mm.seek(0)

                self.current_appconfig = new_app_config_calib
                # 文字列を変更したとき、サイズが変わるため、代入する前にmmapをresize
                tmp_dump = pickle.dumps(new_app_config_calib)
                self._mm.resize(len(tmp_dump))
                self._mm[:] = tmp_dump
                self.size.value = len(tmp_dump)
                self._last_updated.value = time.monotonic_ns()

        except (configparser.NoOptionError, ValueError):
            self._logger.warning("app_configは更新されませんでした。")

    def read(self) -> AppConfigCalibration:
        """
        共有メモリにあるデータをappconfigに(byteから変換して)代入する。
        """
        with self._lock:
            self._mm.seek(0)
            self._mm.resize(self.size.value)
            new_app_config: AppConfigCalibration = pickle.loads(self._mm)
        return new_app_config

    def close(self) -> None:
        """
        共有メモリ破棄
        """
        if not self._mm.closed:
            self._mm.close()
        if os.path.isfile(self._name):
            os.remove(self._name)

    def __del__(self) -> None:
        self.close()
