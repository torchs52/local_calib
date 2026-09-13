from __future__ import annotations

import contextlib
import mmap
import multiprocessing as mp
import os
import pickle
import tempfile
import time
from multiprocessing.sharedctypes import Synchronized
from multiprocessing.synchronize import RLock
from pathlib import Path
from typing import Any

from argus_synchro.common.app_logger import AppLogger, AppLoggerFactory
from argus_synchro.diagnosis.error_config import ErrorConfig
from argus_synchro.shared_data import create_shared_single_data


class SharedErrorConfig:
    def __init__(self, error_config_path: str | Path) -> None:
        self._logger: AppLogger = AppLoggerFactory.from_type(self.__class__)
        self._error_config_path = Path(error_config_path)
        self.current_error_config = self._load_error_config()
        tmp_dump = pickle.dumps(self.current_error_config)

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

    @property
    def error_config_path(self) -> Path:
        return self._error_config_path

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

    def _load_error_config(self) -> ErrorConfig:
        if not self._error_config_path.is_file():
            raise FileNotFoundError(
                f"error config file is not found: {self._error_config_path}"
            )
        new_error_config = ErrorConfig()
        new_error_config.load_from_json(self._error_config_path)
        return new_error_config

    def write(self) -> None:
        """
        設定ファイルの変更を共有メモリに乗せる。
        """
        # ロックを掛けてmmapの変更を反映
        try:
            with self._lock:
                # 共有メモリに反映
                new_error_config = self._load_error_config()
                self._mm.seek(0)
                self.current_error_config = new_error_config
                # 文字列を変更したとき、サイズが変わるため、代入する前にmmapをresize
                tmp_dump = pickle.dumps(new_error_config)
                self._mm.resize(len(tmp_dump))
                self._mm[:] = tmp_dump
                self.size.value = len(tmp_dump)
                self._last_updated.value = time.monotonic_ns()

        except (FileNotFoundError, OSError, ValueError):
            self._logger.warning("error_configは更新されませんでした。")
            raise

    def read(self) -> ErrorConfig:
        """
        共有メモリにあるデータをerror_configに(byteから変換して)代入する。
        """
        with self._lock:
            self._mm.seek(0)
            self._mm.resize(self.size.value)
            new_error_config: ErrorConfig = pickle.loads(self._mm)
        return new_error_config

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
