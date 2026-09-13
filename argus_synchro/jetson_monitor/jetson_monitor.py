#!/usr/bin/env python3
import threading
from collections.abc import Callable
from contextlib import suppress

from argus_synchro.common.app_logger import AppLogger, AppLoggerFactory
from argus_synchro.jetson_monitor.jm.app import main
from argus_synchro.jetson_monitor.jm.models import Metrics
from argus_synchro.shared_app_config import SharedAppConfig
from argus_synchro.shared_errors import SharedErrors


class JetsonMonitor:
    """Jetsonのリソース使用率を監視する。"""

    def __init__(
        self,
        sac: SharedAppConfig,
        ser: SharedErrors,
        app_logger_factory: AppLoggerFactory,
        diag_func: Callable[[Metrics], None],
    ) -> None:
        self._logger: AppLogger = app_logger_factory.register_from_name(__name__)
        self._sac: SharedAppConfig = sac
        self._ser: SharedErrors = ser
        self._diag_func: Callable[[Metrics], None] = diag_func
        self._th: threading.Thread | None = None
        self._stop = threading.Event()

    def start(self) -> None:
        """JetsonMonitorを起動する。"""
        if self._th and self._th.is_alive():
            return

        self._th = threading.Thread(
            target=main,
            args=(self._sac, self._diag_func, self._stop),
            name="JetsonMonitor",
            daemon=True,
        )
        self._th.start()

    def log_register(self, app_logger_factory: AppLoggerFactory) -> None:
        """JetsonMonitorのログを登録する。"""
        app_logger_factory.append_logger(self._logger)

    def stop(self) -> None:
        """JetsonMonitorを停止する。"""
        if self._th and self._th.is_alive():
            self._stop.set()
            with suppress(Exception):
                self._th.join(timeout=5)
