from __future__ import annotations

import time
import traceback
from threading import Timer

from watchdog.events import (
    FileSystemEvent,
    RegexMatchingEventHandler,
)
from watchdog.observers import Observer

from argus_synchro.common import paths
from argus_synchro.common.app_logger import AppLogger, AppLoggerFactory
from argus_synchro.config.app_config import AppConfig
from argus_synchro.machine_profile import MachineProfileHandler
from argus_synchro.shared_app_config import SharedAppConfig
from argus_synchro.shared_errors import ActionErrorIndex, SharedErrors
from argus_synchro.shared_excepts import SharedExcepts


class DebouncedEventHandler(RegexMatchingEventHandler):
    _RELOAD_RETRY_COUNT = 3
    _RELOAD_RETRY_INTERVAL_SEC = 0.2

    def __init__(
        self,
        sac: SharedAppConfig,
        sec: SharedExcepts,
        ser: SharedErrors,
        logger: AppLogger,
        regexes: tuple[str] = (r".*(\\|/)settings.ini",),
        debounce_time: float = 1.0,
    ) -> None:
        super().__init__(regexes=list(regexes))

        self.debounce_time: float = debounce_time
        self.sac: SharedAppConfig = sac
        self._sec: SharedExcepts = sec
        self._ser: SharedErrors = ser
        self.events: dict[bytes | str, Timer] = {}
        self._logger: AppLogger = logger

    def _schedule_event(self, path: bytes | str) -> None:
        if path in self.events:
            self.events[path].cancel()
        timer = Timer(self.debounce_time, self.process_event, [path])
        self.events[path] = timer
        timer.start()

    def on_modified(self, event: FileSystemEvent) -> None:
        self._schedule_event(event.src_path)

    def on_created(self, event: FileSystemEvent) -> None:
        self._schedule_event(event.src_path)

    def on_closed(self, event: FileSystemEvent) -> None:
        self._schedule_event(event.src_path)

    def on_moved(self, event: FileSystemEvent) -> None:
        moved_path = getattr(event, "dest_path", event.src_path)
        self._schedule_event(moved_path)

    def process_event(self, path: str) -> None:
        try:
            # atomic save中の一時的な欠損・書込み途中をCE005と誤判定しない。
            for attempt in range(self._RELOAD_RETRY_COUNT):
                try:
                    self.sac.write(sec=self._sec)
                    return
                except Exception:
                    if attempt == self._RELOAD_RETRY_COUNT - 1:
                        raise
                    time.sleep(self._RELOAD_RETRY_INTERVAL_SEC)
        except Exception as e:
            error: bool = self._ser.action_errors_A_C[
                ActionErrorIndex.CONFIG_FILE_MISSING
            ].excepts_diagnosis(e)
            if error:
                self._logger.error(
                    f"file open error: {e!r}, traceback: {traceback.format_exc()}"
                )
            else:
                self._logger.error(
                    f"Unknown error: {e!r}, traceback: {traceback.format_exc()}"
                )

        finally:
            if path in self.events:
                del self.events[path]


class ChangeModelEventHandler(RegexMatchingEventHandler):
    def __init__(
        self,
        ser: SharedErrors,
        regexes: tuple[str],
        debounce_time: float,
        app_logger_factory: AppLoggerFactory,
        directory_config: paths.DirectoryConfig = paths.DEFAULT_DIRECTORY_CONFIG,
        apply_calib: bool = False,
    ) -> None:
        super().__init__(regexes=list(regexes))
        self._ser: SharedErrors = ser
        self.debounce_time: float = debounce_time
        self.events: dict[bytes | str, Timer] = {}
        MachineProfileHandler.log_register(app_logger_factory)
        self.mprof_handler = MachineProfileHandler(app_logger_factory, directory_config)
        # 監視対象がsettings.ini用かcalib_settings.ini用かを切り替える
        self._apply_calib: bool = apply_calib
        self._logger: AppLogger = app_logger_factory.register_from_type(self.__class__)

    def _schedule_event(self, path: bytes | str) -> None:
        if path in self.events:
            self.events[path].cancel()
        timer = Timer(self.debounce_time, self.process_event, [path])
        self.events[path] = timer
        timer.start()

    def on_modified(self, event: FileSystemEvent) -> None:
        self._schedule_event(event.src_path)

    def on_closed(self, event: FileSystemEvent) -> None:
        self._schedule_event(event.src_path)

    def on_moved(self, event: FileSystemEvent) -> None:
        moved_path = getattr(event, "dest_path", event.src_path)
        self._schedule_event(moved_path)

    def process_event(self, path: str) -> None:
        # ここで機体設定をsettings.ini(またはcalib_settings.ini)に反映
        try:
            if self._apply_calib:
                self.mprof_handler.apply_model_specific_calib_config()
            else:
                self.mprof_handler.apply_model_specific_config()
        except Exception as e:
            error: bool = self._ser.action_errors_A_C[
                ActionErrorIndex.CONFIG_FILE_MISSING
            ].excepts_diagnosis(e)
            if error:
                self._logger.error(
                    f"file open error: {e!r}, traceback: {traceback.format_exc()}"
                )
            else:
                self._logger.error(
                    f"Unknown error: {e!r}, traceback: {traceback.format_exc()}"
                )
        finally:
            if path in self.events:
                del self.events[path]


if __name__ == "__main__":
    path = "./config"
    sac = SharedAppConfig()
    app_config: AppConfig = sac.read()
    sec = SharedExcepts(app_config)
    ser = SharedErrors()
    _logger: AppLogger = AppLoggerFactory.from_name("DebouncedEventHandler_main")
    event_handler = DebouncedEventHandler(
        sac=sac, sec=sec, ser=ser, debounce_time=0.1, logger=_logger
    )
    observer = Observer()
    observer.schedule(event_handler, path)
    observer.start()
    count = 0
    try:
        while observer.is_alive():
            time.sleep(1)
            count += 1
            current_app_config = sac.read()
            _logger.info(str(count), current_app_config.DEFAULT.debug_log)
    finally:
        observer.stop()
        observer.join()
