import mmap
import os
import signal
import struct
import sys
import time
import types
from collections.abc import Callable
from enum import Enum
from pathlib import Path
from subprocess import Popen

from argus_synchro.common import paths
from argus_synchro.common.app_logger import AppLogger


class StatusCode(Enum):
    INIT = 0
    REBOOT = 1
    BOOTING = 2
    RUNNING = 3
    ERROR = -2
    SHUTDOWN = -1


class StatusMMAP:
    size: int = 4
    _last_read_time: float = 0.0

    def __init__(
        self,
        logger: AppLogger,
        *,
        create: bool = False,
        directory_config: paths.DirectoryConfig,
    ) -> None:
        self._logger = logger
        self.path: str = str(paths.get_mmap_dir(directory_config, "status.mmap"))

        status_mmap_path = Path(self.path)

        if create:
            if status_mmap_path.exists():
                self._logger.info(f"[StatusMMAP] 既存 mmap 削除: {status_mmap_path}")
                status_mmap_path.unlink()

            status_mmap_path.parent.mkdir(parents=True, exist_ok=True)
            status_mmap_path.write_bytes(b"\x00" * self.size)

        self.fd: int = os.open(self.path, os.O_RDWR)
        self.mmap: mmap.mmap = mmap.mmap(self.fd, self.size)
        self._closed: bool = False

    def write_status(self, code: int | StatusCode) -> None:
        self.mmap.seek(0)
        if isinstance(code, StatusCode):
            code = code.value
        self.mmap.write(struct.pack("i", code))
        self.mmap.flush()

    def read_status(self) -> int:
        self.mmap.seek(0)
        data: bytes = self.mmap.read(self.size)
        StatusMMAP._last_read_time = time.monotonic()
        return struct.unpack("i", data)[0]

    def close(self) -> None:
        if self._closed:
            return
        self.mmap.close()
        os.close(self.fd)
        self._closed = True

    @staticmethod
    def is_recent(timeout: float = 5.0) -> bool:
        return (time.monotonic() - StatusMMAP._last_read_time) < timeout

    @staticmethod
    def get_status_name(code: int) -> str:
        return next((e.name for e in StatusCode if e.value == code), "UNKNOWN")


def setup_signal_handlers(
    status_obj: StatusMMAP,
    logger: AppLogger,
    name: str = "Process",
    godot_proc_getter: Callable[[], Popen[bytes] | None] | None = None,
    shutdown_callback: Callable[[], None] | None = None,
) -> None:
    def shutdown_handler(signum: int, frame: types.FrameType | None) -> None:
        logger.info(f"[{name}] シャットダウン検知: signal={signum}")
        status_obj.write_status(StatusCode.SHUTDOWN)

        if shutdown_callback is not None:
            shutdown_callback()

        if godot_proc_getter is not None:
            proc = godot_proc_getter()
            if proc and proc.poll() is None:
                logger.info(f"[{name}] Godot 停止 (signal handler)")
                proc.terminate()
                proc.wait()

        status_obj.close()
        # argus_synchroが正常に終了する前に強制終了されるため、os._exitを使用しない (NSW)
        # os._exit(0)
        sys.exit(0)

    signal.signal(signal.SIGINT, shutdown_handler)
    try:
        signal.signal(signal.SIGTERM, shutdown_handler)
    except AttributeError:
        pass

    try:
        signal.signal(signal.SIGBREAK, shutdown_handler)
    except AttributeError:
        pass