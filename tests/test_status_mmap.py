from __future__ import annotations

import signal
from pathlib import Path
from typing import Any

import pytest

from argus_synchro.common.app_logger import AppLoggerFactory
from argus_synchro.common.paths import DirectoryConfig
from argus_synchro.SystemMonitor.status_mmap import (
    StatusCode,
    StatusMMAP,
    setup_signal_handlers,
)


class FakeStatus:
    def __init__(self) -> None:
        self.writes: list[StatusCode] = []
        self.close_count = 0

    def write_status(self, code: StatusCode) -> None:
        self.writes.append(code)

    def close(self) -> None:
        self.close_count += 1


def test_setup_signal_handlers_exits_after_shutdown(monkeypatch: pytest.MonkeyPatch) -> None:
    handlers: dict[int, Any] = {}
    fake_status = FakeStatus()
    logger = AppLoggerFactory.from_name("StatusMMAPTest", to_console=False)
    shutdown_calls: list[None] = []

    monkeypatch.setattr(signal, "signal", handlers.__setitem__)

    setup_signal_handlers(
        fake_status,
        logger=logger,
        name="Test",
        shutdown_callback=lambda: shutdown_calls.append(None),
    )

    with pytest.raises(SystemExit) as exc_info:
        handlers[signal.SIGINT](signal.SIGINT, None)

    assert exc_info.value.code == 0
    assert fake_status.writes == [StatusCode.SHUTDOWN]
    assert shutdown_calls == [None]
    assert fake_status.close_count == 1


def test_status_mmap_close_is_idempotent(
    tmp_path: Path,
) -> None:
    directory_config = DirectoryConfig(tmp_path, tmp_path, tmp_path)
    logger = AppLoggerFactory.from_name("StatusMMAPTest", to_console=False)
    status = StatusMMAP(logger, create=True, directory_config=directory_config)

    status.close()
    status.close()


def test_status_mmap_logger_uses_registered_factory(
    tmp_path: Path,
) -> None:
    mmap_path = tmp_path / "status.mmap"
    mmap_path.write_bytes(b"\x00" * StatusMMAP.size)
    directory_config = DirectoryConfig(tmp_path, tmp_path, tmp_path)

    log_file = tmp_path / "argus.log"
    app_logger_factory = AppLoggerFactory(to_console=False, to_file=str(log_file))
    logger = app_logger_factory.register_from_name("StatusMMAP")
    app_logger_factory.update()

    status = StatusMMAP(logger, create=True, directory_config=directory_config)
    status.close()

    assert "既存 mmap 削除" in log_file.read_text(encoding="utf-8")


def test_status_mmap_recency_uses_monotonic_time(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    directory_config = DirectoryConfig(tmp_path, tmp_path, tmp_path)
    logger = AppLoggerFactory.from_name("StatusMMAPTest", to_console=False)
    monotonic_time = 100.0
    wall_time = 1_000.0
    monkeypatch.setattr(
        "argus_synchro.SystemMonitor.status_mmap.time.monotonic",
        lambda: monotonic_time,
    )
    monkeypatch.setattr(
        "argus_synchro.SystemMonitor.status_mmap.time.time", lambda: wall_time
    )
    status = StatusMMAP(logger, create=True, directory_config=directory_config)

    status.read_status()
    wall_time += 1_000_000.0
    monotonic_time += 4.9
    assert StatusMMAP.is_recent(timeout=5.0) is True
    wall_time -= 2_000_000.0
    monotonic_time += 0.2
    assert StatusMMAP.is_recent(timeout=5.0) is False

    status.close()
