from pathlib import Path

import pytest

from argus_synchro.SystemMonitor import MonitorArgus


def test_write_heartbeat_preserves_temporary_file_creation_error(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    heartbeat_path = tmp_path / "monitor_argus_last_heartbeat"

    def fail_to_create(*_args: object, **_kwargs: object) -> None:
        raise OSError("temporary file creation failed")

    monkeypatch.setattr(MonitorArgus.tempfile, "NamedTemporaryFile", fail_to_create)

    with pytest.raises(OSError, match="temporary file creation failed"):
        MonitorArgus._write_heartbeat(heartbeat_path)


def test_write_heartbeat_replaces_file_atomically(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    heartbeat_path = tmp_path / "monitor_argus_last_heartbeat"
    heartbeat_path.write_text("old", encoding="utf-8")
    monkeypatch.setattr(MonitorArgus.time, "perf_counter", lambda: 123.5)

    MonitorArgus._write_heartbeat(heartbeat_path)

    assert heartbeat_path.read_text(encoding="utf-8") == "123.5"


def test_write_heartbeat_removes_temporary_file_after_replace_error(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    heartbeat_path = tmp_path / "monitor_argus_last_heartbeat"
    temporary_path: Path | None = None

    def fail_to_replace(source: str, _destination: Path) -> None:
        nonlocal temporary_path
        temporary_path = Path(source)
        raise OSError("replace failed")

    monkeypatch.setattr(MonitorArgus.os, "replace", fail_to_replace)

    with pytest.raises(OSError, match="replace failed"):
        MonitorArgus._write_heartbeat(heartbeat_path)

    assert temporary_path is not None
    assert not temporary_path.exists()