from unittest.mock import MagicMock

from argus_synchro import __main__ as app_main
from argus_synchro.process.command_daemon_tegrastats import CommandDaemon


def test_process_wait_deadline_uses_monotonic(monkeypatch) -> None:
    process = MagicMock()
    process.is_alive.return_value = True
    monotonic = MagicMock(side_effect=(10.0, 11.0))
    monkeypatch.setattr(app_main.time, "monotonic", monotonic)
    monkeypatch.setattr(app_main.time, "sleep", lambda _seconds: None)

    app_main._phase_wait_until(10.5, [process])

    assert monotonic.call_count == 2


def test_command_daemon_timeout_uses_monotonic(monkeypatch) -> None:
    daemon = CommandDaemon(name="test", cmd=["unused"], timeout_sec=2.0)
    daemon._stop = MagicMock()
    daemon._stop.is_set.side_effect = (False, True)
    daemon._read_line = MagicMock(return_value="")
    daemon._restart = MagicMock()
    daemon._terminate = MagicMock()
    monotonic = MagicMock(side_effect=(10.0, 13.0))
    monkeypatch.setattr(
        "argus_synchro.process.command_daemon_tegrastats.time.monotonic", monotonic
    )
    monkeypatch.setattr(
        "argus_synchro.process.command_daemon_tegrastats.time.sleep",
        lambda _seconds: None,
    )

    daemon._run()

    daemon._restart.assert_called_once_with()
    daemon._terminate.assert_called_once_with()
    assert monotonic.call_count == 2