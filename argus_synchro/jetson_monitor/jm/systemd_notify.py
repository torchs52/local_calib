from __future__ import annotations

try:
    from systemd.daemon import notify as sd_notify  # type: ignore
except Exception:
    sd_notify = None


def notify_ready() -> None:
    if sd_notify is None:
        return
    try:
        sd_notify("READY=1")
    except Exception:
        pass


def notify_watchdog() -> None:
    if sd_notify is None:
        return
    try:
        sd_notify("WATCHDOG=1")
    except Exception:
        pass
    