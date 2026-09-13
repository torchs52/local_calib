from types import SimpleNamespace
from unittest.mock import MagicMock

from argus_synchro.__main__ import _reload_app_config_if_updated


class SharedAppConfigStub:
    def __init__(self) -> None:
        self.version = 2
        self.read_count = 0

    @property
    def last_updated(self) -> int:
        return self.version

    def read(self):
        self.read_count += 1
        self.version += 1
        return SimpleNamespace(General=SimpleNamespace(operation_mode=1))


def test_reload_keeps_detected_version_when_config_changes_during_read() -> None:
    sac = SharedAppConfigStub()
    current_config = SimpleNamespace(General=SimpleNamespace(operation_mode=0))
    logger = MagicMock()

    current_config, last_updated, reloaded = _reload_app_config_if_updated(
        sac, current_config, 1, logger
    )

    assert reloaded is True
    assert last_updated == 2
    assert sac.last_updated == 3

    _, last_updated, reloaded = _reload_app_config_if_updated(
        sac, current_config, last_updated, logger, "after CALIB restart"
    )

    assert reloaded is True
    assert last_updated == 3
    assert sac.read_count == 2
    logger.info.assert_any_call(
        "config reloaded%s (last_updated=%s, operation_mode=%s)",
        " after CALIB restart",
        3,
        1,
    )


def test_reload_skips_read_when_config_has_not_changed() -> None:
    sac = SharedAppConfigStub()
    current_config = SimpleNamespace(General=SimpleNamespace(operation_mode=0))
    logger = MagicMock()

    app_config, last_updated, reloaded = _reload_app_config_if_updated(
        sac, current_config, sac.last_updated, logger
    )

    assert app_config is current_config
    assert last_updated == 2
    assert reloaded is False
    assert sac.read_count == 0
    logger.info.assert_not_called()