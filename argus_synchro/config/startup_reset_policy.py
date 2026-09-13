from collections.abc import Mapping
from dataclasses import dataclass
from typing import Protocol

from argus_synchro.common.app_logger import AppLogger, AppLoggerFactory


class SupportsIni(Protocol):
    def has_section(self, section: str) -> bool: ...
    def has_option(self, section: str, option: str) -> bool: ...
    def get(self, section: str, option: str) -> str: ...
    def set(self, section: str, option: str, value: str) -> None: ...


@dataclass(frozen=True)
class StartupResetChange:
    section: str
    key: str
    before: str
    after: str


DEFAULT_STARTUP_RESET_VALUES: dict[str, dict[str, str]] = {
    # NOTE:　再起動時に前回の動作モードを継続するため、デフォルトのoperation_modeは指定しない
    "General": {"operation_mode": "0"},
    "CalibMode": {
        "cameraID": "0",
        "isRunning3D3Dcalib": "False",
        "isRunning2D3Dcalib": "False",
        "start2D3DCalibCalc": "False",
        "isRunning2D3Dcheck": "False",
        "start2D3DCheckCalc": "False",
        "isRunningInterfaceDebug": "False",
    },
}


class StartupResetPolicy:
    """
    起動時に ini を必要な設定値へ戻すポリシー。
    書き換えが1件でもあれば WARNING を出す
    変更詳細は INFO で出す
    """

    def __init__(self, app_logger_factory: AppLoggerFactory) -> None:
        self._logger: AppLogger = app_logger_factory.register_from_type(self.__class__)
        self._forced_values: Mapping[str, Mapping[str, str]] = (
            DEFAULT_STARTUP_RESET_VALUES
        )

    def apply(self, ini: SupportsIni) -> list[StartupResetChange]:
        changes: list[StartupResetChange] = []

        for section, kv in self._forced_values.items():
            if not ini.has_section(section):
                continue

            for key, after in kv.items():
                if not ini.has_option(section, key):
                    continue

                before = ini.get(section, key)
                if before != after:
                    ini.set(section, key, after)
                    changes.append(
                        StartupResetChange(
                            section=section,
                            key=key,
                            before=before,
                            after=after,
                        )
                    )

        if changes:
            self._logger.warning(f"起動時リセット適用: {len(changes)} 件")
            for ch in changes:
                self._logger.info(
                    f"起動時リセット適用: [{ch.section}] {ch.key}={ch.before} -> {ch.after}"
                )

        return changes
