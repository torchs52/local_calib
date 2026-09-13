from types import SimpleNamespace
from unittest.mock import MagicMock

import pytest

from argus_synchro.diagnosis.reduced_load_mode import (
    FORCE_ENABLED_ENV,
    PROC_SPEED_NORMAL,
    PROC_SPEED_SLOW,
    ReducedLoadMode,
)
from argus_synchro.process.visual_process import VisualProcess


def test_force_enabled_environment_keeps_mode_enabled(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setenv(FORCE_ENABLED_ENV, "1")
    mode = ReducedLoadMode()

    for _ in range(ReducedLoadMode.DISABLE_THRESHOLD + 1):
        mode.update_state()

    assert mode.enabled


def test_point_count_thresholds_require_five_consecutive_frames() -> None:
    mode = ReducedLoadMode()
    mode.update_proc_speed(PROC_SPEED_SLOW)

    mode.update_pcd_nums(mode.many_points_threshold)
    for _ in range(ReducedLoadMode.PCD_ENABLE_THRESHOLD):
        mode.update_state()
    assert not mode.enabled

    mode.update_pcd_nums(mode.many_points_threshold + 1)
    for _ in range(ReducedLoadMode.PCD_ENABLE_THRESHOLD - 1):
        mode.update_state()
        assert not mode.enabled
    mode.update_state()
    assert mode.enabled

    mode.update_proc_speed(PROC_SPEED_NORMAL)
    mode.update_pcd_nums(mode.few_points_threshold)
    for _ in range(ReducedLoadMode.DISABLE_THRESHOLD):
        mode.update_state()
    assert mode.enabled

    mode.update_pcd_nums(mode.few_points_threshold - 1)
    for _ in range(ReducedLoadMode.DISABLE_THRESHOLD - 1):
        mode.update_state()
        assert mode.enabled
    mode.update_state()
    assert not mode.enabled


def test_point_count_thresholds_are_configurable_ratios() -> None:
    mode = ReducedLoadMode()
    expected_many_points = 20_000
    expected_few_points = 10_000

    mode.configure(0.5, 0.25)

    assert mode.many_points_threshold == expected_many_points
    assert mode.few_points_threshold == expected_few_points


def test_visual_config_load_applies_reduced_load_ratios(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    app_config = SimpleNamespace(
        ReducedLoadMode=SimpleNamespace(
            many_points_ratio=0.5,
            few_points_ratio=0.25,
        )
    )
    process = object.__new__(VisualProcess)
    process._sac = SimpleNamespace(read=MagicMock(return_value=app_config), last_updated=2)
    process._ser = SimpleNamespace(reduced_load_mode=MagicMock())
    process._logger = MagicMock()
    monkeypatch.setattr(VisualProcess, "_get_SceneDescription", MagicMock())
    monkeypatch.setattr(
        "argus_synchro.process.visual_process.argus_synchro_lib.scene.Scene",
        MagicMock(),
    )

    process._config_load()

    process._ser.reduced_load_mode.configure.assert_called_once_with(0.5, 0.25)
