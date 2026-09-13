from __future__ import annotations

import sys
import types
import typing

# Python 3.10環境ではtyping.Selfが存在しないため、テスト時のみ補う。
if not hasattr(typing, "Self"):
    typing.Self = typing.Any  # type: ignore[attr-defined]
if not hasattr(typing, "LiteralString"):
    typing.LiteralString = str  # type: ignore[attr-defined]

# state_d_errors は ProcessBase の型参照のために argus_synchro.process をimportする。
# テストではネイティブ依存を回避するため、最小スタブを事前登録する。
process_pkg = types.ModuleType("argus_synchro.process")
process_mod = types.ModuleType("argus_synchro.process.process")


class _DummyProcessBase:
    pass


process_mod.ProcessBase = _DummyProcessBase
process_pkg.process = process_mod
sys.modules.setdefault("argus_synchro.process", process_pkg)
sys.modules.setdefault("argus_synchro.process.process", process_mod)

import argus_synchro.diagnosis.error_config as ErrConf
from argus_synchro.diagnosis.state_d_errors import MemoryLeakDetected


def _create_diagnosis(
    *,
    window_sec: float = 3.0,
    min_samples: int = 3,
    leak_ratio_threshold: float = 1.5,
) -> MemoryLeakDetected:
    diagnosis = MemoryLeakDetected()
    err_conf = ErrConf.ErrorConfig().load_from_dict(
        {
            "memory_leak_detected": {
                "is_enabled": True,
                "window_sec": window_sec,
                "min_samples": min_samples,
                "leak_ratio_threshold": leak_ratio_threshold,
            }
        }
    )
    diagnosis.update(err_conf)
    return diagnosis


def test_detect_error_sets_first_window_min_as_baseline_and_returns_false() -> None:
    diagnosis = _create_diagnosis(window_sec=3.0, min_samples=3, leak_ratio_threshold=1.5)

    # 1st window: 最小値は9.0
    assert diagnosis.detect_error(10, 0.0) is False
    assert diagnosis.detect_error(9, 1.0) is False
    assert diagnosis.detect_error(11, 2.0) is False

    # window_sec到達時に評価されるが、初回は基準値設定のみでFalse
    assert diagnosis.detect_error(10, 3.0) is False


def test_detect_error_returns_true_when_window_min_reaches_threshold_multiple() -> None:
    diagnosis = _create_diagnosis(window_sec=3.0, min_samples=3, leak_ratio_threshold=1.5)

    # 基準ウィンドウ最小値: 9.0
    assert diagnosis.detect_error(10, 0.0) is False
    assert diagnosis.detect_error(9, 1.0) is False
    assert diagnosis.detect_error(11, 2.0) is False
    assert diagnosis.detect_error(10, 3.0) is False

    # 2nd window最小値: 14.0 (9.0 * 1.5 = 13.5 以上) -> True
    assert diagnosis.detect_error(14, 3.1) is False
    assert diagnosis.detect_error(15, 4.0) is False
    assert diagnosis.detect_error(16, 5.0) is False
    assert diagnosis.detect_error(14, 6.0) is True


def test_detect_error_returns_false_when_sample_count_is_insufficient() -> None:
    diagnosis = _create_diagnosis(window_sec=3.0, min_samples=5, leak_ratio_threshold=1.5)

    # 基準ウィンドウ最小値: 9.0
    assert diagnosis.detect_error(10, 0.0) is False
    assert diagnosis.detect_error(9, 1.0) is False
    assert diagnosis.detect_error(11, 2.0) is False
    assert diagnosis.detect_error(10, 3.0) is False

    # 閾値は満たすが、サンプル数4 < min_samples=5 のため False
    assert diagnosis.detect_error(14, 3.1) is False
    assert diagnosis.detect_error(15, 4.0) is False
    assert diagnosis.detect_error(16, 5.0) is False
    assert diagnosis.detect_error(14, 6.0) is False
