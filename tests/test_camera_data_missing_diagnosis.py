from __future__ import annotations

import typing

import numpy as np

# Python 3.10環境ではtyping.Selfが存在しないため、テスト時のみ補う。
if not hasattr(typing, "Self"):
    typing.Self = typing.Any  # type: ignore[attr-defined]
if not hasattr(typing, "LiteralString"):
    typing.LiteralString = str  # type: ignore[attr-defined]

import argus_synchro.diagnosis.error_config as ErrConf
from argus_synchro.diagnosis.state_d_errors import CameraDataMissing


def _create_diagnosis(
    *,
    black_frame_rate_window_sec: float = 3.0,
    black_frame_rate_threshold: float = 0.2,
) -> CameraDataMissing:
    diagnosis = CameraDataMissing()
    err_conf = ErrConf.ErrorConfig().load_from_dict(
        {
            "camera_data_missing": {
                "is_enabled": True,
                "black_frame_rate_window_sec": black_frame_rate_window_sec,
                "black_frame_rate_threshold": black_frame_rate_threshold,
            }
        }
    )
    diagnosis.update(err_conf)
    return diagnosis


def _black_image() -> np.ndarray:
    return np.zeros((2, 2, 3), dtype=np.uint8)


def _normal_image() -> np.ndarray:
    return np.full((2, 2, 3), 255, dtype=np.uint8)


def test_detect_error_normal_to_error_to_normal_by_time_series() -> None:
    diagnosis = _create_diagnosis(
        black_frame_rate_window_sec=3.0,
        black_frame_rate_threshold=0.2,
    )

    # 正常フェーズ: 直近3秒の黒率が20%未満。
    assert diagnosis.detect_error(0.0, _normal_image()) is False
    assert diagnosis.detect_error(0.5, _normal_image()) is False
    assert diagnosis.detect_error(1.0, _normal_image()) is False
    assert diagnosis.detect_error(1.5, _normal_image()) is False
    assert diagnosis.detect_error(2.0, _normal_image()) is False
    assert diagnosis.detect_error(2.5, _black_image()) is False  # 1/6 = 16.6%

    # エラーフェーズ: 間欠的な黒画面で20%以上になる。
    assert diagnosis.detect_error(2.6, _black_image()) is True  # 2/7 = 28.5%

    # 正常復帰フェーズ: 黒フレームが3秒窓から外れて20%未満に戻る。
    assert diagnosis.detect_error(6.0, _normal_image()) is False


def test_detect_error_returns_true_at_exactly_20_percent() -> None:
    diagnosis = _create_diagnosis(
        black_frame_rate_window_sec=3.0,
        black_frame_rate_threshold=0.2,
    )

    # 5フレーム中1フレーム黒: 20%以上(>=)なのでTrue。
    assert diagnosis.detect_error(0.0, _normal_image()) is False
    assert diagnosis.detect_error(0.5, _normal_image()) is False
    assert diagnosis.detect_error(1.0, _normal_image()) is False
    assert diagnosis.detect_error(1.5, _normal_image()) is False
    assert diagnosis.detect_error(2.0, _black_image()) is True