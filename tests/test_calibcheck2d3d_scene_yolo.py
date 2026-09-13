# ruff: noqa: PLR2004, SLF001

from __future__ import annotations

from types import SimpleNamespace
from typing import cast

import numpy as np
import pytest

from argus_synchro.calibration_mat_generator_modules.ctrl.calibcheck2d3d.SceneDesc import (
    Scene,
    calc_iou,
)
from argus_synchro.calibration_mat_generator_modules.ctrl.calibcheck2d3d.YOLOadapter import (
    YOLODamoBatchAdapter,
)
from argus_synchro.config.app_config import SceneDescriptionConf
from argus_synchro.shared_excepts import SharedExcepts


def _scene() -> Scene:
    return Scene(
        SceneDescriptionConf(
            coarse_lo=0.4,
            coarse_hi=2.5,
            k_min=0.3,
            h_ref_px=80,
            lo_gain=1.0,
            hi_gain=1.0,
            lo_floor=0.1,
            hi_ceil=5.0,
            vertical_w_iou=1.0,
            vertical_w_scale=1.0,
            vertical_w_phi=1.0,
            final_threshold=10.0,
            use_human_gate=True,
            H_min=1.2,
            H_max=2.2,
            W_min=0.2,
            W_max=1.0,
            D_min=0.2,
            D_max=1.0,
            tall_ratio_min=1.5,
        )
    )


def test_scene_iou_and_human_size_gate() -> None:
    assert calc_iou(0, 0, 2, 2, 1, 1, 3, 3) == pytest.approx(
        (1 / 7, 1, 4, 4)
    )

    scene = _scene()
    assert scene.passes_human_size((0.0, 0.6, 0.0, 0.5, 0.0, 1.8)) is True
    assert scene.passes_human_size((0.0, 1.5, 0.0, 0.5, 0.0, 1.8)) is False


def test_scene_selects_matching_projected_bbox() -> None:
    scene = _scene()
    box2d = np.array([0.2, 0.2, 0.8, 0.8], dtype=np.float32)
    matching = np.array(
        [[20, 20], [80, 20], [20, 50], [80, 50], [20, 50], [80, 50], [20, 80], [80, 80]],
        dtype=np.float32,
    )
    distant = matching + np.array([200, 0], dtype=np.float32)
    box3ds = np.concatenate((matching, distant))

    assert scene.get_human_3bb(box2d, 100, 100, box3ds, 2, method="center") == 0


class _DetectorStub:
    def object_detect(self, *, sec: SharedExcepts, frames: np.ndarray) -> SimpleNamespace:
        del sec
        assert frames.shape == (3, 2, 2, 3)
        return SimpleNamespace(
            boxes=np.arange(24, dtype=np.float32).reshape(3, 2, 4),
            scores=np.ones((3, 2), dtype=np.float32),
            classes=np.zeros((3, 2), dtype=np.float32),
            valid_detects=np.array([2, 1, 2], dtype=np.int32),
        )


def test_yolo_adapter_preserves_camera_slots_for_missing_frames() -> None:
    adapter = cast(YOLODamoBatchAdapter, object.__new__(YOLODamoBatchAdapter))
    adapter._camera_count = 3
    adapter._ser = None
    adapter._detector = _DetectorStub()
    frame = np.ones((2, 2, 3), dtype=np.uint8)

    results = adapter.predict_batch(
        cast(SharedExcepts, object()), [frame, None, frame]
    )

    assert len(results) == 3
    assert int(results[0][3]) == 2
    assert results[1][0].shape == (0, 4)
    assert int(results[1][3]) == 0
    assert int(results[2][3]) == 2


def test_yolo_adapter_returns_empty_results_when_all_frames_are_missing() -> None:
    adapter = cast(YOLODamoBatchAdapter, object.__new__(YOLODamoBatchAdapter))
    adapter._camera_count = 3
    adapter._ser = None
    adapter._detector = _DetectorStub()

    results = adapter.predict_batch(
        cast(SharedExcepts, object()), [None, None, None]
    )

    assert [int(result[3]) for result in results] == [0, 0, 0]
    assert [result[0].shape for result in results] == [(0, 4), (0, 4), (0, 4)]


def test_yolo_adapter_rejects_wrong_camera_count() -> None:
    adapter = cast(YOLODamoBatchAdapter, object.__new__(YOLODamoBatchAdapter))
    adapter._camera_count = 3

    with pytest.raises(ValueError, match="frames length must match camera count"):
        adapter.predict_batch(cast(SharedExcepts, object()), [None])
