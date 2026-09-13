from __future__ import annotations

import numpy as np

from argus_synchro.calibration_mat_generator_modules.ctrl.calibcheck2d3d import (
    Scene_CalibCheck2d3d,
)
from argus_synchro.config.app_config import SceneDescriptionConf


def test_calibcheck_scene_links_person_detection_to_3d_bbox() -> None:
    scene = Scene_CalibCheck2d3d.create_for_evaluation(
        scene_conf=SceneDescriptionConf(
            coarse_lo=0.01,
            coarse_hi=100.0,
            k_min=0.3,
            h_ref_px=80,
            lo_gain=1.0,
            hi_gain=1.0,
            lo_floor=0.01,
            hi_ceil=100.0,
            vertical_w_iou=0.0,
            vertical_w_scale=0.0,
            vertical_w_phi=0.0,
            final_threshold=1000.0,
            use_human_gate=False,
            H_min=1.2,
            H_max=2.2,
            W_min=0.2,
            W_max=1.0,
            D_min=0.2,
            D_max=1.0,
            tall_ratio_min=1.5,
        ),
        camera_intrinsics=np.array(
            [[100.0, 0.0, 50.0], [0.0, 100.0, 50.0], [0.0, 0.0, 1.0]],
            dtype=np.float32,
        ),
        image_width=100,
        image_height=100,
    )
    boxpoints = np.array(
        [
            [-1.0, -1.0, 5.0],
            [1.0, -1.0, 5.0],
            [-1.0, 1.0, 5.0],
            [1.0, 1.0, 5.0],
            [-1.0, -1.0, 6.0],
            [1.0, -1.0, 6.0],
            [-1.0, 1.0, 6.0],
            [1.0, 1.0, 6.0],
        ],
        dtype=np.float64,
    )

    result = scene.integrate2d3d_calibcheck(
        rvec=np.zeros((3, 1), dtype=np.float64),
        tvec=np.zeros((3, 1), dtype=np.float64),
        boxpoints=boxpoints,
        minmax3ds=np.array([[-1, 1, -1, 1, 5, 6]], dtype=np.float64),
        bbox2d=np.array([[0.25, 0.25, 0.75, 0.75]], dtype=np.float32),
        yolo_classes=np.array([0], dtype=np.int32),
        n_clusters=1,
        bbox2d_detection_count=1,
        method="endpoints",
    )

    assert result == {0: "HUMAN"}
