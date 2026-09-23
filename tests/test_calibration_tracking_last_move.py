# ruff: noqa: PLR2004, SLF001

from __future__ import annotations

from types import SimpleNamespace
from typing import Any, cast
from unittest.mock import MagicMock, patch

import numpy as np
import pytest

from argus_synchro.calibration_mat_generator_modules.ctrl.calibration2d3d.track_main.detect2D.person_tracker_SORT_2d import (
    proc2d_bboxtracker_recorder,
)
from argus_synchro.calibration_mat_generator_modules.ctrl.calibration2d3d.track_main.detect3D.person_tracker_SORT_3d import (
    proc3d_bboxtracker_recorder,
)


class _LutStub:
    def evaluate(self, x: float, y: float) -> float:
        del x, y
        return 1.0


def test_2d_tracker_uses_the_matching_default_for_each_lut() -> None:
    proc2d = SimpleNamespace(
        lost_track_buffer=30,
        tracking_frame_rate=10.0,
        track_activation_threshold=0.25,
        minimum_consecutive_frames=3,
        minimum_iou_threshold=0.3,
        cam_valmat_coord_A_X=[1.0],
        cam_valmat_coord_B_X=[2.0],
        cam_valmat_coord_A_Y=[3.0],
        cam_valmat_coord_B_Y=[4.0],
        cam_valmat_path=["value.npy"],
        cam_valmat_val_DEFAULT=[0.25],
        cam_workareadef_img_coord_A_X=[5.0],
        cam_workareadef_img_coord_B_X=[6.0],
        cam_workareadef_img_coord_A_Y=[7.0],
        cam_workareadef_img_coord_B_Y=[8.0],
        cam_workareadef_img_path=["workarea.png"],
        cam_workareadef_img_coord_A_ETA=[9.0],
        cam_workareadef_img_coord_B_ETA=[10.0],
        cam_workareadef_img_val_DEFAULT=[0.75],
    )
    app_config = SimpleNamespace(calib2d3d=SimpleNamespace(Proc2d=proc2d))

    with (
        patch(
            "argus_synchro.calibration_mat_generator_modules.ctrl.calibration2d3d.track_main.detect2D.person_tracker_SORT_2d.bbox2d_mot_tracker_wrapper"
        ),
        patch(
            "argus_synchro.calibration_mat_generator_modules.ctrl.calibration2d3d.track_main.detect2D.person_tracker_SORT_2d.NumpyMatrixLUT",
            return_value=MagicMock(),
        ) as matrix_lut,
        patch(
            "argus_synchro.calibration_mat_generator_modules.ctrl.calibration2d3d.track_main.detect2D.person_tracker_SORT_2d.GrayImageLUT",
            return_value=MagicMock(),
        ) as image_lut,
    ):
        proc2d_bboxtracker_recorder(
            app_config_calib=cast(Any, app_config),
            camera_index=0,
            image_size_hw=(720, 1280),
            onnx_model_path="model.onnx",
        )

    assert matrix_lut.call_args.kwargs["DEFAULT_VALUE"] == 0.25
    assert image_lut.call_args.kwargs["DEFAULT_VALUE"] == 0.75


@pytest.mark.parametrize(
    ("recorder_type", "extra_attributes"),
    [
        (proc2d_bboxtracker_recorder, {"evLUT2D": _LutStub(), "evLUT2D_workarea": _LutStub()}),
        (
            proc3d_bboxtracker_recorder,
            {
                "evLUT3D": _LutStub(),
                "evLUT3D_workarea": _LutStub(),
                "camerapos": np.zeros(3),
            },
        ),
    ],
)
def test_tracker_metadata_records_only_the_last_moving_frame(
    recorder_type: type,
    extra_attributes: dict[str, Any],
) -> None:
    recorder = cast(Any, object.__new__(recorder_type))
    recorder.trackingID_data = {}
    recorder.trackingID_bboxlog = {}
    for name, value in extra_attributes.items():
        setattr(recorder, name, value)

    recorder.last_tracker_result = np.array([[0, 0, 2, 2, 0.9, 7]], dtype=float)
    recorder._update_trackinfo(frame_ix=10)
    assert recorder.trackingID_data[7].frame_ix_lastmove == 10

    recorder._update_trackinfo(frame_ix=11)
    assert recorder.trackingID_data[7].frame_ix_max == 11
    assert recorder.trackingID_data[7].frame_ix_lastmove == 10

    recorder.last_tracker_result = np.array([[1, 0, 3, 2, 0.9, 7]], dtype=float)
    recorder._update_trackinfo(frame_ix=12)
    assert recorder.trackingID_data[7].frame_ix_lastmove == 12
