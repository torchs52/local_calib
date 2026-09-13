from types import SimpleNamespace
from unittest.mock import MagicMock

import numpy as np

from argus_synchro.calibration_mat_generator_modules.ctrl import calibration2d3d
from argus_synchro.calibration_mat_generator_modules.ctrl.calibration2d3d.calc_accuracy import (
    calib2d3d_matchecker,
)
from argus_synchro.calibration_mat_generator_modules.ctrl.calibration2d3d.track_main.detect2D import (
    detect2d_axis_faster,
)
from argus_synchro.calibration_mat_generator_modules.ctrl.calibration2d3d.track_main.detect2D.detect2d_axis_faster import (
    Detect2dAxisFaster,
)
from argus_synchro.calibration_mat_generator_modules.ctrl.calibration2d3d.track_main.detect3D.calc_headpoint_z import (
    calc_headpoint_z,
)
from argus_synchro.calibration_mat_generator_modules.ctrl.calibration2d3d.track_main.detect3D import (
    detect3d_class,
)
from argus_synchro.calibration_mat_generator_modules.ctrl.calibration2d3d.track_main.detect3D.person_tracker_SORT_3d import (
    bbox3d_mot_tracker_wrapper,
)
from argus_synchro.calibration_mat_generator_modules.ctrl.calibration2d3d.track_main.target_selector import (
    target_selector,
)


def test_calibration_matrix_round_trip_and_thresholds() -> None:
    rvec = np.array([0.1, -0.2, 0.3])
    tvec = np.array([1.0, 2.0, 3.0])

    restored_rvec, restored_tvec = calib2d3d_matchecker.conv_4x4mat_to_rtvec(
        calib2d3d_matchecker.conv_rtvec_to_mat(rvec, tvec)
    )
    assert np.allclose(restored_rvec, rvec)
    assert np.allclose(restored_tvec, tvec)

    rvec_ok, tvec_ok, _, _ = calib2d3d_matchecker.check_diff(
        rvec,
        tvec,
        rvec + np.deg2rad([2.0, 0.0, 0.0]),
        tvec + np.array([0.0, 0.6, 0.0]),
        rvec_threshold_deg=3.0,
        tvec_threshold=0.5,
        camerapos_mode=False,
    )
    assert bool(rvec_ok) is True
    assert bool(tvec_ok) is False


def test_detect2d_axis_uses_supplied_grid_and_interpolation(monkeypatch) -> None:
    observed_ranges = []
    observed_methods = []

    def fake_calc_slope_value(rvec, tvec, xrange, yrange, Mc):
        observed_ranges.append((xrange, yrange))
        return np.zeros((4, 2)), np.ones(4), np.ones(4)

    def fake_griddata(points, values, xi, method):
        observed_methods.append(method)
        return np.ones(len(xi))

    monkeypatch.setattr(
        Detect2dAxisFaster,
        "_calc_slope_value",
        staticmethod(fake_calc_slope_value),
    )
    monkeypatch.setattr(detect2d_axis_faster, "griddata", fake_griddata)

    result = Detect2dAxisFaster.calc_crosspoints(
        np.array([[0.0, 2.0, 0.0, 4.0]]),
        rvec=np.zeros(3),
        tvec=np.zeros(3),
        Mc=np.eye(3),
        xrange=(-10.0, 10.0, 200),
        yrange=(-20.0, 20.0, 200),
        interpolate_firstmethod="cubic",
        interpolate_secondmethod="nearest",
    )

    assert result.shape == (1, 2, 2)
    assert observed_ranges == [((-10.0, 10.0, 200), (-20.0, 20.0, 200))]
    assert observed_methods == ["cubic", "nearest", "nearest", "nearest"]


def test_headpoint_z_is_corrected_per_grid_section() -> None:
    points = np.array(
        [
            [[0.2, 0.0, 1.0], [0.0, 0.0, 0.0]],
            [[0.8, 0.0, 3.0], [0.0, 0.0, 0.0]],
            [[1.2, 0.0, 10.0], [0.0, 0.0, 0.0]],
            [[1.8, 0.0, 14.0], [0.0, 0.0, 0.0]],
            [[3.0, 0.0, 100.0], [0.0, 0.0, 0.0]],
        ]
    )

    corrected = calc_headpoint_z(
        xrange=(-1.0, 2.0), yrange=(-1.0, 1.0)
    ).apply_per_point(points)

    assert np.allclose(corrected, [2.0, 2.0, 12.0, 12.0, 100.0])

def test_detect3d_extract_uses_grid_headpoint_correction(monkeypatch) -> None:
    detector = detect3d_class.__new__(detect3d_class)
    detector.app_config_calib = SimpleNamespace(
        calib2d3d=SimpleNamespace(
            Proc3d=SimpleNamespace(
                gplane_detection_walkingarea_limit=False,
                is_footpoints_fixed=True,
                footpoints_zval=0.0,
                is_headpoint_overwrite=True,
                is_headpoint_grid_overwrite=True,
            )
        )
    )
    detector.tracking_recorder = SimpleNamespace(
        get_data_array_fbb_point_history=lambda: [
            (
                [np.array([[0.2, -0.5, 0.0], [0.2, 0.0, 1.0]])],
                10,
                [1],
                np.array([[0.0, 1.0, -1.0, 1.0, -1.0, 2.0]]),
            )
        ]
    )
    detector.calc_headpoint_z_inst = SimpleNamespace(
        apply=MagicMock(side_effect=AssertionError("legacy correction selected")),
        apply_per_point=MagicMock(return_value=np.array([7.5])),
    )
    monkeypatch.setattr(
        "argus_synchro.calibration_mat_generator_modules.ctrl.calibration2d3d.track_main.detect3D.find_target_trajectory",
        lambda tracker_result_interface, frame_index: [1],
    )

    points, timestamps = detector.extract(SimpleNamespace(), frame_ix=10)

    assert points[0, 0, 2] == 7.5
    assert np.array_equal(timestamps, [10])
    detector.calc_headpoint_z_inst.apply_per_point.assert_called_once()


def test_overlapping_bbox3d_are_merged() -> None:
    bboxes = np.array(
        [
            [0.0, 2.0, 0.0, 2.0, 0.0, 1.0],
            [1.0, 3.0, 0.0, 2.0, -1.0, 2.0],
            [10.0, 11.0, 10.0, 11.0, 0.0, 1.0],
        ]
    )

    merged = bbox3d_mot_tracker_wrapper.merge_overlapping_bbox3d_xy(
        bboxes, overlap_threshold=0.5
    )

    assert np.allclose(
        merged,
        [
            [0.0, 3.0, 0.0, 2.0, -1.0, 2.0],
            [10.0, 11.0, 10.0, 11.0, 0.0, 1.0],
        ],
    )


def test_bbox_merge_can_be_applied_before_tracking() -> None:
    tracker = bbox3d_mot_tracker_wrapper.__new__(bbox3d_mot_tracker_wrapper)
    tracker.enable_bbox3d_overlap_merge = True
    tracker.bbox3d_overlap_merge_threshold = 0.5
    captured = {}
    tracker.bbox3d_to_sv_detections = lambda bbox3d: captured.setdefault(
        "bbox3d", bbox3d
    )
    tracker.mot = SimpleNamespace(update=lambda detections: detections)
    tracker.sv_detections_to_ndarray = (
        lambda detections, include_tracker_id: np.asarray(detections)
    )

    tracker.update(
        np.array(
            [
                [0.0, 2.0, 0.0, 2.0, 0.0, 1.0],
                [1.0, 3.0, 0.0, 2.0, -1.0, 2.0],
            ]
        )
    )

    assert np.allclose(captured["bbox3d"], [[0.0, 3.0, 0.0, 2.0, -1.0, 2.0]])


def test_3d_target_selection_uses_each_scanned_track_score() -> None:
    metadata = {
        1: SimpleNamespace(
            is_alive=True,
            is_tracking_target=False,
            frame_ix_min=0,
            frame_ix_max=10,
            frame_evval_min=10.0,
        ),
        2: SimpleNamespace(
            is_alive=True,
            is_tracking_target=False,
            frame_ix_min=5,
            frame_ix_max=15,
            frame_evval_min=1.0,
        ),
    }
    tracking_result = SimpleNamespace(trackingIDmetadata=metadata)
    selector = target_selector.__new__(target_selector)
    selector._logger = MagicMock()

    selector._compare_trackresult3d(tracking_result, frame3d_index=15)

    assert metadata[1].is_tracking_target is False
    assert metadata[2].is_tracking_target is True


def test_matrix_reference_check_does_not_block_missing_file(monkeypatch) -> None:
    logger = MagicMock()
    monkeypatch.setattr(calibration2d3d, "_logger", logger)
    instance = calibration2d3d.calibration2d3d_class.__new__(
        calibration2d3d.calibration2d3d_class
    )
    instance.camera_id = 0
    instance.app_config_calib = SimpleNamespace(
        calib2d3d=SimpleNamespace(
            CalcCorrespondence=SimpleNamespace(
                optparam_initialvector="missing-reference.json"
            )
        )
    )

    instance._log_calibration_matrix_difference(np.eye(4), 0.25)

    logger.warning.assert_called_once()


def test_get_calibval_uses_lut_ratio_for_each_3d_center(monkeypatch) -> None:
    instance = calibration2d3d.calibration2d3d_class.__new__(
        calibration2d3d.calibration2d3d_class
    )
    center3d = np.array([[0.0, 0.0, 99.0], [2.0, 0.0, 99.0]])
    axis3d = np.array([[0.0, 0.0, 10.0], [0.0, 0.0, 0.0]])
    center2d = np.zeros((2, 2))
    axis2d = np.zeros((2, 2))
    instance.track_main = SimpleNamespace(
        extract_fromcenter=lambda **kwargs: (center2d, center3d),
        extract_withaxis=lambda *args, **kwargs: (axis2d, axis3d),
    )
    instance._get_result_single = MagicMock(
        side_effect=[(center2d, center3d.copy()), (axis2d, axis3d.copy())]
    )
    instance.proccorr = SimpleNamespace(
        estimate=MagicMock(),
        get_last_rtvec=lambda: (np.zeros(3), np.zeros(3)),
        get_rotation_mat=lambda: np.eye(4),
    )
    instance.procaccr = SimpleNamespace(LOOCV_bytime=lambda *args: 0.0)
    instance.center3d_zratio_LUT = SimpleNamespace(
        evaluate=lambda x, y: 0.25 if x < 1.0 else 0.75
    )
    instance.app_config_calib = SimpleNamespace(
        debug=SimpleNamespace(save_cornerlist_pickle=False),
        calib2d3d=SimpleNamespace(
            CalcCorrespondence=SimpleNamespace(
                corrpoint_mode="CY-XY",
                enable_recalc_center3d_z=True,
                bbox_center3d_z_ratio=0.5,
                corner_rangefilter_mode="Y",
            )
        ),
    )
    instance.use_centerpoint_x_min = -100.0
    instance.use_centerpoint_x_max = 100.0
    instance.use_centerpoint_y_min = -100.0
    instance.use_centerpoint_y_max = 100.0
    instance.corner_rangefilter_x_min = -100.0
    instance.corner_rangefilter_x_max = 100.0
    instance.corner_rangefilter_y_min = -100.0
    instance.corner_rangefilter_y_max = 100.0
    instance.center3d_apply_zratio_x_min = -100.0
    instance.center3d_apply_zratio_x_max = 100.0
    instance.center3d_apply_zratio_y_min = -100.0
    instance.center3d_apply_zratio_y_max = 100.0
    monkeypatch.setattr(calibration2d3d, "debug_store", lambda **kwargs: None)

    instance.get_calibval(frame_ix=10)

    final_3d = instance.proccorr.estimate.call_args_list[-1].args[1]
    assert np.allclose(final_3d[:2, 2], [2.5, 7.5])

    instance.center3d_zratio_LUT = None
    instance._get_result_single = MagicMock(
        side_effect=[(center2d, center3d.copy()), (axis2d, axis3d.copy())]
    )
    instance.proccorr.estimate.reset_mock()

    instance.get_calibval(frame_ix=10)

    fallback_3d = instance.proccorr.estimate.call_args_list[-1].args[1]
    assert np.allclose(fallback_3d[:2, 2], [5.0, 5.0])