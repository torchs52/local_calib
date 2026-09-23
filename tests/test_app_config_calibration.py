# ruff: noqa: PLR2004

from configparser import ConfigParser
from pathlib import Path

import pytest

from directory_config_helper import dev_directory_config

from argus_synchro.common import paths
from argus_synchro.config.app_config import parse_float_list
from argus_synchro.config.app_config_calibration import AppConfigCalibration
from argus_synchro.config.settings_validation import ConfigValidationError


def test_invalid_calibration_setting_is_rejected_before_config_creation(
    tmp_path: Path,
) -> None:
    settings = Path("config/calib_settings.ini").read_text(encoding="utf-8")
    config_path = tmp_path / "calib_settings.ini"
    config_path.write_text(
        settings.replace("camera_count = 3", "camera_count = 5", 1),
        encoding="utf-8",
    )

    with pytest.raises(
        ConfigValidationError,
        match=r"\[CalibCheck2d3d\] camera_count=.*outside the allowed range",
    ):
        AppConfigCalibration(
            configpath=str(config_path),
            arglist=[],
            directory_config=dev_directory_config(),
        )


def test_invalid_calibration_fixed_yaw_flag_is_rejected(tmp_path: Path) -> None:
    settings = Path("config/calib_settings.ini").read_text(encoding="utf-8")
    config_path = tmp_path / "calib_settings.ini"
    config_path.write_text(
        settings.replace("is_fixed_yaw = False", "is_fixed_yaw = automatic", 1),
        encoding="utf-8",
    )

    with pytest.raises(
        ConfigValidationError,
        match=r"\[DataCapture_CAN\] is_fixed_yaw is invalid: must be a boolean",
    ):
        AppConfigCalibration(
            configpath=str(config_path),
            arglist=[],
            directory_config=dev_directory_config(),
        )


def test_outputdir_root_uses_cli_log_dir_for_relative_setting() -> None:
    repo_root = Path.cwd()
    directory_config = paths.DirectoryConfig(
        config_dir=paths.normalize_path("./config", repo_root),
        log_dir=paths.normalize_path("/tmp/log", repo_root),
        mmap_dir=paths.DEFAULT_MMAP_DIR,
    )
    directory_config, _ = paths.load_directory_config_from_ini(directory_config)
    app_config = AppConfigCalibration(
        configpath="config/calib_settings.ini",
        arglist=[],
        directory_config=directory_config,
    )

    assert app_config.default.outputdir_root == "/tmp/log/tmpCalib"
    assert app_config.filepath_io.Calib3d3dmat_lidars[0] == (
        "/tmp/log/tmpCalib/3d-3d/lidar2crane_trans_mat_0.csv"
    )
    assert app_config.calibCheck2d3d.resultfiles[0] == (
        "/tmp/log/tmpCalib/calibcheck2d3d_results_camera0.txt"
    )


def test_outputdir_root_uses_dev_cli_log_dir() -> None:
    app_config = AppConfigCalibration(
        configpath="config/calib_settings.ini",
        arglist=[],
        directory_config=dev_directory_config(),
    )
    expected_root = str(Path.cwd().resolve() / "log" / "tmpCalib")
    assert app_config.default.outputdir_root == expected_root


def test_calibcheck2d3d_evaluation_settings_are_loaded() -> None:
    app_config = AppConfigCalibration(
        configpath="config/calib_settings.ini",
        arglist=[],
        directory_config=dev_directory_config(),
    )
    config = app_config.calibCheck2d3d

    assert config.frame_info_maxlen == 100_000
    assert config.thresh_3dbbox_count_per_frame == 1
    assert config.thresh_3dbbox_count_mean_ratio == 0.1
    assert config.thresh_2dbbox_count_per_frame == 1
    assert config.thresh_2dbbox_count_mean_ratio == 0.1
    assert config.thresh_3dbbox_tracking_idcount == 1
    assert config.thresh_2dbbox_tracking_idcount == 1
    assert config.eval_frame_stride == 1
    assert config.use_legacy_like_metric is True
    assert config.person_not_detected_sec == 10.0
    assert config.debug_calibcheck_enabled is False
    assert config.debug_capture_ui_video_enabled is False
    assert config.debug_capture_frame_text_enabled is False
    assert config.debug_video_fps == 10.0
    assert config.debug_video_prefix == "camera"
    assert config.debug_eval_pickle_path == str(
        Path.cwd().resolve() / "log" / "tmpCalib" / "calibcheck_bboxinfo.pickle"
    )
    assert config.eval_zvalues == (-1.36, 0.0)
    assert config.debug_eval_trace_enabled is False
    assert config.debug_eval_trace_all_frames is False
    assert config.debug_eval_trace_range_start == 1
    assert config.debug_eval_trace_range_end == 5000


def test_calib2d3d_bbox_center_z_ratio_areas_are_loaded() -> None:
    app_config = AppConfigCalibration(
        configpath="config/calib_settings.ini",
        arglist=[],
        directory_config=dev_directory_config(),
    )
    config = app_config.calib2d3d.CalcCorrespondence

    assert config.bbox_center3d_z_ratio_area_xmin == [-100.0, -100.0, -100.0]
    assert config.bbox_center3d_z_ratio_area_xmax == [100.0, 100.0, 100.0]
    assert config.bbox_center3d_z_ratio_area_ymin == [-100.0, -100.0, -100.0]
    assert config.bbox_center3d_z_ratio_area_ymax == [100.0, 100.0, 100.0]


def test_calib2d3d_walking_area_corners_are_loaded_by_zero_based_camera() -> None:
    app_config = AppConfigCalibration(
        configpath="config/calib_settings.ini",
        arglist=[],
        directory_config=dev_directory_config(),
    )

    assert app_config.calib2d3d.Diagnosis.walking_area_corners == [
        ((-15.0, 0.0), (15.0, 0.0), (15.0, 10.0), (-15.0, 10.0)),
        ((-15.0, 0.0), (15.0, 0.0), (15.0, 10.0), (-15.0, 10.0)),
        ((-15.0, 0.0), (15.0, 0.0), (15.0, 10.0), (-15.0, 10.0)),
    ]


def test_calib2d3d_walking_area_requires_four_xy_points(tmp_path: Path) -> None:
    settings = Path("config/calib_settings.ini").read_text(encoding="utf-8")
    config_path = tmp_path / "calib_settings.ini"
    config_path.write_text(
        settings.replace(
            "walking_area_corners_camera0 = [-15,0,15,0,15,10,-15,10]",
            "walking_area_corners_camera0 = [0,0,1,1]",
            1,
        ),
        encoding="utf-8",
    )

    with pytest.raises(ValueError, match="walking area must contain eight values"):
        AppConfigCalibration(
            configpath=str(config_path),
            arglist=[],
            directory_config=dev_directory_config(),
        )


@pytest.mark.parametrize(
    "model_settings_path",
    [
        "config/SCX700-3_calib_settings.ini",
        "config/SCX900-3_calib_settings.ini",
        "config/SCX2000-3_calib_settings.ini",
        "config/SCX3500-3_calib_settings.ini",
    ],
)
def test_model_specific_walking_areas_use_zero_based_camera_keys(
    model_settings_path: str,
) -> None:
    settings = ConfigParser()
    settings.read(model_settings_path, encoding="utf-8")

    for camera_id in range(3):
        values = parse_float_list(
            settings.get(
                "Calib2d3d_Diagnosis",
                f"walking_area_corners_camera{camera_id}",
            )
        )
        assert len(values) == 8


def test_calib2d3d_future_tracking_and_axis_grid_settings_are_loaded() -> None:
    app_config = AppConfigCalibration(
        configpath="config/calib_settings.ini",
        arglist=[],
        directory_config=dev_directory_config(),
    )
    proc2d = app_config.calib2d3d.Proc2d
    proc3d = app_config.calib2d3d.Proc3d

    assert proc2d.trackresult_use_lastmove_ix is False
    assert proc2d.axis_gridpoints_xrange_min == [-10.0, 0.0, -10.0]
    assert proc2d.axis_gridpoints_xrange_max == [10.0, 10.0, 10.0]
    assert proc2d.axis_gridpoints_yrange_min == [0.0, -10.0, -10.0]
    assert proc2d.axis_gridpoints_yrange_max == [10.0, 10.0, 0.0]
    assert proc2d.axis_gridpoints_interpolate_firstmethod == "nearest"
    assert proc2d.axis_gridpoints_interpolate_secondmethod == "nearest"
    assert proc3d.trackresult_use_lastmove_ix is False


def test_remaining_explicit_calibration_settings_are_loaded() -> None:
    app_config = AppConfigCalibration(
        configpath="config/calib_settings.ini",
        arglist=[],
        directory_config=dev_directory_config(),
    )

    assert app_config.default.z_height == -2.0
    assert app_config.default.z_height_withmargin == -1.5
    assert app_config.dataCapture.Can.is_fixed_yaw is False
    assert app_config.dataCapture.Can.c_file.endswith(
        "/can_20240314_134621.csv"
    )
    assert app_config.dataCapture.Can.fixed_yaw_deg == 0.0
    assert app_config.dataCapture.Lidar.dev_str == "mid360"
    assert app_config.calib2d3d.placeholder == ""
    assert app_config.calib2d3d.Proc2d.enable_bbox_shapefilter is False


def test_calibration_parameters_match_shi_values() -> None:
    app_config = AppConfigCalibration(
        configpath="config/calib_settings.ini",
        arglist=[],
        directory_config=dev_directory_config(),
    )
    correspondence = app_config.calib2d3d.CalcCorrespondence

    assert correspondence.use_centerpoint_x_min == [-6.0, 3.3, -6.0]
    assert correspondence.use_centerpoint_x_max == [14.0, 7.3, 14.0]
    assert correspondence.use_centerpoint_y_min == [2.075, -10.0, -5.075]
    assert correspondence.use_centerpoint_y_max == [5.075, 10.0, -2.075]
    assert correspondence.corner_rangefilter_mode == "Y"
    assert correspondence.corner_rangefilter_x_min == [-10.0, -10.0, -10.0]
    assert correspondence.corner_rangefilter_x_max == [10.0, 10.0, 10.0]
    assert correspondence.corner_rangefilter_y_min == [-10.0, -10.0, -10.0]
    assert correspondence.corner_rangefilter_y_max == [10.0, 10.0, 10.0]
    assert app_config.calibCheck2d3d.onnx_model_path.endswith(
        "/checkpoints/new_bench_full_20260625.onnx"
    )
