from configparser import ConfigParser

from argus_synchro.config.settings_validation import (
    SettingRule,
    validate_setting_rules,
)

# calib_settings.iniの代表的な検証ルール。
# 仮の既定値・値域なので、実機仕様確定時に更新して対象項目を拡充する。
_RULES: tuple[SettingRule, ...] = (
    SettingRule("DEFAULT", "File_Input", bool, False),
    SettingRule("DEFAULT", "print_disabled", bool, True),
    SettingRule("DataCapture", "s_frame", int, 1, 0, 10_000_000),
    SettingRule("DataCapture", "e_frame", int, 5_000_000, 1, 10_000_000),
    SettingRule(
        "DataCapture",
        "sync_type",
        str,
        "bytime_nearby",
        allowed_values=frozenset({"bytime_nearby", "byindex"}),
    ),
    SettingRule("DataCapture", "datawait_sec", float, 5.0, 0.0, 60.0),
    SettingRule("DataCapture", "save_sensordata", bool, False),
    SettingRule("DataCapture_CAN", "is_fixed_yaw", bool, False),
    SettingRule("DataCapture_CAN", "fixed_yaw_deg", float, 0.0, 0.0, 359.9),
    SettingRule("DataCapture_Lidar", "count", int, 2, 1, 6),
    SettingRule("DataCapture_Lidar", "accum_time", float, 0.1, 0.001, 10.0),
    SettingRule("DataCapture_Lidar", "data_buffersize", int, 50, 1, 10_000),
    SettingRule(
        "DataCapture_Lidar", "framethinning_bufferlen_threshold", int, 25, 1, 10_000
    ),
    SettingRule("DataCapture_Lidar", "allow_lack", bool, True),
    SettingRule("DataCapture_Lidar", "capture_latency_ms", float, 10.0, 0.0, 10_000.0),
    SettingRule("DataCapture_Camera", "count", int, 3, 1, 4),
    SettingRule("DataCapture_Camera", "video_width", int, 1280, 1, 8_192),
    SettingRule("DataCapture_Camera", "video_height", int, 720, 1, 8_192),
    SettingRule("DataCapture_Camera", "sys_width", int, 1920, 1, 8_192),
    SettingRule("DataCapture_Camera", "sys_height", int, 1080, 1, 8_192),
    SettingRule("DataCapture_Camera", "data_buffersize", int, 50, 1, 10_000),
    SettingRule("DataCapture_Camera", "framerate_div", int, 3, 1, 120),
    SettingRule("DataConverter2D3D_Camera", "undistort_enable", bool, True),
    SettingRule("DataConverter2D3D_Lidar", "sensors", int, 2, 1, 6),
    SettingRule("DataConverter2D3D_Lidar", "accumulate_length", int, 3, 1, 10_000),
    SettingRule("Calib2d3d_Proc2d", "yolo_obj_countlimit", int, 50, 1, 10_000),
    SettingRule("Calib2d3d_Proc2d", "conf_thresh", float, 0.5, 0.0, 1.0),
    SettingRule("Calib2d3d_Proc2d", "nms_thresh", float, 0.45, 0.0, 1.0),
    SettingRule("Calib2d3d_Proc2d", "enable_imgmask", bool, False),
    SettingRule("Calib2d3d_Proc3d", "dbscan_eps", float, 0.5, 0.001, 100.0),
    SettingRule("Calib2d3d_Proc3d", "dbscan_min_samples", int, 10, 1, 1_000_000),
    SettingRule("Calib2d3d_Proc3d", "save_debugdata", bool, False),
    SettingRule("Calib2d3d_CalcProgress", "progress_threshold", float, 0.7, 0.0, 1.0),
    SettingRule(
        "Calib2d3d_CalcProgress",
        "subblock_overwrite_src",
        str,
        "group",
        allowed_values=frozenset({"none", "block", "group"}),
    ),
    SettingRule(
        "Calib2d3d_CalcCorrespondence",
        "calcmethod",
        str,
        "opt",
        allowed_values=frozenset({"old", "opt"}),
    ),
    SettingRule("Calib3d3d_SimParams", "max_range", float, 10.0, 0.001, 1_000.0),
    SettingRule(
        "Calib3d3d_SimParams",
        "lidar_type",
        str,
        "mid360",
        allowed_values=frozenset({"airy", "mid360"}),
    ),
    SettingRule("Calib3d3d_SimParams", "rays_az", int, 240, 1, 100_000),
    SettingRule("Calib3d3d_SimParams", "rays_el", int, 120, 1, 100_000),
    SettingRule("Calib3d3d_SimParams", "points_accum_time", float, 8.0, 0.001, 8.0),
    SettingRule("CalibCheck2d3d", "new_axis_mode", bool, True),
    SettingRule("CalibCheck2d3d", "image_w", int, 1920, 1, 8_192),
    SettingRule("CalibCheck2d3d", "image_h", int, 1080, 1, 8_192),
    SettingRule("CalibCheck2d3d", "camera_count", int, 3, 1, 4),
    SettingRule("CalibCheck2d3d", "score_accept_count_threshold", int, 5, 1, 10_000),
    SettingRule("CalibCheck2d3d", "score_value_threshold", float, 0.1, 0.0, 1.0),
)


def validate_calib_settings(ini: ConfigParser) -> None:
    """代表的なcalib_settings.ini値をstrict検証または実行時補正する。"""
    validate_setting_rules(ini, _RULES, "calib_settings.ini")