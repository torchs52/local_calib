from __future__ import annotations

import logging
from configparser import ConfigParser
from dataclasses import dataclass
from enum import StrEnum


class InvalidValuePolicy(StrEnum):
    STRICT = "strict"
    NORMALIZE = "normalize"


class ConfigValidationError(ValueError):
    """INI設定値が定義済み規則に違反した場合に送出する。"""


@dataclass(frozen=True, slots=True)
class SettingRule:
    section: str
    option: str
    value_type: type[int] | type[float] | type[bool] | type[str]
    default: int | float | bool | str
    minimum: int | float | None = None
    maximum: int | float | None = None
    allowed_values: frozenset[str] | None = None


_RULES: tuple[SettingRule, ...] = (
    SettingRule("DEFAULT", "File_Input", bool, False),
    SettingRule("DEFAULT", "print_disabled", bool, True),
    SettingRule("DEFAULT", "use_shi_lib", bool, False),
    SettingRule("General", "operation_mode", int, 0, 0, 1),
    SettingRule("General", "in_factory", bool, False),
    SettingRule("General", "has_external_guard", bool, False),
    SettingRule("General", "external_guard_offset", float, 0.0, -100.0, 100.0),
    SettingRule("General", "ground_height", float, -1.365, -100.0, 100.0),
    SettingRule("General", "ground_height_margin", float, 0.45, 0.0, 100.0),
    SettingRule("General", "rotation_radius", float, 4.2, 0.0, 100.0),
    SettingRule("General", "enable_cpu_affinity", bool, True),
    SettingRule("CalibMode", "isRunning3D3Dcalib", bool, False),
    SettingRule("CalibMode", "isRunning2D3Dcalib", bool, False),
    SettingRule("CalibMode", "cameraID", int, 0, 0, 3),
    SettingRule("CalibMode", "start2D3DCalibCalc", bool, False),
    SettingRule("CalibMode", "isRunning2D3Dcheck", bool, False),
    SettingRule("CalibMode", "start2D3DCheckCalc", bool, False),
    SettingRule("CalibMode", "isRunningInterfaceDebug", bool, False),
    SettingRule("UI_IF", "damp_out", bool, False),
    SettingRule("UI_IF", "bbox_3d_num", int, 20, 1, 10_000),
    SettingRule("UI_IF", "bbox_3d_distance", float, 8.0, 0.0, 1_000.0),
    SettingRule("UI_IF", "show_unk", bool, True),
    SettingRule("UI_IF", "collision_depict_dist", float, 4.0, 0.0, 1_000.0),
    SettingRule("UI_IF", "collision_attention_dist", float, 3.0, 0.0, 1_000.0),
    SettingRule("UI_IF", "collision_warning_dist", float, 1.5, 0.0, 1_000.0),
    SettingRule("UI_IF", "cliff_attention_dist", float, 6.0, 0.0, 1_000.0),
    SettingRule("UI_IF", "cliff_warning_dist", float, 3.0, 0.0, 1_000.0),
    SettingRule("UI_IF", "draw_bbox_3d", bool, False),
    SettingRule("UI_IF", "draw_collision", bool, True),
    SettingRule("CalibUI_IF", "godot_ui", bool, True),
    SettingRule("CalibUI_IF", "damp_out", bool, True),
    SettingRule("CalibUI_IF", "show_trajectory", bool, True),
    SettingRule("CalibUI_IF", "show_image2d3d", bool, True),
    SettingRule("Lidar", "count", int, 2, 1, 6),
    SettingRule("Lidar", "accum_time", float, 0.1, 0.001, 10.0),
    SettingRule("camera", "count", int, 3, 1, 4),
    SettingRule("AppManager", "interval", float, 0.5, 0.01, 60.0),
    SettingRule("AppManager", "JudegeStopThr", int, 150, 1, 100_000),
    SettingRule("Monitor", "coeff", float, 2.0, 0.01, 100.0),
    SettingRule("Scrutinizer", "s_frame", int, 1, 0, 4_294_967_295),
    SettingRule("Scrutinizer", "e_frame", int, 4_294_967_295, 1, 4_294_967_295),
    SettingRule("Scrutinizer", "file_input_loop", bool, False),
    SettingRule("Scrutinizer", "fast_th_ms", float, 100.0, 0.0, 3_600_000.0),
    SettingRule("Scrutinizer", "slow_th_ms", float, 5_000.0, 0.0, 3_600_000.0),
    SettingRule("Scrutinizer", "short_que", int, 10, 1, 1_000_000),
    SettingRule("Scrutinizer", "long_que", int, 100, 1, 1_000_000),
    SettingRule("Scrutinizer", "get_data_sleep_sec", float, 0.15, 0.0, 60.0),
    SettingRule("CAN", "IsOld", bool, False),
    SettingRule("CAN", "interpretation", int, 1, 0, 1),
    SettingRule("CAN", "yaw_offset_deg", float, 0.0, -360.0, 360.0),
    SettingRule("StateEstimator", "window_sec", float, 0.5, 0.001, 60.0),
    SettingRule("StateEstimator", "delta_db", float, 0.05, 0.0, 1.0),
    SettingRule("StateEstimator", "p_on", float, 0.1, 0.0, 1.0),
    SettingRule("StateEstimator", "p_off", float, 0.05, 0.0, 1.0),
    SettingRule("ReducedLoadMode", "many_points_ratio", float, 0.4, 0.0, 1.0),
    SettingRule("ReducedLoadMode", "few_points_ratio", float, 0.3, 0.0, 1.0),
    SettingRule("JetsonMonitor", "interval", float, 5.0, 0.01, 3_600.0),
    SettingRule("JetsonMonitor", "write_interval", float, 1.0, 0.01, 3_600.0),
    SettingRule("JetsonMonitor", "window_sec", int, 600, 1, 86_400),
    SettingRule("camera", "video_width", int, 1280, 1, 8_192),
    SettingRule("camera", "video_height", int, 720, 1, 8_192),
    SettingRule("camera", "sys_width", int, 1280, 1, 8_192),
    SettingRule("camera", "sys_height", int, 720, 1, 8_192),
    SettingRule("detect2d", "conf_thresh", float, 0.7, 0.0, 1.0),
    SettingRule("detect2d", "nms_thresh", float, 0.5, 0.0, 1.0),
    SettingRule("detect2d", "use_onnx", bool, True),
    SettingRule("detect2d", "is_DAMO_YOLO", bool, True),
    SettingRule("detect3d", "eps", float, 0.4, 0.001, 100.0),
    SettingRule("detect3d", "min_samples", int, 5, 1, 1_000_000),
    SettingRule("camera", "MOTEC", bool, True),
    SettingRule("calibration", "calib_lidar2crane", bool, True),
    SettingRule(
        "camera",
        "undistort_backend",
        str,
        "auto",
        allowed_values=frozenset({"auto", "cpu", "cuda"}),
    ),
    SettingRule("JetsonMonitor", "isApplied", bool, False),
    SettingRule("detect2d", "isApplied", bool, True),
)


def _get_policy(ini: ConfigParser) -> InvalidValuePolicy:
    raw_policy = ini.get("ConfigValidation", "invalid_value_policy", fallback="strict")
    try:
        return InvalidValuePolicy(raw_policy.strip().lower())
    except ValueError as error:
        raise ConfigValidationError(
            "[ConfigValidation] invalid_value_policy must be 'strict' or 'normalize'"
        ) from error


def _parse_value(raw_value: str, rule: SettingRule) -> int | float | bool | str:
    if rule.value_type is bool:
        normalized = raw_value.strip().lower()
        if normalized in {"1", "yes", "true", "on"}:
            return True
        if normalized in {"0", "no", "false", "off"}:
            return False
        raise ValueError("must be a boolean")
    if rule.value_type is int:
        return int(raw_value)
    if rule.value_type is float:
        return float(raw_value)
    return raw_value.strip().lower()


def _validate_value(
    value: int | float | bool | str, rule: SettingRule
) -> int | float | bool | str:
    if rule.allowed_values is not None and value not in rule.allowed_values:
        raise ValueError(f"must be one of {sorted(rule.allowed_values)}")
    if rule.minimum is not None or rule.maximum is not None:
        if not isinstance(value, (int, float)):
            raise ValueError("must be numeric")
        numeric_value: int | float = value
        if rule.minimum is not None and numeric_value < rule.minimum:
            return rule.minimum
        if rule.maximum is not None and numeric_value > rule.maximum:
            return rule.maximum
    return value


def _format_value(value: int | float | bool | str) -> str:
    return str(value).lower() if isinstance(value, bool) else str(value)


def validate_setting_rules(
    ini: ConfigParser, rules: tuple[SettingRule, ...], config_name: str
) -> None:
    """定義済みのINI設定値をstrict検証または実行時補正する。"""
    policy = _get_policy(ini)
    logger = logging.getLogger(__name__)

    for rule in rules:
        if not ini.has_option(rule.section, rule.option):
            continue
        is_normalized = False
        raw_value = ""
        parsed_value = rule.default
        try:
            raw_value = ini.get(rule.section, rule.option)
            parsed_value = _parse_value(raw_value, rule)
            normalized_value = _validate_value(parsed_value, rule)
        except ValueError as error:
            if policy is InvalidValuePolicy.STRICT:
                raise ConfigValidationError(
                    f"[{rule.section}] {rule.option} is invalid: {error}"
                ) from error
            normalized_value = rule.default
            is_normalized = True

        if is_normalized or normalized_value != parsed_value:
            if policy is InvalidValuePolicy.STRICT:
                raise ConfigValidationError(
                    f"[{rule.section}] {rule.option}={raw_value!r} is outside the allowed range"
                )
            corrected_value = _format_value(normalized_value)
            ini.set(rule.section, rule.option, corrected_value)
            logger.warning(
                "%s normalized: [%s] %s=%r -> %s",
                config_name,
                rule.section,
                rule.option,
                raw_value,
                corrected_value,
            )


def validate_settings(ini: ConfigParser) -> None:
    """定義済みのsettings.ini値をstrict検証または実行時補正する。"""
    validate_setting_rules(ini, _RULES, "settings.ini")
