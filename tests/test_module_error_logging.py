from pathlib import Path
from typing import Any

import argus_synchro.diagnosis.state_d_errors as state_d_errors
import pytest
from argus_synchro.common.app_logger import DEBUG, AppLoggerFactory
from argus_synchro.diagnosis.error_config import ErrorConfig
from argus_synchro.diagnosis.error_diagnosis import ResultDiagnosis, StateErrorDiagnosisD
from argus_synchro.diagnosis.state_d_errors import (
    AccumulationModuleError,
    AppManagerModuleError,
    CalibrationModuleError,
    CameraModuleError,
    CameraHumanDetectionModuleError,
    CanModuleError,
    CollisionJudgmentModuleError,
    GetDataModuleError,
    ImuModuleError,
    Integrate2d3dModuleError,
    LidarModuleError,
    LidarShiftMonitorModuleError,
    MainModuleError,
    Object3DDetectionModuleError,
    PointsRefineModuleError,
    VisualModuleError,
)
from argus_synchro.shared_errors import ModuleErrorIndex


def test_module_error_classes_keep_vendor_direct_base() -> None:
    module_error_types = (
        LidarModuleError,
        CameraModuleError,
        AccumulationModuleError,
        CanModuleError,
        Integrate2d3dModuleError,
        Object3DDetectionModuleError,
        CameraHumanDetectionModuleError,
        CollisionJudgmentModuleError,
        CalibrationModuleError,
        ImuModuleError,
        AppManagerModuleError,
        MainModuleError,
        PointsRefineModuleError,
        VisualModuleError,
        LidarShiftMonitorModuleError,
        GetDataModuleError,
    )

    assert all(
        diagnosis_type.__bases__ == (StateErrorDiagnosisD,)
        for diagnosis_type in module_error_types
    )


def test_module_error_log_contains_caught_exception_traceback(tmp_path: Path) -> None:
    log_path = tmp_path / "module-error.log"
    diagnosis = CameraModuleError()
    logger_factory = AppLoggerFactory(
        to_console=False,
        to_file=str(log_path),
        level=DEBUG,
        compress=False,
    )
    diagnosis.log_register(logger_factory)
    logger_factory.update()

    try:
        raise RuntimeError("camera read failed")
    except RuntimeError as error:
        diagnosis.log_output(
            ResultDiagnosis.DETECTION,
            ResultDiagnosis.NORMAL,
            ModuleErrorIndex.CAMERA_MODULE_ERROR,
            error,
            2,
        )

    log_text = log_path.read_text(encoding="utf-8")
    assert "カメラ2モジュールエラー: RuntimeError: camera read failed" in log_text
    assert "Traceback (most recent call last):" in log_text
    assert "RuntimeError: camera read failed" in log_text


def test_camera_module_error_throttles_through_vendor_log_output(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    log_path = tmp_path / "throttled-module-error.log"
    diagnosis = CameraModuleError()
    err_conf = ErrorConfig()
    err_conf.camera_module_error.is_enabled = True
    err_conf.camera_module_error.ongoing_log_interval_sec = 60.0
    diagnosis.update(err_conf)
    logger_factory = AppLoggerFactory(
        to_console=False,
        to_file=str(log_path),
        level=DEBUG,
        compress=False,
    )
    diagnosis.log_register(logger_factory)
    logger_factory.update()
    now = iter((0.0, 1.0, 60.0, 61.0))
    monkeypatch.setattr(state_d_errors.time, "monotonic", lambda: next(now))

    try:
        raise RuntimeError("camera read failed")
    except RuntimeError as error:
        first_error = error
        diagnosis.log_output(
            ResultDiagnosis.DETECTION,
            ResultDiagnosis.DETECTION,
            ModuleErrorIndex.CAMERA_MODULE_ERROR,
            first_error,
            2,
        )
        diagnosis.log_output(
            ResultDiagnosis.DETECTION,
            ResultDiagnosis.DETECTION,
            ModuleErrorIndex.CAMERA_MODULE_ERROR,
            first_error,
            2,
        )
        diagnosis.log_output(
            ResultDiagnosis.DETECTION,
            ResultDiagnosis.DETECTION,
            ModuleErrorIndex.CAMERA_MODULE_ERROR,
            first_error,
            2,
        )
    try:
        raise ValueError("different failure")
    except ValueError as error:
        diagnosis.log_output(
            ResultDiagnosis.DETECTION,
            ResultDiagnosis.DETECTION,
            ModuleErrorIndex.CAMERA_MODULE_ERROR,
            error,
            2,
        )

    log_text = log_path.read_text(encoding="utf-8")
    assert log_text.count("カメラ2モジュールエラー") == 3
    assert log_text.count("Traceback (most recent call last):") == 2
    assert "NoneType: None" not in log_text


def test_lidar_module_error_throttles_through_vendor_log_output(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    log_path = tmp_path / "throttled-lidar-module-error.log"
    diagnosis = LidarModuleError()
    err_conf = ErrorConfig()
    err_conf.lidar_module_error.is_enabled = True
    err_conf.lidar_module_error.ongoing_log_interval_sec = 60.0
    diagnosis.update(err_conf)
    logger_factory = AppLoggerFactory(
        to_console=False,
        to_file=str(log_path),
        level=DEBUG,
        compress=False,
    )
    diagnosis.log_register(logger_factory)
    logger_factory.update()
    now = iter((0.0, 1.0, 60.0, 61.0))
    monkeypatch.setattr(state_d_errors.time, "monotonic", lambda: next(now))

    try:
        raise RuntimeError("LiDAR read failed")
    except RuntimeError as error:
        for _ in range(3):
            diagnosis.log_output(
                ResultDiagnosis.DETECTION,
                ResultDiagnosis.DETECTION,
                ModuleErrorIndex.LIDAR_MODULE_ERROR,
                error,
                1,
            )
    try:
        raise ValueError("different LiDAR failure")
    except ValueError as error:
        diagnosis.log_output(
            ResultDiagnosis.DETECTION,
            ResultDiagnosis.DETECTION,
            ModuleErrorIndex.LIDAR_MODULE_ERROR,
            error,
            1,
        )

    log_text = log_path.read_text(encoding="utf-8")
    assert log_text.count("LiDAR1モジュールエラー") == 3
    assert log_text.count("Traceback (most recent call last):") == 2
    assert "NoneType: None" not in log_text


def test_accumulation_module_error_throttles_through_vendor_log_output(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    log_path = tmp_path / "throttled-accumulation-module-error.log"
    diagnosis = AccumulationModuleError()
    err_conf = ErrorConfig()
    err_conf.storage_module_error.is_enabled = True
    err_conf.storage_module_error.ongoing_log_interval_sec = 60.0
    diagnosis.update(err_conf)
    logger_factory = AppLoggerFactory(
        to_console=False,
        to_file=str(log_path),
        level=DEBUG,
        compress=False,
    )
    diagnosis.log_register(logger_factory)
    logger_factory.update()
    now = iter((0.0, 1.0, 60.0, 61.0))
    monkeypatch.setattr(state_d_errors.time, "monotonic", lambda: next(now))

    try:
        raise RuntimeError("accumulation failed")
    except RuntimeError as error:
        for _ in range(3):
            diagnosis.log_output(
                ResultDiagnosis.DETECTION,
                ResultDiagnosis.DETECTION,
                ModuleErrorIndex.ACCUMULATION_MODULE_ERROR,
                error,
            )
    try:
        raise ValueError("different accumulation failure")
    except ValueError as error:
        diagnosis.log_output(
            ResultDiagnosis.DETECTION,
            ResultDiagnosis.DETECTION,
            ModuleErrorIndex.ACCUMULATION_MODULE_ERROR,
            error,
        )

    log_text = log_path.read_text(encoding="utf-8")
    assert log_text.count("蓄積モジュールエラー") == 3
    assert log_text.count("Traceback (most recent call last):") == 2
    assert "NoneType: None" not in log_text


@pytest.mark.parametrize(
    ("diagnosis_type", "config_attribute", "module_index", "log_message"),
    (
        (
            CanModuleError,
            "can_module_error",
            ModuleErrorIndex.CAN_MODULE_ERROR,
            "CANモジュールエラー",
        ),
        (
            Integrate2d3dModuleError,
            "linkage_2d3d_module_error",
            ModuleErrorIndex.INTEGRATE_2D3D_MODULE_ERROR,
            "2D-3D紐づけモジュールエラー",
        ),
        (
            Object3DDetectionModuleError,
            "object3_d_detection_module_error",
            ModuleErrorIndex.OBJECT_3D_DETECTION_MODULE_ERROR,
            "3D物体検知モジュールエラー",
        ),
        (
            CameraHumanDetectionModuleError,
            "camera_human_detection_module_error",
            ModuleErrorIndex.CAMERA_HUMAN_DETECTION_MODULE_ERROR,
            "カメラ人検知モジュールエラー",
        ),
        (
            CollisionJudgmentModuleError,
            "collision_judgment_module_error",
            ModuleErrorIndex.COLLISION_JUDGMENT_MODULE_ERROR,
            "衝突判定モジュールエラー",
        ),
        (
            CalibrationModuleError,
            "calibration_module_error",
            ModuleErrorIndex.CALIBRATION_MODULE_ERROR,
            "校正モジュールエラー",
        ),
    ),
)
def test_configured_module_errors_throttle_through_vendor_log_output(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    diagnosis_type: type[StateErrorDiagnosisD],
    config_attribute: str,
    module_index: ModuleErrorIndex,
    log_message: str,
) -> None:
    log_path = tmp_path / f"{config_attribute}.log"
    diagnosis = diagnosis_type()
    err_conf = ErrorConfig()
    parameter: Any = getattr(err_conf, config_attribute)
    parameter.is_enabled = True
    parameter.ongoing_log_interval_sec = 60.0
    diagnosis.update(err_conf)
    logger_factory = AppLoggerFactory(
        to_console=False,
        to_file=str(log_path),
        level=DEBUG,
        compress=False,
    )
    diagnosis.log_register(logger_factory)
    logger_factory.update()
    now = iter((0.0, 1.0, 60.0, 61.0))
    monkeypatch.setattr(state_d_errors.time, "monotonic", lambda: next(now))

    try:
        raise RuntimeError("module failed")
    except RuntimeError as error:
        for _ in range(3):
            diagnosis.log_output(
                ResultDiagnosis.DETECTION,
                ResultDiagnosis.DETECTION,
                module_index,
                error,
            )
    try:
        raise ValueError("different module failure")
    except ValueError as error:
        diagnosis.log_output(
            ResultDiagnosis.DETECTION,
            ResultDiagnosis.DETECTION,
            module_index,
            error,
        )

    log_text = log_path.read_text(encoding="utf-8")
    assert log_text.count(log_message) == 3
    assert log_text.count("Traceback (most recent call last):") == 2
    assert "NoneType: None" not in log_text


@pytest.mark.parametrize(
    (
        "diagnosis_type",
        "config_attribute",
        "module_index",
        "log_message",
        "instance_index",
    ),
    (
        (
            ImuModuleError,
            "imu_module_error",
            ModuleErrorIndex.IMU_MODULE_ERROR,
            "IMU1モジュールエラー",
            1,
        ),
        (
            AppManagerModuleError,
            "app_manager_module_error",
            ModuleErrorIndex.APP_MANAGER_MODULE_ERROR,
            "アプリケーションマネージャーモジュールエラー",
            None,
        ),
        (
            MainModuleError,
            "main_module_error",
            ModuleErrorIndex.MAIN_MODULE_ERROR,
            "MainProcessモジュールエラー",
            None,
        ),
        (
            PointsRefineModuleError,
            "points_refine_module_error",
            ModuleErrorIndex.POINTS_REFINE_MODULE_ERROR,
            "PointsRefineモジュールエラー",
            None,
        ),
        (
            VisualModuleError,
            "visual_module_error",
            ModuleErrorIndex.VISUAL_MODULE_ERROR,
            "VisualProcessモジュールエラー",
            None,
        ),
        (
            LidarShiftMonitorModuleError,
            "lidar_shift_monitor_module_error",
            ModuleErrorIndex.LIDAR_SHIFT_MONITOR_MODULE_ERROR,
            "LiDARシフトモニタモジュールエラー",
            None,
        ),
        (
            GetDataModuleError,
            "get_data_module_error",
            ModuleErrorIndex.GET_DATA_MODULE_ERROR,
            "データ取得モジュールエラー",
            None,
        ),
    ),
)
def test_vendor_extension_module_errors_throttle_through_log_output(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    diagnosis_type: type[StateErrorDiagnosisD],
    config_attribute: str,
    module_index: ModuleErrorIndex,
    log_message: str,
    instance_index: int | None,
) -> None:
    log_path = tmp_path / f"{config_attribute}.log"
    diagnosis = diagnosis_type()
    err_conf = ErrorConfig()
    parameter: Any = getattr(err_conf, config_attribute)
    parameter.is_enabled = True
    parameter.ongoing_log_interval_sec = 60.0
    diagnosis.update(err_conf)
    logger_factory = AppLoggerFactory(
        to_console=False,
        to_file=str(log_path),
        level=DEBUG,
        compress=False,
    )
    diagnosis.log_register(logger_factory)
    logger_factory.update()
    now = iter((0.0, 1.0, 60.0, 61.0))
    monkeypatch.setattr(state_d_errors.time, "monotonic", lambda: next(now))

    try:
        raise RuntimeError("module failed")
    except RuntimeError as error:
        log_args = (error,) if instance_index is None else (error, instance_index)
        for _ in range(3):
            diagnosis.log_output(
                ResultDiagnosis.DETECTION,
                ResultDiagnosis.DETECTION,
                module_index,
                *log_args,
            )
    try:
        raise ValueError("different module failure")
    except ValueError as error:
        log_args = (error,) if instance_index is None else (error, instance_index)
        diagnosis.log_output(
            ResultDiagnosis.DETECTION,
            ResultDiagnosis.DETECTION,
            module_index,
            *log_args,
        )

    log_text = log_path.read_text(encoding="utf-8")
    assert log_text.count(log_message) == 3
    assert log_text.count("Traceback (most recent call last):") == 2
    assert "NoneType: None" not in log_text