import sys
from types import SimpleNamespace
from unittest.mock import MagicMock

import pytest

from argus_synchro.diagnosis.action_errors import AiModelLoadFailed
from argus_synchro.diagnosis.error_config import ErrorConfig
from argus_synchro.process.object_detect_process import ObjectDetectProcess
from argus_synchro.shared_errors import (
    ActionErrorIndex,
    ModuleErrorIndex,
    StateErrorDIndex,
)


def _enabled_diagnosis() -> AiModelLoadFailed:
    diagnosis = AiModelLoadFailed()
    diagnosis.update(ErrorConfig())
    return diagnosis


@pytest.mark.parametrize(
    "error",
    (
        FileNotFoundError("missing"),
        PermissionError("denied"),
        OSError("io"),
        RuntimeError("runtime"),
        ValueError("invalid"),
        ImportError("import"),
        ModuleNotFoundError("module"),
    ),
)
def test_ai_model_load_failed_counts_target_exceptions(error: Exception) -> None:
    diagnosis = _enabled_diagnosis()

    assert diagnosis.excepts_diagnosis(error) is True
    assert diagnosis.err_cnt.value == 1


def test_ai_model_load_failed_ignores_non_target_exception() -> None:
    diagnosis = _enabled_diagnosis()

    assert diagnosis.excepts_diagnosis(TypeError("unexpected")) is False
    assert diagnosis.err_cnt.value == 0


def test_ai_model_load_failed_honors_disabled_setting() -> None:
    error_config = ErrorConfig()
    error_config.ai_model_load_failed.is_enabled = False
    diagnosis = AiModelLoadFailed()
    diagnosis.update(error_config)

    assert diagnosis.excepts_diagnosis(RuntimeError("model load failed")) is False
    assert diagnosis.err_cnt.value == 0


def test_object_detect_err_config_load_updates_ai_model_diagnosis() -> None:
    error_config = object()
    invalid_data_input = MagicMock()
    array_shape_error = MagicMock()
    ai_inference_result_error = MagicMock()
    ai_model_load_failed = MagicMock()
    camera_human_detection_module_error = MagicMock()
    process = object.__new__(ObjectDetectProcess)
    process._ser = SimpleNamespace(
        shared_err_conf=SimpleNamespace(read=lambda: error_config),
        state_errors_D={
            StateErrorDIndex.INVALID_DATA_INPUT: invalid_data_input,
            StateErrorDIndex.ARRAY_SHAPE_ERROR: array_shape_error,
            StateErrorDIndex.AI_INFERENCE_RESULT_ERROR: ai_inference_result_error,
        },
        action_errors_A_C={
            ActionErrorIndex.AI_MODEL_LOAD_FAILED: ai_model_load_failed
        },
        module_errors={
            ModuleErrorIndex.CAMERA_HUMAN_DETECTION_MODULE_ERROR: (
                camera_human_detection_module_error
            )
        },
    )

    process._err_config_load()

    ai_model_load_failed.update.assert_called_once_with(error_config)


def test_ai_model_load_failed_owns_error_log_output() -> None:
    diagnosis = _enabled_diagnosis()
    diagnosis._logger = MagicMock()
    error = RuntimeError("engine initialization failed")

    diagnosis.log_output(True, False, ActionErrorIndex.AI_MODEL_LOAD_FAILED, error)

    diagnosis._logger.error.assert_called_once_with(
        "CE013: AI model initialization failed: "
        "RuntimeError('engine initialization failed')"
    )


def _make_object_detect_process(diagnosis: AiModelLoadFailed) -> ObjectDetectProcess:
    process = object.__new__(ObjectDetectProcess)
    process._ProcessBase__process = None
    process._app_config = SimpleNamespace(
        detect2d=SimpleNamespace(
            is_applied=True,
            conf_thresh=0.5,
            nms_thresh=0.5,
            onnx_model_path="missing.onnx",
        ),
        camera=SimpleNamespace(count=1),
    )
    process._applied_detect2d = None
    process._app_logger_factory = MagicMock()
    process._logger = MagicMock()
    process._ser = SimpleNamespace(
        action_errors_A_C={ActionErrorIndex.AI_MODEL_LOAD_FAILED: diagnosis}
    )
    return process


@pytest.mark.parametrize(
    ("error", "expected_count", "expected_diagnosis_log"),
    (
        (RuntimeError("engine initialization failed"), 1, True),
        (TypeError("unexpected constructor error"), 0, False),
    ),
)
def test_object_detect_falls_back_when_model_initialization_fails(
    monkeypatch,
    error: Exception,
    expected_count: int,
    expected_diagnosis_log: bool,
) -> None:
    class FailingDetection:
        def __init__(self, **kwargs: object) -> None:
            raise error

    class NotAppliedDetection:
        pass

    diagnosis = _enabled_diagnosis()
    diagnosis._logger = MagicMock()
    process = _make_object_detect_process(diagnosis)
    fake_detect2d = SimpleNamespace(
        Detect2dDamoYoloOnnx=FailingDetection,
        NotAppliedObjDetection=NotAppliedDetection,
    )
    monkeypatch.setitem(sys.modules, "argus_synchro.detect2d", fake_detect2d)
    monkeypatch.setattr(
        "argus_synchro.process.object_detect_process.psutil.Process",
        lambda pid: SimpleNamespace(cpu_affinity=lambda: []),
    )
    monkeypatch.setattr(ObjectDetectProcess, "_build_provider", lambda self: None)

    process._apply_parameters()

    assert isinstance(process._detect2d, NotAppliedDetection)
    assert process._applied_detect2d is None
    assert diagnosis.err_cnt.value == expected_count
    assert diagnosis._logger.error.called is expected_diagnosis_log
    assert process._logger.warning.called is not expected_diagnosis_log