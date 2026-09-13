from __future__ import annotations

from pathlib import Path
from unittest.mock import MagicMock

import numpy as np
import pytest

from argus_synchro.common.app_logger import AppLoggerFactory
from argus_synchro.config.app_config import AppConfig
from argus_synchro.detect2d import (
    TRT_MAX_WORKSPACE_SIZE,
    Detect2dDamoYoloOnnx,
    _OnnxRuntimeBackend,
    _detect_onnx_runtime_backend,
    _onnx_input_name,
    _onnx_session_providers,
    _trt_cache_directory,
    create_onnx_inference_session,
    trt_ep_options,
    trt_profile_shape,
)


class _FakeOnnxSession:
    def __init__(self, providers: list[str]) -> None:
        self._providers = providers

    def get_providers(self) -> list[str]:
        return self._providers


@pytest.mark.parametrize(
    ("providers", "expected"),
    [
        (["TensorrtExecutionProvider", "CUDAExecutionProvider"], _OnnxRuntimeBackend.TENSORRT),
        (["CUDAExecutionProvider", "CPUExecutionProvider"], _OnnxRuntimeBackend.CUDA),
        (["CPUExecutionProvider"], _OnnxRuntimeBackend.CPU),
    ],
)
def test_detect_onnx_runtime_backend(
    providers: list[str],
    expected: _OnnxRuntimeBackend,
) -> None:
    assert _detect_onnx_runtime_backend(_FakeOnnxSession(providers)) is expected


@pytest.mark.parametrize(
    ("available", "expected_names"),
    [
        (["CPUExecutionProvider"], ["CPUExecutionProvider"]),
        (
            ["CUDAExecutionProvider", "CPUExecutionProvider"],
            ["CUDAExecutionProvider", "CPUExecutionProvider"],
        ),
        (
            [
                "TensorrtExecutionProvider",
                "CUDAExecutionProvider",
                "CPUExecutionProvider",
            ],
            [
                "TensorrtExecutionProvider",
                "CUDAExecutionProvider",
                "CPUExecutionProvider",
            ],
        ),
    ],
)
def test_onnx_session_providers_uses_available_providers_in_priority_order(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
    available: list[str],
    expected_names: list[str],
) -> None:
    monkeypatch.setattr(
        "argus_synchro.detect2d.onnxruntime.get_available_providers",
        lambda: available,
    )
    monkeypatch.setattr("argus_synchro.detect2d._onnx_input_name", lambda _: "images")
    onnx_file = tmp_path / "model.onnx"
    onnx_file.touch()

    providers = _onnx_session_providers(str(onnx_file), 2)
    provider_names = [item[0] if isinstance(item, tuple) else item for item in providers]

    assert provider_names == expected_names


def test_onnx_input_name_reads_model_metadata(monkeypatch: pytest.MonkeyPatch) -> None:
    input_metadata = type("InputMetadata", (), {"name": "input_tensor"})()

    class MetadataSession:
        def get_inputs(self) -> list[object]:
            return [input_metadata]

    def create_session(
        onnx_file: str,
        *,
        providers: list[str],
    ) -> MetadataSession:
        assert onnx_file == "model.onnx"
        assert providers == ["CPUExecutionProvider"]
        return MetadataSession()

    monkeypatch.setattr(
        "argus_synchro.detect2d.onnxruntime.InferenceSession",
        create_session,
    )

    assert _onnx_input_name("model.onnx") == "input_tensor"


def test_onnx_session_failure_includes_model_path(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setattr(
        "argus_synchro.detect2d._onnx_session_providers",
        lambda *args, **kwargs: ["CPUExecutionProvider"],
    )
    original_error = OSError("invalid model")
    monkeypatch.setattr(
        "argus_synchro.detect2d.onnxruntime.InferenceSession",
        MagicMock(side_effect=original_error),
    )

    with pytest.raises(
        RuntimeError,
        match=r"failed to initialize ONNX session: missing\.onnx",
    ) as exc_info:
        create_onnx_inference_session("missing.onnx", batch_size=1)

    assert exc_info.value.__cause__ is original_error


def test_tensorrt_provider_uses_model_input_name(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
) -> None:
    monkeypatch.setattr(
        "argus_synchro.detect2d.onnxruntime.get_available_providers",
        lambda: ["TensorrtExecutionProvider", "CPUExecutionProvider"],
    )
    monkeypatch.setattr("argus_synchro.detect2d._onnx_input_name", lambda _: "input_tensor")
    onnx_file = tmp_path / "model.onnx"
    onnx_file.touch()

    providers = _onnx_session_providers(str(onnx_file), 3)

    assert isinstance(providers[0], tuple)
    assert providers[0][1]["trt_profile_min_shapes"] == "input_tensor:3x3x640x640"


@pytest.mark.parametrize("batch_size", [1, 2, 3])
def test_trt_ep_options_reflects_performance_settings(
    tmp_path: Path,
    batch_size: int,
) -> None:
    onnx_file = tmp_path / "model.onnx"
    onnx_file.touch()

    options = trt_ep_options(str(onnx_file), batch_size)

    assert options["trt_max_workspace_size"] == TRT_MAX_WORKSPACE_SIZE
    assert options["trt_builder_optimization_level"] == 5
    assert options["trt_cuda_graph_enable"] is True
    cache_path = Path(options["trt_engine_cache_path"])
    assert cache_path.parent.parent == tmp_path / "trt_cache"
    assert cache_path.name.startswith("config-")
    assert cache_path.parent.name.startswith("model-")
    assert options["trt_timing_cache_path"] == str(cache_path)
    assert options["trt_profile_min_shapes"] == trt_profile_shape(batch_size)
    assert options["trt_profile_opt_shapes"] == trt_profile_shape(batch_size)
    assert options["trt_profile_max_shapes"] == trt_profile_shape(batch_size)


def test_trt_cache_directory_separates_models_and_batch_settings(tmp_path: Path) -> None:
    first_model = tmp_path / "first.onnx"
    second_model = tmp_path / "second.onnx"
    first_model.write_bytes(b"first model")
    second_model.write_bytes(b"second model")

    first_batch_one = _trt_cache_directory(str(first_model), 1)
    first_batch_two = _trt_cache_directory(str(first_model), 2)
    second_batch_one = _trt_cache_directory(str(second_model), 1)

    assert first_batch_one != first_batch_two
    assert first_batch_one.parent != second_batch_one.parent
    assert first_batch_one.is_dir()
    assert first_batch_two.is_dir()
    assert second_batch_one.is_dir()


@pytest.mark.parametrize("batch_size", [1, 2, 3])
def test_damoyolo_onnx_accepts_batch_on_tensorrt(
    app_config: AppConfig,
    batch_size: int,
) -> None:
    """TRTプロファイルをバッチ数に合わせた場合、batch=1/2/3 推論が通ることを確認する。"""

    model_path = Path(app_config.detect2d.onnx_model_path)
    if not model_path.is_absolute():
        model_path = Path.cwd() / model_path

    assert model_path.exists(), f"ONNXモデルが存在しません: {model_path}"

    detector = Detect2dDamoYoloOnnx(
        conf_thresh=app_config.detect2d.conf_thresh,
        nms_thresh=app_config.detect2d.nms_thresh,
        onnx_model_path=str(model_path),
        batch_size=batch_size,
        app_logger_factory=AppLoggerFactory(to_console=False),
    )

    if "TensorrtExecutionProvider" not in detector.model_sess.get_providers():
        pytest.skip("TensorrtExecutionProvider is not active in this environment")

    frames = np.zeros((batch_size, 640, 640, 3), dtype=np.uint8)
    bboxes, scores, class_ids, valid_detects = detector._inference(frames)  # noqa: SLF001

    assert bboxes.shape == (batch_size, 50, 4)
    assert scores.shape == (batch_size, 50)
    assert class_ids.shape == (batch_size, 50)
    assert valid_detects.shape == (batch_size,)
