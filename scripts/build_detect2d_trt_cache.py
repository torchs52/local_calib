from __future__ import annotations

import argparse
import time
from pathlib import Path

import numpy as np
import onnxruntime
from settings_swap import swap_settings

from argus_synchro.common import paths
from argus_synchro.config.app_config import AppConfig
from argus_synchro.detect2d import (
    create_onnx_inference_runner,
    create_onnx_inference_session,
    trt_profile_shape,
)

CONFIG_DIR = Path("./config")
LOG_DIR = Path("./log")
TRT_CACHE_GLOBS = ("*.engine", "*.timing", "*.profile")


def resolve_onnx_model_path(path: str) -> Path:
    model_path = Path(path)
    if not model_path.is_absolute():
        model_path = Path.cwd() / model_path
    return model_path.resolve()


def load_detect2d_config_from_settings() -> tuple[Path, int]:
    directory_config = paths.DirectoryConfig(
        config_dir=CONFIG_DIR.resolve(),
        log_dir=LOG_DIR,
        mmap_dir=paths.DEFAULT_MMAP_DIR,
    )
    directory_config, app_ini = paths.load_directory_config_from_ini(directory_config)
    app_config = AppConfig(app_ini, directory_config)
    onnx_model_path = resolve_onnx_model_path(app_config.detect2d.onnx_model_path)
    return onnx_model_path, app_config.camera.count


def enable_build_logging() -> None:
    onnxruntime.set_default_logger_severity(0)


def remove_trt_engine_cache(onnx_model_path: Path) -> list[Path]:
    cache_dir = onnx_model_path.parent
    removed: list[Path] = []
    for pattern in TRT_CACHE_GLOBS:
        for path in cache_dir.glob(pattern):
            path.unlink()
            removed.append(path)
    return removed


def build_trt_cache(onnx_model_path: Path, batch_size: int) -> None:
    print(f"TensorRTキャッシュを生成中: onnx={onnx_model_path}")
    print(f"プロファイル形状: {trt_profile_shape(batch_size)}")

    removed = remove_trt_engine_cache(onnx_model_path)
    if removed:
        print("既存のTensorRTキャッシュを削除しました")

    enable_build_logging()

    print("フェーズ: InferenceSession 作成中 (TensorRT エンジンビルド)...")
    started = time.perf_counter()
    session = create_onnx_inference_session(
        str(onnx_model_path),
        batch_size=batch_size,
        detailed_build_log=True,
    )
    session_elapsed = time.perf_counter() - started
    print(f"フェーズ: セッション作成完了 ({session_elapsed:.1f}秒)")
    print(f"プロバイダー: {session.get_providers()}")

    input_meta = session.get_inputs()[0]
    input_shape = (batch_size, 3, *input_meta.shape[2:])
    dummy_input = np.zeros(input_shape, dtype=np.float32)

    print("フェーズ: I/O Binding ウォームアップ推論中...")
    warmup_started = time.perf_counter()
    runner = create_onnx_inference_runner(
        session,
        onnx_file=str(onnx_model_path),
        input_name=input_meta.name,
        input_shape=tuple(dummy_input.shape),
    )
    runner.run(dummy_input)
    warmup_elapsed = time.perf_counter() - warmup_started
    print(f"フェーズ: I/O Binding ウォームアップ推論完了 ({warmup_elapsed:.1f}秒)")
    print("TensorRTキャッシュ生成が完了しました")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="DAMO-YOLO ONNX 向け TensorRT engine cache を事前生成する",
    )
    parser.add_argument(
        "--onnx-model-path",
        type=Path,
        help="ONNXモデルパス。未指定時は config の detect2d.onnx_model_path を使用",
    )
    parser.add_argument(
        "--batch-size",
        type=int,
        help="TRTプロファイルのバッチサイズ。未指定時は config の camera.count を使用",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()

    with swap_settings(CONFIG_DIR, mode="run"):
        if args.onnx_model_path is not None:
            onnx_model_path = resolve_onnx_model_path(str(args.onnx_model_path))
            batch_size = args.batch_size
        else:
            onnx_model_path, batch_size = load_detect2d_config_from_settings()

        if args.batch_size is not None:
            batch_size = args.batch_size

        build_trt_cache(onnx_model_path, batch_size)


if __name__ == "__main__":
    main()
