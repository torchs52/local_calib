from __future__ import annotations

import hashlib
import json
from abc import ABC, abstractmethod
from collections.abc import Sequence
from enum import Enum
from pathlib import Path
from typing import Any, Protocol, cast

import cv2
import numpy as np
import onnxruntime
from numpy.typing import NDArray

from argus_synchro.common.app_logger import AppLogger, AppLoggerFactory
from argus_synchro.config.app_config import AppConfig
from argus_synchro.message.scrutinizer_message import CamDet, CameraDetectionsData
from argus_synchro.shared_app_config import SharedAppConfig
from argus_synchro.shared_excepts import SharedExcepts

TRT_INPUT_NAME = "images"
TRT_MAX_WORKSPACE_SIZE = 16 * 1024 * 1024 * 1024
TRT_BUILDER_OPTIMIZATION_LEVEL = 5


class _OnnxRuntimeBackend(Enum):
    TENSORRT = "tensorrt"
    CUDA = "cuda"
    CPU = "cpu"


def trt_profile_shape(batch_size: int, input_name: str = TRT_INPUT_NAME) -> str:
    return f"{input_name}:{batch_size}x3x640x640"


def _onnx_input_name(onnx_file: str) -> str:
    metadata_session = onnxruntime.InferenceSession(
        onnx_file,
        providers=["CPUExecutionProvider"],
    )
    return metadata_session.get_inputs()[0].name


def _trt_cache_directory(onnx_file: str, batch_size: int) -> Path:
    model_path = Path(onnx_file).resolve()
    with model_path.open("rb") as model_file:
        model_digest = hashlib.file_digest(model_file, "sha256").hexdigest()[:16]

    settings = {
        "batch_size": batch_size,
        "builder_optimization_level": TRT_BUILDER_OPTIMIZATION_LEVEL,
        "context_memory_sharing": True,
        "cuda_graph": True,
        "fp16": True,
        "int8": False,
        "max_workspace_size": TRT_MAX_WORKSPACE_SIZE,
    }
    settings_json = json.dumps(settings, sort_keys=True, separators=(",", ":"))
    settings_digest = hashlib.sha256(settings_json.encode()).hexdigest()[:12]
    cache_dir = (
        model_path.parent
        / "trt_cache"
        / f"model-{model_digest}"
        / f"config-{settings_digest}"
    )
    cache_dir.mkdir(parents=True, exist_ok=True)
    return cache_dir


def trt_ep_options(
    onnx_file: str,
    batch_size: int,
    *,
    detailed_build_log: bool = False,
    input_name: str = TRT_INPUT_NAME,
) -> dict[str, Any]:
    cache_dir = str(_trt_cache_directory(onnx_file, batch_size))
    profile_shape = trt_profile_shape(batch_size, input_name)
    options: dict[str, Any] = {
        "device_id": 0,
        "trt_fp16_enable": True,
        "trt_int8_enable": False,
        "trt_max_workspace_size": TRT_MAX_WORKSPACE_SIZE,
        "trt_builder_optimization_level": TRT_BUILDER_OPTIMIZATION_LEVEL,
        "trt_auxiliary_streams": -1,
        "trt_engine_cache_enable": True,
        "trt_engine_cache_path": cache_dir,
        "trt_timing_cache_enable": True,
        "trt_timing_cache_path": cache_dir,
        "trt_cuda_graph_enable": True,
        "trt_context_memory_sharing_enable": True,
        "trt_dla_enable": False,
        "trt_profile_min_shapes": profile_shape,
        "trt_profile_opt_shapes": profile_shape,
        "trt_profile_max_shapes": profile_shape,
    }
    if detailed_build_log:
        options["trt_detailed_build_log"] = True
    return options


def _detect_onnx_runtime_backend(
    session: onnxruntime.InferenceSession,
) -> _OnnxRuntimeBackend:
    providers = session.get_providers()
    if "TensorrtExecutionProvider" in providers:
        return _OnnxRuntimeBackend.TENSORRT
    if "CUDAExecutionProvider" in providers:
        return _OnnxRuntimeBackend.CUDA
    return _OnnxRuntimeBackend.CPU


def _onnx_session_providers(
    onnx_file: str,
    batch_size: int,
    *,
    detailed_build_log: bool = False,
) -> list[str | tuple[str, dict[str, Any]]]:
    available_providers = onnxruntime.get_available_providers()
    providers: list[str | tuple[str, dict[str, Any]]] = ["CPUExecutionProvider"]
    if "CUDAExecutionProvider" in available_providers:
        providers.insert(0, "CUDAExecutionProvider")
    if "TensorrtExecutionProvider" in available_providers:
        providers.insert(
            0,
            (
                "TensorrtExecutionProvider",
                trt_ep_options(
                    onnx_file,
                    batch_size,
                    detailed_build_log=detailed_build_log,
                    input_name=_onnx_input_name(onnx_file),
                ),
            ),
        )
    return providers


class _OnnxInferenceRunner(Protocol):
    def run(
        self,
        input_numpy: NDArray[np.float32],
    ) -> tuple[NDArray[Any], NDArray[Any]]: ...


def _probe_onnx_output_shapes(
    onnx_file: str,
    *,
    input_name: str,
    input_shape: tuple[int, ...],
) -> dict[str, list[int]]:
    probe_sess = onnxruntime.InferenceSession(
        onnx_file,
        providers=[
            "CUDAExecutionProvider",
            "CPUExecutionProvider",
        ],
    )
    warmup_input = np.zeros(input_shape, dtype=np.float32)
    probe_outputs = cast(
        list[NDArray[Any]],
        probe_sess.run(None, {input_name: warmup_input}),
    )
    return {
        meta.name: list(array.shape)
        for meta, array in zip(probe_sess.get_outputs(), probe_outputs, strict=True)
    }


class _OnnxGpuIoBindingRunner:
    def __init__(
        self,
        session: onnxruntime.InferenceSession,
        *,
        input_name: str,
        input_shape: tuple[int, ...],
        output_shapes: dict[str, list[int]],
    ) -> None:
        self._session = session
        self._io_binding = session.io_binding()
        output_metas = session.get_outputs()
        self._output_names = [output.name for output in output_metas]

        warmup_input = np.zeros(input_shape, dtype=np.float32)

        self._input_ortvalue = onnxruntime.OrtValue.ortvalue_from_shape_and_type(
            list(input_shape),
            np.float32,
            "cuda",
            0,
        )
        self._io_binding.bind_ortvalue_input(input_name, self._input_ortvalue)

        self._output_ortvalues: dict[str, onnxruntime.OrtValue] = {}
        for meta in output_metas:
            ortvalue = onnxruntime.OrtValue.ortvalue_from_shape_and_type(
                output_shapes[meta.name],
                np.float32,
                "cuda",
                0,
            )
            self._io_binding.bind_ortvalue_output(meta.name, ortvalue)
            self._output_ortvalues[meta.name] = ortvalue

        self._input_ortvalue.update_inplace(warmup_input)
        session.run_with_iobinding(self._io_binding)

    def run(
        self,
        input_numpy: NDArray[np.float32],
    ) -> tuple[NDArray[Any], NDArray[Any]]:
        self._input_ortvalue.update_inplace(input_numpy)
        self._session.run_with_iobinding(self._io_binding)
        outputs = [self._output_ortvalues[name].numpy() for name in self._output_names]
        return outputs[0], outputs[1]


class _OnnxCpuRunner:
    def __init__(
        self,
        session: onnxruntime.InferenceSession,
        *,
        input_name: str,
    ) -> None:
        self._session = session
        self._input_name = input_name

    def run(
        self,
        input_numpy: NDArray[np.float32],
    ) -> tuple[NDArray[Any], NDArray[Any]]:
        scores, bboxes = self._session.run(None, {self._input_name: input_numpy})
        return scores, bboxes


def create_onnx_inference_runner(
    session: onnxruntime.InferenceSession,
    *,
    onnx_file: str,
    input_name: str,
    input_shape: tuple[int, ...],
) -> _OnnxInferenceRunner:
    backend = _detect_onnx_runtime_backend(session)
    if backend is _OnnxRuntimeBackend.CPU:
        return _OnnxCpuRunner(session, input_name=input_name)
    output_shapes = _probe_onnx_output_shapes(
        onnx_file,
        input_name=input_name,
        input_shape=input_shape,
    )
    return _OnnxGpuIoBindingRunner(
        session,
        input_name=input_name,
        input_shape=input_shape,
        output_shapes=output_shapes,
    )


def create_onnx_inference_session(
    onnx_file: str,
    *,
    batch_size: int,
    affinity_cores: list[int] | None = None,
    detailed_build_log: bool = False,
) -> onnxruntime.InferenceSession:
    sess_options: onnxruntime.SessionOptions = onnxruntime.SessionOptions()
    sess_options.graph_optimization_level = (
        onnxruntime.GraphOptimizationLevel.ORT_ENABLE_ALL
    )

    if affinity_cores is not None:
        sess_options.intra_op_num_threads = len(affinity_cores)
        extra_cores: list[int] = affinity_cores[1:]
        if extra_cores:
            affinity_cores_str: str = ";".join(str(n + 1) for n in extra_cores)
            sess_options.add_session_config_entry(
                "session.intra_op_thread_affinities",
                affinity_cores_str,
            )

    providers = _onnx_session_providers(
        onnx_file,
        batch_size,
        detailed_build_log=detailed_build_log,
    )
    try:
        return onnxruntime.InferenceSession(
            onnx_file,
            sess_options=sess_options,
            providers=providers,
        )
    except Exception as error:
        # 呼び出し側のCE013診断へ、失敗したモデルの文脈を付けて渡す。
        raise RuntimeError(f"failed to initialize ONNX session: {onnx_file}") from error


class ObjDetectionInterface(ABC):
    """物体検知を行うクラスが共通で持つメソッドを規定したもの
    外部とのインターフェイスとなる処理は、物体検知だけなので、そのメソッドだけ入出力を規定している
    """

    @abstractmethod
    def object_detect(
        self,
        sec: SharedExcepts,
        frames: NDArray[np.uint8],
    ) -> CameraDetectionsData:
        msg = "このクラスを継承したクラスは、object_detectを実装する必要があります"
        raise NotImplementedError(msg)


class ObjDetectionBase(ObjDetectionInterface, ABC):
    def __init__(
        self,
        conf_thresh: float,
        nms_thresh: float,
        onnx_model_path: str | None,
        batch_size: int,
        affinity_cores: list[int] | None,
    ) -> None:
        self.conf_thresh: float = conf_thresh
        self.nms_thresh: float = nms_thresh
        self.onnx_file: str | None = onnx_model_path
        self.batch_size: int = batch_size
        self.affinity_cores: list[int] | None = affinity_cores

    def update(self, app_config: AppConfig) -> None:
        self.conf_thresh: float = app_config.detect2d.conf_thresh
        self.nms_thresh: float = app_config.detect2d.nms_thresh
        self.onnx_file: str | None = app_config.detect2d.onnx_model_path

    def start_onnx_session(self) -> onnxruntime.InferenceSession:
        """
        onnx形式の処理を行うためのsessionを実行する
        """
        if not self.onnx_file:
            msg = "onnx file is needed to start onnx session."
            raise ValueError(msg)
        return create_onnx_inference_session(
            self.onnx_file,
            batch_size=self.batch_size,
            affinity_cores=self.affinity_cores,
        )


class Detect2dDamoYoloOnnx(ObjDetectionBase):
    def __init__(
        self,
        conf_thresh: float,
        nms_thresh: float,
        onnx_model_path: str | None,
        batch_size: int,
        app_logger_factory: AppLoggerFactory,
        affinity_cores: list[int] | None = None,
    ) -> None:
        """DAMO-Yolo用の人検知モデルでonnxファイルを用いて人検知を行うクラス
        参考Github:
        https://github.com/Kazuhito00/DAMO-YOLO-ONNX-Sample

        Detect2dのコンストラクタを呼ぼうと思ったが、このクラスはonnxファイル前提で処理をするが、
        Detect2dのコンストラクタはonnxファイルを使うかどうかを設定ファイルで制御していて、場合によっては意図した処理が行われない可能性があるので、
        Detect2dのコンストラクタを呼ばずに、似た処理を実行することにする
        """
        self._app_logger_factory: AppLoggerFactory = app_logger_factory
        self._logger: AppLogger = app_logger_factory.register_from_type(self.__class__)
        super().__init__(
            conf_thresh=conf_thresh,
            nms_thresh=nms_thresh,
            onnx_model_path=onnx_model_path,
            batch_size=batch_size,
            affinity_cores=affinity_cores,
        )
        self.model_sess: onnxruntime.InferenceSession = self.start_onnx_session()
        self.model_input_size: tuple[int, int] = tuple(
            self.model_sess.get_inputs()[0].shape[2:]
        )
        self.input_name: str = self.model_sess.get_inputs()[0].name
        self._input_buffer: NDArray[np.float32] = np.zeros(
            (batch_size, 3, *self.model_input_size),
            dtype=np.float32,
        )
        self._infer = create_onnx_inference_runner(
            self.model_sess,
            onnx_file=cast(str, self.onnx_file),
            input_name=self.input_name,
            input_shape=tuple(self._input_buffer.shape),
        )

    def _inference(
        self,
        frames: NDArray[np.uint8],
    ) -> tuple[
        NDArray[np.float32],
        NDArray[np.float32],
        NDArray[np.int64],
        NDArray[np.int32],
    ]:
        """damo-yoloを用いて、与えられた画像frameに対して、推論を行い、推論結果を返す"""
        for index, frame in enumerate(frames):
            self._input_buffer[index] = self._preprocess(frame)

        scores, bboxes = self._infer.run(self._input_buffer)

        bboxes, scores, class_ids, valid_detects = self._postprocess(bboxes, scores)

        return (bboxes, scores, class_ids, valid_detects)

    def update(self, app_config: AppConfig) -> None:
        super().update(app_config)

    def object_detect(
        self,
        sec: SharedExcepts,
        frames: NDArray[np.uint8],
    ) -> CameraDetectionsData:
        """
        damo-yoloにおける人検知の処理全体
        """
        ################################
        # カメラ読み込みは外部で実施.
        # cam = self.get_cam_stream()
        ################################
        # frames_recorded = 0
        self._logger.info(f"cam:{sec.check_scrut_mode_is_finished()=}")
        bboxes, s_scores, class_ids, valid_detections = self._inference(frames)
        return CameraDetectionsData(
            0,
            0,
            0,
            bboxes,
            s_scores,
            class_ids,
            valid_detections,
            frames,
        )

    def _preprocess(
        self,
        image: NDArray[np.uint8],
        data_order: tuple[int, int, int] = (2, 0, 1),
    ) -> NDArray[np.float32]:
        """damo-yoloによる物体検知に必要な変換を行う"""
        # damo-yoloはRGBで処理が行われるので、色を変換
        _width, _height = self.model_input_size
        temp_image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

        # damo-yoloの入力サイズに変更
        resized_image = cv2.resize(
            temp_image,
            self.model_input_size,
            interpolation=cv2.INTER_LINEAR,
        )
        resized_image = resized_image.astype(np.uint8)

        # damo-yoloの入力サイズより画像が小さい場合、paddingする
        if len(image.shape) == 3:
            padded_image = np.ones(
                (self.model_input_size[0], self.model_input_size[1], 3),
                dtype=np.uint8,
            )
        else:
            padded_image = np.ones(self.model_input_size, dtype=np.uint8)
        padded_image[: temp_image.shape[0], : temp_image.shape[1]] = resized_image

        # damo-yoloの入力shapeに変更
        padded_image: NDArray[np.int8] = padded_image.transpose(data_order)
        padded_image: NDArray[np.float32] = np.ascontiguousarray(
            padded_image,
            dtype=np.float32,
        )
        return padded_image

    def _postprocess(
        self,
        bboxes: NDArray[np.float32],
        scores: NDArray[np.float32],
        max_output_size_per_class: int = 50,
    ) -> tuple[
        NDArray[np.float32],
        NDArray[np.float32],
        NDArray[np.int64],
        NDArray[np.int32],
    ]:
        """scoresとbboxesの位置を基に最終的なbounding boxを得る"""

        # Note: bboxesでbatch_sizeを取得しようとすると、batch_sizeを可変にしたonnxファイルが意図しない数のbboxesの数になっているが、
        # scoresは正常な数のbatch_sizeが帰ってきているように見えるので、scoresからbatch_sizeを取得している
        # onnxファイルが修正されれば、bboxesから取得するのでも良いかと思われる
        batch_size = scores.shape[0]
        yolo_width, yolo_height = self.model_input_size

        # bboxが0の場合、class_idsは空行列を返す
        bboxes_batch: list[NDArray[np.float32]] = []
        scores_batch: list[NDArray[np.float32]] = []
        classes_batch: list[NDArray[np.int64]] = []
        valid_detects: list[int] = []
        for i in range(batch_size):
            cls_bboxes, cls_scores, cls_class_ids = self._multiclass_nms(
                bboxes[i],
                scores[i],
                max_output_size_per_class,
            )

            # 有効な数が以下の処理で消えるので、その前に有効なbounding box数を追加
            valid_detects.append(len(cls_bboxes))

            appended_bboxes: NDArray[np.float32] = np.zeros(
                (max_output_size_per_class, 4),
                dtype=np.float32,
            )
            appended_scores: NDArray[np.float32] = np.zeros(
                max_output_size_per_class,
                dtype=np.float32,
            )
            appended_classes: NDArray[np.int64] = np.zeros(
                max_output_size_per_class,
                dtype=np.int64,
            )
            if len(cls_bboxes) > 0:
                # bounding boxの座標[0,1]に正規化 -> 後述のdraw_bboxの処理と合わせるための処理
                # 後続処理に合う形にbboxesを変換する
                proc_bboxes: NDArray[np.float32] = cls_bboxes.copy()
                proc_bboxes[..., 0] = cls_bboxes[..., 1] / yolo_height
                proc_bboxes[..., 1] = cls_bboxes[..., 0] / yolo_width
                proc_bboxes[..., 2] = cls_bboxes[..., 3] / yolo_height
                proc_bboxes[..., 3] = cls_bboxes[..., 2] / yolo_width
                appended_bboxes[: len(cls_bboxes)] = proc_bboxes
                appended_scores[: len(cls_scores)] = cls_scores
                appended_classes[: len(cls_class_ids)] = cls_class_ids

            bboxes_batch.append(appended_bboxes)
            scores_batch.append(appended_scores)
            classes_batch.append(appended_classes)

        return (
            np.array(bboxes_batch),
            np.array(scores_batch),
            np.array(classes_batch),
            np.array(valid_detects, dtype=np.int32),
        )

    def _multiclass_nms(
        self,
        bboxes: NDArray[np.float32],
        scores: NDArray[np.float32],
        max_num: int,
        score_factors: NDArray[np.float32] | None = None,
    ) -> tuple[NDArray[np.float32], NDArray[np.float32], NDArray[np.int64]]:
        num_classes: int = scores.shape[1]
        score_th: float = self.conf_thresh
        nms_th: float = self.nms_thresh

        if bboxes.shape[1] > 4:
            # bboxes = bboxes.view(scores.size(0), -1, 4)
            pass
        else:
            bboxes = np.broadcast_to(
                bboxes[:, None],
                (bboxes.shape[0], num_classes, 4),
            )
        valid_mask = scores > score_th
        valid_bboxes: NDArray[np.float32] = bboxes[valid_mask]

        if score_factors is not None:
            scores = scores * score_factors[:, None]
        vlid_scores: NDArray[np.float32] = scores[valid_mask]

        valid_labels: NDArray[np.int64] = valid_mask.nonzero()[1]

        indices: Sequence[int] = cv2.dnn.NMSBoxes(
            valid_bboxes.tolist(),
            vlid_scores.tolist(),
            score_th,
            nms_th,
        )

        if max_num > 0:
            indices: NDArray[np.int32] = indices[:max_num]

        if len(indices) > 0:
            nms_bboxes: NDArray[np.float32] = valid_bboxes[indices]
            nms_scores: NDArray[np.float32] = vlid_scores[indices]
            nms_labels: NDArray[np.int64] = valid_labels[indices]
            return nms_bboxes, nms_scores, nms_labels
        return np.array([]), np.array([]), np.array([])


class NotAppliedObjDetection(ObjDetectionInterface):
    def object_detect(
        self,
        sec: SharedExcepts,
        frames: NDArray[np.uint8],
    ) -> CameraDetectionsData:
        # 人検知結果のメモリをクリアする.
        # bbox座標
        boxes: NDArray[np.float32] = np.zeros(
            [len(frames), CamDet.MAX_TOTAL_SIZE, 4],
        ).astype(np.float32)
        # 確信度
        scores: NDArray[np.float32] = np.zeros(
            [len(frames), CamDet.MAX_TOTAL_SIZE]
        ).astype(np.float32)
        # class ID
        classes: NDArray[np.int64] = np.zeros(
            [len(frames), CamDet.MAX_TOTAL_SIZE]
        ).astype(np.int64)
        valid_detects: NDArray[np.int32] = np.zeros((len(frames),)).astype(np.int32)

        return CameraDetectionsData(
            0,
            0,
            0,
            boxes,
            scores,
            classes,
            valid_detects,
            np.asarray(frames),
        )


if __name__ == "__main__":
    CAMERA_COUNT = 3
    c = NotAppliedObjDetection()

    frames: NDArray[np.uint8] = np.zeros((CAMERA_COUNT, 1280, 720, 3), np.uint8)

    cdd: CameraDetectionsData = c.object_detect(
        SharedExcepts(app_config=SharedAppConfig().read()), frames
    )

    print(f"cdd.boxes:  {cdd.boxes.shape}")
    print(f"cdd.scores: {cdd.scores.shape}")
    print(f"cdd.boxes:  {cdd.classes.shape}")
    print(f"cdd.valid_detects: {cdd.valid_detects}")
    print(f"cdd.image:  {cdd.image.shape}")

    for i in range(CAMERA_COUNT):
        box = cdd.boxes[i]
        scores = cdd.scores[i]
        classes = cdd.classes[i]
        image = cdd.image[i]
        print(f"cdd.boxes[{i}]:  {box}")
        print(f"cdd.scores[{i}]: {scores}")
        print(f"cdd.classes[{i}]:  {classes}")
        print(f"cdd.image[{i}]:  {image}")
