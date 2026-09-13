from __future__ import annotations

from typing import Any

import numpy as np
from numpy.typing import NDArray

from argus_synchro.common.app_logger import AppLogger, AppLoggerFactory
from argus_synchro.config.app_config_calibration import AppConfigCalibration
from argus_synchro.detect2d import Detect2dDamoYoloOnnx
from argus_synchro.shared_errors import (
    ActionErrorIndex,
    SharedErrors,
    StateErrorDIndex,
)
from argus_synchro.shared_excepts import SharedExcepts


def _empty_yoloresult() -> list[NDArray[Any]]:
    return [
        np.zeros((0, 4), dtype=np.float32),
        np.zeros((0,), dtype=np.float32),
        np.zeros((0,), dtype=np.float32),
        np.array(0, dtype=np.int32),
    ]


class YOLODamoBatchAdapter:
    """Adapter to use Detect2dDamoYoloOnnx with calibcheck2d3d result format."""

    def __init__(
        self,
        app_config_calib: AppConfigCalibration,
        app_logger_factory: AppLoggerFactory,
        shared_errors: SharedErrors | None = None,
    ) -> None:
        proc2d = app_config_calib.calib2d3d.Proc2d
        camera_count = app_config_calib.dataCapture.Camera.count
        self._logger: AppLogger = app_logger_factory.register_from_type(self.__class__)
        self._camera_count = camera_count
        self._ser = shared_errors
        if self._ser is not None:
            self._ser.state_errors_D[StateErrorDIndex.AI_INFERENCE_RESULT_ERROR].update(
                self._ser.shared_err_conf.read()
            )
        try:
            self._detector = Detect2dDamoYoloOnnx(
                conf_thresh=proc2d.conf_thresh,
                nms_thresh=proc2d.nms_thresh,
                onnx_model_path=proc2d.yolo_modelpath,
                batch_size=camera_count,
                app_logger_factory=app_logger_factory,
            )
        except Exception as error:
            if self._ser is not None:
                diagnosis = self._ser.action_errors_A_C[
                    ActionErrorIndex.AI_MODEL_LOAD_FAILED
                ]
                if diagnosis.excepts_diagnosis(error):
                    self._logger.error(
                        "AI model initialization failed (CE013): %r", error
                    )
            raise

    def predict_batch(
        self,
        sec: SharedExcepts,
        frames: list[NDArray[np.uint8] | None],
    ) -> list[list[NDArray[Any]]]:
        if len(frames) != self._camera_count:
            msg = f"frames length must match camera count: {len(frames)} != {self._camera_count}"
            raise ValueError(msg)

        valid_mask = [frame is not None for frame in frames]
        valid_frames = [frame for frame in frames if frame is not None]

        if not valid_frames:
            return [_empty_yoloresult() for _ in range(self._camera_count)]

        reference = valid_frames[0]
        assert reference is not None
        filler = np.zeros_like(reference)

        batch_frames = [frame if frame is not None else filler for frame in frames]
        batch_array = np.asarray(batch_frames, dtype=np.uint8)

        detections = self._detector.object_detect(sec=sec, frames=batch_array)

        if self._ser is not None:
            ai_inference_result_error = self._ser.state_errors_D[
                StateErrorDIndex.AI_INFERENCE_RESULT_ERROR
            ]
            result, failsafe_result = ai_inference_result_error.errors_diagnosis(
                detections.boxes,
                detections.scores,
                detections.valid_detects,
            )
            ai_inference_result_error.log_output(
                result,
                failsafe_result,
                StateErrorDIndex.AI_INFERENCE_RESULT_ERROR,
                0,
            )

        results: list[list[NDArray[Any]]] = []
        for ix in range(self._camera_count):
            if not valid_mask[ix]:
                results.append(_empty_yoloresult())
                continue

            results.append(
                [
                    detections.boxes[ix],
                    detections.scores[ix],
                    detections.classes[ix],
                    np.array(detections.valid_detects[ix], dtype=np.int32),
                ]
            )

        return results
