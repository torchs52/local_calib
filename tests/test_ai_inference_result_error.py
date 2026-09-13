from types import SimpleNamespace
from unittest.mock import Mock

import numpy as np
import pytest

from argus_synchro.diagnosis.error_diagnosis import ResultDiagnosis
from argus_synchro.diagnosis.state_d_errors import AiInferenceResultError
from argus_synchro.message.input_message import CameraData
from argus_synchro.process.object_detect_process import ObjectDetectProcess
from argus_synchro.shared_errors import StateErrorDIndex


def _diagnosis() -> AiInferenceResultError:
    diagnosis = AiInferenceResultError()
    diagnosis.update(
        SimpleNamespace(
            ai_inference_result_error=SimpleNamespace(
                is_enabled=True,
                score_min=0.0,
                score_max=1.0,
            )
        )
    )
    return diagnosis


def _results() -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    return (
        np.zeros((3, 4, 4), dtype=np.float32),
        np.zeros((3, 4), dtype=np.float32),
        np.zeros(3, dtype=np.int32),
    )


@pytest.mark.parametrize(
    ("target", "value"),
    [("boxes", np.nan), ("scores", np.inf), ("scores", -0.1), ("scores", 1.1)],
)
def test_ai_inference_result_detects_invalid_numeric_results(
    target: str, value: float
) -> None:
    boxes, scores, valid_detects = _results()
    (boxes if target == "boxes" else scores).flat[0] = value

    assert _diagnosis().errors_diagnosis(boxes, scores, valid_detects)[0] == (
        ResultDiagnosis.DETECTION
    )


@pytest.mark.parametrize("valid_detect", [-1, 5])
def test_ai_inference_result_detects_invalid_count(valid_detect: int) -> None:
    boxes, scores, valid_detects = _results()
    valid_detects[0] = valid_detect

    assert _diagnosis().errors_diagnosis(boxes, scores, valid_detects)[0] == (
        ResultDiagnosis.DETECTION
    )


def test_ai_inference_result_recovers_for_valid_results() -> None:
    diagnosis = _diagnosis()
    boxes, scores, valid_detects = _results()
    scores[0, 0] = np.nan

    assert diagnosis.errors_diagnosis(boxes, scores, valid_detects)[0] == (
        ResultDiagnosis.DETECTION
    )
    scores[0, 0] = 1.0
    valid_detects[0] = boxes.shape[1]
    assert diagnosis.errors_diagnosis(boxes, scores, valid_detects)[0] == (
        ResultDiagnosis.RECOVERY
    )


def test_ai_inference_result_classifies_inference_exception() -> None:
    diagnosis = _diagnosis()
    diagnosis._logger = Mock()
    error = RuntimeError("inference failed")

    assert diagnosis.excepts_diagnosis(error)
    diagnosis.log_output(
        ResultDiagnosis.DETECTION,
        ResultDiagnosis.DETECTION,
        StateErrorDIndex.AI_INFERENCE_RESULT_ERROR,
        error,
    )

    diagnosis._logger.warning.assert_called_once_with(
        "D-level exception: RuntimeError: inference failed"
    )


def test_object_detect_process_diagnoses_inference_result_content() -> None:
    boxes, scores, valid_detects = _results()
    diagnosis = Mock()
    diagnosis.errors_diagnosis.return_value = (
        ResultDiagnosis.DETECTION,
        ResultDiagnosis.NORMAL,
    )
    state_errors = [None] * (StateErrorDIndex.AI_INFERENCE_RESULT_ERROR + 1)
    state_errors[StateErrorDIndex.AI_INFERENCE_RESULT_ERROR] = diagnosis
    process = object.__new__(ObjectDetectProcess)
    process._ser = SimpleNamespace(state_errors_D=state_errors)
    process._index = 2
    output = SimpleNamespace(
        boxes=boxes,
        scores=scores,
        valid_detects=valid_detects,
    )

    process._ai_inference_result_content_diagnosis(output)

    diagnosis.errors_diagnosis.assert_called_once_with(boxes, scores, valid_detects)
    diagnosis.log_output.assert_called_once_with(
        ResultDiagnosis.DETECTION,
        ResultDiagnosis.NORMAL,
        StateErrorDIndex.AI_INFERENCE_RESULT_ERROR,
        2,
    )


def test_object_detect_process_falls_back_to_empty_result_on_inference_error() -> None:
    process = object.__new__(ObjectDetectProcess)
    process._fps_prof = Mock()
    process._resize = SimpleNamespace(apply=lambda image: image)
    process._frames_buf = np.zeros((1, 2, 2, 3), dtype=np.uint8)
    process._provider = SimpleNamespace(
        get_undistort_image=lambda image, dst: np.copyto(dst, image)
    )
    process.input_detect2d_data_diagnosis = lambda: False
    process._detect2d = SimpleNamespace(
        object_detect=Mock(side_effect=RuntimeError("inference failed"))
    )
    process.sec = Mock()
    process._logger = Mock()
    process._ser = SimpleNamespace(
        is_state_error_d_exception=Mock(return_value=True)
    )
    camera = CameraData(
        index=2,
        frame=10,
        time=1.5,
        image=np.zeros((2, 2, 3), dtype=np.uint8),
    )

    result = process._update((camera,))

    assert result is not None
    assert result.index == 2
    assert result.frame == 10
    assert result.time == 1.5
    assert np.count_nonzero(result.boxes) == 0
    assert np.count_nonzero(result.scores) == 0
    assert np.count_nonzero(result.valid_detects) == 0
    process._ser.is_state_error_d_exception.assert_called_once()