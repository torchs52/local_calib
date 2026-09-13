from __future__ import annotations

from types import SimpleNamespace
from unittest.mock import MagicMock

import numpy as np
import pytest

from argus_synchro.calibration_mat_generator_modules.ctrl.calibration2d3d import (
    calibration2d3d_class,
)
from argus_synchro.calibration_mat_generator_modules.ctrl.calibration2d3d.calc_progress import (
    calc_progress_class,
)
from argus_synchro.calibration_mat_generator_modules.ctrl.calibration2d3d.correspondence import (
    correspondence_class_base,
    correspondence_class_optmethod,
    pnp_estimation_opt,
)
from argus_synchro.calibration_mat_generator_modules.ctrl.calibration2d3d.track_main.detect2D.filters.bbox2Dmask_byimage import (
    bbox2Dmask_byimage,
)
from argus_synchro.calibration_mat_generator_modules.ctrl.calibration2d3d.track_main.detect2D.image_preprocess import (
    image_preprocess,
)
from argus_synchro.diagnosis.calib2d3d_result_diagnosis import (
    Calib2d3dErrorCommon,
    Calib2d3dResultDiagnosis,
    CameraCalibrationStatus,
    CameraCalibrationStatusDiagnosis,
)
from argus_synchro.diagnosis.error_diagnosis import ResultDiagnosis
from argus_synchro.shared_errors import StateErrorDIndex
from argus_synchro.shared_excepts import SharedExcepts


class _RecordingDiagnosis:
    def __init__(self) -> None:
        self.diagnosis_calls: list[tuple[object, ...]] = []
        self.log_calls: list[tuple[object, ...]] = []

    def errors_diagnosis(
        self, *args: object
    ) -> tuple[ResultDiagnosis, ResultDiagnosis]:
        self.diagnosis_calls.append(args)
        return ResultDiagnosis.DETECTION, ResultDiagnosis.NORMAL

    def log_output(self, *args: object) -> None:
        self.log_calls.append(args)


def test_calibration2d3d_reports_file_io_error_details() -> None:
    diagnosis = _RecordingDiagnosis()
    state_errors_d: list[object | None] = [None] * (
        StateErrorDIndex.FILE_IO_ERROR + 1
    )
    state_errors_d[StateErrorDIndex.FILE_IO_ERROR] = diagnosis
    calibration = object.__new__(calibration2d3d_class)
    calibration._ser = SimpleNamespace(state_errors_D=state_errors_d)

    calibration._report_file_io_error_impl(
        "/config/area.json",
        "read 2D-3D area definition JSON",
        OSError("read failed"),
    )

    assert diagnosis.diagnosis_calls == [(True,)]
    assert diagnosis.log_calls == [
        (
            ResultDiagnosis.DETECTION,
            ResultDiagnosis.NORMAL,
            StateErrorDIndex.FILE_IO_ERROR,
            "/config/area.json",
            "read 2D-3D area definition JSON",
            "OSError: read failed",
        )
    ]


def test_correspondence_reports_postprocess_matrix_read_error(tmp_path) -> None:
    matrix_path = tmp_path / "missing.csv"
    reports: list[tuple[str, str, Exception]] = []
    config = SimpleNamespace(
        calib2d3d=SimpleNamespace(
            CalcCorrespondence=SimpleNamespace(postprocess_mat=str(matrix_path))
        )
    )
    logger_factory = MagicMock()
    logger_factory.register_from_type.return_value = MagicMock()

    with pytest.raises(OSError):
        correspondence_class_base(
            app_config_calib=config,
            app_logger_factory=logger_factory,
            file_io_error_reporter=lambda path, operation, error: reports.append(
                (path, operation, error)
            ),
        )

    assert reports[0][:2] == (
        str(matrix_path),
        "read 2D-3D postprocess matrix CSV",
    )
    assert isinstance(reports[0][2], OSError)


def test_correspondence_reports_initial_vector_read_error(
    tmp_path, monkeypatch
) -> None:
    vector_path = tmp_path / "missing.json"
    reports: list[tuple[str, str, Exception]] = []
    correspondence = object.__new__(correspondence_class_optmethod)
    correspondence._app_logger_factory = MagicMock()
    correspondence.file_io_error_reporter = (
        lambda path, operation, error: reports.append((path, operation, error))
    )
    config = SimpleNamespace(
        dataCapture=SimpleNamespace(
            Camera=SimpleNamespace(sys_width=1920, sys_height=1080)
        ),
        calib2d3d=SimpleNamespace(
            CalcCorrespondence=SimpleNamespace(
                opt_lambda_center_r=[1.0],
                opt_lambda_center_t=[1.0],
                opt_lambda_axis_r=[1.0],
                opt_lambda_axis_t=[1.0],
                optparam_initialvector=str(vector_path),
            )
        ),
    )
    correspondence.app_config_calib = config
    monkeypatch.setattr(
        pnp_estimation_opt,
        "PnpEstimationCalculator",
        lambda **_kwargs: object(),
    )

    with pytest.raises(OSError):
        correspondence.reset(config, np.eye(3), 0)

    assert reports[0][:2] == (
        str(vector_path),
        "read 2D-3D initial vector JSON",
    )
    assert isinstance(reports[0][2], OSError)


def test_pnp_artifact_write_error_is_reported_and_absorbed(
    tmp_path, monkeypatch
) -> None:
    reports: list[tuple[str, str, Exception]] = []
    logger_factory = MagicMock()
    logger_factory.register_from_type.return_value = MagicMock()
    calculator = pnp_estimation_opt.PnpEstimationCalculator(
        dist_coeffs=np.zeros((1, 5)),
        camera_matrix=np.eye(3),
        app_logger_factory=logger_factory,
        file_io_error_reporter=lambda path, operation, error: reports.append(
            (path, operation, error)
        ),
    )
    calculator.rotation_matrix = np.eye(3)
    calculator.rotation_vector = np.zeros(3)
    calculator.translation_vector = np.zeros(3)
    calculator.corner2d_save = np.zeros((1, 2))
    calculator.corner3d_save = np.zeros((1, 3))
    write_error = OSError("write failed")
    monkeypatch.setattr(np, "save", MagicMock(side_effect=write_error))

    calculator.save(savedir=str(tmp_path))

    assert reports == [
        (
            str(tmp_path / "rotation_vector.npy"),
            "write 2D-3D PnP calibration artifact",
            write_error,
        )
    ]


@pytest.mark.parametrize(
    ("reader", "expected_operation"),
    (
        (image_preprocess, "read 2D-3D camera mask image"),
        (bbox2Dmask_byimage, "read 2D-3D bbox mask image"),
    ),
)
def test_2d_tracking_reports_mask_image_read_error(
    tmp_path, reader, expected_operation: str
) -> None:
    mask_path = tmp_path / "missing.png"
    reports: list[tuple[str, str, Exception]] = []
    config = SimpleNamespace(
        default=SimpleNamespace(print_disabled=True),
        dataCapture=SimpleNamespace(Camera=SimpleNamespace(sys_width=2, sys_height=2)),
        calib2d3d=SimpleNamespace(
            Proc2d=SimpleNamespace(
                enable_imgmask=True,
                enable_bboxfilter_byimg=True,
                camera_mask_images=[str(mask_path)],
            )
        ),
    )
    kwargs = {
        "app_config_calib": config,
        "camera_index": 0,
        "file_io_error_reporter": lambda path, operation, error: reports.append(
            (path, operation, error)
        ),
    }
    if reader is bbox2Dmask_byimage:
        logger_factory = MagicMock()
        logger_factory.register_from_type.return_value = MagicMock()
        kwargs["app_logger_factory"] = logger_factory

    with pytest.raises(OSError, match="image mask could not be loaded"):
        reader(**kwargs)

    assert reports[0][:2] == (str(mask_path), expected_operation)
    assert isinstance(reports[0][2], OSError)


def _autoexit_calibration(point_count: int) -> calibration2d3d_class:
    calibration = object.__new__(calibration2d3d_class)
    calibration.finalized_fileend_autoexit = False
    calibration.datasource_endflag = False
    calibration.lastts2d = 12
    calibration.lastts3d = 10
    calibration.facade_index_offset = 2
    calibration.final_2dpoints = np.ones((point_count, 2))
    calibration.final_3dpoints = np.ones((point_count, 3))
    calibration.current_progress_score = 1.0
    calibration.progress_score = 0.9
    calibration.progress_mem = 0.4
    calibration.camera_id = 1
    calibration.app_config_calib = MagicMock()
    calibration.app_config_calib.calib2d3d.CalcAccuracy.check_enable = True
    calibration._result_diagnosis = Calib2d3dResultDiagnosis()
    return calibration


def test_fileend_autoexit_rejects_insufficient_calibration_points(tmp_path) -> None:
    calibration = _autoexit_calibration(point_count=20)
    calibration.get_calibval = MagicMock()
    monitor = MagicMock()

    assert calibration.finalize_fileend_autoexit(
        monitor=monitor,
        sec=MagicMock(spec=SharedExcepts),
        sac=MagicMock(),
        resultmat_path=str(tmp_path / "result.csv"),
    )

    monitor.set_errorcode_unexpected_exception.assert_called_once_with(True)
    calibration.get_calibval.assert_not_called()


def test_fileend_autoexit_calculates_and_writes_result_once(tmp_path) -> None:
    calibration = _autoexit_calibration(point_count=40)
    calibration.get_calibval = MagicMock(return_value=(np.eye(4), 1.5))
    monitor = MagicMock()
    result_path = tmp_path / "result.csv"
    arguments = {
        "monitor": monitor,
        "sec": MagicMock(spec=SharedExcepts),
        "sac": MagicMock(),
        "resultmat_path": str(result_path),
    }

    assert calibration.finalize_fileend_autoexit(**arguments)
    assert calibration.finalize_fileend_autoexit(**arguments)

    calibration.get_calibval.assert_called_once_with(
        recalc_bbox_index=True,
        frame_ix=10,
    )
    np.testing.assert_allclose(np.loadtxt(result_path, delimiter=","), np.eye(4))
    monitor.set_camera_calibration_status.assert_called_once_with(
        camera_id=1, value=1
    )


def test_result_matrix_write_error_is_reported_and_reraised(
    tmp_path, monkeypatch
) -> None:
    calibration = object.__new__(calibration2d3d_class)
    reports: list[tuple[str, str, Exception]] = []
    calibration._report_file_io_error = (
        lambda path, operation, error: reports.append((path, operation, error))
    )
    result_path = tmp_path / "result.csv"
    write_error = OSError("write failed")
    monkeypatch.setattr(np, "savetxt", MagicMock(side_effect=write_error))

    with pytest.raises(OSError, match="write failed"):
        calibration._write_result_matrix(str(result_path), np.eye(4))

    assert reports == [
        (
            str(result_path),
            "write 2D-3D calibration result matrix CSV",
            write_error,
        )
    ]


def test_async_pickle_write_error_is_reported_and_reraised(tmp_path) -> None:
    calibration = object.__new__(calibration2d3d_class)
    reports: list[tuple[str, str, Exception]] = []
    calibration._report_file_io_error = (
        lambda path, operation, error: reports.append((path, operation, error))
    )
    output_path = tmp_path / "missing" / "sensor.pickle"

    with pytest.raises(OSError):
        calibration._save_pickle_with_diagnosis(str(output_path), {"frame": 1})

    assert len(reports) == 1
    assert reports[0][:2] == (
        str(output_path),
        "write 2D-3D sensor data pickle",
    )
    assert isinstance(reports[0][2], OSError)


def test_calibration2d3d_ui_diagnosis_codes_default_without_invented_rules() -> None:
    assert Calib2d3dResultDiagnosis().diagnose() == Calib2d3dErrorCommon.DEFAULT
    assert (
        CameraCalibrationStatusDiagnosis().diagnose()
        == CameraCalibrationStatus.DEFAULT
    )


@pytest.mark.parametrize(
    ("file_contents", "expected_error"),
    (
        (None, OSError),
        ("not json", ValueError),
    ),
)
def test_calc_progress_reports_area_definition_read_errors(
    tmp_path, file_contents: str | None, expected_error: type[Exception]
) -> None:
    settings_path = tmp_path / "area.json"
    if file_contents is not None:
        settings_path.write_text(file_contents)

    reports: list[tuple[str, str, Exception]] = []
    progress = object.__new__(calc_progress_class)
    progress.calib2d3d_CalcProgress = SimpleNamespace(
        areadefinition_filepathes=[str(settings_path)]
    )
    progress.file_io_error_reporter = lambda path, operation, error: reports.append(
        (path, operation, error)
    )

    with pytest.raises(expected_error):
        progress.read_settings(0)

    assert len(reports) == 1
    assert reports[0][:2] == (
        str(settings_path),
        "read 2D-3D area definition JSON",
    )
    assert isinstance(reports[0][2], expected_error)