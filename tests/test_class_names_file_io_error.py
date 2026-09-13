from types import SimpleNamespace
from unittest.mock import MagicMock

import pytest

from argus_synchro import cam_monitor
from argus_synchro.diagnosis.error_diagnosis import ResultDiagnosis
from argus_synchro.process.visual_process import VisualProcess
from argus_synchro.shared_errors import StateErrorDIndex


def test_class_name_read_error_is_reported_and_reraised(tmp_path) -> None:
    class_file = tmp_path / "missing.names"
    reports: list[tuple[str, Exception]] = []

    with pytest.raises(OSError):
        cam_monitor.Monitoring(
            show_cam=[],
            detect2d_conf=SimpleNamespace(yolo_class=str(class_file)),
            app_logger_factory=MagicMock(),
            file_io_error_callback=lambda path, error: reports.append((path, error)),
        )

    assert len(reports) == 1
    assert reports[0][0] == str(class_file)
    assert isinstance(reports[0][1], OSError)


def test_successful_class_name_read_updates_recovery_state(tmp_path) -> None:
    class_file = tmp_path / "classes.names"
    class_file.write_text("person\nvehicle\n")
    recover = MagicMock()

    monitor = cam_monitor.Monitoring(
        show_cam=[],
        detect2d_conf=SimpleNamespace(yolo_class=str(class_file)),
        app_logger_factory=MagicMock(),
        file_io_recovery_callback=recover,
    )

    assert monitor.classes == {0: "person", 1: "vehicle"}
    recover.assert_called_once_with()


def test_visual_reports_class_name_file_io_error_details() -> None:
    diagnosis = MagicMock()
    diagnosis.errors_diagnosis.return_value = (
        ResultDiagnosis.DETECTION,
        ResultDiagnosis.NORMAL,
    )
    process = object.__new__(VisualProcess)
    process._ser = SimpleNamespace(
        state_errors_D={StateErrorDIndex.FILE_IO_ERROR: diagnosis}
    )
    error = UnicodeError("invalid class name encoding")

    process._report_file_io_error("classes.names", error)

    diagnosis.errors_diagnosis.assert_called_once_with(True)
    diagnosis.log_output.assert_called_once_with(
        ResultDiagnosis.DETECTION,
        ResultDiagnosis.NORMAL,
        StateErrorDIndex.FILE_IO_ERROR,
        "classes.names",
        "read class name text file",
        "UnicodeError: invalid class name encoding",
    )


def test_visual_clears_class_name_file_io_error_after_success() -> None:
    diagnosis = MagicMock()
    process = object.__new__(VisualProcess)
    process._ser = SimpleNamespace(
        state_errors_D={StateErrorDIndex.FILE_IO_ERROR: diagnosis}
    )

    process._recover_file_io_error()

    diagnosis.errors_diagnosis.assert_called_once_with(False)
    diagnosis.log_output.assert_not_called()