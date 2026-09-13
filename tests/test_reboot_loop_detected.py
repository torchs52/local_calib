import json
from types import SimpleNamespace
from unittest.mock import MagicMock

import pytest

from argus_synchro.__main__ import _detect_reboot_loop
from argus_synchro.diagnosis.action_errors import RebootLoopDetectedDiagnosis
from argus_synchro.diagnosis.error_config import RebootLoopDetectedParameters
from argus_synchro.shared_errors import ActionErrorIndex, StateErrorDIndex


def _make_diagnosis(**parameters: object) -> RebootLoopDetectedDiagnosis:
    diagnosis = RebootLoopDetectedDiagnosis()
    diagnosis.param = RebootLoopDetectedParameters(**parameters)
    diagnosis.is_enabled = diagnosis.param.is_enabled
    return diagnosis


@pytest.mark.parametrize(
    ("boot_times", "expected"),
    (
        ([1000.0, 900.0, 800.0, 700.0], False),
        ([1000.0, 850.0, 700.0, 550.0, 400.0], True),
        ([1000.0, 850.0, 700.0, 550.0, 399.9], False),
    ),
)
def test_reboot_loop_detects_five_boots_within_ten_minutes(
    boot_times: list[float], expected: bool
) -> None:
    diagnosis = _make_diagnosis()

    assert diagnosis.detect_error(boot_times) is expected


def test_reboot_loop_owns_warning_log() -> None:
    diagnosis = _make_diagnosis()
    diagnosis._logger = MagicMock()
    boot_times = [1000.0, 850.0, 700.0, 550.0, 400.0]

    diagnosis.log_output(
        True, False, ActionErrorIndex.REBOOT_LOOP_DETECTED, boot_times
    )

    diagnosis._logger.warning.assert_called_once_with(
        "CE012: 再起動ループを検出しました: "
        "boot_count=5, window_sec=600.0, observed_span_sec=600.0"
    )


def test_detect_reboot_loop_reads_uptime_state_and_counts_once(tmp_path) -> None:
    state_path = tmp_path / "uptime_state.json"
    state_path.write_text(
        json.dumps(
            {
                "last_boots": [
                    {"boot_time_iso": f"2026-09-04T00:0{minute}:00+00:00"}
                    for minute in range(5)
                ]
            }
        ),
        encoding="utf-8",
    )
    diagnosis = _make_diagnosis(uptime_state_path=str(state_path))
    diagnosis._logger = MagicMock()
    file_io_error = MagicMock()
    ser = SimpleNamespace(
        action_errors_A_C={ActionErrorIndex.REBOOT_LOOP_DETECTED: diagnosis},
        state_errors_D={StateErrorDIndex.FILE_IO_ERROR: file_io_error},
    )

    _detect_reboot_loop(ser)

    assert diagnosis.err_cnt.value == 1
    file_io_error.errors_diagnosis.assert_called_once_with(False)
    file_io_error.log_output.assert_not_called()


def test_detect_reboot_loop_reports_invalid_uptime_state_as_file_io_error(
    tmp_path,
) -> None:
    state_path = tmp_path / "uptime_state.json"
    state_path.write_text('{"last_boots": "invalid"}', encoding="utf-8")
    diagnosis = _make_diagnosis(uptime_state_path=str(state_path))
    file_io_error = MagicMock()
    file_io_error.errors_diagnosis.return_value = ("detection", "normal")
    ser = SimpleNamespace(
        action_errors_A_C={ActionErrorIndex.REBOOT_LOOP_DETECTED: diagnosis},
        state_errors_D={StateErrorDIndex.FILE_IO_ERROR: file_io_error},
    )

    _detect_reboot_loop(ser)

    assert diagnosis.err_cnt.value == 0
    file_io_error.errors_diagnosis.assert_called_once_with(True)
    file_io_error.log_output.assert_called_once_with(
        "detection",
        "normal",
        StateErrorDIndex.FILE_IO_ERROR,
        str(state_path),
        "read uptime state for CE012",
        "ValueError: last_boots must be a list",
    )