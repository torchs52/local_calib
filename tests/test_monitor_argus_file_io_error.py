from pathlib import Path

import pytest

from argus_synchro.diagnosis.error_config import ErrorConfig
from argus_synchro.diagnosis.error_diagnosis import ResultDiagnosis
from argus_synchro.diagnosis.state_d_errors import FileIoError
from argus_synchro.shared_errors import StateErrorDIndex
from argus_synchro.SystemMonitor.MonitorArgus import load_monitor_config


def test_load_monitor_config_reports_missing_file_io_error(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    config_path = tmp_path / "MonitorArgus.json"
    diagnosis = FileIoError()
    diagnosis.update(ErrorConfig())
    logged: list[tuple[object, ...]] = []
    monkeypatch.setattr(diagnosis, "log_output", lambda *args: logged.append(args))

    with pytest.raises(SystemExit) as exc_info:
        load_monitor_config(config_path, diagnosis)

    assert exc_info.value.code == 1
    assert logged == [
        (
            ResultDiagnosis.DETECTION,
            ResultDiagnosis.NORMAL,
            StateErrorDIndex.FILE_IO_ERROR,
            str(config_path),
            "read MonitorArgus.json",
            f"FileNotFoundError: 設定ファイルが存在しません: {config_path}",
        )
    ]


@pytest.mark.parametrize(
    ("content", "error_name"),
    [
        ("{", "JSONDecodeError"),
        ("[]", "TypeError"),
        ('{"engine": {}}', "KeyError"),
    ],
)
def test_load_monitor_config_reports_invalid_file_io_error(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    content: str,
    error_name: str,
) -> None:
    config_path = tmp_path / "MonitorArgus.json"
    config_path.write_text(content, encoding="utf-8")
    diagnosis = FileIoError()
    diagnosis.update(ErrorConfig())
    logged: list[tuple[object, ...]] = []
    monkeypatch.setattr(diagnosis, "log_output", lambda *args: logged.append(args))

    with pytest.raises(SystemExit) as exc_info:
        load_monitor_config(config_path, diagnosis)

    assert exc_info.value.code == 1
    assert logged[0][:5] == (
        ResultDiagnosis.DETECTION,
        ResultDiagnosis.NORMAL,
        StateErrorDIndex.FILE_IO_ERROR,
        str(config_path),
        "read MonitorArgus.json",
    )
    assert str(logged[0][5]).startswith(f"{error_name}:")


def test_load_monitor_config_returns_valid_config(tmp_path: Path) -> None:
    config_path = tmp_path / "MonitorArgus.json"
    config_path.write_text('{"engine": {}, "appimage": {}}', encoding="utf-8")

    assert load_monitor_config(config_path) == {"engine": {}, "appimage": {}}