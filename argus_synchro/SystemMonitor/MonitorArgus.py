import json
import os
import signal
import subprocess
import sys
import tempfile
import time
from collections.abc import Callable
from pathlib import Path

from argus_synchro.common import paths
from argus_synchro.common.app_logger import AppLogger, AppLoggerFactory
from argus_synchro.diagnosis.error_config import ErrorConfig
from argus_synchro.diagnosis.state_d_errors import FileIoError
from argus_synchro.shared_errors import StateErrorDIndex
from argus_synchro.SystemMonitor.status_mmap import StatusCode, StatusMMAP

CONFIG_NAME = "MonitorArgus.json"

_logger: AppLogger = AppLoggerFactory.from_name("MonitorArgus")


def log_register(app_logger_factory: AppLoggerFactory) -> None:
    app_logger_factory.append_logger(_logger)


def setup_signal_handlers(
    status_obj: StatusMMAP,
    logger: AppLogger,
    name: str = "Process",
    godot_proc_getter: Callable[[], subprocess.Popen[bytes] | None] | None = None,
) -> None:
    def shutdown_handler(signum: int, frame: object | None) -> None:
        logger.info(f"[{name}] シャットダウン検知: signal={signum}")
        status_obj.write_status(StatusCode.SHUTDOWN)

        if godot_proc_getter is not None:
            proc: subprocess.Popen[bytes] | None = godot_proc_getter()
            if proc and proc.poll() is None:
                logger.info(f"[{name}] UI 停止 (シグナル)")
                proc.terminate()
                proc.wait()

        status_obj.close()
        sys.exit(0)

    signal.signal(signal.SIGINT, shutdown_handler)

    try:
        signal.signal(signal.SIGTERM, shutdown_handler)
    except AttributeError:
        pass

    try:
        signal.signal(signal.SIGBREAK, shutdown_handler)
    except AttributeError:
        pass


def _create_file_io_error_diagnosis(
    directory_config: paths.DirectoryConfig,
) -> FileIoError:
    diagnosis = FileIoError()
    error_config_path = paths.get_config_dir(directory_config, "error_config.json")
    error_config = ErrorConfig()
    try:
        error_config.load_from_json(error_config_path)
    except (OSError, ValueError, TypeError) as error:
        _logger.warning(
            "FILE_IO_ERROR settings could not be loaded; using default settings: "
            f"{type(error).__name__}: {error}"
        )
    diagnosis.update(error_config)
    return diagnosis


def _report_file_io_error(
    diagnosis: FileIoError,
    config_path: Path,
    error: Exception,
) -> None:
    result = diagnosis.errors_diagnosis(True)
    diagnosis.log_output(
        *result,
        StateErrorDIndex.FILE_IO_ERROR,
        str(config_path),
        "read MonitorArgus.json",
        f"{type(error).__name__}: {error}",
    )


def load_monitor_config(
    config_path: Path,
    file_io_error: FileIoError | None = None,
) -> dict[str, object]:
    if not config_path.exists():
        error = FileNotFoundError(f"設定ファイルが存在しません: {config_path}")
        if file_io_error is not None:
            _report_file_io_error(file_io_error, config_path, error)
        _logger.error(str(error))
        sys.exit(1)

    try:
        with config_path.open(encoding="utf-8") as f:
            config = json.load(f)
        if not isinstance(config, dict):
            raise TypeError("MonitorArgus.json root must be an object")
        if "engine" not in config:
            raise KeyError("MonitorArgus.json missing required key: engine")
        if "appimage" not in config:
            raise KeyError("MonitorArgus.json missing required key: appimage")
    except (OSError, UnicodeError, json.JSONDecodeError, TypeError, KeyError) as e:
        if file_io_error is not None:
            _report_file_io_error(file_io_error, config_path, e)
        _logger.error(f"設定ファイルの読み込みに失敗: {e}")
        sys.exit(1)

    return config


def build_ui_command(config: dict[str, object]) -> tuple[list[str], str]:
    ui_mode = os.environ.get("ARGUS_UI_MODE", "appimage").strip().lower()

    if ui_mode not in ("engine", "appimage"):
        _logger.error(f"不正な ARGUS_UI_MODE です: {ui_mode}")
        sys.exit(1)

    mode_config = config.get(ui_mode)
    if not isinstance(mode_config, dict):
        _logger.error(f"{ui_mode} 設定が不正です")
        sys.exit(1)

    base_dir = os.path.expanduser(str(mode_config["base_dir"]))
    godot_args = mode_config.get("godot_args", [])

    if not isinstance(godot_args, list):
        _logger.error(f"{ui_mode}.godot_args は配列で指定してください")
        sys.exit(1)

    if ui_mode == "engine":
        for key in ("godot_engine", "godot_project_dir"):
            if key not in mode_config:
                _logger.error(f"engine 設定に必要なキーがありません: {key}")
                sys.exit(1)

        godot_engine_path = os.path.expanduser(
            os.path.join(base_dir, str(mode_config["godot_engine"]))
        )
        godot_project_dir = os.path.expanduser(
            os.path.join(base_dir, str(mode_config["godot_project_dir"]))
        )

        if not os.path.isfile(godot_engine_path):
            _logger.error(f"Godot 実行ファイルが見つかりません: {godot_engine_path}")
            sys.exit(1)

        if not os.path.isdir(godot_project_dir):
            _logger.error(f"Godot プロジェクトパスが存在しません: {godot_project_dir}")
            sys.exit(1)

        cmd = [
            godot_engine_path,
            "--path",
            godot_project_dir,
            *[str(a) for a in godot_args],
        ]
        cwd = godot_project_dir

        _logger.info("UI起動モード: engine")
        _logger.info(f"engine base_dir: {base_dir}")
        return cmd, cwd

    for key in ("godot_appimage",):
        if key not in mode_config:
            _logger.error(f"appimage 設定に必要なキーがありません: {key}")
            sys.exit(1)

    godot_appimage_path = os.path.expanduser(
        os.path.join(base_dir, str(mode_config["godot_appimage"]))
    )

    if not os.path.isfile(godot_appimage_path):
        _logger.error(f"AppImage が見つかりません: {godot_appimage_path}")
        sys.exit(1)

    if not os.access(godot_appimage_path, os.X_OK):
        _logger.error(f"AppImage に実行権限がありません: {godot_appimage_path}")
        sys.exit(1)

    cmd = [godot_appimage_path, *[str(a) for a in godot_args]]
    cwd = os.path.dirname(godot_appimage_path)

    _logger.info("UI起動モード: appimage")
    _logger.info(f"appimage base_dir: {base_dir}")
    return cmd, cwd


def ensure_status_mmap_exists(status_mmap_path: Path) -> None:
    if status_mmap_path.exists():
        if not status_mmap_path.is_file():
            raise FileExistsError(f"Expected file but found directory: {status_mmap_path}")

        if status_mmap_path.stat().st_size != StatusMMAP.size:
            _logger.warning(
                f"status.mmap size is invalid. "
                f"path={status_mmap_path}, size={status_mmap_path.stat().st_size}"
            )
        return

    _logger.info(f"status.mmap が存在しないため作成: {status_mmap_path}")
    status_mmap_path.parent.mkdir(parents=True, exist_ok=True)
    status_mmap_path.write_bytes(b"\x00" * StatusMMAP.size)


def _write_heartbeat(heartbeat_file: Path) -> None:
    temp_name: str | None = None
    try:
        with tempfile.NamedTemporaryFile(
            "w", dir=heartbeat_file.parent, delete=False
        ) as temp_file:
            temp_file.write(f"{time.perf_counter()}")
            temp_name = temp_file.name
        os.replace(temp_name, heartbeat_file)
    except Exception:
        if temp_name is not None and os.path.exists(temp_name):
            os.remove(temp_name)
        raise


def monitor_and_manage_godot() -> None:
    directory_config: paths.DirectoryConfig = paths.parse_directory_config()

    status_mmap_path: Path = paths.get_mmap_dir(directory_config, "status.mmap")
    heartbeat_file: Path = paths.get_mmap_dir(
        directory_config, "monitor_argus_last_heartbeat"
    )
    ensure_status_mmap_exists(status_mmap_path)

    godot_proc: subprocess.Popen[bytes] | None = None

    status = StatusMMAP(
        _logger,
        create=False,
        directory_config=directory_config,
    )

    name = "StatusMonitor"

    config_path: Path = paths.get_config_dir(directory_config, CONFIG_NAME)
    file_io_error = _create_file_io_error_diagnosis(directory_config)
    config = load_monitor_config(config_path, file_io_error)

    ui_cmd, ui_cwd = build_ui_command(config)

    def get_proc() -> subprocess.Popen[bytes] | None:
        return godot_proc

    setup_signal_handlers(
        status_obj=status,
        logger=_logger,
        name=name,
        godot_proc_getter=get_proc,
    )

    while True:
        code = status.read_status()
        code_name = StatusMMAP.get_status_name(code)
        _logger.info(f"status = {code_name} ({code})")

        if code == StatusCode.RUNNING.value and StatusMMAP.is_recent():
            if godot_proc is None or godot_proc.poll() is not None:
                _logger.info(f"RUNNING status を検出し、UI を起動: {' '.join(ui_cmd)}")
                godot_proc = subprocess.Popen(ui_cmd, cwd=ui_cwd)

        elif (
            code in (StatusCode.SHUTDOWN.value, StatusCode.REBOOT.value)
            or not StatusMMAP.is_recent()
        ):
            if code == StatusCode.REBOOT.value:
                _logger.info("REBOOT status を検出.")
            elif code == StatusCode.SHUTDOWN.value:
                _logger.info("SHUTDOWN status を検出.")
            elif not StatusMMAP.is_recent():
                _logger.info("Status is not recent を検出.")

            if godot_proc and godot_proc.poll() is None:
                _logger.info("UI 停止.")
                godot_proc.terminate()
                godot_proc.wait()
                godot_proc = None

            if code == StatusCode.SHUTDOWN.value:
                time.sleep(0.5)
                break


        _write_heartbeat(heartbeat_file)

        time.sleep(1)

    status.close()


if __name__ == "__main__":
    monitor_and_manage_godot()
