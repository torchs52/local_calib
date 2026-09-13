from pathlib import Path
from types import SimpleNamespace
from unittest.mock import MagicMock, call, patch

from argus_synchro.file_watch import ChangeModelEventHandler, DebouncedEventHandler
from argus_synchro.machine_profile import MachineProfileHandler
from argus_synchro.process.app_manager_process import AppManagerProcess
from argus_synchro.process.operation_mode import OPERATION_MODE as OPM
from argus_synchro.shared_errors import ActionErrorIndex


def _event_handler(write_side_effect: object) -> tuple[
    DebouncedEventHandler, MagicMock, MagicMock, MagicMock
]:
    shared_app_config = MagicMock()
    shared_app_config.write.side_effect = write_side_effect
    shared_app_config.read.return_value = SimpleNamespace(
        ReducedLoadMode=SimpleNamespace(
            many_points_ratio=0.4,
            few_points_ratio=0.3,
        )
    )
    diagnosis = MagicMock()
    reduced_load_mode = MagicMock()
    shared_errors = SimpleNamespace(
        action_errors_A_C={ActionErrorIndex.CONFIG_FILE_MISSING: diagnosis},
        reduced_load_mode=reduced_load_mode,
    )
    logger = MagicMock()
    handler = DebouncedEventHandler(
        sac=shared_app_config,
        sec=MagicMock(),
        ser=shared_errors,
        logger=logger,
    )
    return handler, shared_app_config, diagnosis, logger


def test_reload_retry_absorbs_transient_atomic_save_failure() -> None:
    handler, shared_app_config, diagnosis, logger = _event_handler(
        [FileNotFoundError("settings.ini"), OSError("writing"), None]
    )
    path = "config/settings.ini"
    handler.events[path] = MagicMock()

    with patch("argus_synchro.file_watch.time.sleep") as sleep:
        handler.process_event(path)

    assert shared_app_config.write.call_count == 3
    assert sleep.call_args_list == [call(0.2), call(0.2)]
    diagnosis.excepts_diagnosis.assert_not_called()
    shared_app_config.read.assert_not_called()
    handler._ser.reduced_load_mode.configure.assert_not_called()
    logger.error.assert_not_called()
    assert path not in handler.events


def test_reload_retry_reports_ce005_after_all_attempts_fail() -> None:
    handler, shared_app_config, diagnosis, logger = _event_handler(
        FileNotFoundError("settings.ini")
    )
    diagnosis.excepts_diagnosis.return_value = True

    with patch("argus_synchro.file_watch.time.sleep") as sleep:
        handler.process_event("config/settings.ini")

    assert shared_app_config.write.call_count == 3
    assert sleep.call_args_list == [call(0.2), call(0.2)]
    diagnosis.excepts_diagnosis.assert_called_once()
    logger.error.assert_called_once()
    assert logger.error.call_args.args[0].startswith("file open error:")


def test_model_change_handler_applies_calibration_config() -> None:
    shared_errors = SimpleNamespace(action_errors_A_C={})
    handler = ChangeModelEventHandler(
        ser=shared_errors,
        regexes=(r".*SCX900-3_calib_settings.ini",),
        debounce_time=0.1,
        app_logger_factory=MagicMock(),
        apply_calib=True,
    )
    handler.mprof_handler = MagicMock()

    handler.process_event("config/SCX900-3_calib_settings.ini")

    handler.mprof_handler.apply_model_specific_calib_config.assert_called_once_with()
    handler.mprof_handler.apply_model_specific_config.assert_not_called()


def test_calibration_mode_keeps_settings_watch_and_adds_calibration_watch() -> None:
    manager = SimpleNamespace(
        _sac=MagicMock(),
        _sec=MagicMock(),
        _ser=MagicMock(),
        _logger=MagicMock(),
        _directory_config=MagicMock(),
        _app_logger_factory=MagicMock(),
        _app_config=SimpleNamespace(
            General=SimpleNamespace(operation_mode=OPM.CALIB),
        ),
    )
    observer = MagicMock()
    settings_handler = MagicMock()
    model_handler = MagicMock()
    calib_model_handler = MagicMock()

    with (
        patch("watchdog.observers.Observer", return_value=observer),
        patch(
            "argus_synchro.file_watch.DebouncedEventHandler",
            return_value=settings_handler,
        ) as settings_handler_class,
        patch(
            "argus_synchro.file_watch.ChangeModelEventHandler",
            side_effect=[model_handler, calib_model_handler],
        ),
        patch(
            "argus_synchro.process.app_manager_process.paths.get_config_dir",
            return_value="config",
        ),
        patch.object(
            MachineProfileHandler,
            "get_model_specific_config_file_path",
            return_value=Path("config/SCX900-3_settings.ini"),
        ),
        patch.object(
            MachineProfileHandler,
            "get_model_specific_calib_config_file_path",
            return_value=Path("config/SCX900-3_calib_settings.ini"),
        ),
    ):
        AppManagerProcess._startup_observer(manager)

    assert settings_handler_class.call_args.kwargs["regexes"] == (
        r".*(\\|/)settings.ini",
    )
    assert observer.schedule.call_args_list == [
        call(settings_handler, "config"),
        call(model_handler, "config"),
        call(calib_model_handler, "config"),
    ]
    observer.start.assert_called_once_with()