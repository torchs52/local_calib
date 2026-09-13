from types import SimpleNamespace
from unittest.mock import MagicMock, patch

import pytest

from argus_synchro import __main__ as app_main
from argus_synchro.core.closable import CompositeClosable


@pytest.mark.parametrize(
    ("file_input", "mode", "expected"),
    (
        (True, "CALIB", True),
        (False, "CALIB", False),
        (True, "SCRUT", False),
        (False, "SCRUT", False),
    ),
)
def test_automated_calibration_run_requires_calib_file_input(
    file_input: bool, mode: str, expected: bool
) -> None:
    app_config = SimpleNamespace(DEFAULT=SimpleNamespace(File_Input=file_input))

    assert app_main.is_automated_calibration_run(app_config, mode) is expected


def test_automated_calibration_stop_only_disables_and_closes() -> None:
    closables = MagicMock()
    current_activator = MagicMock()
    new_activator = MagicMock()

    with patch.object(
        app_main, "ProcessActivator", return_value=new_activator
    ) as activator_factory:
        new_closables, returned_activator = (
            app_main.stop_automated_calibration_pipeline(
                closables, current_activator
            )
        )

    current_activator.disable.assert_called_once_with()
    closables.close.assert_called_once_with()
    activator_factory.assert_called_once_with()
    new_activator.disable.assert_called_once_with()
    assert isinstance(new_closables, CompositeClosable)
    assert returned_activator is new_activator


def test_completed_file_input_calibration_stops_before_join() -> None:
    app_config = SimpleNamespace(DEFAULT=SimpleNamespace(File_Input=True))
    closables = MagicMock()
    current_activator = MagicMock()
    stopped_closables = MagicMock()
    stopped_activator = MagicMock()

    with patch.object(
        app_main,
        "stop_automated_calibration_pipeline",
        return_value=(stopped_closables, stopped_activator),
    ) as stop_pipeline:
        result = app_main.stop_completed_calibration_pipeline(
            app_config,
            closables,
            current_activator,
        )

    stop_pipeline.assert_called_once_with(closables, current_activator)
    assert result == (stopped_closables, stopped_activator)


def test_completed_real_sensor_calibration_keeps_production_shutdown() -> None:
    app_config = SimpleNamespace(DEFAULT=SimpleNamespace(File_Input=False))
    closables = MagicMock()
    current_activator = MagicMock()

    with patch.object(
        app_main,
        "stop_automated_calibration_pipeline",
    ) as stop_pipeline:
        result = app_main.stop_completed_calibration_pipeline(
            app_config,
            closables,
            current_activator,
        )

    stop_pipeline.assert_not_called()
    assert result == (closables, current_activator)


def test_automated_calibration_completion_uses_bounded_shutdown() -> None:
    processes = MagicMock()
    system_processes = MagicMock()

    app_main.wait_for_pipeline_shutdown(
        processes,
        system_processes,
        automated_calibration_completed=True,
    )

    processes.graceful_stop_all.assert_called_once_with(
        t_grace=10.0,
        t_term=2.0,
        t_kill=1.0,
    )
    system_processes.graceful_stop_all.assert_called_once_with(
        t_grace=3.0,
        t_term=2.0,
        t_kill=1.0,
    )
    processes.join.assert_not_called()
    system_processes.join.assert_not_called()


def test_non_automated_completion_keeps_existing_join() -> None:
    processes = MagicMock()
    system_processes = MagicMock()

    app_main.wait_for_pipeline_shutdown(
        processes,
        system_processes,
        automated_calibration_completed=False,
    )

    processes.join.assert_called_once_with()
    system_processes.join.assert_called_once_with()
    processes.graceful_stop_all.assert_not_called()
    system_processes.graceful_stop_all.assert_not_called()
