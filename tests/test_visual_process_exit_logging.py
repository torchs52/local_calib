from types import SimpleNamespace
from unittest.mock import MagicMock

from argus_synchro.process.process import ProcessBase
from argus_synchro.process.synchronizer import ProcessActivator
from argus_synchro.process.visual_process import VisualProcess


def test_visual_loop_logs_activators_and_keeps_vendor_exit_handling() -> None:
    process = object.__new__(VisualProcess)
    process_activator = ProcessActivator()
    process_activator.disable()
    ProcessBase.__init__(process, MagicMock(), process_activator, "VisualProcess")

    process._logger = MagicMock()
    process.sec = SimpleNamespace(
        Scruti_ex=SimpleNamespace(IsFinished=SimpleNamespace(value=False))
    )
    process._accum_points_input = SimpleNamespace(activator=ProcessActivator())
    process._cliff_inputs = SimpleNamespace(activator=ProcessActivator())
    process._bouding_box_data = SimpleNamespace(activator=ProcessActivator())

    process._loop()

    process._logger.warning.assert_called_once_with(
        "VisualProcess loop ended: process_activator=%s restart_required=%s "
        "flow_activators=%s",
        False,
        False,
        [True, True, True],
    )
    assert process.sec.Scruti_ex.IsFinished.value is True
    process._logger.info.assert_called_once_with("終了条件に到達.")