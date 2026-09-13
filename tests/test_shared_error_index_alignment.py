from pathlib import Path

from argus_synchro.shared_errors import (
    ActionErrorIndex,
    ModuleErrorIndex,
    SharedErrors,
    StateErrorDIndex,
    StateErrorIndex,
)


def _first_reserved_index(enum_type: type[ActionErrorIndex] | type[StateErrorIndex]) -> int:
    return min(member.value for member in enum_type if member.name.startswith("RESERVED_"))


def test_error_indices_align_with_diagnosis_tuples() -> None:
    error_config_path = Path(__file__).parents[1] / "config" / "error_config.json"
    shared_errors = SharedErrors(error_config_path)
    try:
        assert len(shared_errors.state_errors_A_C) == _first_reserved_index(
            StateErrorIndex
        )
        assert len(shared_errors.action_errors_A_C) == _first_reserved_index(
            ActionErrorIndex
        )
        assert len(shared_errors.state_errors_D) == len(StateErrorDIndex)
        assert len(shared_errors.module_errors) == len(ModuleErrorIndex)
    finally:
        shared_errors.shared_err_conf.close()
