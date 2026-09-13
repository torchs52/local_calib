from pathlib import Path
from unittest.mock import MagicMock

import pandas as pd
import pytest

from argus_synchro.common.app_logger import AppLoggerFactory
from argus_synchro.config.app_config import CANConf
from argus_synchro.device.can.can_decoders import (
    DECODER_REGISTRY,
    handle_angle_can_scx2000,
    handle_angle_newcan,
    handle_angle_newcan_inverted,
    handle_angle_oldcan,
    handle_lever,
)
from argus_synchro.device.can.can_receiver import (
    LEVER_PRESSURE_SIGNAL,
    YAW_ANGLE_SIGNAL,
    CanFile,
    CanHandler,
    CanIdMapError,
    DecodedCanMessage,
)
from argus_synchro.provider.can_data import CanFileProvider

CRANE_MODEL = "SCX900-3"
MAP_HEADER = "crane_model,can_id,signal_type,decoder\n"


def make_can_conf(can_id_map_file: Path, *, is_old: bool = True) -> CANConf:
    return CANConf(
        config_file="",
        IsOld=is_old,
        interpretation=1,
        yaw_offset_deg=10.0,
        c_file="",
        can_id_map_file=str(can_id_map_file),
    )


def test_old_can_adds_yaw_offset() -> None:
    result = handle_angle_oldcan("00000BB800000000", 10.0, MagicMock())

    assert result == pytest.approx((70.0,))


def test_new_can_subtracts_yaw_offset() -> None:
    result = handle_angle_newcan("0000000000000000", 10.0, MagicMock())

    assert result == pytest.approx((-10.0,))


def test_new_can_inverted_reverses_direction() -> None:
    result = handle_angle_newcan_inverted("6400000000000000", 10.0, MagicMock())

    assert result == pytest.approx((347.3134328358209,))


def test_lever_decodes_four_big_endian_pressures() -> None:
    result = handle_lever("03E807D00BB80FA000", 0.0, MagicMock())

    assert result == pytest.approx((1.0, 2.0, 3.0, 4.0))


def test_scx2000_decodes_last_two_bytes_and_inverts_direction() -> None:
    result = handle_angle_can_scx2000("000000000000D204", 0.0, MagicMock())

    assert result == pytest.approx((236.6,))


def test_decoder_registry_contains_supported_decoders() -> None:
    assert DECODER_REGISTRY == {
        "handle_angle_oldcan": handle_angle_oldcan,
        "handle_angle_newcan": handle_angle_newcan,
        "handle_angle_newcan_inverted": handle_angle_newcan_inverted,
        "handle_lever": handle_lever,
        "handle_angle_can_scx2000": handle_angle_can_scx2000,
    }


def test_handler_normalizes_can_id_and_dispatches_registered_decoder(
    tmp_path: Path,
) -> None:
    can_id_map_file = tmp_path / "can_id_map.csv"
    can_id_map_file.write_text(
        MAP_HEADER + "SCX900-3,0x18FCE402,yaw_angle,handle_angle_oldcan\n",
        encoding="utf-8",
    )
    handler = CanHandler(
        make_can_conf(can_id_map_file),
        CRANE_MODEL,
        AppLoggerFactory(to_console=False),
    )

    decoded = handler.dispatch(" 0x18fce402 ", "00000BB800000000")

    assert decoded is not None
    assert decoded.can_id == "18FCE402"
    assert decoded.signal_type == YAW_ANGLE_SIGNAL
    assert decoded.values == pytest.approx((70.0,))


def test_handler_rejects_unknown_decoder(tmp_path: Path) -> None:
    can_id_map_file = tmp_path / "can_id_map.csv"
    can_id_map_file.write_text(
        MAP_HEADER + "SCX900-3,0x18FCE402,yaw_angle,unknown_decoder\n",
        encoding="utf-8",
    )

    with pytest.raises(ValueError, match="Unknown CAN decoder function"):
        CanHandler(
            make_can_conf(can_id_map_file),
            CRANE_MODEL,
            AppLoggerFactory(to_console=False),
        )


def test_handler_error_records_can_id_map_path(tmp_path: Path) -> None:
    can_id_map_file = tmp_path / "missing_can_id_map.csv"

    with pytest.raises(CanIdMapError) as error_info:
        CanHandler(
            make_can_conf(can_id_map_file),
            CRANE_MODEL,
            AppLoggerFactory(to_console=False),
        )

    assert error_info.value.file_path == str(can_id_map_file)
    assert f"path={can_id_map_file}" in str(error_info.value)
    assert isinstance(error_info.value.__cause__, FileNotFoundError)


def test_file_input_dispatches_using_configured_pgn(tmp_path: Path) -> None:
    can_id_map_file = tmp_path / "can_id_map.csv"
    can_id_map_file.write_text(
        MAP_HEADER + "SCX900-3,0x18FCE402,yaw_angle,handle_angle_oldcan\n",
        encoding="utf-8",
    )
    can_conf = make_can_conf(can_id_map_file)
    can_file = CanFile.__new__(CanFile)
    can_file._yaw_offset_deg = can_conf.yaw_offset_deg
    can_file.is_old = True
    can_file.angle_data = pd.Series(["00000BB800000000"])
    can_file._logger = MagicMock()
    can_file._handler = CanHandler(
        can_conf, CRANE_MODEL, AppLoggerFactory(to_console=False)
    )

    decoded = can_file.receive_can_data(0)

    assert decoded is not None
    assert decoded.can_id == "18FCE402"
    assert decoded.signal_type == YAW_ANGLE_SIGNAL
    assert decoded.values == pytest.approx((70.0,))


def test_machine_map_selects_normal_or_inverted_decoder_for_same_id(
    tmp_path: Path,
) -> None:
    normal_map = tmp_path / "normal.csv"
    normal_map.write_text(
        MAP_HEADER + "SCX900-3,0x18FFD1D1,yaw_angle,handle_angle_newcan\n",
        encoding="utf-8",
    )
    inverted_map = tmp_path / "inverted.csv"
    inverted_map.write_text(
        MAP_HEADER
        + "SCX900-3,0x18FFD1D1,yaw_angle,handle_angle_newcan_inverted\n",
        encoding="utf-8",
    )

    normal = CanHandler(
        make_can_conf(normal_map), CRANE_MODEL, AppLoggerFactory(to_console=False)
    ).dispatch("18FFD1D1", "6400000000000000")
    inverted = CanHandler(
        make_can_conf(inverted_map),
        CRANE_MODEL,
        AppLoggerFactory(to_console=False),
    ).dispatch("18FFD1D1", "6400000000000000")

    assert normal is not None
    assert inverted is not None
    assert normal.values == pytest.approx((-7.313432835820896,))
    assert inverted.values == pytest.approx((347.3134328358209,))


def test_handler_rejects_duplicate_can_id(tmp_path: Path) -> None:
    can_id_map_file = tmp_path / "can_id_map.csv"
    can_id_map_file.write_text(
        MAP_HEADER
        + "SCX900-3,0x18FFD1D1,yaw_angle,handle_angle_newcan\n"
        "SCX900-3,0x18FFD1D1,lever_pressure,handle_lever\n",
        encoding="utf-8",
    )

    with pytest.raises(ValueError, match="Duplicate CAN ID"):
        CanHandler(
            make_can_conf(can_id_map_file),
            CRANE_MODEL,
            AppLoggerFactory(to_console=False),
        )


def test_handler_requires_exactly_one_yaw_angle_signal(tmp_path: Path) -> None:
    can_id_map_file = tmp_path / "can_id_map.csv"
    can_id_map_file.write_text(
        MAP_HEADER + "SCX900-3,0x18FC4401,lever_pressure,handle_lever\n",
        encoding="utf-8",
    )

    with pytest.raises(ValueError, match="exactly one yaw_angle"):
        CanHandler(
            make_can_conf(can_id_map_file),
            CRANE_MODEL,
            AppLoggerFactory(to_console=False),
        )


def test_handler_exposes_signal_types_from_map(tmp_path: Path) -> None:
    can_id_map_file = tmp_path / "can_id_map.csv"
    can_id_map_file.write_text(
        MAP_HEADER
        + "SCX900-3,0x18FFD1D1,yaw_angle,handle_angle_newcan\n"
        "*,0x18FC4401,lever_pressure,handle_lever\n",
        encoding="utf-8",
    )
    handler = CanHandler(
        make_can_conf(can_id_map_file),
        CRANE_MODEL,
        AppLoggerFactory(to_console=False),
    )

    assert handler.can_id_for(YAW_ANGLE_SIGNAL) == "18FFD1D1"
    assert handler.can_id_for(LEVER_PRESSURE_SIGNAL) == "18FC4401"


def test_handler_ignores_commented_candidate_entries(tmp_path: Path) -> None:
    can_id_map_file = tmp_path / "can_id_map.csv"
    can_id_map_file.write_text(
        MAP_HEADER
        + "# SCX900-3,0x18FCE402,yaw_angle,handle_angle_oldcan\n"
        "SCX900-3,0x18FFD1D1,yaw_angle,handle_angle_newcan\n",
        encoding="utf-8",
    )
    handler = CanHandler(
        make_can_conf(can_id_map_file),
        CRANE_MODEL,
        AppLoggerFactory(to_console=False),
    )

    assert handler.can_id_for(YAW_ANGLE_SIGNAL) == "18FFD1D1"
    assert handler.dispatch("18FCE402", "00000BB800000000") is None


def test_model_specific_entry_overrides_common_entry_for_same_can_id(
    tmp_path: Path,
) -> None:
    can_id_map_file = tmp_path / "can_id_map.csv"
    can_id_map_file.write_text(
        MAP_HEADER
        + "*,0x18FFD1D1,yaw_angle,handle_angle_newcan\n"
        "SCX900-3,0x18FFD1D1,yaw_angle,handle_angle_newcan_inverted\n",
        encoding="utf-8",
    )
    handler = CanHandler(
        make_can_conf(can_id_map_file),
        CRANE_MODEL,
        AppLoggerFactory(to_console=False),
    )

    decoded = handler.dispatch("18FFD1D1", "6400000000000000")

    assert decoded is not None
    assert decoded.values == pytest.approx((347.3134328358209,))


def test_model_specific_signal_overrides_common_signal_with_different_id(
    tmp_path: Path,
) -> None:
    can_id_map_file = tmp_path / "can_id_map.csv"
    can_id_map_file.write_text(
        MAP_HEADER
        + "*,0x18FFD1D1,yaw_angle,handle_angle_newcan\n"
        "SCX900-3,0x18FCE402,yaw_angle,handle_angle_oldcan\n",
        encoding="utf-8",
    )
    handler = CanHandler(
        make_can_conf(can_id_map_file),
        CRANE_MODEL,
        AppLoggerFactory(to_console=False),
    )

    assert handler.dispatch("18FFD1D1", "0000000000000000") is None
    assert handler.can_id_for(YAW_ANGLE_SIGNAL) == "18FCE402"


def test_handler_rejects_unknown_signal_type(tmp_path: Path) -> None:
    can_id_map_file = tmp_path / "can_id_map.csv"
    can_id_map_file.write_text(
        MAP_HEADER + "SCX900-3,0x18FFD1D1,unknown,handle_angle_newcan\n",
        encoding="utf-8",
    )

    with pytest.raises(ValueError, match="Unknown CAN signal type"):
        CanHandler(
            make_can_conf(can_id_map_file),
            CRANE_MODEL,
            AppLoggerFactory(to_console=False),
        )


def test_handler_rejects_map_without_entries_for_model(tmp_path: Path) -> None:
    can_id_map_file = tmp_path / "can_id_map.csv"
    can_id_map_file.write_text(
        MAP_HEADER
        + "SCX2000-3,0x18F0E211,yaw_angle,handle_angle_can_scx2000\n",
        encoding="utf-8",
    )

    with pytest.raises(ValueError, match="SCX900-3"):
        CanHandler(
            make_can_conf(can_id_map_file),
            CRANE_MODEL,
            AppLoggerFactory(to_console=False),
        )


def test_provider_updates_values_by_signal_type_instead_of_can_id() -> None:
    device = MagicMock()
    device.receive_can_data.side_effect = (
        DecodedCanMessage("UNLISTED1", YAW_ANGLE_SIGNAL, (12.5,)),
        DecodedCanMessage(
            "UNLISTED2", LEVER_PRESSURE_SIGNAL, (1.0, 2.0, 3.0, 4.0)
        ),
    )
    provider = CanFileProvider(device, 0)

    yaw_angle, _ = provider.receive_can_data()
    _, lever_pressure = provider.receive_can_data()

    assert yaw_angle == pytest.approx(12.5)
    assert lever_pressure == pytest.approx((1.0, 2.0, 3.0, 4.0))