from __future__ import annotations

import struct
from collections.abc import Callable
from pathlib import Path

import numpy as np
from argus_synchro_lib.error_mmap_writer import ErrorMMapWriter
from argus_synchro_lib.octotree import NodeEntity, OctoTree
from argus_synchro_lib.ui_interface import UI_interface, UIIFConf

LOG_DEBUG = 10
LOG_INFO = 20
TEST_MACHINE_ANGLE = 42


def _ignore_log(_level: object, _message: str) -> None:
    pass


def _record_logs(logs: list[tuple[int, str]]) -> Callable[[int, str], None]:
    def record(level: int, message: str) -> None:
        logs.append((level, message))

    return record


def _read_octotree_count(mmap_path: Path) -> int:
    return struct.unpack_from("<i", mmap_path.read_bytes(), 10)[0]


def _create_ui(tmp_path: Path) -> tuple[UI_interface, list[Path]]:
    mmap_paths = [tmp_path / "map0.dat", tmp_path / "map1.dat"]
    status_mmap_path = tmp_path / "status.mmap"
    status_mmap_path.write_bytes(b"\x00" * 4)
    config = UIIFConf(
        False,
        20,
        8.0,
        [str(path) for path in mmap_paths],
        [],
        True,
        4.0,
        3.0,
        1.5,
        6.0,
        3.0,
        False,
        True,
    )
    return (
        UI_interface(
            config,
            1,
            4.2,
            0,
            False,
            0.0,
            str(status_mmap_path),
            _ignore_log,
        ),
        mmap_paths,
    )


def test_surround_mmap_legacy_error_field_is_always_zero(tmp_path: Path) -> None:
    ui, mmap_paths = _create_ui(tmp_path)

    try:
        with mmap_paths[0].open("r+b") as current_buffer:
            current_buffer.seek(10)
            current_buffer.write(struct.pack("<i", 0x7FFFFFFF))

        ui.preprocess_info()
        ui.error_info(2)
        ui.machine_info(TEST_MACHINE_ANGLE)

        mmap_data = mmap_paths[0].read_bytes()
        assert struct.unpack_from("<i", mmap_data, 10)[0] == 0
        assert struct.unpack_from("<f", mmap_data, 14)[0] == TEST_MACHINE_ANGLE
    finally:
        ui.close_mmap()


def test_empty_octotree_overwrites_previous_point_count(
    tmp_path: Path,
    octotree_obj: OctoTree,
) -> None:
    mmap_paths = [tmp_path / "map0.dat", tmp_path / "map1.dat"]
    status_mmap_path = tmp_path / "status.mmap"
    status_mmap_path.write_bytes(b"\x00" * 4)
    logs: list[tuple[int, str]] = []
    config = UIIFConf(
        False,
        20,
        8.0,
        [str(path) for path in mmap_paths],
        [],
        True,
        4.0,
        3.0,
        1.5,
        6.0,
        3.0,
        False,
        True,
    )
    ui = UI_interface(
        config,
        1,
        4.2,
        0,
        False,
        0.0,
        str(status_mmap_path),
        _record_logs(logs),
    )

    try:
        octotree_obj.insert_or_entity_octonodes(
            np.array([[0.05, 0.05, 0.02]]),
            NodeEntity.OTHER,
            entity_replace=True,
        )
        ui.preprocess_info()
        ui.octotree_info(octotree_obj)
        assert _read_octotree_count(mmap_paths[0]) == 1
        assert any(
            level == LOG_DEBUG and message == "member_points_num = 1"
            for level, message in logs
        )
        assert any(
            level == LOG_INFO and message == "octotree_pcd_num = 1"
            for level, message in logs
        )

        empty_octotree = OctoTree(
            max_xyz=octotree_obj.max_xyz,
            min_xyz=octotree_obj.min_xyz,
            max_tree_depth=octotree_obj.max_tree_depth,
            use_node_stats=True,
            quantile=None,
            origin_w2oct=np.array([0, 0, 0]),
        )
        ui.preprocess_info()
        ui.octotree_info(empty_octotree)

        assert _read_octotree_count(mmap_paths[0]) == 0
    finally:
        ui.close_mmap()


def test_mmap_rotation_immediately_reserves_next_buffer(tmp_path: Path) -> None:
    mmap_paths = [tmp_path / "err0.dat", tmp_path / "err1.dat"]
    writer = ErrorMMapWriter([str(path) for path in mmap_paths], _ignore_log)

    try:
        writer.init()
        assert mmap_paths[0].read_bytes()[0] == 0
        assert mmap_paths[1].read_bytes()[0] == 1

        with mmap_paths[1].open("r+b") as current_buffer:
            current_buffer.seek(1)
            current_buffer.write(b"\x01")

        writer.rotate_if_busy()

        assert mmap_paths[1].read_bytes()[0] == 0
        assert mmap_paths[0].read_bytes()[0] == 1
    finally:
        writer.close()
