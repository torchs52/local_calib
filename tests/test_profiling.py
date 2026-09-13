from __future__ import annotations

from pathlib import Path

from argus_synchro import profiling
from argus_synchro.common.paths import DirectoryConfig


def test_build_argus_extra_args_forwards_log_and_mmap_dirs() -> None:
    directory_config = DirectoryConfig(
        config_dir=Path("/tmp/config"),
        log_dir=Path("/tmp/log"),
        mmap_dir=Path("/tmp/mmap"),
    )

    assert profiling.build_argus_extra_args(directory_config, ["--foo", "bar"]) == [
        "--log-dir",
        "/tmp/log",
        "--mmap-dir",
        "/tmp/mmap",
        "--foo",
        "bar",
    ]
