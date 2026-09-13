#!/usr/bin/env python3
"""End-to-end build entry point for the argus_synchro_lib Conan sidecar.

1. ``conan export`` for Open3D local recipes + ``open3d``
2. ``conan export`` for argus local recipes (if any)
3. ``conan install`` with profile ``linux-gcc-release`` (open3d options in ``conanfile.py``)
4. ``cmake --preset conan-release`` configure/build with ``-S conan``
"""

from __future__ import annotations

import os
import sys
from pathlib import Path

from conan_common import (
    CMAKE_PRESET,
    CMAKE_SOURCE_DIR,
    CONAN_DIR,
    DEFAULT_BUILD_DIR,
    PROFILE,
    REPO_ROOT,
    conan_env,
    export_open3d,
    export_recipes,
    resolve_conan_executable,
    run_command,
)

JOBS = os.cpu_count() or 4


def _cmake(build_dir: Path) -> None:
    env_script = build_dir / "conanbuild.sh"
    run_command(
        [
            "cmake",
            "--preset",
            CMAKE_PRESET,
            "--fresh",
            "-S",
            str(CMAKE_SOURCE_DIR),
        ],
        env_script=env_script,
        cwd=REPO_ROOT,
    )
    run_command(
        [
            "cmake",
            "--build",
            "--preset",
            CMAKE_PRESET,
            "--parallel",
            str(JOBS),
        ],
        env_script=env_script,
        cwd=REPO_ROOT,
    )


def main() -> int:
    conan = resolve_conan_executable()
    build_dir = DEFAULT_BUILD_DIR

    export_open3d(conan)
    export_recipes(conan, CONAN_DIR / "recipes")

    install_cmd = [
        conan,
        "install",
        str(CONAN_DIR),
        "--output-folder",
        str(build_dir),
        "-pr:h",
        str(PROFILE),
        "-pr:b",
        str(PROFILE),
        "--build=missing",
    ]
    run_command(install_cmd, env=conan_env())
    _cmake(build_dir)
    return 0


if __name__ == "__main__":
    sys.exit(main())
