"""Shared helpers for argus_synchro_lib Conan sidecar scripts."""

from __future__ import annotations

import os
import shutil
import subprocess
from pathlib import Path
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from collections.abc import Iterable

REPO_ROOT = Path(__file__).resolve().parent.parent
CONAN_DIR = REPO_ROOT / "conan"
RECIPES_DIR = CONAN_DIR / "recipes"
DEFAULT_BUILD_DIR = REPO_ROOT / "build" / "conan"
PRODUCT_ROOT = REPO_ROOT.parent
OPEN3D_CONAN_DIR = PRODUCT_ROOT / "3rdparty" / "open3d" / "conan"
OPEN3D_RECIPES_DIR = OPEN3D_CONAN_DIR / "recipes"
PROFILE = CONAN_DIR / "profiles" / "linux-gcc-release"
VENV_CONAN = CONAN_DIR / ".venv" / "bin" / "conan"
CMAKE_PRESET = "conan-release"
CMAKE_SOURCE_DIR = CONAN_DIR


def conan_env(overrides: dict[str, str] | None = None) -> dict[str, str]:
    env = os.environ.copy()
    if overrides:
        env.update(overrides)
    return env


def resolve_conan_executable() -> str:
    if VENV_CONAN.exists():
        return str(VENV_CONAN)
    conan = shutil.which("conan")
    if conan is None:
        msg = (
            "`conan` executable not found in PATH and "
            f"{VENV_CONAN} does not exist (install conan>=2.0 or "
            "create conan/.venv)."
        )
        raise RuntimeError(msg)
    return conan


def list_recipes(recipes_dir: Path) -> Iterable[Path]:
    if not recipes_dir.is_dir():
        return
    for entry in sorted(recipes_dir.iterdir()):
        if (entry / "conanfile.py").exists():
            yield entry


def run_command(
    cmd: list[str],
    *,
    env: dict[str, str] | None = None,
    cwd: Path | None = None,
    capture_output: bool = False,
    check: bool = True,
    env_script: Path | None = None,
) -> subprocess.CompletedProcess[str]:
    print(f"+ {' '.join(cmd)}", flush=True)
    if env_script is not None:
        quoted = " ".join(f"'{c}'" for c in cmd)
        cmd = ["bash", "-c", f". '{env_script}' && exec {quoted}"]
        env = None
    return subprocess.run(
        cmd,
        check=check,
        env=env,
        cwd=cwd,
        capture_output=capture_output,
        text=True,
    )


def export_recipes(conan: str, recipes_dir: Path) -> None:
    env = conan_env()
    for recipe_dir in list_recipes(recipes_dir):
        run_command([conan, "export", str(recipe_dir)], env=env)


def export_open3d(conan: str) -> None:
    if not OPEN3D_CONAN_DIR.is_dir():
        msg = f"Open3D Conan sidecar not found: {OPEN3D_CONAN_DIR}"
        raise RuntimeError(msg)
    export_recipes(conan, OPEN3D_RECIPES_DIR)
    run_command([conan, "export", str(OPEN3D_CONAN_DIR)], env=conan_env())
