#!/usr/bin/env python3
"""Generate CycloneDX SBOM and Sunshine HTML dependency report for argus_synchro_lib."""

from __future__ import annotations

import argparse
import subprocess
import sys
import tempfile
from pathlib import Path
from typing import TYPE_CHECKING

from conan_common import (
    CONAN_DIR,
    PROFILE,
    conan_env,
    export_open3d,
    export_recipes,
    resolve_conan_executable,
    run_command,
)

if TYPE_CHECKING:
    from collections.abc import Iterable

CYCLONEDX_DEPLOYER = "cyclone_1.6.py"
CYCLONEDX_SBOM = "sbom-cyclonedx-1.6.json"
SUNSHINE_HTML = "dependency_report.html"
SUNSHINE_SCRIPT = CONAN_DIR / "vendor" / "sunshine" / "sunshine.py"


def run_graph_info(
    conan: str,
    graph_file: Path,
    deploy_folder: Path,
) -> None:
    cmd = [
        conan,
        "graph",
        "info",
        str(CONAN_DIR),
        "-pr:h",
        str(PROFILE),
        "-pr:b",
        str(PROFILE),
        "--format=json",
        "--out-file",
        str(graph_file),
        "--deployer",
        CYCLONEDX_DEPLOYER,
        "-df",
        str(deploy_folder),
    ]
    run_command(cmd, env=conan_env())


def render_sunshine_html(
    sbom_file: Path,
    output_file: Path,
    *,
    enrich: bool = False,
) -> None:
    if not SUNSHINE_SCRIPT.is_file():
        msg = f"Sunshine CLI not found: {SUNSHINE_SCRIPT}"
        raise RuntimeError(msg)

    cmd = [
        sys.executable,
        str(SUNSHINE_SCRIPT),
        "-i",
        str(sbom_file),
        "-o",
        str(output_file),
        "-nl",
    ]
    if enrich:
        cmd.append("-e")

    try:
        subprocess.run(cmd, check=True)
    except subprocess.CalledProcessError as exc:
        raise RuntimeError(f"Sunshine failed (exit {exc.returncode})") from exc


def main(argv: Iterable[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=CONAN_DIR / "reports",
        help="Directory for CycloneDX SBOM and dependency_report.html.",
    )
    parser.add_argument(
        "--sunshine-enrich",
        action="store_true",
        help="Enrich Sunshine HTML with EPSS and CISA KEV (requires network).",
    )
    args = parser.parse_args(list(argv) if argv is not None else None)

    conan = resolve_conan_executable()
    args.output_dir.mkdir(parents=True, exist_ok=True)

    export_open3d(conan)
    export_recipes(conan, CONAN_DIR / "recipes")

    with tempfile.NamedTemporaryFile(suffix=".json", delete=False) as handle:
        graph_path = Path(handle.name)

    try:
        run_graph_info(conan, graph_path, args.output_dir)
    finally:
        graph_path.unlink(missing_ok=True)

    sbom_path = args.output_dir / CYCLONEDX_SBOM
    html_path = args.output_dir / SUNSHINE_HTML
    render_sunshine_html(sbom_path, html_path, enrich=args.sunshine_enrich)

    print(f"Wrote {CYCLONEDX_SBOM}, {SUNSHINE_HTML} under {args.output_dir}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
