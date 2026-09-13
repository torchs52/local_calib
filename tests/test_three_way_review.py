from __future__ import annotations

import subprocess
import sys
from pathlib import Path

SCRIPT_PATH = Path(__file__).parents[1] / "scripts" / "three_way_review.py"


def _run_git(repo: Path, *args: str) -> str:
    result = subprocess.run(
        ["git", "-C", str(repo), *args],
        check=True,
        capture_output=True,
        text=True,
    )
    return result.stdout


def _create_repo(repo: Path, files: dict[str, str]) -> str:
    repo.mkdir()
    _run_git(repo, "init", "--quiet")
    _run_git(repo, "config", "user.name", "Three Way Test")
    _run_git(repo, "config", "user.email", "three-way@example.invalid")
    for relative_path, content in files.items():
        target = repo / relative_path
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_text(content, encoding="utf-8")
    _run_git(repo, "add", ".")
    _run_git(repo, "commit", "--quiet", "-m", "fixture")
    return _run_git(repo, "rev-parse", "HEAD").strip()


def _run_viewer(
    *,
    vendor_repo: Path,
    shi_repo: Path,
    integration_repo: Path,
    relative_path: str,
    output: Path,
) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        [
            sys.executable,
            str(SCRIPT_PATH),
            "--vendor-repo",
            str(vendor_repo),
            "--vendor-ref",
            "HEAD",
            "--shi-repo",
            str(shi_repo),
            "--shi-ref",
            "HEAD",
            "--integration-repo",
            str(integration_repo),
            "--output",
            str(output),
            relative_path,
        ],
        check=False,
        capture_output=True,
        text=True,
    )


def test_renders_three_external_repositories(tmp_path: Path) -> None:
    relative_path = "ctrl/sample.py"
    vendor_repo = tmp_path / "vendor"
    shi_repo = tmp_path / "shi"
    integration_repo = tmp_path / "integration"
    vendor_revision = _create_repo(
        vendor_repo,
        {relative_path: "shared = 1\nvalue = 'vendor'\n"},
    )
    shi_revision = _create_repo(
        shi_repo,
        {relative_path: "shared = 1\nvalue = 'shi'\n"},
    )
    integration_revision = _create_repo(
        integration_repo,
        {relative_path: "shared = 1\nvalue = 'base'\n"},
    )
    (integration_repo / relative_path).write_text(
        "shared = 1\nvalue = '<integration>'\n",
        encoding="utf-8",
    )
    output = tmp_path / "review.html"

    result = _run_viewer(
        vendor_repo=vendor_repo,
        shi_repo=shi_repo,
        integration_repo=integration_repo,
        relative_path=relative_path,
        output=output,
    )

    assert result.returncode == 0, result.stderr
    assert result.stdout.strip() == str(output)
    document = output.read_text(encoding="utf-8")
    assert "<strong>Vendor</strong>" in document
    assert "<strong>SHI</strong>" in document
    assert "<strong>Integration</strong>" in document
    assert vendor_revision[:12] in document
    assert shi_revision[:12] in document
    assert integration_revision[:12] in document
    assert "value = &#x27;vendor&#x27;" in document
    assert "value = &#x27;shi&#x27;" in document
    assert "value = &#x27;&lt;integration&gt;&#x27;" in document
    assert 'class="line all-different"' in document
    assert "uncommitted changes included" in document


def test_renders_missing_integration_file_as_empty_column(tmp_path: Path) -> None:
    relative_path = "ctrl/shi_only.py"
    vendor_repo = tmp_path / "vendor"
    shi_repo = tmp_path / "shi"
    integration_repo = tmp_path / "integration"
    _create_repo(vendor_repo, {"README.md": "vendor\n"})
    _create_repo(shi_repo, {relative_path: "shi_only = True\n"})
    _create_repo(integration_repo, {"README.md": "integration\n"})
    output = tmp_path / "missing-integration.html"

    result = _run_viewer(
        vendor_repo=vendor_repo,
        shi_repo=shi_repo,
        integration_repo=integration_repo,
        relative_path=relative_path,
        output=output,
    )

    assert result.returncode == 0, result.stderr
    document = output.read_text(encoding="utf-8")
    assert "shi_only = True" in document
    assert "working tree (file missing)" in document
    assert 'class="line shi-only"' in document
