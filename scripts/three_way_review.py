#!/usr/bin/env python3
"""Render vendor, SHI, and integration versions of a file side by side."""

from __future__ import annotations

import argparse
import difflib
import html
import subprocess
import sys
from dataclasses import dataclass
from pathlib import Path, PurePosixPath


@dataclass(frozen=True)
class SourceVersion:
    label: str
    source: str
    revision: str
    lines: list[str]


def run_git(repo: Path, *args: str) -> str:
    result = subprocess.run(
        ["git", "-C", str(repo), *args],
        check=False,
        capture_output=True,
        text=True,
    )
    if result.returncode != 0:
        message = result.stderr.strip() or result.stdout.strip()
        raise RuntimeError(f"git {' '.join(args)} failed in {repo}: {message}")
    return result.stdout


def resolve_repo(path: Path) -> Path:
    return Path(run_git(path, "rev-parse", "--show-toplevel").strip())


def read_ref_version(
    *,
    label: str,
    repo: Path,
    ref: str,
    file_path: PurePosixPath,
) -> SourceVersion:
    revision = run_git(repo, "rev-parse", ref).strip()
    tracked_path = run_git(
        repo,
        "ls-tree",
        "--name-only",
        ref,
        "--",
        file_path.as_posix(),
    ).strip()
    lines: list[str] = []
    if tracked_path:
        content = run_git(repo, "show", f"{ref}:{file_path.as_posix()}")
        lines = content.splitlines()
    return SourceVersion(
        label=label,
        source=f"{repo} @ {ref}",
        revision=revision,
        lines=lines,
    )


def read_worktree_version(
    *,
    repo: Path,
    file_path: PurePosixPath,
) -> SourceVersion:
    worktree_path = repo.joinpath(*file_path.parts)
    revision = run_git(repo, "rev-parse", "HEAD").strip()
    if not worktree_path.exists():
        return SourceVersion(
            label="Integration",
            source=f"{repo} working tree (file missing)",
            revision=revision,
            lines=[],
        )
    if not worktree_path.is_file():
        raise RuntimeError(f"integration working-tree path is not a file: {file_path}")
    dirty = run_git(repo, "status", "--short", "--", file_path.as_posix()).strip()
    detail = "working tree (uncommitted changes included)" if dirty else "working tree"
    return SourceVersion(
        label="Integration",
        source=f"{repo} {detail}",
        revision=revision,
        lines=worktree_path.read_text(encoding="utf-8").splitlines(),
    )


AlignedCell = tuple[int | None, str, str]


def align_side_to_base(
    base: list[str], other: list[str]
) -> tuple[list[AlignedCell | None], dict[int, list[AlignedCell]], set[int]]:
    mapped: list[AlignedCell | None] = [None] * len(base)
    insertions: dict[int, list[AlignedCell]] = {}
    base_changed: set[int] = set()
    matcher = difflib.SequenceMatcher(None, base, other, autojunk=False)
    for tag, base_start, base_end, other_start, other_end in matcher.get_opcodes():
        base_count = base_end - base_start
        other_count = other_end - other_start
        paired_count = min(base_count, other_count)
        status = "equal" if tag == "equal" else "changed"

        for offset in range(paired_count):
            other_index = other_start + offset
            mapped[base_start + offset] = (
                other_index + 1,
                other[other_index],
                status,
            )

        if tag != "equal":
            base_changed.update(range(base_start, base_end))

        if other_count > paired_count:
            insertion_point = base_start if base_count == 0 else base_end
            insertions.setdefault(insertion_point, []).extend(
                (other_index + 1, other[other_index], "changed")
                for other_index in range(other_start + paired_count, other_end)
            )

    return mapped, insertions, base_changed


def align_insertions(
    left: list[AlignedCell], right: list[AlignedCell]
) -> list[tuple[AlignedCell | None, AlignedCell | None]]:
    aligned: list[tuple[AlignedCell | None, AlignedCell | None]] = []
    matcher = difflib.SequenceMatcher(
        None,
        [cell[1] for cell in left],
        [cell[1] for cell in right],
        autojunk=False,
    )
    for left_start, left_end, right_start, right_end in (
        (left_start, left_end, right_start, right_end)
        for _, left_start, left_end, right_start, right_end in matcher.get_opcodes()
    ):
        left_block = left[left_start:left_end]
        right_block = right[right_start:right_end]
        block_size = max(len(left_block), len(right_block))
        for offset in range(block_size):
            aligned.append(
                (
                    left_block[offset] if offset < len(left_block) else None,
                    right_block[offset] if offset < len(right_block) else None,
                )
            )
    return aligned


def align_versions(
    vendor: list[str], shi: list[str], integration: list[str]
) -> tuple[list[AlignedCell], list[AlignedCell], list[AlignedCell]]:
    vendor_map, vendor_insertions, vendor_changed = align_side_to_base(
        integration, vendor
    )
    shi_map, shi_insertions, shi_changed = align_side_to_base(integration, shi)
    changed_integration = vendor_changed | shi_changed
    placeholder: AlignedCell = (None, "", "placeholder")
    aligned_vendor: list[AlignedCell] = []
    aligned_shi: list[AlignedCell] = []
    aligned_integration: list[AlignedCell] = []

    for integration_index in range(len(integration) + 1):
        vendor_gap = vendor_insertions.get(integration_index, [])
        shi_gap = shi_insertions.get(integration_index, [])
        for vendor_cell, shi_cell in align_insertions(vendor_gap, shi_gap):
            aligned_vendor.append(vendor_cell or placeholder)
            aligned_shi.append(shi_cell or placeholder)
            aligned_integration.append(placeholder)

        if integration_index == len(integration):
            continue

        integration_status = (
            "changed" if integration_index in changed_integration else "equal"
        )
        aligned_vendor.append(vendor_map[integration_index] or placeholder)
        aligned_shi.append(shi_map[integration_index] or placeholder)
        aligned_integration.append(
            (integration_index + 1, integration[integration_index], integration_status)
        )

    assert len(aligned_vendor) == len(aligned_shi) == len(aligned_integration)
    return aligned_vendor, aligned_shi, aligned_integration


def classify_rows(
    vendor: list[AlignedCell],
    shi: list[AlignedCell],
    integration: list[AlignedCell],
) -> tuple[list[AlignedCell], list[AlignedCell], list[AlignedCell]]:
    classified_vendor: list[AlignedCell] = []
    classified_shi: list[AlignedCell] = []
    classified_integration: list[AlignedCell] = []
    for vendor_cell, shi_cell, integration_cell in zip(
        vendor, shi, integration, strict=True
    ):
        vendor_value = (vendor_cell[0] is not None, vendor_cell[1])
        shi_value = (shi_cell[0] is not None, shi_cell[1])
        integration_value = (integration_cell[0] is not None, integration_cell[1])
        if vendor_value == shi_value == integration_value:
            classes = (vendor_cell[2], shi_cell[2], integration_cell[2])
        elif shi_value == integration_value:
            classes = ("vendor-only", shi_cell[2], integration_cell[2])
        elif vendor_value == integration_value:
            classes = (vendor_cell[2], "shi-only", integration_cell[2])
        elif vendor_value == shi_value:
            classes = (vendor_cell[2], shi_cell[2], "integration-only")
        else:
            classes = ("all-different", "all-different", "all-different")

        classified_vendor.append((vendor_cell[0], vendor_cell[1], classes[0]))
        classified_shi.append((shi_cell[0], shi_cell[1], classes[1]))
        classified_integration.append(
            (integration_cell[0], integration_cell[1], classes[2])
        )
    return classified_vendor, classified_shi, classified_integration


def render_lines(lines: list[AlignedCell]) -> str:
    if not lines:
        return '<div class="empty">File is empty</div>'

    rendered: list[str] = []
    for display_row, (line_number, line, status) in enumerate(lines):
        number = "" if line_number is None else str(line_number)
        rendered.append(
            f'<div class="line {status}" data-row="{display_row}">'
            f'<span class="number">{number}</span>'
            f'<span class="code">{html.escape(line) or " "}</span>'
            "</div>"
        )
    return "\n".join(rendered)


def render_document(
    file_path: PurePosixPath,
    vendor: SourceVersion,
    shi: SourceVersion,
    integration: SourceVersion,
) -> str:
    aligned_vendor, aligned_shi, aligned_integration = align_versions(
        vendor.lines, shi.lines, integration.lines
    )
    aligned_vendor, aligned_shi, aligned_integration = classify_rows(
        aligned_vendor, aligned_shi, aligned_integration
    )
    title = html.escape(file_path.as_posix())
    columns = (
        (vendor, aligned_vendor),
        (shi, aligned_shi),
        (integration, aligned_integration),
    )
    panes: list[str] = []
    for source_version, lines in columns:
        panes.append(
            '<section class="pane">'
            '<header class="pane-header">'
            f"<strong>{html.escape(source_version.label)}</strong>"
            f'<span class="source">{html.escape(source_version.source)}</span>'
            f'<span class="revision">{html.escape(source_version.revision[:12])}</span>'
            "</header>"
            f'<div class="code-view">{render_lines(lines)}</div>'
            "</section>"
        )

    return f"""<!doctype html>
<html lang="ja">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Three-way review: {title}</title>
<style>
:root {{ color-scheme: light; --font-size: 13px; }}
* {{ box-sizing: border-box; }}
html, body {{ height: 100%; margin: 0; }}
body {{ background: #f4f5f7; color: #1f2328; font-family: sans-serif; }}
.toolbar {{
  align-items: center; background: #fff; border-bottom: 1px solid #c9cdd2;
  display: flex; gap: 16px; height: 48px; padding: 0 14px;
}}
.toolbar h1 {{ font-size: 14px; margin: 0 auto 0 0; overflow: hidden; text-overflow: ellipsis; white-space: nowrap; }}
.toolbar label {{ align-items: center; display: flex; font-size: 12px; gap: 6px; white-space: nowrap; }}
.toolbar button {{ background: #fff; border: 1px solid #aeb4bb; padding: 4px 9px; }}
.toolbar button:disabled {{ color: #8c959f; }}
.difference-nav {{ align-items: center; display: flex; gap: 4px; }}
.difference-position {{ font-family: monospace; font-size: 11px; min-width: 48px; text-align: center; }}
.legend {{ align-items: center; display: flex; font-size: 11px; gap: 4px; white-space: nowrap; }}
.legend i {{ border: 1px solid #aeb4bb; display: inline-block; height: 11px; width: 11px; }}
.legend i.vendor-only {{ background: #ffb6b1; }}
.legend i.shi-only {{ background: #a9d6ff; }}
.legend i.integration-only {{ background: #b8e8bf; }}
.legend i.all-different {{ background: #ffd66b; }}
.review {{ display: grid; grid-template-columns: repeat(3, minmax(0, 1fr)); height: calc(100% - 48px); }}
.pane {{ border-right: 1px solid #c9cdd2; display: grid; grid-template-rows: 43px minmax(0, 1fr); min-height: 0; min-width: 0; }}
.pane:last-child {{ border-right: 0; }}
.pane-header {{
  align-items: baseline; background: #e9ebee; border-bottom: 1px solid #c9cdd2;
  display: flex; gap: 8px; overflow: hidden; padding: 6px 10px;
}}
.pane-header strong {{ font-size: 13px; }}
.source, .revision {{ color: #59636e; font-family: monospace; font-size: 11px; overflow: hidden; text-overflow: ellipsis; white-space: nowrap; }}
.revision {{ margin-left: auto; }}
.code-view {{ background: #fff; font-family: monospace; font-size: var(--font-size); line-height: 1.45; overflow: auto; }}
.line {{ display: grid; grid-template-columns: 54px max-content; min-height: 1.45em; }}
.line.placeholder {{ background: #eef0f2; }}
.line.placeholder .number {{ background: #e4e7ea; }}
.line.vendor-only {{ background: #ffb6b1; }}
.line.shi-only {{ background: #a9d6ff; }}
.line.integration-only {{ background: #b8e8bf; }}
.line.all-different {{ background: #ffd66b; }}
.number {{
  background: #f1f2f4; border-right: 1px solid #d8dadd; color: #737b84;
  padding: 0 8px; text-align: right; user-select: none;
}}
.code {{ padding: 0 8px; white-space: pre; }}
body.wrap .line {{ grid-template-columns: 54px minmax(0, 1fr); }}
body.wrap .code {{ overflow-wrap: anywhere; white-space: pre-wrap; }}
.empty {{ color: #737b84; padding: 20px; text-align: center; }}
@media (max-width: 900px) {{ .review {{ min-width: 900px; }} body {{ overflow-x: auto; }} }}
</style>
</head>
<body>
<div class="toolbar">
  <h1>{title}</h1>
  <span class="legend"><i class="vendor-only"></i>Vendorのみ</span>
  <span class="legend"><i class="shi-only"></i>SHIのみ</span>
  <span class="legend"><i class="integration-only"></i>Integrationのみ</span>
  <span class="legend"><i class="all-different"></i>全て異なる</span>
  <span class="difference-nav">
    <button id="previous-difference" type="button" title="前の差分へ移動">↑</button>
    <button id="next-difference" type="button" title="次の差分へ移動">↓</button>
    <span id="difference-position" class="difference-position">0 / 0</span>
  </span>
  <label><input id="sync" type="checkbox" checked>同期スクロール</label>
  <label><input id="wrap" type="checkbox">折り返す</label>
  <button id="smaller" type="button" title="文字を小さくする">A-</button>
  <button id="larger" type="button" title="文字を大きくする">A+</button>
</div>
<main class="review">{"".join(panes)}</main>
<script>
const views = [...document.querySelectorAll('.code-view')];
const sync = document.getElementById('sync');
const differenceClasses = ['vendor-only', 'shi-only', 'integration-only', 'all-different'];
const changedRows = [...new Set(
  views.flatMap(view => [...view.querySelectorAll('.line')]
    .filter(line => differenceClasses.some(name => line.classList.contains(name)))
    .map(line => Number(line.dataset.row)))
)].sort((left, right) => left - right);
const differenceRows = changedRows.filter(
  (row, index) => index === 0 || row > changedRows[index - 1] + 1
);
const differencePosition = document.getElementById('difference-position');
const previousDifference = document.getElementById('previous-difference');
const nextDifference = document.getElementById('next-difference');
let currentDifference = -1;
let scrolling = false;

function updateDifferencePosition() {{
  const position = currentDifference < 0 ? 0 : currentDifference + 1;
  differencePosition.textContent = `${{position}} / ${{differenceRows.length}}`;
  previousDifference.disabled = differenceRows.length === 0;
  nextDifference.disabled = differenceRows.length === 0;
}}
function moveToDifference(direction) {{
  if (!differenceRows.length) return;
  currentDifference = (currentDifference + direction + differenceRows.length) % differenceRows.length;
  const row = views[0].querySelector(`[data-row="${{differenceRows[currentDifference]}}"]`);
  if (row) views[0].scrollTo({{ top: row.offsetTop }});
  updateDifferencePosition();
}}
previousDifference.addEventListener('click', () => moveToDifference(-1));
nextDifference.addEventListener('click', () => moveToDifference(1));
document.addEventListener('keydown', event => {{
  if (event.altKey && event.key === 'ArrowUp') moveToDifference(-1);
  if (event.altKey && event.key === 'ArrowDown') moveToDifference(1);
}});
updateDifferencePosition();

for (const source of views) {{
  source.addEventListener('scroll', () => {{
    if (!sync.checked || scrolling) return;
    scrolling = true;
    const xRatio = source.scrollLeft / Math.max(1, source.scrollWidth - source.clientWidth);
    for (const target of views) {{
      if (target === source) continue;
      target.scrollTop = source.scrollTop;
      target.scrollLeft = xRatio * Math.max(0, target.scrollWidth - target.clientWidth);
    }}
    requestAnimationFrame(() => {{ scrolling = false; }});
  }});
}}
document.getElementById('wrap').addEventListener('change', event => {{
  document.body.classList.toggle('wrap', event.target.checked);
  requestAnimationFrame(alignRowHeights);
}});
function alignRowHeights() {{
  const rowsByView = views.map(view => [...view.querySelectorAll('.line')]);
  if (!rowsByView.length) return;
  for (const rows of rowsByView) for (const row of rows) row.style.height = '';
  for (let index = 0; index < rowsByView[0].length; index++) {{
    const height = Math.max(...rowsByView.map(rows => rows[index].offsetHeight));
    for (const rows of rowsByView) rows[index].style.height = `${{height}}px`;
  }}
}}
let fontSize = 13;
document.getElementById('smaller').addEventListener('click', () => {{
  fontSize = Math.max(9, fontSize - 1);
  document.documentElement.style.setProperty('--font-size', `${{fontSize}}px`);
  requestAnimationFrame(alignRowHeights);
}});
document.getElementById('larger').addEventListener('click', () => {{
  fontSize = Math.min(24, fontSize + 1);
  document.documentElement.style.setProperty('--font-size', `${{fontSize}}px`);
  requestAnimationFrame(alignRowHeights);
}});
window.addEventListener('load', alignRowHeights);
</script>
</body>
</html>
"""


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("file", type=PurePosixPath, help="Repository-relative file")
    parser.add_argument(
        "--vendor-repo",
        type=Path,
        help="Vendor repository (defaults to integration repository)",
    )
    parser.add_argument("--vendor-ref", default="HEAD", help="Vendor Git ref")
    parser.add_argument("--shi-repo", type=Path, required=True, help="SHI repository")
    parser.add_argument("--shi-ref", default="HEAD", help="SHI Git ref")
    parser.add_argument(
        "--integration-repo",
        type=Path,
        default=Path.cwd(),
        help="Integration repository containing the working-tree file",
    )
    parser.add_argument("--output", type=Path, help="Output HTML path")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    try:
        file_path: PurePosixPath = args.file
        integration_repo = resolve_repo(args.integration_repo)
        vendor_repo = resolve_repo(args.vendor_repo or integration_repo)
        shi_repo = resolve_repo(args.shi_repo)
        vendor = read_ref_version(
            label="Vendor",
            repo=vendor_repo,
            ref=args.vendor_ref,
            file_path=file_path,
        )
        shi = read_ref_version(
            label="SHI",
            repo=shi_repo,
            ref=args.shi_ref,
            file_path=file_path,
        )
        integration = read_worktree_version(
            repo=integration_repo,
            file_path=file_path,
        )

        output = args.output or (
            integration_repo
            / ".merge_review"
            / "three-way"
            / f"{file_path.as_posix().replace('/', '__')}.html"
        )
        if not output.is_absolute():
            output = integration_repo / output
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(
            render_document(file_path, vendor, shi, integration),
            encoding="utf-8",
        )
        print(output)
        return 0
    except (OSError, RuntimeError, UnicodeError) as error:
        print(f"error: {error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
