#!/usr/bin/env python3
"""Extract a release-notes section from CHANGELOG.md."""

from __future__ import annotations

import argparse
import re
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
CHANGELOG = ROOT / "CHANGELOG.md"
HEADING_RE = re.compile(r"^##\s+(.+?)\s*$", re.MULTILINE)
PLACEHOLDER_RE = re.compile(r"\b(TBD|TODO|placeholder|No commits found)\b", re.IGNORECASE)


def normalize_heading(value: str) -> str:
    value = value.strip()
    if value.startswith("[") and "]" in value:
        value = value[1 : value.index("]")]
    if " - " in value:
        value = value.split(" - ", 1)[0]
    return value.strip()


def extract_section(changelog: str, version: str) -> str:
    matches = list(HEADING_RE.finditer(changelog))
    if not matches:
        raise SystemExit("CHANGELOG.md has no level-2 release sections")

    wanted = version.removeprefix("v")
    fallback: tuple[int, int] | None = None
    for idx, match in enumerate(matches):
        heading = normalize_heading(match.group(1))
        section_start = match.end()
        section_end = matches[idx + 1].start() if idx + 1 < len(matches) else len(changelog)
        if heading == wanted or heading == f"v{wanted}":
            return changelog[section_start:section_end].strip() + "\n"
        if heading.lower() == "unreleased":
            fallback = (section_start, section_end)

    if fallback is None:
        raise SystemExit(f"CHANGELOG.md has no section for {version} and no Unreleased fallback")
    start, end = fallback
    return changelog[start:end].strip() + "\n"


def validate_notes(notes: str, require_bullet: bool, reject_placeholder: bool) -> None:
    if not notes.strip():
        raise SystemExit("release notes are empty")
    if require_bullet and not any(line.lstrip().startswith("- ") for line in notes.splitlines()):
        raise SystemExit("release notes must contain at least one bullet item")
    if reject_placeholder and PLACEHOLDER_RE.search(notes):
        raise SystemExit("release notes contain placeholder text")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--version", required=True, help="release version, with or without leading v")
    parser.add_argument("--output", help="optional output file")
    parser.add_argument("--require-bullet", action="store_true", help="fail unless notes contain a Markdown bullet")
    parser.add_argument("--reject-placeholder", action="store_true", help="fail on common placeholder text")
    args = parser.parse_args()

    changelog = CHANGELOG.read_text(encoding="utf-8")
    notes = extract_section(changelog, args.version)
    validate_notes(notes, args.require_bullet, args.reject_placeholder)
    if args.output:
        Path(args.output).write_text(notes, encoding="utf-8")
    else:
        sys.stdout.write(notes)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
