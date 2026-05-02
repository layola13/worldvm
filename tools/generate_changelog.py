#!/usr/bin/env python3
"""Generate a Markdown changelog section from git history."""

from __future__ import annotations

import argparse
import re
import subprocess
import sys
from datetime import date
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
CHANGELOG = ROOT / "CHANGELOG.md"
VERSION_RE = re.compile(r"^\d+\.\d+\.\d+(?:[-+][0-9A-Za-z.-]+)?$")


def run_git(args: list[str]) -> str:
    return subprocess.check_output(["git", *args], cwd=ROOT, text=True).strip()


def latest_tag() -> str | None:
    completed = subprocess.run(
        ["git", "describe", "--tags", "--abbrev=0"],
        cwd=ROOT,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.DEVNULL,
        check=False,
    )
    if completed.returncode != 0:
        return None
    tag = completed.stdout.strip()
    return tag or None


def commit_subjects(revision_range: str | None, max_count: int) -> list[str]:
    args = ["log", "--pretty=format:%s", f"--max-count={max_count}"]
    if revision_range:
        args.insert(1, revision_range)
    output = run_git(args)
    return [line for line in output.splitlines() if line]


def render_section(version: str, subjects: list[str]) -> str:
    lines = [f"## {version} - {date.today().isoformat()}", ""]
    if subjects:
        lines.append("### Changes")
        lines.extend(f"- {subject}" for subject in subjects)
    else:
        lines.append("### Changes")
        lines.append("- No commits found for this range.")
    lines.append("")
    return "\n".join(lines)


def prepend_changelog(section: str) -> None:
    if CHANGELOG.exists():
        existing = CHANGELOG.read_text(encoding="utf-8")
    else:
        existing = "# Changelog\n\n"

    if not existing.startswith("# Changelog\n"):
        raise SystemExit("CHANGELOG.md must start with '# Changelog'")

    header = "# Changelog\n\n"
    body = existing[len(header) :]
    CHANGELOG.write_text(header + section + "\n" + body, encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--version", required=True, help="SemVer release version without leading v")
    parser.add_argument("--since", help="start git ref; defaults to latest tag when available")
    parser.add_argument("--max-count", type=int, default=200, help="maximum commits to include")
    parser.add_argument("--write", action="store_true", help="prepend the generated section to CHANGELOG.md")
    args = parser.parse_args()

    if not VERSION_RE.match(args.version):
        raise SystemExit("--version must be SemVer-like, for example 0.1.0")
    if args.max_count <= 0:
        raise SystemExit("--max-count must be > 0")

    since = args.since if args.since is not None else latest_tag()
    revision_range = f"{since}..HEAD" if since else None
    subjects = commit_subjects(revision_range, args.max_count)
    section = render_section(args.version, subjects)

    if args.write:
        prepend_changelog(section)
    else:
        sys.stdout.write(section)
        sys.stdout.write("\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
