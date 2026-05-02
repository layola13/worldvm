#!/usr/bin/env python3
"""Verify Python heredoc snippets embedded in Markdown files."""

from __future__ import annotations

import argparse
import re
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
PYTHON_HEREDOC_RE = re.compile(r"python(?:3)?\s+-\s+<<'PY'\n(?P<body>.*?)\nPY", re.DOTALL)


def verify_markdown(path: Path) -> int:
    text = path.read_text(encoding="utf-8")
    count = 0
    for count, match in enumerate(PYTHON_HEREDOC_RE.finditer(text), start=1):
        body = match.group("body")
        compile(body, f"{path}:python-heredoc-{count}", "exec")
    return count


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("paths", nargs="*", default=["README.md"], help="Markdown files to verify")
    args = parser.parse_args()

    total = 0
    for raw_path in args.paths:
        path = Path(raw_path)
        if not path.is_absolute():
            path = ROOT / path
        total += verify_markdown(path)
    print(f"Markdown Python snippets verified ({total} snippets)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
