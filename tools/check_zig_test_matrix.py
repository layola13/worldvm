#!/usr/bin/env python3
"""Verify every src/*.zig file is listed exactly once in build.zig's test matrix."""

from __future__ import annotations

import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
BUILD_FILE = ROOT / "build.zig"
SRC_DIR = ROOT / "src"
ENTRY_RE = re.compile(r'"(src/[^"]+\.zig)"')


def read_test_entries() -> list[str]:
    entries: list[str] = []
    inside = False
    for line in BUILD_FILE.read_text(encoding="utf-8").splitlines():
        if "const test_files" in line:
            inside = True
            continue
        if inside and line.strip() == "};":
            break
        if inside:
            match = ENTRY_RE.search(line)
            if match:
                entries.append(match.group(1))
    return entries


def main() -> int:
    entries = read_test_entries()
    src_files = sorted(path.relative_to(ROOT).as_posix() for path in SRC_DIR.glob("*.zig"))

    missing = [path for path in src_files if path not in entries]
    extra = [path for path in entries if not (ROOT / path).exists()]
    duplicates = sorted({path for path in entries if entries.count(path) > 1})

    if missing or extra or duplicates:
        print("Zig test matrix check failed", file=sys.stderr)
        if missing:
            print("Missing src/*.zig entries:", file=sys.stderr)
            for path in missing:
                print(f"  - {path}", file=sys.stderr)
        if extra:
            print("Entries that do not exist:", file=sys.stderr)
            for path in extra:
                print(f"  - {path}", file=sys.stderr)
        if duplicates:
            print("Duplicate entries:", file=sys.stderr)
            for path in duplicates:
                print(f"  - {path}", file=sys.stderr)
        return 1

    print(f"Zig test matrix covers {len(entries)}/{len(src_files)} src/*.zig files")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
