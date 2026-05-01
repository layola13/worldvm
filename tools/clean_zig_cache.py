#!/usr/bin/env python3
"""Safely remove local Zig build/cache outputs for this repository."""

from __future__ import annotations

import argparse
import shutil
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_TARGETS = (".zig-cache", "zig-cache")
OPTIONAL_TARGETS = ("zig-out",)
DEFAULT_LIMIT_BYTES = 3 * 1024 * 1024 * 1024


def dir_size(path: Path) -> int:
    if not path.exists():
        return 0
    if path.is_file():
        return path.stat().st_size
    total = 0
    for child in path.rglob("*"):
        try:
            if child.is_file() or child.is_symlink():
                total += child.stat().st_size
        except FileNotFoundError:
            pass
    return total


def format_size(size: int) -> str:
    units = ("B", "KiB", "MiB", "GiB", "TiB")
    value = float(size)
    for unit in units:
        if value < 1024.0 or unit == units[-1]:
            return f"{value:.1f} {unit}"
        value /= 1024.0
    return f"{size} B"


def parse_size(value: str) -> int:
    text = value.strip().lower()
    multipliers = {
        "b": 1,
        "k": 1024,
        "kb": 1024,
        "kib": 1024,
        "m": 1024**2,
        "mb": 1024**2,
        "mib": 1024**2,
        "g": 1024**3,
        "gb": 1024**3,
        "gib": 1024**3,
    }
    for suffix, multiplier in sorted(multipliers.items(), key=lambda item: len(item[0]), reverse=True):
        if text.endswith(suffix):
            return int(float(text[: -len(suffix)].strip()) * multiplier)
    return int(float(text))


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dry-run", action="store_true", help="print what would be removed without deleting anything")
    parser.add_argument("--include-zig-out", action="store_true", help="also remove zig-out build artifacts")
    parser.add_argument("--if-over", metavar="SIZE", help="only remove cache dirs larger than SIZE, e.g. 3GiB")
    args = parser.parse_args()

    targets = list(DEFAULT_TARGETS)
    if args.include_zig_out:
        targets.extend(OPTIONAL_TARGETS)

    total = 0
    for name in targets:
        path = ROOT / name
        size = dir_size(path)
        if not path.exists():
            print(f"skip missing {name}")
            continue
        if args.if_over is not None:
            limit = parse_size(args.if_over)
            if size <= limit:
                print(f"keep {name} ({format_size(size)} <= {format_size(limit)})")
                continue
        total += size
        action = "would remove" if args.dry_run else "removing"
        print(f"{action} {name} ({format_size(size)})")
        if not args.dry_run:
            if path.is_dir() and not path.is_symlink():
                shutil.rmtree(path)
            else:
                path.unlink()

    summary = "would free" if args.dry_run else "freed"
    print(f"{summary} {format_size(total)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
