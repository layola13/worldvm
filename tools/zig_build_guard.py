#!/usr/bin/env python3
"""Run Zig commands after pruning local build caches that exceed a size limit."""

from __future__ import annotations

import argparse
import os
import shutil
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_CACHE_DIRS = (".zig-cache", "zig-cache")
DEFAULT_LIMIT_BYTES = 3 * 1024 * 1024 * 1024


def dir_size(path: Path) -> int:
    if not path.exists():
        return 0
    if path.is_file() or path.is_symlink():
        try:
            return path.stat().st_size
        except FileNotFoundError:
            return 0

    total = 0
    for dirpath, dirnames, filenames in os.walk(path):
        for dirname in list(dirnames):
            child = Path(dirpath) / dirname
            if child.is_symlink():
                try:
                    total += child.stat().st_size
                except FileNotFoundError:
                    pass
                dirnames.remove(dirname)
        for filename in filenames:
            child = Path(dirpath) / filename
            try:
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


def remove_path(path: Path) -> None:
    if path.is_dir() and not path.is_symlink():
        shutil.rmtree(path)
    elif path.exists() or path.is_symlink():
        path.unlink()


def guard_caches(limit: int, dry_run: bool = False) -> int:
    removed = 0
    for name in DEFAULT_CACHE_DIRS:
        path = ROOT / name
        size = dir_size(path)
        if size <= limit:
            print(f"zig cache guard: keep {name} ({format_size(size)} <= {format_size(limit)})")
            continue
        action = "would remove" if dry_run else "removing"
        print(f"zig cache guard: {action} {name} ({format_size(size)} > {format_size(limit)})")
        removed += size
        if not dry_run:
            remove_path(path)
    return removed


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--limit", default="3GiB", help="cache size limit before deletion, default: 3GiB")
    parser.add_argument("--dry-run", action="store_true", help="only report cache actions and do not run command")
    parser.add_argument("cmd", nargs=argparse.REMAINDER, help="command to run after cache guard, e.g. -- zig build test-fast")
    args = parser.parse_args()

    limit = parse_size(args.limit)
    removed = guard_caches(limit, args.dry_run)
    if removed:
        summary = "would free" if args.dry_run else "freed"
        print(f"zig cache guard: {summary} {format_size(removed)}")

    cmd = args.cmd
    if cmd and cmd[0] == "--":
        cmd = cmd[1:]
    if args.dry_run or not cmd:
        return 0

    return subprocess.call(cmd, cwd=ROOT)


if __name__ == "__main__":
    raise SystemExit(main())
