#!/usr/bin/env python3
"""Smoke-test a WorldVM release archive after extracting it."""

from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
import tarfile
import tempfile
from pathlib import Path

TOOLS_DIR = Path(__file__).resolve().parent
if str(TOOLS_DIR) not in sys.path:
    sys.path.insert(0, str(TOOLS_DIR))

import verify_release_package  # type: ignore[import-not-found]


def safe_extract(archive_path: Path, destination: Path) -> Path:
    with tarfile.open(archive_path, "r:gz") as archive:
        members = archive.getmembers()
        roots = {member.name.split("/", 1)[0] for member in members if member.name}
        if len(roots) != 1:
            raise SystemExit("archive must contain exactly one top-level directory")
        root = next(iter(roots))
        destination_root = destination.resolve()
        for member in members:
            member_path = destination / member.name
            resolved = member_path.resolve()
            try:
                resolved.relative_to(destination_root)
            except ValueError:
                raise SystemExit(f"unsafe archive member path: {member.name}")
            if member.issym() or member.islnk():
                raise SystemExit(f"archive member links are not supported: {member.name}")
        archive.extractall(destination)
    return destination / root


def read_manifest(package_root: Path) -> dict[str, object]:
    return json.loads((package_root / "manifest.json").read_text(encoding="utf-8"))


def cli_path(package_root: Path, manifest: dict[str, object]) -> Path:
    system = manifest.get("system")
    if system == "windows":
        return package_root / "zig-out/bin/worldvm.exe"
    return package_root / "zig-out/bin/worldvm"


def run_command(command: list[str], cwd: Path, env: dict[str, str] | None = None) -> subprocess.CompletedProcess[str]:
    completed = subprocess.run(
        command,
        cwd=cwd,
        env=env,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        timeout=30,
        check=False,
    )
    if completed.returncode != 0:
        raise SystemExit(
            f"command failed ({completed.returncode}): {' '.join(command)}\n"
            f"stdout:\n{completed.stdout}\n"
            f"stderr:\n{completed.stderr}"
        )
    return completed


def smoke_archive(archive_path: Path, scenario: str, ticks: int, skip_cli: bool, skip_python: bool) -> None:
    verify_release_package.verify_archive(archive_path)
    with tempfile.TemporaryDirectory(prefix="worldvm-package-smoke-") as temp_dir:
        package_root = safe_extract(archive_path, Path(temp_dir))
        manifest = read_manifest(package_root)

        if not skip_cli:
            cli = cli_path(package_root, manifest)
            if not cli.exists():
                raise SystemExit(f"missing packaged CLI: {cli}")
            completed = run_command(
                [str(cli), "run", "--scenario", scenario, "--ticks", str(ticks)],
                cwd=package_root,
            )
            if "Done." not in completed.stdout:
                raise SystemExit("packaged CLI smoke did not report completion")

        if not skip_python:
            env = dict(os.environ)
            env["PYTHONDONTWRITEBYTECODE"] = "1"
            completed = run_command(
                [sys.executable, "examples/python_lifecycle.py"],
                cwd=package_root,
                env=env,
            )
            if '"instance_count": 1' not in completed.stdout:
                raise SystemExit("packaged Python lifecycle smoke did not report expected instance_count")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("archives", nargs="+", help="release .tar.gz archives to smoke-test")
    parser.add_argument("--scenario", default="apple_table", help="CLI scenario to run")
    parser.add_argument("--ticks", type=int, default=3, help="CLI ticks to run")
    parser.add_argument("--skip-cli", action="store_true", help="skip packaged CLI smoke")
    parser.add_argument("--skip-python", action="store_true", help="skip packaged Python FFI smoke")
    args = parser.parse_args()

    for archive in args.archives:
        smoke_archive(Path(archive), args.scenario, args.ticks, args.skip_cli, args.skip_python)
        print(f"smoked {archive}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
