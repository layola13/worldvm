#!/usr/bin/env python3
"""Build a minimal WorldVM release archive."""

from __future__ import annotations

import argparse
import hashlib
import json
import platform
import shutil
import subprocess
import tarfile
import uuid
from datetime import datetime, timezone
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
DIST_DIR = ROOT / "dist"
COMMON_PACKAGE_FILES = (
    "CHANGELOG.md",
    "README.md",
    "benchmarks/ci_benchmark_baseline.json",
    "docs/development_plan.md",
    "docs/entity16_abi_v1.json",
    "docs/ffi_symbols_v1.json",
    "examples/python_lifecycle.py",
    "tests/fixtures/entity16_abi_v1_default.bin",
    "tests/physics/test_acceptance_scenario.py",
    "tests/test_package_release.py",
    "tests/test_release_verifier.py",
    "tests/test_smoke_release_package.py",
    "tests/test_worldvm_wrapper.py",
    "tools/benchmark_scenarios.py",
    "tools/extract_release_notes.py",
    "tools/generate_changelog.py",
    "tools/generate_entity16_fixture.py",
    "tools/package_release.py",
    "tools/smoke_release_package.py",
    "tools/verify_entity16_abi.py",
    "tools/verify_ffi_manifest.py",
    "tools/verify_python_wrapper_api.py",
    "tools/verify_readme_snippets.py",
    "tools/verify_release_package.py",
    "worldvm.py",
)


def run(command: list[str]) -> None:
    subprocess.run(command, cwd=ROOT, check=True)


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as file:
        for chunk in iter(lambda: file.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def current_system() -> str:
    system = platform.system().lower()
    if system == "darwin":
        return "macos"
    if system in {"linux", "windows"}:
        return system
    raise RuntimeError(f"unsupported release platform: {platform.system()}")


def normalize_machine(machine: str | None = None) -> str:
    if machine is None:
        machine = platform.machine()
    normalized = machine.strip().lower().replace(" ", "_").replace("-", "_")
    if normalized in {"amd64", "x64", "x86_64"}:
        return "x86_64"
    if normalized in {"aarch64", "arm64"}:
        return "arm64"
    if normalized in {"i386", "i686", "x86"}:
        return "x86"
    if not normalized:
        raise RuntimeError("cannot determine target machine")
    return normalized


def target_label(system: str, machine: str | None = None) -> str:
    return f"{system}-{normalize_machine(machine)}"


def platform_package_files(system: str) -> tuple[str, ...]:
    if system == "windows":
        return (
            "zig-out/bin/worldvm.exe",
            "zig-out/bin/worldvm.dll",
        )
    if system == "macos":
        return (
            "zig-out/bin/worldvm",
            "zig-out/lib/libworldvm.dylib",
        )
    if system == "linux":
        return (
            "zig-out/bin/worldvm",
            "zig-out/lib/libworldvm.so",
        )
    raise RuntimeError(f"unsupported release platform: {system}")


def copy_payload(package_dir: Path, package_files: tuple[str, ...]) -> list[dict[str, object]]:
    manifest_files: list[dict[str, object]] = []
    for relative in package_files:
        src = ROOT / relative
        if not src.exists():
            raise FileNotFoundError(f"missing package input: {relative}")
        dst = package_dir / relative
        dst.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(src, dst)
        manifest_files.append(
            {
                "path": relative,
                "size": dst.stat().st_size,
                "sha256": sha256(dst),
            }
        )
    return manifest_files


def make_archive(package_dir: Path, archive_path: Path) -> None:
    with tarfile.open(archive_path, "w:gz") as archive:
        archive.add(package_dir, arcname=package_dir.name)


def write_archive_checksum(archive_path: Path) -> Path:
    checksum_path = archive_path.with_suffix(archive_path.suffix + ".sha256")
    checksum_path.write_text(f"{sha256(archive_path)}  {archive_path.name}\n", encoding="utf-8")
    return checksum_path


def spdx_id(path: str) -> str:
    safe = "".join(char if char.isalnum() else "-" for char in path)
    return f"SPDXRef-File-{safe}"


def write_sbom(package_dir: Path, package_name: str, version: str, files: list[dict[str, object]], created_at: str) -> Path:
    document_id = "SPDXRef-DOCUMENT"
    package_spdx_id = "SPDXRef-Package-worldvm"
    file_entries = []
    relationships = [
        {
            "spdxElementId": document_id,
            "relationshipType": "DESCRIBES",
            "relatedSpdxElement": package_spdx_id,
        }
    ]
    for file_info in files:
        path = str(file_info["path"])
        file_spdx_id = spdx_id(path)
        file_entries.append(
            {
                "SPDXID": file_spdx_id,
                "fileName": path,
                "checksums": [
                    {
                        "algorithm": "SHA256",
                        "checksumValue": str(file_info["sha256"]),
                    }
                ],
            }
        )
        relationships.append(
            {
                "spdxElementId": package_spdx_id,
                "relationshipType": "CONTAINS",
                "relatedSpdxElement": file_spdx_id,
            }
        )

    sbom = {
        "spdxVersion": "SPDX-2.3",
        "dataLicense": "CC0-1.0",
        "SPDXID": document_id,
        "name": package_name,
        "documentNamespace": f"https://worldvm.invalid/sbom/{package_name}/{uuid.uuid5(uuid.NAMESPACE_DNS, package_name)}",
        "creationInfo": {
            "created": created_at,
            "creators": ["Tool: worldvm-package-release"],
        },
        "packages": [
            {
                "name": "worldvm",
                "SPDXID": package_spdx_id,
                "versionInfo": version,
                "downloadLocation": "NOASSERTION",
                "filesAnalyzed": True,
                "licenseConcluded": "NOASSERTION",
                "licenseDeclared": "NOASSERTION",
                "copyrightText": "NOASSERTION",
            }
        ],
        "files": file_entries,
        "relationships": relationships,
    }
    sbom_path = package_dir / "sbom.spdx.json"
    sbom_path.write_text(json.dumps(sbom, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return sbom_path


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--name", default="worldvm", help="package name prefix")
    parser.add_argument("--version", default="dev", help="package version label")
    parser.add_argument("--optimize", default="ReleaseSafe", help="Zig optimization mode")
    parser.add_argument("--skip-build", action="store_true", help="package existing zig-out artifacts")
    args = parser.parse_args()

    if not args.skip_build:
        run(["zig", "build", f"-Doptimize={args.optimize}"])

    system = current_system()
    package_target = target_label(system)
    package_name = f"{args.name}-{args.version}-{package_target}"
    package_dir = DIST_DIR / package_name
    archive_path = DIST_DIR / f"{package_name}.tar.gz"

    if package_dir.exists():
        shutil.rmtree(package_dir)
    if archive_path.exists():
        archive_path.unlink()
    DIST_DIR.mkdir(exist_ok=True)
    package_dir.mkdir()

    package_files = COMMON_PACKAGE_FILES + platform_package_files(system)
    files = copy_payload(package_dir, package_files)
    created_at = datetime.now(timezone.utc).isoformat()
    sbom_path = write_sbom(package_dir, package_name, args.version, files, created_at)
    files.append(
        {
            "path": sbom_path.relative_to(package_dir).as_posix(),
            "size": sbom_path.stat().st_size,
            "sha256": sha256(sbom_path),
        }
    )

    manifest = {
        "name": args.name,
        "version": args.version,
        "system": system,
        "target": package_target,
        "optimize": args.optimize,
        "created_at": created_at,
        "sbom": "sbom.spdx.json",
        "files": files,
    }
    manifest_path = package_dir / "manifest.json"
    manifest_path.write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8")

    make_archive(package_dir, archive_path)
    checksum_path = write_archive_checksum(archive_path)
    print(archive_path.relative_to(ROOT))
    print(checksum_path.relative_to(ROOT))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
