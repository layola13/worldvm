#!/usr/bin/env python3
"""Verify a WorldVM release archive, checksum sidecar, manifest, and SBOM."""

from __future__ import annotations

import argparse
import hashlib
import json
import tarfile
from pathlib import Path


COMMON_REQUIRED_FILES = {
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
    "sbom.spdx.json",
}

PLATFORM_REQUIRED_FILES = {
    "linux": {
        "zig-out/bin/worldvm",
        "zig-out/lib/libworldvm.so",
    },
    "macos": {
        "zig-out/bin/worldvm",
        "zig-out/lib/libworldvm.dylib",
    },
    "windows": {
        "zig-out/bin/worldvm.exe",
        "zig-out/bin/worldvm.dll",
    },
}


def sha256_bytes(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as file:
        for chunk in iter(lambda: file.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def verify_sidecar(archive_path: Path) -> None:
    sidecar_path = archive_path.with_name(archive_path.name + ".sha256")
    if not sidecar_path.exists():
        raise SystemExit(f"missing checksum sidecar: {sidecar_path}")
    fields = sidecar_path.read_text(encoding="utf-8").strip().split()
    if not fields:
        raise SystemExit(f"empty checksum sidecar: {sidecar_path}")
    expected = fields[0]
    if len(expected) != 64:
        raise SystemExit(f"invalid SHA-256 length in {sidecar_path}")
    if len(fields) > 1 and fields[1] != archive_path.name:
        raise SystemExit(f"checksum sidecar filename mismatch: {fields[1]} != {archive_path.name}")
    actual = sha256_file(archive_path)
    if actual != expected:
        raise SystemExit(f"archive checksum mismatch: {archive_path}")


def safe_members(archive: tarfile.TarFile) -> tuple[str, dict[str, tarfile.TarInfo]]:
    members = {member.name: member for member in archive.getmembers() if member.isfile()}
    roots = {name.split("/", 1)[0] for name in members}
    if len(roots) != 1:
        raise SystemExit("archive must contain exactly one top-level directory")
    root = next(iter(roots))
    for name in members:
        parts = Path(name).parts
        if name.startswith("/") or ".." in parts:
            raise SystemExit(f"unsafe archive member path: {name}")
    return root, members


def read_json_member(archive: tarfile.TarFile, members: dict[str, tarfile.TarInfo], name: str) -> dict[str, object]:
    member = members.get(name)
    if member is None:
        raise SystemExit(f"missing archive member: {name}")
    extracted = archive.extractfile(member)
    if extracted is None:
        raise SystemExit(f"cannot read archive member: {name}")
    return json.loads(extracted.read().decode("utf-8"))


def read_member_bytes(archive: tarfile.TarFile, members: dict[str, tarfile.TarInfo], name: str) -> bytes:
    member = members.get(name)
    if member is None:
        raise SystemExit(f"manifest references missing member: {name}")
    extracted = archive.extractfile(member)
    if extracted is None:
        raise SystemExit(f"cannot read archive member: {name}")
    return extracted.read()


def validate_manifest_files(archive: tarfile.TarFile, root: str, members: dict[str, tarfile.TarInfo], manifest: dict[str, object]) -> None:
    files = manifest.get("files")
    if not isinstance(files, list) or not files:
        raise SystemExit("manifest has no files list")
    seen_paths: set[str] = set()
    for item in files:
        if not isinstance(item, dict):
            raise SystemExit("manifest file entry must be an object")
        relative = item.get("path")
        expected_size = item.get("size")
        expected_sha = item.get("sha256")
        if not isinstance(relative, str) or not isinstance(expected_size, int) or not isinstance(expected_sha, str):
            raise SystemExit(f"invalid manifest file entry: {item!r}")
        if relative in seen_paths:
            raise SystemExit(f"duplicate manifest file entry: {relative}")
        seen_paths.add(relative)
        data = read_member_bytes(archive, members, f"{root}/{relative}")
        if len(data) != expected_size:
            raise SystemExit(f"size mismatch for {relative}")
        if sha256_bytes(data) != expected_sha:
            raise SystemExit(f"SHA-256 mismatch for {relative}")


def validate_required_files(members: dict[str, tarfile.TarInfo], root: str, manifest: dict[str, object]) -> None:
    system = manifest.get("system")
    if not isinstance(system, str):
        raise SystemExit("manifest system must be a string")
    platform_files = PLATFORM_REQUIRED_FILES.get(system)
    if platform_files is None:
        raise SystemExit(f"unsupported manifest system: {system}")

    required_manifest_files = COMMON_REQUIRED_FILES | platform_files
    required_archive_files = required_manifest_files | {"manifest.json"}
    manifest_files = manifest.get("files")
    if not isinstance(manifest_files, list):
        raise SystemExit("manifest files must be a list")
    manifest_paths = {
        item.get("path")
        for item in manifest_files
        if isinstance(item, dict) and isinstance(item.get("path"), str)
    }
    missing_from_manifest = sorted(required_manifest_files - manifest_paths)
    if missing_from_manifest:
        raise SystemExit("manifest missing required files: " + ", ".join(missing_from_manifest))

    archive_paths = {
        name.removeprefix(f"{root}/")
        for name in members
        if name.startswith(f"{root}/")
    }
    missing_from_archive = sorted(required_archive_files - archive_paths)
    if missing_from_archive:
        raise SystemExit("archive missing required files: " + ", ".join(missing_from_archive))


def validate_archive_inventory(root: str, members: dict[str, tarfile.TarInfo], manifest: dict[str, object]) -> None:
    files = manifest.get("files")
    if not isinstance(files, list):
        raise SystemExit("manifest files must be a list")
    expected_paths = {
        item.get("path")
        for item in files
        if isinstance(item, dict) and isinstance(item.get("path"), str)
    }
    expected_paths.add("manifest.json")
    archive_paths = {
        name.removeprefix(f"{root}/")
        for name in members
        if name.startswith(f"{root}/")
    }
    unexpected = sorted(archive_paths - expected_paths)
    if unexpected:
        raise SystemExit("archive contains unexpected files: " + ", ".join(unexpected))
    missing = sorted(expected_paths - archive_paths)
    if missing:
        raise SystemExit("archive missing manifest inventory files: " + ", ".join(missing))


def validate_manifest_metadata(root: str, manifest: dict[str, object]) -> None:
    required_strings = ("name", "version", "system", "target", "optimize", "created_at", "sbom")
    for key in required_strings:
        if not isinstance(manifest.get(key), str) or not manifest.get(key):
            raise SystemExit(f"manifest {key} must be a non-empty string")
    expected_target_prefix = f"{manifest['system']}-"
    if not manifest["target"].startswith(expected_target_prefix) or manifest["target"] == expected_target_prefix:
        raise SystemExit("manifest target must start with system name")
    expected_root = f"{manifest['name']}-{manifest['version']}-{manifest['target']}"
    if root != expected_root:
        raise SystemExit(f"archive root mismatch: {root} != {expected_root}")


def validate_sbom(manifest: dict[str, object], sbom: dict[str, object]) -> None:
    if sbom.get("spdxVersion") != "SPDX-2.3":
        raise SystemExit("SBOM must use SPDX-2.3")
    if sbom.get("SPDXID") != "SPDXRef-DOCUMENT":
        raise SystemExit("SBOM document SPDXID mismatch")
    package_name = f"{manifest['name']}-{manifest['version']}-{manifest['target']}"
    if sbom.get("name") != package_name:
        raise SystemExit("SBOM name must match package root")
    namespace = sbom.get("documentNamespace")
    if not isinstance(namespace, str) or f"/{package_name}/" not in namespace:
        raise SystemExit("SBOM documentNamespace must include package root")
    creation_info = sbom.get("creationInfo")
    if not isinstance(creation_info, dict) or creation_info.get("created") != manifest.get("created_at"):
        raise SystemExit("SBOM creation timestamp must match manifest created_at")
    packages = sbom.get("packages")
    if not isinstance(packages, list) or not packages or packages[0].get("name") != "worldvm":
        raise SystemExit("SBOM package metadata missing worldvm package")
    if packages[0].get("versionInfo") != manifest.get("version"):
        raise SystemExit("SBOM package version must match manifest version")
    sbom_files = {
        file_entry.get("fileName"): file_entry
        for file_entry in sbom.get("files", [])
        if isinstance(file_entry, dict)
    }
    for item in manifest.get("files", []):
        if not isinstance(item, dict):
            continue
        path = item.get("path")
        if path == manifest.get("sbom"):
            continue
        sbom_file = sbom_files.get(path)
        if sbom_file is None:
            raise SystemExit(f"SBOM missing manifest file: {path}")
        checksums = sbom_file.get("checksums")
        if not isinstance(checksums, list) or not checksums:
            raise SystemExit(f"SBOM file has no checksum: {path}")
        checksum = checksums[0]
        if checksum.get("algorithm") != "SHA256" or checksum.get("checksumValue") != item.get("sha256"):
            raise SystemExit(f"SBOM checksum mismatch for {path}")


def verify_archive(archive_path: Path) -> None:
    if not archive_path.exists():
        raise SystemExit(f"missing archive: {archive_path}")
    verify_sidecar(archive_path)
    with tarfile.open(archive_path, "r:gz") as archive:
        root, members = safe_members(archive)
        manifest = read_json_member(archive, members, f"{root}/manifest.json")
        sbom_name = manifest.get("sbom")
        if sbom_name != "sbom.spdx.json":
            raise SystemExit("manifest must reference sbom.spdx.json")
        sbom = read_json_member(archive, members, f"{root}/{sbom_name}")
        validate_manifest_metadata(root, manifest)
        validate_archive_inventory(root, members, manifest)
        validate_manifest_files(archive, root, members, manifest)
        validate_required_files(members, root, manifest)
        validate_sbom(manifest, sbom)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("archives", nargs="+", help="release .tar.gz archives to verify")
    args = parser.parse_args()

    for archive in args.archives:
        verify_archive(Path(archive))
        print(f"verified {archive}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
