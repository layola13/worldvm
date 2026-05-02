#!/usr/bin/env python3
"""Verify docs/entity16_abi_v1.json against Entity16 source constants."""

from __future__ import annotations

import json
import re
import hashlib
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / "src" / "entity16.zig"
MANIFEST = ROOT / "docs" / "entity16_abi_v1.json"


def source_int_constant(source: str, name: str) -> int:
    match = re.search(rf"pub const {re.escape(name)}: [^=]+ = (\d+);", source)
    if match is None:
        raise SystemExit(f"missing source constant: {name}")
    return int(match.group(1))


def require(condition: bool, message: str) -> None:
    if not condition:
        raise SystemExit(message)


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as file:
        for chunk in iter(lambda: file.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def main() -> int:
    source = SOURCE.read_text(encoding="utf-8")
    manifest = json.loads(MANIFEST.read_text(encoding="utf-8"))

    require(manifest["schema"] == "worldvm.entity16.abi", "unexpected Entity16 ABI schema")
    require(manifest["version"] == source_int_constant(source, "ENTITY16_EXTENSION_ABI_VERSION"), "ABI version mismatch")
    require(manifest["entity_size"] == source_int_constant(source, "ENTITY_SIZE"), "Entity16 size mismatch")
    require(manifest["endianness"] == "little", "Entity16 ABI must explicitly use little endian")

    expected_sizes = {
        "topology": source_int_constant(source, "TOPOLOGY_WORDS") * 8,
        "physics": source_int_constant(source, "PHYSICS_SIZE"),
        "chemistry": source_int_constant(source, "CHEMISTRY_SIZE"),
        "visual": source_int_constant(source, "VISUAL_SIZE"),
        "semantics": source_int_constant(source, "SEMANTICS_SIZE"),
        "affect": source_int_constant(source, "AFFECT_SIZE"),
        "relations": 64 * 4,
        "behavior": source_int_constant(source, "BEHAVIOR_SIZE"),
        "reserved": source_int_constant(source, "ENTITY_SIZE")
        - (
            source_int_constant(source, "TOPOLOGY_WORDS") * 8
            + source_int_constant(source, "PHYSICS_SIZE")
            + source_int_constant(source, "CHEMISTRY_SIZE")
            + source_int_constant(source, "VISUAL_SIZE")
            + source_int_constant(source, "SEMANTICS_SIZE")
            + source_int_constant(source, "AFFECT_SIZE")
            + 64 * 4
            + source_int_constant(source, "BEHAVIOR_SIZE")
        ),
    }

    offset = 0
    for block in manifest["layout"]:
        name = block["name"]
        require(name in expected_sizes, f"unknown Entity16 layout block: {name}")
        require(block["offset"] == offset, f"offset mismatch for Entity16 block {name}")
        require(block["size"] == expected_sizes[name], f"size mismatch for Entity16 block {name}")
        offset += block["size"]
    require(offset == manifest["entity_size"], "Entity16 layout does not sum to entity_size")

    expected_chemistry = [
        ("chemical_signature", 0, 4),
        ("smell_profile_id", 4, 2),
        ("taste_profile_id", 6, 2),
        ("reaction_rule_set", 8, 2),
        ("toxicity_level", 10, 2),
        ("extension_abi_version", 12, 2),
        ("reserved", 14, 2),
    ]
    for field, expected in zip(manifest["chemistry_fields"], expected_chemistry, strict=True):
        name, field_offset, size = expected
        require(field["name"] == name, f"chemistry field name mismatch: {field['name']} != {name}")
        require(field["offset"] == field_offset, f"chemistry field offset mismatch: {name}")
        require(field["size"] == size, f"chemistry field size mismatch: {name}")
    version_field = next(field for field in manifest["chemistry_fields"] if field["name"] == "extension_abi_version")
    require(version_field["default"] == manifest["version"], "extension_abi_version default must match ABI version")

    fixture = manifest["fixture"]
    fixture_path = ROOT / fixture["path"]
    require(fixture_path.exists(), f"missing Entity16 ABI fixture: {fixture['path']}")
    fixture_bytes = fixture_path.read_bytes()
    require(len(fixture_bytes) == manifest["entity_size"], "Entity16 ABI fixture size mismatch")
    require(sha256(fixture_path) == fixture["sha256"], "Entity16 ABI fixture SHA-256 mismatch")
    chemistry_offset = next(block["offset"] for block in manifest["layout"] if block["name"] == "chemistry")
    version_offset = chemistry_offset + version_field["offset"]
    version_size = version_field["size"]
    version_bytes = fixture_bytes[version_offset : version_offset + version_size]
    require(int.from_bytes(version_bytes, "little") == manifest["version"], "fixture extension ABI version mismatch")
    mutable = bytearray(fixture_bytes)
    mutable[version_offset : version_offset + version_size] = b"\x00" * version_size
    require(all(byte == 0 for byte in mutable), "default fixture must only set extension_abi_version")

    print("Entity16 ABI manifest verified")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
