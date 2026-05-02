#!/usr/bin/env python3
"""Generate the canonical Entity16 ABI v1 binary fixture."""

from __future__ import annotations

import json
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
MANIFEST = ROOT / "docs" / "entity16_abi_v1.json"
FIXTURE = ROOT / "tests" / "fixtures" / "entity16_abi_v1_default.bin"


def main() -> int:
    manifest = json.loads(MANIFEST.read_text(encoding="utf-8"))
    data = bytearray(manifest["entity_size"])
    chemistry_offset = next(block["offset"] for block in manifest["layout"] if block["name"] == "chemistry")
    version_field = next(field for field in manifest["chemistry_fields"] if field["name"] == "extension_abi_version")
    version = int(version_field["default"])
    field_offset = chemistry_offset + int(version_field["offset"])
    field_size = int(version_field["size"])
    data[field_offset : field_offset + field_size] = version.to_bytes(field_size, "little")
    FIXTURE.parent.mkdir(parents=True, exist_ok=True)
    FIXTURE.write_bytes(data)
    print(FIXTURE.relative_to(ROOT))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
