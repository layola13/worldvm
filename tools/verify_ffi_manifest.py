#!/usr/bin/env python3
"""Verify the WorldVM FFI export manifest against src/vm_hook.zig."""

from __future__ import annotations

import argparse
import json
import re
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SOURCE_PATH = ROOT / "src/vm_hook.zig"
MANIFEST_PATH = ROOT / "docs/ffi_symbols_v1.json"


def normalize_type(value: str) -> str:
    return re.sub(r"\s+", " ", value.strip())


def split_params(params: str) -> list[str]:
    parts: list[str] = []
    current: list[str] = []
    bracket_depth = 0
    for char in params:
        if char == "[":
            bracket_depth += 1
        elif char == "]" and bracket_depth:
            bracket_depth -= 1
        elif char == "," and bracket_depth == 0:
            part = "".join(current).strip()
            if part:
                parts.append(part)
            current = []
            continue
        current.append(char)
    part = "".join(current).strip()
    if part:
        parts.append(part)
    return parts


def parse_params(params: str) -> list[dict[str, str]]:
    parsed: list[dict[str, str]] = []
    for raw in split_params(params):
        if ":" not in raw:
            raise SystemExit(f"cannot parse FFI parameter: {raw!r}")
        name, type_name = raw.split(":", 1)
        parsed.append({"name": name.strip(), "type": normalize_type(type_name)})
    return parsed


def parse_exports(source: str) -> list[dict[str, object]]:
    symbols: list[dict[str, object]] = []
    marker = "pub export fn "
    offset = 0
    while True:
        start = source.find(marker, offset)
        if start == -1:
            break
        name_start = start + len(marker)
        match = re.match(r"[A-Za-z_][A-Za-z0-9_]*", source[name_start:])
        if match is None:
            raise SystemExit(f"cannot parse FFI function name near byte {start}")
        name = match.group(0)
        open_paren = source.find("(", name_start + len(name))
        if open_paren == -1:
            raise SystemExit(f"cannot find parameter list for {name}")

        depth = 0
        close_paren = -1
        for index in range(open_paren, len(source)):
            if source[index] == "(":
                depth += 1
            elif source[index] == ")":
                depth -= 1
                if depth == 0:
                    close_paren = index
                    break
        if close_paren == -1:
            raise SystemExit(f"unterminated parameter list for {name}")

        body_open = source.find("{", close_paren)
        if body_open == -1:
            raise SystemExit(f"cannot find body for {name}")
        params = source[open_paren + 1 : close_paren]
        return_type = normalize_type(source[close_paren + 1 : body_open])
        line = source.count("\n", 0, start) + 1
        signature = f"{name}({', '.join(split_params(params))}) {return_type}"
        symbols.append(
            {
                "name": name,
                "parameters": parse_params(params),
                "return_type": return_type,
                "signature": normalize_type(signature),
                "source_line": line,
            }
        )
        offset = body_open + 1
    return sorted(symbols, key=lambda item: item["name"])


def build_manifest() -> dict[str, object]:
    symbols = parse_exports(SOURCE_PATH.read_text(encoding="utf-8"))
    return {
        "abi_version": 1,
        "description": "WorldVM public FFI symbols exported from src/vm_hook.zig.",
        "source": SOURCE_PATH.relative_to(ROOT).as_posix(),
        "symbol_count": len(symbols),
        "symbols": symbols,
    }


def load_manifest(path: Path) -> dict[str, object]:
    if not path.exists():
        raise SystemExit(f"missing FFI manifest: {path}")
    return json.loads(path.read_text(encoding="utf-8"))


def manifest_for_compare(manifest: dict[str, object]) -> dict[str, object]:
    cleaned = dict(manifest)
    cleaned["symbols"] = [
        {key: value for key, value in symbol.items() if key != "source_line"}
        for symbol in cleaned.get("symbols", [])
        if isinstance(symbol, dict)
    ]
    return cleaned


def verify_manifest(path: Path) -> None:
    expected = build_manifest()
    actual = load_manifest(path)
    if manifest_for_compare(actual) != manifest_for_compare(expected):
        expected_names = {symbol["name"] for symbol in expected["symbols"]}
        actual_names = {symbol["name"] for symbol in actual.get("symbols", []) if isinstance(symbol, dict)}
        added = sorted(expected_names - actual_names)
        removed = sorted(actual_names - expected_names)
        details = []
        if added:
            details.append(f"new exports: {', '.join(added[:20])}")
        if removed:
            details.append(f"removed exports: {', '.join(removed[:20])}")
        if not details:
            details.append("signature or metadata drift")
        raise SystemExit("FFI manifest mismatch: " + "; ".join(details))


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", default=str(MANIFEST_PATH), help="FFI manifest path")
    parser.add_argument("--write", action="store_true", help="write the current manifest instead of verifying")
    args = parser.parse_args()

    manifest_path = Path(args.manifest)
    if not manifest_path.is_absolute():
        manifest_path = ROOT / manifest_path

    if args.write:
        manifest = build_manifest()
        manifest_path.parent.mkdir(parents=True, exist_ok=True)
        manifest_path.write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        print(f"wrote {manifest_path.relative_to(ROOT)} ({manifest['symbol_count']} symbols)")
        return 0

    verify_manifest(manifest_path)
    print("FFI manifest verified")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
