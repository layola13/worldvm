#!/usr/bin/env python3
"""Verify that worldvm.py only calls FFI symbols declared in the manifest."""

from __future__ import annotations

import argparse
import ast
import json
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
WRAPPER_PATH = ROOT / "worldvm.py"
MANIFEST_PATH = ROOT / "docs/ffi_symbols_v1.json"


class WrapperFFIVisitor(ast.NodeVisitor):
    def __init__(self) -> None:
        self.symbol_lines: dict[str, set[int]] = {}

    def visit_Attribute(self, node: ast.Attribute) -> None:
        if (
            isinstance(node.value, ast.Attribute)
            and node.value.attr == "lib"
            and isinstance(node.value.value, ast.Name)
            and node.value.value.id == "self"
        ):
            self.symbol_lines.setdefault(node.attr, set()).add(node.lineno)
        self.generic_visit(node)


def load_manifest_symbols(path: Path) -> set[str]:
    manifest = json.loads(path.read_text(encoding="utf-8"))
    symbols = manifest.get("symbols")
    if not isinstance(symbols, list):
        raise SystemExit(f"invalid FFI manifest symbols list: {path}")
    names = {
        symbol.get("name")
        for symbol in symbols
        if isinstance(symbol, dict) and isinstance(symbol.get("name"), str)
    }
    if len(names) != len(symbols):
        raise SystemExit(f"invalid FFI manifest symbol entries: {path}")
    return names


def collect_wrapper_symbols(path: Path) -> dict[str, set[int]]:
    visitor = WrapperFFIVisitor()
    visitor.visit(ast.parse(path.read_text(encoding="utf-8"), filename=str(path)))
    return visitor.symbol_lines


def verify_wrapper(wrapper_path: Path, manifest_path: Path, print_unwrapped: bool) -> None:
    manifest_symbols = load_manifest_symbols(manifest_path)
    wrapper_symbols = collect_wrapper_symbols(wrapper_path)
    missing = sorted(set(wrapper_symbols) - manifest_symbols)
    if missing:
        details = []
        for symbol in missing:
            lines = ",".join(str(line) for line in sorted(wrapper_symbols[symbol]))
            details.append(f"{symbol} at {wrapper_path.name}:{lines}")
        raise SystemExit("Python wrapper references missing FFI symbols: " + "; ".join(details))

    if print_unwrapped:
        unwrapped = sorted(manifest_symbols - set(wrapper_symbols))
        for symbol in unwrapped:
            print(symbol)

    print(f"Python wrapper FFI references verified ({len(wrapper_symbols)} symbols)")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--wrapper", default=str(WRAPPER_PATH), help="Python wrapper path")
    parser.add_argument("--manifest", default=str(MANIFEST_PATH), help="FFI manifest path")
    parser.add_argument("--print-unwrapped", action="store_true", help="print exported FFI symbols not used by the wrapper")
    args = parser.parse_args()

    wrapper_path = Path(args.wrapper)
    manifest_path = Path(args.manifest)
    if not wrapper_path.is_absolute():
        wrapper_path = ROOT / wrapper_path
    if not manifest_path.is_absolute():
        manifest_path = ROOT / manifest_path

    verify_wrapper(wrapper_path, manifest_path, args.print_unwrapped)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
