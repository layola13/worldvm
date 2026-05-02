#!/usr/bin/env python3
"""Run a minimal WorldVM Python FFI lifecycle example."""

from __future__ import annotations

import json
import sys
import argparse
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from worldvm import WorldVM  # noqa: E402


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--library", help="explicit path to libworldvm.so, libworldvm.dylib, or worldvm.dll")
    args = parser.parse_args()

    vm = WorldVM(library_path=args.library)
    try:
        vm.reset()

        anchor_handle = vm.spawn_handle(0, 1, 2, 3)
        transient_handle = vm.spawn_handle(0, 4, 5, 6)
        if anchor_handle == 0 or transient_handle == 0:
            raise RuntimeError("failed to spawn lifecycle example instances")

        transient_index = vm.resolve_instance_handle(transient_handle)
        if transient_index < 0:
            raise RuntimeError("new transient handle did not resolve")

        vm.run(3)
        if vm.mark_instance_broken_handle(transient_handle) != 0:
            raise RuntimeError("failed to mark transient instance broken")

        removed = vm.compact_broken_instances()
        if removed != 1:
            raise RuntimeError(f"expected compact to remove 1 instance, removed {removed}")
        if vm.resolve_instance_handle(transient_handle) != -1:
            raise RuntimeError("removed transient handle still resolves")

        summary = {
            "anchor_handle": anchor_handle,
            "anchor_index": vm.resolve_instance_handle(anchor_handle),
            "instance_count": vm.instance_count(),
            "last_step": vm.last_step(),
            "removed": removed,
            "transient_handle": transient_handle,
        }
        print(json.dumps(summary, indent=2, sort_keys=True))
        return 0
    finally:
        vm.close()


if __name__ == "__main__":
    raise SystemExit(main())
