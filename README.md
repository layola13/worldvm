# WorldVM

WorldVM is a Zig-based voxel world simulation runtime with a small CLI and a Python `ctypes` wrapper. The current implementation focuses on deterministic headless simulation, physics/query primitives, scene instance lifecycle operations, and testable FFI entry points.

## Requirements

- Zig `0.14.1`
- Python `3.10+`

## Build

```bash
zig build
```

This builds:

- `zig-out/bin/worldvm` — CLI executable
- `zig-out/lib/libworldvm.so` — Linux shared library used by `worldvm.py`
- `zig-out/lib/libworldvm.dylib` — macOS shared library used by `worldvm.py`
- `zig-out/bin/worldvm.dll` — Windows shared library used by `worldvm.py`

## CLI Usage

Run the default scenario:

```bash
zig build run -- run --scenario apple_table --ticks 3
```

Dump an initial scenario without ticking:

```bash
zig build run -- dump --scenario apple_table
```

Run a simple benchmark:

```bash
zig build run -- bench --scenario apple_table
```

Run multi-scenario benchmark metadata:

```bash
python3 tools/benchmark_scenarios.py --ticks 100 --runs 3
python3 tools/benchmark_scenarios.py --ticks 100 --runs 3 --max-average-ms 250
python3 tools/benchmark_scenarios.py --scenario apple_table --ticks 3 --runs 1 --skip-build --baseline benchmarks/ci_benchmark_baseline.json
```

Built-in scenario names are selected in `src/main.zig`: `apple_table`, `hammer_glass`, `water_flow`, `bounce_test`, `domino_chain`, `pyramid_collapse`, `multi_stack`, and `gas_expand`.

## Python FFI

Build the shared library first:

```bash
zig build
```

By default, `worldvm.py` loads the platform shared library from `zig-out`. Override this with either `WORLDVM_LIBRARY_PATH` or `WorldVM(library_path=...)`:

```bash
WORLDVM_LIBRARY_PATH="$PWD/zig-out/lib/libworldvm.so" python3 examples/python_lifecycle.py
python3 examples/python_lifecycle.py --library "$PWD/zig-out/lib/libworldvm.so"
```

Minimal lifecycle smoke:

```bash
PYTHONDONTWRITEBYTECODE=1 python3 - <<'PY'
from worldvm import WorldVM

vm = WorldVM()
try:
    vm.reset()
    handle = vm.spawn_handle(0, 1, 2, 3)
    assert handle != 0
    assert vm.instance_count() == 1
    assert vm.resolve_instance_handle(handle) == 0
    assert vm.mark_instance_broken_handle(handle) == 0
    assert vm.compact_broken_instances() == 1
    assert vm.resolve_instance_handle(handle) == -1
    assert vm.instance_count() == 0
finally:
    vm.close()
PY
```

Runnable lifecycle example:

```bash
python3 examples/python_lifecycle.py
```

Important wrapper methods:

- `spawn(eid, x, y, z)`
- `spawn_handle(eid, x, y, z)`
- `instance_count()`
- `instance_handle(inst_idx)`
- `resolve_instance_handle(handle)`
- `remove_instance(inst_idx)`
- `remove_instance_handle(handle)`
- `mark_instance_broken(inst_idx)`
- `mark_instance_broken_handle(handle)`
- `compact_broken_instances()`
- `run(t=1)`
- `last_step()`

## Validation

The GitHub Actions workflow in `.github/workflows/ci.yml` runs the fast gate, builds the CLI/shared library, and executes CLI plus Python FFI smokes.

Fast local gate:

```bash
zig build test-fast
python3 tools/verify_entity16_abi.py
python3 tools/verify_ffi_manifest.py
python3 tools/verify_python_wrapper_api.py
python3 tools/verify_readme_snippets.py README.md
python3 -m py_compile worldvm.py
python3 -m unittest tests.test_package_release
python3 -m unittest tests.test_release_verifier
python3 -m unittest tests.test_smoke_release_package
python3 -m unittest tests.test_worldvm_wrapper
PYTHONDONTWRITEBYTECODE=1 python3 -m unittest tests.physics.test_acceptance_scenario
python3 tools/benchmark_scenarios.py --scenario apple_table --ticks 3 --runs 1 --skip-build --baseline benchmarks/ci_benchmark_baseline.json
```

Full gate:

```bash
zig build check-matrix
zig build test-full
zig build run -- run --scenario apple_table --ticks 3
```

The current test matrix is expected to cover every `src/*.zig` file.

## Release Package

Version tags use SemVer without a leading `v` in generated package metadata and with a leading `v` in Git tags:

- `MAJOR` changes break public CLI, FFI, Python wrapper, or persisted data contracts.
- `MINOR` changes add backward-compatible APIs, scenarios, or packaging targets.
- `PATCH` changes fix bugs, tests, docs, or packaging without changing public contracts.

Generate a changelog section before tagging:

```bash
python3 tools/generate_changelog.py --version 0.1.0 --write
```

Preview release notes that will be used by GitHub Releases:

```bash
python3 tools/extract_release_notes.py --version 0.1.0 --require-bullet --reject-placeholder
```

Create a local release archive:

```bash
python3 tools/package_release.py --version dev
python3 tools/verify_release_package.py dist/worldvm-dev-*.tar.gz
python3 tools/smoke_release_package.py dist/worldvm-dev-*.tar.gz
```

The archive is written under `dist/` and includes:

- `README.md`
- `CHANGELOG.md`
- `worldvm.py`
- `examples/python_lifecycle.py`
- `tests/physics/test_acceptance_scenario.py`
- `tests/test_package_release.py`
- `tests/test_release_verifier.py`
- `tests/test_smoke_release_package.py`
- `tests/test_worldvm_wrapper.py`
- referenced release/validation tools under `tools/`
- `benchmarks/ci_benchmark_baseline.json`
- `docs/development_plan.md`
- `docs/entity16_abi_v1.json`
- `docs/ffi_symbols_v1.json`
- `tests/fixtures/entity16_abi_v1_default.bin`
- platform CLI binary: `worldvm` or `worldvm.exe`
- platform shared library: `libworldvm.so`, `libworldvm.dylib`, or `worldvm.dll`
- `manifest.json` with size and SHA-256 metadata
- `sbom.spdx.json` with SPDX 2.3 file checksums for packaged artifacts
- sibling `.tar.gz.sha256` checksum for the release archive

Use `--skip-build` to package existing `zig-out` artifacts without rebuilding. The package script selects Linux, macOS, or Windows artifact names from the host platform and normalizes target architecture labels such as `AMD64` to `x86_64` and `aarch64` to `arm64`.

Publish a hosted GitHub Release:

```bash
git tag v0.1.0
git push origin v0.1.0
```

The `.github/workflows/release.yml` workflow runs the release gate on Linux, creates Linux/macOS/Windows packages, uploads them as workflow artifacts, creates GitHub provenance attestations, extracts release notes from `CHANGELOG.md`, and attaches the tarballs plus `.sha256` checksum files to the GitHub Release. It can also be started manually with a version input.

Verify a downloaded release archive:

```bash
sha256sum -c worldvm-0.1.0-linux-x86_64.tar.gz.sha256
gh attestation verify worldvm-0.1.0-linux-x86_64.tar.gz -R OWNER/REPO
```

Use the matching `.sha256` sidecar for the archive and replace `OWNER/REPO` with the GitHub repository that published the release.

## Current Development Focus

Detailed engineering assessment and roadmap: `docs/development_plan.md`.

Implemented base usability work:

- Explicit `Entity16` ABI extension blocks while preserving the 4KB entity layout.
- `Scene1024` instance lifecycle operations: remove, mark broken, and compact broken instances.
- FFI and Python wrapper access to the lifecycle operations.
- Stable external instance handles for lifecycle calls that must survive index compaction.
- Versioned `Entity16` extension block contract for ABI compatibility checks.
- Checked `Entity16` ABI v1 manifest fixture for serialized-layout compatibility.
- Binary `Entity16` ABI v1 default fixture with byte-offset validation.
- Checked FFI symbol manifest for exported VM hook signature compatibility.
- Checked Python wrapper FFI references against the exported symbol manifest.
- Checked README Python snippets for copy/paste syntax correctness.
- Scenario-level Python acceptance smoke for terrain/material/medium/lifecycle behavior.
- Python wrapper unit tests for shared-library path resolution and load-failure guidance.
- Minimal release packaging for CLI, shared library, Python wrapper, and manifest metadata.
- Runnable Python lifecycle example for handle-based instance management.
- Configurable Python shared-library loading via environment variable or constructor argument.
- Platform-aware package artifact selection for Linux, macOS, and Windows.
- Multi-scenario benchmark runner that emits JSON metadata for longer local checks.
- Optional benchmark elapsed-time thresholds for local and CI smoke gates.
- Versioned benchmark baseline JSON for CI smoke thresholds.
- Formal SemVer release policy and changelog generation.
- Tag-triggered GitHub Release publishing for Linux, macOS, and Windows packaged artifacts.
- SHA-256 checksum sidecars for hosted release archives.
- GitHub provenance attestations for hosted release archives and checksum sidecars.
- Hosted release notes extracted from `CHANGELOG.md`.
- Release-note quality gates for empty notes, missing bullets, and placeholder text.
- Downstream checksum and provenance verification instructions for release consumers.
- SPDX 2.3 SBOM generation embedded in release packages.
- Release package verifier for checksum, archive root metadata, exact archive inventory, manifest, SBOM metadata/checksum consistency, and required payload files.
- Release package smoke test that extracts the archive and runs the packaged CLI plus Python lifecycle example.
- Release smoke unit tests for safe extraction, package-local tool imports, and platform CLI path selection.
- Release package includes changelog, ABI manifest/fixture, validation tools, smoke tests, and benchmark baseline metadata.
- Package tooling unit tests for platform artifact names and normalized target labels.
- Release verifier unit tests for required payload and exact inventory checks.
- Continuous physics routing through `PhysicsWorld` from the tick engine.
- Crash-defense clamp behavior for instance velocity and position safety bounds.

Recommended next work:

1. Expand SBOM metadata when external dependencies are introduced.
2. Tighten benchmark baselines after enough hosted-runner samples are collected.
3. Add full Entity16 read/write round-trip tests if a persisted entity file format is introduced.
