# Changelog

## Unreleased

### Added
- Base release workflow, packaging, checksum, and provenance tooling.
- Stable instance lifecycle APIs for index and handle-based access.
- Entity16 extension ABI version marker.
- FFI symbol manifest verification for exported VM hook signatures.
- Configurable Python shared-library loading via `WORLDVM_LIBRARY_PATH` or `WorldVM(library_path=...)`.
- Python wrapper FFI reference verification against the exported symbol manifest.
- Python wrapper unit tests for shared-library path resolution.
- README Python snippet syntax verification.
- Release package verification for exact archive inventory and required payload files.
- Package tooling tests for platform artifact names and normalized target labels.
- Release verifier unit tests for required payload, archive root metadata, SBOM metadata, and exact inventory checks.
- Release packages now include referenced validation/release tools and smoke tests.
- Release package smoke tool that extracts an archive and runs packaged CLI plus Python FFI lifecycle checks.
- Release smoke unit tests for safe extraction, package-local imports, and platform CLI path selection.
- Scenario acceptance and benchmark smoke coverage.

### Fixed
- Continuous physics routing through the `PhysicsWorld` bridge.
- Crash-defense instance clamping contract.
