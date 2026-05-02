import tarfile
import unittest

from tools import verify_release_package


def make_members(root, paths):
    return {f"{root}/{path}": tarfile.TarInfo(f"{root}/{path}") for path in paths}


def make_manifest(root, system="linux", payload=None, name="worldvm", version="test", target=None):
    if payload is None:
        payload = (
            verify_release_package.COMMON_REQUIRED_FILES
            | verify_release_package.PLATFORM_REQUIRED_FILES[system]
        )
    if target is None:
        target = root.removeprefix(f"{name}-{version}-")
    return {
        "name": name,
        "version": version,
        "system": system,
        "target": target,
        "optimize": "ReleaseSafe",
        "created_at": "2026-05-02T00:00:00+00:00",
        "sbom": "sbom.spdx.json",
        "files": [{"path": path} for path in sorted(payload)],
    }


def make_sbom(manifest):
    package_name = f"{manifest['name']}-{manifest['version']}-{manifest['target']}"
    return {
        "spdxVersion": "SPDX-2.3",
        "SPDXID": "SPDXRef-DOCUMENT",
        "name": package_name,
        "documentNamespace": f"https://worldvm.invalid/sbom/{package_name}/test",
        "creationInfo": {"created": manifest["created_at"], "creators": ["Tool: test"]},
        "packages": [
            {
                "name": "worldvm",
                "SPDXID": "SPDXRef-Package-worldvm",
                "versionInfo": manifest["version"],
            }
        ],
        "files": [
            {
                "fileName": item["path"],
                "checksums": [{"algorithm": "SHA256", "checksumValue": item.get("sha256", "0" * 64)}],
            }
            for item in manifest["files"]
            if item["path"] != manifest["sbom"]
        ],
    }


class ReleaseVerifierTests(unittest.TestCase):
    def test_required_files_accept_linux_inventory(self):
        root = "worldvm-test-linux-x86_64"
        payload = (
            verify_release_package.COMMON_REQUIRED_FILES
            | verify_release_package.PLATFORM_REQUIRED_FILES["linux"]
        )
        manifest = make_manifest(root, payload=payload)
        members = make_members(root, payload | {"manifest.json"})

        verify_release_package.validate_manifest_metadata(root, manifest)
        verify_release_package.validate_required_files(members, root, manifest)
        verify_release_package.validate_archive_inventory(root, members, manifest)

    def test_required_files_reject_missing_platform_binary(self):
        root = "worldvm-test-linux-x86_64"
        payload = (
            verify_release_package.COMMON_REQUIRED_FILES
            | verify_release_package.PLATFORM_REQUIRED_FILES["linux"]
        ) - {"zig-out/lib/libworldvm.so"}
        manifest = make_manifest(root, payload=payload)
        members = make_members(root, payload | {"manifest.json"})

        with self.assertRaises(SystemExit) as raised:
            verify_release_package.validate_required_files(members, root, manifest)
        self.assertIn("manifest missing required files", str(raised.exception))
        self.assertIn("zig-out/lib/libworldvm.so", str(raised.exception))

    def test_archive_inventory_rejects_unexpected_files(self):
        root = "worldvm-test-linux-x86_64"
        payload = (
            verify_release_package.COMMON_REQUIRED_FILES
            | verify_release_package.PLATFORM_REQUIRED_FILES["linux"]
        )
        manifest = make_manifest(root, payload=payload)
        members = make_members(root, payload | {"manifest.json", "unexpected.txt"})

        with self.assertRaises(SystemExit) as raised:
            verify_release_package.validate_archive_inventory(root, members, manifest)
        self.assertIn("archive contains unexpected files", str(raised.exception))
        self.assertIn("unexpected.txt", str(raised.exception))

    def test_required_files_reject_unsupported_system(self):
        root = "worldvm-test-freebsd-x86_64"
        manifest = {"system": "freebsd", "files": []}
        members = make_members(root, {"manifest.json"})

        with self.assertRaises(SystemExit) as raised:
            verify_release_package.validate_required_files(members, root, manifest)
        self.assertIn("unsupported manifest system", str(raised.exception))

    def test_manifest_metadata_rejects_root_mismatch(self):
        manifest = make_manifest("worldvm-test-linux-x86_64")

        with self.assertRaises(SystemExit) as raised:
            verify_release_package.validate_manifest_metadata("worldvm-other-linux-x86_64", manifest)
        self.assertIn("archive root mismatch", str(raised.exception))

    def test_manifest_metadata_rejects_target_system_mismatch(self):
        manifest = make_manifest("worldvm-test-linux-x86_64")
        manifest["target"] = "windows-x86_64"

        with self.assertRaises(SystemExit) as raised:
            verify_release_package.validate_manifest_metadata("worldvm-test-linux-x86_64", manifest)
        self.assertIn("manifest target must start with system name", str(raised.exception))

    def test_sbom_metadata_accepts_matching_manifest(self):
        manifest = make_manifest("worldvm-test-linux-x86_64")
        for item in manifest["files"]:
            item["sha256"] = "0" * 64
        sbom = make_sbom(manifest)

        verify_release_package.validate_sbom(manifest, sbom)

    def test_sbom_metadata_rejects_version_mismatch(self):
        manifest = make_manifest("worldvm-test-linux-x86_64")
        for item in manifest["files"]:
            item["sha256"] = "0" * 64
        sbom = make_sbom(manifest)
        sbom["packages"][0]["versionInfo"] = "other"

        with self.assertRaises(SystemExit) as raised:
            verify_release_package.validate_sbom(manifest, sbom)
        self.assertIn("SBOM package version must match manifest version", str(raised.exception))

    def test_sbom_metadata_rejects_creation_timestamp_mismatch(self):
        manifest = make_manifest("worldvm-test-linux-x86_64")
        for item in manifest["files"]:
            item["sha256"] = "0" * 64
        sbom = make_sbom(manifest)
        sbom["creationInfo"]["created"] = "2026-05-03T00:00:00+00:00"

        with self.assertRaises(SystemExit) as raised:
            verify_release_package.validate_sbom(manifest, sbom)
        self.assertIn("SBOM creation timestamp must match manifest created_at", str(raised.exception))


if __name__ == "__main__":
    unittest.main()
