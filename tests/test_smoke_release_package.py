import io
import sys
import tarfile
import tempfile
import unittest
from pathlib import Path

from tools import smoke_release_package


def make_archive(path: Path, members: dict[str, bytes]) -> None:
    with tarfile.open(path, "w:gz") as archive:
        for name, data in members.items():
            info = tarfile.TarInfo(name)
            info.size = len(data)
            archive.addfile(info, fileobj=io.BytesIO(data))


class SmokeReleasePackageTests(unittest.TestCase):
    def test_tools_dir_is_importable_when_script_runs_from_package(self):
        self.assertIn(str(smoke_release_package.TOOLS_DIR), sys.path)

    def test_safe_extract_accepts_single_root(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            archive_path = Path(temp_dir) / "package.tar.gz"
            make_archive(archive_path, {"worldvm-test/manifest.json": b"{}"})
            output_dir = Path(temp_dir) / "out"
            output_dir.mkdir()

            root = smoke_release_package.safe_extract(archive_path, output_dir)

            self.assertEqual(root, output_dir / "worldvm-test")
            self.assertTrue((root / "manifest.json").exists())

    def test_safe_extract_rejects_multiple_roots(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            archive_path = Path(temp_dir) / "package.tar.gz"
            make_archive(
                archive_path,
                {
                    "worldvm-one/manifest.json": b"{}",
                    "worldvm-two/manifest.json": b"{}",
                },
            )

            with self.assertRaises(SystemExit) as raised:
                smoke_release_package.safe_extract(archive_path, Path(temp_dir) / "out")
            self.assertIn("exactly one top-level directory", str(raised.exception))

    def test_safe_extract_rejects_links(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            archive_path = Path(temp_dir) / "package.tar.gz"
            with tarfile.open(archive_path, "w:gz") as archive:
                info = tarfile.TarInfo("worldvm-test/link")
                info.type = tarfile.SYMTYPE
                info.linkname = "/tmp/target"
                archive.addfile(info)

            with self.assertRaises(SystemExit) as raised:
                smoke_release_package.safe_extract(archive_path, Path(temp_dir) / "out")
            self.assertIn("links are not supported", str(raised.exception))

    def test_cli_path_selects_platform_binary(self):
        root = Path("/tmp/worldvm")
        self.assertEqual(
            smoke_release_package.cli_path(root, {"system": "windows"}),
            root / "zig-out/bin/worldvm.exe",
        )
        self.assertEqual(
            smoke_release_package.cli_path(root, {"system": "linux"}),
            root / "zig-out/bin/worldvm",
        )
        self.assertEqual(
            smoke_release_package.cli_path(root, {"system": "macos"}),
            root / "zig-out/bin/worldvm",
        )


if __name__ == "__main__":
    unittest.main()
