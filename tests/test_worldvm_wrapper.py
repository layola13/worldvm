import os
import pathlib
import unittest
from unittest import mock

import worldvm


class WorldVMWrapperPathTests(unittest.TestCase):
    def test_platform_library_names(self):
        self.assertEqual(
            worldvm.platform_library_name_and_dir("win32"),
            ("worldvm.dll", os.path.join("zig-out", "bin")),
        )
        self.assertEqual(
            worldvm.platform_library_name_and_dir("darwin"),
            ("libworldvm.dylib", os.path.join("zig-out", "lib")),
        )
        self.assertEqual(
            worldvm.platform_library_name_and_dir("linux"),
            ("libworldvm.so", os.path.join("zig-out", "lib")),
        )

    def test_explicit_library_path_wins(self):
        path = pathlib.Path("/tmp/libworldvm-explicit.so")
        with mock.patch.dict(os.environ, {worldvm.WORLDVM_LIBRARY_PATH_ENV: "/tmp/libworldvm-env.so"}):
            self.assertEqual(worldvm.resolve_library_path(path), os.fspath(path))

    def test_environment_library_path_wins_over_default(self):
        with mock.patch.dict(os.environ, {worldvm.WORLDVM_LIBRARY_PATH_ENV: "/tmp/libworldvm-env.so"}):
            self.assertEqual(worldvm.resolve_library_path(), "/tmp/libworldvm-env.so")

    def test_default_path_falls_back_to_platform_zig_out(self):
        with mock.patch.dict(os.environ, {}, clear=True), mock.patch("worldvm.os.path.exists", return_value=False):
            self.assertEqual(worldvm.resolve_library_path(), worldvm.lib_path)

    def test_load_failure_mentions_override_options(self):
        with self.assertRaises(OSError) as raised:
            worldvm.WorldVM(library_path="/tmp/worldvm-missing-test-library.so")
        message = str(raised.exception)
        self.assertIn(worldvm.WORLDVM_LIBRARY_PATH_ENV, message)
        self.assertIn("WorldVM(library_path=...)", message)


if __name__ == "__main__":
    unittest.main()
