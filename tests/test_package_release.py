import unittest

from tools import package_release


class PackageReleaseTests(unittest.TestCase):
    def test_normalize_machine_aliases(self):
        self.assertEqual(package_release.normalize_machine("AMD64"), "x86_64")
        self.assertEqual(package_release.normalize_machine("x64"), "x86_64")
        self.assertEqual(package_release.normalize_machine("x86-64"), "x86_64")
        self.assertEqual(package_release.normalize_machine("aarch64"), "arm64")
        self.assertEqual(package_release.normalize_machine("ARM64"), "arm64")
        self.assertEqual(package_release.normalize_machine("i686"), "x86")

    def test_target_label_uses_normalized_machine(self):
        self.assertEqual(package_release.target_label("windows", "AMD64"), "windows-x86_64")
        self.assertEqual(package_release.target_label("macos", "arm64"), "macos-arm64")
        self.assertEqual(package_release.target_label("linux", "x86_64"), "linux-x86_64")

    def test_platform_package_files(self):
        self.assertEqual(
            package_release.platform_package_files("linux"),
            ("zig-out/bin/worldvm", "zig-out/lib/libworldvm.so"),
        )
        self.assertEqual(
            package_release.platform_package_files("macos"),
            ("zig-out/bin/worldvm", "zig-out/lib/libworldvm.dylib"),
        )
        self.assertEqual(
            package_release.platform_package_files("windows"),
            ("zig-out/bin/worldvm.exe", "zig-out/bin/worldvm.dll"),
        )

    def test_unsupported_platform_package_files_raise(self):
        with self.assertRaises(RuntimeError):
            package_release.platform_package_files("freebsd")


if __name__ == "__main__":
    unittest.main()
