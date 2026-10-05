"""Build real ZIPs in temporary directories and verify exact artifact members."""
import contextlib
import hashlib
import importlib.util
import io
from pathlib import Path
import shutil
import tempfile
import unittest
from unittest.mock import patch
import zipfile

BUILD_SCRIPT = Path(__file__).resolve().parents[2] / "build_packages.py"
spec = importlib.util.spec_from_file_location("package_builder_under_test", BUILD_SCRIPT)
builder = importlib.util.module_from_spec(spec)
spec.loader.exec_module(builder)


class PackageBuildTests(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory(prefix="maix-package-test-")
        self.addCleanup(temporary.cleanup)
        self.root = Path(temporary.name)
        self.app = self.root / "qr_object_app"
        self.app.mkdir()
        (self.app / "tests").mkdir()
        self.names = ["app.yaml", "config.py", "main.py", "model_fixture.mud", "model_fixture.cvimodel"]
        self.write_manifest()
        (self.app / "config.py").write_text('MODEL_FILE = "model_fixture.mud"\n', encoding="utf-8")
        (self.app / "main.py").write_text('print("fixture")\n', encoding="utf-8")
        (self.app / "model_fixture.mud").write_text('[basic]\nmodel=model_fixture.cvimodel\n', encoding="utf-8")
        (self.app / "model_fixture.cvimodel").write_bytes(b"fixture model\x00\x01")
        (self.app / "README.md").write_text("Source-only instructions\n", encoding="utf-8")
        (self.app / "tests" / "test_probe.py").write_text('assert True\n', encoding="utf-8")
        (self.root / ".gitignore").write_text("dist/\n", encoding="utf-8")
        shutil.copyfile(BUILD_SCRIPT, self.root / "build_packages.py")
        # Unlisted stale models must never leak into either archive.
        (self.app / "model_9541.mud").write_text("old model", encoding="utf-8")
        (self.app / "model_9541.cvimodel").write_bytes(b"old binary")
        root_patch = patch.multiple(builder, ROOT=self.root, APP=self.app)
        root_patch.start()
        self.addCleanup(root_patch.stop)
        output = contextlib.redirect_stdout(io.StringIO())
        output.__enter__()
        self.addCleanup(output.__exit__, None, None, None)

    def write_manifest(self, app_id="fixture_switch", version="9.8.7"):
        text = "id: {}\nversion: {}\nfiles:\n".format(app_id, version)
        text += "".join("  - " + name + "\n" for name in self.names)
        (self.app / "app.yaml").write_text(text, encoding="utf-8")

    def test_zip_names_members_bytes_and_checksums_follow_current_configuration(self):
        builder.main()
        app_zip = self.root / "dist" / "maix-fixture_switch-v9.8.7.zip"
        source_zip = self.root / "dist" / "fixture_switch_source_v9.8.7.zip"
        with zipfile.ZipFile(app_zip) as archive:
            self.assertEqual(archive.namelist(), self.names)
            for name in self.names:
                self.assertEqual(archive.read(name), (self.app / name).read_bytes())
        expected_source = [".gitignore", "build_packages.py"]
        expected_source += ["qr_object_app/" + name for name in self.names + ["README.md"]]
        expected_source += ["qr_object_app/tests/test_probe.py"]
        with zipfile.ZipFile(source_zip) as archive:
            self.assertEqual(archive.namelist(), expected_source)
            for name in expected_source:
                self.assertEqual(archive.read(name), (self.root / name).read_bytes())
        checksums = (self.root / "dist" / "SHA256SUMS.txt").read_text(encoding="utf-8").splitlines()
        self.assertEqual(checksums, [hashlib.sha256(path.read_bytes()).hexdigest() + "  " + path.name
            for path in (app_zip, source_zip)])

    def test_unlisted_configured_model_fails_before_creating_dist(self):
        (self.app / "config.py").write_text('MODEL_FILE = "model_9541.mud"\n', encoding="utf-8")
        with self.assertRaisesRegex(ValueError, "MODEL_FILE"):
            builder.main()
        self.assertFalse((self.root / "dist").exists())

    def test_unlisted_mud_binary_fails_before_creating_dist(self):
        (self.app / "model_fixture.mud").write_text('[basic]\nmodel=model_9541.cvimodel\n', encoding="utf-8")
        with self.assertRaisesRegex(ValueError, "MUD model"):
            builder.main()
        self.assertFalse((self.root / "dist").exists())

    def test_missing_manifest_file_fails_before_creating_dist(self):
        self.names.append("missing.cvimodel")
        self.write_manifest()
        with self.assertRaisesRegex(ValueError, "missing app file"):
            builder.main()
        self.assertFalse((self.root / "dist").exists())

    def test_duplicate_manifest_member_is_rejected(self):
        self.names.append("config.py")
        self.write_manifest()
        with self.assertRaisesRegex(ValueError, "duplicate app files"):
            builder.main()

    def test_metadata_must_include_a_safe_version(self):
        for version in ("", "../9.8.7"):
            with self.subTest(version=version):
                self.write_manifest(version=version)
                with self.assertRaisesRegex(ValueError, "app version"):
                    builder.main()
        self.assertFalse((self.root / "dist").exists())

    def test_readme_in_manifest_is_not_duplicated_in_source_archive(self):
        self.names.append("README.md")
        self.write_manifest()
        builder.main()
        with zipfile.ZipFile(self.root / "dist" / "fixture_switch_source_v9.8.7.zip") as archive:
            self.assertEqual(archive.namelist().count("qr_object_app/README.md"), 1)

    def test_current_project_manifest_contains_both_configured_model_files(self):
        actual_root = BUILD_SCRIPT.parent
        with patch.multiple(builder, ROOT=actual_root, APP=actual_root / "qr_object_app"):
            metadata = builder.app_metadata()
            names = builder.manifest()
            model_file, binary_file = builder.validate_model(names)
        self.assertTrue(metadata["version"])
        self.assertIn(model_file, names)
        self.assertIn(binary_file, names)


if __name__ == "__main__":
    unittest.main(verbosity=2)
