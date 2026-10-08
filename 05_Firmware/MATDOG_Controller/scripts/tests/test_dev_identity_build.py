#!/usr/bin/env python3
"""Offline development identity/build isolation tests; no device or Git writes.

The build invocation runs against an external copied sketch and fake Git/CLI.
This also works in the calibration mutation fixture, which has no .git or docs.
"""
import hashlib
import json
import os
import shutil
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

SCRIPTS = Path(__file__).resolve().parent.parent
SKETCH = SCRIPTS.parent
sys.path.insert(0, str(SCRIPTS))
import build_manifest as bm  # noqa: E402
import matdog_layout as layout  # noqa: E402

HEAD = "aee49bf306f88d56562634aeba08b767f92b247c"
IDENTITY = {"FW_VERSION": "0.2.0-dev.1", "BUILD_UTC": "2026-10-05T00:00:00Z",
            "BUILD_UTC_POLICY": "SOURCE_DATE_EPOCH", "CAL_RECORD_SCHEMA": "1",
            "CAL_MARKER_SCHEMA": "2", "MOTION_STACK": "G1_G5A_COMPILED_UNWIRED"}


def file_hashes(root):
    return {str(path.relative_to(root)): hashlib.sha256(path.read_bytes()).hexdigest()
            for path in root.rglob("*") if path.is_file()}


class IdentityTests(unittest.TestCase):
    def fields(self):
        return bm.parse_manifest(bm.render_manifest(
            source_commit=HEAD, build_id=HEAD[:12], source_state="CLEAN",
            profile="USB_ONLY", ota_ingest_enabled="0", fqbn=layout.PINNED_FQBN,
            application_binary="MATDOG_Controller.ino.bin", application_size=100,
            application_sha256="a" * 64, layout_id=layout.LAYOUT_ID,
            partition_table_sha256=layout.EXPECTED_TABLE_SHA256,
            app_partition_size=layout.APP_SLOT_SIZE, identity=IDENTITY))

    def verdict(self, fields):
        return bm.verify_manifest(fields, head_commit=HEAD, expected_fqbn=layout.PINNED_FQBN,
                                  tree_state="CLEAN", binary_exists=True, binary_size=100,
                                  binary_sha256="a" * 64, requested_profile="USB_ONLY",
                                  requested_ota_ingest="0",
                                  build_partition_table_sha256=layout.EXPECTED_TABLE_SHA256,
                                  binary_embeds_layout_id=True)

    def test_complete_identity_and_original_gate(self):
        fields = self.fields()
        self.assertTrue(self.verdict(fields).ok)
        self.assertEqual(fields["GIT_SHA"], HEAD)
        self.assertEqual(fields["MOTION_AUTHORIZED"], "0")
        fields["SOURCE_STATE"] = "DIRTY"
        self.assertEqual(self.verdict(fields).reason, bm.Refusal.TREE_NOT_CLEAN)

    def test_legacy_v2_remains_compatible(self):
        fields = self.fields()
        for key in bm.IDENTITY_KEYS:
            del fields[key]
        self.assertTrue(self.verdict(fields).ok)

    def test_each_identity_alias_contradiction_and_partial_group_refused(self):
        for key in bm.IDENTITY_KEYS:
            with self.subTest(key=key):
                fields = self.fields()
                fields[key] = "contradiction"
                self.assertEqual(self.verdict(fields).reason, bm.Refusal.IDENTITY_MISMATCH)
                del fields[key]
                self.assertEqual(self.verdict(fields).reason, bm.Refusal.IDENTITY_MISMATCH)

    def test_incomplete_or_multiline_writer_identity_refused(self):
        fields = self.fields()
        for change in ({"CAL_MARKER_SCHEMA": "1"}, {"BUILD_UTC": "2026-10-05T00:00:00Z\nBAD=1"},
                       {"BUILD_UTC": "2026-02-30T00:00:00Z"}, {"FW_VERSION": "0.2.0-dev.01"}):
            bad = dict(fields, **change)
            self.assertIsNotNone(bm.identity_problem(
                bad, source_commit=HEAD, source_state="CLEAN", layout_id=layout.LAYOUT_ID,
                ota_ingest="0", application_sha256="a" * 64))


class ExternalBuildTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix="matdog-dev-identity-")
        self.base = Path(self.temp.name)
        self.repo = self.base / "repository"
        self.sketch = self.repo / "05_Firmware/MATDOG_Controller"
        self.tools = self.base / "tools"
        self.tools.mkdir()
        for name in ("scripts/build.sh", "scripts/build_manifest.py", "scripts/matdog_layout.py",
                     "scripts/ota_partition_logic.py",
                     "src/config/BuildConfig.h", "src/config/HardwareProfile.h", "partitions.csv"):
            target = self.sketch / name
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(SKETCH / name, target)
        self.common = self.base / "git-common"
        self.common.mkdir()
        data = self.base / "offline-data"
        (data / "packages/esp32/hardware/esp32/3.3.11").mkdir(parents=True)
        (data / "package_esp32_index.json").write_text('{"packages": []}')
        self.libraries = self.base / "offline-libraries"
        self.libraries.mkdir()
        self.table = self.base / "partitions.bin"
        generator = Path.home() / ".arduino15/packages/esp32/hardware/esp32/3.3.11/tools/gen_esp32part.py"
        subprocess.run([sys.executable, str(generator), "-q", str(self.sketch / "partitions.csv"),
                        str(self.table)], check=True, capture_output=True)
        self.assertEqual(bm.sha256_file(self.table), layout.EXPECTED_TABLE_SHA256)
        git = self.tools / "git"
        git.write_text('''#!/usr/bin/env python3
import os, sys
a = sys.argv[1:]
if a[:1] == ["-C"]: a = a[2:]
if a == ["rev-parse", "HEAD"]: print(os.environ["FAKE_HEAD"])
elif a == ["status", "--porcelain"]: print(os.environ.get("FAKE_STATUS", ""), end="")
elif a == ["worktree", "list", "--porcelain"]:
    print("worktree " + os.environ["FAKE_REPO"])
    if os.environ.get("FAKE_OTHER_WORKTREE"): print("worktree " + os.environ["FAKE_OTHER_WORKTREE"])
elif a == ["rev-parse", "--git-common-dir"]: print(os.environ["FAKE_COMMON"])
else: raise SystemExit("unexpected Git invocation: " + repr(a))
''')
        cli = self.tools / "arduino-cli"
        cli.write_text('''#!/usr/bin/env python3
import json, os, re, shutil, sys
from pathlib import Path
a = sys.argv[1:]
Path(os.environ["FAKE_CAPTURE"]).write_text(json.dumps(a))
if os.environ.get("FAKE_FAIL"): raise SystemExit(7)
assert "compile" in a and "--export-binaries" not in a and "--output-dir" not in a
config = json.loads(Path(a[a.index("--config-file") + 1]).read_text())
root = Path(os.environ["MATDOG_BUILD_ROOT"]).resolve()
for value in config["directories"].values(): assert Path(value).resolve().is_relative_to(root)
assert Path(config["build_cache"]["path"]).resolve().is_relative_to(root)
assert Path(os.environ["TMPDIR"]).resolve().is_relative_to(root)
assert Path(os.environ["ARDUINO_BUILD_CACHE_PATH"]).resolve().is_relative_to(root)
out = Path(a[a.index("--build-path") + 1])
flags = next(value for value in a if value.startswith("compiler.cpp.extra_flags="))
def macro(name):
    m = re.search(r"-D" + name + r'=\"([^\"]+)\"', flags)
    assert m, (name, flags)
    return m.group(1)
payload = ("MATDOG_LAYOUT_ID=MATDOG_16M_2x5M_NVS_V1\\0" + "0.2.0-dev.1\\0" +
           macro("MATDOG_GIT_SHA") + "\\0" + macro("MATDOG_BUILD_UTC") + "\\0" +
           "G1_G5A_COMPILED_UNWIRED\\0" + flags).encode()
(out / "MATDOG_Controller.ino.bin").write_bytes(payload)
shutil.copyfile(os.environ["FAKE_TABLE"], out / "MATDOG_Controller.ino.partitions.bin")
(out / "MATDOG_Controller.ino.elf").write_bytes(b"fake ELF")
''')
        git.chmod(0o755)
        cli.chmod(0o755)
        self.root = self.base / "external"
        self.capture = self.base / "cli-arguments.json"
        self.env = dict(os.environ, PATH=str(self.tools) + os.pathsep + os.environ["PATH"],
                        ARDUINO_CLI=str(cli), MATDOG_BUILD_ROOT=str(self.root),
                        MATDOG_ARDUINO_DATA_SOURCE=str(data),
                        MATDOG_ARDUINO_LIBRARY_ROOT=str(self.libraries), FAKE_HEAD=HEAD,
                        FAKE_REPO=str(self.repo), FAKE_COMMON=str(self.common),
                        FAKE_CAPTURE=str(self.capture), FAKE_TABLE=str(self.table),
                        SOURCE_DATE_EPOCH="1791158400", PYTHONDONTWRITEBYTECODE="1")
        for name in ("MATDOG_BUILD_DIR", "MATDOG_OUTPUT_DIR", "MATDOG_BUILD_CACHE_DIR",
                     "MATDOG_BUILD_TMP_DIR", "MATDOG_PROFILE", "MATDOG_OTA_INGEST_VALIDATION",
                     "FAKE_STATUS", "FAKE_FAIL", "FAKE_OTHER_WORKTREE"):
            self.env.pop(name, None)
        self.before = file_hashes(self.repo)

    def tearDown(self):
        self.assertEqual(self.before, file_hashes(self.repo), "source fixture changed")
        self.temp.cleanup()

    def run_build(self, *args, **changes):
        env = dict(self.env, **changes)
        return subprocess.run(["bash", str(self.sketch / "scripts/build.sh"), *args],
                              env=env, capture_output=True, text=True)

    def manifest(self, profile="USB_ONLY"):
        return bm.parse_manifest((self.root / "artifacts" / (profile + "-ota0") /
                                  bm.MANIFEST_FILENAME).read_text())

    def test_default_usb_ota_zero_full_identity_and_external_paths(self):
        result = self.run_build()
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        fields = self.manifest()
        self.assertEqual(fields["FW_VERSION"], "0.2.0-dev.1")
        self.assertEqual(fields["GIT_SHA"], HEAD)
        self.assertEqual(fields["GIT_DIRTY"], "0")
        self.assertEqual(fields["BUILD_ID"], HEAD[:12])
        self.assertEqual(fields["HARDWARE_PROFILE"], "USB_ONLY")
        self.assertEqual(fields["OTA_INGEST"], "0")
        self.assertEqual(fields["MOTION_AUTHORIZED"], "0")
        self.assertEqual(fields["BUILD_UTC"], IDENTITY["BUILD_UTC"])
        binary = self.root / "artifacts/USB_ONLY-ota0/MATDOG_Controller.ino.bin"
        self.assertEqual(fields["APP_SHA256"], bm.sha256_file(binary))
        args = json.loads(self.capture.read_text())
        self.assertNotIn("--export-binaries", args)
        self.assertNotIn("--output-dir", args)
        self.assertIn("--clean", args)
        self.assertTrue((self.root / "arduino/data/packages").is_symlink())

    def test_untracked_material_marks_dirty_and_profile_outputs_do_not_overwrite(self):
        first = self.run_build(FAKE_STATUS="?? migration.py\\n")
        self.assertEqual(first.returncode, 0, first.stdout + first.stderr)
        fields = self.manifest()
        self.assertEqual(fields["GIT_DIRTY"], "1")
        self.assertEqual(fields["SOURCE_STATE"], "DIRTY")
        self.assertEqual(fields["BUILD_ID"], HEAD[:12] + "-dirty")
        first_hashes = file_hashes(self.root / "artifacts/USB_ONLY-ota0")
        second = self.run_build("--jobs", "2", MATDOG_PROFILE="ROBOT_POWERED")
        self.assertEqual(second.returncode, 0, second.stdout + second.stderr)
        self.assertEqual(self.manifest("ROBOT_POWERED")["HARDWARE_PROFILE"], "ROBOT_POWERED")
        self.assertEqual(first_hashes, file_hashes(self.root / "artifacts/USB_ONLY-ota0"))

    def test_failed_compile_invalidates_stale_manifest_and_application(self):
        self.assertEqual(self.run_build().returncode, 0)
        self.assertEqual(self.run_build(FAKE_FAIL="1").returncode, 7)
        output = self.root / "artifacts/USB_ONLY-ota0"
        self.assertFalse((output / bm.MANIFEST_FILENAME).exists())
        self.assertFalse((output / "MATDOG_Controller.ino.bin").exists())

    def test_protected_paths_symlink_escape_and_argument_overrides_refused(self):
        for changes in ({"MATDOG_BUILD_ROOT": str(self.repo / "build")},
                        {"MATDOG_BUILD_ROOT": str(self.common / "build")},
                        {"MATDOG_BUILD_ROOT": str(self.base)},
                        {"MATDOG_OUTPUT_DIR": str(self.repo / "build")},
                        {"MATDOG_BUILD_ROOT": str(self.root), "FAKE_OTHER_WORKTREE": str(self.root)},
                        {"MATDOG_BUILD_ROOT": "relative/path"}):
            with self.subTest(changes=changes):
                result = self.run_build(**changes)
                self.assertNotEqual(result.returncode, 0)
                self.assertFalse(self.capture.exists())
        self.root.mkdir()
        (self.root / "escape").symlink_to(self.repo, target_is_directory=True)
        self.assertNotEqual(self.run_build(MATDOG_OUTPUT_DIR=str(self.root / "escape/output")).returncode, 0)
        for args in (("--export-binaries",), ("--output-dir", str(self.repo)),
                     ("--upload",), ("--port", "/dev/ttyFAKE"), ("--config-file", str(self.repo))):
            self.assertNotEqual(self.run_build(*args).returncode, 0)
        self.assertFalse(self.capture.exists())

    def test_invalid_epoch_and_profile_refused_before_cli(self):
        for changes in ({"SOURCE_DATE_EPOCH": "-1"}, {"SOURCE_DATE_EPOCH": "invalid"},
                        {"MATDOG_PROFILE": "typo"}, {"MATDOG_OTA_INGEST_VALIDATION": "2"}):
            self.assertNotEqual(self.run_build(**changes).returncode, 0)
        self.assertFalse(self.capture.exists())


if __name__ == "__main__":
    unittest.main()
