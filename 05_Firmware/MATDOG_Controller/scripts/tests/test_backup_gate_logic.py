#!/usr/bin/env python3
"""Offline tests for backup_gate_logic.py — no device I/O, no real backup
file required. Run directly:

    python3 scripts/tests/test_backup_gate_logic.py

Covers the 2026-09-25 recovery-backup hardening: flash_app_only.sh's backup
gate must accept the ONE historical default backup exactly as before, and
must accept a fresh CUSTOM backup only together with an explicitly
authorized expected SHA256 (stated directly, or read from a companion
recovery manifest) — never by path or size alone.
"""
import contextlib
import io
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from backup_gate_logic import (  # noqa: E402
    DEFAULT_BACKUP_SHA256,
    EXPECTED_BACKUP_SIZE,
    Refusal,
    main,
    read_manifest_sha256,
    verify_backup,
)

DEFAULT_PATH = "/home/x/MATDOG/backups/esp32/matdog_esp32s3_fullflash_2026-09-10.bin"
CUSTOM_PATH = "/home/x/MATDOG/backups/esp32/matdog_esp32s3_fullflash_2026-09-25_205220_nostub.bin"
CUSTOM_SHA = "8758be21fea080c7a824315cd6c3df77d76de420f99a4265c3490a129b7bfe90"


def verify(*, is_default_backup=False, actual_size=EXPECTED_BACKUP_SIZE,
           actual_sha256=DEFAULT_BACKUP_SHA256, custom_expected_sha256=None,
           manifest_sha256=None):
    return verify_backup(
        is_default_backup=is_default_backup,
        actual_size=actual_size,
        actual_sha256=actual_sha256,
        custom_expected_sha256=custom_expected_sha256,
        manifest_sha256=manifest_sha256,
    )


class TestHistoricalDefaultUnchanged(unittest.TestCase):
    def test_historical_default_accepted(self):
        v = verify(is_default_backup=True, actual_sha256=DEFAULT_BACKUP_SHA256)
        self.assertTrue(v.ok, v.detail)
        self.assertEqual(v.expected_sha256, DEFAULT_BACKUP_SHA256)

    def test_historical_default_ignores_a_supplied_custom_hash(self):
        # is_default_backup=True is authoritative on its own — a stray
        # MATDOG_FLASH_BACKUP_SHA256 left set in the environment must not
        # override the pinned historical hash for the default path.
        v = verify(is_default_backup=True, actual_sha256=DEFAULT_BACKUP_SHA256,
                   custom_expected_sha256="0" * 64)
        self.assertTrue(v.ok, v.detail)
        self.assertEqual(v.expected_sha256, DEFAULT_BACKUP_SHA256)

    def test_historical_default_wrong_hash_still_refuses(self):
        v = verify(is_default_backup=True, actual_sha256="a" * 64)
        self.assertFalse(v.ok)
        self.assertEqual(v.reason, Refusal.SHA256_MISMATCH)


class TestCustomBackupAuthorization(unittest.TestCase):
    def test_custom_backup_with_correct_explicit_hash_accepted(self):
        v = verify(is_default_backup=False, actual_sha256=CUSTOM_SHA,
                   custom_expected_sha256=CUSTOM_SHA)
        self.assertTrue(v.ok, v.detail)
        self.assertEqual(v.expected_sha256, CUSTOM_SHA)

    def test_custom_backup_with_correct_manifest_hash_accepted(self):
        v = verify(is_default_backup=False, actual_sha256=CUSTOM_SHA,
                   manifest_sha256=CUSTOM_SHA)
        self.assertTrue(v.ok, v.detail)
        self.assertEqual(v.expected_sha256, CUSTOM_SHA)

    def test_explicit_hash_takes_priority_over_manifest(self):
        # Both supplied, both correct — no ambiguity, but confirms the
        # explicit env var is checked first per the documented precedence.
        v = verify(is_default_backup=False, actual_sha256=CUSTOM_SHA,
                   custom_expected_sha256=CUSTOM_SHA, manifest_sha256="f" * 64)
        self.assertTrue(v.ok, v.detail)
        self.assertEqual(v.expected_sha256, CUSTOM_SHA)

    def test_custom_backup_with_no_expected_hash_refused(self):
        # THE case this hardening exists for: a custom backup is never
        # accepted by path/size alone.
        v = verify(is_default_backup=False, actual_sha256=CUSTOM_SHA)
        self.assertFalse(v.ok)
        self.assertEqual(v.reason, Refusal.NO_EXPECTED_HASH)

    def test_custom_backup_with_empty_string_hash_refused(self):
        # An empty string (e.g. an unset shell variable expanding to "")
        # must be treated as "not supplied", not as a hash to compare
        # against.
        v = verify(is_default_backup=False, actual_sha256=CUSTOM_SHA,
                   custom_expected_sha256="", manifest_sha256="")
        self.assertFalse(v.ok)
        self.assertEqual(v.reason, Refusal.NO_EXPECTED_HASH)

    def test_custom_backup_wrong_explicit_hash_refused(self):
        v = verify(is_default_backup=False, actual_sha256=CUSTOM_SHA,
                   custom_expected_sha256="b" * 64)
        self.assertFalse(v.ok)
        self.assertEqual(v.reason, Refusal.SHA256_MISMATCH)

    def test_custom_backup_wrong_manifest_hash_refused(self):
        v = verify(is_default_backup=False, actual_sha256=CUSTOM_SHA,
                   manifest_sha256="c" * 64)
        self.assertFalse(v.ok)
        self.assertEqual(v.reason, Refusal.SHA256_MISMATCH)

    def test_hash_comparison_is_case_insensitive(self):
        v = verify(is_default_backup=False, actual_sha256=CUSTOM_SHA.upper(),
                   custom_expected_sha256=CUSTOM_SHA)
        self.assertTrue(v.ok, v.detail)


class TestSizeGate(unittest.TestCase):
    def test_wrong_size_refused_even_with_correct_hash(self):
        # Size is checked BEFORE authorization: a truncated/extended file
        # is refused regardless of whether its (now-meaningless) hash
        # would otherwise match.
        v = verify(is_default_backup=True, actual_size=EXPECTED_BACKUP_SIZE - 1,
                   actual_sha256=DEFAULT_BACKUP_SHA256)
        self.assertFalse(v.ok)
        self.assertEqual(v.reason, Refusal.SIZE_MISMATCH)

    def test_wrong_size_refused_for_custom_backup_too(self):
        v = verify(is_default_backup=False, actual_size=EXPECTED_BACKUP_SIZE + 4096,
                   actual_sha256=CUSTOM_SHA, custom_expected_sha256=CUSTOM_SHA)
        self.assertFalse(v.ok)
        self.assertEqual(v.reason, Refusal.SIZE_MISMATCH)

    def test_zero_size_refused(self):
        v = verify(is_default_backup=True, actual_size=0, actual_sha256=DEFAULT_BACKUP_SHA256)
        self.assertFalse(v.ok)
        self.assertEqual(v.reason, Refusal.SIZE_MISMATCH)


class TestManifestReading(unittest.TestCase):
    def test_empty_path_returns_none(self):
        self.assertIsNone(read_manifest_sha256(""))

    def test_missing_file_returns_none(self):
        self.assertIsNone(read_manifest_sha256("/nonexistent/path/to/nothing.manifest.txt"))

    def test_reads_the_backup_sha256_line(self):
        with tempfile.TemporaryDirectory() as tmp:
            p = Path(tmp) / "m.manifest.txt"
            p.write_text("MATDOG_RECOVERY_MANIFEST_VERSION=1\n"
                         f"BACKUP_SHA256={CUSTOM_SHA}\n"
                         "OTHER_FIELD=irrelevant\n")
            self.assertEqual(read_manifest_sha256(str(p)), CUSTOM_SHA)

    def test_no_backup_sha256_line_returns_none(self):
        with tempfile.TemporaryDirectory() as tmp:
            p = Path(tmp) / "m.manifest.txt"
            p.write_text("MATDOG_RECOVERY_MANIFEST_VERSION=1\nOTHER_FIELD=irrelevant\n")
            self.assertIsNone(read_manifest_sha256(str(p)))

    def test_empty_backup_sha256_value_returns_none(self):
        with tempfile.TemporaryDirectory() as tmp:
            p = Path(tmp) / "m.manifest.txt"
            p.write_text("BACKUP_SHA256=\n")
            self.assertIsNone(read_manifest_sha256(str(p)))

    def test_first_matching_line_wins(self):
        with tempfile.TemporaryDirectory() as tmp:
            p = Path(tmp) / "m.manifest.txt"
            p.write_text(f"BACKUP_SHA256={CUSTOM_SHA}\nBACKUP_SHA256={'z' * 64}\n")
            self.assertEqual(read_manifest_sha256(str(p)), CUSTOM_SHA)


class TestCliEndToEnd(unittest.TestCase):
    """Exercises the real CLI against real files, which is what
    flash_app_only.sh actually invokes."""

    @staticmethod
    def _quiet(fn, *args, **kwargs):
        with contextlib.redirect_stdout(io.StringIO()):
            return fn(*args, **kwargs)

    def test_default_backup_via_cli(self):
        rc = self._quiet(main, ["--backup-path", DEFAULT_PATH,
                                "--default-backup-path", DEFAULT_PATH,
                                "--actual-size", str(EXPECTED_BACKUP_SIZE),
                                "--actual-sha256", DEFAULT_BACKUP_SHA256])
        self.assertEqual(rc, 0)

    def test_custom_backup_with_no_hash_refused_via_cli(self):
        rc = self._quiet(main, ["--backup-path", CUSTOM_PATH,
                                "--default-backup-path", DEFAULT_PATH,
                                "--actual-size", str(EXPECTED_BACKUP_SIZE),
                                "--actual-sha256", CUSTOM_SHA])
        self.assertEqual(rc, 1)

    def test_custom_backup_with_explicit_hash_accepted_via_cli(self):
        rc = self._quiet(main, ["--backup-path", CUSTOM_PATH,
                                "--default-backup-path", DEFAULT_PATH,
                                "--actual-size", str(EXPECTED_BACKUP_SIZE),
                                "--actual-sha256", CUSTOM_SHA,
                                "--custom-expected-sha256", CUSTOM_SHA])
        self.assertEqual(rc, 0)

    def test_custom_backup_with_manifest_file_accepted_via_cli(self):
        with tempfile.TemporaryDirectory() as tmp:
            manifest_path = Path(tmp) / "backup.manifest.txt"
            manifest_path.write_text(f"BACKUP_SHA256={CUSTOM_SHA}\n")
            rc = self._quiet(main, ["--backup-path", CUSTOM_PATH,
                                    "--default-backup-path", DEFAULT_PATH,
                                    "--actual-size", str(EXPECTED_BACKUP_SIZE),
                                    "--actual-sha256", CUSTOM_SHA,
                                    "--manifest-path", str(manifest_path)])
            self.assertEqual(rc, 0)

    def test_custom_backup_with_wrong_manifest_hash_refused_via_cli(self):
        with tempfile.TemporaryDirectory() as tmp:
            manifest_path = Path(tmp) / "backup.manifest.txt"
            manifest_path.write_text(f"BACKUP_SHA256={'d' * 64}\n")
            rc = self._quiet(main, ["--backup-path", CUSTOM_PATH,
                                    "--default-backup-path", DEFAULT_PATH,
                                    "--actual-size", str(EXPECTED_BACKUP_SIZE),
                                    "--actual-sha256", CUSTOM_SHA,
                                    "--manifest-path", str(manifest_path)])
            self.assertEqual(rc, 1)


if __name__ == "__main__":
    unittest.main()
