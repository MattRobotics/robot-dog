#!/usr/bin/env python3
"""Synthetic M0 failures and byte-exact readback checks; no hardware access.

Synthetic images are validator inputs, not bootable or flashable firmware.
Only the synthetic bootloader digest is injected for the pure logic tests.
The CLI's real bootloader digest and the layout digests have no override.
"""
import hashlib
import contextlib
import io
import json
import struct
import subprocess
import sys
import tempfile
import unittest
import zlib
from pathlib import Path
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
import build_manifest as manifest
import matdog_layout as layout
import migration_m0 as m0
import ota_partition_logic as ota

SDKCONFIG = "CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE=y\n# CONFIG_BOOTLOADER_APP_ANTI_ROLLBACK is not set\n"
LEGACY_ROWS = (
    ("nvs", 1, 2, 0x9000, 0x5000), ("otadata", 1, 0, 0xE000, 0x2000),
    ("app0", 0, 0x10, 0x10000, 0x300000), ("app1", 0, 0x11, 0x310000, 0x300000),
    ("ffat", 1, 0x81, 0x610000, 0x9E0000), ("coredump", 1, 3, 0xFF0000, 0x10000),
)


def table(rows):
    entries = b"".join(struct.pack("<HBBII16sI", 0x50AA, t, s, o, n, label.encode(), 0)
                       for label, t, s, o, n in rows)
    md5 = b"\xeb\xeb" + b"\xff" * 14 + hashlib.md5(entries).digest()
    return (entries + md5).ljust(0xC00, b"\xff")


def otadata(seq=1, state=ota.OTA_STATE_UNDEFINED, second=None):
    data = bytearray(b"\xff" * 0x2000)
    for sector, (n, st) in enumerate([(seq, state)] + ([second] if second else [])):
        off = sector * 0x1000
        struct.pack_into("<I", data, off, n)
        struct.pack_into("<I", data, off + 24, st)
        struct.pack_into("<I", data, off + 28, zlib.crc32(struct.pack("<I", n), 0xFFFFFFFF))
    return bytes(data)


class MigrationM0Tests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.legacy = table(LEGACY_ROWS)
        cls.target = table([(e.label, e.type, e.subtype, e.offset, e.size) for e in layout.EXPECTED_PARTITIONS])
        assert m0.digest(cls.legacy) == layout.LEGACY_TABLE_SHA256
        assert m0.digest(cls.target) == layout.EXPECTED_TABLE_SHA256
        cls.boot = b"\xe9" + b"synthetic-bootloader".ljust(m0.BOOTLOADER_SIZE - 1, b"\xff")
        b = bytearray(b"\xff" * layout.FLASH_SIZE_BYTES)
        b[:len(cls.boot)] = cls.boot
        b[0x8000:0x8C00] = cls.legacy
        nvs = b"preserved-nvs!!!\0"
        b[0x9000:0x9000 + len(nvs)] = nvs
        b[0xE000:0x10000] = otadata()
        app = b"\xe9installed-build\0"
        b[0x10000:0x10000 + len(app)] = app
        assert len(b) == layout.FLASH_SIZE_BYTES
        cls.before = bytes(b)

    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name)
        self.binary = self.root / "MATDOG_Controller.ino.bin"
        self.binary.write_bytes(b"\xe9" + m0.APPROVED_COMMIT[:12].encode() + b"\0" +
                                layout.LAYOUT_MARKER.encode() + b"\0synthetic-validator-image")
        manifest.partition_table_path_for(self.binary).write_bytes(self.target)
        self.mfile = self.root / "matdog_build_manifest.txt"
        self.write_manifest()

    def write_manifest(self, **changes):
        values = dict(source_commit=m0.APPROVED_COMMIT, build_id=m0.APPROVED_COMMIT[:12],
                      source_state="CLEAN", profile="USB_ONLY", ota_ingest_enabled="0",
                      fqbn=layout.PINNED_FQBN, application_binary=self.binary.name,
                      application_size=self.binary.stat().st_size,
                      application_sha256=m0.digest(self.binary.read_bytes()),
                      layout_id=layout.LAYOUT_ID, partition_table_sha256=layout.EXPECTED_TABLE_SHA256,
                      app_partition_size=layout.APP_SLOT_SIZE)
        values.update(changes)
        self.mfile.write_text(manifest.render_manifest(**values))

    def stop(self, code, fn, *args, **kwargs):
        with self.assertRaises((m0.Stop, layout.LayoutRefusal)) as caught:
            fn(*args, **kwargs)
        self.assertEqual(caught.exception.code, code)

    def inspect(self, image=None, **changes):
        values = dict(sdkconfig=SDKCONFIG, bootloader=self.boot, running_offset=0x10000,
                      installed_build_id="installed-build")
        values.update(changes)
        with patch.object(m0, "BOOTLOADER_SHA256", m0.digest(self.boot)):
            return m0.inspect_legacy(self.before if image is None else image, **values)

    def changed(self, offset, raw):
        b = bytearray(self.before)
        b[offset:offset + len(raw)] = raw
        return bytes(b)

    def backup_files(self, repeat=None):
        a, b = self.root / "read-a.bin", self.root / "read-b.bin"
        a.write_bytes(self.before)
        b.write_bytes(self.before if repeat is None else repeat)
        return a, b

    def artifact(self):
        return m0.verified_artifact(self.binary, self.mfile)

    def test_correct_legacy_selects_app0_without_otadata_or_nvs_write(self):
        r = self.inspect()
        self.assertEqual(r["selected_slot"], 0)
        self.assertEqual(r["otadata_action"], "PRESERVE")
        self.assertEqual(r["matdog_nvs_action"], "PRESERVE_ERASED")

    def test_unexpected_layout(self):
        self.stop("INSTALLED_LAYOUT_UNEXPECTED", self.inspect, self.changed(0x8000, self.target))

    def test_corrupt_legacy_table_padding(self):
        self.stop("INSTALLED_LAYOUT_UNEXPECTED", self.inspect, self.changed(0x8B00, b"\0"))

    def test_selected_slot1_is_not_silently_mapped_to_app0(self):
        self.stop("ACTIVE_SLOT_UNEXPECTED", self.inspect, self.changed(0xE000, otadata(seq=2)))

    def test_running_slot_disagrees_with_otadata(self):
        self.stop("RUNNING_SLOT_MISMATCH", self.inspect, running_offset=0x310000)

    def test_target_nvs_dirty_single_byte(self):
        self.stop("DESTINATION_NVS_NOT_EMPTY", self.inspect, self.changed(0xFEFFFF, b"\0"))

    def test_ffat_data_outside_nvs_is_not_ignored(self):
        self.stop("FFAT_NOT_EMPTY", self.inspect, self.changed(0xA10001, b"\0"))

    def test_new_app1_stale_legacy_tail(self):
        self.stop("NEW_APP1_NOT_EMPTY", self.inspect, self.changed(0x510000, b"\xe9"))

    def test_legacy_app1_data_hidden_by_app0_expansion(self):
        for offset in (0x310000, 0x50FFFF):
            self.stop("LEGACY_APP1_NOT_EMPTY", self.inspect, self.changed(offset, b"\0"))

    def test_table_sector_erase_must_not_destroy_occupied_padding(self):
        self.stop("TABLE_SECTOR_PADDING_OCCUPIED", self.inspect, self.changed(0x8C00, b"\0"))

    def test_powered_profile_must_be_explicit_and_matching(self):
        self.write_manifest(profile="ROBOT_POWERED")
        self.stop("PROFILE_MISMATCH", self.artifact)
        _, _, m, plan = m0.verified_artifact(self.binary, self.mfile, expected_profile="ROBOT_POWERED")
        self.assertEqual(m["HARDWARE_PROFILE"], "ROBOT_POWERED")
        self.assertEqual(plan.start, 0x10000)
        self.stop("PROFILE_UNSUPPORTED", m0.verified_artifact, self.binary, self.mfile,
                  expected_profile="ANY")

    def test_powered_cli_plan_and_both_snapshot_stages(self):
        self.write_manifest(profile="ROBOT_POWERED")
        out = self.root / "powered-plan"
        args = self.plan_args(out) + ["--profile", "ROBOT_POWERED"]
        with patch.object(m0, "BOOTLOADER_SHA256", m0.digest(self.boot)), \
             contextlib.redirect_stdout(io.StringIO()):
            self.assertEqual(m0.main(args), 0)
        report = json.loads((out / "report.json").read_text())
        self.assertEqual(report["expected_profile"], "ROBOT_POWERED")
        self.assertFalse(report["authorization_granted"])
        a, _ = self.backup_files()
        binary, target, _, _ = m0.verified_artifact(self.binary, self.mfile, expected_profile="ROBOT_POWERED")
        for stage in ("app", "table"):
            snapshot = self.root / (stage + "-snapshot.bin")
            snapshot.write_bytes(m0.expected_snapshot(self.before, binary, target, stage=stage))
            check = ["check-snapshot", "--backup", str(a), "--backup-sha256", m0.digest(self.before),
                     "--snapshot", str(snapshot), "--binary", str(self.binary), "--manifest", str(self.mfile),
                     "--stage", stage]
            with contextlib.redirect_stderr(io.StringIO()) as refused:
                self.assertEqual(m0.main(check), 1)  # default USB_ONLY cannot infer powered
            self.assertIn("PROFILE_MISMATCH", refused.getvalue())
            with contextlib.redirect_stdout(io.StringIO()) as passed:
                self.assertEqual(m0.main(check + ["--profile", "ROBOT_POWERED"]), 0)
            self.assertIn("SNAPSHOT_CHECK=PASS", passed.getvalue())
            self.assertIn("AUTHORIZATION_GRANTED=NO", passed.getvalue())

    def test_blank_otadata_ambiguous(self):
        self.stop("OTADATA_AMBIGUOUS", self.inspect, self.changed(0xE000, b"\xff" * 0x2000))

    def test_both_crc_bad(self):
        data = bytearray(otadata())
        data[28] ^= 1
        self.stop("OTADATA_AMBIGUOUS", self.inspect, self.changed(0xE000, data))

    def test_pending_even_in_losing_sector(self):
        for seq, state, second in ((1, ota.OTA_STATE_NEW, None),
                                   (3, ota.OTA_STATE_VALID, (2, ota.OTA_STATE_PENDING_VERIFY))):
            with self.subTest(state=state, second=second):
                self.stop("OTADATA_UNSTABLE", self.inspect,
                          self.changed(0xE000, otadata(seq, state, second)))

    def test_unknown_ota_state(self):
        self.stop("OTADATA_UNSTABLE", self.inspect, self.changed(0xE000, otadata(state=7)))

    def test_unknown_or_antirollback_config(self):
        for text in ("", SDKCONFIG.replace("# CONFIG_BOOTLOADER_APP_ANTI_ROLLBACK is not set",
                                           "CONFIG_BOOTLOADER_APP_ANTI_ROLLBACK=y")):
            self.stop("BOOTLOADER_CONFIG_UNEXPECTED", self.inspect, sdkconfig=text)

    def test_changed_existing_bootloader(self):
        self.stop("BOOTLOADER_NOT_PROVEN", self.inspect, self.changed(100, b"\0"))

    def test_untrusted_reference(self):
        self.stop("BOOTLOADER_REFERENCE_UNEXPECTED", self.inspect, bootloader=b"\0" * m0.BOOTLOADER_SIZE)

    def test_installed_identity_missing(self):
        self.stop("INSTALLED_IDENTITY_NOT_FOUND", self.inspect, installed_build_id="unknown")

    def test_installed_header_invalid(self):
        self.stop("INSTALLED_APP_HEADER_INVALID", self.inspect, self.changed(0x10000, b"\0"))

    def test_two_full_backups_equal(self):
        a, b = self.backup_files()
        self.assertEqual(m0.verified_backup(a, b, m0.digest(self.before)), self.before)

    def test_backup_absent(self):
        self.stop("BACKUP_MISSING", m0.verified_backup, self.root / "absent", self.root / "absent2", "")

    def test_unverified_backup_no_expected_hash(self):
        a, b = self.backup_files()
        self.stop("NO_EXPECTED_HASH", m0.verified_backup, a, b, "")

    def test_backup_wrong_hash(self):
        a, b = self.backup_files()
        self.stop("SHA256_MISMATCH", m0.verified_backup, a, b, "0" * 64)

    def test_independent_reads_disagree(self):
        a, b = self.backup_files(self.changed(0x9000, b"\0"))
        self.stop("INDEPENDENT_READ_MISMATCH", m0.verified_backup, a, b, m0.digest(self.before))

    def test_same_acquisition_is_not_a_second_read(self):
        a, _ = self.backup_files()
        self.stop("READ_INDEPENDENCE_MISSING", m0.verified_backup, a, a, m0.digest(self.before))

    def test_truncated_backup(self):
        a, b = self.backup_files()
        a.write_bytes(self.before[:-1])
        self.stop("SIZE_MISMATCH", m0.verified_backup, a, b, m0.digest(self.before))

    def test_manifest_v2_correct(self):
        _, _, m, plan = self.artifact()
        self.assertEqual(m["SOURCE_STATE"], "CLEAN")
        self.assertLessEqual(plan.erase_end, 0x310000)

    def test_incompatible_manifest_axes(self):
        for code, changes in (("PROFILE_MISMATCH", dict(profile="ROBOT_POWERED")),
                              ("OTA_INGEST_MISMATCH", dict(ota_ingest_enabled="1")),
                              ("FQBN_MISMATCH", dict(fqbn=layout.PINNED_FQBN.replace("PSRAM=opi", "PSRAM=disabled"))),
                              ("SOURCE_COMMIT_MISMATCH", dict(source_commit="0" * 40)),
                              ("TREE_NOT_CLEAN", dict(source_state="DIRTY"))):
            with self.subTest(code=code):
                self.write_manifest(**changes)
                self.stop(code, self.artifact)

    def test_v1_manifest(self):
        self.mfile.write_text(self.mfile.read_text().replace("MATDOG_MANIFEST_VERSION=2", "MATDOG_MANIFEST_VERSION=1"))
        self.stop("MANIFEST_VERSION_UNKNOWN", self.artifact)

    def test_stale_binary_hash(self):
        self.binary.write_bytes(self.binary.read_bytes()[:-1] + b"Z")
        self.stop("BINARY_SHA256_MISMATCH", self.artifact)

    def test_build_id_claim_does_not_prove_embedding(self):
        self.binary.write_bytes(self.binary.read_bytes().replace(m0.APPROVED_COMMIT[:12].encode(), b"0" * 12))
        self.write_manifest()
        self.stop("BUILD_ID_NOT_PROVEN", self.artifact)

    def test_layout_marker_missing(self):
        self.binary.write_bytes(self.binary.read_bytes().replace(layout.LAYOUT_MARKER.encode(), b"X" * len(layout.LAYOUT_MARKER)))
        self.write_manifest()
        self.stop("LAYOUT_ID_NOT_IN_BINARY", self.artifact)

    def test_oversize_image(self):
        self.binary.write_bytes(self.binary.read_bytes().ljust(layout.APP_SLOT_SIZE + 1, b"\0"))
        self.write_manifest()
        self.stop("APPLICATION_TOO_LARGE", self.artifact)

    def test_fits_new_slot_but_not_reviewed_legacy_staging(self):
        self.binary.write_bytes(self.binary.read_bytes().ljust(0x300001, b"\0"))
        self.write_manifest()
        self.stop("STAGING_EXCEEDS_LEGACY_APP0", self.artifact)

    def test_final_snapshot_exact_preservation(self):
        binary, target, _, _ = self.artifact()
        post = m0.expected_snapshot(self.before, binary, target, stage="table")
        self.assertEqual(m0.check_snapshot(self.before, post, binary, target, stage="table"), m0.digest(post))
        self.assertEqual(post[0xE000:0x10000], self.before[0xE000:0x10000])
        self.assertEqual(post[0x9000:0xE000], self.before[0x9000:0xE000])
        self.assertEqual(post[0xFE0000:0x1000000], self.before[0xFE0000:0x1000000])

    def test_partial_write_wrong_hash_or_collateral_damage(self):
        binary, target, _, _ = self.artifact()
        for offset in (0x8000, 0x10010, 0x9000, 0xFE0000, 0xFF0000):
            post = bytearray(m0.expected_snapshot(self.before, binary, target, stage="table"))
            post[offset] ^= 1
            self.stop("SNAPSHOT_BYTE_MISMATCH", m0.check_snapshot, self.before, post, binary, target, stage="table")

    def test_app_stage_and_sector_padding(self):
        binary, target, _, plan = self.artifact()
        post = m0.expected_snapshot(self.before, binary, target, stage="app")
        m0.check_snapshot(self.before, post, binary, target, stage="app")
        self.assertEqual(post[0x8000:0x9000], self.before[0x8000:0x9000])
        self.assertEqual(post[plan.write_end:plan.erase_end], b"\xff" * (plan.erase_end - plan.write_end))

    def test_cli_archive_and_no_evidence_overwrite(self):
        a, b = self.backup_files()
        out = self.root / "evidence"
        cmd = [sys.executable, str(Path(m0.__file__)), "archive", "--backup", str(a),
               "--repeat", str(b), "--backup-sha256", m0.digest(self.before), "--out", str(out)]
        first = subprocess.run(cmd, capture_output=True, text=True)
        self.assertEqual(first.returncode, 0, first.stderr)
        report = json.loads((out / "report.json").read_text())
        self.assertFalse(report["hardware_io"])
        self.assertFalse(report["authorization_granted"])
        for row in report["regions"]:
            self.assertEqual(m0.digest((out / row["file"]).read_bytes()), row["sha256"])
        self.assertNotEqual(subprocess.run(cmd, capture_output=True).returncode, 0)

    def test_existing_app_only_gate_still_refuses_legacy(self):
        self.stop("INSTALLED_LAYOUT_LEGACY", layout.check_table_bytes, self.legacy)

    def plan_args(self, out):
        a, b = self.backup_files()
        sdk = self.root / "sdkconfig"
        sdk.write_text(SDKCONFIG)
        boot = self.root / "bootloader.bin"
        boot.write_bytes(self.boot)
        return ["plan", "--backup", str(a), "--repeat", str(b), "--backup-sha256", m0.digest(self.before),
                "--sdkconfig", str(sdk), "--bootloader", str(boot), "--running-offset", "0x10000",
                "--installed-build-id", "installed-build", "--expected-mac", "14:c1:9f:22:75:94",
                "--observed-mac", "14:C1:9F:22:75:94", "--binary", str(self.binary),
                "--manifest", str(self.mfile), "--out", str(out)]

    def test_cli_plan_prepares_two_regions_and_grants_no_authorization(self):
        out = self.root / "plan"
        args = self.plan_args(out)
        with patch.object(m0, "BOOTLOADER_SHA256", m0.digest(self.boot)), contextlib.redirect_stdout(io.StringIO()):
            self.assertEqual(m0.main(args), 0)
        report = json.loads((out / "report.json").read_text())
        self.assertFalse(report["authorization_granted"])
        self.assertFalse(report["hardware_io"])
        self.assertEqual(report["app_write"]["start"], 0x10000)
        self.assertEqual(report["table_write"]["erase_end"], 0x9000)
        self.assertEqual((out / "target_partition_sector.bin").read_bytes(), self.target + b"\xff" * 0x400)
        self.assertEqual((out / "app0_sector_padded.bin").read_bytes()[:self.binary.stat().st_size], self.binary.read_bytes())

    def test_cli_plan_rejects_device_mismatch_and_creates_no_output(self):
        for mac in ("14:c1:9f:22:75:95", "wrong-format"):
            out = self.root / "refused-plan"
            args = self.plan_args(out)
            args[args.index("--observed-mac") + 1] = mac
            with contextlib.redirect_stderr(io.StringIO()) as log:
                self.assertEqual(m0.main(args), 1)
            self.assertIn("DEVICE_IDENTITY_MISMATCH", log.getvalue())
            self.assertFalse(out.exists())

    def test_snapshot_size_mismatch(self):
        binary, target, _, _ = self.artifact()
        self.stop("SNAPSHOT_SIZE_MISMATCH", m0.check_snapshot, self.before, self.before[:-1], binary, target, stage="table")

    def test_manifest_missing_and_duplicate_keys(self):
        self.mfile.unlink()
        with self.assertRaises(OSError):
            self.artifact()
        self.write_manifest()
        self.mfile.write_text(self.mfile.read_text() + "BUILD_ID=duplicate\n")
        with self.assertRaises(ValueError):
            self.artifact()

    def test_existing_migration_contract_rejects_nvs_and_missing_authorization(self):
        facts = {key: True for key in layout.MIGRATION_PRECONDITIONS}
        regions = [("app0", 0x10000, 0x11000), ("partition_table", 0x8000, 0x9000)]
        layout.check_migration_plan(regions, facts)
        facts["operator_authorization_for_this_session"] = False
        self.stop("MIGRATION_PRECONDITION", layout.check_migration_plan, regions, facts)
        facts["operator_authorization_for_this_session"] = True
        self.stop("MIGRATION_FORBIDDEN_REGION", layout.check_migration_plan,
                  [("matdog_nvs", 0xFE0000, 0xFF0000)], facts)


if __name__ == "__main__":
    unittest.main(verbosity=2)
