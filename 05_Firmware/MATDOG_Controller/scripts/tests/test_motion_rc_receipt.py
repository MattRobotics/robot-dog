#!/usr/bin/env python3
"""RC receipt mutations using the real partition compiler; no hardware."""
from pathlib import Path
import copy
import hashlib
import json
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

SCRIPTS = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SCRIPTS))
import motion_rc_receipt as rc


class ReceiptTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix="matdog-rc-receipt-")
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.binary = self.root / "MATDOG_Controller.ino.bin"
        self.table = rc.manifest.partition_table_path_for(self.binary)
        compiler = Path.home() / ".arduino15/packages/esp32/hardware/esp32/3.3.11/tools/gen_esp32part.py"
        subprocess.run([sys.executable, str(compiler), "-q", str(SCRIPTS.parent / "partitions.csv"), str(self.table)], check=True, capture_output=True)
        self.binary.write_bytes((rc.BASE + "\0" + rc.BASE[:12] + "-dirty\0" +
            "0.2.0-dev.4\0G1_G5A_ADAPTIVE_SERVICE_EXECUTOR_GATED\0" + rc.layout.LAYOUT_MARKER).encode())
        data = self.binary.read_bytes()
        self.text = rc.manifest.render_manifest(source_commit=rc.BASE, build_id=rc.BASE[:12]+"-dirty",
            source_state="DIRTY", profile="ROBOT_POWERED", ota_ingest_enabled="0", fqbn=rc.FQBN,
            application_binary=self.binary.name, application_size=len(data), application_sha256=hashlib.sha256(data).hexdigest(),
            layout_id=rc.layout.LAYOUT_ID, partition_table_sha256=rc.layout.EXPECTED_TABLE_SHA256,
            app_partition_size=rc.layout.APP_SLOT_SIZE, identity=dict(FW_VERSION="0.2.0-dev.4",
            BUILD_UTC="2026-10-12T10:00:00Z", BUILD_UTC_POLICY="UTC_NOW", CAL_RECORD_SCHEMA="1",
            CAL_MARKER_SCHEMA="2", MOTION_STACK="G1_G5A_ADAPTIVE_SERVICE_EXECUTOR_GATED"))
        self.manifest = self.root / rc.manifest.MANIFEST_FILENAME
        self.manifest.write_text(self.text)
        self.snapshot = dict(source_tree_sha256="synthetic_test_inventory", files=[dict(path="fixture.cpp")])

    def verify(self, snapshot=None):
        with patch.object(rc, "inventory", return_value=self.snapshot):
            return rc.verify(snapshot or self.snapshot, self.root)

    def test_verified_software_still_refuses_flash(self):
        result = self.verify()
        self.assertTrue(result["software_artifact_verified"])
        self.assertFalse(result["flash_authorized"])
        self.assertFalse(result["motion_authorized"])
        self.assertEqual(result["flash_gate"], "TREE_NOT_CLEAN")

    def test_manifest_mutations(self):
        for field, value in (("SOURCE_COMMIT", "0"*40), ("SOURCE_STATE", "CLEAN"),
            ("HARDWARE_PROFILE", "USB_ONLY"), ("OTA_INGEST", "1"), ("MOTION_AUTHORIZED", "1"),
            ("FQBN", "wrong"), ("LAYOUT_ID", "wrong"), ("APP_PARTITION_SIZE", "999"),
            ("APPLICATION_SHA256", "0"*64), ("MOTION_STACK", "wrong"),
            ("BUILD_ID", "wrong"), ("FW_VERSION", "0.2.0-dev.3")):
            with self.subTest(field=field):
                fields = rc.manifest.parse_manifest(self.text)
                fields[field] = value
                self.manifest.write_text("\n".join(f"{k}={v}" for k,v in fields.items()))
                with self.assertRaises(ValueError): self.verify()
        self.manifest.write_text(self.text)

    def test_binary_mutation(self):
        self.binary.write_bytes(self.binary.read_bytes() + b"changed")
        with self.assertRaises(ValueError): self.verify()

    def test_partition_mutation(self):
        self.table.write_bytes(b"wrong table")
        with self.assertRaises(ValueError): self.verify()

    def test_source_inventory_mutation(self):
        previous = copy.deepcopy(self.snapshot)
        previous["source_tree_sha256"] = "earlier"
        with self.assertRaises(ValueError): self.verify(previous)


if __name__ == "__main__":
    unittest.main(verbosity=2)
