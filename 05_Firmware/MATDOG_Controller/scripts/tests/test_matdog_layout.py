#!/usr/bin/env python3
"""Offline tests for matdog_layout.py and the flashing-safety scripts of the
MATDOG V1 flash layout. No device I/O, no flash writes, no hardware.

    python3 scripts/tests/test_matdog_layout.py

Where the real Arduino-ESP32 toolchain is installed, the binary tables are
produced by its own gen_esp32part.py (the code that builds the real table), so
"correct layout" is asserted against the real tool, not a stand-in.
"""
import dataclasses
import re
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

SCRIPTS = Path(__file__).resolve().parent.parent
SKETCH = SCRIPTS.parent
sys.path.insert(0, str(SCRIPTS))

import matdog_layout as L  # noqa: E402
from matdog_layout import Code, LayoutRefusal  # noqa: E402

GEN = Path.home() / ".arduino15/packages/esp32/hardware/esp32/3.3.11/tools/gen_esp32part.py"
LEGACY_CSV = GEN.parent / "partitions/app3M_fat9M_16MB.csv"

KiB = 1024
MiB = 1024 * KiB
E = L.EXPECTED_PARTITIONS


def entries_with(index, **changes):
    out = list(E)
    out[index] = dataclasses.replace(out[index], **changes)
    return out


def csv_text(rows):
    return "# Name, Type, SubType, Offset, Size, Flags\n" + "".join(
        f"{n}, {t}, {s}, 0x{o:x}, 0x{z:x},\n" for n, t, s, o, z in rows)


def build_table(csv_path):
    out = Path(tempfile.mkdtemp()) / "t.bin"
    subprocess.run([sys.executable, str(GEN), "-q", str(csv_path), str(out)],
                   check=True, capture_output=True)
    return out.read_bytes()


def refusal_code(fn, *args, **kwargs):
    try:
        fn(*args, **kwargs)
    except LayoutRefusal as exc:
        return exc.code
    return None


def run_script(name, *args):
    return subprocess.run(["bash", str(SCRIPTS / name), *args], capture_output=True, text=True,
                          timeout=60)


class TestContractMatchesSources(unittest.TestCase):
    def test_exactly_the_seven_specified_partitions(self):
        self.assertEqual(
            [(e.label, e.offset, e.size) for e in E],
            [("nvs", 0x9000, 0x5000), ("otadata", 0xE000, 0x2000),
             ("app0", 0x10000, 0x500000), ("app1", 0x510000, 0x500000),
             ("ffat", 0xA10000, 0x5D0000), ("matdog_nvs", 0xFE0000, 0x10000),
             ("coredump", 0xFF0000, 0x10000)])

    def test_partitions_csv_equals_contract(self):
        rows = []
        for line in (SKETCH / "partitions.csv").read_text().splitlines():
            line = line.split("#")[0].strip()
            if line:
                rows.append([c.strip() for c in line.split(",")])
        self.assertEqual(len(rows), len(E))
        for row, e in zip(rows, E):
            self.assertEqual(row[0], e.label)
            self.assertEqual(int(row[3], 0), e.offset, e.label)
            self.assertEqual(int(row[4], 0), e.size, e.label)

    def test_cpp_contract_equals_python_contract(self):
        src = (SKETCH / "src/update/OtaLayoutContract.cpp").read_text()
        self.assertIn(L.LAYOUT_MARKER, src)
        for e in E:
            m = re.search(r'\{"%s",\s*([^{}]*)\}' % re.escape(e.label), src)
            self.assertIsNotNone(m, e.label)
            nums = [int(x, 16) for x in re.findall(r"0x[0-9A-Fa-f]+", m.group(1))]
            self.assertEqual(nums[-2:], [e.offset, e.size], e.label)

    @unittest.skipUnless(GEN.is_file(), "Arduino-ESP32 toolchain not installed")
    def test_real_toolchain_builds_the_pinned_table(self):
        raw = build_table(SKETCH / "partitions.csv")
        self.assertEqual(len(raw), L.PARTITION_TABLE_MAX_BYTES)
        digest, entries = L.check_table_bytes(raw)
        self.assertEqual(digest, L.EXPECTED_TABLE_SHA256)
        self.assertEqual(entries, list(E))

    def test_layout_is_aligned_inside_flash_and_non_overlapping(self):
        L.check_entries(list(E))
        prev_end = 0
        for e in sorted(E, key=lambda x: x.offset):
            self.assertGreaterEqual(e.offset, prev_end, e.label)
            self.assertEqual(e.offset % 0x1000, 0, e.label)
            if e.type == L.TYPE_APP:
                self.assertEqual(e.offset % 0x10000, 0, e.label)
            prev_end = e.offset + e.size
        self.assertEqual(prev_end, L.FLASH_SIZE_BYTES)

    def test_default_nvs_precedes_matdog_nvs(self):
        labels = [e.label for e in E if e.subtype == L.SUBTYPE_DATA_NVS]
        self.assertEqual(labels, ["nvs", "matdog_nvs"])


class TestEntryChecks(unittest.TestCase):
    def test_correct_layout_passes(self):
        L.check_entries(list(E))

    def test_non_conforming_offset(self):
        for i in range(len(E)):
            code = refusal_code(L.check_entries, entries_with(i, offset=E[i].offset + 0x1000))
            self.assertIsNotNone(code, E[i].label)

    def test_non_conforming_size(self):
        for i in range(len(E)):
            code = refusal_code(L.check_entries, entries_with(i, size=E[i].size - 0x1000))
            self.assertIsNotNone(code, E[i].label)

    def test_app_slot_of_3m_is_refused(self):
        self.assertIsNotNone(refusal_code(L.check_entries, entries_with(2, size=0x300000)))

    def test_overlap_is_refused(self):
        entries = entries_with(3, offset=0x300000)
        self.assertEqual(refusal_code(L.check_entries, entries), Code.OVERLAP)

    def test_partition_past_the_end_of_flash(self):
        entries = entries_with(6, offset=0xFF8000)
        self.assertEqual(refusal_code(L.check_entries, entries), Code.OUT_OF_FLASH)

    def test_missing_matdog_nvs(self):
        entries = [e for e in E if e.label != "matdog_nvs"]
        self.assertEqual(refusal_code(L.check_entries, entries), Code.MATDOG_NVS_MISSING)

    def test_moved_matdog_nvs(self):
        for kw in ({"offset": 0xFD0000}, {"size": 0x8000}, {"size": 0x20000}):
            entries = entries_with(5, **kw)
            self.assertIn(refusal_code(L.check_entries, entries),
                          (Code.MATDOG_NVS_MOVED, Code.OVERLAP), kw)
        self.assertEqual(refusal_code(L.check_entries, entries_with(5, size=0x8000)),
                         Code.MATDOG_NVS_MOVED)

    def test_matdog_nvs_not_data_nvs(self):
        entries = entries_with(5, subtype=L.SUBTYPE_DATA_COREDUMP)
        self.assertIsNotNone(refusal_code(L.check_entries, entries))

    def test_wrong_nvs_order(self):
        # matdog_nvs listed before the default nvs: initArduino() would erase
        # the FIRST nvs-subtype partition on a bad default NVS.
        entries = [E[5]] + [e for e in E if e.label != "matdog_nvs"]
        self.assertEqual(refusal_code(L.check_entries, entries), Code.NVS_ORDER)

    def test_extra_partition_is_refused(self):
        extra = L.PartitionEntry("extra", L.TYPE_DATA, 0x99, 0x9000, 0x1000)
        self.assertIsNotNone(refusal_code(L.check_entries, list(E) + [extra]))

    def test_empty_table(self):
        self.assertIsNotNone(refusal_code(L.check_entries, []))


class TestTableBytes(unittest.TestCase):
    @unittest.skipUnless(GEN.is_file(), "Arduino-ESP32 toolchain not installed")
    def setUp(self):
        self.good = build_table(SKETCH / "partitions.csv")

    @unittest.skipUnless(GEN.is_file(), "Arduino-ESP32 toolchain not installed")
    def test_installed_table_equal_to_expected(self):
        digest, _ = L.check_table_bytes(self.good + b"\xff" * 0x400)  # 0x1000 device read
        self.assertEqual(digest, L.EXPECTED_TABLE_SHA256)

    @unittest.skipUnless(GEN.is_file() and LEGACY_CSV.is_file(), "toolchain not installed")
    def test_legacy_installed_table_is_named_and_refused(self):
        legacy = build_table(LEGACY_CSV)
        self.assertEqual(L.table_sha256(legacy), L.LEGACY_TABLE_SHA256)
        self.assertEqual(refusal_code(L.check_table_bytes, legacy + b"\xff" * 0x400),
                         Code.INSTALLED_LAYOUT_LEGACY)

    @unittest.skipUnless(GEN.is_file(), "Arduino-ESP32 toolchain not installed")
    def test_installed_table_differing_from_expected(self):
        rows = [("nvs", "data", "nvs", 0x9000, 0x5000),
                ("otadata", "data", "ota", 0xE000, 0x2000),
                ("app0", "app", "ota_0", 0x10000, 0x300000),
                ("app1", "app", "ota_1", 0x310000, 0x300000),
                ("ffat", "data", "fat", 0x610000, 0x9D0000),
                ("matdog_nvs", "data", "nvs", 0xFE0000, 0x10000),
                ("coredump", "data", "coredump", 0xFF0000, 0x10000)]
        csv = Path(tempfile.mkdtemp()) / "x.csv"
        csv.write_text(csv_text(rows))
        self.assertEqual(refusal_code(L.check_table_bytes, build_table(csv)),
                         Code.PARTITION_MISMATCH)

    @unittest.skipUnless(GEN.is_file(), "Arduino-ESP32 toolchain not installed")
    def test_corrupt_or_blank_tables(self):
        self.assertIsNotNone(refusal_code(L.check_table_bytes, b""))
        self.assertIsNotNone(refusal_code(L.check_table_bytes, b"\xff" * 0x1000))
        self.assertIsNotNone(refusal_code(L.check_table_bytes, self.good[:100]))
        flipped = bytearray(self.good)
        flipped[0x40] ^= 0x01
        self.assertIsNotNone(refusal_code(L.check_table_bytes, bytes(flipped)))


class TestAppSize(unittest.TestCase):
    def test_limits(self):
        self.assertEqual(L.APP_SLOT_SIZE, 5242880)
        self.assertEqual(L.check_app_size(1), "OK")
        self.assertEqual(L.check_app_size(4 * MiB - 1), "OK")
        self.assertEqual(L.check_app_size(4 * MiB), "GROWTH_WARNING")
        self.assertEqual(L.check_app_size(5 * MiB), "GROWTH_WARNING")

    def test_over_the_slot_is_refused(self):
        self.assertEqual(refusal_code(L.check_app_size, 5 * MiB + 1), Code.APP_TOO_LARGE)

    def test_empty_is_refused(self):
        self.assertEqual(refusal_code(L.check_app_size, 0), Code.APP_EMPTY)


class TestWriteRange(unittest.TestCase):
    def test_conforming_application_only_path(self):
        for off, label in ((0x10000, "app0"), (0x510000, "app1")):
            plan = L.check_app_only_write(off, 0x500000, 1_010_000)
            self.assertEqual((plan.label, plan.start), (label, off))
            self.assertEqual(plan.write_end, off + 1_010_000)
            self.assertEqual(plan.erase_end % 0x1000, 0)
            self.assertGreaterEqual(plan.erase_end, plan.write_end)
            self.assertLessEqual(plan.erase_end, off + 0x500000)

    def test_exact_slot_fits_and_is_sector_aligned(self):
        plan = L.check_app_only_write(0x510000, 0x500000, 0x500000)
        self.assertEqual(plan.erase_end, 0xA10000)  # first byte of ffat, not touched

    def test_one_byte_over_the_slot(self):
        self.assertEqual(refusal_code(L.check_app_only_write, 0x10000, 0x500000, 0x500001),
                         Code.APP_TOO_LARGE)

    def test_write_range_exceeding_app(self):
        # A start inside the slot with a length that runs past its end.
        self.assertEqual(refusal_code(L.check_write_range, 0x10000 + 0x4F0000, 0x20000),
                         Code.WRITE_EXCEEDS_APP)

    def test_range_intersecting_matdog_nvs(self):
        for start, length in ((0xFE0000, 0x1000), (0xFD0000, 0x20000), (0xFEF000, 0x1000),
                              (0xA10000, 0x600000)):
            self.assertEqual(refusal_code(L.check_write_range, start, length),
                             Code.WRITE_TOUCHES_MATDOG_NVS, hex(start))

    def test_target_naming_matdog_nvs_is_refused_first(self):
        self.assertEqual(refusal_code(L.check_app_only_write, 0xFE0000, 0x10000, 4096),
                         Code.WRITE_TOUCHES_MATDOG_NVS)

    def test_erase_rounding_is_part_of_the_range(self):
        # 1 byte past a sector boundary erases the whole next sector.
        plan = L.check_write_range(0x10000, 0x1001)
        self.assertEqual(plan.erase_end, 0x12000)
        self.assertEqual(refusal_code(L.check_write_range, 0x510000 + 0x4FF000, 0x1001),
                         Code.WRITE_EXCEEDS_APP)

    def test_non_app_targets_are_refused(self):
        for off in (0x0, 0x8000, 0x9000, 0xE000, 0xA10000, 0xFF0000, 0x20000):
            code = refusal_code(L.check_app_only_write, off, 0x500000, 4096)
            self.assertIn(code, (Code.TARGET_NOT_APP_SLOT, Code.WRITE_TOUCHES_MATDOG_NVS), hex(off))

    def test_legacy_partition_size_is_refused(self):
        self.assertEqual(refusal_code(L.check_app_only_write, 0x10000, 0x300000, 4096),
                         Code.PARTITION_SIZE_MISMATCH)

    def test_legacy_app1_offset_is_not_an_app_slot(self):
        self.assertEqual(refusal_code(L.check_app_only_write, 0x310000, 0x300000, 4096),
                         Code.TARGET_NOT_APP_SLOT)

    def test_unaligned_start(self):
        self.assertEqual(refusal_code(L.check_write_range, 0x10001, 100), Code.WRITE_UNALIGNED)

    def test_empty_image(self):
        self.assertEqual(refusal_code(L.check_app_only_write, 0x10000, 0x500000, 0),
                         Code.APP_EMPTY)

    def test_start_outside_every_slot(self):
        self.assertEqual(refusal_code(L.check_write_range, 0x9000, 4096), Code.WRITE_OUTSIDE_APP)


class TestFqbnAndBinary(unittest.TestCase):
    def test_pinned_fqbn(self):
        L.check_fqbn(L.PINNED_FQBN)
        self.assertEqual(L.partition_scheme_of(L.PINNED_FQBN), "custom")

    def test_legacy_fqbn(self):
        legacy = L.PINNED_FQBN.replace("PartitionScheme=custom", "PartitionScheme=app3M_fat9M_16MB")
        self.assertEqual(refusal_code(L.check_fqbn, legacy), Code.FQBN_LEGACY_SCHEME)

    def test_other_schemes_and_missing_scheme(self):
        for f in (L.PINNED_FQBN.replace("PartitionScheme=custom", "PartitionScheme=default"),
                  "esp32:esp32:esp32s3:FlashSize=16M"):
            self.assertEqual(refusal_code(L.check_fqbn, f), Code.FQBN_NOT_CUSTOM_LAYOUT, f)

    def test_marker_detection(self):
        self.assertTrue(L.binary_embeds_layout_id(b"aa\x00" + L.LAYOUT_MARKER.encode() + b"\x00"))
        self.assertFalse(L.binary_embeds_layout_id(b"MATDOG_LAYOUT_ID=OTHER"))
        self.assertFalse(L.binary_embeds_layout_id(b""))


class TestMigrationContractIsDataOnly(unittest.TestCase):
    def facts(self, **over):
        f = {n: True for n in L.MIGRATION_PRECONDITIONS}
        f.update(over)
        return f

    def test_allowed_plan_with_all_preconditions(self):
        L.check_migration_plan([("partition_table", 0x8000, 0x9000), ("otadata", 0xE000, 0x10000),
                                ("app0", 0x10000, 0x300000)], self.facts())

    def test_every_missing_precondition_refuses(self):
        for name in L.MIGRATION_PRECONDITIONS:
            self.assertEqual(refusal_code(L.check_migration_plan, [], self.facts(**{name: False})),
                             Code.MIGRATION_PRECONDITION, name)
        self.assertEqual(refusal_code(L.check_migration_plan, [], {}), Code.MIGRATION_PRECONDITION)

    def test_protected_regions_are_never_writable(self):
        for name, s, e in L.MIGRATION_PROTECTED_REGIONS:
            self.assertEqual(refusal_code(L.check_migration_plan, [("x", s, e)], self.facts()),
                             Code.MIGRATION_FORBIDDEN_REGION, name)
        self.assertEqual(refusal_code(L.check_migration_plan, [("x", 0xFE8000, 0xFE9000)],
                                      self.facts()), Code.MIGRATION_FORBIDDEN_REGION)

    def test_region_outside_allow_list(self):
        self.assertEqual(refusal_code(L.check_migration_plan, [("ffat", 0xA10000, 0xA20000)],
                                      self.facts()), Code.MIGRATION_FORBIDDEN_REGION)

    def test_forbidden_operations_and_no_executable_surface(self):
        self.assertIn("erase-flash", L.MIGRATION_FORBIDDEN_OPERATIONS)
        self.assertIn("full-image-restore", L.MIGRATION_FORBIDDEN_OPERATIONS)
        src = (SCRIPTS / "matdog_layout.py").read_text()
        for needle in ("subprocess", "import serial", "import esptool", "os.system", "socket"):
            self.assertNotIn(needle, src, needle)


class TestCli(unittest.TestCase):
    def run_cli(self, *args):
        return subprocess.run([sys.executable, str(SCRIPTS / "matdog_layout.py"), *args],
                              capture_output=True, text=True)

    def test_check_write_conforming(self):
        r = self.run_cli("check-write", "--offset", "0x510000", "--partition-size", "5242880",
                         "--image-size", "1500000")
        self.assertEqual(r.returncode, 0, r.stderr)
        self.assertIn("WRITE_TARGET_LABEL=app1", r.stdout)
        self.assertIn("ERASE_END=0x", r.stdout)

    def test_check_write_refusal_prints_code(self):
        r = self.run_cli("check-write", "--offset", "0x10000", "--partition-size", "3145728",
                         "--image-size", "1500000")
        self.assertEqual(r.returncode, 1)
        self.assertIn("REFUSED=PARTITION_SIZE_MISMATCH", r.stderr)

    def test_check_fqbn_cli(self):
        self.assertEqual(self.run_cli("check-fqbn", "--fqbn", L.PINNED_FQBN).returncode, 0)
        legacy = L.PINNED_FQBN.replace("PartitionScheme=custom", "PartitionScheme=app3M_fat9M_16MB")
        r = self.run_cli("check-fqbn", "--fqbn", legacy)
        self.assertEqual(r.returncode, 1)
        self.assertIn("REFUSED=FQBN_LEGACY_SCHEME", r.stderr)

    @unittest.skipUnless(GEN.is_file(), "Arduino-ESP32 toolchain not installed")
    def test_check_build_sizes(self):
        with tempfile.TemporaryDirectory() as tmp:
            table = Path(tmp) / "t.bin"
            table.write_bytes(build_table(SKETCH / "partitions.csv"))
            marker = L.LAYOUT_MARKER.encode() + b"\x00"

            def check(size, body=None):
                b = Path(tmp) / "a.bin"
                b.write_bytes(body if body is not None else marker + b"\x00" * (size - len(marker)))
                return self.run_cli("check-build", "--partitions", str(table), "--binary", str(b),
                                    "--fqbn", L.PINNED_FQBN)

            ok = check(1 * MiB)
            self.assertEqual(ok.returncode, 0, ok.stderr)
            self.assertIn("APPLICATION_SIZE_STATUS=OK", ok.stdout)
            self.assertIn(f"PARTITION_TABLE_SHA256={L.EXPECTED_TABLE_SHA256}", ok.stdout)
            self.assertEqual("", ok.stderr)
            warn = check(4 * MiB)
            self.assertEqual(warn.returncode, 0)
            self.assertIn("GROWTH_WARNING", warn.stdout)
            self.assertIn("WARNING", warn.stderr)
            over = check(5 * MiB + 1)
            self.assertEqual(over.returncode, 1)
            self.assertIn("REFUSED=APP_TOO_LARGE", over.stderr)
            nomark = check(1000, body=b"x" * 1000)
            self.assertEqual(nomark.returncode, 1)
            self.assertIn("REFUSED=LAYOUT_ID_MISSING_FROM_BINARY", nomark.stderr)


class TestUploadAndScripts(unittest.TestCase):
    def test_unauthorized_full_upload_is_refused(self):
        r = run_script("upload.sh")
        self.assertEqual(r.returncode, 1)
        self.assertIn("REFUSE", r.stderr)

    def test_upload_script_has_no_hardware_operation(self):
        text = (SCRIPTS / "upload.sh").read_text()
        code = "\n".join(l for l in text.splitlines() if not l.lstrip().startswith("#"))
        for needle in ("arduino-cli", "$ARDUINO", "esptool", "write-flash", "write_flash",
                       "erase", "--port", "/dev/"):
            self.assertNotIn(needle, code, needle)

    def test_no_erase_flash_in_any_script(self):
        for p in list(SCRIPTS.glob("*.sh")) + list(SCRIPTS.glob("*.py")):
            code = "\n".join(l for l in p.read_text().splitlines()
                             if not l.lstrip().startswith("#"))
            if p.name in ("matdog_layout.py", "static_audit.py"):
                continue  # contract data / audit rules name the forbidden operation
            self.assertNotRegex(code, r"erase[-_]flash|erase_region|erase-region", p.name)

    def test_fqbn_identical_everywhere_and_custom(self):
        for name in ("build.sh", "flash_app_only.sh", "upload.sh"):
            m = re.search(r"^FQBN='([^']+)'", (SCRIPTS / name).read_text(), re.M)
            self.assertIsNotNone(m, name)
            self.assertEqual(m.group(1), L.PINNED_FQBN, name)

    def test_flash_script_checks_layout_before_the_single_write(self):
        text = (SCRIPTS / "flash_app_only.sh").read_text()
        write = text.index('"$ESPTOOL" --chip esp32s3 --port "$PORT" write-flash')
        for needle in ("matdog_layout.py\" check-write", "--expected-table-sha256",
                       "VERIFIED_LAYOUT_ID", "backup_gate_logic.py", "read-mac"):
            idx = text.index(needle)
            self.assertLess(idx, write, needle)
        code = [l for l in text.splitlines() if not l.lstrip().startswith(("#", "echo"))]
        self.assertEqual(sum("write-flash" in l for l in code), 1)

    def test_build_script_runs_the_layout_gate_before_the_manifest(self):
        text = (SCRIPTS / "build.sh").read_text()
        self.assertLess(text.index("matdog_layout.py\" check-build"),
                        text.index("build_manifest.py\" write"))
        self.assertIn("upload.maximum_size=5242880", text)


if __name__ == "__main__":
    unittest.main()
