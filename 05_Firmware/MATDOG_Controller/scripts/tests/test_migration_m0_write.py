#!/usr/bin/env python3
"""Actual pinned esptool API, ROM packets/SLIP and flash simulation; no device.

Only physical serial, flash-ID/MAC discovery and USB inventory are simulated.
The real writer CLI, validators, connect, command, check_command, flash_begin,
flash_block, MD5 and verify_flash execute. Synthetic images are never firmware.
"""
import contextlib
import hashlib
import io
import json
import os
import struct
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
sys.path.insert(0, str(Path(__file__).resolve().parent))
import migration_m0 as m0
import migration_m0_write as writer
import build_manifest as manifest
import matdog_layout as layout
from test_migration_m0 import table, LEGACY_ROWS
import esptool.loader as loader
from esptool.targets.esp32s3 import ESP32S3ROM
import serial


def slip(data):
    return b'\xc0' + data.replace(b'\xdb', b'\xdb\xdd').replace(b'\xc0', b'\xdb\xdc') + b'\xc0'


class Transport:
    def __init__(self, before, fault=None, **kwargs):
        self.flash = bytearray(before)
        self.original = before
        self.fault = fault
        self.port = None
        self.baudrate = kwargs.get('baudrate', 115200)
        self.exclusive = kwargs.get('exclusive')
        self.timeout = 3
        self.write_timeout = 3
        self.is_open = False
        self.dtr = self.rts = False
        self.rx = bytearray()
        self.packets = []
        self.opens = self.closes = self.md5s = 0
        self.begin = None
        self.lines = []

    @property
    def name(self):
        return self.port

    def open(self):
        self.opens += 1
        assert not self.dtr and not self.rts
        if self.fault == 'open':
            raise serial.SerialException('open failure')
        self.is_open = True

    def close(self):
        self.closes += 1
        self.is_open = False

    def setDTR(self, value):
        self.lines.append(('DTR', value))
        if value:
            raise AssertionError('reset line asserted')
        self.dtr = value

    def setRTS(self, value):
        self.lines.append(('RTS', value))
        if value:
            raise AssertionError('reset line asserted')
        self.rts = value

    def flushInput(self):
        self.rx.clear()

    def flushOutput(self):
        pass

    def inWaiting(self):
        return len(self.rx)

    def read(self, size=1):
        result = bytes(self.rx[:size]); del self.rx[:size]
        return result

    @property
    def in_waiting(self):
        return len(self.rx)

    def response(self, op, val=0, data=b'\0' * 4):
        self.rx.extend(slip(struct.pack('<BBHI', 1, op, len(data), val) + data))

    def write(self, wire):
        packet = wire[1:-1].replace(b'\xdb\xdc', b'\xc0').replace(b'\xdb\xdd', b'\xdb')
        direction, op, size, checksum = struct.unpack('<BBHI', packet[:8])
        data = packet[8:]
        assert direction == 0 and size == len(data)
        self.packets.append((op, data, checksum))
        if self.fault == 'sync' and op == 8:
            return len(wire)  # one request, then transport timeout
        if self.fault == 'security' and op == 20:
            self.response(op, data=b'\x01\x05\0\0'); return len(wire)
        if op == 8:
            for _ in range(8):
                self.response(op, 0x1234)
        elif op == 20:
            self.response(op, data=struct.pack('<IBBBBBBBBII', 0, 0, *([0] * 7), 9, 0) + b'\0' * 4)
        elif op == 2:
            if self.fault == 'begin':
                raise serial.SerialException('before begin transmission')
            size, blocks, blocksize, off, encryption = struct.unpack('<IIIII', data)
            assert encryption == 0 and blocksize == 1024
            end = off + ((size + 4095) // 4096) * 4096
            self.begin = (off, size, end)
            self.flash[off:end] = b'\xff' * (end - off)
            self.response(op)
        elif op == 3:
            size, seq, _, _ = struct.unpack('<IIII', data[:16])
            assert size == 1024 and checksum == loader.ESPLoader.checksum(data[16:])
            if self.fault in ('serial-first', 'serial-after') and seq == (0 if self.fault == 'serial-first' else 3):
                # Bytes may already have reached flash; uncertain ACK, never retry.
                off = self.begin[0] + seq * 1024
                self.flash[off:off+128] = data[16:144]
                raise serial.SerialException('lost ACK after partial block')
            off = self.begin[0] + seq * 1024
            self.flash[off:off+1024] = data[16:]
            if self.fault == 'timeout':
                pass
            elif self.fault == 'invalid':
                self.response(op, data=b'\x01\x05\0\0')
            elif self.fault == 'wrong-response':
                for _ in range(100):
                    self.response(10)  # response scanning is not re-transmission
            else:
                self.response(op)
        elif op == 19:
            self.md5s += 1
            off, size, _, _ = struct.unpack('<IIII', data)
            digest = hashlib.md5(self.flash[off:off+size]).hexdigest().encode()
            if self.fault == 'md5' or self.fault == 'verify' and self.md5s == 2:
                digest = b'0' * 32
            elif self.fault == 'md5-timeout':
                return len(wire)
            self.response(op, data=digest + b'\0' * 4)
        else:
            self.response(op, val=0)
        return len(wire)


class WriterTests(unittest.TestCase):
    evidence = []
    @classmethod
    def setUpClass(cls):
        writer.load_tool()  # Fail, do not skip/fallback if exact 5.3.1 unavailable.
        cls.before = bytearray(b'\xff' * 0x1000000)
        cls.before[0x8000:0x8c00] = table(LEGACY_ROWS)
        cls.before[0x9000:0x9010] = b'preservedNVS!!!\0'
        cls.before = bytes(cls.before)

    @classmethod
    def tearDownClass(cls):
        target = os.environ.get('MATDOG_M01_TEST_EVIDENCE')
        if target:
            Path(target).write_text(json.dumps(cls.evidence, indent=2) + '\n')

    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory(); self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name)
        self.a = self.root / 'a.bin'; self.b = self.root / 'b.bin'
        self.a.write_bytes(self.before); self.b.write_bytes(self.before)
        self.app = self.root / 'MATDOG_Controller.ino.bin'
        self.app.write_bytes((b'\xe9' + m0.APPROVED_COMMIT[:12].encode() + b'\0' +
                              layout.LAYOUT_MARKER.encode() + b'\0').ljust(8192, b'\x53'))
        self.target = manifest.partition_table_path_for(self.app)
        self.target.write_bytes(table([(e.label, e.type, e.subtype, e.offset, e.size) for e in layout.EXPECTED_PARTITIONS]))
        self.manifest = self.root / 'matdog_build_manifest.txt'; self.make_manifest()
        self.addCleanup(patch.stopall)
        patch.object(writer, 'APP_SHA256', m0.digest(self.app.read_bytes())).start()
        self.devices = []

    def make_manifest(self):
        self.manifest.write_text(manifest.render_manifest(source_commit=m0.APPROVED_COMMIT,
            build_id=m0.APPROVED_COMMIT[:12], source_state='CLEAN', profile='USB_ONLY',
            ota_ingest_enabled='0', fqbn=layout.PINNED_FQBN, application_binary=self.app.name,
            application_size=self.app.stat().st_size, application_sha256=m0.digest(self.app.read_bytes()),
            layout_id=layout.LAYOUT_ID, partition_table_sha256=layout.EXPECTED_TABLE_SHA256,
            app_partition_size=layout.APP_SLOT_SIZE))

    def args(self, operation='app0'):
        if operation == 'app0':
            offset, artifact = 0x10000, self.app
        elif operation == 'table':
            offset, artifact = 0x8000, self.target
        else:
            offset, end = writer.RECOVERY[operation]
            artifact = self.root / (operation + '.bin'); artifact.write_bytes(self.before[offset:end])
        return [operation, '--artifact', str(artifact), '--sha256', m0.digest(artifact.read_bytes()),
                '--offset', hex(offset), '--gate', 'B' if operation in ('app0', 'table') else 'R2' if operation == 'r2-full' else 'R',
                '--backup', str(self.a), '--backup-repeat', str(self.b), '--backup-sha256', m0.digest(self.before),
                '--port', '/dev/serial/by-id/SIMULATED_ONLY', '--mac', writer.EXPECTED_MAC,
                '--binary', str(self.app), '--manifest', str(self.manifest)]

    def run_writer(self, args, fault=None):
        previous_devices = len(self.devices)
        def factory(**kwargs):
            port = Transport(self.before, fault, **kwargs); self.devices.append(port); return port
        with patch.object(serial, 'Serial', side_effect=factory), \
             patch.object(ESP32S3ROM, 'flash_id', return_value=0x1840ef), \
             patch.object(ESP32S3ROM, 'read_mac', return_value=tuple(bytes.fromhex('14c19f227594'))), \
             patch.object(ESP32S3ROM, 'get_usb_vid_pid', return_value=(0x303a, 0x1001)), \
             contextlib.redirect_stdout(io.StringIO()) as out, contextlib.redirect_stderr(io.StringIO()) as err:
            result = writer.main(args)
        self.out, self.err = out.getvalue(), err.getvalue()
        if self.devices:
            port = self.devices[-1]
            self.assertEqual(port.opens, 1)
            self.assertFalse(port.is_open)
            self.assertTrue(all(not value for _, value in port.lines))
            self.assertFalse(any(op in (4, 5, 6, 7, 16, 17, 18, 208, 209, 211, 212) for op, _, _ in port.packets))
            begins = [p for p in port.packets if p[0] == 2]
            self.assertLessEqual(len(begins), 1)
            seqs = [struct.unpack('<I', data[4:8])[0] for op, data, _ in port.packets if op == 3]
            self.assertEqual(seqs, list(range(len(seqs))))
            self.assertEqual(sum(op == 8 for op, _, _ in port.packets), 0 if fault == 'open' else 1)
            if fault:
                ops = [op for op, _, _ in port.packets]
                if fault == 'open':
                    self.assertEqual(ops, [])
                elif fault == 'sync':
                    self.assertEqual(ops, [8])
                elif fault == 'security':
                    self.assertEqual(ops.count(20), 1)
                    self.assertNotIn(2, ops)
                else:
                    self.assertEqual(ops.count(2), 1, 'fault must reach its W phase')
                    if fault != 'begin':
                        self.assertGreater(ops.count(3), 0)
                    if fault in ('md5', 'verify', 'md5-timeout'):
                        self.assertEqual(port.md5s, 2 if fault == 'verify' else 1)
            if port.begin:
                start, size, end = port.begin
                expected_start = {'app0': 0x10000, 'table': 0x8000}.get(args[0])
                if expected_start is None:
                    expected_start = writer.RECOVERY[args[0]][0]
                self.assertEqual(start, expected_start)
                self.assertEqual(port.flash[:start], self.before[:start])
                self.assertEqual(port.flash[end:], self.before[end:])
        if result:
            self.assertIn('M0_WRITE=STOP', self.err)
            self.assertIn('new independent state reads', self.err)
            self.assertNotIn('M0_WRITE=PASS', self.out)
        port = self.devices[-1] if self.devices and len(self.devices) > previous_devices else None
        self.evidence.append({'operation': args[0], 'fault': fault, 'result': result,
            'opens': port.opens if port else 0,
            'flash_begin_requests': sum(op == 2 for op, _, _ in port.packets) if port else 0,
            'flash_data_requests': sum(op == 3 for op, _, _ in port.packets) if port else 0,
            'MD5_requests': port.md5s if port else 0,
            'reset_assertions': sum(bool(value) for _, value in port.lines) if port else 0,
            'error': self.err.strip()})
        return result

    def test_success_all_authorized_operations_exact_flash(self):
        for operation in ('app0', 'table', *writer.RECOVERY):
            with self.subTest(operation=operation):
                args = self.args(operation); parsed = writer.parser().parse_args(args)
                data, offset, end = writer.prepare(parsed)
                self.assertEqual(self.run_writer(args), 0, self.err)
                port = self.devices[-1]; expected = bytearray(self.before)
                expected[offset:end] = data + b'\xff' * (end - offset - len(data))
                self.assertEqual(port.flash, expected)
                self.assertEqual(port.begin, (offset, len(data), end))
                self.assertEqual(port.md5s, 2)

    def test_reviewed_candidate_repair_with_separate_R(self):
        for operation in ('app0', 'table'):
            with self.subTest(operation=operation):
                args = self.args(operation)
                args[args.index('--gate') + 1] = 'R'
                self.assertEqual(self.run_writer(args), 0, self.err)
                self.assertIn('gate=R', self.out)

    def test_faults_app_and_table(self):
        for operation in ('app0', 'table'):
            for fault in ('open', 'sync', 'security', 'begin', 'serial-first', 'timeout', 'invalid', 'wrong-response', 'md5', 'verify', 'md5-timeout'):
                with self.subTest(operation=operation, fault=fault):
                    self.assertEqual(self.run_writer(self.args(operation), fault), 2)

    def test_serial_exception_after_some_blocks(self):
        self.assertEqual(self.run_writer(self.args(), 'serial-after'), 2)
        port = self.devices[-1]
        self.assertEqual(sum(op == 3 for op, _, _ in port.packets), 4)
        self.assertNotEqual(port.flash, self.before)

    def test_recovery_faults_no_fallback(self):
        for operation in writer.RECOVERY:
            for fault in ('serial-first', 'serial-after', 'invalid', 'verify'):
                with self.subTest(operation=operation, fault=fault):
                    # table/default regions also have at least four packets.
                    self.assertEqual(self.run_writer(self.args(operation), fault), 2)

    def test_missing_artifact_before_open(self):
        args = self.args(); self.app.unlink()
        self.assertEqual(self.run_writer(args), 2); self.assertFalse(self.devices)

    def test_bad_hash_before_open(self):
        args = self.args(); args[args.index('--sha256')+1] = '0' * 64
        self.assertEqual(self.run_writer(args), 2); self.assertFalse(self.devices)

    def test_wrong_offset_before_open(self):
        for op in ('app0', 'table', *writer.RECOVERY):
            with self.subTest(operation=op):
                args = self.args(op); args[args.index('--offset')+1] = '0x9000'
                if op == 'r1-default-nvs': args[args.index('--offset')+1] = '0x8000'
                self.assertEqual(self.run_writer(args), 2); self.assertFalse(self.devices)

    def test_recovery_range_and_bytes_before_open(self):
        for op in writer.RECOVERY:
            args = self.args(op); path = Path(args[args.index('--artifact')+1])
            path.write_bytes(path.read_bytes()[:-4]); args[args.index('--sha256')+1] = m0.digest(path.read_bytes())
            self.assertEqual(self.run_writer(args), 2); self.assertFalse(self.devices)

    def test_gate_port_mac_backup_manifest_before_open(self):
        for flag, value in (('--gate', 'R2'), ('--port', '/dev/ttyACM0'), ('--mac', '00:00:00:00:00:00'),
                            ('--backup-sha256', '0' * 64), ('--manifest', '/missing')):
            args = self.args(); args[args.index(flag)+1] = value
            self.assertEqual(self.run_writer(args), 2); self.assertFalse(self.devices)

    def test_actual_bindings_and_forbidden_operations(self):
        tool, _, _, base, fatal = writer.load_tool()
        self.assertEqual(tool.ESPLoader.WRITE_FLASH_ATTEMPTS, 1)
        self.assertEqual(tool.ESPLoader.flash_block.__globals__['WRITE_BLOCK_ATTEMPTS'], 1)
        cls = writer.guarded_rom(base, tool, fatal, 0x10000, self.app.read_bytes())
        esp = cls(Transport(self.before))
        for name in ('hard_reset', 'soft_reset', 'watchdog_reset', 'run_stub', 'flash_finish'):
            with self.subTest(name=name), self.assertRaises(m0.Stop): getattr(esp, name)()
        esp.connected_once = True
        with self.assertRaises(m0.Stop): esp.connect()

    def test_retry_counter_drift_before_write(self):
        original = writer.load_tool
        def drift():
            tool = original()
            tool[0].ESPLoader.WRITE_FLASH_ATTEMPTS = 2
            return tool
        with patch.object(writer, 'load_tool', side_effect=drift):
            self.assertEqual(self.run_writer(self.args()), 2)
        self.assertFalse(any(op == 2 for op, _, _ in self.devices[-1].packets))
        original()

    def test_wrong_version_before_open(self):
        import esptool
        with patch.object(esptool, '__version__', '4.7'):
            self.assertEqual(self.run_writer(self.args()), 2)
        self.assertFalse(self.devices)

    def test_source_hash_mismatch_before_open(self):
        real = writer.Path.read_text
        def corrupt(path, *args, **kwargs):
            text = real(path, *args, **kwargs)
            if path.name == 'migration_m0_esptool531.json':
                text = text.replace('76dd375f208b0fe142deef06c5fed8cfa462e194296988e404c9ed5f9cb656ef', '0'*64)
            return text
        with patch.object(writer.Path, 'read_text', corrupt):
            self.assertEqual(self.run_writer(self.args()), 2); self.assertFalse(self.devices)

    def test_unsupported_md5_is_stop_not_success(self):
        from esptool.util import NotImplementedInROMError
        def unavailable(esp, *args, **kwargs):
            raise NotImplementedInROMError(esp, ESP32S3ROM.flash_md5sum)
        with patch.object(ESP32S3ROM, 'flash_md5sum', unavailable):
            self.assertEqual(self.run_writer(self.args()), 2)
        self.assertEqual(sum(op == 2 for op, _, _ in self.devices[-1].packets), 1)

    def test_original_usb_candidate_when_supplied(self):
        root = os.environ.get('MATDOG_M0_USB')
        if not root:
            self.skipTest('set MATDOG_M0_USB for immutable original-artifact integration')
        root = Path(root)
        self.app.write_bytes((root / self.app.name).read_bytes())
        self.target.write_bytes((root / self.target.name).read_bytes())
        self.manifest.write_bytes((root / self.manifest.name).read_bytes())
        patch.object(writer, 'APP_SHA256', 'f0f3df4e83708f04d4e7acb44ab35794028521a0abfe50fa95b219694c498c3e').start()
        # Real artifact revision fields require a simulated rev >= min.
        with patch.object(ESP32S3ROM, 'get_chip_revision', return_value=100):
            self.assertEqual(self.run_writer(self.args()), 0, self.err)
        port = self.devices[-1]
        self.assertEqual(port.begin, (0x10000, 1120352, 0x122000))
        self.assertEqual(port.flash[0x10000:0x121860], self.app.read_bytes())
        self.assertEqual(port.flash[0x121860:0x122000], b'\xff' * 0x7a0)


if __name__ == '__main__':
    unittest.main()
