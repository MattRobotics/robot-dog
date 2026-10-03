#!/usr/bin/env python3
"""One gated M0 W only. Never invoke without the runbook's hardware authorization.

No CLI dispatch, auto-detection, stub, reset, retry, chaining or recovery fallback.
Files are validated before opening a port. The library sources must match the
5.3.1 binary reviewed for M0; only its retry policy is changed in memory.
"""
import argparse
import json
import struct
import sys
from pathlib import Path

# Also usable with Python -I (exclude inherited PYTHONPATH and user packages).
sys.path.insert(0, str(Path(__file__).resolve().parent))
import migration_m0 as m0

REVIEWED_CANDIDATES = {
    "USB_ONLY": {
        "bytes": 1120352,
        "app_sha256": "f0f3df4e83708f04d4e7acb44ab35794028521a0abfe50fa95b219694c498c3e",
        "manifest_sha256": "b6dc29ca350af8073c3d9977cd5be89a593d637ef8279a283f1bfa49c55dc5d8",
    },
    "ROBOT_POWERED": {
        "bytes": 1123472,
        "app_sha256": "7cc1cbbc024e58630ed93fd0df8b7491659c0c20d6e1a4916333230eb1a82be0",
        "manifest_sha256": "d5c92755d958203d89eb0e439b2dce80a6faa79302000b7088b1a3209ce1ea48",
    },
}
EXPECTED_MAC = "14:c1:9f:22:75:94"
RECOVERY = {
    "r1-app0": (0x10000, 0x310000),
    "r1-otadata": (0xE000, 0x10000),
    "r1-table": (0x8000, 0x9000),
    "r1-default-nvs": (0x9000, 0xE000),
    "r1-coredump": (0xFF0000, 0x1000000),
    "r2-full": (0, 0x1000000),
}


def prepare(args):
    """Closed set of operations. Hash + exact authorized bytes, not a generic range."""
    # R may explicitly authorize repair with the same reviewed candidate/table;
    # this is distinct from rollback using the r1-* backup extracts.
    gates = ("B", "R") if args.operation in ("app0", "table") else ("R2",) if args.operation == "r2-full" else ("R",)
    if args.gate not in gates:
        raise m0.Stop("GATE_MISMATCH", f"operator gate {gates} required; flag is not authorization")
    if args.mac.lower() != EXPECTED_MAC:
        raise m0.Stop("DEVICE_UNEXPECTED")
    if Path(args.port).parent != Path("/dev/serial/by-id") or Path(args.port).name in ("", ".", ".."):
        raise m0.Stop("PORT_UNEXPECTED", "use the identified Linux by-id port")
    before = m0.verified_backup(args.backup, args.backup_repeat, args.backup_sha256)
    m0.legacy_table(before)
    data = m0.read_file(args.artifact)
    if m0.digest(data) != args.sha256:
        raise m0.Stop("ARTIFACT_HASH_MISMATCH")
    if args.operation in ("app0", "table"):
        if not args.binary or not args.manifest:
            raise m0.Stop("MANIFEST_REQUIRED")
        app, table, _, plan = m0.verified_artifact(args.binary, args.manifest, expected_profile=args.profile)
        candidate = REVIEWED_CANDIDATES[args.profile]
        if m0.digest(app) != candidate["app_sha256"] or len(app) != candidate["bytes"]:
            raise m0.Stop("REVIEWED_APP_MISMATCH")
        if m0.digest(m0.read_file(args.manifest)) != candidate["manifest_sha256"]:
            raise m0.Stop("REVIEWED_MANIFEST_MISMATCH")
        m0.check_reclassified_regions(before)
        offset, expected = (plan.start, app) if args.operation == "app0" else (0x8000, table)
        erase_end = plan.erase_end if args.operation == "app0" else 0x9000
    else:
        offset, erase_end = RECOVERY[args.operation]
        expected = before[offset:erase_end]
    if args.offset != offset or data != expected or offset % 0x1000 or not data or len(data) % 4:
        raise m0.Stop("WRITE_NOT_AUTHORIZED", "offset, interval or exact bytes differ")
    if offset + ((len(data) + 0xFFF) // 0x1000) * 0x1000 != erase_end:
        raise m0.Stop("ERASE_BOUNDARY_MISMATCH")
    return data, offset, erase_end


def load_tool():
    import esptool
    import esptool.loader as loader
    import esptool.cmds as cmds
    import serial
    from esptool.targets.esp32s3 import ESP32S3ROM
    from esptool.util import FatalError
    pins = json.loads(Path(__file__).with_name("migration_m0_esptool531.json").read_text())
    if esptool.__version__ != "5.3.1" or serial.__version__ != pins["pyserial_version"]:
        raise m0.Stop("TOOL_VERSION_MISMATCH")
    roots = {"esptool": Path(esptool.__file__).parent, "serial": Path(serial.__file__).parent}
    for package, files in pins["sources"].items():
        for relative, sha in files.items():
            if m0.digest(m0.read_file(roots[package] / relative)) != sha:
                raise m0.Stop("TOOL_SOURCE_MISMATCH", f"{package}/{relative}")
    loader.ESPLoader.WRITE_FLASH_ATTEMPTS = 1
    loader.WRITE_BLOCK_ATTEMPTS = 1
    if (loader.ESPLoader.WRITE_FLASH_ATTEMPTS, loader.WRITE_BLOCK_ATTEMPTS) != (1, 1):
        raise m0.Stop("RETRY_POLICY_NOT_APPLIED")
    if cmds.write_flash.__globals__["SerialException"] is not serial.SerialException:
        raise m0.Stop("TOOL_BINDING_MISMATCH")
    return loader, cmds, serial, ESP32S3ROM, FatalError


def guarded_rom(base, loader, fatal, offset, data):
    class M0ROM(base):
        connected_once = False
        began = False
        next_seq = 0
        poisoned = False

        def connect(self, mode="no-reset", attempts=1, detecting=False, warnings=False):
            if self.connected_once or mode != "no-reset" or attempts != 1 or detecting:
                raise m0.Stop("RECONNECT_OR_RESET_FORBIDDEN")
            self.connected_once = True
            return super().connect(mode=mode, attempts=1, detecting=False, warnings=warnings)

        def forbidden(self, *args, **kwargs):
            raise m0.Stop("RESET_STUB_OR_FINISH_FORBIDDEN")

        hard_reset = soft_reset = watchdog_reset = run_stub = flash_finish = forbidden
        flash_defl_begin = flash_defl_block = flash_defl_finish = forbidden

        def flash_md5sum(self, *args, **kwargs):
            # Do not let write_flash swallow an unsupported ROM digest.
            try:
                return super().flash_md5sum(*args, **kwargs)
            except fatal as exc:
                self.poisoned = True
                raise m0.Stop("ROM_MD5_ERROR", str(exc)) from exc

        def command(self, *args, **kwargs):
            # sync() uses command() directly; its failed response must not enter
            # the library's five-sync-attempt or security fallback loops.
            if self.poisoned:
                raise m0.Stop("SESSION_UNCERTAIN")
            try:
                return super().command(*args, **kwargs)
            except BaseException as exc:
                self.poisoned = True
                if isinstance(exc, fatal):
                    raise m0.Stop("ROM_PROTOCOL_ERROR", str(exc)) from exc
                raise

        def check_command(self, op_description, op=None, data=b"", chk=0, **kwargs):
            if self.poisoned:
                raise m0.Stop("SESSION_UNCERTAIN")
            allowed = {self.ESP_CMDS[n] for n in (
                "SYNC", "READ_REG", "WRITE_REG", "GET_SECURITY_INFO", "SPI_ATTACH",
                "SPI_SET_PARAMS", "SPI_FLASH_MD5", "FLASH_BEGIN", "FLASH_DATA")}
            try:
                if op not in allowed:
                    raise m0.Stop("ROM_COMMAND_FORBIDDEN", str(op))
                if op == self.ESP_CMDS["FLASH_BEGIN"]:
                    size, blocks, blocksize, address, encrypted = struct.unpack("<IIIII", data)
                    if (self.began or (size, address, encrypted, blocksize, blocks) !=
                        (len(payload), offset, 0, self.FLASH_WRITE_SIZE,
                         (len(payload) + self.FLASH_WRITE_SIZE - 1) // self.FLASH_WRITE_SIZE)):
                        raise m0.Stop("FLASH_BEGIN_NOT_AUTHORIZED")
                    self.began = True  # includes uncertain transmission/erase
                if op == self.ESP_CMDS["FLASH_DATA"]:
                    size, seq, reserved1, reserved2 = struct.unpack("<IIII", data[:16])
                    block = payload[seq * self.FLASH_WRITE_SIZE:(seq + 1) * self.FLASH_WRITE_SIZE]
                    expected = block + b"\xff" * (self.FLASH_WRITE_SIZE - len(block))
                    if (not self.began or not block or seq != self.next_seq or
                        (size, reserved1, reserved2) != (self.FLASH_WRITE_SIZE, 0, 0) or
                        data[16:] != expected or chk != self.checksum(expected)):
                        raise m0.Stop("FLASH_DATA_NOT_AUTHORIZED")
                    self.next_seq += 1  # a lost ACK must never repeat this packet
                return super().check_command(op_description, op, data, chk, **kwargs)
            except BaseException as exc:
                self.poisoned = True
                # Prevent library FatalError fallback/retry handlers from swallowing
                # a failed command (including security-info or MD5 failures).
                if isinstance(exc, fatal):
                    raise m0.Stop("ROM_PROTOCOL_ERROR", str(exc)) from exc
                raise

    payload = data
    return M0ROM


def execute(args):
    data, offset, erase_end = prepare(args)
    loader, cmds, serial, base, fatal = load_tool()
    rom_type = guarded_rom(base, loader, fatal, offset, data)
    port = serial.Serial(port=None, baudrate=115200, exclusive=True)
    try:
        port.dtr = False
        port.rts = False
        port.port = args.port
        port.open()  # exactly once; ROM already entered separately via usb-reset
        esp = rom_type(port, baud=115200)
        esp.connect(mode="no-reset", attempts=1)
        if esp.IS_STUB or esp.sync_stub_detected or esp.secure_download_mode:
            raise m0.Stop("ROM_MODE_UNEXPECTED")
        security = esp.get_security_info()
        if (security["chip_id"] != 9 or security["flags"] & 5 or
            bin(security["flash_crypt_cnt"]).count("1") % 2):
            raise m0.Stop("SECURITY_OR_CHIP_UNEXPECTED")
        if ":".join(f"{v:02x}" for v in esp.read_mac()) != args.mac.lower():
            raise m0.Stop("MAC_MISMATCH")
        # Direct ROM attach only; avoid CLI attach_flash's XMC recovery and
        # volatile NOR reset/fallback paths. No CPU reset, erase or stub here.
        esp.flash_spi_attach(0)
        if cmds.detect_flash_size(esp) != "16MB":
            raise m0.Stop("FLASH_SIZE_UNEXPECTED")
        if (esp.WRITE_FLASH_ATTEMPTS != 1 or loader.WRITE_BLOCK_ATTEMPTS != 1 or
            loader.ESPLoader.flash_block.__globals__["WRITE_BLOCK_ATTEMPTS"] != 1):
            raise m0.Stop("RETRY_POLICY_NOT_APPLIED")
        print(f"M0_WRITE operation={args.operation} gate={args.gate} profile={args.profile} offset=0x{offset:x} "
              f"bytes={len(data)} erase_end=0x{erase_end:x} image_attempts=1 block_attempts=1", flush=True)
        cmds.write_flash(esp, [(offset, data)], flash_mode="keep", flash_freq="keep",
                         flash_size="keep", flash_type="nor", no_compress=True,
                         erase_all=False, encrypt=False, force=False, diff_with=[],
                         skip_flashed=False, no_progress=True)
        # write_flash skips unsupported ROM MD5; mandatory verification must not.
        cmds.verify_flash(esp, [(offset, data)], flash_mode="keep", flash_freq="keep",
                          flash_size="keep", flash_type="nor", diff=False)
        print("M0_WRITE=PASS; independent runbook verify/readback still required", flush=True)
    finally:
        port.close()  # no reset_chip(), flash_finish(), reconnect or fallback


def parser():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("operation", choices=("app0", "table", *RECOVERY))
    for name in ("artifact", "sha256", "gate", "backup", "backup-repeat", "backup-sha256", "port", "mac"):
        p.add_argument("--" + name, required=True)
    p.add_argument("--offset", type=lambda v: int(v, 0), required=True)
    p.add_argument("--binary")
    p.add_argument("--manifest")
    p.add_argument("--profile", choices=m0.PROFILES, default="USB_ONLY",
                   help="explicit ROBOT_POWERED required for its pinned M0.4 candidate")
    return p


def main(argv=None):
    try:
        execute(parser().parse_args(argv))
        return 0
    except (Exception, KeyboardInterrupt) as exc:
        print(f"M0_WRITE=STOP: {exc}. No retry/reset/recovery. Before ANY further W, "
              "obtain new independent state reads and renew the applicable gate.", file=sys.stderr)
        return 2


if __name__ == "__main__":
    sys.exit(main())
