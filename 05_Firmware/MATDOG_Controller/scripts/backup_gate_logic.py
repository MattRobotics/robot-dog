#!/usr/bin/env python3
"""MATDOG full-flash backup gate — pure verifier for scripts/flash_app_only.sh.

WHY THIS EXISTS (2026-09-25 recovery-backup hardening)
--------------------------------------------------------
flash_app_only.sh's backup gate originally pinned exactly one full-flash
backup's SHA256 in source (the 2026-09-10 historical backup) and refused
anything else. That is correct for a backup nobody expects to change, but a
genuine hardware-validation session needs a FRESH backup taken immediately
before that session's flash — and a fresh backup's hash cannot be pinned in
source ahead of time; it does not exist yet when this file is written.

This module is the fail-closed rule for accepting one: the historical
default backup is unchanged (its hash stays pinned here, reviewed once); any
OTHER backup path is a "custom" backup and is accepted ONLY together with an
explicitly authorized expected SHA256 — either stated directly by the
operator, or read from a companion recovery manifest recording one. A custom
backup is never accepted by path or size alone: size is necessary, never
sufficient. Both authorization mechanisms exist so the OPERATOR (not this
script, and not the file itself) is the one asserting what the backup's
bytes should be; this module's only job is to independently prove the file
actually on disk still matches that assertion.

DESIGN
------
Pure logic, no device I/O, no file reads beyond what the caller passes in —
mirrors build_manifest.py's verify_manifest() exactly: every observation
(file size, digest, the manifest's own recorded hash) is a parameter never
read from disk by verify_backup() itself, so the offline test suite can
drive every path without a real file on disk. Only the thin CLI at the
bottom (main(), used by flash_app_only.sh) touches the filesystem, and only
to read a manifest file's text — never the multi-megabyte backup itself
(the caller already hashed that).

Usage:
    backup_gate_logic.py --backup-path P --default-backup-path P \\
                         --actual-size N --actual-sha256 HEX \\
                         [--custom-expected-sha256 HEX] [--manifest-path P]

Exit code 0 = OK (prints VERIFIED_BACKUP_SHA256=...), 1 = REFUSE (reason
printed as REFUSED=<CODE>).
"""
import argparse
import sys

# Reviewed once, here — the ONE historical backup this repository has
# always trusted (2026-09-10). Never overwritten by a fresh session backup;
# see flash_app_only.sh's DEFAULT_BACKUP path.
DEFAULT_BACKUP_SHA256 = "5cbba0b9c5500d0c95247b9b7e7173a29f934b8b13f6800cc9f583374d67fd32"

EXPECTED_BACKUP_SIZE = 16777216


class Refusal:
    SIZE_MISMATCH = "SIZE_MISMATCH"
    NO_EXPECTED_HASH = "NO_EXPECTED_HASH"
    SHA256_MISMATCH = "SHA256_MISMATCH"


class Verdict:
    def __init__(self, ok, reason=None, detail="", expected_sha256=None):
        self.ok = ok
        self.reason = reason
        self.detail = detail
        self.expected_sha256 = expected_sha256

    def __repr__(self):  # pragma: no cover - debugging aid only
        return f"Verdict(ok={self.ok}, reason={self.reason!r})"


def verify_backup(*, is_default_backup, actual_size, actual_sha256,
                  custom_expected_sha256, manifest_sha256):
    """The fail-closed gate. Pure: every observation is passed in.

    `is_default_backup` is True iff the backup path equals the ONE
    historical default path — the only case where DEFAULT_BACKUP_SHA256
    applies. `actual_size`/`actual_sha256` are measured from the real file
    by the caller. `custom_expected_sha256` is the operator-stated
    MATDOG_FLASH_BACKUP_SHA256 (or None/empty if not given).
    `manifest_sha256` is the BACKUP_SHA256 value read from a companion
    recovery manifest (or None if no manifest was found, or it had no such
    line) — see _read_manifest_sha256() below.

    Size is checked before authorization: a backup of the wrong size is
    refused regardless of which hash source would otherwise apply.
    """
    if actual_size != EXPECTED_BACKUP_SIZE:
        return Verdict(False, Refusal.SIZE_MISMATCH,
                       detail=f"backup size {actual_size} != expected "
                              f"{EXPECTED_BACKUP_SIZE}")

    if is_default_backup:
        expected = DEFAULT_BACKUP_SHA256
    elif custom_expected_sha256:
        expected = custom_expected_sha256
    elif manifest_sha256:
        expected = manifest_sha256
    else:
        return Verdict(False, Refusal.NO_EXPECTED_HASH,
                       detail="custom backup has no explicitly authorized expected "
                              "SHA256 - set MATDOG_FLASH_BACKUP_SHA256=<hex> or provide "
                              "a recovery manifest with a BACKUP_SHA256= line; a custom "
                              "backup is never accepted by path or size alone")

    if actual_sha256.lower() != expected.lower():
        return Verdict(False, Refusal.SHA256_MISMATCH,
                       detail=f"backup sha256 {actual_sha256} != expected {expected}",
                       expected_sha256=expected)

    return Verdict(True, expected_sha256=expected)


def read_manifest_sha256(path):
    """Reads a BACKUP_SHA256= line from a recovery manifest file. Returns
    None if the path is empty, the file does not exist, or it has no such
    line — the caller (verify_backup(), via main()) decides whether that
    is a refusal. The FIRST matching line wins; a manifest is not expected
    to ever have more than one, but this does not raise on a malformed one
    the way build_manifest.py's strict parser does — a missing/duplicate
    BACKUP_SHA256 line here means "no manifest-sourced hash available",
    which NO_EXPECTED_HASH already refuses if nothing else supplies one."""
    if not path:
        return None
    try:
        with open(path, encoding="utf-8") as fh:
            text = fh.read()
    except OSError:
        return None
    for line in text.splitlines():
        line = line.strip()
        if line.startswith("BACKUP_SHA256="):
            value = line.split("=", 1)[1].strip()
            return value or None
    return None


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--backup-path", required=True)
    parser.add_argument("--default-backup-path", required=True)
    parser.add_argument("--actual-size", required=True, type=int)
    parser.add_argument("--actual-sha256", required=True)
    # Deliberately default to "" rather than None at the argparse layer —
    # verify_backup() treats an empty string as "not supplied", the same
    # falsy-check every other optional authorization input in this
    # repository's scripts already uses.
    parser.add_argument("--custom-expected-sha256", default="")
    parser.add_argument("--manifest-path", default="")
    args = parser.parse_args(argv)

    manifest_sha256 = read_manifest_sha256(args.manifest_path)

    verdict = verify_backup(
        is_default_backup=(args.backup_path == args.default_backup_path),
        actual_size=args.actual_size,
        actual_sha256=args.actual_sha256,
        custom_expected_sha256=args.custom_expected_sha256 or None,
        manifest_sha256=manifest_sha256,
    )

    if not verdict.ok:
        print(f"REFUSED={verdict.reason}", file=sys.stderr)
        print(f"DETAIL={verdict.detail}", file=sys.stderr)
        return 1

    print(f"VERIFIED_BACKUP_SHA256={verdict.expected_sha256}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
