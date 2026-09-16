#!/usr/bin/env python3
"""MATDOG Controller build manifest — writer, parser and fail-closed verifier.

WHY THIS EXISTS (G2 pre-G3 hardening, review Finding 1)
-------------------------------------------------------
`scripts/build.sh` can produce either a `USB_ONLY` or a `ROBOT_POWERED`
application image from the SAME source commit, and both land at the same
path in the same (gitignored) build directory:

    build/esp32.esp32.esp32s3/MATDOG_Controller.ino.bin

The pre-existing flash gate proved the binary embedded the current commit's
build id. That is necessary but NOT sufficient: the build id is identical
for both profiles, so it cannot distinguish them. Consequences:

  - a stale USB_ONLY binary could be flashed while the operator believes a
    ROBOT_POWERED image is going on the device;
  - a ROBOT_POWERED binary could linger in the shared build directory and
    later be flashed under a USB_ONLY assumption.

Either direction is a provenance failure with real hardware consequences,
because the profile decides whether the firmware expects live servo power,
a live DALY and a powered LED rail.

This module closes that gap by binding the profile (and the binary's exact
bytes) to the build, and making the flash path refuse anything it cannot
positively prove.

DESIGN
------
Pure logic here, device I/O nowhere — deliberately mirroring
`ota_partition_logic.py` (pure, offline-tested) vs
`verify_application_partition.py` (the device-I/O wrapper). There is no
device I/O in the manifest gate at all, so one file is enough; the CLI at
the bottom is a thin shell over the pure functions above it.

FAIL-CLOSED CONTRACT
--------------------
`verify_manifest()` returns a refusal reason for every case it cannot
positively prove. It never has a permissive default, never infers a profile
and never treats "absent" as "fine". The only path to OK is: manifest
present and parseable, its schema version known, its commit equal to HEAD,
its FQBN equal to the caller's pinned FQBN, tree clean at build time and
now, binary present, binary size AND sha256 equal to the recorded ones,
manifest profile recognized, and the requested profile equal to the
manifest profile.

Usage:
    build_manifest.py write  --output P --binary P --source-commit SHA \\
                             --build-id ID --source-state CLEAN|DIRTY|NO_GIT \\
                             --profile USB_ONLY|ROBOT_POWERED --fqbn FQBN
    build_manifest.py verify --manifest P --binary P --head SHA \\
                             --expected-fqbn FQBN --tree-state CLEAN|DIRTY \\
                             --requested-profile USB_ONLY|ROBOT_POWERED

Exit code 0 = OK, 1 = REFUSE (reason printed as REFUSED=<CODE>).
"""
import argparse
import hashlib
import sys
from pathlib import Path

MANIFEST_VERSION = "1"
MANIFEST_FILENAME = "matdog_build_manifest.txt"

# The only profiles that may ever appear in a manifest. Anything else —
# including an empty value or a typo — is refused, never guessed.
KNOWN_PROFILES = ("USB_ONLY", "ROBOT_POWERED")

# Profiles whose flash requires an explicit, deliberate operator
# authorization rather than the backwards-safe default. USB_ONLY is the
# historical default behaviour and stays reachable with no new ceremony;
# ROBOT_POWERED must be asked for by name.
PROFILES_REQUIRING_EXPLICIT_AUTHORIZATION = ("ROBOT_POWERED",)

DEFAULT_FLASH_PROFILE = "USB_ONLY"

REQUIRED_KEYS = (
    "MATDOG_MANIFEST_VERSION",
    "SOURCE_COMMIT",
    "BUILD_ID",
    "SOURCE_STATE",
    "HARDWARE_PROFILE",
    "FQBN",
    "APPLICATION_BINARY",
    "APPLICATION_SIZE",
    "APPLICATION_SHA256",
)


# --- Refusal reason codes ---------------------------------------------------
# Stable strings so the offline tests can assert the EXACT refusal, not just
# "it failed somehow".
class Refusal:
    MANIFEST_MISSING = "MANIFEST_MISSING"
    MANIFEST_UNPARSEABLE = "MANIFEST_UNPARSEABLE"
    MANIFEST_INCOMPLETE = "MANIFEST_INCOMPLETE"
    MANIFEST_VERSION_UNKNOWN = "MANIFEST_VERSION_UNKNOWN"
    SOURCE_COMMIT_MISMATCH = "SOURCE_COMMIT_MISMATCH"
    FQBN_MISMATCH = "FQBN_MISMATCH"
    TREE_NOT_CLEAN = "TREE_NOT_CLEAN"
    BINARY_MISSING = "BINARY_MISSING"
    BINARY_SIZE_MISMATCH = "BINARY_SIZE_MISMATCH"
    BINARY_SHA256_MISMATCH = "BINARY_SHA256_MISMATCH"
    PROFILE_UNKNOWN = "PROFILE_UNKNOWN"
    REQUESTED_PROFILE_UNKNOWN = "REQUESTED_PROFILE_UNKNOWN"
    PROFILE_MISMATCH = "PROFILE_MISMATCH"


class Verdict:
    def __init__(self, ok, reason=None, profile=None, detail=""):
        self.ok = ok
        self.reason = reason
        self.profile = profile
        self.detail = detail

    def __repr__(self):  # pragma: no cover - debugging aid only
        return f"Verdict(ok={self.ok}, reason={self.reason!r}, profile={self.profile!r})"


def sha256_file(path):
    h = hashlib.sha256()
    with open(path, "rb") as fh:
        for chunk in iter(lambda: fh.read(1024 * 1024), b""):
            h.update(chunk)
    return h.hexdigest()


def render_manifest(*, source_commit, build_id, source_state, profile, fqbn,
                    application_binary, application_size, application_sha256):
    """Renders the manifest text. Deliberately a flat, fixed-order
    KEY=VALUE format with no quoting, no nesting and no escaping: it is
    consumed by `grep '^KEY=' | cut -d= -f2-` in shell as well as by this
    module, and anything richer would make that parsing fragile."""
    lines = [
        f"MATDOG_MANIFEST_VERSION={MANIFEST_VERSION}",
        f"SOURCE_COMMIT={source_commit}",
        f"BUILD_ID={build_id}",
        f"SOURCE_STATE={source_state}",
        f"HARDWARE_PROFILE={profile}",
        f"FQBN={fqbn}",
        f"APPLICATION_BINARY={application_binary}",
        f"APPLICATION_SIZE={application_size}",
        f"APPLICATION_SHA256={application_sha256}",
    ]
    return "\n".join(lines) + "\n"


def parse_manifest(text):
    """Strict parser. Returns a dict, or raises ValueError.

    Strict on purpose: a duplicated key, a line without '=', or a blank key
    means the file is not something we understand, and a provenance gate
    must not proceed on a file it only partially understands.
    """
    result = {}
    for lineno, raw in enumerate(text.splitlines(), start=1):
        line = raw.strip()
        if not line or line.startswith("#"):
            continue
        if "=" not in line:
            raise ValueError(f"line {lineno}: no '=' separator: {raw!r}")
        key, _, value = line.partition("=")
        key = key.strip()
        if not key:
            raise ValueError(f"line {lineno}: empty key: {raw!r}")
        if key in result:
            raise ValueError(f"line {lineno}: duplicate key {key!r}")
        result[key] = value.strip()
    return result


def verify_manifest(manifest, *, head_commit, expected_fqbn, tree_state,
                    binary_exists, binary_size, binary_sha256, requested_profile):
    """The fail-closed gate. Pure: every observation is passed in.

    `manifest` is a parsed dict (or None when the file was missing).
    Order of checks is deliberate: cheapest/most fundamental first, so the
    reported refusal names the most upstream problem rather than a
    downstream symptom of it — manifest integrity, then build identity
    (commit + FQBN), then tree state, then the binary's exact bytes, then
    profile authorization.

    A successful verdict positively proves that ALL of the following belong
    to the artifact being authorized: source commit, clean build state,
    clean current tree, application filename, exact binary size, exact
    binary SHA256, hardware profile, and FQBN. Nothing is assumed and
    nothing is inferred from one field to another.

    `expected_fqbn` is deliberately a required keyword argument with no
    default: the caller (flash_app_only.sh) pins its own FQBN and must
    state it. A default here would let a caller that forgot the argument
    silently skip a build-configuration check.
    """
    if manifest is None:
        return Verdict(False, Refusal.MANIFEST_MISSING,
                       detail="no build manifest next to the application binary")

    missing = [k for k in REQUIRED_KEYS if k not in manifest or manifest[k] == ""]
    if missing:
        return Verdict(False, Refusal.MANIFEST_INCOMPLETE,
                       detail=f"missing/empty manifest keys: {', '.join(missing)}")

    if manifest["MATDOG_MANIFEST_VERSION"] != MANIFEST_VERSION:
        return Verdict(False, Refusal.MANIFEST_VERSION_UNKNOWN,
                       detail=f"manifest version {manifest['MATDOG_MANIFEST_VERSION']!r} "
                              f"!= supported {MANIFEST_VERSION!r}")

    if manifest["SOURCE_COMMIT"] != head_commit:
        return Verdict(False, Refusal.SOURCE_COMMIT_MISMATCH,
                       detail=f"manifest built from {manifest['SOURCE_COMMIT']}, "
                              f"HEAD is {head_commit}")

    # The right source compiled with the wrong toolchain configuration is
    # still the wrong artifact: the FQBN carries the partition scheme, flash
    # size/mode, PSRAM mode, USB/CDC mode and CPU frequency. An image built
    # under a different partition scheme can be a valid binary of the right
    # commit and still be wrong for this device's flash layout. Compared in
    # full, exactly — never pattern-matched, never merely checked for
    # presence.
    if manifest["FQBN"] != expected_fqbn:
        return Verdict(False, Refusal.FQBN_MISMATCH,
                       detail=f"manifest FQBN {manifest['FQBN']!r} != expected "
                              f"{expected_fqbn!r}")

    # Both the recorded build state and the live tree must be clean. The
    # manifest's own SOURCE_STATE catches "built dirty, then committed",
    # which a live `git status` alone would not.
    if manifest["SOURCE_STATE"] != "CLEAN":
        return Verdict(False, Refusal.TREE_NOT_CLEAN,
                       detail=f"manifest records SOURCE_STATE="
                              f"{manifest['SOURCE_STATE']} — the binary was built from "
                              f"a tree that was not clean")
    if tree_state != "CLEAN":
        return Verdict(False, Refusal.TREE_NOT_CLEAN,
                       detail=f"working tree is {tree_state}")

    if not binary_exists:
        return Verdict(False, Refusal.BINARY_MISSING,
                       detail="application binary not found")

    if int(manifest["APPLICATION_SIZE"]) != int(binary_size):
        return Verdict(False, Refusal.BINARY_SIZE_MISMATCH,
                       detail=f"manifest {manifest['APPLICATION_SIZE']} bytes, "
                              f"binary on disk {binary_size} bytes")

    if manifest["APPLICATION_SHA256"].lower() != str(binary_sha256).lower():
        return Verdict(False, Refusal.BINARY_SHA256_MISMATCH,
                       detail=f"manifest {manifest['APPLICATION_SHA256']}, "
                              f"binary on disk {binary_sha256}")

    manifest_profile = manifest["HARDWARE_PROFILE"]
    if manifest_profile not in KNOWN_PROFILES:
        return Verdict(False, Refusal.PROFILE_UNKNOWN,
                       detail=f"manifest HARDWARE_PROFILE={manifest_profile!r} is not "
                              f"one of {list(KNOWN_PROFILES)}")

    if requested_profile not in KNOWN_PROFILES:
        return Verdict(False, Refusal.REQUESTED_PROFILE_UNKNOWN,
                       detail=f"requested flash profile {requested_profile!r} is not "
                              f"one of {list(KNOWN_PROFILES)}")

    if manifest_profile != requested_profile:
        # This is the case the whole module exists for, including the
        # "manifest says ROBOT_POWERED but nobody asked for it" default.
        extra = ""
        if manifest_profile in PROFILES_REQUIRING_EXPLICIT_AUTHORIZATION:
            extra = (f" — flashing a {manifest_profile} image requires explicit "
                     f"authorization (MATDOG_FLASH_PROFILE={manifest_profile})")
        return Verdict(False, Refusal.PROFILE_MISMATCH,
                       detail=f"manifest built {manifest_profile}, flash requested "
                              f"{requested_profile}{extra}")

    return Verdict(True, profile=manifest_profile)


def load_manifest(path):
    """Reads and parses a manifest file. Returns (manifest_or_None, error)."""
    p = Path(path)
    if not p.is_file():
        return None, None
    try:
        return parse_manifest(p.read_text(encoding="utf-8")), None
    except (ValueError, UnicodeDecodeError) as exc:
        return None, str(exc)


def _cmd_write(args):
    binary = Path(args.binary)
    if not binary.is_file():
        print(f"REFUSED={Refusal.BINARY_MISSING}", file=sys.stderr)
        print(f"DETAIL=application binary not found: {binary}", file=sys.stderr)
        return 1
    if args.profile not in KNOWN_PROFILES:
        print(f"REFUSED={Refusal.PROFILE_UNKNOWN}", file=sys.stderr)
        print(f"DETAIL=unknown profile {args.profile!r}", file=sys.stderr)
        return 1

    text = render_manifest(
        source_commit=args.source_commit,
        build_id=args.build_id,
        source_state=args.source_state,
        profile=args.profile,
        fqbn=args.fqbn,
        application_binary=binary.name,
        application_size=binary.stat().st_size,
        application_sha256=sha256_file(binary),
    )
    Path(args.output).write_text(text, encoding="utf-8")
    print(f"BUILD_MANIFEST={args.output}")
    print(f"HARDWARE_PROFILE={args.profile}")
    return 0


def _cmd_verify(args):
    manifest, parse_error = load_manifest(args.manifest)
    if parse_error is not None:
        print(f"REFUSED={Refusal.MANIFEST_UNPARSEABLE}", file=sys.stderr)
        print(f"DETAIL={parse_error}", file=sys.stderr)
        return 1

    binary = Path(args.binary)
    binary_exists = binary.is_file()
    verdict = verify_manifest(
        manifest,
        head_commit=args.head,
        expected_fqbn=args.expected_fqbn,
        tree_state=args.tree_state,
        binary_exists=binary_exists,
        binary_size=binary.stat().st_size if binary_exists else 0,
        binary_sha256=sha256_file(binary) if binary_exists else "",
        requested_profile=args.requested_profile,
    )

    if not verdict.ok:
        print(f"REFUSED={verdict.reason}", file=sys.stderr)
        print(f"DETAIL={verdict.detail}", file=sys.stderr)
        return 1

    print(f"VERIFIED_HARDWARE_PROFILE={verdict.profile}")
    print(f"VERIFIED_SOURCE_COMMIT={manifest['SOURCE_COMMIT']}")
    print(f"VERIFIED_FQBN={manifest['FQBN']}")
    print(f"VERIFIED_APPLICATION_SHA256={manifest['APPLICATION_SHA256']}")
    print(f"VERIFIED_APPLICATION_SIZE={manifest['APPLICATION_SIZE']}")
    return 0


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    sub = parser.add_subparsers(dest="command", required=True)

    w = sub.add_parser("write", help="write a build manifest next to the binary")
    w.add_argument("--output", required=True)
    w.add_argument("--binary", required=True)
    w.add_argument("--source-commit", required=True)
    w.add_argument("--build-id", required=True)
    w.add_argument("--source-state", required=True, choices=("CLEAN", "DIRTY", "NO_GIT"))
    w.add_argument("--profile", required=True)
    w.add_argument("--fqbn", required=True)
    w.set_defaults(func=_cmd_write)

    v = sub.add_parser("verify", help="fail-closed pre-flash manifest verification")
    v.add_argument("--manifest", required=True)
    v.add_argument("--binary", required=True)
    v.add_argument("--head", required=True)
    v.add_argument("--tree-state", required=True)
    # Deliberately REQUIRED here, with no default: the backwards-safe
    # USB_ONLY default belongs to the caller (flash_app_only.sh), which
    # states it explicitly. A default in this module would mean a caller
    # that forgot to pass the flag silently got a permissive answer.
    v.add_argument("--requested-profile", required=True)
    # Also required with no default, for the same reason: flash_app_only.sh
    # pins the FQBN and must state it, so a caller that forgets cannot
    # silently skip the build-configuration check.
    v.add_argument("--expected-fqbn", required=True)
    v.set_defaults(func=_cmd_verify)

    args = parser.parse_args(argv)
    return args.func(args)


if __name__ == "__main__":
    sys.exit(main())
