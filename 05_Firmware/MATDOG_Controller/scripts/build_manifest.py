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

OTA INGEST PROVENANCE (I7 hardening, 2026-09-25)
-------------------------------------------------
The same "which build is this, really" problem the hardware profile solves
above applies to `MATDOG_OTA_INGEST_ENABLED`: the source default is `0` and
must stay `0` (scripts/static_audit.py enforces the `#define`), but the ONE
hardware-validation candidate needs ingest compiled in so the same flashed
image can later validate the OTA end-to-end path too, without a second
flash. `OTA_INGEST_ENABLED` is therefore a second, orthogonal authorization
axis on the manifest — same shape as `HARDWARE_PROFILE`, same
explicit-authorization requirement, its own mismatch reason, never inferred
from the profile (a `ROBOT_POWERED` build with ingest still `0` is a
perfectly ordinary, expected combination).

LAYOUT PROVENANCE (manifest V2, P2.3)
-------------------------------------
MATDOG flash layout V1 (scripts/matdog_layout.py, partitions.csv) puts the
persistent MATDOG NVS partition in flash. The manifest therefore also binds
the build to the layout it was made for: LAYOUT_ID, the SHA-256 of the
binary partition table the build ACTUALLY produced (not of the csv), and the
size of the application partitions. V1 manifests, and any manifest without
this information, are refused. verify also re-hashes the table artifact next
to the binary and requires the binary itself to embed the layout id string
compiled into the firmware, so the claim is tied to the artifact and not to
free-standing metadata.

FAIL-CLOSED CONTRACT
--------------------
`verify_manifest()` returns a refusal reason for every case it cannot
positively prove. It never has a permissive default, never infers a profile
and never treats "absent" as "fine". The only path to OK is: manifest
present and parseable, its schema version known, its commit equal to HEAD,
its FQBN equal to the caller's pinned FQBN, the layout id / partition table
digest / app partition size equal to the pinned MATDOG layout, tree clean at
build time and now, binary present, binary size AND sha256 equal to the
recorded ones (and within the application partition), manifest profile
recognized, and the requested profile equal to the manifest profile.

Usage:
    build_manifest.py write  --output P --binary P --source-commit SHA \\
                             --build-id ID --source-state CLEAN|DIRTY|NO_GIT \\
                             --profile USB_ONLY|ROBOT_POWERED --fqbn FQBN \\
                             --ota-ingest 0|1
    (the partition table is read from <binary>.partitions.bin next to the binary)
    build_manifest.py verify --manifest P --binary P --head SHA \\
                             --expected-fqbn FQBN --tree-state CLEAN|DIRTY \\
                             --requested-profile USB_ONLY|ROBOT_POWERED \\
                             --requested-ota-ingest 0|1

Exit code 0 = OK, 1 = REFUSE (reason printed as REFUSED=<CODE>).
"""
import argparse
import datetime
import hashlib
import re
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import matdog_layout  # noqa: E402

MANIFEST_VERSION = "2"
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

# The only OTA-ingest values that may ever appear in a manifest — a string,
# not a bool, for the same reason profile is a name and not a flag: it is
# rendered into and parsed back out of a flat KEY=VALUE text file, and an
# explicit closed set is refused-not-guessed on anything else.
KNOWN_OTA_INGEST_VALUES = ("0", "1")

# "1" (ingest compiled in) requires explicit authorization, the same shape
# as ROBOT_POWERED above and for the same reason: MATDOG_OTA_INGEST_ENABLED
# defaulting to 0 is a permanent safety property (scripts/static_audit.py
# fails the build if the source default is anything else), so a binary that
# overrides it must never be flashed by an unqualified invocation.
OTA_INGEST_VALUES_REQUIRING_EXPLICIT_AUTHORIZATION = ("1",)

DEFAULT_FLASH_OTA_INGEST = "0"

REQUIRED_KEYS = (
    "MATDOG_MANIFEST_VERSION",
    "SOURCE_COMMIT",
    "BUILD_ID",
    "SOURCE_STATE",
    "HARDWARE_PROFILE",
    "OTA_INGEST_ENABLED",
    "FQBN",
    "APPLICATION_BINARY",
    "APPLICATION_SIZE",
    "APPLICATION_SHA256",
    "LAYOUT_ID",
    "PARTITION_TABLE_SHA256",
    "APP_PARTITION_SIZE",
)

# Additive V2 identity group. Historical V2 receipts remain verifiable using
# their original gates; a new receipt must carry the complete group and agree
# with those gates. APP_SHA256 is a host-side digest of the final application,
# never a claim that the application can embed its own final hash.
IDENTITY_KEYS = (
    "FW_VERSION", "GIT_SHA", "GIT_DIRTY", "FLASH_LAYOUT",
    "CAL_RECORD_SCHEMA", "CAL_MARKER_SCHEMA", "OTA_INGEST",
    "MOTION_STACK", "MOTION_AUTHORIZED", "APP_SHA256", "BUILD_UTC",
    "BUILD_UTC_POLICY",
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
    OTA_INGEST_UNKNOWN = "OTA_INGEST_UNKNOWN"
    REQUESTED_OTA_INGEST_UNKNOWN = "REQUESTED_OTA_INGEST_UNKNOWN"
    OTA_INGEST_MISMATCH = "OTA_INGEST_MISMATCH"
    LAYOUT_ID_MISMATCH = "LAYOUT_ID_MISMATCH"
    PARTITION_TABLE_MISMATCH = "PARTITION_TABLE_MISMATCH"
    PARTITION_TABLE_ARTIFACT_MISSING = "PARTITION_TABLE_ARTIFACT_MISSING"
    APP_PARTITION_SIZE_MISMATCH = "APP_PARTITION_SIZE_MISMATCH"
    APPLICATION_TOO_LARGE = "APPLICATION_TOO_LARGE"
    LAYOUT_ID_NOT_IN_BINARY = "LAYOUT_ID_NOT_IN_BINARY"
    IDENTITY_MISMATCH = "IDENTITY_MISMATCH"


class Verdict:
    def __init__(self, ok, reason=None, profile=None, ota_ingest=None, detail=""):
        self.ok = ok
        self.reason = reason
        self.profile = profile
        self.ota_ingest = ota_ingest
        self.detail = detail

    def __repr__(self):  # pragma: no cover - debugging aid only
        return (f"Verdict(ok={self.ok}, reason={self.reason!r}, profile={self.profile!r}, "
               f"ota_ingest={self.ota_ingest!r})")


def sha256_file(path):
    h = hashlib.sha256()
    with open(path, "rb") as fh:
        for chunk in iter(lambda: fh.read(1024 * 1024), b""):
            h.update(chunk)
    return h.hexdigest()


def partition_table_path_for(binary):
    """The binary partition table the build exported next to the application."""
    binary = Path(binary)
    name = binary.name[:-4] if binary.name.endswith(".bin") else binary.name
    return binary.with_name(name + ".partitions.bin")


def render_manifest(*, source_commit, build_id, source_state, profile, ota_ingest_enabled,
                    fqbn, application_binary, application_size, application_sha256,
                    layout_id, partition_table_sha256, app_partition_size, identity=None):
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
        f"OTA_INGEST_ENABLED={ota_ingest_enabled}",
        f"FQBN={fqbn}",
        f"APPLICATION_BINARY={application_binary}",
        f"APPLICATION_SIZE={application_size}",
        f"APPLICATION_SHA256={application_sha256}",
        f"LAYOUT_ID={layout_id}",
        f"PARTITION_TABLE_SHA256={partition_table_sha256}",
        f"APP_PARTITION_SIZE={app_partition_size}",
    ]
    if identity is not None:
        # Derive duplicate provenance fields from the existing owners, rather
        # than accepting independent claims that can disagree with them.
        fields = dict(identity)
        fields.update(GIT_SHA=source_commit,
                      GIT_DIRTY="0" if source_state == "CLEAN" else "1",
                      FLASH_LAYOUT=layout_id, OTA_INGEST=str(ota_ingest_enabled),
                      MOTION_AUTHORIZED="0", APP_SHA256=application_sha256)
        problem = identity_problem(fields, source_commit=source_commit,
                                   source_state=source_state, layout_id=layout_id,
                                   ota_ingest=ota_ingest_enabled,
                                   application_sha256=application_sha256)
        if problem:
            raise ValueError(problem)
        lines += [f"{key}={fields[key]}" for key in IDENTITY_KEYS]
    return "\n".join(lines) + "\n"


def identity_problem(fields, *, source_commit, source_state, layout_id,
                     ota_ingest, application_sha256):
    """Return the first additive identity contradiction, or None."""
    missing = [key for key in IDENTITY_KEYS if not fields.get(key)]
    if missing:
        return "missing identity keys: " + ", ".join(missing)
    if any(not isinstance(fields[key], str) or "\n" in fields[key] or "\r" in fields[key]
           for key in IDENTITY_KEYS):
        return "identity values must be single-line strings"
    if not re.fullmatch(r"(0|[1-9][0-9]*)\.(0|[1-9][0-9]*)\.(0|[1-9][0-9]*)(?:-(?:dev|rc)\.(?:0|[1-9][0-9]*))?", fields["FW_VERSION"]):
        return "FW_VERSION is not a supported release identity"
    if not re.fullmatch(r"[0-9a-f]{40}", fields["GIT_SHA"]):
        return "GIT_SHA must be a full local commit ID"
    expected = {"GIT_SHA": source_commit,
                "GIT_DIRTY": "0" if source_state == "CLEAN" else "1",
                "FLASH_LAYOUT": layout_id, "OTA_INGEST": str(ota_ingest),
                "MOTION_AUTHORIZED": "0", "APP_SHA256": application_sha256,
                "CAL_RECORD_SCHEMA": "1", "CAL_MARKER_SCHEMA": "2",
                "MOTION_STACK": "G1_G5A_COMPILED_UNWIRED"}
    for key, value in expected.items():
        if fields[key] != value:
            return f"{key} disagrees with its owner: {fields[key]!r} != {value!r}"
    if fields["BUILD_UTC_POLICY"] not in ("UTC_NOW", "SOURCE_DATE_EPOCH"):
        return "unknown BUILD_UTC_POLICY"
    try:
        timestamp = datetime.datetime.strptime(fields["BUILD_UTC"], "%Y-%m-%dT%H:%M:%SZ")
        if timestamp.strftime("%Y-%m-%dT%H:%M:%SZ") != fields["BUILD_UTC"]:
            return "BUILD_UTC is not canonical UTC"
    except ValueError:
        return "BUILD_UTC is not canonical UTC"
    return None


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
                    binary_exists, binary_size, binary_sha256, requested_profile,
                    requested_ota_ingest, build_partition_table_sha256,
                    binary_embeds_layout_id):
    """The fail-closed gate. Pure: every observation is passed in.

    `manifest` is a parsed dict (or None when the file was missing).
    Order of checks is deliberate: cheapest/most fundamental first, so the
    reported refusal names the most upstream problem rather than a
    downstream symptom of it — manifest integrity, then build identity
    (commit + FQBN), then tree state, then the binary's exact bytes, then
    profile authorization, then OTA-ingest authorization.

    A successful verdict positively proves that ALL of the following belong
    to the artifact being authorized: source commit, clean build state,
    clean current tree, application filename, exact binary size, exact
    binary SHA256, hardware profile, OTA-ingest compile state, and FQBN.
    Nothing is assumed and nothing is inferred from one field to another —
    in particular, OTA-ingest state is never inferred from the hardware
    profile; a ROBOT_POWERED build with ingest still 0 is ordinary.

    `build_partition_table_sha256` is the digest of the table artifact found
    next to the binary NOW ("" if absent); `binary_embeds_layout_id` says
    whether the binary contains the layout id string compiled into the
    firmware. Both are required, like every other observation.

    `expected_fqbn` is deliberately a required keyword argument with no
    default: the caller (flash_app_only.sh) pins its own FQBN and must
    state it. A default here would let a caller that forgot the argument
    silently skip a build-configuration check.
    """
    if manifest is None:
        return Verdict(False, Refusal.MANIFEST_MISSING,
                       detail="no build manifest next to the application binary")

    # The version is judged first: a V1 manifest (no layout information at
    # all) must be reported as an unsupported version, not as "incomplete".
    version = manifest.get("MATDOG_MANIFEST_VERSION", "")
    if version != "" and version != MANIFEST_VERSION:
        return Verdict(False, Refusal.MANIFEST_VERSION_UNKNOWN,
                       detail=f"manifest version {version!r} != supported "
                              f"{MANIFEST_VERSION!r} (manifests without layout "
                              f"information are not accepted)")

    missing = [k for k in REQUIRED_KEYS if k not in manifest or manifest[k] == ""]
    if missing:
        return Verdict(False, Refusal.MANIFEST_INCOMPLETE,
                       detail=f"missing/empty manifest keys: {', '.join(missing)}")

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

    # Flash layout: the recorded layout must be THE pinned MATDOG layout, the
    # recorded partition table digest must be the pinned one AND equal to the
    # table artifact the build left next to the binary, and the application
    # partition size must be the pinned 5 MiB.
    if manifest["LAYOUT_ID"] != matdog_layout.LAYOUT_ID:
        return Verdict(False, Refusal.LAYOUT_ID_MISMATCH,
                       detail=f"manifest LAYOUT_ID {manifest['LAYOUT_ID']!r} != "
                              f"{matdog_layout.LAYOUT_ID!r}")
    if manifest["PARTITION_TABLE_SHA256"].lower() != matdog_layout.EXPECTED_TABLE_SHA256:
        return Verdict(False, Refusal.PARTITION_TABLE_MISMATCH,
                       detail=f"manifest partition table {manifest['PARTITION_TABLE_SHA256']} "
                              f"!= pinned {matdog_layout.EXPECTED_TABLE_SHA256}")
    if manifest["APP_PARTITION_SIZE"] != str(matdog_layout.APP_SLOT_SIZE):
        return Verdict(False, Refusal.APP_PARTITION_SIZE_MISMATCH,
                       detail=f"manifest APP_PARTITION_SIZE {manifest['APP_PARTITION_SIZE']} "
                              f"!= {matdog_layout.APP_SLOT_SIZE}")
    if build_partition_table_sha256 == "":
        return Verdict(False, Refusal.PARTITION_TABLE_ARTIFACT_MISSING,
                       detail="the binary partition table the build produced is not next "
                              "to the application binary")
    if manifest["PARTITION_TABLE_SHA256"].lower() != str(build_partition_table_sha256).lower():
        return Verdict(False, Refusal.PARTITION_TABLE_MISMATCH,
                       detail=f"manifest partition table {manifest['PARTITION_TABLE_SHA256']} "
                              f"!= table artifact on disk {build_partition_table_sha256}")

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

    # The image must fit the application partition it is declared for, and
    # must itself carry the layout id (identity of the artifact, not of the
    # metadata next to it).
    if int(binary_size) > matdog_layout.APP_SLOT_SIZE:
        return Verdict(False, Refusal.APPLICATION_TOO_LARGE,
                       detail=f"binary {binary_size} bytes > application partition "
                              f"{matdog_layout.APP_SLOT_SIZE} bytes")
    if not binary_embeds_layout_id:
        return Verdict(False, Refusal.LAYOUT_ID_NOT_IN_BINARY,
                       detail=f"binary does not embed '{matdog_layout.LAYOUT_MARKER}'")

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

    manifest_ota_ingest = manifest["OTA_INGEST_ENABLED"]
    if manifest_ota_ingest not in KNOWN_OTA_INGEST_VALUES:
        return Verdict(False, Refusal.OTA_INGEST_UNKNOWN,
                       detail=f"manifest OTA_INGEST_ENABLED={manifest_ota_ingest!r} is not "
                              f"one of {list(KNOWN_OTA_INGEST_VALUES)}")

    if requested_ota_ingest not in KNOWN_OTA_INGEST_VALUES:
        return Verdict(False, Refusal.REQUESTED_OTA_INGEST_UNKNOWN,
                       detail=f"requested OTA ingest {requested_ota_ingest!r} is not one "
                              f"of {list(KNOWN_OTA_INGEST_VALUES)}")

    if manifest_ota_ingest != requested_ota_ingest:
        # The other half of the case this module exists for: a binary built
        # with the OTA firmware-ingest writer compiled in must never be
        # flashed by an invocation that did not explicitly ask for it — the
        # same "never silently the more capable image" rule PROFILE_MISMATCH
        # already gives HARDWARE_PROFILE.
        extra = ""
        if manifest_ota_ingest in OTA_INGEST_VALUES_REQUIRING_EXPLICIT_AUTHORIZATION:
            extra = (" — flashing an image with OTA ingest compiled in requires explicit "
                     "authorization (MATDOG_FLASH_OTA_INGEST=1)")
        return Verdict(False, Refusal.OTA_INGEST_MISMATCH,
                       detail=f"manifest built with OTA_INGEST_ENABLED={manifest_ota_ingest}, "
                              f"flash requested {requested_ota_ingest}{extra}")

    if any(key in manifest for key in IDENTITY_KEYS):
        problem = identity_problem(manifest, source_commit=manifest["SOURCE_COMMIT"],
                                   source_state=manifest["SOURCE_STATE"],
                                   layout_id=manifest["LAYOUT_ID"],
                                   ota_ingest=manifest_ota_ingest,
                                   application_sha256=manifest["APPLICATION_SHA256"])
        if problem:
            return Verdict(False, Refusal.IDENTITY_MISMATCH, detail=problem)

    return Verdict(True, profile=manifest_profile, ota_ingest=manifest_ota_ingest)


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
    if args.ota_ingest not in KNOWN_OTA_INGEST_VALUES:
        print(f"REFUSED={Refusal.OTA_INGEST_UNKNOWN}", file=sys.stderr)
        print(f"DETAIL=unknown ota-ingest value {args.ota_ingest!r}", file=sys.stderr)
        return 1

    # The manifest is the single choke point: it is never written for a build
    # whose layout, table or size is not the pinned one.
    try:
        matdog_layout.check_fqbn(args.fqbn)
        table_path = partition_table_path_for(binary)
        if not table_path.is_file():
            raise matdog_layout.LayoutRefusal(
                Refusal.PARTITION_TABLE_ARTIFACT_MISSING, f"not found: {table_path}")
        table_digest, _ = matdog_layout.check_table_bytes(table_path.read_bytes())
        matdog_layout.check_app_size(binary.stat().st_size)
        if not matdog_layout.binary_embeds_layout_id(binary.read_bytes()):
            raise matdog_layout.LayoutRefusal(
                Refusal.LAYOUT_ID_NOT_IN_BINARY, f"'{matdog_layout.LAYOUT_MARKER}' not in {binary}")
    except matdog_layout.LayoutRefusal as exc:
        print(f"REFUSED={exc.code}", file=sys.stderr)
        print(f"DETAIL={exc.detail}", file=sys.stderr)
        return 1

    identity = None
    if any(getattr(args, name) is not None for name in
           ("fw_version", "build_utc", "build_utc_policy", "motion_stack",
            "cal_record_schema", "cal_marker_schema")):
        identity = {"FW_VERSION": args.fw_version, "BUILD_UTC": args.build_utc,
                    "BUILD_UTC_POLICY": args.build_utc_policy,
                    "MOTION_STACK": args.motion_stack,
                    "CAL_RECORD_SCHEMA": args.cal_record_schema,
                    "CAL_MARKER_SCHEMA": args.cal_marker_schema}
        # Firmware strings are observable in the actual application. The
        # remaining aliases come from the already verified manifest owners.
        for key in ("FW_VERSION", "BUILD_UTC", "MOTION_STACK"):
            value = identity[key]
            if not value or value.encode("ascii", errors="replace") not in binary.read_bytes():
                print(f"REFUSED={Refusal.IDENTITY_MISMATCH}", file=sys.stderr)
                print(f"DETAIL={key} is missing from application bytes", file=sys.stderr)
                return 1
        if args.source_commit.encode("ascii") not in binary.read_bytes():
            print(f"REFUSED={Refusal.IDENTITY_MISMATCH}", file=sys.stderr)
            print("DETAIL=full GIT_SHA is missing from application bytes", file=sys.stderr)
            return 1

    try:
        text = render_manifest(
        source_commit=args.source_commit,
        build_id=args.build_id,
        source_state=args.source_state,
        profile=args.profile,
        ota_ingest_enabled=args.ota_ingest,
        fqbn=args.fqbn,
        application_binary=binary.name,
        application_size=binary.stat().st_size,
        application_sha256=sha256_file(binary),
        layout_id=matdog_layout.LAYOUT_ID,
        partition_table_sha256=table_digest,
        app_partition_size=matdog_layout.APP_SLOT_SIZE,
        identity=identity,
        )
    except ValueError as exc:
        print(f"REFUSED={Refusal.IDENTITY_MISMATCH}", file=sys.stderr)
        print(f"DETAIL={exc}", file=sys.stderr)
        return 1
    Path(args.output).write_text(text, encoding="utf-8")
    print(f"BUILD_MANIFEST={args.output}")
    print(f"HARDWARE_PROFILE={args.profile}")
    print(f"OTA_INGEST_ENABLED={args.ota_ingest}")
    print(f"LAYOUT_ID={matdog_layout.LAYOUT_ID}")
    print(f"PARTITION_TABLE_SHA256={table_digest}")
    return 0


def _cmd_verify(args):
    manifest, parse_error = load_manifest(args.manifest)
    if parse_error is not None:
        print(f"REFUSED={Refusal.MANIFEST_UNPARSEABLE}", file=sys.stderr)
        print(f"DETAIL={parse_error}", file=sys.stderr)
        return 1

    binary = Path(args.binary)
    binary_exists = binary.is_file()
    table_path = partition_table_path_for(binary)
    table_sha = sha256_file(table_path) if table_path.is_file() else ""
    embeds = (binary_exists and
              matdog_layout.binary_embeds_layout_id(binary.read_bytes()))
    verdict = verify_manifest(
        manifest,
        head_commit=args.head,
        expected_fqbn=args.expected_fqbn,
        tree_state=args.tree_state,
        binary_exists=binary_exists,
        binary_size=binary.stat().st_size if binary_exists else 0,
        binary_sha256=sha256_file(binary) if binary_exists else "",
        requested_profile=args.requested_profile,
        requested_ota_ingest=args.requested_ota_ingest,
        build_partition_table_sha256=table_sha,
        binary_embeds_layout_id=embeds,
    )

    if not verdict.ok:
        print(f"REFUSED={verdict.reason}", file=sys.stderr)
        print(f"DETAIL={verdict.detail}", file=sys.stderr)
        return 1

    print(f"VERIFIED_HARDWARE_PROFILE={verdict.profile}")
    print(f"VERIFIED_OTA_INGEST_ENABLED={verdict.ota_ingest}")
    print(f"VERIFIED_SOURCE_COMMIT={manifest['SOURCE_COMMIT']}")
    print(f"VERIFIED_FQBN={manifest['FQBN']}")
    print(f"VERIFIED_APPLICATION_SHA256={manifest['APPLICATION_SHA256']}")
    print(f"VERIFIED_APPLICATION_SIZE={manifest['APPLICATION_SIZE']}")
    print(f"VERIFIED_LAYOUT_ID={manifest['LAYOUT_ID']}")
    print(f"VERIFIED_PARTITION_TABLE_SHA256={manifest['PARTITION_TABLE_SHA256']}")
    print(f"VERIFIED_APP_PARTITION_SIZE={manifest['APP_PARTITION_SIZE']}")
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
    # Deliberately REQUIRED here too, with no default: the backwards-safe "0"
    # belongs to the caller (scripts/build.sh), which states it explicitly -
    # a default here would let a caller that forgot the flag silently record
    # an unauthorized-sounding "0" for a build that was never checked.
    w.add_argument("--ota-ingest", required=True)
    w.add_argument("--fqbn", required=True)
    w.add_argument("--fw-version")
    w.add_argument("--build-utc")
    w.add_argument("--build-utc-policy")
    w.add_argument("--motion-stack")
    w.add_argument("--cal-record-schema")
    w.add_argument("--cal-marker-schema")
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
    # Same reasoning, for the OTA-ingest authorization axis: the
    # backwards-safe "0" default belongs to flash_app_only.sh, stated
    # explicitly, never assumed here.
    v.add_argument("--requested-ota-ingest", required=True)
    # Also required with no default, for the same reason: flash_app_only.sh
    # pins the FQBN and must state it, so a caller that forgets cannot
    # silently skip the build-configuration check.
    v.add_argument("--expected-fqbn", required=True)
    v.set_defaults(func=_cmd_verify)

    args = parser.parse_args(argv)
    return args.func(args)


if __name__ == "__main__":
    sys.exit(main())
