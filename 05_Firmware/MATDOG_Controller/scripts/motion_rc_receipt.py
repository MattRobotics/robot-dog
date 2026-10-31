#!/usr/bin/env python3
"""Bind an uncommitted SOFTWARE RC to its complete source inventory and image.

This never authorizes flash. The existing clean-tree flash manifest verifier
must still refuse TREE_NOT_CLEAN until review, commit and a clean rebuild.
Outputs must be outside the worktree so verifying cannot alter its inventory.
"""
from pathlib import Path
import argparse
import hashlib
import json
import subprocess
import build_manifest as manifest
import matdog_layout as layout

ROOT = Path(__file__).resolve().parents[3]
BASE = "fdd04345c39b50137cc7cdc50fb08cd592b8c2af"
FQBN = "esp32:esp32:esp32s3:USBMode=hwcdc,CDCOnBoot=cdc,UploadMode=default,CPUFreq=240,FlashMode=qio,FlashSize=16M,PartitionScheme=custom,DebugLevel=none,PSRAM=opi"


def git(*args):
    return subprocess.check_output(["git", "-C", str(ROOT), *args])


def digest(data):
    return hashlib.sha256(data).hexdigest()


def inventory():
    head = git("rev-parse", "HEAD").decode().strip()
    if head != BASE:
        raise ValueError("RC must use the reviewed fdd0434 Motion worktree")
    paths = sorted(set(git("ls-files", "-z", "--cached", "--others", "--exclude-standard").decode().split("\0")) - {""})
    entries = []
    for path in paths:
        file = ROOT / path
        if not file.is_file() or file.is_symlink():
            raise ValueError("missing/nonregular source: " + path)
        data = file.read_bytes()
        entries.append(dict(path=path, size=len(data), sha256=digest(data)))
    encoded = json.dumps(entries, sort_keys=True, separators=(",", ":")).encode()
    return dict(schema="MATDOG_SOFTWARE_RC_SOURCE_V1", source_base=head,
                status=git("status", "--porcelain", "--untracked-files=all").decode(),
                source_tree_sha256=digest(encoded), files=entries)


def verify(snapshot, artifacts):
    current = inventory()
    if snapshot != current:
        raise ValueError("source bytes or inventory changed after snapshot; rebuild required")
    binary = artifacts / "MATDOG_Controller.ino.bin"
    table = manifest.partition_table_path_for(binary)
    fields, error = manifest.load_manifest(artifacts / manifest.MANIFEST_FILENAME)
    if error or not fields:
        raise ValueError(error or "manifest missing")
    data = binary.read_bytes()
    table_sha = digest(table.read_bytes())
    expected = dict(SOURCE_COMMIT=BASE, GIT_SHA=BASE, SOURCE_STATE="DIRTY", GIT_DIRTY="1",
                    BUILD_ID=BASE[:12]+"-dirty", FW_VERSION="0.2.0-dev.4",
                    HARDWARE_PROFILE="ROBOT_POWERED", OTA_INGEST_ENABLED="0", OTA_INGEST="0",
                    MOTION_AUTHORIZED="0", FQBN=FQBN, LAYOUT_ID=layout.LAYOUT_ID,
                    PARTITION_TABLE_SHA256=layout.EXPECTED_TABLE_SHA256,
                    APP_PARTITION_SIZE=str(layout.APP_SLOT_SIZE), APPLICATION_SIZE=str(len(data)),
                    APPLICATION_SHA256=digest(data), APP_SHA256=digest(data),
                    MOTION_STACK="G1_G5A_ADAPTIVE_SERVICE_EXECUTOR_GATED")
    for key, value in expected.items():
        if fields.get(key) != value:
            raise ValueError("RC manifest mismatch: " + key)
    if table_sha != layout.EXPECTED_TABLE_SHA256 or len(data) > layout.APP_SLOT_SIZE:
        raise ValueError("RC partition table/image size violation")
    issue = manifest.identity_problem(fields, source_commit=BASE, source_state="DIRTY",
                                      layout_id=layout.LAYOUT_ID, ota_ingest="0", application_sha256=digest(data))
    if issue:
        raise ValueError(issue)
    for token in (BASE, fields["BUILD_ID"], fields["FW_VERSION"], fields["MOTION_STACK"], layout.LAYOUT_MARKER):
        if token.encode() not in data:
            raise ValueError("image identity absent: " + token)
    flash = manifest.verify_manifest(fields, head_commit=BASE, expected_fqbn=FQBN, tree_state="DIRTY",
        binary_exists=True, binary_size=len(data), binary_sha256=digest(data), requested_profile="ROBOT_POWERED",
        requested_ota_ingest="0", build_partition_table_sha256=table_sha, binary_embeds_layout_id=True)
    if flash.ok or flash.reason != manifest.Refusal.TREE_NOT_CLEAN:
        raise ValueError("existing clean-tree flash gate must remain refusing")
    return dict(schema="MATDOG_SOFTWARE_RC_RECEIPT_V1", software_artifact_verified=True,
                hardware_validated=False, flash_authorized=False, motion_authorized=False, ota_ingest=False,
                source_base=BASE, source_state="DIRTY", source_tree_sha256=current["source_tree_sha256"],
                source_file_count=len(current["files"]), application_sha256=digest(data), application_size=len(data),
                layout=layout.LAYOUT_ID, partition_table_sha256=table_sha, hardware_profile="ROBOT_POWERED",
                manifest_sha256=digest((artifacts / manifest.MANIFEST_FILENAME).read_bytes()),
                flash_gate=flash.reason, review_commit_clean_rebuild_required=True)


def external_output(path):
    path = path.resolve()
    if path == ROOT or path.is_relative_to(ROOT):
        raise ValueError("receipt output must be outside the worktree")
    path.parent.mkdir(parents=True, exist_ok=True)
    return path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("mode", choices=("snapshot", "verify"))
    parser.add_argument("--snapshot", type=Path, required=True)
    parser.add_argument("--artifacts", type=Path)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    try:
        if args.mode == "snapshot":
            result = inventory()
            external_output(args.snapshot).write_text(json.dumps(result, indent=2) + "\n")
        else:
            if not args.artifacts or not args.output:
                parser.error("verify requires --artifacts and --output")
            result = verify(json.loads(args.snapshot.read_text()), args.artifacts)
            external_output(args.output).write_text(json.dumps(result, indent=2) + "\n")
        print(json.dumps({k: v for k, v in result.items() if k not in ("files", "status")}))
    except (ValueError, OSError, KeyError) as exc:
        parser.exit(1, "RC_REFUSED=" + str(exc) + "\n")


if __name__ == "__main__":
    main()
