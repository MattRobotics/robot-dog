#!/usr/bin/env python3
"""Build and finalize the persistent MATDOG LF calibration profile.

The native Rust calibrator emits one machine-readable record per LF joint.
This module validates those records against the exact URDF file used for the
run, creates the EEPROM freeze plan, and promotes the profile only after the
serial provisioner reports verified persistent readback.
"""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import tempfile
import xml.etree.ElementTree as ET

FORMAT = "MATDOG_LF_PROFILE_V1"
PREFIX = FORMAT + "|"
BUS_SERIAL_DEFAULT = "5B14114953"
JOINTS = {
    11: "lf_lower_leg_joint",
    12: "lf_upper_leg_joint",
    13: "lf_hip_joint",
}


class ProfileError(RuntimeError):
    pass


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def atomic_write_text(path: Path, text: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, temporary_name = tempfile.mkstemp(prefix=path.name + ".", dir=path.parent)
    temporary = Path(temporary_name)
    try:
        with open(fd, "w", encoding="utf-8", closefd=True) as stream:
            stream.write(text)
            stream.flush()
        temporary.replace(path)
    except Exception:
        temporary.unlink(missing_ok=True)
        raise


def parse_record(line: str) -> dict[str, object]:
    payload = line.strip()
    marker = payload.find(PREFIX)
    if marker < 0:
        raise ProfileError("missing MATDOG LF machine-record prefix")
    payload = payload[marker + len(PREFIX) :]
    values: dict[str, str] = {}
    for component in payload.split("|"):
        key, separator, value = component.partition("=")
        if not separator or not key or key in values:
            raise ProfileError(f"invalid profile component: {component!r}")
        values[key] = value

    required = {
        "joint",
        "joint_name",
        "motor_id",
        "direction",
        "urdf_min_delta",
        "urdf_max_delta",
        "urdf_min_tick",
        "urdf_max_tick",
        "coarse_min",
        "coarse_max",
        "fine_min_1",
        "fine_min_2",
        "fine_max_1",
        "fine_max_2",
        "repeatability_min",
        "repeatability_max",
        "contact_min",
        "contact_max",
        "q0_fixed",
        "q0_affine",
        "endpoint_disagreement",
        "q0_shift",
        "scale_permille",
        "safe_min_tick",
        "safe_max_tick",
        "accepted",
    }
    missing = sorted(required - values.keys())
    extra = sorted(values.keys() - required)
    if missing or extra:
        raise ProfileError(f"profile fields mismatch: missing={missing}, extra={extra}")

    integer_fields = required - {"joint", "joint_name", "accepted"}
    record: dict[str, object] = {
        key: int(value) if key in integer_fields else value
        for key, value in values.items()
    }
    record["accepted"] = values["accepted"] == "true"
    motor_id = int(record["motor_id"])
    if motor_id not in JOINTS or record["joint_name"] != JOINTS[motor_id]:
        raise ProfileError(f"unexpected LF joint identity: M{motor_id} {record['joint_name']}")
    if record["accepted"] is not True:
        raise ProfileError(f"M{motor_id} URDF/freeze gate rejected")
    for key in (
        "urdf_min_tick",
        "urdf_max_tick",
        "coarse_min",
        "coarse_max",
        "fine_min_1",
        "fine_min_2",
        "fine_max_1",
        "fine_max_2",
        "contact_min",
        "contact_max",
        "q0_fixed",
        "q0_affine",
        "safe_min_tick",
        "safe_max_tick",
    ):
        value = int(record[key])
        if not 0 <= value <= 4095:
            raise ProfileError(f"M{motor_id} {key} outside unsigned ST3215 range: {value}")
    return record


def records_from_log(path: Path) -> dict[int, dict[str, object]]:
    records: dict[int, dict[str, object]] = {}
    for line in path.read_text(encoding="utf-8", errors="replace").splitlines():
        if PREFIX not in line:
            continue
        record = parse_record(line)
        records[int(record["motor_id"])] = record
    if set(records) != set(JOINTS):
        raise ProfileError(f"incomplete LF record set: found={sorted(records)}, expected={sorted(JOINTS)}")
    return records


def urdf_limits(path: Path) -> dict[str, tuple[float, float]]:
    root = ET.parse(path).getroot()
    found: dict[str, tuple[float, float]] = {}
    for joint in root.findall("joint"):
        name = joint.attrib.get("name", "")
        if name not in JOINTS.values():
            continue
        limit = joint.find("limit")
        if limit is None or "lower" not in limit.attrib or "upper" not in limit.attrib:
            raise ProfileError(f"URDF joint {name} has no lower/upper limit")
        found[name] = (float(limit.attrib["lower"]), float(limit.attrib["upper"]))
    if set(found) != set(JOINTS.values()):
        raise ProfileError(
            f"URDF LF joints mismatch: found={sorted(found)}, expected={sorted(JOINTS.values())}"
        )
    return found


def radians_to_ticks(value: float) -> int:
    return round(value * 4096.0 / (2.0 * math.pi))


def verify_urdf(records: dict[int, dict[str, object]], urdf_path: Path) -> dict[str, object]:
    limits = urdf_limits(urdf_path)
    verified: dict[str, object] = {}
    for motor_id, record in records.items():
        name = str(record["joint_name"])
        lower, upper = limits[name]
        lower_ticks = radians_to_ticks(lower)
        upper_ticks = radians_to_ticks(upper)
        expected_lower = int(record["urdf_min_delta"])
        expected_upper = int(record["urdf_max_delta"])
        if abs(lower_ticks - expected_lower) > 1 or abs(upper_ticks - expected_upper) > 1:
            raise ProfileError(
                f"URDF/source mismatch for {name}: URDF={lower_ticks}/{upper_ticks} tick, "
                f"calibrator={expected_lower}/{expected_upper} tick"
            )
        verified[name] = {
            "lower_rad": lower,
            "upper_rad": upper,
            "lower_tick_delta": lower_ticks,
            "upper_tick_delta": upper_ticks,
        }
    return verified


def stage_profile(args: argparse.Namespace) -> None:
    station_log = Path(args.station_log).resolve()
    urdf_path = Path(args.urdf).resolve()
    records = records_from_log(station_log)
    verified_urdf = verify_urdf(records, urdf_path)
    urdf_sha = sha256_file(urdf_path)

    profile = {
        "format": FORMAT,
        "status": "LF_STAGED",
        "frozen": False,
        "created_at_utc": utc_now(),
        "bus_serial": args.bus_serial,
        "urdf": {
            "path": str(urdf_path),
            "sha256": urdf_sha,
            "verified_limits": verified_urdf,
        },
        "source": {
            "station_log": str(station_log),
            "calibrator_commit": args.calibrator_commit,
        },
        "leg": "LF",
        "motors": {str(motor_id): records[motor_id] for motor_id in sorted(records)},
    }
    atomic_write_text(Path(args.output_json), json.dumps(profile, indent=2, sort_keys=True) + "\n")

    lines = [
        "format=MATDOG_LF_STAGE_V1",
        "status=LF_STAGED",
        f"bus_serial={args.bus_serial}",
        f"urdf_sha256={urdf_sha}",
    ]
    for motor_id in sorted(records):
        record = records[motor_id]
        prefix = f"motor.{motor_id}."
        lines.extend(
            [
                prefix + f"joint_name={record['joint_name']}",
                prefix + f"estimated_q0_tick={record['q0_affine']}",
                prefix + "accepted=true",
            ]
        )
    atomic_write_text(Path(args.output_plan), "\n".join(lines) + "\n")
    print("MATDOG_LF_PROFILE_STAGE=PASS")


def parse_key_value(path: Path) -> dict[str, str]:
    result: dict[str, str] = {}
    for raw in path.read_text(encoding="utf-8").splitlines():
        line = raw.strip()
        if not line or line.startswith("#"):
            continue
        key, separator, value = line.partition("=")
        if not separator or key in result:
            raise ProfileError(f"invalid key/value line: {line!r}")
        result[key] = value
    return result


def finalize_profile(args: argparse.Namespace) -> None:
    staged = json.loads(Path(args.stage_json).read_text(encoding="utf-8"))
    if staged.get("format") != FORMAT or staged.get("status") != "LF_STAGED":
        raise ProfileError("input profile is not an LF_STAGED profile")
    freeze = parse_key_value(Path(args.freeze_result))
    if freeze.get("format") != "MATDOG_LF_FREEZE_RESULT_V1" or freeze.get("status") != "LF_FROZEN":
        raise ProfileError("EEPROM freeze result is not verified LF_FROZEN")
    if freeze.get("bus_serial") != staged["bus_serial"]:
        raise ProfileError("freeze bus serial does not match staged profile")
    if freeze.get("urdf_sha256") != staged["urdf"]["sha256"]:
        raise ProfileError("freeze URDF hash does not match staged profile")

    for motor_id in sorted(JOINTS):
        prefix = f"motor.{motor_id}."
        if freeze.get(prefix + "position") is None:
            raise ProfileError(f"freeze result missing M{motor_id}")
        position = int(freeze[prefix + "position"])
        if abs(position - 2048) > 10:
            raise ProfileError(f"M{motor_id} final displayed q0 is {position}, not 2048±10")
        staged["motors"][str(motor_id)]["eeprom"] = {
            "old_offset": int(freeze[prefix + "old_offset"]),
            "new_offset": int(freeze[prefix + "new_offset"]),
            "verified_position": position,
            "lock": int(freeze[prefix + "lock"]),
            "torque_enabled": int(freeze[prefix + "torque_enabled"]),
        }

    staged["status"] = "LF_FROZEN"
    staged["frozen"] = True
    staged["frozen_at_utc"] = utc_now()
    staged["eeprom_backup"] = str(Path(args.backup).resolve())
    output = Path(args.output_json)
    atomic_write_text(output, json.dumps(staged, indent=2, sort_keys=True) + "\n")

    global_path = Path(args.global_index)
    if global_path.exists():
        global_profile = json.loads(global_path.read_text(encoding="utf-8"))
    else:
        global_profile = {
            "format": "MATDOG_GLOBAL_PROFILE_V1",
            "status": "PARTIAL",
            "legs": {"LF": None, "RF": None, "RH": None, "LH": None},
        }
    global_profile["urdf_sha256"] = staged["urdf"]["sha256"]
    global_profile["bus_serial"] = staged["bus_serial"]
    global_profile["updated_at_utc"] = utc_now()
    global_profile["legs"]["LF"] = {
        "status": "LF_FROZEN",
        "profile": str(output.resolve()),
        "sha256": sha256_file(output),
    }
    frozen_count = sum(
        1
        for value in global_profile["legs"].values()
        if isinstance(value, dict) and str(value.get("status", "")).endswith("_FROZEN")
    )
    global_profile["frozen_legs"] = frozen_count
    global_profile["status"] = "ACTIVE" if frozen_count == 4 else "PARTIAL"
    atomic_write_text(global_path, json.dumps(global_profile, indent=2, sort_keys=True) + "\n")
    print("MATDOG_LF_PROFILE_FINALIZE=PASS")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser()
    subparsers = parser.add_subparsers(dest="command", required=True)

    stage = subparsers.add_parser("stage")
    stage.add_argument("--station-log", required=True)
    stage.add_argument("--urdf", required=True)
    stage.add_argument("--bus-serial", default=BUS_SERIAL_DEFAULT)
    stage.add_argument("--calibrator-commit", required=True)
    stage.add_argument("--output-json", required=True)
    stage.add_argument("--output-plan", required=True)
    stage.set_defaults(handler=stage_profile)

    finalize = subparsers.add_parser("finalize")
    finalize.add_argument("--stage-json", required=True)
    finalize.add_argument("--freeze-result", required=True)
    finalize.add_argument("--backup", required=True)
    finalize.add_argument("--output-json", required=True)
    finalize.add_argument("--global-index", required=True)
    finalize.set_defaults(handler=finalize_profile)
    return parser


def main() -> int:
    args = build_parser().parse_args()
    try:
        args.handler(args)
    except (ProfileError, OSError, ValueError, ET.ParseError) as error:
        print(f"MATDOG_LF_PROFILE=FAIL: {error}", flush=True)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
