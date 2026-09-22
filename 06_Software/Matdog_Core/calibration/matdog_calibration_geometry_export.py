#!/usr/bin/env python3
"""Export the canonical Geometry Compiler V5 bundle as a compact Controller profile.

This is a REDUCTION, not a second geometry engine. It computes no collision,
no forward kinematics and no contact search: every number it emits is copied
from the canonical V5 artifacts, converted to integer micro-radians, and
bound to the SHA256 provenance of the inputs that produced it.

  URDF + collision meshes
        -> Geometry Compiler V5           (offline, unchanged)
        -> canonical bundle JSON          (unchanged, in 09_Logs)
        -> THIS EXPORTER                  (pure reduction)
        -> CalibrationGeometryProfileData.h
        -> Controller firmware            (verifies provenance, executes
                                           prevalidated primitives only)

Why a generated C++ header rather than a parsed blob: the Controller needs no
JSON parser, no heap and no mesh on the ESP32-S3, and the profile cannot be
swapped under a running image. Regenerating geometry therefore requires a
rebuild, which is the correct cost for data that authorises motion.

Refuses to write anything if the inputs do not match the manifest's recorded
hashes. Deterministic: the same bundle always produces byte-identical output.

Usage:
    python3 matdog_calibration_geometry_export.py [--check]

--check verifies that the committed header matches what this exporter would
produce, without writing. That is what the firmware static audit runs.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import re
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[3]

BUNDLE_DIR = REPO_ROOT / "09_Logs/Validation_Reports/Geometry_Compiler"
BUNDLE_ID = "2026-08-11_131818_MATDOG_GEOMETRY_V5_REMEDIATION_BENCHMARK_D_W4"

RUN_MANIFEST = BUNDLE_DIR / f"{BUNDLE_ID}_RUN_MANIFEST.json"
ENDPOINT_PROFILE = BUNDLE_DIR / f"{BUNDLE_ID}_ENDPOINT_PROFILE.json"
PATH_PARKING = BUNDLE_DIR / f"{BUNDLE_ID}_PATH_PARKING.json"
SAFETY_POLICY = (BUNDLE_DIR /
                 "2026-08-11_132758_MATDOG_GEOMETRY_V5_G12_FINAL_EXTERNAL_SAFETY_POLICY.json")

URDF = REPO_ROOT / "03_CAD/URDF/matt_robodog_rev00/matt_robodog_rev00.urdf"
MESH_MANIFEST = REPO_ROOT / "03_CAD/URDF/matt_robodog_rev00/SHA256SUMS.txt"
ALLOCATION = REPO_ROOT / "06_Software/Matdog_Core/config/MATDOG_SERVO_ALLOCATION.yaml"

OUTPUT = (REPO_ROOT /
          "05_Firmware/MATDOG_Controller/src/actuator/CalibrationGeometryProfileData.h")

SCHEMA_VERSION = "matdog.calibration_geometry_profile.bootstrap.v1"

# The exporter's own unit. Micro-radians in int32: the largest magnitude in the
# bundle is ~2.13 rad and the finest meaningful quantity is the compiler's
# 1e-4 rad bisection resolution, so 1e-6 rad resolution is two orders finer
# than the evidence and cannot lose a distinction the geometry actually made.
URAD = 1_000_000.0

LEG_BY_PREFIX = {"lf": "LF", "rf": "RF", "rh": "RH", "lh": "LH"}
KIND_BY_TOKEN = {"hip": "HIP", "upper": "UPPER", "lower": "LOWER"}

# The four legs in the calibration domain's enum order.
LEG_ORDER = ["LF", "RF", "RH", "LH"]
KIND_ORDER = ["HIP", "UPPER", "LOWER"]


class ExportError(RuntimeError):
    pass


def sha256_file(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def to_urad(value_rad: float) -> int:
    """Round half away from zero, so the magnitude never rounds downward into
    a value the geometry did not prove."""
    scaled = value_rad * URAD
    return int(math.floor(scaled + 0.5)) if scaled >= 0 else int(math.ceil(scaled - 0.5))


def split_joint_name(joint_name: str) -> tuple[str, str]:
    """`lf_upper_leg_joint` -> (LF, UPPER). The binding is VERIFIED against the
    URDF motorId and the servo allocation bus id below; this is only the
    lookup key, never the authority."""
    m = re.match(r"^(lf|rf|rh|lh)_(hip|upper|lower)(?:_leg)?_joint$", joint_name)
    if not m:
        raise ExportError(f"unrecognised leg joint name: {joint_name!r}")
    return LEG_BY_PREFIX[m.group(1)], KIND_BY_TOKEN[m.group(2)]


def load_urdf_joints() -> dict[str, dict]:
    root = ET.parse(URDF).getroot()
    out = {}
    for joint in root.findall("joint"):
        if joint.get("type") != "revolute":
            continue
        name = joint.get("name")
        limit = joint.find("limit")
        motor_id = motor_dir = None
        for element in joint.iter():
            if element.tag == "motorId":
                motor_id = int(element.text)
            elif element.tag == "motorDirection":
                motor_dir = int(element.text)
        if motor_id is None or motor_dir is None:
            raise ExportError(f"{name}: URDF revolute joint without hardware metadata")
        out[name] = {
            "motor_id": motor_id,
            # SPECIFICATION data. This is the sign the URDF model expects, NOT a
            # measured witness for the current installation. Direction evidence
            # is current calibration work; this value is only what that work
            # will be compared against.
            "motor_direction": motor_dir,
            "lower_rad": float(limit.get("lower")),
            "upper_rad": float(limit.get("upper")),
        }
    if len(out) != 12:
        raise ExportError(f"expected 12 revolute joints, found {len(out)}")
    return out


def load_allocation() -> dict[str, dict]:
    """physical unit -> joint -> bus id, from the 2026-08-27 campaign."""
    text = ALLOCATION.read_text(encoding="utf-8")
    blocks = re.findall(
        r"- unit: (\S+)\n\s+joint: (\S+)\n\s+bus_id: (\d+)\n(.*?)(?=\n  - unit:|\Z)",
        text, re.S)
    out = {}
    for unit, joint, bus_id, rest in blocks:
        offset = re.search(r"position_offset: (-?\d+)", rest)
        center = re.search(r"center_physical_raw: (\d+)", rest)
        error = re.search(r"center_error_ticks: (-?\d+)", rest)
        out[joint] = {
            "unit": unit,
            "bus_id": int(bus_id),
            "position_offset": int(offset.group(1)) if offset else None,
            "center_physical_raw": int(center.group(1)) if center else None,
            "center_error_ticks": int(error.group(1)) if error else None,
        }
    if len(out) != 17:
        raise ExportError(f"expected 17 allocated units, found {len(out)}")
    return out


def verify_bundle_provenance(manifest: dict) -> None:
    """Every input the canonical run was gated on must still hash the same."""
    for rel, expected in manifest["provenance"]["input_file_sha256"].items():
        actual = sha256_file(REPO_ROOT / rel)
        if actual != expected:
            raise ExportError(f"input drift: {rel}\n  manifest {expected}\n  actual   {actual}")

    calibration_dir = Path(__file__).resolve().parent
    for name, expected in manifest["provenance"]["canonical_semantic_source_file_sha256"].items():
        actual = sha256_file(calibration_dir / name)
        if actual != expected:
            raise ExportError(f"compiler source drift: {name}\n  manifest {expected}\n"
                              f"  actual   {actual}")

    for artifact in manifest["artifacts"].values():
        path = REPO_ROOT / artifact["relative_path"]
        actual = sha256_file(path)
        if actual != artifact["file_sha256"]:
            raise ExportError(f"artifact drift: {artifact['relative_path']}")


def build_records():
    manifest = json.loads(RUN_MANIFEST.read_text(encoding="utf-8"))
    verify_bundle_provenance(manifest)

    endpoints = json.loads(ENDPOINT_PROFILE.read_text(encoding="utf-8"))
    parking = json.loads(PATH_PARKING.read_text(encoding="utf-8"))
    policy = json.loads(SAFETY_POLICY.read_text(encoding="utf-8"))

    if policy["input_parking_artifact"]["semantic_content_sha256"] != \
            parking["semantic_content_sha256"]:
        raise ExportError("the safety policy was computed from a different parking artifact")

    urdf_joints = load_urdf_joints()
    allocation = load_allocation()

    # --- identity cross-check, name-independent -----------------------------
    # The URDF's motorId and the 2026-08-27 allocation's bus_id must agree for
    # all twelve leg joints. A disagreement means the model and the machine
    # describe different robots, which is a hard failure rather than a warning.
    for joint_name, urdf in urdf_joints.items():
        leg, kind = split_joint_name(joint_name)
        allocation_key = f"{leg}_{'UPPER' if kind == 'UPPER' else kind}"
        entry = allocation.get(allocation_key)
        if entry is None:
            raise ExportError(f"{joint_name}: no allocation entry for {allocation_key}")
        if entry["bus_id"] != urdf["motor_id"]:
            raise ExportError(
                f"{joint_name}: URDF motorId {urdf['motor_id']} != allocation bus id "
                f"{entry['bus_id']} for unit {entry['unit']}")
        if entry["position_offset"] != 0:
            raise ExportError(f"{allocation_key}: PositionOffset is not 0")

    policy_by_endpoint = {row["endpoint_id"]: row for row in policy["endpoint_policy_results"]}
    parking_by_endpoint = {plan["endpoint_id"]: plan for plan in parking["plans"]}

    # --- per-endpoint reduction --------------------------------------------
    endpoint_records = []
    clear_by_joint: dict[str, dict[str, float]] = {}
    max_rounding_error = 0.0

    for search in sorted(endpoints["endpoint_searches"],
                         key=lambda s: s["identity"]["presentation_id"]):
        identity = search["identity"]
        endpoint_id = identity["presentation_id"]
        joint_name = identity["joint_name"]
        side = identity["limit_side"]
        leg, kind = split_joint_name(joint_name)

        contact = search["geometric_contact"]
        obstruction = search["path_obstruction"]
        plan = parking_by_endpoint[endpoint_id]
        verdict = policy_by_endpoint[endpoint_id]

        if contact["status"] != "GEOMETRIC_CONTACT_FOUND":
            raise ExportError(f"{endpoint_id}: no geometric contact in the canonical bundle")

        # The nearest PROVEN-CLEAR angle travelling from q=0 toward this side:
        # the obstruction bracket when the path is obstructed before contact,
        # otherwise the contact bracket. Conservative by construction - it is
        # the last angle the compiler sampled and found clear, not the
        # bisected boundary itself.
        if obstruction["angle_rad"] is not None:
            clear_rad = abs(obstruction["bracket"]["clear_rad"])
            clear_source = "PATH_OBSTRUCTION_BRACKET"
        else:
            clear_rad = abs(contact["bracket"]["clear_rad"])
            clear_source = "GEOMETRIC_CONTACT_BRACKET"
        clear_by_joint.setdefault(joint_name, {})[side] = clear_rad

        aux_leg = aux_kind = ""
        aux_target = 0
        parking_config = plan["parking_configuration_rad"]
        if plan["outcome"] == "FEASIBLE_1DOF_PLAN_FOUND":
            if len(parking_config) != 1:
                raise ExportError(f"{endpoint_id}: 1-DOF plan with "
                                  f"{len(parking_config)} parked joints")
            aux_name, aux_value = next(iter(parking_config.items()))
            aux_leg, aux_kind = split_joint_name(aux_name)
            aux_target = to_urad(aux_value)
            max_rounding_error = max(max_rounding_error,
                                     abs(aux_value - aux_target / URAD))
        elif plan["outcome"] != "NOT_NEEDED":
            raise ExportError(f"{endpoint_id}: unsupported parking outcome "
                              f"{plan['outcome']!r}")
        elif parking_config:
            raise ExportError(f"{endpoint_id}: NOT_NEEDED plan carries a parking pose")

        for value in (contact["angle_rad"], clear_rad, search["declared_limit_rad"]):
            max_rounding_error = max(max_rounding_error, abs(value - to_urad(value) / URAD))

        endpoint_records.append({
            "endpoint_id": endpoint_id,
            "leg": leg,
            "kind": kind,
            "side": "MIN_SIDE" if side == "min" else "MAX_SIDE",
            "target_domain": plan["target_domain"],
            "parking_outcome": plan["outcome"],
            "clearance_policy": verdict["policy_result"],
            "contact_urad": to_urad(contact["angle_rad"]),
            "clear_urad": to_urad(clear_rad if contact["angle_rad"] >= 0 else -clear_rad),
            "declared_limit_urad": to_urad(search["declared_limit_rad"]),
            "clear_source": clear_source,
            "aux_leg": aux_leg,
            "aux_kind": aux_kind,
            "aux_target_urad": aux_target,
            "blocking_relation": plan["first_refined_blocking_relation"] or "NONE",
        })

    # --- per-joint bootstrap envelope --------------------------------------
    joint_records = []
    for joint_name in sorted(urdf_joints):
        leg, kind = split_joint_name(joint_name)
        urdf = urdf_joints[joint_name]
        entry = allocation[f"{leg}_{kind}"]
        sides = clear_by_joint[joint_name]

        # SYMMETRIC by construction. A direction-verification move is commanded
        # as a raw tick delta BEFORE the sign of the joint is known, so the same
        # magnitude must be proven clear on BOTH sides - whichever way the joint
        # actually turns, the excursion stays inside geometry the compiler
        # sampled and found clear.
        half_span_rad = min(sides["min"], sides["max"])

        joint_records.append({
            "joint_name": joint_name,
            "leg": leg,
            "kind": kind,
            "unit": entry["unit"],
            "bus_id": entry["bus_id"],
            "urdf_motor_direction": urdf["motor_direction"],
            "urdf_lower_urad": to_urad(urdf["lower_rad"]),
            "urdf_upper_urad": to_urad(urdf["upper_rad"]),
            "clear_half_span_urad": to_urad(half_span_rad),
            "provisioned_center_raw": entry["center_physical_raw"],
            "provisioned_center_error_ticks": entry["center_error_ticks"],
        })

    provenance = {
        "bundle_id": BUNDLE_ID,
        "urdf_sha256": manifest["provenance"]["urdf_sha256"],
        "mesh_manifest_sha256": sha256_file(MESH_MANIFEST),
        "endpoint_semantic_sha256": manifest["semantic_content_sha256"]["endpoint_profile"],
        "parking_semantic_sha256": manifest["semantic_content_sha256"]["parking_v2"],
        "safety_policy_semantic_sha256": policy["semantic_content_sha256"],
        "allocation_sha256": sha256_file(ALLOCATION),
        "run_manifest_content_sha256": manifest["manifest_content_sha256"],
        "max_rounding_error_rad": max_rounding_error,
    }
    return provenance, joint_records, endpoint_records


def render(provenance, joints, endpoints) -> str:
    def enum(prefix, value):
        return f"{prefix}::{value}"

    lines = []
    add = lines.append
    add("// GENERATED FILE - DO NOT EDIT BY HAND.")
    add("//")
    add("// Produced by 06_Software/Matdog_Core/calibration/"
        "matdog_calibration_geometry_export.py")
    add("// from the canonical Geometry Compiler V5 bundle")
    add(f"//   {provenance['bundle_id']}")
    add("//")
    add("// Every value below is COPIED from that bundle and converted to integer")
    add("// micro-radians. No geometry is computed here and none is computed on the")
    add("// device: the Controller verifies provenance and executes prevalidated")
    add("// plan primitives only. Regenerate with the exporter; never patch a value.")
    add("//")
    add(f"// max rounding error introduced by the micro-radian conversion: "
        f"{provenance['max_rounding_error_rad']:.3e} rad")
    add("//   (the compiler's own bisection resolution is 1.0e-4 rad)")
    add("")
    add("#ifndef MATDOG_ACTUATOR_CALIBRATION_GEOMETRY_PROFILE_DATA_H")
    add("#define MATDOG_ACTUATOR_CALIBRATION_GEOMETRY_PROFILE_DATA_H")
    add("")
    add('#include "CalibrationGeometryProfile.h"')
    add("")
    add("namespace matdog {")
    add("namespace actuator {")
    add("namespace geometry_data {")
    add("")
    add(f'constexpr char kSchemaVersion[] = "{SCHEMA_VERSION}";')
    add(f'constexpr char kBundleId[] = "{provenance["bundle_id"]}";')
    add("")
    add("// The immutable inputs the canonical V5 run was gated on. The Controller")
    add("// refuses to act on a profile whose provenance it cannot match.")
    add("constexpr GeometryProvenance kProvenance = {")
    add(f'    "{provenance["urdf_sha256"]}",')
    add(f'    "{provenance["mesh_manifest_sha256"]}",')
    add(f'    "{provenance["endpoint_semantic_sha256"]}",')
    add(f'    "{provenance["parking_semantic_sha256"]}",')
    add(f'    "{provenance["safety_policy_semantic_sha256"]}",')
    add(f'    "{provenance["allocation_sha256"]}",')
    add("};")
    add("")
    add(f"constexpr uint8_t kJointCount = {len(joints)};")
    add("constexpr GeometryJointRecord kJoints[kJointCount] = {")
    for j in joints:
        add(f"    // {j['joint_name']}  unit {j['unit']}  bus {j['bus_id']}")
        add("    {" +
            f"{{{enum('calibration::Leg', j['leg'])}, "
            f"{enum('calibration::JointKind', j['kind'])}, \"{j['unit']}\"}}, "
            f"{j['bus_id']}, {j['urdf_motor_direction']}, "
            f"{j['urdf_lower_urad']}, {j['urdf_upper_urad']}, "
            f"{j['clear_half_span_urad']}, "
            f"{j['provisioned_center_raw']}, {j['provisioned_center_error_ticks']}" +
            "},")
    add("};")
    add("")
    add(f"constexpr uint8_t kEndpointCount = {len(endpoints)};")
    add("constexpr GeometryEndpointRecord kEndpoints[kEndpointCount] = {")
    for e in endpoints:
        aux = ("true, " + enum("calibration::Leg", e["aux_leg"]) + ", " +
               enum("calibration::JointKind", e["aux_kind"])
               if e["aux_leg"] else
               "false, calibration::Leg::LF, calibration::JointKind::HIP")
        add(f"    // {e['endpoint_id']}  {e['target_domain']}  {e['parking_outcome']}"
            f"  {e['clearance_policy']}  [{e['clear_source']}]")
        add("    {" +
            f"{enum('calibration::Leg', e['leg'])}, "
            f"{enum('calibration::JointKind', e['kind'])}, "
            f"{enum('calibration::ContactSide', e['side'])}, "
            f"{enum('TargetDomain', e['target_domain'])}, "
            f"{enum('ParkingOutcome', e['parking_outcome'])}, "
            f"{enum('ClearancePolicyResult', e['clearance_policy'])}, "
            f"{e['contact_urad']}, {e['clear_urad']}, {e['declared_limit_urad']}, "
            f"{aux}, {e['aux_target_urad']}" +
            "},")
    add("};")
    add("")
    add("}  // namespace geometry_data")
    add("}  // namespace actuator")
    add("}  // namespace matdog")
    add("")
    add("#endif  // MATDOG_ACTUATOR_CALIBRATION_GEOMETRY_PROFILE_DATA_H")
    return "\n".join(lines) + "\n"


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--check", action="store_true",
                        help="verify the committed header without writing")
    args = parser.parse_args()

    try:
        provenance, joints, endpoints = build_records()
        rendered = render(provenance, joints, endpoints)
    except ExportError as exc:
        print(f"CALIBRATION_GEOMETRY_EXPORT = FAIL\n  {exc}", file=sys.stderr)
        return 1

    if args.check:
        if not OUTPUT.exists():
            print(f"CALIBRATION_GEOMETRY_EXPORT = FAIL\n  {OUTPUT} does not exist",
                  file=sys.stderr)
            return 1
        current = OUTPUT.read_text(encoding="utf-8")
        if current != rendered:
            print("CALIBRATION_GEOMETRY_EXPORT = FAIL\n"
                  f"  {OUTPUT.relative_to(REPO_ROOT)} does not match the canonical bundle.\n"
                  "  Regenerate with matdog_calibration_geometry_export.py; do not edit it.",
                  file=sys.stderr)
            return 1
        print("CALIBRATION_GEOMETRY_EXPORT = PASS (header matches the canonical bundle)")
        return 0

    OUTPUT.write_text(rendered, encoding="utf-8")
    print(f"CALIBRATION_GEOMETRY_EXPORT = PASS\n"
          f"  wrote {OUTPUT.relative_to(REPO_ROOT)}\n"
          f"  {len(joints)} joints, {len(endpoints)} endpoints\n"
          f"  max rounding error {provenance['max_rounding_error_rad']:.3e} rad")
    return 0


if __name__ == "__main__":
    sys.exit(main())
