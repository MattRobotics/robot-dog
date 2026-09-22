#!/usr/bin/env python3
"""Mutation tests for the static audit's Safe Actuator Layer boundaries.

Loads the REAL scripts/static_audit.py, runs the boundary checks against the
current firmware sources (must pass), then against single, targeted mutations
of those sources (each must fail, with the expected reason). A rule that a
mutation slips past is a rule that only looks enforced.

The families that matter:
  - purity: the policy core gaining Arduino, a bus handle, Serial or a write
    primitive must fail - it links into the host suite precisely because it
    cannot perform a write;
  - SAFE_OFF independence, in both directions: ServoBus learning about the
    policy, or the @SERVO SAFE_OFF branch consulting it, must fail;
  - operation classes: a persistent/provisioning write class, a torque-removal
    class, a missing class, or a count that drifts from the enum must fail;
  - limit provenance: dropping either half of the promotable/operational test,
    special-casing a replay, or letting a runtime translation unit hold the
    accepted-limit store must fail;
  - and the pre-existing torque guard, cross-checked here because "no Torque ON
    path in the default build" is this phase's claim too.

No hardware, no device I/O, no files written. Run directly or via
static_audit.py (check_safe_actuator_audit_mutation_suite).
"""
import importlib.util
import re
import sys
from pathlib import Path

SCRIPTS_DIR = Path(__file__).resolve().parent.parent
SKETCH_DIR = SCRIPTS_DIR.parent


def load_audit():
    spec = importlib.util.spec_from_file_location("static_audit", SCRIPTS_DIR / "static_audit.py")
    module = importlib.util.module_from_spec(spec)
    saved = sys.argv
    sys.argv = [str(SCRIPTS_DIR / "static_audit.py"), str(SKETCH_DIR)]
    try:
        spec.loader.exec_module(module)
    finally:
        sys.argv = saved
    return module


audit = load_audit()
BASE = [(p, audit.strip_comments(p.read_text(encoding="utf-8"))) for p in audit.iter_source_files()]

failures = []


def run_boundary_checks(files):
    audit.failures.clear()
    audit.check_safe_actuator_boundaries(files, SKETCH_DIR)
    return list(audit.failures)


def run_geometry_checks(files):
    audit.failures.clear()
    audit.check_calibration_geometry_boundaries(files, SKETCH_DIR)
    return list(audit.failures)


def run_offset_checks(files):
    audit.failures.clear()
    audit.check_position_offset_boundary(files, SKETCH_DIR)
    audit.check_forbidden_literals(files)
    return list(audit.failures)


def run_profile_checks(files):
    audit.failures.clear()
    audit.check_servo_profile_contract(files, SKETCH_DIR)
    return list(audit.failures)


def run_preflight_checks(files):
    audit.failures.clear()
    audit.check_h0_preflight_boundaries(files, SKETCH_DIR)
    return list(audit.failures)


def run_binding_checks(files):
    audit.failures.clear()
    audit.check_evidence_geometry_binding(files, SKETCH_DIR)
    return list(audit.failures)


def run_population_checks(files):
    audit.failures.clear()
    audit.check_servo_population_model(files, SKETCH_DIR)
    return list(audit.failures)


def run_direction_checks(files):
    audit.failures.clear()
    audit.check_direction_is_contractual(files, SKETCH_DIR)
    return list(audit.failures)


def run_torque_checks(files):
    audit.failures.clear()
    audit.check_torque_enable(files)
    audit.check_forbidden_literals(files)
    return list(audit.failures)


def mutate(filename, pattern, repl):
    """Returns BASE with exactly one regex substitution applied in `filename`."""
    out = []
    hits = 0
    for path, code in BASE:
        if path.name == filename:
            code, n = re.subn(pattern, repl, code, count=1)
            hits += n
        out.append((path, code))
    if hits != 1:
        raise AssertionError(f"mutation anchor {pattern!r} matched {hits} times in {filename}")
    return out


def expect_pass(name, found):
    if found:
        failures.append(f"{name}: expected no findings, got {found}")


def expect_fail(name, found, needle):
    if not found:
        failures.append(f"{name}: mutation was NOT caught")
        return
    if not any(needle in f for f in found):
        failures.append(f"{name}: caught, but no finding mentioned {needle!r}: {found}")


def case(name, filename, pattern, repl, needle, runner=run_boundary_checks):
    try:
        mutated = mutate(filename, pattern, repl)
    except AssertionError as exc:
        failures.append(f"{name}: {exc}")
        return
    expect_fail(name, runner(mutated), needle)


POLICY_H = "ActuatorWritePolicy.h"
POLICY_CPP = "ActuatorWritePolicy.cpp"
PROFILE_H = "CalibrationGeometryProfile.h"
PROFILE_CPP = "CalibrationGeometryProfile.cpp"
PROFILE_DATA_H = "CalibrationGeometryProfileData.h"
BUS_CPP = "ServoBus.cpp"
SERVO_PROFILE_CPP = "ServoProfile.cpp"
SERVO_PROFILE_DATA_H = "ServoProfileData.h"
PREFLIGHT_CPP = "ServoPreflight.cpp"
POPULATION_H = "ServoPopulation.h"
ROUTER_CPP = "CommandRouter.cpp"


def main():
    # The unmutated tree must be clean, or every "mutation caught" below would
    # be meaningless.
    expect_pass("baseline", run_boundary_checks(BASE))
    expect_pass("baseline torque/literals", run_torque_checks(BASE))

    # --- purity of the decision core --------------------------------------
    case("policy gains Arduino", POLICY_H,
         r'#include "\.\./core/OperatingMode\.h"',
         '#include "../core/OperatingMode.h"\n#include <Arduino.h>',
         "Arduino.h")
    case("policy gains a bus handle", POLICY_CPP,
         r"using calibration::JointIdentity;",
         "using calibration::JointIdentity;\nstatic SMS_STS g_bus;",
         "SMS_STS")
    case("policy gains the transport", POLICY_CPP,
         r"using calibration::JointIdentity;",
         "using calibration::JointIdentity;\nstatic servo::ServoBus* g_bus = nullptr;",
         "ServoBus")
    case("policy gains Serial", POLICY_CPP,
         r"  counters_\.resets\+\+;",
         '  Serial.println("reset");\n  counters_.resets++;',
         "Serial")

    # --- no write path anywhere under src/actuator/ ------------------------
    case("policy gains torque-on", POLICY_CPP,
         r"  counters_\.commits\+\+;",
         "  bus_.EnableTorque(1, 1);\n  counters_.commits++;",
         "EnableTorque")
    case("policy gains a goal-position write", POLICY_CPP,
         r"  counters_\.commits\+\+;",
         "  bus_.WritePos(1, 2048, 0);\n  counters_.commits++;",
         "WritePos")
    case("policy gains a synchronised write", POLICY_CPP,
         r"  counters_\.commits\+\+;",
         "  bus_.SyncWrite(1);\n  counters_.commits++;",
         "SyncWrite")

    # --- SAFE_OFF independence, both directions ---------------------------
    case("ServoBus learns about the policy", "ServoBus.h",
         r"class ServoBus \{",
         "class ServoBus {\n public:\n  void setPolicy(actuator::SafeActuatorPolicy*);",
         "SAFE_OFF")
    case("SAFE_OFF branch consults the policy", "CommandRouter.cpp",
         r"      printServoSafeOff\(id\);",
         "      actuator::WriteDecision d{};\n      (void)d;\n      printServoSafeOff(id);",
         "SAFE_OFF")

    # --- operation classes -------------------------------------------------
    case("a persistent write becomes expressible", POLICY_H,
         r"(  CALIBRATION_AUXILIARY_MOVE = 5,)",
         r"\1\n  EEPROM_WRITE               = 6,",
         "EEPROM")
    case("a provisioning write becomes expressible", POLICY_H,
         r"(  CALIBRATION_AUXILIARY_MOVE = 5,)",
         r"\1\n  POSITION_OFFSET_WRITE      = 6,",
         "OFFSET")
    case("torque removal becomes expressible", POLICY_H,
         r"(  CALIBRATION_AUXILIARY_MOVE = 5,)",
         r"\1\n  TORQUE_DISABLE             = 6,",
         "DISABLE")
    case("an operation class disappears", POLICY_H,
         r"  CALIBRATION_CONTACT_PROBE  = 3,[^\n]*",
         "",
         "CALIBRATION_CONTACT_PROBE")
    case("the operation count drifts upward", POLICY_H,
         r"kActuatorOperationCount = 6;",
         "kActuatorOperationCount = 7;",
         "kActuatorOperationCount")
    case("the operation count drifts downward", POLICY_H,
         r"kActuatorOperationCount = 6;",
         "kActuatorOperationCount = 5;",
         "kActuatorOperationCount")

    # --- limit provenance --------------------------------------------------
    case("provenance stops requiring a live session", POLICY_CPP,
         r"  if \(!calibration::mayPromote\(origin\)\) return false;",
         "",
         "mayPromote")
    case("provenance stops requiring promotion", POLICY_CPP,
         r"  return calibration::isOperationalEvidence\(state\);",
         "  return true;",
         "isOperationalEvidence")
    case("the policy special-cases a replay", POLICY_CPP,
         r"  if \(!present\) return false;",
         "  if (origin == calibration::CalibrationOrigin::HISTORICAL_REPLAY) return true;\n"
         "  if (!present) return false;",
         "HISTORICAL_REPLAY")
    case("a runtime unit holds the accepted-limit store", "Controller.h",
         r"class Controller \{",
         "class Controller {\n  actuator::ActuatorLimitTable limits_;",
         "ActuatorLimitTable")

    # --- one boundary ------------------------------------------------------
    case("a second policy instance appears", "CommandRouter.h",
         r"class CommandRouter \{",
         "class CommandRouter {\n  actuator::SafeActuatorPolicy policy_;",
         "SafeActuatorPolicy")

    # --- the calibration bootstrap geometry contract -----------------------
    expect_pass("baseline geometry", run_geometry_checks(BASE))

    case("the executability door drops the URDF domain", PROFILE_CPP,
         r"  if \(endpoint\.domain != TargetDomain::EXECUTABLE_URDF_DOMAIN\) return false;",
         "",
         "target domain", runner=run_geometry_checks)
    case("the executability door accepts any clearance", PROFILE_CPP,
         r"  return endpoint\.clearance == ClearancePolicyResult::PASS;",
         "  return true;",
         "PASS clearance", runner=run_geometry_checks)

    case("a diagnostic endpoint is promoted to executable", PROFILE_DATA_H,
         r"TargetDomain::DIAGNOSTIC_GEOMETRY_OUTSIDE_URDF_LIMITS, "
         r"ParkingOutcome::NOT_NEEDED, "
         r"ClearancePolicyResult::UNRESOLVED_LOWER_BOUND_BELOW_THRESHOLD",
         "TargetDomain::EXECUTABLE_URDF_DOMAIN, ParkingOutcome::NOT_NEEDED, "
         "ClearancePolicyResult::UNRESOLVED_LOWER_BOUND_BELOW_THRESHOLD",
         "unresolved clearance evidence", runner=run_geometry_checks)
    case("an obstructed plan loses its auxiliary", PROFILE_DATA_H,
         r"ParkingOutcome::FEASIBLE_1DOF_PLAN_FOUND, ClearancePolicyResult::PASS, "
         r"(-?\d+), (-?\d+), (-?\d+), true",
         r"ParkingOutcome::FEASIBLE_1DOF_PLAN_FOUND, ClearancePolicyResult::PASS, "
         r"\1, \2, \3, false",
         "fail closed", runner=run_geometry_checks)
    case("a parking pose reverts to a legacy 50 degree prerequisite", PROFILE_DATA_H,
         r"calibration::JointKind::UPPER, 610865\}",
         "calibration::JointKind::UPPER, 872665}",
         "superseded hardcoded prerequisite", runner=run_geometry_checks)
    case("the geometry profile learns floating point", PROFILE_H,
         r"using MicroRad = int32_t;",
         "using MicroRad = int32_t;\nusing Approx = double;",
         "floating point", runner=run_geometry_checks)
    case("the geometry profile reaches the bus", PROFILE_CPP,
         r"namespace actuator \{",
         "namespace actuator {\nstatic SMS_STS g_bus;",
         "SMS_STS", runner=run_geometry_checks)

    case("POSITION_COMMAND is routed through the geometry envelope", POLICY_CPP,
         r"  return operation == ActuatorOperation::DIRECTION_VERIFY;",
         "  return operation == ActuatorOperation::DIRECTION_VERIFY ||\n"
         "         operation == ActuatorOperation::POSITION_COMMAND;",
         "bootstrap envelope", runner=run_geometry_checks)
    case("a calibration class is routed through accepted limits", POLICY_CPP,
         r"  return operation == ActuatorOperation::POSITION_COMMAND;",
         "  return operation == ActuatorOperation::POSITION_COMMAND ||\n"
         "         operation == ActuatorOperation::CALIBRATION_CONTACT_PROBE;",
         "mutually exclusive", runner=run_geometry_checks)

    # --- B1: evidence is bound to the geometry it was measured under -------
    expect_pass("baseline binding", run_binding_checks(BASE))

    case("a joint bound loses its geometry tag", POLICY_H,
         r"  GeometryProvenanceTag geometry = kNoGeometryProvenance;\n  uint16_t min_tick",
         "  uint16_t min_tick",
         "GeometryProvenanceTag", runner=run_binding_checks)
    case("admit stops requiring a geometry", POLICY_CPP,
         r"  if \(!limit\.boundToGeometry\(\)\) return false;",
         "",
         "boundToGeometry", runner=run_binding_checks)
    case("transform admit stops requiring a geometry", POLICY_CPP,
         r"  if \(!transform\.boundToGeometry\(\)\) return false;",
         "",
         "boundToGeometry", runner=run_binding_checks)
    case("an unbound tag stops failing closed", POLICY_CPP,
         r"  if \(geometry == kNoGeometryProvenance\) return nullptr;\n"
         r"  const JointLimit\* entry = findAny\(joint\);",
         "  const JointLimit* entry = findAny(joint);",
         "fail closed", runner=run_binding_checks)
    case("the decision path looks evidence up by identity alone", POLICY_CPP,
         r"limits_\.find\(command\.joint, currentGeometryTag\(\)\)",
         "limits_.findAny(command.joint)",
         "findAny()", runner=run_binding_checks)
    case("the current tag stops checking the expected provenance", POLICY_CPP,
         r"  if \(!geometry_->provenanceMatches\(\*expected_provenance_\)\) "
         r"return kNoGeometryProvenance;",
         "",
         "provenanceMatches", runner=run_binding_checks)

    # --- PositionOffset: one read accessor, never a write ------------------
    expect_pass("baseline offset boundary", run_offset_checks(BASE))

    # THE case this gate exists for.
    case("a PositionOffset WRITE appears in the accessor", BUS_CPP,
         r"  const int raw = st_\.readWord\(static_cast<uint8_t>\(id\), SMS_STS_OFS_L\);",
         "  st_.writeWord(static_cast<uint8_t>(id), SMS_STS_OFS_L, 0);\n"
         "  const int raw = st_.readWord(static_cast<uint8_t>(id), SMS_STS_OFS_L);",
         "writeWord", runner=run_offset_checks)
    case("a PositionOffset write appears elsewhere", ROUTER_CPP,
         r"void CommandRouter::printServoRead\(int id\) \{",
         "void CommandRouter::printServoRead(int id) {\n"
         "  modules_.servo_bus->writeWord(id, SMS_STS_OFS_L, 0);",
         "WRITE", runner=run_offset_checks)
    case("the offset register is read outside the approved accessor", ROUTER_CPP,
         r"void CommandRouter::printServoRead\(int id\) \{",
         "void CommandRouter::printServoRead(int id) {\n"
         "  int ofs = readWord(id, 0x1F);\n  (void)ofs;",
         "0x1F", runner=run_offset_checks)
    case("the accessor stops being a read", BUS_CPP,
         r"  const int raw = st_\.readWord\(static_cast<uint8_t>\(id\), SMS_STS_OFS_L\);",
         "  const int raw = SMS_STS_OFS_L;",
         "readWord", runner=run_offset_checks)
    case("the offset decoder becomes sign-magnitude", SERVO_PROFILE_CPP,
         r"  return static_cast<int16_t>\(raw\);",
         "  return (raw & 0x8000) ? -(int16_t)(raw & 0x7FFF) : (int16_t)raw;",
         "two's-complement", runner=run_offset_checks)

    # --- the MATDOG_C018_V1 contract ---------------------------------------
    expect_pass("baseline profile contract", run_profile_checks(BASE))

    case("runtime RAM state leaks into the persistent profile", SERVO_PROFILE_DATA_H,
         r'\{0x27, 1, 200, "VelocityClosedLoopI"\},',
         '{0x27, 1, 200, "VelocityClosedLoopI"},\n    {0x30, 2, 1000, "TorqueLimit"},',
         "runtime RAM state", runner=run_profile_checks)
    case("a persistent register disappears", SERVO_PROFILE_DATA_H,
         r'\{0x09, 2, 0, "MinAngle"\},',
         "",
         "expected the canonical 20", runner=run_profile_checks)
    case("an unread register can fold into MATCH", SERVO_PROFILE_CPP,
         r"      return ProfileVerdict::INCOMPLETE;",
         "      return running;",
         "INCOMPLETE", runner=run_profile_checks)
    case("the profile contract reaches the bus", SERVO_PROFILE_CPP,
         r"namespace servo \{",
         "namespace servo {\nstatic SMS_STS g_bus;",
         "SMS_STS", runner=run_profile_checks)

    # --- the H0 preflight stays read-only ----------------------------------
    expect_pass("baseline preflight", run_preflight_checks(BASE))

    case("the preflight gains a write primitive", PREFLIGHT_CPP,
         r"  record->expected_bus_id = expected\.bus_id;",
         "  bus_->EnableTorque(expected.bus_id, 1);\n"
         "  record->expected_bus_id = expected.bus_id;",
         "EnableTorque", runner=run_preflight_checks)
    case("the preflight loses its MAINTENANCE gate", ROUTER_CPP,
         r'      Serial\.println\("REASON=NOT_IN_MAINTENANCE_MODE"\);\n'
         r'      Serial\.printf\("MODE=%s\\n", toString\(modules_\.operating_mode->mode\(\)\)\);\n'
         r"      return;\n    \}\n    if \(modules_\.servo_preflight->start\(\)\)",
         "      return;\n    }\n    if (modules_.servo_preflight->start())",
         "MAINTENANCE-gated", runner=run_preflight_checks)
    case("the report claims an observed physical unit", ROUTER_CPP,
         r'"  JOINT expected_physical_unit=%s joint=%s expected_bus_id=%u "',
         '"  JOINT observed_physical_unit=%s joint=%s expected_bus_id=%u "',
         "OBSERVED physical unit", runner=run_preflight_checks)
    case("the report drops the q0 warning", ROUTER_CPP,
         r'  Serial\.println\("  NOTE present_position is a raw liveness tick, NOT q0"\);',
         "",
         "NOT q0", runner=run_preflight_checks)

    # --- the expected physical unit must match the allocation --------------
    expect_pass("baseline population", run_population_checks(BASE))

    case("the expected physical unit drifts from the allocation", POPULATION_H,
         r'\{11, "LF_LOWER", "M33", CurrentConfig::INSTALLED\}',
         '{11, "LF_LOWER", "M99", CurrentConfig::INSTALLED}',
         "physical unit binding", runner=run_population_checks)

    # --- direction is contract data, not a recalibration datum -------------
    expect_pass("baseline direction contract", run_direction_checks(BASE))

    case("direction becomes stored transform evidence again", PROFILE_H,
         r"  uint16_t q0_tick = 0;",
         "  uint16_t q0_tick = 0;\n  int8_t direction = 0;",
         "second source of truth", runner=run_direction_checks)
    case("the direction resolver stops reading the URDF", PROFILE_CPP,
         r"  const int8_t direction = record->urdf_motor_direction;",
         "  const int8_t direction = 1;",
         "urdf_motor_direction", runner=run_direction_checks)
    case("the direction resolver bypasses the bound profile", PROFILE_CPP,
         r"  const GeometryJointRecord\* record = profile\.findJoint\(joint\);",
         "  const GeometryJointRecord* record = &geometry_data_kJoints[0];",
         "geometry provenance tag", runner=run_direction_checks)
    case("provenance demands a measured direction again", PROFILE_CPP,
         r"bool JointTransform::usableProvenance\(\) const \{\n  if \(!present\) return false;",
         "bool JointTransform::usableProvenance() const {\n"
         "  if (direction != 1) return false;\n  if (!present) return false;",
         "invalidates q0 only", runner=run_direction_checks)
    case("the diagnostic budget gates a calibration move", POLICY_CPP,
         r"  if \(parking_planned && !parked_here\) return WriteDecision::REJECT_PARKING_REQUIRED;",
         "  if (bootstrap_.direction_verify_tick_budget <= 0) return "
         "WriteDecision::REJECT_NO_ENVELOPE_BUDGET;\n"
         "  if (parking_planned && !parked_here) return WriteDecision::REJECT_PARKING_REQUIRED;",
         "OPTIONAL diagnostic budget", runner=run_direction_checks)

    # --- the pre-existing torque guard still bites -------------------------
    # "No Torque ON path reachable in the default build" is this phase's claim
    # as much as the previous one's, so it is re-proven here rather than
    # assumed to still hold.
    case("torque-on in the servo transport", "ServoBus.cpp",
         r"st_\.EnableTorque\(static_cast<uint8_t>\(id\), 0\);",
         "st_.EnableTorque(static_cast<uint8_t>(id), 1);",
         "non-zero argument", runner=run_torque_checks)

    if failures:
        print(f"SAFE_ACTUATOR_AUDIT_MUTATION_TESTS = FAIL ({len(failures)})")
        for f in failures:
            print(f"  - {f}")
        return 1
    print("SAFE_ACTUATOR_AUDIT_MUTATION_TESTS = PASS")
    return 0


if __name__ == "__main__":
    sys.exit(main())
