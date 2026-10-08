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

def run_motion_surface_checks(files):
    audit.failures.clear()
    audit.check_servo_motion_write_surface(files)
    return list(audit.failures)


def run_first_motion_command_checks(files):
    audit.failures.clear()
    audit.check_first_motion_command_wiring(files)
    return list(audit.failures)


def run_search_checks(files):
    audit.failures.clear()
    audit.check_calibration_search_boundaries(files)
    return list(audit.failures)


def run_thermal_checks(files):
    audit.failures.clear()
    audit.check_thermal_confirmation(files)
    return list(audit.failures)


def run_full_leg_wiring_checks(files):
    audit.failures.clear()
    audit.check_full_leg_calibration_wiring(files)
    return list(audit.failures)


def run_sequence_checks(files):
    audit.failures.clear()
    audit.check_full_calibration_sequence(files, SKETCH_DIR)
    return list(audit.failures)


def run_servo_write_checks(files):
    audit.failures.clear()
    audit.check_servo_id_write(files)
    return list(audit.failures)


def run_engine_checks(files):
    audit.failures.clear()
    audit.check_calibration_execution_engine_boundaries(files)
    return list(audit.failures)


# The FULL LEG handler's own plan declaration (INITIAL RECOVERY has one too,
# above it): anchored on the FULL LEG-only refusal just before it.
FULL_PLAN = (r'(Serial\.println\("CALIBRATION_FULL_LEG=REFUSED"\);\s*'
             r'Serial\.println\("REASON=PREVIOUS_LEG_RUN_NOT_FINALIZED"\);\s*return;\s*\}\s*)'
             r'calibration::FullLegPlan plan\{\};')
FULL_START = (r'if \(!modules_\.full_leg_calibration->start\(plan\.request, context, millis\(\)\)\) \{'
              r'(\s*Serial\.println\("CALIBRATION_FULL_LEG=REFUSED"\);)')
RECOVERY_START = (r'if \(!\(post_abort \? modules_\.full_leg_calibration->startPostAbortRecovery\(plan\.request, context, millis\(\)\)'
                  r'\s*: modules_\.full_leg_calibration->start\(plan\.request, context, millis\(\)\)\)\) \{'
                  r'(\s*Serial\.println\("CALIBRATION_INITIAL_RECOVERY=REFUSED"\);)')


def run_q0_promotion_checks(files):
    audit.failures.clear()
    audit.check_calibration_q0_promotion_wiring(files, SKETCH_DIR)
    return list(audit.failures)


def run_actuator_infrastructure_checks(files):
    audit.failures.clear()
    audit.check_actuator_infrastructure_wired_fail_closed(files)
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
BUS_H = "ServoBus.h"
SERVO_PROFILE_CPP = "ServoProfile.cpp"
SERVO_PROFILE_DATA_H = "ServoProfileData.h"
PREFLIGHT_CPP = "ServoPreflight.cpp"
POPULATION_H = "ServoPopulation.h"
ROUTER_CPP = "CommandRouter.cpp"
CONTROLLER_CPP = "Controller.cpp"
FINALIZER_H = "FullLegCalibrationFinalizer.h"
FINALIZER_CPP = "FullLegCalibrationFinalizer.cpp"
PLAN_CPP = "FullLegCalibrationPlan.cpp"
Q0_PREP_CPP = "CalibrationQ0EvidencePreparation.cpp"


def main():
    # The unmutated tree must be clean, or every "mutation caught" below would
    # be meaningless.
    expect_pass("baseline", run_boundary_checks(BASE))
    expect_pass("baseline torque/literals", run_torque_checks(BASE))
    expect_pass("baseline CR3 motion surface", run_motion_surface_checks(BASE))
    expect_pass("baseline CR3 first-motion command",
                run_first_motion_command_checks(BASE))
    expect_pass("baseline four-leg Full Calibration wiring",
                run_full_leg_wiring_checks(BASE))
    expect_pass("unmutated tree: calibration search boundaries", run_search_checks(BASE))
    expect_pass("unmutated tree: LF V25 thermal confirmation", run_thermal_checks(BASE))
    expect_pass("baseline CR3 actuator infrastructure fail-closed",
                run_actuator_infrastructure_checks(BASE))
    expect_pass("baseline current-boot q0 promotion", run_q0_promotion_checks(BASE))
    expect_pass("baseline 24-contact sequence", run_sequence_checks(BASE))
    expect_pass("baseline reviewed servo raw write", run_servo_write_checks(BASE))
    expect_pass("baseline execution engine boundary", run_engine_checks(BASE))

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
         r"kActuatorOperationCount = 8;",
         "kActuatorOperationCount = 9;",
         "kActuatorOperationCount")
    case("the operation count drifts downward", POLICY_H,
         r"kActuatorOperationCount = 8;",
         "kActuatorOperationCount = 7;",
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
    case("the direction resolver stops reading the encoder_direction", PROFILE_CPP,
         r"  const int8_t direction = record->encoder_direction;",
         "  const int8_t direction = 1;",
         "jointDirection() must read", runner=run_direction_checks)
    case("the direction resolver falls back to a URDF field", PROFILE_CPP,
         r"  const int8_t direction = record->encoder_direction;",
         "  const int8_t direction = record->encoder_direction ? record->encoder_direction "
         ": urdf_spec_direction(record);",
         "jointDirection() must read", runner=run_direction_checks)
    case("the record carries urdf_motor_direction again", PROFILE_H,
         r"  int8_t encoder_direction;",
         "  int8_t urdf_motor_direction;\n  int8_t encoder_direction;",
         "carries urdf_motor_direction again", runner=run_direction_checks)
    case("the generated profile reverts LF LOWER to the URDF +1", "CalibrationGeometryProfileData.h",
         r'"M33"\}, 11, -1, ',
         '"M33"}, 11, 1, ',
         "are not exactly the 12 records", runner=run_direction_checks)
    case("the generated profile mislabels a direction source", "CalibrationGeometryProfileData.h",
         r'"NEW01"\}, 23, -1, EncoderDirectionSource::HISTORICAL_SLOT_UNCHANGED',
         '"NEW01"}, 23, -1, EncoderDirectionSource::CURRENT_HARDWARE_WITNESS',
         "are not exactly the 12 records", runner=run_direction_checks)
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

    # --- CR3 exact first-motion command surface ----------------------------
    case("first-motion bus drifts", ROUTER_CPP,
         r"constexpr uint8_t kFirstMotionBusId = 12;",
         "constexpr uint8_t kFirstMotionBusId = 13;",
         "kFirstMotionBusId = 12",
         runner=run_first_motion_command_checks)

    case("first-motion delta drifts", ROUTER_CPP,
         r"request\.delta_ticks = 16;",
         "request.delta_ticks = 32;",
         "request.delta_ticks = 16",
         runner=run_first_motion_command_checks)

    case("first-motion command becomes +32", ROUTER_CPP,
         r"@CALIBRATION MOTION DIRECTION_VERIFY LF_UPPER \+16 CONFIRM_FIRST_MOTION",
         "@CALIBRATION MOTION DIRECTION_VERIFY LF_UPPER +32 CONFIRM_FIRST_MOTION",
         "exact command",
         runner=run_first_motion_command_checks)

    case("first-motion branch gains runtime parser", ROUTER_CPP,
         r"calibration::FirstMotionRequest request\{\};",
         'int parsed = 0; sscanf(upper.c_str(), "%d", &parsed);\n'
         '    calibration::FirstMotionRequest request{};',
         "runtime parser",
         runner=run_first_motion_command_checks)

    case("second first-motion start call appears", ROUTER_CPP,
         r"if \(!modules_\.first_motion->start\(request, context, millis\(\)\)\) \{",
         "modules_.first_motion->start(request, context, millis());\n"
         "    if (!modules_.first_motion->start(request, context, millis())) {",
         "exactly one production first_motion->start()",
         runner=run_first_motion_command_checks)

    case("permit bypasses common fact builder", ROUTER_CPP,
         r"calibration::buildCalibrationMotionPermitFacts\(inputs\)",
         "calibration::CalibrationMotionPermitFacts{}",
         "buildCalibrationMotionPermitFacts(inputs)",
         runner=run_first_motion_command_checks)

    case("success no longer requests safe off", "Controller.cpp",
         r"terminal_state\s*==\s*calibration::FirstMotionState::COMPLETE",
         "false",
         "FirstMotionState::COMPLETE",
         runner=run_first_motion_command_checks)

    # --- DIRECTION_VERIFY stays the pinned LF-only diagnostic --------------
    case("direction verify accepts a non-LF session", ROUTER_CPP,
         r"status\(\)\.leg != calibration::Leg::LF",
         "status().leg != calibration::Leg::RF",
         "first-motion branch missing pinned token",
         runner=run_first_motion_command_checks)

    case("session start loses the leg cross-check", ROUTER_CPP,
         r"if \(modules_\.calibration->status\(\)\.leg != command_leg\) \{",
         "if (false) {",
         "status().leg != command_leg",
         runner=run_first_motion_command_checks)

    # --- staged calibration endpoint search (2026-09-29, LF V25 oracle) ------
    case("search guard widened 64 -> 128", "CalibrationTargetResolver.h",
         r"constexpr uint16_t kCalibrationSearchGuardOvershootTicks = 64;",
         "constexpr uint16_t kCalibrationSearchGuardOvershootTicks = 128;",
         "GUARD_OVERSHOOT_TICKS", runner=run_search_checks)

    case("fine step widened 8 -> 16", "ContactProbeEngine.h",
         r"constexpr uint16_t kSearchFineStepTicks = 8;",
         "constexpr uint16_t kSearchFineStepTicks = 16;",
         "kSearchFineStepTicks must be exactly", runner=run_search_checks)

    case("hard-current abort raised 200 -> 400", "ContactProbeEngine.h",
         r"constexpr int32_t kSearchHardCurrentAbortRaw = 200;",
         "constexpr int32_t kSearchHardCurrentAbortRaw = 400;",
         "kSearchHardCurrentAbortRaw must be exactly", runner=run_search_checks)

    case("persistence weakened 3 -> 1 sample", "ContactProbeEngine.h",
         r"constexpr uint8_t kSearchPersistenceSamples = 3;",
         "constexpr uint8_t kSearchPersistenceSamples = 1;",
         "kSearchPersistenceSamples must be exactly", runner=run_search_checks)

    case("policy stops refusing the search flag on non-probe operations", POLICY_CPP,
         r"if \(command\.calibration_search &&\s*command\.operation != "
         r"ActuatorOperation::CALIBRATION_CONTACT_PROBE\)",
         "if (false)",
         "calibration-only gate", runner=run_search_checks)

    case("policy lets POSITION_COMMAND use the V25 speed profile", POLICY_CPP,
         r"command\.operation == ActuatorOperation::CALIBRATION_SEQUENCE_MOVE\)\)\)",
         "command.operation == ActuatorOperation::CALIBRATION_SEQUENCE_MOVE || "
         "command.operation == ActuatorOperation::POSITION_COMMAND)))",
         "calibration-only gate", runner=run_search_checks)

    case("policy no longer bounds a search step by the corridor", POLICY_CPP,
         r"!searchCorridorAdmits\(corridor, command\.target_tick\)",
         "false",
         "search-corridor bound", runner=run_search_checks)

    case("engine stops refusing a search step on other intents", "CalibrationExecutionEngine.cpp",
         r"if \(request\.calibration_search &&\s*operation != "
         r"actuator::ActuatorOperation::CALIBRATION_CONTACT_PROBE\) \{",
         "if (false) {",
         "refuse a search step", runner=run_search_checks)

    case("probe issues the step past the guard", "ContactProbeEngine.cpp",
         r"if \(next_depth > guard_depth\) \{",
         "if (next_depth > guard_depth + 64) {",
         "the step past the guard is never issued", runner=run_search_checks)

    case("probe accepts a stall outside the corridor as contact", "ContactProbeEngine.cpp",
         r"return inside_acceptance \? ContactDetectorState::CONTACT_CONFIRMED\s*"
         r": ContactDetectorState::EARLY_STALL;",
         "return ContactDetectorState::CONTACT_CONFIRMED;",
         "a stall outside the corridor is never contact", runner=run_search_checks)

    case("probe drops the V25 sample cadence", "ContactProbeEngine.cpp",
         r"if \(has_cadence_sample_ && now_ms - last_cadence_ms_ < kSearchSampleIntervalMs\) return;",
         "",
         "V25 sample cadence", runner=run_search_checks)

    # --- the LF V25 coarse contact scout (2026-09-30, PR #35 D6 closed) -------
    case("coarse scout clamped to the corridor entry again", "ContactProbeEngine.cpp",
         r"next_depth = target_depth \+ kSearchCoarseStepTicks;",
         "next_depth = target_depth + kSearchCoarseStepTicks;\n"
         "    const int32_t entry_depth = depth(request_.corridor.entry_tick);\n"
         "    if (next_depth > entry_depth) next_depth = entry_depth;",
         "clamped to the corridor entry", runner=run_search_checks)

    case("fine pass allowed without a scout", "ContactProbeEngine.cpp",
         r"if \(!status_\.scout_valid\) \{\s*failSafeOff\(ContactProbeFailure::SCOUT_MISSING\);",
         "if (false) {\n      failSafeOff(ContactProbeFailure::SCOUT_MISSING);",
         "no fine pass without a scout", runner=run_search_checks)

    case("fine passes judged against fine pass 1 instead of the scout", "ContactProbeEngine.cpp",
         r"if \(!searchFineContactReproducesScout\(request_\.corridor, position, status_\.scout_tick\)\) \{",
         "if (!searchFineContactReproducesScout(request_.corridor, position, status_.pass1_contact_tick)) {",
         "fine_contact_reproduces_coarse_depth", runner=run_search_checks)

    case("adaptive corridor no longer bounded by the static entry", "ContactProbeEngine.cpp",
         r"return adaptive < entry \? adaptive : entry;",
         "return adaptive;",
         "extends HOME-ward only", runner=run_search_checks)

    case("scout-lag tolerance doubled", "ContactProbeEngine.cpp",
         r"<=\s*static_cast<int32_t>\(kSearchFineScoutLagToleranceTicks\);",
         "<= 2 * static_cast<int32_t>(kSearchFineScoutLagToleranceTicks);",
         "one fine step", runner=run_search_checks)

    case("kinematic plateau opened to the coarse scout", "ContactProbeEngine.cpp",
         r"if \(status_\.pass == 0\) \{\s*failSafeOff\(ContactProbeFailure::TRACKING_FAILED\);\s*return;\s*\}",
         "",
         "exists only for passes with a scout", runner=run_search_checks)

    case("backoff always from fine pass 1", "ContactProbeEngine.cpp",
         r"depth\(contact_tick_\) - kSearchBackoffTicks;",
         "depth(status_.pass1_contact_tick) - kSearchBackoffTicks;",
         "V25 backoff", runner=run_search_checks)

    case("the contact search branches on a leg", "ContactProbeEngine.cpp",
         r"void ContactProbeEngine::onCandidate\(uint16_t position\) \{",
         "void ContactProbeEngine::onCandidate(uint16_t position) {\n"
         "  if (request_.endpoint_leg == Leg::LF) return;",
         "names a leg or joint", runner=run_search_checks)

    case("a second contact probe engine in the executor", "FullLegCalibrationExecutor.h",
         r"ContactProbeEngine probe_;",
         "ContactProbeEngine probe_;\n  ContactProbeEngine hip_probe_;",
         "exactly one ContactProbeEngine", runner=run_search_checks)

    case("executor records fine pass 1 as the scout", "FullLegCalibrationExecutor.cpp",
         r"e\.coarse_tick = probe_\.status\(\)\.scout_tick;",
         "e.coarse_tick = probe_.status().pass1_contact_tick;",
         "the scout is recorded as coarse_tick", runner=run_search_checks)

    case("executor accepts a contact without a scout", "FullLegCalibrationExecutor.cpp",
         r" \|\| !probe_\.status\(\)\.scout_valid\) \{",
         ") {",
         "no evidence without a scout", runner=run_search_checks)

    case("diagnostics read the coarse scout", "FullLegCalibrationExecutor.cpp",
         r"midpoint\(min_side\.fine_tick_1, min_side\.fine_tick_2\)",
         "midpoint(min_side.coarse_tick, min_side.fine_tick_1)",
         "the fine passes only", runner=run_search_checks)

    case("envelope bounded by the coarse scout", "OperationalEnvelope.cpp",
         r"minTick\(request\.min_side_evidence\.fine_tick_2,\s*request\.max_side_evidence\.fine_tick_2\)",
         "minTick(request.min_side_evidence.coarse_tick, request.max_side_evidence.coarse_tick)",
         "second fine passes", runner=run_search_checks)

    case("baseline deadline stretched 12 s -> 30 s", "ContactProbeEngine.h",
         r"constexpr uint32_t kSearchBaselineTimeoutMs = 12000;",
         "constexpr uint32_t kSearchBaselineTimeoutMs = 30000;",
         "kSearchBaselineTimeoutMs must be exactly", runner=run_search_checks)

    case("kinematic-plateau span widened 3 -> 8", "ContactProbeEngine.h",
         r"constexpr uint16_t kSearchKinematicPlateauSpanTicks = 3;",
         "constexpr uint16_t kSearchKinematicPlateauSpanTicks = 8;",
         "kSearchKinematicPlateauSpanTicks must be exactly", runner=run_search_checks)

    # --- V25 backoff StableTargetGate, TELEMETRY_TIMEOUT, recentred q0 ---------
    case("backoff settle window 400 -> 0 ms", "ContactProbeEngine.h",
         r"constexpr uint32_t kSearchBackoffSettleWindowMs = 400;",
         "constexpr uint32_t kSearchBackoffSettleWindowMs = 0;",
         "kSearchBackoffSettleWindowMs must be exactly", runner=run_search_checks)

    case("backoff settle speed limit 4 -> 40 raw", "ContactProbeEngine.h",
         r"constexpr int32_t kSearchBackoffSettleMaxSpeedRaw = 4;",
         "constexpr int32_t kSearchBackoffSettleMaxSpeedRaw = 40;",
         "kSearchBackoffSettleMaxSpeedRaw must be exactly", runner=run_search_checks)

    case("search telemetry timeout back to 3 s", "ContactProbeEngine.h",
         r"constexpr uint32_t kSearchTelemetryTimeoutMs = 2000;",
         "constexpr uint32_t kSearchTelemetryTimeoutMs = 3000;",
         "kSearchTelemetryTimeoutMs must be exactly", runner=run_search_checks)

    case("backoff arrived on the first in-band sample (no settle gate)", "ContactProbeEngine.cpp",
         r"if \(!settle_\.observe\(static_cast<uint16_t>\(sample\.present_position\),\s*"
         r"magnitude\(sample\.present_speed\), status_\.target_tick, now_ms\)\) \{\s*return;\s*\}",
         "(void)settle_;",
         "through the V25 StableTargetGate", runner=run_search_checks)

    case("settle gate keeps counting across a bad sample", "ContactProbeEngine.cpp",
         r"if \(!qualifies\) \{\s*reset\(\);\s*return false;\s*\}",
         "if (!qualifies) {\n    return false;\n  }",
         "band, speed, reset", runner=run_search_checks)

    case("backoff deadman telemetry age back to 3 s", "Controller.cpp",
         r"full_leg_backoff\.max_telemetry_age_ms = calibration::kSearchTelemetryTimeoutMs;",
         "full_leg_backoff.max_telemetry_age_ms = 3000;",
         "V25 figures", runner=run_search_checks)

    case("corridor home taken from the raw servo centre", "CalibrationTargetResolver.cpp",
         r"c\.home_tick = transform\.q0_tick;",
         "c.home_tick = 2048;",
         "raw servo centre", runner=run_search_checks)

    # --- the final bounded partial coarse-scout step (2026-09-30) -------------
    case("partial scout step beyond the guard", "ContactProbeEngine.cpp",
         r"next_depth = guard_depth;",
         "next_depth = guard_depth + 8;",
         "beyond the guard", runner=run_search_checks)

    case("partial scout step repeated at the guard", "ContactProbeEngine.cpp",
         r"if \(next_depth > guard_depth && target_depth < guard_depth\) \{",
         "if (next_depth > guard_depth) {",
         "never repeated", runner=run_search_checks)

    # --- LF V25 runtime PresentTemperature confirmation (2026-09-30) ---------
    case("thermal majority weakened to 1 of 3", "ThermalConfirmation.h",
         r"constexpr uint8_t kThermalConfirmedOverLimit = 3;",
         "constexpr uint8_t kThermalConfirmedOverLimit = 1;",
         "thermal/UART safety invariant missing", runner=run_thermal_checks)

    case("thermal confirmation wait removed", "ThermalConfirmation.cpp",
         r"now_ms - last_read_ms_ < kThermalConfirmationDelayMs",
         "",
         "thermal/UART safety invariant missing", runner=run_thermal_checks)

    case("thermal confirmation reads another servo", "ThermalConfirmation.cpp",
         r"port->readPresentTemperatureDirect\(bus, &value\)",
         "port->readPresentTemperatureDirect(static_cast<uint8_t>(bus + 1), &value)",
         "thermal/UART safety invariant missing", runner=run_thermal_checks)

    # --- dev.3: block-read temperature is diagnostic, direct reads decide ----
    case("block-read sample counted as direct thermal evidence", "ThermalConfirmation.cpp",
         r"for \(uint8_t i = 1; i < result_\.sample_count; \+\+i\) \{",
         "for (uint8_t i = 0; i < result_.sample_count; ++i) {",
         "thermal/UART safety invariant missing", runner=run_thermal_checks)

    case("thermal telemetry fault published at the limit", "ThermalConfirmation.cpp",
         r"result_\.published_c = kThermalLimitC \+ 1;",
         "result_.published_c = kThermalLimitC;",
         "thermal/UART safety invariant missing", runner=run_thermal_checks)

    case("incoherent direct samples no longer fail closed", "ThermalConfirmation.cpp",
         r"else if \(result_\.sample_count == kThermalConfirmationReads\) \{\s*return telemetryFault\(\);",
         "else if (result_.sample_count == kThermalConfirmationReads) { return result_;",
         "thermal/UART safety invariant missing", runner=run_thermal_checks)

    case("refuted block-read values latch a thermal fault again", "ThermalConfirmation.cpp",
         r"if \(bulk_artifacts_ < 0xFFFF\) \+\+bulk_artifacts_;",
         "if (bulk_artifacts_ < 0xFFFF) ++bulk_artifacts_; if (bulk_artifacts_ >= 3) return telemetryFault();",
         "may not latch a thermal verdict", runner=run_thermal_checks)

    case("over-temperature confirmed from the block read alone", "ThermalConfirmation.cpp",
         r"if \(observed <= kThermalLimitC\) return result_;",
         "if (observed <= kThermalLimitC) return result_; result_.decision = ThermalDecision::CONFIRMED; return result_;",
         "only be confirmed by the direct-sample majority", runner=run_thermal_checks)

    case("direct clearing majority weakened to 2", "ThermalConfirmation.h",
         r"constexpr uint8_t kThermalDirectNormalToClear = 3;",
         "constexpr uint8_t kThermalDirectNormalToClear = 2;",
         "thermal/UART safety invariant missing", runner=run_thermal_checks)

    case("Controller skips the thermal confirmation", "Controller.cpp",
         r"state\.update\(&thermal_read_port_, buses\[i\], sample\.present_temperature, millis\(\)\)",
         "state.result()",
         "thermal/UART safety invariant missing", runner=run_thermal_checks)

    case("CommandRouter derives its own search corridor", ROUTER_CPP,
         r"void CommandRouter::printServoRead\(int id\) \{",
         "void CommandRouter::printServoRead(int id) {\n  actuator::CalibrationSearchCorridor c{};\n"
         "  (void)actuator::resolveCalibrationSearchCorridor(*modules_.geometry_profile, "
         "actuator::geometry_data::kProvenance, actuator::JointTransform{}, calibration::Leg::LF, "
         "calibration::JointKind::UPPER, calibration::ContactSide::MIN_SIDE, &c);",
         "only the reviewed", runner=run_search_checks)

    case("FirstMotionExecutor requests the V25 speed profile", "FirstMotionExecutor.cpp",
         r"cmd\.operation = actuator::ActuatorOperation::TORQUE_ENABLE;",
         "cmd.operation = actuator::ActuatorOperation::TORQUE_ENABLE;\n"
         "  cmd.motion_profile = actuator::MotionProfile::CALIBRATION_SEARCH;",
         "only the calibration", runner=run_search_checks)

    case("ServoBus search envelope raised 160 -> 400", BUS_H,
         r"static constexpr uint16_t kSearchEnvelopeSpeed = 160;",
         "static constexpr uint16_t kSearchEnvelopeSpeed = 400;",
         "GoalPosition speed envelope", runner=run_motion_surface_checks)

    case("ServoBus bounded speed raised 40 -> 80", BUS_H,
         r"static constexpr uint16_t kBoundedWriteSpeed = 40;",
         "static constexpr uint16_t kBoundedWriteSpeed = 80;",
         "GoalPosition speed envelope", runner=run_motion_surface_checks)

    case("Full-Leg backoff deadman falls back to the DIRECTION_VERIFY budget", CONTROLLER_CPP,
         r"full_leg_config\.probe_backoff_deadman = full_leg_backoff;",
         "full_leg_config.probe_backoff_deadman = deadman;",
         "long-move deadman", runner=run_full_leg_wiring_checks)

    case("Full-Leg long moves lose the travel-aware floor", CONTROLLER_CPP,
         r"full_leg_backoff\.nominal_travel_ticks_per_s = calibration::kSearchMinExpectedTicksPerSecond;",
         "",
         "long-move deadman", runner=run_full_leg_wiring_checks)

    # --- four-leg Full Calibration: command surface ------------------------
    case("leg matcher gains a fifth token", ROUTER_CPP,
         r'\{"LH", calibration::Leg::LH\},',
         '{"LH", calibration::Leg::LH},\n      {"XX", calibration::Leg::LH},',
         "table must be exactly",
         runner=run_full_leg_wiring_checks)

    case("leg matcher maps a name to another leg", ROUTER_CPP,
         r'\{"RH", calibration::Leg::RH\}',
         '{"RH", calibration::Leg::LH}',
         "table must be exactly",
         runner=run_full_leg_wiring_checks)

    case("leg matcher becomes a prefix match", ROUTER_CPP,
         r"if \(line == candidate\) \{",
         "if (line.startsWith(candidate)) {",
         "whole line for equality",
         runner=run_full_leg_wiring_checks)

    case("full leg is matched with startsWith", ROUTER_CPP,
         r'matchLegCommand\(upper, "@CALIBRATION FULL LEG ", " CONFIRM_FULL_CALIBRATION",\s*&command_leg\)',
         'upper.startsWith("@CALIBRATION FULL LEG ")',
         "may not use startsWith",
         runner=run_full_leg_wiring_checks)

    case("session start drops the permit-released gate", ROUTER_CPP,
         r'if \(modules_\.motion_permit->active\(\)\) \{\s*Serial\.println\("CALIBRATION_SESSION=REFUSED"\);',
         'if (false) {\n      Serial.println("CALIBRATION_SESSION=REFUSED");',
         "between-leg gate",
         runner=run_full_leg_wiring_checks)

    case("session start drops the finalized-run gate", ROUTER_CPP,
         r'if \(modules_\.full_leg_run->armed\) \{\s*Serial\.println\("CALIBRATION_SESSION=REFUSED"\);',
         'if (false) {\n      Serial.println("CALIBRATION_SESSION=REFUSED");',
         "between-leg gate",
         runner=run_full_leg_wiring_checks)

    case("session start drops the authority-NONE gate", ROUTER_CPP,
         r"modules_\.authority->current\(\) != ActuatorAuthority::NONE\) \{",
         "false) {",
         "between-leg gate",
         runner=run_full_leg_wiring_checks)

    case("session start hard-codes LF", ROUTER_CPP,
         r"command_leg,\s*modules_\.operating_mode->mode\(\)\);",
         "calibration::Leg::LF, modules_.operating_mode->mode());",
         "names a leg literal",
         runner=run_full_leg_wiring_checks)

    case("full leg hard-codes a bus id", ROUTER_CPP,
         FULL_PLAN,
         r"\1calibration::FullLegPlan plan{};\n    plan.request.probe_bus_id = 12;",
         "assigns into the plan/request",
         runner=run_full_leg_wiring_checks)

    case("full leg restores the LF backoff residue", ROUTER_CPP,
         FULL_PLAN,
         r"\1calibration::FullLegPlan plan{};\n    const int32_t kBackoff = -700000;",
         "hard-codes a per-leg bus/backoff",
         runner=run_full_leg_wiring_checks)

    case("full leg names a leg literal", ROUTER_CPP,
         FULL_PLAN,
         r"\1calibration::FullLegPlan plan{};\n    const calibration::Leg fixed = calibration::Leg::LF;",
         "names a leg literal",
         runner=run_full_leg_wiring_checks)

    case("full leg gains a runtime parser", ROUTER_CPP,
         FULL_PLAN,
         r'\1calibration::FullLegPlan plan{};\n    int parsed = 0; sscanf(upper.c_str(), "%d", &parsed);',
         "runtime parser",
         runner=run_full_leg_wiring_checks)

    case("full leg writes to the bus from the handler", ROUTER_CPP,
         FULL_PLAN,
         r"\1calibration::FullLegPlan plan{};\n    modules_.servo_bus->safeOff(12);",
         "write/transaction primitive",
         runner=run_full_leg_wiring_checks)

    case("full leg arms the run before the executor accepts", ROUTER_CPP,
         FULL_START,
         "modules_.full_leg_run->arm(plan, 0, 0);\n"
         r"    if (!modules_.full_leg_calibration->start(plan.request, context, millis())) {\1",
         "arm the run record only AFTER",
         runner=run_full_leg_wiring_checks)

    case("second full leg start call appears", ROUTER_CPP,
         FULL_START,
         "modules_.full_leg_calibration->start(plan.request, context, millis());\n"
         r"    if (!modules_.full_leg_calibration->start(plan.request, context, millis())) {\1",
         "exactly two production full_leg_calibration->start()",
         runner=run_full_leg_wiring_checks)

    case("full leg stops resolving the plan", ROUTER_CPP,
         r"calibration::resolveFullLegPlan\(",
         "calibration::resolveFullLegPlanUnchecked(",
         "resolveFullLegPlan(",
         runner=run_full_leg_wiring_checks)

    case("plan resolver special-cases a leg", PLAN_CPP,
         r"const actuator::GeometryJointRecord\* record = profile\.findJoint\(found\.identity\);",
         "const bool lf_only = (leg == Leg::LF);\n"
         "  const actuator::GeometryJointRecord* record = profile.findJoint(found.identity);",
         "nothing in the plan may be special-cased",
         runner=run_full_leg_wiring_checks)

    # --- four-leg Full Calibration: Controller -----------------------------
    case("the V5 auxiliary window is opened in production", CONTROLLER_CPP,
         r"ctx\.auxiliary_parked = false;",
         "ctx.auxiliary_parked = true;",
         "sequence bootstrap context lost",
         runner=run_full_leg_wiring_checks)

    case("the sequence phase is not the executor's", CONTROLLER_CPP,
         r"ctx\.sequence_phase = full_leg_calibration_\.sequencePhase\(\);",
         "ctx.sequence_phase = calibration::CalibrationPhase::HIP_MAX;",
         "sequence bootstrap context lost",
         runner=run_full_leg_wiring_checks)

    case("held prerequisites always reported verified", CONTROLLER_CPP,
         r"ctx\.sequence_prerequisites_verified = full_leg_calibration_\.prerequisitesVerified\(\);",
         "ctx.sequence_prerequisites_verified = true;",
         "sequence bootstrap context lost",
         runner=run_full_leg_wiring_checks)

    case("a parked leg is hard-coded again", CONTROLLER_CPP,
         r"ctx\.auxiliary_parked = false;",
         "ctx.auxiliary_parked = false;\n  ctx.parked_leg = calibration::Leg::LF;",
         "no production path may open the V5 auxiliary window",
         runner=run_full_leg_wiring_checks)

    case("finalization is never called", CONTROLLER_CPP,
         r"\n  updateFullLegFinalization\(\);",
         "",
         "must call updateFullLegFinalization()",
         runner=run_full_leg_wiring_checks)

    case("finalization uses ad-hoc envelope parameters", CONTROLLER_CPP,
         r"context\.parameters = calibration::productionEnvelopeParameters\(\);",
         "context.parameters = calibration::FullLegEnvelopeParameters{};",
         "productionEnvelopeParameters()",
         runner=run_full_leg_wiring_checks)

    case("finalization result is not stored", CONTROLLER_CPP,
         r"full_leg_evidence_\.put\(record\);",
         "(void)record;",
         "full_leg_evidence_.put(record)",
         runner=run_full_leg_wiring_checks)

    case("finalization writes to the bus", CONTROLLER_CPP,
         r"full_leg_run_\.clear\(\);",
         "full_leg_run_.clear();\n  servo_bus_.safeOff(12);",
         "updateFullLegFinalization() contains",
         runner=run_full_leg_wiring_checks)

    case("finalization admits a limit itself", CONTROLLER_CPP,
         r"void Controller::printBootBanner\(\) \{",
         "void Controller::printBootBanner() {\n"
         "  actuator_policy_.admitOperationalLimit(actuator::JointLimit{});",
         "admitOperationalLimit() may only be called",
         runner=run_full_leg_wiring_checks)

    case("final SAFE_OFF targets a fixed bus", CONTROLLER_CPP,
         r"servo_bus_\.safeOff\(off\[i\]\)",
         "servo_bus_.safeOff(12)",
         "must call safeOff() only for",
         runner=run_full_leg_wiring_checks)

    case("SAFE_OFF ignores the executor's request", CONTROLLER_CPP,
         r"const uint8_t m = full_leg_calibration_\.safeOffRequest\(off, calibration::kFullLegPopulation\);",
         "const uint8_t m = 0;",
         "safeOffRequest",
         runner=run_full_leg_wiring_checks)

    case("a Full Leg run skips its phase report", CONTROLLER_CPP,
         r"if \(!calibration_\.noteExecutionPhase\(full_leg_calibration_\.status\(\)\.phase\)\) \{",
         "if (false) {",
         "noteExecutionPhase",
         runner=run_full_leg_wiring_checks)

    # --- four-leg Full Calibration: finalizer ------------------------------
    case("production envelope parameters become approved", FINALIZER_H,
         r"constexpr bool kFullLegOperationalParametersApproved = false;",
         "constexpr bool kFullLegOperationalParametersApproved = true;",
         "kFullLegOperationalParametersApproved must be declared",
         runner=run_full_leg_wiring_checks)

    case("finalizer no longer completes the session", FINALIZER_CPP,
         r"manager\.completeSession\(\)",
         "manager.abortSession()",
         "lost 'manager.completeSession()'",
         runner=run_full_leg_wiring_checks)

    case("finalizer stops revoking the permit", FINALIZER_CPP,
         r"if \(context\.permit != nullptr\) context\.permit->revoke\(CalibrationPermitRevokeReason::EXPLICIT\);",
         "",
         "lost 'context.permit->revoke('",
         runner=run_full_leg_wiring_checks)

    case("finalizer skips the contact recording", FINALIZER_CPP,
         r"manager\.recordContact\(joint\.contact\[s\]\)",
         "manager.recordContactSkipped(joint.contact[s])",
         "manager.recordContact(joint.contact[s])",
         runner=run_full_leg_wiring_checks)

    # --- TRUE FULL CALIBRATION = 24 CONTACTS: the definition -----------------
    case("a leg accepted with 5/6 contacts", FINALIZER_CPP,
         r"if \(measured != kFullLegContactsExpected \|\|",
         "if (measured < kFullLegContactsExpected - 1 ||",
         "24-contact definition", runner=run_full_leg_wiring_checks)
    case("a failed leg keeps its partial count", FINALIZER_CPP,
         r"record->verdict = FullLegVerdict::FAILED;\s*record->contacts_accepted = 0;",
         "record->verdict = FullLegVerdict::FAILED;",
         "24-contact definition", runner=run_full_leg_wiring_checks)
    case("the four-leg verdict ignores the contact total", FINALIZER_H,
         r"totalContactsAccepted\(\) == kFullCalibrationContactsExpected;",
         "totalContactsAccepted() >= 8;",
         "24-contact definition", runner=run_full_leg_wiring_checks)
    case("the UPPER-only definition returns", FINALIZER_H,
         r"constexpr uint8_t kFullLegContactsExpected = kFullLegContactCount;",
         "constexpr uint8_t kFullLegContactsExpected = 2;",
         "24-contact definition", runner=run_full_leg_wiring_checks)

    # --- INITIAL RECOVERY: its own command, never a leg run ------------------
    case("recovery command arms a leg run", ROUTER_CPP,
         RECOVERY_START,
         "modules_.full_leg_run->arm(plan, 0, 0);\n"
         r"    if (!modules_.full_leg_calibration->start(plan.request, context, millis())) {\1",
         "may never arm or record a leg run", runner=run_full_leg_wiring_checks)
    case("recovery command forgets recovery_only", ROUTER_CPP,
         r"plan\.request\.recovery_only = true;",
         "",
         "recovery_only = true before start()", runner=run_full_leg_wiring_checks)
    case("FULL LEG runs recovery only", ROUTER_CPP,
         FULL_START,
         "plan.request.recovery_only = true;\n"
         r"    if (!modules_.full_leg_calibration->start(plan.request, context, millis())) {\1",
         "FULL LEG may not touch recovery_only", runner=run_full_leg_wiring_checks)
    case("recovery command drops the permit gate", ROUTER_CPP,
         r"if \(!modules_\.motion_permit->active\(\) \|\|(\s*!modules_\.motion_authorization->operator_authorized \|\|\s*"
         r"!modules_\.motion_authorization->token\.valid\(\)\) \{\s*"
         r'Serial\.println\("CALIBRATION_INITIAL_RECOVERY=REFUSED"\);)',
         r"if (false ||\1",
         "INITIAL RECOVERY branch missing pinned token", runner=run_full_leg_wiring_checks)
    case("a recovery-only run is phase-reported", CONTROLLER_CPP,
         r"if \(full_leg_calibration_\.request\(\)\.recovery_only\) \{",
         "if (false) {",
         "recovery-only run", runner=run_full_leg_wiring_checks)

    # --- the V25 sequence itself ----------------------------------------------
    case("a V25 held-drift tolerance widens", "FullLegCalibrationExecutor.h",
         r"constexpr uint16_t kSequenceStaticToleranceTicks = 10;",
         "constexpr uint16_t kSequenceStaticToleranceTicks = 20;",
         "kSequenceStaticToleranceTicks must be exactly 10", runner=run_sequence_checks)
    case("a leg becomes 5 contacts", "FullLegCalibrationExecutor.h",
         r"constexpr uint8_t kFullLegContactCount = 6;",
         "constexpr uint8_t kFullLegContactCount = 5;",
         "kFullLegContactCount must be exactly 6", runner=run_sequence_checks)
    case("the sequence skips LOWER_MAX", "FullLegCalibrationExecutor.cpp",
         r"case CalibrationPhase::LOWER_MIN:\s*enterPhase\(CalibrationPhase::LOWER_MAX, now_ms\); return;",
         "case CalibrationPhase::LOWER_MIN:         enterPhase(CalibrationPhase::LOWER_FOLDED, now_ms); return;",
         "one V25 phase at a time", runner=run_sequence_checks)
    case("the sequence ends after the UPPER", "FullLegCalibrationExecutor.cpp",
         r"case CalibrationPhase::UPPER_HORIZONTAL:\s*enterPhase\(CalibrationPhase::LOWER_MIN, now_ms\); return;",
         "case CalibrationPhase::UPPER_HORIZONTAL:  enterPhase(CalibrationPhase::CLEANUP, now_ms); return;",
         "one V25 phase at a time", runner=run_sequence_checks)
    case("COMPLETE without diagnostics", "FullLegCalibrationExecutor.cpp",
         r"status_\.contacts_accepted == kFullLegContactCount && diagnostics_accepted_",
         "status_.contacts_accepted == kFullLegContactCount",
         "COMPLETE rule", runner=run_sequence_checks)
    case("INITIAL_RECOVERY skips a joint already near q0", "FullLegCalibrationExecutor.cpp",
         r"if \(distance > static_cast<int32_t>\(actuator::kSequencePrimeMaxDistanceTicks\)\) \{",
         "if (distance <= 10) { ++recover_index_; return; }\n"
         "      if (distance > static_cast<int32_t>(actuator::kSequencePrimeMaxDistanceTicks)) {",
         "INITIAL_RECOVERY may not skip a joint", runner=run_sequence_checks)
    case("UPPER probe drops the held LOWER", "FullLegCalibrationExecutor.cpp",
         r"want\[kSlotLower\] = true; want_tick\[kSlotLower\] = request_\.joint\[kSlotLower\]\.q0_tick;",
         "want_tick[kSlotLower] = request_.joint[kSlotLower].q0_tick;",
         "held-set rule", runner=run_sequence_checks)
    case("held joints stop checking the TorqueLimit", "FullLegCalibrationExecutor.cpp",
         r"sample->torque_limit != static_cast<int32_t>\(request_\.torque_limit\) \|\|\s*"
         r"sample->goal_position != static_cast<int32_t>\(st\.target_tick\)",
         "sample->goal_position != static_cast<int32_t>(st.target_tick)",
         "monitorHeld() lost", runner=run_sequence_checks)
    # V25 ActivelyHeld supervision: no speed abort on an already-held joint;
    # speed stays the settling gate (StableTargetGate / INITIAL_RECOVERY).
    case("a held joint aborts on its speed again", "FullLegCalibrationExecutor.cpp",
         r"    st\.speed_transient = fast;",
         "    if (fast) return false;\n    st.speed_transient = fast;",
         "may read a held joint's speed only", runner=run_sequence_checks)
    case("a held joint fails on speed before the V25 checks", "FullLegCalibrationExecutor.cpp",
         r"    st\.last_good_ms = now_ms;\n    st\.has_last_sample = true;",
         "    st.last_good_ms = now_ms;\n    if (magnitude(sample->present_speed) > 40) "
         "{ failHeldRole(observeHeld(s, sample, now_ms, t), FullLegFailure::HELD_JOINT_DRIFT); "
         "return false; }\n    st.has_last_sample = true;",
         "may read a held joint's speed only", runner=run_sequence_checks)
    case("the retired HELD_JOINT_SPEED failure reappears", "FullLegCalibrationExecutor.h",
         r"  HELD_JOINT_DRIFT,\n",
         "  HELD_JOINT_DRIFT,\n  HELD_JOINT_SPEED,\n",
         "retired post-V25 HELD_JOINT_SPEED", runner=run_sequence_checks)
    case("held joints stop checking drift", "FullLegCalibrationExecutor.cpp",
         r"if \(absDiff\(sample->present_position, st\.target_tick\) >",
         "if (false && absDiff(sample->present_position, st.target_tick) >",
         "monitorHeld() lost", runner=run_sequence_checks)
    case("held joints skip status / current / temperature", "FullLegCalibrationExecutor.cpp",
         r"    if \(safety != FullLegFailure::NONE\) \{",
         "    if (false) {",
         "monitorHeld() lost", runner=run_sequence_checks)
    case("the StableTargetGate loses its speed criterion", "FullLegCalibrationExecutor.cpp",
         r"absDiff\(s->present_position, p\.target_tick\) <= static_cast<int32_t>\(kSequenceStaticToleranceTicks\) &&"
         r"\s*magnitude\(s->present_speed\) <= static_cast<int32_t>\(kSequenceSettleMaxSpeedRaw\);",
         "absDiff(s->present_position, p.target_tick) <= static_cast<int32_t>(kSequenceStaticToleranceTicks);",
         "StableTargetGate speed criterion", runner=run_sequence_checks)
    case("the INITIAL_RECOVERY settle loses its speed criterion", "FullLegCalibrationExecutor.cpp",
         r"absDiff\(s->present_position, j\.q0_tick\) <= static_cast<int32_t>\(kSequenceStaticToleranceTicks\) &&"
         r"\s*magnitude\(s->present_speed\) <= static_cast<int32_t>\(kSequenceSettleMaxSpeedRaw\);",
         "absDiff(s->present_position, j.q0_tick) <= static_cast<int32_t>(kSequenceStaticToleranceTicks);",
         "StableTargetGate speed criterion", runner=run_sequence_checks)
    case("the held speed-transient diagnostic becomes unbounded", "FullLegCalibrationExecutor.h",
         r"constexpr uint8_t kHeldSpeedTransientEventCap = 32;",
         "constexpr uint8_t kHeldSpeedTransientEventCap = 255;",
         "must stay bounded", runner=run_sequence_checks)
    case("the validated plan data is hand-edited", "CalibrationSequencePlanData.h",
         r"-697961\}",
         "-1518641}",
         "exporter's output", runner=run_sequence_checks)
    case("a leg's geometry validation is flipped", "CalibrationSequencePlanData.h",
         r"\{calibration::Leg::RH, true,",
         "{calibration::Leg::RH, false,",
         "exporter's output", runner=run_sequence_checks)

    # --- the one reviewed raw register write -----------------------------------
    case("the RAM TorqueLimit rises to 1000", BUS_H,
         r"static constexpr uint16_t kReviewedRamTorqueLimit = 500;",
         "static constexpr uint16_t kReviewedRamTorqueLimit = 1000;",
         "kReviewedRamTorqueLimit must be exactly 500", runner=run_servo_write_checks)
    case("the TorqueLimit write targets another register", BUS_CPP,
         r"st_\.writeWord\(static_cast<uint8_t>\(id\), SMS_STS_TORQUE_LIMIT_L, kReviewedRamTorqueLimit\);",
         "st_.writeWord(static_cast<uint8_t>(id), SMS_STS_MAX_TORQUE_LIMIT_L, kReviewedRamTorqueLimit);",
         "writeReviewedRamTorqueLimit() lost", runner=run_servo_write_checks)
    case("a second raw register write appears", BUS_CPP,
         r"bool ServoBus::readRuntimeState\(int id, RuntimeState\* out\) \{",
         "void ServoBus::rogue(int id) { st_.writeWord((uint8_t)id, 31, 0); }\n"
         "bool ServoBus::readRuntimeState(int id, RuntimeState* out) {",
         "exactly one raw writeWord()", runner=run_servo_write_checks)
    case("a raw byte write appears", BUS_CPP,
         r"bool ServoBus::readRuntimeState\(int id, RuntimeState\* out\) \{",
         "void ServoBus::rogue(int id) { st_.writeByte((uint8_t)id, 5, 1); }\n"
         "bool ServoBus::readRuntimeState(int id, RuntimeState* out) {",
         "writeByte", runner=run_servo_write_checks)
    case("the TorqueLimit verdict stops reading back", BUS_CPP,
         r"st_\.readWord\(static_cast<uint8_t>\(id\), SMS_STS_TORQUE_LIMIT_L\);",
         "0;",
         "writeReviewedRamTorqueLimit() lost", runner=run_servo_write_checks)

    # --- the engine carries the phase label, never branches on it ----------------
    case("the execution engine branches on a V25 phase", "CalibrationExecutionEngine.cpp",
         r"command\.sequence_phase = request\.sequence_phase;",
         "command.sequence_phase = request.sequence_phase;\n"
         "  if (request.sequence_phase == CalibrationPhase::HIP_MAX) return result;",
         "CalibrationPhase", runner=run_engine_checks)


    case("finalizer admits limits before the envelopes", FINALIZER_CPP,
         r"const actuator::GeometryProvenanceTag current = policy\.currentGeometryTag\(\);",
         "policy.admitOperationalLimit(actuator::JointLimit{});\n"
         "  const actuator::GeometryProvenanceTag current = policy.currentGeometryTag();",
         "exactly one admitOperationalLimit() call site",
         runner=run_full_leg_wiring_checks)

    case("finalizer touches the servo bus", FINALIZER_CPP,
         r"const FullLegJointRef& planJoint",
         "void bad() { servo::ServoBus* bus = nullptr; (void)bus; }\n"
         "const FullLegJointRef& planJoint",
         "pure decision unit",
         runner=run_full_leg_wiring_checks)

    # --- current-boot q0 promotion ----------------------------------------
    # PROMOTE must consume the capture of THIS boot, in RAM, through the one
    # reviewed pipeline, and refuse while anything could be using the old q0.
    case("PROMOTE goes back to the frozen CR2-C package", ROUTER_CPP,
         r"actuator::prepareFreshQ0Evidence\(",
         "actuator::prepareCurrentQ0Evidence(",
         "lost required gate 'prepareFreshQ0Evidence('",
         runner=run_q0_promotion_checks)

    case("PROMOTE reads the frozen data directly", ROUTER_CPP,
         r"const actuator::FreshQ0Capture fresh_capture = modules_\.q0_capture->freshCapture\(\);",
         "const unsigned frozen = actuator::q0_evidence_data::kSampleCount;\n"
         "    const actuator::FreshQ0Capture fresh_capture = modules_.q0_capture->freshCapture();",
         "q0_evidence_data",
         runner=run_q0_promotion_checks)

    case("PROMOTE stops using the capture view", ROUTER_CPP,
         r"const actuator::FreshQ0Capture fresh_capture = modules_\.q0_capture->freshCapture\(\);",
         "const actuator::FreshQ0Capture fresh_capture = actuator::FreshQ0Capture{};",
         "modules_.q0_capture->freshCapture()",
         runner=run_q0_promotion_checks)

    case("PROMOTE no longer refuses during a live capture", ROUTER_CPP,
         r'if \(modules_\.q0_capture->active\(\)\) \{(\s*)Serial\.println\("CALIBRATION_Q0_PROMOTE=BUSY"\);',
         'if (false) {\\1Serial.println("CALIBRATION_Q0_PROMOTE=BUSY");',
         "modules_.q0_capture->active()",
         runner=run_q0_promotion_checks)

    case("PROMOTE no longer refuses under a live session or armed run", ROUTER_CPP,
         r"modules_\.full_leg_run->armed \|\|(\s*)modules_\.calibration->sessionLive\(\)\) \{(\s*)"
         r'Serial\.println\("CALIBRATION_Q0_PROMOTE=BUSY"\);',
         'modules_.motion_permit->active()) {\\2Serial.println("CALIBRATION_Q0_PROMOTE=BUSY");',
         "modules_.full_leg_run->armed",
         runner=run_q0_promotion_checks)

    case("PROMOTE touches the servo EEPROM", ROUTER_CPP,
         r"if \(modules_\.actuator_policy->transforms\(\)\.admit\(prepared\.transforms\[i\]\)\) \+\+admitted;",
         "{ CalibrationOfs(1, 0); modules_.actuator_policy->transforms().admit(prepared.transforms[i]); ++admitted; }",
         "CalibrationOfs",
         runner=run_q0_promotion_checks)

    case("a second production transform admission site appears", FINALIZER_CPP,
         r"const FullLegJointRef& planJoint",
         "void bad(actuator::SafeActuatorPolicy& p) "
         "{ p.transforms().admit(actuator::JointTransform{}); }\n"
         "const FullLegJointRef& planJoint",
         "only permitted inside",
         runner=run_q0_promotion_checks)

    case("production code consumes the frozen CR2-C package", PLAN_CPP,
         r"FullLegPlanStatus resolveFullLegPlan\(",
         "void bad() { actuator::prepareCurrentQ0Evidence(profile, prov, true); }\n"
         "FullLegPlanStatus resolveFullLegPlan(",
         "historical regression oracle",
         runner=run_q0_promotion_checks)

    case("SESSION START stops requiring the promoted current capture", ROUTER_CPP,
         r"!actuator::freshQ0CaptureIsPromoted\(\s*modules_\.q0_capture->freshCapture\(\),\s*"
         r"modules_\.actuator_policy->transforms\(\),\s*"
         r"modules_\.actuator_policy->currentGeometryTag\(\)\)",
         "false",
         "SESSION START lost the fresh-q0 promotion gate",
         runner=run_q0_promotion_checks)

    case("q0 preparation gains Serial", Q0_PREP_CPP,
         r"Q0EvidencePreparation prepareFreshQ0Evidence\(",
         'void bad() { Serial.println("x"); }\n'
         "Q0EvidencePreparation prepareFreshQ0Evidence(",
         "q0 preparation contains 'Serial.'",
         runner=run_q0_promotion_checks)

    case("q0 preparation mutates the transform table itself", Q0_PREP_CPP,
         r"Q0EvidencePreparation prepareFreshQ0Evidence\(",
         "void bad(JointTransformTable& t) { t.admit(JointTransform{}); }\n"
         "Q0EvidencePreparation prepareFreshQ0Evidence(",
         "q0 preparation contains '.admit('",
         runner=run_q0_promotion_checks)

    case("q0 preparation depends on the capture session", Q0_PREP_CPP,
         r'#include "CalibrationQ0EvidencePreparation\.h"',
         '#include "CalibrationQ0EvidencePreparation.h"\n'
         '#include "../calibration/CalibrationQ0CaptureSession.h"',
         "q0 preparation contains 'CalibrationQ0CaptureSession'",
         runner=run_q0_promotion_checks)

    # --- the low-level transaction door stays shut, even next to the two ---
    # --- exempt executor-level abort() calls this same file now contains ---
    case("router reaches the low-level policy transaction door", ROUTER_CPP,
         r"void CommandRouter::printServoSafeOff\(int id\) \{",
         "void CommandRouter::printServoSafeOff(int id) {\n"
         "  modules_.actuator_policy->plan(actuator::ActuatorCommand{}, "
         "core::AuthorityLease{}, core::OperatingMode::MAINTENANCE, nullptr);",
         "contains a call to plan()",
         runner=run_actuator_infrastructure_checks)

    # --- the reviewed CR3 transport surface stays singular ----------------
    # The one torque-on call is ServoBus::enableTorqueOn(). Turning SAFE_OFF
    # into a second torque-on call must therefore fail the exact-one gate.
    case("second torque-on in the servo transport", "ServoBus.cpp",
         r"st_\.EnableTorque\(static_cast<uint8_t>\(id\), 0\);",
         "st_.EnableTorque(static_cast<uint8_t>(id), 1);",
         "exactly one", runner=run_torque_checks)
    case("second GoalPosition primitive appears", "ServoBus.cpp",
         r"bool ServoBus::readRuntimeState\(int id, RuntimeState\* out\) \{",
         "bool ServoBus::extraWrite(int id) { st_.WritePosEx((uint8_t)id, 2048, 40, 10); return true; }\n"
         "bool ServoBus::readRuntimeState(int id, RuntimeState* out) {",
         "exactly one", runner=run_motion_surface_checks)

    if failures:
        print(f"SAFE_ACTUATOR_AUDIT_MUTATION_TESTS = FAIL ({len(failures)})")
        for f in failures:
            print(f"  - {f}")
        return 1
    print("SAFE_ACTUATOR_AUDIT_MUTATION_TESTS = PASS")
    return 0


if __name__ == "__main__":
    sys.exit(main())
