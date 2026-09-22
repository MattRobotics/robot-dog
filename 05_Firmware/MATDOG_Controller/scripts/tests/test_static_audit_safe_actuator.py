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
         r"(  CALIBRATION_CONTACT_PROBE = 3,)",
         r"\1\n  EEPROM_WRITE              = 4,",
         "EEPROM")
    case("a provisioning write becomes expressible", POLICY_H,
         r"(  CALIBRATION_CONTACT_PROBE = 3,)",
         r"\1\n  POSITION_OFFSET_WRITE     = 4,",
         "OFFSET")
    case("torque removal becomes expressible", POLICY_H,
         r"(  CALIBRATION_CONTACT_PROBE = 3,)",
         r"\1\n  TORQUE_DISABLE            = 4,",
         "DISABLE")
    case("an operation class disappears", POLICY_H,
         r"  CALIBRATION_CONTACT_PROBE = 3,",
         "",
         "CALIBRATION_CONTACT_PROBE")
    case("the operation count drifts upward", POLICY_H,
         r"kActuatorOperationCount = 4;",
         "kActuatorOperationCount = 5;",
         "kActuatorOperationCount")
    case("the operation count drifts downward", POLICY_H,
         r"kActuatorOperationCount = 4;",
         "kActuatorOperationCount = 3;",
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
