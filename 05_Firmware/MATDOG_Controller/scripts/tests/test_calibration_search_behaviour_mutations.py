#!/usr/bin/env python3
"""Behavioural mutation gate for the staged calibration endpoint search.

The static-audit mutation suite (test_static_audit_safe_actuator.py) proves
the AUDIT notices a changed token. This suite proves the HOST TESTS notice a
changed BEHAVIOUR: each mutation below re-creates a way the LF V25 oracle
semantics could silently regress (early stall accepted as contact, a wider
guard, a coarser fine step, no plateau bypass, no 20 ms cadence, ...). The
sketch is copied to a temp directory, one mutation is applied, and the full
scripts/tests/run_host_tests.sh must FAIL there. An unmutated copy must pass
first, so a broken environment cannot masquerade as "every mutation caught".

Offline only: no hardware, no Arduino toolchain, never touches the working
tree. ~35 s per mutation; run explicitly (it is not part of static_audit.py):

    python3 scripts/tests/test_calibration_search_behaviour_mutations.py
"""
import re
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

SKETCH = Path(__file__).resolve().parents[2]

PROBE_CPP = "src/calibration/ContactProbeEngine.cpp"
PROBE_H = "src/calibration/ContactProbeEngine.h"
RESOLVER_H = "src/actuator/CalibrationTargetResolver.h"
POLICY_CPP = "src/actuator/ActuatorWritePolicy.cpp"

# (name, file, exact anchor (must occur exactly once), replacement)
MUTATIONS = [
    ("early stall outside the corridor accepted as contact", PROBE_CPP,
     ": ContactDetectorState::EARLY_STALL;",
     ": ContactDetectorState::CONTACT_CONFIRMED;"),
    ("guard widened to URDF limit + 128", RESOLVER_H,
     "kCalibrationSearchGuardOvershootTicks = 64;",
     "kCalibrationSearchGuardOvershootTicks = 128;"),
    ("fine step 16 instead of 8", PROBE_H,
     "kSearchFineStepTicks = 8;", "kSearchFineStepTicks = 16;"),
    ("backoff 64 instead of 96", PROBE_H,
     "kSearchBackoffTicks = 96;", "kSearchBackoffTicks = 64;"),
    ("settle band 2 instead of 10 (servo shortfall becomes 'contact')", PROBE_H,
     "kSearchStaticToleranceTicks = 10;", "kSearchStaticToleranceTicks = 2;"),
    ("persistence 1 sample instead of 3", PROBE_H,
     "kSearchPersistenceSamples = 3;", "kSearchPersistenceSamples = 1;"),
    ("minimum contact travel 0 instead of 24", PROBE_H,
     "kSearchMinContactTravelTicks = 24;", "kSearchMinContactTravelTicks = 0;"),
    ("hard-current abort disabled (200 -> 2000)", PROBE_H,
     "kSearchHardCurrentAbortRaw = 200;", "kSearchHardCurrentAbortRaw = 2000;"),
    ("20 ms telemetry cadence gate removed", PROBE_CPP,
     "if (has_cadence_sample_ && now_ms - last_cadence_ms_ < kSearchSampleIntervalMs) return;",
     ""),
    ("unread speed (-1) counted as low velocity", PROBE_CPP,
     "const bool low_velocity = speed_magnitude >= 0 && speed_magnitude <= kSearchMaxVelocityRaw;",
     "const bool low_velocity = speed_magnitude <= kSearchMaxVelocityRaw;"),
    ("guard check removed (next step past the guard issued)", PROBE_CPP,
     "if (next_depth > guard_depth) {", "if (next_depth > guard_depth + 100000) {"),
    ("pass-2 friction plateau bypass removed", PROBE_CPP,
     "if (lag > static_cast<int32_t>(kSearchFineScoutLagToleranceTicks)) {",
     "if (lag > 100000) {"),
    ("repeatability check disabled", PROBE_CPP,
     "static_cast<int32_t>(request_.repeatability_tolerance_ticks)) {",
     "static_cast<int32_t>(4096)) {"),
    ("pass 1 alone completes (no backoff, no pass 2)", PROBE_CPP,
     "    status_.pass1_contact_tick = position;\n"
     "    status_.phase = ContactProbePhase::BACKOFF_PENDING;\n",
     "    status_.pass1_contact_tick = position;\n"
     "    status_.pass2_contact_tick = position;\n"
     "    finish(ContactProbePhase::COMPLETE, ContactProbeFailure::NONE,\n"
     "           actuator::WriteDecision::ACCEPT);\n"),
    ("backoff current-recovery check removed", PROBE_CPP,
     "if (current < 0 || current > threshold) {",
     "if (current < 0 || current > threshold + 100000) {"),
    ("backoff obstruction treated as arrival", PROBE_CPP,
     "    case actuator::MotionDeadmanVerdict::STALLED:\n"
     "      failSafeOff(ContactProbeFailure::UNEXPECTED_STALL_DURING_BACKOFF);\n"
     "      return;\n",
     "    case actuator::MotionDeadmanVerdict::STALLED:\n"
     "      beginPass(2, static_cast<uint16_t>(sample.present_position));\n"
     "      status_.phase = ContactProbePhase::STEP_PENDING;\n"
     "      return;\n"),
    ("calibration-search corridor granted to every operation", POLICY_CPP,
     "  if (command.calibration_search &&\n"
     "      command.operation != ActuatorOperation::CALIBRATION_CONTACT_PROBE) {\n"
     "    return WriteDecision::REJECT_CALIBRATION_SEARCH;\n"
     "  }\n",
     ""),
    ("V25 speed profile granted to POSITION_COMMAND", POLICY_CPP,
     "        (command.operation == ActuatorOperation::CALIBRATION_CONTACT_PROBE ||\n",
     "        (command.operation == ActuatorOperation::CALIBRATION_CONTACT_PROBE ||\n"
     "         command.operation == ActuatorOperation::POSITION_COMMAND ||\n"),
]


def copy_sketch(dst: Path) -> None:
    for sub in ("src", "scripts"):
        shutil.copytree(SKETCH / sub, dst / sub,
                        ignore=shutil.ignore_patterns("__pycache__", "*.pyc"))


def run_host_tests(root: Path) -> subprocess.CompletedProcess:
    return subprocess.run(["bash", str(root / "scripts/tests/run_host_tests.sh")],
                          cwd=root, capture_output=True, text=True, timeout=900)


COUNT_RX = re.compile(r"(\S+): (\d+) checks, (\d+) failures")
RUN_RX = re.compile(r"checks_run=\d+ failures=(\d+)")


def classify(out: str):
    """('behaviour', line) when a host-test suite ran and reported failures;
    ('compile', line) when the mutant did not even build - that proves
    nothing about behaviour and is NOT counted as caught; else ('other', line)."""
    for line in out.splitlines():
        if "error:" in line:
            return "compile", line.strip()[:160]
    for line in out.splitlines():
        m = COUNT_RX.search(line)
        if m and int(m.group(3)) > 0:
            return "behaviour", line.strip()[:160]
        m = RUN_RX.search(line)
        if m and int(m.group(1)) > 0:
            return "behaviour", line.strip()[:160]
        if "= FAIL" in line:
            return "behaviour", line.strip()[:160]
    return "other", "(non-zero exit without a failing suite summary)"


def main() -> int:
    for name, rel, anchor, _ in MUTATIONS:
        count = (SKETCH / rel).read_text().count(anchor)
        if count != 1:
            print(f"BEHAVIOUR_MUTATIONS = FAIL (anchor for '{name}' occurs {count}x in {rel})")
            return 1

    with tempfile.TemporaryDirectory(prefix="matdog_mut_") as tmp:
        baseline = Path(tmp) / "baseline"
        copy_sketch(baseline)
        res = run_host_tests(baseline)
        if res.returncode != 0:
            print("BEHAVIOUR_MUTATIONS = FAIL (unmutated copy does not pass: "
                  f"{classify(res.stdout + res.stderr)[1]})")
            return 1
        print("baseline (unmutated copy): host tests PASS", flush=True)

        caught = 0
        for i, (name, rel, anchor, repl) in enumerate(MUTATIONS, 1):
            root = Path(tmp) / f"m{i:02d}"
            copy_sketch(root)
            path = root / rel
            path.write_text(path.read_text().replace(anchor, repl, 1))
            res = run_host_tests(root)
            kind, detail = classify(res.stdout + res.stderr)
            ok = res.returncode != 0 and kind == "behaviour"
            caught += ok
            if res.returncode == 0:
                tag, detail = "MISSED", "host tests still PASS"
            elif not ok:
                tag = "MISSED"
                detail = f"{kind}: {detail}"
            else:
                tag = "CAUGHT"
            print(f"[{tag}] {i:02d} {name}: {detail}", flush=True)
            shutil.rmtree(root, ignore_errors=True)

    total = len(MUTATIONS)
    verdict = "PASS" if caught == total else "FAIL"
    print(f"BEHAVIOUR_MUTATIONS = {verdict} ({caught}/{total} caught)")
    return 0 if caught == total else 1


if __name__ == "__main__":
    sys.exit(main())
