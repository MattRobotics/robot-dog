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

The LF V25 coarse contact scout (restored 2026-09-30) adds one mutation per
dependency: no scout, a scout clamped at the corridor entry, a scout not
stored, a scout promoted to metrology, no backoff after it, fine passes
without the adaptive corridor / against fine pass 1 / with a wider lag, the
kinematic plateau removed, opened to the scout or loosened, the baseline
move removed or loosened, each per-sample readback dropped in the new
stages, the evidence mapped wrongly, and one endpoint kind run differently.
The V25 backoff StableTargetGate and TELEMETRY_TIMEOUT (2 s) have their own
mutations (no gate, a shorter window, fewer samples, a looser speed or band,
no reset, torque ignored while settling, the inherited 3 s timeout). The
final bounded partial coarse-scout step (a current-installation deviation) has
mutations for its removal, a target past the guard, a repeat at the guard and
its extension to the fine passes. The LF V25 runtime PresentTemperature
confirmation has mutations for removed confirmation reads, a 1-of-3 majority,
another servo's reading, a cached value, no 50 ms wait, and a failed read that
no longer fails closed.

The LF V25 ActivelyHeld supervision (the retired post-V25 held-joint speed
abort) has mutations for a speed abort reintroduced (at 40 or at the settle
bound 4), a looser drift, each held readback and safety check dropped, the
StableTargetGate / INITIAL_RECOVERY settle losing its speed criterion or
bound, and the held-role evidence / bounded transient diagnostic regressing.

The 24-contact Full Calibration sequence adds its own ORCHESTRATION
mutations (FullLegCalibrationExecutor / the sequence plan / the policy's
sequence door / the 24-contact finalizer): a skipped recovery, a limp held
joint, an unwatched drift, a missing prime, an unverified SAFE_OFF, a 5/6 or
18/24 success... each must be caught by the full-sequence host tests.

Offline only: no hardware, no Arduino toolchain, never touches the working
tree. Mutants run in parallel (MATDOG_MUTATION_JOBS, default 4); run
explicitly (it is not part of static_audit.py):

    python3 scripts/tests/test_calibration_search_behaviour_mutations.py
"""
import concurrent.futures
import os
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
EXEC_CPP = "src/calibration/FullLegCalibrationExecutor.cpp"
EXEC_H = "src/calibration/FullLegCalibrationExecutor.h"
SEQ_CPP = "src/actuator/CalibrationSequencePlan.cpp"
FIN_CPP = "src/calibration/FullLegCalibrationFinalizer.cpp"
FIN_H = "src/calibration/FullLegCalibrationFinalizer.h"
ENV_CPP = "src/actuator/OperationalEnvelope.cpp"
THERMAL_CPP = "src/calibration/ThermalConfirmation.cpp"

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
    ("fine-pass friction plateau bypass removed", PROBE_CPP,
     "if (!searchFineContactReproducesScout(request_.corridor, position, status_.scout_tick)) {",
     "if (false) {"),
    ("repeatability check disabled", PROBE_CPP,
     "static_cast<int32_t>(request_.repeatability_tolerance_ticks)) {",
     "static_cast<int32_t>(4096)) {"),
    ("fine pass 1 alone completes (no backoff, no fine pass 2)", PROBE_CPP,
     "  if (status_.pass == 1) {\n    status_.pass1_contact_tick = position;\n  } else {",
     "  if (status_.pass == 1) {\n    status_.pass1_contact_tick = position;\n"
     "    status_.pass2_contact_tick = position;\n"
     "    finish(ContactProbePhase::COMPLETE, ContactProbeFailure::NONE,\n"
     "           actuator::WriteDecision::ACCEPT);\n    return;\n  } else {"),
    ("backoff current-recovery check removed", PROBE_CPP,
     "if (current < 0 || current > threshold) {",
     "if (current < 0 || current > threshold + 100000) {"),
    ("backoff obstruction treated as arrival", PROBE_CPP,
     "    case actuator::MotionDeadmanVerdict::STALLED:\n"
     "      failSafeOff(ContactProbeFailure::UNEXPECTED_STALL_DURING_BACKOFF);\n"
     "      return;\n",
     "    case actuator::MotionDeadmanVerdict::STALLED:\n"
     "      beginPass(static_cast<uint8_t>(status_.pass + 1),\n"
     "                static_cast<uint16_t>(sample.present_position));\n"
     "      if (!active()) return;\n"
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

    # --- the LF V25 coarse contact scout (2026-09-30) ----------------------
    ("no coarse scout: 8-tick steps from the baseline end", PROBE_CPP,
     "    step_ticks_ = kSearchCoarseStepTicks;\n"
     "    next_depth = target_depth + kSearchCoarseStepTicks;\n",
     "    step_ticks_ = kSearchFineStepTicks;\n"
     "    next_depth = target_depth + kSearchFineStepTicks;\n"),
    ("coarse scout clamped at the corridor entry (the #34/#35 transit)", PROBE_CPP,
     "    next_depth = target_depth + kSearchCoarseStepTicks;\n",
     "    next_depth = target_depth + kSearchCoarseStepTicks;\n"
     "    if (target_depth < depth(request_.corridor.entry_tick) &&\n"
     "        next_depth > depth(request_.corridor.entry_tick)) {\n"
     "      next_depth = depth(request_.corridor.entry_tick);\n    }\n"),
    ("coarse scout accepted in the adaptive (fine) corridor", PROBE_CPP,
     "  int32_t acceptance_entry = depth(request_.corridor.entry_tick);\n  if (pass > 0) {",
     "  int32_t acceptance_entry = depth(request_.corridor.entry_tick) - kSearchAdaptiveScoutTicks;\n"
     "  if (pass > 0) {"),
    ("scout tick not stored as reference evidence", PROBE_CPP,
     "    status_.scout_tick = position;\n", ""),
    ("scout promoted to metrology (repeatability scout vs fine 2)", PROBE_CPP,
     "  if (absDiff(status_.pass1_contact_tick, status_.pass2_contact_tick) <=",
     "  if (absDiff(status_.scout_tick, status_.pass2_contact_tick) <="),
    ("witness deviation measured against the scout", PROBE_CPP,
     "      static_cast<uint16_t>(absDiff(status_.pass1_contact_tick, status_.pass2_contact_tick));",
     "      static_cast<uint16_t>(absDiff(status_.scout_tick, status_.pass2_contact_tick));"),
    ("scout release (stop_pressure) skipped", PROBE_CPP,
     "    status_.scout_valid = true;\n    beginRelease(position);\n",
     "    status_.scout_valid = true;\n    contact_tick_ = position;\n"
     "    status_.phase = ContactProbePhase::BACKOFF_PENDING;\n"),
    ("release written but never read back", PROBE_CPP,
     "  status_.stage = ContactSearchStage::RELEASE;\n"
     "  status_.phase = ContactProbePhase::RELEASE_VERIFYING;\n",
     "  status_.stage = ContactSearchStage::RELEASE;\n"
     "  status_.phase = status_.pass < 2 ? ContactProbePhase::BACKOFF_PENDING\n"
     "                                   : ContactProbePhase::RELEASE_VERIFYING;\n"),
    ("no backoff after the scout (fine pass 1 starts on the stop)", PROBE_CPP,
     "  if (status_.pass < 2) {\n    status_.phase = ContactProbePhase::BACKOFF_PENDING;\n    return;\n  }",
     "  if (status_.pass == 0) {\n    beginPass(1, contact_tick_);\n    if (!active()) return;\n"
     "    status_.phase = ContactProbePhase::STEP_PENDING;\n    return;\n  }\n"
     "  if (status_.pass < 2) {\n    status_.phase = ContactProbePhase::BACKOFF_PENDING;\n    return;\n  }"),
    ("fine passes without the scout-adaptive corridor", PROBE_CPP,
     "    acceptance_entry = searchAdaptiveAcceptanceEntryDepth(request_.corridor, status_.scout_tick);",
     "    (void)searchAdaptiveAcceptanceEntryDepth(request_.corridor, status_.scout_tick);"),
    ("adaptive corridor extended toward the guard", PROBE_CPP,
     "  return adaptive < entry ? adaptive : entry;", "  (void)entry;\n  return adaptive;"),
    ("ADAPTIVE_FINE_SCOUT_TICKS 32 -> 0", PROBE_H,
     "kSearchAdaptiveScoutTicks = 32;", "kSearchAdaptiveScoutTicks = 0;"),
    ("fine passes judged against fine pass 1 (the #34/#35 rule)", PROBE_CPP,
     "if (!searchFineContactReproducesScout(request_.corridor, position, status_.scout_tick)) {",
     "if (status_.pass == 2 &&\n"
     "      !searchFineContactReproducesScout(request_.corridor, position, status_.pass1_contact_tick)) {"),
    ("FINE_CONTACT_SCOUT_LAG_TOLERANCE_TICKS 8 -> 16", PROBE_H,
     "kSearchFineScoutLagToleranceTicks = 8;", "kSearchFineScoutLagToleranceTicks = 16;"),
    ("scout lag measured the wrong way round", PROBE_CPP,
     "  const int32_t lag = actuator::searchDepth(corridor, scout) - actuator::searchDepth(corridor, candidate);",
     "  const int32_t lag = actuator::searchDepth(corridor, candidate) - actuator::searchDepth(corridor, scout);"),
    ("kinematic-plateau path removed", PROBE_CPP,
     "    if (status_.pass == 0) {\n      failSafeOff(ContactProbeFailure::TRACKING_FAILED);",
     "    if (true) {\n      failSafeOff(ContactProbeFailure::TRACKING_FAILED);"),
    ("kinematic plateau opened to the coarse scout", PROBE_CPP,
     "    if (status_.pass == 0) {\n      failSafeOff(ContactProbeFailure::TRACKING_FAILED);\n      return;\n    }\n",
     ""),
    ("kinematic plateau: scout-distance rule removed", PROBE_CPP,
     "  const bool near_scout =\n"
     "      absDiff(position, status_.scout_tick) <= static_cast<int32_t>(kSearchAdaptiveScoutTicks);",
     "  const bool near_scout = true;"),
    ("kinematic plateau: span 3 -> 30 ticks", PROBE_H,
     "kSearchKinematicPlateauSpanTicks = 3;", "kSearchKinematicPlateauSpanTicks = 30;"),
    ("kinematic plateau: 1 sample instead of 3", PROBE_H,
     "kSearchKinematicPlateauSamples = 3;", "kSearchKinematicPlateauSamples = 1;"),
    ("no baseline move (baseline travel 64 -> 0)", PROBE_H,
     "kSearchBaselineTravelTicks = 64;", "kSearchBaselineTravelTicks = 0;"),
    ("baseline counts idle samples as moving", PROBE_CPP,
     "          (last_cadence_position_ >= 0 && position != last_cadence_position_) || speed > 0;",
     "          speed >= 0;"),
    ("baseline deadline never expires", PROBE_CPP,
     "  if (now_ms - baseline_started_ms_ >= kSearchBaselineTimeoutMs) {", "  if (false) {"),
    ("baseline minimum samples 6 -> 0", PROBE_H,
     "kSearchBaselineMinSamples = 6;", "kSearchBaselineMinSamples = 0;"),
    ("baseline move skips the per-sample readback", PROBE_CPP,
     "  if (usableSafeSample(now_ms, telemetry_available, telemetry)) {",
     "  if (telemetry_available && sampleUsable(telemetry)) {"),
    ("release verification skips the per-sample readback", PROBE_CPP,
     "  if (!usableSafeSample(now_ms, telemetry_available, telemetry)) return;\n  if (status_.pass < 2) {",
     "  (void)now_ms;\n  if (!(telemetry_available && sampleUsable(telemetry))) return;\n"
     "  if (status_.pass < 2) {"),
    ("search steps ignore the hard current", PROBE_CPP,
     "  if (magnitude(telemetry.present_current) >= kSearchHardCurrentAbortRaw) {\n"
     "    failSafeOff(ContactProbeFailure::HARD_CURRENT_ABORT);",
     "  if (false) {\n    failSafeOff(ContactProbeFailure::HARD_CURRENT_ABORT);"),
    ("search steps ignore the GoalPosition readback", PROBE_CPP,
     "  if (telemetry.goal_position != static_cast<int32_t>(status_.target_tick)) {\n"
     "    failSafeOff(ContactProbeFailure::GOAL_READBACK_MISMATCH);",
     "  if (false) {\n    failSafeOff(ContactProbeFailure::GOAL_READBACK_MISMATCH);"),
    ("search steps ignore the TorqueLimit readback", PROBE_CPP,
     "  if (telemetry.torque_limit != static_cast<int32_t>(request_.expected_torque_limit)) {\n"
     "    failSafeOff(ContactProbeFailure::TORQUE_LIMIT_CHANGED);",
     "  if (false) {\n    failSafeOff(ContactProbeFailure::TORQUE_LIMIT_CHANGED);"),
    ("telemetry loss never goes stale", PROBE_CPP,
     "    if (now_ms - last_good_ms_ >= kSearchTelemetryTimeoutMs) {\n"
     "      failSafeOff(ContactProbeFailure::STALE_TELEMETRY);",
     "    if (false) {\n      failSafeOff(ContactProbeFailure::STALE_TELEMETRY);"),
    ("HIP endpoints skip the coarse scout (not one generic search)", PROBE_CPP,
     "  if (status_.pass == 0) {\n    // The V25 coarse contact scout",
     "  if (status_.pass == 0 && request_.endpoint_joint != JointKind::HIP) {\n"
     "    // The V25 coarse contact scout"),
    ("executor records fine pass 1 as the scout", EXEC_CPP,
     "  e.coarse_tick = probe_.status().scout_tick;",
     "  e.coarse_tick = probe_.status().pass1_contact_tick;"),
    ("executor records fine pass 2 twice (the #34/#35 mapping)", EXEC_CPP,
     "  e.fine_tick_1 = probe_.status().pass1_contact_tick;",
     "  e.fine_tick_1 = probe_.status().pass2_contact_tick;"),
    ("diagnostics read the coarse scout", EXEC_CPP,
     "  d.min_contact_tick = midpoint(min_side.fine_tick_1, min_side.fine_tick_2);",
     "  d.min_contact_tick = midpoint(min_side.coarse_tick, min_side.fine_tick_1);"),
    ("contact envelope bounded by the coarse scout", ENV_CPP,
     "  const uint16_t lo = minTick(request.min_side_evidence.fine_tick_2,\n"
     "                              request.max_side_evidence.fine_tick_2);",
     "  const uint16_t lo = minTick(request.min_side_evidence.coarse_tick,\n"
     "                              request.max_side_evidence.coarse_tick);"),

    # --- V25 backoff StableTargetGate and TELEMETRY_TIMEOUT (2026-09-30) -----
    ("backoff arrived on the first in-band sample (no V25 settle gate)", PROBE_CPP,
     "      if (!settle_.observe(static_cast<uint16_t>(sample.present_position),\n"
     "                           magnitude(sample.present_speed), status_.target_tick, now_ms)) {\n"
     "        return;\n      }\n",
     "      (void)settle_;\n"),
    ("backoff settle window 400 -> 0 ms", PROBE_H,
     "kSearchBackoffSettleWindowMs = 400;", "kSearchBackoffSettleWindowMs = 0;"),
    ("backoff settle needs 1 sample instead of 4", PROBE_H,
     "kSearchBackoffSettledSamples = 4;", "kSearchBackoffSettledSamples = 1;"),
    ("backoff settle speed limit 4 -> 1000 raw", PROBE_H,
     "kSearchBackoffSettleMaxSpeedRaw = 4;", "kSearchBackoffSettleMaxSpeedRaw = 1000;"),
    ("backoff settle band 12 -> 40 ticks", PROBE_H,
     "kSearchBackoffSettleToleranceTicks = 12;", "kSearchBackoffSettleToleranceTicks = 40;"),
    ("settle gate keeps counting across a non-qualifying sample", PROBE_CPP,
     "  if (!qualifies) {\n    reset();\n    return false;\n  }",
     "  if (!qualifies) {\n    return false;\n  }"),
    ("leaving the arrival band does not restart the settle gate", PROBE_CPP,
     "      if (usable) settle_.reset();  // outside the band: V25 restarts the gate\n", ""),
    ("backoff ignores TorqueEnable while settling in the band", PROBE_CPP,
     "    if (!sampleSafe(telemetry)) return;\n  } else if (telemetry_available) {",
     "    if (telemetry.torque_enable != 0 && !sampleSafe(telemetry)) return;\n"
     "  } else if (telemetry_available) {"),
    ("search telemetry timeout 2 s -> 3 s (the inherited value)", PROBE_H,
     "kSearchTelemetryTimeoutMs = 2000;", "kSearchTelemetryTimeoutMs = 3000;"),

    # --- the final bounded partial coarse-scout step (2026-09-30) ------------
    ("final partial scout step removed (the V25 grid gap back)", PROBE_CPP,
     "    if (next_depth > guard_depth && target_depth < guard_depth) {",
     "    if (false && next_depth > guard_depth && target_depth < guard_depth) {"),
    ("final partial scout step one tick beyond the guard", PROBE_CPP,
     "      next_depth = guard_depth;\n", "      next_depth = guard_depth + 1;\n"),
    ("final partial scout step repeated at the guard", PROBE_CPP,
     "    if (next_depth > guard_depth && target_depth < guard_depth) {",
     "    if (next_depth > guard_depth) {"),
    ("final partial step taken by the fine passes too", PROBE_CPP,
     "  } else {\n    step_ticks_ = kSearchFineStepTicks;\n    next_depth = target_depth + kSearchFineStepTicks;\n",
     "  } else {\n    step_ticks_ = kSearchFineStepTicks;\n    next_depth = target_depth + kSearchFineStepTicks;\n"
     "    if (next_depth > guard_depth && target_depth < guard_depth) next_depth = guard_depth;\n"),

    # --- LF V25 runtime PresentTemperature confirmation (2026-09-30) ---------
    ("thermal: the two confirmation reads removed", THERMAL_CPP,
     "  for (uint8_t i = 1; i < kThermalConfirmationReads; ++i) {",
     "  for (uint8_t i = 1; i < 1; ++i) {"),
    ("thermal: >= 2 of 3 weakened to 1 of 3", THERMAL_CPP,
     "  if (over_limit >= kThermalConfirmedOverLimit) {", "  if (over_limit >= 1) {"),
    ("thermal: confirmation reads another servo", THERMAL_CPP,
     "    if (!port->readPresentTemperatureDirect(bus_id, &celsius) || celsius < 0) {",
     "    if (!port->readPresentTemperatureDirect(static_cast<uint8_t>(bus_id + 1), &celsius) ||\n"
     "        celsius < 0) {"),
    ("thermal: cached trigger value instead of a fresh direct read", THERMAL_CPP,
     "    if (!port->readPresentTemperatureDirect(bus_id, &celsius) || celsius < 0) {",
     "    celsius = observed_c;\n    if (celsius < 0) {"),
    ("thermal: the 50 ms wait before each confirmation removed", THERMAL_CPP,
     "    port->delayMs(kThermalConfirmationDelayMs);\n", ""),
    ("thermal: a failed confirmation read no longer fails closed", THERMAL_CPP,
     "    if (!port->readPresentTemperatureDirect(bus_id, &celsius) || celsius < 0) {\n"
     "      out.decision = ThermalDecision::CONFIRMATION_READ_FAILED;\n",
     "    if (!port->readPresentTemperatureDirect(bus_id, &celsius) || celsius < 0) {\n"
     "      out.decision = ThermalDecision::CONFIRMATION_READ_FAILED;\n"
     "      out.published_c = kThermalLimitC;\n"),

    # --- the 24-contact Full Calibration orchestration ---------------------
    ("INITIAL_RECOVERY skips joints already near q0", EXEC_CPP,
     "      if (distance > static_cast<int32_t>(actuator::kSequencePrimeMaxDistanceTicks)) {",
     "      if (distance <= static_cast<int32_t>(kSequenceStaticToleranceTicks)) {\n"
     "        ++recover_index_;\n        return;\n      }\n"
     "      if (distance > static_cast<int32_t>(actuator::kSequencePrimeMaxDistanceTicks)) {"),
    ("TorqueEnable without the V25 prime (stale GoalPosition)", EXEC_CPP,
     "    op_prime_tick_ = static_cast<uint16_t>(s->present_position);\n"
     "    if (!writePrime(ctx, j, op_prime_tick_)) return;\n",
     "    op_prime_tick_ = static_cast<uint16_t>(s->goal_position);\n"),
    ("UPPER probed with a limp HIP", EXEC_CPP,
     "      addStep(Op::HOLD, kSlotHip);\n      addStep(Op::ENERGIZE, kSlotLower);",
     "      addStep(Op::ENERGIZE, kSlotLower);"),
    ("LOWER probed without the UPPER horizontal pose", EXEC_CPP,
     "      addStep(Op::MOVE, kSlotUpper, r.upper_for_lower_tick, r.upper_for_lower_urad);\n",
     ""),
    ("HIP MAX skips the per-side clearance pose", EXEC_CPP,
     "      if (hip_poses_differ) {\n        // V25 per-side clearance",
     "      if (hip_poses_differ && false) {\n        // V25 per-side clearance"),
    ("held-joint drift ignored", EXEC_CPP,
     "      failHeldRole(observeHeld(s, sample, now_ms, t), FullLegFailure::HELD_JOINT_DRIFT);\n"
     "      return false;",
     "      (void)0;"),
    ("held-joint TorqueLimit readback ignored", EXEC_CPP,
     "        sample->torque_limit != static_cast<int32_t>(request_.torque_limit) ||\n"
     "        sample->goal_position != static_cast<int32_t>(st.target_tick)) {",
     "        sample->goal_position != static_cast<int32_t>(st.target_tick)) {"),
    ("held-joint telemetry loss never fails", EXEC_CPP,
     "      if (now_ms - st.last_good_ms >= kSequenceMaxTelemetryAgeMs) {\n"
     "        failHeldRole(observeHeld(s, nullptr, now_ms, t), FullLegFailure::STALE_TELEMETRY);",
     "      if (false) {\n"
     "        failHeldRole(observeHeld(s, nullptr, now_ms, t), FullLegFailure::STALE_TELEMETRY);"),
    # LF V25 ActivelyHeld supervision (2026-09-30 correction): an already-held
    # joint is never aborted on its speed alone; speed stays the settling gate.
    ("held-joint speed abort reintroduced (the retired post-V25 D5 rule)", EXEC_CPP,
     "    st.speed_transient = fast;",
     "    if (fast) {\n"
     "      failHeldRole(observeHeld(s, sample, now_ms, t), FullLegFailure::HELD_JOINT_DRIFT);\n"
     "      return false;\n    }\n    st.speed_transient = fast;"),
    ("held-joint speed abort at the V25 settle bound (|speed| > 4)", EXEC_CPP,
     "    st.last_good_ms = now_ms;\n    st.has_last_sample = true;",
     "    st.last_good_ms = now_ms;\n"
     "    if (magnitude(sample->present_speed) > static_cast<int32_t>(kSequenceSettleMaxSpeedRaw)) {\n"
     "      fail(FullLegFailure::HELD_JOINT_DRIFT);\n      return false;\n    }\n"
     "    st.has_last_sample = true;"),
    ("held-joint drift tolerance 11 instead of 10", EXEC_CPP,
     "        static_cast<int32_t>(kSequenceStaticToleranceTicks)) {\n"
     "      failHeldRole(observeHeld(s, sample, now_ms, t), FullLegFailure::HELD_JOINT_DRIFT);",
     "        static_cast<int32_t>(kSequenceStaticToleranceTicks) + 1) {\n"
     "      failHeldRole(observeHeld(s, sample, now_ms, t), FullLegFailure::HELD_JOINT_DRIFT);"),
    ("held-joint TorqueEnable readback ignored", EXEC_CPP,
     "    if (sample->torque_enable != 1 ||\n"
     "        sample->torque_limit != static_cast<int32_t>(request_.torque_limit) ||",
     "    if (sample->torque_limit != static_cast<int32_t>(request_.torque_limit) ||"),
    ("held-joint GoalPosition readback ignored", EXEC_CPP,
     "        sample->goal_position != static_cast<int32_t>(st.target_tick)) {\n"
     "      failHeldRole(",
     "        false) {\n      failHeldRole("),
    ("held-joint status / current / temperature skipped", EXEC_CPP,
     "    if (safety != FullLegFailure::NONE) {",
     "    if (false) {"),
    ("StableTargetGate promotes a moving joint to held (speed criterion removed)", EXEC_CPP,
     "      absDiff(s->present_position, p.target_tick) <= static_cast<int32_t>(kSequenceStaticToleranceTicks) &&\n"
     "      magnitude(s->present_speed) <= static_cast<int32_t>(kSequenceSettleMaxSpeedRaw);",
     "      absDiff(s->present_position, p.target_tick) <= static_cast<int32_t>(kSequenceStaticToleranceTicks);"),
    ("INITIAL_RECOVERY settles a moving joint (speed criterion removed)", EXEC_CPP,
     "          absDiff(s->present_position, j.q0_tick) <= static_cast<int32_t>(kSequenceStaticToleranceTicks) &&\n"
     "          magnitude(s->present_speed) <= static_cast<int32_t>(kSequenceSettleMaxSpeedRaw);",
     "          absDiff(s->present_position, j.q0_tick) <= static_cast<int32_t>(kSequenceStaticToleranceTicks);"),
    ("settle speed bound 5 instead of the V25 4", EXEC_H,
     "constexpr uint16_t kSequenceSettleMaxSpeedRaw = 4;",
     "constexpr uint16_t kSequenceSettleMaxSpeedRaw = 5;"),
    ("held-role failure evidence not latched", EXEC_CPP,
     "    held_role_failure_ = observation;",
     "    (void)observation;"),
    ("held-role failure evidence observed after the failure (TORQUE_OFF, probe gone)", EXEC_CPP,
     "      failHeldRole(observeHeld(s, sample, now_ms, t), safety);",
     "      fail(safety);\n      failHeldRole(observeHeld(s, sample, now_ms, t), safety);"),
    ("held speed transient not recorded", EXEC_CPP,
     "        held_transients_tick_[held_transients_tick_count_++] = observeHeld(s, sample, now_ms, t);\n",
     ""),
    ("held speed transient recorded every sample, not per rising edge", EXEC_CPP,
     "    if (fast && !st.speed_transient) {",
     "    if (fast) {"),
    ("held speed transient records unbounded", EXEC_CPP,
     "      if (held_transients_recorded_ < kHeldSpeedTransientEventCap &&",
     "      if (true &&"),
    ("bystander drift ignored", EXEC_CPP,
     "  } else if (absDiff(sample->present_position, ps.entry_tick) >",
     "  } else if (false && absDiff(sample->present_position, ps.entry_tick) >"),
    ("hard current on a held joint ignored", EXEC_CPP,
     "  if (magnitude(s.present_current) >= kSearchHardCurrentAbortRaw) {",
     "  if (magnitude(s.present_current) >= 100000) {"),
    ("SAFE_OFF_ALL completes without verification", EXEC_CPP,
     "  if ((safe_off_verified_mask_ & all) != all) return;  // retried every tick until verified",
     "  (void)all;"),
    ("diagnostics rejection skips the reviewed return", EXEC_CPP,
     "    return_after_diagnostics_failure_ = true;",
     "    fail(FullLegFailure::DIAGNOSTICS_REJECTED);"),
    ("phase table lets any joint take the horizontal pose", SEQ_CPP,
     "      return upper && target == p.upper_for_lower;",
     "      return target == p.upper_for_lower;"),
    ("policy trusts the caller's sequence tick", POLICY_CPP,
     "          expected_tick != command.target_tick) {",
     "          false) {"),
    ("sequence probe ignores the held prerequisites", POLICY_CPP,
     "  if (!bootstrap_.sequence_prerequisites_verified) {",
     "  if (false) {"),
    ("finalizer accepts a 5/6 leg", FIN_CPP,
     "  if (measured != kFullLegContactsExpected || outcome.contacts_measured != kFullLegContactsExpected) {",
     "  if (measured < kFullLegContactsExpected - 1) {"),
    # (Relaxing only the 24/24 total is an equivalent mutant: 4 legs at 6/6
    # already imply it - the total is defence in depth. Both guards at once:)
    ("four-leg verdict counts legs present, not contacts", FIN_H,
     "return legsContactCalibrated() == kLegCount &&\n"
     "           totalContactsAccepted() == kFullCalibrationContactsExpected;",
     "return legsPresent() == kLegCount;"),
]


def copy_sketch(dst: Path) -> None:
    for sub in ("src", "scripts"):
        shutil.copytree(SKETCH / sub, dst / sub,
                        ignore=shutil.ignore_patterns("__pycache__", "*.pyc"))


def run_host_tests(root: Path) -> subprocess.CompletedProcess:
    return subprocess.run(["bash", str(root / "scripts/tests/run_host_tests.sh")],
                          cwd=root, capture_output=True, text=True, timeout=3600)


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

        def one(item):
            i, (name, rel, anchor, repl) = item
            root = Path(tmp) / f"m{i:02d}"
            copy_sketch(root)
            path = root / rel
            path.write_text(path.read_text().replace(anchor, repl, 1))
            res = run_host_tests(root)
            kind, detail = classify(res.stdout + res.stderr)
            ok = res.returncode != 0 and kind == "behaviour"
            if res.returncode == 0:
                tag, detail = "MISSED", "host tests still PASS"
            elif not ok:
                tag = "MISSED"
                detail = f"{kind}: {detail}"
            else:
                tag = "CAUGHT"
            shutil.rmtree(root, ignore_errors=True)
            return i, name, tag, detail, ok

        jobs = int(os.environ.get("MATDOG_MUTATION_JOBS", "4"))
        caught = 0
        with concurrent.futures.ThreadPoolExecutor(max_workers=jobs) as pool:
            for i, name, tag, detail, ok in pool.map(one, enumerate(MUTATIONS, 1)):
                caught += ok
                print(f"[{tag}] {i:02d} {name}: {detail}", flush=True)

    total = len(MUTATIONS)
    verdict = "PASS" if caught == total else "FAIL"
    print(f"BEHAVIOUR_MUTATIONS = {verdict} ({caught}/{total} caught)")
    return 0 if caught == total else 1


if __name__ == "__main__":
    sys.exit(main())
