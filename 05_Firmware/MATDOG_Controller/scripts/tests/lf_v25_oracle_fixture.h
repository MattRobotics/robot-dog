#ifndef MATDOG_TESTS_LF_V25_ORACLE_FIXTURE_H
#define MATDOG_TESTS_LF_V25_ORACLE_FIXTURE_H

#include <stdint.h>

#include "../../src/calibration/CalibrationDomain.h"

// LF V25 historical hardware oracle — deterministic replay fixture.
//
// EVERY NUMBER BELOW IS QUOTED FROM THE ARCHIVE. Nothing is interpolated,
// rounded or invented. Where the archive is silent the field is absent rather
// than guessed.
//
// Primary sources, all inside this repository:
//
//   06_Software/Matdog_Core/calibration/MATDOG_LF_CALIBRATION_V25_FINAL.md
//       section 1  step count 58/58, stage LF_STAGED, freeze LF_FROZEN
//       section 2  accepted MIN/MAX contact ticks + fine repeatability
//       section 3  affine q0 per joint
//       section 4  EEPROM offsets and final displayed positions
//   06_Software/Matdog_Core/calibration/
//       MATDOG_LF_V25_HARDWARE_EVIDENCE_2026-08-04.json
//       contact angles in radians/degrees, status IMMUTABLE_EXTERNAL_EVIDENCE
//   09_Logs/Historical/NormaCore_MATDOG_Archive/LF_V25_Hardware_Oracle/
//       source/.../auto_calibrate/matdog.rs
//       JointSpec direction constants, LF_CONTACT_WITNESS_TOLERANCE_TICKS = 24
//       source/tools/matdog/matdog_headless_auto_calibrate.py
//       EXPECTED_FULL_TOTAL_STEPS = 58
//
// ⚠️ WHAT THIS FIXTURE IS NOT
//
// It is not current calibration and cannot become current calibration. It
// describes the PRE-2026-08-27 physical installation. All 17 servos were
// removed, re-provisioned to PositionOffset = 0 and remounted; bus ids were
// recoded and physical units moved between joints (the unit labelled M11 in
// this fixture is NECK_PITCH today). Every record here is tagged
// CalibrationOrigin::HISTORICAL_REPLAY, and mayPromote() refuses that origin.
//
// The motor ids below are HISTORICAL PROVENANCE ONLY. They must never be used
// to address hardware.

namespace matdog {
namespace tests {

// MATDOG_LF_CALIBRATION_V25_FINAL.md §1 and
// matdog_headless_auto_calibrate.py::EXPECTED_FULL_TOTAL_STEPS.
constexpr uint16_t kLfV25ExpectedTotalSteps = 58;

// The run calibrated ONE leg. The profile table covers 24 contacts; LF V25
// executed the 6 that belong to LF. Claiming 24 hardware-validated contacts
// would be false.
constexpr uint8_t kLfV25ExecutedContacts = 6;

struct LfV25JointRecord {
  calibration::JointKind joint;
  uint8_t historical_motor_id;   // PROVENANCE ONLY - not a current bus id
  int8_t spec_direction;         // matdog.rs JointSpec - a CONSTANT, not measured
  uint16_t accepted_min_tick;
  uint16_t accepted_max_tick;
  uint16_t q0_affine_tick;       // NOTE: none of these is 2048
  int16_t eeprom_previous_offset;
  int16_t eeprom_frozen_offset;
  uint16_t final_displayed_position;
};

// MATDOG_LF_CALIBRATION_V25_FINAL.md §2, §3, §4.
constexpr LfV25JointRecord kLfV25Records[3] = {
    // | joint | M  | dir | MIN  | MAX  | q0_affine | off_prev | off_new | displayed |
    {calibration::JointKind::HIP,   13, -1, 2535, 1600, 2067, -505, -486, 2048},
    {calibration::JointKind::UPPER, 12, +1, 1439, 3443, 2040,  859,  851, 2051},
    {calibration::JointKind::LOWER, 11, -1, 3093, 1658, 2074,  101,  127, 2046},
};

// MATDOG_LF_CALIBRATION_V25_FINAL.md §2 — the two documented fine sequences.
// These are the only per-sample traces the archive records; the remaining
// contacts report an accepted value without a published sample sequence, so
// no sequence is fabricated for them.
struct LfV25FineSequence {
  calibration::JointKind joint;
  calibration::ContactSide side;
  uint16_t coarse_tick;
  uint16_t fine_tick_1;
  uint16_t fine_tick_2;
  uint16_t documented_spread_ticks;
};

constexpr LfV25FineSequence kLfV25FineSequences[2] = {
    // M13 MAX reached the real contact after the V24 false abort.
    {calibration::JointKind::HIP,   calibration::ContactSide::MAX_SIDE, 1595, 1599, 1601, 2},
    // M12 MAX returned to the expected physical region once the cable
    // obstruction was removed.
    {calibration::JointKind::UPPER, calibration::ContactSide::MAX_SIDE, 3446, 3443, 3444, 1},
};

// MATDOG_LF_CALIBRATION_V25_FINAL.md §5.4 — the uniform hardware witness
// rejected the cable-obstructed M12 MAX and accepted the unobstructed result.
// This is the one documented FAILURE the archive supports replaying.
constexpr uint16_t kLfV25M12MaxObstructedTick = 3397;
constexpr uint16_t kLfV25M12MaxAcceptedTick = 3443;

// matdog.rs: LF_CONTACT_WITNESS_TOLERANCE_TICKS = 24.
//
// HISTORICAL AND LF-ONLY. It was measured on one leg, on an installation that
// no longer exists, and the evidence file forbids mirroring LF results onto
// RF/RH/LH. It is deliberately NOT a constant in the domain model: every
// witness there must be given its own band explicitly.
constexpr uint16_t kLfV25ContactWitnessToleranceTicks = 24;

// MATDOG_LF_CALIBRATION_V25_FINAL.md §5.3 — the bounded tracking-lag rule,
// stated as general and not M13-specific.
constexpr uint16_t kLfV25SettleContinueTicks = 13;  // continue
constexpr uint16_t kLfV25SettleFailTicks = 17;      // fail closed
constexpr uint16_t kLfV25SettleFloorTicks = 16;     // the global floor

// The phase order the LF session actually walked (matdog.rs LfSessionState).
constexpr calibration::CalibrationPhase kLfV25PhaseSequence[] = {
    calibration::CalibrationPhase::PREFLIGHT,
    calibration::CalibrationPhase::INITIAL_RECOVERY,
    calibration::CalibrationPhase::PARKING,
    calibration::CalibrationPhase::UPPER_MIN,
    calibration::CalibrationPhase::UPPER_MAX,
    calibration::CalibrationPhase::UPPER_HORIZONTAL,
    calibration::CalibrationPhase::LOWER_MIN,
    calibration::CalibrationPhase::LOWER_MAX,
    calibration::CalibrationPhase::LOWER_FOLDED,
    calibration::CalibrationPhase::HIP_MIN,
    calibration::CalibrationPhase::HIP_MAX,
    calibration::CalibrationPhase::DIAGNOSTICS,
    calibration::CalibrationPhase::RETURN_HIP,
    calibration::CalibrationPhase::RETURN_LOWER_HELD,
    calibration::CalibrationPhase::RETURN_UPPER,
    calibration::CalibrationPhase::RESTORE_PARKING,
    calibration::CalibrationPhase::CLEANUP,
    calibration::CalibrationPhase::TORQUE_OFF,
};

constexpr size_t kLfV25PhaseSequenceLength =
    sizeof(kLfV25PhaseSequence) / sizeof(kLfV25PhaseSequence[0]);

}  // namespace tests
}  // namespace matdog

#endif  // MATDOG_TESTS_LF_V25_ORACLE_FIXTURE_H
