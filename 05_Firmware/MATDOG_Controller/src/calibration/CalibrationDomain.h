#ifndef MATDOG_CALIBRATION_CALIBRATION_DOMAIN_H
#define MATDOG_CALIBRATION_CALIBRATION_DOMAIN_H

#include <stddef.h>
#include <stdint.h>

// MATDOG calibration domain model — recovered from the LF V25 hardware oracle,
// not redesigned.
//
// Pure: <stdint.h> only. No <Arduino.h>, no ServoBus, no Wi-Fi, no OTA, no
// hardware call of any kind. Same contract as network/WifiPolicy,
// update/OtaPolicy and core/ActuatorAuthority, and for the same reason:
// scripts/tests/test_calibration_domain.cpp links the REAL model.
//
// PROVENANCE OF EVERY CONCEPT BELOW
// ---------------------------------
//   09_Logs/Historical/NormaCore_MATDOG_Archive/LF_V25_Hardware_Oracle/
//     source/software/drivers/st3215/src/auto_calibrate/matdog.rs
//       enum Leg / JointKind / ContactSide / ContactState / LfSessionState
//       all_profiles()  -> 4 legs x 3 joints x 2 sides
//       build_profile() -> arm_value "{LEG}_{JOINT}_M{id}_{SIDE}"
//     source/tools/matdog/matdog_lf_profile.py
//       the 26-field per-joint record, LF_STAGED -> LF_FROZEN
//   06_Software/Matdog_Core/calibration/MATDOG_JOINT_CALIBRATION.yaml
//       the calibration_reset: block - the CURRENT authority
//
// Recovered as-is. Nothing here was invented to fill a gap; where the archive
// is silent, the gap is represented explicitly (see DirectionSource).

namespace matdog {
namespace calibration {

// ---------------------------------------------------------------------------
// The Cartesian contact-profile model
// ---------------------------------------------------------------------------

enum class Leg : uint8_t { LF = 0, RF = 1, RH = 2, LH = 3 };
enum class JointKind : uint8_t { HIP = 0, UPPER = 1, LOWER = 2 };

// NOTE: the obvious names MIN and MAX cannot be used - both are #defined by
// the ESP32 Arduino core. The same trap the Wi-Fi layer hit with DISABLED.
// The LABELS stay "MIN"/"MAX" because they are part of the oracle's profile
// key format and changing them would break the recovered token.
enum class ContactSide : uint8_t { MIN_SIDE = 0, MAX_SIDE = 1 };

constexpr uint8_t kLegCount = 4;
constexpr uint8_t kJointKindCount = 3;
constexpr uint8_t kContactSideCount = 2;

// DERIVED, never hard-coded. The archive's own test asserts 24
// (matdog_test.rs::profile_table_covers_exactly_24_unique_contacts), and this
// expression is what must produce it.
constexpr uint16_t kContactProfileCount =
    static_cast<uint16_t>(kLegCount) * kJointKindCount * kContactSideCount;

bool isKnownLeg(Leg leg);
bool isKnownJointKind(JointKind joint);
bool isKnownContactSide(ContactSide side);

// The identity of one mechanical contact to be calibrated.
//
// Deliberately keyed by (leg, joint, side) and NOT by motor/bus id. The 2026-08-27
// reassembly recoded bus ids and moved physical units between joints - unit M11
// is NECK_PITCH today - so any key carrying a historical id would silently
// associate the wrong joint. The historical id is carried separately, as
// provenance, by the oracle fixture.
struct ContactProfileKey {
  Leg leg = Leg::LF;
  JointKind joint = JointKind::HIP;
  ContactSide side = ContactSide::MIN_SIDE;

  bool valid() const {
    return isKnownLeg(leg) && isKnownJointKind(joint) && isKnownContactSide(side);
  }
  bool sameAs(const ContactProfileKey& other) const {
    return leg == other.leg && joint == other.joint && side == other.side;
  }
};

// Dense index in [0, kContactProfileCount). Returns kContactProfileCount for
// any invalid key - fail closed, never a wrapped or clamped index.
uint16_t contactProfileIndex(const ContactProfileKey& key);

// ---------------------------------------------------------------------------
// Joint identity — physical unit first, bus id nowhere
// ---------------------------------------------------------------------------

// A bus id is an ADDRESS, not an identity. The 2026-08-27 reassembly recoded
// ids and moved physical units between joints: today bus id 11 still means
// "LF lower", but the servo answering there is unit M33, while the unit
// labelled M11 - whose calibration the LF V25 archive records - is NECK_PITCH.
//
// So evidence is keyed by (leg, joint) plus the PHYSICAL UNIT LABEL from
// config/MATDOG_SERVO_ALLOCATION.yaml ("M33", "ELR01", ...). There is no bus
// id field anywhere in this model, which makes a bus-id-only match impossible
// to express rather than merely discouraged.
constexpr size_t kPhysicalUnitLabelBytes = 8;

struct JointIdentity {
  Leg leg = Leg::LF;
  JointKind joint = JointKind::HIP;
  char physical_unit[kPhysicalUnitLabelBytes] = {0};

  bool unitKnown() const { return physical_unit[0] != '\0'; }
  bool valid() const { return isKnownLeg(leg) && isKnownJointKind(joint); }
};

void setPhysicalUnit(JointIdentity* identity, const char* label);

// Same mechanical joint position on the robot.
bool sameJointSlot(const JointIdentity& a, const JointIdentity& b);
// Same physical servo. Two unknown units are NOT the same unit.
bool samePhysicalUnit(const JointIdentity& a, const JointIdentity& b);

// Required before any historical evidence may be applied to a current joint:
// BOTH the slot and the physical unit must agree. Slot agreement alone is what
// the reassembly made unsafe.
bool identityPermitsEvidenceReuse(const JointIdentity& historical,
                                  const JointIdentity& current);

// Inverse of contactProfileIndex(). Returns false for an out-of-range index.
bool contactProfileFromIndex(uint16_t index, ContactProfileKey* out);

// The oracle's profile token, e.g. "LF_HIP_M13_MAX". `motor_id` is HISTORICAL
// provenance from the LF V25 spec table; it is not a current bus id and must
// never be used to address hardware. Writes at most `out_size` bytes including
// the NUL and always terminates.
void formatProfileToken(const ContactProfileKey& key, uint8_t historical_motor_id,
                        char* out, size_t out_size);

// ---------------------------------------------------------------------------
// Contact detection - matdog.rs enum ContactState
// ---------------------------------------------------------------------------

enum class ContactState : uint8_t {
  FREE_MOTION       = 0,
  CONTACT_SUSPECTED = 1,
  CONTACT_CONFIRMED = 2,
  EARLY_STALL       = 3,
  HARD_ABORT        = 4,
};

// Only CONTACT_CONFIRMED is evidence of a mechanical endpoint. EARLY_STALL and
// HARD_ABORT are failures, and treating either as a contact is precisely the
// mistake the V24 run made before the witness gate was added.
bool isContactEvidence(ContactState state);
bool isContactFailure(ContactState state);

// ---------------------------------------------------------------------------
// Evidence lifecycle
// ---------------------------------------------------------------------------

// THREE vocabularies exist in this repository and they are NOT the same:
//
//   LF V25 artefacts        LF_STAGED -> LF_FROZEN, plus a per-record
//                           `accepted` boolean and a global PARTIAL -> ACTIVE.
//   geometry compiler V5    HARDWARE_VALIDATED / HARDWARE_CONTRADICTED /
//                           GEOMETRIC_ENDPOINT_CANDIDATE
//   DEVELOPMENT_GATES       MEASURED -> CANDIDATE -> ACCEPTED -> PROMOTED
//
// The four-stage lifecycle below is the CURRENT repository requirement (it is
// a stated PASS CRITERION of the calibration gate). It was NOT recovered from
// LF V25 - the archive has no PROMOTED stage. The mapping is recorded in
// docs rather than collapsed here.
enum class EvidenceState : uint8_t {
  UNKNOWN   = 0,  // nothing measured
  MEASURED  = 1,  // a raw observation exists
  CANDIDATE = 2,  // it survived its own consistency checks
  ACCEPTED  = 3,  // it passed the witness/affine gate  (LF V25: LF_STAGED)
  PROMOTED  = 4,  // it is operational calibration      (LF V25: LF_FROZEN)
  REJECTED  = 5,  // a gate refused it
};

// The legal forward transitions. Everything else is refused, including every
// shortcut that would let a replayed historical measurement become operational
// calibration in one step.
bool isLegalEvidenceTransition(EvidenceState from, EvidenceState to);

// PROMOTED is the only state that authorises operational use, and it is
// deliberately unreachable from a replay: see CalibrationOrigin.
bool isOperationalEvidence(EvidenceState state);

// Where a piece of evidence came from. Historical replay must never be able to
// reach PROMOTED - that is the safety property the whole lifecycle exists for.
enum class CalibrationOrigin : uint8_t {
  NONE               = 0,
  HISTORICAL_REPLAY  = 1,  // LF V25 oracle replayed offline. NEVER promotable.
  LIVE_SESSION       = 2,  // measured on the current installation
};

bool mayPromote(CalibrationOrigin origin);

// ---------------------------------------------------------------------------
// q0 - a MEASURED mechanical reference, never a constant
// ---------------------------------------------------------------------------

// The single most dangerous number in MATDOG calibration is 2048, because it
// is three different things that all happen to share a value:
//
//   kServoRawCenter        the ST3215 raw centre. A servo-level fact only.
//                          MATDOG_JOINT_CALIBRATION.yaml: "says nothing about
//                          joint zero, mounting or direction".
//   displayed-after-freeze what the servo reads back once PositionOffset has
//                          been written (LF V25 accepted 2048 +/- 10).
//   q0                     the measured mechanical zero. LF V25 measured
//                          2067 / 2040 / 2074 - NOT ONE OF THEM IS 2048.
//
// These are kept in separate types so the third can never silently inherit the
// first. The current repository rule is explicit:
//   "The final value MUST BE MEASURED, not assumed or asserted.
//    Do not impose q0_correction = 0."
constexpr uint16_t kServoRawCenter = 2048;

// LF V25 produced TWO independent zero estimates per joint and did not treat
// them as interchangeable.
enum class Q0Estimator : uint8_t {
  NONE          = 0,
  FIXED_SCALE   = 1,  // ModelZeroEstimate  - nominal tick scale, kept as diagnostic
  AFFINE        = 2,  // AffineJointCalibration - measured span; AUTHORITATIVE in V25
};

struct Q0Evidence {
  bool measured = false;              // false means "no measurement exists", not "2048"
  Q0Estimator estimator = Q0Estimator::NONE;
  EvidenceState state = EvidenceState::UNKNOWN;
  CalibrationOrigin origin = CalibrationOrigin::NONE;
  // Which joint AND which physical servo this was measured on. Without both,
  // a q0 cannot be applied to anything.
  JointIdentity identity{};
  uint16_t tick = 0;                  // meaningless unless measured == true
  uint16_t shift_from_digital_home_ticks = 0;
  uint16_t endpoint_disagreement_ticks = 0;
  uint16_t scale_permille = 0;
  bool accepted_by_gate = false;      // the V25 per-record `accepted` field

  // The whole point of this type. An unmeasured q0 has no value at all, and in
  // particular is NOT the raw servo centre.
  bool hasUsableValue() const { return measured && estimator != Q0Estimator::NONE; }
};

// ---------------------------------------------------------------------------
// Direction - recovered honestly, including what is NOT there
// ---------------------------------------------------------------------------

// AUDIT FINDING: there is no "direction witness" in the LF V25 archive.
// `direction` is a compile-time constant in JointSpec, used arithmetically as
//     tick = HOME_TICK + direction * q_delta
// and is never measured, cross-checked or validated against evidence.
//
// The witness that DOES exist is the CONTACT witness (see ContactWitness
// below), which compares measured contacts against a supervised hardware band.
//
// Representing a spec constant as if it were measured evidence would be a
// fabrication, so the provenance is part of the type. A direction may only be
// treated as calibration evidence when it is MEASURED_WITNESS.
enum class DirectionState : uint8_t {
  UNKNOWN              = 0,  // nothing establishes it
  SPECIFIED_HISTORICAL = 1,  // LF V25 JointSpec.direction - a static spec, NOT evidence
  MEASURED_CANDIDATE   = 2,  // TO_IMPLEMENT: no historical mechanism exists to recover
  ACCEPTED             = 3,  // TO_IMPLEMENT: requires current measured evidence
  CONFLICT             = 4,  // sources disagree; fail closed
};

struct DirectionEvidence {
  DirectionState state = DirectionState::UNKNOWN;
  int8_t sign = 0;  // -1 or +1 when known; 0 otherwise
  JointIdentity identity{};

  // Only an ACCEPTED direction, backed by current measured evidence, may drive
  // current calibration. SPECIFIED_HISTORICAL is usable by the historical
  // replay and by nothing else:
  //
  //   "LF V25 replay validates historical behaviour using historical direction
  //    specs. It does not establish current joint direction."
  bool isCurrentCalibrationEvidence() const {
    return state == DirectionState::ACCEPTED && (sign == -1 || sign == 1);
  }
  // What the historical replay is allowed to consume.
  bool isHistoricalSpecification() const {
    return state == DirectionState::SPECIFIED_HISTORICAL && (sign == -1 || sign == 1);
  }
};

// ---------------------------------------------------------------------------
// Contact witness - the real V25 gate
// ---------------------------------------------------------------------------

// A contact is accepted only if BOTH endpoint deviations stay inside the
// supervised band. This is what rejected the cable-obstructed M12 MAX (~3397)
// and accepted the unobstructed result (~3443).
//
// THE TOLERANCE IS DELIBERATELY NOT A CONSTANT HERE. LF V25's
// LF_CONTACT_WITNESS_TOLERANCE_TICKS = 24 was measured on ONE leg, on a
// physical installation that no longer exists. Promoting it to a universal
// parameter for all four legs would be exactly the kind of unearned
// generalisation the archive warns against ("never mirror LF evidence onto
// RF, RH or LH"). It lives in the historical fixture; every witness must be
// given its own band, and a band of zero tolerance is never assumed valid.
struct ContactWitness {
  bool evaluated = false;
  bool tolerance_set = false;
  uint16_t min_deviation_ticks = 0;
  uint16_t max_deviation_ticks = 0;
  uint16_t tolerance_ticks = 0;

  bool accepted() const {
    return evaluated && tolerance_set && min_deviation_ticks <= tolerance_ticks &&
           max_deviation_ticks <= tolerance_ticks;
  }
};

// Builds a witness with an explicit band. There is no overload that defaults
// the tolerance.
ContactWitness makeContactWitness(uint16_t min_deviation_ticks, uint16_t max_deviation_ticks,
                                  uint16_t tolerance_ticks);

// ---------------------------------------------------------------------------
// Per-contact evidence record
// ---------------------------------------------------------------------------

struct ContactEvidence {
  ContactProfileKey key{};
  EvidenceState state = EvidenceState::UNKNOWN;
  CalibrationOrigin origin = CalibrationOrigin::NONE;
  ContactState detection = ContactState::FREE_MOTION;
  ContactWitness witness{};

  uint16_t coarse_tick = 0;
  uint16_t fine_tick_1 = 0;
  uint16_t fine_tick_2 = 0;
  uint16_t repeatability_ticks = 0;
  bool has_measurement = false;
};

// A q0 may only be applied to a current joint when the identity matches on
// BOTH axes and the evidence came from a live session. Three shortcuts are
// therefore impossible to express, not merely discouraged:
//
//   raw_center   -> q0     Q0Evidence starts empty, not centred
//   historical q0 -> current q0   origin HISTORICAL_REPLAY can never promote
//   replay q0     -> promoted q0  mayPromote() refuses that origin
bool q0MayBeAppliedTo(const Q0Evidence& evidence, const JointIdentity& current_joint);

// ---------------------------------------------------------------------------
// Leg population gate  (the concept historical docs call "Full-Leg H1")
// ---------------------------------------------------------------------------

// NAMING, deliberately: this is NOT called H1 in new code. The repository uses
// "H1" for two unrelated things - the Full-Leg calibration population gate,
// and the Controller's own boot hardware test H1 in VALIDATION.md. Historical
// documents keep their wording; new code uses an unambiguous name.
//
// This gate EVALUATES evidence. It does not scan: MATDOG already has exactly
// one bus discovery path, servo/ServoCensus, and a second census is forbidden.
// The observed population arrives here as an input.
//
// 12 leg servos: 4 legs x 3 joints. ID 51 (NECK_ROTATION) is explicitly NOT
// part of the Full-Leg population, and neither are the other head/jaw units.
constexpr uint8_t kLegServoSlotCount = kLegCount * kJointKindCount;

enum class PopulationVerdict : uint8_t {
  NOT_EVALUATED = 0,
  PASS          = 1,  // every expected leg slot observed
  FAIL          = 2,  // at least one missing
  INVALID       = 3,  // the evidence itself is malformed; fail closed
};

struct LegPopulationEvidence {
  bool evaluated = false;
  CalibrationOrigin origin = CalibrationOrigin::NONE;
  // Bit per leg slot, index = leg * kJointKindCount + joint. Using a mask
  // rather than a count is what makes "6 of 12" say WHICH six.
  uint16_t observed_mask = 0;
  uint16_t unexpected_count = 0;   // responders outside the 12 leg slots
  uint32_t session_ms = 0;         // provenance
};

uint8_t legSlotIndex(Leg leg, JointKind joint);
bool legSlotFromIndex(uint8_t index, Leg* out_leg, JointKind* out_joint);
uint8_t observedLegSlotCount(const LegPopulationEvidence& evidence);
PopulationVerdict evaluateLegPopulation(const LegPopulationEvidence& evidence);

// The last formal Full-Leg result on record is 6/12, and it is HISTORICAL.
// A Controller census finding servos on the bus proves bus visibility, not
// calibration population - the repository states this explicitly. Historical
// evidence can never produce a current PASS.
bool populationIsCurrentPass(const LegPopulationEvidence& evidence);

// ---------------------------------------------------------------------------
// Execution phases - matdog.rs enum LfSessionState, recovered verbatim
// ---------------------------------------------------------------------------
//
// LAYERING: these are the phases of the LF V25 HARDWARE EXECUTION sequence.
// They belong to a future calibration execution engine, not to the session
// lifecycle. CalibrationManager owns session lifecycle and does not drive
// these; it only records which one a future execution layer last reported, so
// that restore intent can be expressed. The offline oracle replay walks them
// directly, which is the only thing that does today.

enum class CalibrationPhase : uint8_t {
  PREFLIGHT         = 0,
  INITIAL_RECOVERY  = 1,
  PARKING           = 2,
  UPPER_MIN         = 3,
  UPPER_MAX         = 4,
  UPPER_HORIZONTAL  = 5,
  LOWER_MIN         = 6,
  LOWER_MAX         = 7,
  LOWER_FOLDED      = 8,
  HIP_MIN           = 9,
  HIP_MAX           = 10,
  DIAGNOSTICS       = 11,
  RETURN_HIP        = 12,
  RETURN_LOWER_HELD = 13,
  RETURN_UPPER      = 14,
  RESTORE_PARKING   = 15,
  CLEANUP           = 16,
  TORQUE_OFF        = 17,
};

constexpr uint8_t kCalibrationPhaseCount = 18;

// The oracle's order is strict and load-bearing: UPPER and LOWER are proven
// before HIP is attempted at all. matdog.rs::hardware_profile_allowed() refuses
// every isolated HIP profile for exactly this reason.
bool isLegalPhaseTransition(CalibrationPhase from, CalibrationPhase to);
bool isMeasurementPhase(CalibrationPhase phase);

// RESTORE is not SAFE_OFF and not ABORT. The oracle separates them:
//   restore   RETURN_HIP -> RETURN_LOWER_HELD -> RETURN_UPPER -> RESTORE_PARKING
//             an ordered return of the leg to a parked pose
//   cleanup   CLEANUP     release of session resources
//   torque off TORQUE_OFF the terminal safety state, verified on EVERY exit
bool isRestorePhase(CalibrationPhase phase);
bool isTerminalPhase(CalibrationPhase phase);

// ---------------------------------------------------------------------------
// Failure taxonomy
// ---------------------------------------------------------------------------

enum class CalibrationFailure : uint8_t {
  NONE                       = 0,
  HARD_CURRENT_ABORT         = 1,  // HARD_CURRENT_ABORT_RAW = 200
  EARLY_STALL                = 2,
  CONTACT_WITNESS_REJECTED   = 3,  // the uniform supervised band refused it
  REPEATABILITY_EXCEEDED     = 4,  // REPEATABILITY_TOLERANCE_TICKS = 16
  MOTION_TIMEOUT             = 5,  // MOTION_TIMEOUT = 12 s
  TELEMETRY_STALE            = 6,  // MAX_TELEMETRY_AGE = 3 s
  STATIC_JOINT_MOVED         = 7,  // a held/torque-off joint drifted
  AFFINE_GATE_REJECTED       = 8,
  ISOLATED_HIP_BLOCKED       = 9,  // hardware_profile_allowed()
  OPERATOR_ABORT             = 10,
  AUTHORITY_LOST             = 11, // ActuatorAuthority no longer held
  STALE_CALIBRATION_REFUSED  = 12, // CALIBRATION_RESET_PENDING_FULL_RECALIBRATION
};

// Every failure lands in the same place. The oracle's rule was
// "global torque OFF must be verified on success and every failure path".
bool requiresRestore(CalibrationFailure failure);

// ---------------------------------------------------------------------------
// Restore intent - state only. No motion is produced by this model.
// ---------------------------------------------------------------------------

struct RestorePlan {
  bool required = false;
  CalibrationPhase resume_from = CalibrationPhase::TORQUE_OFF;
  CalibrationFailure cause = CalibrationFailure::NONE;
  // Recovered from the oracle: torque off is verified on every exit, success
  // or failure. It is an assertion about the final state, not an action this model
  // performs.
  bool torque_off_required = true;
};

RestorePlan restorePlanFor(CalibrationPhase phase, CalibrationFailure failure);

const char* toString(Leg leg);
const char* toString(JointKind joint);
const char* toString(ContactSide side);
const char* toString(ContactState state);
const char* toString(EvidenceState state);
const char* toString(CalibrationOrigin origin);
const char* toString(CalibrationPhase phase);
const char* toString(CalibrationFailure failure);
const char* toString(DirectionState state);
const char* toString(PopulationVerdict verdict);
const char* toString(Q0Estimator estimator);

}  // namespace calibration
}  // namespace matdog

#endif  // MATDOG_CALIBRATION_CALIBRATION_DOMAIN_H
