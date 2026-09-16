#ifndef MATDOG_SERVO_SERVO_POPULATION_H
#define MATDOG_SERVO_SERVO_POPULATION_H

#include <stdint.h>

// MATDOG servo population model — G2 (ROBOT_POWERED configuration support).
//
// PURPOSE
// -------
// Answers the question the low-level bus deliberately does NOT answer.
// ServoBus reports a physical fact: "IDs {…} responded to a ping over the
// range [lo, hi]". This module answers what that observation MEANS for the
// MATDOG robot as currently built. The two concerns stay separate on
// purpose (handoff section 17): ServoBus owns the transport, this owns the
// configuration policy.
//
// THREE DISTINCT POPULATIONS — never collapse them
// ------------------------------------------------
//   CANONICAL_ALLOCATED (17)
//       Every bus ID the MATDOG design allocates. Fixed by the project's
//       servo allocation, independent of what is screwed onto the robot
//       today. Provenance: 06_Software/Matdog_Core/config/
//       MATDOG_SERVO_ALLOCATION.yaml (17 units, profile MATDOG_C018_V1,
//       allocated 2026-08-27).
//
//   EXPECTED_IN_CURRENT_CONFIGURATION (13)
//       The subset physically installed in the robot right now. The five
//       head/neck units were allocated and bench-provisioned, but only
//       51 NECK_ROTATION is mounted; 52-55 are not yet installed.
//
//   OBSERVED (live)
//       Whatever actually answered the last census scan.
//
// The critical invariant the handoff calls out explicitly: a powered census
// that finds 13 of 17 is HEALTHY. "17 must respond for PASS" is false for
// the current robot and must never be hardcoded anywhere.
//
// PROVENANCE OF THE EMBEDDED TABLE
// --------------------------------
// MATDOG_SERVO_ALLOCATION.yaml remains the canonical PROJECT authority: it
// carries the physical unit identity, provisioning session, cold-verify
// evidence and recode history for each servo. None of that belongs in
// firmware. The table below is a deliberately minimal embedded projection
// of that file — bus ID, joint name, and whether the unit is installed
// today — because an ESP32-S3 cannot parse YAML at boot and the census must
// work with no host attached. It is NOT a competing source of truth: if the
// two ever disagree, the YAML wins and this table is the thing to fix.
// scripts/static_audit.py cross-checks the counts so the drift cannot pass
// silently.
//
// TRANSPORT INDEPENDENCE (handoff sections 7/8/9)
// -----------------------------------------------
// Everything here is pure data + pure functions: <stdint.h> only, no
// Arduino, no Serial, no ServoBus, no hardware transaction. The census
// result is a fixed-size POD struct, so a future Controller telemetry
// snapshot (and through it the Web UI / HostLink) can copy the SAME
// classification the USB CDC adapter prints, without re-scanning the bus.
// CommandRouter formats this; it does not compute it.

namespace matdog {
namespace servo {

// Whether a canonically allocated servo is physically installed in the
// robot's CURRENT configuration. This is orthogonal to the power profile
// (config/HardwareProfile.h): a USB_ONLY bench and a fully powered robot
// can have the same installed servo set.
enum class CurrentConfig : uint8_t {
  INSTALLED        = 0,  // expected to answer on a powered bus
  ABSENT_BY_DESIGN = 1,  // allocated, provisioned, not yet mounted — NOT a failure
};

struct CanonicalServo {
  uint8_t bus_id;
  const char* joint;
  CurrentConfig current_config;
};

// Canonical MATDOG allocation — all 17 units.
// Mirrors MATDOG_SERVO_ALLOCATION.yaml (see provenance note above).
constexpr CanonicalServo kCanonicalServos[] = {
    // Left front
    {11, "LF_LOWER", CurrentConfig::INSTALLED},
    {12, "LF_UPPER", CurrentConfig::INSTALLED},
    {13, "LF_HIP",   CurrentConfig::INSTALLED},
    // Right front
    {21, "RF_LOWER", CurrentConfig::INSTALLED},
    {22, "RF_UPPER", CurrentConfig::INSTALLED},
    {23, "RF_HIP",   CurrentConfig::INSTALLED},
    // Right hind
    {31, "RH_LOWER", CurrentConfig::INSTALLED},
    {32, "RH_UPPER", CurrentConfig::INSTALLED},
    {33, "RH_HIP",   CurrentConfig::INSTALLED},
    // Left hind
    {41, "LH_LOWER", CurrentConfig::INSTALLED},
    {42, "LH_UPPER", CurrentConfig::INSTALLED},
    {43, "LH_HIP",   CurrentConfig::INSTALLED},
    // Head / neck — allocated and bench-provisioned, only 51 is mounted.
    {51, "NECK_ROTATION", CurrentConfig::INSTALLED},
    {52, "NECK_PITCH",    CurrentConfig::ABSENT_BY_DESIGN},
    {53, "HEAD_ROTATION", CurrentConfig::ABSENT_BY_DESIGN},
    {54, "HEAD_PITCH",    CurrentConfig::ABSENT_BY_DESIGN},
    {55, "JAW",           CurrentConfig::ABSENT_BY_DESIGN},
};

constexpr uint8_t kCanonicalServoCount =
    static_cast<uint8_t>(sizeof(kCanonicalServos) / sizeof(kCanonicalServos[0]));

// A census must probe at least this span to be able to conclude anything:
// it covers every canonical ID (11..55 = 45 IDs, within ServoBus's bounded
// scan range). A narrower scan cannot distinguish "absent" from
// "never asked" — see CensusVerdict::RANGE_INCOMPLETE.
constexpr int kCanonicalScanLo = 11;
constexpr int kCanonicalScanHi = 55;

// Upper bound on IDs recorded per category. Sized so a full canonical
// population plus a plausible number of stray responders fits; overflow is
// reported rather than silently truncated (see CensusResult::truncated).
constexpr uint8_t kMaxReportedIds = 24;

// Per-ID meaning of the live observation. The three "not a failure" and
// three "is a problem" cases are deliberately distinct values: collapsing
// ABSENT_BY_DESIGN into MISSING_EXPECTED is precisely the bug this gate
// exists to prevent.
enum class IdClassification : uint8_t {
  PRESENT_EXPECTED        = 0,  // installed-now servo answered            -> healthy
  ABSENT_BY_DESIGN        = 1,  // not-installed servo stayed silent       -> healthy
  MISSING_EXPECTED        = 2,  // installed-now servo did NOT answer      -> problem
  ABSENT_BY_DESIGN_PRESENT = 3, // not-installed servo ANSWERED            -> problem, surfaced
  UNEXPECTED_ID           = 4,  // responder outside the canonical table   -> problem
  NOT_PROBED              = 5,  // canonical ID outside the scanned range  -> inconclusive
};

const char* toString(IdClassification c);

// Overall verdict for one census.
enum class CensusVerdict : uint8_t {
  // Every installed-now servo answered, every absent-by-design servo stayed
  // silent, no unexpected responders, and the scan covered every canonical
  // ID. For the current robot this is 13/13 present + 4 absent by design.
  PASS = 0,

  // At least one MISSING_EXPECTED, ABSENT_BY_DESIGN_PRESENT or
  // UNEXPECTED_ID. The live population does not match the declared current
  // configuration.
  PROFILE_MISMATCH = 1,

  // The scan did not cover every canonical ID, or the responder list
  // overflowed. Fail-closed: no conclusion is possible, so this is never
  // reported as PASS even when nothing anomalous was seen in the part that
  // WAS probed.
  RANGE_INCOMPLETE = 2,

  // No census has been run yet.
  NOT_RUN = 3,
};

const char* toString(CensusVerdict v);

// Structured, fixed-size, copyable census result. No pointers into scan
// buffers, no dynamic allocation, no Serial: a future telemetry snapshot
// stores this whole struct by value.
struct CensusResult {
  // Population counts. canonical_allocated and expected_now are compile-time
  // facts, carried in the result so a consumer never has to re-derive them
  // (and so "17" and "13" can never be re-hardcoded at a call site).
  uint8_t canonical_allocated = kCanonicalServoCount;  // 17
  uint8_t expected_now = 0;                            // 13
  uint8_t present_expected = 0;
  uint8_t missing_expected = 0;
  uint8_t absent_by_design = 0;           // correctly silent
  uint8_t absent_by_design_present = 0;   // silent-expected servo that answered
  uint8_t unexpected_id = 0;
  uint8_t not_probed = 0;

  uint8_t missing_ids[kMaxReportedIds] = {0};
  uint8_t missing_id_count = 0;
  uint8_t unexpected_ids[kMaxReportedIds] = {0};
  uint8_t unexpected_id_count = 0;
  uint8_t absent_by_design_present_ids[kMaxReportedIds] = {0};
  uint8_t absent_by_design_present_id_count = 0;

  // Range actually probed, recorded as evidence.
  int scan_lo = 0;
  int scan_hi = 0;

  // The observed-ID list could not be fully represented (ServoBus's own
  // found_ids cap, or kMaxReportedIds). Forces a non-PASS verdict.
  bool truncated = false;

  CensusVerdict verdict = CensusVerdict::NOT_RUN;
};

// Compile-time-ish population accessors. Deliberately functions rather than
// two more constants, so the counts are always recomputed from the single
// table above and cannot drift from it.
uint8_t canonicalAllocatedCount();      // 17
uint8_t expectedNowCount();             // 13
uint8_t absentByDesignCount();          // 4

// Returns the canonical entry for a bus ID, or nullptr if the ID is not
// allocated by the MATDOG design.
const CanonicalServo* findCanonical(uint8_t bus_id);

// Classifies ONE canonical ID given the live observation. Exposed
// separately from classifyObserved() so per-joint UI/telemetry can ask
// about a single joint without re-running a census.
IdClassification classifyId(uint8_t bus_id, bool responded, int scan_lo, int scan_hi);

// THE census classification entry point. Pure: no hardware, no I/O, no
// global state. `observed_ids` is the responder list from a ServoBus scan
// over [scan_lo, scan_hi]; `observed_count` may exceed `observed_len` when
// the underlying scan buffer overflowed, which is reported as truncated.
CensusResult classifyObserved(const int* observed_ids,
                              int observed_len,
                              int observed_count,
                              int scan_lo,
                              int scan_hi);

}  // namespace servo
}  // namespace matdog

#endif  // MATDOG_SERVO_SERVO_POPULATION_H
