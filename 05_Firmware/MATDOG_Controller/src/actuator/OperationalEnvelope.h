#ifndef MATDOG_ACTUATOR_OPERATIONAL_ENVELOPE_H
#define MATDOG_ACTUATOR_OPERATIONAL_ENVELOPE_H

#include <stdint.h>

#include "../calibration/CalibrationDomain.h"
#include "CalibrationGeometryProfile.h"

// CR3 Priority 6 — the Full Calibration operational-envelope builder for the
// twelve leg joints, prepared and offline-tested but producing no envelope
// from real hardware evidence by this session (none exists yet).
//
// Pure: <stdint.h> plus already-pure MATDOG units. No Arduino, no ServoBus,
// no floating point (every bound stays an integer micro-radian or raw tick,
// the same discipline CalibrationGeometryProfile.h already enforces).
//
// THIS IS NOT ActuatorLimitTable/JointLimit
// -------------------------------------------
// ActuatorWritePolicy.h's JointLimit is the live, runtime POSITION_COMMAND
// gate — a different concept with a different consumer. Conflating "the Full
// Calibration completeness record for one joint's safe range" with "the
// value a future motion controller's write is checked against right now" is
// exactly the kind of collapsing this codebase's architecture repeatedly
// warns against (see JointLimit/JointTransform's own file comments on
// keeping physical-unit identity, calibration provenance and geometry
// provenance on separate axes). OperationalEnvelope below is deliberately a
// SEPARATE type; nothing here admits into ActuatorLimitTable, and nothing
// here is consulted by SafeActuatorPolicy.
//
// TWO SOURCES, NEVER CONFLATED
// -------------------------------------------------------------------------
//   UPPER (8 EXECUTABLE_URDF_DOMAIN endpoints)
//     buildContactDerivedEnvelope() — requires REAL, CURRENT, CONFIRMED,
//     repeatability-witnessed ContactEvidence on BOTH sides. Fails closed
//     (present=false) on anything less: missing evidence, wrong slot,
//     historical/replay origin, unconfirmed detection, a rejected witness.
//     THIS SESSION NEVER CALLS IT WITH REAL EVIDENCE — no physical contact
//     probing has happened; the eight physical UPPER results are never
//     fabricated here or anywhere else in this offline session.
//
//   HIP / LOWER (all 16 endpoints DIAGNOSTIC_GEOMETRY_OUTSIDE_URDF_LIMITS —
//   there is no executable contact to measure inside the URDF at all)
//     buildGeometryDerivedEnvelope() — derives a bound strictly INSIDE the
//     joint's own URDF domain from a caller-supplied required stand/gait
//     workspace and a caller-supplied safety margin. Neither the workspace
//     range nor the margin is invented here — see each request struct's own
//     comment. Never touches, and structurally cannot reach, a diagnostic
//     (beyond-URDF) endpoint: this file has no path to endpoint records at
//     all, only to GeometryJointRecord's own urdf_lower/urdf_upper.
//
// GEOMETRY PROVENANCE, AND THE ONE GAP THIS FILE DOES NOT PAPER OVER
// -------------------------------------------------------------------------
// calibration::ContactEvidence (recovered from the LF V25 oracle) carries no
// GeometryProvenanceTag field — unlike JointTransform/JointLimit, which are
// CR3-era types built with that axis from the start. Rather than widen a
// historical "recovered as-is" type to paper over that, the geometry tag
// each side's evidence was captured under is a required, explicit field on
// ContactDerivedEnvelopeRequest below: the caller (whatever future session
// orchestration records contact evidence) must carry that fact itself until
// ContactEvidence is reviewed and extended. See the CR3 development log.

namespace matdog {
namespace actuator {

enum class EnvelopeSource : uint8_t {
  NONE                   = 0,
  MEASURED_CONTACT       = 1,  // UPPER: from confirmed, repeatable contact evidence
  DERIVED_FROM_GEOMETRY  = 2,  // HIP/LOWER: from URDF + required workspace + margin
};

enum class EnvelopeBuildStatus : uint8_t {
  NOT_EVALUATED                  = 0,
  READY                          = 1,
  REJECT_NO_GEOMETRY             = 2,
  REJECT_UNKNOWN_JOINT           = 3,
  REJECT_NO_TRANSFORM            = 4,  // geometry path: no usable promoted q0
  REJECT_WORKSPACE_MALFORMED     = 5,  // geometry path: required_min > required_max
  REJECT_WORKSPACE_OUTSIDE_URDF  = 6,  // geometry path: required range exceeds urdf_lower/upper
  REJECT_MARGIN_INVALID          = 7,  // negative margin supplied
  REJECT_MARGIN_COLLAPSES_RANGE  = 8,  // margin leaves min > max
  REJECT_TARGET_RESOLUTION       = 9,  // the checked resolver refused a margined bound
  REJECT_MISSING_CONTACT_EVIDENCE     = 10,  // contact path: one or both sides absent
  REJECT_CONTACT_WRONG_SLOT           = 11,  // contact path: leg/joint slot does not match
  REJECT_CONTACT_NOT_CURRENT           = 12,  // contact path: origin/state not live+promoted
  REJECT_CONTACT_NOT_CONFIRMED         = 13,  // contact path: detection != CONTACT_CONFIRMED
  REJECT_CONTACT_WITNESS_REJECTED      = 14,  // contact path: witness.accepted() == false
  REJECT_CONTACT_GEOMETRY_MISMATCH     = 15,  // contact path: evidence geometry tag != current
  REJECT_CONTACT_ORDER_INVALID         = 16,  // contact path: margin leaves min > max
};

struct OperationalEnvelope {
  calibration::JointIdentity identity{};
  EnvelopeSource source = EnvelopeSource::NONE;
  GeometryProvenanceTag geometry = kNoGeometryProvenance;
  uint16_t min_tick = 0;
  uint16_t max_tick = 0;
  bool present = false;  // false means "no envelope exists", never "unbounded"

  bool ordered() const { return min_tick <= max_tick; }
};

// --- HIP / LOWER: derived from URDF + workspace + margin -------------------

struct GeometryDerivedEnvelopeRequest {
  calibration::JointIdentity joint{};
  // The workspace the caller (a reviewed stand/gait requirement, not this
  // file) actually needs. Must already lie inside the joint's own URDF
  // domain — this function verifies that, it does not clip to it.
  MicroRad required_min_urad = 0;
  MicroRad required_max_urad = 0;
  // Symmetric inset applied to BOTH ends of the required workspace, in
  // micro-radians. A reviewed safety-margin choice; never defaulted or
  // invented here, and never negative (an expanding "margin" is a
  // contradiction in terms).
  MicroRad safety_margin_urad = 0;
};

EnvelopeBuildStatus buildGeometryDerivedEnvelope(
    const CalibrationGeometryProfile& profile, const GeometryProvenance& expected_provenance,
    const JointTransform& transform, const GeometryDerivedEnvelopeRequest& request,
    OperationalEnvelope* out);

// --- UPPER: derived from confirmed, repeatable contact evidence ------------

struct ContactDerivedEnvelopeRequest {
  calibration::JointIdentity joint{};
  calibration::ContactEvidence min_side_evidence{};
  calibration::ContactEvidence max_side_evidence{};
  // Which geometry model each side's evidence was captured under — see the
  // file comment on why this is a required caller-supplied field rather
  // than a ContactEvidence member.
  GeometryProvenanceTag min_side_geometry = kNoGeometryProvenance;
  GeometryProvenanceTag max_side_geometry = kNoGeometryProvenance;
  // Symmetric inset applied to both confirmed contact ticks, in raw ticks —
  // a reviewed safety-margin choice, never defaulted or invented here.
  uint16_t safety_margin_ticks = 0;
};

EnvelopeBuildStatus buildContactDerivedEnvelope(
    const CalibrationGeometryProfile& profile, const GeometryProvenance& expected_provenance,
    const ContactDerivedEnvelopeRequest& request, OperationalEnvelope* out);

const char* toString(EnvelopeSource source);
const char* toString(EnvelopeBuildStatus status);

}  // namespace actuator
}  // namespace matdog

#endif  // MATDOG_ACTUATOR_OPERATIONAL_ENVELOPE_H
