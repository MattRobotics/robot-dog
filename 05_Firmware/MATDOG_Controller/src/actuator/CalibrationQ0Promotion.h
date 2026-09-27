#ifndef MATDOG_ACTUATOR_CALIBRATION_Q0_PROMOTION_H
#define MATDOG_ACTUATOR_CALIBRATION_Q0_PROMOTION_H

#include <stdint.h>

#include "CalibrationQ0Bootstrap.h"

namespace matdog {
namespace actuator {

// CR3-M1: acceptance/promotion policy for a CR2 current-installation q0
// candidate. Pure and transport-free: no Arduino, ServoBus, authority,
// EEPROM or persistence I/O.
//
// The plausibility budget is derived independently from the mechanical
// spline contract, never fitted to CR2-C's observed max offset:
//
//   ST3215 revolution = 4096 ticks
//   output spline     = 25 teeth
//   half tooth        = 4096 / (2*25) = 81.92 ticks
//
// The existing reviewed CR2 stability quantum is 16 ticks. The largest whole
// multiple of that quantum strictly below half a tooth is therefore 80 ticks.
// This intentionally leaves CR2-A free to record stable far-away candidates;
// only this CR3 acceptance gate applies the mechanical plausibility rule.
constexpr uint16_t kOutputSplineTeeth = 25;
constexpr uint16_t kQ0AcceptanceMinSamples = 9;
constexpr uint16_t kQ0AcceptanceMaxSpreadTicks = 16;
constexpr uint16_t kQ0PlausibilityTicks =
    static_cast<uint16_t>(
        ((kTicksPerRevolution / (2u * kOutputSplineTeeth)) /
         kQ0AcceptanceMaxSpreadTicks) *
        kQ0AcceptanceMaxSpreadTicks);

static_assert(kQ0PlausibilityTicks == 80, "CR3 q0 plausibility derivation drifted");
static_assert(static_cast<uint32_t>(kQ0PlausibilityTicks) * 2u * kOutputSplineTeeth <
                  static_cast<uint32_t>(kTicksPerRevolution),
              "q0 plausibility must remain strictly below half a spline tooth");

enum class Q0AcceptanceStatus : uint8_t {
  NOT_EVALUATED = 0,
  ACCEPTED = 1,
  REJECT_NOT_CANDIDATE = 2,
  REJECT_GEOMETRY = 3,
  REJECT_IDENTITY = 4,
  REJECT_BUS_BINDING = 5,
  REJECT_SESSION = 6,
  REJECT_SAMPLE_POLICY = 7,
  REJECT_STABILITY = 8,
  REJECT_RAW_DOMAIN = 9,
  REJECT_PLAUSIBILITY = 10,
  REJECT_MALFORMED_DIAGNOSTIC = 11,
};

struct AcceptedQ0 {
  Q0AcceptanceStatus status = Q0AcceptanceStatus::NOT_EVALUATED;
  calibration::Q0Evidence evidence{};
  GeometryProvenanceTag geometry = kNoGeometryProvenance;
  uint8_t bus_id = 0;
  uint32_t capture_session_id = 0;
  uint8_t sample_count = 0;
  uint16_t stability_spread_ticks = 0;

  bool accepted() const {
    return status == Q0AcceptanceStatus::ACCEPTED &&
           evidence.state == calibration::EvidenceState::ACCEPTED &&
           evidence.accepted_by_gate;
  }
};

// CANDIDATE -> ACCEPTED. No persistence and no promotion occur here.
AcceptedQ0 acceptQ0Candidate(const CalibrationGeometryProfile& profile,
                             const GeometryProvenance& expected_provenance,
                             const Q0BootstrapCandidate& candidate);

// Promotion is deliberately a separate, explicit currentness transaction.
// It is RAM-only at this stage and therefore disappears on reboot. This is
// exactly the CR3 rule: until a later persistence gate exists, reboot fails
// closed rather than reconstructing operational evidence from a guess.
struct Q0PromotionRequest {
  bool explicit_currentness_confirmation = false;
  uint32_t capture_session_id = 0;
};

enum class Q0PromotionStatus : uint8_t {
  NOT_EVALUATED = 0,
  PROMOTED = 1,
  REJECT_NOT_ACCEPTED = 2,
  REJECT_GEOMETRY = 3,
  REJECT_CURRENTNESS_NOT_CONFIRMED = 4,
  REJECT_SESSION_MISMATCH = 5,
};

struct PromotedQ0 {
  Q0PromotionStatus status = Q0PromotionStatus::NOT_EVALUATED;
  JointTransform transform{};

  bool promoted() const {
    return status == Q0PromotionStatus::PROMOTED &&
           transform.state == calibration::EvidenceState::PROMOTED &&
           transform.usableProvenance();
  }
};

// ACCEPTED -> PROMOTED. This produces a JointTransform eligible for admission
// to JointTransformTable. It does not itself admit it and never persists it.
PromotedQ0 promoteAcceptedQ0(const CalibrationGeometryProfile& profile,
                             const GeometryProvenance& expected_provenance,
                             const AcceptedQ0& accepted,
                             const Q0PromotionRequest& request);

const char* toString(Q0AcceptanceStatus status);
const char* toString(Q0PromotionStatus status);

}  // namespace actuator
}  // namespace matdog

#endif  // MATDOG_ACTUATOR_CALIBRATION_Q0_PROMOTION_H
