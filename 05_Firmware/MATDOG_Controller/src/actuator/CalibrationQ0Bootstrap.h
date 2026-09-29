#ifndef MATDOG_ACTUATOR_CALIBRATION_Q0_BOOTSTRAP_H
#define MATDOG_ACTUATOR_CALIBRATION_Q0_BOOTSTRAP_H

#include <stdint.h>

#include "CalibrationGeometryProfile.h"

// CR2 - read-only q0 bootstrap evidence.
//
// This module is intentionally pure. It receives observations; it never reads
// ServoBus, owns no UART, sends no command and writes nothing. A later
// Controller/session orchestration layer may collect the observations after an
// operator has manually aligned the mechanism to nominal URDF q=0.
//
// The output is CANDIDATE evidence only. It is never PROMOTED here and is never
// inserted into JointTransformTable. CR3 owns acceptance/promotion/persistence.

namespace matdog {
namespace actuator {

constexpr uint8_t kQ0BootstrapMinSamples = 3;
constexpr uint8_t kQ0BootstrapMaxSamples = 32;

struct Q0CaptureSample {
  bool read_ok = false;
  int32_t raw_tick = -1;       // valid domain: unsigned ST3215 0..4095
  int32_t torque_enable = -1;  // must be exactly 0 for every sample
};

struct Q0BootstrapRequest {
  calibration::JointIdentity identity{};
  uint8_t bus_id = 0;  // transport metadata only; never calibration identity

  // Must identify one current capture session. 0 means not assigned and is
  // refused so evidence from different operator placements cannot be merged.
  uint32_t capture_session_id = 0;

  // Explicit operator attestation. CR2 never infers pose correctness from raw
  // distance to 2048: the bootstrap contract requires manual URDF q=0.
  bool nominal_zero_pose_confirmed = false;

  // Stability only, NOT the final q0 plausibility/acceptance window. A zero
  // tick budget is a valid deliberately-strict choice, so a separate presence
  // bit makes "operator/orchestrator supplied 0" distinguishable from
  // "caller forgot to supply a budget".
  bool stability_budget_specified = false;
  uint16_t max_stability_spread_ticks = 0;
};

enum class Q0BootstrapStatus : uint8_t {
  NOT_EVALUATED                  = 0,
  REJECT_POPULATION_NOT_CURRENT  = 1,
  REJECT_GEOMETRY_UNBOUND        = 2,
  REJECT_GEOMETRY_PROVENANCE     = 3,
  REJECT_INVALID_REQUEST         = 4,
  REJECT_JOINT_NOT_IN_PROFILE    = 5,
  REJECT_BUS_ID_MISMATCH         = 6,
  REJECT_POSE_NOT_CONFIRMED      = 7,
  REJECT_SAMPLE_COUNT            = 8,
  REJECT_SAMPLE_READ             = 9,
  REJECT_TORQUE_NOT_OFF          = 10,
  REJECT_RAW_DOMAIN              = 11,
  REJECT_UNSTABLE                = 12,
  CANDIDATE                      = 13,
};

struct Q0BootstrapCandidate {
  Q0BootstrapStatus status = Q0BootstrapStatus::NOT_EVALUATED;

  // Current calibration evidence. On success:
  // measured=true, estimator=MANUAL_ZERO_POSE, state=CANDIDATE,
  // origin=LIVE_SESSION, accepted_by_gate=false.
  calibration::Q0Evidence evidence{};

  // Which model and which acquisition session this candidate belongs to.
  GeometryProvenanceTag geometry = kNoGeometryProvenance;
  uint8_t bus_id = 0;  // transport metadata only
  uint32_t capture_session_id = 0;

  uint8_t sample_count = 0;
  uint16_t stability_spread_ticks = 0;
};

// A finished current-boot CR2-B acquisition seen as plain data, so the CR3
// promotion path can consume it without depending on the capture session.
struct FreshQ0Capture {
  bool complete = false;          // COMPLETE with no failure and every candidate built
  bool population_pass = false;   // 12/12 current-boot population evidence PASS
  uint32_t capture_session_id = 0;
  const Q0BootstrapCandidate* candidates = nullptr;
  uint8_t candidate_count = 0;
};

// Builds one current q0 candidate from repeated read-only observations.
// expected_provenance is the model this build/session expects. The supplied
// profile must already be bound to exactly that provenance.
//
// This pure reducer cannot prove wall-clock freshness by itself. Its
// LegPopulationEvidence and sample bundle must be assembled by one future
// read-only orchestration transaction; production wiring is forbidden until
// that same-session contract exists.
//
// No comparison against raw 2048 is performed. A candidate at 3000 is allowed
// to exist if the operator placement and measurements are internally stable;
// plausibility and later acceptance are separate gates.
Q0BootstrapCandidate buildQ0BootstrapCandidate(
    const CalibrationGeometryProfile& profile,
    const GeometryProvenance& expected_provenance,
    const calibration::LegPopulationEvidence& population,
    const Q0BootstrapRequest& request,
    const Q0CaptureSample* samples,
    uint8_t sample_count);

const char* toString(Q0BootstrapStatus status);

}  // namespace actuator
}  // namespace matdog

#endif  // MATDOG_ACTUATOR_CALIBRATION_Q0_BOOTSTRAP_H
