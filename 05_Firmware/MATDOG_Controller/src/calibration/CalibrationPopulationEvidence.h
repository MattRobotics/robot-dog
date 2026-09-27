#ifndef MATDOG_CALIBRATION_CALIBRATION_POPULATION_EVIDENCE_H
#define MATDOG_CALIBRATION_CALIBRATION_POPULATION_EVIDENCE_H

#include <stdint.h>

#include "CalibrationDomain.h"
#include "../servo/ServoPopulation.h"
#include "../servo/ServoPreflight.h"

// CR1 - formal current leg-population evidence producer.
//
// This is a pure adapter over already-produced structured Controller evidence.
// It never scans a bus, owns no transport and cannot write hardware. The raw
// census remains the servo census service responsibility and the twelve-joint
// configuration/profile verification remains ServoPreflight's.
//
// This does NOT rename @SERVO PREFLIGHT as "H1". Formal calibration population
// evidence is derived only when a current-session observation bundle is
// explicitly supplied and both source products satisfy the formal gate.

namespace matdog {
namespace calibration {

enum class PopulationEvidenceBuildStatus : uint8_t {
  NOT_EVALUATED                 = 0,
  REJECT_SOURCE_NOT_CURRENT     = 1,
  REJECT_CENSUS_INCOMPLETE      = 2,
  REJECT_BUS_ANOMALY            = 3,
  REJECT_LEG_MISSING_IN_CENSUS  = 4,
  REJECT_PREFLIGHT_INCOMPLETE   = 5,
  REJECT_IDENTITY_MISMATCH      = 6,
  REJECT_JOINT_QUALIFICATION    = 7,
  REJECT_DUPLICATE_SLOT         = 8,
  PASS                          = 9,
};

// The session orchestrator owns freshness. CR2-B is the first reviewed
// production caller: it may set current_observation_bundle only after it has
// itself sequenced a fresh census followed by a fresh preflight inside one
// acquisition transaction. Cached stand-alone diagnostics remain ineligible.
struct PopulationEvidenceBuildContext {
  bool current_observation_bundle = false;
  uint32_t session_ms = 0;
};

struct PopulationEvidenceBuildResult {
  PopulationEvidenceBuildStatus status = PopulationEvidenceBuildStatus::NOT_EVALUATED;
  LegPopulationEvidence evidence{};
  uint8_t qualified_slots = 0;
  uint16_t rejected_slots = 0;
};

// One canonical semantic mapping shared by CR1 and CR2-B. This converts the
// reviewed current allocation row into JointIdentity; it does not observe
// physical-unit identity from hardware (ST3215 cannot report that label).
bool semanticIdentityFromCanonical(const servo::CanonicalServo& canonical,
                                   JointIdentity* out);

PopulationEvidenceBuildResult buildCurrentLegPopulationEvidence(
    const servo::CensusResult& census,
    const servo::PreflightResult& preflight,
    const PopulationEvidenceBuildContext& context);

const char* toString(PopulationEvidenceBuildStatus status);

}  // namespace calibration
}  // namespace matdog

#endif  // MATDOG_CALIBRATION_CALIBRATION_POPULATION_EVIDENCE_H
