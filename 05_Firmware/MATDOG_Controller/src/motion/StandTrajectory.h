#ifndef MATDOG_MOTION_STAND_TRAJECTORY_H
#define MATDOG_MOTION_STAND_TRAJECTORY_H
#include "FootContact.h"
namespace matdog { namespace motion {
// Semantic future MotionSafetyInterface input. No timing/rates or actuator IDs.
// Canonical order LF, RF, RH, LH; hip/upper/lower URDF radians in each leg.
struct JointTargetFrame {
  LegJointAngles legs[4]{};
  uint32_t sequence = 0;
  bool valid = false;  // all 12 targets must succeed atomically
};
struct StandDefinition {
  Vector3 contactsWorldM[4];
  LegJointAngles standSeed[4];
  double lowBodyHeightM, standBodyHeightM;
  uint16_t samples;
};
const StandDefinition& canonicalStandDefinition();
enum class StandStatus : uint8_t { OK, INVALID_DEFINITION, CONTACT_FAILURE, NOT_READY, COMPLETE };
struct StandSample {
  StandStatus status = StandStatus::NOT_READY;
  ContactStatus contactStatus = ContactStatus::INVALID_INPUT;
  uint8_t failedLeg = 255;
  JointTargetFrame target{};
  double bodyHeightM = 0.0;  // world translation (0,0,height), rotation identity
  Vector3 contactsWorldM[4]{};
  LegJointAngles jointDeltaRad[4]{};  // signed from preceding emitted sample
  LegJointAngles jointLimitMarginRad[4]{};
  ContactBranch branches[4]{};
  double maxContactResidualM = 0.0;
  double maxContactDriftM = 0.0;  // relative to first emitted contact, not target
};
struct StandMetrics {
  LegJointAngles maxAbsJointDeltaRad[4]{};
  double maxJointDeltaRad = 0.0;
  double maxContactResidualM = 0.0;
  double maxContactDriftM = 0.0;
  double minJointLimitMarginRad = 0.0;
  uint32_t branchChanges[4]{};
  uint16_t emittedSamples = 0;
};
// C4-A target solution, using the definition's preferred semantic seeds.
StandSample generateStandTarget(const StandDefinition& definition);

// Incremental low->stand ramp, one four-leg contact IK solve per next().
// initialize() anchors the branch at the final stand then solves the low pose.
// The caller must already provide a contact-compatible low stance; this does
// not generate a q=0->low-stance acquisition path. No time or rate is implied.
class ContactLockedStand {
 public:
  StandStatus initialize(const StandDefinition& definition);
  StandSample next();
  void stop() { ready_ = false; }
  const StandMetrics& metrics() const { return metrics_; }
 private:
  StandDefinition definition_{};
  StandSample previous_{};
  Vector3 firstContacts_[4]{};
  StandMetrics metrics_{};
  bool ready_ = false;
};
} }
#endif
