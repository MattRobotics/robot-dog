#ifndef MATDOG_MOTION_STARTUP_ACQUISITION_H
#define MATDOG_MOTION_STARTUP_ACQUISITION_H
#include "BodyPose.h"
#include "StandTrajectory.h"
namespace matdog { namespace motion {
enum class StartupCondition : uint8_t { UNKNOWN, SUSPENDED_REFERENCE, FLOOR_CONTACT_UNVERIFIED, VERIFIED_LOW_STANCE };
enum class SupportRegime : uint8_t { UNKNOWN, EXTERNAL_SUSPENSION, UNVERIFIED_FLOOR, FOUR_CONTACT_LOCKED };
enum class AcquisitionStatus : uint8_t {
  READY, UNKNOWN_POSE, SUSPENDED_PATH_UNPROVEN, FLOOR_ACQUISITION_UNPROVEN,
  MISSING_EVIDENCE, INVALID_OBSERVATION, BODY_POSE_MISMATCH, JOINT_LIMIT,
  LOW_STANCE_MISMATCH, CONTACT_MISMATCH, REFERENCE_FAILURE
};
struct StartupObservation {
  StartupCondition condition = StartupCondition::UNKNOWN;
  BodyPose body{};
  JointTargetFrame joints{};
  // Explicit external evidence inputs, NOT measurements or permission to actuate.
  bool fresh = false;
  bool stationary = false;
  bool fourFootSupportConfirmed = false;
  bool flatGroundConfirmed = false;
  bool collisionClearanceReviewed = false;
  bool supportAndLoadReviewed = false;
};
struct AcquisitionResult {
  AcquisitionStatus status = AcquisitionStatus::UNKNOWN_POSE;
  SupportRegime support = SupportRegime::UNKNOWN;
  StandSample lowStance{};  // valid ONLY for READY; a zero-motion acquisition hold
};
// Only canonical C4 low stance is supported. Agreement tolerances (1e-8 rad,
// 1e-9 m, 1e-10 rotation entries) are numerical/model tolerances, NOT approved
// physical sensor uncertainty. Other starting conditions produce no target.
AcquisitionResult evaluateStartup(const StartupObservation& observation);
} }
#endif
