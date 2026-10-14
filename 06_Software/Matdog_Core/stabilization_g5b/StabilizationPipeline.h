#ifndef MATDOG_G5B_STABILIZATION_PIPELINE_H
#define MATDOG_G5B_STABILIZATION_PIPELINE_H
#include "BodyStabilizer.h"
#include "TiltCompensation.h"

namespace matdog { namespace stabilization {
using motion::Vector3;
using motion::BodyPose;
using motion::JointTargetFrame;

// These values describe input evidence, never actuator authority.
enum class Evidence : uint8_t { UNAVAILABLE, OFFLINE_FIXTURE, EXTERNAL_MEASUREMENT };
enum class ContactState : uint8_t { UNKNOWN, SWING, CONFIRMED_SUPPORT };
enum class SurfaceMode : uint8_t { WORLD_LOCKED_HORIZONTAL, MOVING_PLANE_EXPERIMENTAL };
struct ContactObservation {
  Vector3 worldM{}; // G2 cylinder reference, NOT foot_link origin or inferred load
  ContactState state = ContactState::UNKNOWN;
  double confidence = 0;
};
struct SupportSnapshot {
  Vector3 normalWorld{0, 0, 1}, originWorldM{};
  ContactObservation contacts[4]{}; // LF RF RH LH; all four must be confirmed in G5-B
  double stampS = 0;
  uint32_t sequence = 0, revision = 0;
  Evidence evidence = Evidence::UNAVAILABLE;
  SurfaceMode mode = SurfaceMode::WORLD_LOCKED_HORIZONTAL;
};
struct Bno085Sample {
  motion::ImuSnapshot orientation{};
  uint32_t acquisitionEpoch = 0; // sensor reset/counter wrap requires an explicit re-arm
  bool connected = false;
};
// Tilt derivatives in rad/s in the same calibrated tilt convention as ImuAttitude.
// Raw gyro X/Y are NOT generally these derivatives; conversion belongs to acquisition.
struct TiltRateSample {
  double rollRadS = 0, pitchRadS = 0, stampS = 0;
  uint32_t sequence = 0;
  bool valid = false;
};
struct NominalPose {
  BodyPose pose{};
  JointTargetFrame anchor{}; // only used by enable(): caller-acknowledged semantic pose
  uint32_t revision = 0;
};
struct OperationalBounds {
  motion::JointLimit joints[4][3]{}; // caller supplies; subset of immutable URDF limits
  Evidence evidence = Evidence::UNAVAILABLE;
  uint32_t revision = 0;
};
struct PipelineConfig {
  motion::AttitudePolicy attitude{};
  motion::StabilizerConfig stabilizer{}; // G5-A baseline, including calibrated reference
  Evidence referenceEvidence = Evidence::UNAVAILABLE;
  uint32_t referenceRevision = 0;
  OperationalBounds bounds{};
  double maxSampleGapS = 0, maxCommandLeadS = 0, maxMeasurementToCommandS = 0;
  double maxSupportAgeS = 0, minContactConfidence = 0, planeToleranceM = 0;
  double maxPlaneTiltRad = 0, maxContactSpeedMS = 0, maxNormalRateRadS = 0;
  double minJointMarginRad = 0, maxJointStepRad = 0, maxJointRateRadS = 0;
  double maxIkCondition = 0, ikToleranceM = 0;
  unsigned maxIkIterations = 0;
  bool allowMovingPlaneExperiment = false;
  bool allowDampingExperiment = false;
  double dampingTimeS = 0, maxTiltRateRadS = 0;
};
enum class Reason : uint8_t {
  OK, NOT_CONFIGURED, DISABLED, FAULT_LATCHED, INVALID_TIME, DISCONNECTED,
  IMU_REJECTED, DUPLICATE_SAMPLE, INCOHERENT_SAMPLE, SAMPLE_GAP, EPOCH_CHANGED,
  COMMAND_TOO_LATE, INVALID_NOMINAL, INVALID_SUPPORT, STALE_SUPPORT,
  UNCONFIRMED_CONTACT, SUPPORT_ORDER, PLANE_GATE, PLANE_MOTION,
  INVALID_RATE, IK_FAILURE, CONDITION_LIMIT, OPERATIONAL_LIMIT, TARGET_DISCONTINUITY,
  OUTPUT_SEQUENCE_EXHAUSTED
};
const char* reasonName(Reason reason);
struct PipelineOutput {
  Reason reason = Reason::NOT_CONFIGURED;
  motion::AttitudeStatus attitudeStatus = motion::AttitudeStatus::OK;
  motion::StabilizerOutput correction{}; // diagnostics only when target.valid is false
  BodyPose correctedPose{};
  JointTargetFrame target{}; // invalid and zeroed on EVERY refusal; no cached targets
  Vector3 achievedContactsWorldM[4]{};
  uint32_t inputSequence = 0, supportSequence = 0, nominalRevision = 0;
  uint32_t referenceRevision = 0, boundsRevision = 0, acquisitionEpoch = 0;
  uint32_t skippedSamples = 0;
  double generatedAtS = 0, intendedCommandAtS = 0, validUntilS = 0;
  double measurementAgeS = 0, measurementToCommandS = 0, sampleIntervalS = 0;
  double maxContactResidualM = 0, minJointMarginRad = 0, maxJointDeltaRad = 0;
  double maxCondition = 0;
  unsigned maxIterations = 0;
  uint8_t failedLeg = 255;
  motion::TiltedIkResult::Status ikStatus = motion::TiltedIkResult::Status::OK;
  bool faultLatched = false, experimental = false;
};
// Pure C++17 value-state machine. No clock, heap, peripherals, threads, calibration writes,
// motion ownership or actuator backend. A validated frame remains a kinematic proposal.
class StabilizationPipeline {
 public:
  bool configure(const PipelineConfig& config); // allowed only before enable or after draining
  bool enable(const NominalPose& initial, double nowS);
  void disable(); // no more frames; G5-A correction drains internally on subsequent update()
  bool resetFault(); // only after disable + drain to zero; explicit new enable required
  PipelineOutput update(const Bno085Sample& sample, const NominalPose& nominal,
                        const SupportSnapshot& support, double nowS, double intendedCommandS,
                        const TiltRateSample& rate = TiltRateSample{});
  bool faultLatched() const { return fault_; }
 private:
  PipelineOutput reject(PipelineOutput out, Reason reason, double nowS);
  PipelineConfig cfg_{};
  motion::AttitudeMonitor monitor_{};
  motion::BodyStabilizer stabilizer_{};
  motion::StabilizerOutput diagnostic_{};
  JointTargetFrame previous_{};
  Bno085Sample lastSample_{};
  SupportSnapshot lastSupport_{};
  double previousCommandS_ = 0, lastNewNowS_ = 0, lastCallS_ = 0;
  uint32_t outputSequence_ = 0;
  bool configured_ = false, enabled_ = false, fault_ = false;
  bool haveSample_ = false, haveSupport_ = false;
};
} }
#endif
