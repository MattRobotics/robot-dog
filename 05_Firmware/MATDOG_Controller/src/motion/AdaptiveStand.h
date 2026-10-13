#ifndef MATDOG_MOTION_ADAPTIVE_STAND_H
#define MATDOG_MOTION_ADAPTIVE_STAND_H
#include "PairedProneRoute.h"
namespace matdog { namespace motion {
enum class AdaptiveStandState : uint8_t { IDLE, OBSERVE, CLASSIFY, ACQUIRE_SUPPORT, RISE, VERIFY_STAND, STAND, REJECTED, STOPPED, PRE_ALIGN, REAR_DEPLOY, FRONT_DEPLOY, LOAD_TRANSFER, PAIRED_RISE };
enum class ObservedPoseClass : uint8_t { UNKNOWN, BELLY_FOLDED, SUPPORTED_INTERMEDIATE, SUPPORTED_STAND, NOMINAL_STAND, UNRECOVERABLE, PARTIAL_SUPPORT };
const char* adaptiveStandStateName(AdaptiveStandState state);
struct AcquisitionWaypoint {
  BodyPose body{};
  Vector3 feetWorldM[4]{};
  uint8_t requiredFeet=0;
  bool bellySupportRequired=false;
  bool externalSupportRequired=false;
  double durationS=0;
};
// Offline Geometry Compiler V5 supplies a bounded Cartesian route. It must begin
// at the observed pose, finish in four-foot support, and carry independent segment
// witnesses. Route existence or a digest is never collision/contact permission.
struct AcquisitionRoute {
  static constexpr uint8_t kMaxWaypoints=32;
  AcquisitionWaypoint waypoints[kMaxWaypoints]{};
  uint8_t count=0;
  bool supervisedSegments=false;
  double timeScale=1;
  bool pairedProne=false; // immutable profile, no editable waypoint copy
  bool geometryIdentityVerified=false;
  EvidenceSource source=EvidenceSource::UNKNOWN;
};
// Reproducible OFFLINE/SYNTHETIC candidate only. Physical review rejects its
// provenance; V5 sampled audit and independent support/load witnesses remain required.
bool restGroundAcquisitionCandidate(AcquisitionRoute& route);
struct AdaptiveStandRequest {
  double targetHeightM=.150, riseDurationS=8;
  uint32_t firstSequence=1;
  const AcquisitionRoute* acquisition=nullptr; // copied at request; caller may release
};
class AdaptiveStand {
 public:
  bool request(const AdaptiveStandRequest& request,const FeedbackPolicy& policy,double nowS);
  FeedbackFrame update(const AdaptiveObservation& observation,double nowS);
  bool assess(const SegmentAssessment& assessment,double nowS);
  void stop();
  AdaptiveStandState state() const {return state_;}
  ObservedPoseClass poseClass() const {return poseClass_;}
  FeedbackReason reason() const {return reason_;}
  unsigned proneSegment() const {return pairedSegment_;}
  bool approveProneSegment(unsigned segment){if(segment!=pairedSegment_||!route_.supervisedSegments||pendingValid_)return false;approvedSegment_=segment;return true;}
  bool physicalReadiness() const {return physicalReady_;}
 private:
  FeedbackFrame reject(FeedbackReason reason);
  bool beginSegment(const AcquisitionWaypoint& endpoint);
  bool beginRise();
  bool beginPairedSegment();
  FeedbackFrame generate(double nowS);
  FeedbackPolicy policy_{};
  AdaptiveStandRequest request_{};
  AcquisitionRoute route_{};
  AdaptiveStandState state_=AdaptiveStandState::IDLE;
  ObservedPoseClass poseClass_=ObservedPoseClass::UNKNOWN;
  FeedbackReason reason_=FeedbackReason::NOT_REQUESTED;
  LocomotionFrame previous_{}, segmentStart_{}, pendingSolution_{};
  AcquisitionWaypoint endpoint_{};
  FeedbackFrame pending_{};
  SegmentAssessment assessment_{};
  uint32_t sequence_=0, segmentIndex_=0, segmentIntervals_=0;
  uint8_t routeIndex_=0;
  uint16_t pairedSegment_=0,approvedSegment_=UINT16_MAX;
  double entry_progress_=0;
  LegJointAngles entry_offset_[4]{};
  uint16_t entry_knot_=UINT16_MAX;
  double lastNow_=0, pendingSince_=0, verifySince_=0;
  bool pendingValid_=false, assessed_=false, physicalReady_=false;
};
} }
#endif
