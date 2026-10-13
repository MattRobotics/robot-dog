#ifndef MATDOG_MOTION_PAIRED_PRONE_ROUTE_H
#define MATDOG_MOTION_PAIRED_PRONE_ROUTE_H
#include "MotionFeedback.h"
namespace matdog { namespace motion {
enum class PronePhase : uint8_t { PRE_ALIGN, REAR_DEPLOY, FRONT_DEPLOY, LOAD_TRANSFER, RISE };
// Immutable 251-segment semantic profile. Synthetic geometry is compiled into
// flash; it supplies targets, never measured support, limits or authority.
constexpr unsigned kPairedProneSegments=251;
const char* pairedProneDigest();
double pairedProneDuration(unsigned segment);
PronePhase pairedPronePhase(unsigned segment);
LocomotionFrame pairedProneSample(unsigned segment,double u);
// Recover only an identified segment boundary; arbitrary torque-off poses and
// interpolation chords are not routes. No nearest-pose fallback outside gates.
double canonicalFootPlaneMinimum(LegId,const LegJointAngles&,const BodyPose&);
double canonicalFootRadialCorrectionBound();
int pairedProneMatch(const AdaptiveObservation&,const FeedbackPolicy&);
} }
#endif
