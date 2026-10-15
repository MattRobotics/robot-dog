#ifndef MATDOG_MOTION_PAIRED_PRONE_ROUTE_H
#define MATDOG_MOTION_PAIRED_PRONE_ROUTE_H
#include "MotionFeedback.h"
namespace matdog { namespace motion {
enum class PronePhase : uint8_t { PRE_ALIGN, REAR_DEPLOY, FRONT_DEPLOY, LOAD_TRANSFER, RISE };
// Immutable 251-segment semantic profile. Synthetic geometry is compiled into
// flash; it supplies targets, never measured support, limits or authority.
constexpr unsigned kPairedProneSegments=251;
const char* pairedProneDigest(bool fixtureLimitVariant=false);
double pairedProneDuration(unsigned segment);
PronePhase pairedPronePhase(unsigned segment);
LocomotionFrame pairedProneSample(unsigned segment,double u,bool fixtureLimitVariant=false,double beginProgress=0);
// The same polyline point (segment, linear progress 0..1) with a caller-supplied
// time law: d(progress)/dt and d2(progress)/dt2 (Issue #52 phase-timed STAND).
LocomotionFrame pairedProneSampleAt(unsigned segment,double progress,double rate,double acceleration,bool fixtureLimitVariant=false);
// Loaded routes recover verified boundaries. A suspended reviewed tube may
// admit a measured common progress on a certified segment; never arbitrary Q.
double canonicalFootPlaneMinimum(LegId,const LegJointAngles&,const BodyPose&);
double canonicalFootRadialCorrectionBound();
// C-infinity conservative minimum used only by the suspended WALK adapter.
// hardMin - epsilon*log(vertexCount) <= softMin <= hardMin. No load claim.
double canonicalFootPlaneSoftMinimum(LegId,const LegJointAngles&,const BodyPose&,double temperatureScale=1);
double canonicalFootSoftMinimumBound();
struct PairedProneCursor {int segment=-1;double progress=0;};
PairedProneCursor pairedProneCursor(const AdaptiveObservation&,const FeedbackPolicy&);
int pairedProneMatch(const AdaptiveObservation&,const FeedbackPolicy&);
} }
#endif
