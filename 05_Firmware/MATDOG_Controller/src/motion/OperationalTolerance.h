#ifndef MATDOG_MOTION_OPERATIONAL_TOLERANCE_H
#define MATDOG_MOTION_OPERATIONAL_TOLERANCE_H
namespace matdog { namespace motion { namespace tolerance {
// Candidate V1: CAD/offline bounds are not actuator accuracy claims.
constexpr double kTickRad=6.28318530717958647692/4096;

constexpr double kTrackingRad=.020;
constexpr double kGeometryTubeRad=.040;
constexpr double kQuantizationRad=kTickRad/2+2e-6;
constexpr double kArrivalRad=10*kTickRad+kQuantizationRad;
constexpr double kMaxStepRad=.004;
// Qualification allowance, NOT a measured encoder/install accuracy.
constexpr double kModelUncertaintyRad=.015;
// Folded-side proof bounds, rounded conservatively below real gen-1 limits.
// These CHECK the loaded limits; they never replace raw limits or calibration.
constexpr double kEntryLowerMeasuredMinRad[4]={-1.5985,-1.5801,-1.5924,-1.5909};
constexpr double kEntryAdjacentModelRad=.008; // local angular qualification
// A coordinated segment may occupy its independent joint box, including a
// polyline corner. Charge the WHOLE step from a certified endpoint plus raw
// rounding; no assumption that the box midpoint lies on the polyline.
static_assert(kTrackingRad+kMaxStepRad+kQuantizationRad+kModelUncertaintyRad<kGeometryTubeRad,
              "tracking + segment box + quantization must fit the proof");
static_assert(kArrivalRad<kTrackingRad,"arrival is tighter than following");
} } }
#endif
