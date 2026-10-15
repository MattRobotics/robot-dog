#ifndef MATDOG_MOTION_PHASE_TIMED_ROUTE_H
#define MATDOG_MOTION_PHASE_TIMED_ROUTE_H
// Junction-limited time law for the certified paired PRONE route (Issue #52, 2026-10-15).
//
// The GEOMETRY is unchanged: the same 251-segment joint-space polyline as
// pairedProneSample() (same knots, fixture-variant floors, IK). Only the time
// law changes. Baseline: a rest-to-rest quintic on EVERY segment (251 stops,
// 60 s x route_time_scale). Here: the path parameter s runs along the
// infinity-norm arclength (|dq_j/ds| <= 1 for every joint) with a trapezoidal
// speed profile (|s''| <= a), cruise |s'| <= v, rest only at the 5 support
// transitions (PRE_ALIGN .. RISE), and at every interior knot a junction speed
//   s'_k <= a * dt / max_j |d_{k,j} - d_{k-1,j}|
// so the joint-velocity jump across a polyline corner is at most a*dt (one
// frame period). Forward/backward passes give the time-optimal profile under
// these bounds (look-ahead planner). Durations are DERIVED, never chosen.
#include "PairedProneRoute.h"
namespace matdog { namespace motion {
struct PhaseTimedRoute {
  bool valid = false;
  unsigned first[6]{};                       // first segment of each phase, first[5] = end
  double length[kPairedProneSegments]{};     // infinity-norm joint delta of each segment
  double entrySpeed[kPairedProneSegments + 1]{};  // s' at each knot (rad/s along the inf-norm arclength)
  double startS[kPairedProneSegments + 1]{};      // time of each knot from the start of its phase
  double durationS[5]{};
  double velocityRadS = 0, accelerationRadS2 = 0, frameS = 0, totalS = 0;
};
bool buildPhaseTimedRoute(bool fixtureLimitVariant, double velocityRadS, double accelerationRadS2, double frameS,
                          PhaseTimedRoute* out);
// Sample of phase `phase` at t seconds into it (clamped to its duration).
LocomotionFrame phaseTimedSample(const PhaseTimedRoute& route, unsigned phase, double t, bool fixtureLimitVariant,
                                 unsigned* segment = nullptr);
} }
#endif
