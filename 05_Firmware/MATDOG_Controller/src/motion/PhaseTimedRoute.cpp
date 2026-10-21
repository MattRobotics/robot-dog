#include "PhaseTimedRoute.h"
#include <cmath>
namespace matdog { namespace motion {
namespace {
double slotOf(const LegJointAngles& q, unsigned j) { return j == 0 ? q.hip : j == 1 ? q.upper : q.lower; }
// Trapezoid on one segment: entry v0, exit v1, length L, accel a, cruise vmax.
struct Trap { double v0, v1, vp, ta, tc, td; };
Trap trap(double L, double v0, double v1, double a, double vmax) {
  Trap t{v0, v1, 0, 0, 0, 0};
  t.vp = std::fmin(vmax, std::sqrt(std::fmax(0.0, a * L + 0.5 * (v0 * v0 + v1 * v1))));
  t.vp = std::fmax(t.vp, std::fmax(v0, v1));
  t.ta = (t.vp - v0) / a; t.td = (t.vp - v1) / a;
  const double dacc = (t.vp * t.vp - v0 * v0) / (2 * a), ddec = (t.vp * t.vp - v1 * v1) / (2 * a);
  t.tc = t.vp > 0 ? std::fmax(0.0, L - dacc - ddec) / t.vp : 0;
  return t;
}
}
bool buildPhaseTimedRoute(bool variant, double v, double a, double dt, PhaseTimedRoute* out) {
  if (!out) return false;
  static const PhaseTimedRoute kEmpty{};
  *out = kEmpty;
  if (!std::isfinite(v) || v <= 0 || !std::isfinite(a) || a <= 0 || !std::isfinite(dt) || dt <= 0) return false;
  // Junction caps: v by default; 0 at the 5 phase boundaries and the route end;
  // at an interior corner a*dt/|jump| (jump between the nearest non-zero
  // segments on each side; zero-length segments move no joint and take no time).
  double* cap = out->junctionCap;
  for (unsigned k = 0; k <= kPairedProneSegments; ++k) cap[k] = v;
  double prev[12]{}; bool havePrev = false;
  unsigned phase = 0; out->first[0] = 0;
  for (unsigned i = 0; i < kPairedProneSegments; ++i) {
    const unsigned ph = static_cast<unsigned>(pairedPronePhase(i));
    if (ph < phase || ph > 4) return false;   // contiguous, ordered phases
    while (phase < ph) { out->first[++phase] = i; cap[i] = 0; havePrev = false; }
    // Controller-thread static scratch: two LocomotionFrames (~1.7 KB) off the loop stack.
    static LocomotionFrame s0, s1;
    s0 = pairedProneSampleAt(i, 0, 0, 0, variant); s1 = pairedProneSampleAt(i, 1, 0, 0, variant);
    if (!s0.target.valid || !s1.target.valid) return false;
    double d[12], l = 0;
    for (unsigned k = 0; k < 12; ++k) { d[k] = slotOf(s1.target.legs[k / 3], k % 3) - slotOf(s0.target.legs[k / 3], k % 3); l = std::fmax(l, std::fabs(d[k])); }
    out->length[i] = l;
    if (l <= 0) continue;
    for (unsigned k = 0; k < 12; ++k) d[k] /= l;
    if (havePrev) { double jump = 0; for (unsigned k = 0; k < 12; ++k) jump = std::fmax(jump, std::fabs(d[k] - prev[k]));
      if (jump > 1e-12) cap[i] = std::fmin(cap[i], a * dt / jump); }
    else if (i > 0) cap[i] = 0;    // first moving segment of a phase starts from rest
    for (unsigned k = 0; k < 12; ++k) prev[k] = d[k];
    havePrev = true;
  }
  while (phase < 5) out->first[++phase] = kPairedProneSegments;
  cap[0] = 0; cap[kPairedProneSegments] = 0;
  // Forward / backward passes.
  double* vk = out->entrySpeed;
  for (unsigned k = 0; k <= kPairedProneSegments; ++k) vk[k] = cap[k];
  vk[0] = 0;
  for (unsigned k = 0; k < kPairedProneSegments; ++k) vk[k + 1] = std::fmin(vk[k + 1], std::sqrt(vk[k] * vk[k] + 2 * a * out->length[k]));
  for (unsigned k = kPairedProneSegments; k-- > 0;) vk[k] = std::fmin(vk[k], std::sqrt(vk[k + 1] * vk[k + 1] + 2 * a * out->length[k]));
  for (unsigned p = 0; p < 5; ++p) {
    double t = 0;
    for (unsigned i = out->first[p]; i < out->first[p + 1]; ++i) {
      out->startS[i] = t;
      if (out->length[i] > 0) { const Trap tr = trap(out->length[i], vk[i], vk[i + 1], a, v); t += tr.ta + tr.tc + tr.td; }
    }
    out->durationS[p] = std::fmax(t, dt);   // a zero-length phase still holds one frame
    out->totalS += out->durationS[p];
  }
  out->startS[kPairedProneSegments] = out->durationS[4];
  out->velocityRadS = v; out->accelerationRadS2 = a; out->frameS = dt; out->valid = true;
  return true;
}
namespace {
// Segment, linear progress and its time derivatives of phase `phase` at t (clamped to the phase duration).
struct RoutePoint { bool valid = false; unsigned segment = 0; double progress = 0, rate = 0, acceleration = 0; };
RoutePoint locate(const PhaseTimedRoute& r, unsigned phase, double t) {
  RoutePoint p;
  if (!r.valid || phase >= 5 || !std::isfinite(t) || t < 0) return p;
  const unsigned begin = r.first[phase], end = r.first[phase + 1];
  if (begin >= end) return p;
  if (t > r.durationS[phase]) t = r.durationS[phase];
  // Last segment whose start is <= t (zero-length segments occupy no time).
  unsigned i = begin;
  for (unsigned k = begin; k < end; ++k) if (r.length[k] > 0 && r.startS[k] <= t) i = k;
  p.valid = true;
  if (r.length[i] <= 0) { p.segment = end - 1; p.progress = 1; return p; }
  const Trap tr = trap(r.length[i], r.entrySpeed[i], r.entrySpeed[i + 1], r.accelerationRadS2, r.velocityRadS);
  double tau = t - r.startS[i], s = 0, sd = 0, sdd = 0;
  const double a = r.accelerationRadS2;
  if (tau <= tr.ta) { s = tr.v0 * tau + .5 * a * tau * tau; sd = tr.v0 + a * tau; sdd = a; }
  else if (tau <= tr.ta + tr.tc) { const double u = tau - tr.ta; s = (tr.vp * tr.vp - tr.v0 * tr.v0) / (2 * a) + tr.vp * u; sd = tr.vp; sdd = 0; }
  else { const double u = std::fmin(tau - tr.ta - tr.tc, tr.td);
    s = (tr.vp * tr.vp - tr.v0 * tr.v0) / (2 * a) + tr.vp * tr.tc + tr.vp * u - .5 * a * u * u; sd = tr.vp - a * u; sdd = u < tr.td ? -a : 0; }
  const double L = r.length[i];
  p.segment = i; p.progress = std::fmin(1.0, std::fmax(0.0, s / L)); p.rate = sd / L; p.acceleration = sdd / L;
  return p;
}
}
LocomotionFrame phaseTimedSample(const PhaseTimedRoute& r, unsigned phase, double t, bool variant, unsigned* segment) {
  const RoutePoint p = locate(r, phase, t);
  if (!p.valid) return {};
  if (segment) *segment = p.segment;
  return pairedProneSampleAt(p.segment, p.progress, p.rate, p.acceleration, variant);
}
bool phaseTimedJoints(const PhaseTimedRoute& r, unsigned phase, double t, bool variant, double q[12]) {
  const RoutePoint p = locate(r, phase, t);
  return p.valid && pairedProneJointsAt(p.segment, p.progress, variant, q);
}
} }
