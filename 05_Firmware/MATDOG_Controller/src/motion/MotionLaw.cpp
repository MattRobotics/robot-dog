#include "MotionLaw.h"
#include <cmath>

namespace matdog { namespace motion {

namespace {
bool pos(double x) { return std::isfinite(x) && x > 0; }
bool limitsOk(const LawLimits& l) { return pos(l.vMax) && pos(l.aMax) && pos(l.jMax); }
}

SpeedChange speedChange(double v0, double v1, double aMax, double jMax) {
  SpeedChange r;
  if (!std::isfinite(v0) || !std::isfinite(v1) || v0 < 0 || v1 < 0 || !pos(aMax) || !pos(jMax)) return r;
  const double dv = std::fabs(v1 - v0);
  if (dv <= aMax * aMax / jMax) { r.timeS = 2 * std::sqrt(dv / jMax); r.peakAccel = std::sqrt(dv * jMax); }
  else { r.timeS = dv / aMax + aMax / jMax; r.peakAccel = aMax; }
  r.distance = 0.5 * (v0 + v1) * r.timeS;
  r.valid = true;
  return r;
}

bool ScurveProfile::plan(double length, const LawLimits& lim) {
  *this = ScurveProfile{};
  if (!pos(length) || !limitsOk(lim)) return false;
  const double a = lim.aMax, j = lim.jMax, vmax = lim.vMax;
  const double vTri = a * a / j;          // largest speed reached with a triangular acceleration pulse
  auto accelTime = [&](double v) { return v <= vTri ? 2 * std::sqrt(v / j) : v / a + a / j; };
  double vp;
  if (length >= vmax * accelTime(vmax)) vp = vmax;   // a cruise exists
  else {
    vp = std::cbrt(length * length * j / 4);          // (L*sqrt(j)/2)^(2/3), triangular acceleration
    if (vp > vTri) vp = 0.5 * a * (-a / j + std::sqrt(a * a / (j * j) + 4 * length / a));
  }
  if (!pos(vp)) return false;
  const bool tri = vp <= vTri;
  t1_ = tri ? std::sqrt(vp / j) : a / j;
  t2_ = tri ? 0 : vp / a - a / j;
  const double ta = 2 * t1_ + t2_;
  tv_ = std::fmax(0.0, length / vp - ta);
  duration_ = 2 * ta + tv_;
  vPeak_ = vp; aPeak_ = tri ? std::sqrt(vp * j) : a; j_ = j; length_ = length;
  const double d[7] = {t1_, t2_, t1_, tv_, t1_, t2_, t1_};
  const double jj[7] = {j, 0, -j, 0, -j, 0, j};
  double t = 0, s = 0, v = 0, ac = 0;
  for (unsigned i = 0; i < 7; ++i) {
    jerk_[i] = jj[i];
    s += v * d[i] + 0.5 * ac * d[i] * d[i] + jj[i] * d[i] * d[i] * d[i] / 6;
    v += ac * d[i] + 0.5 * jj[i] * d[i] * d[i];
    ac += jj[i] * d[i];
    t += d[i];
    tEnd_[i] = t; sEnd_[i] = s; vEnd_[i] = v; aEnd_[i] = ac;
  }
  valid_ = std::isfinite(duration_) && duration_ > 0;
  return valid_;
}

LawSample ScurveProfile::at(double t) const {
  LawSample r;
  if (!valid_) return r;
  if (!(t > 0)) return r;                                    // also NaN
  if (t >= duration_) { r.s = length_; return r; }
  unsigned i = 0;
  while (i < 6 && t > tEnd_[i]) ++i;
  const double t0 = i ? tEnd_[i - 1] : 0, s0 = i ? sEnd_[i - 1] : 0, v0 = i ? vEnd_[i - 1] : 0, a0 = i ? aEnd_[i - 1] : 0;
  const double d = t - t0, jj = jerk_[i];
  r.j = jj;
  r.a = a0 + jj * d;
  r.v = v0 + a0 * d + 0.5 * jj * d * d;
  r.s = s0 + v0 * d + 0.5 * a0 * d * d + jj * d * d * d / 6;
  if (r.s > length_) r.s = length_;
  if (r.v < 0) r.v = 0;
  return r;
}

double scurveMinTime(double length, const LawLimits& limits) {
  ScurveProfile p;
  return p.plan(length, limits) ? p.durationS() : 0;
}

double quinticMinTime(double length, double vMax, double aMax) {
  if (!pos(length) || !pos(vMax) || !pos(aMax)) return 0;
  // quintic smoothstep: peak velocity 15/8*L/T, peak acceleration 10/sqrt(3)*L/T^2
  const double byV = 1.875 * length / vMax;
  const double byA = std::sqrt(5.773502691896258 * length / aMax);
  return byV > byA ? byV : byA;
}

double accelSafeLatenessS(double v, double a, double period, double step, double maxLatenessS) {
  if (!pos(v) || !pos(a) || !pos(period) || !pos(step) || !pos(maxLatenessS) || v * period > step * (1 + 1e-9)) return 0;
  auto accel = [&](double late) {
    const double dt = period + late;
    const double advance = std::fmin(dt, step / v);          // generator: path time per frame, capped by the step budget
    const double vLate = v * advance / dt;
    return std::fabs(vLate - v) / (0.5 * (dt + period));     // executor rule: secant acceleration over release intervals
  };
  // The secant acceleration of a late frame rises then falls again (the denominator grows), so the tolerance is the
  // FIRST lateness that violates the limit; frames later than the stream gap rule are refused anyway.
  const unsigned n = 2000;
  double lo = 0;
  for (unsigned i = 1; i <= n; ++i) {
    const double late = maxLatenessS * i / n;
    if (accel(late) > a) {
      double hi = late;
      for (unsigned k = 0; k < 60; ++k) { const double mid = 0.5 * (lo + hi); (accel(mid) <= a ? lo : hi) = mid; }
      return lo;
    }
    lo = late;
  }
  return maxLatenessS;
}

double velocityForLateness(double vCap, double a, double period, double step, double target) {
  if (!pos(vCap) || !pos(a) || !pos(period) || !pos(step) || !(target >= 0)) return 0;
  const double vStep = step / period;                      // above this the nominal frame already exceeds the step cap
  double hi = vCap < vStep ? vCap : vStep;
  auto ok = [&](double v) { return accelSafeLatenessS(v, a, period, step, target + 1e-9) >= target; };
  if (ok(hi)) return hi;
  double lo = 0;
  for (unsigned i = 0; i < 60; ++i) { const double mid = 0.5 * (lo + hi); (ok(mid) ? lo : hi) = mid; }
  return lo;
}

DynamicAnalysis analyzeDynamicProfile(const DynamicProfile& p) {
  DynamicAnalysis r;
  if (!pos(p.velocityRadS) || !pos(p.accelerationRadS2) || !pos(p.periodS) || !pos(p.maxStepRad) || p.jerkRadS3 < 0 || !std::isfinite(p.jerkRadS3)) {
    r.reason = "INVALID_PROFILE"; return r;
  }
  r.stepAtNominalRad = p.velocityRadS * p.periodS;
  r.stepHeadroomMs = (p.maxStepRad / p.velocityRadS - p.periodS) * 1000.0;
  r.driveSpeedRadS = p.goalSpeed * kServoTickRad;
  r.driveAccelRadS2 = p.acc * 100.0 * kServoTickRad;
  r.driveSpeedRatio = r.driveSpeedRadS / p.velocityRadS;
  r.driveAccelRatio = r.driveAccelRadS2 / p.accelerationRadS2;
  r.accelSafeLatenessMs = accelSafeLatenessS(p.velocityRadS, p.accelerationRadS2, p.periodS, p.maxStepRad, kStreamMaxGapS - p.periodS) * 1000.0;
  if (r.stepAtNominalRad > p.maxStepRad * (1 + 1e-9)) r.reason = "STEP_EXCEEDS_CAP";
  else if (r.driveSpeedRatio < 1.0) r.reason = "DRIVE_SPEED_BELOW_REFERENCE";
  else if (r.driveAccelRatio < 1.0) r.reason = "DRIVE_ACCEL_BELOW_REFERENCE";
  else { r.valid = true; r.reason = "OK"; }
  return r;
}

} }
