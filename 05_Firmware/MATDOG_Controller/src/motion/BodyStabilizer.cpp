#include "BodyStabilizer.h"
#include <cmath>
namespace matdog { namespace motion {
namespace {
double clampAbs(double v, double lim, bool& hit) { if (v > lim) { hit = true; return lim; } if (v < -lim) { hit = true; return -lim; } return v; }
// Move `v` toward `target` by at most `maxStep`.
double towards(double v, double target, double maxStep) {
  const double d = target - v;
  return std::abs(d) <= maxStep ? target : v + (d > 0 ? maxStep : -maxStep);
}
}
double maxStableIntegralGain(double dtS, unsigned delaySteps) {
  // Plant = pure delay of N steps. Loop  c[k+1] = c[k] - g c[k-N]  with g = ki dt has characteristic polynomial
  // z^(N+1) - z^N + g. It is asymptotically stable iff  g < 2 sin(pi / (2 (2N+1))).
  if (!std::isfinite(dtS) || dtS <= 0) return 0;
  const double pi = 3.14159265358979323846;
  return 2.0 * std::sin(pi / (2.0 * (2.0 * delaySteps + 1.0))) / dtS;
}
bool validStabilizerConfig(const StabilizerConfig& c) {
  const double vals[] = {c.integralGainPerS, c.maxCorrectionRad, c.maxRateRadS, c.deadbandRad, c.sampleDtS, c.holdBeforeFaultS, c.rollReferenceRad, c.pitchReferenceRad};
  for (double v : vals) if (!std::isfinite(v)) return false;
  return c.integralGainPerS > 0 && c.maxCorrectionRad > 0 && c.maxCorrectionRad < 0.5 && c.maxRateRadS > 0 && c.deadbandRad >= 0 &&
         c.sampleDtS > 0 && c.holdBeforeFaultS > 0 && c.integralGainPerS < maxStableIntegralGain(c.sampleDtS, c.loopDelaySteps);
}
bool BodyStabilizer::configure(const StabilizerConfig& config) {
  *this = BodyStabilizer();
  if (!validStabilizerConfig(config)) return false;
  cfg_ = config; configured_ = true;
  return true;
}
bool BodyStabilizer::enable(double nowS) {
  if (!configured_ || state_ != StabilizerState::DISABLED || !std::isfinite(nowS)) return false;  // never from FAULT or while ramping
  roll_ = pitch_ = 0; state_ = StabilizerState::ACTIVE; lastNowS_ = lastValidS_ = nowS; haveNow_ = true; pendingDisable_ = false;
  return true;
}
void BodyStabilizer::disable() {
  if (state_ == StabilizerState::DISABLED || state_ == StabilizerState::FAULT) return;
  // The correction is removed at the bounded rate, never as a step.
  if (roll_ == 0 && pitch_ == 0) state_ = StabilizerState::DISABLED; else { state_ = StabilizerState::RAMPING_DOWN; pendingDisable_ = true; }
}
void BodyStabilizer::resetFault() { if (state_ == StabilizerState::FAULT && roll_ == 0 && pitch_ == 0) state_ = StabilizerState::DISABLED; }
StabilizerOutput BodyStabilizer::step(AttitudeStatus status, const Attitude& a, bool isNew, double nowS) {
  StabilizerOutput out; out.attitudeStatus = status;
  if (!configured_ || state_ == StabilizerState::DISABLED) { out.state = StabilizerState::DISABLED; return out; }
  if (!std::isfinite(nowS) || (haveNow_ && nowS < lastNowS_)) { status = AttitudeStatus::TIME_REGRESSION; out.attitudeStatus = status; }
  const bool timeOk = std::isfinite(nowS) && (!haveNow_ || nowS >= lastNowS_);
  const double dt = timeOk && haveNow_ ? nowS - lastNowS_ : 0.0;
  const bool dtOk = dt <= 4.0 * cfg_.sampleDtS;
  if (timeOk) { lastNowS_ = nowS; haveNow_ = true; }
  const bool valid = status == AttitudeStatus::OK && std::isfinite(a.rollRad) && std::isfinite(a.pitchRad) && timeOk && dtOk;
  const bool latched = state_ == StabilizerState::FAULT;
  const bool draining = state_ == StabilizerState::RAMPING_DOWN && pendingDisable_;
  if (valid && !latched && !draining) {
    if (isNew && dt > 0) {
      bool hit = false;
      auto dead = [&](double e) { return std::abs(e) <= cfg_.deadbandRad ? 0.0 : (e > 0 ? e - cfg_.deadbandRad : e + cfg_.deadbandRad); };
      const double er = dead(a.rollRad - cfg_.rollReferenceRad), ep = dead(a.pitchRad - cfg_.pitchReferenceRad);
      const double step = cfg_.maxRateRadS * dt;
      roll_ = towards(roll_, clampAbs(roll_ - cfg_.integralGainPerS * dt * er, cfg_.maxCorrectionRad, hit), step);
      pitch_ = towards(pitch_, clampAbs(pitch_ - cfg_.integralGainPerS * dt * ep, cfg_.maxCorrectionRad, hit), step);
      out.saturated = hit;
    }  // the same sample again (or dt == 0): hold; freshness is the monitor's job
    state_ = StabilizerState::ACTIVE; lastValidS_ = nowS;
  } else {
    // Invalid, stale, regressed, late, missing, draining or latched: no new correction, ramp to zero at the bounded rate.
    const double step = dtOk ? cfg_.maxRateRadS * dt : 0.0;
    roll_ = towards(roll_, 0.0, step); pitch_ = towards(pitch_, 0.0, step);
    if (!latched && !draining) {
      state_ = StabilizerState::RAMPING_DOWN;
      if (nowS - lastValidS_ > cfg_.holdBeforeFaultS) state_ = StabilizerState::FAULT;  // latched; keeps ramping, never jumps
    }
    if (draining && roll_ == 0 && pitch_ == 0) { state_ = StabilizerState::DISABLED; pendingDisable_ = false; }
  }
  out.rollCorrectionRad = roll_; out.pitchCorrectionRad = pitch_; out.state = state_;
  return out;
}
} }
