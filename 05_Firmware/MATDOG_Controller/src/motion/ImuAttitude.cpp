#include "ImuAttitude.h"
#include <cmath>
namespace matdog { namespace motion {
bool validAttitudePolicy(const AttitudePolicy& p) {
  return std::isfinite(p.maxAgeS) && p.maxAgeS > 0 && std::isfinite(p.maxAccuracyRad) && p.maxAccuracyRad > 0 &&
         p.minAccuracyStatus <= 3 && std::isfinite(p.normTolerance) && p.normTolerance > 0 && p.normTolerance < 1e-2 &&
         std::isfinite(p.maxPlausibleTiltRad) && p.maxPlausibleTiltRad > 0 && p.maxPlausibleTiltRad < 1.5707963267948966;
}
bool tiltFromQuaternion(double w, double x, double y, double z, Attitude& out) {
  if (!std::isfinite(w) || !std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) return false;
  const double n = std::sqrt(w*w + x*x + y*y + z*z);
  if (!(std::abs(n - 1.0) <= 1e-6)) return false;
  w /= n; x /= n; y /= n; z /= n;
  // Third row of R_world_from_base: world +Z expressed in base_link.
  const double gx = 2*(x*z - w*y), gy = 2*(y*z + w*x), gz = 1 - 2*(x*x + y*y);
  Attitude a;
  a.upBase[0] = gx; a.upBase[1] = gy; a.upBase[2] = gz;
  a.rollRad = std::atan2(gy, gz);
  a.pitchRad = std::asin(std::fmax(-1.0, std::fmin(1.0, -gx)));
  out = a;
  return true;
}
bool AttitudeMonitor::configure(const AttitudePolicy& policy) {
  *this = AttitudeMonitor();
  if (!validAttitudePolicy(policy)) return false;
  policy_ = policy; configured_ = true;
  return true;
}
void AttitudeMonitor::reset() { haveSample_ = false; haveNow_ = false; }
AttitudeStatus AttitudeMonitor::evaluate(const ImuSnapshot& s, double nowS, Attitude& out, bool* isNewSample) {
  if (isNewSample) *isNewSample = false;
  if (!configured_) return AttitudeStatus::INVALID_POLICY;
  if (!std::isfinite(nowS) || !std::isfinite(s.stampS) || !std::isfinite(s.accuracyRad)) return AttitudeStatus::NONFINITE;
  if (haveNow_ && nowS < lastNowS_) return AttitudeStatus::TIME_REGRESSION;
  haveNow_ = true; lastNowS_ = nowS;
  if (s.stampS > nowS) return AttitudeStatus::FUTURE_STAMP;
  if (nowS - s.stampS > policy_.maxAgeS) return AttitudeStatus::STALE;
  if (haveSample_ && s.stampS < lastStampS_) return AttitudeStatus::TIME_REGRESSION;
  // A wrapped counter would need explicit handling by the adapter; a decreasing sequence is never accepted silently.
  if (haveSample_ && static_cast<int32_t>(s.sequence - lastSequence_) < 0) return AttitudeStatus::SEQUENCE_REGRESSION;
  const double n = std::sqrt(s.w*s.w + s.x*s.x + s.y*s.y + s.z*s.z);
  if (!std::isfinite(n)) return AttitudeStatus::NONFINITE;
  if (std::abs(n - 1.0) > policy_.normTolerance) return AttitudeStatus::BAD_QUATERNION_NORM;
  if (s.accuracyStatus < policy_.minAccuracyStatus) return AttitudeStatus::ACCURACY_STATUS_LOW;
  if (s.accuracyRad > policy_.maxAccuracyRad) return AttitudeStatus::ACCURACY_RAD_EXCEEDED;
  Attitude a;
  if (!tiltFromQuaternion(s.w/n, s.x/n, s.y/n, s.z/n, a)) return AttitudeStatus::BAD_QUATERNION_NORM;
  if (std::abs(a.rollRad) > policy_.maxPlausibleTiltRad || std::abs(a.pitchRad) > policy_.maxPlausibleTiltRad) return AttitudeStatus::TILT_OUT_OF_RANGE;
  if (isNewSample) *isNewSample = !haveSample_ || s.sequence != lastSequence_;
  haveSample_ = true; lastStampS_ = s.stampS; lastSequence_ = s.sequence;
  out = a;
  return AttitudeStatus::OK;
}
} }
