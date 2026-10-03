#ifndef MATDOG_MOTION_IMU_ATTITUDE_H
#define MATDOG_MOTION_IMU_ATTITUDE_H
#include <stdint.h>
namespace matdog { namespace motion {
// G5-A attitude data contract between sensor acquisition and body stabilization. Pure, deterministic, fixed storage,
// no heap, no device, clock or framework dependency: acquisition (the existing BNO085 driver) fills an ImuSnapshot, the
// caller supplies time, and everything below is arithmetic.
//
// Frame (MATDOG canonical, frozen by hardware Phase D for the BNO085): the quaternion (w,x,y,z) is the SH2 rotation vector
// and represents R_world_from_base_link; the sensor axes coincide with base_link (+X forward, +Y left, +Z up, no
// permutation, no sign flip). Only the TILT (roll and pitch from the measured up-vector) is used: the rotation-vector yaw
// has an arbitrary, magnetically referenced zero and is never consumed.
//   up-vector in base_link  g = R^T e_z = (-sin p, sin r cos p, cos r cos p)
//   roll  r = atan2(g_y, g_z)   (+r: left side rises)      pitch p = asin(-g_x)   (+p: nose down)
//   R = Rz(yaw) Ry(p) Rx(r)  (ZYX intrinsic)
struct ImuSnapshot {
  double w = 1, x = 0, y = 0, z = 0;  // SH2 rotation vector, R_world_from_base_link
  double stampS = 0;                  // acquisition time in the caller's time base (seconds)
  uint32_t sequence = 0;              // increases with every new sample
  double accuracyRad = 0;             // BNO085 reported estimated accuracy, radians
  uint8_t accuracyStatus = 0;         // BNO085 report status 0..3 (3 = high)
};
enum class AttitudeStatus : uint8_t {
  OK, INVALID_POLICY, NONFINITE, BAD_QUATERNION_NORM, STALE, FUTURE_STAMP, TIME_REGRESSION, SEQUENCE_REGRESSION,
  ACCURACY_STATUS_LOW, ACCURACY_RAD_EXCEEDED, TILT_OUT_OF_RANGE
};
// Every field is explicit: there is NO default policy and no value is a hardware-approved setting.
struct AttitudePolicy {
  double maxAgeS = 0;               // freshness: age == maxAgeS is fresh, greater is stale
  double maxAccuracyRad = 0;        // reject a sample whose reported accuracy is worse
  uint8_t minAccuracyStatus = 0;    // reject a sample whose status is lower
  double normTolerance = 0;         // |norm(q) - 1| must not exceed this
  double maxPlausibleTiltRad = 0;   // beyond this the sample is a fault, not a correctable tilt
};
struct Attitude { double rollRad = 0, pitchRad = 0, upBase[3] = {0, 0, 1}; };
bool validAttitudePolicy(const AttitudePolicy& p);
// Pure tilt extraction from a (normalised) quaternion; false if not finite or not unit within 1e-6.
bool tiltFromQuaternion(double w, double x, double y, double z, Attitude& out);
// Stateful guard: rejects time and sequence regression, so a replayed or reordered sample cannot drive a correction.
class AttitudeMonitor {
 public:
  bool configure(const AttitudePolicy& policy);
  // `out` is written only when the result is OK. nowS is caller time and must not run backwards.
  // `isNewSample` (optional) is true when the sequence advanced: a fast loop must not integrate the same sample twice.
  AttitudeStatus evaluate(const ImuSnapshot& snapshot, double nowS, Attitude& out, bool* isNewSample = nullptr);
  void reset();
 private:
  AttitudePolicy policy_{};
  bool configured_ = false, haveSample_ = false, haveNow_ = false;
  uint32_t lastSequence_ = 0;
  double lastStampS_ = 0, lastNowS_ = 0;
};
} }
#endif
