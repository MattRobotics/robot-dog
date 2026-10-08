#ifndef MATDOG_MOTION_LEG_KINEMATICS_H
#define MATDOG_MOTION_LEG_KINEMATICS_H

#include <stdint.h>

namespace matdog {
namespace motion {

// Canonical order. Angles are URDF radians; positions are base_link metres:
// +X forward, +Y left, +Z up. No actuator identity or calibration is present.
enum class LegId : uint8_t { LF = 0, RF = 1, RH = 2, LH = 3 };
struct Vector3 { double x, y, z; };
struct LegJointAngles { double hip, upper, lower; };
struct JointLimit { double lower, upper; };
struct LegModel {
  Vector3 hipOrigin, upperOrigin, lowerOrigin, footOrigin;
  JointLimit limits[3];
};

// Read-only generated model; nullptr for invalid identities.
const LegModel* legModel(LegId leg);
const char* geometrySourceSha256();

enum class KinematicsStatus : uint8_t {
  OK, INVALID_INPUT, JOINT_LIMIT,
  UNREACHABLE_GEOMETRY, UNREACHABLE_LIMITS, NUMERICAL_FAILURE
};
enum class Reachability : uint8_t { UNKNOWN, REACHABLE, UNREACHABLE };

struct FkResult {
  KinematicsStatus status = KinematicsStatus::INVALID_INPUT;
  Vector3 footOriginM{};
  double baseFromFootRotation[3][3]{};
};

// FK always enforces the canonical URDF limits (not operational safety limits).
FkResult forwardKinematics(LegId leg, const LegJointAngles& q);

struct IkOptions {
  // Select the in-limit solution closest to this seed in squared joint radians.
  // A seed outside the limits is rejected, never silently clamped.
  LegJointAngles seed{};
  double toleranceM = 1e-9;
};
struct IkResult {
  KinematicsStatus status = KinematicsStatus::INVALID_INPUT;
  Reachability reachability = Reachability::UNKNOWN;
  // These fields are usable ONLY when status == OK.
  LegJointAngles joints{};
  Vector3 achievedFootOriginM{};
  double residualM = 0.0;
  uint8_t solutions = 0;
};

// Target is the foot_link ORIGIN, not the ground contact reference. Bounded
// analytic enumeration, no iteration/heap/I/O. OK is kinematic reachability,
// never collision clearance, stand eligibility or actuator authorization.
IkResult inverseKinematics(LegId leg, const Vector3& targetFootOriginM,
                           const IkOptions& options = IkOptions{});

}  // namespace motion
}  // namespace matdog
#endif
