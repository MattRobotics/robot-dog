#ifndef MATDOG_MOTION_LEG_KINEMATICS_INTERNAL_H
#define MATDOG_MOTION_LEG_KINEMATICS_INTERNAL_H
#include "TiltedContactIk.h"
#include "FootContactData.h"
namespace matdog { namespace motion { namespace detail {
// FP32 kinematic core (Motion FP32, 18/11/2026). The ESP32-S3 FPU computes
// binary32 in hardware; binary64 is a software library on that chip. The
// bounded per-release kinematics (base-frame vectors, rotations, joint
// angles, contact geometry, Jacobians, Newton) is computed here in float.
// The public double API converts at its boundary. World-frame translations
// and the gait clock grow with a walk and stay double, as do the joint
// frames sent to the servos: RAW conversion, boxes and the STOP proof read
// the same doubles as before.
struct Vec3f { float x, y, z; };
struct JointsF { float hip, upper, lower; };
constexpr Vec3f toF(const Vector3& v) { return {float(v.x), float(v.y), float(v.z)}; }
constexpr Vector3 toD(const Vec3f& v) { return {v.x, v.y, v.z}; }
constexpr LegJointAngles toD(const JointsF& q) { return {q.hip, q.upper, q.lower}; }
// Numerical criteria of the core, from the binary32 unit roundoff
// u = 2^-24 = 6.0e-8 on decimetre geometry. Convergence and roundoff
// tolerances only: no joint limit, workspace, box, contact policy or STOP
// margin is derived from them.
//  kResidualM: FK of a float IK solution misses its target by <= ~3e-7 m
//    (angles within a few ulp, ~20 roundings of 0.2 m terms; measured max
//    5.1e-8 m on 78 200 law cases, 07/11). 1 um is 1/50 of the 50 um of foot
//    height that half a RAW tick moves (|J^-1 e_z| <= 15.3 rad/m).
//  kDomainEpsilon: roundoff guard of the reach tests (m) and of the elbow
//    cosine (roundoff ~3e-7); a target passing it is clamped to the boundary
//    and must still verify within kResidualM.
//  kLimitEpsilonRad: an angle from float acos/atan2 within ~5e-7 rad of a
//    limit is clamped to it (6.5e-4 tick).
//  kPivot: singular pivot of a 3x3 leg Jacobian, m/rad (operational pivots
//    0.03-0.2; float roundoff of a zero pivot ~1e-8).
//  kDegenerate: cylinder axis within 1e-5 rad of the ground normal.
//  kRotationTolerance: orthonormality of a float rotation (worst-case
//    roundoff of a float-rounded rotation ~1.5e-6 in the determinant); a
//    point 0.3 m away moves by <= 3 um.
constexpr float kResidualM = 1e-6f, kDomainEpsilon = 1e-6f, kLimitEpsilonRad = 1e-6f;
constexpr float kPivot = 1e-6f, kDegenerate = 1e-5f, kRotationTolerance = 1e-5f;

// Generated model rounded to nearest float, joint limits rounded inward: a
// float angle inside them is inside the URDF limits as a double too.
struct ModelF { Vec3f hipOrigin, upperOrigin, lowerOrigin, footOrigin; float lower[3], upper[3]; };
const ModelF* modelF(LegId leg);
// A double joint triple checked exactly as the double kinematics did (finite,
// inside the double URDF limits, never clamped as a policy); its float image
// is then held inside the inward float limits (at most one float ulp).
// Returns OK, INVALID_INPUT (not finite) or JOINT_LIMIT.
KinematicsStatus jointsF(LegId leg, const LegJointAngles& q, JointsF& out);
bool inLimits(const ModelF& model, const JointsF& q);
// Inverse of a 3x3 Jacobian in one Gauss-Jordan pass on [J|I]: partial pivots,
// singular below kPivot, one reciprocal per pivot.
bool invert3(const float j[3][3], float inverse[3][3]);
// Internal analytic reduction for exporter-verified X/Y/Y chains only.
// hipFrameZ is a translation AFTER hip rotation, BEFORE the planar chain.
// Public G1 callers always use zero; G2 uses it for the radial contact term.
struct AnalyticBranch { int8_t hip = 0; int8_t elbow = 0; };
struct FkF { KinematicsStatus status = KinematicsStatus::INVALID_INPUT; Vec3f footOriginM{}; float baseFromFootRotation[3][3]{}; };
FkF forwardWithModel(const ModelF& model, const JointsF& q, float hipFrameZ);
struct IkF {
  KinematicsStatus status = KinematicsStatus::INVALID_INPUT;
  Reachability reachability = Reachability::UNKNOWN;
  JointsF joints{};
  Vec3f achievedFootOriginM{};
  float residualM = 0;
  uint8_t solutions = 0;
};
// target: from finite doubles (an overflow to infinity is unreachable);
// seed: a jointsF image; toleranceM in [kResidualM, 1e-3].
IkF inverseWithModel(const ModelF& model, const Vec3f& target, const JointsF& seed,
                     float toleranceM, float hipFrameZ, AnalyticBranch* branch = nullptr);

// Foot contact, FootContact.h semantics. The generated contact model rounded to nearest float.
struct ContactModelF { Vec3f centerInFootM, axisInFoot; float radiusM, supportWidthM, filletRadiusM, nominalTiltRad; };
constexpr ContactModelF kContactF = {toF(generated::kContact.centerInFootM), toF(generated::kContact.axisInFoot),
                                     float(generated::kContact.radiusM), float(generated::kContact.supportWidthM),
                                     float(generated::kContact.filletRadiusM), float(generated::kContact.nominalTiltRad)};
struct ContactF {
  ContactStatus status = ContactStatus::INVALID_INPUT;
  SupportMode support = SupportMode::UNKNOWN;
  Vec3f cylinderCenterM{}, cylinderAxis{}, radialDown{}, referenceM{}, stripEndAM{}, stripEndBM{}, lowestCoreM{};
  float axisTiltRad = 0;
};
ContactResult toD(const ContactF& c);
float edgeContactLift(float axisTiltRad);
// normal: unit length, base_link.
ContactF contactForward(LegId leg, const JointsF& q, const Vec3f& normal);
struct ContactIkF {
  ContactStatus status = ContactStatus::INVALID_INPUT;
  Reachability reachability = Reachability::UNKNOWN;
  JointsF joints{};
  ContactF contact{};
  float residualM = 0;
  ContactBranch branch{};
  uint8_t solutions = 0;
};
// Analytic contact IK, ground normal +Z in base_link (contactInverseKinematics).
ContactIkF contactInverse(LegId leg, const Vec3f& target, const JointsF& seed, float toleranceM, bool requireNominalStrip);

// world_from_base_link with the world translation kept double: a point is
// moved to base_link by subtracting in double, then rotating in float.
struct PoseF { double translation[3]; float rotation[3][3]; };
// validBodyPose on the float image of the rotation; false leaves out unusable.
bool poseF(const BodyPose& pose, PoseF& out);
Vec3f toBase(const PoseF& pose, const Vector3& world);
Vector3 toWorld(const PoseF& pose, const Vec3f& base);
// World +Z in base_link: the third row of the rotation (R^T e_z).
inline Vec3f groundNormal(const PoseF& pose) { return {pose.rotation[2][0], pose.rotation[2][1], pose.rotation[2][2]}; }

// Tilted-ground contact IK (Newton), TiltedContactIk.h semantics.
struct TiltedIkF {
  TiltedIkResult::Status status = TiltedIkResult::Status::INVALID_INPUT;
  JointsF joints{};
  ContactF contact{};
  float residualM = 0, condition = 0;
  unsigned iterations = 0, newtonSteps = 0, fkEvaluations = 0, solves = 0;
};
TiltedIkResult toD(const TiltedIkF& r);
// normal: unit length; seed: a jointsF image.
TiltedIkF tiltedInverse(LegId leg, const Vec3f& target, const Vec3f& normal, const JointsF& seed,
                        float toleranceM, unsigned maxIterations, bool requireNominalStrip);
// Edge contact on a validated pose (worldEdgeContactInverseKinematicsTilted).
TiltedIkF edgeTiltedInverse(LegId leg, const Vector3& worldTarget, const PoseF& pose, JointsF seed,
                            float toleranceM, unsigned maxIterations);
} } }
#endif
