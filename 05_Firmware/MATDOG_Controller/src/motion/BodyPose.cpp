#include "BodyPose.h"
#include "LegKinematicsInternal.h"
#include "MotionHotPath.h"
#include <cmath>
namespace matdog { namespace motion {
namespace {
bool finite(Vector3 p) { return std::isfinite(p.x)&&std::isfinite(p.y)&&std::isfinite(p.z); }
// Exactly level (yaw and translation only): the rotation entries are tested
// as stored, on a validated pose.
bool flatRotation(const BodyPose& pose) {
  return std::abs(pose.rotation[2][0])<=1e-12&&std::abs(pose.rotation[2][1])<=1e-12&&std::abs(pose.rotation[2][2]-1.0)<=1e-12;
}
}
// The rotation is checked as the FP32 core uses it: its float image is
// orthonormal with determinant 1 within kRotationTolerance.
MATDOG_MOTION_HOT bool detail::poseF(const BodyPose& p, PoseF& out) {
  if (!finite(p.translationM)) return false;
  out.translation[0]=p.translationM.x;out.translation[1]=p.translationM.y;out.translation[2]=p.translationM.z;
  for(unsigned i=0;i<3;++i) for(unsigned j=0;j<3;++j) {
    if(!std::isfinite(p.rotation[i][j])) return false;
    out.rotation[i][j]=static_cast<float>(p.rotation[i][j]);
  }
  const auto& r=out.rotation;
  for(unsigned i=0;i<3;++i) for(unsigned j=0;j<3;++j) {
    float dot=0; for(unsigned k=0;k<3;++k)dot+=r[i][k]*r[j][k];
    if(!std::isfinite(dot)||std::abs(dot-(i==j?1.0f:0.0f))>kRotationTolerance)return false;
  }
  const float det=r[0][0]*(r[1][1]*r[2][2]-r[1][2]*r[2][1])-
      r[0][1]*(r[1][0]*r[2][2]-r[1][2]*r[2][0])+r[0][2]*(r[1][0]*r[2][1]-r[1][1]*r[2][0]);
  return std::abs(det-1.0f)<kRotationTolerance;
}
MATDOG_MOTION_HOT detail::Vec3f detail::toBase(const PoseF& pose, const Vector3& world) {
  const float dx=static_cast<float>(world.x-pose.translation[0]),dy=static_cast<float>(world.y-pose.translation[1]),
              dz=static_cast<float>(world.z-pose.translation[2]);
  const auto& r=pose.rotation;
  return {r[0][0]*dx+r[1][0]*dy+r[2][0]*dz,
          r[0][1]*dx+r[1][1]*dy+r[2][1]*dz,
          r[0][2]*dx+r[1][2]*dy+r[2][2]*dz};
}
MATDOG_MOTION_HOT Vector3 detail::toWorld(const PoseF& pose, const Vec3f& base) {
  const auto& r=pose.rotation;
  return {pose.translation[0]+double(r[0][0]*base.x+r[0][1]*base.y+r[0][2]*base.z),
          pose.translation[1]+double(r[1][0]*base.x+r[1][1]*base.y+r[1][2]*base.z),
          pose.translation[2]+double(r[2][0]*base.x+r[2][1]*base.y+r[2][2]*base.z)};
}
bool validBodyPose(const BodyPose& p) { detail::PoseF pose; return detail::poseF(p,pose); }
bool pointToWorld(const BodyPose& pose, Vector3 base, Vector3& world) {
  if(!validBodyPose(pose)||!finite(base)){world={};return false;}
  const auto& r=pose.rotation;
  Vector3 p={pose.translationM.x+r[0][0]*base.x+r[0][1]*base.y+r[0][2]*base.z,
             pose.translationM.y+r[1][0]*base.x+r[1][1]*base.y+r[1][2]*base.z,
             pose.translationM.z+r[2][0]*base.x+r[2][1]*base.y+r[2][2]*base.z};
  if(!finite(p)){world={};return false;}
  world=p;return true;
}
bool pointToBase(const BodyPose& pose, Vector3 world, Vector3& base) {
  if(!validBodyPose(pose)||!finite(world)){base={};return false;}
  Vector3 d={world.x-pose.translationM.x,world.y-pose.translationM.y,world.z-pose.translationM.z};
  const auto& r=pose.rotation;
  Vector3 p={r[0][0]*d.x+r[1][0]*d.y+r[2][0]*d.z,
             r[0][1]*d.x+r[1][1]*d.y+r[2][1]*d.z,
             r[0][2]*d.x+r[1][2]*d.y+r[2][2]*d.z};
  if(!finite(p)){base={};return false;}
  base=p;return true;
}
bool supportsFlatContactIk(const BodyPose& pose) { return validBodyPose(pose)&&flatRotation(pose); }
WorldContactIkResult worldContactInverseKinematics(LegId leg,Vector3 target,
                                          const BodyPose& pose,const ContactIkOptions& options) {
  WorldContactIkResult result;detail::PoseF f;
  if(!detail::poseF(pose,f)||!finite(target))return result;
  if(!flatRotation(pose)) {result.status=WorldContactIkResult::Status::UNSUPPORTED_ORIENTATION;return result;}
  result.baseResult=contactInverseKinematics(leg,detail::toD(detail::toBase(f,target)),options);
  result.status=WorldContactIkResult::Status::IK_FAILURE;
  if(result.baseResult.status!=ContactStatus::OK)return result;
  result.achievedWorldM=detail::toWorld(f,detail::toF(result.baseResult.contact.referenceM));
  result.status=WorldContactIkResult::Status::OK;return result;
}
} }
