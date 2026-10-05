#include "BodyPose.h"
#include <cmath>
namespace matdog { namespace motion {
namespace {
bool finite(Vector3 p) { return std::isfinite(p.x)&&std::isfinite(p.y)&&std::isfinite(p.z); }
}
bool validBodyPose(const BodyPose& p) {
  if (!finite(p.translationM)) return false;
  const auto& r=p.rotation;
  for(unsigned i=0;i<3;++i) for(unsigned j=0;j<3;++j) if(!std::isfinite(r[i][j])) return false;
  for(unsigned i=0;i<3;++i) for(unsigned j=0;j<3;++j) {
    double dot=0; for(unsigned k=0;k<3;++k)dot+=r[i][k]*r[j][k];
    if(!std::isfinite(dot)||std::abs(dot-(i==j?1.0:0.0))>1e-10)return false;
  }
  const double det=r[0][0]*(r[1][1]*r[2][2]-r[1][2]*r[2][1])-
      r[0][1]*(r[1][0]*r[2][2]-r[1][2]*r[2][0])+r[0][2]*(r[1][0]*r[2][1]-r[1][1]*r[2][0]);
  return std::abs(det-1)<1e-10;
}
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
bool supportsFlatContactIk(const BodyPose& pose) {
  return validBodyPose(pose)&&std::abs(pose.rotation[2][0])<=1e-12&&
         std::abs(pose.rotation[2][1])<=1e-12&&std::abs(pose.rotation[2][2]-1.0)<=1e-12;
}
WorldContactIkResult worldContactInverseKinematics(LegId leg,Vector3 target,
                                          const BodyPose& pose,const ContactIkOptions& options) {
  WorldContactIkResult result;Vector3 base;
  if(!pointToBase(pose,target,base))return result;
  if(!supportsFlatContactIk(pose)) {result.status=WorldContactIkResult::Status::UNSUPPORTED_ORIENTATION;return result;}
  result.baseResult=contactInverseKinematics(leg,base,options);
  result.status=WorldContactIkResult::Status::IK_FAILURE;
  if(result.baseResult.status!=ContactStatus::OK)return result;
  if(!pointToWorld(pose,result.baseResult.contact.referenceM,result.achievedWorldM))return result;
  result.status=WorldContactIkResult::Status::OK;return result;
}
} }
