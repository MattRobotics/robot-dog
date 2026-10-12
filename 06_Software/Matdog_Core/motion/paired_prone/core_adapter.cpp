// Offline ABI around existing G1/G2/G5-A; no servo or Controller dependencies.
#include "LegKinematics.h"
#include "FootContact.h"
#include "BodyPose.h"
#include "TiltedContactIk.h"
#include <cmath>
using namespace matdog::motion;
static BodyPose pose(double pitch,double x,double z) {
  BodyPose b;b.translationM={x,0,z};const double c=std::cos(pitch),s=std::sin(pitch);
  b.rotation[0][0]=c;b.rotation[0][2]=s;b.rotation[2][0]=-s;b.rotation[2][2]=c;
  return b;
}
extern "C" int contact_fk(int leg,const double*q,double p,double x,double z,double*out) {
  const LegJointAngles a{q[0],q[1],q[2]};const BodyPose b=pose(p,x,z);
  const auto c=contactForwardKinematics(static_cast<LegId>(leg),a,{-std::sin(p),0,std::cos(p)});
  if(c.status!=ContactStatus::OK)return static_cast<int>(c.status);
  Vector3 w{},low{};pointToWorld(b,c.referenceM,w);pointToWorld(b,c.lowestCoreM,low);
  out[0]=w.x;out[1]=w.y;out[2]=w.z;out[3]=low.x;out[4]=low.y;out[5]=low.z;out[6]=c.axisTiltRad;
  return 0;
}
extern "C" int contact_ik(int leg,const double*seed,const double*target,double p,double x,double z,double*out) {
  const BodyPose b=pose(p,x,z);const LegJointAngles q{seed[0],seed[1],seed[2]};
  if(std::abs(p)<1e-12) {
    ContactIkOptions options;options.kinematics.seed=q;options.kinematics.toleranceM=1e-10;
    const auto result=worldContactInverseKinematics(static_cast<LegId>(leg),{target[0],target[1],target[2]},b,options);
    if(result.status!=WorldContactIkResult::Status::OK)return 100+static_cast<int>(result.status);
    const auto &a=result.baseResult;out[0]=a.joints.hip;out[1]=a.joints.upper;out[2]=a.joints.lower;
    out[3]=a.residualM;out[4]=a.branch.hip;out[5]=a.branch.elbow;
  } else {
    TiltedIkOptions options;options.seed=q;options.toleranceM=1e-10;options.maxIterations=40;
    const auto result=worldContactInverseKinematicsTilted(static_cast<LegId>(leg),{target[0],target[1],target[2]},b,options);
    if(result.status!=TiltedIkResult::Status::OK)return 200+static_cast<int>(result.status);
    const auto&a=result.base;out[0]=a.joints.hip;out[1]=a.joints.upper;out[2]=a.joints.lower;
    out[3]=a.residualM;out[4]=0;out[5]=0;
  }
  return 0;
}
extern "C" int roundtrip_g1(int leg,const double*q,double*out) {
  const LegJointAngles a{q[0],q[1],q[2]};const auto f=forwardKinematics(static_cast<LegId>(leg),a);
  if(f.status!=KinematicsStatus::OK)return static_cast<int>(f.status);
  IkOptions options;options.seed=a;const auto result=inverseKinematics(static_cast<LegId>(leg),f.footOriginM,options);
  if(result.status!=KinematicsStatus::OK)return 100+static_cast<int>(result.status);
  out[0]=result.joints.hip;out[1]=result.joints.upper;out[2]=result.joints.lower;out[3]=result.residualM;
  return 0;
}
extern "C" int origin_fk(int leg,const double*q,double*out) {
  const auto f=forwardKinematics(static_cast<LegId>(leg),{q[0],q[1],q[2]});
  if(f.status!=KinematicsStatus::OK)return static_cast<int>(f.status);
  out[0]=f.footOriginM.x;out[1]=f.footOriginM.y;out[2]=f.footOriginM.z;
  for(unsigned i=0;i<3;++i)for(unsigned j=0;j<3;++j)out[3+3*i+j]=f.baseFromFootRotation[i][j];
  return 0;
}
