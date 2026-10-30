#include "LegKinematics.h"
#include <cstdio>
#include <cmath>
using namespace matdog::motion;
int main(){
  const double poses[][3]={{0,0,0},{0.1,0.6,-1.2},{-0.3,1.0,-0.9},{0.4,0.2,-0.5}};
  const char* n[4]={"lf","rf","rh","lh"};
  int bad=0;
  for(int l=0;l<4;++l) for(auto&p:poses){
    LegJointAngles q{p[0],p[1],p[2]}; auto f=forwardKinematics((LegId)l,q);
    std::printf("FK %s %.17g %.17g %.17g %.17g %.17g | %d\n",n[l],p[0],p[1],p[2],f.footOriginM.x,f.footOriginM.y,(int)f.status);
    std::printf("FKZ %s %.17g\n",n[l],f.footOriginM.z);
    IkOptions o; o.seed=q; auto ik=inverseKinematics((LegId)l,f.footOriginM,o);
    double e=std::abs(ik.joints.hip-q.hip)+std::abs(ik.joints.upper-q.upper)+std::abs(ik.joints.lower-q.lower);
    std::printf("IK %s status=%d err=%.3e resid=%.3e\n",n[l],(int)ik.status,e,ik.residualM); if(ik.status!=KinematicsStatus::OK||e>1e-7) ++bad;
  }
  std::printf("SHA %s\nIK_ROUNDTRIP_BAD %d\n",geometrySourceSha256(),bad); return bad;
}
