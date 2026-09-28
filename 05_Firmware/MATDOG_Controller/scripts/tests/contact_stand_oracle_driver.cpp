// Host-only text adapter. Production motion code contains no I/O.
#include "../../src/motion/StandTrajectory.h"
#include <iostream>
#include <iomanip>
#include <string>
using namespace matdog::motion;
static void vec(Vector3 v) { std::cout<<'['<<v.x<<','<<v.y<<','<<v.z<<']'; }
static void qvec(LegJointAngles q) { vec({q.hip,q.upper,q.lower}); }
static void contact(ContactResult c) {
  std::cout<<"{\"status\":"<<unsigned(c.status)<<",\"mode\":"<<unsigned(c.support)<<",\"tilt\":"<<c.axisTiltRad;
  const char* keys[]={"center","axis","radial","reference","end_a","end_b","lowest"};
  Vector3 points[]={c.cylinderCenterM,c.cylinderAxis,c.radialDown,c.referenceM,c.stripEndAM,c.stripEndBM,c.lowestCoreM};
  for(unsigned i=0;i<7;++i){std::cout<<",\""<<keys[i]<<"\":";vec(points[i]);}
  std::cout<<'}';
}
static void sample(const StandSample& s) {
  std::cout<<"{\"status\":"<<unsigned(s.status)<<",\"valid\":"<<(s.target.valid?"true":"false")
           <<",\"index\":"<<s.target.sequence<<",\"height\":"<<s.bodyHeightM
           <<",\"residual\":"<<s.maxContactResidualM<<",\"drift\":"<<s.maxContactDriftM<<",\"legs\":[";
  for(unsigned i=0;i<4;++i){
    if(i)std::cout<<',';
    std::cout<<"{\"q\":";qvec(s.target.legs[i]);
    std::cout<<",\"delta\":";qvec(s.jointDeltaRad[i]);
    std::cout<<",\"margin\":";qvec(s.jointLimitMarginRad[i]);
    std::cout<<",\"contact\":";vec(s.contactsWorldM[i]);
    std::cout<<",\"branch\":["<<int(s.branches[i].hip)<<','<<int(s.branches[i].elbow)<<"]}";
  }
  std::cout<<"]}";
}
int main(int argc,char** argv) {
  std::cout<<std::setprecision(17);
  if(argc==2 && std::string(argv[1])=="--stand") {
    const auto& d=canonicalStandDefinition();
    ContactLockedStand trajectory;
    if(trajectory.initialize(d)!=StandStatus::OK)return 2;
    std::cout<<"{\"stand\":";sample(generateStandTarget(d));std::cout<<",\"frames\":[";
    for(unsigned i=0;i<d.samples;++i){if(i)std::cout<<',';sample(trajectory.next());}
    const auto& m=trajectory.metrics();
    std::cout<<"],\"metrics\":{\"max_delta\":[";
    for(unsigned i=0;i<4;++i){if(i)std::cout<<',';qvec(m.maxAbsJointDeltaRad[i]);}
    std::cout<<"],\"branch_changes\":[";
    for(unsigned i=0;i<4;++i){if(i)std::cout<<',';std::cout<<m.branchChanges[i];}
    std::cout<<"],\"max_joint_delta\":"<<m.maxJointDeltaRad<<",\"max_residual\":"<<m.maxContactResidualM
             <<",\"max_drift\":"<<m.maxContactDriftM<<",\"min_margin\":"<<m.minJointLimitMarginRad
             <<",\"samples\":"<<m.emittedSamples<<"}}\n";
    return 0;
  }
  if(argc!=1)return 2;
  unsigned leg,nominal;LegJointAngles q{};Vector3 target{},normal{};ContactIkOptions options;
  while(std::cin>>leg>>q.hip>>q.upper>>q.lower>>target.x>>target.y>>target.z
        >>options.kinematics.seed.hip>>options.kinematics.seed.upper>>options.kinematics.seed.lower
        >>normal.x>>normal.y>>normal.z>>nominal){
    if(leg>3 || nominal>1)return 2;
    options.requireNominalStrip=nominal!=0;
    const auto ik=contactInverseKinematics(static_cast<LegId>(leg),target,options);
    std::cout<<"{\"fk\":";contact(contactForwardKinematics(static_cast<LegId>(leg),q,normal));
    std::cout<<",\"ik_status\":"<<unsigned(ik.status)<<",\"q\":";qvec(ik.joints);
    std::cout<<",\"residual\":"<<ik.residualM<<",\"branch\":["<<int(ik.branch.hip)<<','<<int(ik.branch.elbow)
             <<"],\"solutions\":"<<unsigned(ik.solutions)<<"}\n";
  }
  return std::cin.eof()?0:2;
}
