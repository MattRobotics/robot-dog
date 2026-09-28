// Synthetic offline observation and text-only oracle adapter; no acquisition I/O.
#include "../../src/motion/StandTransition.h"
#include <iostream>
#include <iomanip>
#include <cstdlib>
using namespace matdog::motion;
static void vec(Vector3 v){std::cout<<'['<<v.x<<','<<v.y<<','<<v.z<<']';}
static void qvec(LegJointAngles q){vec({q.hip,q.upper,q.lower});}
static void sample(const TimedStandSample& s,MotionState state){
  std::cout<<"{\"status\":"<<unsigned(s.status)<<",\"valid\":"<<(s.geometry.target.valid?"true":"false")
           <<",\"state\":"<<unsigned(state)<<",\"time\":"<<s.timeS<<",\"u\":"<<s.normalizedTime
           <<",\"s\":"<<s.progress<<",\"phase\":"<<unsigned(s.phase)<<",\"height\":"<<s.body.translationM.z
           <<",\"residual\":"<<s.geometry.maxContactResidualM<<",\"drift\":"<<s.geometry.maxContactDriftM<<",\"legs\":[";
  for(unsigned i=0;i<4;++i){
    if(i)std::cout<<',';
    std::cout<<"{\"q\":";qvec(s.geometry.target.legs[i]);std::cout<<",\"v\":";qvec(s.velocityRadS[i]);
    std::cout<<",\"a\":";qvec(s.accelerationRadS2[i]);std::cout<<",\"margin\":";qvec(s.geometry.jointLimitMarginRad[i]);
    std::cout<<",\"contact\":";vec(s.geometry.contactsWorldM[i]);
    std::cout<<",\"branch\":["<<int(s.geometry.branches[i].hip)<<','<<int(s.geometry.branches[i].elbow)<<"]}";
  }
  std::cout<<"]}";
}
int main(int argc,char** argv){
  if(argc!=3)return 2;
  char *end=nullptr;double duration=std::strtod(argv[1],&end);if(*end)return 2;
  unsigned long intervals=std::strtoul(argv[2],&end,10);if(*end||intervals>1000000)return 2;
  StartupObservation o;const auto& d=canonicalStandDefinition();auto stand=generateStandTarget(d);
  o.joints=sampleStandPath(d,0,stand.target.legs).target;o.body.translationM.z=d.lowBodyHeightM;
  o.condition=StartupCondition::VERIFIED_LOW_STANCE;
  o.fresh=o.stationary=o.fourFootSupportConfirmed=o.flatGroundConfirmed=o.collisionClearanceReviewed=o.supportAndLoadReviewed=true;
  StandTransition t;t.apply(MotionEvent::ENABLE);
  if(!t.begin(o,{duration,static_cast<uint32_t>(intervals)}))return 3;
  std::cout<<std::setprecision(17)<<"{\"frames\":[";
  for(unsigned long i=0;i<=intervals;++i){
    if(i)std::cout<<',';
    auto s=t.next();if(!s.geometry.target.valid)return 4;sample(s,t.state());
  }
  const auto& m=t.metrics();std::cout<<"],\"metrics\":{\"velocity\":[";
  for(unsigned i=0;i<4;++i){if(i)std::cout<<',';qvec(m.peakVelocityRadS[i]);}
  std::cout<<"],\"acceleration\":[";for(unsigned i=0;i<4;++i){if(i)std::cout<<',';qvec(m.peakAccelerationRadS2[i]);}
  std::cout<<"],\"branch_changes\":[";for(unsigned i=0;i<4;++i){if(i)std::cout<<',';std::cout<<m.branchChanges[i];}
  std::cout<<"],\"duration\":"<<m.durationS<<",\"margin\":"<<m.minJointLimitMarginRad
           <<",\"residual\":"<<m.maxContactResidualM<<",\"drift\":"<<m.maxContactDriftM<<",\"samples\":"<<m.emitted<<"}}\n";
}
