// Exercise the existing G4 scheduler and IK; no driver, bus or authority code.
#include "Gait.h"
#include <cmath>
using namespace matdog::motion;
static LocomotionFrame previous;
static bool hasPrevious=false;
extern "C" void walk_reset(){hasPrevious=false;}
extern "C" int walk_frame(double cycle,double rate,double accel,double holdShift,double shiftVelocity,double shiftAcceleration,int hold,double originX,const double*offset,double*out) {
  GaitParameters p;p.type=GaitType::WALK;p.heightM=.15;p.liftM=.005;p.duty=.8;
  p.advanceXM=.005;p.swayXM=.04;
  auto s=hold?gaitHold(p,cycle,p.heightM,holdShift,0.,shiftVelocity,0.,shiftAcceleration):gaitCartesian(p,cycle,rate,accel,1.);
  // Explicit STL-derived support-reference correction, external/offline only.
  // All 17-mesh checks retain 1nm ground tolerance. Core model is unchanged.
  s.body.translationM.x+=originX;
  for(unsigned i=0;i<4;++i){s.feet[i].z+=offset[i];s.feet[i].x+=originX;}
  auto f=solveGait(p,s,hasPrevious?&previous:nullptr);
  if(f.status!=GaitStatus::OK)return static_cast<int>(f.status);
  previous=f;hasPrevious=true;
  for(unsigned i=0;i<4;++i){out[i*3]=f.target.legs[i].hip;out[i*3+1]=f.target.legs[i].upper;out[i*3+2]=f.target.legs[i].lower;
    out[12+i*3]=s.feet[i].x;out[13+i*3]=s.feet[i].y;out[14+i*3]=s.feet[i].z;out[24+i]=s.phases[i].stance?1:0;
    out[30+i*2]=f.branches[i].hip;out[31+i*2]=f.branches[i].elbow;
    out[38+i*3]=f.velocityRadS[i].hip;out[39+i*3]=f.velocityRadS[i].upper;out[40+i*3]=f.velocityRadS[i].lower;
    out[50+i*3]=f.accelerationRadS2[i].hip;out[51+i*3]=f.accelerationRadS2[i].upper;out[52+i*3]=f.accelerationRadS2[i].lower;}
  out[28]=s.body.translationM.x;out[29]=s.body.translationM.z;out[62]=f.maxResidualM;out[63]=f.minJointMarginRad;
  return 0;
}
