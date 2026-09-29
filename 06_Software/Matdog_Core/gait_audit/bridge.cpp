// Host-only stable numeric bridge. No framework/device dependencies.
#include "Locomotion.h"
using namespace matdog::motion;
static int encode(const LocomotionFrame& f,double* out);
extern "C" int g4_frame(const double* p,double s,double rate,double accel,double terminal,
                         const double* previous,double* out) {
  GaitParameters g;g.type=p[0]==0?GaitType::WALK:GaitType::TROT;
  g.heightM=p[1];g.liftM=p[2];g.duty=p[3];g.advanceXM=p[4];g.advanceYM=p[5];g.yawRad=p[6];g.swayXM=p[7];g.maxCondition=p[8];
  LocomotionFrame old;
  if(previous){old.target.valid=true;for(unsigned i=0;i<4;++i){old.target.legs[i]={previous[i*3],previous[i*3+1],previous[i*3+2]};old.branches[i]={static_cast<int8_t>(previous[12+i*2]),static_cast<int8_t>(previous[13+i*2])};}}
  auto f=solveGait(g,gaitCartesian(g,s,rate,accel,terminal),previous?&old:nullptr);
  return encode(f,out);
}

static int encode(const LocomotionFrame& f,double* out) {
  if(!f.target.valid){out[0]=f.failedLeg;return static_cast<int>(f.status);}
  unsigned n=0;
  for(unsigned i=0;i<4;++i){auto q=f.target.legs[i];out[n++]=q.hip;out[n++]=q.upper;out[n++]=q.lower;}
  for(auto b:f.branches){out[n++]=b.hip;out[n++]=b.elbow;}
  for(auto q:f.velocityRadS){out[n++]=q.hip;out[n++]=q.upper;out[n++]=q.lower;}
  for(auto q:f.accelerationRadS2){out[n++]=q.hip;out[n++]=q.upper;out[n++]=q.lower;}
  const auto& c=f.cartesian;
  for(auto v:c.feet){out[n++]=v.x;out[n++]=v.y;out[n++]=v.z;}
  for(auto v:c.velocity){out[n++]=v.x;out[n++]=v.y;out[n++]=v.z;}
  for(auto v:c.acceleration){out[n++]=v.x;out[n++]=v.y;out[n++]=v.z;}
  out[n++]=c.body.translationM.x;out[n++]=c.body.translationM.y;out[n++]=c.body.translationM.z;
  for(auto& row:c.body.rotation)for(double v:row)out[n++]=v;
  for(auto phase:c.phases){out[n++]=phase.phase;out[n++]=phase.stance;out[n++]=phase.boundary;}
  for(double v:f.condition)out[n++]=v;
  out[n++]=f.minJointMarginRad;out[n++]=f.maxResidualM;
  return 0;
}

extern "C" int g4_lifecycle(const double* p,double period,double stopAt,const double* times,unsigned count,double* output,int* statuses,int* states) {
  Locomotion x;StartupObservation o;const auto& d=canonicalStandDefinition();auto stand=generateStandTarget(d);
  o.joints=sampleStandPath(d,0,stand.target.legs).target;o.body.translationM.z=d.lowBodyHeightM;o.condition=StartupCondition::VERIFIED_LOW_STANCE;
  o.fresh=o.stationary=o.fourFootSupportConfirmed=o.flatGroundConfirmed=o.collisionClearanceReviewed=o.supportAndLoadReviewed=true;
  if(!x.apply(MotionEvent::ENABLE)||!x.beginStartup(o,{1,2}))return 1;
  for(unsigned i=0;i<3;++i)if(x.nextStartup().status!=TimedStatus::OK)return 2;
  MotionCommand c;c.gait.type=p[0]==0?GaitType::WALK:GaitType::TROT;c.gait.heightM=p[1];c.gait.liftM=p[2];c.gait.duty=p[3];
  c.gait.advanceXM=p[4];c.gait.advanceYM=p[5];c.gait.yawRad=p[6];c.gait.swayXM=p[7];c.gait.maxCondition=p[8];c.periodS=period;
  if(!x.start(c,0,100*period))return 3;
  bool stopping=false;
  for(unsigned i=0;i<count;++i){auto f=x.sample(times[i]);statuses[i]=encode(f,output+110*i);states[i]=static_cast<int>(x.state());
    if(!stopping&&times[i]>=stopAt){x.requestStop();stopping=true;}
    if(!f.target.valid)return 4;
  }return 0;
}
