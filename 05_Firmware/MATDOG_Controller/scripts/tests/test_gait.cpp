#include "../../src/motion/Locomotion.h"
#include <cmath>
#include <cstdio>
#include <limits>
#include <initializer_list>
using namespace matdog::motion;
static unsigned checks=0,failures=0;
#define CHECK(x) do {++checks;if(!(x)){++failures;std::printf("FAIL %d: %s\n",__LINE__,#x);}}while(0)
static double v(LegJointAngles q,unsigned j){return j==0?q.hip:j==1?q.upper:q.lower;}
static bool near(double a,double b,double eps=1e-9){return std::abs(a-b)<=eps;}
static StartupObservation ready(){StartupObservation o;const auto& d=canonicalStandDefinition();auto s=generateStandTarget(d);
  o.joints=sampleStandPath(d,0,s.target.legs).target;o.body.translationM.z=d.lowBodyHeightM;o.condition=StartupCondition::VERIFIED_LOW_STANCE;
  o.fresh=o.stationary=o.fourFootSupportConfirmed=o.flatGroundConfirmed=o.collisionClearanceReviewed=o.supportAndLoadReviewed=true;return o;}
static Locomotion standing(){Locomotion x;CHECK(x.apply(MotionEvent::ENABLE));CHECK(x.beginStartup(ready(),{1,2}));
  for(unsigned i=0;i<3;++i){CHECK(x.nextStartup().status==TimedStatus::OK);}
  CHECK(x.state()==LocomotionState::STAND);return x;}
int main(){
 const double nan=std::numeric_limits<double>::quiet_NaN();GaitParameters p;
 CHECK(validGaitParameters(p));CHECK(!validGaitPeriod(0));CHECK(!validGaitPeriod(nan));CHECK(!validGaitPeriod(1e-300));
 for(unsigned field=0;field<8;++field){auto b=p;double* fields[]={&b.heightM,&b.liftM,&b.duty,&b.advanceXM,&b.advanceYM,&b.yawRad,&b.swayXM,&b.maxCondition};*fields[field]=nan;CHECK(!validGaitParameters(b));}
 auto bad=p;bad.duty=.749;CHECK(!validGaitParameters(bad));bad=p;bad.duty=1;CHECK(!validGaitParameters(bad));
 bad=p;bad.type=static_cast<GaitType>(2);CHECK(!validGaitParameters(bad));bad=p;bad.liftM=-1;CHECK(!validGaitParameters(bad));
 CHECK(!gaitCartesian(p,-1,1,0).valid);CHECK(!gaitCartesian(p,nan,1,0).valid);CHECK(!gaitCartesian(p,1e7,1,0).valid);
 const double offsets[4]={0,.5,.75,.25};for(unsigned i=0;i<4;++i)CHECK(gaitOffset(p.type,static_cast<LegId>(i))==offsets[i]);
 const unsigned order[4]={2,1,3,0};
 for(unsigned n=0;n<4;++n){unsigned swing=0;for(unsigned i=0;i<4;++i){auto a=legPhase(p,static_cast<LegId>(i),n*.25+.15);if(!a.stance){++swing;CHECK(i==order[n]);}}CHECK(swing==1);}
 for(unsigned kind=0;kind<2;++kind){p.type=kind?GaitType::TROT:GaitType::WALK;p.duty=kind?.6:.8;
  p.heightM=.1;p.swayXM=kind?0:.004;
  LocomotionFrame previous;bool have=false;
  for(unsigned n=0;n<=600;++n){double s=n/200.;auto cart=gaitCartesian(p,s,1,0);auto f=solveGait(p,cart,have?&previous:nullptr);CHECK(f.target.valid);if(!f.target.valid)break;
   auto rerun=solveGait(p,cart,have?&previous:nullptr);auto slow=solveGait(p,gaitCartesian(p,s,.5,0),have?&previous:nullptr);
   CHECK(f.maxResidualM<1e-9&&f.minJointMarginRad>0);CHECK(!f.dynamicStabilityProven);
   unsigned support=0;for(unsigned i=0;i<4;++i){CHECK(cart.phases[i].phase>=0&&cart.phases[i].phase<1);CHECK(cart.feet[i].z>=-1e-14);
    if(cart.phases[i].stance)++support;
    auto contact=contactForwardKinematics(static_cast<LegId>(i),f.target.legs[i]);Vector3 world;CHECK(pointToWorld(cart.body,contact.referenceM,world));
    CHECK(near(world.x,cart.feet[i].x)&&near(world.y,cart.feet[i].y)&&near(world.z,cart.feet[i].z));
    if(have&&cart.phases[i].stance&&previous.cartesian.phases[i].stance&&cart.phases[i].phase>=previous.cartesian.phases[i].phase){CHECK(near(cart.feet[i].x,previous.cartesian.feet[i].x));CHECK(near(cart.feet[i].y,previous.cartesian.feet[i].y));}
    for(unsigned j=0;j<3;++j){CHECK(v(f.target.legs[i],j)==v(rerun.target.legs[i],j));CHECK(std::isfinite(v(f.accelerationRadS2[i],j)));
     CHECK(near(v(f.velocityRadS[i],j),2*v(slow.velocityRadS[i],j)));CHECK(near(v(f.accelerationRadS2[i],j),4*v(slow.accelerationRadS2[i],j)));}
   }CHECK(support>=(kind?2u:3u));
   if(kind){CHECK(cart.phases[0].stance==cart.phases[2].stance);CHECK(cart.phases[1].stance==cart.phases[3].stance);}
   previous=f;have=true;
  }
  for(unsigned i=0;i<4;++i){double td=2-gaitOffset(p.type,static_cast<LegId>(i));
   for(double s:{td-1+p.duty,td}){auto f=gaitCartesian(p,s,1,0);CHECK(near(f.feet[i].z,0));CHECK(near(f.velocity[i].z,0));CHECK(near(f.acceleration[i].z,0));}}
  auto base=solveGait(p,gaitCartesian(p,1.2,1,0));auto branch=base;branch.branches[0].elbow*=-1;
  CHECK(solveGait(p,gaitCartesian(p,1.2,1,0),&branch).status==GaitStatus::BRANCH_CHANGE);
  auto b=p;b.maxCondition=1;CHECK(solveGait(b,gaitCartesian(b,1.2,1,0)).status==GaitStatus::ILL_CONDITIONED);
  b=p;b.heightM=1;CHECK(solveGait(b,gaitCartesian(b,1.2,1,0)).status==GaitStatus::IK_UNREACHABLE);
  auto limitCart=gaitCartesian(p,1.2,1,0);const auto& model=*legModel(LegId::LF);
  double r=footContactModel().radiusM,reach=-model.lowerOrigin.z+std::hypot(model.footOrigin.x,model.footOrigin.z+r)+r;
  Vector3 extended={model.hipOrigin.x,model.hipOrigin.y+model.upperOrigin.y+model.footOrigin.y,model.hipOrigin.z-reach};
  pointToWorld(limitCart.body,extended,limitCart.feet[0]);CHECK(solveGait(p,limitCart).status==GaitStatus::JOINT_LIMIT);
  for(double boundary:{1.,1.05,1.25,1.3,1.5,1.55,1.75,1.8,2.}){
   auto center=solveGait(p,gaitCartesian(p,boundary,1,0));
   auto left=solveGait(p,gaitCartesian(p,boundary-1e-8,1,0),&center),right=solveGait(p,gaitCartesian(p,boundary+1e-8,1,0),&center);
   CHECK(left.target.valid&&right.target.valid);
   for(unsigned i=0;i<4;++i)for(unsigned j=0;j<3;++j){CHECK(near(v(left.target.legs[i],j),v(right.target.legs[i],j),1e-6));CHECK(near(v(left.velocityRadS[i],j),v(right.velocityRadS[i],j),1e-5));CHECK(near(v(left.accelerationRadS2[i],j),v(right.accelerationRadS2[i],j),.001));}
  }
  auto cart=gaitCartesian(p,1.2,1,0);cart.feet[2].x=nan;CHECK(solveGait(p,cart).status==GaitStatus::NONFINITE);
  auto phaseBad=gaitCartesian(p,1.2,1,0);phaseBad.phases[0].phase=nan;CHECK(solveGait(p,phaseBad).status==GaitStatus::NONFINITE);
  auto contactBad=base;CHECK(assessGait(contactBad,p.type,{true,false,true})==GaitStatus::CONTACT_INVALID);
  auto supportBad=base;CHECK(assessGait(supportBad,p.type,{true,true,false})==(kind?GaitStatus::OK:GaitStatus::SUPPORT_INVALID));
  CHECK(assessGait(base,p.type,{false,true,true})==GaitStatus::COLLISION&&!base.target.valid);
 }
 // Every start retains the G3 evidence gate; public completion cannot forge it.
 Locomotion gate;MotionCommand command;CHECK(!gate.start(command,0,1));CHECK(!gate.apply(MotionEvent::STAND_COMPLETE));
 CHECK(gate.apply(MotionEvent::ENABLE));CHECK(!gate.beginStartup({}, {1,2}));CHECK(!gate.start(command,0,1));
 CommandWatchdog w;CHECK(!w.configure(0));CHECK(w.configure(1));CHECK(w.check(0)==CommandStatus::INVALID);
 command.sequence=1;CHECK(w.accept(command,0)==CommandStatus::FRESH);CHECK(w.check(1)==CommandStatus::FRESH);CHECK(w.check(1.01)==CommandStatus::STALE);
 CHECK(w.accept(command,1.01)==CommandStatus::INVALID);command.sequence=2;command.stampS=2;CHECK(w.accept(command,1.01)==CommandStatus::INVALID);
 command.stampS=1.01;command.gait.advanceXM=0;CHECK(w.accept(command,1.01)==CommandStatus::ZERO);
 command.sequence=3;command.gait.advanceXM=.01;CHECK(w.accept(command,1.01)==CommandStatus::CHANGED);CHECK(w.check(.5)==CommandStatus::INVALID);
 for(unsigned kind=0;kind<2;++kind)for(unsigned when=0;when<4;++when){
  auto x=standing();MotionCommand c;c.gait.type=kind?GaitType::TROT:GaitType::WALK;c.gait.duty=kind?.6:.8;c.gait.heightM=.1;c.gait.swayXM=kind?0:.004;c.sequence=1;
  CHECK(x.start(c,0,100));CHECK(x.state()==LocomotionState::GAIT_START);CHECK(!x.apply(MotionEvent::STAND_COMPLETE));
  CHECK(x.nextStartup().status!=TimedStatus::OK&&x.state()==LocomotionState::GAIT_START);
  auto first=x.sample(0);CHECK(first.target.valid);auto stand=generateStandTarget(canonicalStandDefinition());
  for(unsigned i=0;i<4;++i)for(unsigned j=0;j<3;++j){CHECK(near(v(first.target.legs[i],j),v(stand.target.legs[i],j)));CHECK(v(first.velocityRadS[i],j)==0);}
  double stopTime=when==0?.4:when==1?1.6:when==2?3.15:4.0;auto prev=first;bool ended=false;
  for(unsigned n=1;n<=1000;++n){double t=n*.01;
   if(t>=stopTime&&!ended){CHECK(x.requestStop());CHECK(!x.apply(MotionEvent::STOP_COMPLETE));ended=true;}
   auto f=x.sample(t);CHECK(f.target.valid);if(!f.target.valid)break;
   for(unsigned i=0;i<4;++i)for(unsigned j=0;j<3;++j)CHECK(std::abs(v(f.target.legs[i],j)-v(prev.target.legs[i],j))<.05);
   prev=f;if(x.state()==LocomotionState::STAND){for(unsigned i=0;i<4;++i)for(unsigned j=0;j<3;++j){CHECK(near(v(f.target.legs[i],j),v(stand.target.legs[i],j)));CHECK(near(v(f.velocityRadS[i],j),0));CHECK(near(v(f.accelerationRadS2[i],j),0));}break;}
  }CHECK(x.state()==LocomotionState::STAND);
  c.stampS=20;c.sequence=2;CHECK(x.start(c,20,100));auto restart=x.sample(20);CHECK(restart.target.valid);
  for(unsigned i=0;i<4;++i)CHECK(near(restart.cartesian.feet[i].x,prev.cartesian.feet[i].x));
  CHECK(x.apply(MotionEvent::DISABLE));CHECK(x.state()==LocomotionState::OFF);CHECK(!x.sample(21).target.valid);
 }
 auto timeout=standing();command={};command.sequence=1;CHECK(timeout.start(command,0,.1));CHECK(timeout.sample(0).target.valid);CHECK(timeout.sample(.11).target.valid);CHECK(timeout.state()==LocomotionState::STOPPING);
 auto changed=standing();CHECK(changed.start(command,0,1));command.sequence=2;command.gait.type=GaitType::TROT;CHECK(changed.command(command,0)==CommandStatus::CHANGED);CHECK(changed.state()==LocomotionState::STOPPING);
 auto held=standing();CHECK(held.apply(MotionEvent::STOP));CHECK(!held.sample(0).target.valid);CHECK(!held.requestStop());CHECK(held.apply(MotionEvent::STOP_COMPLETE));CHECK(held.state()==LocomotionState::IDLE);
 auto idle=standing();MotionCommand z;z.gait.advanceXM=0;CHECK(!idle.start(z,0,1));CHECK(idle.state()==LocomotionState::STAND);
 std::printf("GAIT_HOST = %s: %u checks, %u failures\n",failures?"FAIL":"PASS",checks,failures);return failures?1:0;
}
