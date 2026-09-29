#include "Locomotion.h"
#include <cmath>
namespace matdog { namespace motion {
namespace {
bool same(const MotionCommand& a,const MotionCommand& b){const auto& x=a.gait;const auto& y=b.gait;
  return a.periodS==b.periodS&&x.type==y.type&&x.heightM==y.heightM&&x.liftM==y.liftM&&x.duty==y.duty&&
    x.advanceXM==y.advanceXM&&x.advanceYM==y.advanceYM&&x.yawRad==y.yawRad&&x.swayXM==y.swayXM&&x.maxCondition==y.maxCondition;
}
bool zero(const MotionCommand& c){return c.gait.advanceXM==0&&c.gait.advanceYM==0&&c.gait.yawRad==0;}
double integral(double u){return u*u*u*u*(2.5+u*(-3+u));}
Vector3 rotate(const BodyPose& b,Vector3 v){return {b.rotation[0][0]*v.x+b.rotation[0][1]*v.y,b.rotation[1][0]*v.x+b.rotation[1][1]*v.y,v.z};}
void relocate(CartesianSample& s,const BodyPose& origin){
  Vector3 t;pointToWorld(origin,s.body.translationM,t);s.body.translationM=t;
  double c=s.body.rotation[0][0],si=s.body.rotation[1][0];
  double nc=origin.rotation[0][0]*c+origin.rotation[0][1]*si,ns=origin.rotation[1][0]*c+origin.rotation[1][1]*si;
  s.body.rotation[0][0]=nc;s.body.rotation[0][1]=-ns;s.body.rotation[1][0]=ns;s.body.rotation[1][1]=nc;
  s.bodyVelocity=rotate(origin,s.bodyVelocity);s.bodyAcceleration=rotate(origin,s.bodyAcceleration);
  for(unsigned i=0;i<4;++i){pointToWorld(origin,s.feet[i],t);s.feet[i]=t;s.velocity[i]=rotate(origin,s.velocity[i]);s.acceleration[i]=rotate(origin,s.acceleration[i]);}
}
}
bool CommandWatchdog::configure(double t){*this={};if(!std::isfinite(t)||t<=0)return false;timeout_=t;configured_=true;return true;}
CommandStatus CommandWatchdog::accept(const MotionCommand& c,double now){
  if(!configured_||!std::isfinite(now)||now<0||(clockSeen_&&now<lastNow_)||!std::isfinite(c.stampS)||c.stampS<0||c.stampS>now||
     !validGaitParameters(c.gait)||!validGaitPeriod(c.periodS)||(received_&&(c.sequence<=command_.sequence||c.stampS<command_.stampS)))return CommandStatus::INVALID;
  lastNow_=now;clockSeen_=true;
  if(now-c.stampS>timeout_)return CommandStatus::STALE;
  bool changed=received_&&!same(c,command_);command_=c;received_=true;
  return zero(c)?CommandStatus::ZERO:changed?CommandStatus::CHANGED:CommandStatus::FRESH;
}
CommandStatus CommandWatchdog::check(double now){
  if(!configured_||!received_||!std::isfinite(now)||now<0||(clockSeen_&&now<lastNow_))return CommandStatus::INVALID;
  lastNow_=now;clockSeen_=true;return now-command_.stampS>timeout_?CommandStatus::STALE:zero(command_)?CommandStatus::ZERO:CommandStatus::FRESH;
}
bool Locomotion::apply(MotionEvent e){
  if(e==MotionEvent::DISABLE||e==MotionEvent::FAULT){startup_.apply(e);state_=MotionState::OFF;gaitActive_=false;previous_={};origin_={};status_=GaitStatus::CANCELLED;return true;}
  if(e==MotionEvent::STOP&&(state_==MotionState::GAIT_START||state_==MotionState::WALK||state_==MotionState::TROT||(state_==MotionState::STOPPING&&gaitActive_)))return requestStop();
  if(state_==MotionState::GAIT_START||state_==MotionState::WALK||state_==MotionState::TROT||(state_==MotionState::STOPPING&&gaitActive_))return false;
  if(!startup_.apply(e))return false;
  state_=startup_.state();return true;
}
bool Locomotion::beginStartup(const StartupObservation& o,const TimingSpec& t){
  if(state_!=MotionState::IDLE||!startup_.begin(o,t))return false;
  state_=startup_.state();return true;
}
TimedStandSample Locomotion::nextStartup(){if(state_!=MotionState::STAND_TRANSITION)return {};auto f=startup_.next();state_=startup_.state();return f;}
bool Locomotion::start(const MotionCommand& c,double now,double timeout){
  if(state_!=MotionState::STAND||!watchdog_.configure(timeout))return false;
  commandStatus_=watchdog_.accept(c,now);if(commandStatus_!=CommandStatus::FRESH)return false;
  active_=c;startTime_=lastTime_=now;stopCycle_=-1;stopRequested_=holdOnlyStop_=false;
  auto initial=gaitHold(c.gait,0,canonicalStandDefinition().standBodyHeightM,0);relocate(initial,origin_);
  previous_=solveGait(c.gait,initial);status_=previous_.status;if(!previous_.target.valid)return false;
  state_=MotionState::GAIT_START;gaitActive_=true;return true;
}
CommandStatus Locomotion::command(const MotionCommand& c,double now){
  commandStatus_=watchdog_.accept(c,now);
  if(commandStatus_!=CommandStatus::FRESH)requestStop();
  return commandStatus_;
}
bool Locomotion::requestStop(){
  if(state_!=MotionState::GAIT_START&&state_!=MotionState::WALK&&state_!=MotionState::TROT&&state_!=MotionState::STOPPING)return false;
  if(!stopRequested_){stopRequested_=true;double elapsed=(lastTime_-startTime_)/active_.periodS;
    holdOnlyStop_=elapsed<1;stopCycle_=elapsed<3?1:std::ceil(elapsed-2);
  }
  state_=MotionState::STOPPING;return true;
}
LocomotionFrame Locomotion::sample(double now){
  LocomotionFrame failure;
  if(state_!=MotionState::GAIT_START&&state_!=MotionState::WALK&&state_!=MotionState::TROT&&state_!=MotionState::STOPPING)return failure;
  if(!std::isfinite(now)||now<lastTime_){failure.status=GaitStatus::STATE_ERROR;status_=failure.status;return failure;}
  // Expiry starts the stop from the last emitted valid configuration, never
  // from an unobserved phase. The resulting path remains defined across gaps.
  commandStatus_=watchdog_.check(now);if(commandStatus_!=CommandStatus::FRESH)requestStop();
  const auto& p=active_.gait;double T=active_.periodS,t=(now-startTime_)/T;
  double initialShift=p.type==GaitType::WALK?-p.swayXM:0;
  CartesianSample cart;bool finished=false;
  if(t<=1){auto law=quinticTimeLaw(t,T);double dz=p.heightM-canonicalStandDefinition().standBodyHeightM;
    cart=gaitHold(p,0,canonicalStandDefinition().standBodyHeightM+dz*law.progress,initialShift*law.progress,dz*law.velocity,initialShift*law.velocity,dz*law.acceleration,initialShift*law.acceleration);
  }else if(holdOnlyStop_){double u=std::fmin(1.,t-1);auto law=quinticTimeLaw(u,T);double dz=canonicalStandDefinition().standBodyHeightM-p.heightM;
    cart=gaitHold(p,0,p.heightM+dz*law.progress,initialShift*(1-law.progress),dz*law.velocity,-initialShift*law.velocity,dz*law.acceleration,-initialShift*law.acceleration);finished=u==1;
  }else if(t<3){double u=(t-1)*.5;auto law=quinticTimeLaw(u,2*T);
    cart=gaitCartesian(p,2*integral(u),law.progress/T,law.velocity/T);
  }else{
    double stopStart=stopCycle_+2;
    if(stopRequested_&&t>=stopStart){double terminal=stopCycle_+1;
      if(t<stopStart+2){double u=(t-stopStart)*.5;auto law=quinticTimeLaw(u,2*T);
        cart=gaitCartesian(p,stopCycle_+2*(u-integral(u)),(1-law.progress)/T,-law.velocity/T,terminal);
      }else{double u=std::fmin(1.,t-stopStart-2);auto law=quinticTimeLaw(u,T);double dz=canonicalStandDefinition().standBodyHeightM-p.heightM;
        cart=gaitHold(p,terminal,p.heightM+dz*law.progress,initialShift*(1-law.progress),dz*law.velocity,-initialShift*law.velocity,dz*law.acceleration,-initialShift*law.acceleration);finished=u==1;}
    }else{cart=gaitCartesian(p,t-2,1/T,0);if(!stopRequested_)state_=p.type==GaitType::WALK?MotionState::WALK:MotionState::TROT;}
  }
  relocate(cart,origin_);auto out=solveGait(p,cart,&previous_);status_=out.status;
  if(!out.target.valid){state_=MotionState::OFF;startup_.apply(MotionEvent::FAULT);return out;}
  out.target.sequence=previous_.target.sequence+1;previous_=out;lastTime_=now;
  if(finished){state_=MotionState::STAND;gaitActive_=false;origin_=out.cartesian.body;origin_.translationM.z=0;}
  return out;
}
} }
