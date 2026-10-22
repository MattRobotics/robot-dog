#include "SimpleMotion.h"
#include "../motion/PairedProneData.h"
#include "../motion/GaitTiming.h"
#include "../motion/TiltCompensation.h"
#include "../motion/ImuAttitude.h"
#include <algorithm>
#include <cmath>
#include <cstring>
namespace matdog { namespace simple {
using namespace motion;
namespace {
constexpr double kArrivalRad=12*kTickRad; // V25 native positioning allowance
constexpr double kStrideM[]={.020,.030,.040};
constexpr double kPeriodS=7,kRampS=7;
int magnitude(int x){return x&0x7fff;}
bool contains(const collision::Box& b,const JointTargetFrame& q,double extra=0){
  if(!q.valid)return false;
  for(unsigned i=0;i<12;++i)if(!(jointSlot(q,i)>=b.lo[i]+extra&&jointSlot(q,i)<=b.hi[i]-extra))return false;
  return true;
}
JointTargetFrame routePoint(unsigned knot){
  JointTargetFrame q;if(knot>251)return q;
  const double* p=paired_data::kKnots[knot].p;
  for(unsigned l=0;l<4;++l){q.legs[l]={((l==0||l==3)?-1:1)*p[7],p[l<2?2:0],std::fmax((l<2?-88.5:-89.)*3.14159265358979323846/180,p[l<2?3:1])};}
  q.valid=true;return q;
}
JointTargetFrame cellEnd(unsigned cell){
  const unsigned e=collision::kCellEdge[cell];const double a=collision::kCellEnd[cell];
  auto q=routePoint(collision::kPhaseKnots[e]);auto to=routePoint(collision::kPhaseKnots[e+1]);
  for(unsigned i=0;i<12;++i)jointSlot(q,i)+=a*(jointSlot(to,i)-jointSlot(q,i));
  return q;
}
}
const char* name(State s){switch(s){case State::READY:return "READY";case State::MOVING:return "MOVING";case State::STANDING:return "STANDING";case State::WALKING:return "WALKING";case State::STOPPED:return "STOPPED";case State::FAULT:return "FAULT";}return "FAULT";}
const char* SimpleMotion::phase() const {
  static const char* const p[]={"IDLE","ACQUIRE","REFERENCE","ENABLE","DISENGAGE","ALIGN","STAND_ROUTE","HEIGHT","HOLD","WALK_STANDARD","WALK_SETTLE"};
  return p[static_cast<unsigned>(phase_)];
}
void SimpleMotion::stop(const char* reason) { stopReason_.store(reason);stop_.store(true,std::memory_order_release); }
void SimpleMotion::fault(const char* r){enterOff(true,r);}
void SimpleMotion::enterOff(bool fault,const char* r){
  state_=fault?State::FAULT:State::STOPPED;faultLatched_=fault;reason_=r;
  if(!fault)operatorStopped_=strcmp(r,"BOOTING")!=0;
  phase_=Phase::IDLE;offPending_=true;offSent_=false;offMask_=0;offNext_=0;
  walkRequested_=false;walkStopping_=false;rollCorrection_=pitchCorrection_=0;
}
void SimpleMotion::serviceOff(){
  if(!port_||!offPending_)return;
  if(!offSent_){offSent_=port_->off();return;}
  if(port_->verifyOff(offNext_))offMask_|=1u<<offNext_;
  else {offMask_&=~(1u<<offNext_);offSent_=false;}
  offNext_=(offNext_+1)%13;
  if(offMask_==0x1fff){energized_=false;offPending_=false;}
}
bool SimpleMotion::fresh(uint64_t now) const {
  for(const auto& f:fb_)if(!f.valid||(now>=f.stampUs&&now-f.stampUs>250000)||(f.stampUs>now&&f.stampUs-now>15000))return false;
  return true;
}
bool SimpleMotion::pointSafe(const JointTargetFrame& q) const {
  if(!q.valid)return false;
  for(unsigned i=0;i<12;++i){const double v=jointSlot(q,i);if(!std::isfinite(v)||v<input_.minRad[i]||v>input_.maxRad[i])return false;}
  if(contains(collision::kEntry,q)||contains(collision::kWalk,q)||contains(collision::kHeight,q))return true;
  for(const auto& b:collision::kStandCells)if(contains(b,q))return true;
  return false;
}
bool SimpleMotion::planInitialReference(){
  // Restart after STOP/boot from any already proved configuration region.
  // This is physical path selection from measured Q, never route tracking.
  if(pointSafe(reference_)&&contains(collision::kHeight,reference_)){initialRoute_=InitialRoute::HEIGHT;return true;}
  if(contains(collision::kEntry,reference_)){initialRoute_=InitialRoute::ENTRY;return true;}
  if(!pointSafe(reference_))return false;
  if(contains(collision::kWalk,reference_)){initialRoute_=InitialRoute::WALK;return true;}
  for(unsigned c=collision::kStandCellCount;c>0;--c)if(contains(collision::kStandCells[c-1],reference_)){
    initialRoute_=InitialRoute::ROUTE;initialCell_=c-1;return true;
  }
  return false;
}
bool SimpleMotion::transitionSafe(const JointTargetFrame& a,const JointTargetFrame& b,const collision::Box& box) const {
  // All independent joint progress between two endpoints is enclosed. No
  // assumption that twelve motors have the same instantaneous response.
  return contains(box,a)&&contains(box,b)&&pointSafe(b);
}
bool SimpleMotion::heightTarget(double h,JointTargetFrame& q) const {
  if(!std::isfinite(h)||h<.060||h>.150)return false; // effective C4 workspace
  auto d=canonicalStandDefinition();d.lowBodyHeightM=h/2;d.standBodyHeightM=h;
  const auto s=generateStandTarget(d);q=s.target;return s.status==StandStatus::OK&&pointSafe(q)&&contains(collision::kHeight,q);
}
const char* SimpleMotion::stand(double h,uint64_t now){
  if(stopping())return "SAFE_OFF_IN_PROGRESS";
  if(faultLatched_)return "FAULT_STOP_REQUIRED";
  if(!input_.ready)return input_.refusal;
  if(!input_.powerOk)return "POWER_NOT_READY";
  if(!input_.imuValid)return "IMU_UNAVAILABLE";
  if(state_==State::MOVING||state_==State::WALKING)return "MOTION_BUSY";
  JointTargetFrame q;if(!heightTarget(h,q))return "HEIGHT_OUTSIDE_MATDOG_WORKSPACE";
  requestedHeight_=h;walkRequested_=false;heartbeat(now);
  if(state_==State::STANDING){
    JointTargetFrame actual;actual.valid=true;for(unsigned i=0;i<12;++i)jointSlot(actual,i)=fb_[i].q;
    if(!fresh(now)||!transitionSafe(actual,q,collision::kHeight))return "HEIGHT_PATH_NOT_CLEAR";
    state_=State::MOVING;phase_=Phase::HEIGHT;reason_="HEIGHT_CHANGE";
    if(!send(q,kStandDrive,now))return reason_;
  }else{state_=State::MOVING;phase_=Phase::ACQUIRE;acquire_=0;reason_="ACQUIRING_REAL_POSE";}
  return nullptr;
}
const char* SimpleMotion::walk(unsigned stride,uint64_t now){
  if(stride>=3)return "STANDARD_STRIDE_REQUIRED";
  if(state_==State::WALKING)return "ALREADY_WALKING";
  // One user command can acquire, energize, stand at 150 mm and then walk.
  const char* r=stand(.150,now);if(r)return r;
  stride_=stride;walkRequested_=true;return nullptr;
}
void SimpleMotion::walkStop(){
  walkRequested_=false;
  if(state_==State::WALKING&&!walkStopping_){
    stopCycle_=std::fmax(1.,std::ceil(cycles_-1e-12));
    stopClock_=stopCycle_*kPeriodS+.5*kRampS;walkStopping_=true;
    reason_="WALK_STOPPING";
  }
}
bool SimpleMotion::send(const JointTargetFrame& q,Drive drive,uint64_t now,bool holdReference){
  uint16_t raw[12]{};double distance=0;
  if(!holdReference&&!pointSafe(q)){fault("TARGET_LIMIT_OR_COLLISION");return false;}
  for(unsigned i=0;i<12;++i){
    if(holdReference)raw[i]=fb_[i].raw;
    else if(!port_->resolve(i,jointSlot(q,i),raw[i])){fault("TARGET_CONVERSION_FAILED");return false;}
    distance=std::fmax(distance,std::abs(jointSlot(q,i)-fb_[i].q));
  }
  if(stop_.load())return false;
  if(!port_->frame(raw,drive)){fault("SYNC_WRITE_FAILED");return false;}
  commandedDrive_=drive;goal_=q;for(unsigned i=0;i<12;++i)commandedRaw_[i]=raw[i];goalSentUs_=now;
  // Native speed/acceleration transit estimate, conservative 2x + V25's
  // 12 s positioning timeout. This is a completion deadline, not a lag test.
  const double speed=drive.speed*kTickRad,acc=drive.acc*100*kTickRad;
  goalDeadlineUs_=now+static_cast<uint64_t>((12+2*(distance/speed+2*speed/acc))*1e6);
  return true;
}
bool SimpleMotion::arrived(uint64_t now) const {
  if(!fresh(now)||now-goalSentUs_<100000)return false;
  for(unsigned i=0;i<12;++i)if(fb_[i].stampUs<=goalSentUs_||std::abs(fb_[i].q-jointSlot(goal_,i))>kArrivalRad||magnitude(fb_[i].speed)>50)return false;
  return true;
}
void SimpleMotion::measureHeight(){
  // Native encoders + measured IMU orientation + flat-ground assumption.
  // No absolute height sensor exists, especially on a suspended fixture.
  heightValid_=false;if(!fresh(nowUs_)||!input_.imuValid)return;
  Attitude a;if(!tiltFromQuaternion(input_.quaternion[0],input_.quaternion[1],input_.quaternion[2],input_.quaternion[3],a))return;
  double sum=0,lo=1,hi=-1;
  const auto body=bodyPoseFromRollPitch({0,0,0},a.rollRad,a.pitchRad);
  for(unsigned l=0;l<4;++l){LegJointAngles q{fb_[l*3].q,fb_[l*3+1].q,fb_[l*3+2].q};
    const auto f=contactForwardKinematics(static_cast<LegId>(l),q,{a.upBase[0],a.upBase[1],a.upBase[2]});
    Vector3 world;if(f.status!=ContactStatus::OK||!pointToWorld(body,f.referenceM,world))return;
    const double h=-world.z;sum+=h;lo=std::fmin(lo,h);hi=std::fmax(hi,h);
  }
  actualHeight_=sum/4; heightValid_=hi-lo<.015&&(phase_==Phase::HOLD||phase_==Phase::HEIGHT);
}
void SimpleMotion::nextStand(uint64_t now){
  JointTargetFrame actual;actual.valid=true;for(unsigned i=0;i<12;++i)jointSlot(actual,i)=fb_[i].q;
  if(phase_==Phase::LOWER){
    auto q=reference_,head=routePoint(0);for(unsigned l=0;l<4;++l)q.legs[l].lower=head.legs[l].lower;
    if(!transitionSafe(actual,q,collision::kEntry)){fault("DISENGAGE_PATH_NOT_CLEAR");return;}
    phase_=Phase::ALIGN;send(q,kStandDrive,now);return;
  }
  if(phase_==Phase::ALIGN){
    auto q=routePoint(0);if(!transitionSafe(actual,q,collision::kEntry)){fault("ALIGN_PATH_NOT_CLEAR");return;}
    phase_=Phase::ROUTE;routeCell_=0;send(q,kStandDrive,now);return;
  }
  if(phase_==Phase::ROUTE&&routeCell_<collision::kStandCellCount){
    const unsigned c=routeCell_++;auto q=cellEnd(c);
    if(!transitionSafe(actual,q,collision::kStandCells[c])){fault("STAND_CLEARANCE_UNAVAILABLE");return;}
    send(q,kStandDrive,now);return;
  }
  if(phase_==Phase::ROUTE){
    JointTargetFrame q;if(!heightTarget(requestedHeight_,q)||!transitionSafe(actual,q,collision::kHeight)){fault("HEIGHT_PATH_NOT_CLEAR");return;}
    phase_=Phase::HEIGHT;send(q,kStandDrive,now);return;
  }
  if(phase_==Phase::HEIGHT||phase_==Phase::WALK_HOLD){
    phase_=Phase::HOLD;state_=State::STANDING;reason_="STAND_REACHED";
    if(walkRequested_)startWalk(now);
  }
}
void SimpleMotion::startWalk(uint64_t now){
  GaitParameters g;g.heightM=.15;g.liftM=.015;g.duty=.8;g.advanceXM=kStrideM[stride_];
  walkPrevious_=solveGait(g,gaitCartesian(g,0,0,0));
  if(!pointSafe(walkPrevious_.target)||!contains(collision::kWalk,walkPrevious_.target)){fault("WALK_IK_OR_WORKSPACE");return;}
  walkStartUs_=now;walkStopping_=false;stopCycle_=-1;cycles_=0;
  phase_=Phase::WALK;state_=State::WALKING;reason_="WALK_STANDARD";
}
void SimpleMotion::compensate(LocomotionFrame& f,uint64_t now){
  imuLimited_=false;Attitude a;
  if(!input_.imuValid||now<input_.imuStampUs||now-input_.imuStampUs>200000||
     !tiltFromQuaternion(input_.quaternion[0],input_.quaternion[1],input_.quaternion[2],input_.quaternion[3],a)){imuLimited_=true;return;}
  // A small proportional BODY correction, slew limited, no servo PID or
  // integral loop. The same target generator owns nominal + correction.
  auto step=[](double old,double tilt){const double desired=std::abs(tilt)<.005?0:std::clamp(-.5*tilt,-.020,.020);return old+std::clamp(desired-old,-.0008,.0008);};
  const double roll=step(rollCorrection_,a.rollRad),pitch=step(pitchCorrection_,a.pitchRad);
  for(unsigned trial=0;trial<5;++trial){const double scale=std::ldexp(1.,-static_cast<int>(trial));
    auto b=bodyPoseFromRollPitch(f.cartesian.body.translationM,roll*scale,pitch*scale);
    JointTargetFrame q;q.valid=true;
    for(unsigned l=0;l<4;++l){TiltedIkOptions opt;opt.seed=f.target.legs[l];opt.maxIterations=8;opt.requireNominalStrip=false;
      const auto ik=worldContactInverseKinematicsTilted(static_cast<LegId>(l),f.cartesian.feet[l],b,opt);
      if(ik.status!=TiltedIkResult::Status::OK){q.valid=false;break;}q.legs[l]=ik.base.joints;}
    const auto& box=phase_==Phase::WALK?collision::kWalk:collision::kHeight;
    if(pointSafe(q)&&contains(box,q)){f.target=q;rollCorrection_=roll*scale;pitchCorrection_=pitch*scale;imuLimited_=trial!=0;return;}
  }
  // If no in-workspace correction exists, the nominal safe target survives.
  rollCorrection_=pitchCorrection_=0;imuLimited_=true;
}
void SimpleMotion::generateWalk(uint64_t now){
  const double t=(now-walkStartUs_)/1e6;double rate=0,acc=0;
  if(walkStopping_&&t>=stopClock_){
    const double u=std::fmin(1.,(t-stopClock_)/(2*kPeriodS)),s=1-u;const auto law=quinticTimeLaw(u,2*kPeriodS);
    cycles_=stopCycle_+1-s*s*s*s*(5-6*s+2*s*s);rate=law.valid?(1-law.progress)/kPeriodS:0;acc=-law.velocity/kPeriodS;
  }else if(t<kRampS){const double u=t/kRampS;const auto law=quinticTimeLaw(u,kRampS);
    cycles_=kRampS/kPeriodS*(2.5*std::pow(u,4)-3*std::pow(u,5)+std::pow(u,6));rate=law.progress/kPeriodS;acc=law.velocity/kPeriodS;
  }else{cycles_=(t-.5*kRampS)/kPeriodS;rate=1/kPeriodS;}
  GaitParameters g;g.heightM=.15;g.liftM=.015;g.duty=.8;g.advanceXM=kStrideM[stride_];
  auto cart=gaitCartesian(g,cycles_,rate,acc,walkStopping_&&t>=stopClock_?stopCycle_+1:-1);
  auto f=solveGait(g,cart,&walkPrevious_);
  if(!f.target.valid||!contains(collision::kWalk,f.target)||!pointSafe(f.target)){fault("WALK_IK_OR_WORKSPACE");return;}
  walkPrevious_=f;compensate(f,now);
  if(!send(f.target,kWalkDrive,now))return;
  if(walkStopping_&&t>=stopClock_+2*kPeriodS){phase_=Phase::WALK_HOLD;state_=State::MOVING;reason_="WALK_SETTLING";}
}
void SimpleMotion::update(uint64_t now){
  nowUs_=now;
  if(stop_.exchange(false,std::memory_order_acq_rel))enterOff(false,stopReason_.load());
  if(offPending_){serviceOff();return;}
  if(!engaged()&&state_==State::READY&&(!input_.ready||!input_.powerOk||!input_.imuValid)){state_=State::STOPPED;reason_=input_.refusal;}
  if(state_==State::STOPPED&&input_.ready&&input_.powerOk&&input_.imuValid&&offVerified()&&!operatorStopped_){state_=State::READY;reason_="READY";}
  if(!input_.ready&&!engaged())return;
  if(!nextUs_)nextUs_=now;
  if(now<nextUs_)return;
  const uint64_t gap=lastReleaseUs_?now-lastReleaseUs_:0;
  maxGapUs_=std::max(maxGapUs_,gap);lastReleaseUs_=now;++releases_;
  const auto skipped=(now-nextUs_)/kPeriodUs;missed_+=skipped;nextUs_+=(skipped+1)*kPeriodUs;
  if(energized_&&gap>100000){fault("MOTION_DEADLINE_LOST");serviceOff();return;}
  if(engaged()&&(!input_.ready||!input_.powerOk||!input_.authorityOk)){fault(input_.refusal);serviceOff();return;}
  if(engaged()&&(!heartbeatUs_||now-heartbeatUs_>3000000)){fault("OPERATOR_LINK_LOST");serviceOff();return;}
  if(energized_&&(!input_.imuValid||now-input_.imuStampUs>600000)){fault("IMU_STALE");serviceOff();return;}
  // Four single-block reads per 40 ms: whole population every 120 ms, at
  // most 8 ms wait. One 2-block control read each 120 ms adds <=4 ms.
  for(unsigned n=0;n<4;++n){const unsigned i=pollNext_++%12;Feedback f;
    if(port_->read(i,false,f)){
      f.torque=fb_[i].torque;f.goal=fb_[i].goal;f.goalSpeed=fb_[i].goalSpeed;f.acc=fb_[i].acc;f.controlStampUs=fb_[i].controlStampUs;fb_[i]=f;failures_[i]=0;
      if(energized_&&(f.status||f.temp>=70||f.voltage<40||f.voltage>140)){fault("SERVO_NATIVE_ALARM_OR_ELECTRICAL_LIMIT");break;}
      // Hard physical limits, no comparison to a target's time history.
      if(energized_&&phase_!=Phase::ENABLE&&phase_!=Phase::LOWER&&phase_!=Phase::ALIGN&&
         (f.q<input_.minRad[i]-2*kTickRad||f.q>input_.maxRad[i]+2*kTickRad)){fault("MEASURED_MECHANICAL_LIMIT");break;}
    }else if(++failures_[i]>=3&&energized_){fault("SERVO_COMMUNICATION_LOST");break;}
    if(stop_.load())break;
  }
  if(offPending_||stop_.load()){if(stop_.exchange(false))enterOff(false,stopReason_.load());serviceOff();return;}
  if(phase_==Phase::ACQUIRE){
    Feedback f;if(!port_->read(acquire_,true,f)){fault("INITIAL_SERVO_READ_FAILED");return;}
    fb_[acquire_]=f;
    if(f.torque!=0||f.status||f.temp>=70||f.voltage<40||f.voltage>140||magnitude(f.speed)>50){fault("INITIAL_SERVO_NOT_READY");return;}
    if(++acquire_==12){
      reference_.valid=true;for(unsigned i=0;i<12;++i)jointSlot(reference_,i)=fb_[i].q;
      if(!fresh(now)||!planInitialReference()){fault("INITIAL_POSE_OUTSIDE_SAFE_ENTRY");return;}
      phase_=Phase::REFERENCE;
    }
    return;
  }
  if(phase_==Phase::REFERENCE){
    // Write the real measured reference with torque OFF before enabling.
    reference_.valid=true;for(unsigned i=0;i<12;++i)jointSlot(reference_,i)=fb_[i].q;
    if(!planInitialReference()){fault("INITIAL_POSE_OUTSIDE_SAFE_ENTRY");return;}
    if(!send(reference_,kStandDrive,now,true))return;
    phase_=Phase::ENABLE;enable_=0;return;
  }
  if(phase_==Phase::ENABLE){
    // Verify each exact GoalPosition/RAM profile BEFORE enabling that servo.
    Feedback f;
    if(!port_->read(enable_,true,f)||f.goal!=commandedRaw_[enable_]||f.goalSpeed!=kStandDrive.speed||f.acc!=kStandDrive.acc||f.torque!=0||std::abs(f.q-jointSlot(reference_,enable_))>kArrivalRad){fault("REFERENCE_CHANGED_BEFORE_ENABLE");return;}
    energized_=true;offMask_=0; // even a failed enable may have applied; SAFE_OFF required
    if(!port_->enable(enable_)){fault("TORQUE_ENABLE_UNCERTAIN");return;}
    if(++enable_==12){
      if(initialRoute_==InitialRoute::ENTRY){phase_=Phase::LOWER;nextStand(now);}
      else if(initialRoute_==InitialRoute::ROUTE){phase_=Phase::ROUTE;routeCell_=initialCell_;nextStand(now);}
      else if(initialRoute_==InitialRoute::HEIGHT){phase_=Phase::ROUTE;routeCell_=collision::kStandCellCount;nextStand(now);}
      else {
        // A stopped gait returns through the proved whole WALK region to
        // 150 mm, then the ordinary height branch reaches the requested Q.
        JointTargetFrame q,actual;actual.valid=true;
        for(unsigned i=0;i<12;++i)jointSlot(actual,i)=fb_[i].q;
        if(!heightTarget(.150,q)||!transitionSafe(actual,q,collision::kWalk)){fault("HEIGHT_PATH_NOT_CLEAR");return;}
        phase_=Phase::ROUTE;routeCell_=collision::kStandCellCount;send(q,kStandDrive,now);
      }
    }
    return;
  }
  if(releases_%3==0){
    const unsigned i=controlNext_++%12;Feedback f;
    if(port_->read(i,true,f)){
      fb_[i]=f;controlFailures_[i]=0;
      if(energized_&&(f.torque!=1||f.goal!=commandedRaw_[i]||f.goalSpeed!=commandedDrive_.speed||f.acc!=commandedDrive_.acc)){fault("SERVO_TORQUE_OR_GOAL_LOST");return;}
    }else if(energized_&&++controlFailures_[i]>=3){fault("SERVO_CONTROL_READ_LOST");return;}
  }
  if(energized_&&!fresh(now)){fault("SERVO_FEEDBACK_STALE");return;}
  measureHeight();
  if(phase_==Phase::WALK){generateWalk(now);return;}
  if(state_==State::MOVING){
    if(arrived(now)){nextStand(now);return;}
    if(goalDeadlineUs_&&now>goalDeadlineUs_){fault("NATIVE_TARGET_NOT_REACHED");return;}
  }
  if(phase_==Phase::HOLD&&releases_%3==0){
    GaitParameters g;g.heightM=requestedHeight_;auto f=solveGait(g,gaitHold(g,0,requestedHeight_,0));
    if(f.target.valid){compensate(f,now);send(f.target,kStandDrive,now);}
  }
}
} }
