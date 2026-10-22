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
constexpr double kPeriodS=kWalkPeriodS,kRampS=kWalkRampS;
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
  segmentBox_=nullptr;segmentTo_.valid=false;settleUs_=0;imuStartUs_=0;
  phase_=Phase::IDLE;offPending_=true;offSent_=false;offMask_=0;offNext_=0;
  walkRequested_=false;walkStopping_=false;pendingGoalValid_=false;rollCorrection_=pitchCorrection_=0;
}
void SimpleMotion::serviceOff(){
  if(!port_||!offPending_)return;
  if(!offSent_){offSent_=port_->off();return;}
  if(port_->verifyOff(offNext_))offMask_|=1u<<offNext_;
  else {offMask_&=~(1u<<offNext_);offSent_=false;}
  offNext_=(offNext_+1)%13;
  if(offMask_==0x1fff){energized_=false;offPending_=false;}
}
bool SimpleMotion::serviceStop(){
  if(!stop_.exchange(false,std::memory_order_acq_rel))return false;
  enterOff(false,stopReason_.load());serviceOff();return true;
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
  if(!std::isfinite(h)||h<kMinStandM||h>.150)return false; // effective C4 workspace
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
  if(state_==State::STANDING){
    JointTargetFrame actual;actual.valid=true;for(unsigned i=0;i<12;++i)jointSlot(actual,i)=fb_[i].q;
    if(!fresh(now)||!transitionSafe(actual,q,collision::kHeight))return "HEIGHT_PATH_NOT_CLEAR";
    requestedHeight_=h;walkRequested_=false;heartbeat(now);
    state_=State::MOVING;phase_=Phase::HEIGHT;reason_="HEIGHT_CHANGE";
    pendingGoal_=q;pendingGoalValid_=true; // intent only; TX at the next Motion release
  }else{requestedHeight_=h;walkRequested_=false;heartbeat(now);state_=State::MOVING;phase_=Phase::ACQUIRE;acquire_=0;reason_="ACQUIRING_REAL_POSE";}
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
  if(serviceStop())return false;
  // Identical RAM goals need no retransmission, especially in HOLD. Always
  // write the measured reference with torque OFF before enabling.
  if(!holdReference&&goal_.valid&&drive.speed==commandedDrive_.speed&&drive.acc==commandedDrive_.acc&&
     std::equal(raw,raw+12,commandedRaw_)){goal_=q;return true;}
  if(!port_->frame(raw,drive)){if(!serviceStop())fault("SYNC_WRITE_FAILED");return false;}
  if(serviceStop())return false; // STOP latched while the atomic UART burst drained
  commandedDrive_=drive;goal_=q;for(unsigned i=0;i<12;++i)commandedRaw_[i]=raw[i];goalSentUs_=now;
  // Native speed/acceleration transit estimate, conservative 2x + V25's
  // 12 s positioning timeout. This is a completion deadline, not a lag test.
  const double speed=drive.speed*kTickRad,acc=drive.acc*100*kTickRad;
  goalDeadlineUs_=now+static_cast<uint64_t>((12+2*(distance/speed+2*speed/acc))*1e6);
  return true;
}
bool SimpleMotion::arrived(uint64_t now) const {
  const auto terminal=segmentBox_?settleUs_:goalSentUs_;
  if(!terminal||now<terminal||!fresh(now)||now-terminal<100000)return false;
  for(unsigned i=0;i<12;++i)if(fb_[i].stampUs<=terminal||std::abs(fb_[i].q-jointSlot(goal_,i))>kArrivalRad||magnitude(fb_[i].speed)>50)return false;
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
    phase_=Phase::ALIGN;beginStandSegment(q,collision::kEntry,now,true);return;
  }
  if(phase_==Phase::ALIGN){
    auto q=routePoint(0);if(!transitionSafe(actual,q,collision::kEntry)){fault("ALIGN_PATH_NOT_CLEAR");return;}
    phase_=Phase::ROUTE;routeCell_=0;beginStandSegment(q,collision::kEntry,now,true);return;
  }
  if(phase_==Phase::ROUTE&&routeCell_<collision::kStandCellCount){
    const unsigned c=routeCell_++;auto q=cellEnd(c);
    if(!transitionSafe(actual,q,collision::kStandCells[c])){fault("STAND_CLEARANCE_UNAVAILABLE");return;}
    beginStandSegment(q,collision::kStandCells[c],now);return;
  }
  if(phase_==Phase::ROUTE){
    JointTargetFrame q;if(!heightTarget(requestedHeight_,q)||!transitionSafe(actual,q,collision::kHeight)){fault("HEIGHT_PATH_NOT_CLEAR");return;}
    double delta=0;for(unsigned i=0;i<12;++i)delta=std::fmax(delta,std::abs(jointSlot(segmentTo_,i)-jointSlot(q,i)));
    phase_=Phase::HEIGHT;
    if(delta>1e-5){beginStandSegment(q,collision::kHeight,now);return;}
    // The route already ends at C4/STAND150; no duplicate final waypoint.
  }
  if(phase_==Phase::HEIGHT||phase_==Phase::WALK_HOLD){
    segmentBox_=nullptr;
    phase_=Phase::HOLD;state_=State::STANDING;reason_="STAND_REACHED";
    measureHeight();
    if(walkRequested_)startWalk(now);
  }
}
void SimpleMotion::beginStandSegment(const JointTargetFrame& to,const collision::Box& box,uint64_t now,bool native){
  // Only contact/phase boundaries settle; CAD cells are not waypoints.
  // Preserve the preceding NOMINAL endpoint; feeding an already compensated
  // encoder pose back as a new nominal would apply the IMU correction twice.
  segmentFrom_=segmentTo_;segmentFrom_.valid=true;double distance=0;
  for(unsigned i=0;i<12;++i){if(!segmentTo_.valid)jointSlot(segmentFrom_,i)=fb_[i].q;distance=std::fmax(distance,std::abs(jointSlot(to,i)-jointSlot(segmentFrom_,i)));}
  segmentTo_=to;segmentBox_=&box;segmentStartUs_=now;
  // Quintic endpoint v=a=0, below native 300/8 (0.460/1.227 SI).
  // Folded entry stays a native position move: its observed start may be
  // outside URDF, so it must never become an interpolated command.
  const double duration=native?0:std::fmax(.20,std::fmax(1.875*distance/.30,std::sqrt(5.773503*distance/.60)));
  segmentDurationUs_=static_cast<uint64_t>(std::ceil(duration*1e6));settleUs_=now+segmentDurationUs_;
  segmentDeadlineUs_=settleUs_+static_cast<uint64_t>((12+2*(distance/(kStandDrive.speed*kTickRad)+2.*kStandDrive.speed/(kStandDrive.acc*100.)))*1e6);
  generateStand(now);
}
void SimpleMotion::generateStand(uint64_t now){
  if(!segmentBox_)return;
  const double u=segmentDurationUs_?std::fmin(1.,double(now-segmentStartUs_)/segmentDurationUs_):1;
  const auto law=quinticTimeLaw(u,segmentDurationUs_?segmentDurationUs_/1e6:1);
  LocomotionFrame f;f.target=segmentTo_;f.cartesian.valid=true;
  if(segmentDurationUs_)for(unsigned i=0;i<12;++i)jointSlot(f.target,i)=jointSlot(segmentFrom_,i)+law.progress*(jointSlot(segmentTo_,i)-jointSlot(segmentFrom_,i));
  if(phase_==Phase::HEIGHT&&segmentDurationUs_){
    // Same contact IK for the complete height trajectory, including small
    // encoder rounding at its initial reference. No joint-space foot drag.
    double fromHeight=0;ContactResult from[4];
    for(unsigned l=0;l<4;++l){from[l]=contactForwardKinematics(static_cast<LegId>(l),segmentFrom_.legs[l]);if(from[l].status!=ContactStatus::OK){fault("STAND_CONTACT_IK");return;}fromHeight-=from[l].referenceM.z/4;}
    f.cartesian.body.translationM.z=fromHeight+law.progress*(requestedHeight_-fromHeight);
    for(unsigned l=0;l<4;++l){const auto a=from[l].referenceM,b=canonicalStandDefinition().contactsWorldM[l];
      f.cartesian.feet[l]={a.x+law.progress*(b.x-a.x),a.y+law.progress*(b.y-a.y),(a.z+fromHeight)*(1-law.progress)};
      TiltedIkOptions opt;opt.seed=f.target.legs[l];opt.maxIterations=8;opt.requireNominalStrip=false;
      const auto ik=worldContactInverseKinematicsTilted(static_cast<LegId>(l),f.cartesian.feet[l],f.cartesian.body,opt);
      if(ik.status!=TiltedIkResult::Status::OK){fault("STAND_CONTACT_IK");return;}f.target.legs[l]=ik.base.joints;
    }
  }else{
    // Entry has intentional body pitch. Retain that reference when applying
    // the measured tilt error; do not try to flatten the folded robot.
    unsigned edge=0;double alpha=0;
    if(phase_==Phase::ROUTE&&routeCell_){const unsigned c=routeCell_-1;edge=collision::kCellEdge[c];const double a=c&&collision::kCellEdge[c-1]==edge?collision::kCellEnd[c-1]:0;alpha=a+law.progress*(collision::kCellEnd[c]-a);}
    const auto* a=paired_data::kKnots[collision::kPhaseKnots[edge]].p;
    const auto* b=paired_data::kKnots[collision::kPhaseKnots[edge+1]].p;
    const double pitch=phase_==Phase::WALK_HOLD?0:a[4]+alpha*(b[4]-a[4]);
    f.cartesian.body=phase_==Phase::WALK_HOLD?BodyPose{}:bodyPoseFromRollPitch({a[5]+alpha*(b[5]-a[5]),0,a[6]+alpha*(b[6]-a[6])},0,pitch);
    const Vector3 normal{-std::sin(pitch),0,std::cos(pitch)};
    for(unsigned l=0;l<4;++l){const auto c=contactForwardKinematics(static_cast<LegId>(l),f.target.legs[l],normal);
      if(c.status!=ContactStatus::OK||!pointToWorld(f.cartesian.body,c.referenceM,f.cartesian.feet[l])){fault("STAND_CONTACT_IK");return;}}
  }
  if(!pointSafe(f.target)||!contains(*segmentBox_,f.target)){fault("STAND_CLEARANCE_UNAVAILABLE");return;}
  compensate(f,now);send(f.target,kStandDrive,now);
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
  auto step=[](double old,double tilt){const double error=std::copysign(std::fmax(0.,std::abs(tilt)-.005),tilt);const double desired=std::clamp(-.5*error,-.020,.020);constexpr double step=.020*kPeriodUs/1e6;return old+std::clamp(desired-old,-step,step);};
  const double nominalPitch=std::atan2(-f.cartesian.body.rotation[2][0],f.cartesian.body.rotation[0][0]);
  const double roll=step(rollCorrection_,a.rollRad),pitch=step(pitchCorrection_,a.pitchRad-nominalPitch);
  if(roll==0&&pitch==0){rollCorrection_=pitchCorrection_=0;return;}
  const auto& box=phase_==Phase::WALK?collision::kWalk:segmentBox_?*segmentBox_:collision::kHeight;
  auto solve=[&](double scale){
    auto b=bodyPoseFromRollPitch(f.cartesian.body.translationM,roll*scale,nominalPitch+pitch*scale);
    JointTargetFrame q;q.valid=true;
    for(unsigned l=0;l<4;++l){TiltedIkOptions opt;opt.seed=f.target.legs[l];opt.maxIterations=8;opt.requireNominalStrip=false;
      const auto ik=worldContactInverseKinematicsTilted(static_cast<LegId>(l),f.cartesian.feet[l],b,opt);
      if(ik.status!=TiltedIkResult::Status::OK){q.valid=false;break;}q.legs[l]=ik.base.joints;}
    return q;
  };
  // Smooth geometric attenuation before the hard workspace boundary. A small
  // IK perturbation measures local joint sensitivity; reserve half the joint
  // room. This avoids normal corrections jumping from full to half amplitude
  // as the nominal trajectory approaches a limit. No encoder tracking loop.
  constexpr double probeScale=1./32;
  const auto probe=solve(probeScale);double sum=0;
  if(probe.valid)for(unsigned i=0;i<12;++i){const auto& lim=legModel(static_cast<LegId>(i/3))->limits[i%3];
    const double q=jointSlot(f.target,i),delta=(jointSlot(probe,i)-q)/probeScale;
    const double lo=std::fmax(box.lo[i],std::fmax(input_.minRad[i],lim.lower));
    const double hi=std::fmin(box.hi[i],std::fmin(input_.maxRad[i],lim.upper));
    // The smallest recovered CAD reserve is 25 mrad. A fixed symmetric
    // local allowance also keeps attenuation continuous across adjacent
    // segment boxes instead of changing gain at every phase boundary.
    const double room=std::fmin(.025,std::fmin(hi-q,q-lo));
    if(room<=0){sum=INFINITY;break;}const double ratio=delta/room;sum+=ratio*ratio;
  }
  // C2 engagement of stabilization after automatic torque enable. Uses the
  // same Motion epoch, not a second timer or a servo acceleration controller.
  const double envelope=quinticTimeLaw(std::fmin(1.,double(now-imuStartUs_)/1e6),1).progress;
  const double projection=probe.valid?envelope/std::sqrt(1+4*sum):0;
  for(unsigned trial=0;trial<5&&projection>0;++trial){const double scale=projection*std::ldexp(1.,-static_cast<int>(trial));
    const auto q=solve(scale);
    if(pointSafe(q)&&contains(box,q)){f.target=q;rollCorrection_=roll;pitchCorrection_=pitch;imuLimited_=projection<.999||trial!=0;return;}
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
  if(walkStopping_&&t>=stopClock_+2*kPeriodS){
    phase_=Phase::WALK_HOLD;state_=State::MOVING;reason_="WALK_SETTLING";
    // The gait clock has finished, but native settling still needs live IMU
    // correction and a fixed completion deadline. Reuse the same generator.
    beginStandSegment(walkPrevious_.target,collision::kWalk,now,true);
  }
}
void SimpleMotion::update(uint64_t now){
  nowUs_=now;
  if(serviceStop())return;
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
  // Two single-block reads per 20 ms: whole population every 120 ms, at
  // most 4 ms wait. One 2-block control read each 120 ms adds <=4 ms.
  for(unsigned n=0;n<2;++n){const unsigned i=pollNext_++%12;Feedback f;
    if(port_->read(i,false,f)){
      f.torque=fb_[i].torque;f.goal=fb_[i].goal;f.goalSpeed=fb_[i].goalSpeed;f.acc=fb_[i].acc;f.controlStampUs=fb_[i].controlStampUs;fb_[i]=f;failures_[i]=0;
      if(energized_&&(f.status||f.temp>=70||f.voltage<40||f.voltage>140)){fault("SERVO_NATIVE_ALARM_OR_ELECTRICAL_LIMIT");break;}
      // Hard physical limits, no comparison to a target's time history.
      if(energized_&&phase_!=Phase::ENABLE&&phase_!=Phase::LOWER&&phase_!=Phase::ALIGN&&
         (f.q<input_.minRad[i]-2*kTickRad||f.q>input_.maxRad[i]+2*kTickRad)){fault("MEASURED_MECHANICAL_LIMIT");break;}
    }else if(++failures_[i]>=3&&energized_){fault("SERVO_COMMUNICATION_LOST");break;}
    if(stop_.load())break;
  }
  if(serviceStop())return;
  if(offPending_){serviceOff();return;}
  if(phase_==Phase::ACQUIRE){
    Feedback f;const bool read=port_->read(acquire_,true,f);if(serviceStop())return;
    if(!read){fault("INITIAL_SERVO_READ_FAILED");return;}
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
    const bool read=port_->read(enable_,true,f);if(serviceStop())return;
    if(!read||f.goal!=commandedRaw_[enable_]||f.goalSpeed!=kStandDrive.speed||f.acc!=kStandDrive.acc||f.torque!=0||std::abs(f.q-jointSlot(reference_,enable_))>kArrivalRad){fault("REFERENCE_CHANGED_BEFORE_ENABLE");return;}
    energized_=true;offMask_=0; // even a failed enable may have applied; SAFE_OFF required
    const bool enabled=port_->enable(enable_);if(serviceStop())return;
    if(!enabled){fault("TORQUE_ENABLE_UNCERTAIN");return;}
    if(++enable_==12){
      imuStartUs_=now;
      if(initialRoute_==InitialRoute::ENTRY){phase_=Phase::LOWER;nextStand(now);}
      else if(initialRoute_==InitialRoute::ROUTE){phase_=Phase::ROUTE;routeCell_=initialCell_;nextStand(now);}
      else if(initialRoute_==InitialRoute::HEIGHT){phase_=Phase::ROUTE;routeCell_=collision::kStandCellCount;nextStand(now);}
      else {
        // A stopped gait returns through the proved whole WALK region to
        // 150 mm, then the ordinary height branch reaches the requested Q.
        JointTargetFrame q,actual;actual.valid=true;
        for(unsigned i=0;i<12;++i)jointSlot(actual,i)=fb_[i].q;
        if(!heightTarget(.150,q)||!transitionSafe(actual,q,collision::kWalk)){fault("HEIGHT_PATH_NOT_CLEAR");return;}
        phase_=Phase::ROUTE;routeCell_=collision::kStandCellCount;beginStandSegment(q,collision::kWalk,now);
      }
    }
    return;
  }
  if(releases_%6==0){
    const unsigned i=controlNext_++%12;Feedback f;
    const bool read=port_->read(i,true,f);if(serviceStop())return;
    if(read){
      fb_[i]=f;controlFailures_[i]=0;
      if(energized_&&(f.torque!=1||f.goal!=commandedRaw_[i]||f.goalSpeed!=commandedDrive_.speed||f.acc!=commandedDrive_.acc)){fault("SERVO_TORQUE_OR_GOAL_LOST");return;}
    }else if(energized_&&++controlFailures_[i]>=3){fault("SERVO_CONTROL_READ_LOST");return;}
  }
  if(serviceStop())return;
  if(energized_&&!fresh(now)){fault("SERVO_FEEDBACK_STALE");return;}
  measureHeight();
  if(pendingGoalValid_){
    pendingGoalValid_=false;JointTargetFrame actual;actual.valid=true;
    for(unsigned i=0;i<12;++i)jointSlot(actual,i)=fb_[i].q;
    if(!transitionSafe(actual,pendingGoal_,collision::kHeight)){fault("HEIGHT_PATH_NOT_CLEAR");return;}
    beginStandSegment(pendingGoal_,collision::kHeight,now);return;
  }
  if(phase_==Phase::WALK){generateWalk(now);return;}
  if(state_==State::MOVING){
    if(segmentBox_)generateStand(now);
    if(offPending_||stop_.load())return;
    if(arrived(now)){nextStand(now);return;}
    if(now>(segmentBox_?segmentDeadlineUs_:goalDeadlineUs_)){fault("NATIVE_TARGET_NOT_REACHED");return;}
  }
  if(phase_==Phase::HOLD){
    GaitParameters g;g.heightM=requestedHeight_;auto f=solveGait(g,gaitHold(g,0,requestedHeight_,0));
    if(f.target.valid){compensate(f,now);send(f.target,kStandDrive,now);}
  }
}
} }
