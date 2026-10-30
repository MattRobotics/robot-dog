#include "SimpleMotion.h"
#include "MotionTiming.h"
#include "../motion/PairedProneData.h"
#include "../motion/GaitTiming.h"
#include "../motion/TiltCompensation.h"
#include "../motion/ImuAttitude.h"
#include <algorithm>
#include <esp_timer.h>
#include <cmath>
#include <cstring>
namespace matdog { namespace simple {
using namespace motion;
namespace {
constexpr double kArrivalRad=12*kTickRad; // V25 native positioning allowance
// Completion under a static load (RCA 28/10, STAND50 from PRONE on the floor):
// the ST3215 position loop is P=32, I=0, so a loaded joint rests at a residual
// proportional to its load (measured 5.6..8.7 permille PresentLoad per tick,
// driven toward the goal in 163/163 samples). Only that state completes a
// segment outside kArrivalRad: stationary for kLoadedSettleUs (speed <= 50
// AND every RAW within kLoadedCreepTicks of its value at the dwell start: the
// loaded joints held 0 ticks, unloaded ones 1 LSB, over 12 s on the robot),
// driven toward the goal with at least kLoadedPerTick permille per tick, at
// most kLoadedTicks away, and no joint above kLoadedMaxPermille PresentLoad
// (half of the native OverloadTorque threshold of 80% in the same 0.1% scale).
constexpr int kLoadedTicks=48,kLoadedPerTick=4,kLoadedMaxPermille=400,kLoadedCreepTicks=1;
// Temperature >= kHotC must be confirmed (RCA 29/10): under torque the C018
// returned isolated single-sample values of 46..83 C (ID 21, 31, 42; valid
// checksum, status 0, servo cold, next sample normal; none in 8640 samples
// at rest). A hot sample opens a window per joint; only a later VALID sample
// still >= kHotC at least kHotConfirmUs after it faults. Only a VALID sample
// below kHotResetC closes the window: a missing, deferred or stale read never
// counts as normal (those keep their own faults). Status and voltage stay
// immediate; the 70 C threshold and the servo's own protection are unchanged.
// A window is evidence only while its valid samples are continuous (each
// within kHotGapUs = the feedback freshness limit of the previous one). After
// a gap (SAFE_OFF, not READY, lost reads) it restarts from the current sample,
// or closes if that sample is below kHotC: STOP/SAFE_OFF/fault never clear a
// continuous window, and an old window never confirms a new single sample.
constexpr int kHotC=70,kHotResetC=65;
constexpr uint64_t kHotConfirmUs=500000,kHotGapUs=250000;
constexpr uint64_t kLoadedSettleUs=500000;
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
double knotHeight(unsigned knot){return paired_data::kKnots[knot].p[6];}
// Nearest frozen route knot of a group (infinity norm): restart after STOP
// resumes the same certified group from the measured pose.
template<unsigned N> unsigned nearestVia(const unsigned (&knots)[N],const JointTargetFrame& q){
  unsigned best=0;double distance=INFINITY;
  for(unsigned k=knots[0];k<=knots[N-1];++k){const auto r=routePoint(k);double d=0;
    for(unsigned i=0;i<12;++i)d=std::fmax(d,std::abs(jointSlot(r,i)-jointSlot(q,i)));
    if(d<distance){distance=d;best=k;}}
  return best;
}
constexpr unsigned kDeployEnd=collision::kDeployKnots[sizeof(collision::kDeployKnots)/sizeof(collision::kDeployKnots[0])-1];
static_assert(kDeployEnd==collision::kLiftKnots[0],"DEPLOY ends where LIFT starts");
GaitParameters walkGait(unsigned stride){
  GaitParameters g;g.heightM=.15;g.liftM=kWalkLiftM;g.duty=kWalkDuty;g.advanceXM=kStrideM[stride];
  g.swayXM=kWalkSwayXM;g.swayYM=kWalkSwayYM;g.startCycles=kWalkStartCycles;g.stopCycles=kWalkStopCycles;return g;
}
}
bool SimpleMotion::readTimed(unsigned slot,bool control,Feedback& out){
  timing::Section section(timing::READ);
  if(timing::state.active){if(control)++timing::state.working.controlReads;else ++timing::state.working.fastReads;}
  return port_->read(slot,control,out);
}
bool SimpleMotion::frameTimed(const uint16_t raw[12],Drive drive){
  timing::Section section(timing::FRAME);
  if(timing::state.active)++timing::state.working.frames;
  return port_->frame(raw,drive);
}
const char* name(State s){switch(s){case State::READY:return "READY";case State::MOVING:return "MOVING";case State::STANDING:return "STANDING";case State::WALKING:return "WALKING";case State::STOPPED:return "STOPPED";case State::FAULT:return "FAULT";}return "FAULT";}
const char* SimpleMotion::phase() const {
  static const char* const p[]={"IDLE","ACQUIRE","REFERENCE","ENABLE","DISENGAGE","ALIGN","STAND_DEPLOY","STAND_LIFT","HEIGHT","HOLD","WALK_STANDARD","WALK_SETTLE"};
  return p[static_cast<unsigned>(phase_)];
}
void SimpleMotion::stop(const char* reason) { stopReason_.store(reason);stop_.store(true,std::memory_order_release); }
void SimpleMotion::fault(const char* r){enterOff(true,r);}
void SimpleMotion::enterOff(bool fault,const char* r){
  // LOADDIAG: freeze the first exit of an engaged command before any state
  // below changes; a later STOP during the same SAFE_OFF keeps this record.
  if(!exit_.valid&&engaged()){
    auto& e=exit_;e.valid=true;e.fault=fault;e.energized=energized_;e.reason=r;e.phase=phase();e.state=state_;
    e.atUs=esp_timer_get_time();e.nowUs=nowUs_;e.goalSentUs=goalSentUs_;e.settleUs=settleUs_;e.deadlineUs=deadlineUs();e.maxGapUs=maxGapUs_;
    e.releases=releases_;e.missed=missed_;e.lastPolled=(pollNext_+11)%12;e.drive=commandedDrive_;
    for(unsigned i=0;i<12;++i){e.commandedRaw[i]=commandedRaw_[i];e.fb[i]=fb_[i];}
    e.loadedMask=loadedMask_;e.loadedPhase=loadedPhase_;
  }
  loadedSinceUs_=0;
  // A repeated STOP while the same SAFE_OFF is still pending keeps its
  // broadcast and 13-servo verification progress; a failed read still
  // re-broadcasts in serviceOff. A fault, or a STOP after a fault, during
  // motion or after completion starts SAFE_OFF again from the broadcast.
  const bool repeated=!fault&&state_==State::STOPPED&&offPending_;
  state_=fault?State::FAULT:State::STOPPED;faultLatched_=fault;reason_=r;
  if(!fault)operatorStopped_=strcmp(r,"BOOTING")!=0;
  segmentBox_=nullptr;segmentTo_.valid=false;planned_=false;settleUs_=0;imuStartUs_=0;
  phase_=Phase::IDLE;offPending_=true;if(!repeated){offSent_=false;offMask_=0;offNext_=0;}
  walkRequested_=false;walkStopping_=false;pendingGoalValid_=false;rollCorrection_=pitchCorrection_=0;
}
void SimpleMotion::serviceOff(){
  if(!port_||!offPending_)return;
  if(!offSent_){offSent_=port_->off();return;}
  if(port_->verifyOff(offNext_))offMask_|=1u<<offNext_;
  else {offMask_&=~(1u<<offNext_);offSent_=false;}
  offNext_=(offNext_+1)%13;
  if(offMask_==0x1fff){energized_=false;offPending_=false;offVerifiedUs_=esp_timer_get_time();}
}
bool SimpleMotion::serviceStop(){
  if(!stop_.exchange(false,std::memory_order_acq_rel))return false;
  enterOff(false,stopReason_.load());serviceOff();return true;
}
bool SimpleMotion::fresh(uint64_t now) const {
  for(const auto& f:fb_)if(!f.valid||(now>=f.stampUs&&now-f.stampUs>250000)||(f.stampUs>now&&f.stampUs-now>15000))return false;
  return true;
}
// The one ordered membership test of a joint frame in the certified regions.
// ENTRY admits a folded measured contact outside the joint limits; every other
// region also needs them (pointSafe). Shared by restart planning and telemetry.
bool SimpleMotion::certifiedRegion(const JointTargetFrame& q,InitialRoute& route) const {
  if(pointSafe(q)&&contains(collision::kHeight,q)){route=InitialRoute::HEIGHT;return true;}
  if(contains(collision::kEntry,q)){route=InitialRoute::ENTRY;return true;}
  if(!pointSafe(q))return false;
  if(contains(collision::kWalk,q)){route=InitialRoute::WALK;return true;}
  if(contains(collision::kLift,q)){route=InitialRoute::LIFT;return true;}
  if(contains(collision::kDeploy,q)){route=InitialRoute::DEPLOY;return true;}
  return false;
}
const char* SimpleMotion::poseRegion(uint64_t now) const {
  // Telemetry of the 12 fresh ACTUAL angles. Encoder evidence only: the
  // ground contact itself is not measured.
  InitialRoute route=InitialRoute::ENTRY;
  if(!fresh(now))return "UNKNOWN";
  if(!certifiedRegion(measured(),route))return "OUTSIDE_CERTIFIED";
  switch(route){
    case InitialRoute::HEIGHT:return "STAND_HEIGHT";
    case InitialRoute::ENTRY:return "PRONE_ENTRY";
    case InitialRoute::WALK:return "WALK";
    case InitialRoute::LIFT:return "STAND_LIFT";
    case InitialRoute::DEPLOY:return "STAND_DEPLOY";
  }
  return "OUTSIDE_CERTIFIED";
}
bool SimpleMotion::pointSafe(const JointTargetFrame& q) const {
  timing::Section section(timing::CONVERT);
  if(!q.valid)return false;
  for(unsigned i=0;i<12;++i){const double v=jointSlot(q,i);if(!std::isfinite(v)||v<input_.minRad[i]||v>input_.maxRad[i])return false;}
  return contains(collision::kEntry,q)||contains(collision::kWalk,q)||contains(collision::kHeight,q)||
         contains(collision::kDeploy,q)||contains(collision::kLift,q);
}
bool SimpleMotion::planInitialReference(){
  timing::Section section(timing::NOMINAL);
  // Restart after STOP/boot from any already proved configuration region.
  // This is physical path selection from measured Q, never route tracking.
  return certifiedRegion(reference_,initialRoute_);
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
    referenceFaultSlot_=12; // new accepted command: invalidate previous incident
    requestedHeight_=h;walkRequested_=false;
    state_=State::MOVING;phase_=Phase::HEIGHT;reason_="HEIGHT_CHANGE";
    pendingGoal_=q;pendingGoalValid_=true; // intent only; TX at the next Motion release
  }else{referenceFaultSlot_=12;requestedHeight_=h;walkRequested_=false;state_=State::MOVING;phase_=Phase::ACQUIRE;acquire_=0;reason_="ACQUIRING_REAL_POSE";
    std::fill(controlFailures_,controlFailures_+12,0);}
  timing::arm();exit_.valid=false;loadedMask_=0;loadedPhase_=""; // LOADDIAG: re-arm the first-exit record
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
    // Decelerate from the next touchdown (quarter cycle) after the start
    // ramp. Feet already planted or in flight keep their targets.
    stopCycle_=std::fmax(kWalkStartCycles,std::ceil(cycles_*4)/4);walkStopping_=true;
    reason_="WALK_STOPPING";
  }
}
bool SimpleMotion::send(const JointTargetFrame& q,Drive drive,uint64_t now,bool holdReference){
  timing::Section section(timing::CONVERT);
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
  if(!frameTimed(raw,drive)){if(!serviceStop())fault("SYNC_WRITE_FAILED");return false;}
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
  if(!terminal||now<terminal||!fresh(esp_timer_get_time())||now-terminal<100000)return false;
  for(unsigned i=0;i<12;++i)if(fb_[i].stampUs<=terminal||std::abs(fb_[i].q-jointSlot(goal_,i))>kArrivalRad||magnitude(fb_[i].speed)>50)return false;
  return true;
}
// LIFT -> HEIGHT only (RCA 29/10): at the 50 mm lift top a loaded LOWER rests
// a few ticks outside kHeight (robot: LF_LOWER 7 ticks) although the pose is
// inside kLift. HEIGHT starts from the nominal lift top, a two-via plan that is
// monotonic per joint. A joint outside kHeight is admitted only if that
// nominal start is inside kHeight and HEIGHT moves the joint away from the
// violated bound, i.e. back into the box. Everything else stays as before.
bool SimpleMotion::entersHeight(const JointTargetFrame& actual) const {
  if(phase_!=Phase::LIFT||std::abs(plannedHeight_-requestedHeight_)<=1e-9)return false;
  JointTargetFrame q;if(!segmentTo_.valid||!heightTarget(requestedHeight_,q))return false;
  const auto& b=collision::kHeight;
  for(unsigned i=0;i<12;++i){
    const double a=jointSlot(actual,i),n=jointSlot(segmentTo_,i),d=jointSlot(q,i)-n;
    if(a>=b.lo[i]&&a<=b.hi[i])continue;
    if(n<b.lo[i]||n>b.hi[i]||!(a<b.lo[i]?d>0:d<0))return false;
  }
  return true;
}
bool SimpleMotion::settledUnderLoad(uint64_t now){
  // Per phase: which joints may rest loaded, and the certified box in which
  // the NEXT segment starts from this actual pose. Targets are unchanged; the
  // next segment still starts from the commanded (nominal) reference, so the
  // loaded servos keep their drive instead of being re-commanded to the sag.
  uint16_t allowed=0;const collision::Box* next=segmentBox_;
  switch(phase_){
    // DISENGAGE commands only the LOWER joints; hips/uppers hold their own
    // measured reference and must still arrive within kArrivalRad.
    case Phase::LOWER:allowed=0x924;break;
    case Phase::ALIGN:allowed=0xfff;next=&collision::kDeploy;break;
    case Phase::DEPLOY:allowed=0xfff;next=&collision::kLift;break;
    case Phase::LIFT:allowed=0xfff;if(std::abs(plannedHeight_-requestedHeight_)>1e-9)next=&collision::kHeight;break;
    case Phase::HEIGHT:allowed=0xfff;break;
    default:break;  // WALK_SETTLE and HOLD keep the 12-tick arrival only
  }
  const auto terminal=segmentBox_?settleUs_:goalSentUs_;
  bool ok=allowed&&segmentBox_&&terminal&&now>=terminal&&now-terminal>=100000&&fresh(esp_timer_get_time());
  uint16_t loaded=0;
  for(unsigned i=0;ok&&i<12;++i){
    const auto& f=fb_[i];
    if(f.stampUs<=terminal||magnitude(f.speed)>50||f.status||f.torque!=1||(f.load&1023)>kLoadedMaxPermille){ok=false;break;}
    if(std::abs(f.q-jointSlot(goal_,i))<=kArrivalRad)continue;
    const int e=int(commandedRaw_[i])-int(f.raw);
    ok=(allowed>>i&1)&&std::abs(e)<=kLoadedTicks&&(e>0)==((f.load&1024)!=0)&&(f.load&1023)>=kLoadedPerTick*std::abs(e);
    loaded|=1u<<i;
  }
  const auto actual=measured();
  ok=ok&&loaded&&pointSafe(actual)&&contains(*segmentBox_,actual)&&(contains(*next,actual)||entersHeight(actual));
  if(!ok){loadedSinceUs_=0;return false;}
  // Slow creep below the speed threshold restarts the dwell from the new RAW.
  bool still=loadedSinceUs_!=0;
  for(unsigned i=0;still&&i<12;++i)still=std::abs(int(fb_[i].raw)-int(loadedRaw_[i]))<=kLoadedCreepTicks;
  if(!still){loadedSinceUs_=now;for(unsigned i=0;i<12;++i)loadedRaw_[i]=fb_[i].raw;return false;}
  if(now-loadedSinceUs_<kLoadedSettleUs)return false;
  loadedMask_=loaded;loadedPhase_=phase();loadedSinceUs_=0;return true;
}
void SimpleMotion::measureHeight(){
  // Native encoders + measured IMU orientation + flat-ground assumption.
  // No absolute height sensor exists, especially on a suspended fixture.
  heightValid_=false;if(!fresh(esp_timer_get_time())||!input_.imuValid)return;
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
void SimpleMotion::updateTelemetry(uint64_t now){
  if(now<nextTelemetryUs_)return;
  nextTelemetryUs_=now+100000;measureHeight();
}
JointTargetFrame SimpleMotion::measured() const {
  JointTargetFrame q;q.valid=true;for(unsigned i=0;i<12;++i)jointSlot(q,i)=fb_[i].q;return q;
}
bool SimpleMotion::via(const JointTargetFrame& q,double pitch,double x,double z){
  if(!q.valid)return false;double p[ViaPlan::kDim];
  for(unsigned i=0;i<12;++i)p[i]=jointSlot(q,i);p[12]=pitch;p[13]=x;p[14]=z;return plan_.add(p);
}
bool SimpleMotion::knotVia(unsigned knot){const double* p=paired_data::kKnots[knot].p;return via(routePoint(knot),p[4],p[5],p[6]);}
bool SimpleMotion::heightVias(double from,double to){
  // Level stands between the two heights are frozen route knots (identical
  // to the canonical contact IK); the requested height is solved exactly.
  constexpr unsigned n=sizeof(collision::kLiftKnots)/sizeof(collision::kLiftKnots[0]);
  for(unsigned j=0;j<n;++j){const unsigned k=collision::kLiftKnots[from<to?j:n-1-j];const double z=knotHeight(k);
    if(k>=160&&std::fmin(from,to)+1e-6<z&&z<std::fmax(from,to)-1e-6&&!knotVia(k))return false;}
  JointTargetFrame q;return heightTarget(to,q)&&via(q,0,0,to);
}
void SimpleMotion::beginPlan(Phase next,const collision::Box& box,uint64_t now){
  // The whole time-continuous group lies in one certified box: each joint
  // stays within the range of its via points, which are all checked here.
  if(plan_.n<1||!plan_.plan(kStandSpeedRadS,kStandAccelerationRadS2)){fault("STAND_PLAN_UNAVAILABLE");return;}
  double distance=0;
  for(unsigned k=0;k<plan_.n;++k){JointTargetFrame q;q.valid=true;for(unsigned i=0;i<12;++i)jointSlot(q,i)=plan_.p[k][i];
    if(!contains(box,q)||!pointSafe(q)){fault("STAND_CLEARANCE_UNAVAILABLE");return;}
    for(unsigned i=0;i<12;++i)distance=std::fmax(distance,std::abs(plan_.p[k][i]-plan_.p[0][i]));
    segmentTo_=q;}
  phase_=next;planned_=true;segmentBox_=&box;segmentStartUs_=now;loadedSinceUs_=0;
  segmentDurationUs_=static_cast<uint64_t>(std::ceil(plan_.duration*1e6));settleUs_=now+segmentDurationUs_;
  segmentDeadlineUs_=settleUs_+static_cast<uint64_t>((12+2*(distance/(kStandDrive.speed*kTickRad)+2.*kStandDrive.speed/(kStandDrive.acc*100.)))*1e6);
  generateStand(now);
}
void SimpleMotion::beginNative(Phase next,const JointTargetFrame& to,const collision::Box& box,uint64_t now,double pitch,double x,double z){
  // Folded entry and the final WALK settle stay native position moves: their
  // observed start may be outside URDF, never an interpolated command.
  double distance=0;for(unsigned i=0;i<12;++i)distance=std::fmax(distance,std::abs(jointSlot(to,i)-fb_[i].q));
  phase_=next;planned_=false;segmentTo_=to;segmentBox_=&box;segmentStartUs_=now;segmentDurationUs_=0;settleUs_=now;loadedSinceUs_=0;
  nativePitch_=pitch;nativeX_=x;nativeZ_=z;
  segmentDeadlineUs_=settleUs_+static_cast<uint64_t>((12+2*(distance/(kStandDrive.speed*kTickRad)+2.*kStandDrive.speed/(kStandDrive.acc*100.)))*1e6);
  generateStand(now);
}
void SimpleMotion::beginDeploy(const JointTargetFrame* start,uint64_t now){
  // PRONE, possible ventral support: PRE_ALIGN, REAR_DEPLOY, FRONT_DEPLOY and
  // LOAD_TRANSFER follow the frozen route knots without intermediate stops.
  plan_.clear();unsigned first=0;
  if(start){first=nearestVia(collision::kDeployKnots,*start);const double* p=paired_data::kKnots[first].p;
    if(!via(*start,p[4],p[5],p[6])){fault("STAND_PLAN_UNAVAILABLE");return;}}
  for(unsigned k:collision::kDeployKnots)if((!start||k>first||k==kDeployEnd)&&!knotVia(k)){fault("STAND_PLAN_UNAVAILABLE");return;}
  beginPlan(Phase::DEPLOY,collision::kDeploy,now);
}
void SimpleMotion::beginLift(const JointTargetFrame* start,uint64_t now){
  // Body disengagement, foot contacts, contact-locked rise and levelling, then
  // straight up to the requested height in the same continuous reference: no
  // detour through STAND150. Above the certified lift top a HEIGHT move follows.
  const double top=knotHeight(collision::kLiftTopKnot),h=std::fmin(requestedHeight_,top);
  plan_.clear();unsigned first=collision::kLiftKnots[0];
  if(start){first=nearestVia(collision::kLiftKnots,*start);const double* p=paired_data::kKnots[first].p;
    if(!via(*start,p[4],p[5],p[6])){fault("STAND_PLAN_UNAVAILABLE");return;}}
  for(unsigned k:collision::kLiftKnots)
    if((!start||k>first)&&knotHeight(k)<h-1e-6&&!knotVia(k)){fault("STAND_PLAN_UNAVAILABLE");return;}
  JointTargetFrame q;if(!heightTarget(h,q)||!via(q,0,0,h)){fault("HEIGHT_PATH_NOT_CLEAR");return;}
  plannedHeight_=h;beginPlan(Phase::LIFT,collision::kLift,now);
}
void SimpleMotion::beginHeight(const JointTargetFrame& from,double fromHeight,const collision::Box& box,uint64_t now){
  plan_.clear();
  if(!via(from,0,0,fromHeight)||!heightVias(fromHeight,requestedHeight_)){fault("HEIGHT_PATH_NOT_CLEAR");return;}
  plannedHeight_=requestedHeight_;beginPlan(Phase::HEIGHT,box,now);
}
void SimpleMotion::nextStand(uint64_t now){
  timing::Section section(timing::NOMINAL);
  const JointTargetFrame actual=measured();
  const double* head=paired_data::kKnots[0].p;
  switch(phase_){
    case Phase::ENABLE:{  // folded entry: lower joints to the route head first
      auto q=reference_,h=routePoint(0);for(unsigned l=0;l<4;++l)q.legs[l].lower=h.legs[l].lower;
      if(!transitionSafe(actual,q,collision::kEntry)){fault("DISENGAGE_PATH_NOT_CLEAR");return;}
      beginNative(Phase::LOWER,q,collision::kEntry,now,head[4],head[5],head[6]);return;}
    case Phase::LOWER:{
      auto q=routePoint(0);if(!transitionSafe(actual,q,collision::kEntry)){fault("ALIGN_PATH_NOT_CLEAR");return;}
      beginNative(Phase::ALIGN,q,collision::kEntry,now,head[4],head[5],head[6]);return;}
    case Phase::ALIGN:beginDeploy(nullptr,now);return;
    // The single settled checkpoint inside the route: four feet placed under
    // the body, before its weight leaves a possible ventral support.
    case Phase::DEPLOY:beginLift(nullptr,now);return;
    case Phase::LIFT:case Phase::HEIGHT:
      if(std::abs(plannedHeight_-requestedHeight_)>1e-9){beginHeight(segmentTo_,plannedHeight_,collision::kHeight,now);return;}
      [[fallthrough]];
    case Phase::WALK_HOLD:
      segmentBox_=nullptr;planned_=false;
      phase_=Phase::HOLD;state_=State::STANDING;reason_="STAND_REACHED";
      {GaitParameters g;g.heightM=requestedHeight_;holdNominal_=solveGait(g,gaitHold(g,0,requestedHeight_,0),nullptr,false);}
      heightValid_=false;nextTelemetryUs_=0;
      if(walkRequested_)startWalk(now);
      return;
    default:return;
  }
}
void SimpleMotion::generateStand(uint64_t now){
  timing::Section section(timing::NOMINAL);
  if(!segmentBox_)return;
  LocomotionFrame f;f.cartesian.valid=true;double pitch=nativePitch_,x=nativeX_,z=nativeZ_;
  if(planned_){double s[ViaPlan::kDim];plan_.sample((now-segmentStartUs_)/1e6,s);f.target.valid=true;
    for(unsigned i=0;i<12;++i)jointSlot(f.target,i)=s[i];pitch=s[12];x=s[13];z=s[14];}
  else f.target=segmentTo_;
  // Nominal route body pose (intentional pitch preserved); the measured tilt
  // error is corrected around it by the same contact IK, never flattened.
  f.cartesian.body=bodyPoseFromRollPitch({x,0,z},0,pitch);
  const Vector3 normal{-std::sin(pitch),0,std::cos(pitch)};
  for(unsigned l=0;l<4;++l){const auto c=contactForwardKinematics(static_cast<LegId>(l),f.target.legs[l],normal);
    if(c.status!=ContactStatus::OK||!pointToWorld(f.cartesian.body,c.referenceM,f.cartesian.feet[l])){fault("STAND_CONTACT_IK");return;}}
  if(!pointSafe(f.target)||!contains(*segmentBox_,f.target)){fault("STAND_CLEARANCE_UNAVAILABLE");return;}
  compensate(f,now);send(f.target,kStandDrive,now);
}
void SimpleMotion::startWalk(uint64_t now){
  timing::Section section(timing::NOMINAL);
  const auto g=walkGait(stride_);plannedHeight_=.150;
  walkPrevious_=solveGait(g,gaitCartesian(g,0,0,0),nullptr,false);
  if(!pointSafe(walkPrevious_.target)||!contains(collision::kWalk,walkPrevious_.target)){fault("WALK_IK_OR_WORKSPACE");return;}
  walkStartUs_=now;walkStopping_=false;stopCycle_=-1;cycles_=0;
  phase_=Phase::WALK;state_=State::WALKING;reason_="WALK_STANDARD";
}
void SimpleMotion::compensate(LocomotionFrame& f,uint64_t now){
  timing::Section section(timing::COMPENSATE);
  imuLimited_=false;Attitude a;
  if(!input_.imuValid||now<input_.imuStampUs||now-input_.imuStampUs>200000||
     !tiltFromQuaternion(input_.quaternion[0],input_.quaternion[1],input_.quaternion[2],input_.quaternion[3],a)){imuLimited_=true;return;}
  // A small proportional BODY correction, slew limited, no servo PID or
  // integral loop. The same target generator owns nominal + correction.
  // One slew step per Motion release: a group boundary regenerates the
  // reference in the same release and must not advance the correction twice.
  const double slew=now==compensatedUs_?0:.020*kPeriodUs/1e6;compensatedUs_=now;
  auto step=[slew](double old,double tilt){const double error=std::copysign(std::fmax(0.,std::abs(tilt)-.005),tilt);const double desired=std::clamp(-.5*error,-.020,.020);return old+std::clamp(desired-old,-slew,slew);};
  const double nominalPitch=std::atan2(-f.cartesian.body.rotation[2][0],f.cartesian.body.rotation[0][0]);
  const double roll=step(rollCorrection_,a.rollRad),pitch=step(pitchCorrection_,a.pitchRad-nominalPitch);
  if(roll==0&&pitch==0){rollCorrection_=pitchCorrection_=0;return;}
  const auto& box=phase_==Phase::WALK?collision::kWalk:segmentBox_?*segmentBox_:collision::kHeight;
  auto solve=[&](double scale){
    timing::count(timing::SOLVES);
    auto b=bodyPoseFromRollPitch(f.cartesian.body.translationM,roll*scale,nominalPitch+pitch*scale);
    JointTargetFrame q;q.valid=true;
    // 1 um contact residual: <1% of one servo tick at the foot (~2e-4 m),
    // usually reached by the first Newton step from the nominal seed.
    for(unsigned l=0;l<4;++l){TiltedIkOptions opt;opt.seed=f.target.legs[l];opt.toleranceM=1e-6;opt.maxIterations=8;opt.requireNominalStrip=false;
      const auto ik=worldContactInverseKinematicsTilted(static_cast<LegId>(l),f.cartesian.feet[l],b,opt);
      timing::count(timing::IK);timing::count(timing::NEWTON,ik.base.newtonSteps);timing::count(timing::FK,ik.base.fkEvaluations);
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
  timing::Section section(timing::NOMINAL);
  // One constant gait clock. Start and WALK STOP are geometric C2 stride
  // ramps inside the gait law, so the body leaves and reaches rest with zero
  // velocity and acceleration; the last window recentres with four feet down.
  const auto g=walkGait(stride_);
  const double terminal=walkStopping_?stopCycle_+g.stopCycles:-1,end=terminal+gaitSettleCycles(g);
  cycles_=(now-walkStartUs_)/1e6/kWalkPeriodS;
  const bool done=walkStopping_&&cycles_>=end;if(done)cycles_=end;
  auto f=solveGait(g,gaitCartesian(g,cycles_,done?0:1/kWalkPeriodS,0,terminal),&walkPrevious_,false);
  if(!f.target.valid||!contains(collision::kWalk,f.target)||!pointSafe(f.target)){fault("WALK_IK_OR_WORKSPACE");return;}
  walkPrevious_=f;compensate(f,now);
  if(!send(f.target,kWalkDrive,now))return;
  if(done){
    state_=State::MOVING;reason_="WALK_SETTLING";
    // The gait clock has finished, but native settling still needs live IMU
    // correction and a fixed completion deadline. Reuse the same generator.
    beginNative(Phase::WALK_HOLD,walkPrevious_.target,collision::kWalk,now,0,0,.15);
  }
}
void SimpleMotion::update(uint64_t now){
  timing::Visit visit;
  nowUs_=now;
  if(serviceStop())return;
  if(offPending_){serviceOff();return;}
  // Electrical supervision is not gated by the 20 ms trajectory release.
  if(engaged()&&!input_.powerOk){fault(input_.powerReason);serviceOff();return;}
  if(!engaged()&&state_==State::READY&&(!input_.ready||!input_.powerOk||!input_.imuValid)){state_=State::STOPPED;reason_=input_.refusal;}
  if(state_==State::STOPPED&&input_.ready&&input_.powerOk&&input_.imuValid&&offVerified()&&!operatorStopped_){state_=State::READY;reason_="READY";}
  if(!input_.ready&&!engaged())return;
  if(!nextUs_)nextUs_=now;
  if(now<nextUs_)return;
  const uint64_t gap=lastReleaseUs_?now-lastReleaseUs_:0;
  maxGapUs_=std::max(maxGapUs_,gap);lastReleaseUs_=now;++releases_;
  const uint64_t lateness=now-nextUs_;
  const auto skipped=(now-nextUs_)/kPeriodUs;missed_+=skipped;nextUs_+=(skipped+1)*kPeriodUs;
  timing::Release timer(maxReleaseWorkUs_,now,phase(),releases_,lateness,missed_,skipped);
  if(energized_&&gap>100000){fault("MOTION_DEADLINE_LOST");serviceOff();return;}
  if(engaged()&&(!input_.ready||!input_.powerOk||!input_.authorityOk)){fault(input_.refusal);serviceOff();return;}
  // An accepted command continues without any command source: no browser,
  // session or heartbeat condition exists here, only physical faults below.
  if(energized_&&(!input_.imuValid||now-input_.imuStampUs>600000)){fault("IMU_STALE");serviceOff();return;}
  // Two single-block reads per 20 ms: whole population every 120 ms, at
  // most 4 ms wait. One 2-block control read each 120 ms adds <=4 ms.
  for(unsigned n=0;n<2;++n){const unsigned i=pollNext_++%12;Feedback f;
    if(readTimed(i,false,f)){
      f.torque=fb_[i].torque;f.torqueLimit=fb_[i].torqueLimit;f.goal=fb_[i].goal;f.goalSpeed=fb_[i].goalSpeed;f.acc=fb_[i].acc;f.controlStampUs=fb_[i].controlStampUs;fb_[i]=f;failures_[i]=0;
      bool hot=false;
      if(hotSinceUs_[i]&&f.stampUs-hotLastUs_[i]>kHotGapUs)hotSinceUs_[i]=0;  // continuity lost
      if(f.temp>=kHotC){if(!hotSinceUs_[i])hotSinceUs_[i]=f.stampUs;hotLastUs_[i]=f.stampUs;hot=f.stampUs-hotSinceUs_[i]>=kHotConfirmUs;}
      else if(f.temp<kHotResetC){if(hotSinceUs_[i]){hotSinceUs_[i]=0;++tempSpikes_;}}
      else if(hotSinceUs_[i])hotLastUs_[i]=f.stampUs;  // 65..69 C keeps a continuous window
      if(energized_&&(f.status||hot||f.voltage<40||f.voltage>140)){fault("SERVO_NATIVE_ALARM_OR_ELECTRICAL_LIMIT");break;}
      // Hard physical limits, no comparison to a target's time history.
      if(energized_&&phase_!=Phase::ENABLE&&phase_!=Phase::LOWER&&phase_!=Phase::ALIGN&&
         (f.q<input_.minRad[i]-2*kTickRad||f.q>input_.maxRad[i]+2*kTickRad)){fault("MEASURED_MECHANICAL_LIMIT");break;}
    }else if(!port_->readDeferred()&&++failures_[i]>=3&&energized_){fault("SERVO_COMMUNICATION_LOST");break;}
    if(stop_.load())break;
  }
  if(serviceStop())return;
  if(offPending_){serviceOff();return;}
  if(energized_&&phase_==Phase::ENABLE&&!fresh(esp_timer_get_time())){fault("SERVO_FEEDBACK_STALE");serviceOff();return;}
  if(phase_==Phase::ACQUIRE){
    Feedback f;const bool read=readTimed(acquire_,true,f);if(serviceStop())return;
    // An unanswered READ measures nothing: the same servo is read again at the
    // next release. RX_PENDING defers without a new lost reply; three actual
    // failed attempts still fault. The pending horizon is owned by ServoBus.
    if(!read){if(port_->readDeferred())return;if(++controlFailures_[acquire_]>=3)fault("INITIAL_SERVO_READ_FAILED");return;}
    controlFailures_[acquire_]=0;fb_[acquire_]=f;
    if(f.torque!=0||f.status||f.temp>=70||f.voltage<40||f.voltage>140||magnitude(f.speed)>50){fault("INITIAL_SERVO_NOT_READY");return;}
    if(++acquire_==12){
      reference_.valid=true;for(unsigned i=0;i<12;++i)jointSlot(reference_,i)=fb_[i].q;
      if(!fresh(esp_timer_get_time())||!planInitialReference()){fault("INITIAL_POSE_OUTSIDE_SAFE_ENTRY");return;}
      phase_=Phase::REFERENCE;
    }
    return;
  }
  if(phase_==Phase::REFERENCE){
    // Write the real measured reference with torque OFF before enabling.
    reference_.valid=true;for(unsigned i=0;i<12;++i)jointSlot(reference_,i)=fb_[i].q;
    if(!planInitialReference()){fault("INITIAL_POSE_OUTSIDE_SAFE_ENTRY");return;}
    // ST-3215-C018 position-profile writes can arm torque (QC V6.1).
    // Restore OFF immediately; ENABLE independently verifies it per joint.
    energized_=true;offMask_=0; // even an uncertain reference TX may have armed
    if(!send(reference_,kStandDrive,now,true))return;
    if(!port_->off()){fault("REFERENCE_TORQUE_OFF_UNCERTAIN");serviceOff();return;}
    phase_=Phase::ENABLE;enable_=0;referenceOffRetries_=0;return;
  }
  if(phase_==Phase::ENABLE){
    // Verify each exact GoalPosition/RAM profile BEFORE enabling that servo.
    Feedback f;
    const bool read=readTimed(enable_,true,f);if(serviceStop())return;
    if(read){
      fb_[enable_]=f; // publish this actual observation, including failed admission
      if(f.status||f.temp>=70||f.voltage<40||f.voltage>140||magnitude(f.speed)>50){fault("INITIAL_SERVO_NOT_READY");serviceOff();return;}
    }
    if(read&&(f.torque!=0||std::abs(f.q-jointSlot(reference_,enable_))>kArrivalRad)){
      // A profile write can arm an ST3215 and a broadcast OFF has no ACK.
      // Only a stationary, unchanged, exact holding reference can retry OFF.
      // No goal/ON write here: the next release must independently prove OFF
      // and the same real position again. A lost OFF stays a bounded fault.
      if(f.torque==1&&std::abs(f.q-jointSlot(reference_,enable_))<=kArrivalRad&&
         f.goal==commandedRaw_[enable_]&&f.goalSpeed==kStandDrive.speed&&f.acc==kStandDrive.acc&&referenceOffRetries_<2){
        ++referenceOffRetries_;
        const bool off=port_->off(enable_);if(serviceStop())return;
        if(!off){fault("REFERENCE_TORQUE_OFF_UNCERTAIN");serviceOff();}
        return;
      }
      // Emit a reference guard only for the fault, not a recovered OFF retry.
      referenceFaultSlot_=enable_;referenceFaultFeedback_=f;referenceFaultRaw_=commandedRaw_[enable_];
      fault("REFERENCE_CHANGED_BEFORE_ENABLE");
      serviceOff(); // de-escalate before emitting the single fault diagnostic
      port_->referenceFault(enable_,f,jointSlot(reference_,enable_),commandedRaw_[enable_]);
      return;
    }
    // No answer, or the reference SYNC_WRITE (no reply exists) not applied by
    // this servo, is not a pose change. Re-send the identical frame, then
    // restore OFF only for the not-yet-enabled suffix: a profile can arm it.
    // Already enabled joints keep holding; the same slot is re-verified.
    if(!read&&port_->readDeferred())return; // no new unanswered reply during RX recovery
    if(!read||f.goal!=commandedRaw_[enable_]||f.goalSpeed!=kStandDrive.speed||f.acc!=kStandDrive.acc){
      if(++controlFailures_[enable_]>=3){fault(read?"SERVO_TORQUE_OR_GOAL_LOST":"SERVO_CONTROL_READ_LOST");return;}
      if(read){
        const bool sent=frameTimed(commandedRaw_,kStandDrive);
        const bool off=port_->off(enable_);
        if(serviceStop())return;
        if(!sent||!off){fault(sent?"REFERENCE_TORQUE_OFF_UNCERTAIN":"SYNC_WRITE_FAILED");serviceOff();}
      }
      return;
    }
    controlFailures_[enable_]=0;
    energized_=true;offMask_=0; // even a failed enable may have applied; SAFE_OFF required
    const bool enabled=port_->enable(enable_);if(serviceStop())return;
    if(!enabled){fault("TORQUE_ENABLE_UNCERTAIN");return;}
    fb_[enable_].torque=1; // independent torque readback succeeded in enable()
    referenceOffRetries_=0;
    if(++enable_==12){
      imuStartUs_=now;const auto actual=measured();
      switch(initialRoute_){
        case InitialRoute::ENTRY:nextStand(now);break;  // phase_==ENABLE: DISENGAGE first
        case InitialRoute::DEPLOY:beginDeploy(&actual,now);break;
        case InitialRoute::LIFT:beginLift(&actual,now);break;
        case InitialRoute::HEIGHT:{
          // Level stance: same contact FK as the height estimate, flat ground.
          double h=0;for(unsigned l=0;l<4;++l){const auto c=contactForwardKinematics(static_cast<LegId>(l),actual.legs[l]);
            if(c.status!=ContactStatus::OK){fault("STAND_CONTACT_IK");return;}h-=c.referenceM.z/4;}
          beginHeight(actual,std::fmin(.150,std::fmax(kMinStandM,h)),collision::kHeight,now);break;}
        case InitialRoute::WALK:{
          // A stopped gait returns through the proved whole WALK region to
          // 150 mm, then the ordinary height move reaches the requested Q.
          JointTargetFrame q;plan_.clear();
          if(!heightTarget(.150,q)||!transitionSafe(actual,q,collision::kWalk)||!via(actual,0,0,.15)||!via(q,0,0,.15)){fault("HEIGHT_PATH_NOT_CLEAR");return;}
          plannedHeight_=.150;beginPlan(Phase::HEIGHT,collision::kWalk,now);break;}
      }
    }
    return;
  }
  if(releases_%6==0){
    const unsigned i=controlNext_++%12;Feedback f;
    const bool read=readTimed(i,true,f);if(serviceStop())return;
    if(read){
      fb_[i]=f;controlFailures_[i]=0;
      if(energized_&&(f.torque!=1||f.goal!=commandedRaw_[i]||f.goalSpeed!=commandedDrive_.speed||f.acc!=commandedDrive_.acc)){fault("SERVO_TORQUE_OR_GOAL_LOST");return;}
    }else if(energized_&&!port_->readDeferred()&&++controlFailures_[i]>=3){fault("SERVO_CONTROL_READ_LOST");return;}
  }
  if(serviceStop())return;
  // READ publication can follow release entry by more than the future tolerance.
  // Use the gate clock; keep the trajectory epoch and all age limits unchanged.
  if(energized_&&!fresh(esp_timer_get_time())){fault("SERVO_FEEDBACK_STALE");return;}
  if(pendingGoalValid_){
    pendingGoalValid_=false;JointTargetFrame actual;actual.valid=true;
    for(unsigned i=0;i<12;++i)jointSlot(actual,i)=fb_[i].q;
    if(!transitionSafe(actual,pendingGoal_,collision::kHeight)){fault("HEIGHT_PATH_NOT_CLEAR");return;}
    // Start from the preceding NOMINAL stand: an already compensated encoder
    // pose fed back as nominal would apply the IMU correction twice.
    beginHeight(segmentTo_,plannedHeight_,collision::kHeight,now);return;
  }
  if(phase_==Phase::WALK){generateWalk(now);return;}
  if(state_==State::MOVING){
    // At a settled boundary generate only the next segment in this release.
    if(arrived(now)||settledUnderLoad(now)){nextStand(now);return;}
    if(segmentBox_)generateStand(now);
    if(offPending_||stop_.load())return;
    if(now>(segmentBox_?segmentDeadlineUs_:goalDeadlineUs_)){fault("NATIVE_TARGET_NOT_REACHED");return;}
  }
  if(phase_==Phase::HOLD){
    // Nominal height/contacts are constant in HOLD; only the IMU correction
    // changes. Do not solve the same nominal IK and differential every 20 ms.
    auto f=holdNominal_;
    if(f.target.valid){compensate(f,now);send(f.target,kStandDrive,now);}
  }
}
} }
