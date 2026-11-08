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
double baseLinkHeightFromNominalM(double h){return h+kFrontHipAboveNominalM-motion::legModel(motion::LegId::LF)->hipOrigin.z;}
double nominalHeightFromBaseLinkM(double b){return b-kFrontHipAboveNominalM+motion::legModel(motion::LegId::LF)->hipOrigin.z;}
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
// Inner convex envelope of the unchanged HEIGHT box, gen-1 RAW, nominal
// support strip (including rounding), and the existing StopPlan over the full
// WALK120/2mm-sway phase set. Vertices are roll/pitch BODY-in-support radians;
// they are not independent rectangular limits or a claim of physical contact.
struct TiltVertex { double roll,pitch; };
constexpr TiltVertex kWalkTilt45[]={
 {0.014807950855685186,0.03574955578674701},
 {0,0.11930093706093331},
 {-0.014656913177372907,0.035384918575352142},
 {-0.011779818742757926,-0.028438998171065412},
 {-0,-0.053422005857361891},
 {0.011644818622368949,-0.028113079049515703},
};
constexpr TiltVertex kWalkTilt60[]={
 {0.014373513909108453,0.014373513909108453},
 {0.013471848316828587,0.032523918916725118},
 {0,0.10137685731507248},
 {-0.013436993467988561,0.032439771867942953},
 {-0.014373513909108453,0.014373513909108453},
 {-0.0085559368620721186,-0.0035439850870762082},
 {-0.0067072009272594714,-0.0067072009272594714},
 {-0.0034862036684149191,-0.0084164401774712468},
 {-0,-0.0084610410728295175},
 {0.0065962325017923174,-0.0065962325017923174},
 {0.0085559368620717925,-0.0035439850870778045},
};
// Level correction about the support axes, composed with the gait heading:
// the support fit returns gravity's tilt in the support frame. Identity
// heading (every unsteered walk) returns the roll/pitch pose bit for bit.
BodyPose headed(const BodyPose& nominal,double roll,double pitch){
  auto b=bodyPoseFromRollPitch({},roll,pitch);double r[3][3];
  for(unsigned i=0;i<3;++i)for(unsigned j=0;j<3;++j)r[i][j]=b.rotation[i][0]*nominal.rotation[0][j]+b.rotation[i][1]*nominal.rotation[1][j]+b.rotation[i][2]*nominal.rotation[2][j];
  for(unsigned i=0;i<3;++i)for(unsigned j=0;j<3;++j)b.rotation[i][j]=r[i][j];return b;
}
template<unsigned N> bool projectWalkTilt(const TiltVertex (&vertices)[N],double& roll,double& pitch){
  double scale=1;
  for(unsigned i=0;i<N;++i){const auto a=vertices[i],b=vertices[(i+1)%N];
    const double x=b.roll-a.roll,y=b.pitch-a.pitch,edge=x*a.pitch-y*a.roll,ray=x*pitch-y*roll;
    if(ray<edge)scale=std::fmin(scale,edge/ray);
  }
  if(scale>=1)return false;
  roll*=scale;pitch*=scale;return true;
}
GaitParameters walkGait(unsigned stride){
  GaitParameters g;g.heightM=baseLinkHeightFromNominalM(kWalkHeightM);g.liftM=kWalkLiftM;g.duty=kWalkDuty;g.advanceXM=kStrideM[stride];
  g.swayXM=kWalkSwayXM;g.swayYM=kWalkSwayYM;g.startCycles=kWalkStartCycles;g.stopCycles=kWalkStopCycles;return g;
}
GaitParameters steerGait(const double goal[3]){
  auto g=walkGait(0);g.advanceXM=goal[0];g.advanceYM=goal[1];g.yawRad=goal[2];g.reverse=goal[0]<0;
  g.swayYM=joystickSwayYM(goal[1],goal[2],kWalkSwayYM);g.edgeTiltRad=kEdgeContactMaxTiltRad;return g;
}
// Inner convex envelope of kSteer, gen-1 RAW, the tread fillet up to
// kEdgeContactMaxTiltRad and STOP over the steered phase set (offline rays).
constexpr TiltVertex kSteerTilt[]={{0.0245,0.0245},{0,0.07},{-0.0245,0.0245},{-0.0245,-0.0245},{0,-0.042},{0.0245,-0.0245}};
}
bool SimpleMotion::readTimed(unsigned slot,bool control,Feedback& out){
  timing::Section section(timing::READ);
  if(timing::state.active){if(control)++timing::state.working.controlReads;else ++timing::state.working.fastReads;}
  // A latched FAULT still permits these bounded, read-only observations after
  // SAFE_OFF. A newly pending STOP/OFF always preempts them through the bus hook.
  const bool previous=stopIo_;stopIo_=true;
  const bool ok=port_->read(slot,control,out);readInterrupted_=stop_.load()||safeOffRequest_.load();stopIo_=previous;
  if(port_->tempCheck(tempCheck_[tempChecks_%2]))++tempChecks_;
  return ok;
}
bool SimpleMotion::frameTimed(const uint16_t raw[12],Drive drive){
  timing::Section section(timing::FRAME);
  if(timing::state.active)++timing::state.working.frames;
  const bool previous=stopIo_;
  if(stopFrame_){stopRaw_=raw;stopIo_=true;}
  const bool ok=port_->frame(raw,drive);
  stopRaw_=nullptr;stopIo_=previous;return ok;
}
bool SimpleMotion::stopFrameAuthorized(const uint16_t raw[12],Drive drive) const {
  return raw&&stopRaw_&&stopActive_&&stopFrame_&&stopIo_&&!safeOffRequest_.load()&&!stop_.load()&&!offPending_&&!faultLatched_&&
    drive.speed==stopDrive_.speed&&drive.acc==stopDrive_.acc&&std::equal(raw,raw+12,stopRaw_);
}
const char* name(State s){switch(s){case State::READY:return "READY";case State::MOVING:return "MOVING";case State::STANDING:return "STANDING";case State::WALKING:return "WALKING";case State::STOPPED:return "STOPPED";case State::FAULT:return "FAULT";}return "FAULT";}
const char* SimpleMotion::phase() const {
  static const char* const p[]={"IDLE","ACQUIRE","REFERENCE","ENABLE","DISENGAGE","ALIGN","STAND_DEPLOY","STAND_LIFT","HEIGHT","HOLD","WALK_STANDARD","WALK_SETTLE","PRONE_HEIGHT","PRONE_LIFT","PRONE_DEPLOY","PRONE_LAND","STOP"};
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
  loadedSinceUs_=0;holdReason_=nullptr;
  // A repeated STOP while the same SAFE_OFF is still pending keeps its
  // broadcast and 13-servo verification progress; a failed read still
  // re-broadcasts in serviceOff. A fault, or a STOP after a fault, during
  // motion or after completion starts SAFE_OFF again from the broadcast.
  const bool repeated=!fault&&state_==State::STOPPED&&offPending_;
  faultLatched_=faultLatched_||fault;state_=faultLatched_?State::FAULT:State::STOPPED;reason_=r;
  stopActive_=stopVerified_=stopIo_=stopFrame_=false;stopRaw_=nullptr;issuedUs_=previousUs_=previous2Us_=0;
  if(!fault)operatorStopped_=strcmp(r,"BOOTING")!=0;
  segmentBox_=nullptr;segmentTo_.valid=false;planned_=false;settleUs_=0;imuStartUs_=0;
  phase_=Phase::IDLE;offPending_=true;proneRest_=false;if(!repeated){offSent_=false;offMask_=0;offNext_=0;}
  walkRequested_=joystick_=false;walkStopping_=walkRecover_=false;pendingGoalValid_=false;rollCorrection_=pitchCorrection_=correctionScale_=0;
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
  if(safeOffRequest_.exchange(false,std::memory_order_acq_rel)){
    stop_.store(false,std::memory_order_release);operatorStopped_=true;
    if(!offPending_||proneRest_)enterOff(false,faultLatched_?reason_:"SAFE_OFF_REQUESTED");
    serviceOff();return true;
  }
  if(!stop_.exchange(false,std::memory_order_acq_rel))return false;
  // An explicit STOP at the PRONE rest is the operator latch: while its OFF is
  // pending the progress is kept, once verified the usual SAFE_OFF restarts.
  if(proneRest_)enterOff(false,stopReason_.load());
  if(faultLatched_||offPending_){serviceOff();return true;}
  if(stopActive_)return false; // duplicate never restarts the clock or dwell
  holdReason_=nullptr;beginStop(esp_timer_get_time());return true;
}
void SimpleMotion::rememberReference(uint64_t now){
  // Called BEFORE commandedRaw_ is replaced following a successful TX, or
  // for an unchanged RAW goal. Never publish a failed/preempted frame.
  // now is the instant the reference was sampled at, not TX completion:
  // compute/bus latency must not shorten the secant of the reference.
  if(issuedUs_&&now>issuedUs_){
    previous2Us_=previousUs_;std::copy(previousRaw_,previousRaw_+12,previous2Raw_);
    previousUs_=issuedUs_;std::copy(commandedRaw_,commandedRaw_+12,previousRaw_);}
  issuedUs_=now;
  if(stopActive_)return; // keep the revoked operation metadata, not stale RAW history
  // A pending posture command first releases the levelled HOLD reference.
  // Its samples are synchronous HOLD, not an endpoint-only native move. Set
  // this before send() services a STOP which may have arrived during TX.
  issuedPhase_=pendingGoalValid_&&(phase_==Phase::HEIGHT||phase_==Phase::PRONE_HEIGHT)?Phase::HOLD:phase_;issuedPlanned_=planned_;
}
// Reference rate for a STOP plan: secant between q (sampled at atUs) and the
// most recent accepted reference at least one release period older, from the
// history (current, previous, previous but one), after skipping `skip`
// entries; the oldest available if none spans a period. Two references issued
// a few ms apart differ by RAW quantization (one tick over 1.5 ms would read
// as ~1 rad/s, a rate no law commands), so the window is never shorter than
// the reference's own sampling period when the history allows it. The span
// is bounded to 100 ms (stale history); only C0 is claimed at the handoff.
bool SimpleMotion::rateFromHistory(const double q[12],uint64_t atUs,unsigned skip,double vmax,double v[12]) const {
  const uint16_t* raws[3]={commandedRaw_,previousRaw_,previous2Raw_};const uint64_t times[3]={issuedUs_,previousUs_,previous2Us_};
  const uint16_t* far=nullptr;uint64_t farUs=0;
  for(unsigned k=skip;k<3;++k){if(!times[k]||atUs<=times[k])break;far=raws[k];farUs=times[k];if(atUs-times[k]>=kPeriodUs)break;}
  if(!far||atUs-farUs>100000)return false;
  const double dt=(atUs-farUs)/1e6;
  for(unsigned i=0;i<12;++i){double old=0;if(!port_->decodeRaw(i,far[i],old))return false;v[i]=std::clamp((q[i]-old)/dt,-vmax,vmax);}
  return true;
}
void SimpleMotion::beginStop(uint64_t now){
  // Revoke the old operation immediately. STOP never calls nextStand, PRONE or
  // WALK STOP, and never enables a previously OFF servo.
  walkRequested_=pendingGoalValid_=joystick_=false;operatorStopped_=true;
  if(!energized_||phase_==Phase::REFERENCE||phase_==Phase::ENABLE){enterOff(false,stopReason_.load());serviceOff();return;}
  if(!input_.powerOk){fault(input_.powerReason);serviceOff();return;}
  if(!input_.ready||!input_.authorityOk){fault(input_.refusal);serviceOff();return;}
  if(!input_.imuValid||now<input_.imuStampUs||now-input_.imuStampUs>600000){fault("IMU_STALE");serviceOff();return;}
  if(!fresh(now)){fault("SERVO_FEEDBACK_STALE");serviceOff();return;}
  if(lastReleaseUs_&&now-lastReleaseUs_>100000){fault("MOTION_DEADLINE_LOST");serviceOff();return;}
  // Native moves have only an endpoint, not a synchronous in-flight reference.
  // Never pretend that a distant goal is the current pose. Retain the existing
  // fail-closed SAFE_OFF if control cannot construct an authorized reference.
  // WALK_SETTLE re-sends the final gait frame each release, like HOLD: same WALK origin.
  const auto source=phase_==Phase::HOLD?phase_:issuedPhase_==Phase::WALK_HOLD?Phase::WALK:issuedPhase_;
  if(!goal_.valid||!issuedUs_||now<issuedUs_||now-issuedUs_>100000||
      (!issuedPlanned_&&source!=Phase::WALK&&source!=Phase::HOLD)){
    fault("STOP_REFERENCE_UNAVAILABLE");serviceOff();return;
  }
  // The authoritative reference is what the writer accepted, including
  // quantization and the actually applied IMU correction. A secant defines
  // only a new bounded reference rate; it is not a measured motor velocity
  // or an analytic derivative of the preceding IMU-corrected signal.
  double q[12]{},v[12]{},a[12]{},lo[12],hi[12];JointTargetFrame start;start.valid=true;
  if(!(previousUs_&&issuedUs_>previousUs_&&issuedUs_-previousUs_<=100000)){fault("STOP_REFERENCE_UNAVAILABLE");serviceOff();return;}
  for(unsigned i=0;i<12;++i){if(!port_->decodeRaw(i,commandedRaw_[i],q[i])){fault("STOP_REFERENCE_UNAVAILABLE");serviceOff();return;}jointSlot(start,i)=q[i];}
  // Rate: secant from the current reference back over at least one period
  // (rateFromHistory, the same estimate compensationSafe proves against).
  // Zero is the chosen initial acceleration of the NEW reference. We do not
  // fit or promise continuity of an undefined prior acceleration.
  if(!rateFromHistory(q,issuedUs_,1,commandedDrive_.speed*kTickRad,v)){fault("STOP_REFERENCE_UNAVAILABLE");serviceOff();return;}
  stopBox_=source==Phase::WALK?&walkBox():segmentBox_;
  if(!stopBox_)stopBox_=contains(collision::kHeight,start)?&collision::kHeight:&collision::kLift;
  if((!contains(*stopBox_,measured())||!contains(*stopBox_,start))&&contains(collision::kLift,start)&&contains(collision::kLift,measured()))stopBox_=&collision::kLift;
  for(unsigned i=0;i<12;++i){
    lo[i]=std::fmax(stopBox_->lo[i],input_.minRad[i]);hi[i]=std::fmin(stopBox_->hi[i],input_.maxRad[i]);
  }
  stopDrive_=commandedDrive_;
  if(!stopPlan_.build(q,v,a,lo,hi,stopDrive_.speed*kTickRad,stopDrive_.acc*100*kTickRad)){
    fault("STOP_REFERENCE_OUTSIDE_BOUNDS");serviceOff();return;
  }
  // q starts exactly at the decoded accepted RAW. All times use the ESP
  // clock; no unwritten nominal point or asynchronous encoder pose is used.
  stopOrigin_=source;walkStopping_=walkRecover_=false;
  stopActive_=true;stopVerified_=false;stopStartUs_=now;stopTerminalUs_=stopDwellUs_=0;
  // One existing control-read population takes 1.44 s, plus the 0.5 s dwell.
  stopDeadlineUs_=now+static_cast<uint64_t>(std::ceil(stopPlan_.duration*1e6))+3000000;
  segmentBox_=stopBox_;planned_=false;settleUs_=0;heightValid_=false;
  phase_=Phase::STOP;state_=State::MOVING;reason_="STOP_IN_PROGRESS";
}
void SimpleMotion::generateStop(uint64_t now){
  if(serviceStop()||!stopActive_||stopTimedOut(esp_timer_get_time()))return;
  now=esp_timer_get_time(); // same clock as writer publication, after feedback I/O
  if(!pointSafe(measured())||!contains(*stopBox_,measured())){fault("STOP_MEASURED_OUTSIDE_BOX");return;}
  JointTargetFrame q;q.valid=true;double p[12];stopPlan_.sample((now-stopStartUs_)/1e6,p);
  for(unsigned i=0;i<12;++i)jointSlot(q,i)=p[i];
  stopFrame_=true;const bool sent=send(q,stopDrive_,now);stopFrame_=false;
  if(!stopActive_)return;
  if(serviceStop()||stopTimedOut(esp_timer_get_time())||!sent)return;
  if(stopVerified_){
    // Acquisition is terminal. Keep its RAW reference and result; a later
    // disturbance is supervised as a hold, never against the old brake deadline.
    // Freshness, torque/goal, authority, electrical and box guards still run.
    for(unsigned i=0;i<12;++i)if(std::abs(fb_[i].q-jointSlot(goal_,i))>kLoadedTicks*kTickRad||
        (fb_[i].load&1023)>kLoadedMaxPermille){fault("STOP_HOLD_TRACKING_UNSAFE");return;}
    return;
  }
  if((now-stopStartUs_)/1e6<stopPlan_.duration)return;
  if(!stopTerminalUs_)stopTerminalUs_=now;
  bool resting=fresh(esp_timer_get_time());
  for(unsigned i=0;i<12;++i)if(fb_[i].stampUs<=stopTerminalUs_||fb_[i].controlStampUs<=stopTerminalUs_||
      fb_[i].torque!=1||fb_[i].goal!=commandedRaw_[i]||fb_[i].goalSpeed!=stopDrive_.speed||fb_[i].acc!=stopDrive_.acc||magnitude(fb_[i].speed)>50||
      std::abs(fb_[i].q-jointSlot(goal_,i))>kLoadedTicks*kTickRad||(fb_[i].load&1023)>kLoadedMaxPermille)resting=false;
  if(resting){
    if(!stopDwellUs_){stopDwellUs_=now;for(unsigned i=0;i<12;++i)stopDwellRaw_[i]=fb_[i].raw;}
    for(unsigned i=0;i<12;++i)if(std::abs(int(fb_[i].raw)-int(stopDwellRaw_[i]))>kLoadedCreepTicks)resting=false;
  }
  if(!resting){stopDwellUs_=0;stopVerified_=false;state_=State::MOVING;reason_="STOP_IN_PROGRESS";}
  else if(now-stopDwellUs_>=kLoadedSettleUs){stopVerified_=true;state_=State::STOPPED;reason_=holdReason_?holdReason_:"STOPPED_SUPPORT_UNVERIFIED";}
  stopTimedOut(esp_timer_get_time());
}
// A PRONE segment that cannot hand over by its deadline, or whose loaded pose
// leaves the segment box but stays in a certified box, is not a physical fault: the body is still carried by 12 torque-ON servos. Arrest it through
// the ordinary contextual STOP (same reference, box, freshness, power, IMU and
// deadline guards; any failed guard is the usual fault + SAFE_OFF) instead of
// releasing the weight at an intermediate height. The hold keeps every
// electrical, thermal, feedback and tracking guard; the operator chooses the
// existing STAND recovery (certified region) or SAFE_OFF.
void SimpleMotion::proneNotCompleted(uint64_t now,const char* held){
  stopReason_.store("PRONE_NOT_COMPLETED");holdReason_=held;
  beginStop(now);
  if(stopActive_)reason_="PRONE_NOT_COMPLETED_STOPPING";
}
bool SimpleMotion::stopTimedOut(uint64_t now){
  if(stopActive_&&!stopVerified_&&now>stopDeadlineUs_){fault("STOP_NOT_SETTLED");return true;}
  return false;
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
  if(contains(collision::kWalk,q)||contains(collision::kSteer,q)){route=InitialRoute::WALK;return true;}
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
         contains(collision::kDeploy,q)||contains(collision::kLift,q)||contains(collision::kSteer,q);
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
// A low HOLD can rest loaded outside kHeight: it starts the same move as the
// LIFT -> HEIGHT handover when that leads back into kHeight, otherwise (a change
// between low heights) stays wholly inside kLift. Boxes and targets are unchanged.
bool SimpleMotion::heightStartClear(const JointTargetFrame& actual,const JointTargetFrame& q) const {
  return transitionSafe(actual,q,collision::kHeight)||entersHeight(actual,q)||
    (contains(collision::kLift,segmentTo_)&&transitionSafe(actual,q,collision::kLift));
}
bool SimpleMotion::heightTarget(double h,JointTargetFrame& q) const {
  if(!std::isfinite(h)||h<kMinStandM||h>.150)return false; // effective C4 workspace
  const double body=baseLinkHeightFromNominalM(h);auto d=canonicalStandDefinition();d.lowBodyHeightM=body/2;d.standBodyHeightM=body;
  const auto s=generateStandTarget(d);q=s.target;return s.status==StandStatus::OK&&pointSafe(q)&&contains(collision::kHeight,q);
}
bool SimpleMotion::walkRecoveryStart(WalkRecoveryStart& start) const {
  auto& held=start.held;auto& first=start.first;auto& c=start.cycles;auto& stopCycle=start.stopCycle;
  // The held STOP reference is the gait extrapolated by half the arrest time:
  // resume the law there. A WALK STOP already in progress keeps its terminal,
  // so planted feet keep their anchors; otherwise it starts at the next touchdown.
  if(!port_)return false;
  held.valid=true;
  for(unsigned i=0;i<12;++i)if(!port_->decodeRaw(i,commandedRaw_[i],jointSlot(held,i)))return false;
  const auto& g=gait_;
  c=cycles_+.5*stopPlan_.duration/kWalkPeriodS;
  stopCycle=stopCycle_>=0?stopCycle_:std::fmax(kWalkStartCycles,std::ceil(c*4)/4);
  c=std::fmin(c,stopCycle+g.stopCycles+gaitSettleCycles(g));
  auto track=track_;  // the admission check never advances the live track
  first=solveGait(g,gaitTrackCartesian(track,g,c,0,0,stopCycle+g.stopCycles),&walkPrevious_,false).target;
  return pointSafe(held)&&contains(walkBox(),held)&&first.valid&&pointSafe(first)&&contains(walkBox(),first);
}
const char* SimpleMotion::stand(double h,uint64_t now){
  if(faultLatched_)return "FAULT_STOP_REQUIRED";
  if(offPending_||safeOffRequest_.load()||stop_.load())return "SAFE_OFF_IN_PROGRESS";
  // Reuse this admission's computed handoff; no second IK solve or persistent cache.
  WalkRecoveryStart recovery;
  if(stopActive_){if(const char* r=standResumeRefusal(now,recovery))return r;}
  else if(operatorStopped_&&!energized_)return "SAFE_OFF_STAND_REFUSED";
  if(!input_.ready)return input_.refusal;
  if(!input_.powerOk)return "POWER_NOT_READY";
  if(!input_.imuValid)return "IMU_UNAVAILABLE";
  if(state_==State::MOVING||state_==State::WALKING)return "MOTION_BUSY";
  JointTargetFrame q;if(!heightTarget(h,q))return "HEIGHT_OUTSIDE_MATDOG_WORKSPACE";
  if(stopActive_){
    if(stopOrigin_==Phase::WALK){
      // Complete the interrupted WALK STOP with the same gait law: planted feet
      // keep their anchors, a swing lands on its own target, then four-contact
      // recentring and WALK_SETTLE to STAND150. No new walking intent.
      walkRecover_=true;recoverCycles_=recovery.cycles;recoverUs_=0;stopCycle_=recovery.stopCycle;walkStopping_=true;resetLookahead();
      requestedHeight_=h;plannedHeight_=kWalkHeightM;
      stopActive_=stopVerified_=false;operatorStopped_=false;walkRequested_=false;
      phase_=Phase::WALK;state_=State::MOVING;segmentBox_=nullptr;planned_=false;
      reason_="STAND_FROM_STOPPED_WALK";referenceFaultSlot_=12;
      rememberReference(now); // a new STOP before the first release still has a held reference
      timing::arm();exit_.valid=false;loadedMask_=0;loadedPhase_="";
      return nullptr;
    }
    if(stoppedInProneDescent()){
      // Same restart routes as after ENABLE, from the held RAW with torque ON:
      // no ACQUIRE/ENABLE, no new route. The correction restarts from zero
      // (the held RAW already contains the old one), engaged by the usual ramp.
      JointTargetFrame held;if(const char* r=proneRecovery(held,initialRoute_))return r;
      reference_=held;segmentTo_=held;stopActive_=stopVerified_=false;operatorStopped_=false;
      rollCorrection_=pitchCorrection_=correctionScale_=0;imuStartUs_=now;compensatedUs_=0;
      referenceFaultSlot_=12;requestedHeight_=h;walkRequested_=false;state_=State::MOVING;reason_="STAND_FROM_STOPPED_PRONE";
      timing::arm();exit_.valid=false;loadedMask_=0;loadedPhase_="";
      switch(initialRoute_){
        case InitialRoute::ENTRY:phase_=Phase::ENABLE;nextStand(now);break;  // DISENGAGE first
        case InitialRoute::DEPLOY:beginDeploy(&held,now);break;
        case InitialRoute::LIFT:beginLift(&held,now);break;
        default:{double z=0;for(unsigned l=0;l<4;++l){const auto c=contactForwardKinematics(static_cast<LegId>(l),held.legs[l]);
            if(c.status!=ContactStatus::OK){fault("STAND_CONTACT_IK");return nullptr;}z-=c.referenceM.z/4;}
          beginHeight(held,std::fmin(.150,std::fmax(kMinStandM,nominalHeightFromBaseLinkM(z))),collision::kHeight,now);}
      }
      if(state_==State::MOVING)rememberReference(now);  // a new STOP before the first release still has a held reference
      return nullptr;
    }
    if((stopOrigin_==Phase::HOLD||stopOrigin_==Phase::HEIGHT)&&!heightStartClear(measured(),q))return "HEIGHT_PATH_NOT_CLEAR";
    if(stopOrigin_==Phase::HEIGHT){
      // A stopped synchronous HEIGHT reference can restart the same existing
      // HEIGHT planner from exact held RAW. The old future endpoint must not
      // become its start. This never re-acquires or enables a servo.
      JointTargetFrame held;held.valid=true;
      for(unsigned i=0;i<12;++i)if(!port_->decodeRaw(i,commandedRaw_[i],jointSlot(held,i)))return "STAND_PATH_NOT_CLEAR";
      double pose[ViaPlan::kDim];plan_.sample(std::clamp((double(stopStartUs_-segmentStartUs_)/1e6+.5*stopPlan_.duration),0.,plan_.duration),pose);
      segmentTo_=held;plannedHeight_=pose[14];
      rollCorrection_=pitchCorrection_=correctionScale_=0;imuStartUs_=now;compensatedUs_=0;
    }
    // Explicit, admitted recovery with existing torque: reuse HEIGHT.
    // ACQUIRE/REFERENCE/ENABLE are never entered here.
    stopActive_=stopVerified_=false;operatorStopped_=false;holdReason_=nullptr;
    state_=State::STANDING;phase_=Phase::HOLD;segmentBox_=nullptr;
    rememberReference(now); // seed the held writer reference for an immediate new STOP
  }
  if(state_==State::STANDING){
    JointTargetFrame actual;actual.valid=true;for(unsigned i=0;i<12;++i)jointSlot(actual,i)=fb_[i].q;
    if(!fresh(now)||!heightStartClear(actual,q))return "HEIGHT_PATH_NOT_CLEAR";
    referenceFaultSlot_=12; // new accepted command: invalidate previous incident
    requestedHeight_=h;walkRequested_=false;
    state_=State::MOVING;phase_=Phase::HEIGHT;reason_="HEIGHT_CHANGE";
    pendingGoal_=q;pendingGoalValid_=true; // intent only; TX at the next Motion release
    segmentDeadlineUs_=now+static_cast<uint64_t>((12+std::fmax(std::abs(rollCorrection_),std::abs(pitchCorrection_))/.020)*1e6);
  }else{  // from torque OFF (boot, PRONE rest): the measured pose decides the route
    referenceFaultSlot_=12;requestedHeight_=h;walkRequested_=false;proneRest_=false;state_=State::MOVING;phase_=Phase::ACQUIRE;acquire_=0;reason_="ACQUIRING_REAL_POSE";
    std::fill(controlFailures_,controlFailures_+12,0);}
  timing::arm();exit_.valid=false;loadedMask_=0;loadedPhase_=""; // LOADDIAG: re-arm the first-exit record
  return nullptr;
}
const char* SimpleMotion::standResumeRefusal(uint64_t now) const {
  WalkRecoveryStart recovery;return standResumeRefusal(now,recovery);
}
// STOP in the PRONE descent: the held RAW (the reference the servos keep, not
// the interrupted segment's future endpoint) must lie, with the measured pose,
// in one certified region box; that region's existing restart route (as after
// ENABLE) is then clear for any joint progress. ENTRY also needs its DISENGAGE.
const char* SimpleMotion::proneRecovery(JointTargetFrame& held,InitialRoute& route) const {
  held.valid=true;for(unsigned i=0;i<12;++i)if(!port_->decodeRaw(i,commandedRaw_[i],jointSlot(held,i)))return "STAND_PATH_NOT_CLEAR";
  if(!certifiedRegion(held,route)||route==InitialRoute::WALK)return "STOP_RECOVERY_POSTURE_UNQUALIFIED";
  const auto& box=route==InitialRoute::ENTRY?collision::kEntry:route==InitialRoute::DEPLOY?collision::kDeploy:
                  route==InitialRoute::LIFT?collision::kLift:collision::kHeight;
  const auto actual=measured();
  if(!contains(box,actual)||!contains(box,held)||(route!=InitialRoute::ENTRY&&!pointSafe(actual)))return "STAND_PATH_NOT_CLEAR";
  if(route==InitialRoute::ENTRY){auto d=held;const auto h=routePoint(0);for(unsigned l=0;l<4;++l)d.legs[l].lower=h.legs[l].lower;
    if(!transitionSafe(actual,d,collision::kEntry))return "STAND_PATH_NOT_CLEAR";}
  return nullptr;
}
const char* SimpleMotion::standResumeRefusal(uint64_t now,WalkRecoveryStart& recovery) const {
  if(faultLatched_)return "FAULT_STOP_REQUIRED";
  if(offPending_||safeOffRequest_.load()||stop_.load()||!energized_)return "SAFE_OFF_STAND_REFUSED";
  if(!stopActive_||!stopVerified_)return "STOP_IN_PROGRESS";
  if(!input_.ready)return input_.refusal;
  if(!input_.powerOk)return "POWER_NOT_READY";
  if(!input_.authorityOk)return "ACTUATOR_OWNER_BUSY";
  if(!input_.imuValid||now<input_.imuStampUs||now-input_.imuStampUs>200000)return "IMU_UNAVAILABLE";
  if(!fresh(now))return "STOP_RECOVERY_FEEDBACK_UNAVAILABLE";
  for(const auto& f:fb_)if(f.torque!=1||!f.controlStampUs||now<f.controlStampUs||now-f.controlStampUs>12*6*kPeriodUs+250000)return "STOP_RECOVERY_TORQUE_UNVERIFIED";
  if(stopOrigin_==Phase::WALK){
    // Kinematic admission only: encoders and the gait law never prove contact.
    if(!walkRecoveryStart(recovery)||!transitionSafe(measured(),recovery.held,walkBox())||
       !transitionSafe(recovery.held,recovery.first,walkBox()))return "STOP_RECOVERY_PATH_NOT_CLEAR";
    return nullptr;
  }
  if(stoppedInProneDescent()){JointTargetFrame held;InitialRoute route;return proneRecovery(held,route);}
  const bool height=stopOrigin_==Phase::HEIGHT&&issuedPlanned_&&stopBox_==&collision::kHeight;
  if(stopOrigin_!=Phase::HOLD&&!height)return "STOP_RECOVERY_POSTURE_UNQUALIFIED";
  InitialRoute route;const auto actual=measured();
  if(!certifiedRegion(actual,route)||!pointSafe(actual))return "STAND_PATH_NOT_CLEAR";
  if(height){
    if(!contains(collision::kHeight,actual))return "HEIGHT_PATH_NOT_CLEAR";
    JointTargetFrame held;held.valid=true;for(unsigned i=0;i<12;++i)
      if(!port_->decodeRaw(i,commandedRaw_[i],jointSlot(held,i)))return "STAND_PATH_NOT_CLEAR";
    if(!pointSafe(held)||!contains(collision::kHeight,held))return "HEIGHT_PATH_NOT_CLEAR";
  }
  if(!heightStartClear(actual,segmentTo_))return "HEIGHT_PATH_NOT_CLEAR";
  return nullptr;
}
const char* SimpleMotion::prone(uint64_t now){
  if(stopActive_)return stopVerified_?"STOP_LATCHED":"STOP_IN_PROGRESS";
  if(stopping())return "SAFE_OFF_IN_PROGRESS";
  if(faultLatched_)return "FAULT_STOP_REQUIRED";
  if(!input_.ready)return input_.refusal;
  if(!input_.powerOk)return "POWER_NOT_READY";
  if(!input_.imuValid||now<input_.imuStampUs||now-input_.imuStampUs>200000)return "IMU_UNAVAILABLE";
  if(state_!=State::STANDING||phase_!=Phase::HOLD||!energized_)return "PRONE_REQUIRES_STAND_HOLD";
  if(!fresh(now)||!segmentTo_.valid)return "PRONE_FEEDBACK_UNAVAILABLE";
  const auto& box=plannedHeight_>kProneHandoverM?collision::kHeight:collision::kLift;
  if(!pointSafe(measured())||!contains(box,measured())||!contains(box,segmentTo_))return "PRONE_PATH_NOT_CLEAR";
  // Intent only. Keep the preceding nominal support goals until a Motion release.
  // kProneHandoverM is inside both existing boxes, including the recorded loaded sag.
  pendingGoalValid_=true;walkRequested_=false;state_=State::MOVING;phase_=Phase::PRONE_HEIGHT;
  segmentDeadlineUs_=now+static_cast<uint64_t>((12+std::fmax(std::abs(rollCorrection_),std::abs(pitchCorrection_))/.020)*1e6);
  reason_="PRONE_DESCENT_NOT_VALIDATED";requestedHeight_=0;heightValid_=false;
  timing::arm();exit_.valid=false;loadedMask_=0;loadedPhase_="";referenceFaultSlot_=12;
  return nullptr;
}
const char* SimpleMotion::walk(unsigned stride,uint64_t now){
  // dev.17: after a verified STOP, WALK is admitted exactly as STAND after STOP
  // (standResumeRefusal: torque, feedback, IMU, certified path from the STOP
  // origin) and takes stand()'s resume route to the WALK height, then walks.
  // A STOP in progress, SAFE_OFF and a latched fault still refuse it.
  if(stopping()&&!(stopActive_&&stopVerified_))return stopActive_?"STOP_IN_PROGRESS":"SAFE_OFF_IN_PROGRESS";
  if(stride>=3)return "STANDARD_STRIDE_REQUIRED";
  if(state_==State::WALKING)return "ALREADY_WALKING";
  // One user command can acquire, energize, stand at the WALK height and then walk.
  const char* r=stand(kWalkHeightM,now);if(r)return r;
  requestedStride_=stride;joystick_=false;walkRequested_=true;
  // A resumed WALK STOP finishes with its own gait and tilt envelope; the
  // requested stride starts at HOLD (startWalk).
  if(phase_!=Phase::WALK){stride_=stride;gait_=walkGait(stride);}
  return nullptr;
}
const char* SimpleMotion::drive(double vx,double vy,double yawRate,uint64_t now){
  if(stopping())return stopActive_?(stopVerified_?"STOP_LATCHED":"STOP_IN_PROGRESS"):"SAFE_OFF_IN_PROGRESS";
  const double goal[3]={vx*kWalkPeriodS,vy*kWalkPeriodS,yawRate*kWalkPeriodS},limit[3]={kJoystickStrideM,kJoystickLateralM,kJoystickYawRad};
  double norm=0;for(unsigned j=0;j<3;++j){if(!std::isfinite(goal[j]))return "JOYSTICK_VELOCITY_RANGE";norm+=(goal[j]/limit[j])*(goal[j]/limit[j]);}
  if(norm==0||norm>1+1e-9)return "JOYSTICK_VELOCITY_RANGE";
  // A walk keeps the leg order of its fore/aft direction: forward, or its time
  // reverse backward. The opposite direction ends it with WALK STOP; the next
  // command after STANDING starts the other one. Lateral and yaw are continuous.
  const bool reverse=goal[0]<0;
  if(state_==State::WALKING){
    if(!joystick_)return "WALK_STANDARD_ACTIVE";
    if(walkStopping_)return "WALK_STOPPING";
    if(goal[0]!=0&&reverse!=gait_.reverse){walkStop();reason_="JOYSTICK_REVERSING";return nullptr;}
  }else if(joystick_&&walkRequested_){
    if(goal[0]!=0)gait_.reverse=reverse;  // still rising: the walk starts with the latest direction
  }else{
    // One command can acquire, energize, stand at the WALK height and walk.
    const char* r=stand(kWalkHeightM,now);if(r)return r;
    gait_=steerGait(goal);walkRequested_=true;
  }
  joystick_=true;for(unsigned j=0;j<3;++j)joystickGoal_[j]=goal[j];joystickDeadlineUs_=now+kJoystickTimeoutUs;
  return nullptr;
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
  if(stopActive_&&!stopFrame_)return false;
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
     std::equal(raw,raw+12,commandedRaw_)){goal_=q;rememberReference(now);return true;}
  if(!frameTimed(raw,drive)){
    // A repeated STOP may preempt even its authorized frame. Retry the same
    // arrest clock next release; intentional preemption is not a bus failure.
    if(port_->framePreempted()&&(stop_.load()||safeOffRequest_.load()))serviceStop();else fault("SYNC_WRITE_FAILED");return false;
  }
  rememberReference(now);
  commandedDrive_=drive;goal_=q;for(unsigned i=0;i<12;++i)commandedRaw_[i]=raw[i];goalSentUs_=now;
  if(serviceStop())return false; // include the completed atomic UART burst in STOP history
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
  // The positioning allowance is not proof of entry into the next group.
  // Keep the current safe reference and its deadline until tracking gets there.
  const auto* next=proneHandoverBox();
  return !next||contains(*next,measured());
}
const collision::Box* SimpleMotion::proneHandoverBox() const {
  switch(phase_){
    case Phase::PRONE_HEIGHT:return &collision::kLift;
    case Phase::PRONE_LIFT:return &collision::kDeploy;
    case Phase::PRONE_DEPLOY:return &collision::kEntry;
    default:return nullptr;
  }
}
// LIFT -> HEIGHT only (RCA 29/10): at the 50 mm lift top a loaded LOWER rests
// a few ticks outside kHeight (robot: LF_LOWER 7 ticks) although the pose is
// inside kLift. HEIGHT starts from the nominal lift top, a two-via plan that is
// monotonic per joint. A joint outside kHeight is admitted only if that
// nominal start is inside kHeight and HEIGHT moves the joint away from the
// violated bound, i.e. back into the box. Everything else stays as before.
bool SimpleMotion::entersHeight(const JointTargetFrame& actual,const JointTargetFrame& q) const {
  if(!segmentTo_.valid||!q.valid||!pointSafe(actual)||!contains(collision::kLift,actual))return false;
  const auto& b=collision::kHeight;
  for(unsigned i=0;i<12;++i){
    const double a=jointSlot(actual,i),n=jointSlot(segmentTo_,i),d=jointSlot(q,i)-n;
    if(a>=b.lo[i]&&a<=b.hi[i])continue;
    if(n<b.lo[i]||n>b.hi[i]||!(a<b.lo[i]?d>0:d<0))return false;
  }
  return true;
}
bool SimpleMotion::liftEntersHeight(const JointTargetFrame& actual) const {
  JointTargetFrame q;
  return phase_==Phase::LIFT&&std::abs(plannedHeight_-requestedHeight_)>1e-9&&heightTarget(requestedHeight_,q)&&entersHeight(actual,q);
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
    case Phase::HEIGHT:case Phase::WALK_HOLD:allowed=0xfff;break;
    case Phase::PRONE_HEIGHT:case Phase::PRONE_LIFT:case Phase::PRONE_DEPLOY:allowed=0xfff;next=proneHandoverBox();break;
    case Phase::PRONE_LAND:allowed=0xfff;next=&collision::kEntry;break;
    default:break;  // moving gait and ordinary HOLD do not complete a native segment
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
  // A descent to a low height rests loaded in kLift, where its target also lies.
  const bool lowRest=phase_==Phase::HEIGHT&&contains(collision::kLift,segmentTo_)&&contains(collision::kLift,actual);
  ok=ok&&loaded&&pointSafe(actual)&&(lowRest||(contains(*segmentBox_,actual)&&(contains(*next,actual)||liftEntersHeight(actual))));
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
  actualHeight_=nominalHeightFromBaseLinkM(sum/4); heightValid_=hi-lo<.015&&(phase_==Phase::HOLD||phase_==Phase::HEIGHT);
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
void SimpleMotion::beginPlan(Phase next,const collision::Box& box,uint64_t now,double speed,double acc){
  // The whole time-continuous group lies in one certified box: each joint
  // stays within the range of its via points, which are all checked here.
  if(plan_.n<1||!plan_.plan(speed,acc)){fault("STAND_PLAN_UNAVAILABLE");return;}
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
  const double releaseLevel=next==Phase::WALK_HOLD&&std::abs(plannedHeight_-requestedHeight_)>1e-9?std::fmax(std::abs(rollCorrection_),std::abs(pitchCorrection_))/.020:0;
  segmentDeadlineUs_=settleUs_+static_cast<uint64_t>((12+releaseLevel+2*(distance/(kStandDrive.speed*kTickRad)+2.*kStandDrive.speed/(kStandDrive.acc*100.)))*1e6);
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
void SimpleMotion::beginProne(Phase next,uint64_t now){
  // Same ViaPlan/FK/IK, native RAM profile and independent-joint box proof as
  // STAND. No target is based on the loaded encoder sag as a new nominal pose.
  // PRONE-only front LOWER floor: +20 mrad from the stand route's folded
  // target. Recorded STAND50 sag otherwise exits kLift; STAND100/150 leave
  // only 1-2 ticks. Keep all boxes/gen1 and the operational STAND untouched.
  constexpr double frontLowerFloor=-1.5246163880149817;
  auto pronePoint=[&](unsigned k){auto q=routePoint(k);
    for(unsigned l=0;l<2;++l)q.legs[l].lower=std::fmax(q.legs[l].lower,frontLowerFloor);
    return q;};
  auto proneVia=[&](unsigned k){const auto* p=paired_data::kKnots[k].p;
    return via(pronePoint(k),p[4],p[5],p[6]);};
  plan_.clear();const collision::Box* box=nullptr;
  if(next==Phase::PRONE_HEIGHT){
    box=&collision::kHeight;
    if(!via(segmentTo_,0,0,plannedHeight_)||!heightVias(plannedHeight_,kProneHandoverM)){fault("PRONE_PATH_NOT_CLEAR");return;}
    plannedHeight_=kProneHandoverM;
  }else if(next==Phase::PRONE_LIFT){
    box=&collision::kLift;
    if(!via(segmentTo_,0,0,plannedHeight_)){fault("PRONE_PATH_NOT_CLEAR");return;}
    for(unsigned j=sizeof(collision::kLiftKnots)/sizeof(unsigned);j-->0;){const auto k=collision::kLiftKnots[j];
      if(knotHeight(k)<plannedHeight_-1e-6&&!proneVia(k)){fault("PRONE_PATH_NOT_CLEAR");return;}}
  }else if(next==Phase::PRONE_DEPLOY){
    box=&collision::kDeploy;
    for(unsigned j=sizeof(collision::kDeployKnots)/sizeof(unsigned);j-->0;)
      if(!proneVia(collision::kDeployKnots[j])){fault("PRONE_PATH_NOT_CLEAR");return;}
  }else if(next==Phase::PRONE_LAND){
    box=&collision::kEntry;
    auto q=pronePoint(0);for(unsigned l=0;l<4;++l)q.legs[l].hip=(l==0||l==3?1:-1)*.17;
    if(!proneVia(0)||!via(q,.08183102100559651,0,.009626496609908074)){fault("PRONE_PATH_NOT_CLEAR");return;}
  }else {fault("PRONE_PATH_NOT_CLEAR");return;}
  // Check the measured handover too, even when ordinary arrival (12 ticks)
  // rather than loaded-settle completed the preceding segment. No box bypass.
  if(!pointSafe(measured())||!contains(*box,measured())){fault("PRONE_PATH_NOT_CLEAR");return;}
  beginPlan(next,*box,now,next==Phase::PRONE_LAND?.10:kStandSpeedRadS,
                         next==Phase::PRONE_LAND?.20:kStandAccelerationRadS2);
}
void SimpleMotion::nextStand(uint64_t now){
  timing::Section section(timing::NOMINAL);
  const JointTargetFrame actual=measured();
  const double* head=paired_data::kKnots[0].p;
  switch(phase_){
    case Phase::PRONE_HEIGHT:beginProne(Phase::PRONE_LIFT,now);return;
    case Phase::PRONE_LIFT:beginProne(Phase::PRONE_DEPLOY,now);return;
    case Phase::PRONE_DEPLOY:beginProne(Phase::PRONE_LAND,now);return;
    case Phase::PRONE_LAND:
      // PRONE rest: the verified arrival (arrived/settledUnderLoad, as every
      // segment) releases the torque through the ordinary SAFE_OFF: no target
      // is generated or written from here, OFF broadcast, 13/13 readback, the
      // same retries and faults. Not an operator STOP: once verified the state
      // returns to READY and an explicit STAND starts from the settled pose
      // through ACQUIRE/REFERENCE/ENABLE. Passive support is not measured.
      enterOff(false,"PRONE_REST");operatorStopped_=false;proneRest_=true;serviceOff();return;
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
    case Phase::LIFT:case Phase::HEIGHT:case Phase::WALK_HOLD:
      if(std::abs(plannedHeight_-requestedHeight_)>1e-9){beginHeight(segmentTo_,plannedHeight_,collision::kHeight,now);return;}
      segmentBox_=nullptr;planned_=false;
      rollCorrection_*=correctionScale_;pitchCorrection_*=correctionScale_;correctionScale_=1;
      phase_=Phase::HOLD;state_=State::STANDING;reason_="STAND_REACHED";
      {GaitParameters g;g.heightM=baseLinkHeightFromNominalM(requestedHeight_);holdNominal_=solveGait(g,gaitHold(g,0,g.heightM,0),nullptr,false);}
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
  compensate(f,now);if(!offPending_)send(f.target,kStandDrive,now);
}
void SimpleMotion::startWalk(uint64_t now){
  timing::Section section(timing::NOMINAL);
  if(!joystick_){stride_=requestedStride_;gait_=walkGait(stride_);}
  plannedHeight_=kWalkHeightM;track_={};steered_=joystick_;steeredCycles_=0;resetLookahead();
  // The latest target, from the start ramp; never against the leg order.
  if(steered_){gait_.advanceXM=(joystickGoal_[0]<0)!=gait_.reverse?0:joystickGoal_[0];gait_.advanceYM=joystickGoal_[1];gait_.yawRad=joystickGoal_[2];}
  walkPrevious_=solveGait(gait_,gaitTrackCartesian(track_,gait_,0,0,0),nullptr,false);
  if(!pointSafe(walkPrevious_.target)||!contains(walkBox(),walkPrevious_.target)){fault("WALK_IK_OR_WORKSPACE");return;}
  walkStartUs_=now;walkStopping_=walkRecover_=false;stopCycle_=-1;cycles_=0;
  phase_=Phase::WALK;state_=State::WALKING;reason_=steered_?"WALK_JOYSTICK":"WALK_STANDARD";
}
void SimpleMotion::steerWalk(uint64_t now){
  if(!joystick_||walkStopping_||walkRecover_){for(double& r:track_.goalRate)r=0;return;}
  // Released, unreachable or silent client: the Controller ends the walk itself.
  if(now>joystickDeadlineUs_){walkStop();reason_="JOYSTICK_TIMEOUT";return;}
  auto next=gait_;double* axis[3]={&next.advanceXM,&next.advanceYM,&next.yawRad};
  const double limit[3]={kJoystickStrideM,kJoystickLateralM,kJoystickYawRad},dc=std::fmax(0.,cycles_-steeredCycles_);steeredCycles_=cycles_;
  for(unsigned j=0;j<3;++j){
    const double goal=j==0&&(joystickGoal_[0]<0)!=gait_.reverse?0:joystickGoal_[j],rate=limit[j]/kJoystickRampCycles;  // never against the leg order
    *axis[j]+=std::clamp(goal-*axis[j],-rate*dc,rate*dc);track_.goal[j]=goal;track_.goalRate[j]=rate;}
  // Lateral sway follows the rate-limited lateral/yaw demand, changing only
  // while all four feet are down (last nominal sample): the body never crosses
  // the fillet contact band (SimpleMotion.h) during a three-contact phase.
  const double swayRate=(kJoystickSwayYM-kWalkSwayYM)/kJoystickSwayRampCycles;
  bool fourDown=true;for(const auto& ph:walkPrevious_.cartesian.phases)fourDown=fourDown&&ph.stance;
  if(fourDown)next.swayYM+=std::clamp(joystickSwayYM(next.advanceYM,next.yawRad,gait_.swayYM)-next.swayYM,-swayRate*dc,swayRate*dc);
  if((next.advanceXM!=gait_.advanceXM||next.advanceYM!=gait_.advanceYM||next.yawRad!=gait_.yawRad||next.swayYM!=gait_.swayYM)&&gaitRetarget(track_,gait_,next,cycles_)){
#ifdef MATDOG_T2_SHADOW
    gaitRetarget(shadowTrack_,gait_,next,cycles_);
#endif
    gait_=next;}
}
bool SimpleMotion::limitWalkTilt(double& roll,double& pitch) const {
  if(steered_)return projectWalkTilt(kSteerTilt,roll,pitch);
  return stride_==2?projectWalkTilt(kWalkTilt60,roll,pitch):projectWalkTilt(kWalkTilt45,roll,pitch);
}
JointTargetFrame SimpleMotion::compensationTarget(const LocomotionFrame& f,const BodyPose& body,const JointTargetFrame& seed,bool nominalStrip,double edgeTilt){
  timing::count(timing::SOLVES);
  JointTargetFrame q;q.valid=true;
  for(unsigned l=0;l<4;++l){TiltedIkOptions opt;opt.seed=seed.legs[l];opt.toleranceM=1e-6;opt.maxIterations=8;opt.requireNominalStrip=nominalStrip&&edgeTilt==0;
    // Edge contact: the tread fillet point meets the ground.
    const auto ik=edgeTilt>0?worldEdgeContactInverseKinematicsTilted(static_cast<LegId>(l),f.cartesian.feet[l],body,opt):
                             worldContactInverseKinematicsTilted(static_cast<LegId>(l),f.cartesian.feet[l],body,opt);
    timing::count(timing::IK,ik.base.solves);timing::count(timing::NEWTON,ik.base.newtonSteps);timing::count(timing::FK,ik.base.fkEvaluations);
    // Cylinder axis depends only on HIP. Reserve its exact half-RAW rounding
    // interval so the transmitted pose retains the same contact mode.
    const double tiltLimit=edgeTilt>0?edgeTilt:footContactModel().nominalTiltRad;
    if(ik.status!=TiltedIkResult::Status::OK||((nominalStrip||edgeTilt>0)&&ik.base.contact.axisTiltRad>tiltLimit-.5*kTickRad)){q.valid=false;break;}q.legs[l]=ik.base.joints;}
  return q;
}
bool SimpleMotion::compensationSafe(const JointTargetFrame& q,const collision::Box& box,bool stopProof) const {
  if(!pointSafe(q)||!contains(box,q))return false;
  // The same gen-1 conversion as TX/readback; an in-box floating point IK
  // solution alone is insufficient at a box edge after RAW quantization.
  auto decoded=q;
  for(unsigned i=0;i<12;++i){uint16_t raw=0;if(!port_->resolve(i,jointSlot(q,i),raw)||!port_->decodeRaw(i,raw,jointSlot(decoded,i)))return false;}
  if(!pointSafe(decoded)||!contains(box,decoded))return false;
  const collision::Box* arrestBox=&box;
  if(phase_!=Phase::WALK){
    // Keep the measured static-load residual when checking the next small
    // HOLD step. A goal alone being in-box is insufficient: the recorded
    // loaded pose must retain a common certified region with its reference.
    auto loaded=decoded;double lagRoom[12];const uint64_t at=esp_timer_get_time();
    for(unsigned i=0;i<12;++i){double old=0;if(!port_->decodeRaw(i,commandedRaw_[i],old))return false;
      jointSlot(loaded,i)+=fb_[i].q-old;
      // Feedback is staggered across a population. Reserve the proposed RAW
      // step over its age plus this release, rather than treating it as a
      // simultaneous observation of the current reference under static load.
      lagRoom[i]=std::abs(jointSlot(decoded,i)-old)*(1+double(at>=fb_[i].stampUs?at-fb_[i].stampUs:0)/kPeriodUs);
      if(jointSlot(loaded,i)-lagRoom[i]<input_.minRad[i]||jointSlot(loaded,i)+lagRoom[i]>input_.maxRad[i])return false;}
    auto loadedInside=[&](const collision::Box& b){for(unsigned i=0;i<12;++i)
      if(jointSlot(loaded,i)-lagRoom[i]<b.lo[i]||jointSlot(loaded,i)+lagRoom[i]>b.hi[i])return false;return true;};
    const auto actual=measured();
    if(!contains(box,actual)||!loadedInside(box)){
      if(!contains(collision::kLift,decoded)||!contains(collision::kLift,actual)||!loadedInside(collision::kLift))return false;
      arrestBox=&collision::kLift;
    }
    if(!pointSafe(loaded))return false;
  }
  if(!stopProof)return true;
  // A reachable boundary is not necessarily stoppable from its sampled RAW
  // rate. Reuse STOP's own proof before issuing a new levelled reference;
  // no second trajectory law, relaxed limit or retained cache is introduced.
  const uint64_t at=esp_timer_get_time();
  if(!issuedUs_||at<=issuedUs_||at-issuedUs_>100000)return false;
  const Drive drive=phase_==Phase::WALK?kWalkDrive:kStandDrive;
  // Same secant STOP would take after this send: both samples on the reference clock.
  if(nowUs_<=issuedUs_)return false;
  StopPlan arrest;double lo[12],hi[12];
  for(unsigned i=0;i<12;++i){arrest.q[i]=jointSlot(decoded,i);lo[i]=std::fmax(arrestBox->lo[i],input_.minRad[i]);hi[i]=std::fmin(arrestBox->hi[i],input_.maxRad[i]);}
  if(!rateFromHistory(arrest.q,nowUs_,0,drive.speed*kTickRad,arrest.v))return false;  // same estimate beginStop would take after this send
  return arrest.build(arrest.q,arrest.v,arrest.a,lo,hi,drive.speed*kTickRad,drive.acc*100*kTickRad);
}
// One slew step per Motion release, scaled by the physical time since the last
// one: a group boundary regenerates the reference in the same release and must
// not advance the correction twice; a late release does not slow it down.
double SimpleMotion::slewStep(uint64_t now,uint64_t maxDtUs){
  const uint64_t dt=now==compensatedUs_?0:compensatedUs_&&now>compensatedUs_?std::min<uint64_t>(now-compensatedUs_,maxDtUs):kPeriodUs;
  compensatedUs_=now;return kImuSlewRadS*double(dt)/1e6;
}
void SimpleMotion::rememberCorrection(const JointTargetFrame& accepted,const JointTargetFrame& nominal,uint64_t now){
  reuseOffset_.valid=accepted.valid&&nominal.valid;
  for(unsigned i=0;i<12;++i)jointSlot(reuseOffset_,i)=jointSlot(accepted,i)-jointSlot(nominal,i);
  reusePhase_=phase_;reuseUs_=now;
}
void SimpleMotion::compensateLevel(LocomotionFrame& f,const Vector3& up,uint64_t now,bool unwind){
  const bool walk=phase_==Phase::WALK;
  const collision::Box& safeBox=(walk||phase_==Phase::WALK_HOLD)?walkBox():collision::kHeight;
  // T1 light release (dev.7 WALK budget, restored for WALK and HOLD): while
  // the accepted correction is settled (the last full solve did not step it),
  // every other release keeps it as a joint offset on the new nominal target.
  // Same limits, boxes, RAW, loaded and STOP proof; any slew, phase, gait,
  // IMU, unwind or recovery change forces the full solve.
  // T2 balance: in WALK the light compensation is taken on the releases that
  // paid the nominal IK solve, so the look-ahead release (no nominal solve)
  // carries the full compensation and no release carries both.
  // T1b (motion-perf-next): a steerWalk retarget of the displacement per cycle
  // no longer forces the full solve. The accepted correction is a body
  // attitude offset expressed on the nominal of the previous release; the
  // rate-limited retarget moves that nominal by less than one release of the
  // same law does, and the reused frame still passes compensationSafe here.
  // Offline, with the random joystick runs of run_motion_frame_digest.sh, the
  // light fraction rises from 8 % to 40 % with the same shadow error (1.2 tick).
  const bool reusable=reuseOffset_.valid&&reuseSettled_&&!reuseLight_&&!unwind&&!walkRecover_&&reusePhase_==phase_&&
    (phase_==Phase::WALK||phase_==Phase::HOLD)&&(phase_!=Phase::WALK||nominalSolved_)&&now>reuseUs_&&now-reuseUs_<=2*kPeriodUs+kPeriodUs/2&&
    !(up.x==0&&up.y==0&&up.z==0);
  if(reusable){
    auto q=f.target;for(unsigned i=0;i<12;++i)jointSlot(q,i)+=jointSlot(reuseOffset_,i);
    if(compensationSafe(q,safeBox,true)){
#ifdef MATDOG_T1_SHADOW
      {auto body=headed(f.cartesian.body,rollCorrection_,pitchCorrection_);const auto t=f.cartesian.body.translationM;
       body.translationM={t.x+t.z*body.rotation[0][2],t.y+t.z*body.rotation[1][2],t.z*body.rotation[2][2]};
       const auto exact=(rollCorrection_==0&&pitchCorrection_==0&&walk)?f.target:
         compensationTarget(f,body,goal_.valid?goal_:f.target,true,walk?gait_.edgeTiltRad:0);
       if(exact.valid&&!(phase_==Phase::HOLD&&imuLimited_))for(unsigned i=0;i<12;++i)
         reuseStats_.maxShadowRad=std::fmax(reuseStats_.maxShadowRad,std::abs(jointSlot(exact,i)-jointSlot(q,i)));}
#endif
      f.target=q;reuseLight_=true;++reuseStats_.light;return;}
    ++reuseStats_.rejected;
  }
  // A full solve after a light release steps at most one period, and no more
  // than the wall time since the last issued reference: the correction's rate
  // between consecutive references stays <= kImuSlewRadS, so the RAW secant
  // the STOP proof reads over a short release interval is not inflated by a
  // correction step accumulated over the light release (review 2026-11-08).
  const bool afterLight=reuseLight_;reuseLight_=false;++reuseStats_.full;
  const uint64_t sinceIssuedUs=issuedUs_&&now>issuedUs_?now-issuedUs_:kPeriodUs;
  const double oldRoll=rollCorrection_,oldPitch=pitchCorrection_;
  const JointTargetFrame nominal=f.target;
  // Angles are the accepted body orientation in the existing support frame,
  // not an integral of IMU error. Fit its orientation from labelled stance
  // anchors and fresh encoder FK; the measured gravity direction supplies
  // the world vertical. Contacts remain modelled, never asserted as sensed.
  const auto previous=headed(f.cartesian.body,rollCorrection_,pitchCorrection_);
  double wantedRoll=0,wantedPitch=0;bool stance[4];for(unsigned l=0;l<4;++l)stance[l]=f.cartesian.phases[l].stance;
  if(walk){
    // In a four-contact window omit the newest touchdown from the fit until
    // the next liftoff. The other three planted anchors retain their labels;
    // delayed feedback from the formerly swinging leg cannot tilt the fit.
    unsigned count=0,newest=0;for(unsigned l=0;l<4;++l)if(stance[l]){++count;if(f.cartesian.phases[l].phase<f.cartesian.phases[newest].phase)newest=l;}
    if(count==4)stance[newest]=false;
  }
  const bool fitted=unwind||supportLevelTarget(f.cartesian.feet,measured(),stance,up,previous,wantedRoll,wantedPitch);
  timing::count(timing::FK,unwind?0:3);
  imuLimited_=!fitted;imuReason_=fitted?"OK":up.x==0&&up.y==0&&up.z==0?"IMU_STALE":"SUPPORT_FIT";
  if(!fitted){wantedRoll=rollCorrection_;wantedPitch=pitchCorrection_;}
  if(walk&&limitWalkTilt(wantedRoll,wantedPitch)){imuLimited_=true;imuReason_="ENVELOPE";}
  imuWanted_[0]=wantedRoll;imuWanted_[1]=wantedPitch;
  const double slew=slewStep(now,afterLight?std::min<uint64_t>(kPeriodUs,sinceIssuedUs):kImuSlewMaxDtUs);
  auto step=[&](double old,double wanted){return !unwind&&std::abs(wanted-old)<.001?old:old+std::clamp(wanted-old,-slew,slew);};
  double roll=step(rollCorrection_,wantedRoll),pitch=step(pitchCorrection_,wantedPitch);
  if(walk){const double dr=wantedRoll-rollCorrection_,dp=wantedPitch-pitchCorrection_;
    const double distance=std::fmax(std::abs(dr),std::abs(dp)),fraction=distance>.001?std::fmin(1.,slew/distance):0;
    roll=rollCorrection_+fraction*dr;pitch=pitchCorrection_+fraction*dp;}
  if(!imuLimited_&&std::fmax(std::abs(wantedRoll-roll),std::abs(wantedPitch-pitch))>.001)imuReason_="SLEW";
  if(!walk&&roll==rollCorrection_&&pitch==pitchCorrection_&&goal_.valid){f.target=goal_;rememberCorrection(goal_,nominal,now);reuseSettled_=true;return;}
  if(walk&&roll==0&&pitch==0){correctionScale_=1;rememberCorrection(nominal,nominal,now);reuseSettled_=oldRoll==0&&oldPitch==0;return;} // nominal gait already solved
  auto body=headed(f.cartesian.body,roll,pitch);const auto t=f.cartesian.body.translationM;
  // Keep the nominal vertical clearance, with the body over the same gait
  // XY in gravity's direction. Anchors stay in their support-plane frame.
  body.translationM={t.x+t.z*body.rotation[0][2],t.y+t.z*body.rotation[1][2],t.z*body.rotation[2][2]};
  // Large edge-biased roll solutions do not preserve this model's nominal
  // support strip. Keep its existing contact domain, not a new IMU-angle cap.
  const auto q=compensationTarget(f,body,goal_.valid?goal_:f.target,true,walk||phase_==Phase::WALK_HOLD?gait_.edgeTiltRad:0);
  if(compensationSafe(q,safeBox,!walkRecover_)){
    f.target=q;rollCorrection_=roll;pitchCorrection_=pitch;correctionScale_=1;rememberCorrection(q,nominal,now);reuseSettled_=roll==oldRoll&&pitch==oldPitch;
  }else{
    // Retain the accepted pose at the real workspace boundary. The desired
    // geometric orientation is re-estimated next release, so reversal leaves
    // saturation immediately; there is no accumulated integral to unwind.
    if(walk){fault("WALK_IK_OR_WORKSPACE");return;}
    if(goal_.valid){f.target=goal_;rememberCorrection(goal_,nominal,now);}imuLimited_=true;imuReason_="WORKSPACE";reuseSettled_=true;
  }
}
void SimpleMotion::compensate(LocomotionFrame& f,uint64_t now){
  timing::Section section(timing::COMPENSATE);
  Attitude a;
  const bool attitudeFresh=input_.imuValid&&now>=input_.imuStampUs&&now-input_.imuStampUs<=200000&&
     tiltFromQuaternion(input_.quaternion[0],input_.quaternion[1],input_.quaternion[2],input_.quaternion[3],a);
  if(phase_==Phase::HOLD||phase_==Phase::WALK||phase_==Phase::WALK_HOLD){
    compensateLevel(f,attitudeFresh?Vector3{a.upBase[0],a.upBase[1],a.upBase[2]}:Vector3{},now,
      phase_==Phase::WALK_HOLD&&std::abs(plannedHeight_-requestedHeight_)>1e-9);return;}
  if(!attitudeFresh){imuLimited_=true;imuReason_="IMU_STALE";return;}
  imuLimited_=false;imuReason_="OK";
  // A small proportional BODY correction, slew limited, no servo PID or
  // integral loop. The same target generator owns nominal + correction.
  const double slew=slewStep(now);
  auto desired=[](double tilt){const double error=std::copysign(std::fmax(0.,std::abs(tilt)-.005),tilt);return std::clamp(-.5*error,-.020,.020);};
  const double nominalPitch=std::atan2(-f.cartesian.body.rotation[2][0],f.cartesian.body.rotation[0][0]);
  imuWanted_[0]=desired(a.rollRad);imuWanted_[1]=desired(a.pitchRad-nominalPitch);
  const double roll=rollCorrection_+std::clamp(imuWanted_[0]-rollCorrection_,-slew,slew),pitch=pitchCorrection_+std::clamp(imuWanted_[1]-pitchCorrection_,-slew,slew);
  if(roll!=imuWanted_[0]||pitch!=imuWanted_[1])imuReason_="SLEW";
  if(roll==0&&pitch==0){rollCorrection_=pitchCorrection_=correctionScale_=0;return;}
  // PRONE_HEIGHT hands over to kLift at kProneHandoverM (11.7 ticks inside): once the
  // nominal target is inside it, the correction stays inside it too.
  const auto& box=phase_==Phase::PRONE_HEIGHT&&contains(collision::kLift,f.target)?collision::kLift:segmentBox_?*segmentBox_:collision::kHeight;
  auto keep=[&](const JointTargetFrame& q){f.target=q;};
  auto solve=[&](double scale,const JointTargetFrame& seed){
    auto b=bodyPoseFromRollPitch(f.cartesian.body.translationM,roll*scale,nominalPitch+pitch*scale);
    return compensationTarget(f,b,seed);
  };
  // Smooth geometric attenuation before the hard workspace boundary. A small
  // IK perturbation measures local joint sensitivity; reserve half the joint
  // room. This avoids normal corrections jumping from full to half amplitude
  // as the nominal trajectory approaches a limit. No encoder tracking loop.
  // The last accepted scale is the probe: while the attenuation stays within 2%
  // of it, the probe is already the target and the release needs one solve.
  constexpr double probeScale=1./32;
  const double trialScale=correctionScale_>0?correctionScale_:probeScale;
  const auto probe=solve(trialScale,f.target);double sum=0;
  if(probe.valid)for(unsigned i=0;i<12;++i){const auto& lim=legModel(static_cast<LegId>(i/3))->limits[i%3];
    const double q=jointSlot(f.target,i),delta=(jointSlot(probe,i)-q)/trialScale;
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
  if(projection>0&&std::abs(projection-trialScale)<=.02*projection&&pointSafe(probe)&&contains(box,probe)){
    keep(probe);rollCorrection_=roll;pitchCorrection_=pitch;imuLimited_=projection<.999;if(imuLimited_)imuReason_="ATTENUATED";return;}
  for(unsigned trial=0;trial<5&&projection>0;++trial){const double scale=projection*std::ldexp(1.,-static_cast<int>(trial));
    const auto q=solve(scale,probe.valid?probe:f.target);
    if(pointSafe(q)&&contains(box,q)){keep(q);rollCorrection_=roll;pitchCorrection_=pitch;correctionScale_=scale;imuLimited_=projection<.999||trial!=0;if(imuLimited_)imuReason_="ATTENUATED";return;}
  }
  // If no in-workspace correction exists, the nominal safe target survives.
  rollCorrection_=pitchCorrection_=0;correctionScale_=0;imuLimited_=true;imuReason_="WORKSPACE";
}
namespace {
// Fields of the law other than the steered displacement per cycle and the
// steered sway: a change of these is structural (new walk or WALK STOP
// terminal); advance, lateral, yaw and sway amplitude change through
// gaitRetarget by rate-limited steps that keep the body pose continuous.
bool sameLawShape(const GaitParameters& a,const GaitParameters& b){
  return a.type==b.type&&a.heightM==b.heightM&&a.liftM==b.liftM&&a.duty==b.duty&&a.swayXM==b.swayXM&&
    a.startCycles==b.startCycles&&a.stopCycles==b.stopCycles&&a.reverse==b.reverse&&a.edgeTiltRad==b.edgeTiltRad&&a.maxCondition==b.maxCondition;
}
}
// T2: the reference at the release clock, linear in joint space between the
// exact solutions at walkExactUs_ and walkAheadUs_ (u in [0,1)). Branches
// and conditioning are those of the look-ahead. The Cartesian sample used by
// the IMU compensation is rebuilt for this clock: planted feet keep their
// anchors (the solution on the planted side of a lift-off/touchdown inside
// the bracket), a swinging foot is the contact reference of the interpolated
// joints (the fillet lift the steered IK added is removed again), the body
// pose is interpolated in translation and yaw, the phases are the law's.
void SimpleMotion::lerpFrame(LocomotionFrame& f,double u,const GaitParameters& g,double terminal) const {
  const LocomotionFrame& a=walkExact_;const LocomotionFrame& b=walkAhead_;
  f.status=b.status;f.failedLeg=b.failedLeg;f.minJointMarginRad=std::fmin(a.minJointMarginRad,b.minJointMarginRad);f.maxResidualM=std::fmax(a.maxResidualM,b.maxResidualM);
  for(unsigned l=0;l<4;++l){f.branches[l]=b.branches[l];f.condition[l]=std::fmax(a.condition[l],b.condition[l]);}
  f.target.valid=true;
  for(unsigned i=0;i<12;++i)jointSlot(f.target,i)=jointSlot(a.target,i)+(jointSlot(b.target,i)-jointSlot(a.target,i))*u;
  auto& s=f.cartesian;s.valid=true;s.cycles=cycles_;
  const auto& ta=a.cartesian.body.translationM;const auto& tb=b.cartesian.body.translationM;
  s.body.translationM={ta.x+(tb.x-ta.x)*u,ta.y+(tb.y-ta.y)*u,ta.z+(tb.z-ta.z)*u};
  const double ya=std::atan2(a.cartesian.body.rotation[1][0],a.cartesian.body.rotation[0][0]),yb=std::atan2(b.cartesian.body.rotation[1][0],b.cartesian.body.rotation[0][0]);
  double dy=yb-ya;if(dy>3.14159265358979323846)dy-=2*3.14159265358979323846;else if(dy<-3.14159265358979323846)dy+=2*3.14159265358979323846;
  const double yaw=ya+dy*u,cy=std::cos(yaw),sy=std::sin(yaw);
  s.body.rotation[0][0]=cy;s.body.rotation[0][1]=-sy;s.body.rotation[1][0]=sy;s.body.rotation[1][1]=cy;
  for(unsigned l=0;l<4;++l){
    auto phase=legPhase(g,static_cast<LegId>(l),cycles_);
    if(terminal>=0&&cycles_>=terminal&&!phase.stance){phase.stance=phase.boundary=true;phase.swingProgress=0;}
    s.phases[l]=phase;
    if(phase.stance){s.feet[l]=a.cartesian.phases[l].stance?a.cartesian.feet[l]:b.cartesian.feet[l];continue;}
    const auto c=contactForwardKinematics(static_cast<LegId>(l),f.target.legs[l]);
    Vector3 w{};if(c.status!=ContactStatus::OK||!pointToWorld(s.body,c.referenceM,w)){f.target.valid=false;return;}
    if(g.edgeTiltRad>0)w.z-=edgeContactLiftM(c.axisTiltRad);
    s.feet[l]=w;
  }
}
// T2: the nominal frame of this release, written into the caller's frame
// (target.valid false when a solve failed; the caller faults
// WALK_IK_OR_WORKSPACE as before). Positions only (A1).
void SimpleMotion::nominalRelease(LocomotionFrame& f,const GaitParameters& g,double terminal,bool clockDone){
  // The eased recovery clock and the terminal frame are solved exactly at the
  // release clock, as before T2; the terminal frame also ends the look-ahead.
  if(walkRecover_||clockDone){
    resetLookahead();++lookaheadStats_.exact;
    solveGaitInto(f,g,gaitTrackCartesian(track_,g,cycles_,0,0,terminal),&walkPrevious_,false);return;
  }
  // Exact solutions live on the release grid (update() advanced nextUs_
  // before this release): the law's track is sampled in order, two periods
  // at a time, and every release clock falls inside [exact, ahead).
  const uint64_t gridUs=nextUs_-kPeriodUs;
  auto cyc=[&](uint64_t us){return double(us-walkStartUs_)/1e6/kWalkPeriodS;};
  const bool shape=walkExactUs_&&walkExactTerminal_==terminal&&sameLawShape(walkExactGait_,g);
  if(shape&&walkAheadValid_&&gridUs==walkAheadUs_){walkExact_=walkAhead_;walkExactUs_=walkAheadUs_;walkExactGait_=walkAheadGait_;walkAheadValid_=false;}  // the bracket moves on
  if(!(shape&&walkExactUs_<=gridUs&&gridUs<walkExactUs_+2*kPeriodUs)){
    // Start, missed release or structural change: exact solution at this grid
    // point, sent as it is; the look-ahead is solved by the next release so
    // that no release ever carries two nominal solves.
    solveGaitInto(walkExact_,g,gaitTrackCartesian(track_,g,cyc(gridUs),0,0,terminal),&walkPrevious_,false);
    walkExactUs_=gridUs;walkExactTerminal_=terminal;walkExactGait_=g;walkAheadValid_=false;nominalSolved_=true;++lookaheadStats_.resync;
    f=walkExact_;return;
  }
  bool solved=false;
  if(!walkAheadValid_){
    const double ca=cyc(walkExactUs_+2*kPeriodUs);
    if(terminal<0||ca<=terminal+gaitSettleCycles(g)){
      solveGaitInto(walkAhead_,g,gaitTrackCartesian(track_,g,ca,0,0,terminal),&walkExact_,false);solved=true;
      if(walkAhead_.target.valid){walkAheadUs_=walkExactUs_+2*kPeriodUs;walkAheadGait_=g;walkAheadValid_=true;}
    }
  }
  nominalSolved_=solved;if(solved)++lookaheadStats_.exact;else ++lookaheadStats_.interp;
  if(!walkAheadValid_){f=walkExact_;return;}  // no look-ahead inside the law (terminal zone): the grid-point solution
  const double u=std::clamp(double(nowUs_-walkExactUs_)/double(walkAheadUs_-walkExactUs_),0.,1.);
  lerpFrame(f,u,g,terminal);
#ifdef MATDOG_T2_SHADOW
  // Exact solve at the release clock on a track sampled every release. Compared
  // only while the law has not been retargeted since the bracket's exact
  // sample: after a retarget the two tracks plan a lift-off at different
  // instants with different goals, so "the exact reference" is not unique.
  {const auto exact=solveGait(g,gaitTrackCartesian(shadowTrack_,g,cycles_,0,0,terminal),&walkPrevious_,false);
   const bool sameLaw=walkExactGait_.advanceXM==g.advanceXM&&walkExactGait_.advanceYM==g.advanceYM&&walkExactGait_.yawRad==g.yawRad;
   if(sameLaw&&exact.target.valid&&f.target.valid)for(unsigned i=0;i<12;++i)lookaheadStats_.maxShadowRad=std::fmax(lookaheadStats_.maxShadowRad,std::abs(jointSlot(exact.target,i)-jointSlot(f.target,i)));}
#endif
}
void SimpleMotion::generateWalk(uint64_t now){
  timing::Section section(timing::NOMINAL);
  // One constant gait clock. Start and WALK STOP are geometric C2 stride
  // ramps inside the gait law, so the body leaves and reaches rest with zero
  // velocity and acceleration; the last window recentres with four feet down.
  double fade=0;const bool firstRecovery=walkRecover_&&!recoverUs_;
  if(walkRecover_){
    // First release retains the accepted body-in-support correction; bridge
    // from the stopped RAW to the compensated gait, never apply it twice.
    if(!recoverUs_){
      // Recheck the actual pose after I/O in the same certified WALK box.
      if(!pointSafe(measured())||!contains(walkBox(),measured())){fault("STOP_RECOVERY_PATH_NOT_CLEAR");return;}
      recoverUs_=now;}
    // Same C2 ease as the WALK start ramp: gait rate 0->1 over one second.
    const double t=(now-recoverUs_)/1e6,u=std::fmin(1.,t);
    cycles_=recoverCycles_+(t<1?u*u*u*u*(2.5+u*(-3+u)):t-.5)/kWalkPeriodS;
    fade=1-quinticTimeLaw(u,1).progress;
  }else cycles_=(now-walkStartUs_)/1e6/kWalkPeriodS;
  steerWalk(now);
  const auto g=gait_;
  const double terminal=walkStopping_?stopCycle_+g.stopCycles:-1,end=terminal+gaitSettleCycles(g);
  const bool clockDone=walkStopping_&&cycles_>=end;if(clockDone)cycles_=end;
  // A STOP in WALK_SETTLE can restart at the terminal clock itself. Finish
  // the existing held-RAW bridge before handing off, even with no gait left.
  const bool done=clockDone&&(!walkRecover_||now-recoverUs_>=1000000);
  LocomotionFrame f;nominalRelease(f,g,terminal,clockDone);
  if(!f.target.valid||!contains(walkBox(),f.target)||!pointSafe(f.target)){fault("WALK_IK_OR_WORKSPACE");return;}
  walkPrevious_=f;compensate(f,now);
  if(offPending_)return;
  if(firstRecovery)for(unsigned i=0;i<12;++i){double held=0;if(!port_->decodeRaw(i,commandedRaw_[i],held)){fault("STOP_RECOVERY_PATH_NOT_CLEAR");return;}
    jointSlot(recoverOffset_,i)=held-jointSlot(f.target,i);}
  if(fade>0){for(unsigned i=0;i<12;++i)jointSlot(f.target,i)+=fade*jointSlot(recoverOffset_,i);
    if(!contains(walkBox(),f.target)||!pointSafe(f.target)){fault("WALK_IK_OR_WORKSPACE");return;}}
  if(walkRecover_&&!compensationSafe(f.target,walkBox())){fault("WALK_IK_OR_WORKSPACE");return;}
  if(!send(f.target,kWalkDrive,now))return;
  if(done){
    walkRecover_=false;state_=State::MOVING;reason_="WALK_SETTLING";
    // The gait clock has finished, but native settling still needs live IMU
    // correction and a fixed completion deadline. Reuse the same generator.
    beginNative(Phase::WALK_HOLD,walkPrevious_.target,walkBox(),now,0,0,baseLinkHeightFromNominalM(kWalkHeightM));
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
  if(state_==State::STOPPED&&input_.ready&&input_.powerOk&&input_.imuValid&&offVerified()&&!operatorStopped_){state_=State::READY;reason_=proneRest_?"PRONE_REST":"READY";}
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
  // A joystick intent that expires while rising to walk cancels only the walk;
  // once walking (walkRequested_ stays set), steerWalk ends it with WALK STOP.
  if(joystick_&&walkRequested_&&phase_!=Phase::WALK&&now>joystickDeadlineUs_)walkRequested_=joystick_=false;
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
    }else if(readInterrupted_){--pollNext_;} // retry this slot after intentional STOP preemption
    else if(!readDeferred()&&++failures_[i]>=3&&energized_){fault("SERVO_COMMUNICATION_LOST");break;}
    if(stop_.load()||safeOffRequest_.load())break;
  }
  if(serviceStop())return;
  if(offPending_){serviceOff();return;}
  if(energized_&&phase_==Phase::ENABLE&&!fresh(esp_timer_get_time())){fault("SERVO_FEEDBACK_STALE");serviceOff();return;}
  if(phase_==Phase::ACQUIRE){
    Feedback f;const bool read=readTimed(acquire_,true,f);if(serviceStop())return;
    // An unanswered READ measures nothing: the same servo is read again at the
    // next release. RX_PENDING defers without a new lost reply; three actual
    // failed attempts still fault. The pending horizon is owned by ServoBus.
    if(!read){if(readDeferred())return;if(++controlFailures_[acquire_]>=3)fault("INITIAL_SERVO_READ_FAILED");return;}
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
    if(!read&&readDeferred())return; // no new unanswered reply during RX recovery
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
          beginHeight(actual,std::fmin(.150,std::fmax(kMinStandM,nominalHeightFromBaseLinkM(h))),collision::kHeight,now);break;}
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
    const bool read=readTimed(i,true,f);if(!read&&readInterrupted_)--controlNext_;
    if(serviceStop())return;
    if(read){
      fb_[i]=f;controlFailures_[i]=0;
      if(energized_&&(f.torque!=1||f.goal!=commandedRaw_[i]||f.goalSpeed!=commandedDrive_.speed||f.acc!=commandedDrive_.acc)){fault("SERVO_TORQUE_OR_GOAL_LOST");return;}
    }else if(energized_&&!readDeferred()&&++controlFailures_[i]>=3){fault("SERVO_CONTROL_READ_LOST");return;}
  }
  if(serviceStop())return;
  // READ publication can follow release entry by more than the future tolerance.
  // Use the gate clock; keep the trajectory epoch and all age limits unchanged.
  if(energized_&&!fresh(esp_timer_get_time())){fault("SERVO_FEEDBACK_STALE");return;}
  // The revoked operation must not advance a pending group, gait or posture.
  // STOP keeps the same ordinary feedback/electrical/authority guards above.
  if(stopActive_){generateStop(now);return;}
  if(pendingGoalValid_){
    if((phase_==Phase::HEIGHT||phase_==Phase::PRONE_HEIGHT)&&holdNominal_.target.valid&&
       (rollCorrection_!=0||pitchCorrection_!=0)){
      if(now>segmentDeadlineUs_){if(returningProne())proneNotCompleted(now,"PRONE_NOT_COMPLETED_SUPPORT_HELD");else fault("NATIVE_TARGET_NOT_REACHED");return;}
      timing::Section section(timing::COMPENSATE);
      auto f=holdNominal_;compensateLevel(f,{},now,true);send(f.target,kStandDrive,now);return;
    }
    pendingGoalValid_=false;
    if(phase_==Phase::PRONE_HEIGHT){beginProne(plannedHeight_>kProneHandoverM?Phase::PRONE_HEIGHT:Phase::PRONE_LIFT,now);return;}
    JointTargetFrame actual;actual.valid=true;
    for(unsigned i=0;i<12;++i)jointSlot(actual,i)=fb_[i].q;
    if(!heightStartClear(actual,pendingGoal_)){fault("HEIGHT_PATH_NOT_CLEAR");return;}
    // Start from the preceding NOMINAL stand: an already compensated encoder
    // pose fed back as nominal would apply the IMU correction twice.
    if(transitionSafe(actual,pendingGoal_,collision::kHeight)||entersHeight(actual,pendingGoal_)){beginHeight(segmentTo_,plannedHeight_,collision::kHeight,now);return;}
    plan_.clear();if(!via(segmentTo_,0,0,plannedHeight_)||!via(pendingGoal_,0,0,requestedHeight_)){fault("HEIGHT_PATH_NOT_CLEAR");return;}
    plannedHeight_=requestedHeight_;beginPlan(Phase::LIFT,collision::kLift,now);return;
  }
  if(returningProne()){
    const auto actual=measured();
    if(!segmentBox_||!pointSafe(actual)||!contains(*segmentBox_,actual)){
      // Loaded sag out of the segment box but inside a certified box: arrest
      // with torque (STOP re-checks its box every release). Otherwise SAFE_OFF.
      if(segmentBox_&&pointSafe(actual)&&(contains(collision::kHeight,actual)||contains(collision::kLift,actual))){proneNotCompleted(now,"PRONE_OUTSIDE_SEGMENT_SUPPORT_HELD");return;}
      fault("PRONE_MEASURED_OUTSIDE_BOX");return;}
  }
  if(phase_==Phase::WALK){generateWalk(now);return;}
  if(state_==State::MOVING){
    // At a settled boundary generate only the next segment in this release.
    const bool releasingLevel=phase_==Phase::WALK_HOLD&&std::abs(plannedHeight_-requestedHeight_)>1e-9&&(rollCorrection_!=0||pitchCorrection_!=0);
    if(!releasingLevel&&(arrived(now)||settledUnderLoad(now))){nextStand(now);return;}
    if(segmentBox_)generateStand(now);
    if(offPending_||stop_.load())return;
    if(now>(segmentBox_?segmentDeadlineUs_:goalDeadlineUs_)){if(returningProne())proneNotCompleted(now,"PRONE_NOT_COMPLETED_SUPPORT_HELD");else fault("NATIVE_TARGET_NOT_REACHED");return;}
  }
  if(phase_==Phase::HOLD){
    // Nominal height/contacts are constant in HOLD; only the IMU correction
    // changes. Do not solve the same nominal IK and differential every 20 ms.
    auto f=holdNominal_;
    if(f.target.valid){compensate(f,now);send(f.target,kStandDrive,now);}
  }
}
} }
