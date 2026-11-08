// Scripted software observations only. No servo/ground-contact dynamics model:
// passing these tests does not qualify physical braking or support.
#include "../../src/simple/SimpleMotion.h"
#include "../../src/simple/StopPlan.h"
#include "../../src/core/MotionService.h"
#include "../../src/core/MotionJson.h"
#include "../../src/motion/PairedProneData.h"
#include "../../src/motion/TiltCompensation.h"
#include <array>
#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <limits>
#include <vector>
static int64_t test_clock_us=1000000;
extern "C" int64_t esp_timer_get_time(){return test_clock_us;}
using namespace matdog;
namespace matdog { namespace simple {
struct StopReferenceProbe {
  struct Knot { double q[12]{},v[12]{},a[12]{}; };
  static Knot original(const SimpleMotion& m,uint64_t){
    Knot k;assert(m.issuedUs_>m.previousUs_&&m.previousUs_);
    const double dt=(m.issuedUs_-m.previousUs_)/1e6, vmax=m.commandedDrive_.speed*kTickRad;
    for(unsigned i=0;i<12;++i){double previous=0;
      assert(m.port_->decodeRaw(i,m.commandedRaw_[i],k.q[i]));
      assert(m.port_->decodeRaw(i,m.previousRaw_[i],previous));
      k.v[i]=std::clamp((k.q[i]-previous)/dt,-vmax,vmax);}
    return k;
  }
  static double correction(const SimpleMotion& m){
    auto nominal=m.phase_==SimpleMotion::Phase::WALK?m.walkPrevious_.target:m.holdNominal_.target;
    if(m.planned_&&m.phase_!=SimpleMotion::Phase::HOLD){double q[ViaPlan::kDim];
      m.plan_.sample((m.lastReleaseUs_-m.segmentStartUs_)/1e6,q);
      for(unsigned i=0;i<12;++i)motion::jointSlot(nominal,i)=q[i];}
    double d=0;for(unsigned i=0;i<12;++i)d=std::fmax(d,std::abs(motion::jointSlot(m.goal_,i)-motion::jointSlot(nominal,i)));
    return d;
  }
  static uint64_t deadline(const SimpleMotion& m){return m.stopDeadlineUs_;}
  static unsigned nextRead(const SimpleMotion& m){return m.pollNext_%12;}
  static void joined(const SimpleMotion& m,const Knot& k){
    assert(m.stopActive_);double q[12],v[12],a[12];m.stopPlan_.sample(0,q,v,a);
    for(unsigned i=0;i<12;++i){assert(std::abs(q[i]-k.q[i])<1e-12);assert(std::abs(v[i]-k.v[i])<1e-12);assert(std::abs(a[i]-k.a[i])<1e-12);}
    const auto& b=*m.stopBox_;
    for(unsigned n=0;n<=1000;++n){m.stopPlan_.sample(m.stopPlan_.duration*n/1000,q,v,a);
      for(unsigned i=0;i<12;++i){assert(q[i]>=b.lo[i]&&q[i]<=b.hi[i]);assert(std::abs(v[i])<=m.stopDrive_.speed*kTickRad+1e-10);assert(std::abs(a[i])<=m.stopDrive_.acc*100*kTickRad+1e-10);}}
  }

};
} }
namespace {
struct ObservationPort : simple::ServoPort {
  simple::SimpleMotion* motion=nullptr;
  uint64_t now=1000000;
  simple::Feedback observed[12]{};
  uint16_t target[12]{};
  simple::Drive drive{};
  bool missing=false,stale=false,freeze=false,failStopFrame=false,verify=true;
  bool stopInFrame=false,stopAfterFrame=false,frameInterrupted=false,failWithDuplicate=false,failWithSafeOff=false,safeOffInFrame=false,safeOffAfterFrame=false,faultAfterFrame=false;
  bool honorPreempt=false,stopInRead=false,safeOffInRead=false;
  unsigned preemptedReads=0;
  unsigned stopDuringReads=0,frameDelayUs=0;
  int offsetTicks=0;
  unsigned frames=0,stopFrames=0,offCalls=0,enables=0;
  std::vector<std::array<uint16_t,12>> sent;
  ObservationPort(){
    const auto* p=motion::paired_data::kKnots[0].p;
    for(unsigned l=0;l<4;++l){
      const double q[3]={((l==0||l==3)?-1:1)*p[7],p[l<2?2:0],
        std::fmax((l<2?-88.5:-89.)*3.14159265358979323846/180,p[l<2?3:1])};
      for(unsigned j=0;j<3;++j){auto& f=observed[l*3+j];f.q=q[j];
        f.raw=2048+std::lround(q[j]/simple::kTickRad);f.torque=0;
        f.temp=30;f.voltage=120;f.valid=true;f.goal=f.raw;target[l*3+j]=f.raw;}
    }
  }
  bool read(unsigned slot,bool control,simple::Feedback& out) override {
    if(stopInRead){stopInRead=false;motion->stop();}
    if(safeOffInRead){safeOffInRead=false;motion->safeOff();}
    // Use SimpleMotion's production preemption predicate, as ServoBus does.
    if(honorPreempt&&motion->busPreemptRequested()){++preemptedReads;return false;}
    if(stopDuringReads&&motion->stopActive()){--stopDuringReads;motion->stop();return false;}
    if(missing)return false;
    out=observed[slot];
    // The observations deliberately do not represent a simultaneous pose.
    out.stampUs=now-(stale?300000:slot*7000);
    if(control)out.controlStampUs=now;
    out.goal=target[slot];out.goalSpeed=drive.speed;out.acc=drive.acc;
    return true;
  }
  bool resolve(unsigned,double q,uint16_t& raw) override {
    raw=2048+std::lround(q/simple::kTickRad);return raw<4096;
  }
  bool decodeRaw(unsigned,uint16_t raw,double& q) override {q=(int(raw)-2048)*simple::kTickRad;return raw<4096;}
  bool frame(const uint16_t raw[12],simple::Drive d) override {
    frameInterrupted=false;test_clock_us+=frameDelayUs;
    const bool stopping=motion&&motion->stopping();
    assert(!stopping||motion->stopFrameAuthorized(raw,d));
    if(safeOffInFrame){safeOffInFrame=false;frameInterrupted=true;motion->safeOff();return false;}
    if(stopping&&stopInFrame){stopInFrame=false;frameInterrupted=true;motion->stop();return false;}
    if(stopping&&failStopFrame){if(failWithDuplicate)motion->stop();if(failWithSafeOff)motion->safeOff();return false;}
    ++frames;if(stopping)++stopFrames;drive=d;
    std::array<uint16_t,12> copy{};
    for(unsigned i=0;i<12;++i)copy[i]=target[i]=raw[i];
    sent.push_back(copy);if(safeOffAfterFrame){safeOffAfterFrame=false;motion->safeOff();if(faultAfterFrame)motion->fault("CONCURRENT_PHYSICAL_FAULT");}if(stopAfterFrame){stopAfterFrame=false;motion->stop();}return true;
  }
  bool framePreempted() const override { return frameInterrupted; }
  bool enable(unsigned i) override {observed[i].torque=1;++enables;return true;}
  bool off(unsigned first=0) override {
    ++offCalls;for(unsigned i=first;i<12;++i)observed[i].torque=0;return true;
  }
  bool verifyOff(unsigned i) override {return verify&&(i==12||observed[i].torque==0);}
  void publish(){
    if(freeze)return;
    for(unsigned i=0;i<12;++i){observed[i].raw=target[i]+(i%3?offsetTicks:0);
      observed[i].q=(int(observed[i].raw)-2048)*simple::kTickRad;}
  }
};
struct Rig {
  simple::SimpleMotion m;ObservationPort p;simple::Inputs in;
  Rig(){m.begin(&p);p.motion=&m;in.ready=in.powerOk=in.authorityOk=in.imuValid=true;in.refusal="READY";
    const double lo[12]={-.783864,-.914253,-1.598408,-.783864,-.914253,-1.58,-.783864,-.914253,-1.592272,-.783864,-.914253,-1.590738};
    const double hi[12]={.783864,2.110758,.590583,.783864,2.10002,.581379,.771592,2.136835,.590583,.78233,2.129166,.585981};
    for(unsigned i=0;i<12;++i){in.minRad[i]=lo[i];in.maxRad[i]=hi[i];}
    for(unsigned i=0;i<15;++i)tick(false);assert(m.offVerified());
  }
  void tick(bool publish=true){p.now+=simple::kPeriodUs;if(publish)p.publish();in.imuStampUs=p.now;
    m.inputs(in);test_clock_us=p.now;m.update(p.now);m.updateTelemetry(p.now);}
  void ticks(unsigned n){for(unsigned k=0;k<n;++k)tick();}
  void stand(double height){assert(!m.stand(height,p.now));
    for(unsigned k=0;k<3000&&m.state()!=simple::State::STANDING;++k){tick();assert(m.state()!=simple::State::FAULT);}
    assert(m.state()==simple::State::STANDING);}
  void walk(unsigned stride=1){assert(!m.walk(stride,p.now));
    for(unsigned k=0;k<3000&&m.state()!=simple::State::WALKING;++k){tick();assert(m.state()!=simple::State::FAULT);}
    assert(m.state()==simple::State::WALKING);}
  void phase(const char* expected){for(unsigned k=0;k<3000&&strcmp(m.phase(),expected);++k){tick();assert(m.state()!=simple::State::FAULT);}assert(!strcmp(m.phase(),expected));}
  bool finishStop(unsigned maximum=250){for(unsigned k=0;k<maximum&&!m.stopVerified()&&m.state()!=simple::State::FAULT;++k)tick();return m.stopVerified();}
  void finishOff(){for(unsigned k=0;k<60&&!m.offVerified();++k)tick(false);assert(m.offVerified());}
};
void checkHeldStop(Rig& r){
  const auto off=r.p.offCalls,enables=r.p.enables;r.m.stop();
  assert(r.m.stopping());r.tick();assert(r.m.stopActive()&&!r.m.stopVerified());
  assert(r.p.offCalls==off);assert(r.finishStop());
  assert(r.m.state()==simple::State::STOPPED&&r.m.energized());
  assert(!r.m.offVerified()&&!r.m.stopSupportVerified());assert(r.p.offCalls==off&&r.p.enables==enables);
  const auto frames=r.p.frames;r.ticks(30);assert(r.p.frames==frames);
  assert(r.m.prone(r.p.now));
  // dev.17: WALK is an explicit resume, admitted exactly as STAND after STOP
  // (test_simple_motion stopThenWalk); refused, it gives the same reason.
  if(const char* why=r.m.standResumeRefusal(r.p.now)){const char* w=r.m.walk(1,r.p.now);assert(w&&!strcmp(w,why));}
  r.m.walkStop();r.ticks(30);assert(r.p.frames==frames&&r.p.enables==enables);
  assert(r.m.stopVerified()&&!r.m.stopSupportVerified());
}
void stopPlanBounds(){
  // Dense sampling supplements the production analytic extrema/convex-box
  // checks; it does not stand in for an actuator response model.
  constexpr double vmax=.7363107781851077,amax=1.994;
  double q[12]{},v[12]{},a[12]{},lo[12],hi[12];
  for(unsigned i=0;i<12;++i){lo[i]=-2;hi[i]=2;}
  unsigned accepted=0;
  for(double speed:{-.55,-.20,0.,.20,.55})for(double accel:{-1.10,-.20,0.,.20,1.10}){
    for(unsigned i=0;i<12;++i){q[i]=.04*i;v[i]=i%2?speed:-speed;a[i]=i%2?accel:-accel;}
    simple::StopPlan plan;assert(plan.build(q,v,a,lo,hi,vmax,amax));++accepted;
    assert(plan.duration>=.04&&plan.duration<=simple::StopPlan::kMaxDurationS);
    double x[12],dx[12],ddx[12];plan.sample(0,x,dx,ddx);
    for(unsigned i=0;i<12;++i){assert(std::abs(x[i]-q[i])<1e-12);assert(std::abs(dx[i]-v[i])<1e-12);assert(std::abs(ddx[i]-a[i])<1e-12);}
    for(unsigned k=0;k<=2000;++k){plan.sample(plan.duration*k/2000,x,dx,ddx);
      for(unsigned i=0;i<12;++i){assert(x[i]>=lo[i]-1e-12&&x[i]<=hi[i]+1e-12);
        assert(std::abs(x[i]-q[i])<=simple::StopPlan::kMaxResidualRad+1e-12);
        assert(std::abs(dx[i])<=vmax+1e-10&&std::abs(ddx[i])<=amax+1e-10);}}
    plan.sample(plan.duration,x,dx,ddx);
    for(unsigned i=0;i<12;++i){assert(std::abs(x[i]-plan.end[i])<1e-12);assert(std::abs(dx[i])<1e-12&&std::abs(ddx[i])<1e-12);}
  }
  for(unsigned i=0;i<12;++i){q[i]=0;v[i]=0;a[i]=0;}
  simple::StopPlan p;assert(p.build(q,v,a,lo,hi,vmax,amax));
  v[0]=std::numeric_limits<double>::quiet_NaN();assert(!p.build(q,v,a,lo,hi,vmax,amax));v[0]=0;
  a[1]=std::numeric_limits<double>::infinity();assert(!p.build(q,v,a,lo,hi,vmax,amax));a[1]=0;
  q[2]=std::numeric_limits<double>::quiet_NaN();assert(!p.build(q,v,a,lo,hi,vmax,amax));q[2]=0;
  assert(!p.build(q,v,a,lo,hi,0,amax));assert(!p.build(q,v,a,lo,hi,vmax,-1));
  q[0]=hi[0]-.001;v[0]=.55;assert(!p.build(q,v,a,lo,hi,vmax,amax));
  q[0]=0;v[0]=vmax+.001;assert(!p.build(q,v,a,lo,hi,vmax,amax));
  v[0]=0;a[0]=amax+.001;assert(!p.build(q,v,a,lo,hi,vmax,amax));
  std::printf("StopPlan: %u signed velocity/acceleration cases, 2001 samples each; C2 endpoints, V/A/box/residual caps and invalid-input refusal\n",accepted);
}
void holdAndProne(){
  for(double h:{.05,.10,.15}){Rig r;r.stand(h);r.ticks(100);checkHeldStop(r);}
  // PRONE ends at rest (dev.15): STOP there is the ordinary OFF latch, no held
  // reference and no frame; STAND then needs the usual SAFE_OFF recovery.
  for(unsigned wait:{0u,100u}){Rig prone;prone.stand(.05);assert(!prone.m.prone(prone.p.now));
    for(unsigned n=0;n<8000&&!prone.m.proneRest();++n)prone.tick();assert(prone.m.proneRest());prone.ticks(wait);
    const auto frames=prone.p.frames;prone.m.stop();prone.ticks(40);
    assert(!prone.m.stopActive()&&!prone.m.proneRest()&&prone.m.offVerified()&&!prone.m.energized()&&prone.p.frames==frames);
    assert(!strcmp(prone.m.stand(.10,prone.p.now),"SAFE_OFF_STAND_REFUSED"));}
  // Loaded low HOLD outside kHeight, already admitted by dev.7 inside kLift.
  Rig loaded;loaded.stand(.05);loaded.ticks(100);
  loaded.p.observed[2].raw+=7;loaded.p.observed[2].q+=7*simple::kTickRad;
  loaded.p.freeze=true;loaded.p.observed[2].load=1024+176;
  const auto nominal=loaded.m.target();checkHeldStop(loaded);
  for(unsigned i=0;i<12;++i)assert(std::abs(motion::jointSlot(loaded.m.target(),i)-motion::jointSlot(nominal,i))<=simple::kTickRad/2);
  std::puts("STOP HOLD50/100/150: torque maintained, endpoint observed, support unverified, WALK/PRONE refused; STOP at the PRONE rest: OFF latch, no frame; explicit STAND tested separately");
}
void movementAndDuplicates(){
  for(unsigned scenario=0;scenario<3;++scenario){Rig one,repeated;
    auto prepare=[&](Rig& r){
      if(scenario==0){r.stand(.10);r.ticks(100);assert(!r.m.stand(.15,r.p.now));r.ticks(30);}
      else if(scenario==1){r.stand(.10);assert(!r.m.prone(r.p.now));r.ticks(30);}
      else{r.walk();r.ticks(380);}
    };
    prepare(one);prepare(repeated);
    const auto off=one.p.offCalls,frames=one.p.frames;one.m.stop();repeated.m.stop();
    bool done=false;
    for(unsigned k=0;k<250;++k){one.tick();repeated.m.stop();repeated.tick();
      assert(one.m.state()==repeated.m.state());assert(one.m.stopVerified()==repeated.m.stopVerified());
      assert(one.p.target[0]==repeated.p.target[0]);
      for(unsigned i=0;i<12;++i)assert(std::abs(motion::jointSlot(one.m.target(),i)-motion::jointSlot(repeated.m.target(),i))<1e-12);
      if(one.m.stopVerified()){done=true;break;}assert(one.m.state()!=simple::State::FAULT);
    }
    assert(done&&one.p.offCalls==off&&one.p.frames>frames&&one.p.stopFrames>0);
    assert(!one.m.stopSupportVerified());
  }
  std::puts("STOP posture/PRONE descent/WALK: bounded authorized reference replaces operation; repeated STOP has identical targets/completion");
}
void evidenceAndFaults(){
  // A stationary reference and acknowledged goal do not establish arrival.
  {Rig r;r.stand(.15);r.ticks(100);for(auto& f:r.p.observed)f.speed=100;
    r.m.stop();r.ticks(70);assert(r.m.stopInProgress()&&!r.m.stopVerified());
    for(auto& f:r.p.observed)f.speed=0;r.tick();assert(!r.m.stopVerified());
    assert(r.finishStop());}
  // Every physical safety input retains the ordinary SAFE_OFF route.
  for(unsigned what=0;what<5;++what){Rig r;r.walk();r.ticks(380);r.m.stop();r.tick();
    const auto frames=r.p.frames;
    if(what==0){r.in.powerOk=false;r.in.powerReason="TEST_LOW_POWER";}
    else if(what==1)r.in.authorityOk=false;
    else if(what==2)r.p.missing=true;
    else if(what==3)r.p.stale=true;
    else for(auto& f:r.p.observed)f.status=8;
    for(unsigned k=0;k<30&&r.m.state()!=simple::State::FAULT;++k)r.tick();
    assert(r.m.state()==simple::State::FAULT&&!r.m.stopVerified());r.finishOff();
    assert(!r.m.energized()&&r.p.frames>=frames);
    const auto enable=r.p.enables;r.m.stop();r.ticks(30);
    assert(r.m.state()==simple::State::FAULT&&r.p.enables==enable&&r.m.stand(.15,r.p.now));
  }
  {Rig r;r.stand(.15);r.ticks(100);r.m.stop();r.tick();r.p.now+=200000;
    test_clock_us=r.p.now;r.in.imuStampUs=r.p.now;r.m.inputs(r.in);r.m.update(r.p.now);
    assert(r.m.state()==simple::State::FAULT&&!strcmp(r.m.reason(),"MOTION_DEADLINE_LOST"));r.finishOff();}
  {Rig r;r.walk();r.ticks(380);r.p.failStopFrame=true;r.m.stop();
    for(unsigned k=0;k<20&&r.m.state()!=simple::State::FAULT;++k)r.tick();
    assert(r.m.state()==simple::State::FAULT);r.finishOff();}
  {Rig r;r.walk();r.ticks(380);r.p.failStopFrame=r.p.failWithDuplicate=true;r.m.stop();
    for(unsigned k=0;k<20&&r.m.state()!=simple::State::FAULT;++k)r.tick();
    assert(r.m.state()==simple::State::FAULT&&!strcmp(r.m.reason(),"SYNC_WRITE_FAILED"));r.finishOff();}
  std::puts("STOP verification requires post-terminal speed/convergence/dwell; power/authority/feedback/alarm/deadline/write failures retain latched FAULT + 13/13 SAFE_OFF");
}
void supportPlaneLevelTarget(){
  // Kinematic support/no-slip hypotheses only, not contact or hardware stability evidence.
  constexpr double deg=3.14159265358979323846/180;
  const auto& stand=motion::canonicalStandDefinition();
  unsigned cases=0;
  const std::array<std::array<double,2>,11> angles={{{0,0},{0,15},{0,-15},{.25,0},{-.25,0},{1.0,0},{-1.0,0},{1.8,0},{-1.8,0},{.5,.5},{-.5,-.5}}};
  for(const auto& angle:angles){
    const auto board=motion::bodyPoseFromRollPitch({0,0,0},angle[0]*deg,angle[1]*deg);
    const motion::Vector3 upSupport={board.rotation[2][0],board.rotation[2][1],board.rotation[2][2]};
    const double expectedRoll=-std::asin(upSupport.y),expectedPitch=std::atan2(upSupport.x,upSupport.z);
    // Both before response (body parallel to plane) and after exact leveling:
    // the measured encoder/IMU geometry returns the same target, no accumulated error.
    for(bool leveled:{false,true}){
      const double currentRoll=leveled?expectedRoll:0,currentPitch=leveled?expectedPitch:0;
      const auto solved=motion::compensateStand(stand,simple::baseLinkHeightFromNominalM(.100),currentRoll,currentPitch,stand.standSeed,false);
      assert(solved.status==motion::TiltCompensation::Status::OK);
      for(unsigned i=0;i<12;++i)assert(motion::jointSlot(solved.target,i)>=simple::collision::kHeight.lo[i]&&motion::jointSlot(solved.target,i)<=simple::collision::kHeight.hi[i]);
      // R_WB = R_WS * R_SB; its third row is gravity-up expressed in body.
      const auto& current=solved.pose;
      const motion::Vector3 upBody={upSupport.x*current.rotation[0][0]+upSupport.y*current.rotation[1][0]+upSupport.z*current.rotation[2][0],
        upSupport.x*current.rotation[0][1]+upSupport.y*current.rotation[1][1]+upSupport.z*current.rotation[2][1],
        upSupport.x*current.rotation[0][2]+upSupport.y*current.rotation[1][2]+upSupport.z*current.rotation[2][2]};
      for(unsigned omitted=0;omitted<=4;++omitted){bool stance[4];for(unsigned i=0;i<4;++i)stance[i]=i!=omitted;
        double roll=9,pitch=9;assert(motion::supportLevelTarget(stand.contactsWorldM,solved.target,stance,upBody,current,roll,pitch));
        assert(std::abs(roll-expectedRoll)<1e-7&&std::abs(pitch-expectedPitch)<1e-7);++cases;
      }
    }
  }
  const auto solved=motion::compensateStand(stand,simple::baseLinkHeightFromNominalM(.100),0,0,stand.standSeed,false);
  bool stance[4]={true,true,true,true};double roll=9,pitch=9;motion::Vector3 up={0,0,1};
  auto invalid=solved.target;invalid.valid=false;
  assert(!motion::supportLevelTarget(stand.contactsWorldM,invalid,stance,up,solved.pose,roll,pitch)&&roll==0&&pitch==0);
  invalid=solved.target;invalid.legs[0].hip=std::numeric_limits<double>::quiet_NaN();
  assert(!motion::supportLevelTarget(stand.contactsWorldM,invalid,stance,up,solved.pose,roll,pitch));
  assert(!motion::supportLevelTarget(stand.contactsWorldM,solved.target,stance,{0,0,0},solved.pose,roll,pitch));
  assert(!motion::supportLevelTarget(stand.contactsWorldM,solved.target,stance,{0,0,-1},solved.pose,roll,pitch));
  auto badPose=solved.pose;badPose.rotation[0][0]=2;
  assert(!motion::supportLevelTarget(stand.contactsWorldM,solved.target,stance,up,badPose,roll,pitch));
  stance[2]=stance[3]=false;assert(!motion::supportLevelTarget(stand.contactsWorldM,solved.target,stance,up,solved.pose,roll,pitch));
  stance[2]=true;motion::Vector3 collinear[4]={{0,0,0},{.1,0,0},{.2,0,0},{.3,0,0}};
  assert(!motion::supportLevelTarget(collinear,solved.target,stance,up,solved.pose,roll,pitch));
  std::printf("Support level target: %u production-IK roundtrips (pitch +/-15deg100mm, roll and combined signs, every three-stance set), retained level at zero IMU error; invalid geometry refused\n",cases);
}
void terminalStopHold(){
  {Rig r;r.stand(.15);r.ticks(100);core::MotionService service;service.begin(&r.m,nullptr,nullptr);service.setReady(true,"READY");
    motion::MotionCommand c;c.owner=motion::ClientOwner::DASHBOARD;c.intent=motion::Intent::STOP;
    c.sequence=1;c.issued_at_ms=r.p.now/1000;c.ttl_ms=1000;strcpy(c.id,"terminal-stop");
    assert(service.command(c,r.p.now/1000).outcome==motion::CommandOutcome::ACCEPTED);
    assert(r.finishStop());service.update();r.ticks(2000); // 40 s, well past acquisition deadline
    const auto held=r.p.target[11];const auto enables=r.p.enables,off=r.p.offCalls;
    r.p.freeze=true;r.p.observed[11].raw+=4;r.p.observed[11].q+=4*simple::kTickRad;r.ticks(100);service.update();
    assert(r.m.stopVerified()&&r.m.energized()&&!r.m.stopSupportVerified()&&r.m.state()==simple::State::STOPPED&&r.p.target[11]==held);
    core::MotionStatusSnapshot status;assert(service.captureStatus(status,r.p.now/1000));
    assert(!strcmp(status.activeOutcome,"COMPLETED")&&!strcmp(status.activeReason,"STOPPED_SUPPORT_UNVERIFIED"));
    r.m.stop();r.tick();assert(r.m.stopVerified()); // duplicate cannot revoke terminal result
    c.intent=motion::Intent::STAND;c.sequence=2;c.issued_at_ms=r.p.now/1000;c.target_height_m=.15;strcpy(c.id,"stand-after-terminal-stop");
    assert(service.command(c,r.p.now/1000).outcome==motion::CommandOutcome::ACCEPTED);
    r.p.freeze=false;for(unsigned n=0;n<3000&&r.m.state()!=simple::State::STANDING;++n){r.tick();service.update();assert(r.m.state()!=simple::State::FAULT);}
    service.update();assert(service.captureStatus(status,r.p.now/1000));
    assert(r.m.state()==simple::State::STANDING&&!strcmp(status.activeReason,"STAND_REACHED")&&!strcmp(status.activeOutcome,"COMPLETED")&&!strcmp(status.activeId,"stand-after-terminal-stop")&&status.activeSequence==2);
    assert(r.p.enables==enables&&r.p.offCalls==off);r.m.safeOff();r.tick();r.finishOff();assert(r.m.offVerified());
  }
  for(unsigned fault=0;fault<7;++fault){Rig r;r.stand(.15);r.ticks(100);r.m.stop();assert(r.finishStop());r.ticks(200);
    r.p.freeze=true;
    if(fault==0){r.p.observed[1].raw+=49;r.p.observed[1].q+=49*simple::kTickRad;}
    if(fault==1)r.p.stale=true;
    if(fault==2)for(auto& f:r.p.observed)f.torque=0;
    if(fault==3)r.in.authorityOk=false;
    if(fault==4)r.in.powerOk=false;
    if(fault==5)for(auto& f:r.p.observed)f.status=8;
    if(fault==6)for(auto& f:r.p.observed)f.load=401;
    for(unsigned n=0;n<150&&r.m.state()!=simple::State::FAULT;++n)r.tick();
    assert(r.m.state()==simple::State::FAULT&&strcmp(r.m.reason(),"STOP_NOT_SETTLED"));
    if(fault==0||fault==6)assert(!strcmp(r.m.reason(),"STOP_HOLD_TRACKING_UNSAFE"));
    r.finishOff();assert(r.m.offVerified());
  }
  {Rig r;r.stand(.15);r.ticks(100);for(auto& f:r.p.observed)f.speed=100;r.m.stop();
    for(unsigned n=0;n<250&&r.m.state()!=simple::State::FAULT;++n)r.tick();
    assert(r.m.state()==simple::State::FAULT&&!strcmp(r.m.reason(),"STOP_NOT_SETTLED"));r.finishOff();}
  puts("Terminal STOP: 4-tick disturbance after 40s keeps reference/result/torque; new STAND ACK completes; acquisition timeout and physical faults/SAFE_OFF retained");
}
void fallbackAndWalkStop(){
  {Rig r;assert(!r.m.stand(.15,r.p.now));r.phase("DISENGAGE");r.m.stop();r.tick();
    assert(r.m.state()==simple::State::FAULT&&!strcmp(r.m.reason(),"STOP_REFERENCE_UNAVAILABLE"));r.finishOff();}
  {Rig r;assert(!r.m.stand(.15,r.p.now));r.m.stop();r.tick();r.finishOff();assert(!r.m.energized());}
  {Rig r;r.walk();r.ticks(500);const auto off=r.p.offCalls;r.m.walkStop();
    for(unsigned k=0;k<1500&&r.m.state()!=simple::State::STANDING;++k)r.tick();
    assert(r.m.state()==simple::State::STANDING&&r.m.energized()&&!r.m.stopActive()&&r.p.offCalls==off);}
  std::puts("STOP unsupported native acquisition reference explicitly falls back SAFE_OFF; WALK STOP remains ordinary gait completion to STAND torque ON");
}
void phaseCoverageAndRaces(){
  // Every leg's swing and each intervening four-stance window, at the longest
  // stride: mid-window w/2 and mid-swing (w+.25)/2 of each quarter (w = duty-.75),
  // on the walk clock (walkClockS: gait time = real time - 0.5 s after the ease).
  const double w=simple::kWalkDuty-.75;
  for(unsigned quarter=0;quarter<4;++quarter)for(bool swing:{false,true}){
    Rig r;r.walk(2);const double cycle=1+quarter*.25+(swing?(w+.25)/2:w/2);
    r.ticks(static_cast<unsigned>(std::ceil((cycle*simple::kWalkPeriodS+.5)/.02)));
    motion::GaitParameters gait;gait.duty=simple::kWalkDuty;unsigned stance=0;
    for(unsigned i=0;i<4;++i)stance+=motion::legPhase(gait,static_cast<motion::LegId>(i),r.m.walkCycles()).stance;
    assert(stance==(swing?3u:4u));checkHeldStop(r);
    const auto frames=r.p.frames,enables=r.p.enables,off=r.p.offCalls;
    assert(!r.m.standResumeRefusal(r.p.now));
    core::MotionService service;service.begin(&r.m,nullptr,nullptr);service.setReady(true,"READY");
    core::MotionStatusSnapshot status;assert(service.captureStatus(status,r.p.now/1000));
    assert(status.standResumeAllowed&&!status.standResumeReason&&!status.stopSupportVerified);
    char json[core::kMotionStatusJsonBytes];assert(service.statusJson(json,sizeof(json),r.p.now/1000));
    assert(strstr(json,"\"stand_resume_allowed\":true")&&strstr(json,"STAND_RECOVERY_AVAILABLE"));
    motion::MotionCommand stand;stand.owner=motion::ClientOwner::DASHBOARD;stand.intent=motion::Intent::STAND;
    stand.sequence=1;stand.issued_at_ms=r.p.now/1000;stand.ttl_ms=1000;stand.target_height_m=.15;strcpy(stand.id,"walk-stop-recovery");
    const auto ack=service.command(stand,r.p.now/1000);
    assert(ack.outcome==motion::CommandOutcome::ACCEPTED);
    assert(r.m.energized()&&!r.m.stopSupportVerified());
    assert(r.p.frames==frames&&r.p.enables==enables&&r.p.offCalls==off);
    for(unsigned n=0;n<3000&&r.m.state()!=simple::State::STANDING;++n){
      r.tick();service.update();assert(r.m.state()!=simple::State::FAULT&&r.m.state()!=simple::State::WALKING);}
    service.update();assert(service.captureStatus(status,r.p.now/1000));
    assert(r.m.state()==simple::State::STANDING&&!strcmp(r.m.reason(),"STAND_REACHED"));
    assert(!strcmp(status.activeOutcome,"COMPLETED")&&!strcmp(status.activeReason,"STAND_REACHED"));
    assert(r.m.energized()&&r.p.enables==enables&&r.p.offCalls==off);
  }
  // STOP before the first release of a newly accepted posture must cancel it.
  for(bool prone:{false,true}){Rig r;r.stand(.10);r.ticks(100);
    assert(!(prone?r.m.prone(r.p.now):r.m.stand(.15,r.p.now)));checkHeldStop(r);}
  // An admitted duplicate inside a TX/read must not become a bus fault.
  {Rig r;r.walk();r.ticks(380);r.m.stop();r.tick();r.p.stopInFrame=true;
    r.tick();assert(r.m.stopActive()&&r.m.state()!=simple::State::FAULT);assert(r.finishStop());}
  {Rig r;r.walk();r.ticks(380);r.m.stop();r.tick();
    for(unsigned i=0;i<4;++i){r.p.stopDuringReads=1;r.ticks(10);
      if(!r.m.stopActive())std::fprintf(stderr,"read duplicate %u failed: %s\n",i,r.m.reason());assert(r.m.stopActive());}
    assert(r.finishStop());}
  // A STOP arriving after an operational UART burst uses that completed goal.
  {Rig r;r.walk();r.ticks(380);r.p.stopAfterFrame=true;r.tick();
    assert(r.m.stopActive());for(unsigned i=0;i<12;++i)assert(r.m.commandedRaw()[i]==r.p.target[i]);
    assert(r.finishStop());}
  {Rig r;r.stand(.15);r.ticks(100);const auto frames=r.p.frames;
    r.in.powerOk=false;r.m.inputs(r.in);r.m.stop();r.tick();
    assert(r.m.state()==simple::State::FAULT&&r.p.frames==frames);r.finishOff();}
  std::puts("STOP stance/swing per leg, queued posture cancellation and TX/read duplicate races: PASS (support never asserted)");
}
void serviceCompletion(){
  for(bool engaged:{false,true}){Rig r;if(engaged){r.stand(.15);r.ticks(100);}
    core::MotionService service;service.begin(&r.m,nullptr,nullptr);service.setReady(true,"READY");
    motion::MotionCommand stop;stop.owner=motion::ClientOwner::DASHBOARD;stop.intent=motion::Intent::STOP;
    stop.sequence=1;std::strcpy(stop.id,"stop-original");test_clock_us=r.p.now;
    assert(service.command(stop,r.p.now/1000).outcome==motion::CommandOutcome::ACCEPTED);
    service.update();core::MotionStatusSnapshot status;assert(service.captureStatus(status,r.p.now/1000));
    assert(!strcmp(status.activeOutcome,"ACCEPTED")&&status.finishedMs==0&&!status.stopVerified);
    r.tick();service.update();assert(service.captureStatus(status,r.p.now/1000));assert(!strcmp(status.activeOutcome,"ACCEPTED"));
    const auto accepted=status.acceptedMs;const auto epoch=service.stopEpoch();
    std::strcpy(stop.id,"stop-repeat");stop.sequence=2;
    assert(service.command(stop,r.p.now/1000).outcome==motion::CommandOutcome::ACCEPTED&&service.stopEpoch()==epoch+1);
    for(unsigned k=0;k<250;++k){r.tick();service.update();assert(service.captureStatus(status,r.p.now/1000));
      if(!strcmp(status.activeOutcome,"COMPLETED"))break;
      assert(status.finishedMs==0&&!strcmp(status.activeOutcome,"ACCEPTED"));}
    assert(!strcmp(status.activeId,"stop-original")&&status.acceptedMs==accepted&&status.finishedMs>accepted);
    assert(!strcmp(status.activeOutcome,"COMPLETED"));
    assert(!strcmp(status.activeReason,engaged?"STOPPED_SUPPORT_UNVERIFIED":"SAFE_OFF_VERIFIED"));
    assert(status.stopVerified==engaged&&!status.stopSupportVerified);
    char json[core::kMotionStatusJsonBytes];assert(service.statusJson(json,sizeof(json),r.p.now/1000));
    assert(strstr(json,engaged?"\"stand_resume_allowed\":true":"\"stand_resume_allowed\":false"));
    assert(strstr(json,engaged?"\"stand_resume_reason\":\"STAND_RECOVERY_AVAILABLE\"":"\"stand_resume_reason\":\"STOP_NOT_ACTIVE\""));
    if(engaged){motion::MotionCommand move;move.owner=motion::ClientOwner::DASHBOARD;move.intent=motion::Intent::STAND;
      move.sequence=3;move.issued_at_ms=r.p.now/1000;move.ttl_ms=1000;move.target_height_m=.15;std::strcpy(move.id,"late-motion");
      assert(service.command(move,r.p.now/1000).outcome==motion::CommandOutcome::ACCEPTED);}
  }
  std::puts("STOP service: ACK is not completion; first identity retained, common epoch, feedback/SAFE_OFF terminal distinguished");
}
void safeOffAndRecovery(){
  for(unsigned scenario=0;scenario<7;++scenario){Rig r;r.walk();r.ticks(380);
    core::MotionService service;service.begin(&r.m,nullptr,nullptr);service.setReady(true,"READY");
    if(scenario==1||scenario==2||scenario==3){r.m.stop();r.tick();if(scenario==2)assert(r.finishStop());}
    const auto enables=r.p.enables;
    if(scenario==3){r.p.safeOffInFrame=true;r.tick();}
    else if(scenario==4||scenario==6){r.p.safeOffAfterFrame=true;r.p.faultAfterFrame=scenario==6;r.tick();}
    else {if(scenario==5)r.m.fault("CONCURRENT_PHYSICAL_FAULT");const auto epoch=service.stopEpoch();service.safeOff();assert(service.stopEpoch()==epoch+1);r.tick();}
    const auto frames=r.p.frames;
    // A dashboard STOP after explicit SAFE_OFF cannot restore a torque-ON terminal.
    motion::MotionCommand c;c.intent=motion::Intent::STOP;c.owner=motion::ClientOwner::DASHBOARD;strcpy(c.id,"http-after-safe-off");c.sequence=1;
    assert(service.command(c,r.p.now/1000).outcome==motion::CommandOutcome::ACCEPTED);
    r.finishOff();r.ticks(20);assert(r.m.offMask()==8191&&!r.m.energized()&&!r.m.stopActive());
    assert(r.p.enables==enables&&r.p.frames==frames&&r.m.stand(.15,r.p.now));
    if(scenario==5||scenario==6)assert(r.m.state()==simple::State::FAULT&&!strcmp(r.m.reason(),"CONCURRENT_PHYSICAL_FAULT"));
  }
  {Rig r;r.walk();r.ticks(380);r.p.failStopFrame=r.p.failWithSafeOff=true;r.m.stop();
    for(unsigned n=0;n<20&&r.m.state()!=simple::State::FAULT;++n)r.tick();
    assert(r.m.state()==simple::State::FAULT&&!strcmp(r.m.reason(),"SYNC_WRITE_FAILED"));r.finishOff();}
  {Rig r;r.stand(.10);r.ticks(100);
    r.m.stop();r.tick();assert(!strcmp(r.m.stand(.15,r.p.now),"STOP_IN_PROGRESS"));assert(r.finishStop());
    const auto enables=r.p.enables,off=r.p.offCalls,frames=r.p.frames;r.ticks(30);assert(r.p.frames==frames);
    assert(!r.m.standResumeRefusal(r.p.now));assert(!r.m.stand(.15,r.p.now));
    for(unsigned n=0;n<3000&&r.m.state()!=simple::State::STANDING;++n){r.tick();assert(r.m.state()!=simple::State::FAULT);}
    assert(r.m.state()==simple::State::STANDING&&r.p.enables==enables&&r.p.offCalls==off);
  }
  {Rig r;r.walk();r.ticks(380);r.m.stop();assert(r.finishStop());assert(!r.m.stand(.15,r.p.now));}
  {Rig r;r.stand(.10);r.ticks(50);assert(!r.m.stand(.15,r.p.now));r.ticks(20);r.m.stop();assert(r.finishStop());
    const auto enables=r.p.enables,off=r.p.offCalls;std::array<uint16_t,12> held;std::copy(r.m.commandedRaw(),r.m.commandedRaw()+12,held.begin());
    assert(!r.m.standResumeRefusal(r.p.now)&&!r.m.stand(.15,r.p.now));r.tick();
    assert(std::equal(held.begin(),held.end(),r.m.commandedRaw()));
    for(unsigned n=0;n<2000&&r.m.state()!=simple::State::STANDING;++n){r.tick();assert(r.m.state()!=simple::State::FAULT);}
    assert(r.m.state()==simple::State::STANDING&&r.p.enables==enables&&r.p.offCalls==off);}
  {Rig r;r.stand(.15);r.ticks(50);r.m.stop();assert(r.finishStop());const auto n=r.p.enables;
    r.in.authorityOk=false;r.m.inputs(r.in);assert(!strcmp(r.m.stand(.15,r.p.now),"ACTUATOR_OWNER_BUSY"));
    r.in.authorityOk=true;r.in.powerOk=false;r.m.inputs(r.in);assert(!strcmp(r.m.stand(.15,r.p.now),"POWER_NOT_READY"));assert(r.p.enables==n);}
  motion::MotionCommand parsed;const char* why=nullptr;
  const char bad[]={"{\"version\":1,\"id\":\"off\",\"sequence\":1,\"issued_at_ms\":0,\"ttl_ms\":1000,\"owner\":\"dashboard\",\"type\":\"SERVO.SAFE_OFF\",\"params\":{}}"};
  assert(!core::parseMotionJson(bad,strlen(bad),&parsed,&why));
  puts("SAFE_OFF global 13/13 overrides Motion/STOP/TX and concurrent fault; HTTP STOP never undoes OFF. Explicit STAND uses existing torque only from admitted held poses");
}
void writerHandoffWithImu(){
  // Constant and varying ordinary IMU correction in HOLD, HEIGHT50->100,
  // and all four gait swing / four-stance windows. No contact claim.
  for(unsigned mode=0;mode<3;++mode)for(unsigned window=0;window<(mode==2?8u:2u);++window){
    Rig r;if(mode==2)r.walk();else {r.stand(.05);r.ticks(100);}
    if(mode==1){assert(!r.m.stand(.10,r.p.now));r.ticks(20);}
    const double w=simple::kWalkDuty-.75;
    unsigned ticks=mode==2?unsigned(std::ceil(((1.+(window/2)*.25+(window%2?(w+.25)/2:w/2))*simple::kWalkPeriodS+.5)/.02)):10;
    for(unsigned n=0;n<ticks;++n){const double angle=.02+(window%2?.006*std::sin(n*.3):0);
      r.in.quaternion[0]=std::cos(angle/2);r.in.quaternion[1]=std::sin(angle/2);r.tick();assert(r.m.state()!=simple::State::FAULT);}
    assert(simple::StopReferenceProbe::correction(r.m)>1e-8);
    if(mode==2){motion::GaitParameters gait;gait.duty=simple::kWalkDuty;unsigned stance=0;
      for(unsigned l=0;l<4;++l)stance+=motion::legPhase(gait,static_cast<motion::LegId>(l),r.m.walkCycles()).stance;
      assert(stance==(window%2?3u:4u));}
    const auto k=simple::StopReferenceProbe::original(r.m,r.p.now);
    const auto off=r.p.offCalls,enables=r.p.enables;
    r.m.stop();r.tick();simple::StopReferenceProbe::joined(r.m,k);
    assert(r.finishStop()&&r.m.energized()&&!r.m.stopSupportVerified());
    assert(r.p.offCalls==off&&r.p.enables==enables);
    if(mode==0){assert(!r.m.standResumeRefusal(r.p.now));assert(!r.m.stand(.10,r.p.now));
      for(unsigned n=0;n<1500&&r.m.state()!=simple::State::STANDING;++n){r.tick();assert(r.m.state()!=simple::State::FAULT);}
      assert(r.m.state()==simple::State::STANDING&&r.p.enables==enables);}
    else if(mode==1){
      std::array<uint16_t,12> held;std::copy(r.m.commandedRaw(),r.m.commandedRaw()+12,held.begin());
      assert(!r.m.standResumeRefusal(r.p.now)&&!r.m.stand(.15,r.p.now));r.tick();
      assert(std::equal(held.begin(),held.end(),r.m.commandedRaw()));
      for(unsigned n=0;n<2000&&r.m.state()!=simple::State::STANDING;++n){r.tick();assert(r.m.state()!=simple::State::FAULT);}
      assert(r.m.state()==simple::State::STANDING&&r.p.enables==enables&&r.p.offCalls==off);
    } // synchronous HEIGHT in kHeight restarts from held RAW, never its old endpoint
    else assert(!r.m.stand(.15,r.p.now));            // WALK: gait-law completion
  }
  // Sub-tick changes are not emitted frames. The accepted RAW, not the
  // unquantized floating goal or asynchronous encoder snapshot, is authoritative.
  {Rig r;r.stand(.15);r.ticks(100);const auto raw=r.p.target[1];const auto frames=r.p.frames;
    for(unsigned n=0;n<10;++n){r.in.quaternion[0]=std::cos(1e-7*n/2);r.in.quaternion[1]=std::sin(1e-7*n/2);r.tick();}
    assert(r.p.target[1]==raw&&r.p.frames==frames);
    const auto k=simple::StopReferenceProbe::original(r.m,r.p.now);
    for(double v:k.v)assert(v==0);r.m.stop();r.tick();simple::StopReferenceProbe::joined(r.m,k);assert(r.finishStop());}
  puts("Writer RAW handoff C0: constant/variable IMU HOLD, HEIGHT50->100 and WALK phase windows PASS; bounded new curve, no C1/C2 or physical braking claim");
}
void referenceBoundaries(){
  // The last accepted planned frame remains authoritative when the generator
  // changes phase before its next emission (including WALK's first release).
  {Rig r;r.walk();r.m.stop();r.tick();assert(r.m.stopActive());assert(r.finishStop());}
  {Rig r;r.stand(.10);r.ticks(100);r.m.stop();assert(r.finishStop());
    const auto enables=r.p.enables;assert(!r.m.stand(.15,r.p.now));r.m.stop();r.tick();
    assert(r.m.stopActive()&&r.finishStop()&&r.m.energized()&&r.p.enables==enables);}
}
void preemptionDeadline(){
  // The deadline is immutable even when every candidate frame is preempted.
  for(bool expire:{false,true}){Rig r;r.walk();r.ticks(380);r.m.stop();r.tick();
    const auto deadline=simple::StopReferenceProbe::deadline(r.m);const auto off=r.p.offCalls,enables=r.p.enables;
    for(unsigned n=0;n<(expire?300u:20u)&&r.m.state()!=simple::State::FAULT;++n){
      r.p.stopInFrame=true;r.m.stop();r.tick();
      assert(simple::StopReferenceProbe::deadline(r.m)==deadline);
      assert(r.m.stand(.10,r.p.now));
      if(r.p.now<=deadline)assert(r.m.state()!=simple::State::FAULT);
    }
    if(expire){assert(r.m.state()==simple::State::FAULT&&!strcmp(r.m.reason(),"STOP_NOT_SETTLED"));r.finishOff();
      assert(!r.m.energized()&&r.m.offMask()==8191&&r.p.offCalls>off);}
    else {r.p.stopInFrame=false;assert(r.finishStop()&&r.m.energized()&&r.p.offCalls==off);}
    assert(r.p.enables==enables);
  }
  // A preempted atomic attempt crossing the existing deadline is checked on
  // return, without sending another frame or waiting for the next release.
  {Rig r;r.walk();r.ticks(380);r.m.stop();r.tick();const auto deadline=simple::StopReferenceProbe::deadline(r.m);
    while(r.p.now+simple::kPeriodUs+10000<deadline){r.p.stopInFrame=true;r.tick();}
    r.p.frameDelayUs=30000;r.p.stopInFrame=true;r.tick();
    assert(test_clock_us>int64_t(deadline)&&r.m.state()==simple::State::FAULT&&!strcmp(r.m.reason(),"STOP_NOT_SETTLED"));
    r.p.frameDelayUs=0;r.finishOff();assert(r.m.offMask()==8191&&!r.m.energized());}
  puts("STOP preemption: short retry retains torque; continuous duplicates/concurrent STAND cannot extend deadline; crossing deadline in TX faults to SAFE_OFF 13/13");
}
void faultFeedback(){
  for(const char* reason:{"PRONE_PATH_NOT_CLEAR","INITIAL_POSE_OUTSIDE_SAFE_ENTRY"}){
    Rig r;r.stand(.15);r.ticks(100);r.p.honorPreempt=true;
    r.m.fault(reason);r.finishOff();const auto frames=r.p.frames,enables=r.p.enables;
    assert(r.m.busPreemptRequested()); // fault still fences every operational writer
    r.ticks(100); // longer than fast AND control feedback populations
    assert(r.m.fresh(r.p.now));
    assert(r.m.state()==simple::State::FAULT&&!strcmp(r.m.reason(),reason));
    assert(r.m.offVerified()&&r.m.offMask()==8191&&!r.m.energized());
    for(const auto& f:r.p.observed)assert(f.torque==0);
    for(unsigned i=0;i<12;++i)assert(r.m.feedback()[i].torque==0&&r.m.feedback()[i].controlStampUs>r.m.offVerifiedUs());
    assert(!strcmp(r.m.stand(.15,r.p.now),"FAULT_STOP_REQUIRED"));
    assert(!strcmp(r.m.prone(r.p.now),"FAULT_STOP_REQUIRED"));
    const auto preempted=r.p.preemptedReads;r.p.stopInRead=true;r.tick();
    assert(r.p.preemptedReads>preempted&&r.m.state()==simple::State::FAULT);
    const auto afterStop=r.p.preemptedReads;r.p.safeOffInRead=true;r.tick();
    assert(r.p.preemptedReads>afterStop);r.finishOff();r.ticks(100);
    assert(r.m.fresh(r.p.now)&&r.m.state()==simple::State::FAULT&&!strcmp(r.m.reason(),reason));
    assert(r.m.offVerified()&&r.p.frames==frames&&r.p.enables==enables);
  }
  puts("FAULT: bounded feedback continues after SAFE_OFF 13/13; reason/FAULT/write fence retained, incoming STOP and SAFE_OFF still preempt reads");
}
void stopReceivedInFault(){
  for(bool offAlready:{false,true}){
    Rig r;r.stand(.15);r.ticks(100);
    core::MotionService service;service.begin(&r.m,nullptr,nullptr);service.setReady(true,"READY");
    r.m.fault("PRONE_PATH_NOT_CLEAR");if(offAlready)r.finishOff();else r.p.verify=false;
    const auto frames=r.p.frames,enables=r.p.enables;
    motion::MotionCommand stop;stop.owner=motion::ClientOwner::DASHBOARD;stop.intent=motion::Intent::STOP;
    stop.sequence=1;strcpy(stop.id,"stop-in-fault");
    const auto ack=service.command(stop,r.p.now/1000);
    assert(ack.outcome==motion::CommandOutcome::ACCEPTED&&!strcmp(ack.reason,"STOP_REQUESTED_FAULT_LATCHED"));
    r.tick();service.update();core::MotionStatusSnapshot s;assert(service.captureStatus(s,r.p.now/1000));
    if(!offAlready){assert(!strcmp(s.activeOutcome,"ACCEPTED")&&s.finishedMs==0);r.p.verify=true;}
    for(unsigned n=0;n<60;++n){r.tick();service.update();assert(service.captureStatus(s,r.p.now/1000));if(!strcmp(s.activeOutcome,"COMPLETED"))break;}
    assert(!strcmp(s.activeOutcome,"COMPLETED")&&!strcmp(s.activeReason,"SAFE_OFF_VERIFIED_FAULT_LATCHED"));
    assert(s.state==simple::State::FAULT&&!strcmp(s.reason,"PRONE_PATH_NOT_CLEAR")&&s.offVerified&&s.offMask==8191&&!s.energized);
    assert(r.p.frames==frames&&r.p.enables==enables&&r.m.stand(.15,r.p.now));
    // A later STOP has its own receipt even after a previous terminal STOP.
    stop.sequence=2;strcpy(stop.id,"stop-in-fault-again");
    assert(service.command(stop,r.p.now/1000).outcome==motion::CommandOutcome::ACCEPTED);
    r.tick();service.update();assert(service.captureStatus(s,r.p.now/1000));
    assert(!strcmp(s.activeId,stop.id)&&!strcmp(s.activeOutcome,"COMPLETED")&&!strcmp(s.activeReason,"SAFE_OFF_VERIFIED_FAULT_LATCHED"));
  }
  // A NEW physical fault during an ordinary STOP still aborts that arrest.
  Rig r;r.walk();r.ticks(380);core::MotionService service;service.begin(&r.m,nullptr,nullptr);service.setReady(true,"READY");
  motion::MotionCommand stop;stop.owner=motion::ClientOwner::DASHBOARD;stop.intent=motion::Intent::STOP;stop.sequence=1;strcpy(stop.id,"stop-before-fault");
  assert(service.command(stop,r.p.now/1000).outcome==motion::CommandOutcome::ACCEPTED);r.tick();r.m.fault("TEST_NEW_PHYSICAL_FAULT");service.update();
  core::MotionStatusSnapshot s;assert(service.captureStatus(s,r.p.now/1000));
  assert(!strcmp(s.activeOutcome,"ABORTED")&&!strcmp(s.activeReason,"TEST_NEW_PHYSICAL_FAULT"));
  stop.sequence=2;strcpy(stop.id,"stop-after-new-fault");
  assert(service.command(stop,r.p.now/1000).outcome==motion::CommandOutcome::ACCEPTED);
  r.finishOff();service.update();assert(service.captureStatus(s,r.p.now/1000));
  assert(!strcmp(s.activeId,stop.id)&&!strcmp(s.activeReason,"SAFE_OFF_VERIFIED_FAULT_LATCHED"));
  assert(s.state==simple::State::FAULT&&!strcmp(s.reason,"TEST_NEW_PHYSICAL_FAULT"));
  puts("STOP received in FAULT reports its own verified-OFF completion; original fault stays latched; a new fault during ordinary STOP still aborts it");
}
void compatibleWalkRecovery(){
  // Explicit STAND after a verified WALK STOP completes the same gait's WALK STOP
  // from the stopped phase (kinematic proof only; contact/support NOT measured).
  const auto& w=simple::kWalkBox;constexpr double heights[]={.05,.10,.15};
  auto decode=[](const std::array<uint16_t,12>& raw,unsigned i){return (int(raw[i])-2048)*simple::kTickRad;};
  unsigned cases=0;
  for(bool ending:{false,true})for(unsigned stride=0;stride<3;++stride)for(double tilt:{-.01,.01})for(unsigned at:{1u,380u,1003u}){
    Rig r;r.stand(.15);r.in.quaternion[0]=std::cos(tilt/2);r.in.quaternion[1]=std::sin(tilt/2);r.ticks(50);
    r.walk(stride);r.ticks(at);if(ending){r.m.walkStop();r.ticks(150);}
    r.m.stop();assert(r.finishStop());
    if(const char* why=r.m.standResumeRefusal(r.p.now))std::fprintf(stderr,"walk recovery ending=%u stride=%u at=%u refused %s\n",ending,stride,at,why);
    assert(!r.m.standResumeRefusal(r.p.now)&&!r.m.stopSupportVerified());
    std::array<uint16_t,12> held;std::copy(r.m.commandedRaw(),r.m.commandedRaw()+12,held.begin());
    const auto frames=r.p.sent.size();const auto enables=r.p.enables,off=r.p.offCalls;
    assert(!r.m.stand(heights[stride],r.p.now));assert(r.p.sent.size()==frames); // intent only
    r.tick();assert(r.p.sent.size()==frames||r.p.sent[frames]==held);          // C0: same RAW, not even resent
    bool walking=true;
    for(unsigned n=0;n<3000&&r.m.state()!=simple::State::STANDING;++n){
      const double roll=tilt+.002*std::sin(n*.03);r.in.quaternion[0]=std::cos(roll/2);r.in.quaternion[1]=std::sin(roll/2);
      const auto before=r.p.sent.size();r.tick();assert(r.m.state()!=simple::State::FAULT&&r.m.state()!=simple::State::WALKING);
      walking=walking&&!strcmp(r.m.phase(),"WALK_STANDARD");
      if(walking&&r.p.sent.size()>before){const auto& f=r.p.sent.back();
        assert(r.p.drive.speed==simple::kWalkDrive.speed&&r.p.drive.acc==simple::kWalkDrive.acc);
        for(unsigned i=0;i<12;++i){assert(decode(f,i)>=w.lo[i]&&decode(f,i)<=w.hi[i]);
          if(before>frames)assert(std::abs(int(f[i])-int(r.p.sent[before-1][i]))<=20);}}  // <= 1000 ticks/s per release
    }
    assert(r.m.state()==simple::State::STANDING&&r.m.energized()&&r.p.enables==enables&&r.p.offCalls==off);
    assert(!strcmp(r.m.reason(),"STAND_REACHED")&&r.m.heightValid());
    assert(std::abs(r.m.requestedHeightM()-heights[stride])<1e-12);++cases;
  }
  // A second STOP during the recovery keeps the active terminal; STOP in WALK_SETTLE holds torque.
  {Rig r;r.walk(2);r.ticks(600);r.m.stop();assert(r.finishStop());assert(!r.m.stand(.15,r.p.now));r.ticks(150);
    assert(r.m.state()==simple::State::MOVING);r.m.stop();assert(r.finishStop());assert(!r.m.stand(.15,r.p.now));
    for(unsigned n=0;n<3000&&r.m.state()!=simple::State::STANDING;++n){r.tick();assert(r.m.state()!=simple::State::FAULT);}
    assert(r.m.state()==simple::State::STANDING&&!strcmp(r.m.reason(),"STAND_REACHED"));++cases;}
  {Rig r;r.walk(2);r.ticks(500);r.m.walkStop();r.phase("WALK_SETTLE");const auto enables=r.p.enables,off=r.p.offCalls;
    r.m.stop();r.tick();assert(r.m.stopActive()&&r.m.state()!=simple::State::FAULT);assert(r.finishStop());assert(!r.m.stand(.10,r.p.now));
    for(unsigned n=0;n<3000&&r.m.state()!=simple::State::STANDING;++n){r.tick();assert(r.m.state()!=simple::State::FAULT);}
    assert(r.p.enables==enables&&r.p.offCalls==off);++cases;}
  // Real faults during an already advancing recovery still take the existing SAFE_OFF path.
  for(bool powerFault:{false,true}){Rig r;r.walk(2);r.ticks(500);r.m.stop();assert(r.finishStop());
    const auto enables=r.p.enables;assert(!r.m.stand(.15,r.p.now));r.ticks(100);
    assert(r.m.state()==simple::State::MOVING);
    if(powerFault)r.in.powerOk=false;else r.in.imuValid=false;
    r.tick();assert(r.m.state()==simple::State::FAULT);r.finishOff();
    assert(!r.m.energized()&&r.m.offVerified()&&r.p.enables==enables);
  }
  for(unsigned race=0;race<4;++race){Rig r;r.walk();r.ticks(700);r.m.stop();assert(r.finishStop());
    const auto frames=r.p.frames,enables=r.p.enables,off=r.p.offCalls;assert(!r.m.stand(.10,r.p.now));
    if(race==0)r.m.stop();
    if(race==1)r.p.stopAfterFrame=true;
    if(race==2)r.m.safeOff();
    if(race==3){const unsigned i=simple::StopReferenceProbe::nextRead(r.m);const double q=w.hi[i]+.02<r.in.maxRad[i]-.005?w.hi[i]+.02:w.lo[i]-.02;
      r.p.observed[i].q=q;r.p.resolve(i,q,r.p.observed[i].raw);r.p.freeze=true;}
    r.tick();if(race==1)for(unsigned n=0;n<10&&!r.m.stopActive();++n)r.tick(); // first release resends nothing (same RAW)
    if(race<2){if(!r.m.stopActive())std::fprintf(stderr,"race %u state=%s reason=%s phase=%s\n",race,simple::name(r.m.state()),r.m.reason(),r.m.phase());assert(r.m.stopActive()&&r.finishStop()&&r.m.energized()&&r.p.offCalls==off);}
    else {if(race==3)std::fprintf(stderr,"race3 state=%s reason=%s phase=%s\n",simple::name(r.m.state()),r.m.reason(),r.m.phase());if(race==3)assert(r.m.state()==simple::State::FAULT&&!strcmp(r.m.reason(),"STOP_RECOVERY_PATH_NOT_CLEAR"));
      r.finishOff();assert(!r.m.energized());}
    assert(r.p.enables==enables);(void)frames;
  }
  std::printf("Explicit STAND from stopped WALK: %u gait-law completions (stance/swing, 30/45/60, decel, settle, re-STOP), RAW C0, configured WALK box, no enable/OFF; races retained (support NOT proven)\n",cases);
}
}
int main(){supportPlaneLevelTarget();terminalStopHold();compatibleWalkRecovery();stopReceivedInFault();stopPlanBounds();holdAndProne();movementAndDuplicates();evidenceAndFaults();fallbackAndWalkStop();phaseCoverageAndRaces();serviceCompletion();safeOffAndRecovery();writerHandoffWithImu();referenceBoundaries();preemptionDeadline();faultFeedback();
  std::puts("PASS contextual STOP software tests; PHYSICAL BRAKING AND SUPPORT NOT QUALIFIED");}
