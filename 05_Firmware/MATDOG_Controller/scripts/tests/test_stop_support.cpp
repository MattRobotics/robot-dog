// Scripted software observations only. No servo/ground-contact dynamics model:
// passing these tests does not qualify physical braking or support.
#include "../../src/simple/SimpleMotion.h"
#include "../../src/simple/StopPlan.h"
#include "../../src/core/MotionService.h"
#include "../../src/core/MotionJson.h"
#include "../../src/motion/PairedProneData.h"
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
      assert(m.port_->radians(i,m.commandedRaw_[i],k.q[i]));
      assert(m.port_->radians(i,m.previousRaw_[i],previous));
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
  bool radians(unsigned,uint16_t raw,double& q) override {q=(int(raw)-2048)*simple::kTickRad;return raw<4096;}
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
  assert(r.m.prone(r.p.now));assert(r.m.walk(1,r.p.now));
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
  Rig prone;prone.stand(.05);assert(!prone.m.prone(prone.p.now));prone.phase("PRONE_CONTACT_UNVERIFIED");
  checkHeldStop(prone);
  // PRONE has a fixed reference without UART traffic; STOP remains available.
  Rig longProne;longProne.stand(.05);assert(!longProne.m.prone(longProne.p.now));
  longProne.phase("PRONE_CONTACT_UNVERIFIED");longProne.ticks(100);checkHeldStop(longProne);
  // Loaded low HOLD outside kHeight, already admitted by dev.7 inside kLift.
  Rig loaded;loaded.stand(.05);loaded.ticks(100);
  loaded.p.observed[2].raw+=7;loaded.p.observed[2].q+=7*simple::kTickRad;
  loaded.p.freeze=true;loaded.p.observed[2].load=1024+176;
  const auto nominal=loaded.m.target();checkHeldStop(loaded);
  for(unsigned i=0;i<12;++i)assert(std::abs(motion::jointSlot(loaded.m.target(),i)-motion::jointSlot(nominal,i))<=simple::kTickRad/2);
  std::puts("STOP HOLD50/100/150 and PRONE: torque maintained, endpoint observed, support unverified, WALK/PRONE refused; explicit STAND tested separately");
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
  // Every leg's swing and each intervening four-stance window, at stride60.
  for(unsigned quarter=0;quarter<4;++quarter)for(bool swing:{false,true}){
    Rig r;r.walk(2);const double cycle=1+quarter*.25+(swing?.1625:.0375);
    r.ticks(static_cast<unsigned>(std::ceil(cycle*simple::kWalkPeriodS/.02)));
    motion::GaitParameters gait;gait.duty=simple::kWalkDuty;unsigned stance=0;
    for(unsigned i=0;i<4;++i)stance+=motion::legPhase(gait,static_cast<motion::LegId>(i),r.m.walkCycles()).stance;
    assert(stance==(swing?3u:4u));checkHeldStop(r);
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
  for(bool prone:{false,true}){Rig r;r.stand(.10);r.ticks(100);if(prone){assert(!r.m.prone(r.p.now));r.phase("PRONE_CONTACT_UNVERIFIED");}
    r.m.stop();r.tick();assert(!strcmp(r.m.stand(.15,r.p.now),"STOP_IN_PROGRESS"));assert(r.finishStop());
    const auto enables=r.p.enables,off=r.p.offCalls,frames=r.p.frames;r.ticks(30);assert(r.p.frames==frames);
    assert(!r.m.standResumeRefusal(r.p.now));assert(!r.m.stand(.15,r.p.now));
    for(unsigned n=0;n<3000&&r.m.state()!=simple::State::STANDING;++n){r.tick();assert(r.m.state()!=simple::State::FAULT);}
    assert(r.m.state()==simple::State::STANDING&&r.p.enables==enables&&r.p.offCalls==off);
  }
  {Rig r;r.walk();r.ticks(380);r.m.stop();assert(r.finishStop());assert(r.m.stand(.15,r.p.now));}
  {Rig r;r.stand(.10);r.ticks(50);assert(!r.m.stand(.15,r.p.now));r.ticks(20);r.m.stop();assert(r.finishStop());
    assert(!strcmp(r.m.stand(.15,r.p.now),"STOP_RECOVERY_POSTURE_UNQUALIFIED"));}
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
    unsigned ticks=mode==2?unsigned(std::ceil((1.+(window/2)*.25+(window%2?.1625:.0375))*simple::kWalkPeriodS/.02)):10;
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
    else assert(r.m.stand(.15,r.p.now));
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

}
int main(){stopPlanBounds();holdAndProne();movementAndDuplicates();evidenceAndFaults();fallbackAndWalkStop();phaseCoverageAndRaces();serviceCompletion();safeOffAndRecovery();writerHandoffWithImu();referenceBoundaries();preemptionDeadline();
  std::puts("PASS contextual STOP software tests; PHYSICAL BRAKING AND SUPPORT NOT QUALIFIED");}
