// Essential offline checks. Scripted observations exercise software sequencing;
// they are NOT a motor model and do not qualify physical STAND/WALK.
#include "../../src/simple/SimpleMotion.h"
#include "../../src/core/MotionService.h"
#include "../../src/core/MotionJson.h"
static int64_t test_clock_us=1000000;
extern "C" int64_t esp_timer_get_time(){return test_clock_us;}
#include "../../src/motion/PairedProneData.h"
#include "../../src/motion/GaitTiming.h"
#include <cassert>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <initializer_list>
using namespace matdog;
struct ScriptedObservationPort: simple::ServoPort {
  uint64_t now=1000000;simple::Feedback observed[12]{};
  uint16_t target[12]{};simple::Drive drive{};
  unsigned frames=0,enables=0,offCalls=0,reads=0;
  bool missing=false,holdObservation=false,verify=true,failFrame=false,stopOnRead=false,stopOnControl=false,stopOnEnable=false,stopOnFrame=false;
  simple::SimpleMotion* motion=nullptr;
  ScriptedObservationPort(){
    const auto* p=motion::paired_data::kKnots[0].p;
    for(unsigned l=0;l<4;++l){const double q[3]={((l==0||l==3)?-1:1)*p[7],p[l<2?2:0],std::fmax((l<2?-88.5:-89.)*3.14159265358979323846/180,p[l<2?3:1])};
      for(unsigned j=0;j<3;++j){auto& f=observed[l*3+j];f.q=q[j];f.raw=2048+std::lround(q[j]/simple::kTickRad);f.torque=0;f.temp=30;f.voltage=120;f.valid=true;f.goal=f.raw;target[l*3+j]=f.raw;}}
  }
  bool read(unsigned i,bool control,simple::Feedback& out) override {
    ++reads;if((stopOnRead||(control&&stopOnControl))&&motion){motion->stop();stopOnRead=stopOnControl=false;return false;}
    if(missing)return false;
    out=observed[i];out.stampUs=now;out.goal=target[i];out.goalSpeed=drive.speed;out.acc=drive.acc;
    (void)control;return true;
  }
  bool resolve(unsigned,double q,uint16_t& raw) override{raw=2048+std::lround(q/simple::kTickRad);return raw<4096;}
  bool frame(const uint16_t raw[12],simple::Drive d) override{
    assert(!motion||!motion->stopping());if(failFrame)return false;++frames;drive=d;
    for(unsigned i=0;i<12;++i)target[i]=raw[i];if(stopOnFrame&&motion){motion->stop();stopOnFrame=false;}return true;
  }
  bool enable(unsigned i) override{assert(frames>0);assert(target[i]==observed[i].raw);observed[i].torque=1;++enables;if(stopOnEnable&&motion){motion->stop();stopOnEnable=false;return false;}return true;}
  bool off() override{++offCalls;for(auto& f:observed)f.torque=0;return true;}
  bool verifyOff(unsigned) override{return verify;}
  // Explicitly publish scripted encoder snapshots. No assertion is made that
  // a servo would reach them with this timing, load, profile or torque.
  void publishCommandedPose(){if(holdObservation)return;for(unsigned i=0;i<12;++i){observed[i].raw=target[i];observed[i].q=(static_cast<int>(target[i])-2048)*simple::kTickRad;}}
};
struct Rig {
  simple::SimpleMotion m;ScriptedObservationPort p;simple::Inputs in;
  Rig(){m.begin(&p);p.motion=&m;in.ready=in.powerOk=in.authorityOk=in.imuValid=true;in.refusal="READY";
    const double lo[12]={-.783864,-.914253,-1.598408,-.783864,-.914253,-1.58,-.783864,-.914253,-1.592272,-.783864,-.914253,-1.590738};
    const double hi[12]={.783864,2.110758,.590583,.783864,2.10002,.581379,.771592,2.136835,.590583,.78233,2.129166,.585981};
    for(unsigned i=0;i<12;++i){in.minRad[i]=lo[i];in.maxRad[i]=hi[i];}
    for(unsigned i=0;i<15;++i)tick(false);assert(m.offVerified());
  }
  void tick(bool publish=true){p.now+=simple::kPeriodUs;in.imuStampUs=p.now;m.inputs(in);m.heartbeat(p.now);if(publish)p.publishCommandedPose();m.update(p.now);}
  bool reachStand(){for(unsigned i=0;i<3000;++i){tick();if(m.state()==simple::State::STANDING)return true;if(m.state()==simple::State::FAULT){std::printf("fault=%s phase=%s frames=%u\n",m.reason(),m.phase(),p.frames);return false;}}return false;}
};
int main(){
  // Exercise actual gait + IK across stance/swing joins, including the
  // terminal touchdown rule. Geometry is unchanged when timing is 6 s.
  motion::GaitParameters continuity;continuity.advanceXM=.04;continuity.liftM=.015;
  for(int terminal : {-1,4})for(unsigned leg=0;leg<4;++leg)for(unsigned cycle=1;cycle<4;++cycle)for(double event : {0.,.8}){
    const double c=cycle-motion::gaitOffset(motion::GaitType::WALK,static_cast<motion::LegId>(leg))+event;
    auto a=motion::solveGait(continuity,motion::gaitCartesian(continuity,c-1e-7,1./simple::kWalkPeriodS,0,terminal));
    auto b=motion::solveGait(continuity,motion::gaitCartesian(continuity,c+1e-7,1./simple::kWalkPeriodS,0,terminal));
    assert(a.target.valid&&b.target.valid);
    for(unsigned l=0;l<4;++l){
      const auto av=a.velocityRadS[l],bv=b.velocityRadS[l],aa=a.accelerationRadS2[l],ba=b.accelerationRadS2[l];
      assert(std::abs(av.hip-bv.hip)<1e-4&&std::abs(av.upper-bv.upper)<1e-4&&std::abs(av.lower-bv.lower)<1e-4);
      assert(std::abs(aa.hip-ba.hip)<.002&&std::abs(aa.upper-ba.upper)<.002&&std::abs(aa.lower-ba.lower)<.002);
    }
    for(unsigned i=0;i<12;++i)assert(std::abs(motion::jointSlot(a.target,i)-motion::jointSlot(b.target,i))<1e-5);
  }
  Rig r;assert(r.m.state()==simple::State::READY);
  assert(r.m.stand(0,r.p.now));assert(r.p.enables==0&&r.p.frames==0);
  assert(!r.m.stand(.15,r.p.now));assert(r.reachStand());assert(r.p.enables==12);assert(r.m.heightValid());
  std::printf("STAND sequence: %u coordinated pose writes; scripted observation only\n",r.p.frames);
  const auto beforeHeight=r.p.frames;assert(r.m.stand(.049,r.p.now));assert(!r.m.stand(.05,r.p.now));assert(r.p.frames==beforeHeight);assert(r.reachStand());assert(!r.m.stand(.15,r.p.now));assert(r.reachStand());
  assert(!r.m.walk(1,r.p.now));
  for(unsigned i=0;i<2000&&r.m.state()!=simple::State::WALKING;++i)r.tick();assert(r.m.state()==simple::State::WALKING);
  double maxStep=0,prev[12]{};for(unsigned i=0;i<12;++i)prev[i]=motion::jointSlot(r.m.target(),i);
  const auto start=std::chrono::steady_clock::now();
  for(unsigned k=0;k<1200;++k){r.tick();assert(r.m.state()==simple::State::WALKING);
    for(unsigned i=0;i<12;++i){const double q=motion::jointSlot(r.m.target(),i);maxStep=std::fmax(maxStep,std::abs(q-prev[i]));prev[i]=q;}}
  const auto elapsed=std::chrono::duration<double,std::micro>(std::chrono::steady_clock::now()-start).count();
  assert(maxStep<.05);r.m.walkStop();assert(r.reachStand());
  std::printf("WALK STANDARD production gait/IK: max target step %.6f rad, host mean %.1f us/frame (not ESP timing)\n",maxStep,elapsed/1200);
  const auto frames=r.p.frames;const auto offBefore=r.p.offCalls;r.m.stop();r.m.update(r.p.now+1);assert(r.p.offCalls==offBefore+1);r.tick(false);assert(r.p.frames==frames);assert(r.m.stopping());
  for(unsigned i=0;i<14;++i)r.tick(false);assert(r.m.offVerified());assert(r.m.state()==simple::State::STOPPED);
  // STOP must not impose a manual return to PRONE before another STAND.
  assert(!r.m.stand(.15,r.p.now));assert(r.reachStand());assert(r.p.enables==24);
  Rig interrupted;assert(!interrupted.m.stand(.15,interrupted.p.now));
  for(unsigned i=0;i<400&&interrupted.p.frames<25;++i)interrupted.tick();
  interrupted.m.stop();for(unsigned i=0;i<15;++i)interrupted.tick(false);assert(interrupted.m.offVerified());
  assert(!interrupted.m.stand(.15,interrupted.p.now));assert(interrupted.reachStand());
  Rig gaitRestart;assert(!gaitRestart.m.walk(2,gaitRestart.p.now));
  for(unsigned i=0;i<1200&&gaitRestart.m.state()!=simple::State::WALKING;++i)gaitRestart.tick();
  for(unsigned i=0;i<300;++i)gaitRestart.tick();gaitRestart.m.stop();for(unsigned i=0;i<15;++i)gaitRestart.tick(false);
  assert(gaitRestart.m.offVerified());assert(!gaitRestart.m.stand(.06,gaitRestart.p.now));assert(gaitRestart.reachStand());
  Rig native;assert(!native.m.stand(.15,native.p.now));native.p.holdObservation=true;
  for(unsigned i=0;i<3000&&native.m.state()!=simple::State::FAULT;++i)native.tick(false);
  assert(native.m.state()==simple::State::FAULT);assert(!strcmp(native.m.reason(),"NATIVE_TARGET_NOT_REACHED"));
  Rig alarm;assert(!alarm.m.stand(.15,alarm.p.now));assert(alarm.reachStand());alarm.p.observed[3].status=8;alarm.tick(false);alarm.tick(false);alarm.tick(false);alarm.tick(false);assert(alarm.m.state()==simple::State::FAULT);
  Rig comm;assert(!comm.m.stand(.15,comm.p.now));assert(comm.reachStand());comm.p.missing=true;
  for(unsigned i=0;i<20&&comm.m.state()!=simple::State::FAULT;++i)comm.tick(false);assert(comm.m.state()==simple::State::FAULT);
  Rig stop;assert(!stop.m.stand(.15,stop.p.now));assert(stop.reachStand());const unsigned before=stop.p.frames;stop.p.stopOnRead=true;stop.tick(false);assert(stop.p.frames==before&&stop.m.stopping());
  Rig link;assert(!link.m.stand(.15,link.p.now));assert(link.reachStand());link.p.now+=3100000;link.m.inputs(link.in);link.m.update(link.p.now);assert(link.m.state()==simple::State::FAULT);
  Rig bad;bad.in.maxRad[1]=.2;bad.m.inputs(bad.in);assert(bad.m.stand(.15,bad.p.now));assert(bad.p.enables==0);
  for(unsigned stride=0;stride<3;++stride){Rig preset;assert(!preset.m.walk(stride,preset.p.now));
    for(unsigned i=0;i<3000&&preset.m.state()!=simple::State::WALKING;++i)preset.tick();assert(preset.m.state()==simple::State::WALKING);
    // Both signs of a real quaternion; corrected targets must remain in limits.
    for(unsigned k=0;k<450;++k){const double roll=.015*std::sin(k*.01),pitch=.01*std::cos(k*.01);
      preset.in.quaternion[0]=std::cos(roll/2)*std::cos(pitch/2);preset.in.quaternion[1]=std::sin(roll/2)*std::cos(pitch/2);
      preset.in.quaternion[2]=std::cos(roll/2)*std::sin(pitch/2);preset.in.quaternion[3]=-std::sin(roll/2)*std::sin(pitch/2);
      preset.tick();assert(preset.m.state()==simple::State::WALKING);
      for(unsigned i=0;i<12;++i)assert(motion::jointSlot(preset.m.target(),i)>=preset.in.minRad[i]&&motion::jointSlot(preset.m.target(),i)<=preset.in.maxRad[i]);
    }
  }
  // Apply roll/pitch while STAND is still moving, not only at HOLD.
  Rig levelStand,tiltedStand;assert(!levelStand.m.stand(.15,levelStand.p.now));assert(!tiltedStand.m.stand(.15,tiltedStand.p.now));
  unsigned correctedMoving=0;double standCorrection=0;
  for(unsigned k=0;k<1200&&levelStand.m.state()!=simple::State::STANDING;++k){
    const double roll=.02*std::sin(k*.007),pitch=.01*std::sin(k*.011);
    tiltedStand.in.quaternion[0]=std::cos(roll/2)*std::cos(pitch/2);tiltedStand.in.quaternion[1]=std::sin(roll/2)*std::cos(pitch/2);
    tiltedStand.in.quaternion[2]=std::cos(roll/2)*std::sin(pitch/2);tiltedStand.in.quaternion[3]=-std::sin(roll/2)*std::sin(pitch/2);
    levelStand.tick();tiltedStand.tick();assert(tiltedStand.m.state()!=simple::State::FAULT);
    if(levelStand.m.state()==simple::State::MOVING&&tiltedStand.m.state()==simple::State::MOVING&&
       !strcmp(levelStand.m.phase(),tiltedStand.m.phase())&&levelStand.m.target().valid&&tiltedStand.m.target().valid){
      double delta=0;for(unsigned i=0;i<12;++i)delta=std::fmax(delta,std::abs(motion::jointSlot(levelStand.m.target(),i)-motion::jointSlot(tiltedStand.m.target(),i)));
      if(delta>1e-5){++correctedMoving;standCorrection=std::fmax(standCorrection,delta);}
    }
  }
  assert(correctedMoving>50);assert(tiltedStand.reachStand());
  assert(!tiltedStand.m.stand(.05,tiltedStand.p.now));assert(tiltedStand.reachStand());
  std::printf("IMU during MOVING STAND: %u different releases, max reference difference %.6f rad (software only)\n",correctedMoving,standCorrection);
  Rig stopControl;assert(!stopControl.m.stand(.15,stopControl.p.now));assert(stopControl.reachStand());stopControl.p.stopOnControl=true;
  for(unsigned k=0;k<6&&!stopControl.m.stopping();++k){const auto count=stopControl.p.offCalls;stopControl.tick(false);if(stopControl.m.stopping())assert(stopControl.p.offCalls==count+1);}
  assert(stopControl.m.stopping());
  for(unsigned phase=0;phase<4;++phase){Rig interrupt;assert(!interrupt.m.stand(.15,interrupt.p.now));
    if(phase!=0){for(unsigned k=0;k<100&&strcmp(interrupt.m.phase(),"ENABLE");++k)interrupt.tick();assert(!strcmp(interrupt.m.phase(),"ENABLE"));}
    if(phase<2)interrupt.p.stopOnControl=true;else if(phase==2)interrupt.p.stopOnEnable=true;else interrupt.p.stopOnFrame=true;
    const unsigned off=interrupt.p.offCalls;
    for(unsigned k=0;k<30&&!interrupt.m.stopping();++k)interrupt.tick(false);
    assert(interrupt.m.stopping()&&interrupt.p.offCalls==off+1&&interrupt.m.state()==simple::State::STOPPED);
    if(phase<2)assert(interrupt.p.enables==0);
  }
  Rig rejectedHeight;assert(!rejectedHeight.m.stand(.15,rejectedHeight.p.now));assert(rejectedHeight.reachStand());
  assert(rejectedHeight.m.stand(.05,rejectedHeight.p.now+300000));assert(rejectedHeight.m.requestedHeightM()==.15);
  Rig settling;assert(!settling.m.walk(0,settling.p.now));
  for(unsigned k=0;k<2000&&settling.m.state()!=simple::State::WALKING;++k)settling.tick();assert(settling.m.state()==simple::State::WALKING);
  for(unsigned k=0;k<1500&&settling.m.walkCycles()<1.5;++k)settling.tick();settling.m.walkStop();
  for(unsigned k=0;k<1500&&strcmp(settling.m.phase(),"WALK_SETTLE");++k)settling.tick();assert(!strcmp(settling.m.phase(),"WALK_SETTLE"));
  const auto settleFrames=settling.p.frames;for(auto& f:settling.p.observed)f.speed=100;
  settling.in.quaternion[0]=std::cos(.025/2);settling.in.quaternion[1]=std::sin(.025/2);
  for(unsigned k=0;k<25;++k){settling.tick();assert(!strcmp(settling.m.phase(),"WALK_SETTLE"));}
  assert(settling.p.frames>settleFrames);for(auto& f:settling.p.observed)f.speed=0;assert(settling.reachStand());
  Rig api;core::MotionService service;service.begin(&api.m,nullptr,nullptr);service.setReady(true,"READY");
  motion::MotionCommand cmd;cmd.owner=motion::ClientOwner::DASHBOARD;cmd.intent=motion::Intent::STAND;cmd.sequence=1;cmd.issued_at_ms=api.p.now/1000;cmd.ttl_ms=1000;std::strcpy(cmd.id,"test-api-1");
  test_clock_us=api.p.now;auto ack=service.command(cmd,api.p.now/1000);assert(ack.outcome==motion::CommandOutcome::ACCEPTED);
  assert(service.command(cmd,api.p.now/1000).outcome==motion::CommandOutcome::ACCEPTED);assert(api.reachStand());test_clock_us=api.p.now;service.update();
  char status[4096]{};assert(service.statusJson(status,sizeof(status)-400,api.p.now/1000));
  std::printf("API status %zu bytes (4096 B transport, 400 B reserved); JSON=%s\n",std::strlen(status),status);
  std::strcpy(cmd.id,"new-browser-session-1");cmd.intent=motion::Intent::STATUS;cmd.issued_at_ms=api.p.now/1000;assert(service.command(cmd,api.p.now/1000).outcome==motion::CommandOutcome::ACCEPTED);
  cmd.intent=motion::Intent::STOP;cmd.ttl_ms=0;cmd.issued_at_ms=0;assert(service.command(cmd,api.p.now/1000).outcome==motion::CommandOutcome::ACCEPTED);api.tick(false);assert(api.m.stopping());
  Rig unsafe;unsafe.p.observed[0].q=.7;assert(!unsafe.m.stand(.15,unsafe.p.now));for(unsigned i=0;i<30&&unsafe.m.state()!=simple::State::FAULT;++i)unsafe.tick(false);
  assert(unsafe.m.state()==simple::State::FAULT&&unsafe.p.enables==0);
  Rig off;off.p.verify=false;off.m.stop();for(unsigned i=0;i<30;++i)off.tick(false);assert(!off.m.offVerified()&&off.m.stopping());
  std::puts("PASS software sequencing, reachable heights, gait continuity, native-target timeout, STOP, alarms, communication/deadline loss; NO HARDWARE MOTION VALIDATION");
}
