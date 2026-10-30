// Essential offline checks. Scripted observations exercise software sequencing;
// they are NOT a motor model and do not qualify physical STAND/WALK.
#include "../../src/simple/SimpleMotion.h"
#include "../../src/core/MotionService.h"
#include "../../src/core/MotionJson.h"
#include "../../src/servo/ServoPacket.h"
#include "../../src/actuator/CalibrationTargetResolver.h"
static int64_t test_clock_us=1000000;
extern "C" int64_t esp_timer_get_time(){return test_clock_us;}
#include "../../src/motion/PairedProneData.h"
#include "../../src/motion/GaitTiming.h"
#include <array>
#include <cassert>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <initializer_list>
#include <iterator>
#include <string>
#include <vector>
using namespace matdog;
struct ScriptedObservationPort: simple::ServoPort {
  uint64_t now=1000000;simple::Feedback observed[12]{};
  uint16_t target[12]{};simple::Drive drive{};
  unsigned frames=0,enables=0,offCalls=0,reads=0;std::vector<unsigned> offFirst;
  unsigned referenceFaults=0,faultSlot=99;simple::Feedback faultFeedback;double faultReference=0;uint16_t faultRaw=0;
  void referenceFault(unsigned i,const simple::Feedback& f,double reference,uint16_t raw) override {
    assert(motion&&motion->stopping()&&offCalls>=2); // OFF precedes logging
    for(unsigned j=0;j<12;++j)if(int(j)!=stuckTorque)assert(observed[j].torque==0);
    ++referenceFaults;faultSlot=i;faultFeedback=f;faultReference=reference;faultRaw=raw;
  }
  bool missing=false,holdObservation=false,verify=true,failFrame=false,stopOnRead=false,stopOnControl=false,stopOnEnable=false,stopOnFrame=false;
  // One servo's next control READs go unanswered / its next SYNC_WRITEs are not
  // applied (no reply exists to tell): the bus faults a real rig can show.
  int missControlSlot=-1,dropFrameSlot=-1;unsigned missControlReads=0,dropFrames=0;
  // Every transmitted frame: RAW, drive, servos torque ON at that moment.
  std::vector<std::array<uint16_t,12>> sent;std::vector<simple::Drive> sentDrive;std::vector<unsigned> sentTorqueOn;
  simple::SimpleMotion* motion=nullptr;
  ScriptedObservationPort(){
    const auto* p=motion::paired_data::kKnots[0].p;
    for(unsigned l=0;l<4;++l){const double q[3]={((l==0||l==3)?-1:1)*p[7],p[l<2?2:0],std::fmax((l<2?-88.5:-89.)*3.14159265358979323846/180,p[l<2?3:1])};
      for(unsigned j=0;j<3;++j){auto& f=observed[l*3+j];f.q=q[j];f.raw=2048+std::lround(q[j]/simple::kTickRad);f.torque=0;f.temp=30;f.voltage=120;f.valid=true;f.goal=f.raw;target[l*3+j]=f.raw;}}
  }
  bool read(unsigned i,bool control,simple::Feedback& out) override {
    ++reads;if((stopOnRead||(control&&stopOnControl))&&motion){motion->stop();stopOnRead=stopOnControl=false;return false;}
    if(missing)return false;
    if(control&&static_cast<int>(i)==missControlSlot&&missControlReads){--missControlReads;return false;}
    out=observed[i];out.stampUs=now;out.goal=target[i];out.goalSpeed=drive.speed;out.acc=drive.acc;
    (void)control;return true;
  }
  bool resolve(unsigned,double q,uint16_t& raw) override{raw=2048+std::lround(q/simple::kTickRad);return raw<4096;}
  bool frame(const uint16_t raw[12],simple::Drive d) override{
    assert(!motion||!motion->stopping());if(failFrame)return false;++frames;drive=d;
    {std::array<uint16_t,12> f;unsigned on=0;for(unsigned i=0;i<12;++i){f[i]=raw[i];on+=observed[i].torque==1;}sent.push_back(f);sentDrive.push_back(d);sentTorqueOn.push_back(on);}
    const bool drop=dropFrames>0;if(drop)--dropFrames;
    for(unsigned i=0;i<12;++i)if(!drop||static_cast<int>(i)!=dropFrameSlot)target[i]=raw[i];if(stopOnFrame&&motion){motion->stop();stopOnFrame=false;}return true;
  }
  bool enable(unsigned i) override{assert(frames>0);assert(std::abs(int(target[i])-int(observed[i].raw))<=12);observed[i].torque=1;++enables;if(stopOnEnable&&motion){motion->stop();stopOnEnable=false;return false;}return true;}
  bool off(unsigned first=0) override{++offCalls;offFirst.push_back(first);for(unsigned i=first;i<12;++i)if(int(i)!=stuckTorque)observed[i].torque=0;return true;}
  int stuckTorque=-1;
  int failVerify=-1;  // one servo whose torque-OFF read-back fails
  bool verifyOff(unsigned i) override{return verify&&static_cast<int>(i)!=failVerify&&(i==12||observed[i].torque==0);}
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
  void tick(bool publish=true){p.now+=simple::kPeriodUs;in.imuStampUs=p.now;m.inputs(in);if(publish)p.publishCommandedPose();test_clock_us=p.now;m.update(p.now);m.updateTelemetry(p.now);}
  bool reachStand(){for(unsigned i=0;i<3000;++i){tick();if(m.state()==simple::State::STANDING)return true;if(m.state()==simple::State::FAULT){std::printf("fault=%s phase=%s frames=%u\n",m.reason(),m.phase(),p.frames);return false;}}return false;}
};
int main(){
  // Exercise the production gait + IK across every join: swing lift-off and
  // touchdown, four-contact sway windows, end of the start ramp, decel start,
  // terminal touchdown and recentring end, for each stride and quarter stop.
  for(unsigned stride=0;stride<3;++stride)for(double stop : {1.,1.25,1.5,1.75,2.}){
    motion::GaitParameters g;g.liftM=simple::kWalkLiftM;g.duty=simple::kWalkDuty;g.advanceXM=simple::kStrideM[stride];
    g.swayXM=simple::kWalkSwayXM;g.swayYM=simple::kWalkSwayYM;g.startCycles=simple::kWalkStartCycles;g.stopCycles=simple::kWalkStopCycles;
    const double terminal=stop+g.stopCycles,w=motion::gaitSettleCycles(g);
    for(double base=0;base<terminal+1;base+=.25)for(double event : {0.,w}){  // quarter touchdown and four-contact window end
      const double c=base+event;if(c<=1e-6||c>=terminal+w-1e-6)continue;
      const double term=c>stop?terminal:-1;
      auto a=motion::solveGait(g,motion::gaitCartesian(g,c-1e-7,1./simple::kWalkPeriodS,0,term));
      auto b=motion::solveGait(g,motion::gaitCartesian(g,c+1e-7,1./simple::kWalkPeriodS,0,term));
      const auto native=motion::solveGait(g,b.cartesian,nullptr,false);
      assert(native.status==b.status&&native.target.valid==b.target.valid);
      for(unsigned l=0;l<4;++l)assert(native.condition[l]==b.condition[l]);
      for(unsigned i=0;i<12;++i)assert(motion::jointSlot(native.target,i)==motion::jointSlot(b.target,i));
      assert(a.target.valid&&b.target.valid);
      for(unsigned l=0;l<4;++l){
        const auto av=a.velocityRadS[l],bv=b.velocityRadS[l],aa=a.accelerationRadS2[l],ba=b.accelerationRadS2[l];
        assert(std::abs(av.hip-bv.hip)<1e-4&&std::abs(av.upper-bv.upper)<1e-4&&std::abs(av.lower-bv.lower)<1e-4);
        assert(std::abs(aa.hip-ba.hip)<.002&&std::abs(aa.upper-ba.upper)<.002&&std::abs(aa.lower-ba.lower)<.002);
      }
      for(unsigned i=0;i<12;++i)assert(std::abs(motion::jointSlot(a.target,i)-motion::jointSlot(b.target,i))<1e-5);
    }
    // Rest at both ends: zero joint velocity/acceleration in the gait clock.
    for(double c : {0.,terminal+w}){auto f=motion::solveGait(g,motion::gaitCartesian(g,c,1./simple::kWalkPeriodS,0,c>0?terminal:-1));assert(f.target.valid);
      for(unsigned l=0;l<4;++l){const auto v=f.velocityRadS[l],a=f.accelerationRadS2[l];
        assert(std::abs(v.hip)+std::abs(v.upper)+std::abs(v.lower)<1e-9&&std::abs(a.hip)+std::abs(a.upper)+std::abs(a.lower)<1e-9);}}
  }
  // STAND reference: certified route vias, analytic ceilings and per-joint range.
  for(int group=0;group<2;++group){simple::ViaPlan plan;
    for(unsigned k:group?std::vector<unsigned>(std::begin(simple::collision::kLiftKnots),std::end(simple::collision::kLiftKnots)):
                         std::vector<unsigned>(std::begin(simple::collision::kDeployKnots),std::end(simple::collision::kDeployKnots))){
      const double* p=motion::paired_data::kKnots[k].p;double x[simple::ViaPlan::kDim];
      for(unsigned l=0;l<4;++l){x[l*3]=((l==0||l==3)?-1:1)*p[7];x[l*3+1]=p[l<2?2:0];x[l*3+2]=std::fmax((l<2?-88.5:-89.)*3.14159265358979323846/180,p[l<2?3:1]);}
      x[12]=p[4];x[13]=p[5];x[14]=p[6];assert(plan.add(x));}
    assert(plan.plan(simple::kStandSpeedRadS,simple::kStandAccelerationRadS2)&&plan.iterations<16);
    double previous[simple::ViaPlan::kDim]{};
    for(double t=0;t<=plan.duration+1e-9;t+=.001){double q[simple::ViaPlan::kDim],v[simple::ViaPlan::kDim],a[simple::ViaPlan::kDim];plan.sample(t,q,v,a);
      for(unsigned i=0;i<12;++i){double lo=1e9,hi=-1e9;for(unsigned k=0;k<plan.n;++k){lo=std::fmin(lo,plan.p[k][i]);hi=std::fmax(hi,plan.p[k][i]);}
        assert(q[i]>=lo-1e-12&&q[i]<=hi+1e-12&&std::abs(v[i])<=simple::kStandSpeedRadS+1e-9&&std::abs(a[i])<=simple::kStandAccelerationRadS2+1e-9);
        if(t>0)assert(std::abs(q[i]-previous[i])<=simple::kStandSpeedRadS*.001+1e-9);previous[i]=q[i];}}
    std::printf("STAND %s reference: %u vias, %.3f s, no intermediate arrival\n",group?"LIFT":"DEPLOY",plan.n,plan.duration);
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
  // STOP inside STAND_LIFT: restart resumes the certified LIFT group from the measured pose.
  Rig liftStop;assert(!liftStop.m.stand(.15,liftStop.p.now));
  for(unsigned i=0;i<1000&&strcmp(liftStop.m.phase(),"STAND_LIFT");++i)liftStop.tick();
  for(unsigned i=0;i<60;++i)liftStop.tick();assert(!strcmp(liftStop.m.phase(),"STAND_LIFT"));
  liftStop.m.stop();for(unsigned i=0;i<15;++i)liftStop.tick(false);assert(liftStop.m.offVerified());
  assert(!liftStop.m.stand(.15,liftStop.p.now));const char* resumed="";
  for(unsigned i=0;i<200&&(!strcmp(resumed,"")||!strcmp(resumed,"ACQUIRE")||!strcmp(resumed,"REFERENCE")||!strcmp(resumed,"ENABLE")||!strcmp(resumed,"IDLE"));++i){liftStop.tick();resumed=liftStop.m.phase();}
  assert(!strcmp(resumed,"STAND_LIFT"));assert(liftStop.reachStand());
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
  // A blocked Controller loop (3.1 s without a release) is a real fault.
  Rig blocked;assert(!blocked.m.stand(.15,blocked.p.now));assert(blocked.reachStand());blocked.p.now+=3100000;blocked.m.inputs(blocked.in);blocked.m.update(blocked.p.now);
  assert(blocked.m.state()==simple::State::FAULT&&!strcmp(blocked.m.reason(),"MOTION_DEADLINE_LOST"));
  // No command source after acceptance (closed Dashboard, no session, no
  // poll): STAND holds and WALK continues for 10 simulated minutes; only a
  // command or a physical fault ends them.
  Rig held;assert(!held.m.stand(.10,held.p.now));assert(held.reachStand());
  for(unsigned k=0;k<30000;++k){held.tick();assert(held.m.state()==simple::State::STANDING);}
  assert(held.m.energized()&&!held.m.stopping()&&held.m.requestedHeightM()==.10);
  Rig walking;assert(!walking.m.walk(0,walking.p.now));
  for(unsigned k=0;k<3000&&walking.m.state()!=simple::State::WALKING;++k)walking.tick();
  for(unsigned k=0;k<30000;++k){walking.tick();assert(walking.m.state()==simple::State::WALKING);}
  assert(walking.m.walkCycles()>90&&walking.m.energized());
  walking.m.walkStop();assert(walking.reachStand());
  std::printf("No command source: STAND held 600 s, WALK STANDARD %.1f cycles in 600 s, WALK STOP -> STAND (software only)\n",walking.m.walkCycles());
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
  char status[core::kMotionStatusJsonBytes]{};assert(service.statusJson(status,sizeof(status)-640,api.p.now/1000));
  std::printf("API status %zu bytes (5120 B transport, 640 B reserved); JSON=%s\n",std::strlen(status),status);
  std::strcpy(cmd.id,"new-browser-session-1");cmd.intent=motion::Intent::STATUS;cmd.issued_at_ms=api.p.now/1000;assert(service.command(cmd,api.p.now/1000).outcome==motion::CommandOutcome::ACCEPTED);
  cmd.intent=motion::Intent::STOP;cmd.ttl_ms=0;cmd.issued_at_ms=0;assert(service.command(cmd,api.p.now/1000).outcome==motion::CommandOutcome::ACCEPTED);api.tick(false);assert(api.m.stopping());
  Rig unsafe;unsafe.p.observed[0].q=.7;assert(!unsafe.m.stand(.15,unsafe.p.now));for(unsigned i=0;i<30&&unsafe.m.state()!=simple::State::FAULT;++i)unsafe.tick(false);
  assert(unsafe.m.state()==simple::State::FAULT&&unsafe.p.enables==0);
  Rig off;off.p.verify=false;off.m.stop();for(unsigned i=0;i<30;++i)off.tick(false);assert(!off.m.offVerified()&&off.m.stopping());
  // PRONE -> STAND50: one DEPLOY and one LIFT reference, no HEIGHT move and
  // so no STAND150 detour. The rig IMU stays level while the route nominal is
  // pitched up to 0.18 rad, so a bounded correction is active during the rise;
  // with level nominal and level IMU in HOLD it must decay to the exact stand.
  Rig low;assert(!low.m.stand(.05,low.p.now));std::string lowPhases;const char* lowPhase="";
  for(unsigned k=0;k<3000&&low.m.state()!=simple::State::STANDING;++k){low.tick();assert(low.m.state()!=simple::State::FAULT);
    if(strcmp(lowPhase,low.m.phase())){lowPhase=low.m.phase();lowPhases+=std::string(lowPhase)+" ";}}
  assert(low.m.state()==simple::State::STANDING&&lowPhases=="ACQUIRE REFERENCE ENABLE DISENGAGE ALIGN STAND_DEPLOY STAND_LIFT HOLD ");
  const double lowSeconds=(low.p.now-1300000)/1e6;
  for(unsigned k=0;k<100;++k)low.tick();  // 2 s: correction slew 20 mrad/s
  {auto d=motion::canonicalStandDefinition();d.lowBodyHeightM=.025;d.standBodyHeightM=.05;const auto s=motion::generateStandTarget(d).target;
   for(unsigned i=0;i<12;++i)assert(std::abs(motion::jointSlot(low.m.target(),i)-motion::jointSlot(s,i))<1e-9);}
  std::printf("PRONE->STAND50: %s(%.2f s replay)\n",lowPhases.c_str(),lowSeconds);
  // WALK STOP ends recentred on the canonical 150 mm stand.
  Rig centred;assert(!centred.m.walk(2,centred.p.now));
  for(unsigned k=0;k<3000&&centred.m.state()!=simple::State::WALKING;++k)centred.tick();
  for(unsigned k=0;k<500;++k)centred.tick();centred.m.walkStop();
  for(unsigned k=0;k<2000&&strcmp(centred.m.phase(),"WALK_SETTLE");++k)centred.tick();assert(!strcmp(centred.m.phase(),"WALK_SETTLE"));
  {auto d=motion::canonicalStandDefinition();const auto s=motion::generateStandTarget(d).target;  // last gait sample
   for(unsigned i=0;i<12;++i)assert(std::abs(motion::jointSlot(centred.m.target(),i)-motion::jointSlot(s,i))<1e-6);}
  assert(centred.reachStand());
  // Repeated STOP while SAFE_OFF is pending (dashboard/USB duplicates): one
  // torque-OFF broadcast, then 13 read-backs, never restarted, no re-enable;
  // from STANDING, from MOVING (STAND_LIFT) and from WALKING.
  for(unsigned during=0;during<3;++during){
    Rig s;
    if(during==0){assert(!s.m.stand(.15,s.p.now));assert(s.reachStand());}
    else if(during==1){assert(!s.m.stand(.15,s.p.now));for(unsigned i=0;i<1000&&strcmp(s.m.phase(),"STAND_LIFT");++i)s.tick();assert(!strcmp(s.m.phase(),"STAND_LIFT"));}
    else{assert(!s.m.walk(1,s.p.now));for(unsigned i=0;i<3000&&s.m.state()!=simple::State::WALKING;++i)s.tick();assert(s.m.state()==simple::State::WALKING);}
    const unsigned off=s.p.offCalls,enables=s.p.enables,frames=s.p.frames;unsigned updates=0;
    while(!s.m.offVerified()&&updates<60){s.m.stop();s.tick(false);++updates;}
    assert(updates==14&&s.p.offCalls==off+1&&s.m.offMask()==0x1fff&&!s.m.energized()&&!s.m.stopping());
    assert(s.p.enables==enables&&s.p.frames==frames&&s.m.state()==simple::State::STOPPED&&!strcmp(s.m.reason(),"STOP_REQUESTED"));
    assert(!s.m.stand(.15,s.p.now));assert(s.reachStand());  // a STOP never needs a manual reset
  }
  // One servo fails its read-back: SAFE_OFF stays pending and re-broadcasts;
  // repeated STOPs keep the 12 verified servos; nothing can re-enable; the
  // servo recovering completes 13/13 without another STOP.
  Rig failing;assert(!failing.m.stand(.15,failing.p.now));assert(failing.reachStand());failing.p.failVerify=5;
  const unsigned failOff=failing.p.offCalls,failEnables=failing.p.enables,failFrames=failing.p.frames;
  for(unsigned i=0;i<80;++i){if(i%3==0)failing.m.stop();failing.tick(false);}
  assert(!failing.m.offVerified()&&failing.m.stopping()&&failing.m.energized()&&failing.m.offMask()==(0x1fff&~(1u<<5)));
  assert(failing.p.offCalls>failOff+3&&failing.p.enables==failEnables&&failing.p.frames==failFrames);
  assert(!strcmp(failing.m.stand(.15,failing.p.now),"SAFE_OFF_IN_PROGRESS")&&failing.p.enables==failEnables);
  failing.p.failVerify=-1;unsigned recovered=0;while(!failing.m.offVerified()&&recovered<40){failing.tick(false);++recovered;}
  assert(failing.m.offVerified()&&recovered<=14&&!failing.m.energized());
  // A STOP after completion is a new SAFE_OFF (fresh broadcast + 13 reads).
  Rig again;assert(again.m.offVerified());const unsigned againOff=again.p.offCalls;again.m.stop();again.tick(false);
  assert(!again.m.offVerified()&&again.p.offCalls==againOff+1);for(unsigned i=0;i<13;++i)again.tick(false);assert(again.m.offVerified());
  // STOP after a FAULT clears the latch once (one restart); duplicates then keep progress.
  Rig faulted;assert(!faulted.m.stand(.15,faulted.p.now));assert(faulted.reachStand());faulted.p.missing=true;
  for(unsigned i=0;i<20&&faulted.m.state()!=simple::State::FAULT;++i)faulted.tick(false);assert(faulted.m.state()==simple::State::FAULT);
  faulted.p.missing=false;const unsigned faultOff=faulted.p.offCalls;unsigned afterStop=0;
  while(!faulted.m.offVerified()&&afterStop<60){faulted.m.stop();faulted.tick(false);++afterStop;}
  assert(afterStop==14&&faulted.p.offCalls==faultOff+1&&faulted.m.state()==simple::State::STOPPED);
  assert(!faulted.m.stand(.15,faulted.p.now));assert(faulted.reachStand());
  // pose_region: shared certified-region predicate on 12 fresh ACTUAL angles.
  Rig pose;for(unsigned i=0;i<8;++i)pose.tick(false);assert(!strcmp(pose.m.poseRegion(pose.p.now),"PRONE_ENTRY"));
  assert(!pose.m.stand(.15,pose.p.now));assert(pose.reachStand());assert(!strcmp(pose.m.poseRegion(pose.p.now),"STAND_HEIGHT"));
  pose.p.missing=true;for(unsigned i=0;i<20;++i)pose.tick(false);assert(!strcmp(pose.m.poseRegion(pose.p.now),"UNKNOWN"));
  std::puts("STOP repeated: 13/13 in 14 updates from STANDING/MOVING/WALKING with one broadcast; failing read-back re-broadcasts and keeps 12/13; no re-enable");
  // Pre-enable acquisition on the real PRONE of 24/10/2026 (USB log
  // MATDOG_STAND_WALK_20261024_152020: RAW and ACTUAL angles). GoalPosition
  // reads 0 at power-on, before the reference SYNC_WRITE, as on the robot.
  {
    const uint16_t raw[12]={1989,3077,3111,2023,1114,967,2092,1012,997,2022,3140,3130};
    const double q[12]={-.009204,1.506369,-1.547787,.010738,1.506369,-1.540117,.00767,1.609146,-1.590738,-.009204,1.607612,-1.592272};
    auto prone=[&](Rig& r){for(unsigned i=0;i<12;++i){r.p.observed[i].raw=raw[i];r.p.observed[i].q=q[i];r.p.target[i]=0;}
      for(unsigned i=0;i<8;++i)r.tick(false);assert(!strcmp(r.m.poseRegion(r.p.now),"PRONE_ENTRY"));};
    auto preparing=[](const Rig& r){return !strcmp(r.m.phase(),"ACQUIRE")||!strcmp(r.m.phase(),"REFERENCE")||!strcmp(r.m.phase(),"ENABLE");};
    // Torque OFF + identical frames: the scripted pose is held until all 12 are enabled.
    auto prepare=[&](Rig& r){unsigned n=0;while(n<100&&r.m.state()==simple::State::MOVING&&preparing(r)){r.tick(false);++n;}return n;};
    auto toEnable=[](Rig& r,unsigned slot){for(unsigned i=0;i<60&&!(r.p.enables==slot&&!strcmp(r.m.phase(),"ENABLE"));++i)r.tick(false);
      assert(r.p.enables==slot&&!strcmp(r.m.phase(),"ENABLE"));};
    auto shift=[&](Rig& r,unsigned slot,int ticks){r.p.observed[slot].raw=raw[slot]+ticks;r.p.observed[slot].q=q[slot]+ticks*simple::kTickRad;};
    // 1. Stable PRONE -> STAND50: ACQUIRE 12 + REFERENCE 1 + ENABLE 12 releases, then the route.
    {Rig r;prone(r);assert(!r.m.stand(.05,r.p.now));assert(prepare(r)==25&&r.p.enables==12&&r.p.frames==2&&r.m.state()==simple::State::MOVING);  // + DISENGAGE
     assert(r.reachStand());}
    // 2./3. Encoder variation inside the 12-tick native allowance is not a change; beyond it, abort before torque.
    for(int ticks:{1,-2,11,-11,13,-13}){Rig r;prone(r);assert(!r.m.stand(.05,r.p.now));toEnable(r,4);shift(r,9,ticks);prepare(r);
      if(std::abs(ticks)<=11)assert(r.p.enables==12&&r.m.state()==simple::State::MOVING);
      else assert(r.m.state()==simple::State::FAULT&&!strcmp(r.m.reason(),"REFERENCE_CHANGED_BEFORE_ENABLE")&&r.p.enables==9);}
    // One diagnostic identifies the exact predicate/slot, even if the next
    // HTTP sample sees OFF again. Stable encoders alone do not rule out torque.
    for(unsigned slot : {0u,5u,11u}) for(bool torque : {false,true}) {
      Rig r;prone(r);assert(!r.m.stand(.05,r.p.now));toEnable(r,slot);
      if(torque){r.p.observed[slot].torque=1;r.p.stuckTorque=slot;}else shift(r,slot,13);
      r.tick(false);if(torque){r.tick(false);r.tick(false);}
      assert(r.m.state()==simple::State::FAULT&&r.p.enables==slot&&r.p.referenceFaults==1&&r.p.faultSlot==slot);
      assert(r.p.faultFeedback.torque==(torque?1:0)&&r.p.faultRaw==raw[slot]);
      assert(std::abs(r.p.faultReference-q[slot])<1e-9);
      assert(r.p.faultFeedback.raw==raw[slot]+(torque?0:13));
      r.p.stuckTorque=-1;for(unsigned n=0;n<30;++n)r.tick(false);
      assert(r.m.offVerified()&&r.p.referenceFaults==1);
    }
    // A single unanswered control READ (ENABLE or ACQUIRE) or a reference frame one
    // servo did not apply is not a pose change: same servo again, torque still OFF.
    for(unsigned misses:{1u,2u}){Rig r;prone(r);assert(!r.m.stand(.05,r.p.now));toEnable(r,5);r.p.missControlSlot=5;r.p.missControlReads=misses;
      assert(prepare(r)==7+misses&&r.p.enables==12&&r.m.state()==simple::State::MOVING);assert(r.reachStand());}
    {Rig r;prone(r);assert(!r.m.stand(.05,r.p.now));r.p.missControlSlot=3;r.p.missControlReads=2;
     assert(prepare(r)==27&&r.p.enables==12);assert(r.reachStand());}
    {Rig r;prone(r);r.p.dropFrameSlot=7;r.p.dropFrames=1;assert(!r.m.stand(.05,r.p.now));
     assert(prepare(r)==26&&r.p.frames==3&&r.p.enables==12);assert(r.reachStand());}
    std::puts("PRONE 24/10 pre-enable: stable/<=11 ticks/1-2 missed READs/1 unapplied frame reach ENABLE x12; 13 ticks aborts before torque");
    // Persistent: three consecutive failures on one servo end the command; that servo is never enabled.
    {Rig r;prone(r);assert(!r.m.stand(.05,r.p.now));toEnable(r,5);r.p.missControlSlot=5;r.p.missControlReads=3;prepare(r);
     assert(r.m.state()==simple::State::FAULT&&!strcmp(r.m.reason(),"SERVO_CONTROL_READ_LOST")&&r.p.enables==5);}
    {Rig r;prone(r);r.p.dropFrameSlot=7;r.p.dropFrames=100;assert(!r.m.stand(.05,r.p.now));prepare(r);
     assert(r.m.state()==simple::State::FAULT&&!strcmp(r.m.reason(),"SERVO_TORQUE_OR_GOAL_LOST")&&r.p.enables==7&&r.p.frames==3);}
    // 4. No feedback: no reference, no frame, no torque.
    {Rig r;prone(r);assert(!r.m.stand(.05,r.p.now));r.p.missing=true;prepare(r);
     assert(r.m.state()==simple::State::FAULT&&!strcmp(r.m.reason(),"INITIAL_SERVO_READ_FAILED")&&r.p.enables==0&&r.p.frames==0);}
    // 5. STOP while a READ is being retried: SAFE_OFF 13/13, nothing more enabled.
    {Rig r;prone(r);assert(!r.m.stand(.05,r.p.now));toEnable(r,2);r.p.missControlSlot=2;r.p.missControlReads=2;r.tick(false);
     r.m.stop();for(unsigned i=0;i<15;++i)r.tick(false);
     assert(r.m.offVerified()&&r.m.state()==simple::State::STOPPED&&r.p.enables==2&&!r.m.energized());}
    // 6. A real change before enable latches FAULT; nothing runs by itself; STOP then a new STAND proceeds.
    {Rig r;prone(r);assert(!r.m.stand(.05,r.p.now));toEnable(r,4);shift(r,9,40);prepare(r);
     assert(r.m.state()==simple::State::FAULT&&!strcmp(r.m.reason(),"REFERENCE_CHANGED_BEFORE_ENABLE"));
     const unsigned frames=r.p.frames,enables=r.p.enables;for(unsigned i=0;i<200;++i)r.tick(false);
     assert(r.m.offVerified()&&r.m.state()==simple::State::FAULT&&r.p.frames==frames&&r.p.enables==enables);
     assert(!strcmp(r.m.stand(.05,r.p.now),"FAULT_STOP_REQUIRED"));
     r.m.stop();for(unsigned i=0;i<15;++i)r.tick(false);assert(r.m.state()==simple::State::STOPPED&&r.m.offVerified());
     for(unsigned i=0;i<12;++i)r.p.target[i]=0;  // torque OFF: the new reference comes from the new ACTUAL pose
     assert(!r.m.stand(.05,r.p.now));assert(prepare(r)==25&&r.p.enables==enables+12);assert(r.reachStand());}
    std::puts("PRONE 24/10 pre-enable: 3 consecutive failures/no feedback refuse torque; STOP during retry -> SAFE_OFF; real change -> FAULT, STOP, new STAND");
    // Re-sent reference while 7 servos already hold torque: the identical
    // packet (real encoder, real slot IDs), the measured RAW, 300/8; OFF only
    // for the not-yet-enabled suffix, no other goal frame before the 12th enable.
    {Rig r;prone(r);r.p.dropFrameSlot=7;r.p.dropFrames=1;assert(!r.m.stand(.05,r.p.now));const unsigned off=r.p.offCalls;prepare(r);
     assert(r.p.enables==12&&r.p.sent.size()==3&&r.p.offCalls==off+2&&r.p.offFirst[off]==0&&r.p.offFirst[off+1]==7&&r.p.sentTorqueOn[0]==0&&r.p.sentTorqueOn[1]==7);
     uint8_t ids[12];for(unsigned i=0;i<12;++i)ids[i]=actuator::slotServo(i)->bus_id;
     uint8_t first[servo::packet::kGoalFrameBytes],again[servo::packet::kGoalFrameBytes];
     for(unsigned k=0;k<2;++k){assert(r.p.sentDrive[k].speed==simple::kStandDrive.speed&&r.p.sentDrive[k].acc==simple::kStandDrive.acc);for(unsigned i=0;i<12;++i)assert(r.p.sent[k][i]==raw[i]);}
     assert(servo::packet::buildGoalFrame(ids,r.p.sent[0].data(),12,simple::kStandDrive.speed,simple::kStandDrive.acc,first,sizeof(first))==sizeof(first));
     assert(servo::packet::buildGoalFrame(ids,r.p.sent[1].data(),12,simple::kStandDrive.speed,simple::kStandDrive.acc,again,sizeof(again))==sizeof(again));
     assert(!std::memcmp(first,again,sizeof(first)));
     for(unsigned i=0;i<12;++i)assert(r.p.observed[i].torque==1);}  // already enabled prefix was never disabled/re-enabled
    // First, middle and last servo: 1-2 failures recover in the same STAND, the
    // third ends it before that servo is enabled (READ and unapplied frame).
    for(unsigned slot:{0u,5u,11u})for(unsigned n:{1u,2u,3u}){
      {Rig r;prone(r);assert(!r.m.stand(.05,r.p.now));toEnable(r,slot);r.p.missControlSlot=slot;r.p.missControlReads=n;prepare(r);
       if(n<3)assert(r.p.enables==12&&r.m.state()==simple::State::MOVING);
       else assert(r.m.state()==simple::State::FAULT&&!strcmp(r.m.reason(),"SERVO_CONTROL_READ_LOST")&&r.p.enables==slot&&r.p.observed[slot].torque==0);}
      {Rig r;prone(r);r.p.dropFrameSlot=slot;r.p.dropFrames=n;assert(!r.m.stand(.05,r.p.now));prepare(r);
       if(n<3)assert(r.p.enables==12&&r.m.state()==simple::State::MOVING&&r.p.sent.size()==n+2);
       else assert(r.m.state()==simple::State::FAULT&&!strcmp(r.m.reason(),"SERVO_TORQUE_OR_GOAL_LOST")&&r.p.enables==slot&&r.p.observed[slot].torque==0);
       for(unsigned k=1;k+1<r.p.sent.size()||(n>=3&&k<r.p.sent.size());++k)assert(r.p.sent[k]==r.p.sent[0]);}
    }
    // STOP latched while the reference is re-sent: consumed first, SAFE_OFF 13/13, nothing more enabled or sent.
    {Rig r;prone(r);r.p.dropFrameSlot=7;r.p.dropFrames=1;assert(!r.m.stand(.05,r.p.now));toEnable(r,7);r.p.stopOnFrame=true;
     r.tick(false);assert(r.m.stopping()&&r.p.sent.size()==2);for(unsigned i=0;i<15;++i)r.tick(false);
     assert(r.m.offVerified()&&r.m.state()==simple::State::STOPPED&&r.p.enables==7&&r.p.sent.size()==2&&!r.m.energized());}
    // Failure counters start from zero at every STAND: a persistent fault, STOP,
    // then one missed READ on the same servo is again recovered.
    {Rig r;prone(r);assert(!r.m.stand(.05,r.p.now));toEnable(r,5);r.p.missControlSlot=5;r.p.missControlReads=3;prepare(r);
     assert(r.m.state()==simple::State::FAULT);r.m.stop();for(unsigned i=0;i<15;++i)r.tick(false);assert(r.m.offVerified());
     for(unsigned i=0;i<12;++i)r.p.target[i]=0;const unsigned before=r.p.enables;
     assert(!r.m.stand(.05,r.p.now));r.p.missControlSlot=5;r.p.missControlReads=1;prepare(r);  // first READ of slot 5 in ACQUIRE
     assert(r.p.enables==before+12&&r.m.state()==simple::State::MOVING);assert(r.reachStand());}
    std::puts("PRONE 24/10 re-sent reference: byte-identical packet with 7 servos ON, no SAFE_OFF; slots 0/5/11 recover 1-2, fault on 3 before torque; STOP during re-send; counters reset per STAND");
  }
  std::puts("PASS software sequencing, reachable heights, gait continuity, native-target timeout, STOP, alarms, communication/deadline loss; NO HARDWARE MOTION VALIDATION");
}
