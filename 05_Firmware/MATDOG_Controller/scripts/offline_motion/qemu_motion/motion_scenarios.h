// Motion scenarios shared by the host build and the ESP32-S3 build run under
// Espressif QEMU (walk modes 2026-11-09, QEMU feasibility). The production
// SimpleMotion (one source tree, compiled unchanged) drives a scripted ServoPort
// with ideal tracking (optionally one release of feedback lag and loaded sag),
// through fixed scenarios, on a simulated clock: esp_timer_get_time() returns it
// (host: defined by the host main; ESP32-S3: --wrap=esp_timer_get_time while a
// scenario runs). Each scenario prints the transmitted frames, an FNV-1a digest
// of every RAW frame and drive, the final state/phase/reason, the first fault
// and the exit cause, and the per-release cost counter of the platform (QEMU:
// CCOUNT under -icount, an instruction count proxy, NOT ESP32-S3 time; host: 0).
// No hardware, no timing claim.
#pragma once
#include "SimpleMotion.h"
#include "PairedProneData.h"
#include <new>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <algorithm>
namespace qemu_motion {
using namespace matdog;
extern int64_t gClock;           // simulated microseconds
extern bool gSim;                // esp_timer_get_time() returns gClock
uint32_t cycles();               // platform cost counter (0 on the host)
extern void (*gProfile)(const char* scenario,uint32_t releases);  // QEMU profile build: per-scenario operation counts
struct Digest{uint64_t h=1469598103934665603ull;void add(uint32_t v,unsigned bytes){for(unsigned k=0;k<bytes;++k){h^=(v>>(8*k))&0xff;h*=1099511628211ull;}}};
struct Port:simple::ServoPort{
  simple::Feedback observed[12]{};uint16_t target[12]{},previous[12]{};simple::Drive drive{};
  unsigned frames=0,maxStep=0,lag=0;bool sag=false,failReads=false;Digest digest;
  void reset(){
    // The folded entry pose of the scripted host rig (test_simple_motion.cpp ScriptedObservationPort).
    *this=Port{};const double* p=motion::paired_data::kKnots[0].p;
    for(unsigned l=0;l<4;++l){const double q[3]={((l==0||l==3)?-1:1)*p[7],p[l<2?2:0],std::fmax((l<2?-88.5:-89.)*3.14159265358979323846/180,p[l<2?3:1])};
      for(unsigned j=0;j<3;++j){auto& f=observed[l*3+j];f.q=q[j];f.raw=2048+std::lround(q[j]/simple::kTickRad);f.torque=0;f.temp=30;f.voltage=120;f.valid=true;f.goal=f.raw;target[l*3+j]=previous[l*3+j]=f.raw;}}}
  bool read(unsigned i,bool control,simple::Feedback& out) override{
    if(failReads)return false;
    out=observed[i];out.stampUs=gClock;out.goal=target[i];out.goalSpeed=drive.speed;out.acc=drive.acc;if(control)out.controlStampUs=gClock;return true;}
  bool resolve(unsigned,double q,uint16_t& raw) override{raw=2048+std::lround(q/simple::kTickRad);return raw<4096;}
  bool decodeRaw(unsigned,uint16_t raw,double& q) override{q=(int(raw)-2048)*simple::kTickRad;return raw<4096;}
  bool frame(const uint16_t raw[12],simple::Drive d) override{++frames;drive=d;
    for(unsigned i=0;i<12;++i){digest.add(raw[i],2);maxStep=std::max(maxStep,unsigned(std::abs(int(raw[i])-int(target[i]))));target[i]=raw[i];}
    digest.add(d.speed,2);digest.add(d.acc,1);return true;}
  bool enable(unsigned i) override{observed[i].torque=1;return true;}
  bool off(unsigned first=0) override{for(unsigned i=first;i<12;++i)observed[i].torque=0;return true;}
  bool verifyOff(unsigned i) override{return i==12||observed[i].torque==0;}
  // Encoder snapshot of the commanded pose, `lag` releases late; with sag the
  // loaded joints sit a few ticks below it (dev.15 sag pattern), torque ON only.
  void publish(bool energized){
    static const int err[12]={-2,-1,-1,0,2,3,3,-4,-1,7,5,2};
    const uint16_t* src=lag?previous:target;
    for(unsigned i=0;i<12;++i){const int raw=int(src[i])-(sag&&energized?err[i]:0);observed[i].raw=uint16_t(raw);observed[i].q=(raw-2048)*simple::kTickRad;}
    std::copy(target,target+12,previous);}
};
struct Cost{uint32_t max=0,n=0;uint64_t sum=0;uint32_t hist[8]{};  // per-release counter, bins of 2^17
  void add(uint32_t c){max=std::max(max,c);sum+=c;++n;++hist[std::min<uint32_t>(7,c>>17)];}};
struct Sim{
  simple::SimpleMotion m;Port p;simple::Inputs in;Cost cost;const char* firstFault=nullptr;
  void begin(){p.reset();cost=Cost{};firstFault=nullptr;gClock=1000000;
    m.~SimpleMotion();new(&m) simple::SimpleMotion();m.begin(&p);
    // Register 85 verified at its target (dev.6): without it every STAND is refused.
    in=simple::Inputs{};in.ready=in.powerOk=in.authorityOk=in.imuValid=true;in.refusal="READY";in.accCeiling=simple::kAccCeilingTarget;
    const double lo[12]={-.783864,-.914253,-1.598408,-.783864,-.914253,-1.58,-.783864,-.914253,-1.592272,-.783864,-.914253,-1.590738};
    const double hi[12]={.783864,2.110758,.590583,.783864,2.10002,.581379,.771592,2.136835,.590583,.78233,2.129166,.585981};
    for(unsigned i=0;i<12;++i){in.minRad[i]=lo[i];in.maxRad[i]=hi[i];}
    for(unsigned i=0;i<15;++i)tick(false);}
  void tilt(double roll,double pitch){const double cr=std::cos(roll/2),sr=std::sin(roll/2),cp=std::cos(pitch/2),sp=std::sin(pitch/2);
    in.quaternion[0]=cr*cp;in.quaternion[1]=sr*cp;in.quaternion[2]=cr*sp;in.quaternion[3]=-sr*sp;}
  void tick(bool publish=true){gClock+=simple::kPeriodUs;in.imuStampUs=gClock;m.inputs(in);if(publish)p.publish(m.energized());
    const uint32_t c0=cycles();m.update(gClock);const uint32_t c1=cycles();cost.add(c1-c0);m.updateTelemetry(gClock);
    if(!firstFault&&m.state()==simple::State::FAULT)firstFault=m.reason();}
  bool until(simple::State s,unsigned n){for(unsigned k=0;k<n;++k){tick();if(m.state()==s)return true;if(m.state()==simple::State::FAULT)return false;}return false;}
};
inline void report(const char* name,Sim& s){
  const auto& e=s.m.exitSnapshot();
  std::printf("SCENARIO %-22s frames=%5u digest=%016llx max_step=%u state=%s/%s reason=%s first_fault=%s exit=%s/%s refused_by=%u releases=%lu cost_max=%lu cost_mean=%lu hist=%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu\n",
    name,s.p.frames,(unsigned long long)s.p.digest.h,s.p.maxStep,simple::name(s.m.state()),s.m.phase(),s.m.reason(),s.firstFault?s.firstFault:"-",
    e.valid?e.reason:"-",e.valid?e.phase:"-",unsigned(e.refusedBy),(unsigned long)s.cost.n,(unsigned long)s.cost.max,(unsigned long)(s.cost.n?s.cost.sum/s.cost.n:0),
    (unsigned long)s.cost.hist[0],(unsigned long)s.cost.hist[1],(unsigned long)s.cost.hist[2],(unsigned long)s.cost.hist[3],(unsigned long)s.cost.hist[4],(unsigned long)s.cost.hist[5],(unsigned long)s.cost.hist[6],(unsigned long)s.cost.hist[7]);
  if(gProfile)gProfile(name,s.cost.n);
}
// Every scenario starts from the folded entry, STAND 150 and (WALK) the latched mode.
inline void run(Sim& s){
  using simple::State;using simple::WalkMode;
  auto stand=[&](Sim& x){x.begin();x.m.stand(.15,gClock);return x.until(State::STANDING,3000);};
  auto walk=[&](Sim& x,WalkMode mode,unsigned stride){x.m.selectWalkMode(mode);if(x.m.walk(stride,gClock))return false;return x.until(State::WALKING,4000);};
  // 1-3: long walks with a small board tilt (IMU compensation active); WALK100
  // SLOW 6000 releases (120 s, 24 cycles) passes the cycle 19.5 of the dev.4 T2 defect.
  {stand(s);s.tilt(.005,.008);walk(s,WalkMode::SLOW,2);for(unsigned k=0;k<6000;++k)s.tick();report("WALK100_SLOW_120s",s);}
  {stand(s);s.tilt(.005,.008);walk(s,WalkMode::STANDARD,1);for(unsigned k=0;k<4500;++k)s.tick();report("WALK75_STANDARD_90s",s);}
  {stand(s);s.tilt(.005,.008);walk(s,WalkMode::FAST,2);for(unsigned k=0;k<4500;++k)s.tick();report("WALK80_FAST_90s",s);}
  // 4: joystick held sequence with continuous regime changes and a full reversal.
  {stand(s);const double T=simple::kJoystickPeriodS,pub=simple::kStrideM[2]/T,lat=simple::kJoystickLateralM/T,yaw=simple::kJoystickYawRad/T;
   const double seg[][3]={{pub,0,0},{.7071*pub,-.7071*lat,0},{0,-lat,0},{0,0,yaw},{-.5*simple::kJoystickStrideM/T,-.7*lat,0},{-.7*simple::kJoystickStrideM/T,-.5*lat,0},{pub,0,0}};
   const unsigned len[]={1500,1500,1500,1500,1500,2000,3000};
   for(unsigned g=0;g<7;++g)for(unsigned k=0;k<len[g];++k){if(k%12==0)s.m.drive(seg[g][0],seg[g][1],seg[g][2],gClock);s.tick();}
   report("JOYSTICK_HELD_SEQUENCE",s);}
  // 4b (Motion vNext 2026-11-11): on each mode's own law, a held steered sequence
  // (lateral, diagonal, yaw, backward-lateral) and a backward walk at full scale.
  for(unsigned mi=0;mi<3;++mi){const auto mode=static_cast<WalkMode>(mi);const auto& j=simple::joystickSpec(mode);const double T=simple::walkModeSpec(mode).periodS;
   const double lat=j.lateralM/T,yaw=j.yawRad/T,fore=j.strideM/T,back=j.backM/T;char n[48];
   {stand(s);s.tilt(.005,.008);s.m.selectWalkMode(mode);const double seg[][3]={{0,lat,0},{.7*fore,-.7*lat,0},{0,0,yaw},{-.6*fore,.6*lat,0}};
    for(unsigned g=0;g<4;++g)for(unsigned k=0;k<1500;++k){if(k%12==0)s.m.drive(seg[g][0],seg[g][1],seg[g][2],gClock);s.tick();}
    std::snprintf(n,sizeof n,"JOY_STEER_%s",simple::walkModeSpec(mode).name);report(n,s);}
   {stand(s);s.tilt(.005,.008);s.m.selectWalkMode(mode);for(unsigned k=0;k<3000;++k){if(k%12==0)s.m.drive(-back,0,0,gClock);s.tick();}
    std::snprintf(n,sizeof n,"JOY_BACK_%s",simple::walkModeSpec(mode).name);report(n,s);}
   // vNext audit: steered -> fore/aft handover, with the loaded-arrest gate (regimeHandover).
   {stand(s);s.tilt(.005,.008);s.m.selectWalkMode(mode);const double top=simple::walkModeSpec(mode).strideM[2]/T;
    for(unsigned k=0;k<1500;++k){if(k%12==0)s.m.drive(0,lat,0,gClock);s.tick();}
    for(unsigned k=0;k<1500;++k){if(k%12==0)s.m.drive(top,0,0,gClock);s.tick();}
    std::snprintf(n,sizeof n,"JOY_HANDOVER_%s",simple::walkModeSpec(mode).name);report(n,s);}}
  // 5: STOP at eight instants of a WALK100 SLOW cycle (swing and four-contact),
  // each verified, then the STAND recovery completes the interrupted WALK STOP.
  {unsigned ok=0;Sim& x=s;stand(x);walk(x,WalkMode::SLOW,2);for(unsigned k=0;k<300;++k)x.tick();
   for(unsigned j=0;j<8;++j){for(unsigned k=0;k<31;++k)x.tick();x.m.stop();for(unsigned k=0;k<400&&!x.m.stopVerified();++k)x.tick();
     if(!x.m.stopVerified()||x.m.walk(2,gClock)||!x.until(State::WALKING,4000))break;
   ++ok;}
   x.m.walkStop();x.until(State::STANDING,3000);char n[32];std::snprintf(n,sizeof n,"STOP_SWING_STANCE_%u_of_8",ok);report(n,x);}
  // 6: IMU demand beyond the WALK100 envelope (pitch-down 30 mrad): ENVELOPE, no accumulation, no fault.
  {stand(s);s.tilt(0,-.030);walk(s,WalkMode::SLOW,2);for(unsigned k=0;k<3000;++k)s.tick();report("IMU_BEYOND_ENVELOPE",s);}
  // 7: communication fault while walking: latched FAULT, SAFE_OFF verified, STAND
  // refused before and after the operator STOP (no automatic recovery exists).
  {stand(s);walk(s,WalkMode::SLOW,1);for(unsigned k=0;k<500;++k)s.tick();s.p.failReads=true;for(unsigned k=0;k<200;++k)s.tick();
   s.p.failReads=false;for(unsigned k=0;k<100;++k)s.tick();const char* a=s.m.stand(.15,gClock);s.m.stop();for(unsigned k=0;k<200;++k)s.tick();const char* b=s.m.stand(.15,gClock);
   char n[96];std::snprintf(n,sizeof n,"FAULT_LATCH_off%u_%s_%s",unsigned(s.m.offVerified()),a?a:"ACCEPTED",b?b:"ACCEPTED");report(n,s);}
  // 8: one release of feedback lag and loaded sag in WALK100 SLOW and WALK75 FAST.
  {stand(s);s.p.lag=1;s.p.sag=true;walk(s,WalkMode::SLOW,2);for(unsigned k=0;k<3000;++k)s.tick();report("LAG_SAG_WALK100_SLOW",s);}
  {stand(s);s.p.lag=1;s.p.sag=true;walk(s,WalkMode::FAST,2);for(unsigned k=0;k<3000;++k)s.tick();report("LAG_SAG_WALK80_FAST",s);}
}
}
