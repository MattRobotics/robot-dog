// Production SimpleMotion + ServoBus + installed SDK + production transforms.
// Only UART register physics/arrival time and Controller port binding are modeled.
#include <cassert>
#include <cmath>
#include <cstring>
#include "../../src/simple/SimpleMotion.h"
#include "../../src/servo/ServoBus.h"
#include "../../src/servo/ServoPacket.h"
#include "../../src/actuator/CalibrationTargetResolver.h"
#include "../../src/actuator/CalibrationGeometryProfileData.h"
using namespace matdog;
// The production Controller::SimpleServoPort::frame (SimpleMotionIntegration.cpp),
// extracted verbatim by run_preenable_bus_tests.sh, bound to this harness's
// SimpleMotion and real ServoBus: the frame path of the robot (trace 44).
namespace matdog { namespace core {
namespace { uint8_t slotId(unsigned i){const auto* s=actuator::slotServo(i);return s?s->bus_id:0;} }
struct Controller {
 simple::SimpleMotion& simple_motion_;servo::ServoBus& servo_bus_;
 struct SimpleServoPort {Controller* owner=nullptr;bool frame_preempted=false;bool frame(const uint16_t raw[12],simple::Drive drive);};
};
#include "production_simple_servo_port_frame.inc"
} }
// Exact ACTUAL snapshot: STAND50_LIVE_20261024_191332, 19:16:37.699893.
static const uint16_t rawPose[12]={1989,3077,3111,2023,1114,967,2065,1004,997,2023,3132,3130};
// Gen1 be8d5902bb4e37da, TOLERANCE_V1_20261018/gen1-restored.json.
static const uint16_t q0[12]={1983,2095,2102,2030,2096,1971,2087,2061,2034,2028,2092,2092};
struct Port : simple::ServoPort {
 servo::ServoBus bus;HardwareSerial* uart;simple::SimpleMotion* m=nullptr;
 actuator::CalibrationGeometryProfile geometry;actuator::JointTransform transforms[12];
 int missSlot=-1;int misses=0;unsigned faultKind=0;bool persistent=false,dead=false;unsigned deferred=0,failedTx=0,enables=0,events=0;
 simple::Feedback checked[12];unsigned postControlMask=0;
 unsigned eventSlot=99;simple::Feedback event;double ref=0;uint16_t refRaw=0;
 bool follow=false,stopRead=false,stopFrame=false,failOff=false;std::vector<std::vector<uint8_t>> frames;
 std::vector<simple::Drive> drives;std::vector<const char*> phases;bool preempted=false;unsigned refusedFrames=0;  // per transmitted frame / refused before TX
 uint16_t initialRaw[12]{};
 Port(const uint16_t* pose=rawPose){assert(bus.begin());uart=HardwareSerial::bus;uart->torque_on_goal=true;
  geometry.bind(&actuator::geometry_data::kProvenance,actuator::geometry_data::kJoints,actuator::geometry_data::kJointCount,actuator::geometry_data::kEndpoints,actuator::geometry_data::kEndpointCount);
  for(unsigned i=0;i<12;++i){auto& t=transforms[i];assert(actuator::slotIdentity(i,&t.identity));t.state=calibration::EvidenceState::PROMOTED;t.origin=calibration::CalibrationOrigin::LIVE_SESSION;t.geometry=geometry.provenanceTag();t.q0_tick=q0[i];t.present=true;
   initialRaw[i]=pose[i];auto& regs=uart->regs[id(i)];regs[56]=pose[i]&255;regs[57]=pose[i]>>8;regs[62]=112;regs[63]=28;}
 }
 static uint8_t id(unsigned i){return actuator::slotServo(i)->bus_id;}
 bool readDeferred() const {return bus.lastBusOutcome()==servo::BusOutcome::RX_PENDING;}
 bool read(unsigned i,bool control,simple::Feedback& f) override {
  if(stopRead&&control){m->stop();stopRead=false;}
  const bool inject=(control||dead)&&int(i)==missSlot&&(persistent||misses>0);
  if(inject){if(faultKind==0)uart->plan.silent=true;if(faultKind==1)uart->plan.truncate_to=9;if(faultKind==2)uart->plan.corrupt_checksum=true;}const auto count=uart->requests.size();bool ok;
  if(control){servo::ServoBus::ControlFeedbackSnapshot r;ok=bus.readControlFeedbackBudgeted(id(i),&r,2);
   if(ok){f.raw=r.present_position;f.torque=r.torque_enable;f.goal=r.goal_position;f.goalSpeed=r.goal_speed;f.acc=r.goal_acceleration;f.temp=r.present_temperature;f.voltage=r.present_voltage;f.status=r.status;f.speed=r.present_speed;}}
  else {servo::ServoBus::FastFeedback r;ok=bus.readFastFeedback(id(i),&r,2);if(ok){f.raw=r.present_position;f.temp=r.present_temperature;f.voltage=r.present_voltage;f.status=r.status;f.speed=r.present_speed;}}
  uart->plan=HardwareSerial::ReplyPlan{};
  if(control&&!ok){if(readDeferred())++deferred;else ++failedTx;
   if(inject&&uart->requests.size()>count&&misses>0)--misses;
   std::printf("PREP slot=%u id=%u phase=%s us=%llu tx=%zu outcome=%s\n",i,id(i),m->phase(),(unsigned long long)host_us,uart->requests.size()-count,servo::toString(bus.lastBusOutcome()));}
  if(!ok)return false;
  actuator::MicroRad q=0;assert(actuator::resolveRawObservation(geometry,actuator::geometry_data::kProvenance,transforms[i],f.raw,&q)==actuator::TargetResolveStatus::OK);
  f.q=actuator::microRadToRadians(q);f.valid=true;f.stampUs=host_us;if(control){f.controlStampUs=host_us;checked[i]=f;if(enables==12)postControlMask|=1u<<i;}return true;
 }
 // Same gen-1 decode as Controller::SimpleServoPort::decodeRaw. Without it the
 // base class refused every compensationSafe RAW check (RAW_CONVERSION as soon
 // as a WALK applied an IMU correction): a harness gap, not a Motion limit.
 bool decodeRaw(unsigned i,uint16_t raw,double& q) override{actuator::MicroRad v=0;
  if(i>=12||actuator::resolveRawObservation(geometry,actuator::geometry_data::kProvenance,transforms[i],raw,&v)!=actuator::TargetResolveStatus::OK)return false;q=actuator::microRadToRadians(v);return true;}
 bool resolve(unsigned i,double q,uint16_t& raw) override{return actuator::resolveRadiansToRaw(geometry,actuator::geometry_data::kProvenance,transforms[i],q,&raw)==actuator::TargetResolveStatus::OK;}
 bool frame(const uint16_t raw[12],simple::Drive d) override {
  core::Controller ctl{*m,bus};core::Controller::SimpleServoPort port{&ctl};const auto before=uart->requests.size();
  const bool ok=port.frame(raw,d);preempted=port.frame_preempted;
  if(ok){assert(uart->requests.size()==before+1);frames.push_back(uart->requests.back());drives.push_back(d);phases.push_back(m->phase());}
  else{assert(uart->requests.size()==before);if(!preempted)++refusedFrames;}  // nothing on the wire
  if(stopFrame){m->stop();stopFrame=false;}return ok;
 }
 bool framePreempted() const override {return preempted;}
 bool enable(unsigned i) override {assert(checked[i].valid&&checked[i].torque==0&&checked[i].goal==initialRaw[i]&&checked[i].goalSpeed==simple::kStandDrive.speed&&checked[i].acc==simple::kStandDrive.acc);assert(std::abs(int(checked[i].raw)-int(initialRaw[i]))<=12);bus.setValidationTimeout(2);bool ok=bus.enableTorqueOn(id(i))==servo::ServoWriteVerifyResult::VERIFIED_APPLIED;bus.setValidationTimeout(0);if(ok)++enables;return ok;}
 // Both signatures let this same harness compile against the preserved baseline API.
 bool off(){return off(0);}
 bool off(unsigned first){if(failOff){failOff=false;return false;}uint8_t ids[13];for(unsigned i=0;i<12;++i)ids[i]=id(i);ids[12]=51;return first<13&&bus.syncTorqueOff(ids+first,13-first)==8+2*(13-first);}
 bool verifyOff(unsigned i) override{return bus.verifyTorqueOff(i<12?id(i):51,2)==servo::SafeOffResult::VERIFIED_OFF;}
 void referenceFault(unsigned i,const simple::Feedback& f,double q,uint16_t raw) override {++events;eventSlot=i;event=f;ref=q;refRaw=raw;const auto& off=uart->requests.back();assert(off.size()==34&&off[4]==0x83&&off[5]==40&&off[6]==1);
  for(unsigned k=0;k<13;++k){assert(off[7+2*k]==(k<12?id(k):51)&&off[8+2*k]==0);if(int(off[7+2*k])!=uart->ignore_torque_off_id)assert(uart->regs[off[7+2*k]][40]==0);}}
};
struct Rig {
 simple::SimpleMotion m;Port p;simple::Inputs in;uint64_t release=1000000;unsigned maxUs=0;
 Rig(const uint16_t* pose=rawPose):p(pose){host_us=release;p.m=&m;busStop();m.begin(&p);in.ready=in.powerOk=in.authorityOk=in.imuValid=true;in.accCeiling=220;in.refusal="READY";
 const double lo[12]={-.783864,-.914253,-1.598408,-.783864,-.914253,-1.58,-.783864,-.914253,-1.592272,-.783864,-.914253,-1.590738};
 const double hi[12]={.783864,2.110758,.590583,.783864,2.10002,.581379,.771592,2.136835,.590583,.78233,2.129166,.585981};
 for(unsigned i=0;i<12;++i){in.minRad[i]=lo[i];in.maxRad[i]=hi[i];}for(unsigned i=0;i<30;++i)tick();assert(m.offVerified()&&m.fresh(host_us));}
 // The production hook (Controller::stopPreempt, without the HTTP lane): the
 // STOP's own authorized frames and reads are not preempted. stopping() here
 // preempted them too, so no energized arrest could ever be exercised.
 void busStop(){p.bus.setStopPreempt(+[](void* ctx){return static_cast<simple::SimpleMotion*>(ctx)->busPreemptRequested();},&m);}
 void tick(){release+=simple::kPeriodUs;assert(host_us<release);host_us=release;in.imuStampUs=host_us;m.inputs(in);
  if(p.follow)for(unsigned i=0;i<12;++i){auto& r=p.uart->regs[Port::id(i)];if(r[40]){r[56]=r[42];r[57]=r[43];}}
  m.update(release);maxUs=std::max(maxUs,unsigned(host_us-release));assert(host_us-release<simple::kPeriodUs);}
 void toEnable(unsigned slot){for(unsigned n=0;n<100&&!(std::strcmp(m.phase(),"ENABLE")==0&&p.enables==slot)&&m.state()!=simple::State::FAULT;++n)tick();assert(!strcmp(m.phase(),"ENABLE")&&p.enables==slot);}
 bool prepare(){for(unsigned n=0;n<200&&p.enables<12&&m.state()!=simple::State::FAULT;++n)tick();return p.enables==12;}
};
// One SYNC_WRITE goal frame as it left the UART: header, then per servo
// id, ACC, GoalPosition (LE), GoalTime 0, GoalSpeed (LE), checksum.
static bool goalFrameCarries(const std::vector<uint8_t>& p,simple::Drive d,const uint16_t* raw=nullptr){
 if(p.size()!=servo::packet::kGoalFrameBytes||p[0]!=0xFF||p[1]!=0xFF||p[2]!=servo::packet::kBroadcastId||p[3]!=p.size()-4||
    p[4]!=servo::packet::kInstSync||p[5]!=servo::packet::kAddrAcc||p[6]!=servo::packet::kGoalWidth)return false;
 for(unsigned i=0;i<12;++i){const uint8_t* e=&p[7+8*i];
  if(e[0]!=Port::id(i)||e[1]!=d.acc||e[4]||e[5]||(e[6]|e[7]<<8)!=d.speed||(raw&&(e[2]|e[3]<<8)!=raw[i]))return false;}
 unsigned sum=0;for(size_t k=2;k+1<p.size();++k)sum+=p[k];return p.back()==static_cast<uint8_t>(~sum);
}
// Trace 44 (dev.6 70abcff, 09/11/2026): the walk-mode RAM pairs reach the UART
// through the production SimpleServoPort::frame and the real ServoBus, the
// gate still refuses any other pair before TX, and the robot's sequence
// STAND -> WALK SLOW -> WALK STOP -> WALK STANDARD walks. Ideal register
// tracking: the gate and the bytes are verified, not the servos' physics
// (ACC 180/220 have never been measured).
static unsigned walkModeDrives(){
 unsigned failures=0;auto check=[&](bool ok,const char* what){std::printf("WALK_MODE_DRIVE %s %s\n",ok?"PASS":"FAIL",what);failures+=!ok;return ok;};
 // 1. The gate admits exactly the Controller's table (two lists kept equal here).
 {servo::ServoBus bus;assert(bus.begin());char what[96];
  std::snprintf(what,sizeof what,"STAND {%u,%u} admitted",unsigned(simple::kStandDrive.speed),unsigned(simple::kStandDrive.acc));check(bus.setOperationalDrive(simple::kStandDrive.speed,simple::kStandDrive.acc),what);
  for(unsigned m=0;m<3;++m){const auto& w=simple::walkModeSpec(static_cast<simple::WalkMode>(m));
   std::snprintf(what,sizeof what,"%s {%u,%u} admitted",w.name,unsigned(w.drive.speed),unsigned(w.drive.acc));check(bus.setOperationalDrive(w.drive.speed,w.drive.acc),what);}}
 // 2. Any other pair is refused, and nothing is transmitted.
 {Rig r;const simple::Drive other[]={{1800,181},{1801,180},{1950,219},{1949,220},{1000,51},{480,12},{1200,100},{300,20},{160,8},{3400,254},{0,0}};
  uint16_t raw[12];for(unsigned i=0;i<12;++i)raw[i]=rawPose[i];bool refused=true;
  for(const auto& d:other){const size_t before=r.p.uart->requests.size();const unsigned refusals=r.p.refusedFrames;
   refused=refused&&!r.p.frame(raw,d)&&!r.p.framePreempted()&&r.p.refusedFrames==refusals+1&&r.p.uart->requests.size()==before;}
  check(refused,"11 unlisted pairs refused by the production frame path, zero bytes on the UART");}
 // 3. Each admitted pair is the one in the transmitted packet, with the 12 targets.
 for(unsigned m=0;m<4;++m){Rig r;const simple::Drive d=m<3?simple::walkModeSpec(static_cast<simple::WalkMode>(m)).drive:simple::kStandDrive;
  uint16_t raw[12];for(unsigned i=0;i<12;++i)raw[i]=uint16_t(rawPose[i]+7*i);const auto before=r.p.uart->requests.size();
  const bool sent=r.p.frame(raw,d);char what[96];std::snprintf(what,sizeof what,"SYNC_WRITE {%u,%u}: 104 B, 12 ids, ACC, GoalSpeed, 12 targets, checksum",unsigned(d.speed),unsigned(d.acc));
  check(sent&&r.p.uart->requests.size()==before+1&&goalFrameCarries(r.p.uart->requests.back(),d,raw),what);}
 // 4. The robot's sequence, every frame decoded from the UART.
 {Rig r;assert(!r.m.stand(.15,host_us));assert(r.prepare());r.p.follow=true;
  for(unsigned n=0;n<3000&&r.m.state()!=simple::State::STANDING&&r.m.state()!=simple::State::FAULT;++n)r.tick();
  assert(r.m.state()==simple::State::STANDING);
  auto framesSince=[&](size_t from,simple::Drive d,bool walking){unsigned n=0;bool ok=true;
   for(size_t k=from;k<r.p.frames.size();++k){const bool w=!strcmp(r.p.phases[k],"WALK_STANDARD");if(w!=walking)continue;++n;
    ok=ok&&r.p.drives[k].speed==d.speed&&r.p.drives[k].acc==d.acc&&goalFrameCarries(r.p.frames[k],d);}return ok&&n>0;};
  check(framesSince(0,simple::kStandDrive,false),"STAND 150: every frame {480,13}");
  auto walk=[&](simple::WalkMode mode,unsigned preset,unsigned releases,bool stopAtEnd){const auto& w=simple::walkModeSpec(mode);const size_t from=r.p.frames.size();
   r.m.selectWalkMode(mode);const char* why=r.m.walk(preset,host_us);
   for(unsigned n=0;!why&&n<4000&&r.m.state()!=simple::State::WALKING&&r.m.state()!=simple::State::FAULT;++n)r.tick();
   for(unsigned n=0;!why&&n<releases&&r.m.state()==simple::State::WALKING;++n)r.tick();
   char what[160];std::snprintf(what,sizeof what,"WALK %s preset %u: %s %s, %.2f cycles, mode %s, refused frames %u",w.name,preset,simple::name(r.m.state()),r.m.reason(),r.m.walkCycles(),
     simple::walkModeSpec(r.m.walkMode()).name,r.p.refusedFrames);
   check(!why&&r.m.state()==simple::State::WALKING&&r.m.walkMode()==mode&&r.m.walkCycles()>2&&r.p.refusedFrames==0,what);
   std::snprintf(what,sizeof what,"WALK %s: every walk frame {%u,%u} on the UART",w.name,unsigned(w.drive.speed),unsigned(w.drive.acc));check(framesSince(from,w.drive,true),what);
   if(stopAtEnd){r.m.walkStop();for(unsigned n=0;n<8000&&r.m.state()!=simple::State::STANDING&&r.m.state()!=simple::State::FAULT;++n)r.tick();
    std::snprintf(what,sizeof what,"WALK %s -> WALK STOP -> %s %s",w.name,simple::name(r.m.state()),r.m.reason());check(r.m.state()==simple::State::STANDING,what);}};
  // A SLOW walk with FAST, then STANDARD selected 6 s apart during it (2026-11-10):
  // the walk moves to STANDARD without leaving WALK. On the UART every walk
  // frame is a SYNC_WRITE of a table drive (its ACC in register 41) and nothing
  // else is written: no register 85, no TorqueEnable (trace 44, trace 45).
  {const size_t from=r.p.frames.size();r.m.selectWalkMode(simple::WalkMode::SLOW);const char* why=r.m.walk(2,host_us);
   for(unsigned n=0;!why&&n<4000&&r.m.state()!=simple::State::WALKING&&r.m.state()!=simple::State::FAULT;++n)r.tick();
   const size_t req=r.p.uart->requests.size();bool walking=r.m.state()==simple::State::WALKING;
   for(unsigned n=0;!why&&n<1650&&walking;++n){r.tick();walking=r.m.state()==simple::State::WALKING;
    if(n==600)r.m.selectWalkMode(simple::WalkMode::FAST);if(n==900)r.m.selectWalkMode(simple::WalkMode::STANDARD);}
   unsigned writes=0;for(size_t k=req;k<r.p.uart->requests.size();++k)writes+=r.p.uart->requests[k][4]==3;
   bool table=true;unsigned walkFrames=0;
   for(size_t k=from;k<r.p.frames.size();++k){if(strcmp(r.p.phases[k],"WALK_STANDARD"))continue;++walkFrames;bool in=false;
    for(unsigned m=0;m<3;++m){const auto& d=simple::walkModeSpec(static_cast<simple::WalkMode>(m)).drive;in=in||(r.p.drives[k].speed==d.speed&&r.p.drives[k].acc==d.acc&&goalFrameCarries(r.p.frames[k],d));}
    table=table&&in;}
   const auto& std_=simple::walkModeSpec(simple::WalkMode::STANDARD).drive;char what[220];
   std::snprintf(what,sizeof what,"WALK SLOW, FAST then STANDARD selected during it: %s %s, law %s, drive {%u,%u}, %u walk frames on table drives, %u WRITE instructions on the UART",
     simple::name(r.m.state()),r.m.reason(),simple::walkModeSpec(r.m.walkMode()).name,unsigned(r.p.drives.back().speed),unsigned(r.p.drives.back().acc),walkFrames,writes);
   check(!why&&walking&&r.m.walkMode()==simple::WalkMode::STANDARD&&r.m.walkTarget()==simple::WalkMode::STANDARD&&r.p.drives.back().speed==std_.speed&&
         r.p.drives.back().acc==std_.acc&&table&&walkFrames>0&&writes==0&&r.p.refusedFrames==0,what);
   r.m.walkStop();for(unsigned n=0;n<8000&&r.m.state()!=simple::State::STANDING&&r.m.state()!=simple::State::FAULT;++n)r.tick();
   check(r.m.state()==simple::State::STANDING,"WALK STOP after the mode change -> STANDING");}
  check(r.m.selectedWalkMode()==simple::WalkMode::STANDARD,"the selection stays for the next walk");
  walk(simple::WalkMode::STANDARD,1,1000,false);  // trace 44: faulted SYNC_WRITE_FAILED at cycle 0.000 on 70abcff
  if(r.m.state()!=simple::State::WALKING)check(false,"STOP, resume, FAST and SLOW-again steps not run: STANDARD is not walking");
  else{{const size_t from=r.p.frames.size();r.m.stop();for(unsigned n=0;n<400&&!r.m.stopVerified()&&r.m.state()!=simple::State::FAULT;++n)r.tick();
   const auto& std_=simple::walkModeSpec(simple::WalkMode::STANDARD).drive;bool stopFrames=r.p.frames.size()>from;
   for(size_t k=from;k<r.p.frames.size();++k)stopFrames=stopFrames&&r.p.drives[k].speed==std_.speed&&r.p.drives[k].acc==std_.acc&&goalFrameCarries(r.p.frames[k],std_);
   std::printf("  STOP during STANDARD: state=%s reason=%s verified=%d energized=%d arrest_frames=%zu\n",simple::name(r.m.state()),r.m.reason(),int(r.m.stopVerified()),int(r.m.energized()),r.p.frames.size()-from);
   check(r.m.stopVerified()&&r.m.energized()&&stopFrames,"STOP during STANDARD: arrest frames carry {1800,180}, STOP verified, torque ON");
   const auto r0=r.p.frames.size();assert(!r.m.walk(1,host_us));
   for(unsigned n=0;n<4000&&r.m.state()!=simple::State::WALKING&&r.m.state()!=simple::State::FAULT;++n)r.tick();for(unsigned n=0;n<300&&r.m.state()==simple::State::WALKING;++n)r.tick();
   check(r.m.state()==simple::State::WALKING&&r.m.walkMode()==simple::WalkMode::STANDARD&&r.p.frames.size()>r0,"WALK after STOP resumes STANDARD");
   r.m.walkStop();for(unsigned n=0;n<8000&&r.m.state()!=simple::State::STANDING&&r.m.state()!=simple::State::FAULT;++n)r.tick();check(r.m.state()==simple::State::STANDING,"STANDARD -> WALK STOP -> STANDING");}
  walk(simple::WalkMode::FAST,2,1000,true);
  {const auto stand=r.p.frames.size();r.m.selectWalkMode(simple::WalkMode::SLOW);assert(!r.m.walk(2,host_us));
   for(unsigned n=0;n<4000&&r.m.state()!=simple::State::WALKING&&r.m.state()!=simple::State::FAULT;++n)r.tick();for(unsigned n=0;n<200&&r.m.state()==simple::State::WALKING;++n)r.tick();
   check(framesSince(stand,simple::kStandDrive,false)&&framesSince(stand,simple::kWalkDrive,true)&&r.m.walkMode()==simple::WalkMode::SLOW,"back to SLOW after FAST: HEIGHT {480,13}, walk {1000,50}");}}
  r.m.safeOff();for(unsigned n=0;n<60;++n)r.tick();bool off=r.m.offVerified()&&!r.m.energized();for(unsigned i=0;i<13;++i)off=off&&r.p.uart->regs[i<12?Port::id(i):51][40]==0;
  check(off,"SAFE_OFF from WALK: 13/13 torque OFF verified");}
 std::printf("WALK_MODE_DRIVES %s failures=%u hardware=false (gate, bytes and sequence; ACC 180/220 physics not qualified)\n",failures?"FAIL":"PASS",failures);
 return failures;
}
int main(){
 std::setvbuf(stdout,nullptr,_IONBF,0);
 // Decisive case first: model the hardware-observed side effect, not just RAM.
 {Rig r;assert(!r.m.stand(.05,host_us));r.toEnable(0);
  assert(r.p.enables==0&&r.p.frames.size()==1);
  r.tick();
  if(r.m.state()==simple::State::FAULT){
   std::printf("DECISIVE FAIL phase=ENABLE slot=%u id=%u explicit_enables=%u torque=%d raw=%u ref_raw=%u delta_ticks=%.3f goal=%d speed=%d acc=%d reason=%s\n",r.p.eventSlot,Port::id(r.p.eventSlot),r.p.enables,r.p.event.torque,r.p.event.raw,r.p.refRaw,(r.p.event.q-r.p.ref)/simple::kTickRad,r.p.event.goal,r.p.event.goalSpeed,r.p.event.acc,r.m.reason());return 1;
  }
  for(unsigned i=0;i<12;++i){
   if(i)r.tick();assert(r.p.enables==i+1);
   simple::Feedback f;assert(r.p.read(i,true,f)&&f.valid&&f.torque==1&&f.raw==rawPose[i]);
   if(i<11)for(unsigned j=i+1;j<12;++j)assert(r.p.uart->regs[Port::id(j)][40]==0);
   if(i==0||i==5||i==11)std::printf("DECISIVE slot=%u id=%u OFF->enable->validated_ON raw=%u next_enable_count=%u\n",i,Port::id(i),f.raw,r.p.enables);
  }
  assert(r.p.events==0);r.p.follow=true;
  for(unsigned n=0;n<1500&&r.m.state()!=simple::State::STANDING&&r.m.state()!=simple::State::FAULT;++n)r.tick();
  assert(r.m.state()==simple::State::STANDING);
  r.p.postControlMask=0;for(unsigned n=0;n<80;++n)r.tick();assert(r.p.postControlMask==0xfff&&r.m.state()==simple::State::STANDING);
  std::puts("DECISIVE PASS STAND50 + post-enable control rotation 12/12; hardware=false");
 }
 // A transient implicit torque-ON / lost OFF never authorizes ON directly.
 // The only recovery TX is suffix OFF; the next independent control read must
 // show OFF with the same exact reference before the existing enable checks.
 for(unsigned slot:{0u,5u,11u}){Rig r;assert(!r.m.stand(.05,host_us));r.toEnable(slot);
  r.p.uart->regs[Port::id(slot)][40]=1;const auto frames=r.p.frames.size();r.tick();
  assert(r.p.enables==slot&&r.p.events==0&&r.p.frames.size()==frames&&r.p.uart->regs[Port::id(slot)][40]==0);
  r.tick();assert(r.p.enables==slot+1&&r.p.checked[slot].torque==0);assert(r.prepare());
  std::printf("REFERENCE_OFF_RETRY slot=%u OFF_verified_before_enable no_goal_resend\n",slot);}
 // PRIME must be followed by OFF on the wire, then independent readback.
 // A lost OFF cannot be treated as permission to accept ON before enable.
 {Rig r;assert(!r.m.stand(.05,host_us));r.p.uart->ignore_torque_off_id=13;r.toEnable(0);r.tick();assert(r.p.enables==0&&r.p.events==0);r.tick();assert(r.p.enables==0&&r.p.events==0);r.tick();
  assert(r.m.state()==simple::State::FAULT&&r.p.enables==0&&r.p.events==1&&r.p.event.torque==1);
  r.p.uart->ignore_torque_off_id=-1;for(unsigned n=0;n<40;++n)r.tick();assert(r.m.offVerified());
  std::puts("PRIME lost_OFF rejected; SAFE_OFF=13/13");}
 for(bool stop : {false,true}){Rig r;assert(!r.m.stand(.05,host_us));while(strcmp(r.m.phase(),"REFERENCE"))r.tick();
  if(stop)r.p.stopFrame=true;else r.p.failOff=true;r.tick();
  assert(r.p.enables==0&&r.m.state()==(stop?simple::State::STOPPED:simple::State::FAULT));
  for(unsigned n=0;n<40;++n)r.tick();assert(r.m.offVerified()&&!r.m.energized());
  std::printf("PRIME %s SAFE_OFF=13/13 no_enable\n",stop?"STOP_during_frame":"OFF_TX_uncertain");}
 unsigned failures=0;
 for(unsigned kind=0;kind<3;++kind)for(bool enable : {false,true})for(int slot : {0,5,6}){
  Rig r;assert(!r.m.stand(.05,host_us));if(enable)r.toEnable(slot);r.p.missSlot=slot;r.p.misses=1;r.p.faultKind=kind;
  const bool ok=r.prepare();std::printf("INTEGRATED recovery kind=%u phase=%s slot=%d reached=%u deferred=%u failed_tx=%u reason=%s\n",kind,enable?"ENABLE":"ACQUIRE",slot,r.p.enables,r.p.deferred,r.p.failedTx,r.m.reason());
  if(!ok){++failures;continue;}assert(r.p.deferred>0&&r.p.failedTx==1&&r.p.events==0&&r.m.missed()==0);
  r.p.follow=true;for(unsigned n=0;n<1500&&r.m.state()!=simple::State::STANDING&&r.m.state()!=simple::State::FAULT;++n)r.tick();assert(r.m.state()==simple::State::STANDING); // ideal register tracking, NOT hardware qualification
  std::printf("STAND50 reached; max_virtual_release_us=%u\n",r.maxUs);
 }
 // No communication loss, and two genuinely transmitted failures followed
 // by a valid response: both must complete the production STAND50 route.
 for(int lost : {0,2})for(bool enable : {false,true}) {
  Rig r;assert(!r.m.stand(.05,host_us));if(enable)r.toEnable(5);r.p.missSlot=5;r.p.misses=lost;
  if(!r.prepare()){++failures;std::printf("RECOVERY_%d premature abort\n",lost);continue;}
  assert(r.p.failedTx==unsigned(lost)&&r.p.events==0);r.p.follow=true;
  for(unsigned n=0;n<1500&&r.m.state()!=simple::State::STANDING&&r.m.state()!=simple::State::FAULT;++n)r.tick();
  assert(r.m.state()==simple::State::STANDING&&r.m.missed()==0);
  std::printf("STAND50 actual_failed_tx=%u completed max_virtual_release_us=%u\n",r.p.failedTx,r.maxUs);
 }
 // A pending control transaction can predate the STAND command itself.
 {Rig r;r.p.missSlot=0;r.p.misses=1;simple::Feedback f;assert(!r.p.read(0,true,f));assert(!r.m.stand(.05,host_us));
  if(!r.prepare()){++failures;std::puts("PREEXISTING_PENDING premature abort");}else assert(r.p.deferred>0&&r.p.failedTx==1&&r.p.events==0);}
 // Entire servo absent (fast AND control), not merely a lost control reply.
 {Rig r;assert(!r.m.stand(.05,host_us));r.p.missSlot=0;r.p.persistent=r.p.dead=true;const auto start=host_us;
  assert(!r.prepare()&&r.m.state()==simple::State::FAULT&&r.p.enables==0&&host_us-start<1000000);
  std::printf("ABSENT_SERVO finite_fault_us=%llu reason=%s\n",(unsigned long long)(host_us-start),r.m.reason());}
 // Width10 succeeds but width15 never responds, including background fast
 // polls: a deferred second block must not make preparation unbounded.
 for(bool enable : {false,true}){Rig r;assert(!r.m.stand(.05,host_us));if(enable)r.toEnable(5);
  r.p.uart->silent_read_id=Port::id(5);r.p.uart->silent_read_width=15;r.p.uart->silent_reads=-1;const auto start=host_us;
  assert(!r.prepare()&&r.m.state()==simple::State::FAULT&&host_us-start<1000000);
  std::printf("SECOND_BLOCK_ABSENT finite_fault_us=%llu reason=%s\n",(unsigned long long)(host_us-start),r.m.reason());}
 // Re-send the actual production SyncWrite after seven servos are enabled.
 {Rig r;assert(!r.m.stand(.05,host_us));r.toEnable(7);auto& regs=r.p.uart->regs[Port::id(7)];regs[42]^=1;
  r.tick();assert(r.p.frames.size()==2&&r.p.frames[0]==r.p.frames[1]&&r.p.enables==7);
  for(unsigned i=0;i<7;++i)assert(r.p.uart->regs[Port::id(i)][40]==1);
  assert(r.prepare()&&r.p.events==0);}
 // STOP while recovery is deferred, then SAFE_OFF of all 13.
 {Rig r;assert(!r.m.stand(.05,host_us));r.p.missSlot=0;r.p.misses=1;r.tick();r.tick();assert(r.p.deferred>0);
  r.m.stop();for(unsigned i=0;i<30;++i)r.tick();assert(r.m.offVerified()&&r.p.enables==0);}
 for(bool enable : {false,true}){Rig r;assert(!r.m.stand(.05,host_us));if(enable)r.toEnable(5);r.p.missSlot=5;r.p.persistent=true;const auto start=host_us;
  assert(!r.prepare());if(r.p.failedTx!=3){++failures;std::printf("PREMATURE_FAULT failed_tx=%u deferred=%u\n",r.p.failedTx,r.p.deferred);continue;}assert(r.m.state()==simple::State::FAULT&&host_us-start<1000000);assert(strcmp(r.m.reason(),enable?"SERVO_CONTROL_READ_LOST":"INITIAL_SERVO_READ_FAILED")==0);
  std::printf("PERSISTENT finite_fault_us=%llu transmitted_failures=%u deferred=%u\n",(unsigned long long)(host_us-start),r.p.failedTx,r.p.deferred);
  // The fault is latched (c2d95a8, 2026-11-02): STOP keeps SAFE_OFF verified
  // and STAND is refused with FAULT_STOP_REQUIRED, nothing is enabled again
  // (recovery is a power cycle). Before c2d95a8 this case re-enabled 12 servos.
  r.p.persistent=false;r.m.stop();for(unsigned i=0;i<30;++i)r.tick();assert(r.m.offVerified());const auto enabled=r.p.enables;
  const char* again=r.m.stand(.05,host_us);assert(again&&!strcmp(again,"FAULT_STOP_REQUIRED"));for(unsigned n=0;n<100;++n)r.tick();
  assert(r.p.enables==enabled&&r.m.offVerified()&&!r.m.energized()&&r.m.state()==simple::State::FAULT);
 }
 for(unsigned slot : {0u,5u,11u})for(bool torque : {false,true}){Rig r;assert(!r.m.stand(.05,host_us));r.toEnable(slot);auto& regs=r.p.uart->regs[Port::id(slot)];if(torque){regs[40]=1;r.p.uart->ignore_torque_off_id=Port::id(slot);}else{const auto moved=rawPose[slot]+13;regs[56]=moved&255;regs[57]=moved>>8;}r.tick();if(torque){assert(r.p.enables==slot&&r.p.events==0);r.tick();assert(r.p.enables==slot&&r.p.events==0);r.tick();}assert(r.m.state()==simple::State::FAULT&&r.p.enables==slot&&r.p.events==1&&r.p.eventSlot==slot&&r.p.refRaw==rawPose[slot]);assert(r.p.event.torque==(torque?1:0));assert((std::abs(r.p.event.q-r.p.ref)>12*simple::kTickRad)==!torque);r.p.uart->ignore_torque_off_id=-1;for(unsigned n=0;n<30;++n)r.tick();assert(r.m.offMask()==0x1fff&&!r.m.energized());for(unsigned i=0;i<13;++i)assert(r.p.uart->regs[i<12?Port::id(i):51][40]==0);std::printf("PROTECTION slot=%u fault=%s SAFE_OFF=13/13\n",slot,torque?"unexpected_ON":"13_ticks");}
 for(unsigned phase=0;phase<3;++phase){Rig r;assert(!r.m.stand(.05,host_us));if(phase==1){while(strcmp(r.m.phase(),"REFERENCE"))r.tick();}if(phase==2)r.toEnable(5);auto count=r.p.enables;r.p.uart->regs[51][40]=1;r.m.stop();for(unsigned i=0;i<30;++i)r.tick();assert(r.m.offVerified()&&r.p.enables==count&&!r.m.energized());for(unsigned i=0;i<13;++i)assert(r.p.uart->regs[i<12?Port::id(i):51][40]==0);for(unsigned n=0;n<60;++n)r.tick();assert(r.m.state()==simple::State::STOPPED&&r.p.enables==count);std::printf("STOP phase=%u SAFE_OFF=13/13 no_auto_restart\n",phase);}
 failures+=walkModeDrives();
 std::printf("PREENABLE_INTEGRATED %s failures=%u hardware=false\n",failures?"FAIL":"PASS",failures);return failures?1:0;
}
