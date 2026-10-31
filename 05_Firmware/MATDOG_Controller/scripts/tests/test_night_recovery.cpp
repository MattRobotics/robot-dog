// Reuse the production-port harness; only servo physics/arrival is modeled.
#define main existing_preenable_suite
#include "test_preenable_bus.cpp"
#undef main
#include "../../src/power/DalyBms.h"
#include "../../src/power/MotionPowerPolicy.h"
#include "night_pose_fixture.h"
int main(){
 unsigned count=0;
 for(const auto& pose:nightPoses){Rig r(pose);assert(!r.m.stand(.05,host_us));assert(r.prepare());r.p.follow=true;
  for(unsigned n=0;n<1800&&r.m.state()!=simple::State::STANDING&&r.m.state()!=simple::State::FAULT;++n)r.tick();
  std::printf("CAPTURE pose=%u state=%s reason=%s\n",count++,simple::name(r.m.state()),r.m.reason());assert(r.m.state()==simple::State::STANDING);}
 // Cooperative scheduler: actual Motion + actual DALY, no Motion-slack input.
 // CPU work is an explicit 6 ms synthetic stimulus on each release, not an
 // ESP32 execution-time measurement. UART waits use the real bus budgets.
 {Rig r;power::DalyBms b;assert(b.begin());auto* u=HardwareSerial::bms;
  std::vector<uint8_t> reply(power::kDalyTelemetryResponseLen);reply[0]=0xd2;reply[1]=3;reply[2]=124;
  auto reg=[&](unsigned i,unsigned v){reply[3+2*i]=v>>8;reply[4+2*i]=v&255;};reg(54,1);reg(40,120);
  auto crc=[&](){auto c=power::crc16Modbus(reply.data(),reply.size()-2);reply[reply.size()-2]=c&255;reply.back()=c>>8;};crc();
  uint64_t responseAt=0;size_t sent=0;uint32_t replies=0;uint64_t maxAge=0;
  auto pass=[&](){
   const uint32_t before=r.m.releases();r.in.imuStampUs=host_us;r.in.powerReason="READY";r.in.powerOk=true;
   r.m.inputs(r.in);r.m.update(host_us);
   if(r.m.releases()!=before)host_us+=6000; // prescribed CPU load
   b.update(host_us/1000,core::OperatingMode::RUN);
   if(u->requests.size()!=sent){sent=u->requests.size();responseAt=host_us+140000;}
   if(responseAt&&host_us>=responseAt){for(auto c:reply)u->rx.push_back({host_us,c});responseAt=0;++replies;}
   if(r.p.follow)for(unsigned i=0;i<12;++i){auto& q=r.p.uart->regs[Port::id(i)];if(q[40]){q[56]=q[42];q[57]=q[43];}}
   if(r.p.enables==12)for(unsigned i=0;i<12;++i){const auto& f=r.m.feedback()[i];if(f.valid)maxAge=std::max(maxAge,host_us-f.stampUs);}
   host_us+=1000; // loopTask yield/granularity
  };
  while(!b.sample().valid)pass();assert(!r.m.stand(.05,host_us));
  while(r.p.enables<12&&r.m.state()!=simple::State::FAULT)pass();assert(r.p.enables==12);r.p.follow=true;
  const auto end=host_us+30000000;
  while(host_us<end){pass();assert(power::motionPower(b.sample(),host_us/1000,true)==power::MotionPower::OK);assert(r.m.state()!=simple::State::FAULT);}
  assert(r.m.state()==simple::State::STANDING&&replies>=14&&r.m.missed()==0&&r.m.maxGapUs()<22000&&maxAge<250000);
  std::printf("COOPERATIVE PASS synthetic_cpu_us=6000 max_gap_us=%llu missed=%u max_feedback_age_us=%llu bms_replies=%u hardware=false\n",(unsigned long long)r.m.maxGapUs(),r.m.missed(),(unsigned long long)maxAge,replies);
  // Alarm stops immediately, before a future Motion release, through the
  // production supervisor policy; the SAFE_OFF authority remains Motion.
  auto s=b.sample();s.alarms[0]=1;r.in.powerOk=power::motionPower(s,host_us/1000,true)==power::MotionPower::OK;r.in.powerReason=power::name(power::motionPower(s,host_us/1000,true));r.m.inputs(r.in);
  r.m.update(host_us);assert(r.m.state()==simple::State::FAULT&&!strcmp(r.m.reason(),"BMS_ALARM"));
  for(unsigned i=0;i<30;++i){host_us+=1000;r.m.update(host_us);}assert(r.m.offVerified());
 }
 // A successful block40 followed by deferred block56 is not a lost
 // control transaction. Three such deferrals must not fault at 60 ms.
 {Rig r;assert(!r.m.stand(.05,host_us));r.toEnable(5);
  r.p.uart->plan.silent=true;servo::ServoBus::FastFeedback f;
  assert(!r.p.bus.readFastFeedback(Port::id(5),&f,2));r.p.uart->plan={};
  const auto before=r.p.failedTx;for(unsigned n=0;n<3;++n){r.tick();assert(r.m.state()!=simple::State::FAULT);}
  assert(r.p.failedTx==before&&r.p.deferred>=3);assert(r.prepare());
  std::puts("CONTROL_SECOND_BLOCK_DEFERRED PASS answered_first_block not_counted_as_lost_reply");}
 // Persistent feedback loss during ENABLE must observe the same250 ms gate.
 {Rig r;assert(!r.m.stand(.05,host_us));r.toEnable(5);
  r.p.uart->silent_read_id=Port::id(5);r.p.uart->silent_read_width=15;r.p.uart->silent_reads=-1;
  const auto start=host_us;while(r.m.state()!=simple::State::FAULT&&host_us-start<1000000)r.tick();
  assert(r.m.state()==simple::State::FAULT&&r.p.enables==5&&host_us-start<300000);
  assert(!strcmp(r.m.reason(),"SERVO_FEEDBACK_STALE"));
  std::puts("ENABLE_PERSISTENT_FEEDBACK PASS stale_gate250ms before_further_enable");}
 // A single lost fast reply recovers on the next ordinary 120 ms rotation.
 {Rig r;assert(!r.m.stand(.05,host_us));assert(r.prepare());r.p.follow=true;for(unsigned n=0;n<1200&&r.m.state()!=simple::State::STANDING;++n)r.tick();assert(r.m.state()==simple::State::STANDING);
  r.p.uart->silent_read_id=Port::id(5);r.p.uart->silent_read_width=15;r.p.uart->silent_reads=1;
  for(unsigned n=0;n<100;++n){r.tick();assert(r.m.state()==simple::State::STANDING&&r.m.fresh(host_us));}
  // Persistent lack of new data must still fault at 250 ms, not be hidden by
  // RX_PENDING deferrals or an extended freshness limit.
  r.p.uart->silent_reads=-1;auto start=host_us;for(unsigned n=0;n<40&&r.m.state()!=simple::State::FAULT;++n)r.tick();
  assert(r.m.state()==simple::State::FAULT&&host_us-start<400000);assert(!strcmp(r.m.reason(),"SERVO_FEEDBACK_STALE")||!strcmp(r.m.reason(),"SERVO_COMMUNICATION_LOST"));
 }
 std::puts("NIGHT_RECOVERY PASS hardware captured poses13/13, cooperative deadlines, live monitoring/alarm stop, transient/persistent feedback; hardware=false");
}
