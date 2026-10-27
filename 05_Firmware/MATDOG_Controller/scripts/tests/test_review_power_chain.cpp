// Review regression: continuously wire actual DALY sample -> actual power
// policy -> actual Motion. UART and mechanics modeled; Controller's tiny
// assignment boundary is mirrored, not claimed as full Controller execution.
#define main existing_preenable_suite
#include "test_preenable_bus.cpp"
#undef main
#include "../../src/power/DalyBms.h"
#include "../../src/power/MotionPowerPolicy.h"
struct PowerRig {
 Rig r;power::DalyBms b;HardwareSerial* u;
 std::vector<uint8_t> response=std::vector<uint8_t>(power::kDalyTelemetryResponseLen);
 size_t requests=0;uint64_t due=0;bool dropNext=false;uint32_t replyDelay=140000;
 bool running=true;unsigned policyChecks=0;
 PowerRig(){assert(b.begin());u=HardwareSerial::bms;response[0]=0xd2;response[1]=3;response[2]=124;reg(40,120);reg(54,1);crc();
  const auto deadline=host_us+5000000;while(!b.sample().valid&&host_us<deadline)pass();assert(b.sample().valid);}
 void reg(unsigned i,unsigned v){response[3+2*i]=v>>8;response[4+2*i]=v&255;}
 void crc(){auto c=power::crc16Modbus(response.data(),response.size()-2);response[response.size()-2]=c&255;response.back()=c>>8;}
 void supervisor(){
  const auto p=power::motionPower(b.sample(),host_us/1000,running);++policyChecks;
  r.in.powerOk=p==power::MotionPower::OK;r.in.powerReason=power::name(p);r.in.refusal=power::name(p);
  r.in.imuStampUs=host_us;r.m.inputs(r.in);r.m.update(host_us);
 }
 void pass(){
  if(r.p.enables==12)for(unsigned i=0;i<12;++i){auto& q=r.p.uart->regs[Port::id(i)];if(q[40]){q[56]=q[42];q[57]=q[43];}}
  // Same cooperative ordering as Controller: Motion, DALY, Motion.
  supervisor();b.update(host_us/1000,core::OperatingMode::RUN);
  if(requests!=u->requests.size()){requests=u->requests.size();if(dropNext)dropNext=false;else due=host_us+replyDelay;}
  if(due&&host_us>=due){for(auto c:response)u->rx.push_back({host_us,c});due=0;}
  supervisor();host_us+=1000;
 }
 void stand(){assert(!r.m.stand(.05,host_us));const auto end=host_us+15000000;
  while(r.m.state()!=simple::State::STANDING&&r.m.state()!=simple::State::FAULT&&host_us<end)pass();
  assert(r.m.state()==simple::State::STANDING&&r.p.enables==12);}
 void toOff(){const auto end=host_us+1000000;while(!r.m.offVerified()&&host_us<end)pass();assert(r.m.offVerified());}
};
int main(){
 {PowerRig t;t.stand();const auto frames=t.r.p.frames.size();t.replyDelay=600000;auto start=t.requests;
  while(t.requests<start+3||t.due){t.pass();assert(t.r.m.state()==simple::State::STANDING);assert(t.r.in.powerOk);}
  std::printf("POWER_CHAIN late600ms PASS sample_age_ms=%lu policy_checks=%u frames_before=%zu hardware=false\n",(unsigned long)(host_us/1000-t.b.sample().sampled_at_ms),t.policyChecks,frames);
  // One genuinely unanswered poll: retain original sample and stop at3 s.
  t.replyDelay=140000;t.dropNext=true;start=t.requests;while(t.requests==start)t.pass();const auto stamp=t.b.sample().sampled_at_ms;
  while(t.r.m.state()!=simple::State::FAULT)t.pass();
  assert(!strcmp(t.r.m.reason(),"BMS_TELEMETRY_STALE"));assert(t.b.sample().sampled_at_ms==stamp);
  const auto age=host_us/1000-stamp;assert(age>3000&&age<3010&&t.b.lastCommResult()==power::DalyCommResult::TIMEOUT);t.toOff();
  const auto enables=t.r.p.enables;const auto writes=t.r.p.frames.size();const auto end=host_us+6000000;
  while(host_us<end)t.pass();assert(t.r.in.powerOk&&t.r.m.state()==simple::State::FAULT&&t.r.m.offVerified());
  assert(t.r.p.enables==enables&&t.r.p.frames.size()==writes);
  std::printf("POWER_CHAIN lost_poll PASS stop_age_ms=%llu fresh_recovery_no_restart hardware=false\n",(unsigned long long)age);
 }
 {PowerRig t;t.stand();t.reg(58,1);t.crc();const auto end=host_us+3000000;
  while(t.r.m.state()!=simple::State::FAULT&&host_us<end)t.pass();
  assert(t.b.sample().alarms[0]==1&&!strcmp(t.r.m.reason(),"BMS_ALARM"));assert(t.r.in.powerOk==false);t.toOff();
  std::puts("POWER_CHAIN actual_alarm_frame PASS policy_to_Motion_to_SAFE_OFF13 hardware=false");}
 std::puts("REVIEW_POWER_CHAIN PASS actual DALY/parser/policy/Motion; Controller assignments mirrored; hardware=false");
}
