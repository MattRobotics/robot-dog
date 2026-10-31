#include "../../src/servo/ServoBusActuatorBackend.h"
#include <string>
#include <cstring>
#ifndef MATDOG_PRIME_BUS_ONLY
#define MATDOG_FIXTURE_COMMISSIONING_NO_MAIN
#include "test_fixture_commissioning.cpp"
#else
#include <cstdio>
static int checks=0,failures=0;
#define CHECK(c) do{++checks;if(!(c)){++failures;std::printf("FAIL %d: %s\n",__LINE__,#c);}}while(0)
#define REASON(a,b) CHECK(std::strcmp((a),(b))==0)
#endif
using namespace matdog;
using namespace matdog::servo;
using diagnostics::ReadValidity;
constexpr uint8_t prime_ids[12]={13,12,11,23,22,21,33,32,31,43,42,41};
void beginBus(ServoBus& bus){CHECK(bus.begin());bus.setValidationDrive(20,5);bus.setValidationTimeout(8);host_us=0;Serial.text.clear();}
void originalTransaction(const HardwareSerial& uart,uint8_t id,uint16_t target){
 CHECK(uart.requests.size()==2);
 if(uart.requests.size()!=2)return;
 const auto& write=uart.requests[0];const auto& read=uart.requests[1];
 CHECK(write.size()==14&&write[2]==id&&write[4]==3&&write[5]==41&&write[6]==5);
 CHECK((write[7]|(write[8]<<8))==target&&write[9]==0&&write[10]==0&&write[11]==20&&write[12]==0);
 CHECK(read.size()==8&&read[2]==id&&read[4]==2&&read[5]==40&&read[6]==10);
}
void busTests(){
 for(unsigned slot=0;slot<12;++slot){const uint8_t id=prime_ids[slot];const uint16_t target=2018+slot;
  for(int failed_register:{42,46,41}){
   ServoBus bus;beginBus(bus);auto& uart=*HardwareSerial::bus;uart.fault=HardwareSerial::NOT_APPLIED;uart.fault_id=id;uart.fault_address=failed_register;
   CHECK(bus.writeGoalPosition(id,target,GoalMotionProfile::BOUNDED)==ServoWriteVerifyResult::VERIFIED_NOT_APPLIED);
   originalTransaction(uart,id,target);const auto& c=bus.lastGoalWriteCapture();
   CHECK(c.attempted&&c.id==id&&c.position==target&&c.speed==20&&c.acc==5&&c.ack==1);
   CHECK(c.response.validity==ReadValidity::VALID&&c.response.address==40&&c.response.width==10&&c.response.size==16);
   CHECK(c.mismatch==(failed_register==42?1:failed_register==46?2:4));
   CHECK(c.response.bytes[0]==255&&c.response.bytes[2]==id&&c.response.bytes[3]==12);
   CHECK(c.position_read==(failed_register==42?0:target));CHECK(c.speed_read==(failed_register==46?0:20));CHECK(c.acc_read==(failed_register==41?0:5));
   CHECK(c.write_done_ms>=c.started_ms&&c.finished_ms>=c.response.started_ms);CHECK(Serial.text.empty());
  }
  for(auto fault:{HardwareSerial::DROP,HardwareSerial::CHECKSUM,HardwareSerial::WRONG_ID,HardwareSerial::LENGTH,HardwareSerial::STATUS,HardwareSerial::TRUNCATE}){
   ServoBus bus;beginBus(bus);auto& uart=*HardwareSerial::bus;uart.fault=fault;uart.fault_id=id;uart.fault_address=40;
   CHECK(bus.writeGoalPosition(id,target,GoalMotionProfile::BOUNDED)==ServoWriteVerifyResult::UNVERIFIED_NO_RESPONSE);
   const auto& c=bus.lastGoalWriteCapture();CHECK(c.response.validity!=ReadValidity::VALID);
   CHECK(c.position_read==-1&&c.speed_read==-1&&c.acc_read==-1);CHECK(c.mismatch==0);
   CHECK(c.response.size==(fault==HardwareSerial::DROP?0:fault==HardwareSerial::TRUNCATE?15:16));
   const auto expected=fault==HardwareSerial::DROP?ReadValidity::NO_RESPONSE:fault==HardwareSerial::CHECKSUM?ReadValidity::CHECKSUM:
     fault==HardwareSerial::WRONG_ID?ReadValidity::ID:fault==HardwareSerial::LENGTH?ReadValidity::LENGTH:fault==HardwareSerial::STATUS?ReadValidity::STATUS:ReadValidity::SHORT_PACKET;
   CHECK(c.response.validity==expected);originalTransaction(uart,id,target);CHECK(host_us<20000);
   diagnostics::PrimeRejection record;record.goal=c;char text[768]{};
   CHECK(diagnostics::formatPrimeRejection(record,0,text,sizeof(text)));
   CHECK(std::strstr(text,"decoded_valid=0"));
   CHECK(std::strstr(text,fault==HardwareSerial::DROP?"raw_position=-1":"raw_position=20")!=nullptr); // targets 2018..2029
  }
  {ServoBus bus;beginBus(bus);auto& uart=*HardwareSerial::bus;uart.lose_ack=true;
   CHECK(bus.writeGoalPosition(id,target,GoalMotionProfile::BOUNDED)==ServoWriteVerifyResult::VERIFIED_APPLIED);
   CHECK(bus.lastGoalWriteCapture().ack==0&&bus.lastGoalWriteCapture().mismatch==0);originalTransaction(uart,id,target);
  }
 }
 // Operational image: nothing in the bus layer inhibits torque any more. The
 // normal ENERGIZE write is applied and read back, and SAFE_OFF still verifies OFF.
 {ServoBus bus;beginBus(bus);auto& uart=*HardwareSerial::bus;ServoBusActuatorBackend backend;backend.begin(&bus);
  for(auto id:prime_ids){CHECK(backend.enableTorque(id)==actuator::BackendWriteOutcome::VERIFIED_APPLIED);CHECK(uart.regs[id][40]==1);}
  CHECK(!uart.requests.empty());
  for(auto id:prime_ids){CHECK(bus.safeOff(id)==SafeOffResult::VERIFIED_OFF);CHECK(uart.regs[id][40]==0);}
 }
 {diagnostics::PrimeRejectionLog log;for(unsigned i=0;i<log.kCapacity;++i){auto* r=log.append();CHECK(r);r->slot=i%12;r->attempt=i/12+1;}
  CHECK(!log.append()&&log.overflow&&log.at(0)->attempt==1);CHECK(!log.at(36));char tiny[4];CHECK(!diagnostics::formatPrimeRejection(*log.at(0),0,tiny,sizeof(tiny)));
 }
 {diagnostics::PrimeRejection r;r.slot=11;r.id=43;r.attempt=3;r.target_position=4095;r.session_ms=UINT32_MAX;r.reason="UNCERTAIN_REQUIRES_SAFE_OFF";
  r.goal.started_ms=r.goal.write_done_ms=r.goal.finished_ms=r.goal.response.started_ms=r.goal.response.finished_ms=UINT32_MAX;
  r.goal.response.size=38;r.goal.response.width=32;r.goal.response.drained=64;char text[768];CHECK(diagnostics::formatPrimeRejection(r,35,text,sizeof(text)));
 }
 std::printf("PRIME_DIAGNOSTIC_BUS slots=0..11 fields=41,42,46 parser_faults=6 original_write_plus_one_block=true checks=%d failures=%d hardware=false\n",checks,failures);
}
#ifndef MATDOG_PRIME_BUS_ONLY
std::string recordText(const diagnostics::PrimeRejection& r,unsigned index){char text[768]{};CHECK(diagnostics::formatPrimeRejection(r,index,text,sizeof(text)));return text;}
void offAndConfirm(Bench& b,ServoBus& bus){for(auto id:prime_ids)CHECK(bus.safeOff(id)==SafeOffResult::VERIFIED_OFF);b.r.executor.confirmSafeOff(true);b.session.confirmSafeOff();CHECK(b.session.state()==MotionSessionState::OFF);}
void sessionTests(){
 for(unsigned slot=0;slot<12;++slot)for(int field:{42,46,41,40}){
  Bench b;ServoBus bus;beginBus(bus);ServoBusActuatorBackend backend;backend.begin(&bus);b.r.runtime.begin(&b.r.policy,&backend);
  auto& uart=*HardwareSerial::bus;uart.fault_id=prime_ids[slot];uart.fault_address=field;uart.fault=field==40?HardwareSerial::DROP:HardwareSerial::NOT_APPLIED;
  CHECK(b.session.request(b.evidence,b.r.facts,b.r.envelope,b.r.feedback,true,b.now));
  for(unsigned k=0;k<50&&b.session.state()==MotionSessionState::PRIMING;++k){b.fresh();b.session.update(b.evidence,b.r.facts,b.r.feedback,b.now);}
  CHECK(b.session.safeOffRequired()&&b.session.failedJoint()==int(slot));CHECK(!b.r.executor.active());CHECK(b.session.primeDiagnostics().count()==3&&b.session.primeDiagnostics().used);
  std::string saved[3];for(unsigned i=0;i<3;++i){const auto* r=b.session.primeDiagnostics().at(i);CHECK(r);
   CHECK(r->slot==slot&&r->id==prime_ids[slot]&&r->attempt==i+1);CHECK(r->target_position==r->goal.position&&r->goal.speed==20&&r->goal.acc==5);
   CHECK(r->goal.response.validity==(field==40?ReadValidity::NO_RESPONSE:ReadValidity::VALID));saved[i]=recordText(*r,i);
  }
  uart.fault=HardwareSerial::NONE;offAndConfirm(b,bus);
  for(unsigned i=0;i<3;++i)CHECK(recordText(*b.session.primeDiagnostics().at(i),i)==saved[i]);
  auto exported=uart.requests.size();for(unsigned i=0;i<3;++i)recordText(*b.session.primeDiagnostics().at(i),i);CHECK(uart.requests.size()==exported);
  // The next admitted session replaces the evidence of the previous one: stale records can never block it.
  b.fresh();CHECK(b.session.request(b.evidence,b.r.facts,b.r.envelope,b.r.feedback,true,b.now));CHECK(b.session.primeDiagnostics().count()==0);
 }
 // Normal operation through the REAL ServoBus: PRIME writes the 12 holding goals
 // with torque OFF (24 transactions), only then ENERGIZE enables torque on every
 // joint, ending in ARMING; an operator STOP afterwards is verified OFF.
 {Bench b;ServoBus bus;beginBus(bus);ServoBusActuatorBackend backend;backend.begin(&bus);b.r.runtime.begin(&b.r.policy,&backend);auto& uart=*HardwareSerial::bus;
  CHECK(b.session.request(b.evidence,b.r.facts,b.r.envelope,b.r.feedback,true,b.now));
  for(unsigned i=0;i<12;++i){b.fresh();b.session.update(b.evidence,b.r.facts,b.r.feedback,b.now);
   if(i<11){CHECK(b.session.state()==MotionSessionState::PRIMING);}
   for(auto id:prime_ids)CHECK(uart.regs[id][40]==0);}
  CHECK(b.session.state()==MotionSessionState::ENERGIZING&&!b.session.safeOffRequired());
  CHECK(uart.requests.size()==24);CHECK(b.session.primeDiagnostics().count()==0);
  for(unsigned i=0;i<12;++i){b.fresh();b.session.update(b.evidence,b.r.facts,b.r.feedback,b.now);
   CHECK(!b.session.safeOffRequired());
   if(i<11){CHECK(b.session.state()==MotionSessionState::ENERGIZING);}}
  CHECK(b.session.state()==MotionSessionState::ARMING);
  for(auto id:prime_ids)CHECK(uart.regs[id][40]==1);
  b.session.revoke("LOCAL_OPERATOR_STOP");b.fresh();b.session.update(b.evidence,b.r.facts,b.r.feedback,b.now);
  CHECK(b.session.safeOffRequired());offAndConfirm(b,bus);for(auto id:prime_ids)CHECK(uart.regs[id][40]==0);
 }
 // A persistent PRIME failure repeated many times (more than the log capacity
 // of three sessions): every attempt reports its OWN cause, the log never
 // accumulates and never becomes the reason of a refusal or revocation.
 {Bench b;ServoBus bus;beginBus(bus);ServoBusActuatorBackend backend;backend.begin(&bus);b.r.runtime.begin(&b.r.policy,&backend);auto& uart=*HardwareSerial::bus;
  uart.fault_id=prime_ids[0];uart.fault_address=42;uart.fault=HardwareSerial::NOT_APPLIED;
  for(unsigned attempt=0;attempt<40;++attempt){
   b.fresh();CHECK(b.session.request(b.evidence,b.r.facts,b.r.envelope,b.r.feedback,true,b.now));
   for(unsigned k=0;k<50&&b.session.state()==MotionSessionState::PRIMING;++k){b.fresh();b.session.update(b.evidence,b.r.facts,b.r.feedback,b.now);}
   CHECK(b.session.safeOffRequired());CHECK(b.session.primeDiagnostics().count()==3&&!b.session.primeDiagnostics().overflow);
   REASON(b.session.lastRevoke(),"PRIME_BACKEND_NOT_APPLIED");
   offAndConfirm(b,bus);
  }
 }
 // One transient rejection on EVERY joint, followed by success: later writes
 // overwrite the bus cache but must never overwrite earlier frozen records.
 {Bench b;ServoBus bus;beginBus(bus);ServoBusActuatorBackend backend;backend.begin(&bus);b.r.runtime.begin(&b.r.policy,&backend);auto& uart=*HardwareSerial::bus;
  CHECK(b.session.request(b.evidence,b.r.facts,b.r.envelope,b.r.feedback,true,b.now));
  for(unsigned i=0;i<12;++i){
   uart.fault_id=prime_ids[i];uart.fault_address=i%3==0?42:i%3==1?46:41;uart.fault=HardwareSerial::NOT_APPLIED;
   b.fresh();b.session.update(b.evidence,b.r.facts,b.r.feedback,b.now);
   uart.fault=HardwareSerial::NONE;b.fresh();b.session.update(b.evidence,b.r.facts,b.r.feedback,b.now);
  }
  CHECK(b.session.state()==MotionSessionState::ENERGIZING&&b.session.primeDiagnostics().count()==12);
  for(unsigned i=0;i<12;++i){const auto* r=b.session.primeDiagnostics().at(i);CHECK(r->slot==i&&r->id==prime_ids[i]&&r->attempt==1&&r->goal.mismatch==(1u<<(i%3)));}
  b.session.revoke("LOCAL_OPERATOR_STOP");b.fresh();b.session.update(b.evidence,b.r.facts,b.r.feedback,b.now);offAndConfirm(b,bus);CHECK(b.session.primeDiagnostics().count()==12);
 }
 // A commit refusal has no new bus response; do not attach a stale capture.
 {Bench b;ServoBus bus;beginBus(bus);ServoBusActuatorBackend backend;backend.begin(&bus);b.r.runtime.begin(nullptr,&backend);
  CHECK(b.session.request(b.evidence,b.r.facts,b.r.envelope,b.r.feedback,true,b.now));b.fresh();b.session.update(b.evidence,b.r.facts,b.r.feedback,b.now);
  CHECK(b.session.safeOffRequired()&&b.session.primeDiagnostics().count()==1);CHECK(!b.session.primeDiagnostics().at(0)->goal.attempted);CHECK(HardwareSerial::bus->requests.empty());
 }
 {Bench b;ServoBus bus;beginBus(bus);ServoBusActuatorBackend backend;backend.begin(&bus);b.r.runtime.begin(&b.r.policy,&backend);auto& uart=*HardwareSerial::bus;
  uart.fault_id=13;uart.fault_address=46;uart.fault=HardwareSerial::NOT_APPLIED;
  CHECK(b.session.request(b.evidence,b.r.facts,b.r.envelope,b.r.feedback,true,b.now));b.fresh();b.session.update(b.evidence,b.r.facts,b.r.feedback,b.now);
  auto saved=recordText(*b.session.primeDiagnostics().at(0),0);auto count=uart.requests.size();b.session.revoke("LOCAL_OPERATOR_STOP");b.fresh();b.session.update(b.evidence,b.r.facts,b.r.feedback,b.now);
  CHECK(uart.requests.size()==count);uart.fault=HardwareSerial::NONE;offAndConfirm(b,bus);CHECK(recordText(*b.session.primeDiagnostics().at(0),0)==saved);
 }
 for(unsigned stop_slot=0;stop_slot<12;++stop_slot){Bench b;ServoBus bus;beginBus(bus);ServoBusActuatorBackend backend;backend.begin(&bus);b.r.runtime.begin(&b.r.policy,&backend);
  CHECK(b.session.request(b.evidence,b.r.facts,b.r.envelope,b.r.feedback,true,b.now));
  for(unsigned i=0;i<stop_slot;++i){b.fresh();b.session.update(b.evidence,b.r.facts,b.r.feedback,b.now);}
  auto count=HardwareSerial::bus->requests.size();b.session.revoke("LOCAL_OPERATOR_STOP");b.fresh();b.session.update(b.evidence,b.r.facts,b.r.feedback,b.now);
  CHECK(HardwareSerial::bus->requests.size()==count&&b.session.safeOffRequired());offAndConfirm(b,bus);
 }
 for(bool stale:{false,true}){Bench b;CHECK(b.session.request(b.evidence,b.r.facts,b.r.envelope,b.r.feedback,true,b.now));
  b.now+=stale?b.r.envelope.feedback_max_age_ms+1:b.r.envelope.batch_timeout_ms;
  if(!stale)for(auto& t:b.r.feedback.sampled_at_ms)t=b.now;
  b.session.update(b.evidence,b.r.facts,b.r.feedback,b.now);CHECK(b.session.safeOffRequired()&&b.r.backend.attempts==0);
  REASON(b.session.reason(),stale?"STALE_FEEDBACK":"PRIME_ARM_TIMEOUT");
 }
 std::printf("PRIME_DIAGNOSTIC_SESSION all_12_slots retries=3 fields_and_timeout_frozen STOP_all_slots PRIME_ENERGIZE_ARMING=true repeated_failures=40 checks=%d failures=%d hardware=false\n",checks,failures);
}
#endif
int main(){busTests();
#ifndef MATDOG_PRIME_BUS_ONLY
 sessionTests();
#endif
 std::printf("PRIME_DIAGNOSTIC %s checks=%d failures=%d hardware=false\n",failures?"FAIL":"PASS",checks,failures);return failures?1:0;
}
