#!/usr/bin/env python3
"""Execute production USB diagnostic/STOP branches with controller-state fakes.
No UART driver or device API: also assert only RAM export is reachable.
"""
from pathlib import Path
import subprocess,tempfile
fw=Path(__file__).resolve().parents[2]
source=(fw/'src/core/LocalValidation.cpp').read_text()
def block(marker):
    start=source.index(marker);at=source.index('{',start);depth=1;end=at+1
    while depth:
        depth+=(source[end]=='{')-(source[end]=='}');end+=1
    return source[start:end]
diag=block('if(!strcmp(t[0],"PRIME_DIAG"))')
stop=block('if(!strcmp(t[0],"STOP"))')
controller=(fw/'src/core/Controller.cpp').read_text()
assert controller.index('servo_bus_.inhibitTorqueForPrimeDiagnostics();') < controller.index('servo_bus_.begin(')
assert 'prime_diagnostic_mode_=true;' in (fw/'src/core/Controller.h').read_text()
assert 'true,now,prime_diagnostic)' in controller
assert 'if(prime_diagnostic_mode_&&!prime_diagnostic)return "PRIME_DIAGNOSTIC_MODE_USB_ONLY";' in controller
assert 'servo_bus_.read' not in diag and 'servo_bus_.write' not in diag
assert source.index('if(!strcmp(t[0],"PRIME_DIAG"))') < source.index('if(!build::kSupervisedValidationCapable)')
unit=r'''
#include "diagnostics/PrimeDiagnostic.h"
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cstdint>
#include <cerrno>
using namespace matdog;
static unsigned checks=0;
#define CHECK(c) do{++checks;if(!(c)){std::printf("FAIL %d: %s\n",__LINE__,#c);std::exit(1);}}while(0)
bool uintToken(const char* s,uint32_t& v){char* e=nullptr;errno=0;auto n=std::strtoul(s,&e,10);if(!*s||*e||*s=='-'||errno||n>UINT32_MAX)return false;v=n;return true;}
uint32_t esp_random(){return 43;}
namespace execution {enum class MotionSessionState{OFF,PRIMING,SAFE_OFF_REQUIRED};}
struct Controller {
 struct Session {
  execution::MotionSessionState current=execution::MotionSessionState::OFF;
  diagnostics::PrimeRejectionLog log;
  const auto& primeDiagnostics()const{return log;}
  auto state()const{return current;}
  const char* stateName()const{return current==execution::MotionSessionState::OFF?"OFF":"BUSY";}
  const char* reason()const{return "TEST";}
  const char* lastRevoke()const{return "RETAINED";}
  void revoke(const char*){current=execution::MotionSessionState::SAFE_OFF_REQUIRED;}
  bool clearPrimeDiagnostics(){log.clear();return true;}
 }operational_session_;
 struct Executor{bool live=false;bool active()const{return live;}void stop(const char*){live=false;}}motion_executor_;
 struct Service{bool live=false;bool busy()const{return live;}}motion_service_;
 struct Bus{bool inhibited=true;void inhibitTorqueForPrimeDiagnostics(){inhibited=true;}bool primeDiagnosticInhibit()const{return inhibited;}}servo_bus_;
 struct Wifi{bool busy=false;bool configBusy()const{return busy;}}wifi_;
 struct Producer{struct Joint{int torque=0,speed=0;}joint[12];bool fresh=true;
  bool rawFresh(uint32_t,uint32_t)const{return fresh;}
  const Joint& raw(unsigned i)const{return joint[i];}
 }operational_producer_;
 struct {struct {uint32_t feedback_max_age_ms=400;}execution;}reviewed_motion_;
 struct {bool operator_permit=false;}operational_facts_;
 bool prime_diagnostic_mode_=true,boot_safe_off_pending_=false,validation_present_=false,validation_walk_admitted_=false;
 uint32_t validation_nonce_=42,validation_presence_ms_=0;unsigned arm_calls=0;bool arm_refuse=false;
 const char* armOperationalSession(uint32_t,bool diagnostic){CHECK(diagnostic);++arm_calls;return arm_refuse?"FAULT_GATE":nullptr;}
 bool command(const char* line,char* out,size_t size){
  const char* reason="USAGE_OR_LIVE_GATE";bool ok=false;uint32_t now=100;
  auto reply=[&](){std::snprintf(out,size,"VALIDATION=%s REASON=%s",ok?"PASS":"REFUSED",reason);return ok;};
  char copy[241];std::snprintf(copy,sizeof(copy),"%s",line);const char* t[18]{};unsigned n=0;char* save=nullptr;
  for(char* p=strtok_r(copy," ",&save);p;p=strtok_r(nullptr," ",&save)){if(n==18)return reply();t[n++]=p;}if(!n)return reply();
  STOP_BRANCH
  DIAGNOSTIC_BRANCH
  return reply();
 }
};
const char* start="PRIME_DIAG START 42 CONFIRM_FIXTURE_PRESENT_POWER_CUT_TORQUE_OFF_NO_ENERGIZE";
int main(){char out[768];
 for(unsigned bad=0;bad<10;++bad){Controller c;
  if(bad==0)c.validation_nonce_=43;
  if(bad==1)c.motion_executor_.live=true;
  if(bad==2)c.motion_service_.live=true;
  if(bad==3)c.operational_session_.current=execution::MotionSessionState::PRIMING;
  if(bad==4)c.boot_safe_off_pending_=true;
  if(bad==5)c.wifi_.busy=true;
  if(bad==6)c.operational_producer_.fresh=false;
  if(bad==7)c.operational_producer_.joint[11].torque=1;
  if(bad==8)c.operational_producer_.joint[11].speed=1;
  if(bad==9)c.operational_session_.log.used=true;
  CHECK(!c.command(start,out,sizeof(out)));CHECK(c.arm_calls==0&&!c.validation_present_);
 }
 {Controller c;CHECK(!c.command("PRIME_DIAG START 42 BAD_TOKEN",out,sizeof(out)));CHECK(!c.arm_calls);
  CHECK(c.command(start,out,sizeof(out)));CHECK(c.arm_calls==1&&c.validation_present_&&c.validation_presence_ms_==100&&c.servo_bus_.inhibited);
 }
 {Controller c;c.arm_refuse=true;CHECK(!c.command(start,out,sizeof(out)));CHECK(c.arm_calls==1&&!c.validation_present_&&!c.operational_facts_.operator_permit&&c.servo_bus_.inhibited);}
 {Controller c;auto* r=c.operational_session_.log.append();r->id=13;r->attempt=1;r->reason="BACKEND_REJECTED";r->goal.attempted=true;r->goal.response.validity=diagnostics::ReadValidity::VALID;
  r->goal.response.size=16;for(unsigned i=0;i<16;++i)r->goal.response.bytes[i]=i;
  CHECK(c.command("PRIME_DIAG READ 0",out,sizeof(out)));CHECK(std::strstr(out,"rx=000102030405060708090A0B0C0D0E0F"));CHECK(!c.arm_calls&&!c.validation_present_);
  CHECK(!c.command("PRIME_DIAG READ 1",out,sizeof(out)));CHECK(!c.command("PRIME_DIAG READ -1",out,sizeof(out)));
  CHECK(c.command("PRIME_DIAG STATUS",out,sizeof(out)));CHECK(std::strstr(out,"records=1"));
  c.operational_session_.current=execution::MotionSessionState::PRIMING;CHECK(!c.command("PRIME_DIAG READ 0",out,sizeof(out)));
  CHECK(c.command("STOP",out,sizeof(out)));CHECK(c.operational_session_.log.count()==1&&c.servo_bus_.inhibited);CHECK(!c.command("PRIME_DIAG READ 0",out,sizeof(out)));
  c.operational_session_.current=execution::MotionSessionState::OFF;
  CHECK(!c.command("PRIME_DIAG CLEAR 41 CONFIRM_CLEAR_RAM_EVIDENCE",out,sizeof(out)));CHECK(c.operational_session_.log.count()==1);
  CHECK(c.command("PRIME_DIAG CLEAR 43 CONFIRM_CLEAR_RAM_EVIDENCE",out,sizeof(out)));CHECK(c.operational_session_.log.count()==0&&c.servo_bus_.inhibited);
 }
 std::printf("PRIME_DIAGNOSTIC_USB PASS checks=%u actual_production_branches=true hardware=false\n",checks);
}
'''.replace('STOP_BRANCH',stop).replace('DIAGNOSTIC_BRANCH',diag)
with tempfile.TemporaryDirectory(prefix='matdog-prime-usb-') as d:
    p=Path(d);(p/'test.cpp').write_text(unit)
    subprocess.run(['g++','-std=c++17','-O1','-Wall','-Wextra','-Werror','-I',str(fw/'src'),str(p/'test.cpp'),str(fw/'src/diagnostics/PrimeDiagnostic.cpp'),'-o',str(p/'test')],check=True)
    subprocess.run([str(p/'test')],check=True)
