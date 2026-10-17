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
# Operational image: no diagnostic interlock may exist anywhere in the sources,
# and the USB view must be unable to start or alter a session or touch the bus.
for path in (fw/'src').rglob('*'):
    if path.suffix in ('.h','.cpp'):
        text=path.read_text()
        for token in ('inhibitTorqueForPrimeDiagnostics','primeDiagnosticInhibit','prime_diagnostic_mode_','PRIME_DIAGNOSTIC_MODE','COMPLETE_NO_ENERGIZE'):
            assert token not in text,(path,token)
assert 'armOperationalSession(now,true)' not in source and 'armOperationalSession' not in diag
for token in ('servo_bus_','validation_present_','operational_facts_','START','CLEAR'):
    assert token not in diag,token
assert source.index('if(!strcmp(t[0],"PRIME_DIAG"))') < source.index('if(!build::kSupervisedValidationCapable)')
unit=r'''
#include "diagnostics/PrimeDiagnostic.h"
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cstdint>
#include <cerrno>
#include <initializer_list>
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
 }operational_session_;
 struct Executor{bool live=false;bool active()const{return live;}void stop(const char*){live=false;}}motion_executor_;
 struct Service{bool live=false;bool busy()const{return live;}}motion_service_;
 struct Wifi{bool busy=false;bool configBusy()const{return busy;}}wifi_;
 struct Producer{struct Joint{int torque=0,speed=0;}joint[12];bool fresh=true;
  bool rawFresh(uint32_t,uint32_t)const{return fresh;}
  const Joint& raw(unsigned i)const{return joint[i];}
 }operational_producer_;
 struct {struct {uint32_t feedback_max_age_ms=400;}execution;}reviewed_motion_;
 struct {bool operator_permit=false;}operational_facts_;
 bool boot_safe_off_pending_=false,validation_present_=false,validation_walk_admitted_=false;
 uint32_t validation_nonce_=42,validation_presence_ms_=0;unsigned arm_calls=0;
 const char* armOperationalSession(uint32_t){++arm_calls;return nullptr;}
 bool command(const char* line,char* out,size_t size){
  const char* reason="USAGE_OR_LIVE_GATE";bool ok=false;uint32_t now=100;(void)now;
  auto reply=[&](){std::snprintf(out,size,"VALIDATION=%s REASON=%s",ok?"PASS":"REFUSED",reason);return ok;};
  char copy[241];std::snprintf(copy,sizeof(copy),"%s",line);const char* t[18]{};unsigned n=0;char* save=nullptr;
  for(char* p=strtok_r(copy," ",&save);p;p=strtok_r(nullptr," ",&save)){if(n==18)return reply();t[n++]=p;}if(!n)return reply();
  STOP_BRANCH
  DIAGNOSTIC_BRANCH
  return reply();
 }
};
int main(){char out[768];
 // The USB view is read-only: no form of START/CLEAR can arm a session or edit evidence.
 for(const char* line:{"PRIME_DIAG START 42 CONFIRM_FIXTURE_PRESENT_POWER_CUT_TORQUE_OFF_NO_ENERGIZE","PRIME_DIAG CLEAR 42 CONFIRM_CLEAR_RAM_EVIDENCE","PRIME_DIAG","PRIME_DIAG READ","PRIME_DIAG STATUS 1"}){
  Controller c;auto* r=c.operational_session_.log.append();r->id=13;
  CHECK(!c.command(line,out,sizeof(out)));CHECK(c.arm_calls==0&&!c.validation_present_&&!c.operational_facts_.operator_permit&&c.operational_session_.log.count()==1);
 }
 {Controller c;auto* r=c.operational_session_.log.append();r->id=13;r->attempt=1;r->reason="BACKEND_REJECTED";r->goal.attempted=true;r->goal.response.validity=diagnostics::ReadValidity::VALID;
  r->goal.response.size=16;for(unsigned i=0;i<16;++i)r->goal.response.bytes[i]=i;
  CHECK(c.command("PRIME_DIAG READ 0",out,sizeof(out)));CHECK(std::strstr(out,"rx=000102030405060708090A0B0C0D0E0F"));CHECK(!c.arm_calls&&!c.validation_present_);
  CHECK(!c.command("PRIME_DIAG READ 1",out,sizeof(out)));CHECK(!c.command("PRIME_DIAG READ -1",out,sizeof(out)));
  CHECK(c.command("PRIME_DIAG STATUS",out,sizeof(out)));CHECK(std::strstr(out,"records=1"));CHECK(!std::strstr(out,"torque_inhibit")&&!std::strstr(out,"mode="));
  c.operational_session_.current=execution::MotionSessionState::PRIMING;CHECK(!c.command("PRIME_DIAG READ 0",out,sizeof(out)));
  CHECK(c.command("STOP",out,sizeof(out)));CHECK(c.operational_session_.log.count()==1);CHECK(!c.command("PRIME_DIAG READ 0",out,sizeof(out)));
  c.operational_session_.current=execution::MotionSessionState::OFF;
  c.motion_executor_.live=true;CHECK(!c.command("PRIME_DIAG READ 0",out,sizeof(out)));c.motion_executor_.live=false;
  c.motion_service_.live=true;CHECK(!c.command("PRIME_DIAG READ 0",out,sizeof(out)));c.motion_service_.live=false;
  CHECK(c.command("PRIME_DIAG READ 0",out,sizeof(out)));
 }
 std::printf("PRIME_DIAGNOSTIC_USB PASS checks=%u actual_production_branches=true hardware=false\n",checks);
}
'''.replace('STOP_BRANCH',stop).replace('DIAGNOSTIC_BRANCH',diag)
with tempfile.TemporaryDirectory(prefix='matdog-prime-usb-') as d:
    p=Path(d);(p/'test.cpp').write_text(unit)
    subprocess.run(['g++','-std=c++17','-O1','-Wall','-Wextra','-Werror','-I',str(fw/'src'),str(p/'test.cpp'),str(fw/'src/diagnostics/PrimeDiagnostic.cpp'),'-o',str(p/'test')],check=True)
    subprocess.run([str(p/'test')],check=True)
