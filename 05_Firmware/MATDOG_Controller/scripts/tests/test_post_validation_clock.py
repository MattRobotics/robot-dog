#!/usr/bin/env python3
"""Compile the real Controller presence hooks/expiry branch with a fake session."""
from pathlib import Path
import subprocess,tempfile
fw=Path(__file__).resolve().parents[2]
s=(fw/'src/core/Controller.cpp').read_text()
start=s.index('bool Controller::operatorPresent(');end=s.index('\nconst char* Controller::armOperationalSession',start)
methods=s[start:end]
start=s.index('  if(validation_present_&&now_ms-validation_presence_ms_>presenceWindowMs())')
end=s.index('  // OPERATIONAL:',start)
expiry=s[start:end]
assert 'http_transport_.update(millis());' in s
assert 'now=millis();\n  operational_feedback_=operational_producer_.snapshot(now,' in s
unit=r'''
#include <cassert>
#include <cstdint>
#include <cstring>
namespace execution {enum class RuntimeTestMode { OPERATIONAL, DIAGNOSTIC };enum class MotionSessionState {OFF,ARMED,SAFE_OFF_REQUIRED};}
struct Controller {
 execution::RuntimeTestMode runtime_mode_=execution::RuntimeTestMode::OPERATIONAL;bool commissioning_bound_=true;
 bool validation_present_=false,validation_walk_admitted_=false;uint32_t validation_presence_ms_=0;
 struct {bool local_operator_present=false;}local_trial_evidence_;
 struct {bool operator_permit=false;}operational_facts_;
 struct {execution::MotionSessionState current=execution::MotionSessionState::OFF;const char* reason="OLD_ADMISSION_RESULT";
  auto state()const{return current;}void revoke(const char*r){reason=r;current=execution::MotionSessionState::SAFE_OFF_REQUIRED;}}operational_session_;
 uint32_t presenceWindowMs()const{return 3000;}
 bool operatorPresent(uint32_t)const;
 struct HeartbeatHook {Controller* owner;void heartbeat(uint32_t);};
 void expire(uint32_t now_ms){EXPIRY}
};
METHODS
int main(){
 Controller c;Controller::HeartbeatHook hook{&c};hook.heartbeat(10000);
 assert(c.operatorPresent(10000)&&c.operatorPresent(13000)&&!c.operatorPresent(13001));
 assert(!c.operatorPresent(9999));c.operational_facts_.operator_permit=true;c.expire(13001);
 assert(!c.validation_present_&&!c.operational_facts_.operator_permit);
 assert(!strcmp(c.operational_session_.reason,"OLD_ADMISSION_RESULT")); // OFF history unchanged
 hook.heartbeat(14000);c.operational_session_.current=execution::MotionSessionState::ARMED;c.expire(17001);
 assert(c.operational_session_.current==execution::MotionSessionState::SAFE_OFF_REQUIRED);
 assert(!strcmp(c.operational_session_.reason,"LOCAL_PRESENCE_DEADMAN_EXPIRED"));
 hook.heartbeat(UINT32_MAX-1000);assert(c.operatorPresent(1999)&&!c.operatorPresent(2000));
 c.validation_present_=false;assert(!c.operatorPresent(100)); // stored browser session alone proves no presence
 c.commissioning_bound_=false;hook.heartbeat(100);assert(!c.validation_present_);
}
'''.replace('EXPIRY',expiry).replace('METHODS',methods)
with tempfile.TemporaryDirectory(prefix='matdog-presence-') as d:
 p=Path(d);(p/'test.cpp').write_text(unit)
 subprocess.run(['g++','-std=c++17','-Wall','-Wextra','-Werror',str(p/'test.cpp'),'-o',str(p/'test')],check=True)
 subprocess.run([str(p/'test')],check=True)
print('POST_VALIDATION_CLOCK PASS: 3s boundary, future, wrap, OFF history, active SAFE_OFF, auth != presence; production hooks and admission clock wiring')
