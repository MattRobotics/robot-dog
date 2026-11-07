#!/usr/bin/env python3
"""slackExempt() extracted unmodified from src/network/HttpTransport.cpp: LOGIN and
SESSION (AP and STA) and, since dev.17, the two status pages are served without
Motion slack; every other kind keeps its previous classification. The status
pages' loop-side bodies are also checked to stay RAM-only. No device; host g++ only."""
from pathlib import Path
import os,subprocess,tempfile,sys
SRC=Path(os.environ.get('HTTP_SRC_DIR') or Path(__file__).resolve().parent.parent.parent/'src')
c=(SRC/'network/HttpTransport.cpp').read_text()
body='static bool slackExempt('+c.split('static bool slackExempt(',1)[1].split('} // slackExempt',1)[0]+'}'
prog=r'''
#include <cstdio>
#include <cstdint>
namespace motion { enum class Intent { STOP, STAND, BASE_VELOCITY }; struct MotionCommand { Intent intent=Intent::STOP; double vx=0,vy=0,yaw_rate=0; }; }
enum class HttpRequestKind : uint8_t { NONE, WEB_STATUS, OTA_CHALLENGE, OTA_BEGIN, OTA_CHUNK, OTA_FINISH, OTA_ABORT, WIFI_STATUS, WIFI_LOGIN,
  WIFI_CONFIG, WIFI_SCAN, WIFI_AP, WIFI_SESSION, WIFI_LOGOUT, MOTION_STATUS, MOTION_CAPABILITIES, MOTION_COMMAND, MOTION_LOGIN, MOTION_SESSION,
  MOTION_LOGOUT, CALIBRATION_RESTORE };
struct HttpTransportRequest { HttpRequestKind kind=HttpRequestKind::NONE; motion::MotionCommand motion_command{}; };
'''+body+r'''
int main(){int bad=0;
 for(int k=0;k<=20;++k){HttpTransportRequest r;r.kind=static_cast<HttpRequestKind>(k);r.motion_command.intent=motion::Intent::STAND;
  const bool want=k==1||k==7||k==8||k==12||k==14||k==17||k==18; // WEB_STATUS WIFI_STATUS (dev.17) WIFI_LOGIN WIFI_SESSION MOTION_STATUS MOTION_LOGIN MOTION_SESSION
  if(slackExempt(r)!=want){std::printf("kind %d exempt=%d want=%d\n",k,slackExempt(r),want);++bad;}}
 HttpTransportRequest w;w.kind=HttpRequestKind::MOTION_COMMAND;w.motion_command.intent=motion::Intent::BASE_VELOCITY;
 if(!slackExempt(w)){std::puts("WALK STOP not exempt");++bad;}
 w.motion_command.vx=.01;if(slackExempt(w)){std::puts("moving BASE_VELOCITY exempt");++bad;}
 std::printf("SLACK_EXEMPT_SESSION %s failures=%d\n",bad?"FAIL":"PASS",bad);return bad!=0;}
'''
# dev.17: /status and /wifi/status are slack-exempt because they format RAM
# facts only. Source-anchored guard: their loop-side bodies call no servo bus,
# NVS, Wi-Fi driver, wait or lock, and only reviewed cached accessors.
web=c.split('case HttpRequestKind::WEB_STATUS: {',1)[1].split('case HttpRequestKind::',1)[0]
wifi_case=c.split('case HttpRequestKind::WIFI_STATUS: {',1)[1].split('case HttpRequestKind::',1)[0]
wifi=c.split('void HttpTransport::wifiResponse(',1)[1].split('\n}\n',1)[0]
import re
cached={'bmsSample','calibrationStatus','otaStatus','systemHealth','powerState','operatingMode','authorityOwner','imuAvailability',
        'bmsAvailability','servoAvailability','ledAvailability','imuStreamEnabled','imuRvCount','bmsLastCommResult','bmsLastResultAgeMs',
        'bmsHasValidSample','wifiStatus','ledPresentationState','actuatorPolicyEpoch','actuatorPolicyGeometryBound','motionSnapshot',
        'networkAdminAccess','readiness'}  # readiness(): constexpr flags + pure classifier
guard_bad=0
for name,text in (('WEB_STATUS',web),('WIFI_STATUS',wifi_case),('wifiResponse',wifi)):
  for token in ('ServoBus','servo_bus','readTimed','Preferences','esp_wifi_','WiFi.','vTaskDelay','delay(','xSemaphoreTake',
                'ulTaskNotifyTake','Serial','esp_http_client','scanNetworks'):
    if token in text: print(f'{name}: forbidden {token!r} on the slack-exempt path');guard_bad+=1
  if re.search(r'\bnvs_[a-z_]+\s*\(',text): print(f'{name}: NVS API call on the slack-exempt path');guard_bad+=1
  extra=set(re.findall(r'service_->(\w+)\(',text))-cached
  if extra: print(f'{name}: accessor(s) not reviewed as cached: {sorted(extra)}');guard_bad+=1
print(f'STATUS_PAGES_RAM_ONLY {"FAIL" if guard_bad else "PASS"} accessors={len(set(re.findall(r"service_->(\w+)\(",web+wifi_case+wifi)))}')
if guard_bad: sys.exit(1)
d=Path(tempfile.mkdtemp());(d/'t.cpp').write_text(prog)
subprocess.run(['g++','-std=c++17','-Wall','-Werror',str(d/'t.cpp'),'-o',str(d/'t')],check=True)
sys.exit(subprocess.run([str(d/'t')]).returncode)
