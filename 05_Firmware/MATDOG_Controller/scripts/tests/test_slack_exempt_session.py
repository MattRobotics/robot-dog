#!/usr/bin/env python3
"""slackExempt() extracted unmodified from src/network/HttpTransport.cpp: LOGIN and
SESSION (AP and STA) are served without Motion slack; every other kind keeps its
previous classification. No device; host g++ only."""
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
  const bool want=k==8||k==12||k==14||k==17||k==18; // WIFI_LOGIN WIFI_SESSION MOTION_STATUS MOTION_LOGIN MOTION_SESSION
  if(slackExempt(r)!=want){std::printf("kind %d exempt=%d want=%d\n",k,slackExempt(r),want);++bad;}}
 HttpTransportRequest w;w.kind=HttpRequestKind::MOTION_COMMAND;w.motion_command.intent=motion::Intent::BASE_VELOCITY;
 if(!slackExempt(w)){std::puts("WALK STOP not exempt");++bad;}
 w.motion_command.vx=.01;if(slackExempt(w)){std::puts("moving BASE_VELOCITY exempt");++bad;}
 std::printf("SLACK_EXEMPT_SESSION %s failures=%d\n",bad?"FAIL":"PASS",bad);return bad!=0;}
'''
d=Path(tempfile.mkdtemp());(d/'t.cpp').write_text(prog)
subprocess.run(['g++','-std=c++17','-Wall','-Werror',str(d/'t.cpp'),'-o',str(d/'t')],check=True)
sys.exit(subprocess.run([str(d/'t')]).returncode)
