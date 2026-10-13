#!/usr/bin/env python3
"""Execute the actual production goal writer with a bus/register test double."""
import subprocess,tempfile
from pathlib import Path
FW=Path(__file__).resolve().parents[2]
source=(FW/'src/servo/ServoBus.cpp').read_text();start=source.index('ServoWriteVerifyResult ServoBus::writeGoalPosition(');body=source.index('{',start);depth=1;end=body+1
while depth:
 depth+=(source[end]=='{')-(source[end]=='}');end+=1
function=source[start:end]
prefix=r"""
#include <cstdint>
#include <cstring>
#include <cstdio>
#include <cstdlib>
using s16=int16_t;
namespace core {enum class DetectedState{ONLINE,NO_RESPONSE};}
enum class GoalMotionProfile{BOUNDED,SEARCH_ENVELOPE};
enum class ServoWriteVerifyResult{VERIFIED_APPLIED,VERIFIED_NOT_APPLIED,UNVERIFIED_NO_RESPONSE};
ServoWriteVerifyResult classifyServoWriteVerify(int32_t readback,int32_t expected){return readback<0?ServoWriteVerifyResult::UNVERIFIED_NO_RESPONSE:readback==expected?ServoWriteVerifyResult::VERIFIED_APPLIED:ServoWriteVerifyResult::VERIFIED_NOT_APPLIED;}
constexpr unsigned SMS_STS_TORQUE_ENABLE=40,SMS_STS_ACC=41,SMS_STS_GOAL_POSITION_L=42,SMS_STS_GOAL_SPEED_L=46;
struct Bus {uint8_t c[10]{};unsigned writes=0;int width=10,goal=1200,speed=0,acc=0;void WritePosEx(uint8_t,s16,int s,int a){++writes;speed=s;acc=a;}int Read(uint8_t,unsigned,uint8_t* p,unsigned n){memcpy(p,c,n);return width;}int readWord(uint8_t,unsigned){return goal;}void profile(unsigned q,unsigned v,unsigned a){c[2]=q;c[3]=q>>8;c[6]=v;c[7]=v>>8;c[1]=a;}};
struct ScopedIOTimeout{ScopedIOTimeout(Bus&,unsigned){}};
struct ServoBus{Bus st_;unsigned validation_timeout_ms_=10,validation_speed_=20,validation_acceleration_=1;static constexpr unsigned kOperationalTimeoutMs=100,kSearchEnvelopeSpeed=160,kSearchEnvelopeAcceleration=10,kBoundedWriteSpeed=40,kBoundedWriteAcceleration=1;core::DetectedState last_detected_{};ServoWriteVerifyResult writeGoalPosition(int,uint16_t,GoalMotionProfile);};
"""
test=r"""
int main(){unsigned checks=0;auto check=[&](bool v){++checks;if(!v){printf("FAIL %u\n",checks);exit(1);}};ServoBus b;using R=ServoWriteVerifyResult;auto write=[&](){return b.writeGoalPosition(11,1200,GoalMotionProfile::BOUNDED);};
 b.st_.profile(1200,20,1);check(write()==R::VERIFIED_APPLIED);check(b.st_.speed==20&&b.st_.acc==1);
 b.st_.profile(1200,21,1);check(write()==R::VERIFIED_NOT_APPLIED);
 b.st_.profile(1200,20,2);check(write()==R::VERIFIED_NOT_APPLIED);
 b.st_.profile(1199,20,1);check(write()==R::VERIFIED_NOT_APPLIED);
 for(int n:{0,1,9}){b.st_.width=n;check(write()==R::UNVERIFIED_NO_RESPONSE);}
 auto before=b.st_.writes;check(b.writeGoalPosition(-1,1200,GoalMotionProfile::BOUNDED)==R::VERIFIED_NOT_APPLIED);check(b.writeGoalPosition(11,4096,GoalMotionProfile::BOUNDED)==R::VERIFIED_NOT_APPLIED);check(b.writeGoalPosition(11,1200,static_cast<GoalMotionProfile>(8))==R::VERIFIED_NOT_APPLIED);check(b.st_.writes==before);
 b.validation_speed_=0;check(write()==R::VERIFIED_APPLIED);check(b.st_.speed==40);check(b.writeGoalPosition(11,1200,GoalMotionProfile::SEARCH_ENVELOPE)==R::VERIFIED_APPLIED);check(b.st_.speed==160);
 printf("VALIDATION_DRIVE_READBACK PASS checks=%u production_function=true hardware=false\n",checks);}
"""
with tempfile.TemporaryDirectory(prefix='matdog-goal-readback-') as d:
 unit=Path(d)/'unit.cpp';unit.write_text('#include <initializer_list>\n'+prefix+function+test);exe=Path(d)/'unit';subprocess.run(['g++','-std=c++17','-O2',str(unit),'-o',str(exe)],check=True);subprocess.run([str(exe)],check=True)
