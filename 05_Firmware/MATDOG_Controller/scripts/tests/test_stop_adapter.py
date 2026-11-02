#!/usr/bin/env python3
"""Compile the production STOP writer gates and HTTP cancellation fence.

Only scheduling/admission is tested. Model, bus and HTTP are explicit doubles;
no timing, torque, support or mechanical-arrest qualification is implied.
"""
from pathlib import Path
import subprocess
import tempfile

HERE = Path(__file__).resolve().parent
SRC = HERE.parent.parent / "src"


def extract(path, start, end):
    text = (SRC / path).read_text()
    assert text.count(start) == text.count(end) == 1
    return text.split(start, 1)[1].split(end, 1)[0]


frame = "bool Controller::SimpleServoPort::frame" + extract(
    "core/SimpleMotionIntegration.cpp", "bool Controller::SimpleServoPort::frame",
    "bool Controller::SimpleServoPort::enable")
enable = "bool Controller::SimpleServoPort::enable" + extract(
    "core/SimpleMotionIntegration.cpp", "bool Controller::SimpleServoPort::enable",
    "bool Controller::SimpleServoPort::off")
hook = "bool Controller::stopPreempt" + extract(
    "core/Controller.cpp", "bool Controller::stopPreempt", "void Controller::update(uint32_t now_ms)")
case = "case HttpRequestKind::MOTION_COMMAND: {" + extract(
    "network/HttpTransport.cpp", "case HttpRequestKind::MOTION_COMMAND: {",
    "case HttpRequestKind::CALIBRATION_RESTORE: {")
http = (SRC / "network/HttpTransport.cpp").read_text()
assert "r.stop_epoch=self->service_->motionStopEpoch();" in http
assert "stop_epoch_" not in http

fixture = r'''
#include <cassert>
#include <cstdint>
#include <cstring>
#include <functional>
#include <cstdio>
uint32_t millis(){return 1000;}
int64_t esp_timer_get_time(){return 1000000;}
namespace simple {struct Drive{uint16_t speed;uint8_t acc;};}
namespace servo {
enum class ServoWriteVerifyResult { VERIFIED_APPLIED, FAILED };
struct ServoBus {
 struct FrameTx {bool started=false,complete=false,preempted=false;};
 unsigned writes=0,enables=0,driveChanges=0;
 bool validDrive=true;std::function<bool()> preempt;
 bool setOperationalDrive(uint16_t,uint8_t){++driveChanges;return validDrive;}
 bool syncWriteGoalPositions(const uint8_t*,const uint16_t*,FrameTx* tx){
  if(preempt&&preempt()){tx->preempted=true;return false;}++writes;tx->started=tx->complete=true;return true;
 }
 void setValidationTimeout(unsigned){}
 ServoWriteVerifyResult enableTorqueOn(uint8_t){++enables;return ServoWriteVerifyResult::VERIFIED_APPLIED;}
};}
uint8_t slotId(unsigned i){static const uint8_t ids[]={13,12,11,23,22,21,33,32,31,43,42,41};return ids[i];}
struct StopAdmissionDouble {
 bool active=false,pending=false,off=false,fault=false,frameScope=false,readScope=false;
 uint16_t authorized[12]{};simple::Drive drive{480,13};
 bool stopping()const{return active||pending||off;}
 bool stopFrameAuthorized(const uint16_t* raw,simple::Drive d)const {
  return active&&frameScope&&!pending&&!off&&!fault&&d.speed==drive.speed&&d.acc==drive.acc&&
   !memcmp(raw,authorized,sizeof(authorized));
 }
 bool busPreemptRequested()const{return pending||off||fault||(active&&!frameScope&&!readScope);}
};
struct StopLaneDouble {
 bool pending=false,admitted=false;StopAdmissionDouble* motion=nullptr;
 bool stopPending()const{return pending;}
 void serviceStopLane(uint32_t){pending=false;if(admitted)motion->pending=true;}
};
struct Controller {
 StopAdmissionDouble simple_motion_;servo::ServoBus servo_bus_;StopLaneDouble http_transport_;
 Controller(){http_transport_.motion=&simple_motion_;servo_bus_.preempt=[this]{return stopPreempt(this);};}
 struct SimpleServoPort {Controller* owner=nullptr;bool frame_preempted=false;bool frame(const uint16_t[12],simple::Drive);bool enable(unsigned);};
 static bool stopPreempt(void*);
};
'''
fixture += frame + enable + hook
fixture += r'''
enum class HttpRequestKind {MOTION_COMMAND};
enum class Intent {STAND,STOP};
struct Command {Intent intent=Intent::STAND;};
struct Request {bool local_authorized=true;uint32_t stop_epoch=0,peer=0;char cookie[1]{},csrf[1]{};Command motion_command;};
struct Response {bool ok=false;const char* message="";char text[64]{};size_t text_len=0;};
struct Portal {bool admitted=true;bool authorize(uint64_t,uint32_t,const char*,const char*)const{return admitted;}};
struct Mailbox {bool pending=true;bool awaitingResponse()const{return pending;}};
struct Service {
 uint32_t epoch=0,calls=0;
 uint32_t motionStopEpoch()const{return epoch;}
 Command motionCommand(Command c,uint32_t){++calls;if(c.intent==Intent::STOP)++epoch;return c;}
 bool motionAck(Command,char* out,size_t){strcpy(out,"ACK");return true;}
};
namespace motion {using Intent=::Intent;}
struct Http {
 Service service;Service* service_=&service;Portal portal_;Mailbox mailbox_;
 Response dispatch(const Request& req){Response resp;switch(HttpRequestKind::MOTION_COMMAND){
'''
fixture += case
fixture += r'''
 }return resp;}
};
int main(){
 Controller c;Controller::SimpleServoPort port{&c};uint16_t raw[12]{};simple::Drive drive{480,13};
 assert(port.frame(raw,drive)&&c.servo_bus_.writes==1);
 c.simple_motion_.active=true;
 assert(!port.frame(raw,drive)&&!port.enable(0)&&c.servo_bus_.writes==1&&c.servo_bus_.enables==0);
 c.simple_motion_.frameScope=true;
 assert(port.frame(raw,drive)&&c.servo_bus_.writes==2);
 raw[5]=1;assert(!port.frame(raw,drive));raw[5]=0;
 assert(!port.frame(raw,{1000,50}));
 c.simple_motion_.pending=true;assert(!port.frame(raw,drive));c.simple_motion_.pending=false;
 c.simple_motion_.fault=true;assert(!port.frame(raw,drive));c.simple_motion_.fault=false;
 c.simple_motion_.off=true;assert(!port.frame(raw,drive));c.simple_motion_.off=false;
 // A newly admitted STOP during the bus hook still preempts an authorized TX.
 c.http_transport_.pending=c.http_transport_.admitted=true;
 assert(!port.frame(raw,drive)&&c.servo_bus_.writes==2);
 c.simple_motion_.pending=false;c.http_transport_.admitted=false;c.http_transport_.pending=true;
 assert(port.frame(raw,drive)&&c.servo_bus_.writes==3); // unauthenticated hint alone grants no preemption
 c.simple_motion_.frameScope=false;c.simple_motion_.readScope=true;
 assert(!Controller::stopPreempt(&c)&&!port.frame(raw,drive));
 c.simple_motion_.readScope=false;assert(Controller::stopPreempt(&c));
 Http h;Request req;req.stop_epoch=h.service.motionStopEpoch();
 assert(h.dispatch(req).ok&&h.service.calls==1);
 // The common service fence advances for a USB STOP too, outside HTTP's lane.
 h.service.motionCommand({Intent::STOP},millis());
 assert(!h.dispatch(req).ok&&!strcmp(h.dispatch(req).message,"SUPERSEDED_BY_STOP")&&h.service.calls==2);
 req.stop_epoch=h.service.motionStopEpoch();h.portal_.admitted=false;
 assert(!h.dispatch(req).ok&&!strcmp(h.dispatch(req).message,"AUTH_FAILED"));h.portal_.admitted=true;
 h.mailbox_.pending=false;assert(!h.dispatch(req).ok&&h.service.calls==2);h.mailbox_.pending=true;
 req.motion_command.intent=Intent::STOP;req.stop_epoch=0;
 assert(h.dispatch(req).ok&&h.service.calls==3); // STOP itself is never rejected as superseded
 puts("PASS production STOP writer admission, preemption and common HTTP cancellation fence (doubles, no hardware)");
}
'''
with tempfile.TemporaryDirectory(prefix="matdog-stop-adapter-") as temporary:
    source = Path(temporary) / "adapter.cpp"
    source.write_text(fixture)
    binary = Path(temporary) / "adapter"
    subprocess.run(["g++", "-std=c++17", "-O1", "-Wall", "-Wextra", "-Werror",
                    "-Wno-misleading-indentation", "-fsanitize=address,undefined",
                    "-fno-omit-frame-pointer", str(source), "-o", str(binary)], check=True)
    subprocess.run([str(binary)], check=True)
