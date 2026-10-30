#define PRONE_HEIGHT_NO_MAIN
#include "test_prone_height_residuals.cpp"
int main(){(void)checks;(void)lowers;
 for(double at:{.01,.50,1.01,1.24,2.90}){
  Replay r;r.in.quaternion[0]=1;r.in.quaternion[2]=0;
  assert(!r.m.walk(1,r.p.now));
  for(unsigned n=0;n<8000&&r.m.state()!=simple::State::WALKING;++n)r.tick();assert(r.m.state()==simple::State::WALKING);
  while(r.m.walkCycles()<at)r.tick();
  core::MotionService svc;svc.begin(&r.m,nullptr,nullptr);svc.setReady(true,"READY");
  motion::MotionCommand c;c.intent=motion::Intent::BASE_VELOCITY;c.owner=motion::ClientOwner::DASHBOARD;c.sequence=1;c.issued_at_ms=r.p.now/1000;c.ttl_ms=1000;strcpy(c.id,"walk-stop-timing");
  const auto start=r.p.now;const auto frames=r.p.frames,reads=r.p.reads,off=r.p.offCalls;
  const auto ack=svc.command(c,r.p.now/1000);assert(ack.outcome==motion::CommandOutcome::ACCEPTED);assert(!strcmp(ack.executor_state,"WALKING"));
  assert(r.p.frames==frames&&r.p.reads==reads&&r.p.offCalls==off);svc.update();
  core::MotionStatusSnapshot snap;assert(svc.captureStatus(snap,r.p.now/1000));assert(!strcmp(snap.activeId,c.id)&&snap.activeSequence==1&&snap.acceptedMs==start/1000&&snap.finishedMs==0);
  const auto repeated=svc.command(c,r.p.now/1000);assert(repeated.outcome==ack.outcome);
  for(unsigned n=0;n<2000&&r.m.state()!=simple::State::STANDING;++n){r.tick();svc.update();}
  assert(r.m.state()==simple::State::STANDING&&r.m.energized()&&r.p.offCalls==off);assert(svc.captureStatus(snap,r.p.now/1000));assert(!strcmp(snap.activeOutcome,"COMPLETED")&&snap.finishedMs>=snap.acceptedMs);
  char json[8192];assert(svc.statusJson(json,sizeof(json),r.p.now/1000));assert(strstr(json,"\"id\":\"walk-stop-timing\"")&&strstr(json,"\"commanded_raw\":"));
  printf("WALK_STOP cycle=%.3f ACK_no_IO=true completion_s=%.3f torque_ON=true\n",at,(r.p.now-start)/1e6);
 }
 {core::MotionStatusSnapshot s;memset(s.activeId,'x',48);s.activeSequence=s.acceptedMs=s.finishedMs=UINT32_MAX;s.reason=s.activeReason="SERVO_NATIVE_ALARM_OR_ELECTRICAL_LIMIT";
  s.capturedMs=UINT32_MAX;s.capturedUs=uint64_t(UINT32_MAX)*1000;test_clock_us=s.capturedUs;s.imuStampUs=1;s.imuValid=true;s.referenceFaultSlot=11;
  for(auto& f:s.feedback){f.valid=true;f.stampUs=f.controlStampUs=1;f.raw=f.goal=4095;f.load=2047;f.speed=f.current=32767;f.temp=f.status=f.voltage=255;f.q=-1.598408;}
  char json[core::kMotionStatusJsonBytes];assert(core::MotionService::formatStatus(s,json,sizeof(json)-640));printf("STATUS bounded long-age/ID/electrical values: %zu bytes, 640 B transport reserve\n",strlen(json));
 }
 puts("WALK_STOP observation PASS; software timing, not HTTP latency or motor dynamics");
}
