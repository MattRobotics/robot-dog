// Scripted encoder/electrical observations only. No thermal/motor model.
#define STAND_LOAD_NO_MAIN
#include "test_stand_load_completion.cpp"
static void standing(LoadRig& r,double h){CHECK(!r.m.stand(h,r.p.now),"stand admitted");CHECK(r.run(4000,simple::State::STANDING),"stand reached");r.in.quaternion[0]=1;r.in.quaternion[2]=0;}
int main(){
 {LoadRig r;standing(r,.10);lowers(r,20,-18,-12,10);for(int i=0;i<100;++i)r.tick();
  const unsigned frames=r.p.frames,off=r.p.offCalls;const uint64_t start=r.p.now;
  // Six hours of virtual 20 ms releases, fresh sensors, fixed load residuals.
  for(unsigned i=0;i<6*60*60*50;++i){r.tick();if(r.m.state()!=simple::State::STANDING){CHECK(false,"persistent HOLD");}}
  CHECK(r.p.now-start==21600000000ULL,"six virtual hours");CHECK(r.m.energized(),"torque remains enabled");
  CHECK(r.p.offCalls==off&&r.p.frames==frames,"constant goals retained without retransmission");
  for(auto& f:r.p.observed)CHECK(f.torque==1,"each servo torque ON");
  puts("PASS six virtual hours under scripted residuals; NOT a motor endurance or temperature test");}
 for(double from:{.10,.15}){LoadRig r;standing(r,from);CHECK(!r.m.stand(.05,r.p.now),"descending HEIGHT admitted");
  unsigned begin=r.p.sent.size();CHECK(r.run(4000,simple::State::STANDING),"descending HEIGHT reaches HOLD50");
  int step=0;for(unsigned n=begin;n<r.p.sent.size();++n){CHECK(r.p.sentTorqueOn[n]==12,"descent torque ON");if(n)for(unsigned i=0;i<12;++i)step=std::max(step,std::abs(int(r.p.sent[n][i])-int(r.p.sent[n-1][i])));}
  CHECK(step<=10,"descending HEIGHT continuity within 10 ticks/release");printf("PASS HEIGHT %.0f->50 mm max step=%d ticks (flat nominal)\n",from*1000,step);}
 // The loaded low pose is genuinely outside kHeight: don't expand its box.
 {LoadRig r;standing(r,.05);lowers(r,20,18,18,18);for(unsigned i=0;i<30;++i)r.tick();
  CHECK(r.m.stand(.10,r.p.now)!=nullptr,"loaded low pose outside HEIGHT refused");CHECK(r.m.state()==simple::State::STANDING,"refusal retains HOLD");}
 // A plausible load residual near 50 mm is not certified by the HEIGHT box.
 {LoadRig r;standing(r,.15);CHECK(!r.m.stand(.05,r.p.now),"loaded descent admitted at high pose");
  lowers(r,20,18,18,18);CHECK(r.run(4000,simple::State::FAULT),"loaded descent cannot falsely complete outside HEIGHT");
  CHECK(!strcmp(r.m.reason(),"NATIVE_TARGET_NOT_REACHED"),"loaded HEIGHT boundary remains fail closed");
  puts("PASS loaded descent to 50 blocked by existing HEIGHT box -> NATIVE_TARGET_NOT_REACHED (not qualified under load)");}
 // Bus lost also prevents verifying SAFE_OFF; never report 13/13 from silence.
 {LoadRig r;standing(r,.10);r.p.missing=true;r.p.verify=false;
  CHECK(r.run(50,simple::State::FAULT),"lost feedback faults");unsigned frames=r.p.frames;
  for(int i=0;i<200;++i)r.tick();CHECK(!r.m.offVerified()&&r.m.stopping(),"no SAFE_OFF verification without readback");
  CHECK(frames==r.p.frames,"no recovery descent on dead bus");}
 for(int which=0;which<9;++which){LoadRig r;standing(r,.15);CHECK(!r.m.stand(.05,r.p.now),"descent admitted");r.tick();
  if(which==0)r.m.stop();
  if(which==1)r.p.stopOnRead=true;
  if(which==2)r.p.missing=true;
  if(which==3){r.in.powerOk=false;r.in.powerReason="POWER_LOST";}
  if(which==4)for(auto& f:r.p.observed)f.status=1;
  if(which==5)for(auto& f:r.p.observed)f.voltage=35;
  if(which==6)for(auto& f:r.p.observed)f.torque=0;
  if(which==7){r.in.authorityOk=false;r.in.refusal="AUTHORITY_LOST";}
  if(which==8)r.in.imuValid=false;
  for(int i=0;i<30&&r.m.state()==simple::State::MOVING;++i)r.tick();
  CHECK(r.m.state()==(which<2?simple::State::STOPPED:simple::State::FAULT),"STOP/fault interrupts descending HEIGHT");
  std::string reason=r.m.reason();unsigned frames=r.p.frames;for(int i=0;i<25;++i)r.tick();CHECK(r.p.frames==frames,"no trajectory frames after STOP/fault");CHECK(r.m.offVerified(),"mock SAFE_OFF readback 13/13");
  // The mock's verifyOff works even when read() is missing: not a physical bus-loss guarantee.
  printf("PASS descending HEIGHT interruption case=%d reason=%s\n",which,reason.c_str());}
 printf("PASS operational checks=%d; full PRONE descent remains blocked\n",checks);
}
