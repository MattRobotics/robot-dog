// Offline replay scenarios, not a dynamical or hardware landing test.
#define STAND_LOAD_NO_MAIN
#include "test_stand_load_completion.cpp"
#include "prone_residual_fixture.h"
struct Replay: LoadRig {
  const ResidualCase* sample=nullptr;
  void tick(bool publish=true){
    p.now+=simple::kPeriodUs;in.imuStampUs=p.now;m.inputs(in);
    if(publish){p.publishCommandedPose();if(sample)for(unsigned i=0;i<12;++i){auto& f=p.observed[i];
      f.raw=p.target[i]-sample->error[i];f.q=(int(f.raw)-2048)*simple::kTickRad;f.load=sample->load[i];f.speed=0;}}
    test_clock_us=p.now;m.update(p.now);m.updateTelemetry(p.now);
    if(phases.empty()||phases.back().first!=m.phase())phases.push_back({m.phase(),p.now});
  }
  void start(){in.quaternion[0]=1;in.quaternion[2]=0;assert(!m.stand(.10,p.now));
    for(unsigned n=0;n<6000&&m.state()==simple::State::MOVING;++n)tick();assert(m.state()==simple::State::STANDING);}
};
#ifndef PRONE_HEIGHT_NO_MAIN
int main(){
 (void)checks;(void)lowers;
 unsigned blocked=0,passed=0;double worstMargin=1,leastMargin=-1;
 for(const auto& c:residualCases){Replay r;r.start();r.sample=&c;for(unsigned n=0;n<12;++n)r.tick();
   assert(!r.m.stand(.05,r.p.now));
   for(unsigned n=0;n<4000&&r.m.state()==simple::State::MOVING;++n)r.tick();
   if(r.m.state()==simple::State::FAULT){++blocked;assert(!strcmp(r.m.reason(),"NATIVE_TARGET_NOT_REACHED"));
     assert(!strcmp(r.m.exitSnapshot().phase,"HEIGHT"));double margin=1;
     for(unsigned i=0;i<12;++i){double q=r.m.exitSnapshot().fb[i].q;
       margin=std::fmin(margin,std::fmin(q-simple::collision::kHeight.lo[i],simple::collision::kHeight.hi[i]-q));
       assert(q>=simple::collision::kLift.lo[i]&&q<=simple::collision::kLift.hi[i]);}
     worstMargin=std::fmin(worstMargin,margin);leastMargin=std::fmax(leastMargin,margin);
   }else {assert(r.m.state()==simple::State::STANDING);++passed;}
 }
 assert(blocked==23&&passed==0);
 std::printf("REPLAY 23/23 blocked at HEIGHT completion; actual in kLift; kHeight margin %.9f..%.9f rad (%.3f..%.3f ticks)\n",worstMargin,leastMargin,worstMargin/simple::kTickRad,leastMargin/simple::kTickRad);
 // Candidate overlap: finish ordinary HEIGHT at 53 mm, before reverse LIFT.
 // This checks existing Controller behavior, not an implemented PRONE command.
 for(const auto& c:residualCases){Replay r;r.start();r.sample=&c;for(unsigned n=0;n<12;++n)r.tick();
   assert(!r.m.stand(.053,r.p.now));for(unsigned n=0;n<4000&&r.m.state()==simple::State::MOVING;++n)r.tick();
   if(r.m.state()!=simple::State::STANDING){std::printf("53mm failed: %s\n",r.m.reason());for(unsigned i=0;i<12;++i){const auto& f=r.m.exitSnapshot().fb[i];std::printf("slot%u q %.9f err %d load %d goal %.9f heightmargin %.9f\n",i,f.q,int(r.m.commandedRaw()[i])-int(f.raw),f.load,motion::jointSlot(r.m.target(),i),std::fmin(f.q-simple::collision::kHeight.lo[i],simple::collision::kHeight.hi[i]-f.q));}return 3;}
   for(unsigned i=0;i<12;++i){double q=r.m.feedback()[i].q;double nominal=motion::jointSlot(r.m.target(),i);
     assert(q>=simple::collision::kHeight.lo[i]&&q<=simple::collision::kHeight.hi[i]);
     assert(q>=simple::collision::kLift.lo[i]&&q<=simple::collision::kLift.hi[i]);
     assert(nominal>=simple::collision::kLift.lo[i]&&nominal<=simple::collision::kLift.hi[i]);}
 }
 std::puts("REPLAY 23/23 HEIGHT100->53 completes inside BOTH kHeight and kLift");
 // STOP preempts a descending production HEIGHT even before the next release.
 {Replay r;r.start();assert(!r.m.stand(.05,r.p.now));for(unsigned n=0;n<30;++n)r.tick();
  const auto frames=r.p.frames;r.m.stop();r.m.update(r.p.now+1);assert(r.m.stopping());
  for(unsigned n=0;n<20;++n)r.tick(false);assert(r.m.offVerified()&&r.p.frames==frames);}
 // Fault handling during actual Controller HEIGHT: no recovery trajectory.
 for(int fault=0;fault<4;++fault){Replay r;r.start();assert(!r.m.stand(.05,r.p.now));for(unsigned n=0;n<10;++n)r.tick();
   if(fault==0)r.p.missing=true;
   if(fault==1)r.in.powerOk=false;
   if(fault==2)r.p.observed[2].status=32;
   if(fault==3)r.p.observed[2].voltage=30;
   for(unsigned n=0;n<200&&r.m.state()!=simple::State::FAULT;++n)r.tick(false);
   assert(r.m.state()==simple::State::FAULT);const auto frames=r.p.frames;
   for(unsigned n=0;n<30;++n)r.tick(false);assert(r.m.offVerified()&&r.p.frames==frames);
   std::printf("FAULT descending HEIGHT %d: %s; OFF, no recovery\n",fault,r.m.reason());}
 std::puts("PASS actual Controller replay and STOP/fault regressions; NO PHYSICAL VALIDATION");
}

#endif
