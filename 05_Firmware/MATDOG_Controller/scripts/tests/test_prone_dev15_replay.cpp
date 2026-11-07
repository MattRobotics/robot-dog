// dev.16 PRONE replay of the dev.15 hardware handovers (2 timeouts, 2 rests).
// Offline: recorded PRONE_HEIGHT residual/load held during HOLD and PRONE_HEIGHT,
// ideal tracking afterwards. Not a dynamical or hardware landing test.
#define PRONE_HEIGHT_NO_MAIN
#include "test_prone_height_residuals.cpp"
#include "prone_dev15_fixture.h"
namespace {
struct PhaseReplay: Replay {
  bool always=false;
  void step(){const auto* keep=sample;const char* ph=m.phase();
    if(!always&&strcmp(ph,"HOLD")&&strcmp(ph,"PRONE_HEIGHT"))sample=nullptr;tick();sample=keep;}
  void standAt(int mm){in.quaternion[0]=1;in.quaternion[2]=0;assert(!m.stand(mm/1000.,p.now));
    for(unsigned n=0;n<6000&&m.state()==simple::State::MOVING;++n)tick();assert(m.state()==simple::State::STANDING);}
  double minLiftMarginTicks() const {double b=1e9;for(unsigned i=0;i<12;++i){double q=m.feedback()[i].q;
    b=std::fmin(b,std::fmin(q-simple::collision::kLift.lo[i],simple::collision::kLift.hi[i]-q));}return b/simple::kTickRad;}
};
}
int main(){
 std::setvbuf(stdout,nullptr,_IONBF,0);(void)checks;(void)lowers;unsigned rest=0;
 // 1. The four recorded descents reach PRONE_REST with the dev.16 handover.
 for(const auto& c:dev15ProneCases){PhaseReplay r;r.standAt(c.startMm);r.sample=&c.residual;for(unsigned n=0;n<12;++n)r.step();
  const char* admission=r.m.prone(r.p.now);
  if(admission){std::printf("RESULT,%s,REJECTED,%s\n",c.label,admission);continue;}
  const uint64_t t0=r.p.now;double handover=1e9;const auto off0=r.p.offCalls;
  for(unsigned n=0;n<6000&&!r.m.proneRest()&&r.m.state()!=simple::State::FAULT&&r.m.state()!=simple::State::STOPPED;++n){
   const bool height=!strcmp(r.m.phase(),"PRONE_HEIGHT");r.step();
   if(height&&!strcmp(r.m.phase(),"PRONE_LIFT"))handover=r.minLiftMarginTicks();}
  std::printf("RESULT,%s,trace(%d),from %d mm,%s,%s,%.2f s,handover min kLift margin %.1f ticks,off calls during descent %d\n",c.label,c.trace,c.startMm,
   simple::name(r.m.state()),r.m.reason(),(r.p.now-t0)/1e6,handover,int(r.p.offCalls-off0));
  rest+=r.m.proneRest();
 }
 assert(rest==4);
 // 2. A handover that cannot complete is held, not dropped. 40 ticks of loaded
 //    RH_UPPER sag (inside kLoadedTicks) keeps the pose in kHeight, outside kLift.
 {ResidualCase sag{{0,0,0,0,0,0,0,40,0,0,0,0},{0,0,0,0,0,0,0,1024|200,0,0,0,0}};
  PhaseReplay r;r.standAt(120);r.always=true;r.sample=&sag;for(unsigned n=0;n<12;++n)r.step();
  assert(!r.m.prone(r.p.now));const auto off0=r.p.offCalls;
  for(unsigned n=0;n<4000&&r.m.state()!=simple::State::STOPPED&&r.m.state()!=simple::State::FAULT;++n)r.step();
  std::printf("BLOCKED_HANDOVER,%s,%s,energized=%d,off_calls=%d\n",simple::name(r.m.state()),r.m.reason(),r.m.energized(),int(r.p.offCalls-off0));
  assert(r.m.state()==simple::State::STOPPED&&!strcmp(r.m.reason(),"PRONE_NOT_COMPLETED_SUPPORT_HELD"));
  assert(r.m.energized()&&r.p.offCalls==off0&&r.m.stopVerified());
  for(unsigned n=0;n<200;++n)r.step();   // the hold persists with every guard running
  assert(r.m.state()==simple::State::STOPPED&&r.m.energized()&&r.p.offCalls==off0);
  const char* resume=r.m.standResumeRefusal(r.p.now);
  std::printf("BLOCKED_HANDOVER_STAND_RESUME,%s\n",resume?resume:"ALLOWED");
  if(!resume){assert(!r.m.stand(.10,r.p.now));r.sample=nullptr;
   for(unsigned n=0;n<6000&&r.m.state()!=simple::State::STANDING&&r.m.state()!=simple::State::FAULT;++n)r.step();
   std::printf("BLOCKED_HANDOVER_RECOVERY,%s,%s\n",simple::name(r.m.state()),r.m.reason());assert(r.m.state()==simple::State::STANDING);}
  // Operator SAFE_OFF from the hold still verifies OFF on the 13 servos.
  r.m.safeOff();for(unsigned n=0;n<60&&!r.m.offVerified();++n)r.tick(false);assert(r.m.offVerified()&&!r.m.energized());}
 // 3. A real fault during the hold is still SAFE_OFF (power lost).
 {ResidualCase sag{{0,0,0,0,0,0,0,40,0,0,0,0},{0,0,0,0,0,0,0,1024|200,0,0,0,0}};
  PhaseReplay r;r.standAt(120);r.always=true;r.sample=&sag;for(unsigned n=0;n<12;++n)r.step();assert(!r.m.prone(r.p.now));
  for(unsigned n=0;n<4000&&r.m.state()!=simple::State::STOPPED&&r.m.state()!=simple::State::FAULT;++n)r.step();
  assert(r.m.state()==simple::State::STOPPED);r.in.powerOk=false;r.in.powerReason="BMS_UNDERVOLTAGE";
  for(unsigned n=0;n<60&&!r.m.offVerified();++n)r.step();
  std::printf("HOLD_POWER_LOSS,%s,%s,off_verified=%d\n",simple::name(r.m.state()),r.m.reason(),r.m.offVerified());
  assert(r.m.state()==simple::State::FAULT&&r.m.offVerified()&&!r.m.energized());}
 std::printf("PRONE_DEV15_REPLAY 4/4 PRONE_REST; blocked handover held, recoverable, SAFE_OFF on request and on power loss; NO PHYSICAL VALIDATION\n");
}
