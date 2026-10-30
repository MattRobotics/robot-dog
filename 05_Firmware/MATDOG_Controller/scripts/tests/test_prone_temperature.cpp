#define PRONE_HEIGHT_NO_MAIN
#include "test_prone_height_residuals.cpp"
int main(){(void)checks;(void)lowers;unsigned cases=0;
 for(const char* phase:{"PRONE_HEIGHT","PRONE_LIFT","PRONE_DEPLOY","PRONE_LAND","PRONE_CONTACT_UNVERIFIED"}){
  for(bool persistent:{false,true}){Replay r;r.start();assert(!r.m.prone(r.p.now));
   for(unsigned n=0;n<4000&&strcmp(r.m.phase(),phase);++n)r.tick();assert(!strcmp(r.m.phase(),phase));
   r.p.observed[8].temp=98;
   for(unsigned n=0;n<20&&!(r.m.hotPendingMask()&(1<<8));++n)r.tick();assert(r.m.hotPendingMask()&(1<<8));
   const auto opened=r.p.now;assert(r.m.state()!=simple::State::FAULT);
   if(!persistent)r.p.observed[8].temp=24;
   for(unsigned n=0;n<40&&r.m.state()!=simple::State::FAULT;++n)r.tick();
   if(persistent){assert(r.m.state()==simple::State::FAULT&&!strcmp(r.m.reason(),"SERVO_NATIVE_ALARM_OR_ELECTRICAL_LIMIT"));assert(r.p.now-opened>=500000&&r.p.now-opened<=640000);for(unsigned n=0;n<30;++n)r.tick(false);assert(r.m.offVerified());}
   else {assert(r.m.state()!=simple::State::FAULT&&r.m.tempSpikes()==1&&!r.m.hotPendingMask()&&r.m.energized());r.m.stop();for(unsigned n=0;n<30;++n)r.tick(false);assert(r.m.offVerified());}
   ++cases;
  }
 }
 printf("PRONE_THERMAL PASS %u scenarios, isolated 98->24 and persistent 98, all five phases; offline only\n",cases);
}
