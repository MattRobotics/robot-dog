#define PRONE_HEIGHT_NO_MAIN
#include "test_prone_height_residuals.cpp"
static const char* terminal="PRONE_CONTACT_UNVERIFIED";
static bool atEnd(const Replay& r){return !strcmp(r.m.phase(),terminal);}
static void descend(Replay& r){
 const auto off=r.p.offCalls,enables=r.p.enables,frames=r.p.frames;assert(!r.m.prone(r.p.now));assert(r.p.frames==frames);
 double mechanical[12],boxMargin[12];for(unsigned i=0;i<12;++i)mechanical[i]=boxMargin[i]=1;
 const auto releases=r.m.releases(),reads=r.p.reads;
 int maxStep=0;unsigned previous[12];for(unsigned i=0;i<12;++i)previous[i]=r.p.target[i];
 for(unsigned n=0;n<4000&&!atEnd(r)&&r.m.state()!=simple::State::FAULT;++n){r.tick();
  const char* ph=r.m.phase();const auto& box=!strcmp(ph,"PRONE_HEIGHT")?simple::collision::kHeight:!strcmp(ph,"PRONE_LIFT")?simple::collision::kLift:!strcmp(ph,"PRONE_DEPLOY")?simple::collision::kDeploy:simple::collision::kEntry;
  for(unsigned i=0;i<12;++i){double q=r.m.feedback()[i].q;mechanical[i]=std::fmin(mechanical[i],std::fmin(q-r.in.minRad[i],r.in.maxRad[i]-q));boxMargin[i]=std::fmin(boxMargin[i],std::fmin(q-box.lo[i],box.hi[i]-q));maxStep=std::max(maxStep,std::abs(int(r.p.target[i])-int(previous[i])));previous[i]=r.p.target[i];}}
 if(!atEnd(r)){printf("FAIL descent: %s | %s\n",r.m.reason(),r.trail().c_str());std::exit(1);}
 assert(r.p.reads-reads<=2*(r.m.releases()-releases)+(r.m.releases()-releases+5)/6);
 for(unsigned i=0;i<12;++i){assert(mechanical[i]>=0&&boxMargin[i]>=0);printf("MARGIN,%u,%.12f,%.12f\n",i,mechanical[i],boxMargin[i]);}
 assert(r.m.energized()&&r.m.state()==simple::State::PRONE&&!r.m.offVerified());
 assert(r.p.offCalls==off&&r.p.enables==enables&&maxStep<=12);
 for(unsigned j=frames;j<r.p.sent.size();++j)assert(r.p.sentTorqueOn[j]==12);
 for(unsigned i=0;i<12;++i)assert(r.p.observed[i].q>=r.in.minRad[i]&&r.p.observed[i].q<=r.in.maxRad[i]);
 printf("PASS descent max step %d ticks | %s\n",maxStep,r.trail().c_str());
 printf("ENDPOSE");for(unsigned i=0;i<12;++i)printf(",%.12f",r.m.feedback()[i].q);puts("");
}
static void until(Replay& r,const char* phase){for(unsigned n=0;n<4000&&strcmp(r.m.phase(),phase)&&r.m.state()!=simple::State::FAULT;++n)r.tick();assert(!strcmp(r.m.phase(),phase));}
int main(){
 setvbuf(stdout,nullptr,_IOLBF,0);(void)checks;(void)lowers;
 for(double h:{.05,.10,.15}){Replay r;r.in.quaternion[0]=1;r.in.quaternion[2]=0;assert(!r.m.stand(h,r.p.now));
   for(unsigned n=0;n<6000&&r.m.state()==simple::State::MOVING;++n)r.tick();assert(r.m.state()==simple::State::STANDING);descend(r);
   const auto frames=r.p.frames,off=r.p.offCalls;for(unsigned n=0;n<30000;++n)r.tick();
   assert(atEnd(r)&&r.p.frames==frames&&r.p.offCalls==off&&r.m.energized()); // virtual 600 s, NOT thermal validation
   // dev.5: STAND directly from the held terminal, torque kept ON (no SAFE_OFF,
   // no re-enable), through DISENGAGE/ALIGN/DEPLOY/LIFT; PRONE is refused there.
   assert(!strcmp(r.m.prone(r.p.now),"PRONE_REQUIRES_STAND_HOLD"));
   const auto enables=r.p.enables;assert(!r.m.stand(h,r.p.now)&&r.m.state()==simple::State::MOVING);
   assert(!strcmp(r.m.stand(h,r.p.now),"MOTION_BUSY"));
   for(unsigned n=0;n<6000&&r.m.state()==simple::State::MOVING;++n)r.tick();
   if(r.m.state()!=simple::State::STANDING){printf("FAIL rise from PRONE: %s | %s\n",r.m.reason(),r.trail().c_str());std::exit(1);}
   assert(r.p.offCalls==off&&r.p.enables==enables&&r.m.energized());
   printf("PASS STAND%.0f -> PRONE -> STAND%.0f without STOP | %s\n",h*1000,h*1000,r.trail().c_str());
   descend(r); // and down again
   r.m.stop();for(unsigned n=0;n<20;++n)r.tick(false);assert(r.m.offVerified());}
 // WALK from the held terminal: same STAND(150) rise, then the gait; WALK STOP -> HOLD, torque ON throughout.
 {Replay r;r.start();assert(!r.m.prone(r.p.now));until(r,terminal);const auto off=r.p.offCalls,enables=r.p.enables;
  assert(!r.m.walk(1,r.p.now));for(unsigned n=0;n<8000&&r.m.state()!=simple::State::WALKING&&r.m.state()!=simple::State::FAULT;++n)r.tick();
  assert(r.m.state()==simple::State::WALKING);for(unsigned n=0;n<150;++n)r.tick();r.m.walkStop();
  for(unsigned n=0;n<4000&&r.m.state()!=simple::State::STANDING&&r.m.state()!=simple::State::FAULT;++n)r.tick();
  assert(r.m.state()==simple::State::STANDING&&r.p.offCalls==off&&r.p.enables==enables&&r.m.energized());
  printf("PASS PRONE -> WALK -> WALK STOP -> HOLD without STOP | %s\n",r.trail().c_str());}
 // Rise refused without TX (stays PRONE, torque ON) on stale feedback; a measured
 // pose outside the entry box is caught first by the existing terminal guard.
 {Replay r;r.start();assert(!r.m.prone(r.p.now));until(r,terminal);const auto f=r.p.frames;
  assert(!strcmp(r.m.stand(.05,r.p.now+400000),"PRONE_FEEDBACK_UNAVAILABLE")&&r.m.state()==simple::State::PRONE&&r.p.frames==f&&r.m.energized());
  r.p.holdObservation=true;r.p.observed[0].raw+=400;r.p.observed[0].q+=400*simple::kTickRad;
  for(unsigned n=0;n<12&&r.m.state()==simple::State::PRONE;++n)r.tick();
  assert(r.m.state()==simple::State::FAULT&&!strcmp(r.m.reason(),"PRONE_MEASURED_OUTSIDE_BOX"));
  for(unsigned n=0;n<30;++n)r.tick(false);assert(r.m.offVerified());
  puts("PASS rise refused on stale feedback; outside-box terminal guard unchanged");}
 for(const auto& c:residualCases){Replay r;r.start();r.sample=&c;for(unsigned n=0;n<12;++n)r.tick();descend(r);}
 for(const char* phase:{"PRONE_HEIGHT","PRONE_LIFT","PRONE_DEPLOY","PRONE_LAND",terminal}){
  for(int mode=0;mode<6;++mode){Replay r;r.start();assert(!r.m.prone(r.p.now));until(r,phase);r.tick();
    if(mode==0){const auto f=r.p.frames;r.m.stop();r.m.update(r.p.now+1);assert(r.p.frames==f);}
    if(mode==1)r.p.stopOnRead=true;
    if(mode==2)r.p.missing=true;
    if(mode==3)r.in.powerOk=false;
    if(mode==4)r.p.observed[2].status=32;
    if(mode==5)r.p.observed[2].voltage=30;
    for(unsigned n=0;n<200&&r.m.state()!=simple::State::FAULT&&r.m.state()!=simple::State::STOPPED;++n)r.tick(false);
    assert(r.m.state()==(mode<2?simple::State::STOPPED:simple::State::FAULT));const auto f=r.p.frames;
    for(unsigned n=0;n<30;++n)r.tick(false);assert(r.m.offVerified()&&r.p.frames==f);
  }
 }
 // STOP latched inside a native frame; no later target or landing transition.
 {Replay r;r.start();assert(!r.m.prone(r.p.now));until(r,"PRONE_LAND");r.p.stopOnFrame=true;
  for(unsigned n=0;n<50&&r.m.state()!=simple::State::STOPPED;++n)r.tick();assert(r.m.state()==simple::State::STOPPED);
  for(unsigned n=0;n<20;++n)r.tick(false);assert(r.m.offVerified());}
 // Service completion never claims that the body is supported.
 {Replay r;r.start();core::MotionService svc;svc.begin(&r.m,nullptr,nullptr);svc.setReady(true,"READY");
  motion::MotionCommand c;c.intent=motion::Intent::LIE_DOWN;c.owner=motion::ClientOwner::DASHBOARD;c.sequence=1;c.issued_at_ms=r.p.now/1000;c.ttl_ms=1000;strcpy(c.id,"prone-end");
  assert(svc.command(c,r.p.now/1000).outcome==motion::CommandOutcome::ACCEPTED);until(r,terminal);svc.update();core::MotionStatusSnapshot s;assert(svc.captureStatus(s,r.p.now/1000));
  assert(!strcmp(s.activeOutcome,"COMPLETED")&&!strcmp(s.activeReason,terminal)&&s.energized&&!s.offVerified&&s.finishedMs);
  // WALK STOP has nothing to stop there and must not take over the active command.
  motion::MotionCommand w=c;w.intent=motion::Intent::BASE_VELOCITY;w.vx=0;w.sequence=2;w.issued_at_ms=r.p.now/1000;strcpy(w.id,"walk-stop");
  const auto a=svc.command(w,r.p.now/1000);assert(a.outcome==motion::CommandOutcome::REJECTED&&!strcmp(a.reason,"NOT_WALKING"));
  svc.update();assert(svc.captureStatus(s,r.p.now/1000)&&!strcmp(s.activeId,"prone-end")&&!strcmp(s.activeOutcome,"COMPLETED"));
  motion::MotionCommand st=c;st.intent=motion::Intent::STAND;st.target_height_m=.05;st.sequence=3;st.issued_at_ms=r.p.now/1000;strcpy(st.id,"stand-again");
  assert(svc.command(st,r.p.now/1000).outcome==motion::CommandOutcome::ACCEPTED);
  for(unsigned n=0;n<6000&&r.m.state()==simple::State::MOVING;++n)r.tick();svc.update();assert(svc.captureStatus(s,r.p.now/1000));
  assert(r.m.state()==simple::State::STANDING&&!strcmp(s.activeId,"stand-again")&&!strcmp(s.activeOutcome,"COMPLETED"));
  puts("PASS service: PRONE COMPLETED, WALK STOP rejected NOT_WALKING, STAND accepted and COMPLETED");}
 puts("PRONE PASS: 3 heights (STAND->PRONE->STAND->PRONE), 23 measured-residual scenarios, 30 STOP/fault cases, STOP in TX, 600 s virtual terminal; IMPLEMENTED NOT VALIDATED");
}
