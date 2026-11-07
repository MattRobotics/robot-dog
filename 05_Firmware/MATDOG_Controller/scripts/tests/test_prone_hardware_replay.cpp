#define PRONE_HEIGHT_NO_MAIN
#include "test_prone_height_residuals.cpp"
#include "prone_hardware_fixture.h"
int main(){
 (void)checks;(void)lowers;
 unsigned reached=0,rejected=0,faults=0;
 for(const auto& c:hardwareCases){Replay r;r.in.quaternion[0]=1;r.in.quaternion[2]=0;assert(!r.m.stand(c.height/1000.,r.p.now));
  for(unsigned n=0;n<6000&&r.m.state()==simple::State::MOVING;++n)r.tick();assert(r.m.state()==simple::State::STANDING);
  r.sample=&c.residual;for(unsigned n=0;n<12;++n)r.tick();
  const char* admission=r.m.prone(r.p.now);
  if(admission){printf("REJECT,%d,%d,%s\n",c.height,c.ms,admission);++rejected;continue;}
  double mech[12],boxm[12];const char* mp[12],*bp[12];for(unsigned i=0;i<12;++i){mech[i]=boxm[i]=1;mp[i]=bp[i]="";}
  for(unsigned n=0;n<4000&&!r.m.proneRest()&&r.m.state()!=simple::State::FAULT;++n){r.tick();
   // From a hold at or below the handover height prone() admits the pose in kLift
   // (the phase name stays PRONE_HEIGHT for the pending release only).
   const char* ph=r.m.phase();const auto& box=!strcmp(ph,"PRONE_HEIGHT")?(c.height/1000.>simple::kProneHandoverM?simple::collision::kHeight:simple::collision::kLift):!strcmp(ph,"PRONE_LIFT")?simple::collision::kLift:!strcmp(ph,"PRONE_DEPLOY")?simple::collision::kDeploy:simple::collision::kEntry;
   for(unsigned i=0;i<12;++i){double q=r.m.feedback()[i].q;double a=std::fmin(q-r.in.minRad[i],r.in.maxRad[i]-q),b=std::fmin(q-box.lo[i],box.hi[i]-q);
    if(a<mech[i]){mech[i]=a;mp[i]=ph;}if(b<boxm[i]){boxm[i]=b;bp[i]=ph;}}
  }
  bool end=r.m.proneRest();reached+=end;faults+=!end;  // dev.15: the terminal is the PRONE rest
  if(end)for(unsigned i=0;i<12;++i){assert(mech[i]>4*simple::kTickRad);assert(boxm[i]>4*simple::kTickRad);}
  printf("RESULT,%d,%d,%s,%s\n",c.height,c.ms,end?"TERMINAL":"BLOCKED",r.m.reason());
  for(unsigned i=0;i<12;++i)printf("REAL_MARGIN,%d,%d,%u,%.6f,%s,%.6f,%s\n",c.height,c.ms,i,mech[i]/simple::kTickRad,mp[i],boxm[i]/simple::kTickRad,bp[i]);
  if(end){printf("REAL_END,%d,%d",c.height,c.ms);for(unsigned i=0;i<12;++i)printf(",%.12f",r.m.feedback()[i].q);puts("");
   for(unsigned n=0;n<30&&!r.m.offVerified();++n)r.tick(false);assert(r.m.offVerified()&&!r.m.energized()&&r.m.proneRest());}
  else {for(unsigned n=0;n<30;++n)r.tick(false);assert(r.m.offVerified());}
 }
 assert(reached==69&&rejected==0&&faults==0);
 printf("REPLAY_SUMMARY terminal=%u rejected=%u blocked=%u; NO PHYSICAL VALIDATION\n",reached,rejected,faults);
}
