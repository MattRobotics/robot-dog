// Included by run_version_repro.sh into test_simple_motion.cpp of the revision under test (its Rig).
// Version-independent repro probes (public API + gait/walkNominal probes).
namespace repro {
using P=simple::StopReferenceProbe;
struct Walk{unsigned stops=0,faults=0;bool walking=false;double maxY=0,lo=1e9,hi=-1e9;double pp()const{return hi>lo?.5*(hi-lo):0;}};
static void step(Rig& r,Walk& w,double vx,double vy,double yaw,unsigned n){
  if(n%12==0)r.m.drive(vx,vy,yaw,r.p.now);r.tick();
  const bool on=r.m.state()==simple::State::WALKING;if(w.walking&&!on)++w.stops;w.walking=on;if(r.m.state()==simple::State::FAULT)++w.faults;
  if(on){const double y=P::walkNominal(r.m).cartesian.body.translationM.y;w.lo=std::fmin(w.lo,y);w.hi=std::fmax(w.hi,y);}}
static double S(){return simple::kJoystickStrideM/simple::kJoystickPeriodS;}
static double L(){return simple::kJoystickLateralM/simple::kJoystickPeriodS;}
// R1 dev.4 ground: forward stick with 3-13 % lateral touch jitter.
static void jitterForward(){Rig r;r.m.stand(.15,r.p.now);r.reachStand();const double j[]={.03,.06,-.09,.13,-.03,.09};Walk w;
  const double vx=.5*S();for(unsigned n=0;n<1300;++n)step(r,w,vx,j[(n/12)%6]*L(),0,n);w.lo=1e9;w.hi=-1e9;
  for(unsigned n=1300;n<1300+325;++n)step(r,w,vx,j[(n/12)%6]*L(),0,n);
  std::printf("REPRO R1 jitter-forward: sway=%.1f mm lateral excursion (half p-p, last cycle)=%.2f mm stops=%u faults=%u state=%s\n",P::gait(r.m).swayYM*1e3,w.pp()*1e3,w.stops,w.faults,simple::name(r.m.state()));}
// R2 review 09/11: steer, then forward with the same jitter.
static void jitterAfterSteer(){Rig r;r.m.stand(.15,r.p.now);r.reachStand();const double j[]={.03,.06,-.09,.13,-.03,.09};Walk w;
  for(unsigned n=0;n<1500;++n)step(r,w,.5*S(),.7*L(),0,n);
  for(unsigned n=1500;n<4000;++n)step(r,w,.5*S(),j[(n/12)%6]*L(),0,n);w.lo=1e9;w.hi=-1e9;
  for(unsigned n=4000;n<4325;++n)step(r,w,.5*S(),j[(n/12)%6]*L(),0,n);
  std::printf("REPRO R2 jitter-after-steer: sway=%.1f mm lateral excursion (half p-p, last cycle)=%.2f mm vy=%.2f mm/s stops=%u faults=%u\n",P::gait(r.m).swayYM*1e3,w.pp()*1e3,r.m.walkVelocityY()*1e3,w.stops,w.faults);}
// R3 held FORWARD (WALK STANDARD 100 speed where published) -> DIAGONAL -> FORWARD.
static void lawChange(){Rig r;r.m.stand(.15,r.p.now);r.reachStand();Walk w;const double fx=simple::kStrideM[2]/simple::kWalkPeriodS;
  const char* first=nullptr;
  for(unsigned n=0;n<1500;++n){if(n%12==0){const char* e=r.m.drive(fx,0,0,r.p.now);if(e&&!first)first=e;}r.tick();}
  if(first)std::printf("REPRO R3 note: forward %.1f mm/s refused (%s), using the steered range\n",fx*1e3,first);
  const double f=first?.9*S():fx;Walk w1;for(unsigned n=0;n<1500;++n)step(r,w1,f,0,0,n);const double v0=r.m.walkVelocityX();
  Walk w2;for(unsigned n=0;n<2000;++n)step(r,w2,.7*S(),.7*L(),0,n);const double vyD=r.m.walkVelocityY();
  Walk w3;for(unsigned n=0;n<4000;++n)step(r,w3,f,0,0,n);
  std::printf("REPRO R3 law-change: fwd vx=%.2f; diag stops=%u vy=%.2f; back-to-fwd stops=%u vx=%.2f sway=%.1f mm (demand %.2f mm/s) faults=%u\n",v0*1e3,w2.stops,vyD*1e3,w3.stops,r.m.walkVelocityX()*1e3,P::gait(r.m).swayYM*1e3,f*1e3,w1.faults+w2.faults+w3.faults);}
// R4 long WALK STANDARD 100.
static void longWalk(){Rig r;r.m.stand(.15,r.p.now);r.reachStand();r.m.walk(2,r.p.now);unsigned n=0;
  for(;n<8000&&r.m.state()!=simple::State::FAULT;++n)r.tick();
  std::printf("REPRO R4 long WALK100: releases=%u cycles=%.2f state=%s reason=%s\n",n,r.m.walkCycles(),simple::name(r.m.state()),r.m.reason());}
}
