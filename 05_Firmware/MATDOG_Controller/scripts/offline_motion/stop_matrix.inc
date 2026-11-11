// STOP matrices under the servo_plant model (vNext audit, 2026-11-11), included
// after servo_plant.inc by run_stop_matrix.sh. Software only, not a hardware result.
//  window MODE SEEDS BACK: a walk at the mode's top preset (BACK=1: the joystick
//    backward at its limit), one STOP per run at each of 14 releases from the
//    start of a four-contact window (the sway transfer), the window chosen by the
//    seed (quarter 0-3 of cycle 3); every attempt counts, each failure is listed.
//  handover MODE DIR SEEDS: the Codex audit replay of a joystick regime change
//    (DIR 0 forward -> lateral, 1 lateral -> forward, 2 forward -> mixed, 3 mixed
//    -> forward) under six board attitudes; a STOP the release the regime
//    changes. A trial whose handover never came is listed, not counted as a STOP.
namespace stopmatrix {
using simple::State;using simple::WalkMode;
static int window(int mode,unsigned seeds,bool back){
  const auto m=static_cast<WalkMode>(mode);const auto& w=simple::walkModeSpec(m);const double rel=.02/w.periodS;unsigned tried=0,ok=0;
  for(double volt:{11.1,9.6})for(unsigned s=1;s<=seeds;++s)for(unsigned k=0;k<14;++k){
    std::mt19937 g(s*7919+77+mode);Rig r;if(r.m.stand(.15,r.p.now))return 2;const auto prm=plant::draw(g,volt);plant::Model p(r,prm);
    for(unsigned n=0;n<4000&&r.m.state()!=State::STANDING&&r.m.state()!=State::FAULT;++n)p.tick();
    if(r.m.state()!=State::STANDING){std::printf("  %.1fV seed %u: NO_STANDING\n",volt,s);++tried;continue;}
    r.m.selectWalkMode(m);plant::tilt(r,.004,.006);
    const double vb=-simple::joystickSpec(m).backM/w.periodS;if(!back&&r.m.walk(2,r.p.now))return 2;
    const double target=3+(s%4)*.25+k*rel;bool stopped=false;
    for(unsigned n=0;n<9000&&r.m.state()!=State::FAULT;++n){
      if(back&&n%12==0&&!stopped)r.m.drive(vb,0,0,r.p.now);
      p.tick();
      if(!stopped&&r.m.state()==State::WALKING&&r.m.walkCycles()>=target){++tried;r.m.stop();stopped=true;}
      if(stopped&&(r.m.stopVerified()||r.m.state()==State::FAULT))break;}
    if(!stopped){std::printf("  %.1fV seed %u k %u: STOP_NOT_ATTEMPTED\n",volt,s,k);++tried;continue;}
    if(r.m.stopVerified())++ok;else std::printf("  %.1fV seed %u (flex %.3f) k %u c %.3f: %s\n",volt,s,prm.sagScale,k,r.m.exitSnapshot().cycles,r.m.reason());}
  std::printf("STOP_WINDOW %s %s: %u/%u STOPs verified (%u seeds x 2 V x 14 releases, window %.3f cycles)\n",w.name,back?"backward":"forward",ok,tried,seeds,w.duty-.75);
  return ok==tried?0:1;
}
static int handover(int mode,int dir,unsigned seeds){
  const auto m=static_cast<WalkMode>(mode);const auto& w=simple::walkModeSpec(m);const auto& j=simple::joystickSpec(m);unsigned stops=0,ok=0,none=0;
  for(double volt:{11.1,9.6})for(unsigned s=1;s<=seeds;++s){
    std::mt19937 g(s*7919+31*8+mode);Rig r;if(r.m.stand(.15,r.p.now))return 2;plant::Model p(r,plant::draw(g,volt));
    for(unsigned k=0;k<4000&&r.m.state()!=State::STANDING&&r.m.state()!=State::FAULT;++k)p.tick();
    if(r.m.state()!=State::STANDING)return 2;r.m.selectWalkMode(m);
    const double h[]={.004,-.012,0,.018,-.010,0},q[]={.006,.012,.031,-.026,-.036,0};plant::tilt(r,h[(s-1)%6],q[(s-1)%6]);
    const bool startSteer=dir%2;double a[3]={startSteer?0:w.strideM[2]/w.periodS,startSteer?j.lateralM/w.periodS:0,0},b[3]={startSteer?w.strideM[2]/w.periodS:0,startSteer?0:j.lateralM/w.periodS,0};
    if(dir==2){b[0]=.4*j.strideM/w.periodS;b[1]=-.66*j.lateralM/w.periodS;b[2]=.5*j.yawRad/w.periodS;}
    if(dir==3){a[0]=.55*j.strideM/w.periodS;a[1]=.4*j.lateralM/w.periodS;a[2]=.6*j.yawRad/w.periodS;}
    bool handed=false;
    for(unsigned k=0;k<3400&&r.m.state()!=State::FAULT&&!handed;++k){
      const bool later=k>=1250+5*s;const double* v=later?b:a;
      if(k%12==0)r.m.drive(v[0],v[1],v[2],r.p.now);
      const bool before=r.m.walkSteered();p.tick();
      if(later&&r.m.state()==State::WALKING&&before!=r.m.walkSteered()){handed=true;++stops;const double c=r.m.walkCycles();r.m.stop();
        for(unsigned n=0;n<600&&!r.m.stopVerified()&&r.m.state()!=State::FAULT;++n)p.tick();
        if(r.m.stopVerified())++ok;else std::printf("  %.1fV seed %u: STOP at the handover c %.3f -> %s\n",volt,s,c,r.m.reason());}}
    if(!handed){++none;std::printf("  %.1fV seed %u: no handover (%s, %s)\n",volt,s,simple::name(r.m.state()),r.m.reason());}}
  std::printf("STOP_HANDOVER %s dir %d: %u/%u STOPs verified at the handover, %u trials without handover\n",w.name,dir,ok,stops,none);
  return ok==stops?0:1;
}
}
