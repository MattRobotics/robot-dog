// FAST preset walk, then a slower selection under a board tilt: does the law shape switch?
int main(int argc,char** argv){
  using simple::State;using simple::WalkMode;
  const WalkMode to=static_cast<WalkMode>(atoi(argv[1]));
  for(double roll:{0.,.004,.008,-.008})for(double pitch:{0.,.006,.010,-.010}){
    Rig r;assert(!r.m.stand(.105,r.p.now));assert(r.reachStand());assert(!r.m.selectWalkMode(WalkMode::FAST));
    plant::tilt(r,roll,pitch);
    assert(!r.m.walk(2,r.p.now));unsigned n=0;
    for(;n<3000&&!(r.m.state()==State::WALKING&&r.m.walkCycles()>4);++n)r.tick();
    if(r.m.state()!=State::WALKING){printf("roll %+.3f pitch %+.3f: no walk (%s)\n",roll,pitch,r.m.reason());continue;}
    if(argc>2){assert(!r.m.selectWalkMode(WalkMode::SLOW));for(unsigned j=0;j<1000;++j)r.tick();}
    assert(!r.m.selectWalkMode(to));unsigned k=0;
    for(;k<4000&&r.m.walkMode()!=to&&r.m.state()==State::WALKING;++k)r.tick();
    printf("roll %+.3f pitch %+.3f: FAST->%s %s after %.1f s (state %s, imu %s applied %.1f/%.1f)\n",roll,pitch,simple::walkModeSpec(to).name,
      r.m.walkMode()==to?"switched":"NOT switched",k*.02,simple::name(r.m.state()),r.m.imuReason(),r.m.imuApplied(0)*1e3,r.m.imuApplied(1)*1e3);
  }
}
