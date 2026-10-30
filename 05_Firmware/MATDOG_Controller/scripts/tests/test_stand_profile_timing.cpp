// STAND profile timing and target continuity (rc.10). Scripted observations:
// unloaded (servos reach every frame) and the 28/10 measured loads (LOWER
// residuals in DISENGAGE/ALIGN, lift-top sag). Prints per-phase durations of
// the Motion sequence and checks every transmitted frame against the planner
// ceilings. NO HARDWARE MOTION VALIDATION: real servo lag is not modelled.
#define STAND_LOAD_NO_MAIN
#include "test_stand_load_completion.cpp"
int main(){
  for(int loaded=0;loaded<2;++loaded)for(double h:{.05,.10,.15}){
    LoadRig r;if(loaded)lowers(r,24,3,21,19);
    CHECK(!r.m.stand(h,r.p.now),"STAND accepted");
    const uint64_t t0=r.p.now;unsigned n=0;std::vector<std::string> fp;
    while(r.m.state()==simple::State::MOVING&&n++<20000){
      if(loaded&&std::string(r.m.phase())=="STAND_LIFT"&&r.res[2]!=26){for(unsigned i=0;i<12;++i)r.res[i]=0;r.res[2]=26;r.res[5]=16;r.res[8]=10;}
      const size_t before=r.p.sent.size();r.tick();
      while(fp.size()<r.p.sent.size())fp.push_back(r.p.sent.size()>before?r.m.phase():"?");}
    CHECK(r.m.state()==simple::State::STANDING,"STAND reached");
    // Planned segments (DEPLOY/LIFT/HEIGHT): per-release step of each RAW goal.
    // Native DISENGAGE/ALIGN are single targets; the servo profile shapes them.
    double step=0,native=0;const auto& f=r.p.sent;
    auto planned=[](const std::string& p){return p=="STAND_DEPLOY"||p=="STAND_LIFT"||p=="HEIGHT"||p=="HOLD";};
    for(size_t k=1;k<f.size();++k)for(unsigned i=0;i<12;++i){const double d=std::abs(int(f[k][i])-int(f[k-1][i]))*simple::kTickRad;
      if(planned(fp[k])&&planned(fp[k-1]))step=std::fmax(step,d);else if(fp[k]=="DISENGAGE"||fp[k]=="ALIGN")native=std::fmax(native,d);}
    std::printf("%s STAND%3.0f total %5.2f s |",loaded?"loaded  ":"unloaded",h*1000,(r.at("HOLD")-t0)/1e6);
    for(size_t k=0;k+1<r.phases.size();++k){const auto& a=r.phases[k];if(a.first=="IDLE")continue;
      std::printf(" %s %.2f",a.first.c_str(),(r.phases[k+1].second-a.second)/1e6);}
    std::printf(" | planned step max %.4f rad/release (%.1f ticks), native jump max %.0f ticks\n",step,step/simple::kTickRad,native/simple::kTickRad);
    // Planner ceiling per release + IMU correction slew (0.020 rad/s) + 1 tick quantisation.
    CHECK(step<=simple::kStandSpeedRadS*simple::kPeriodUs/1e6+0.02*simple::kPeriodUs/1e6+simple::kTickRad,"planned frame step within the planner speed ceiling");
  }
  std::printf("STAND_PROFILE_TIMING PASS checks=%d speed=%.2f rad/s accel=%.2f rad/s^2 drive=%u/%u\n",checks,simple::kStandSpeedRadS,simple::kStandAccelerationRadS2,simple::kStandDrive.speed,simple::kStandDrive.acc);
  return 0;
}
