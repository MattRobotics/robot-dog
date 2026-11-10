// RCA trace (38): joystick forward -> lateral -> backward reversal held, restart at STANDING.
#define main simple_motion_main
#include "test_simple_motion.cpp"  // build: g++ -std=c++17 -O2 -w -I scripts/tests/simple_host_stub -I scripts/tests <this> + run_simple_motion_tests.sh sources
#undef main
#include <cstdlib>
struct Run {
  Rig r;bool lag;double pitchDeg,rollDeg;std::string last;unsigned seq=0;
  Run(bool lagPlant,double pitch,double roll):lag(lagPlant),pitchDeg(pitch),rollDeg(roll){}
  double qv[12]{};bool init=false;
  void tick(){
    constexpr double d=3.14159265358979323846/180;const double hp=pitchDeg*d/2,hr=rollDeg*d/2;
    // body tilt quaternion (roll then pitch), constant: open loop IMU offset/board
    const double cr=std::cos(hr),sr=std::sin(hr),cp=std::cos(hp),sp=std::sin(hp);
    r.in.quaternion[0]=cr*cp;r.in.quaternion[1]=sr*cp;r.in.quaternion[2]=cr*sp;r.in.quaternion[3]=-sr*sp;
    if(!lag){r.tick();}
    else{
      if(!init){for(unsigned i=0;i<12;++i)qv[i]=r.p.observed[i].q;init=true;}
      for(unsigned i=0;i<12;++i){const double t=(int(r.p.target[i])-2048)*simple::kTickRad;qv[i]+=.2*(t-qv[i]);
        static const int sag[12]={3,2,1,0,-3,-3,-3,6,2,-8,-5,-2};  // trace (38): raw - commanded after the fault
        r.p.observed[i].raw=2048+std::lround(qv[i]/simple::kTickRad)+(r.m.energized()&&std::getenv("SAG")?sag[i]:0);r.p.observed[i].q=(int(r.p.observed[i].raw)-2048)*simple::kTickRad;}
      r.tick(false);
    }
    char b[256];std::snprintf(b,sizeof b,"%s/%s/%s imu=%s",simple::name(r.m.state()),r.m.phase(),r.m.reason(),r.m.imuReason());
    if(last!=b){std::printf("  t=%7.2f s %-60s corr r/p=%+.4f/%+.4f wanted=%+.4f/%+.4f cyc=%.3f\n",r.p.now/1e6,b,simple::StopReferenceProbe::roll(r.m),simple::StopReferenceProbe::pitch(r.m),r.m.imuWanted(0),r.m.imuWanted(1),r.m.walkCycles());last=b;}
  }
  const char* drive(double fx,double ly){const double T=simple::kJoystickPeriodS;return r.m.drive(fx*simple::kJoystickStrideM/T,ly*simple::kJoystickLateralM/T,0,r.p.now);}
};
int main(int argc,char** argv){
  const bool lag=argc>1&&atoi(argv[1]);const double pitch=argc>2?atof(argv[2]):0,roll=argc>3?atof(argv[3]):0;
  Run x(lag,pitch,roll);auto& r=x.r;
  std::printf("lag=%d imu pitch=%.2f roll=%.2f deg\n",lag,pitch,roll);
  assert(!r.m.stand(.12,r.p.now));for(unsigned n=0;n<4000&&r.m.state()!=simple::State::STANDING;++n){x.tick();if(r.m.state()==simple::State::FAULT)return 1;}
  for(unsigned n=0;n<200;++n)x.tick();
  auto hold=[&](double fx,double ly,double seconds,bool quiet=false){unsigned n=unsigned(seconds/.02);
    for(unsigned k=0;k<n;++k){if(k%12==0&&!quiet){const char* w=x.drive(fx,ly);if(w&&std::string(w)!=x.last)std::printf("    drive(%.2f,%.2f) -> %s\n",fx,ly,w);}x.tick();if(r.m.state()==simple::State::FAULT)return false;}return true;};
  const double s=std::sqrt(.5);
  bool ok=hold(1,0,12)&&hold(s,-s,7)&&hold(0,-1,7)&&hold(-.5,-.866,7);
  if(ok){std::printf("  -> back (-1,0)\n");const char* w=x.drive(-1,0);std::printf("    drive(-1,0) -> %s reason=%s\n",w?w:"accepted",r.m.reason());}
  // page silent until it sees STANDING (poll ~350 ms + tick 240 ms), then refreshes the held back intent
  unsigned seen=0;for(unsigned n=0;ok&&n<6000&&r.m.state()!=simple::State::FAULT;++n){x.tick();if(r.m.state()==simple::State::STANDING&&!seen)seen=n;if(seen&&n>=seen+30)break;}
  const int variant=argc>4?atoi(argv[4]):0;
  if(r.m.state()!=simple::State::FAULT){
    if(variant==0)hold(-1,0,25);
    else if(variant==1){hold(-1,0,.5);std::printf("  -> release (neutral) during restart\n");r.m.walkStop();hold(0,0,25,true);}
    else if(variant==2){hold(-1,0,.5);hold(-.5,-.866,1);hold(.7,-.7,1);hold(-1,0,25);}
    else if(variant==3){hold(-1,0,3.5);std::printf("  -> forward again (second reversal) right after the restart\n");hold(1,0,25);}
    else if(variant==4){hold(-1,0,1.2);std::printf("  -> release just after the walk starts\n");r.m.walkStop();hold(0,0,25,true);}
  }
  std::printf("END state=%s reason=%s phase=%s\n",simple::name(r.m.state()),r.m.reason(),r.m.phase());
  return r.m.state()==simple::State::FAULT?2:0;
}
