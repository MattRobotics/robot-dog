// Completion under a static load (RCA 28/10). Scripted observations, NOT a
// motor model: a "loaded" joint rests short of its goal by a fixed number of
// ticks with PresentLoad driven toward the goal (8 permille/tick, as measured
// on the robot). Positive and negative cases; no hardware qualification.
// Reuses the unchanged scripted harness of test_simple_motion.cpp.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wreturn-type"
#define main simple_motion_main
#include "test_simple_motion.cpp"
#undef main
#pragma GCC diagnostic pop
struct LoadRig {
  simple::SimpleMotion m;ScriptedObservationPort p;simple::Inputs in;
  int res[12]{},perTick[12]{},sign[12]{},speed[12]{};bool shaping=true;
  std::vector<std::pair<std::string,uint64_t>> phases;
  LoadRig(){
    m.begin(&p);p.motion=&m;in.ready=in.powerOk=in.authorityOk=in.imuValid=true;in.refusal="READY";
    const double lo[12]={-.783864,-.914253,-1.598408,-.783864,-.914253,-1.58,-.783864,-.914253,-1.592272,-.783864,-.914253,-1.590738};
    const double hi[12]={.783864,2.110758,.590583,.783864,2.10002,.581379,.771592,2.136835,.590583,.78233,2.129166,.585981};
    for(unsigned i=0;i<12;++i){in.minRad[i]=lo[i];in.maxRad[i]=hi[i];perTick[i]=8;sign[i]=1;}
    // Body pitched like the PRONE route head (0.0968 rad): the IMU correction
    // is then ~0 and the DISENGAGE goal equals the robot's (-1.5446 rad).
    in.quaternion[0]=std::cos(.0968/2);in.quaternion[2]=std::sin(.0968/2);
    // Folded PRONE contact measured on the robot (LOWER outside the route head).
    const double folded[4]={-1.6015,-1.5831,-1.5969,-1.5953};
    for(unsigned l=0;l<4;++l){auto& f=p.observed[l*3+2];f.q=folded[l];f.raw=2048+std::lround(folded[l]/simple::kTickRad);f.goal=f.raw;p.target[l*3+2]=f.raw;}
    for(unsigned i=0;i<15;++i)tick(false);assert(m.offVerified());
  }
  void shape(){
    const std::string ph=m.phase();
    if(!shaping||ph=="IDLE"||ph=="ACQUIRE"||ph=="REFERENCE"||ph=="ENABLE")return;
    for(unsigned i=0;i<12;++i){auto& f=p.observed[i];f.speed=speed[i];
      if(!res[i]){f.load=0;continue;}
      f.raw=static_cast<uint16_t>(int(p.target[i])-res[i]);f.q=(int(f.raw)-2048)*simple::kTickRad;
      const int mag=std::min(1000,perTick[i]*std::abs(res[i]));const bool towardGoal=sign[i]>0;
      f.load=mag|(((res[i]>0)==towardGoal)?1024:0);}
  }
  void tick(bool publish=true){
    p.now+=simple::kPeriodUs;in.imuStampUs=p.now;m.inputs(in);
    if(publish){p.publishCommandedPose();shape();}
    test_clock_us=p.now;m.update(p.now);m.updateTelemetry(p.now);
    if(phases.empty()||phases.back().first!=m.phase())phases.push_back({m.phase(),p.now});
  }
  bool run(unsigned n,simple::State until){for(unsigned i=0;i<n;++i){tick();if(m.state()==until||m.state()==simple::State::FAULT)break;}return m.state()==until;}
  uint64_t at(const char* phase) const {for(auto& x:phases)if(x.first==phase)return x.second;return 0;}
  std::string trail() const {std::string s;for(auto& x:phases)s+=x.first+">";return s;}
};
static void lowers(LoadRig& r,int lf,int rf,int rh,int lh){r.res[2]=lf;r.res[5]=rf;r.res[8]=rh;r.res[11]=lh;}
static int checks=0;
#define CHECK(c,msg) do{++checks;if(!(c)){std::printf("FAIL %s | trail=%s reason=%s\n",msg,r.trail().c_str(),r.m.reason());std::exit(1);}}while(0)
#ifndef STAND_LOAD_NO_MAIN  // test_stand_profile_timing.cpp reuses the rig
int main(){
  // 1. Measured DISENGAGE stall (24/3/21/19 ticks short, driven toward goal):
  //    completes DISENGAGE and ALIGN under load; the rest of the route arrives.
  {LoadRig r;lowers(r,24,3,21,19);
    CHECK(!r.m.stand(.05,r.p.now),"STAND accepted");
    r.run(1000,simple::State::MOVING);
    // Stop shaping once DEPLOY starts: later phases arrive normally here.
    unsigned n=0;while(r.m.state()==simple::State::MOVING&&std::string(r.m.phase())!="STAND_DEPLOY"&&n++<3000)r.tick();
    CHECK(std::string(r.m.phase())=="STAND_DEPLOY","DISENGAGE and ALIGN completed under the measured load");
    const uint64_t dis=r.at("DISENGAGE"),ali=r.at("ALIGN"),dep=r.at("STAND_DEPLOY");
    CHECK(ali-dis>=600000,"DISENGAGE waited for the 500 ms stationary dwell after the 100 ms settle");
    CHECK(dep-ali>=600000,"ALIGN waited for the dwell");
    CHECK(!strcmp(r.m.loadedPhase(),"ALIGN")&&r.m.loadedMask()==0x904,"loaded joints recorded: slots 2,8,11");
    r.shaping=false;CHECK(r.run(4000,simple::State::STANDING),"STAND50 reached");
    std::printf("PASS measured DISENGAGE load: trail %s\n",r.trail().c_str());}
  // 2. Every joint loaded through DEPLOY and LIFT (12 ticks + 6 short): STAND50.
  {LoadRig r;lowers(r,24,3,21,19);
    CHECK(!r.m.stand(.05,r.p.now),"STAND accepted");unsigned n=0;
    while(r.m.state()==simple::State::MOVING&&std::string(r.m.phase())!="STAND_DEPLOY"&&n++<3000)r.tick();
    for(unsigned i=0;i<12;++i)r.res[i]=(i%3==0)?0:18;
    CHECK(r.run(6000,simple::State::STANDING),"STAND50 reached with DEPLOY and LIFT loaded");
    CHECK(!strcmp(r.m.loadedPhase(),"STAND_LIFT"),"LIFT completed under load");
    std::printf("PASS loaded DEPLOY/LIFT: trail %s\n",r.trail().c_str());}
  // Negative cases: the residual stays, the segment is NOT completed and the
  // unchanged deadline faults NATIVE_TARGET_NOT_REACHED in that phase.
  auto timeout=[](const char* name,void(*setup)(LoadRig&),const char* phase){
    LoadRig r;setup(r);CHECK(!r.m.stand(.05,r.p.now),"STAND accepted");
    r.run(3000,simple::State::FAULT);
    CHECK(r.m.state()==simple::State::FAULT&&!strcmp(r.m.reason(),"NATIVE_TARGET_NOT_REACHED"),name);
    CHECK(r.m.exitSnapshot().valid&&!strcmp(r.m.exitSnapshot().phase,phase),name);
    std::printf("PASS %s -> NATIVE_TARGET_NOT_REACHED in %s\n",name,phase);};
  timeout("loaded joint near overload (24 ticks, 816 permille)",[](LoadRig& r){lowers(r,24,3,21,19);r.perTick[2]=34;},"DISENGAGE");
  timeout("arrived joint above 400 permille (5 ticks, 500 permille)",[](LoadRig& r){lowers(r,24,5,21,19);r.perTick[5]=100;},"DISENGAGE");
  timeout("no drive (PresentLoad 0)",[](LoadRig& r){lowers(r,24,3,21,19);r.perTick[2]=0;},"DISENGAGE");
  timeout("drive away from the goal",[](LoadRig& r){lowers(r,24,3,21,19);r.sign[8]=-1;},"DISENGAGE");
  timeout("drive too weak for the residual (2/tick)",[](LoadRig& r){lowers(r,24,3,21,19);r.perTick[11]=2;},"DISENGAGE");
  timeout("residual above 48 ticks",[](LoadRig& r){lowers(r,24,3,21,19);r.res[5]=-50;},"DISENGAGE");
  timeout("hip holding reference off by 20 (not a DISENGAGE joint)",[](LoadRig& r){lowers(r,24,3,21,19);r.res[0]=20;},"DISENGAGE");
  timeout("never stationary (speed 120)",[](LoadRig& r){lowers(r,24,3,21,19);r.speed[2]=120;},"DISENGAGE");
  timeout("lower outside the joint limit (LF 36 ticks short)",[](LoadRig& r){lowers(r,36,3,21,19);},"DISENGAGE");
  timeout("flat IMU: corrected LF goal leaves the actual 1 tick outside kDeploy",[](LoadRig& r){lowers(r,24,3,21,19);r.in.quaternion[0]=1;r.in.quaternion[2]=0;},"ALIGN");
  timeout("actual outside kDeploy for the next segment (RH 30 short)",[](LoadRig& r){lowers(r,24,3,30,19);r.in.minRad[8]=-1.62;},"ALIGN");
  // 2b. Highest accepted drive: 24 ticks x 16 = 384 permille <= 400.
  {LoadRig r;lowers(r,24,3,21,19);r.perTick[2]=16;CHECK(!r.m.stand(.05,r.p.now),"STAND accepted");unsigned n=0;
    while(r.m.state()==simple::State::MOVING&&std::string(r.m.phase())!="STAND_DEPLOY"&&n++<3000)r.tick();
    CHECK(std::string(r.m.phase())=="STAND_DEPLOY","384 permille accepted");
    std::printf("PASS 384 permille accepted\n");}
  // 2c. Hardware-like 1-LSB jitter on a loaded joint does not block completion.
  {LoadRig r;lowers(r,24,3,21,19);CHECK(!r.m.stand(.05,r.p.now),"STAND accepted");unsigned n=0;
    while(r.m.state()==simple::State::MOVING&&std::string(r.m.phase())!="ALIGN"&&n++<3000){r.res[8]=21+int(n&1);r.tick();}
    CHECK(std::string(r.m.phase())=="ALIGN","1-tick jitter tolerated");
    std::printf("PASS 1-LSB jitter tolerated\n");}
  // 2d. Slow creep with speed 0: +1 tick every 200 ms never satisfies the
  //     RAW dwell; completion only >= 500 ms after the creep stops.
  {LoadRig r;lowers(r,14,3,21,19);CHECK(!r.m.stand(.05,r.p.now),"STAND accepted");unsigned n=0;
    while(std::string(r.m.phase())!="DISENGAGE"&&n++<500)r.tick();
    for(unsigned i=0;i<150;++i){r.res[2]=14+int(i/10);r.tick();}
    CHECK(std::string(r.m.phase())=="DISENGAGE"&&r.m.state()==simple::State::MOVING,"no completion while creeping at speed 0");
    const uint64_t stopped=r.p.now;
    while(std::string(r.m.phase())=="DISENGAGE"&&n++<3000)r.tick();
    CHECK(std::string(r.m.phase())=="ALIGN"&&r.at("ALIGN")-stopped>=500000-120000,"completes only after a full still dwell");
    std::printf("PASS slow creep at speed 0 blocks completion\n");}
  // 3. Intermittent motion restarts the dwell.
  {LoadRig r;lowers(r,24,3,21,19);CHECK(!r.m.stand(.05,r.p.now),"STAND accepted");unsigned n=0;
    while(std::string(r.m.phase())!="DISENGAGE"&&n++<500)r.tick();
    const uint64_t t0=r.p.now;
    // Moving 7 of every 15 releases: each window spans a fast read of slot 2.
    for(unsigned i=0;i<60;++i){r.speed[2]=(i%15<7)?120:0;r.tick();}
    r.speed[2]=0;while(std::string(r.m.phase())=="DISENGAGE"&&n++<3000)r.tick();
    CHECK(std::string(r.m.phase())=="ALIGN","completes once stationary");
    CHECK(r.at("ALIGN")-t0>=52*simple::kPeriodUs+500000-120000,"dwell restarted after the last observed motion");
    std::printf("PASS motion restarts the dwell\n");}
  // 4. Stale feedback during the dwell: fault, never a completion.
  {LoadRig r;lowers(r,24,3,21,19);CHECK(!r.m.stand(.05,r.p.now),"STAND accepted");unsigned n=0;
    while(std::string(r.m.phase())!="DISENGAGE"&&n++<500)r.tick();
    for(unsigned i=0;i<10;++i)r.tick();r.p.missing=true;
    r.run(200,simple::State::FAULT);
    CHECK(r.m.state()==simple::State::FAULT&&r.at("ALIGN")==0,"stale feedback faults before completion");
    std::printf("PASS stale feedback -> %s, no ALIGN\n",r.m.reason());}
  // 5. Native alarm bit during the dwell.
  {LoadRig r;lowers(r,24,3,21,19);CHECK(!r.m.stand(.05,r.p.now),"STAND accepted");unsigned n=0;
    while(std::string(r.m.phase())!="DISENGAGE"&&n++<500)r.tick();
    for(unsigned i=0;i<10;++i)r.tick();r.p.observed[8].status=0x20;
    for(unsigned i=0;i<20&&r.m.state()!=simple::State::FAULT;++i)r.tick(false);
    CHECK(!strcmp(r.m.reason(),"SERVO_NATIVE_ALARM_OR_ELECTRICAL_LIMIT")&&r.at("ALIGN")==0,"alarm faults before completion");
    CHECK(r.m.exitSnapshot().fb[8].status==0x20,"alarm joint frozen in the exit snapshot");
    std::printf("PASS alarm during dwell\n");}
  // 6. STOP during the dwell: SAFE_OFF, no next segment.
  {LoadRig r;lowers(r,24,3,21,19);CHECK(!r.m.stand(.05,r.p.now),"STAND accepted");unsigned n=0;
    while(std::string(r.m.phase())!="DISENGAGE"&&n++<500)r.tick();
    for(unsigned i=0;i<15;++i)r.tick();r.m.stop();
    for(unsigned i=0;i<30;++i)r.tick(false);
    CHECK(r.m.state()==simple::State::STOPPED&&r.m.offVerified()&&r.at("ALIGN")==0,"STOP wins over completion");
    CHECK(!strcmp(r.m.exitSnapshot().phase,"DISENGAGE")&&!r.m.exitSnapshot().fault,"STOP recorded in DISENGAGE");
    std::printf("PASS STOP during dwell\n");}
  // 8. LIFT -> HEIGHT (rc.10): the 28/10 HOLD pose had LF_LOWER 7 ticks below
  //    kHeight (26-tick loaded sag). Reproduce it at the lift top for STAND100
  //    and STAND150; STAND50 (LIFT -> HOLD) is unchanged.
  auto liftSag=[](LoadRig& r,int lf,int hip){unsigned n=0;
    while(r.m.state()==simple::State::MOVING&&std::string(r.m.phase())!="STAND_LIFT"&&n++<4000)r.tick();
    for(unsigned i=0;i<12;++i)r.res[i]=0;r.res[2]=lf;r.res[5]=16;r.res[8]=10;r.res[0]=hip;};
  for(double h:{.10,.15,.05}){LoadRig r;lowers(r,24,3,21,19);
    CHECK(!r.m.stand(h,r.p.now),"STAND accepted");liftSag(r,26,0);
    CHECK(r.run(8000,simple::State::STANDING),"STAND reached with the measured LIFT sag");
    CHECK(std::abs(r.m.requestedHeightM()-h)<1e-9&&std::string(r.m.phase())=="HOLD","HOLD at the requested height");
    if(h>.05)CHECK(r.at("HEIGHT")>r.at("STAND_LIFT"),"HEIGHT followed LIFT");
    std::printf("PASS STAND%.0f with LF_LOWER 26-tick lift-top sag: trail %s\n",h*1000,r.trail().c_str());}
  {LoadRig r;lowers(r,24,3,21,19);CHECK(!r.m.stand(.10,r.p.now),"STAND accepted");liftSag(r,26,20);
    r.run(4000,simple::State::FAULT);
    CHECK(r.m.state()==simple::State::FAULT&&!strcmp(r.m.reason(),"NATIVE_TARGET_NOT_REACHED")&&!strcmp(r.m.exitSnapshot().phase,"STAND_LIFT"),"hip outside kHeight that HEIGHT does not move back: refused");
    std::printf("PASS hip 20 ticks outside kHeight (HEIGHT does not move it back) -> NATIVE_TARGET_NOT_REACHED in STAND_LIFT\n");}
  std::printf("STAND_LOAD_COMPLETION PASS checks=%d (scripted observations; NO HARDWARE MOTION VALIDATION)\n",checks);
  return 0;
}
#endif
