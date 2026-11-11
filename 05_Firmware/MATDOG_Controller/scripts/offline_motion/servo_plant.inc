// ST3215 response model for the production rig (walk modes 2026-11-09):
// included by run_servo_plant.sh into test_simple_motion.cpp (its Rig).
// Each servo follows its latest goal with its own trapezoidal profile limited by
// the RAM Goal Speed / ACC of the frame that carried it and by a capability:
//  - speed: bench QC V6.1 no-load 2829 step/s at 11.1 V (16 units), scaled by
//    voltage, a per-unit factor and a load factor (stance legs slower);
//  - acceleration: the register ACC; bench QC shows ACC 50 followed (4 918
//    step/s^2 median) and an arrival braking of 8 400 (p10)-9 231 (median);
//    above ACC 50 nothing is measured, so the capability is drawn in
//    [6 000, 15 000] step/s^2 times the load factor;
//  - command latency 0-3 ms, a position deadband of 1-3 ticks (bench precision
//    MAE 2-2.75 ticks), static sag of the loaded joints (dev.15 pattern scaled).
// Braking (walk modes final 2026-11-09): apart from the acceleration, bounded by
// a braking capability brakeCap (default = accCap, the earlier model). With
// PLANT_BRAKE_LO/HI (step/s^2, no load) it is drawn in that range times the
// unit and load factors: the sensitivity of STOP to a braking below the bench
// one (QC position analysis: profile followed at 0.8-1.0x the register up to
// ACC 50, >= 10 000 at full speed, unloaded; loaded never measured).
// Feedback is the modelled encoder at the read. Parameter ranges, not one
// plant tuned to pass: every run draws them from its seed. Not a contact or
// body-dynamics model: the body is not simulated, only the 12 servo angles.
namespace plant {
using P=simple::StopReferenceProbe;
struct Param{double volt,load,accCap,latencyUs,deadband,sagScale;double unit[12];double brakeCap;};
struct Servo{double pos=0,vel=0,goal=0;double vmax=1000,amax=5000,bmax=5000;uint64_t applyUs=0;double nextGoal=0;double nextV=1000,nextA=5000,nextB=5000;};
struct Out{unsigned faults=0,stopsOk=0,stopsTried=0;const char* reason="";uint8_t refusedBy=0;double cycles=0,maxTrack=0,maxStopOvershoot=0,maxStopSettleS=0,maxStopBoxOut=0;unsigned walkReleases=0;uint64_t speedBound=0,moving=0;};
struct Model{
  Rig& r;Param prm;Servo s[12];std::array<uint16_t,12> lastTarget{};simple::Drive lastDrive{};size_t seen=0;
  static constexpr int sag[12]={-2,-1,-1,0,2,3,3,-4,-1,7,5,2};
  Model(Rig& rr,const Param& p):r(rr),prm(p){for(unsigned i=0;i<12;++i){s[i].pos=s[i].goal=s[i].nextGoal=r.p.target[i];}}
  uint64_t speedBound=0,moving=0;  // ms with |vel| at a capability below the register / ms moving
  void caps(unsigned i,simple::Drive d,double& v,double& a,double& b) const {
    const double noLoad=2829*prm.volt/11.1*prm.unit[i]*prm.load;
    // Torque, hence the acceleration and braking capability, also scales with the
    // bus voltage (stall current V/R); the earlier model scaled only the speed,
    // which the law never reached: 9.6 and 11.1 V gave identical tracking.
    const double volt=prm.volt/11.1;
    v=std::min(d.speed?double(d.speed):1e9,noLoad);a=std::min(d.acc?d.acc*100.:1e9,prm.accCap*prm.unit[i]*prm.load*volt);
    b=std::min(d.acc?d.acc*100.:1e9,prm.brakeCap*prm.unit[i]*prm.load*volt);}
  double reg[12]{};  // register speed of the latest frame, for the speed-bound count
  // New SYNC_WRITE frames since the last step take effect after the latency.
  void collect(){for(;seen<r.p.sent.size();++seen){const auto& f=r.p.sent[seen];const auto d=r.p.sentDrive[seen];
      for(unsigned i=0;i<12;++i){s[i].nextGoal=f[i];caps(i,d,s[i].nextV,s[i].nextA,s[i].nextB);reg[i]=d.speed?d.speed:1e9;s[i].applyUs=r.p.now+uint64_t(prm.latencyUs);}}}
  void advance(uint64_t fromUs,uint64_t toUs){
    const bool on=r.m.energized();
    for(uint64_t t=fromUs;t<toUs;t+=1000){const double dt=1e-3;
      for(unsigned i=0;i<12;++i){auto& x=s[i];if(x.applyUs&&t>=x.applyUs){x.goal=x.nextGoal;x.vmax=x.nextV;x.amax=x.nextA;x.bmax=x.nextB;x.applyUs=0;}
        if(!on||r.p.observed[i].torque!=1){x.vel=0;continue;}  // torque OFF: the joint stays where it is (no gravity model)
        const double d=x.goal-x.pos;
        if(std::abs(d)<=prm.deadband&&std::abs(x.vel)<x.amax*dt*2){x.vel=0;continue;}
        const double dir=d>0?1:-1,stopDist=x.vel*x.vel/(2*x.bmax);
        if(x.vel*dir>0&&std::abs(d)<=stopDist)x.vel-=dir*x.bmax*dt;   // brake onto the goal
        else x.vel=std::clamp(x.vel+dir*(x.vel*dir<0?x.bmax:x.amax)*dt,-x.vmax,x.vmax);  // moving away: braking
        if(std::abs(x.vel)>1){++moving;if(x.vmax<reg[i]&&std::abs(x.vel)>=x.vmax-1e-9)++speedBound;}
        const double step=x.vel*dt;if(std::abs(step)>=std::abs(d)&&x.vel*dir>0){x.pos=x.goal;x.vel=0;}else x.pos+=step;}}
    for(unsigned i=0;i<12;++i){auto& o=r.p.observed[i];const double sagTicks=on?sag[i]*prm.sagScale:0;
      const int raw=int(std::lround(s[i].pos-sagTicks));o.raw=uint16_t(raw);o.q=(raw-2048)*simple::kTickRad;o.voltage=int(prm.volt*10);o.speed=int(std::abs(s[i].vel))|(s[i].vel<0?0x8000:0);}}  // ST sign-magnitude
  void tick(){const uint64_t from=r.p.now;r.p.now+=simple::kPeriodUs;r.in.imuStampUs=r.p.now;r.m.inputs(r.in);
    collect();advance(from,r.p.now);test_clock_us=r.p.now;r.m.update(r.p.now);r.m.updateTelemetry(r.p.now);collect();}
  double tracking() const {double m=0;for(unsigned i=0;i<12;++i)m=std::max(m,std::abs(s[i].pos-double(r.p.target[i])));return m;}
};
static void tilt(Rig& r,double roll,double pitch){const double hr=roll/2,hp=pitch/2;
  r.in.quaternion[0]=std::cos(hr)*std::cos(hp);r.in.quaternion[1]=std::sin(hr)*std::cos(hp);r.in.quaternion[2]=std::cos(hr)*std::sin(hp);r.in.quaternion[3]=-std::sin(hr)*std::sin(hp);}
static Param draw(std::mt19937& g,double volt){std::uniform_real_distribution<double> u(0,1);Param p;p.volt=volt;
  p.load=.6+.35*u(g);p.accCap=6000+9000*u(g);p.latencyUs=3000*u(g);p.deadband=1+2*u(g);p.sagScale=.5+1.0*u(g);for(double& k:p.unit)k=.9+.15*u(g);
  static const char *blo=getenv("PLANT_BRAKE_LO"),*bhi=getenv("PLANT_BRAKE_HI");
  p.brakeCap=blo&&bhi?atof(blo)+(atof(bhi)-atof(blo))*u(g):p.accCap;return p;}
// scenario 0: long walk at the mode's top preset (4500 releases, 90 s) with a
// small board tilt; 1: the same with 8 STOP at spread instants, each verified,
// then the walk resumed (STAND recovery); 2: IMU at the mode's pitch-down vertex;
// 3 (2026-11-10): the selector moved four times during the walk (every mode
// change up and down), a STOP 1 s into the second and the fourth change, each
// verified, then the walk resumed.
static Out run(simple::WalkMode mode,int scenario,unsigned seed,double volt){
  std::mt19937 g(seed);Rig r;Out o;double boxOut=0;  // real joint beyond the WALK box during a STOP, ticks
  if(r.m.stand(.15,r.p.now)){o.faults=1;o.reason="STAND_REFUSED";return o;}
  const Param prm=draw(g,volt);Model pl(r,prm);
  static const bool trace=getenv("PLANT_TRACE")!=nullptr;std::deque<std::string> last;
  auto watch=[&]{
    if(trace&&r.m.state()==simple::State::WALKING){char b[200];std::snprintf(b,sizeof b,"  c%.3f %s wanted %.1f/%.1f applied %.1f/%.1f refused %u track %.0f",r.m.walkCycles(),r.m.imuReason(),
        r.m.imuWanted(0)*1e3,r.m.imuWanted(1)*1e3,r.m.imuApplied(0)*1e3,r.m.imuApplied(1)*1e3,unsigned(r.m.imuRefusedBy()),pl.tracking());last.push_back(b);if(last.size()>40)last.pop_front();}
    if(trace&&r.m.state()==simple::State::FAULT&&!o.faults){for(auto& l:last)std::puts(l.c_str());}
    if(r.m.state()==simple::State::FAULT&&!o.faults){o.faults=1;o.reason=r.m.reason();o.refusedBy=r.m.exitSnapshot().refusedBy;o.cycles=r.m.exitSnapshot().cycles;}};
  for(unsigned n=0;n<4000&&r.m.state()!=simple::State::STANDING&&!o.faults;++n){pl.tick();watch();}
  if(r.m.state()!=simple::State::STANDING){if(!o.faults){o.faults=1;o.reason="NO_STANDING";}return o;}
  const auto& w=simple::walkModeSpec(mode);r.m.selectWalkMode(mode);
  static const double fraction=getenv("VERTEX_FRACTION")?atof(getenv("VERTEX_FRACTION")):.9;
  if(scenario==2)tilt(r,0,fraction*(*w.tiltTop)[12].pitch);else tilt(r,.004,.006);
  if(r.m.walk(2,r.p.now)){o.faults=1;o.reason="WALK_REFUSED";return o;}
  for(unsigned n=0;n<4000&&r.m.state()!=simple::State::WALKING&&!o.faults;++n){pl.tick();watch();}
  const unsigned cycleReleases=unsigned(w.periodS/.02);
  for(unsigned n=0;n<4500&&!o.faults;++n){pl.tick();watch();if(r.m.state()==simple::State::WALKING){++o.walkReleases;o.maxTrack=std::max(o.maxTrack,pl.tracking());}
    if(scenario==3&&(n==300||n==1100||n==1900||n==2700))r.m.selectWalkMode(static_cast<simple::WalkMode>((unsigned(mode)+1+n/800)%3));
    if((scenario==1&&n>300&&n%(cycleReleases+cycleReleases/8+7)==0&&o.stopsTried<8)||(scenario==3&&(n==1150||n==2750))){++o.stopsTried;r.m.stop();const uint64_t t0=r.p.now;
      for(unsigned k=0;k<600&&!r.m.stopVerified()&&!o.faults;++k){pl.tick();watch();
        for(unsigned i=0;i<12;++i){const double q=(pl.s[i].pos-2048)*simple::kTickRad;boxOut=std::max(boxOut,std::max(simple::kWalkBox.lo[i]-q,q-simple::kWalkBox.hi[i])/simple::kTickRad);}}
      if(o.faults||!r.m.stopVerified())break;
      const auto& plan=P::stopPlan(r.m);double over=0;for(unsigned i=0;i<12;++i)over=std::max(over,std::abs(pl.s[i].pos-std::lround(plan.end[i]/simple::kTickRad)-2048));
      o.maxStopOvershoot=std::max(o.maxStopOvershoot,over);o.maxStopSettleS=std::max(o.maxStopSettleS,(r.p.now-t0)/1e6);
      o.maxStopBoxOut=std::max(o.maxStopBoxOut,boxOut);boxOut=0;
      if(r.m.walk(2,r.p.now)){o.faults=1;o.reason="RESUME_REFUSED";break;}
      for(unsigned k=0;k<4000&&r.m.state()!=simple::State::WALKING&&!o.faults;++k){pl.tick();watch();}
      if(r.m.state()==simple::State::WALKING)++o.stopsOk;}}
  o.speedBound=pl.speedBound;o.moving=pl.moving;return o;
}
// Motion vNext joystick scenarios (2026-11-11), on each mode's own law: 4: a steered
// sequence (lateral, yaw, diagonal, backward-lateral; 2 cycles each) with 8 STOP at
// spread instants, each verified, then STAND and the joystick again; 5: backward at
// the mode's full backward speed, same STOPs; 6: the steered sequence with the IMU
// at 0.9 of the steered envelope's pitch-down vertex; 7: backward with the selector
// moved every 20 s (the backward walk keeps its law; a resumed walk takes the
// selection), a STOP 1 s after the second and the fourth. A mode without backward
// (JoystickSpec backM 0) reports 5 and 7 NOT_APPLICABLE.
static Out runJoy(simple::WalkMode mode,int scenario,unsigned seed,double volt){
  std::mt19937 g(seed);Rig r;Out o;
  if(r.m.stand(.15,r.p.now)){o.faults=1;o.reason="STAND_REFUSED";return o;}
  const Param prm=draw(g,volt);Model pl(r,prm);
  auto watch=[&]{if(r.m.state()==simple::State::FAULT&&!o.faults){o.faults=1;o.reason=r.m.reason();o.refusedBy=r.m.exitSnapshot().refusedBy;o.cycles=r.m.exitSnapshot().cycles;}};
  for(unsigned n=0;n<4000&&r.m.state()!=simple::State::STANDING&&!o.faults;++n){pl.tick();watch();}
  if(r.m.state()!=simple::State::STANDING){if(!o.faults){o.faults=1;o.reason="NO_STANDING";}return o;}
  r.m.selectWalkMode(mode);
  auto lim=[](simple::WalkMode m,unsigned k){const auto& j=simple::joystickSpec(m);const double T=simple::walkModeSpec(m).periodS;
    const double v[4]={j.strideM/T,j.lateralM/T,j.yawRad/T,j.backM/T};return v[k];};
  if(scenario==6)tilt(r,0,.9*simple::steerTilt(mode)[4].pitch);else tilt(r,.004,.006);
  const unsigned cycleReleases=unsigned(simple::walkModeSpec(mode).periodS/.02);
  for(unsigned n=0;n<4500&&!o.faults;++n){
    if(scenario==7&&(n==1000||n==2000||n==3000||n==4000))r.m.selectWalkMode(static_cast<simple::WalkMode>((unsigned(mode)+1+n/1000)%3));
    const simple::WalkMode sel=r.m.selectedWalkMode();double v[3]={-lim(sel,3),0,0};
    if(scenario==4||scenario==6){const double s[6][3]={{0,1,0},{0,0,-1},{.7,-.7,0},{0,0,1},{-.6,.6,0},{0,-1,0}};const auto& d=s[(n/(2*cycleReleases))%6];
      for(unsigned k=0;k<3;++k)v[k]=d[k]*lim(sel,k);}
    if(n%12==0&&r.m.state()!=simple::State::FAULT&&!r.m.stopActive())r.m.drive(v[0],v[1],v[2],r.p.now);
    pl.tick();watch();if(r.m.state()==simple::State::WALKING){++o.walkReleases;o.maxTrack=std::max(o.maxTrack,pl.tracking());}
    const bool stopNow=scenario==7?(n==2050||n==4050):(n>300&&n%(cycleReleases+cycleReleases/8+7)==0&&o.stopsTried<8);
    if(stopNow&&r.m.state()==simple::State::WALKING){++o.stopsTried;r.m.stop();const uint64_t t0=r.p.now;double boxOut=0;const auto& box=P::box(r.m);
      for(unsigned k=0;k<600&&!r.m.stopVerified()&&!o.faults;++k){pl.tick();watch();
        for(unsigned i=0;i<12;++i){const double q=(pl.s[i].pos-2048)*simple::kTickRad;boxOut=std::max(boxOut,std::max(box.lo[i]-q,q-box.hi[i])/simple::kTickRad);}}
      if(o.faults||!r.m.stopVerified())break;
      o.maxStopSettleS=std::max(o.maxStopSettleS,(r.p.now-t0)/1e6);o.maxStopBoxOut=std::max(o.maxStopBoxOut,boxOut);
      if(r.m.stand(simple::walkModeSpec(r.m.walkMode()).heightM,r.p.now)){o.faults=1;o.reason="RESUME_REFUSED";break;}
      for(unsigned k=0;k<4000&&r.m.state()!=simple::State::STANDING&&!o.faults;++k){pl.tick();watch();}
      if(r.m.state()==simple::State::STANDING)++o.stopsOk;}}
  o.speedBound=pl.speedBound;o.moving=pl.moving;return o;
}
static void matrix(){
  const int onlyMode=getenv("MODE")?atoi(getenv("MODE")):-1,onlyScenario=getenv("SCENARIO")?atoi(getenv("SCENARIO")):-1;
  const char* sn[]={"WALK_90s","STOP8_RESUME","IMU_PITCH_VERTEX","MODE_CHANGES","JOY_STEER_STOP8","JOY_BACK_STOP8","JOY_STEER_IMU","JOY_BACK_MODES"};const unsigned seeds=getenv("SEEDS")?atoi(getenv("SEEDS")):6;
  for(unsigned m=0;m<3;++m)for(int sc=0;sc<8;++sc)for(double volt:{11.1,9.6}){if((onlyMode>=0&&int(m)!=onlyMode)||(onlyScenario>=0&&sc!=onlyScenario))continue;
    if((sc==5||sc==7)&&simple::joystickSpec(static_cast<simple::WalkMode>(m)).backM<=0){
      std::printf("PLANT %-8s %-16s %4.1fV NOT_APPLICABLE (no backward on this law)\n",simple::walkModeSpec(static_cast<simple::WalkMode>(m)).name,sn[sc],volt);continue;}unsigned f=0,stopsOk=0,stopsTried=0;double track=0,over=0,settle=0,boxOut=0;uint64_t sb=0,mv=0;std::string reasons;
    for(unsigned seed=1;seed<=seeds;++seed){const auto o=sc<4?run(static_cast<simple::WalkMode>(m),sc,seed*7919+sc*31+m,volt):runJoy(static_cast<simple::WalkMode>(m),sc,seed*7919+sc*31+m,volt);f+=o.faults;stopsOk+=o.stopsOk;stopsTried+=o.stopsTried;
      track=std::max(track,o.maxTrack);over=std::max(over,o.maxStopOvershoot);settle=std::max(settle,o.maxStopSettleS);boxOut=std::max(boxOut,o.maxStopBoxOut);sb+=o.speedBound;mv+=o.moving;
      if(o.faults){char b[128];std::snprintf(b,sizeof b," %s[%s]@c%.2f",o.reason,o.refusedBy<10?simple::kSafeChecks[o.refusedBy]:"?",o.cycles);reasons+=b;}}
    std::printf("PLANT %-8s %-16s %4.1fV seeds=%u faults=%u stops_ok=%u/%u max_tracking=%.1f ticks max_stop_end_offset=%.1f ticks max_stop_settle=%.2f s stop_box_out=%.1f ticks speed_bound=%.1f%%%s\n",
      simple::walkModeSpec(static_cast<simple::WalkMode>(m)).name,sn[sc],volt,seeds,f,stopsOk,stopsTried,track,over,settle,boxOut,mv?100.*sb/mv:0.,reasons.c_str());std::fflush(stdout);}
}
}
