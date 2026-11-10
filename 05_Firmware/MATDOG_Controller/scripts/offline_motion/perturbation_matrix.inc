// dev.6 consolidation perturbation matrix: included by run_perturbation_matrix.sh into test_simple_motion.cpp (its Rig).
// dev.6 consolidation perturbation matrix on the production rig (software only).
namespace perturb {
using P=simple::StopReferenceProbe;
struct Pert{const char* name;double imuRoll=0,imuPitch=0;bool offsets=false;int lag=0;int jitterUs=0;double missP=0;int64_t uartUs=0;int voltage=120;};
struct Out{unsigned faults=0,stops=0,maxRaw=0,missed=0,holds=0;const char* reason="";double cycles=0;bool stopOk=true;};
struct Plant{
  Rig& r;const Pert& p;std::mt19937& g;std::deque<std::array<uint16_t,12>> hist;
  static constexpr int err[12]={-2,-1,-1,0,2,3,3,-4,-1,7,5,2},load[12]={24,0,0,0,1048,1056,1056,40,0,1088,1072,1048};
  Plant(Rig& rr,const Pert& pp,std::mt19937& gg):r(rr),p(pp),g(gg){r.p.frameDelayUs=p.uartUs;
    const double hr=p.imuRoll/2,hp=p.imuPitch/2;r.in.quaternion[0]=std::cos(hr)*std::cos(hp);r.in.quaternion[1]=std::sin(hr)*std::cos(hp);r.in.quaternion[2]=std::cos(hr)*std::sin(hp);r.in.quaternion[3]=-std::sin(hr)*std::sin(hp);}
  void tick(){
    uint64_t dt=simple::kPeriodUs;if(p.jitterUs)dt+=std::uniform_int_distribution<int>(0,p.jitterUs)(g);
    if(p.missP>0&&std::uniform_real_distribution<double>(0,1)(g)<p.missP)dt+=simple::kPeriodUs;
    r.p.now+=dt;r.in.imuStampUs=r.p.now;r.m.inputs(r.in);
    std::array<uint16_t,12> t;std::copy(r.p.target,r.p.target+12,t.begin());hist.push_back(t);while(int(hist.size())>p.lag+1)hist.pop_front();
    const auto& src=hist.front();const bool on=r.m.energized();
    for(unsigned i=0;i<12;++i){auto& o=r.p.observed[i];const int raw=int(src[i])-(on&&p.offsets?err[i]:0);o.raw=uint16_t(raw);o.q=(raw-2048)*simple::kTickRad;o.load=on&&p.offsets?load[i]:0;o.voltage=p.voltage;}
    test_clock_us=r.p.now;r.m.update(r.p.now);r.m.updateTelemetry(r.p.now);}
};
static void track(Rig& r,Out& o,bool& walking,std::array<uint16_t,12>& prev){
  const bool w=r.m.state()==simple::State::WALKING;if(walking&&!w)++o.stops;walking=w;
  if(r.m.state()==simple::State::FAULT&&!o.faults){++o.faults;o.reason=r.m.reason();o.cycles=r.m.walkCycles();}
  if(w)for(unsigned i=0;i<12;++i)o.maxRaw=std::max(o.maxRaw,unsigned(std::abs(int(r.p.target[i])-int(prev[i]))));
  std::copy(r.p.target,r.p.target+12,prev.begin());}
// scenario 0 WALK100 160 s, 1 WALK75 160 s, 2 held sequence, 3 random joystick 200 s, 4 random joystick + STOP at a random instant + STAND recovery
static Out run(const Pert& p,int scenario,unsigned seed){
  std::mt19937 g(seed);Rig r;Plant pl(r,p,g);Out o;bool walking=false;std::array<uint16_t,12> prev;std::copy(r.p.target,r.p.target+12,prev.begin());
  if(r.m.stand(.15,r.p.now)){o.faults=1;o.reason="STAND_REFUSED";return o;}
  for(unsigned n=0;n<3000&&r.m.state()!=simple::State::STANDING&&r.m.state()!=simple::State::FAULT;++n)pl.tick();
  if(r.m.state()!=simple::State::STANDING){o.faults=1;o.reason=r.m.state()==simple::State::FAULT?r.m.reason():"NO_STANDING";return o;}
  const double T=simple::kJoystickPeriodS,TW=simple::kWalkPeriodS,lim[3]={simple::kJoystickStrideM/T,simple::kJoystickLateralM/T,simple::kJoystickYawRad/T},pub0=simple::kStrideM[2]/TW;
  auto loop=[&](unsigned n,auto&& cmd){for(unsigned k=0;k<n&&!o.faults;++k){cmd(k);pl.tick();track(r,o,walking,prev);}};
  if(scenario<2){r.m.walk(scenario?1:2,r.p.now);loop(8000,[](unsigned){});}
  else if(scenario==2){const double seg[][3]={{pub0,0,0},{.7071*pub0,-.7071*lim[1],0},{0,-lim[1],0},{0,0,lim[2]},{-.5*lim[0],-.7*lim[1],0},{-.7*lim[0],-.5*lim[1],0},{pub0,0,0}};
    const unsigned len[]={1500,1500,1500,1500,1500,2000,4000};for(unsigned s=0;s<7;++s)loop(len[s],[&](unsigned k){if(k%12==0)r.m.drive(seg[s][0],seg[s][1],seg[s][2],r.p.now);});}
  else{std::uniform_real_distribution<double> u(-1,1);double v[3]={0,0,0};const unsigned stopAt=scenario==4?std::uniform_int_distribution<unsigned>(500,9500)(g):~0u;
    loop(10000,[&](unsigned k){if(k%600==0){for(;;){v[0]=u(g)*pub0;v[1]=u(g)*lim[1];v[2]=u(g)*lim[2];const double n=v[0]*v[0]/(pub0*pub0)+v[1]*v[1]/(lim[1]*lim[1])+v[2]*v[2]/(lim[2]*lim[2]);if(n<=1)break;}
        if(std::uniform_int_distribution<int>(0,3)(g)==0)v[1]=v[2]=0;}
      if(k==stopAt){r.m.stop();}
      if(k>=stopAt){if(r.m.stopVerified()&&k<stopAt+2000){if(r.m.stand(.12,r.p.now))o.stopOk=false;}return;}
      if(k%12==0)r.m.drive(v[0],v[1],v[2],r.p.now);});
    if(scenario==4){for(unsigned k=0;k<3000&&r.m.state()!=simple::State::STANDING&&r.m.state()!=simple::State::FAULT;++k)pl.tick();o.stopOk=o.stopOk&&r.m.state()==simple::State::STANDING;}}
  o.missed=r.m.missed();if(!o.cycles)o.cycles=r.m.walkCycles();return o;
}
static void matrix(){
  const Pert ps[]={{"nominal"},{"imu +1.5deg pitch",0,.026},{"imu -1.5deg pitch",0,-.026},{"imu +3deg roll",.052,0},{"imu -3deg pitch",0,-.052},
    {"leg offsets+sag/load",0,0,true},{"feedback lag 1 rel",0,0,false,1},{"release jitter 0-8ms",0,0,false,0,8000},{"missed 2%",0,0,false,0,0,.02},
    {"uart 4ms",0,0,false,0,0,0,4000},{"voltage 9.0V",0,0,false,0,0,0,0,90},{"combined",0,-.026,true,1,6000,.01,3000,100}};
  const char* sn[]={"WALK100","WALK75","HELD_SEQ","RANDOM_JOY","RANDOM_STOP"};
  const unsigned seeds=getenv("SEEDS")?atoi(getenv("SEEDS")):4;
  for(const auto& p:ps)for(int s=0;s<5;++s){unsigned f=0,maxRaw=0,miss=0,stopsMax=0,stopFail=0;std::string reasons;
    for(unsigned seed=1;seed<=(s>=3||p.jitterUs||p.missP>0?seeds:1);++seed){const auto o=run(p,s,seed*7919+s);f+=o.faults;maxRaw=std::max(maxRaw,o.maxRaw);miss=std::max(miss,o.missed);stopsMax=std::max(stopsMax,o.stops);stopFail+=!o.stopOk;
      if(o.faults){char b[96];std::snprintf(b,sizeof b," %s@c%.2f",o.reason,o.cycles);reasons+=b;}}
    std::printf("PERTURB %-22s %-11s faults=%u stop_recovery_fail=%u max_stops=%u maxRAW=%u max_missed=%u%s\n",p.name,sn[s],f,stopFail,stopsMax,maxRaw,miss,reasons.c_str());std::fflush(stdout);}
}
}
