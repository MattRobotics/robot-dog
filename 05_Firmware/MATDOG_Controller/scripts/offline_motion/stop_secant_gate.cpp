// STOP secant gate, offline (F3 2026-11-16). Every reference the production
// SimpleMotion sends must stay stoppable from its own sampled rate: the STOP
// proof (StopPlan with a = 0, compensationSafe in WALK and beginStop in every
// phase) refuses a joint secant above a frontier set by the drive's braking,
// the 0.35 rad residual and the 20 ms duration grid (FAST: 2.405 rad/s, 31.36
// RAW ticks per release). The fixture FAULT of 16/11 (F3, FAST preset 2) was a
// 32-tick secant at the swing peak of a law that reaches 31.
// For every mode, preset, joystick direction and transition (and the suite's
// E -> F3 sequence), on flat ground, with a constant fixture tilt (correction
// at the envelope), on a levelling slope, with one release of feedback lag and
// with late releases, each sent frame's secant is taken as rateFromHistory
// takes it (the latest earlier frame at least one period older, else the
// oldest of three) and, with one RAW tick of quantization added over that
// window, must be accepted by StopPlan::build itself at the drive of that
// frame (frontier found by bisection on one joint in an unbounded box).
// Ideal tracking, idealised gen-1 conversion (2048 + q/tick); geometry, servo
// and box margins are other gates. Exit 0 = PASS. Usage: stop_secant_gate [--verbose]
#include "../../src/simple/SimpleMotion.h"
#include "../../src/simple/StopPlan.h"
#include "../../src/motion/PairedProneData.h"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <map>
#include <string>
static int64_t clock_us=1000000;
extern "C" int64_t esp_timer_get_time(){return clock_us;}
using namespace matdog;
namespace {
const double kLo[12]={-.783864,-.914253,-1.598408,-.783864,-.914253,-1.58,-.783864,-.914253,-1.592272,-.783864,-.914253,-1.590738};
const double kHi[12]={.783864,2.110758,.590583,.783864,2.10002,.581379,.771592,2.136835,.590583,.78233,2.129166,.585981};
constexpr double kQuantTicks=1;  // RAW quantization added over each secant window
struct Sent{int64_t us;uint16_t raw[12];simple::Drive drive;};
struct Port: simple::ServoPort {
  simple::Feedback o[12]{};uint16_t t[12]{},prev[12]{};simple::Drive d{};unsigned lag=0;Sent hist[3]{};unsigned n=0;bool fresh=false;
  Port(){const auto* p=motion::paired_data::kKnots[0].p;
    for(unsigned l=0;l<4;++l){const double q[3]={((l==0||l==3)?-1:1)*p[7],p[l<2?2:0],std::fmax((l<2?-88.5:-89.)*M_PI/180,p[l<2?3:1])};
      for(unsigned j=0;j<3;++j){auto& f=o[l*3+j];f.q=q[j];f.raw=uint16_t(2048+std::lround(q[j]/simple::kTickRad));f.torque=0;f.temp=30;f.voltage=120;f.valid=true;t[l*3+j]=prev[l*3+j]=f.raw;}}}
  bool read(unsigned i,bool control,simple::Feedback& out) override{out=o[i];out.stampUs=uint64_t(clock_us);out.goal=t[i];out.goalSpeed=d.speed;out.acc=d.acc;if(control)out.controlStampUs=uint64_t(clock_us);return true;}
  bool resolve(unsigned,double q,uint16_t& raw) override{raw=uint16_t(2048+std::lround(q/simple::kTickRad));return raw<4096;}
  bool decodeRaw(unsigned,uint16_t raw,double& q) override{q=(int(raw)-2048)*simple::kTickRad;return raw<4096;}
  bool frame(const uint16_t raw[12],simple::Drive dr) override{d=dr;std::copy(raw,raw+12,t);
    hist[2]=hist[1];hist[1]=hist[0];hist[0].us=clock_us;std::copy(raw,raw+12,hist[0].raw);hist[0].drive=dr;n=std::min(n+1,3u);fresh=true;return true;}
  bool enable(unsigned i) override{o[i].torque=1;return true;}
  bool off(unsigned) override{for(auto& f:o)f.torque=0;return true;}
  bool verifyOff(unsigned i) override{return i==12||o[i].torque==0;}
  void publish(){const uint16_t* s=lag?prev:t;for(unsigned i=0;i<12;++i){o[i].raw=s[i];o[i].q=(int(s[i])-2048)*simple::kTickRad;}std::copy(t,t+12,prev);}
};
// Largest secant (rad/s) StopPlan::build accepts for one joint at this drive (no box, a = 0).
double frontier(simple::Drive drive){
  static std::map<std::pair<unsigned,unsigned>,double> cache;const auto key=std::make_pair(unsigned(drive.speed),unsigned(drive.acc));
  if(auto it=cache.find(key);it!=cache.end())return it->second;
  auto ok=[&](double v){simple::StopPlan p;double q[12]{},vv[12]{},a[12]{},lo[12],hi[12];vv[1]=v;
    for(unsigned i=0;i<12;++i){lo[i]=-100;hi[i]=100;}
    return p.build(q,vv,a,lo,hi,simple::stopSpeedRadS(drive),simple::stopAccelerationRadS2(drive));};
  double lo=0,hi=simple::stopSpeedRadS(drive);
  if(ok(hi))return cache[key]=hi;
  for(int k=0;k<60;++k){const double mid=.5*(lo+hi);(ok(mid)?lo:hi)=mid;}
  return cache[key]=lo;
}
struct Worst{double margin=1e9,secant=0,limit=0;std::string where;unsigned frames=0;};
struct Cond{const char* name;double roll,pitch;bool ground;unsigned lag;bool jitter;};
const Cond kConds[]={{"flat",0,0,true,0,false},{"flat_lag",0,0,true,1,false},{"flat_jitter",0,0,true,0,true},
  {"fixture_a",-.006,.006,false,1,false},{"fixture_b",.006,-.006,false,1,false},{"slope",-.004,.004,true,0,true}};
struct Run{
  simple::SimpleMotion m;Port port;simple::Inputs in;const Cond& c;Worst& w;Worst& ws;std::string tag;uint32_t rng=12345;bool verbose;bool started=false;
  // w: WALKING frames; ws: every other moving frame (STAND, HEIGHT, WALK_SETTLE, STOP) of the same scenario group
  Run(const Cond& cond,std::map<std::string,Worst>& g,const std::string& group,std::string t,bool v):c(cond),w(g[group+" [walk]"]),ws(g[group+" [stand/settle]"]),tag(std::move(t)),verbose(v){
    port.lag=c.lag;m.begin(&port);in.ready=in.powerOk=in.authorityOk=in.imuValid=true;in.refusal="READY";in.accCeiling=simple::kAccCeilingTarget;
    std::copy(kLo,kLo+12,in.minRad);std::copy(kHi,kHi+12,in.maxRad);tilt(0,0);}
  void tilt(double r,double p){const double cr=std::cos(r/2),sr=std::sin(r/2),cp=std::cos(p/2),sp=std::sin(p/2);
    in.quaternion[0]=cr*cp;in.quaternion[1]=sr*cp;in.quaternion[2]=cr*sp;in.quaternion[3]=-sr*sp;}
  void tick(){
    rng=rng*1664525u+1013904223u;const int64_t late=c.jitter&&(rng>>28)<4?int64_t((rng>>8)%3000):0;  // a quarter of releases late by 0-3 ms
    clock_us=(clock_us/simple::kPeriodUs+1)*simple::kPeriodUs+late;in.imuStampUs=uint64_t(clock_us);
    if(c.ground)tilt(c.roll+m.imuApplied(0),c.pitch+m.imuApplied(1));else tilt(c.roll,c.pitch);  // ground: the body levels
    m.inputs(in);port.publish();port.fresh=false;m.update(uint64_t(clock_us));
    if(!port.fresh||port.n<2||!started)return;
    // rateFromHistory: the latest earlier frame at least one period older, else the oldest available of three.
    const Sent& now=port.hist[0];const Sent* far=nullptr;
    for(unsigned k=1;k<port.n;++k){far=&port.hist[k];if(now.us-port.hist[k].us>=int64_t(simple::kPeriodUs))break;}
    const double dt=double(now.us-far->us)/1e6;if(dt<=0||dt>.1)return;
    const double limit=frontier(now.drive);Worst& w=m.state()==simple::State::WALKING?this->w:ws;++w.frames;
    for(unsigned i=0;i<12;++i){const double v=(std::abs(int(now.raw[i])-int(far->raw[i]))+kQuantTicks)*simple::kTickRad/dt;
      const double margin=(limit-v)*.02/simple::kTickRad;  // ticks per 20 ms
      if(margin<w.margin){w.margin=margin;w.secant=std::abs(int(now.raw[i])-int(far->raw[i]))*.02/dt;w.limit=limit*.02/simple::kTickRad;
        char b[200];std::snprintf(b,sizeof b,"%s/%s %s %s joint %u cycles %.3f",tag.c_str(),c.name,simple::name(m.state()),m.phase(),i,m.walkCycles());w.where=b;}}
  }
  bool until(simple::State s,unsigned n){for(unsigned k=0;k<n;++k){tick();if(m.state()==s)return true;if(m.state()==simple::State::FAULT)return false;}return false;}
  bool settle(){for(unsigned k=0;k<3000;++k){tick();if(m.state()==simple::State::FAULT)return false;if(m.state()==simple::State::STANDING&&k>50)return true;}return false;}
  bool stand(){for(int i=0;i<15;++i)tick();started=true;if(m.stand(.120,uint64_t(clock_us)))return false;return until(simple::State::STANDING,3000);}
  bool fault(){if(m.state()!=simple::State::FAULT)return false;if(verbose)std::printf("  FAULT %s in %s/%s\n",m.reason(),tag.c_str(),c.name);return true;}
};
}
int main(int argc,char** argv){
  const bool verbose=argc>1&&!std::strcmp(argv[1],"--verbose");unsigned faults=0,fails=0;
  std::map<std::string,Worst> groups;
  auto record=[&](Run& r){if(r.fault())++faults;};
  for(unsigned mi=0;mi<3;++mi){
    const auto mode=static_cast<simple::WalkMode>(mi);const auto& law=simple::walkModeSpec(mode);const auto& js=simple::joystickSpec(mode);
    const double T=law.periodS,top=law.strideM[2]/T;const std::string mn=law.name;
    for(const auto& c:kConds){
      for(unsigned p=0;p<3;++p){Run r(c,groups,mn+" preset"+std::to_string(p),mn+" preset"+std::to_string(p),verbose);
        if(!r.stand()){record(r);continue;}r.m.selectWalkMode(mode);r.m.walk(p,uint64_t(clock_us));
        for(unsigned k=0;k<900&&!r.fault();++k)r.tick();r.m.walkStop();r.settle();record(r);}
      const double seg[][3]={{top,0,0},{.9*top,0,0},{-js.backM/T,0,0},{0,js.lateralM/T,0},{0,-js.lateralM/T,0},{0,0,js.yawRad/T},{0,0,-js.yawRad/T},
        {.7*js.strideM/T,.7*js.lateralM/T,0},{.7*js.strideM/T,0,.7*js.yawRad/T}};
      for(const auto& s:seg){if(s[0]<0&&js.backM<=0)continue;if((s[1]!=0||s[2]!=0)&&js.lateralM<=0)continue;
        Run r(c,groups,mn+" joystick",mn+" joystick",verbose);if(!r.stand()){record(r);continue;}r.m.selectWalkMode(mode);
        for(unsigned k=0;k<900&&!r.fault();++k){if(k%12==0)r.m.drive(s[0],s[1],s[2],uint64_t(clock_us));r.tick();}
        r.m.drive(0,0,0,uint64_t(clock_us));r.m.walkStop();r.settle();record(r);}
      {Run r(c,groups,mn+" transitions",mn+" preset1to2",verbose);
        if(r.stand()){r.m.selectWalkMode(mode);r.m.walk(1,uint64_t(clock_us));for(unsigned k=0;k<300&&!r.fault();++k)r.tick();
          r.m.walk(2,uint64_t(clock_us));for(unsigned k=0;k<600&&!r.fault();++k)r.tick();r.m.walkStop();r.settle();}record(r);
        if(mi<2){Run q(c,groups,mn+" transitions",mn+" to "+simple::walkModeSpec(static_cast<simple::WalkMode>(mi+1)).name,verbose);
          if(q.stand()){q.m.selectWalkMode(mode);q.m.walk(2,uint64_t(clock_us));for(unsigned k=0;k<300&&!q.fault();++k)q.tick();
            q.m.selectWalkMode(static_cast<simple::WalkMode>(mi+1));for(unsigned k=0;k<900&&!q.fault();++k)q.tick();q.m.walkStop();q.settle();}record(q);}
        if(mi>0){Run q(c,groups,mn+" transitions",mn+" to lower",verbose);
          if(q.stand()){q.m.selectWalkMode(mode);q.m.walk(2,uint64_t(clock_us));for(unsigned k=0;k<300&&!q.fault();++k)q.tick();
            q.m.selectWalkMode(static_cast<simple::WalkMode>(mi-1));for(unsigned k=0;k<900&&!q.fault();++k)q.tick();q.m.walkStop();q.settle();}record(q);}}
    }
  }
  // The fixture suite's block E then F1-F3 (16/11 FAULT): STANDARD preset 1, MODE SLOW, MODE STANDARD, WALK STOP, FAST preset 2.
  for(const auto& c:kConds){Run r(c,groups,"E->F3","E->F3",verbose);
    if(r.stand()){r.m.selectWalkMode(simple::WalkMode::STANDARD);r.m.walk(1,uint64_t(clock_us));for(unsigned k=0;k<300;++k)r.tick();
      r.m.selectWalkMode(simple::WalkMode::SLOW);for(unsigned k=0;k<600;++k)r.tick();r.m.selectWalkMode(simple::WalkMode::STANDARD);for(unsigned k=0;k<600;++k)r.tick();
      r.m.walkStop();r.settle();r.m.selectWalkMode(simple::WalkMode::FAST);r.m.walk(2,uint64_t(clock_us));for(unsigned k=0;k<900&&!r.fault();++k)r.tick();
      r.m.walkStop();r.settle();}record(r);}
  std::printf("STOP secant gate: secant + %.0f tick of quantization inside the StopPlan frontier of the frame's drive\n",kQuantTicks);
  for(const auto& [g,w]:groups){const bool ok=w.margin>=0&&w.frames>0;fails+=!ok;
    std::printf("%-22s frames %6u  worst secant %5.2f ticks/20 ms  frontier %5.2f  margin %+5.2f  %s  [%s]\n",g.c_str(),w.frames,w.secant,w.limit,w.margin,ok?"PASS":"FAIL",w.where.c_str());}
  std::printf("faults %u\n%s\n",faults,fails||faults?"STOP_SECANT_GATE FAIL":"STOP_SECANT_GATE PASS");
  return fails||faults?1:0;
}
