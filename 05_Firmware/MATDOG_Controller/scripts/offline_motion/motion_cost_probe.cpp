// Motion release cost, offline (dev.15). Production SimpleMotion on the scripted
// rig of test_simple_motion.cpp with a constant tilted IMU (compensation active):
// HOLD100, WALK75, WALK100, a steered combined walk. Counts per Motion release
// (SimpleMotion::update only; cooperative telemetry excluded): libm
// transcendental/sqrt calls through link-time wrappers (opcount.c, soft-double
// cost proxy for the ESP32-S3) and the firmware's own work counters. Host wall
// time is reported only as a relative figure, NOT ESP32-S3 timing.
// Build: run_motion_cost_probe.sh.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wreturn-type"
#define main simple_motion_main
#include "../tests/test_simple_motion.cpp"
#undef main
#pragma GCC diagnostic pop
#include <chrono>
#include "../../src/simple/MotionTiming.h"
extern "C" { extern uint64_t op_trig,op_inv,op_sqrt,op_hypot,op_rem,op_floor; }
namespace {
struct Stat {
  std::vector<double> us,trig,inv,sq,hyp,rem,flr;uint32_t work[simple::timing::WORKS]{};
  void release(Rig& r){r.p.now+=simple::kPeriodUs;r.in.imuStampUs=r.p.now;r.m.inputs(r.in);r.p.publishCommandedPose();test_clock_us=r.p.now;
    const uint64_t t0=op_trig,i0=op_inv,s0=op_sqrt,h0=op_hypot,r0=op_rem,f0=op_floor;const auto a=std::chrono::steady_clock::now();
    r.m.update(r.p.now);
    us.push_back(std::chrono::duration<double,std::micro>(std::chrono::steady_clock::now()-a).count());
    trig.push_back(double(op_trig-t0));inv.push_back(double(op_inv-i0));sq.push_back(double(op_sqrt-s0));hyp.push_back(double(op_hypot-h0));rem.push_back(double(op_rem-r0));flr.push_back(double(op_floor-f0));
    const auto& w=simple::timing::state.previousWork;for(unsigned i=0;i<simple::timing::WORKS;++i)work[i]=std::max(work[i],w[i]);
    r.m.updateTelemetry(r.p.now);}
  static double mean(const std::vector<double>& v){double s=0;for(double x:v)s+=x;return s/v.size();}
  static double max(const std::vector<double>& v){return *std::max_element(v.begin(),v.end());}
  // Per-release libm total: the release maximum, not the mean, meets the 20 ms deadline.
  std::vector<double> totals() const {std::vector<double> t(us.size());for(size_t i=0;i<us.size();++i)t[i]=trig[i]+inv[i]+sq[i]+hyp[i]+rem[i]+flr[i];return t;}
  static double percentile(std::vector<double> v,double q){std::sort(v.begin(),v.end());return v[std::min(v.size()-1,size_t(q*v.size()))];}
  void print(const char* name){
    std::printf("%-16s n=%4zu trig mean/max=%6.1f/%4.0f  inv-trig=%5.1f/%3.0f  sqrt=%6.1f/%4.0f  hypot=%5.1f rem=%5.1f floor=%5.1f | libm mean=%6.1f | host_us mean=%5.1f max=%6.1f | work max solves=%u ik=%u newton=%u fk=%u\n",
      name,us.size(),mean(trig),max(trig),mean(inv),max(inv),mean(sq),max(sq),mean(hyp),mean(rem),mean(flr),mean(trig)+mean(inv)+mean(sq)+mean(hyp)+mean(rem)+mean(flr),mean(us),max(us),
      work[simple::timing::SOLVES],work[simple::timing::IK],work[simple::timing::NEWTON],work[simple::timing::FK]);
    const auto t=totals();std::printf("%-16s libm per release p50=%.0f p90=%.0f p99=%.0f max=%.0f\n",name,percentile(t,.5),percentile(t,.9),percentile(t,.99),max(t));}};
void tilt(Rig& r,double roll,double pitch){  // constant board tilt seen by the IMU
  const double cr=std::cos(roll/2),sr=std::sin(roll/2),cp=std::cos(pitch/2),sp=std::sin(pitch/2);
  r.in.quaternion[0]=cr*cp;r.in.quaternion[1]=sr*cp;r.in.quaternion[2]=cr*sp;r.in.quaternion[3]=-sr*sp;}
}
int main(){
  constexpr double degree=3.14159265358979323846/180;
  {Rig r;assert(!r.m.stand(.10,r.p.now));assert(r.reachStand());tilt(r,.8*degree,1.2*degree);Stat s;
    for(unsigned n=0;n<650;++n)s.release(r);s.print("HOLD100 tilted");}
  for(unsigned stride:{1u,2u}){Rig r;assert(!r.m.stand(.15,r.p.now));assert(r.reachStand());assert(!r.m.walk(stride,r.p.now));
    for(unsigned n=0;n<3000&&r.m.state()!=simple::State::WALKING;++n)r.tick();tilt(r,.3*degree,.8*degree);Stat s;
    for(unsigned n=0;n<975;++n)s.release(r);assert(r.m.state()==simple::State::WALKING);s.print(stride==1?"WALK75 tilted":"WALK100 tilted");}
  {Rig r;assert(!r.m.stand(.15,r.p.now));assert(r.reachStand());const double T=simple::kJoystickPeriodS,v[3]={.5*simple::kJoystickStrideM/T,.5*simple::kJoystickLateralM/T,.5*simple::kJoystickYawRad/T};
    for(unsigned n=0;n<3000&&r.m.state()!=simple::State::WALKING;++n){if(n%12==0)r.m.drive(v[0],v[1],v[2],r.p.now);r.tick();}
    tilt(r,.3*degree,.8*degree);Stat s;for(unsigned n=0;n<975;++n){if(n%12==0)r.m.drive(v[0],v[1],v[2],r.p.now);s.release(r);}
    assert(r.m.state()==simple::State::WALKING);s.print("JOYSTICK tilted");}
}
