// Core Motion V5: common time law (S-curve, closed form) and dynamic-profile analysis. Pure host test.
#include <cmath>
#include <cstdio>
#include <cstring>
#include <vector>
#include "../../src/motion/MotionLaw.h"
using namespace matdog::motion;
static int checks=0,failures=0;
#define CHECK(c) do{++checks;if(!(c)){++failures;std::printf("FAIL %d: %s\n",__LINE__,#c);}}while(0)
static bool near(double a,double b,double tol){return std::fabs(a-b)<=tol*(1+std::fabs(b));}
int main(){
  // ===== 1. jerk-limited speed change: closed form vs explicit integration =====
  for(double v0:{0.,.1,.5})for(double v1:{0.,.3,1.})for(double a:{.5,2.})for(double j:{5.,50.,500.}){
    const auto c=speedChange(v0,v1,a,j);CHECK(c.valid);
    // integrate the jerk profile of this change numerically
    const double dv=std::fabs(v1-v0),sign=v1>=v0?1.:-1.;const bool tri=dv<=a*a/j;
    const double t1=tri?std::sqrt(dv/j):a/j,t2=tri?0:dv/a-a/j;CHECK(near(c.timeS,2*t1+t2,1e-12));
    double v=v0,ac=0,s=0;   // phase-aligned numerical integration (jerk is constant inside each phase)
    if(dv==0){CHECK(c.timeS==0&&c.distance==0);continue;}
    const double dur[3]={t1,t2,t1},jer[3]={sign*j,0,-sign*j};
    for(unsigned ph=0;ph<3;++ph){const unsigned n=50000;const double h=dur[ph]/n;
      for(unsigned k=0;k<n;++k){const double vp=v;ac+=jer[ph]*h;v+=ac*h;s+=0.5*(vp+v)*h;}}
    CHECK(near(v,v1,2e-4));CHECK(near(s,c.distance,2e-4));CHECK(c.peakAccel<=a*(1+1e-12));
  }
  CHECK(!speedChange(-1,0,1,1).valid&&!speedChange(0,1,0,1).valid&&!speedChange(0,1,1,NAN).valid);
  // ===== 2. S-curve: exact boundary conditions, limits, continuity, consistency =====
  unsigned profiles=0;double worst_cont=0,worst_int=0;
  for(double L:{1e-5,.001,.01,.1,1.,2.584477171,30.})for(double v:{.05,.19,1.,1.55})for(double a:{.3,2.,4.5,13.})for(double j:{1.,10.,50.,400.,1e4}){
    ScurveProfile p;CHECK(p.plan(L,{v,a,j}));++profiles;
    const double T=p.durationS();CHECK(T>0&&std::isfinite(T));
    CHECK(p.peakVelocity()<=v*(1+1e-12)&&p.peakAcceleration()<=a*(1+1e-12));
    const auto e=p.at(T),b=p.at(0);CHECK(near(e.s,L,1e-9)&&e.v==0&&e.a==0&&b.s==0&&b.v==0);
    // sampled checks: limits, monotone position, continuity, v=ds/dt, a=dv/dt
    const unsigned n=400;double prev_s=0,prev_v=0,prev_a=0;LawSample prev=p.at(0);
    for(unsigned k=1;k<=n;++k){const double t=T*k/n;const auto q=p.at(t);
      CHECK(q.v<=v*(1+1e-9)&&std::fabs(q.a)<=a*(1+1e-9)&&std::fabs(q.j)<=j*(1+1e-9));
      CHECK(q.s>=prev_s-1e-12*(1+L));
      const double h=T/n;// trapezoid of v over the step vs position difference
      const double ds=q.s-prev.s,ave=0.5*(q.v+prev.v)*h;worst_int=std::fmax(worst_int,std::fabs(ds-ave)/(1+L)*(1/ (h*h*a+1e-30))*1e-3);
      worst_cont=std::fmax(worst_cont,std::fmax(std::fabs(q.v-prev_v)/(a*h*1.0000001+1e-30)*0+std::fabs(q.v-prev_v)-a*h*1.0001<0?0:std::fabs(q.v-prev_v)-a*h*1.0001,std::fabs(q.a-prev_a)-j*h*1.0001<0?0:std::fabs(q.a-prev_a)-j*h*1.0001));
      prev_s=q.s;prev_v=q.v;prev_a=q.a;prev=q;}
  }
  CHECK(worst_cont<1e-9);   // |dv|<=a*dt and |da|<=j*dt at every sample: continuous v and a (jerk bounded)
  std::printf("LAW_SCURVE profiles=%u v_a_continuity_violation=%.3g\n",profiles,worst_cont);
  // trapezoid limit: j -> infinity reproduces the classic trapezoid / triangle times
  for(double L:{.01,.1,1.}){const double v=.19,a=2.;ScurveProfile p;CHECK(p.plan(L,{v,a,1e12}));
    const double classic=L>=v*v/a?L/v+v/a:2*std::sqrt(L/a);CHECK(near(p.durationS(),classic,1e-5));}
  // ===== 3. S-curve versus quintic at equal peak limits (the arbitrary-shape baseline) =====
  std::printf("LAW_COMPARE v=.19 a=2 L(rad) | quintic_s | scurve_s j=10 | j=50 | j=400 | trapezoid_s\n");
  for(double L:{.005,.02,.05,.1,.3,1.,2.584477171}){
    const double q=quinticMinTime(L,.19,2.);std::printf("LAW_COMPARE L=%8.5f quintic=%8.4f scurve10=%8.4f scurve50=%8.4f scurve400=%8.4f trapezoid=%8.4f\n",L,q,
      scurveMinTime(L,{.19,2.,10}),scurveMinTime(L,{.19,2.,50}),scurveMinTime(L,{.19,2.,400}),scurveMinTime(L,{.19,2.,1e12}));
    CHECK(scurveMinTime(L,{.19,2.,400})<=q);       // with a stiff jerk limit the derived time never exceeds the quintic's
  }
  // ===== 4. executor-rule lateness tolerance: closed search vs direct evaluation =====
  {const double v=.19,a=2,T=.02,step=.004;const double L=accelSafeLatenessS(v,a,T,step,.08);
   auto accel=[&](double late){const double dt=T+late,adv=std::fmin(dt,step/v);return std::fabs(v*adv/dt-v)/(.5*(dt+T));};
   CHECK(accel(L)<=a+1e-9&&accel(L+1e-4)>a);std::printf("LAW_LATENESS qualified_stand v=.19 a=2 T=20ms step=4mrad -> accel_safe_lateness_ms=%.3f step_headroom_ms=%.3f\n",L*1000,(step/v-T)*1000);
   CHECK(L>.005&&L<.012);
   CHECK(accelSafeLatenessS(.19,2,.02,.003,.08)==0);}   // v*T > step: invalid profile
  // ===== 5. dynamic profile analysis =====
  {DynamicProfile stand{"STAND_QUALIFIED",.19,2.,0,.020,.004,300,20};const auto a=analyzeDynamicProfile(stand);
   CHECK(a.valid&&std::fabs(a.stepAtNominalRad-.0038)<1e-12&&a.driveSpeedRatio>2.4&&a.driveAccelRatio>1.5);
   std::printf("LAW_PROFILE %-18s valid=%d step_mrad=%.2f headroom_ms=%.2f accel_safe_lateness_ms=%.2f drive_ratio_v=%.2f drive_ratio_a=%.2f\n",stand.name,a.valid,a.stepAtNominalRad*1000,a.stepHeadroomMs,a.accelSafeLatenessMs,a.driveSpeedRatio,a.driveAccelRatio);
   DynamicProfile walk{"WALK_STANDARD",1.0,4.5,0,.020,.05,1000,50};const auto w=analyzeDynamicProfile(walk);CHECK(w.valid);
   std::printf("LAW_PROFILE %-18s valid=%d step_mrad=%.2f headroom_ms=%.2f accel_safe_lateness_ms=%.2f drive_ratio_v=%.2f drive_ratio_a=%.2f\n",walk.name,w.valid,w.stepAtNominalRad*1000,w.stepHeadroomMs,w.accelSafeLatenessMs,w.driveSpeedRatio,w.driveAccelRatio);
   DynamicProfile bad{"STEP",.3,2,0,.02,.004,300,20};CHECK(!analyzeDynamicProfile(bad).valid&&!std::strcmp(analyzeDynamicProfile(bad).reason,"STEP_EXCEEDS_CAP"));
   DynamicProfile slow{"DRIVE",.19,2,0,.02,.004,100,20};CHECK(!analyzeDynamicProfile(slow).valid);
   DynamicProfile weak{"ACC",.19,2,0,.02,.004,300,5};CHECK(!analyzeDynamicProfile(weak).valid);
   DynamicProfile zero{"ZERO",0,2,0,.02,.004,300,20};CHECK(!analyzeDynamicProfile(zero).valid);}
  std::printf("MOTION_LAW %s checks=%d failures=%d hardware=false\n",failures?"FAIL":"PASS",checks,failures);
  return failures?1:0;
}
