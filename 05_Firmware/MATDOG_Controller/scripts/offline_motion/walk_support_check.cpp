// WALK STANDARD support and law check (gait redesign 2026-11-08), host only.
// For one stride of the production law (simple::walkGait, optionally reshaped
// by the same what-if flags as walk_tilt_envelope_check.cpp) it samples the law
// every 5 ms on the firmware walk clock (walkClockS start ease) through the
// start ramp, two steady cycles and every quarter WALK STOP from the end of the
// start ramp to the first steady cycle, solves the production contact IK and
// reports:
//  - static support: distance of the URDF COM (REV01-FIX inertials below,
//    cross-checked against scripts/offline_prone/geometry.py to 1e-6 m; the
//    --com-error-x/y-mm flags shift it in base_link to bound model error) inside
//    the polygon of the stance contacts (nominal strip: contact = foot target);
//  - cart-table ZMP margin (COM minus z/g times its acceleration, 5 ms central
//    differences) inside the same polygon;
//  - peak joint speed/acceleration from the law's clock jets on every segment,
//    against the ST3215 RAM drive kWalkDrive;
//  - the smallest margin of each joint to the WALK box kHeight.
// Rigid flat ground, ideal tracking, no contact compliance, friction or
// dynamic stability model; a positive margin is a static model margin, not a
// hardware qualification. The STOP proof and the IMU envelope are checked by
// walk_tilt_envelope_check.cpp. Usage: see run_walk_support_check.sh.
#include "../../src/simple/SimpleMotion.h"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>
static int64_t clock_us=0;
extern "C" int64_t esp_timer_get_time(){return clock_us;}
using namespace matdog;using namespace matdog::motion;
namespace {
const char* kJoint[12]={"LF_HIP","LF_UPPER","LF_LOWER","RF_HIP","RF_UPPER","RF_LOWER","RH_HIP","RH_UPPER","RH_LOWER","LH_HIP","LH_UPPER","LH_LOWER"};
// URDF inertials (03_CAD/URDF/matt_robodog_rev00, REV01-FIX), link frames; s = +1 left, -1 right.
struct Link{double m,c[3];};
Link hipLink(int s){return {.050,{.00366,s*.02982,.00694}};}
Link upperLink(int s){return {.152,{.00122,s*.00014,-.04503}};}
Link lowerLink(int s){return {.059,{.03075,-s*.00118,-.01958}};}
const Link kFoot{.014,{-.00074,0,.01572}},kBase{1.828,{-.00693,-.00044,.03657}};
void rotX(double a,const double v[3],double o[3]){const double c=std::cos(a),s=std::sin(a);o[0]=v[0];o[1]=c*v[1]-s*v[2];o[2]=s*v[1]+c*v[2];}
void rotY(double a,const double v[3],double o[3]){const double c=std::cos(a),s=std::sin(a);o[0]=c*v[0]+s*v[2];o[1]=v[1];o[2]=-s*v[0]+c*v[2];}
void plus(double v[3],const Vector3& t){v[0]+=t.x;v[1]+=t.y;v[2]+=t.z;}
// URDF COM in base_link for joint frame q (chain hip X, upper Y, lower Y, fixed foot).
void com(const JointTargetFrame& q,double out[3]){
  double mass=kBase.m,sum[3]={kBase.m*kBase.c[0],kBase.m*kBase.c[1],kBase.m*kBase.c[2]};
  for(unsigned l=0;l<4;++l){const auto& m=*legModel(static_cast<LegId>(l));const int s=(l==0||l==3)?1:-1;const auto& j=q.legs[l];
    const Link links[4]={hipLink(s),upperLink(s),lowerLink(s),kFoot};
    for(unsigned k=0;k<4;++k){double p[3]={links[k].c[0],links[k].c[1],links[k].c[2]},t[3];
      if(k>=3)plus(p,m.footOrigin);
      if(k>=2){rotY(j.lower,p,t);std::copy(t,t+3,p);plus(p,m.lowerOrigin);}
      if(k>=1){rotY(j.upper,p,t);std::copy(t,t+3,p);plus(p,m.upperOrigin);}
      rotX(j.hip,p,t);std::copy(t,t+3,p);plus(p,m.hipOrigin);
      mass+=links[k].m;for(unsigned i=0;i<3;++i)sum[i]+=links[k].m*p[i];}}
  for(unsigned i=0;i<3;++i)out[i]=sum[i]/mass;
}
// Signed distance of (x,y) inside the convex polygon of pts (positive inside).
double inside(std::vector<Vector3> pts,double x,double y){
  double cx=0,cy=0;for(const auto& p:pts){cx+=p.x;cy+=p.y;}cx/=pts.size();cy/=pts.size();
  std::sort(pts.begin(),pts.end(),[&](const Vector3& a,const Vector3& b){return std::atan2(a.y-cy,a.x-cx)<std::atan2(b.y-cy,b.x-cx);});
  double m=1e9;for(size_t i=0;i<pts.size();++i){const auto& a=pts[i];const auto& b=pts[(i+1)%pts.size()];const double ex=b.x-a.x,ey=b.y-a.y;
    m=std::fmin(m,(ex*(y-a.y)-ey*(x-a.x))/std::hypot(ex,ey));}
  return m;
}
struct Sample{double c,rate,accel;};
}
int main(int argc,char** argv){
  unsigned stride=2;
  for(int i=1;i+1<argc;i+=2)if(!std::strcmp(argv[i],"--stride"))stride=std::atoi(argv[i+1]);
  if(stride>2)return 2;
  GaitParameters g=simple::walkGait(stride);double period=simple::kWalkPeriodS,comDx=0,comDy=0;
  struct Opt{const char* name;double* value;double scale;};
  const Opt opts[]={{"--period",&period,1},{"--duty",&g.duty,1},{"--lift-mm",&g.liftM,1e-3},{"--swayx-mm",&g.swayXM,1e-3},{"--swayy-mm",&g.swayYM,1e-3},
    {"--com-mm",&g.comShiftXM,1e-3},{"--com-error-x-mm",&comDx,1e-3},{"--com-error-y-mm",&comDy,1e-3},{"--ramp",&g.swingRamp,1},{"--settle",&g.settleCycles,1},{"--start",&g.startCycles,1},{"--stop",&g.stopCycles,1},{"--stride-mm",&g.advanceXM,1e-3}};
  for(int i=1;i+1<argc;i+=2){if(!std::strcmp(argv[i],"--stride"))continue;bool known=false;
    for(const auto& o:opts)if(!std::strcmp(argv[i],o.name)){*o.value=std::atof(argv[i+1])*o.scale;known=true;}if(!known)return 2;}
  if(!validGaitParameters(g))return 2;
  const auto& box=simple::kWalkBox;const double dt=.005,gravity=9.80665;
  const double vDrive=simple::kWalkDrive.speed*simple::kTickRad,aDrive=simple::kWalkDrive.acc*100*simple::kTickRad,settle=gaitSettleCycles(g);
  double support=1e9,supportC=0,zmp=1e9,vmax=0,amax=0,margin[12];int vJ=0,aJ=0;unsigned fails=0;for(double& m:margin)m=1e9;
  double phaseMin[3]={1e9,1e9,1e9};  // start ramp, steady cycles, WALK STOP segments
  auto run=[&](const std::vector<Sample>& seg,double terminal){
    std::vector<double> cx,cy,cz;std::vector<std::vector<Vector3>> polygons;LocomotionFrame prev;bool have=false;
    for(const auto& smp:seg){
      const auto s=gaitCartesian(g,smp.c,smp.rate,smp.accel,terminal);const auto f=solveGait(g,s,have?&prev:nullptr,true);
      if(!f.target.valid){++fails;have=false;cx.push_back(NAN);cy.push_back(NAN);cz.push_back(NAN);polygons.emplace_back();continue;}
      for(unsigned l=0;l<4;++l){const double v[3]={f.velocityRadS[l].hip,f.velocityRadS[l].upper,f.velocityRadS[l].lower},a[3]={f.accelerationRadS2[l].hip,f.accelerationRadS2[l].upper,f.accelerationRadS2[l].lower};
        for(unsigned k=0;k<3;++k){if(std::abs(v[k])>vmax){vmax=std::abs(v[k]);vJ=l*3+k;}if(std::abs(a[k])>amax){amax=std::abs(a[k]);aJ=l*3+k;}}}
      for(unsigned i=0;i<12;++i){const double q=jointSlot(f.target,i);margin[i]=std::fmin(margin[i],std::fmin(q-box.lo[i],box.hi[i]-q));}
      double cb[3];com(f.target,cb);cb[0]+=comDx;cb[1]+=comDy;const auto& t=s.body.translationM;const auto& r=s.body.rotation;
      const double w[3]={t.x+r[0][0]*cb[0]+r[0][1]*cb[1]+r[0][2]*cb[2],t.y+r[1][0]*cb[0]+r[1][1]*cb[1]+r[1][2]*cb[2],t.z+r[2][0]*cb[0]+r[2][1]*cb[1]+r[2][2]*cb[2]};
      std::vector<Vector3> pts;for(unsigned l=0;l<4;++l)if(s.phases[l].stance)pts.push_back(s.feet[l]);
      if(pts.size()>=3){const double m=inside(pts,w[0],w[1]);if(m<support){support=m;supportC=smp.c;}
        double& pm=phaseMin[terminal>=0?2:smp.c<g.startCycles+.25?0:1];pm=std::fmin(pm,m);}
      cx.push_back(w[0]);cy.push_back(w[1]);cz.push_back(w[2]);polygons.push_back(pts);prev=f;have=true;}
    for(size_t k=1;k+1<cx.size();++k){if(polygons[k].size()<3||!std::isfinite(cx[k-1]+cx[k+1]))continue;
      const double ax=(cx[k+1]-2*cx[k]+cx[k-1])/(dt*dt),ay=(cy[k+1]-2*cy[k]+cy[k-1])/(dt*dt);
      zmp=std::fmin(zmp,inside(polygons[k],cx[k]-cz[k]/gravity*ax,cy[k]-cz[k]/gravity*ay));}
  };
  // Start on the walk clock (C2 ease of the rate over the first second), ramp and two steady cycles.
  {std::vector<Sample> s;for(double t=0;;t+=dt){const double u=std::fmin(1.,t),c=simple::walkClockS(t)/period;if(c>g.startCycles+2)break;
     s.push_back({c,(t<1?u*u*u*(10+u*(-15+6*u)):1)/period,(t<1?30*u*u*(1-u)*(1-u):0)/period});}run(s,-1);}
  // Every quarter WALK STOP from the end of the start ramp through the first steady cycle.
  for(double q=0;q<1.5;q+=.25){const double terminal=g.startCycles+q+g.stopCycles;std::vector<Sample> s;
    for(double c=std::fmax(0.,terminal-g.stopCycles-.3);c<=terminal+settle+1e-12;c+=dt/period)s.push_back({std::fmin(c,terminal+settle),1/period,0});run(s,terminal);}
  double hip=1e9,upper=1e9,lower=1e9;
  for(unsigned i=0;i<12;++i)(i%3==0?hip:i%3==1?upper:lower)=std::fmin(i%3==0?hip:i%3==1?upper:lower,margin[i]);
  const bool pass=!fails&&support>0&&zmp>0&&vmax<vDrive&&amax<aDrive&&hip>0&&upper>0&&lower>0;
  std::printf("stride=%.0f period=%.2f v=%.2f mm/s duty=%.3f com=%.1f swayx/y=%.1f/%.1f ramp=%.2f settle=%.2f | solve fails=%u | static support min %.2f mm (c%.3f) | ZMP min %.2f mm | "
              "start/steady/stop %.2f/%.2f/%.2f mm | joint vmax %.3f rad/s %s (drive %.3f) amax %.2f rad/s2 %s (drive %.2f) | box margin ticks HIP %.1f UPPER %.1f LOWER %.1f | %s\n",
    g.advanceXM*1000,period,g.advanceXM/period*1000,g.duty,g.comShiftXM*1000,g.swayXM*1000,g.swayYM*1000,g.swingRamp,settle,fails,support*1000,supportC,zmp*1000,
    phaseMin[0]*1000,phaseMin[1]*1000,phaseMin[2]*1000,
    vmax,kJoint[vJ],vDrive,amax,kJoint[aJ],aDrive,hip/simple::kTickRad,upper/simple::kTickRad,lower/simple::kTickRad,pass?"PASS(static model only)":"FAIL");
  return pass?0:1;
}
