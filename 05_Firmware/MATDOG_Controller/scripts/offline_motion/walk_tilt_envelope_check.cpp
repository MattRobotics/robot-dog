// WALK tilt-envelope and cadence check (motion-perf-next 2026-11-07; WALK
// STANDARD v2 2026-11-08), host only. For one WALK STANDARD stride it walks the
// production law (simple::walkGait, optionally reshaped by what-if flags) on its
// 20 ms release grid with the firmware walk clock (walkClockS start ease):
// start ramp, steady cycles, every quarter-cycle WALK STOP terminal with its
// recentring window. At every release it solves the nominal contact IK and, at
// the IMU tilt rays, the levelled contact IK exactly as compensationTarget does
// (headed body, nominal strip, 1e-6 m, 8 Newton steps), then applies the
// compensationSafe geometry: joint limits, the kHeight WALK box before and after
// gen-1 RAW rounding, and the StopPlan proof on the RAW secant with the WALK
// drive. It reports the law's peak joint speed/acceleration (clock jets) against
// the ST3215 RAM drive, the largest per-release RAW step, peak body accelerations
// and the first failing joint of each kind.
//   check (default): rays = vertices, edge midpoints and quarter points of the firmware envelope
//     of this stride (kWalkTilt75/kWalkTilt100 from SimpleMotion.h);
//   --search N: largest feasible tilt on N rays (bisection), printed as an inner
//     convex polygon in C++ initializer form, then re-checked at its vertices
//     and edge midpoints.
// Geometry and the law only: no servo tracking, contact, friction or dynamic
// stability model; a PASS here is not a hardware qualification.
// Usage: see run_walk_tilt_envelope_check.sh.
#include "../../src/simple/SimpleMotion.h"
#include "../../src/motion/TiltCompensation.h"
#include "../../src/motion/TiltedContactIk.h"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>
static int64_t clock_us=1000000;
extern "C" int64_t esp_timer_get_time(){return clock_us;}
using namespace matdog;using namespace matdog::motion;
namespace {
const double kLo[12]={-.783864,-.914253,-1.598408,-.783864,-.914253,-1.58,-.783864,-.914253,-1.592272,-.783864,-.914253,-1.590738};
const double kHi[12]={.783864,2.110758,.590583,.783864,2.10002,.581379,.771592,2.136835,.590583,.78233,2.129166,.585981};
const char* kJoint[12]={"LF_HIP","LF_UPPER","LF_LOWER","RF_HIP","RF_UPPER","RF_LOWER","RH_HIP","RH_UPPER","RH_LOWER","LH_HIP","LH_UPPER","LH_LOWER"};
using V=simple::TiltVertex;
constexpr double kRuntimeMargin=.85;
bool contains(const simple::collision::Box& b,const JointTargetFrame& q){for(unsigned i=0;i<12;++i)if(!(jointSlot(q,i)>=b.lo[i]&&jointSlot(q,i)<=b.hi[i]))return false;return true;}
bool limits(const JointTargetFrame& q){for(unsigned i=0;i<12;++i){const double v=jointSlot(q,i);if(!std::isfinite(v)||v<kLo[i]||v>kHi[i])return false;}return true;}
double marginTicks(const simple::collision::Box& b,const JointTargetFrame& q){double m=1e9;for(unsigned i=0;i<12;++i)m=std::fmin(m,std::fmin(jointSlot(q,i)-b.lo[i],b.hi[i]-jointSlot(q,i)));return m/simple::kTickRad;}
int worstJoint(const simple::collision::Box& b,const JointTargetFrame& q){int w=0;double m=1e9;for(unsigned i=0;i<12;++i){const double d=std::fmin(jointSlot(q,i)-b.lo[i],b.hi[i]-jointSlot(q,i));if(d<m){m=d;w=i;}}return w;}
JointTargetFrame rounded(const JointTargetFrame& q){JointTargetFrame r=q;for(unsigned i=0;i<12;++i)jointSlot(r,i)=std::lround(jointSlot(q,i)/simple::kTickRad)*simple::kTickRad;return r;}
// headed(): the level correction about the support axes composed with the gait heading (SimpleMotion.cpp).
BodyPose headed(const BodyPose& nominal,double roll,double pitch){
  auto b=bodyPoseFromRollPitch({},roll,pitch);double r[3][3];
  for(unsigned i=0;i<3;++i)for(unsigned j=0;j<3;++j)r[i][j]=b.rotation[i][0]*nominal.rotation[0][j]+b.rotation[i][1]*nominal.rotation[1][j]+b.rotation[i][2]*nominal.rotation[2][j];
  for(unsigned i=0;i<3;++i)for(unsigned j=0;j<3;++j)b.rotation[i][j]=r[i][j];return b;
}
bool compensated(const LocomotionFrame& f,double roll,double pitch,JointTargetFrame& q){
  if(roll==0&&pitch==0){q=f.target;return true;}
  auto body=headed(f.cartesian.body,roll,pitch);const auto t=f.cartesian.body.translationM;
  body.translationM={t.x+t.z*body.rotation[0][2],t.y+t.z*body.rotation[1][2],t.z*body.rotation[2][2]};
  q.valid=true;
  for(unsigned l=0;l<4;++l){TiltedIkOptions opt;opt.seed=f.target.legs[l];opt.toleranceM=1e-6;opt.maxIterations=8;opt.requireNominalStrip=true;
    const auto ik=worldContactInverseKinematicsTilted(static_cast<LegId>(l),f.cartesian.feet[l],body,opt);
    if(ik.status!=TiltedIkResult::Status::OK||ik.base.contact.axisTiltRad>footContactModel().nominalTiltRad-.5*simple::kTickRad)return false;
    q.legs[l]=ik.base.joints;}
  return true;
}
struct Law{GaitParameters g;double period;double vmaxDrive,amaxDrive;};
// One release-grid sample of the law: clock in cycles and its first two derivatives.
struct Sample{double cycles,rate,accel,terminal;};
std::vector<std::vector<Sample>> segments(const Law& L){
  std::vector<std::vector<Sample>> out;const auto& g=L.g;const double settle=gaitSettleCycles(g);
  // Start (walkClockS ease), ramp and two steady cycles, no terminal.
  {std::vector<Sample> s;for(double t=0;;t+=.02){const double u=std::fmin(1.,t),c=simple::walkClockS(t)/L.period;if(c>g.startCycles+2)break;
     s.push_back({c,(t<1?u*u*u*(10+u*(-15+6*u)):1)/L.period,(t<1?30*u*u*(1-u)*(1-u):0)/L.period,-1});}out.push_back(s);}
  // Every quarter WALK STOP from the end of the start ramp (a stop requested in
  // the ramp decelerates from stopCycle = startCycles, SimpleMotion::walkStop)
  // through the first steady cycle: steady clock from before the decel cycle to
  // the end of recentring.
  for(double q=0;q<1.5;q+=.25){const double stopCycle=g.startCycles+q,terminal=stopCycle+g.stopCycles;
    std::vector<Sample> s;for(double c=std::fmax(0.,terminal-g.stopCycles-.3);c<=terminal+settle+1e-12;c+=.02/L.period)s.push_back({std::fmin(c,terminal+settle),1/L.period,0,terminal});out.push_back(s);}
  return out;
}
struct Stats{unsigned samples=0,checks=0,failIk=0,failBox=0,failStop=0,failNominal=0;double minBox=1e9,minBoxNominal=1e9,maxStepNom=0,maxStepComp=0,vmax=0,amax=0,ayMax=0,axMax=0,azMax=0;
  int vJoint=-1,aJoint=-1;char firstBox[160]="",firstStop[160]="";};
// All checks of one ray set over the whole phase set. Returns true when nothing failed.
bool run(const Law& L,const std::vector<V>& rays,Stats& st,bool jets){
  const auto& box=simple::kWalkBox;
  for(const auto& seg:segments(L)){
    LocomotionFrame prev;bool have=false;std::vector<JointTargetFrame> prevRay(rays.size()),prevExact(rays.size());std::vector<bool> prevRayValid(rays.size(),false);
    for(const auto& smp:seg){
      auto f=solveGait(L.g,gaitCartesian(L.g,smp.cycles,0,0,smp.terminal),have?&prev:nullptr,false);
      if(!f.target.valid){++st.failNominal;have=false;continue;}
      ++st.samples;st.minBoxNominal=std::fmin(st.minBoxNominal,marginTicks(box,rounded(f.target)));
      if(jets){auto j=solveGait(L.g,gaitCartesian(L.g,smp.cycles,smp.rate,smp.accel,smp.terminal),have?&prev:nullptr,true);
        if(j.target.valid){for(unsigned l=0;l<4;++l){const double v[3]={j.velocityRadS[l].hip,j.velocityRadS[l].upper,j.velocityRadS[l].lower},a[3]={j.accelerationRadS2[l].hip,j.accelerationRadS2[l].upper,j.accelerationRadS2[l].lower};
            for(unsigned k=0;k<3;++k){if(std::abs(v[k])>st.vmax){st.vmax=std::abs(v[k]);st.vJoint=l*3+k;}if(std::abs(a[k])>st.amax){st.amax=std::abs(a[k]);st.aJoint=l*3+k;}}}
          st.ayMax=std::fmax(st.ayMax,std::abs(j.cartesian.bodyAcceleration.y));st.axMax=std::fmax(st.axMax,std::abs(j.cartesian.bodyAcceleration.x));st.azMax=std::fmax(st.azMax,std::abs(j.cartesian.bodyAcceleration.z));}}
      if(have)for(unsigned i=0;i<12;++i)st.maxStepNom=std::fmax(st.maxStepNom,std::abs(jointSlot(f.target,i)-jointSlot(prev.target,i))/simple::kTickRad);
      for(size_t r=0;r<rays.size();++r){++st.checks;JointTargetFrame q;
        if(!compensated(f,rays[r].roll,rays[r].pitch,q)){++st.failIk;prevRayValid[r]=false;continue;}
        const auto d=rounded(q);
        if(!limits(q)||!contains(box,q)||!limits(d)||!contains(box,d)){
          if(!st.failBox++){const int w=worstJoint(box,d);std::snprintf(st.firstBox,sizeof st.firstBox,"%s %.4f rad in [%.4f, %.4f] at c%.3f ray (%.4f, %.4f)",kJoint[w],jointSlot(d,w),box.lo[w],box.hi[w],smp.cycles,rays[r].roll,rays[r].pitch);}
          prevRayValid[r]=false;continue;}
        st.minBox=std::fmin(st.minBox,marginTicks(box,d));
        if(prevRayValid[r]){simple::StopPlan plan;double pos[12],vel[12],acc[12]{},lo[12],hi[12];
          // The firmware reads this secant from RAW references sampled on its own
          // release clock, at any phase of the quantization: a reference moving
          // by dq in a period can show ceil(|dq|/tick) ticks. Prove the STOP with
          // that worst case of the unrounded motion, not this grid's own rounding.
          for(unsigned i=0;i<12;++i){pos[i]=jointSlot(d,i);const double dq=jointSlot(q,i)-jointSlot(prevExact[r],i);
            vel[i]=std::clamp(std::copysign(std::ceil(std::abs(dq)/simple::kTickRad-1e-9),dq)*simple::kTickRad/.02,-L.vmaxDrive,L.vmaxDrive);lo[i]=std::fmax(box.lo[i],kLo[i]);hi[i]=std::fmin(box.hi[i],kHi[i]);
            st.maxStepComp=std::fmax(st.maxStepComp,std::abs(jointSlot(d,i)-jointSlot(prevRay[r],i))/simple::kTickRad);}
          if(!plan.build(pos,vel,acc,lo,hi,L.vmaxDrive,L.amaxDrive)&&!st.failStop++){
            // Attribution: the common duration set by the fastest joint, and the
            // joint whose stop end point (q + t v/2) leaves the box first.
            int w=0;double vm=0;for(unsigned i=0;i<12;++i)if(std::abs(vel[i])>vm){vm=std::abs(vel[i]);w=i;}
            const double t=std::fmax(.04,std::ceil(1.5*vm/L.amaxDrive/.02)*.02);int e=0;double worst=1e9;
            for(unsigned i=0;i<12;++i){const double end=pos[i]+t*vel[i]/2,room=std::fmin(end-lo[i],hi[i]-end);if(room<worst){worst=room;e=i;}}
            std::snprintf(st.firstStop,sizeof st.firstStop,"fastest %s %.3f rad/s sets %.2f s; %s ends %.1f ticks from the box (v %.3f rad/s), at c%.3f ray (%.4f, %.4f)",kJoint[w],vel[w],t,kJoint[e],worst/simple::kTickRad,vel[e],smp.cycles,rays[r].roll,rays[r].pitch);}}
        prevRay[r]=d;prevExact[r]=q;prevRayValid[r]=true;}
      prev=f;have=true;}
  }
  return !st.failNominal&&!st.failIk&&!st.failBox&&!st.failStop;
}
std::vector<V> withMidpoints(const std::vector<V>& v){std::vector<V> r;for(size_t i=0;i<v.size();++i){r.push_back(v[i]);const auto& b=v[(i+1)%v.size()];r.push_back({.5*(v[i].roll+b.roll),.5*(v[i].pitch+b.pitch)});}return r;}
}
int main(int argc,char** argv){
  unsigned stride=2,search=0;Law L{simple::walkGait(2),simple::kWalkPeriodS,simple::kWalkDrive.speed*simple::kTickRad,simple::kWalkDrive.acc*100*simple::kTickRad};
  // What-if flags reshape the production law (model only; the firmware law is SimpleMotion.h).
  struct Opt{const char* name;double* value;double scale;};
  double period=simple::kWalkPeriodS,driveSpeed=simple::kWalkDrive.speed,driveAcc=simple::kWalkDrive.acc;
  for(int i=1;i+1<argc;i+=2){if(!std::strcmp(argv[i],"--stride"))stride=std::atoi(argv[i+1]);}
  if(stride>2)return 2;
  GaitParameters g=simple::walkGait(stride);
  const Opt opts[]={{"--period",&period,1},{"--drive-speed",&driveSpeed,1},{"--drive-acc",&driveAcc,1},{"--duty",&g.duty,1},{"--lift-mm",&g.liftM,1e-3},
    {"--swayx-mm",&g.swayXM,1e-3},{"--swayy-mm",&g.swayYM,1e-3},{"--com-mm",&g.comShiftXM,1e-3},{"--ramp",&g.swingRamp,1},{"--settle",&g.settleCycles,1},
    {"--start",&g.startCycles,1},{"--stop",&g.stopCycles,1},{"--stride-mm",&g.advanceXM,1e-3}};
  std::vector<V> single;
  for(int i=1;i+1<argc;i+=2){if(!std::strcmp(argv[i],"--stride"))continue;if(!std::strcmp(argv[i],"--search")){search=std::atoi(argv[i+1]);continue;}
    if(!std::strcmp(argv[i],"--ray")){V v{};if(std::sscanf(argv[i+1],"%lf,%lf",&v.roll,&v.pitch)!=2)return 2;single.push_back(v);continue;}
    bool known=false;for(const auto& o:opts)if(!std::strcmp(argv[i],o.name)){*o.value=std::atof(argv[i+1])*o.scale;known=true;}if(!known)return 2;}
  L.g=g;L.period=period;L.vmaxDrive=driveSpeed*simple::kTickRad;L.amaxDrive=driveAcc*100*simple::kTickRad;
  if(!validGaitParameters(L.g))return 2;
  std::vector<V> poly;
  if(search){
    // Largest feasible tilt on each ray (12 bisections up to 0.15 rad), inner convex polygon.
    std::vector<V> pts;
    for(unsigned k=0;k<search;++k){const double a=2*3.14159265358979323846*k/search,dr=std::cos(a),dp=std::sin(a);double lo=0,hi=.15;
      for(unsigned it=0;it<12;++it){const double m=.5*(lo+hi);Stats s;if(run(L,{{m*dr,m*dp}},s,false))lo=m;else hi=m;}
      pts.push_back({lo*dr,lo*dp});std::fprintf(stderr,"ray %2u at %5.1f deg: %.4f rad\n",k,a*180/3.14159265358979323846,lo);}
    // Inner convex polygon: a vertex b dented inside the chord of its neighbours
    // pulls the larger neighbour in along its ray until b lies on the chord.
    // Only inward moves (every vertex stays at or below its feasible maximum);
    // the small, binding vertices are kept and the large ones trimmed.
    poly=pts;auto cross=[](V u,V w){return u.roll*w.pitch-u.pitch*w.roll;};
    for(bool changed=true;changed;){changed=false;
      for(size_t i=0;i<poly.size();++i){auto& a=poly[(i+poly.size()-1)%poly.size()];const auto& b=poly[i];auto& c=poly[(i+1)%poly.size()];
        const V ab{b.roll-a.roll,b.pitch-a.pitch},ac{c.roll-a.roll,c.pitch-a.pitch};
        if(cross(ac,ab)<=1e-15)continue;
        const bool trimA=std::hypot(a.roll,a.pitch)>std::hypot(c.roll,c.pitch);
        const double s=trimA?cross(c,b)/(cross(c,a)+cross(a,b)):cross(a,b)/(cross(c,b)+cross(a,c));
        if(!(s>0&&s<1))continue;
        V& t=trimA?a:c;t.roll*=s;t.pitch*=s;changed=true;}}
    // Runtime allowance, applied to the whole polygon: the correction slews
    // (<= kImuSlewRadS), T2 sends interpolated references (<= 0.75 tick) and
    // releases come late, none of which this constant-tilt sweep sees. The v1.0.1 envelopes sat at ~93 % (pitch) and 50-70 % (roll)
    // of the same sweep's maxima; 0.85 is applied uniformly.
    for(auto& p:poly){p.roll*=kRuntimeMargin;p.pitch*=kRuntimeMargin;}
    // Edges must hold between rays too, and with the pessimistic RAW secant the
    // proof is not monotone in the tilt: shrink until vertices, midpoints and
    // quarter points all pass.
    for(unsigned it=0;it<40;++it){Stats s;if(run(L,withMidpoints(withMidpoints(poly)),s,false))break;for(auto& p:poly){p.roll*=.97;p.pitch*=.97;}std::fprintf(stderr,"edge failure: polygon x0.97\n");}
    std::printf("constexpr TiltVertex kWalkTilt%s[]={",stride==2?"100":"75");
    for(size_t i=0;i<poly.size();++i)std::printf("%s{%.6f,%.6f}",i?",":"",poly[i].roll,poly[i].pitch);std::printf("};\n");
  }else if(!single.empty()){
    // --ray roll,pitch: these rays alone (attribution of a single direction).
    Stats st;const bool pass=run(L,single,st,false);
    std::printf("rays %zu: fail nominal/ik/box/stop=%u/%u/%u/%u compensated box margin %.1f ticks %s\n",single.size(),st.failNominal,st.failIk,st.failBox,st.failStop,st.minBox,pass?"PASS":"FAIL");
    if(st.failBox)std::printf("  first box failure: %s\n",st.firstBox);
    if(st.failStop)std::printf("  first StopPlan failure: %s\n",st.firstStop);
    return pass?0:1;
  }else{
    const V* v=stride==2?simple::kWalkTilt100:simple::kWalkTilt75;const size_t n=stride==2?sizeof(simple::kWalkTilt100)/sizeof(V):sizeof(simple::kWalkTilt75)/sizeof(V);
    poly.assign(v,v+n);
  }
  Stats st;auto rays=withMidpoints(withMidpoints(poly));rays.insert(rays.begin(),V{0,0});
  const bool pass=run(L,rays,st,true)&&st.vmax<L.vmaxDrive&&st.amax<L.amaxDrive;
  std::printf("stride=%.0f period=%.2f duty=%.3f lift=%.0f swayx=%.1f com=%.1f ramp=%.2f settle=%.2f start=%.2f stop=%.2f drive=%.0f/%.0f v_cmd=%.2f mm/s samples=%u checks=%u fail nominal/ik/box/stop=%u/%u/%u/%u "
              "box_margin_ticks nominal/compensated=%.2f/%.2f step_ticks/release nominal/compensated=%.2f/%.2f joint vmax=%.3f rad/s %s (drive %.3f) amax=%.2f rad/s2 %s (drive %.2f) "
              "body a_x/a_y/a_z max=%.3f/%.3f/%.3f m/s2 %s\n",
    L.g.advanceXM*1000,L.period,L.g.duty,L.g.liftM*1000,L.g.swayXM*1000,L.g.comShiftXM*1000,L.g.swingRamp,gaitSettleCycles(L.g),L.g.startCycles,L.g.stopCycles,driveSpeed,driveAcc,L.g.advanceXM/L.period*1000,
    st.samples,st.checks,st.failNominal,st.failIk,st.failBox,st.failStop,st.minBoxNominal,st.minBox,st.maxStepNom,st.maxStepComp,
    st.vmax,st.vJoint>=0?kJoint[st.vJoint]:"-",L.vmaxDrive,st.amax,st.aJoint>=0?kJoint[st.aJoint]:"-",L.amaxDrive,st.axMax,st.ayMax,st.azMax,pass?"PASS(geometry+law only)":"FAIL");
  if(st.failBox)std::printf("  first box failure: %s\n",st.firstBox);
  if(st.failStop)std::printf("  first StopPlan failure: %s\n",st.firstStop);
  return pass?0:1;
}
