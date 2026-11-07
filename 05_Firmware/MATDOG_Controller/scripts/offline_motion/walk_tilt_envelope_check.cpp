// WALK tilt-envelope and cadence check (motion-perf-next, 2026-11-07), host only.
// For one WALK STANDARD law (stride, start/stop ramp cycles, gait period) it
// walks the production gait law on its 20 ms release grid through the start
// ramp, steady cycles and every quarter-cycle WALK STOP terminal with the
// four-contact settle window, solves the nominal contact IK, and at every
// vertex and edge midpoint of the IMU tilt envelope (kWalkTilt45/60, copied
// from SimpleMotion.cpp) solves the levelled contact IK exactly as
// compensationTarget does (headed body, nominal strip, 1e-6 m, 8 Newton
// steps), then applies the compensationSafe geometry: joint limits, the
// kHeight WALK box before and after gen-1 RAW rounding, and the StopPlan
// proof on the RAW secant with the WALK drive. It also reports, from the
// law's own jets at that period, the peak joint speed/acceleration against
// the ST3215 RAM drive (GoalSpeed 1000 steps/s, ACC 50 x 100 steps/s^2), the
// largest per-release RAW step and the peak body accelerations (lateral
// transfer, fore/aft ramps). Geometry and the law only: no servo tracking,
// contact, friction or dynamic stability model; a PASS here is not a
// hardware qualification. Usage: see run_walk_tilt_envelope_check.sh.
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
struct V{double roll,pitch;};
const V kWalkTilt45[]={{0.014807950855685186,0.03574955578674701},{0,0.11930093706093331},{-0.014656913177372907,0.035384918575352142},
 {-0.011779818742757926,-0.028438998171065412},{-0,-0.053422005857361891},{0.011644818622368949,-0.028113079049515703}};
const V kWalkTilt60[]={{0.014373513909108453,0.014373513909108453},{0.013471848316828587,0.032523918916725118},{0,0.10137685731507248},
 {-0.013436993467988561,0.032439771867942953},{-0.014373513909108453,0.014373513909108453},{-0.0085559368620721186,-0.0035439850870762082},
 {-0.0067072009272594714,-0.0067072009272594714},{-0.0034862036684149191,-0.0084164401774712468},{-0,-0.0084610410728295175},
 {0.0065962325017923174,-0.0065962325017923174},{0.0085559368620717925,-0.0035439850870778045}};
bool contains(const simple::collision::Box& b,const JointTargetFrame& q){for(unsigned i=0;i<12;++i)if(!(jointSlot(q,i)>=b.lo[i]&&jointSlot(q,i)<=b.hi[i]))return false;return true;}
bool limits(const JointTargetFrame& q){for(unsigned i=0;i<12;++i){const double v=jointSlot(q,i);if(!std::isfinite(v)||v<kLo[i]||v>kHi[i])return false;}return true;}
double marginTicks(const simple::collision::Box& b,const JointTargetFrame& q){double m=1e9;for(unsigned i=0;i<12;++i)m=std::fmin(m,std::fmin(jointSlot(q,i)-b.lo[i],b.hi[i]-jointSlot(q,i)));return m/simple::kTickRad;}
JointTargetFrame rounded(const JointTargetFrame& q){JointTargetFrame r=q;for(unsigned i=0;i<12;++i)jointSlot(r,i)=std::lround(jointSlot(q,i)/simple::kTickRad)*simple::kTickRad;return r;}
// headed(): the level correction about the support axes composed with the gait heading (SimpleMotion.cpp).
BodyPose headed(const BodyPose& nominal,double roll,double pitch){
  auto b=bodyPoseFromRollPitch({},roll,pitch);double r[3][3];
  for(unsigned i=0;i<3;++i)for(unsigned j=0;j<3;++j)r[i][j]=b.rotation[i][0]*nominal.rotation[0][j]+b.rotation[i][1]*nominal.rotation[1][j]+b.rotation[i][2]*nominal.rotation[2][j];
  for(unsigned i=0;i<3;++i)for(unsigned j=0;j<3;++j)b.rotation[i][j]=r[i][j];return b;
}
bool compensated(const LocomotionFrame& f,double roll,double pitch,JointTargetFrame& q){
  auto body=headed(f.cartesian.body,roll,pitch);const auto t=f.cartesian.body.translationM;
  body.translationM={t.x+t.z*body.rotation[0][2],t.y+t.z*body.rotation[1][2],t.z*body.rotation[2][2]};
  q.valid=true;
  for(unsigned l=0;l<4;++l){TiltedIkOptions opt;opt.seed=f.target.legs[l];opt.toleranceM=1e-6;opt.maxIterations=8;opt.requireNominalStrip=true;
    const auto ik=worldContactInverseKinematicsTilted(static_cast<LegId>(l),f.cartesian.feet[l],body,opt);
    if(ik.status!=TiltedIkResult::Status::OK||ik.base.contact.axisTiltRad>footContactModel().nominalTiltRad-.5*simple::kTickRad)return false;
    q.legs[l]=ik.base.joints;}
  return true;
}
struct Stats{unsigned samples=0,checks=0,failIk=0,failBox=0,failStop=0,failNominal=0;double minBox=1e9,minBoxNominal=1e9,maxStepNom=0,maxStepComp=0,vmax=0,amax=0,ayMax=0,axMax=0,azMax=0;};
}
int main(int argc,char** argv){
  double strideMm=45,start=simple::kWalkStartCycles,stop=simple::kWalkStopCycles,period=simple::kWalkPeriodS;
  for(int i=1;i+1<argc;i+=2){if(!std::strcmp(argv[i],"--stride"))strideMm=std::atof(argv[i+1]);else if(!std::strcmp(argv[i],"--start"))start=std::atof(argv[i+1]);
    else if(!std::strcmp(argv[i],"--stop"))stop=std::atof(argv[i+1]);else if(!std::strcmp(argv[i],"--period"))period=std::atof(argv[i+1]);else return 2;}
  GaitParameters g;g.heightM=simple::baseLinkHeightFromNominalM(simple::kWalkHeightM);g.liftM=simple::kWalkLiftM;g.duty=simple::kWalkDuty;g.advanceXM=strideMm/1000;
  g.swayXM=simple::kWalkSwayXM;g.swayYM=simple::kWalkSwayYM;g.startCycles=start;g.stopCycles=stop;
  const auto& box=simple::collision::kHeight;
  std::vector<V> rays;auto addRays=[&](const V* v,unsigned n){for(unsigned i=0;i<n;++i){rays.push_back(v[i]);rays.push_back({.5*(v[i].roll+v[(i+1)%n].roll),.5*(v[i].pitch+v[(i+1)%n].pitch)});}};
  if(strideMm>=59.9)addRays(kWalkTilt60,sizeof(kWalkTilt60)/sizeof(V));else addRays(kWalkTilt45,sizeof(kWalkTilt45)/sizeof(V));
  const double step=.02/period,settle=gaitSettleCycles(g);
  const double vmaxDrive=simple::kWalkDrive.speed*simple::kTickRad,amaxDrive=simple::kWalkDrive.acc*100*simple::kTickRad;
  Stats st;
  // Segments: [0, start+1.25] without terminal; and for every quarter terminal after the start ramp a decel+settle.
  struct Seg{double from,to,terminal;};std::vector<Seg> segs{{0,start+1.25,-1}};
  for(double q=0;q<1;q+=.25){const double stopCycle=std::fmax(start,std::ceil((start+.5+q)*4)/4),terminal=stopCycle+stop;segs.push_back({std::fmax(0.,terminal-stop-.3),terminal+settle,terminal});}
  for(const auto& seg:segs){
    LocomotionFrame prev;bool have=false;std::vector<JointTargetFrame> prevRay(rays.size());std::vector<bool> prevRayValid(rays.size(),false);
    for(double c=seg.from;c<=seg.to+1e-12;c+=step){const double cc=std::fmin(c,seg.to);
      auto f=solveGait(g,gaitCartesian(g,cc,0,0,seg.terminal),have?&prev:nullptr,false);
      if(!f.target.valid){++st.failNominal;continue;}
      ++st.samples;st.minBoxNominal=std::fmin(st.minBoxNominal,marginTicks(box,rounded(f.target)));
      // Jets at this period: joint speed/acceleration and body accelerations.
      auto j=solveGait(g,gaitCartesian(g,cc,1/period,0,seg.terminal),have?&prev:nullptr,true);
      if(j.target.valid){for(unsigned l=0;l<4;++l){const double v[3]={j.velocityRadS[l].hip,j.velocityRadS[l].upper,j.velocityRadS[l].lower},a[3]={j.accelerationRadS2[l].hip,j.accelerationRadS2[l].upper,j.accelerationRadS2[l].lower};
          for(unsigned k=0;k<3;++k){st.vmax=std::fmax(st.vmax,std::abs(v[k]));st.amax=std::fmax(st.amax,std::abs(a[k]));}}
        st.ayMax=std::fmax(st.ayMax,std::abs(j.cartesian.bodyAcceleration.y));st.axMax=std::fmax(st.axMax,std::abs(j.cartesian.bodyAcceleration.x));st.azMax=std::fmax(st.azMax,std::abs(j.cartesian.bodyAcceleration.z));}
      if(have)for(unsigned i=0;i<12;++i)st.maxStepNom=std::fmax(st.maxStepNom,std::abs(jointSlot(f.target,i)-jointSlot(prev.target,i))/simple::kTickRad);
      for(size_t r=0;r<rays.size();++r){++st.checks;JointTargetFrame q;
        if(!compensated(f,rays[r].roll,rays[r].pitch,q)){++st.failIk;prevRayValid[r]=false;continue;}
        const auto d=rounded(q);
        if(!limits(q)||!contains(box,q)||!limits(d)||!contains(box,d)){++st.failBox;prevRayValid[r]=false;continue;}
        st.minBox=std::fmin(st.minBox,marginTicks(box,d));
        if(prevRayValid[r]){simple::StopPlan plan;double pos[12],vel[12],acc[12]{},lo[12],hi[12];
          for(unsigned i=0;i<12;++i){pos[i]=jointSlot(d,i);vel[i]=std::clamp((jointSlot(d,i)-jointSlot(prevRay[r],i))/.02,-vmaxDrive,vmaxDrive);lo[i]=std::fmax(box.lo[i],kLo[i]);hi[i]=std::fmin(box.hi[i],kHi[i]);
            st.maxStepComp=std::fmax(st.maxStepComp,std::abs(jointSlot(d,i)-jointSlot(prevRay[r],i))/simple::kTickRad);}
          if(!plan.build(pos,vel,acc,lo,hi,vmaxDrive,amaxDrive))++st.failStop;}
        prevRay[r]=d;prevRayValid[r]=true;}
      prev=f;have=true;}
  }
  const bool pass=!st.failNominal&&!st.failIk&&!st.failBox&&!st.failStop&&st.vmax<vmaxDrive&&st.amax<amaxDrive;
  std::printf("stride=%.0f start=%.2f stop=%.2f period=%.3f v_cmd=%.2f mm/s samples=%u checks=%u fail nominal/ik/box/stop=%u/%u/%u/%u "
              "box_margin_ticks nominal/compensated=%.2f/%.2f step_ticks/release nominal/compensated=%.2f/%.2f joint vmax=%.3f rad/s (drive %.3f) amax=%.2f rad/s2 (drive %.2f) "
              "body a_x/a_y/a_z max=%.3f/%.3f/%.3f m/s2 zmp_shift_y~%.1f mm %s\n",
    strideMm,start,stop,period,strideMm/period,st.samples,st.checks,st.failNominal,st.failIk,st.failBox,st.failStop,st.minBoxNominal,st.minBox,st.maxStepNom,st.maxStepComp,
    st.vmax,vmaxDrive,st.amax,amaxDrive,st.axMax,st.ayMax,st.azMax,st.ayMax/9.81*simple::kWalkHeightM*1000,pass?"PASS(geometry+law only)":"FAIL");
  return pass?0:1;
}
