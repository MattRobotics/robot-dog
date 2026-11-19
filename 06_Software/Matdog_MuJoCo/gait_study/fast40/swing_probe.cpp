// FAST 40 study (MuJoCo M6 worktree, offline, host only): where in the cycle the
// production law's joint rates and kHeight margins come from. Same what-if
// flags as walk_tilt_envelope_check.cpp; one steady cycle after the start ramp,
// 2 ms steps, jets of the law clock (no release grid, no IMU rays, no STOP).
// Prints one CSV row per sample: cycles, per leg (stance, swing u, hip/upper/
// lower rate rad/s, hip/upper/lower margin to kWalkBox in ticks) and the body
// x velocity. Compiled against an unmodified (or patched export) firmware tree.
#include "src/simple/SimpleMotion.h"
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
static int64_t clock_us=0;
extern "C" int64_t esp_timer_get_time(){return clock_us;}
using namespace matdog;using namespace matdog::motion;
int main(int argc,char** argv){
  unsigned modeIndex=2,stride=2;
  for(int i=1;i+1<argc;i+=2){if(!std::strcmp(argv[i],"--stride"))stride=std::atoi(argv[i+1]);if(!std::strcmp(argv[i],"--mode"))modeIndex=std::atoi(argv[i+1]);}
  const auto mode=static_cast<simple::WalkMode>(modeIndex);const auto& spec=simple::walkModeSpec(mode);
  GaitParameters g=simple::walkGait(stride,mode);double period=spec.periodS,height=spec.heightM;
  struct Opt{const char* name;double* value;double scale;};
  const Opt opts[]={{"--period",&period,1},{"--duty",&g.duty,1},{"--lift-mm",&g.liftM,1e-3},{"--swayx-mm",&g.swayXM,1e-3},{"--swayy-mm",&g.swayYM,1e-3},
    {"--com-mm",&g.comShiftXM,1e-3},{"--ramp",&g.swingRamp,1},{"--ramp-end",&g.swingRampEnd,1},{"--stride-mm",&g.advanceXM,1e-3},{"--height-mm",&height,1e-3}};
  for(int i=1;i+1<argc;i+=2){if(!std::strcmp(argv[i],"--stride")||!std::strcmp(argv[i],"--mode"))continue;bool known=false;
    for(const auto& o:opts)if(!std::strcmp(argv[i],o.name)){*o.value=std::atof(argv[i+1])*o.scale;known=true;}if(!known)return 2;}
  g.heightM=simple::baseLinkHeightFromNominalM(height);
  if(!validGaitParameters(g))return 2;
  const auto& box=simple::kWalkBox;
  std::printf("c");for(const char* l:{"LF","RF","RH","LH"})std::printf(",%s_stance,%s_u,%s_vh,%s_vu,%s_vl,%s_mh,%s_mu,%s_ml",l,l,l,l,l,l,l,l);std::printf(",body_vx\n");
  LocomotionFrame prev;bool have=false;
  for(double c=g.startCycles+1;c<g.startCycles+2+1e-9;c+=.002/period){
    auto j=solveGait(g,gaitCartesian(g,c,1/period,0,-1),have?&prev:nullptr,true);
    if(!j.target.valid){std::fprintf(stderr,"solve failed at c%.4f\n",c);return 1;}
    std::printf("%.5f",c);
    for(unsigned l=0;l<4;++l){const auto& ph=j.cartesian.phases[l];const double v[3]={j.velocityRadS[l].hip,j.velocityRadS[l].upper,j.velocityRadS[l].lower};
      std::printf(",%d,%.4f,%.4f,%.4f,%.4f",ph.stance?1:0,ph.swingProgress,v[0],v[1],v[2]);
      for(unsigned k=0;k<3;++k){const unsigned i=l*3+k;const double q=jointSlot(j.target,i);std::printf(",%.2f",std::fmin(q-box.lo[i],box.hi[i]-q)/simple::kTickRad);}}
    std::printf(",%.5f\n",j.cartesian.bodyVelocity.x);
    prev=j;have=true;}
}
