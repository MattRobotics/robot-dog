// Profile build of the QEMU Motion bench (run_qemu_motion.sh --profile only):
// a separate translation unit, outside the .ino prototype generator.
#ifdef MOTION_PROFILE
#include <cstdio>
#include <cstdint>
#include <xtensa/hal.h>
#include "motion_scenarios.h"
// Profile build (run_qemu_motion.sh --profile): every double-precision libgcc
// routine and libm function the Motion objects call is wrapped (-Wl,--wrap) and
// counted; a microbenchmark gives the CCOUNT of one call of each. Counts per
// scenario; wrappers add their own instructions, so totals of this build are
// not the cost figures of the plain build.
#define PROFILED(X) X(__adddf3,double,(double a,double b),(a,b)) X(__subdf3,double,(double a,double b),(a,b)) X(__muldf3,double,(double a,double b),(a,b)) \
  X(__divdf3,double,(double a,double b),(a,b)) X(__ltdf2,int,(double a,double b),(a,b)) X(__gtdf2,int,(double a,double b),(a,b)) X(__ledf2,int,(double a,double b),(a,b)) \
  X(__gedf2,int,(double a,double b),(a,b)) X(__eqdf2,int,(double a,double b),(a,b)) X(__nedf2,int,(double a,double b),(a,b)) X(__unorddf2,int,(double a,double b),(a,b)) \
  X(__floatsidf,double,(int a),(a)) X(__floatunsidf,double,(unsigned a),(a)) X(__fixdfsi,int,(double a),(a)) X(__extendsfdf2,double,(float a),(a)) X(__truncdfsf2,float,(double a),(a)) \
  X(sin,double,(double a),(a)) X(cos,double,(double a),(a)) X(atan2,double,(double a,double b),(a,b)) X(asin,double,(double a),(a)) X(acos,double,(double a),(a)) \
  X(atan,double,(double a),(a)) X(sqrt,double,(double a),(a)) X(hypot,double,(double a,double b),(a,b)) X(floor,double,(double a),(a)) X(ceil,double,(double a),(a)) \
  X(fmod,double,(double a,double b),(a,b)) X(remainder,double,(double a,double b),(a,b)) X(lround,long,(double a),(a))
#define DECL(n,r,args,call) extern "C" r __real_##n args; static uint64_t count_##n=0; extern "C" r __wrap_##n args {++count_##n;return __real_##n call;}
PROFILED(DECL)
static void profile(const char* scenario,uint32_t releases){
  printf("PROFILE %s releases=%lu",scenario,(unsigned long)releases);
#define PRINT(n,r,args,call) if(count_##n)printf(" %s=%.1f",#n,double(count_##n)/(releases?releases:1));count_##n=0;
  PROFILED(PRINT)
  printf("\n");}

// CCOUNT of one call of the main routines (200-call loop; operands fixed).
static void microbench(){
  volatile double a=1.2345,b=0.6789,s=0;volatile float f=1.5f;volatile int i=3;
  auto run=[&](const char* n,auto&& fn){const uint32_t c0=xthal_get_ccount();for(int k=0;k<200;++k)fn();printf(" %s=%lu",n,(unsigned long)((xthal_get_ccount()-c0)/200));};
  printf("MICROBENCH ccount_per_call");
  run("loop_only",[&]{s=s;});
  run("__adddf3",[&]{s=__real___adddf3(a,b);});run("__muldf3",[&]{s=__real___muldf3(a,b);});run("__divdf3",[&]{s=__real___divdf3(a,b);});
  run("__ltdf2",[&]{i=__real___ltdf2(a,b);});run("__floatsidf",[&]{s=__real___floatsidf(i);});run("__extendsfdf2",[&]{s=__real___extendsfdf2(f);});
  run("sin",[&]{s=__real_sin(a);});run("cos",[&]{s=__real_cos(a);});run("atan2",[&]{s=__real_atan2(a,b);});run("asin",[&]{s=__real_asin(b);});run("acos",[&]{s=__real_acos(b);});
  run("sqrt",[&]{s=__real_sqrt(a);});run("hypot",[&]{s=__real_hypot(a,b);});run("floor",[&]{s=__real_floor(a);});run("fmod",[&]{s=__real_fmod(a,b);});run("lround",[&]{i=__real_lround(a);});
  printf("\n");}
void qemuProfileInstall(){qemu_motion::gProfile=profile;microbench();}
#endif
