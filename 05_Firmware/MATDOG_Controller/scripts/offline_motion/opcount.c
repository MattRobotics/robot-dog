/* Link-time wrappers counting libm transcendental/sqrt calls (offline cost proxy).
   motion-perf-next: hypot, remainder/fmod and floor/ceil are counted too; every
   one of them is a soft-double library routine on the ESP32-S3. */
#include <stdint.h>
uint64_t op_trig=0,op_inv=0,op_sqrt=0,op_hypot=0,op_rem=0,op_floor=0;
double __real_sin(double),__real_cos(double),__real_asin(double),__real_acos(double),__real_atan2(double,double),__real_sqrt(double),__real_atan(double),__real_tan(double);
void __real_sincos(double,double*,double*);
double __wrap_sin(double x){++op_trig;return __real_sin(x);}
double __wrap_cos(double x){++op_trig;return __real_cos(x);}
void __wrap_sincos(double x,double*s,double*c){op_trig+=2;__real_sincos(x,s,c);}
double __wrap_tan(double x){++op_trig;return __real_tan(x);}
double __wrap_asin(double x){++op_inv;return __real_asin(x);}
double __wrap_acos(double x){++op_inv;return __real_acos(x);}
double __wrap_atan(double x){++op_inv;return __real_atan(x);}
double __wrap_atan2(double y,double x){++op_inv;return __real_atan2(y,x);}
double __wrap_sqrt(double x){++op_sqrt;return __real_sqrt(x);}
double __real_hypot(double,double),__real_remainder(double,double),__real_fmod(double,double),__real_floor(double),__real_ceil(double);
double __wrap_hypot(double x,double y){++op_hypot;return __real_hypot(x,y);}
double __wrap_remainder(double x,double y){++op_rem;return __real_remainder(x,y);}
double __wrap_fmod(double x,double y){++op_rem;return __real_fmod(x,y);}
double __wrap_floor(double x){++op_floor;return __real_floor(x);}
double __wrap_ceil(double x){++op_floor;return __real_ceil(x);}
