/* Link-time wrappers counting libm transcendental/sqrt calls (offline cost proxy). */
#include <stdint.h>
uint64_t op_trig=0,op_inv=0,op_sqrt=0;
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
