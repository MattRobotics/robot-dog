#pragma once
#include <cmath>
#include <algorithm>
#include <initializer_list>

namespace matdog { namespace simple {
// Candidate reference arrest, NOT an actuator/contact model. Initial derivatives
// describe the synchronous issued reference, never twelve encoder observations.
// The quartic is C2 internally, joins that reference's quadratic interpolant at
// q,v,a, and ends with v=a=0. No claim of physical C2 tracking is made.
struct StopPlan {
  static constexpr double kMaxDurationS=1.2, kMaxResidualRad=.35;
  double q[12]{},v[12]{},a[12]{},end[12]{},duration=0;
  bool build(const double position[12],const double velocity[12],const double acceleration[12],
             const double lo[12],const double hi[12],double vmax,double amax) {
    duration=0;
    if(!(vmax>0&&amax>0)||!std::isfinite(vmax)||!std::isfinite(amax))return false;
    for(unsigned i=0;i<12;++i){
      q[i]=position[i];v[i]=velocity[i];a[i]=acceleration[i];
      if(!std::isfinite(q[i])||!std::isfinite(v[i])||!std::isfinite(a[i])||
         !(lo[i]<=q[i]&&q[i]<=hi[i]))return false;
    }
    // Choose the shortest 20 ms multiple satisfying the entire polynomial,
    // including extrema between frames. No servo tuning or clipping of q,v,a.
    for(unsigned steps=2;steps<=60;++steps){
      const double t=steps*.02;bool valid=true;
      for(unsigned i=0;i<12&&valid;++i){
        const double c1=a[i]*t,c2=-3*v[i]-2*c1,c3=2*v[i]+c1;
        const double p1=q[i]+t*v[i]/4;
        end[i]=q[i]+t*v[i]/2+t*t*a[i]/12;
        // Quartic Bezier controls q,p1,end,end,end. A convex certified box
        // encloses every continuous reference and asynchronous joint progress.
        for(double p:{q[i],p1,end[i]})if(!std::isfinite(p)||p<lo[i]||p>hi[i]||std::abs(p-q[i])>kMaxResidualRad)valid=false;
        auto velocityAt=[&](double u){return v[i]+u*(c1+u*(c2+u*c3));};
        auto accelerationAt=[&](double u){return (c1+u*(2*c2+u*3*c3))/t;};
        auto checkV=[&](double u){if(u>=0&&u<=1&&std::abs(velocityAt(u))>vmax+1e-12)valid=false;};
        auto checkA=[&](double u){if(u>=0&&u<=1&&std::abs(accelerationAt(u))>amax+1e-12)valid=false;};
        checkV(0);checkV(1);checkA(0);checkA(1);
        if(std::abs(c3)>1e-15){
          checkA(-c2/(3*c3));const double disc=4*c2*c2-12*c3*c1;
          if(disc>=0){checkV((-2*c2+std::sqrt(disc))/(6*c3));checkV((-2*c2-std::sqrt(disc))/(6*c3));}
        }else if(std::abs(c2)>1e-15)checkV(-c1/(2*c2));
      }
      if(valid){duration=t;return true;}
    }
    return false;
  }
  void sample(double seconds,double out[12],double velocity[12]=nullptr,double acceleration[12]=nullptr) const {
    const double u=std::clamp(seconds/duration,0.,1.),u2=u*u,u3=u2*u,u4=u3*u;
    for(unsigned i=0;i<12;++i){
      out[i]=q[i]+duration*v[i]*(u-u3+.5*u4)+duration*duration*a[i]*(.5*u2-2./3*u3+.25*u4);
      if(velocity)velocity[i]=v[i]*(1-3*u2+2*u3)+a[i]*duration*(u-2*u2+u3);
      if(acceleration)acceleration[i]=v[i]*(-6*u+6*u2)/duration+a[i]*(1-4*u+3*u2);
    }
  }
};
} }
