#pragma once
#include <cmath>
// One time-continuous STAND reference through joint-space via points: straight
// segments joined by quintic velocity blends centred on each via point. The
// velocity is always a convex blend of two cruise velocities, so per joint
// |qdot|<=vmax and |qddot|<=amax hold by construction; position is C3 and every
// coordinate stays inside the [min,max] of its via points (blends cut corners
// inward and never overlap). No arrival is required at an intermediate via:
// the 50 Hz samples are execution samples of this single reference.
namespace matdog { namespace simple {
struct ViaPlan {
  static constexpr unsigned kMax=24, kJoints=12, kDim=15;  // joints + nominal body pitch, x, z
  double p[kMax][kDim]{};
  unsigned n=0;
  // Virtual passage time and blend length at each via; cruise speed and
  // infinity-norm joint length of the segment that ends at each via.
  double t[kMax]{},b[kMax]{},v[kMax]{},d[kMax]{};
  double duration=0;
  unsigned iterations=0;
  void clear(){n=0;duration=0;}
  bool add(const double x[kDim]){
    for(unsigned i=0;i<kDim;++i)if(!std::isfinite(x[i]))return false;
    if(n){double m=0;for(unsigned i=0;i<kJoints;++i)m=std::fmax(m,std::abs(x[i]-p[n-1][i]));
      if(m<1e-9){for(unsigned i=0;i<kDim;++i)p[n-1][i]=x[i];return true;}}
    if(n==kMax)return false;
    for(unsigned i=0;i<kDim;++i)p[n][i]=x[i];
    if(n){double m=0;for(unsigned i=0;i<kJoints;++i)m=std::fmax(m,std::abs(x[i]-p[n-1][i]));d[n]=m;}
    ++n;return true;
  }
  double cruise(unsigned k,unsigned i) const {return k==0||k>=n?0:(p[k][i]-p[k-1][i])*v[k]/d[k];}
  bool plan(double vmax,double amax){
    duration=0;if(!n||!(vmax>0)||!(amax>0))return false;
    if(n==1){t[0]=b[0]=0;return true;}
    for(unsigned k=1;k<n;++k)v[k]=vmax;
    // Short or sharply turning segments slow down until neighbouring blends
    // fit without overlap. Blend length scales with speed and the segment
    // time inversely, so a square-root step converges in a few passes.
    for(iterations=0;;++iterations){
      for(unsigned k=0;k<n;++k){double m=0;for(unsigned i=0;i<kJoints;++i)m=std::fmax(m,std::abs(cruise(k+1,i)-cruise(k,i)));b[k]=1.875*m/amax;}
      bool fits=true;
      for(unsigned k=1;k<n;++k){const double need=.5*(b[k-1]+b[k]),time=d[k]/v[k];
        if(time<need-1e-12){v[k]*=std::fmax(.25,.98*std::sqrt(time/need));fits=false;}}
      if(fits)break;
      if(iterations==64)return false;
    }
    t[0]=.5*b[0];for(unsigned k=1;k<n;++k)t[k]=t[k-1]+d[k]/v[k];
    duration=t[n-1]+.5*b[n-1];return std::isfinite(duration);
  }
  void sample(double tau,double x[kDim],double vel[kDim]=nullptr,double acc[kDim]=nullptr) const {
    tau=std::fmin(std::fmax(tau,0.),duration);
    unsigned k=0;while(k<n&&t[k]<=tau)++k;  // polyline segment k ends at via k
    for(unsigned i=0;i<kDim;++i){
      x[i]=k==0?p[0][i]:k>=n?p[n-1][i]:p[k-1][i]+cruise(k,i)*(tau-t[k-1]);
      if(vel)vel[i]=cruise(k,i);if(acc)acc[i]=0;
    }
    for(unsigned j=0;j<n;++j){
      if(b[j]<=0||std::abs(tau-t[j])>.5*b[j])continue;
      const double u=(tau-t[j])/b[j]+.5,s=u*u*u*(10+u*(-15+6*u)),ds=30*u*u*(1-u)*(1-u),r=u*u*u*u*(2.5+u*(-3+u));
      for(unsigned i=0;i<kDim;++i){const double a=cruise(j,i),w=cruise(j+1,i)-a;
        x[i]+=w*(b[j]*r-std::fmax(0.,tau-t[j]));if(vel)vel[i]=a+w*s;if(acc)acc[i]=w*ds/b[j];}
      break;
    }
  }
};
} }
