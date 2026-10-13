#include "StlContactGait.h"
#include <cmath>
namespace matdog { namespace motion {
namespace {
LocomotionFrame adapted(const GaitParameters& p,const CartesianSample& input,const LocomotionFrame* previous){
 auto cart=input;double offsets[4]{};
 for(unsigned iteration=0;iteration<5;++iteration){
  for(unsigned i=0;i<4;++i)cart.feet[i].z=input.feet[i].z+offsets[i];
  auto f=solveGait(p,cart,previous);if(!f.target.valid)return f;
  bool converged=true;
  for(unsigned i=0;i<4;++i){const double error=canonicalFootPlaneMinimum(static_cast<LegId>(i),f.target.legs[i],cart.body)-input.feet[i].z;
   if(!std::isfinite(error))return {};
   if(std::abs(error)>1e-10)converged=false;
   offsets[i]-=error;
   if(std::abs(offsets[i])>canonicalFootRadialCorrectionBound()+1e-8)return {};
  }
  if(converged)return f;
 }return {};
}
Vector3 shifted(Vector3 x,Vector3 v,Vector3 a,double t){return {x.x+t*v.x+.5*t*t*a.x,x.y+t*v.y+.5*t*t*a.y,x.z+t*v.z+.5*t*t*a.z};}
CartesianSample stencil(CartesianSample c,double t){
 c.body.translationM=shifted(c.body.translationM,c.bodyVelocity,c.bodyAcceleration,t);
 const double yaw=t*c.yawVelocity+.5*t*t*c.yawAcceleration,cos=std::cos(yaw),sin=std::sin(yaw);
 for(unsigned j=0;j<3;++j){const double x=c.body.rotation[0][j],y=c.body.rotation[1][j];c.body.rotation[0][j]=cos*x-sin*y;c.body.rotation[1][j]=sin*x+cos*y;}
 for(unsigned i=0;i<4;++i)c.feet[i]=shifted(c.feet[i],c.velocity[i],c.acceleration[i],t);
 return c;
}
}
LocomotionFrame stlContactPosition(const GaitParameters& p,const CartesianSample& c,const LocomotionFrame* previous){return adapted(p,c,previous);}
LocomotionFrame solveStlContactGait(const GaitParameters& p,const CartesianSample& c,const LocomotionFrame* previous,double h){
 if(!std::isfinite(h)||h<=0||h>.002)return {};
 auto f=adapted(p,c,previous);if(!f.target.valid)return f;
 const auto left=adapted(p,stencil(c,-h),&f),right=adapted(p,stencil(c,h),&f);
 const auto halfLeft=adapted(p,stencil(c,-h/2),&f),halfRight=adapted(p,stencil(c,h/2),&f);
 if(!left.target.valid||!right.target.valid||!halfLeft.target.valid||!halfRight.target.valid)return {};
 for(unsigned i=0;i<4;++i){const auto l=left.target.legs[i],q=f.target.legs[i],r=right.target.legs[i];
  const auto hl=halfLeft.target.legs[i],hr=halfRight.target.legs[i];
  const double a[3]={l.hip,l.upper,l.lower},b[3]={q.hip,q.upper,q.lower},d[3]={r.hip,r.upper,r.lower};
  const double ha[3]={hl.hip,hl.upper,hl.lower},hd[3]={hr.hip,hr.upper,hr.lower};double v[3],acc[3];
  for(unsigned j=0;j<3;++j){v[j]=(hd[j]-ha[j])/h;acc[j]=4*(hd[j]-2*b[j]+ha[j])/(h*h);
   // A change of the active STL contact vertex can make the complete map
   // nondifferentiable. Reject before submission, rather than declaring the
   // uncorrected G4 derivative qualified or hiding the cusp with a larger h.
   if(std::abs(v[j]-(d[j]-a[j])/(2*h))>1e-7||std::abs(acc[j]-(d[j]-2*b[j]+a[j])/(h*h))>1e-5)return {};
  }
  f.velocityRadS[i]={v[0],v[1],v[2]};f.accelerationRadS2[i]={acc[0],acc[1],acc[2]};
 }
 return f;
}
} }
