#include "PairedProneRoute.h"
#include "PairedProneData.h"
#include "TimedStand.h"
#include <cmath>
namespace matdog { namespace motion {
namespace {
Vector3 add(Vector3 a,Vector3 b){return {a.x+b.x,a.y+b.y,a.z+b.z};}
Vector3 ry(Vector3 v,double a){return {std::cos(a)*v.x+std::sin(a)*v.z,v.y,-std::sin(a)*v.x+std::cos(a)*v.z};}
Vector3 rx(Vector3 v,double a){return {v.x,std::cos(a)*v.y-std::sin(a)*v.z,std::sin(a)*v.y+std::cos(a)*v.z};}
Vector3 point(unsigned link,Vector3 v,const double* p){
 if(!link)return v;
 const unsigned leg=(link-1)/4,part=(link-1)%4;
 const auto* m=legModel(static_cast<LegId>(leg));
 const double h=(leg==0||leg==3)?-p[7]:p[7],u=leg<2?p[2]:p[0],l=leg<2?p[3]:p[1];
 if(part==3)v=add(v,m->footOrigin);
 if(part>=2)v=add(ry(v,l),m->lowerOrigin);
 if(part>=1)v=add(ry(v,u),m->upperOrigin);
 return add(rx(v,h),m->hipOrigin);
}
}
const char* pairedProneDigest(){return paired_data::kRouteSha256;}
double pairedProneDuration(unsigned i){return i<kPairedProneSegments?paired_data::kSegments[i].duration:0;}
PronePhase pairedPronePhase(unsigned i){return static_cast<PronePhase>(i<kPairedProneSegments?paired_data::kSegments[i].phase:4);}
LocomotionFrame pairedProneSample(unsigned i,double u){
 LocomotionFrame f;
 if(i>=kPairedProneSegments||!std::isfinite(u)||u<0||u>1)return f;
 const auto law=quinticTimeLaw(u,pairedProneDuration(i));if(!law.valid)return f;
 double p[8],v[8],a[8];
 for(unsigned j=0;j<8;++j){const double start=paired_data::kKnots[i].p[j],delta=paired_data::kKnots[i+1].p[j]-start;
  p[j]=start+delta*law.progress;v[j]=delta*law.velocity;a[j]=delta*law.acceleration;}
 auto& body=f.cartesian.body;body.translationM={p[5],0,p[6]};
 const double c=std::cos(p[4]),s=std::sin(p[4]);
 body.rotation[0][0]=c;body.rotation[0][2]=s;body.rotation[2][0]=-s;body.rotation[2][2]=c;
 // Linear plane extrema of the complete canonical convex hull equal extrema
 // of all STL vertices. Bound is fixed in flash: 17 hulls, no heap / solver.
 double minimum=0;
 for(unsigned n=0;n<17;++n){const auto& hull=paired_data::kHulls[n];
  // Affine plane coefficients once per link. Trigonometry is bounded by 17
  // links, rather than repeated for every flash-resident hull vertex.
  const auto origin=point(n,{0,0,0},p),x=point(n,{1,0,0},p),y=point(n,{0,1,0},p),z=point(n,{0,0,1},p);
  const double offset=-s*origin.x+c*origin.z+p[6];
  const Vector3 plane{-s*(x.x-origin.x)+c*(x.z-origin.z),-s*(y.x-origin.x)+c*(y.z-origin.z),-s*(z.x-origin.x)+c*(z.z-origin.z)};
  for(unsigned j=0;j<hull.count;++j){const auto vtx=hull.vertices[j];minimum=std::fmin(minimum,offset+plane.x*vtx.x+plane.y*vtx.y+plane.z*vtx.z);}}
 if(-minimum>1e-5+1e-12)return f; // frozen certificate's unchanged 10 um proximity
 body.translationM.z-=minimum;
 for(unsigned leg=0;leg<4;++leg){const double sign=(leg==0||leg==3)?-1:1;
  f.target.legs[leg]={sign*p[7],leg<2?p[2]:p[0],leg<2?p[3]:p[1]};
  f.velocityRadS[leg]={sign*v[7],leg<2?v[2]:v[0],leg<2?v[3]:v[1]};
  f.accelerationRadS2[leg]={sign*a[7],leg<2?a[2]:a[0],leg<2?a[3]:a[1]};
  const auto fk=contactForwardKinematics(static_cast<LegId>(leg),f.target.legs[leg],{-s,0,c});
  if(fk.status!=ContactStatus::OK||!pointToWorld(body,fk.referenceM,f.cartesian.feet[leg]))return {};
  // This mask is a demand for independent load witnesses, not proof from FK.
  f.cartesian.phases[leg].stance=static_cast<unsigned>(pairedPronePhase(i))>=3;
 }
 f.target.valid=true;f.cartesian.valid=true;f.status=GaitStatus::OK;return f;
}
double canonicalFootRadialCorrectionBound(){return paired_data::kFootRadialBound;}
double canonicalFootPlaneMinimum(LegId leg,const LegJointAngles& q,const BodyPose& body){
 const unsigned i=static_cast<unsigned>(leg);if(i>=4||!validBodyPose(body))return NAN;
 const auto fk=forwardKinematics(leg,q);if(fk.status!=KinematicsStatus::OK)return NAN;
 const auto& hull=paired_data::kHulls[4+i*4];double minimum=1e100;
 for(unsigned j=0;j<hull.count;++j){const auto v=hull.vertices[j];Vector3 base{
   fk.footOriginM.x+fk.baseFromFootRotation[0][0]*v.x+fk.baseFromFootRotation[0][1]*v.y+fk.baseFromFootRotation[0][2]*v.z,
   fk.footOriginM.y+fk.baseFromFootRotation[1][0]*v.x+fk.baseFromFootRotation[1][1]*v.y+fk.baseFromFootRotation[1][2]*v.z,
   fk.footOriginM.z+fk.baseFromFootRotation[2][0]*v.x+fk.baseFromFootRotation[2][1]*v.y+fk.baseFromFootRotation[2][2]*v.z};
   minimum=std::fmin(minimum,body.translationM.z+body.rotation[2][0]*base.x+body.rotation[2][1]*base.y+body.rotation[2][2]*base.z);
 }return minimum;
}
int pairedProneMatch(const AdaptiveObservation& o,const FeedbackPolicy& policy){
 if(!o.actual.valid||!validBodyPose(o.body))return -1;
 int match=-1;double best=1e100;
 for(unsigned knot=0;knot<=kPairedProneSegments;++knot){
  const double* p=paired_data::kKnots[knot].p;bool jointsFit=true;
  for(unsigned leg=0;leg<4;++leg){const auto q=o.actual.legs[leg];
   if(!std::isfinite(q.hip)||!std::isfinite(q.upper)||!std::isfinite(q.lower)||
      std::abs(q.hip-((leg==0||leg==3)?-p[7]:p[7]))>1e-7||
      std::abs(q.upper-p[leg<2?2:0])>1e-7||std::abs(q.lower-p[leg<2?3:1])>1e-7)jointsFit=false;}
  if(!jointsFit)continue; // bounded cheap search, hull evaluation only for matches
  const auto f=pairedProneSample(knot==kPairedProneSegments?knot-1:knot,knot==kPairedProneSegments?1:0);
  if(!f.target.valid)continue;
  // Route membership must be stricter than the tracking abort threshold.
  // Adjacent observations must not admit an unproved shortcut to a waypoint.
  double error=0;bool fits=true;
  for(unsigned leg=0;leg<4;++leg){const auto x=o.actual.legs[leg],y=f.target.legs[leg];
   const double e=std::fmax(std::abs(x.hip-y.hip),std::fmax(std::abs(x.upper-y.upper),std::abs(x.lower-y.lower)));
   if(e>1e-7)fits=false;
   error=std::fmax(error,e);}
  if(std::abs(o.body.translationM.z-f.cartesian.body.translationM.z)>1e-8||
     std::hypot(o.body.translationM.x-f.cartesian.body.translationM.x,o.body.translationM.y)>1e-8)fits=false;
  for(unsigned a=0;a<3;++a)for(unsigned b=0;b<3;++b)if(std::abs(o.body.rotation[a][b]-f.cartesian.body.rotation[a][b])>1e-7)fits=false;
  if(fits&&error<best){match=static_cast<int>(knot);best=error;}
 }
 (void)policy;return match;
}
} }
