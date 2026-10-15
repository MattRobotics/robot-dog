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
const char* pairedProneDigest(bool variant){return variant?"ba83dbcb38483d03a9a6054927dc01810ce901bad29e5d1f3c970a831846e70c":paired_data::kRouteSha256;}
double pairedProneDuration(unsigned i){return i<kPairedProneSegments?paired_data::kSegments[i].duration:0;}
PronePhase pairedPronePhase(unsigned i){return static_cast<PronePhase>(i<kPairedProneSegments?paired_data::kSegments[i].phase:4);}
LocomotionFrame pairedProneSample(unsigned i,double u,bool variant,double beginProgress){
 if(i>=kPairedProneSegments||!std::isfinite(beginProgress)||beginProgress<0||beginProgress>=1||!std::isfinite(u)||u<0||u>1)return {};
 const auto law=quinticTimeLaw(u,pairedProneDuration(i));if(!law.valid)return {};
 return pairedProneSampleAt(i,beginProgress+(1-beginProgress)*law.progress,(1-beginProgress)*law.velocity,(1-beginProgress)*law.acceleration,variant);
}
LocomotionFrame pairedProneSampleAt(unsigned i,double progress,double rate,double acceleration,bool variant){
 LocomotionFrame f;
 if(i>=kPairedProneSegments||!std::isfinite(progress)||progress<0||progress>1||!std::isfinite(rate)||!std::isfinite(acceleration))return f;
 double p[8],v[8],a[8];
 for(unsigned j=0;j<8;++j){const double floor=j==3?-88.5*3.14159265358979323846/180:-89.0*3.14159265358979323846/180;
  const bool constrained=variant&&(j==1||j==3);
  const double start=constrained?std::fmax(floor,paired_data::kKnots[i].p[j]):paired_data::kKnots[i].p[j];
  const double end=constrained?std::fmax(floor,paired_data::kKnots[i+1].p[j]):paired_data::kKnots[i+1].p[j],delta=end-start;
  p[j]=start+delta*progress;v[j]=delta*rate;a[j]=delta*acceleration;}
 auto& body=f.cartesian.body;body.translationM={p[5],0,p[6]};
 const double c=std::cos(p[4]),s=std::sin(p[4]);
 body.rotation[0][0]=c;body.rotation[0][2]=s;body.rotation[2][0]=-s;body.rotation[2][2]=c;
 // Linear plane extrema of the complete canonical convex hull equal extrema
 // of all STL vertices. Bound is fixed in flash: 17 hulls, no heap / solver.
 double minimum=0;
 for(unsigned n=0;n<17&&!variant;++n){const auto& hull=paired_data::kHulls[n];
  // Affine plane coefficients once per link. Trigonometry is bounded by 17
  // links, rather than repeated for every flash-resident hull vertex.
  const auto origin=point(n,{0,0,0},p),x=point(n,{1,0,0},p),y=point(n,{0,1,0},p),z=point(n,{0,0,1},p);
  const double offset=-s*origin.x+c*origin.z+p[6];
  const Vector3 plane{-s*(x.x-origin.x)+c*(x.z-origin.z),-s*(y.x-origin.x)+c*(y.z-origin.z),-s*(z.x-origin.x)+c*(z.z-origin.z)};
  for(unsigned j=0;j<hull.count;++j){const auto vtx=hull.vertices[j];minimum=std::fmin(minimum,offset+plane.x*vtx.x+plane.y*vtx.y+plane.z*vtx.z);}}
 if(!variant&&-minimum>1e-5+1e-12)return f; // frozen certificate's unchanged 10 um proximity
 // The suspended derivative carries NO ground/contact certificate. Its
 // displayed body is a virtual geometric reference; actual fixture is external.
 if(!variant)body.translationM.z-=minimum;
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
double canonicalFootPlaneSoftMinimum(LegId leg,const LegJointAngles& q,const BodyPose& body,double scale){
 const unsigned i=static_cast<unsigned>(leg);if(i>=4||!validBodyPose(body))return NAN;
 const auto fk=forwardKinematics(leg,q);if(fk.status!=KinematicsStatus::OK)return NAN;
 if(!std::isfinite(scale)||scale<0||scale>1)return NAN;
 if(scale==0)return canonicalFootPlaneMinimum(leg,q,body);
 const double epsilon=1e-5*scale;
 const auto& hull=paired_data::kHulls[4+i*4];const double minimum=canonicalFootPlaneMinimum(leg,q,body);double sum=0;
 for(unsigned j=0;j<hull.count;++j){const auto v=hull.vertices[j];Vector3 base{
   fk.footOriginM.x+fk.baseFromFootRotation[0][0]*v.x+fk.baseFromFootRotation[0][1]*v.y+fk.baseFromFootRotation[0][2]*v.z,
   fk.footOriginM.y+fk.baseFromFootRotation[1][0]*v.x+fk.baseFromFootRotation[1][1]*v.y+fk.baseFromFootRotation[1][2]*v.z,
   fk.footOriginM.z+fk.baseFromFootRotation[2][0]*v.x+fk.baseFromFootRotation[2][1]*v.y+fk.baseFromFootRotation[2][2]*v.z};
   const double z=body.translationM.z+body.rotation[2][0]*base.x+body.rotation[2][1]*base.y+body.rotation[2][2]*base.z;
   sum+=std::exp((minimum-z)/epsilon);
 }return minimum-epsilon*std::log(sum);
}
double canonicalFootSoftMinimumBound(){double bound=0;for(unsigned i=0;i<4;++i)bound=std::fmax(bound,1e-5*std::log(paired_data::kHulls[4+i*4].count));return bound;}
PairedProneCursor pairedProneCursor(const AdaptiveObservation& o,const FeedbackPolicy& p){
 PairedProneCursor best;double bestError=1e100;
 if(p.mode!=FeedbackMode::FIXTURE_REVIEW||!p.routeTubeReviewed||!o.actual.valid||!validBodyPose(o.body)||p.routeJointTubeRad<=0||p.routeJointTubeRad>.01)return best;
 for(unsigned i=p.routeFirstKnot;i<p.routeLastKnot&&i<251;++i){double lo=0,hi=1;bool fits=true;
  for(unsigned leg=0;leg<4&&fits;++leg){auto a=paired_data::kKnots[i].p,b=paired_data::kKnots[i+1].p;const auto q=o.actual.legs[leg];const double actual[3]={q.hip,q.upper,q.lower};const unsigned slot[3]={7,leg<2?2u:0u,leg<2?3u:1u};
   for(unsigned j=0;j<3;++j){const double sign=j==0&&(leg==0||leg==3)?-1:1;double x=sign*a[slot[j]],y=sign*b[slot[j]];
    if(j==2&&p.fixtureLimitVariant){const double floor=(leg<2?-88.5:-89.0)*3.14159265358979323846/180;x=std::fmax(x,floor);y=std::fmax(y,floor);}
    const double d=y-x;if(!std::isfinite(actual[j])){fits=false;break;}
    if(std::abs(d)<1e-15){if(std::abs(actual[j]-x)>p.routeJointTubeRad)fits=false;}
    else{const double a=(actual[j]-x-p.routeJointTubeRad)/d,b=(actual[j]-x+p.routeJointTubeRad)/d;lo=std::fmax(lo,std::fmin(a,b));hi=std::fmin(hi,std::fmax(a,b));if(lo>hi)fits=false;}
   }
  }
  if(!fits)continue;
  const double alpha=(lo+hi)/2;const auto nominal=pairedProneSample(i,0,p.fixtureLimitVariant,alpha==1?0:alpha);if(!nominal.target.valid)continue;double error=0;
  for(unsigned leg=0;leg<4;++leg){const auto q=o.actual.legs[leg],n=alpha==1?pairedProneSample(i,1,p.fixtureLimitVariant).target.legs[leg]:nominal.target.legs[leg];error=std::fmax(error,std::fmax(std::abs(q.hip-n.hip),std::fmax(std::abs(q.upper-n.upper),std::abs(q.lower-n.lower))));}
  if(error<bestError){bestError=error;best.segment=alpha==1?i+1:i;best.progress=alpha==1?0:alpha;}
 }
 return best;
}
int pairedProneMatch(const AdaptiveObservation& o,const FeedbackPolicy& policy){
 if(!o.actual.valid||!validBodyPose(o.body))return -1;
 if(policy.mode==FeedbackMode::FIXTURE_REVIEW&&policy.routeTubeReviewed){auto cursor=pairedProneCursor(o,policy);return cursor.segment;}
 const bool tube=policy.routeTubeReviewed&&policy.routeJointTubeRad>0&&policy.routeJointTubeRad<=.01;
 const double qtol=tube?policy.routeJointTubeRad:1e-7,ptol=tube?policy.routePositionTubeM:1e-8,rtol=tube?policy.routeRotationTube:1e-7;
 int match=-1;double best=1e100;
 for(unsigned knot=0;knot<=kPairedProneSegments;++knot){
  if(tube&&(knot<policy.routeFirstKnot||knot>policy.routeLastKnot))continue;
  const double* p=paired_data::kKnots[knot].p;bool jointsFit=true;
  for(unsigned leg=0;leg<4;++leg){const auto q=o.actual.legs[leg];
   if(!std::isfinite(q.hip)||!std::isfinite(q.upper)||!std::isfinite(q.lower)||
      std::abs(q.hip-((leg==0||leg==3)?-p[7]:p[7]))>qtol||
      std::abs(q.upper-p[leg<2?2:0])>qtol||std::abs(q.lower-(policy.fixtureLimitVariant?std::fmax(p[leg<2?3:1],(leg<2?-88.5:-89.0)*3.14159265358979323846/180):p[leg<2?3:1]))>qtol)jointsFit=false;}
  if(!jointsFit)continue; // bounded cheap search, hull evaluation only for matches
  const auto f=pairedProneSample(knot==kPairedProneSegments?knot-1:knot,knot==kPairedProneSegments?1:0,policy.fixtureLimitVariant);
  if(!f.target.valid)continue;
  // Route membership must be stricter than the tracking abort threshold.
  // Adjacent observations must not admit an unproved shortcut to a waypoint.
  double error=0;bool fits=true;
  for(unsigned leg=0;leg<4;++leg){const auto x=o.actual.legs[leg],y=f.target.legs[leg];
   const double e=std::fmax(std::abs(x.hip-y.hip),std::fmax(std::abs(x.upper-y.upper),std::abs(x.lower-y.lower)));
   if(e>qtol)fits=false;
   error=std::fmax(error,e);}
  if(policy.mode!=FeedbackMode::FIXTURE_REVIEW&&(std::abs(o.body.translationM.z-f.cartesian.body.translationM.z)>ptol||
     std::hypot(o.body.translationM.x-f.cartesian.body.translationM.x,o.body.translationM.y)>ptol))fits=false;
  for(unsigned a=0;a<3&&policy.mode!=FeedbackMode::FIXTURE_REVIEW;++a)for(unsigned b=0;b<3;++b)if(std::abs(o.body.rotation[a][b]-f.cartesian.body.rotation[a][b])>rtol)fits=false;
  if(fits&&error<best){match=static_cast<int>(knot);best=error;}
 }
 (void)policy;return match;
}
} }
