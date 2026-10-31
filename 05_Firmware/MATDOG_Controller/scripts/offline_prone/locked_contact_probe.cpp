// Offline alternative: keep four production contact-FK references fixed,
// vary body pitch/x while placing its front underside on the plane.
#include "../../src/simple/SimpleMotion.h"
#include "../../src/motion/PairedProneData.h"
#include "../../src/motion/TiltCompensation.h"
#include <cstdio>
#include <cmath>
#include <algorithm>
using namespace matdog;
int main(){
 const double lo[12]={-.783864,-.914253,-1.598408,-.783864,-.914253,-1.58,-.783864,-.914253,-1.592272,-.783864,-.914253,-1.590738};
 const double hi[12]={.783864,2.110758,.590583,.783864,2.10002,.581379,.771592,2.136835,.590583,.78233,2.129166,.585981};
 motion::LegJointAngles q[4];motion::Vector3 feet[4];const auto* p=motion::paired_data::kKnots[0].p;
 const double startPitch=.09172596992080767,startZ=.01729126491775537;
 auto body=motion::bodyPoseFromRollPitch({0,0,startZ},0,startPitch);motion::Vector3 normal{-sin(startPitch),0,cos(startPitch)};
 for(unsigned l=0;l<4;++l){q[l]={(l==0||l==3?-1:1)*p[7],p[l<2?2:0],(l<2?-88.5:-89.)*M_PI/180};
  auto f=motion::contactForwardKinematics(static_cast<motion::LegId>(l),q[l],normal);if(f.status!=motion::ContactStatus::OK)return 2;motion::pointToWorld(body,f.referenceM,feet[l]);}
 unsigned feasible=0,solved=0,total=0;double bestMargin=-1e9,bestPitch=0,bestX=0;unsigned failedSlot=99;double bestQ[12]{};
 for(unsigned ip=0;ip<=120;++ip)for(int ix=-40;ix<=40;++ix){
  ++total;const double pitch=ip*.001,x=ix*.0005,z=.11773*sin(pitch);body=motion::bodyPoseFromRollPitch({x,0,z},0,pitch);
  double all[12];bool ok=true;for(unsigned l=0;l<4;++l){motion::TiltedIkOptions o;o.seed=q[l];o.requireNominalStrip=false;
   auto r=motion::worldContactInverseKinematicsTilted(static_cast<motion::LegId>(l),feet[l],body,o);
   if(r.status!=motion::TiltedIkResult::Status::OK){ok=false;break;}all[l*3]=r.base.joints.hip;all[l*3+1]=r.base.joints.upper;all[l*3+2]=r.base.joints.lower;}
  if(!ok)continue;++solved;double margin=1e9;unsigned slot=99;
  for(unsigned i=0;i<12;++i){double a=std::max(lo[i],simple::collision::kEntry.lo[i]),b=std::min(hi[i],simple::collision::kEntry.hi[i]);double m=std::min(all[i]-a,b-all[i]);if(m<margin){margin=m;slot=i;}}
  if(margin>=0){++feasible;printf("CANDIDATE %.9f %.9f %.9f",pitch,x,z);for(double v:all)printf(" %.12f",v);puts("");}if(margin>bestMargin){bestMargin=margin;bestPitch=pitch;bestX=x;failedSlot=slot;std::copy(all,all+12,bestQ);}
 }
 printf("poses=%u solved=%u feasible=%u best_margin_rad=%.9f limiting_slot=%u pitch=%.6f x=%.6f\n",total,solved,feasible,bestMargin,failedSlot,bestPitch,bestX);
 printf("best_q=");for(double v:bestQ)printf(" %.9f",v);puts("");
}
