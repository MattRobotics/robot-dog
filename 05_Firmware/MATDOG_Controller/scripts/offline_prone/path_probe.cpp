// Study-only executable: production ViaPlan; no bus, controller or hardware I/O.
#include "../../src/simple/SimpleMotion.h"
#include "../../src/motion/PairedProneData.h"
#include <cstdio>
#include <cmath>
#include <cassert>
using namespace matdog;
void knot(unsigned k,double* q){const auto* p=motion::paired_data::kKnots[k].p;
 for(unsigned l=0;l<4;++l){q[l*3]=(l==0||l==3?-1:1)*p[7];q[l*3+1]=p[l<2?2:0];q[l*3+2]=std::fmax((l<2?(-88.5*M_PI/180+.020):(-89.*M_PI/180)),p[l<2?3:1]);}
 q[12]=p[4];q[13]=p[5];q[14]=p[6];}
int main(){
 const double lo[12]={-.783864,-.914253,-1.598408,-.783864,-.914253,-1.58,-.783864,-.914253,-1.592272,-.783864,-.914253,-1.590738};
 const double hi[12]={.783864,2.110758,.590583,.783864,2.10002,.581379,.771592,2.136835,.590583,.78233,2.129166,.585981};
 puts("segment,t,pitch,body_x,body_z,box_margin,joint_margin,q0,q1,q2,q3,q4,q5,q6,q7,q8,q9,q10,q11");
 double prev[15]{};bool have=false;
 for(unsigned group=0;group<3;++group){simple::ViaPlan plan;const simple::collision::Box* box=nullptr;
  if(group<2){const unsigned* ks=group==0?simple::collision::kLiftKnots:simple::collision::kDeployKnots;unsigned n=group==0?7:11;
   box=group==0?&simple::collision::kLift:&simple::collision::kDeploy;
   if(group==0){auto d=motion::canonicalStandDefinition();d.lowBodyHeightM=.053/2;d.standBodyHeightM=.053;auto stand=motion::generateStandTarget(d);assert(stand.target.valid);double q[15]{};for(unsigned i=0;i<12;++i)q[i]=motion::jointSlot(stand.target,i);q[14]=.053;assert(plan.add(q));}
   for(unsigned j=n;j-->0;){double q[15];knot(ks[j],q);assert(plan.add(q));}}
  else {box=&simple::collision::kEntry;double q[15];knot(0,q);assert(plan.add(q));
   // Purpose-built final closure, not inversion of DISENGAGE beyond limits.
   // Endpoint body pose is rigid CAD rest, not a guaranteed physical pose.
   for(unsigned l=0;l<4;++l)q[3*l]=(l==0||l==3?1:-1)*.17;
   q[12]=.08183102100559651;q[13]=0;q[14]=.009626496609908074;assert(plan.add(q));}
  const double speed=group==2?.10:simple::kStandSpeedRadS,acc=group==2?.20:simple::kStandAccelerationRadS2;
  assert(plan.plan(speed,acc));double q[15],v[15],a[15];plan.sample(0,q);
  if(have)for(unsigned i=0;i<12;++i)assert(std::abs(prev[i]-q[i])<1e-12);
  double lastTime=0;
  for(double t=0;;t=std::fmin(t+.01,plan.duration)){
   plan.sample(t,q,v,a);double bm=1,jm=1;
   for(unsigned i=0;i<12;++i){bm=std::fmin(bm,std::fmin(q[i]-box->lo[i],box->hi[i]-q[i]));jm=std::fmin(jm,std::fmin(q[i]-lo[i],hi[i]-q[i]));
    assert(std::abs(v[i])<=speed+1e-8&&std::abs(a[i])<=acc+1e-8);
    if(t>0)assert(std::abs(q[i]-prev[i])<=speed*(t-lastTime)+1e-8);
    if(t==0||t==plan.duration)assert(std::abs(v[i])+std::abs(a[i])<1e-8);prev[i]=q[i];}
   assert(bm>=0&&jm>=0);printf("%u,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f",group,t,q[12],q[13],q[14],bm,jm);
   for(unsigned i=0;i<12;++i)printf(",%.12f",q[i]);puts("");lastTime=t;if(t==plan.duration)break;
  }have=true;
 }
}
