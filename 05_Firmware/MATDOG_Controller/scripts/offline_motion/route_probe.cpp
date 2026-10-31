// Offline only: production ViaPlan and contact FK. No bus/controller instance.
#include "../../src/simple/SimpleMotion.h"
#include "../../src/motion/PairedProneData.h"
#include "../../src/motion/TiltCompensation.h"
#include <cstdio>
#include <cmath>
#include <cassert>
using namespace matdog;
void knot(unsigned k,double* q,bool early){
 const double* p=motion::paired_data::kKnots[k].p;
 // Study only: begin front deploy at 43, after initial rear clearance, then
 // lead by up to 10 source knots and rejoin at 100. Body/rear unchanged.
 double f=k;
 if(early&&k>42&&k<100)f=k+10*std::sin(3.141592653589793*(k-42)/58.);
 unsigned a=unsigned(f),b=std::min(251u,a+1);double u=f-a;
 for(unsigned l=0;l<4;++l){q[l*3]=((l==0||l==3)?-1:1)*p[7];
  q[l*3+1]=l<2?(1-u)*motion::paired_data::kKnots[a].p[2]+u*motion::paired_data::kKnots[b].p[2]:p[0];
  q[l*3+2]=std::fmax((l<2?-88.5:-89.)*3.141592653589793/180,l<2?(1-u)*motion::paired_data::kKnots[a].p[3]+u*motion::paired_data::kKnots[b].p[3]:p[1]);}
 q[12]=p[4];q[13]=p[5];q[14]=p[6];
}
void dump(const char* name,int group,double t,const double* q){
 const auto& box=group?simple::collision::kLift:simple::collision::kDeploy;double margin=1e9;
 for(unsigned i=0;i<12;++i)margin=std::fmin(margin,std::fmin(q[i]-box.lo[i],box.hi[i]-q[i]));
 printf("%s,%d,%.6f,%.9f,%.9f,%.9f,%.9f",name,group,t,q[12],q[13],q[14],margin);
 for(unsigned i=0;i<12;++i)printf(",%.9f",q[i]);
 auto body=motion::bodyPoseFromRollPitch({q[13],0,q[14]},0,q[12]);motion::Vector3 n{-std::sin(q[12]),0,std::cos(q[12])};
 for(unsigned l=0;l<4;++l){auto c=motion::contactForwardKinematics(static_cast<motion::LegId>(l),{q[l*3],q[l*3+1],q[l*3+2]},n);motion::Vector3 w;
 assert(c.status==motion::ContactStatus::OK&&motion::pointToWorld(body,c.referenceM,w));printf(",%.9f,%.9f,%.9f",w.x,w.y,w.z);}
 puts("");
}
int main(){
 printf("route,group,t,pitch,body_x,body_z,box_margin");for(int i=0;i<12;++i)printf(",q%d",i);for(int l=0;l<4;++l)printf(",f%dx,f%dy,f%dz",l,l,l);puts("");
 for(int early=0;early<2;++early)for(int group=0;group<2;++group){simple::ViaPlan p;
  const unsigned *ks=group?simple::collision::kLiftKnots:simple::collision::kDeployKnots;unsigned n=group?7:11;
  for(unsigned j=0;j<n;++j){double q[15];knot(ks[j],q,early);assert(p.add(q));}assert(p.plan(simple::kStandSpeedRadS,simple::kStandAccelerationRadS2));
  for(double t=0;;t=std::fmin(t+.01,p.duration)){double q[15];p.sample(t,q);dump(early?"front_early":"current",group,t,q);if(t==p.duration)break;}
  if(!early){simple::ViaPlan reverse;for(unsigned j=n;j-->0;){double q[15];knot(ks[j],q,false);assert(reverse.add(q));}assert(reverse.plan(simple::kStandSpeedRadS,simple::kStandAccelerationRadS2));
   for(double t=0;;t=std::fmin(t+.01,reverse.duration)){double q[15];reverse.sample(t,q);dump("reverse",group,t,q);if(t==reverse.duration)break;}}
 }
}
