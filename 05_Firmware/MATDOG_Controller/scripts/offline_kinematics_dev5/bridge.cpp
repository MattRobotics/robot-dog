// Host-only adapter: calls the unchanged production FK/IK and ViaPlan.
#include "../../src/simple/SimpleMotion.h"
#include "../../src/motion/PairedProneData.h"
#include "../../src/motion/TiltCompensation.h"
using namespace matdog;
extern "C" {
void route(unsigned k,double* q){const auto*p=motion::paired_data::kKnots[k].p;for(unsigned l=0;l<4;++l){q[3*l]=(l==0||l==3?-1:1)*p[7];q[3*l+1]=p[l<2?2:0];q[3*l+2]=std::fmax((l<2?-88.5:-89.)*M_PI/180,p[l<2?3:1]);}q[12]=p[4];q[13]=p[5];q[14]=p[6];}
int fk(unsigned l,const double*q,const double*p,double*out){auto b=motion::bodyPoseFromRollPitch({p[1],0,p[2]},0,p[0]);auto r=motion::contactForwardKinematics(static_cast<motion::LegId>(l),{q[0],q[1],q[2]},{-sin(p[0]),0,cos(p[0])});if(r.status!=motion::ContactStatus::OK)return 0;motion::Vector3 v;motion::pointToWorld(b,r.referenceM,v);out[0]=v.x;out[1]=v.y;out[2]=v.z;return 1;}
int ik(unsigned l,const double*foot,const double*p,const double*seed,double*out){auto b=motion::bodyPoseFromRollPitch({p[1],0,p[2]},0,p[0]);motion::TiltedIkOptions o;o.seed={seed[0],seed[1],seed[2]};o.requireNominalStrip=false;auto r=motion::worldContactInverseKinematicsTilted(static_cast<motion::LegId>(l),{foot[0],foot[1],foot[2]},b,o);if(r.status!=motion::TiltedIkResult::Status::OK)return 0;out[0]=r.base.joints.hip;out[1]=r.base.joints.upper;out[2]=r.base.joints.lower;return 1;}
int ik6(unsigned l,const double*foot,const double*p,const double*seed,double*out){auto b=motion::bodyPoseFromRollPitch({p[2],p[3],p[4]},p[0],p[1]);motion::TiltedIkOptions o;o.seed={seed[0],seed[1],seed[2]};o.requireNominalStrip=false;auto r=motion::worldContactInverseKinematicsTilted(static_cast<motion::LegId>(l),{foot[0],foot[1],foot[2]},b,o);if(r.status!=motion::TiltedIkResult::Status::OK)return 0;out[0]=r.base.joints.hip;out[1]=r.base.joints.upper;out[2]=r.base.joints.lower;return 1;}
int sample(const double*via,unsigned n,double t,double*out){simple::ViaPlan p;for(unsigned i=0;i<n;++i)if(!p.add(via+15*i))return 0;if(!p.plan(.55,1.10))return 0;p.sample(t,out);out[15]=p.duration;return 1;}
}
