#include "../../src/motion/StlContactGait.h"
using namespace matdog::motion;
extern "C" int replay(double cycle,double rate,double acceleration,double shift,double shiftV,double shiftA,int hold,double origin,double stencil,double* output){
 GaitParameters p;p.type=GaitType::WALK;p.heightM=.15;p.liftM=.005;p.duty=.8;p.advanceXM=.005;p.swayXM=.04;
 auto c=hold?gaitHold(p,cycle,.15,shift,0,shiftV,0,shiftA):gaitCartesian(p,cycle,rate,acceleration,1);
 c.body.translationM.x+=origin;for(auto& foot:c.feet)foot.x+=origin;
 auto f=stencil==0?stlContactPosition(p,c,nullptr):solveStlContactGait(p,c,nullptr,stencil);if(!f.target.valid)return static_cast<int>(f.status)+1;
 for(unsigned i=0;i<4;++i){const auto q=f.target.legs[i],v=f.velocityRadS[i],a=f.accelerationRadS2[i];
  output[i*3]=q.hip;output[i*3+1]=q.upper;output[i*3+2]=q.lower;
  output[12+i*3]=v.hip;output[13+i*3]=v.upper;output[14+i*3]=v.lower;
  output[24+i*3]=a.hip;output[25+i*3]=a.upper;output[26+i*3]=a.lower;
 }return 0;
}
