#define main simple_motion_main
#include "test_simple_motion.cpp"  // build: g++ -std=c++17 -O2 -w -I scripts/tests/simple_host_stub -I scripts/tests <this> + run_simple_motion_tests.sh sources
#undef main
int main(){
  Rig r;assert(!r.m.stand(.12,r.p.now));assert(r.reachStand());
  const auto& f=simple::StopReferenceProbe::hold(r.m);
  const double act[12]={-0.009204,0.575243,0.042951,-0.001534,0.676486,-0.144194,0.001534,1.348369,-0.641204,-0.009204,1.231787,-0.495476};
  auto pose=[&](double roll,double pitch,double out[12]){
    auto t=f.cartesian.body.translationM;auto body=motion::bodyPoseFromRollPitch(t,roll,pitch);
    body.translationM={t.x+t.z*body.rotation[0][2],t.y+t.z*body.rotation[1][2],t.z*body.rotation[2][2]};
    for(unsigned l=0;l<4;++l){motion::TiltedIkOptions o;o.seed=f.target.legs[l];o.toleranceM=1e-6;o.maxIterations=20;o.requireNominalStrip=false;
      const auto ik=motion::worldContactInverseKinematicsTilted(static_cast<motion::LegId>(l),f.cartesian.feet[l],body,o);
      if(ik.status!=motion::TiltedIkResult::Status::OK)return false;out[l*3]=ik.base.joints.hip;out[l*3+1]=ik.base.joints.upper;out[l*3+2]=ik.base.joints.lower;}
    return true;};
  double best=1e9,br=0,bp=0;double q[12];
  for(double rr=-.25;rr<=.25;rr+=.0025)for(double pp=-.25;pp<=.25;pp+=.0025){if(!pose(rr,pp,q))continue;double e=0;for(unsigned i=0;i<12;++i)e+=(q[i]-act[i])*(q[i]-act[i]);if(e<best){best=e;br=rr;bp=pp;}}
  pose(br,bp,q);std::printf("best roll %+.4f pitch %+.4f rad (%.2f/%.2f deg), rms %.4f rad\n",br,bp,br*57.2958,bp*57.2958,std::sqrt(best/12));
  for(unsigned i=0;i<12;++i)std::printf("%2u fit %+.3f fault %+.3f canon %+.3f d %+5.1f ticks\n",i,q[i],act[i],motion::jointSlot(f.target,i),(act[i]-q[i])/simple::kTickRad);
}
