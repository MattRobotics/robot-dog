// JOYSTICK envelope, offline (dev.15). Drives the production SimpleMotion with
// scripted joystick commands (ideal tracking, flat IMU) and checks every WALK
// release of the nominal law: joint margin to the walk box and gen-1, foot
// axis tilt, and the IMU tilt envelope (rays to the boundary of the steered
// envelope, tilted contact IK with the fillet lift, RAW quantization). Writes
// the frames in the body heading frame for the URDF static-support and
// clearance checks, and the per-joint envelope used by certify_steer_box.py.
// Geometry and software sequencing only: no servo, contact or dynamic model.
#include "../../src/simple/SimpleMotion.h"
#include "../../src/motion/TiltCompensation.h"
#include "../../src/motion/TiltedContactIk.h"
#include "../../src/motion/PairedProneData.h"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>
static int64_t clock_us=1000000;
extern "C" int64_t esp_timer_get_time(){return clock_us;}
using namespace matdog;
namespace matdog { namespace simple {
struct StopReferenceProbe {
  static const motion::LocomotionFrame& walkNominal(const SimpleMotion& m){return m.walkPrevious_;}
  static const motion::GaitParameters& gait(const SimpleMotion& m){return m.gait_;}
  static const collision::Box& box(const SimpleMotion& m){return m.walkBox();}
};
} }
namespace {
const double kLo[12]={-.783864,-.914253,-1.598408,-.783864,-.914253,-1.58,-.783864,-.914253,-1.592272,-.783864,-.914253,-1.590738};
const double kHi[12]={.783864,2.110758,.590583,.783864,2.10002,.581379,.771592,2.136835,.590583,.78233,2.129166,.585981};
// Same polygon as kSteerTilt in SimpleMotion.cpp; boundary by rays.
struct V{double r,p;};
const V kSteerTilt[]={{0.0245,0.0245},{0,0.07},{-0.0245,0.0245},{-0.0245,-0.0245},{0,-0.042},{0.0245,-0.0245}};
template<unsigned N> double rayScale(const V (&v)[N],double r,double p){double s=1e9;
  for(unsigned i=0;i<N;++i){const auto a=v[i],b=v[(i+1)%N];const double x=b.r-a.r,y=b.p-a.p,e=x*a.p-y*a.r,ray=x*p-y*r;if(ray<0)s=std::fmin(s,e/ray);}
  return s;}
struct Port: simple::ServoPort {
  simple::Feedback o[12]{};uint16_t t[12]{};simple::Drive d{};uint64_t now=1000000;
  Port(){const auto* p=motion::paired_data::kKnots[0].p;
    for(unsigned l=0;l<4;++l){const double q[3]={((l==0||l==3)?-1:1)*p[7],p[l<2?2:0],std::fmax((l<2?-88.5:-89.)*M_PI/180,p[l<2?3:1])};
      for(unsigned j=0;j<3;++j){auto& f=o[l*3+j];f.q=q[j];f.raw=2048+std::lround(q[j]/simple::kTickRad);f.torque=0;f.temp=30;f.voltage=120;f.valid=true;t[l*3+j]=f.raw;}}}
  bool read(unsigned i,bool control,simple::Feedback& out) override{out=o[i];out.stampUs=now;out.goal=t[i];out.goalSpeed=d.speed;out.acc=d.acc;if(control)out.controlStampUs=now;return true;}
  bool resolve(unsigned,double q,uint16_t& raw) override{raw=2048+std::lround(q/simple::kTickRad);return raw<4096;}
  bool decodeRaw(unsigned,uint16_t raw,double& q) override{q=(int(raw)-2048)*simple::kTickRad;return raw<4096;}
  bool frame(const uint16_t raw[12],simple::Drive dr) override{d=dr;std::copy(raw,raw+12,t);return true;}
  bool enable(unsigned i) override{o[i].torque=1;return true;}
  bool off(unsigned) override{for(auto& f:o)f.torque=0;return true;}
  bool verifyOff(unsigned i) override{return i==12||o[i].torque==0;}
  void publish(){for(unsigned i=0;i<12;++i){o[i].raw=t[i];o[i].q=(int(t[i])-2048)*simple::kTickRad;}}
};
struct Result{double footLo=1e9,footHi=-1e9,box=1e9,gen1=1e9,tilt=1e9,footTilt=0,nominalTilt=0,liftErr=0,maxStep=0;unsigned frames=0,tiltFail=0;double lo[12],hi[12];std::string end;
  Result(){for(unsigned i=0;i<12;++i){lo[i]=1e9;hi[i]=-1e9;}}
  void joint(unsigned i,double q){lo[i]=std::fmin(lo[i],q);hi[i]=std::fmax(hi[i],q);}};
}
int main(int argc,char** argv){
  // scenario: holds "seconds:vx[,vy,yaw_rate]" (SI); vx '-' = silent client;
  // "rand:SEED:SECONDS" = random commands inside the Controller ellipsoid.
  if(argc<3){std::fprintf(stderr,"usage: joystick_envelope out.txt hold|rand ...\n");return 2;}
  FILE* out=std::fopen(argv[1],"w");if(!out)return 2;
  const double T=simple::kWalkPeriodS,lim[3]={simple::kJoystickStrideM/T,simple::kJoystickLateralM/T,simple::kJoystickYawRad/T};
  struct Step{double seconds,v[3];bool silent;int walk=-1;};std::vector<Step> steps;  // walk: WALK STANDARD preset instead of a joystick command
  for(int a=2;a<argc;++a){unsigned seed=0;double total=0;
    if(std::sscanf(argv[a],"rand:%u:%lf",&seed,&total)==2){std::srand(seed);
      for(double t=0;t<total;){Step s{.25+std::rand()%700/100.,{0,0,0},false};double n=0;
        for(double& v:s.v){v=std::rand()%2001/1000.-1;n+=v*v;}
        n=std::sqrt(n);const double radius=std::rand()%3==0?1:std::rand()%1001/1000.;for(unsigned j=0;j<3;++j)s.v[j]*=radius/n*lim[j];
        steps.push_back(s);t+=s.seconds;}
      steps.push_back({1,{0,0,0},false});}
    else if(std::sscanf(argv[a],"walk:%d:%lf",&seed,&total)==2){Step s{total,{0,0,0},false};s.walk=int(seed);steps.push_back(s);}  // "walk:<preset>:<seconds>"
    else{Step s{0,{0,0,0},false};char v[64]{};if(std::sscanf(argv[a],"%lf:%63s",&s.seconds,v)!=2)return 2;
      if(!std::strcmp(v,"-"))s.silent=true;else std::sscanf(v,"%lf,%lf,%lf",&s.v[0],&s.v[1],&s.v[2]);steps.push_back(s);}}
  simple::SimpleMotion m;Port port;simple::Inputs in;m.begin(&port);
  in.ready=in.powerOk=in.authorityOk=in.imuValid=true;in.refusal="READY";std::copy(kLo,kLo+12,in.minRad);std::copy(kHi,kHi+12,in.maxRad);
  auto tick=[&](){port.now+=simple::kPeriodUs;clock_us=port.now;in.imuStampUs=port.now;m.inputs(in);port.publish();m.update(port.now);};
  for(int i=0;i<15;++i)tick();
  if(m.stand(.120,port.now)){std::puts("STAND refused");return 1;}
  for(int i=0;i<3000&&m.state()!=simple::State::STANDING;++i)tick();
  if(m.state()!=simple::State::STANDING){std::puts("STAND not reached");return 1;}
  Result r;motion::JointTargetFrame last{};
  for(const auto& st:steps){
    for(int n=0;n<std::lround(st.seconds*50);++n){
      if(st.walk>=0){if(n==0)m.walk(unsigned(st.walk),port.now);}
      else if(!st.silent&&n%12==0){if(st.v[0]==0&&st.v[1]==0&&st.v[2]==0)m.walkStop();else m.drive(st.v[0],st.v[1],st.v[2],port.now);}  // dashboard refresh 240 ms
      tick();
      if(m.state()==simple::State::FAULT){r.end=std::string("FAULT ")+m.reason();break;}
      if(std::strcmp(m.phase(),"WALK_STANDARD"))continue;
      const auto& f=simple::StopReferenceProbe::walkNominal(m);if(!f.target.valid)continue;
      const auto& g=simple::StopReferenceProbe::gait(m);const auto& box=simple::StopReferenceProbe::box(m);++r.frames;
      for(unsigned i=0;i<12;++i){const double q=motion::jointSlot(f.target,i);r.joint(i,q);
        r.box=std::fmin(r.box,std::fmin(q-box.lo[i],box.hi[i]-q));r.gen1=std::fmin(r.gen1,std::fmin(q-kLo[i],kHi[i]-q));
        if(last.valid)r.maxStep=std::fmax(r.maxStep,std::abs(q-motion::jointSlot(last,i)));}
      last=f.target;
      const auto& b=f.cartesian.body;const double cy=b.rotation[0][0],sy=b.rotation[1][0];const auto& c=motion::canonicalStandDefinition().contactsWorldM;
      for(unsigned l=0;l<4;++l)if(f.cartesian.phases[l].stance){const double dx=f.cartesian.feet[l].x-b.translationM.x,dy=f.cartesian.feet[l].y-b.translationM.y;
        const double x=cy*dx+sy*dy-c[l].x;r.footLo=std::fmin(r.footLo,x);r.footHi=std::fmax(r.footHi,x);r.nominalTilt=std::fmax(r.nominalTilt,std::abs(f.target.legs[l].hip));}
      // IMU tilt envelope: 32 rays to its boundary (heading frame), same lift and checks as compensationTarget.
      for(unsigned k=0;k<32;++k){const double ang=2*M_PI*k/32,dr=std::cos(ang),dp=std::sin(ang);
        // Rays on the certified polygon in the heading (body) frame, taken into
        // the support frame like SimpleMotion::limitWalkTilt (the hip/fillet
        // loading they bound does not depend on the accumulated heading).
        const double sc=rayScale(kSteerTilt,dr,dp),rb=sc*dr,pb=sc*dp,roll=cy*rb-sy*pb,pitch=sy*rb+cy*pb;
        auto body=motion::bodyPoseFromRollPitch({},roll,pitch);const auto t=b.translationM;
        {double rr[3][3];for(unsigned i=0;i<3;++i)for(unsigned j=0;j<3;++j)rr[i][j]=body.rotation[i][0]*b.rotation[0][j]+body.rotation[i][1]*b.rotation[1][j]+body.rotation[i][2]*b.rotation[2][j];
         for(unsigned i=0;i<3;++i)for(unsigned j=0;j<3;++j)body.rotation[i][j]=rr[i][j];}  // same heading as compensateLevel
        body.translationM={t.x+t.z*body.rotation[0][2],t.y+t.z*body.rotation[1][2],t.z*body.rotation[2][2]};
        bool ok=true;
        for(unsigned l=0;l<4&&ok;++l){motion::TiltedIkOptions opt;opt.seed=f.target.legs[l];opt.toleranceM=1e-6;opt.maxIterations=8;opt.requireNominalStrip=g.edgeTiltRad==0;
          const auto ik=g.edgeTiltRad>0?motion::worldEdgeContactInverseKinematicsTilted(static_cast<motion::LegId>(l),f.cartesian.feet[l],body,opt):
                                        motion::worldContactInverseKinematicsTilted(static_cast<motion::LegId>(l),f.cartesian.feet[l],body,opt);
          const double limit=g.edgeTiltRad>0?g.edgeTiltRad:motion::footContactModel().nominalTiltRad;
          if(ik.status!=motion::TiltedIkResult::Status::OK||ik.base.contact.axisTiltRad>limit-.5*simple::kTickRad){ok=false;break;}
          r.footTilt=std::fmax(r.footTilt,ik.base.contact.axisTiltRad);
          if(g.edgeTiltRad>0)r.liftErr=std::fmax(r.liftErr,std::abs(ik.achievedWorldM.z-f.cartesian.feet[l].z-motion::edgeContactLiftM(ik.base.contact.axisTiltRad)));
          const double q3[3]={ik.base.joints.hip,ik.base.joints.upper,ik.base.joints.lower};
          for(unsigned j=0;j<3;++j){const unsigned i=l*3+j;const double q=std::lround(q3[j]/simple::kTickRad)*simple::kTickRad;r.joint(i,q);
            const double mg=std::fmin(std::fmin(q-box.lo[i],box.hi[i]-q),std::fmin(q-kLo[i],kHi[i]-q));r.tilt=std::fmin(r.tilt,mg);if(mg<0)ok=false;}}
        if(!ok){++r.tiltFail;if(std::getenv("JOY_DEBUG"))std::fprintf(stderr,"tilt fail c=%.4f roll=%.4f pitch=%.4f\n",f.cartesian.cycles,roll,pitch);}}
      // walk_quality.py row in the body heading frame: cycles, body xyz, q12, feet (x,y,z,stance)x4
      std::fprintf(out,"%.9f 0 0 %.9f",f.cartesian.cycles,b.translationM.z);
      for(unsigned i=0;i<12;++i)std::fprintf(out," %.9f",motion::jointSlot(f.target,i));
      for(unsigned l=0;l<4;++l){const double dx=f.cartesian.feet[l].x-b.translationM.x,dy=f.cartesian.feet[l].y-b.translationM.y;
        std::fprintf(out," %.9f %.9f %.9f %d",cy*dx+sy*dy,-sy*dx+cy*dy,f.cartesian.feet[l].z,f.cartesian.phases[l].stance?1:0);}
      std::fprintf(out,"\n");
    }
    if(!r.end.empty())break;
  }
  for(int n=0;n<50*60&&r.end.empty()&&m.state()!=simple::State::STANDING;++n){tick();if(m.state()==simple::State::FAULT)r.end=std::string("FAULT ")+m.reason();}
  if(r.end.empty())r.end=std::string(simple::name(m.state()))+" "+m.reason();
  std::fclose(out);
  if(const char* env=std::getenv("JOY_ENVELOPE")){FILE* e=std::fopen(env,"a");if(e){for(unsigned i=0;i<12;++i)std::fprintf(e,"%s%.9f %.9f",i?" ":"",r.lo[i],r.hi[i]);std::fprintf(e,"\n");std::fclose(e);}}
  if(!r.frames){std::printf("frames=0 walk=NOT_APPLICABLE end=%s\n",r.end.c_str());return r.end.rfind("FAULT",0)==0?1:0;}  // never reached WALK_STANDARD
  std::printf("foot_x=[%.1f,%.1f] mm frames=%u box_margin=%.1f mrad gen1_margin=%.1f mrad tilt_margin=%.1f mrad tilt_fail=%u foot_tilt nominal/compensated=%.2f/%.2f deg lift_err=%.1f um max_target_step=%.5f rad end=%s\n",
    r.footLo*1e3,r.footHi*1e3,r.frames,r.box*1e3,r.gen1*1e3,r.tilt*1e3,r.tiltFail,r.nominalTilt*180/M_PI,r.footTilt*180/M_PI,r.liftErr*1e6,r.maxStep,r.end.c_str());
  return r.end.rfind("FAULT",0)==0||r.tiltFail||r.box<0?1:0;
}
