// JOYSTICK envelope, offline (dev.15). Drives the production SimpleMotion with
// scripted joystick commands (ideal tracking, flat IMU) and checks every WALK
// release of the nominal law: joint margin to the WALK box and to gen-1, the
// IMU tilt envelope of a steered walk (rays to the boundary of both preset
// envelopes, tilted contact IK with the nominal strip, RAW quantization) and
// writes the frames for the URDF static-support check (walk_quality.py).
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
  static double stride(const SimpleMotion& m){return m.gait_.advanceXM;}
};
} }
namespace {
const double kLo[12]={-.783864,-.914253,-1.598408,-.783864,-.914253,-1.58,-.783864,-.914253,-1.592272,-.783864,-.914253,-1.590738};
const double kHi[12]={.783864,2.110758,.590583,.783864,2.10002,.581379,.771592,2.136835,.590583,.78233,2.129166,.585981};
// Same envelopes as SimpleMotion.cpp (kWalkTilt45/kWalkTilt60), boundary by rays.
struct V{double r,p;};
const V k45[]={{0.014807950855685186,0.03574955578674701},{0,0.11930093706093331},{-0.014656913177372907,0.035384918575352142},
 {-0.011779818742757926,-0.028438998171065412},{-0,-0.053422005857361891},{0.011644818622368949,-0.028113079049515703}};
const V k60[]={{0.014373513909108453,0.014373513909108453},{0.013471848316828587,0.032523918916725118},{0,0.10137685731507248},
 {-0.013436993467988561,0.032439771867942953},{-0.014373513909108453,0.014373513909108453},{-0.0085559368620721186,-0.0035439850870762082},
 {-0.0067072009272594714,-0.0067072009272594714},{-0.0034862036684149191,-0.0084164401774712468},{-0,-0.0084610410728295175},
 {0.0065962325017923174,-0.0065962325017923174},{0.0085559368620717925,-0.0035439850870778045}};
template<unsigned N> double rayScale(const V (&v)[N],double r,double p){double s=1e9;
  for(unsigned i=0;i<N;++i){const auto a=v[i],b=v[(i+1)%N];const double x=b.r-a.r,y=b.p-a.p,e=x*a.p-y*a.r,ray=x*p-y*r;if(ray<0)s=std::fmin(s,e/ray);}
  return s;}
struct Port: simple::ServoPort {
  simple::Feedback o[12]{};uint16_t t[12]{};simple::Drive d{};
  Port(){const auto* p=motion::paired_data::kKnots[0].p;
    for(unsigned l=0;l<4;++l){const double q[3]={((l==0||l==3)?-1:1)*p[7],p[l<2?2:0],std::fmax((l<2?-88.5:-89.)*M_PI/180,p[l<2?3:1])};
      for(unsigned j=0;j<3;++j){auto& f=o[l*3+j];f.q=q[j];f.raw=2048+std::lround(q[j]/simple::kTickRad);f.torque=0;f.temp=30;f.voltage=120;f.valid=true;t[l*3+j]=f.raw;}}}
  uint64_t now=1000000;
  bool read(unsigned i,bool control,simple::Feedback& out) override{out=o[i];out.stampUs=now;out.goal=t[i];out.goalSpeed=d.speed;out.acc=d.acc;if(control)out.controlStampUs=now;return true;}
  bool resolve(unsigned,double q,uint16_t& raw) override{raw=2048+std::lround(q/simple::kTickRad);return raw<4096;}
  bool decodeRaw(unsigned,uint16_t raw,double& q) override{q=(int(raw)-2048)*simple::kTickRad;return raw<4096;}
  bool frame(const uint16_t raw[12],simple::Drive dr) override{d=dr;std::copy(raw,raw+12,t);return true;}
  bool enable(unsigned i) override{o[i].torque=1;return true;}
  bool off(unsigned) override{for(auto& f:o)f.torque=0;return true;}
  bool verifyOff(unsigned i) override{return i==12||o[i].torque==0;}
  void publish(){for(unsigned i=0;i<12;++i){o[i].raw=t[i];o[i].q=(int(t[i])-2048)*simple::kTickRad;}}
};
struct Result{double footLo=1e9,footHi=-1e9,box=1e9,gen1=1e9,tilt=1e9,maxStep=0,maxStrideStep=0,minStride=1e9,maxStride=-1e9;unsigned frames=0,tiltFail=0;std::string end;};
}
int main(int argc,char** argv){
  // scenario: list of "seconds:vx" holds; vx in m/s, '-' = silent client.
  if(argc<3){std::fprintf(stderr,"usage: joystick_envelope out.txt t1:vx1 [t2:vx2 ...]\n");return 2;}
  FILE* out=std::fopen(argv[1],"w");if(!out)return 2;
  simple::SimpleMotion m;Port port;simple::Inputs in;m.begin(&port);
  in.ready=in.powerOk=in.authorityOk=in.imuValid=true;in.refusal="READY";std::copy(kLo,kLo+12,in.minRad);std::copy(kHi,kHi+12,in.maxRad);
  auto tick=[&](){port.now+=simple::kPeriodUs;clock_us=port.now;in.imuStampUs=port.now;m.inputs(in);port.publish();m.update(port.now);};
  for(int i=0;i<15;++i)tick();
  if(m.stand(.120,port.now)){std::puts("STAND refused");return 1;}
  for(int i=0;i<3000&&m.state()!=simple::State::STANDING;++i)tick();
  if(m.state()!=simple::State::STANDING){std::puts("STAND not reached");return 1;}
  Result r;motion::JointTargetFrame last{};double lastStride=NAN;bool started=false;
  const auto& box=simple::kWalkBox;
  std::vector<std::string> steps;
  for(int a=2;a<argc;++a){unsigned seed=0;double total=0,vmax=0;
    if(std::sscanf(argv[a],"rand:%u:%lf:%lf",&seed,&total,&vmax)==3){std::srand(seed);
      for(double t=0;t<total;){const double hold=.25+std::rand()%700/100.,u=std::rand()%2001/1000.-1;
        const double v=std::rand()%4==0?(u<0?-vmax:vmax):u*vmax;char b[64];std::snprintf(b,sizeof b,"%.2f:%.6f",hold,v==0?1e-5:v);steps.push_back(b);t+=hold;}
      steps.push_back("1:0");}
    else steps.push_back(argv[a]);}
  for(const auto& step:steps){double seconds=0;char vx[32]{};if(std::sscanf(step.c_str(),"%lf:%31s",&seconds,vx)!=2)return 2;
    const bool silent=!std::strcmp(vx,"-");const double v=silent?0:std::atof(vx);
    for(int n=0;n<std::lround(seconds*50);++n){
      if(!silent&&n%12==0){  // dashboard refresh 240 ms
        const char* why=nullptr;
        if(v==0)m.walkStop();else why=m.drive(v,0,0,port.now);
        (void)why;
      }
      tick();
      if(m.state()==simple::State::FAULT){r.end=std::string("FAULT ")+m.reason();break;}
      if(std::strcmp(m.phase(),"WALK_STANDARD"))continue;
      const auto& f=simple::StopReferenceProbe::walkNominal(m);if(!f.target.valid)continue;
      started=true;++r.frames;const double s=simple::StopReferenceProbe::stride(m);
      if(!std::isnan(lastStride))r.maxStrideStep=std::fmax(r.maxStrideStep,std::abs(s-lastStride));lastStride=s;r.minStride=std::fmin(r.minStride,s);r.maxStride=std::fmax(r.maxStride,s);
      for(unsigned i=0;i<12;++i){const double q=motion::jointSlot(f.target,i);
        r.box=std::fmin(r.box,std::fmin(q-box.lo[i],box.hi[i]-q));r.gen1=std::fmin(r.gen1,std::fmin(q-kLo[i],kHi[i]-q));
        if(last.valid)r.maxStep=std::fmax(r.maxStep,std::abs(q-motion::jointSlot(last,i)));}
      last=f.target;
      // Planted foot relative to its canonical place under the body (fore/aft).
      {const auto& b=f.cartesian.body;const auto& c=motion::canonicalStandDefinition().contactsWorldM;
        for(unsigned l=0;l<4;++l)if(f.cartesian.phases[l].stance){const double dx=f.cartesian.feet[l].x-b.translationM.x,dy=f.cartesian.feet[l].y-b.translationM.y;
          const double x=b.rotation[0][0]*dx+b.rotation[1][0]*dy-c[l].x;r.footLo=std::fmin(r.footLo,x);r.footHi=std::fmax(r.footHi,x);}}
      // IMU tilt envelope: 32 rays to the boundary of both preset envelopes.
      for(unsigned k=0;k<32;++k){const double ang=2*M_PI*k/32,dr=std::cos(ang),dp=std::sin(ang);
        const double sc=std::fmin(rayScale(k45,dr,dp),rayScale(k60,dr,dp));const double roll=sc*dr,pitch=sc*dp;
        auto body=motion::bodyPoseFromRollPitch({},roll,pitch);const auto t=f.cartesian.body.translationM;
        body.translationM={t.x+t.z*body.rotation[0][2],t.y+t.z*body.rotation[1][2],t.z*body.rotation[2][2]};
        bool ok=true;
        for(unsigned l=0;l<4&&ok;++l){motion::TiltedIkOptions opt;opt.seed=f.target.legs[l];opt.toleranceM=1e-6;opt.maxIterations=8;opt.requireNominalStrip=true;
          const auto ik=motion::worldContactInverseKinematicsTilted(static_cast<motion::LegId>(l),f.cartesian.feet[l],body,opt);
          if(ik.status!=motion::TiltedIkResult::Status::OK||ik.base.contact.axisTiltRad>motion::footContactModel().nominalTiltRad-.5*simple::kTickRad){ok=false;break;}
          const double q3[3]={ik.base.joints.hip,ik.base.joints.upper,ik.base.joints.lower};
          for(unsigned j=0;j<3;++j){const unsigned i=l*3+j;const double q=std::lround(q3[j]/simple::kTickRad)*simple::kTickRad;
            const double mg=std::fmin(std::fmin(q-box.lo[i],box.hi[i]-q),std::fmin(q-kLo[i],kHi[i]-q));r.tilt=std::fmin(r.tilt,mg);if(mg<0)ok=false;}}
        if(!ok){++r.tiltFail;if(std::getenv("JOY_DEBUG"))std::fprintf(stderr,"tilt fail c=%.4f stride=%.4f roll=%.4f pitch=%.4f\n",f.cartesian.cycles,s,roll,pitch);}}
      // walk_quality.py row: cycles, body xyz, q12, feet (x,y,z,stance)x4
      std::fprintf(out,"%.9f %.9f %.9f %.9f",f.cartesian.cycles,f.cartesian.body.translationM.x,f.cartesian.body.translationM.y,f.cartesian.body.translationM.z);
      for(unsigned i=0;i<12;++i)std::fprintf(out," %.9f",motion::jointSlot(f.target,i));
      for(unsigned l=0;l<4;++l)std::fprintf(out," %.9f %.9f %.9f %d",f.cartesian.feet[l].x,f.cartesian.feet[l].y,f.cartesian.feet[l].z,f.cartesian.phases[l].stance?1:0);
      std::fprintf(out,"\n");
    }
    if(!r.end.empty())break;
  }
  for(int n=0;n<50*60&&r.end.empty()&&m.state()!=simple::State::STANDING;++n){tick();if(m.state()==simple::State::FAULT)r.end=std::string("FAULT ")+m.reason();}
  if(r.end.empty())r.end=std::string(simple::name(m.state()))+" "+m.reason();
  std::fclose(out);
  std::printf("foot_x=[%.1f,%.1f] mm frames=%u stride=[%.4f,%.4f] m/cycle max_stride_step=%.6f box_margin=%.1f mrad gen1_margin=%.1f mrad tilt_margin=%.1f mrad tilt_fail=%u max_target_step=%.5f rad end=%s walked=%d\n",
    r.footLo*1e3,r.footHi*1e3,r.frames,r.minStride,r.maxStride,r.maxStrideStep,r.box*1e3,r.gen1*1e3,r.tilt*1e3,r.tiltFail,r.maxStep,r.end.c_str(),started);
  return r.end.rfind("FAULT",0)==0||r.tiltFail||r.box<0?1:0;
}
