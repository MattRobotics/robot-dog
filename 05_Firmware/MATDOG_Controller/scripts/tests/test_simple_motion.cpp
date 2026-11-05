// Essential offline checks. Scripted observations exercise software sequencing;
// they are NOT a motor model and do not qualify physical STAND/WALK.
#include "../../src/simple/SimpleMotion.h"
#include "../../src/core/MotionService.h"
#include "../../src/core/MotionJson.h"
#include "../../src/servo/ServoPacket.h"
#include "../../src/actuator/CalibrationTargetResolver.h"
static int64_t test_clock_us=1000000;
extern "C" int64_t esp_timer_get_time(){return test_clock_us;}
#include "../../src/motion/PairedProneData.h"
#include "../../src/motion/GaitTiming.h"
#include "../../src/motion/TiltCompensation.h"
#include <algorithm>
#include <array>
#include <cassert>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <initializer_list>
#include <iterator>
#include <string>
#include <vector>
using namespace matdog;
namespace matdog { namespace simple {
struct StopReferenceProbe {
  static double roll(const SimpleMotion& m){return m.rollCorrection_*m.correctionScale_;}
  static double pitch(const SimpleMotion& m){return m.pitchCorrection_*m.correctionScale_;}
  static const motion::LocomotionFrame& hold(const SimpleMotion& m){return m.holdNominal_;}
  static bool pending(const SimpleMotion& m){return m.pendingGoalValid_;}
  static const motion::JointTargetFrame& segmentTo(const SimpleMotion& m){return m.segmentTo_;}
  static const motion::LocomotionFrame& walkNominal(const SimpleMotion& m){return m.walkPrevious_;}
  static bool reverse(const SimpleMotion& m){return m.gait_.reverse;}
  static double stride(const SimpleMotion& m){return m.gait_.advanceXM;}
  static const motion::GaitParameters& gait(const SimpleMotion& m){return m.gait_;}
  static const collision::Box& box(const SimpleMotion& m){return m.walkBox();}
};
} }
struct ScriptedObservationPort: simple::ServoPort {
  uint64_t now=1000000;simple::Feedback observed[12]{};
  uint16_t target[12]{};simple::Drive drive{};
  unsigned frames=0,enables=0,offCalls=0,reads=0;std::vector<unsigned> offFirst;
  unsigned referenceFaults=0,faultSlot=99;simple::Feedback faultFeedback;double faultReference=0;uint16_t faultRaw=0;
  void referenceFault(unsigned i,const simple::Feedback& f,double reference,uint16_t raw) override {
    assert(motion&&motion->stopping()&&offCalls>=2); // OFF precedes logging
    for(unsigned j=0;j<12;++j)if(int(j)!=stuckTorque)assert(observed[j].torque==0);
    ++referenceFaults;faultSlot=i;faultFeedback=f;faultReference=reference;faultRaw=raw;
  }
  bool missing=false,holdObservation=false,verify=true,failFrame=false,stopOnRead=false,stopOnControl=false,stopOnEnable=false,stopOnFrame=false;
  // One servo's next control READs go unanswered / its next SYNC_WRITEs are not
  // applied (no reply exists to tell): the bus faults a real rig can show.
  int missControlSlot=-1,dropFrameSlot=-1;unsigned missControlReads=0,dropFrames=0;
  // Every transmitted frame: RAW, drive, servos torque ON at that moment.
  std::vector<std::array<uint16_t,12>> sent;std::vector<simple::Drive> sentDrive;std::vector<unsigned> sentTorqueOn;
  simple::SimpleMotion* motion=nullptr;
  ScriptedObservationPort(){
    const auto* p=motion::paired_data::kKnots[0].p;
    for(unsigned l=0;l<4;++l){const double q[3]={((l==0||l==3)?-1:1)*p[7],p[l<2?2:0],std::fmax((l<2?-88.5:-89.)*3.14159265358979323846/180,p[l<2?3:1])};
      for(unsigned j=0;j<3;++j){auto& f=observed[l*3+j];f.q=q[j];f.raw=2048+std::lround(q[j]/simple::kTickRad);f.torque=0;f.temp=30;f.voltage=120;f.valid=true;f.goal=f.raw;target[l*3+j]=f.raw;}}
  }
  bool read(unsigned i,bool control,simple::Feedback& out) override {
    ++reads;if((stopOnRead||(control&&stopOnControl))&&motion){motion->stop();stopOnRead=stopOnControl=false;return false;}
    if(missing)return false;
    if(control&&static_cast<int>(i)==missControlSlot&&missControlReads){--missControlReads;return false;}
    out=observed[i];out.stampUs=now;out.goal=target[i];out.goalSpeed=drive.speed;out.acc=drive.acc;
    if(control)out.controlStampUs=now;return true;
  }
  bool resolve(unsigned,double q,uint16_t& raw) override{raw=2048+std::lround(q/simple::kTickRad);return raw<4096;}
  bool decodeRaw(unsigned,uint16_t raw,double& q) override {q=(int(raw)-2048)*simple::kTickRad;return raw<4096;}
  bool frame(const uint16_t raw[12],simple::Drive d) override{
    assert(!motion||!motion->stopping()||motion->stopFrameAuthorized(raw,d));if(failFrame)return false;++frames;drive=d;test_clock_us+=frameDelayUs;
    {std::array<uint16_t,12> f;unsigned on=0;for(unsigned i=0;i<12;++i){f[i]=raw[i];on+=observed[i].torque==1;}sent.push_back(f);sentDrive.push_back(d);sentTorqueOn.push_back(on);}
    const bool drop=dropFrames>0;if(drop)--dropFrames;
    for(unsigned i=0;i<12;++i)if(!drop||static_cast<int>(i)!=dropFrameSlot)target[i]=raw[i];if(stopOnFrame&&motion){motion->stop();stopOnFrame=false;}return true;
  }
  bool enable(unsigned i) override{assert(frames>0);assert(std::abs(int(target[i])-int(observed[i].raw))<=12);observed[i].torque=1;++enables;if(stopOnEnable&&motion){motion->stop();stopOnEnable=false;return false;}return true;}
  bool off(unsigned first=0) override{++offCalls;offFirst.push_back(first);for(unsigned i=first;i<12;++i)if(int(i)!=stuckTorque)observed[i].torque=0;return true;}
  int stuckTorque=-1;
  int64_t frameDelayUs=0;  // ESP time consumed by the SYNC_WRITE itself
  int failVerify=-1;  // one servo whose torque-OFF read-back fails
  bool verifyOff(unsigned i) override{return verify&&static_cast<int>(i)!=failVerify&&(i==12||observed[i].torque==0);}
  // Explicitly publish scripted encoder snapshots. No assertion is made that
  // a servo would reach them with this timing, load, profile or torque.
  void publishCommandedPose(){if(holdObservation)return;for(unsigned i=0;i<12;++i){observed[i].raw=target[i];observed[i].q=(static_cast<int>(target[i])-2048)*simple::kTickRad;}}
};
struct Rig {
  simple::SimpleMotion m;ScriptedObservationPort p;simple::Inputs in;
  Rig(){m.begin(&p);p.motion=&m;in.ready=in.powerOk=in.authorityOk=in.imuValid=true;in.refusal="READY";
    const double lo[12]={-.783864,-.914253,-1.598408,-.783864,-.914253,-1.58,-.783864,-.914253,-1.592272,-.783864,-.914253,-1.590738};
    const double hi[12]={.783864,2.110758,.590583,.783864,2.10002,.581379,.771592,2.136835,.590583,.78233,2.129166,.585981};
    for(unsigned i=0;i<12;++i){in.minRad[i]=lo[i];in.maxRad[i]=hi[i];}
    for(unsigned i=0;i<15;++i)tick(false);assert(m.offVerified());
  }
  void tick(bool publish=true){p.now+=simple::kPeriodUs;in.imuStampUs=p.now;m.inputs(in);if(publish)p.publishCommandedPose();test_clock_us=p.now;m.update(p.now);m.updateTelemetry(p.now);}
  bool reachStand(){for(unsigned i=0;i<3000;++i){tick();if(m.state()==simple::State::STANDING)return true;if(m.state()==simple::State::FAULT){std::printf("fault=%s phase=%s frames=%u\n",m.reason(),m.phase(),p.frames);return false;}}return false;}
};
static void proneHandoverWait(){
  // Scripted late tracking, not a model of PRONE or ground support. The existing
  // 12-tick arrival allowance can include a pose outside the NEXT certified box.
  auto atHeight=[](Rig& r){assert(!r.m.stand(.10,r.p.now));assert(r.reachStand());assert(!r.m.prone(r.p.now));
    for(unsigned n=0;n<100&&strcmp(r.m.phase(),"PRONE_HEIGHT");++n)r.tick();assert(!strcmp(r.m.phase(),"PRONE_HEIGHT"));};
  Rig reference;atHeight(reference);motion::JointTargetFrame terminal;
  for(unsigned n=0;n<2000&&!strcmp(reference.m.phase(),"PRONE_HEIGHT");++n){terminal=reference.m.target();reference.tick();assert(reference.m.state()!=simple::State::FAULT);}
  assert(!strcmp(reference.m.phase(),"PRONE_LIFT")&&terminal.valid);
  constexpr unsigned slot=7; // Rear UPPER: leave kLift just below its floor.
  const auto& current=simple::collision::kHeight;const auto& next=simple::collision::kLift;
  const int raw=2048+static_cast<int>(std::floor(next.lo[slot]/simple::kTickRad));
  const double lag=(raw-2048)*simple::kTickRad,end=motion::jointSlot(terminal,slot);
  assert(lag>=current.lo[slot]&&lag<=current.hi[slot]&&lag<next.lo[slot]);
  assert(std::abs(lag-end)<=12*simple::kTickRad);
  std::printf("PRONE handover replay slot=%u q=%.12f end=%.12f error_ticks=%.6f current=[%.12f,%.12f] next=[%.12f,%.12f]\n",
              slot,lag,end,(lag-end)/simple::kTickRad,current.lo[slot],current.hi[slot],next.lo[slot],next.hi[slot]);std::fflush(stdout);
  auto delayed=[&](Rig& r){r.p.now+=simple::kPeriodUs;r.in.imuStampUs=r.p.now;r.m.inputs(r.in);r.p.publishCommandedPose();
    r.p.observed[slot].raw=raw;r.p.observed[slot].q=lag;test_clock_us=r.p.now;r.m.update(r.p.now);r.m.updateTelemetry(r.p.now);};
  for(bool catchesUp:{true,false}){Rig r;atHeight(r);
    for(unsigned n=0;n<2000;++n){double distance=0;for(unsigned i=0;i<12;++i)distance=std::fmax(distance,std::abs(motion::jointSlot(r.m.target(),i)-motion::jointSlot(terminal,i)));
      if(distance<.001)break;r.tick();assert(!strcmp(r.m.phase(),"PRONE_HEIGHT"));}
    const auto deadline=r.m.deadlineUs();const auto off=r.p.offCalls,enables=r.p.enables;
    // Wait long enough to exercise arrival, retaining the preceding group and
    // its original timeout; every frame remains in that group's certified box.
    for(unsigned n=0;n<40;++n){const auto frames=r.p.sent.size();delayed(r);
      if(r.m.state()==simple::State::FAULT)std::printf("PRONE handover premature fault=%s previous_phase=%s q=%.12f now=%llu deadline=%llu\n",r.m.reason(),r.m.exitSnapshot().phase,r.m.exitSnapshot().fb[slot].q,(unsigned long long)r.p.now,(unsigned long long)deadline);
      std::fflush(stdout);assert(r.m.state()!=simple::State::FAULT&&!strcmp(r.m.phase(),"PRONE_HEIGHT"));
      assert(r.m.deadlineUs()==deadline&&r.p.offCalls==off&&r.p.enables==enables);
      for(unsigned f=frames;f<r.p.sent.size();++f)for(unsigned i=0;i<12;++i){const double q=(int(r.p.sent[f][i])-2048)*simple::kTickRad;assert(q>=current.lo[i]&&q<=current.hi[i]);}}
    if(catchesUp){for(unsigned n=0;n<20&&!strcmp(r.m.phase(),"PRONE_HEIGHT");++n)r.tick();
      assert(!strcmp(r.m.phase(),"PRONE_LIFT")&&r.m.state()!=simple::State::FAULT&&r.p.offCalls==off&&r.p.enables==enables);
    }else{for(unsigned n=0;n<2000&&r.m.state()!=simple::State::FAULT;++n){assert(r.m.deadlineUs()==deadline);delayed(r);}
      assert(r.p.now>deadline&&r.p.now<=deadline+simple::kPeriodUs);
      assert(r.m.state()==simple::State::FAULT&&!strcmp(r.m.reason(),"NATIVE_TARGET_NOT_REACHED"));
      for(unsigned n=0;n<30&&!r.m.offVerified();++n)r.tick(false);assert(r.m.offVerified()&&!r.m.energized());}
  }
  std::puts("PRONE handover waits inside current box, follows catch-up into next box, and retains the original timeout");
}
// Quasistatic kinematic board model, not dynamics or proof of physical contact.
// Four nominal references are assumed planted/no-slip on one rigid plane. Servo
// encoder angles follow accepted RAW with a 0.1 s first-order lag and tick
// quantization. A measured-FK rigid fit recovers body-in-support orientation;
// IMU receives board_from_support * support_from_body, never a prescribed error.
struct LevelBoardPlant {
  Rig& r; motion::LocomotionFrame nominal; motion::BodyPose actual{};
  double q[12]{}; unsigned maxRawStep=0; double maxFitResidual=0,maxSettledError=0,maxMovingError=0;
  explicit LevelBoardPlant(Rig& rig):r(rig),nominal(simple::StopReferenceProbe::hold(r.m)){
    assert(nominal.target.valid);
    for(unsigned i=0;i<12;++i)q[i]=r.p.observed[i].q;
    actual=motion::bodyPoseFromRollPitch({},simple::StopReferenceProbe::roll(r.m),simple::StopReferenceProbe::pitch(r.m));
  }
  static motion::Vector3 sub(motion::Vector3 a,motion::Vector3 b){return {a.x-b.x,a.y-b.y,a.z-b.z};}
  static double norm(motion::Vector3 a){return std::sqrt(a.x*a.x+a.y*a.y+a.z*a.z);}
  static motion::Vector3 cross(motion::Vector3 a,motion::Vector3 b){return {a.y*b.z-a.z*b.y,a.z*b.x-a.x*b.z,a.x*b.y-a.y*b.x};}
  static motion::Vector3 unit(motion::Vector3 a){const double n=norm(a);assert(n>1e-6);return {a.x/n,a.y/n,a.z/n};}
  static motion::BodyPose triad(const motion::Vector3 p[4]){
    const auto x=unit(sub(p[1],p[0])),z=unit(cross(x,sub(p[2],p[0]))),y=cross(z,x);
    motion::BodyPose b;const motion::Vector3 col[3]={x,y,z};
    for(unsigned j=0;j<3;++j){b.rotation[0][j]=col[j].x;b.rotation[1][j]=col[j].y;b.rotation[2][j]=col[j].z;}return b;
  }
  static motion::BodyPose multiply(const motion::BodyPose& a,const motion::BodyPose& b){
    motion::BodyPose c;for(unsigned i=0;i<3;++i)for(unsigned j=0;j<3;++j){c.rotation[i][j]=0;for(unsigned k=0;k<3;++k)c.rotation[i][j]+=a.rotation[i][k]*b.rotation[k][j];}return c;
  }
  // Robust matrix -> quaternion, including combined roll/pitch (no Euler sum).
  static void quaternion(const motion::BodyPose& p,double out[4]){
    const auto& a=p.rotation;const double trace=a[0][0]+a[1][1]+a[2][2];
    if(trace>0){const double s=2*std::sqrt(1+trace);out[0]=s/4;out[1]=(a[2][1]-a[1][2])/s;out[2]=(a[0][2]-a[2][0])/s;out[3]=(a[1][0]-a[0][1])/s;}
    else{unsigned i=0;if(a[1][1]>a[i][i])i=1;if(a[2][2]>a[i][i])i=2;const unsigned j=(i+1)%3,k=(i+2)%3;
      const double s=2*std::sqrt(1+a[i][i]-a[j][j]-a[k][k]);out[i+1]=s/4;out[0]=(a[k][j]-a[j][k])/s;out[j+1]=(a[j][i]+a[i][j])/s;out[k+1]=(a[k][i]+a[i][k])/s;}
  }
  motion::BodyPose measuredBody(){
    motion::Vector3 feet[4];const auto support=triad(nominal.cartesian.feet);
    // The contact reference depends on the plane normal in base_link. Iterate
    // this small algebraic registration to consistency; no controller state or
    // control law is emulated here. Production control still runs unmodified.
    for(unsigned iteration=0;iteration<8;++iteration){
      const motion::Vector3 normal{actual.rotation[2][0],actual.rotation[2][1],actual.rotation[2][2]};
      for(unsigned l=0;l<4;++l){const motion::LegJointAngles angles{r.p.observed[l*3].q,r.p.observed[l*3+1].q,r.p.observed[l*3+2].q};
        const auto fk=motion::contactForwardKinematics(static_cast<motion::LegId>(l),angles,normal);assert(fk.status==motion::ContactStatus::OK);feet[l]=fk.referenceM;}
      const auto body=triad(feet);motion::BodyPose fit;
      // body * support^T maps support points INTO base_link; transpose it
      // for the physical support_from_body rotation used by worldR below.
      for(unsigned i=0;i<3;++i)for(unsigned j=0;j<3;++j){fit.rotation[i][j]=0;for(unsigned k=0;k<3;++k)fit.rotation[i][j]+=body.rotation[j][k]*support.rotation[i][k];}actual=fit;
    }
    const auto delta=sub(nominal.cartesian.feet[3],nominal.cartesian.feet[0]);motion::Vector3 predicted{};
    predicted.x=actual.rotation[0][0]*delta.x+actual.rotation[1][0]*delta.y+actual.rotation[2][0]*delta.z;
    predicted.y=actual.rotation[0][1]*delta.x+actual.rotation[1][1]*delta.y+actual.rotation[2][1]*delta.z;
    predicted.z=actual.rotation[0][2]*delta.x+actual.rotation[1][2]*delta.y+actual.rotation[2][2]*delta.z;
    maxFitResidual=std::fmax(maxFitResidual,norm(sub(predicted,sub(feet[3],feet[0]))));return actual;
  }
  motion::BodyPose step(double boardRoll,double boardPitch,bool freshImu=true){
    std::array<uint16_t,12> previous;std::copy(r.p.target,r.p.target+12,previous.begin());
    for(unsigned i=0;i<12;++i){const double target=(int(r.p.target[i])-2048)*simple::kTickRad;q[i]+=.2*(target-q[i]);
      r.p.observed[i].raw=2048+std::lround(q[i]/simple::kTickRad);r.p.observed[i].q=(int(r.p.observed[i].raw)-2048)*simple::kTickRad;}
    const auto world=multiply(motion::bodyPoseFromRollPitch({},boardRoll,boardPitch),measuredBody());quaternion(world,r.in.quaternion);
    if(freshImu)r.tick(false);else{r.p.now+=simple::kPeriodUs;r.m.inputs(r.in);test_clock_us=r.p.now;r.m.update(r.p.now);r.m.updateTelemetry(r.p.now);}
    if(r.m.state()==simple::State::FAULT){std::printf("board fault=%s phase=%s board=%.6f,%.6f accepted=%.6f,%.6f\n",r.m.reason(),r.m.phase(),boardRoll,boardPitch,simple::StopReferenceProbe::roll(r.m),simple::StopReferenceProbe::pitch(r.m));std::fflush(stdout);}
    assert(r.m.state()!=simple::State::FAULT);
    bool inHeight=true,inLift=true;
    for(unsigned i=0;i<12;++i){const auto n=unsigned(std::abs(int(r.p.target[i])-int(previous[i])));maxRawStep=std::max(maxRawStep,n);assert(n<=10);
      const double target=motion::jointSlot(r.m.target(),i),decoded=(int(r.p.target[i])-2048)*simple::kTickRad;assert(target>=r.in.minRad[i]&&target<=r.in.maxRad[i]);
      inHeight&=target>=simple::collision::kHeight.lo[i]&&target<=simple::collision::kHeight.hi[i]&&decoded>=simple::collision::kHeight.lo[i]&&decoded<=simple::collision::kHeight.hi[i];
      inLift&=target>=simple::collision::kLift.lo[i]&&target<=simple::collision::kLift.hi[i]&&decoded>=simple::collision::kLift.lo[i]&&decoded<=simple::collision::kLift.hi[i];}
    assert(inHeight||inLift);
    // New full HOLD leveling must retain the production finite-cylinder
    // nominal support mode, including the RAW frame actually sent to servos.
    const auto accepted=motion::bodyPoseFromRollPitch({},simple::StopReferenceProbe::roll(r.m),simple::StopReferenceProbe::pitch(r.m));
    const motion::Vector3 normal{accepted.rotation[2][0],accepted.rotation[2][1],accepted.rotation[2][2]};
    for(unsigned l=0;l<4;++l){auto decoded=r.m.target().legs[l];decoded.hip=(int(r.p.target[l*3])-2048)*simple::kTickRad;decoded.upper=(int(r.p.target[l*3+1])-2048)*simple::kTickRad;decoded.lower=(int(r.p.target[l*3+2])-2048)*simple::kTickRad;
      for(const auto& angles:{r.m.target().legs[l],decoded}){const auto fk=motion::contactForwardKinematics(static_cast<motion::LegId>(l),angles,normal);assert(fk.status==motion::ContactStatus::OK&&fk.support==motion::SupportMode::NOMINAL_STRIP);}}
    const auto frames=r.p.frames;const double roll=simple::StopReferenceProbe::roll(r.m),pitch=simple::StopReferenceProbe::pitch(r.m);
    r.m.update(r.p.now);assert(r.p.frames==frames&&simple::StopReferenceProbe::roll(r.m)==roll&&simple::StopReferenceProbe::pitch(r.m)==pitch);
    return world;
  }
  static double error(const motion::BodyPose& world){return std::acos(std::clamp(world.rotation[2][2],-1.,1.));}
  void settle(double roll,double pitch,unsigned count=1100,double tolerance=.003){motion::BodyPose w;for(unsigned n=0;n<count;++n)w=step(roll,pitch);
    const double e=error(w);maxSettledError=std::fmax(maxSettledError,e);if(e>=tolerance)std::printf("level error=%.9f board=%.6f,%.6f accepted=%.6f,%.6f limited=%u\n",e,roll,pitch,simple::StopReferenceProbe::roll(r.m),simple::StopReferenceProbe::pitch(r.m),r.m.stabilizationLimited());std::fflush(stdout);assert(e<tolerance);}
};
static void standHoldTilt(){
  constexpr double degree=3.14159265358979323846/180;
  for(double height:{.05,.10,.15}){
    Rig r;assert(!r.m.stand(height,r.p.now));assert(r.reachStand());
    const auto enables=r.p.enables,off=r.p.offCalls;LevelBoardPlant plant(r);
    std::printf("closed-loop board begin h=%.0fmm accepted=%.9f,%.9f\n",height*1000,simple::StopReferenceProbe::roll(r.m),simple::StopReferenceProbe::pitch(r.m));std::fflush(stdout);
    // Close the physical plant before taking the flat baseline. The preceding
    // route uses the legacy scripted IMU (identity independent of its intentional
    // body pitch), so its retained tiny route correction is not a flat plant.
    plant.settle(0,0,150);std::array<uint16_t,12> flat;std::copy(r.p.target,r.p.target+12,flat.begin());
    for(unsigned i=0;i<12;++i){uint16_t nominal=0;assert(r.p.resolve(i,motion::jointSlot(plant.nominal.target,i),nominal));assert(std::abs(int(flat[i])-int(nominal))<=3);}
    plant.settle(0,0,100);for(unsigned i=0;i<12;++i)assert(r.p.target[i]==flat[i]);
    // Workspace differs with height. At 100 mm pitch +/-15 degrees is feasible;
    // the tighter 50/150 mm certified boxes admit a smaller proven neighborhood.
    const double pitch=height==.10?15*degree:.2*degree,roll=height==.10?degree:.1*degree;
    for(double sign:{-1.,1.}){plant.settle(0,sign*pitch);assert(sign*simple::StopReferenceProbe::pitch(r.m)<0);plant.settle(0,0);
      plant.settle(sign*roll,0,height==.10?1100:350);assert(sign*simple::StopReferenceProbe::roll(r.m)<0);plant.settle(0,0,height==.10?1100:350);}
    if(height==.10){
      plant.settle(.5*degree,8*degree,800);plant.settle(-.5*degree,-8*degree,1000);plant.settle(0,0,800);
      // Slowly moving plane: +/-15 degrees at 0.01 rad/s, below correction slew.
      for(double from:{0.,15*degree,-15*degree}){const double to=from==0?15*degree:from>0?-15*degree:0.;
        const unsigned steps=unsigned(std::ceil(std::abs(to-from)/(.01*.02)));
        for(unsigned n=1;n<=steps;++n){const auto world=plant.step(0,from+(to-from)*n/steps);plant.maxMovingError=std::fmax(plant.maxMovingError,LevelBoardPlant::error(world));assert(LevelBoardPlant::error(world)<.008);}}
      plant.settle(0,0,200);
      // Transient IMU age 200..580 ms: retain the last compensated RAW, never
      // jump to the uncompensated reference; the existing >600 ms fault remains.
      plant.settle(0,5*degree,500);std::array<uint16_t,12> held;std::copy(r.p.target,r.p.target+12,held.begin());
      for(unsigned n=0;n<29;++n)plant.step(0,5*degree,false);
      for(unsigned i=0;i<12;++i)assert(std::abs(int(r.p.target[i])-int(held[i]))<=1);
      plant.settle(0,0,550);
    }
    // No accumulated integral: after returning flat the angular/RAW residual is
    // limited by the production 0.001 rad target deadband + encoder quantization.
    assert(std::abs(simple::StopReferenceProbe::roll(r.m))<.002&&std::abs(simple::StopReferenceProbe::pitch(r.m))<.002);
    for(unsigned i=0;i<12;++i)assert(std::abs(int(r.p.target[i])-int(flat[i]))<=3);
    assert(plant.maxFitResidual<.001&&r.p.enables==enables&&r.p.offCalls==off);
    std::printf("closed-loop board STAND h=%.0fmm pitch=+/-%.3fdeg roll=+/-%.3fdeg max_RAW_step=%u fourth_reference_residual=%.6fmm max_settled_error=%.6fdeg max_moving_error=%.6fdeg\n",height*1000,pitch/degree,roll/degree,plant.maxRawStep,plant.maxFitResidual*1000,plant.maxSettledError/degree,plant.maxMovingError/degree);
  }
  // Saturation must remain bounded and recover when the plane reverses. No
  // reduction of workspace/feedback guards or extra actuator enable/OFF occurs.
  {Rig r;assert(!r.m.stand(.10,r.p.now));assert(r.reachStand());LevelBoardPlant plant(r);const auto enables=r.p.enables,off=r.p.offCalls;
    for(unsigned n=0;n<1100;++n)plant.step(10*degree,0);assert(r.m.stabilizationLimited());assert(std::abs(simple::StopReferenceProbe::roll(r.m))<10*degree);
    plant.settle(-degree,0,1100);plant.settle(0,0,1100);assert(r.p.enables==enables&&r.p.offCalls==off);}
  // Full leveling must unwind before the existing HEIGHT/PRONE route starts.
  // STOP during either held leveling or outgoing unwind keeps the issued held
  // reference and a later explicit STAND has its own accepted command lifetime.
  for(bool prone:{false,true})for(bool stopDuringUnwind:{false,true}){
    Rig r;assert(!r.m.stand(.10,r.p.now));assert(r.reachStand());LevelBoardPlant plant(r);plant.settle(0,8*degree,700);
    const auto enables=r.p.enables,off=r.p.offCalls;
    assert(prone?!r.m.prone(r.p.now):!r.m.stand(.15,r.p.now));assert(simple::StopReferenceProbe::pending(r.m));
    const auto unwindDeadline=r.m.deadlineUs();assert(unwindDeadline>r.p.now);
    for(unsigned n=0;n<50;++n){plant.step(0,8*degree);assert(r.m.deadlineUs()==unwindDeadline);}
    if(stopDuringUnwind){r.m.stop();for(unsigned n=0;n<150&&!r.m.stopVerified();++n)plant.step(0,8*degree);
      assert(r.m.stopVerified()&&r.m.energized());assert(!r.m.stand(.10,r.p.now));
      for(unsigned n=0;n<2000&&r.m.state()!=simple::State::STANDING;++n)plant.step(0,8*degree);
      assert(r.m.state()==simple::State::STANDING);
    }else{bool unwound=false;for(unsigned n=0;n<1500&&simple::StopReferenceProbe::pending(r.m);++n){plant.step(0,8*degree);
        unwound|=simple::StopReferenceProbe::pending(r.m)&&simple::StopReferenceProbe::roll(r.m)==0&&simple::StopReferenceProbe::pitch(r.m)==0;}
      std::printf("unwind prone=%u pending=%u phase=%s accepted=%.9f,%.9f limited=%u\n",prone,simple::StopReferenceProbe::pending(r.m),r.m.phase(),simple::StopReferenceProbe::roll(r.m),simple::StopReferenceProbe::pitch(r.m),r.m.stabilizationLimited());std::fflush(stdout);
      // beginPlan generates the next route's first frame immediately and its
      // existing bounded route compensation may already be nonzero. Observe
      // exact zero on the preceding held frame, before that handoff occurs.
      assert(unwound&&!simple::StopReferenceProbe::pending(r.m));
      // Beyond the unwind this test returns to the existing scripted route
      // observations; body/board contact during PRONE is not modeled here.
      r.in.quaternion[0]=1;r.in.quaternion[1]=r.in.quaternion[2]=r.in.quaternion[3]=0;
      for(unsigned n=0;n<2500&&r.m.state()!=(prone?simple::State::PRONE:simple::State::STANDING);++n)r.tick();
      assert(r.m.state()==(prone?simple::State::PRONE:simple::State::STANDING));}
    assert(r.p.enables==enables&&r.p.offCalls==off);
  }
  {Rig r;assert(!r.m.stand(.10,r.p.now));assert(r.reachStand());LevelBoardPlant plant(r);plant.settle(0,-8*degree,700);
    const auto enables=r.p.enables,off=r.p.offCalls;r.m.stop();for(unsigned n=0;n<150&&!r.m.stopVerified();++n)plant.step(0,-8*degree);
    assert(r.m.stopVerified()&&r.m.energized());assert(!r.m.stand(.10,r.p.now));plant.settle(0,-8*degree,700);
    assert(r.m.state()==simple::State::STANDING&&r.p.enables==enables&&r.p.offCalls==off);
    // Permanent stale attitude uses the existing IMU_STALE SAFE_OFF path.
    for(unsigned n=0;n<40&&r.m.state()!=simple::State::FAULT;++n){r.p.now+=simple::kPeriodUs;r.p.publishCommandedPose();r.m.inputs(r.in);test_clock_us=r.p.now;r.m.update(r.p.now);}
    assert(r.m.state()==simple::State::FAULT&&!std::strcmp(r.m.reason(),"IMU_STALE"));}
  for(double sign:{-1.,1.}){Rig r;assert(!r.m.stand(.10,r.p.now));assert(r.reachStand());LevelBoardPlant plant(r);
    const auto enables=r.p.enables,off=r.p.offCalls;for(unsigned n=0;n<1100;++n)plant.step(sign*10*degree,0);assert(r.m.stabilizationLimited());
    // Move briefly away from the real workspace edge, then one accepted frame
    // toward it: STOP must remain feasible while the leveling reference moves.
    for(unsigned n=0;n<10;++n)plant.step(0,0);const double before=simple::StopReferenceProbe::roll(r.m);plant.step(sign*10*degree,0);
    assert(std::abs(simple::StopReferenceProbe::roll(r.m)-before)>1e-5);r.m.stop();
    for(unsigned n=0;n<200&&!r.m.stopVerified();++n)plant.step(sign*10*degree,0);
    assert(r.m.stopVerified()&&r.m.energized()&&r.p.enables==enables&&r.p.offCalls==off);}
  {Rig r;assert(!r.m.stand(.10,r.p.now));assert(r.reachStand());LevelBoardPlant plant(r);plant.settle(0,8*degree,700);
    assert(!r.m.stand(.15,r.p.now));for(unsigned n=0;n<50;++n)plant.step(0,8*degree);
    const auto frames=r.p.frames,enables=r.p.enables,off=r.p.offCalls;r.m.safeOff();for(unsigned n=0;n<30&&!r.m.offVerified();++n)r.tick(false);
    assert(r.m.offVerified()&&!r.m.energized()&&r.p.frames==frames&&r.p.enables==enables&&r.p.offCalls>off);}
  std::puts("production closed-loop STAND board plant PASS: quantized 0.1s servo lag, signed/combined/slow tilt, saturation reversal, flat return, stale IMU, height/PRONE unwind, STOP and explicit STAND; no hardware stability claim");
}
// Moving-board WALK plant: only labelled stance anchors constrain the body.
// The newest touchdown is excluded during each four-contact window, matching
// production's use of the other three settled references. This models slow
// planar/no-slip contact, not impact, friction, COM or physical stability.
struct WalkBoardPlant {
  Rig& r;motion::BodyPose actual{};double q[12]{};unsigned maxRawStep=0;double maxSteadyError=0;
  explicit WalkBoardPlant(Rig& rig):r(rig){assert(r.m.state()==simple::State::WALKING);for(unsigned i=0;i<12;++i)q[i]=r.p.observed[i].q;}
  motion::BodyPose measuredBody(){
    const auto& f=simple::StopReferenceProbe::walkNominal(r.m);assert(f.target.valid);bool stance[4];unsigned count=0,newest=0;
    for(unsigned l=0;l<4;++l){stance[l]=f.cartesian.phases[l].stance;if(stance[l]){++count;if(f.cartesian.phases[l].phase<f.cartesian.phases[newest].phase)newest=l;}}
    if(count==4)stance[newest]=false;unsigned ids[3],n=0;for(unsigned l=0;l<4;++l)if(stance[l]){assert(n<3);ids[n++]=l;}assert(n==3);
    motion::Vector3 anchors[4],feet[4];for(unsigned i=0;i<3;++i)anchors[i]=f.cartesian.feet[ids[i]];const auto support=LevelBoardPlant::triad(anchors);
    for(unsigned iteration=0;iteration<8;++iteration){const motion::Vector3 normal{actual.rotation[2][0],actual.rotation[2][1],actual.rotation[2][2]};
      for(unsigned i=0;i<3;++i){const unsigned l=ids[i];const motion::LegJointAngles angles{r.p.observed[l*3].q,r.p.observed[l*3+1].q,r.p.observed[l*3+2].q};
        const auto fk=motion::contactForwardKinematics(static_cast<motion::LegId>(l),angles,normal);assert(fk.status==motion::ContactStatus::OK);feet[i]=fk.referenceM;}
      const auto body=LevelBoardPlant::triad(feet);motion::BodyPose fit;
      for(unsigned i=0;i<3;++i)for(unsigned j=0;j<3;++j){fit.rotation[i][j]=0;for(unsigned k=0;k<3;++k)fit.rotation[i][j]+=body.rotation[j][k]*support.rotation[i][k];}actual=fit;}
    return actual;
  }
  motion::BodyPose step(double boardRoll,double boardPitch){
    std::array<uint16_t,12> previous;std::copy(r.p.target,r.p.target+12,previous.begin());
    for(unsigned i=0;i<12;++i){const double target=(int(r.p.target[i])-2048)*simple::kTickRad;q[i]+=.2*(target-q[i]);r.p.observed[i].raw=2048+std::lround(q[i]/simple::kTickRad);r.p.observed[i].q=(int(r.p.observed[i].raw)-2048)*simple::kTickRad;}
    const auto world=LevelBoardPlant::multiply(motion::bodyPoseFromRollPitch({},boardRoll,boardPitch),measuredBody());LevelBoardPlant::quaternion(world,r.in.quaternion);r.tick(false);
    if(r.m.state()==simple::State::FAULT){std::printf("WALK board fault=%s phase=%s board=%.6f,%.6f accepted=%.6f,%.6f cycles=%.6f\n",r.m.reason(),r.m.phase(),boardRoll,boardPitch,simple::StopReferenceProbe::roll(r.m),simple::StopReferenceProbe::pitch(r.m),r.m.walkCycles());std::fflush(stdout);}assert(r.m.state()!=simple::State::FAULT);
    for(unsigned i=0;i<12;++i){const double target=motion::jointSlot(r.m.target(),i),decoded=(int(r.p.target[i])-2048)*simple::kTickRad;
      assert(target>=r.in.minRad[i]&&target<=r.in.maxRad[i]&&decoded>=r.in.minRad[i]&&decoded<=r.in.maxRad[i]);
      const auto& box=simple::StopReferenceProbe::box(r.m);  // the walk's own certified box
      assert(target>=box.lo[i]&&target<=box.hi[i]&&decoded>=box.lo[i]&&decoded<=box.hi[i]);
      const auto n=unsigned(std::abs(int(r.p.target[i])-int(previous[i])));maxRawStep=std::max(maxRawStep,n);assert(n<=unsigned(std::ceil(simple::kWalkDrive.speed*simple::kPeriodUs/1e6))+1);}
    const auto frames=r.p.frames;const double roll=simple::StopReferenceProbe::roll(r.m),pitch=simple::StopReferenceProbe::pitch(r.m);r.m.update(r.p.now);
    assert(r.p.frames==frames&&simple::StopReferenceProbe::roll(r.m)==roll&&simple::StopReferenceProbe::pitch(r.m)==pitch);return world;
  }
  void hold(double roll,double pitch,unsigned count=1000){double e=0,rollError=0;const unsigned cycle=unsigned(std::ceil(simple::kWalkPeriodS*1e6/simple::kPeriodUs));assert(count>=cycle);
    for(unsigned n=0;n<count;++n){const auto world=step(roll,pitch);if(n+cycle>=count){e=std::fmax(e,LevelBoardPlant::error(world));rollError=std::fmax(rollError,std::abs(std::atan2(world.rotation[2][1],world.rotation[2][2])));}}
    // A whole final gait cycle covers all support changes; a single favorable
    // sample must not hide a periodic attitude error at another foot phase.
    maxSteadyError=std::fmax(maxSteadyError,e);if(e>=.005||rollError>=.0025){std::printf("WALK board steadyerror=%.9f rollerror=%.9f board=%.6f,%.6f accepted=%.6f,%.6f\n",e,rollError,roll,pitch,simple::StopReferenceProbe::roll(r.m),simple::StopReferenceProbe::pitch(r.m));std::fflush(stdout);}assert(e<.005&&rollError<.0025);
    if(roll!=0)assert(roll*simple::StopReferenceProbe::roll(r.m)<0&&std::abs(simple::StopReferenceProbe::roll(r.m))>std::abs(roll)*.25);
    if(pitch!=0)assert(pitch*simple::StopReferenceProbe::pitch(r.m)<0&&std::abs(simple::StopReferenceProbe::pitch(r.m))>std::abs(pitch)*.75);}
  void ramp(double fromRoll,double fromPitch,double toRoll,double toPitch){const unsigned count=std::max(1u,unsigned(std::ceil(std::fmax(std::abs(toRoll-fromRoll),std::abs(toPitch-fromPitch))/.0001)));
    for(unsigned n=1;n<=count;++n)step(fromRoll+(toRoll-fromRoll)*n/count,fromPitch+(toPitch-fromPitch)*n/count);}
  void holdLimited(double pitch,unsigned count=1000){double minimum=10,maximum=0;unsigned limited=0;const unsigned cycle=unsigned(std::ceil(simple::kWalkPeriodS*1e6/simple::kPeriodUs));
    for(unsigned n=0;n<count;++n){const auto world=step(0,pitch);if(n+cycle>=count){const double e=LevelBoardPlant::error(world);minimum=std::fmin(minimum,e);maximum=std::fmax(maximum,e);limited+=r.m.stabilizationLimited();}}
    // Positive board pitch reaches the real front-joint/STOP boundary sooner.
    // Outside that demonstrated domain require a declared bounded residual,
    // rather than calling a non-level body fully compensated.
    assert(limited>cycle/2&&minimum>.25*3.14159265358979323846/180&&maximum<std::abs(pitch));
    assert(pitch*simple::StopReferenceProbe::pitch(r.m)<0&&std::abs(simple::StopReferenceProbe::pitch(r.m))>.001);
    std::printf("WALK board bounded pitch=%.3fdeg residual=[%.6f,%.6f]deg limited_tail=%u/%u\n",pitch*180/3.14159265358979323846,minimum*180/3.14159265358979323846,maximum*180/3.14159265358979323846,limited,cycle);}
};
static void walkLevelBoard(){
  constexpr double degree=3.14159265358979323846/180;
  for(unsigned stride:{1u,2u}){
    // One explicit WALK owns the height transition and starts the gait only
    // after its120mm stand is reached; admission itself emits no RAW frame.
    Rig r;assert(!r.m.stand(.15,r.p.now));assert(r.reachStand());const auto enables=r.p.enables,off=r.p.offCalls,frames=r.p.frames;
    assert(!r.m.walk(stride,r.p.now)&&r.p.frames==frames);bool entry=false;std::array<uint16_t,12> before;
    for(unsigned n=0;n<3000&&r.m.state()!=simple::State::WALKING;++n){entry|=!std::strcmp(r.m.phase(),"HEIGHT");std::copy(r.p.target,r.p.target+12,before.begin());r.tick();assert(r.m.state()!=simple::State::FAULT);}
    assert(entry&&r.m.state()==simple::State::WALKING&&r.m.requestedHeightM()==simple::kWalkHeightM&&r.p.enables==enables&&r.p.offCalls==off);
    for(unsigned i=0;i<12;++i)assert(std::abs(int(r.p.target[i])-int(before[i]))<=1);
    WalkBoardPlant plant(r);plant.hold(0,0,650);
    for(double pitch:{(stride==1?2.:.3)*degree,-2*degree,-4*degree}){plant.ramp(0,0,0,pitch);plant.hold(0,pitch);plant.ramp(0,pitch,0,0);plant.hold(0,0,650);}
    for(double pitch:{2*degree,4*degree}){if(stride==1&&pitch==2*degree)continue;plant.ramp(0,0,0,pitch);plant.holdLimited(pitch);plant.ramp(0,pitch,0,0);plant.hold(0,0,650);}
    for(double sign:{1.,-1.}){plant.ramp(0,0,sign*.25*degree,0);plant.hold(sign*.25*degree,0,650);plant.ramp(sign*.25*degree,0,sign*.25*degree,-2*degree);plant.hold(sign*.25*degree,-2*degree);plant.ramp(sign*.25*degree,-2*degree,0,0);plant.hold(0,0,650);}
    plant.ramp(0,0,.25*degree,-2*degree);plant.hold(.25*degree,-2*degree);r.m.walkStop();
    for(unsigned n=0;n<3000&&r.m.state()!=simple::State::STANDING;++n)plant.step(.25*degree,-2*degree);
    assert(r.m.state()==simple::State::STANDING&&!std::strcmp(r.m.reason(),"STAND_REACHED")&&r.m.requestedHeightM()==simple::kWalkHeightM);plant.hold(.25*degree,-2*degree,650);
    assert(r.p.enables==enables&&r.p.offCalls==off);
    std::printf("WALK%u closed-loop movingboard feasible pitch %.1f/-2/-4deg, bounded +4%s, roll+/-0.25deg/combined, HOLD%.0f reached; max_RAW=%u max_steady_error=%.6fdeg\n",unsigned(simple::kStrideM[stride]*1000),stride==1?2.:.3,stride==2?"/+2":"",simple::kWalkHeightM*1000,plant.maxRawStep,plant.maxSteadyError/degree);
  }
  for(unsigned stride:{1u,2u}){Rig r;assert(!r.m.stand(.15,r.p.now));assert(r.reachStand());const auto enables=r.p.enables,off=r.p.offCalls;assert(!r.m.walk(stride,r.p.now));for(unsigned n=0;n<10;++n)r.tick();assert(r.m.state()==simple::State::MOVING&&!std::strcmp(r.m.phase(),"HEIGHT"));
    r.m.stop();for(unsigned n=0;n<200&&!r.m.stopVerified();++n)r.tick();assert(r.m.stopVerified()&&r.m.energized()&&r.p.enables==enables&&r.p.offCalls==off);
    for(unsigned n=0;n<100;++n)r.tick();assert(r.m.state()==simple::State::STOPPED&&r.m.stopVerified());
    assert(!r.m.stand(.15,r.p.now));assert(r.reachStand());assert(r.m.requestedHeightM()==.15&&r.p.enables==enables&&r.p.offCalls==off);}
  std::puts("WALK profile entry STOP and production closed-loop board tests PASS; no hardware contact/stability qualification");
}

// TX/compute latency is not reference motion (dev.13 regression): with 7 ms
// TX or alternating 0/7 ms and late releases, the WALK pre-check and STOP
// take the secant on the sampling clock and neither fakes a stop failure.
static void walkTxLatency(){
  constexpr double degree=3.14159265358979323846/180;
  for(unsigned stride:{1u,2u}){
    {Rig r;assert(!r.m.stand(.15,r.p.now));assert(r.reachStand());assert(!r.m.walk(stride,r.p.now));
      for(unsigned n=0;n<3000&&r.m.state()!=simple::State::WALKING;++n){r.tick();assert(r.m.state()!=simple::State::FAULT);}
      WalkBoardPlant plant(r);r.p.frameDelayUs=7000;plant.ramp(0,0,.1*degree,-.3*degree);plant.hold(.1*degree,-.3*degree,650);
      for(unsigned n=0;n<650;++n){r.p.frameDelayUs=n&1?0:7000;if(n%7==0)r.p.now+=15000;plant.step(.1*degree,-.3*degree);}
      const uint64_t start=r.p.now;r.m.stop();
      for(unsigned n=0;n<300&&!r.m.stopVerified();++n){r.p.frameDelayUs=n&1?0:7000;plant.step(.1*degree,-.3*degree);}
      assert(r.m.stopVerified()&&r.m.energized()&&r.p.now-start<6000000);}
    // STOP at every 13th release over two cycles under alternating TX latency.
    for(unsigned when=0;when<650;when+=13){Rig r;assert(!r.m.stand(.15,r.p.now));assert(r.reachStand());assert(!r.m.walk(stride,r.p.now));
      for(unsigned n=0;n<3000&&r.m.state()!=simple::State::WALKING;++n)r.tick();
      WalkBoardPlant plant(r);for(unsigned n=0;n<200+when;++n){r.p.frameDelayUs=n&1?0:7000;plant.step(.1*degree,-.3*degree);}
      r.m.stop();for(unsigned n=0;n<300&&!r.m.stopVerified();++n){r.p.frameDelayUs=n&1?0:7000;plant.step(.1*degree,-.3*degree);}
      if(!r.m.stopVerified())std::printf("WALK%u STOP at %u not verified: %s\n",unsigned(simple::kStrideM[stride]*1000),when,r.m.reason());
      assert(r.m.stopVerified());}
  }
  // Long HOLD with unchanged RAW and TX latency: the levelled reference still
  // follows a later board change (no stale-reference wedge, no fault).
  {Rig r;assert(!r.m.stand(.12,r.p.now));assert(r.reachStand());LevelBoardPlant plant(r);r.p.frameDelayUs=7000;
    plant.settle(0,0,3000);plant.settle(0,2*degree,1100);assert(simple::StopReferenceProbe::pitch(r.m)<-1.5*degree);plant.settle(0,0,1100);}
  std::puts("TX latency 7 ms / alternating 0-7 ms / late releases: WALK45/60 pre-check, STOP in WALK and long HOLD PASS (software timing only)");
}

// JOYSTICK (dev.15): continuous BASE_VELOCITY (vx, vy, yaw_rate) on the WALK
// law, the ESP clock deadline, the fore/aft leg order latched per walk.
// Scripted observation and a kinematic board plant: no motor or contact model.
static void joystickWalk(){
  constexpr double degree=3.14159265358979323846/180,T=simple::kWalkPeriodS;
  const double lim[3]={simple::kJoystickStrideM/T,simple::kJoystickLateralM/T,simple::kJoystickYawRad/T};
  using V3=std::array<double,3>;
  auto drive=[](Rig& r,V3 v){return r.m.drive(v[0],v[1],v[2],r.p.now);};
  auto walkTo=[&](Rig& r,V3 v){for(unsigned n=0;n<3000&&r.m.state()!=simple::State::WALKING;++n){if(n%12==0)drive(r,v);r.tick();assert(r.m.state()!=simple::State::FAULT);}
    assert(r.m.state()==simple::State::WALKING);};
  auto hold=[&](Rig& r,V3 v,unsigned releases){for(unsigned n=0;n<releases;++n){if(n%12==0)assert(!drive(r,v));r.tick();assert(r.m.state()==simple::State::WALKING);}};
  auto pose=[](const Rig& r){const auto& b=simple::StopReferenceProbe::walkNominal(r.m).cartesian.body;return V3{b.translationM.x,b.translationM.y,std::atan2(b.rotation[1][0],b.rotation[0][0])};};
  {Rig r;assert(!r.m.stand(.15,r.p.now));assert(r.reachStand());std::array<uint16_t,12> held;std::copy(r.p.target,r.p.target+12,held.begin());
    assert(!std::strcmp(drive(r,{.8*lim[0],.8*lim[1],0}),"JOYSTICK_VELOCITY_RANGE"));  // outside the ellipsoid
    for(unsigned j=0;j<3;++j){V3 v{0,0,0};v[j]=1.001*lim[j];assert(!std::strcmp(drive(r,v),"JOYSTICK_VELOCITY_RANGE"));v[j]=-v[j];assert(!std::strcmp(drive(r,v),"JOYSTICK_VELOCITY_RANGE"));}
    assert(!std::strcmp(drive(r,{NAN,0,0}),"JOYSTICK_VELOCITY_RANGE"));assert(!std::strcmp(drive(r,{0,0,INFINITY}),"JOYSTICK_VELOCITY_RANGE"));
    for(unsigned n=0;n<50;++n)r.tick();assert(r.m.state()==simple::State::STANDING&&!r.m.joystickActive());
    assert(std::equal(held.begin(),held.end(),r.p.target));}  // refusals move nothing
  // Sign and proportionality on all three axes: over two whole cycles the body
  // moves by exactly the SE(2) exponential of 2 x (vx, vy, yaw_rate) x period.
  // +x forward, +y left, +yaw counter-clockwise; backward takes the reversed order.
  const V3 cases[]={{lim[0],0,0},{lim[0]/2,0,0},{-lim[0],0,0},{-lim[0]/3,0,0},{0,lim[1],0},{0,-lim[1]/2,0},{0,0,lim[2]},{0,0,-lim[2]/2},
                    {lim[0]/2,lim[1]/2,lim[2]/2},{-lim[0]/2,-lim[1]/2,-lim[2]/2}};
  for(const auto& v:cases){Rig r;assert(!r.m.stand(.15,r.p.now));assert(r.reachStand());walkTo(r,v);
    assert(simple::StopReferenceProbe::reverse(r.m)==(v[0]<0)&&!std::strcmp(r.m.reason(),"WALK_JOYSTICK")&&r.m.joystickActive());
    int first=-1;for(unsigned n=0;n<400&&first<0;++n){if(n%12==0)drive(r,v);r.tick();const auto& f=simple::StopReferenceProbe::walkNominal(r.m);
      for(unsigned l=0;l<4;++l)if(!f.cartesian.phases[l].stance)first=int(l);}
    assert(first==(v[0]<0?3:2));  // first swing: LH backward, RH otherwise
    while(r.m.walkCycles()<1.5)hold(r,v,1);
    const auto a=pose(r);hold(r,v,650);const auto b=pose(r);
    const double w=2*v[2]*T,si=w!=0?std::sin(w)/(v[2]*T):2,co=w!=0?(1-std::cos(w))/(v[2]*T):0;
    const double dx=v[0]*T*si-v[1]*T*co,dy=v[0]*T*co+v[1]*T*si;
    // The body sway (<= 16 mm, body frame) turns with the heading: |R(w)-I| x sway.
    const double tol=1e-9+std::abs(w)*.016;
    assert(std::abs(b[0]-a[0]-(std::cos(a[2])*dx-std::sin(a[2])*dy))<tol&&std::abs(b[1]-a[1]-(std::sin(a[2])*dx+std::cos(a[2])*dy))<tol&&std::abs(std::remainder(b[2]-a[2]-w,2*M_PI))<1e-9);
    assert(std::abs(r.m.walkVelocityX()-v[0])<1e-12&&std::abs(r.m.walkVelocityY()-v[1])<1e-12&&std::abs(r.m.walkYawRate()-v[2])<1e-12);
    // Neutral stick: WALK STOP from the next touchdown, then HOLD at the WALK height.
    r.m.walkStop();assert(!std::strcmp(r.m.reason(),"WALK_STOPPING")&&!r.m.joystickActive());assert(r.reachStand());assert(r.m.requestedHeightM()==simple::kWalkHeightM);}
  // The SE(2) track equals the closed-form law bit for bit with constant steered
  // parameters (lateral, yaw, fillet contact), through WALK STOP and recentring.
  {motion::GaitParameters g;g.heightM=simple::baseLinkHeightFromNominalM(simple::kWalkHeightM);g.liftM=simple::kWalkLiftM;g.duty=simple::kWalkDuty;
    g.swayXM=simple::kWalkSwayXM;g.swayYM=simple::kJoystickSwayYM;g.startCycles=g.stopCycles=1;g.edgeTiltRad=motion::kEdgeContactMaxTiltRad;
    g.advanceXM=.02;g.advanceYM=-.01;g.yawRad=.1;motion::GaitTrack t;const double term=4,end=term+motion::gaitSettleCycles(g);
    for(double c=0;c<=end+1e-12;c+=1/325.){const double cc=std::fmin(c,end),tm=cc>=2.9?term:-1;
      const auto x=motion::gaitCartesian(g,cc,1/T,0,tm),y=motion::gaitTrackCartesian(t,g,cc,1/T,0,tm);assert(x.valid&&y.valid);
      for(unsigned l=0;l<4;++l)assert(x.feet[l].x==y.feet[l].x&&x.feet[l].y==y.feet[l].y&&x.feet[l].z==y.feet[l].z&&x.velocity[l].x==y.velocity[l].x&&x.velocity[l].y==y.velocity[l].y);
      assert(x.body.translationM.x==y.body.translationM.x&&x.body.translationM.y==y.body.translationM.y&&x.body.rotation[1][0]==y.body.rotation[1][0]&&x.yawVelocity==y.yawVelocity);}}
  // Refreshed for 30 s (30 deadlines) it keeps walking; silence ends it with
  // WALK STOP at the first release after the ESP deadline.
  {Rig r;assert(!r.m.stand(.15,r.p.now));assert(r.reachStand());const V3 v{lim[0]/2,lim[1]/2,-lim[2]/2};walkTo(r,v);hold(r,v,1500);
    const uint64_t last=r.p.now;assert(!drive(r,v));
    for(unsigned n=0;n<200&&std::strcmp(r.m.reason(),"JOYSTICK_TIMEOUT");++n)r.tick();
    assert(!std::strcmp(r.m.reason(),"JOYSTICK_TIMEOUT")&&r.p.now-last>simple::kJoystickTimeoutUs&&r.p.now-last<=simple::kJoystickTimeoutUs+simple::kPeriodUs);
    assert(!r.m.joystickActive()&&!std::strcmp(drive(r,v),"WALK_STOPPING"));assert(r.reachStand());}
  // Silence while rising from PRONE: the stand completes, the walk never starts.
  {Rig r;assert(!drive(r,{0,0,lim[2]}));assert(r.m.joystickActive());bool walked=false;
    for(unsigned n=0;n<3000&&r.m.state()!=simple::State::STANDING;++n){r.tick();walked|=r.m.state()==simple::State::WALKING;assert(r.m.state()!=simple::State::FAULT);}
    assert(!walked&&r.m.state()==simple::State::STANDING&&r.m.requestedHeightM()==simple::kWalkHeightM&&!r.m.joystickActive());
    for(unsigned n=0;n<500;++n){r.tick();assert(r.m.state()==simple::State::STANDING);}}
  // Fore/aft reversal: WALK STOP with the forward order, STANDING, then the reversed
  // order. Lateral and yaw reverse continuously inside one walk.
  {Rig r;assert(!r.m.stand(.15,r.p.now));assert(r.reachStand());walkTo(r,{lim[0],0,0});hold(r,{lim[0],0,0},400);
    assert(!drive(r,{-lim[0],0,0}));assert(!std::strcmp(r.m.reason(),"JOYSTICK_REVERSING")&&!r.m.joystickActive());
    bool stood=false;
    for(unsigned n=0;n<4000&&!(r.m.state()==simple::State::WALKING&&simple::StopReferenceProbe::reverse(r.m));++n){
      if(n%12==0){const char* why=drive(r,{-lim[0],0,0});assert(!why||!std::strcmp(why,"WALK_STOPPING")||!std::strcmp(why,"MOTION_BUSY"));}
      r.tick();stood|=r.m.state()==simple::State::STANDING;assert(r.m.state()!=simple::State::FAULT);}
    assert(stood&&r.m.state()==simple::State::WALKING&&simple::StopReferenceProbe::reverse(r.m)&&r.m.walkCycles()<1);
    r.m.walkStop();assert(r.reachStand());}
  // Steps on every axis: each follows at its limited rate, fore/aft never crosses
  // zero inside a walk, every RAW step stays below the WALK drive per release.
  {Rig r;assert(!r.m.stand(.15,r.p.now));assert(r.reachStand());walkTo(r,{lim[0],0,0});hold(r,{lim[0],0,0},300);
    double prev[3]={simple::StopReferenceProbe::stride(r.m),0,0},maxD[3]{};std::array<uint16_t,12> raw;std::copy(r.p.target,r.p.target+12,raw.begin());unsigned maxRaw=0;
    const V3 seq[]={{lim[0]/5,lim[1]*.9,0},{lim[0]*.6,-lim[1]*.6,lim[2]*.5},{1e-9,0,-lim[2]},{lim[0],0,0}};
    for(unsigned n=0;n<3200;++n){const V3 v=seq[std::min(3u,n/700)];if(n%12==0)assert(!drive(r,v));r.tick();assert(r.m.state()==simple::State::WALKING);
      const auto& g=simple::StopReferenceProbe::gait(r.m);const double now[3]={g.advanceXM,g.advanceYM,g.yawRad};
      for(unsigned j=0;j<3;++j){maxD[j]=std::fmax(maxD[j],std::abs(now[j]-prev[j]));prev[j]=now[j];}assert(g.advanceXM>=0);
      for(unsigned i=0;i<12;++i){maxRaw=std::max(maxRaw,unsigned(std::abs(int(r.p.target[i])-int(raw[i]))));raw[i]=r.p.target[i];}}
    const double perRelease=simple::kPeriodUs/1e6/T/simple::kJoystickRampCycles,full[3]={simple::kJoystickStrideM,simple::kJoystickLateralM,simple::kJoystickYawRad};
    for(unsigned j=0;j<3;++j)assert(maxD[j]<=full[j]*perRelease*(1+1e-6));
    assert(std::abs(prev[0]-simple::kJoystickStrideM)<1e-12&&prev[1]==0&&prev[2]==0&&maxRaw<=simple::kWalkDrive.speed*simple::kPeriodUs/1000000);
    r.m.walkStop();assert(r.reachStand());
    std::printf("JOYSTICK axis steps: rates %.6f m %.6f m %.6f rad per release (limits %.6f %.6f %.6f), max RAW step %u ticks/release\n",
      maxD[0],maxD[1],maxD[2],full[0]*perRelease,full[1]*perRelease,full[2]*perRelease,maxRaw);}
  // STOP is independent of the joystick and of its deadline: immediate
  // contextual arrest, refreshes refused, explicit STAND completes the gait.
  for(unsigned when=0;when<325;when+=65){Rig r;assert(!r.m.stand(.15,r.p.now));assert(r.reachStand());const V3 v{-lim[0]/2,lim[1]/2,lim[2]/2};walkTo(r,v);hold(r,v,300+when);
    r.m.stop();for(unsigned n=0;n<150&&!r.m.stopVerified();++n){if(n%12==0)assert(drive(r,v));r.tick();}
    assert(r.m.stopVerified()&&r.m.energized()&&!r.m.joystickActive()&&!std::strcmp(drive(r,v),"STOP_LATCHED"));
    assert(!r.m.stand(.12,r.p.now));assert(r.reachStand());}
  // IMU compensation during steered walks on a moving board: pitch +2/-2 deg,
  // roll 0.25 deg, lateral and turning commands, no fault (bounded outside the envelope).
  {Rig r;assert(!r.m.stand(.15,r.p.now));assert(r.reachStand());walkTo(r,{lim[0]/2,lim[1]/2,lim[2]/2});WalkBoardPlant plant(r);unsigned n=0;
    auto go=[&](double roll0,double pitch0,double roll1,double pitch1,unsigned steps,V3 v){for(unsigned k=0;k<steps;++k,++n){
      if(n%12==0)assert(!drive(r,v));const double u=double(k+1)/steps;plant.step(roll0+(roll1-roll0)*u,pitch0+(pitch1-pitch0)*u);assert(r.m.state()==simple::State::WALKING);}};
    go(0,0,0,0,650,{lim[0]/2,lim[1]/2,lim[2]/2});go(0,0,0,2*degree,325,{0,lim[1],0});go(0,2*degree,0,2*degree,650,{0,0,-lim[2]});
    go(0,2*degree,.25*degree,-2*degree,650,{lim[0]/2,-lim[1]/2,lim[2]/2});go(.25*degree,-2*degree,0,0,650,{lim[0],0,0});
    r.m.walkStop();for(unsigned k=0;k<3000&&r.m.state()!=simple::State::STANDING;++k)plant.step(0,0);assert(r.m.state()==simple::State::STANDING);
    std::printf("JOYSTICK closed-loop movingboard pitch +2/-2deg roll 0.25deg, lateral/turn/combined: no fault, max_RAW=%u\n",plant.maxRawStep);}
  // Contract: BASE_VELOCITY gait JOYSTICK through the JSON parser and MotionService.
  {Rig api;core::MotionService service;service.begin(&api.m,nullptr,nullptr);service.setReady(true,"READY");assert(!api.m.stand(.15,api.p.now));assert(api.reachStand());
    unsigned seq=0;auto send=[&](const char* params){char json[400];const int n=std::snprintf(json,sizeof json,
      "{\"version\":1,\"id\":\"joy-%u\",\"sequence\":%u,\"issued_at_ms\":%lu,\"ttl_ms\":1000,\"owner\":\"dashboard\",\"type\":\"BASE_VELOCITY\",\"params\":%s}",seq+1,seq+1,(unsigned long)(api.p.now/1000),params);++seq;
      motion::MotionCommand c;const char* why=nullptr;assert(core::parseMotionJson(json,size_t(n),&c,&why));test_clock_us=api.p.now;return service.command(c,api.p.now/1000);};
    char p[200];
    std::snprintf(p,sizeof p,"{\"vx\":0,\"vy\":%.12f,\"yaw_rate\":%.12f,\"gait\":\"JOYSTICK\"}",lim[1],lim[2]);auto ack=send(p);
    assert(ack.outcome==motion::CommandOutcome::REJECTED&&!std::strcmp(ack.reason,"JOYSTICK_VELOCITY_RANGE"));
    ack=send("{\"vx\":0.001,\"vy\":0,\"yaw_rate\":0,\"gait\":\"TROT\"}");assert(!std::strcmp(ack.reason,"GAIT_NOT_IN_SIMPLE_V1"));
    ack=send("{\"vx\":0.001,\"vy\":0,\"yaw_rate\":0}");assert(!std::strcmp(ack.reason,"STANDARD_STRIDE_REQUIRED"));  // untimed presets only without JOYSTICK
    std::snprintf(p,sizeof p,"{\"vx\":%.12f,\"vy\":%.12f,\"yaw_rate\":%.12f,\"gait\":\"JOYSTICK\"}",-lim[0]*.6,-lim[1]*.5,lim[2]*.5);ack=send(p);assert(ack.outcome==motion::CommandOutcome::ACCEPTED);
    for(unsigned n=0;n<3000&&api.m.state()!=simple::State::WALKING;++n){if(n%12==0)assert(send(p).outcome==motion::CommandOutcome::ACCEPTED||api.m.state()==simple::State::MOVING);api.tick();}
    assert(api.m.state()==simple::State::WALKING&&simple::StopReferenceProbe::reverse(api.m));
    for(unsigned n=0;n<200;++n){if(n%12==0)assert(send(p).outcome==motion::CommandOutcome::ACCEPTED);api.tick();}
    std::snprintf(p,sizeof p,"{\"vx\":%.12f,\"vy\":0,\"yaw_rate\":0}",simple::kStrideM[1]/T);assert(!std::strcmp(send(p).reason,"ALREADY_WALKING"));
    test_clock_us=api.p.now;service.update();char status[core::kMotionStatusJsonBytes]{};assert(service.statusJson(status,sizeof(status)-640,api.p.now/1000));
    assert(std::strstr(status,"\"walk_velocity\":{\"vx_m_s\":-0.00415,\"vy_m_s\":-0.00154,\"yaw_rate_rad_s\":0.01538,\"joystick\":true}"));
    char caps[2048];assert(service.capabilitiesJson(caps,sizeof caps));assert(std::strstr(caps,"\"joystick\":{\"vx_max_m_s\":0.00692,\"vy_max_m_s\":0.00308,\"yaw_rate_max_rad_s\":0.03077,\"envelope\":\"ELLIPSOID\",\"timeout_ms\":1000"));
    ack=send("{\"vx\":0,\"vy\":0,\"yaw_rate\":0,\"gait\":\"JOYSTICK\"}");assert(ack.outcome==motion::CommandOutcome::ACCEPTED&&!std::strcmp(api.m.reason(),"WALK_STOPPING"));
    assert(api.reachStand());std::printf("JOYSTICK API status %zu bytes, capabilities %zu bytes\n",std::strlen(status),std::strlen(caps));}
  std::puts("JOYSTICK PASS: vx/vy/yaw_rate sign and SE(2) proportionality, ellipsoid saturation, neutral WALK STOP, ESP deadline, rising timeout, fore/aft reversal via WALK STOP, per-axis rate limits and RAW continuity, SE(2) track == closed-form law, STOP and STAND recovery, IMU board; software only, NO HARDWARE MOTION VALIDATION");
}

// IMU level slew in physical time (dev.15): a regular release moves the
// correction by at most kImuSlewRadS x period, a late one by its real elapsed
// time capped at kImuSlewMaxDtUs, never a catch-up jump; reason and values
// are exported.
static void imuSlewTime(){
  constexpr double degree=3.14159265358979323846/180;
  Rig r;assert(!r.m.stand(.10,r.p.now));assert(r.reachStand());
  const double half=1.5*degree/2;r.in.quaternion[0]=std::cos(half);r.in.quaternion[1]=std::sin(half);r.in.quaternion[2]=r.in.quaternion[3]=0;  // 1.5 deg roll
  const double regular=simple::kImuSlewRadS*simple::kPeriodUs/1e6,capped=simple::kImuSlewRadS*simple::kImuSlewMaxDtUs/1e6;
  r.tick(false);  // first correction after HOLD entry: its physical gap may be two periods
  double prev=simple::StopReferenceProbe::roll(r.m),maxRegular=0,late=0;bool slewing=false;
  for(unsigned n=0;n<6;++n){r.tick(false);const double now=simple::StopReferenceProbe::roll(r.m);maxRegular=std::fmax(maxRegular,std::abs(now-prev));prev=now;slewing|=!std::strcmp(r.m.imuReason(),"SLEW");}
  r.p.now+=3*simple::kPeriodUs;r.tick(false);late=std::abs(simple::StopReferenceProbe::roll(r.m)-prev);  // three releases lost, 80 ms < deadline guard
  assert(r.m.state()==simple::State::STANDING&&slewing&&maxRegular>.9*regular&&maxRegular<=regular*(1+1e-9)&&late>regular*1.5&&late<=capped*(1+1e-9));
  assert(std::abs(r.m.imuApplied(0)-simple::StopReferenceProbe::roll(r.m))<1e-12);
  std::printf("IMU slew in physical time: %.2f mrad per regular release (%.3f rad/s), %.2f mrad after 3 lost releases (cap %.2f), reason SLEW exported\n",maxRegular*1e3,simple::kImuSlewRadS,late*1e3,capped*1e3);
}

// STOP inside the PRONE descent, then STAND (dev.15, dev.14 HW RCA A): the held
// RAW restarts the certified route of its region with torque ON. Every phase
// at early/mid/late instants; first frame continuous with the held RAW, no
// enable/SAFE_OFF, a second STOP during the recovery, STANDING at the end.
static void proneStopRecovery(){
  const char* phases[]={"PRONE_HEIGHT","PRONE_LIFT","PRONE_DEPLOY","PRONE_LAND"};unsigned admitted=0,refused=0;bool observedCase=false;
  for(const char* target:phases)for(unsigned after:{3u,30u,90u}){
    Rig r;assert(!r.m.stand(.10,r.p.now));assert(r.reachStand());assert(!r.m.prone(r.p.now));
    unsigned n=0;for(;n<6000&&std::strcmp(r.m.phase(),target);++n){r.tick();assert(r.m.state()!=simple::State::FAULT);}
    if(std::strcmp(r.m.phase(),target))continue;
    for(unsigned k=0;k<after&&!std::strcmp(r.m.phase(),target);++k)r.tick();
    if(std::strcmp(r.m.phase(),target))continue;  // phase shorter than this instant
    const std::string region=r.m.poseRegion(r.p.now);
    const auto enables=r.p.enables,offs=r.p.offCalls;r.m.stop();r.m.stop();  // double STOP
    for(unsigned k=0;k<300&&!r.m.stopVerified();++k)r.tick();assert(r.m.stopVerified()&&r.m.energized());
    std::array<uint16_t,12> held;std::copy(r.p.target,r.p.target+12,held.begin());
    const char* why=r.m.standResumeRefusal(r.p.now);
    if(why){assert(!std::strcmp(why,"STAND_PATH_NOT_CLEAR")||!std::strcmp(why,"STOP_RECOVERY_POSTURE_UNQUALIFIED"));
      assert(r.m.stand(.10,r.p.now));++refused;std::printf("PRONE STOP %s+%u (%s): STAND refused %s\n",target,after,region.c_str(),why);continue;}
    assert(!r.m.stand(.10,r.p.now));assert(!std::strcmp(r.m.reason(),"STAND_FROM_STOPPED_PRONE"));
    const auto frames=r.p.sent.size();r.tick();unsigned first=0;
    if(r.p.sent.size()>frames)for(unsigned i=0;i<12;++i)first=std::max(first,unsigned(std::abs(int(r.p.sent[frames][i])-int(held[i]))));
    assert(first<=12);  // the first written frame starts at the held RAW (one release of route motion)
    // STOP during the recovery, inside its first planned segment (native DISENGAGE/
    // ALIGN keep the dev.14 STOP: no synchronous reference, SAFE_OFF), then STAND again.
    for(unsigned k=0;k<3000&&std::strcmp(r.m.phase(),"STAND_DEPLOY")&&std::strcmp(r.m.phase(),"STAND_LIFT")&&std::strcmp(r.m.phase(),"HEIGHT");++k){r.tick();assert(r.m.state()!=simple::State::FAULT);}
    for(unsigned k=0;k<20;++k)r.tick();
    r.m.stop();for(unsigned k=0;k<300&&!r.m.stopVerified();++k)r.tick();assert(r.m.stopVerified()&&r.m.energized());
    const char* again=r.m.standResumeRefusal(r.p.now);
    if(again)std::fprintf(stderr,"second recovery refused %s after %s\n",again,target);
    assert(!again&&!r.m.stand(.10,r.p.now));
    for(unsigned k=0;k<6000&&r.m.state()!=simple::State::STANDING;++k){r.tick();assert(r.m.state()!=simple::State::FAULT);}
    assert(r.m.state()==simple::State::STANDING&&r.p.enables==enables&&r.p.offCalls==offs);
    observedCase|=!std::strcmp(target,"PRONE_DEPLOY")&&region=="PRONE_ENTRY";++admitted;
    std::printf("PRONE STOP %s+%u (%s): STAND admitted, first frame %u ticks from held RAW, STOP in recovery, STANDING, torque never cycled\n",target,after,region.c_str(),first);}
  assert(admitted>0&&observedCase);
  std::printf("PRONE STOP -> STAND: %u admitted, %u refused with a precise reason (software only)\n",admitted,refused);
}

int main(){standHoldTilt();
  proneStopRecovery();
  imuSlewTime();
  joystickWalk();
  walkLevelBoard();
  walkTxLatency();
  proneHandoverWait();
  // Exercise the production gait + IK across every join: swing lift-off and
  // touchdown, four-contact sway windows, end of the start ramp, decel start,
  // terminal touchdown and recentring end, for each stride and quarter stop.
  for(unsigned stride=0;stride<3;++stride)for(double stop : {1.,1.25,1.5,1.75,2.}){
    motion::GaitParameters g;g.heightM=simple::baseLinkHeightFromNominalM(simple::kWalkHeightM);g.liftM=simple::kWalkLiftM;g.duty=simple::kWalkDuty;g.advanceXM=simple::kStrideM[stride];
    g.swayXM=simple::kWalkSwayXM;g.swayYM=simple::kWalkSwayYM;g.startCycles=simple::kWalkStartCycles;g.stopCycles=simple::kWalkStopCycles;
    const double terminal=stop+g.stopCycles,w=motion::gaitSettleCycles(g);
    for(double base=0;base<terminal+1;base+=.25)for(double event : {0.,w}){  // quarter touchdown and four-contact window end
      const double c=base+event;if(c<=1e-6||c>=terminal+w-1e-6)continue;
      const double term=c>stop?terminal:-1;
      auto a=motion::solveGait(g,motion::gaitCartesian(g,c-1e-7,1./simple::kWalkPeriodS,0,term));
      auto b=motion::solveGait(g,motion::gaitCartesian(g,c+1e-7,1./simple::kWalkPeriodS,0,term));
      const auto native=motion::solveGait(g,b.cartesian,nullptr,false);
      assert(native.status==b.status&&native.target.valid==b.target.valid);
      for(unsigned l=0;l<4;++l)assert(native.condition[l]==b.condition[l]);
      for(unsigned i=0;i<12;++i)assert(motion::jointSlot(native.target,i)==motion::jointSlot(b.target,i));
      assert(a.target.valid&&b.target.valid);
      for(unsigned l=0;l<4;++l){
        const auto av=a.velocityRadS[l],bv=b.velocityRadS[l],aa=a.accelerationRadS2[l],ba=b.accelerationRadS2[l];
        assert(std::abs(av.hip-bv.hip)<1e-4&&std::abs(av.upper-bv.upper)<1e-4&&std::abs(av.lower-bv.lower)<1e-4);
        assert(std::abs(aa.hip-ba.hip)<.002&&std::abs(aa.upper-ba.upper)<.002&&std::abs(aa.lower-ba.lower)<.002);
      }
      for(unsigned i=0;i<12;++i)assert(std::abs(motion::jointSlot(a.target,i)-motion::jointSlot(b.target,i))<1e-5);
    }
    // Rest at both ends: zero joint velocity/acceleration in the gait clock.
    for(double c : {0.,terminal+w}){auto f=motion::solveGait(g,motion::gaitCartesian(g,c,1./simple::kWalkPeriodS,0,c>0?terminal:-1));assert(f.target.valid);
      for(unsigned l=0;l<4;++l){const auto v=f.velocityRadS[l],a=f.accelerationRadS2[l];
        assert(std::abs(v.hip)+std::abs(v.upper)+std::abs(v.lower)<1e-9&&std::abs(a.hip)+std::abs(a.upper)+std::abs(a.lower)<1e-9);}}
  }
  // STAND reference: certified route vias, analytic ceilings and per-joint range.
  for(int group=0;group<2;++group){simple::ViaPlan plan;
    for(unsigned k:group?std::vector<unsigned>(std::begin(simple::collision::kLiftKnots),std::end(simple::collision::kLiftKnots)):
                         std::vector<unsigned>(std::begin(simple::collision::kDeployKnots),std::end(simple::collision::kDeployKnots))){
      const double* p=motion::paired_data::kKnots[k].p;double x[simple::ViaPlan::kDim];
      for(unsigned l=0;l<4;++l){x[l*3]=((l==0||l==3)?-1:1)*p[7];x[l*3+1]=p[l<2?2:0];x[l*3+2]=std::fmax((l<2?-88.5:-89.)*3.14159265358979323846/180,p[l<2?3:1]);}
      x[12]=p[4];x[13]=p[5];x[14]=p[6];assert(plan.add(x));}
    assert(plan.plan(simple::kStandSpeedRadS,simple::kStandAccelerationRadS2)&&plan.iterations<16);
    double previous[simple::ViaPlan::kDim]{};
    for(double t=0;t<=plan.duration+1e-9;t+=.001){double q[simple::ViaPlan::kDim],v[simple::ViaPlan::kDim],a[simple::ViaPlan::kDim];plan.sample(t,q,v,a);
      for(unsigned i=0;i<12;++i){double lo=1e9,hi=-1e9;for(unsigned k=0;k<plan.n;++k){lo=std::fmin(lo,plan.p[k][i]);hi=std::fmax(hi,plan.p[k][i]);}
        assert(q[i]>=lo-1e-12&&q[i]<=hi+1e-12&&std::abs(v[i])<=simple::kStandSpeedRadS+1e-9&&std::abs(a[i])<=simple::kStandAccelerationRadS2+1e-9);
        if(t>0)assert(std::abs(q[i]-previous[i])<=simple::kStandSpeedRadS*.001+1e-9);previous[i]=q[i];}}
    std::printf("STAND %s reference: %u vias, %.3f s, no intermediate arrival\n",group?"LIFT":"DEPLOY",plan.n,plan.duration);
  }
  Rig r;assert(r.m.state()==simple::State::READY);
  assert(r.m.stand(0,r.p.now));assert(r.p.enables==0&&r.p.frames==0);
  assert(!r.m.stand(.15,r.p.now));assert(r.reachStand());assert(r.p.enables==12);assert(r.m.heightValid());
  // Nominal STAND 150: the joint targets qualified on the robot; base_link 147.8 mm from the canonical geometry.
  for(unsigned i=0;i<12;++i)assert(std::abs(motion::jointSlot(r.m.target(),i)-(i%3==0?motion::canonicalStandDefinition().standSeed[i/3].hip:i%3==1?motion::canonicalStandDefinition().standSeed[i/3].upper:motion::canonicalStandDefinition().standSeed[i/3].lower))<1e-6);
  assert(std::abs(simple::baseLinkHeightFromNominalM(.15)-.1478)<1e-12);assert(std::abs(simple::nominalHeightFromBaseLinkM(.1478)-.15)<1e-12);assert(std::abs(r.m.requestedHeightM()-.15)<1e-12);
  for(auto leg:{motion::LegId::LF,motion::LegId::RF})assert(motion::legModel(leg)->hipOrigin.z==.0487);for(auto leg:{motion::LegId::RH,motion::LegId::LH})assert(motion::legModel(leg)->hipOrigin.z==.0287);
  std::printf("STAND sequence: %u coordinated pose writes; scripted observation only\n",r.p.frames);
  const auto beforeHeight=r.p.frames;assert(r.m.stand(.049,r.p.now));assert(!r.m.stand(.05,r.p.now));assert(r.p.frames==beforeHeight);assert(r.reachStand());assert(!r.m.stand(.15,r.p.now));assert(r.reachStand());
  assert(!r.m.walk(1,r.p.now));
  for(unsigned i=0;i<2000&&r.m.state()!=simple::State::WALKING;++i)r.tick();assert(r.m.state()==simple::State::WALKING);
  double maxStep=0,prev[12]{};for(unsigned i=0;i<12;++i)prev[i]=motion::jointSlot(r.m.target(),i);
  const auto start=std::chrono::steady_clock::now();
  for(unsigned k=0;k<1200;++k){r.tick();assert(r.m.state()==simple::State::WALKING);
    for(unsigned i=0;i<12;++i){const double q=motion::jointSlot(r.m.target(),i);maxStep=std::fmax(maxStep,std::abs(q-prev[i]));prev[i]=q;}}
  const auto elapsed=std::chrono::duration<double,std::micro>(std::chrono::steady_clock::now()-start).count();
  assert(maxStep<.05);r.m.walkStop();assert(r.reachStand());
  std::printf("WALK STANDARD production gait/IK: max target step %.6f rad, host mean %.1f us/frame (not ESP timing)\n",maxStep,elapsed/1200);
  const auto offBefore=r.p.offCalls;r.m.stop();r.m.update(r.p.now+1);assert(r.p.offCalls==offBefore);assert(r.m.stopping());
  for(unsigned i=0;i<150&&!r.m.stopVerified();++i)r.tick();assert(r.m.stopVerified());assert(r.m.state()==simple::State::STOPPED&&r.m.energized());
  // No automatic resume: an admitted explicit STAND reuses the existing torque.
  assert(!r.m.stand(.15,r.p.now));assert(r.p.enables==12);assert(r.reachStand());
  Rig interrupted;assert(!interrupted.m.stand(.15,interrupted.p.now));
  for(unsigned i=0;i<400&&interrupted.p.frames<25;++i)interrupted.tick();
  interrupted.m.stop();for(unsigned i=0;i<150&&!interrupted.m.offVerified()&&!interrupted.m.stopVerified();++i)interrupted.tick();
  assert(interrupted.m.offVerified()||interrupted.m.stopVerified());
  // dev.15: a STOP held in a planned route segment restarts that region's route
  // with torque ON; a native move ends in SAFE_OFF and STAND stays refused.
  if(interrupted.m.stopVerified()){assert(!interrupted.m.stand(.15,interrupted.p.now));assert(interrupted.reachStand());}
  else assert(interrupted.m.stand(.15,interrupted.p.now));
  // STOP inside STAND_LIFT retains the nominal route reference and torque; no implicit resume.
  Rig liftStop;assert(!liftStop.m.stand(.15,liftStop.p.now));
  for(unsigned i=0;i<1000&&strcmp(liftStop.m.phase(),"STAND_LIFT");++i)liftStop.tick();
  for(unsigned i=0;i<60;++i)liftStop.tick();assert(!strcmp(liftStop.m.phase(),"STAND_LIFT"));
  liftStop.m.stop();for(unsigned i=0;i<150&&!liftStop.m.stopVerified();++i)liftStop.tick();
  // IMU-corrected writer samples now admit C0 arrest without an analytic
  // derivative claim for the preceding compensation signal.
  assert(liftStop.m.stopVerified()&&liftStop.m.energized()&&!liftStop.m.offVerified());
  const auto liftEnables=liftStop.p.enables;assert(!liftStop.m.stand(.15,liftStop.p.now));  // dev.15: held RAW restarts the certified LIFT route
  assert(liftStop.reachStand()&&liftStop.p.enables==liftEnables);
  Rig gaitRestart;assert(!gaitRestart.m.walk(2,gaitRestart.p.now));
  for(unsigned i=0;i<1200&&gaitRestart.m.state()!=simple::State::WALKING;++i)gaitRestart.tick();
  for(unsigned i=0;i<300;++i)gaitRestart.tick();gaitRestart.m.stop();for(unsigned i=0;i<150&&!gaitRestart.m.stopVerified();++i)gaitRestart.tick();
  assert(gaitRestart.m.stopVerified()&&gaitRestart.m.energized());assert(!gaitRestart.m.stand(.06,gaitRestart.p.now)); // gait-law completion, then HEIGHT 60
  Rig native;assert(!native.m.stand(.15,native.p.now));native.p.holdObservation=true;
  for(unsigned i=0;i<3000&&native.m.state()!=simple::State::FAULT;++i)native.tick(false);
  assert(native.m.state()==simple::State::FAULT);assert(!strcmp(native.m.reason(),"NATIVE_TARGET_NOT_REACHED"));
  Rig alarm;assert(!alarm.m.stand(.15,alarm.p.now));assert(alarm.reachStand());alarm.p.observed[3].status=8;alarm.tick(false);alarm.tick(false);alarm.tick(false);alarm.tick(false);assert(alarm.m.state()==simple::State::FAULT);
  Rig comm;assert(!comm.m.stand(.15,comm.p.now));assert(comm.reachStand());comm.p.missing=true;
  for(unsigned i=0;i<20&&comm.m.state()!=simple::State::FAULT;++i)comm.tick(false);assert(comm.m.state()==simple::State::FAULT);
  Rig stop;assert(!stop.m.stand(.15,stop.p.now));assert(stop.reachStand());const unsigned before=stop.p.frames;stop.p.stopOnRead=true;stop.tick(false);assert(stop.p.frames==before&&stop.m.stopping());
  // A blocked Controller loop (3.1 s without a release) is a real fault.
  Rig blocked;assert(!blocked.m.stand(.15,blocked.p.now));assert(blocked.reachStand());blocked.p.now+=3100000;blocked.m.inputs(blocked.in);blocked.m.update(blocked.p.now);
  assert(blocked.m.state()==simple::State::FAULT&&!strcmp(blocked.m.reason(),"MOTION_DEADLINE_LOST"));
  // No command source after acceptance (closed Dashboard, no session, no
  // poll): STAND holds and WALK continues for 10 simulated minutes; only a
  // command or a physical fault ends them.
  Rig held;assert(!held.m.stand(.10,held.p.now));assert(held.reachStand());
  for(unsigned k=0;k<30000;++k){held.tick();assert(held.m.state()==simple::State::STANDING);}
  assert(held.m.energized()&&!held.m.stopping()&&held.m.requestedHeightM()==.10);
  Rig walking;assert(!walking.m.walk(0,walking.p.now));
  for(unsigned k=0;k<3000&&walking.m.state()!=simple::State::WALKING;++k)walking.tick();
  for(unsigned k=0;k<30000;++k){walking.tick();assert(walking.m.state()==simple::State::WALKING);}
  assert(walking.m.walkCycles()>90&&walking.m.energized());
  walking.m.walkStop();assert(walking.reachStand());
  std::printf("No command source: STAND held 600 s, WALK STANDARD %.1f cycles in 600 s, WALK STOP -> STAND (software only)\n",walking.m.walkCycles());
  Rig bad;bad.in.maxRad[1]=.2;bad.m.inputs(bad.in);assert(bad.m.stand(.15,bad.p.now));assert(bad.p.enables==0);
  for(unsigned stride=0;stride<3;++stride){Rig preset;assert(!preset.m.walk(stride,preset.p.now));
    for(unsigned i=0;i<3000&&preset.m.state()!=simple::State::WALKING;++i)preset.tick();assert(preset.m.state()==simple::State::WALKING);
    // Both signs of a real quaternion; corrected targets must remain in limits.
    for(unsigned k=0;k<450;++k){const double roll=.015*std::sin(k*.01),pitch=.01*std::cos(k*.01);
      preset.in.quaternion[0]=std::cos(roll/2)*std::cos(pitch/2);preset.in.quaternion[1]=std::sin(roll/2)*std::cos(pitch/2);
      preset.in.quaternion[2]=std::cos(roll/2)*std::sin(pitch/2);preset.in.quaternion[3]=-std::sin(roll/2)*std::sin(pitch/2);
      preset.tick();assert(preset.m.state()==simple::State::WALKING);
      for(unsigned i=0;i<12;++i)assert(motion::jointSlot(preset.m.target(),i)>=preset.in.minRad[i]&&motion::jointSlot(preset.m.target(),i)<=preset.in.maxRad[i]);
    }
  }
  // Apply roll/pitch while STAND is still moving, not only at HOLD.
  Rig levelStand,tiltedStand;assert(!levelStand.m.stand(.15,levelStand.p.now));assert(!tiltedStand.m.stand(.15,tiltedStand.p.now));
  unsigned correctedMoving=0;double standCorrection=0;
  for(unsigned k=0;k<1200&&levelStand.m.state()!=simple::State::STANDING;++k){
    const double roll=.02*std::sin(k*.007),pitch=.01*std::sin(k*.011);
    tiltedStand.in.quaternion[0]=std::cos(roll/2)*std::cos(pitch/2);tiltedStand.in.quaternion[1]=std::sin(roll/2)*std::cos(pitch/2);
    tiltedStand.in.quaternion[2]=std::cos(roll/2)*std::sin(pitch/2);tiltedStand.in.quaternion[3]=-std::sin(roll/2)*std::sin(pitch/2);
    levelStand.tick();tiltedStand.tick();assert(tiltedStand.m.state()!=simple::State::FAULT);
    if(levelStand.m.state()==simple::State::MOVING&&tiltedStand.m.state()==simple::State::MOVING&&
       !strcmp(levelStand.m.phase(),tiltedStand.m.phase())&&levelStand.m.target().valid&&tiltedStand.m.target().valid){
      double delta=0;for(unsigned i=0;i<12;++i)delta=std::fmax(delta,std::abs(motion::jointSlot(levelStand.m.target(),i)-motion::jointSlot(tiltedStand.m.target(),i)));
      if(delta>1e-5){++correctedMoving;standCorrection=std::fmax(standCorrection,delta);}
    }
  }
  assert(correctedMoving>50);assert(tiltedStand.reachStand());
  assert(!tiltedStand.m.stand(.05,tiltedStand.p.now));assert(tiltedStand.reachStand());
  std::printf("IMU during MOVING STAND: %u different releases, max reference difference %.6f rad (software only)\n",correctedMoving,standCorrection);
  Rig stopControl;assert(!stopControl.m.stand(.15,stopControl.p.now));assert(stopControl.reachStand());stopControl.p.stopOnControl=true;
  for(unsigned k=0;k<6&&!stopControl.m.stopping();++k){const auto count=stopControl.p.offCalls;stopControl.tick(false);if(stopControl.m.stopping())assert(stopControl.p.offCalls==count);}
  assert(stopControl.m.stopping());
  for(unsigned phase=0;phase<4;++phase){Rig interrupt;assert(!interrupt.m.stand(.15,interrupt.p.now));
    if(phase!=0){for(unsigned k=0;k<100&&strcmp(interrupt.m.phase(),"ENABLE");++k)interrupt.tick();assert(!strcmp(interrupt.m.phase(),"ENABLE"));}
    if(phase<2)interrupt.p.stopOnControl=true;else if(phase==2)interrupt.p.stopOnEnable=true;else interrupt.p.stopOnFrame=true;
    const unsigned off=interrupt.p.offCalls;
    for(unsigned k=0;k<30&&!interrupt.m.stopping();++k)interrupt.tick(false);
    assert(interrupt.m.stopping()&&interrupt.p.offCalls==off+1);
    assert(interrupt.m.state()==(phase==3?simple::State::FAULT:simple::State::STOPPED));
    if(phase==3)assert(!strcmp(interrupt.m.reason(),"STOP_REFERENCE_UNAVAILABLE"));
    if(phase<2)assert(interrupt.p.enables==0);
  }
  Rig rejectedHeight;assert(!rejectedHeight.m.stand(.15,rejectedHeight.p.now));assert(rejectedHeight.reachStand());
  assert(rejectedHeight.m.stand(.05,rejectedHeight.p.now+300000));assert(rejectedHeight.m.requestedHeightM()==.15);
  Rig settling;assert(!settling.m.walk(0,settling.p.now));
  for(unsigned k=0;k<2000&&settling.m.state()!=simple::State::WALKING;++k)settling.tick();assert(settling.m.state()==simple::State::WALKING);
  for(unsigned k=0;k<1500&&settling.m.walkCycles()<1.5;++k)settling.tick();settling.m.walkStop();
  for(unsigned k=0;k<1500&&strcmp(settling.m.phase(),"WALK_SETTLE");++k)settling.tick();assert(!strcmp(settling.m.phase(),"WALK_SETTLE"));
  const auto settleFrames=settling.p.frames;for(auto& f:settling.p.observed)f.speed=100;
  settling.in.quaternion[0]=std::cos(.025/2);settling.in.quaternion[1]=std::sin(.025/2);
  for(unsigned k=0;k<25;++k){settling.tick();assert(!strcmp(settling.m.phase(),"WALK_SETTLE"));}
  assert(settling.p.frames>settleFrames);for(auto& f:settling.p.observed)f.speed=0;assert(settling.reachStand());
  Rig api;core::MotionService service;service.begin(&api.m,nullptr,nullptr);service.setReady(true,"READY");
  motion::MotionCommand cmd;cmd.owner=motion::ClientOwner::DASHBOARD;cmd.intent=motion::Intent::STAND;cmd.sequence=1;cmd.issued_at_ms=api.p.now/1000;cmd.ttl_ms=1000;std::strcpy(cmd.id,"test-api-1");
  test_clock_us=api.p.now;auto ack=service.command(cmd,api.p.now/1000);assert(ack.outcome==motion::CommandOutcome::ACCEPTED);
  assert(service.command(cmd,api.p.now/1000).outcome==motion::CommandOutcome::ACCEPTED);assert(api.reachStand());test_clock_us=api.p.now;service.update();
  char status[core::kMotionStatusJsonBytes]{};assert(service.statusJson(status,sizeof(status)-640,api.p.now/1000));
  std::printf("API status %zu bytes (%zu B transport, 640 B reserved); JSON=%s\n",std::strlen(status),core::kMotionStatusJsonBytes,status);
  std::strcpy(cmd.id,"new-browser-session-1");cmd.intent=motion::Intent::STATUS;cmd.issued_at_ms=api.p.now/1000;assert(service.command(cmd,api.p.now/1000).outcome==motion::CommandOutcome::ACCEPTED);
  cmd.intent=motion::Intent::STOP;cmd.ttl_ms=0;cmd.issued_at_ms=0;assert(service.command(cmd,api.p.now/1000).outcome==motion::CommandOutcome::ACCEPTED);api.tick(false);assert(api.m.stopping());
  Rig unsafe;unsafe.p.observed[0].q=.7;assert(!unsafe.m.stand(.15,unsafe.p.now));for(unsigned i=0;i<30&&unsafe.m.state()!=simple::State::FAULT;++i)unsafe.tick(false);
  assert(unsafe.m.state()==simple::State::FAULT&&unsafe.p.enables==0);
  Rig off;off.p.verify=false;off.m.stop();for(unsigned i=0;i<30;++i)off.tick(false);assert(!off.m.offVerified()&&off.m.stopping());
  // PRONE -> STAND50: one DEPLOY and one LIFT reference, no HEIGHT move and
  // so no STAND150 detour. The rig IMU stays level while the route nominal is
  // pitched up to 0.18 rad, so a bounded correction is active during the rise.
  // In HOLD close the board plant: prescribed identity IMU while the encoders
  // keep a tilted body would instead describe a tilted plane, not flat ground.
  Rig low;assert(!low.m.stand(.05,low.p.now));std::string lowPhases;const char* lowPhase="";
  for(unsigned k=0;k<3000&&low.m.state()!=simple::State::STANDING;++k){low.tick();assert(low.m.state()!=simple::State::FAULT);
    if(strcmp(lowPhase,low.m.phase())){lowPhase=low.m.phase();lowPhases+=std::string(lowPhase)+" ";}}
  assert(low.m.state()==simple::State::STANDING&&lowPhases=="ACQUIRE REFERENCE ENABLE DISENGAGE ALIGN STAND_DEPLOY STAND_LIFT HOLD ");
  const double lowSeconds=(low.p.now-1300000)/1e6;
  LevelBoardPlant lowPlant(low);lowPlant.settle(0,0,150);
  {const double body=simple::baseLinkHeightFromNominalM(.05);auto d=motion::canonicalStandDefinition();d.lowBodyHeightM=body/2;d.standBodyHeightM=body;const auto s=motion::generateStandTarget(d).target;
   for(unsigned i=0;i<12;++i){assert(std::abs(motion::jointSlot(simple::StopReferenceProbe::hold(low.m).target,i)-motion::jointSlot(s,i))<1e-9);
     uint16_t raw=0;assert(low.p.resolve(i,motion::jointSlot(s,i),raw));assert(std::abs(int(low.p.target[i])-int(raw))<=3);}}
  std::printf("PRONE->STAND50: %s(%.2f s replay)\n",lowPhases.c_str(),lowSeconds);
  // WALK STOP ends recentred on the canonical profile height.
  Rig centred;assert(!centred.m.walk(2,centred.p.now));
  for(unsigned k=0;k<3000&&centred.m.state()!=simple::State::WALKING;++k)centred.tick();
  for(unsigned k=0;k<500;++k)centred.tick();centred.m.walkStop();
  for(unsigned k=0;k<2000&&strcmp(centred.m.phase(),"WALK_SETTLE");++k)centred.tick();assert(!strcmp(centred.m.phase(),"WALK_SETTLE"));
  {const double body=simple::baseLinkHeightFromNominalM(simple::kWalkHeightM);auto d=motion::canonicalStandDefinition();d.lowBodyHeightM=body/2;d.standBodyHeightM=body;const auto s=motion::generateStandTarget(d).target;  // last gait sample
   // Canonical gait recentring is exact before attitude compensation. Its
   // measured-world terminal orientation is checked by the closed-loop plant;
   // a prescribed identity IMU with moving encoders is not that physical plant.
   for(unsigned i=0;i<12;++i)assert(std::abs(motion::jointSlot(simple::StopReferenceProbe::walkNominal(centred.m).target,i)-motion::jointSlot(s,i))<1e-6);}
  assert(centred.reachStand());
  // Repeated healthy STOPs keep the same controlled arrest and evidence dwell;
  // the dedicated STOP tests compare the entire single/duplicate reference.
  for(unsigned during=0;during<3;++during){
    Rig s;
    if(during==0){assert(!s.m.stand(.15,s.p.now));assert(s.reachStand());for(unsigned i=0;i<100;++i)s.tick();}
    else if(during==1){assert(!s.m.stand(.10,s.p.now));assert(s.reachStand());for(unsigned i=0;i<100;++i)s.tick();assert(!s.m.stand(.15,s.p.now));for(unsigned i=0;i<10;++i)s.tick();}
    else{assert(!s.m.walk(1,s.p.now));for(unsigned i=0;i<3000&&s.m.state()!=simple::State::WALKING;++i)s.tick();for(unsigned i=0;i<20;++i)s.tick();}
    const unsigned off=s.p.offCalls,enables=s.p.enables;unsigned updates=0;
    while(!s.m.stopVerified()&&updates<150){s.m.stop();s.tick();++updates;}
    assert(s.m.stopVerified()&&s.p.offCalls==off&&s.m.energized()&&s.m.stopping());
    assert(s.p.enables==enables&&s.m.state()==simple::State::STOPPED&&!s.m.stopSupportVerified());
    assert(!s.m.stand(.15,s.p.now));assert(s.reachStand());
    assert(s.p.enables==enables&&s.p.offCalls==off);if(during==0)for(unsigned i=0;i<100;++i)s.tick();
  }
  // One servo fails its read-back: SAFE_OFF stays pending and re-broadcasts;
  // repeated STOPs keep the 12 verified servos; nothing can re-enable; the
  // servo recovering completes 13/13 without another STOP.
  Rig failing;assert(!failing.m.stand(.15,failing.p.now));assert(failing.reachStand());failing.p.failVerify=5;
  const unsigned failOff=failing.p.offCalls,failEnables=failing.p.enables,failFrames=failing.p.frames;
  failing.m.fault("TEST_SAFE_OFF_READBACK_FAILURE");
  for(unsigned i=0;i<80;++i){if(i%3==0)failing.m.stop();failing.tick(false);}
  assert(!failing.m.offVerified()&&failing.m.stopping()&&failing.m.energized()&&failing.m.offMask()==(0x1fff&~(1u<<5)));
  assert(failing.p.offCalls>failOff+3&&failing.p.enables==failEnables&&failing.p.frames==failFrames);
  assert(!strcmp(failing.m.stand(.15,failing.p.now),"FAULT_STOP_REQUIRED")&&failing.p.enables==failEnables);
  failing.p.failVerify=-1;unsigned recovered=0;while(!failing.m.offVerified()&&recovered<40){failing.tick(false);++recovered;}
  assert(failing.m.offVerified()&&recovered<=14&&!failing.m.energized());
  // A STOP after completion is a new SAFE_OFF (fresh broadcast + 13 reads).
  Rig again;assert(again.m.offVerified());const unsigned againOff=again.p.offCalls;again.m.stop();again.tick(false);
  assert(!again.m.offVerified()&&again.p.offCalls==againOff+1);for(unsigned i=0;i<13;++i)again.tick(false);assert(again.m.offVerified());
  // STOP after a FAULT cannot clear the fault latch or restart SAFE_OFF progress.
  Rig faulted;assert(!faulted.m.stand(.15,faulted.p.now));assert(faulted.reachStand());faulted.p.missing=true;
  for(unsigned i=0;i<20&&faulted.m.state()!=simple::State::FAULT;++i)faulted.tick(false);assert(faulted.m.state()==simple::State::FAULT);
  faulted.p.missing=false;const unsigned faultOff=faulted.p.offCalls;unsigned afterStop=0;
  while(!faulted.m.offVerified()&&afterStop<60){faulted.m.stop();faulted.tick(false);++afterStop;}
  assert(afterStop<=14&&faulted.p.offCalls<=faultOff+1&&faulted.m.state()==simple::State::FAULT);
  assert(faulted.m.stand(.15,faulted.p.now));
  // pose_region: shared certified-region predicate on 12 fresh ACTUAL angles.
  Rig pose;for(unsigned i=0;i<8;++i)pose.tick(false);assert(!strcmp(pose.m.poseRegion(pose.p.now),"PRONE_ENTRY"));
  assert(!pose.m.stand(.15,pose.p.now));assert(pose.reachStand());assert(!strcmp(pose.m.poseRegion(pose.p.now),"STAND_HEIGHT"));
  pose.p.missing=true;for(unsigned i=0;i<20;++i)pose.tick(false);assert(!strcmp(pose.m.poseRegion(pose.p.now),"UNKNOWN"));
  std::puts("Healthy STOP repeated: torque maintained from STANDING/MOVING/WALKING; fault SAFE_OFF read-back retries preserved, no fault clear or re-enable");
  // Pre-enable acquisition on the real PRONE of 24/10/2026 (USB log
  // MATDOG_STAND_WALK_20261024_152020: RAW and ACTUAL angles). GoalPosition
  // reads 0 at power-on, before the reference SYNC_WRITE, as on the robot.
  {
    const uint16_t raw[12]={1989,3077,3111,2023,1114,967,2092,1012,997,2022,3140,3130};
    const double q[12]={-.009204,1.506369,-1.547787,.010738,1.506369,-1.540117,.00767,1.609146,-1.590738,-.009204,1.607612,-1.592272};
    auto prone=[&](Rig& r){for(unsigned i=0;i<12;++i){r.p.observed[i].raw=raw[i];r.p.observed[i].q=q[i];r.p.target[i]=0;}
      for(unsigned i=0;i<8;++i)r.tick(false);assert(!strcmp(r.m.poseRegion(r.p.now),"PRONE_ENTRY"));};
    auto preparing=[](const Rig& r){return !strcmp(r.m.phase(),"ACQUIRE")||!strcmp(r.m.phase(),"REFERENCE")||!strcmp(r.m.phase(),"ENABLE");};
    // Torque OFF + identical frames: the scripted pose is held until all 12 are enabled.
    auto prepare=[&](Rig& r){unsigned n=0;while(n<100&&r.m.state()==simple::State::MOVING&&preparing(r)){r.tick(false);++n;}return n;};
    auto toEnable=[](Rig& r,unsigned slot){for(unsigned i=0;i<60&&!(r.p.enables==slot&&!strcmp(r.m.phase(),"ENABLE"));++i)r.tick(false);
      assert(r.p.enables==slot&&!strcmp(r.m.phase(),"ENABLE"));};
    auto shift=[&](Rig& r,unsigned slot,int ticks){r.p.observed[slot].raw=raw[slot]+ticks;r.p.observed[slot].q=q[slot]+ticks*simple::kTickRad;};
    // 1. Stable PRONE -> STAND50: ACQUIRE 12 + REFERENCE 1 + ENABLE 12 releases, then the route.
    {Rig r;prone(r);assert(!r.m.stand(.05,r.p.now));assert(prepare(r)==25&&r.p.enables==12&&r.p.frames==2&&r.m.state()==simple::State::MOVING);  // + DISENGAGE
     assert(r.reachStand());}
    // 2./3. Encoder variation inside the 12-tick native allowance is not a change; beyond it, abort before torque.
    for(int ticks:{1,-2,11,-11,13,-13}){Rig r;prone(r);assert(!r.m.stand(.05,r.p.now));toEnable(r,4);shift(r,9,ticks);prepare(r);
      if(std::abs(ticks)<=11)assert(r.p.enables==12&&r.m.state()==simple::State::MOVING);
      else assert(r.m.state()==simple::State::FAULT&&!strcmp(r.m.reason(),"REFERENCE_CHANGED_BEFORE_ENABLE")&&r.p.enables==9);}
    // One diagnostic identifies the exact predicate/slot, even if the next
    // HTTP sample sees OFF again. Stable encoders alone do not rule out torque.
    for(unsigned slot : {0u,5u,11u}) for(bool torque : {false,true}) {
      Rig r;prone(r);assert(!r.m.stand(.05,r.p.now));toEnable(r,slot);
      if(torque){r.p.observed[slot].torque=1;r.p.stuckTorque=slot;}else shift(r,slot,13);
      r.tick(false);if(torque){r.tick(false);r.tick(false);}
      assert(r.m.state()==simple::State::FAULT&&r.p.enables==slot&&r.p.referenceFaults==1&&r.p.faultSlot==slot);
      assert(r.p.faultFeedback.torque==(torque?1:0)&&r.p.faultRaw==raw[slot]);
      assert(std::abs(r.p.faultReference-q[slot])<1e-9);
      assert(r.p.faultFeedback.raw==raw[slot]+(torque?0:13));
      r.p.stuckTorque=-1;for(unsigned n=0;n<30;++n)r.tick(false);
      assert(r.m.offVerified()&&r.p.referenceFaults==1);
    }
    // A single unanswered control READ (ENABLE or ACQUIRE) or a reference frame one
    // servo did not apply is not a pose change: same servo again, torque still OFF.
    for(unsigned misses:{1u,2u}){Rig r;prone(r);assert(!r.m.stand(.05,r.p.now));toEnable(r,5);r.p.missControlSlot=5;r.p.missControlReads=misses;
      assert(prepare(r)==7+misses&&r.p.enables==12&&r.m.state()==simple::State::MOVING);assert(r.reachStand());}
    {Rig r;prone(r);assert(!r.m.stand(.05,r.p.now));r.p.missControlSlot=3;r.p.missControlReads=2;
     assert(prepare(r)==27&&r.p.enables==12);assert(r.reachStand());}
    {Rig r;prone(r);r.p.dropFrameSlot=7;r.p.dropFrames=1;assert(!r.m.stand(.05,r.p.now));
     assert(prepare(r)==26&&r.p.frames==3&&r.p.enables==12);assert(r.reachStand());}
    std::puts("PRONE 24/10 pre-enable: stable/<=11 ticks/1-2 missed READs/1 unapplied frame reach ENABLE x12; 13 ticks aborts before torque");
    // Persistent: three consecutive failures on one servo end the command; that servo is never enabled.
    {Rig r;prone(r);assert(!r.m.stand(.05,r.p.now));toEnable(r,5);r.p.missControlSlot=5;r.p.missControlReads=3;prepare(r);
     assert(r.m.state()==simple::State::FAULT&&!strcmp(r.m.reason(),"SERVO_CONTROL_READ_LOST")&&r.p.enables==5);}
    {Rig r;prone(r);r.p.dropFrameSlot=7;r.p.dropFrames=100;assert(!r.m.stand(.05,r.p.now));prepare(r);
     assert(r.m.state()==simple::State::FAULT&&!strcmp(r.m.reason(),"SERVO_TORQUE_OR_GOAL_LOST")&&r.p.enables==7&&r.p.frames==3);}
    // 4. No feedback: no reference, no frame, no torque.
    {Rig r;prone(r);assert(!r.m.stand(.05,r.p.now));r.p.missing=true;prepare(r);
     assert(r.m.state()==simple::State::FAULT&&!strcmp(r.m.reason(),"INITIAL_SERVO_READ_FAILED")&&r.p.enables==0&&r.p.frames==0);}
    // 5. STOP while a READ is being retried: SAFE_OFF 13/13, nothing more enabled.
    {Rig r;prone(r);assert(!r.m.stand(.05,r.p.now));toEnable(r,2);r.p.missControlSlot=2;r.p.missControlReads=2;r.tick(false);
     r.m.stop();for(unsigned i=0;i<15;++i)r.tick(false);
     assert(r.m.offVerified()&&r.m.state()==simple::State::STOPPED&&r.p.enables==2&&!r.m.energized());}
    // 6. A real change before enable latches FAULT; STOP cannot implicitly re-enable.
    {Rig r;prone(r);assert(!r.m.stand(.05,r.p.now));toEnable(r,4);shift(r,9,40);prepare(r);
     assert(r.m.state()==simple::State::FAULT&&!strcmp(r.m.reason(),"REFERENCE_CHANGED_BEFORE_ENABLE"));
     const unsigned frames=r.p.frames,enables=r.p.enables;for(unsigned i=0;i<200;++i)r.tick(false);
     assert(r.m.offVerified()&&r.m.state()==simple::State::FAULT&&r.p.frames==frames&&r.p.enables==enables);
     assert(!strcmp(r.m.stand(.05,r.p.now),"FAULT_STOP_REQUIRED"));
     r.m.stop();for(unsigned i=0;i<15;++i)r.tick(false);assert(r.m.state()==simple::State::FAULT&&r.m.offVerified());
     assert(r.m.stand(.05,r.p.now));assert(r.p.frames==frames&&r.p.enables==enables);}
    std::puts("PRONE 24/10 pre-enable: 3 consecutive failures/no feedback refuse torque; STOP during retry -> SAFE_OFF; real change -> latched FAULT");
    // Re-sent reference while 7 servos already hold torque: the identical
    // packet (real encoder, real slot IDs), the measured RAW, 300/8; OFF only
    // for the not-yet-enabled suffix, no other goal frame before the 12th enable.
    {Rig r;prone(r);r.p.dropFrameSlot=7;r.p.dropFrames=1;assert(!r.m.stand(.05,r.p.now));const unsigned off=r.p.offCalls;prepare(r);
     assert(r.p.enables==12&&r.p.sent.size()==3&&r.p.offCalls==off+2&&r.p.offFirst[off]==0&&r.p.offFirst[off+1]==7&&r.p.sentTorqueOn[0]==0&&r.p.sentTorqueOn[1]==7);
     uint8_t ids[12];for(unsigned i=0;i<12;++i)ids[i]=actuator::slotServo(i)->bus_id;
     uint8_t first[servo::packet::kGoalFrameBytes],again[servo::packet::kGoalFrameBytes];
     for(unsigned k=0;k<2;++k){assert(r.p.sentDrive[k].speed==simple::kStandDrive.speed&&r.p.sentDrive[k].acc==simple::kStandDrive.acc);for(unsigned i=0;i<12;++i)assert(r.p.sent[k][i]==raw[i]);}
     assert(servo::packet::buildGoalFrame(ids,r.p.sent[0].data(),12,simple::kStandDrive.speed,simple::kStandDrive.acc,first,sizeof(first))==sizeof(first));
     assert(servo::packet::buildGoalFrame(ids,r.p.sent[1].data(),12,simple::kStandDrive.speed,simple::kStandDrive.acc,again,sizeof(again))==sizeof(again));
     assert(!std::memcmp(first,again,sizeof(first)));
     for(unsigned i=0;i<12;++i)assert(r.p.observed[i].torque==1);}  // already enabled prefix was never disabled/re-enabled
    // First, middle and last servo: 1-2 failures recover in the same STAND, the
    // third ends it before that servo is enabled (READ and unapplied frame).
    for(unsigned slot:{0u,5u,11u})for(unsigned n:{1u,2u,3u}){
      {Rig r;prone(r);assert(!r.m.stand(.05,r.p.now));toEnable(r,slot);r.p.missControlSlot=slot;r.p.missControlReads=n;prepare(r);
       if(n<3)assert(r.p.enables==12&&r.m.state()==simple::State::MOVING);
       else assert(r.m.state()==simple::State::FAULT&&!strcmp(r.m.reason(),"SERVO_CONTROL_READ_LOST")&&r.p.enables==slot&&r.p.observed[slot].torque==0);}
      {Rig r;prone(r);r.p.dropFrameSlot=slot;r.p.dropFrames=n;assert(!r.m.stand(.05,r.p.now));prepare(r);
       if(n<3)assert(r.p.enables==12&&r.m.state()==simple::State::MOVING&&r.p.sent.size()==n+2);
       else assert(r.m.state()==simple::State::FAULT&&!strcmp(r.m.reason(),"SERVO_TORQUE_OR_GOAL_LOST")&&r.p.enables==slot&&r.p.observed[slot].torque==0);
       for(unsigned k=1;k+1<r.p.sent.size()||(n>=3&&k<r.p.sent.size());++k)assert(r.p.sent[k]==r.p.sent[0]);}
    }
    // STOP latched while the reference is re-sent: consumed first, SAFE_OFF 13/13, nothing more enabled or sent.
    {Rig r;prone(r);r.p.dropFrameSlot=7;r.p.dropFrames=1;assert(!r.m.stand(.05,r.p.now));toEnable(r,7);r.p.stopOnFrame=true;
     r.tick(false);assert(r.m.stopping()&&r.p.sent.size()==2);for(unsigned i=0;i<15;++i)r.tick(false);
     assert(r.m.offVerified()&&r.m.state()==simple::State::STOPPED&&r.p.enables==7&&r.p.sent.size()==2&&!r.m.energized());}
    // A persistent fault remains latched after STOP; an independent fresh
    // controller still recovers one transient READ on the same servo.
    {Rig r;prone(r);assert(!r.m.stand(.05,r.p.now));toEnable(r,5);r.p.missControlSlot=5;r.p.missControlReads=3;prepare(r);
     assert(r.m.state()==simple::State::FAULT);r.m.stop();for(unsigned i=0;i<15;++i)r.tick(false);assert(r.m.offVerified());
     assert(r.m.state()==simple::State::FAULT&&r.m.stand(.05,r.p.now));
     Rig fresh;prone(fresh);assert(!fresh.m.stand(.05,fresh.p.now));fresh.p.missControlSlot=5;fresh.p.missControlReads=1;prepare(fresh);
     assert(fresh.p.enables==12&&fresh.m.state()==simple::State::MOVING);assert(fresh.reachStand());}
    std::puts("PRONE 24/10 re-sent reference: byte-identical packet with 7 servos ON, no SAFE_OFF; slots 0/5/11 recover 1-2, fault on 3 before torque; STOP during re-send; counters reset per STAND");
  }
  // PRONE from above 53 mm on a tilted floor: the correction carried to the
  // PRONE_LIFT handover must stay inside kLift (2.6 mrad rear UPPER margin).
  for(double roll:{0.,.0175,-.0175})for(double pitch:{0.,-.0175}){Rig r;
    r.in.quaternion[0]=std::cos(roll/2)*std::cos(pitch/2);r.in.quaternion[1]=std::sin(roll/2)*std::cos(pitch/2);r.in.quaternion[2]=std::cos(roll/2)*std::sin(pitch/2);r.in.quaternion[3]=-std::sin(roll/2)*std::sin(pitch/2);
    assert(!r.m.stand(.10,r.p.now));assert(r.reachStand());for(unsigned i=0;i<100;++i)r.tick();
    assert(!r.m.prone(r.p.now));for(unsigned i=0;i<6000&&r.m.state()!=simple::State::PRONE;++i){r.tick();assert(r.m.state()!=simple::State::FAULT);}
    assert(r.m.state()==simple::State::PRONE);}
  std::puts("PRONE from STAND100 under +-1 deg roll/pitch: handover inside kLift, no fault");
  // Height changes from real loaded holds: follower with the HOLD residuals of
  // the robot (commanded-raw, 31/10/2026) at 50/100/150 mm, signed by servo direction.
  {static const double kE[3][12]={{-1,3,-25,1,1,17,3,0,9,1,0,-5},{-1,3,-24,1,1,19,2,11,9,3,-8,-9},{-1,-1,-9,-2,1,1,5,3,4,1,1,1}};
   static const double kS[12]={-1,1,-1,-1,-1,1,1,-1,1,1,1,-1},kL[3]={-1.0655,-.467,.3496};
   auto step=[](Rig& r,double scale){r.p.now+=simple::kPeriodUs;r.in.imuStampUs=r.p.now;r.m.inputs(r.in);
     bool on=true;for(unsigned i=0;i<12;++i)on=on&&r.p.observed[i].torque==1;
     if(!on)r.p.publishCommandedPose();
     else{const double q=(int(r.p.target[2])-2048)*simple::kTickRad;
       const unsigned k=q<=kL[1]?0:1;const double t=std::clamp((q-kL[k])/(kL[k+1]-kL[k]),0.,1.);
       for(unsigned i=0;i<12;++i){const double e=q<kL[0]-.02?0:(kE[k][i]+t*(kE[k+1][i]-kE[k][i]))*scale*kS[i];
         const int goal=std::lround(r.p.target[i]-e),raw=r.p.observed[i].raw+std::clamp(goal-int(r.p.observed[i].raw),-10,10);
         r.p.observed[i].raw=raw;r.p.observed[i].q=(raw-2048)*simple::kTickRad;const int d=int(r.p.target[i])-raw;r.p.observed[i].load=(d>0?1024:0)|std::min(900,4*std::abs(d));}}
     test_clock_us=r.p.now;r.m.update(r.p.now);r.m.updateTelemetry(r.p.now);};
   auto reach=[&](Rig& r,double h,double scale){const char* refusal=r.m.stand(h,r.p.now);if(refusal){std::printf("loaded hold height refusal=%s h=%.3f scale=%.1f phase=%s accepted=%.9f,%.9f\n",refusal,h,scale,r.m.phase(),simple::StopReferenceProbe::roll(r.m),simple::StopReferenceProbe::pitch(r.m));
       for(unsigned i=0;i<12;++i)std::printf("loaded slot=%u observed=%.9f nominal=%.9f accepted=%.9f height=[%.9f,%.9f] lift=[%.9f,%.9f]\n",i,r.p.observed[i].q,motion::jointSlot(simple::StopReferenceProbe::segmentTo(r.m),i),motion::jointSlot(r.m.target(),i),simple::collision::kHeight.lo[i],simple::collision::kHeight.hi[i],simple::collision::kLift.lo[i],simple::collision::kLift.hi[i]);std::fflush(stdout);}assert(!refusal);for(unsigned i=0;i<4000;++i){step(r,scale);assert(r.m.state()!=simple::State::FAULT);
       if(i>2&&r.m.state()==simple::State::STANDING){assert(std::abs(r.m.requestedHeightM()-h)<1e-9);for(unsigned k=0;k<15;++k)step(r,scale);return;}}assert(false);};
   for(double scale:{0.,1.,1.5}){Rig r;for(double h:{.05,.07,.11,.10,.15,.10,.05,.10,.15,.05,.15,.07,.05,.052,.05,.053,.054,.05,.073,.127})reach(r,h,scale);}
   Rig r;reach(r,.05,1);assert(!r.m.walk(1,r.p.now));for(unsigned i=0;i<1500&&r.m.state()!=simple::State::WALKING;++i)step(r,1);assert(r.m.state()==simple::State::WALKING);
   for(unsigned i=0;i<200;++i)step(r,1);r.m.walkStop();for(unsigned i=0;i<1500&&r.m.state()!=simple::State::STANDING;++i)step(r,1);
   if(r.m.state()!=simple::State::STANDING){std::printf("loaded WALK_STOP state=%s phase=%s reason=%s now=%llu deadline=%llu\n",simple::name(r.m.state()),r.m.phase(),r.m.reason(),(unsigned long long)r.p.now,(unsigned long long)r.m.deadlineUs());
     for(unsigned i=0;i<12;++i)std::printf("loaded WALK_STOP slot=%u observed_raw=%u goal_raw=%u load=%u torque=%u\n",i,r.p.observed[i].raw,r.p.target[i],r.p.observed[i].load,r.p.observed[i].torque);std::fflush(stdout);}
   assert(r.m.state()==simple::State::STANDING);
   reach(r,.05,1);assert(!r.m.prone(r.p.now));for(unsigned i=0;i<2500&&r.m.state()!=simple::State::PRONE;++i){step(r,1);assert(r.m.state()!=simple::State::FAULT);}assert(r.m.state()==simple::State::PRONE);
   reach(r,.05,1);assert(!r.m.stand(.10,r.p.now));for(unsigned i=0;i<30;++i)step(r,1);r.m.stop();for(unsigned i=0;i<150&&!r.m.stopVerified();++i)step(r,1);assert(r.m.stopVerified()&&r.m.energized());
   std::puts("STAND height changes from loaded holds: 50<->150 and intermediate heights, WALK and PRONE from low holds, STOP mid-change");}
  std::puts("PASS software sequencing, reachable heights, gait continuity, native-target timeout, STOP, alarms, communication/deadline loss; NO HARDWARE MOTION VALIDATION");
}
