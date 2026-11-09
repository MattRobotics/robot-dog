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
  // STOP reference history (read-only): issue instants and the built plan.
  static uint64_t issuedUs(const SimpleMotion& m){return m.issuedUs_;}
  static uint64_t previousUs(const SimpleMotion& m){return m.previousUs_;}
  static uint64_t previous2Us(const SimpleMotion& m){return m.previous2Us_;}
  static const StopPlan& stopPlan(const SimpleMotion& m){return m.stopPlan_;}
  static const uint16_t* commandedRaw(const SimpleMotion& m){return m.commandedRaw_;}
  static const uint16_t* previous2Raw(const SimpleMotion& m){return m.previous2Raw_;}
  static bool limitWalkTilt(const SimpleMotion& m,double& roll,double& pitch,double c,double s){return m.limitWalkTilt(roll,pitch,c,s);}
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
  // A bulk PresentTemperature >= 70 C on slot bulkSlot, confirmed like the
  // Controller port: direct value (-1 = no valid reply, read fails) and outcome.
  int bulkSlot=-1,bulkTemp=0,directTemp=0;uint8_t directOutcome=1;bool checked=false;
  bool tempCheck(simple::TempCheck& t) const override{if(checked)t={now,uint8_t(bulkSlot),directOutcome,int16_t(bulkTemp),int16_t(directTemp)};return checked;}
  bool read(unsigned i,bool control,simple::Feedback& out) override {
    checked=false;++reads;if((stopOnRead||(control&&stopOnControl))&&motion){motion->stop();stopOnRead=stopOnControl=false;return false;}
    if(missing)return false;
    if(control&&static_cast<int>(i)==missControlSlot&&missControlReads){--missControlReads;return false;}
    out=observed[i];out.stampUs=now;out.goal=target[i];out.goalSpeed=drive.speed;out.acc=drive.acc;
    if(int(i)==bulkSlot){checked=true;if(directTemp<0)return false;out.temp=directTemp;}
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
  // Scripted late tracking, not a model of PRONE or ground support. dev.16: the
  // handover target (kProneHandoverM) lies far inside the NEXT box, so a pose
  // outside it is now beyond the 12-tick arrival allowance as well.
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
  assert(lag<end&&(end-lag)/simple::kTickRad<=48);
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
    }else{for(unsigned n=0;n<2000&&r.m.state()!=simple::State::FAULT&&!strcmp(r.m.phase(),"PRONE_HEIGHT");++n){assert(r.m.deadlineUs()==deadline);delayed(r);}
      assert(r.p.now>deadline&&r.p.now<=deadline+simple::kPeriodUs);
      // dev.16: no torque release at an intermediate height; the ordinary contextual STOP holds.
      assert(r.m.state()!=simple::State::FAULT&&!strcmp(r.m.phase(),"STOP")&&r.m.energized()&&r.p.offCalls==off&&r.p.enables==enables);
      for(unsigned n=0;n<400&&r.m.state()!=simple::State::STOPPED&&r.m.state()!=simple::State::FAULT;++n)delayed(r);
      assert(r.m.state()==simple::State::STOPPED&&!strcmp(r.m.reason(),"PRONE_NOT_COMPLETED_SUPPORT_HELD")&&r.m.energized()&&r.p.offCalls==off);
      r.m.safeOff();for(unsigned n=0;n<30&&!r.m.offVerified();++n)r.tick(false);assert(r.m.offVerified()&&!r.m.energized());}
  }
  std::puts("PRONE handover waits inside current box, follows catch-up into next box; at the original timeout it holds (contextual STOP), no SAFE_OFF");
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
      auto done=[&]{return prone?r.m.proneRest()&&r.m.state()==simple::State::READY:r.m.state()==simple::State::STANDING;};
      for(unsigned n=0;n<2500&&!done();++n)r.tick();
      assert(done());}
    assert(r.p.enables==enables&&r.p.offCalls==off+(prone&&!stopDuringUnwind?1:0));  // the PRONE rest is one OFF broadcast
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
  Rig& r;motion::BodyPose actual{};double q[12]{};unsigned maxRawStep=0;double maxSteadyError=0;uint64_t lastUs=0;
  explicit WalkBoardPlant(Rig& rig):r(rig),lastUs(rig.p.now){assert(r.m.state()==simple::State::WALKING);for(unsigned i=0;i<12;++i)q[i]=r.p.observed[i].q;}
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
    // A late or missed release carries the law over the whole time since the
    // last frame that changed the goals: the drive bound is per elapsed time.
    const uint64_t intervalUs=std::max<uint64_t>(r.p.now-lastUs,simple::kPeriodUs);
    if(!std::equal(previous.begin(),previous.end(),r.p.target))lastUs=r.p.now;
    if(r.m.state()==simple::State::FAULT){std::printf("WALK board fault=%s phase=%s board=%.6f,%.6f accepted=%.6f,%.6f cycles=%.6f\n",r.m.reason(),r.m.phase(),boardRoll,boardPitch,simple::StopReferenceProbe::roll(r.m),simple::StopReferenceProbe::pitch(r.m),r.m.walkCycles());std::fflush(stdout);}assert(r.m.state()!=simple::State::FAULT);
    for(unsigned i=0;i<12;++i){const double target=motion::jointSlot(r.m.target(),i),decoded=(int(r.p.target[i])-2048)*simple::kTickRad;
      assert(target>=r.in.minRad[i]&&target<=r.in.maxRad[i]&&decoded>=r.in.minRad[i]&&decoded<=r.in.maxRad[i]);
      const auto& box=simple::StopReferenceProbe::box(r.m);  // the walk's own certified box
      assert(target>=box.lo[i]&&target<=box.hi[i]&&decoded>=box.lo[i]&&decoded<=box.hi[i]);
      const auto n=unsigned(std::abs(int(r.p.target[i])-int(previous[i])));maxRawStep=std::max(maxRawStep,n);assert(n<=unsigned(std::ceil(simple::kWalkDrive.speed*intervalUs/1e6))+1);}
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
    // Past the stride's tilt envelope (kWalkTilt75/kWalkTilt100) require a
    // declared bounded residual, rather than calling a non-level body fully
    // compensated.
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
    // Board pitch inside / past the stride's envelope: the correction is about
    // minus the board pitch, kWalkTilt75 +3.8/-4.7 deg, kWalkTilt100 +2.2/-1.07 deg.
    const std::vector<double> feasible=stride==1?std::vector<double>{2,-2,4}:std::vector<double>{.3,-2},bounded=stride==1?std::vector<double>{-5,6}:std::vector<double>{2,4,-4};
    for(double pitch:feasible){pitch*=degree;plant.ramp(0,0,0,pitch);plant.hold(0,pitch);plant.ramp(0,pitch,0,0);plant.hold(0,0,650);}
    for(double pitch:bounded){pitch*=degree;plant.ramp(0,0,0,pitch);plant.holdLimited(pitch);plant.ramp(0,pitch,0,0);plant.hold(0,0,650);}
    for(double sign:{1.,-1.}){plant.ramp(0,0,sign*.25*degree,0);plant.hold(sign*.25*degree,0,650);plant.ramp(sign*.25*degree,0,sign*.25*degree,-2*degree);plant.hold(sign*.25*degree,-2*degree);plant.ramp(sign*.25*degree,-2*degree,0,0);plant.hold(0,0,650);}
    plant.ramp(0,0,.25*degree,-2*degree);plant.hold(.25*degree,-2*degree);r.m.walkStop();
    for(unsigned n=0;n<3000&&r.m.state()!=simple::State::STANDING;++n)plant.step(.25*degree,-2*degree);
    assert(r.m.state()==simple::State::STANDING&&!std::strcmp(r.m.reason(),"STAND_REACHED")&&r.m.requestedHeightM()==simple::kWalkHeightM);plant.hold(.25*degree,-2*degree,650);
    assert(r.p.enables==enables&&r.p.offCalls==off);
    std::printf("WALK%u closed-loop movingboard feasible pitch %s deg, bounded %s deg, roll+/-0.25deg/combined, HOLD%.0f reached; max_RAW=%u max_steady_error=%.6fdeg\n",unsigned(simple::kStrideM[stride]*1000),stride==1?"+2/-2/+4":"+0.3/-2",stride==1?"-5/+6":"+2/+4/-4",simple::kWalkHeightM*1000,plant.maxRawStep,plant.maxSteadyError/degree);
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
  constexpr double degree=3.14159265358979323846/180,T=simple::kJoystickPeriodS,TW=simple::kWalkPeriodS;
  // lim: the steered law; pub: the published ellipsoid (fore/aft at the WALK
  // STANDARD 100 speed); back: the fore/aft backward stride (dev.5).
  const double lim[3]={simple::kJoystickStrideM/T,simple::kJoystickLateralM/T,simple::kJoystickYawRad/T};
  const double pub[3]={simple::kStrideM[2]/TW,lim[1],lim[2]},back=simple::kJoystickBackM/TW;
  using V3=std::array<double,3>;
  auto drive=[](Rig& r,V3 v){return r.m.drive(v[0],v[1],v[2],r.p.now);};
  auto walkTo=[&](Rig& r,V3 v){for(unsigned n=0;n<3000&&r.m.state()!=simple::State::WALKING;++n){if(n%12==0)drive(r,v);r.tick();assert(r.m.state()!=simple::State::FAULT);}
    assert(r.m.state()==simple::State::WALKING);};
  auto hold=[&](Rig& r,V3 v,unsigned releases){for(unsigned n=0;n<releases;++n){if(n%12==0)assert(!drive(r,v));r.tick();assert(r.m.state()==simple::State::WALKING);}};
  auto pose=[](const Rig& r){const auto& b=simple::StopReferenceProbe::walkNominal(r.m).cartesian.body;return V3{b.translationM.x,b.translationM.y,std::atan2(b.rotation[1][0],b.rotation[0][0])};};
  {Rig r;assert(!r.m.stand(.15,r.p.now));assert(r.reachStand());std::array<uint16_t,12> held;std::copy(r.p.target,r.p.target+12,held.begin());
    assert(!std::strcmp(drive(r,{.8*pub[0],.8*pub[1],0}),"JOYSTICK_VELOCITY_RANGE"));  // outside the ellipsoid
    for(unsigned j=0;j<3;++j){V3 v{0,0,0};v[j]=1.001*pub[j];assert(!std::strcmp(drive(r,v),"JOYSTICK_VELOCITY_RANGE"));v[j]=-v[j];assert(!std::strcmp(drive(r,v),"JOYSTICK_VELOCITY_RANGE"));}
    assert(!std::strcmp(drive(r,{NAN,0,0}),"JOYSTICK_VELOCITY_RANGE"));assert(!std::strcmp(drive(r,{0,0,INFINITY}),"JOYSTICK_VELOCITY_RANGE"));
    for(unsigned n=0;n<50;++n)r.tick();assert(r.m.state()==simple::State::STANDING&&!r.m.joystickActive());
    assert(std::equal(held.begin(),held.end(),r.p.target));}  // refusals move nothing
  // Sign and proportionality on all three axes: over two whole cycles the body
  // moves by exactly the SE(2) exponential of 2 x (vx, vy, yaw_rate) x period.
  // +x forward, +y left, +yaw counter-clockwise; backward takes the reversed order.
  // dev.5: fore/aft alone walks the WALK STANDARD law (period TW), the rest the steered law (T).
  const V3 cases[]={{pub[0],0,0},{pub[0]/2,0,0},{-back,0,0},{-back/3,0,0},{0,lim[1],0},{0,-lim[1]/2,0},{0,0,lim[2]},{0,0,-lim[2]/2},
                    {lim[0]/2,lim[1]/2,lim[2]/2},{-lim[0]/2,-lim[1]/2,-lim[2]/2}};
  for(const auto& v:cases){Rig r;assert(!r.m.stand(.15,r.p.now));assert(r.reachStand());walkTo(r,v);
    const bool frontal=v[1]==0&&v[2]==0;const double T=frontal?TW:simple::kJoystickPeriodS;
    assert(&simple::StopReferenceProbe::box(r.m)==(frontal?&simple::kWalkBox:&simple::collision::kSteer));
    assert(!frontal||(simple::StopReferenceProbe::gait(r.m).swayYM==simple::kWalkSwayYM&&simple::StopReferenceProbe::gait(r.m).comShiftXM==simple::kWalkComShiftXM));
    assert(simple::StopReferenceProbe::reverse(r.m)==(v[0]<0)&&!std::strcmp(r.m.reason(),"WALK_JOYSTICK")&&r.m.joystickActive());
    int first=-1;for(unsigned n=0;n<400&&first<0;++n){if(n%12==0)drive(r,v);r.tick();const auto& f=simple::StopReferenceProbe::walkNominal(r.m);
      for(unsigned l=0;l<4;++l)if(!f.cartesian.phases[l].stance)first=int(l);}
    assert(first==(v[0]<0?3:2));  // first swing: LH backward, RH otherwise
    while(r.m.walkCycles()<1.5)hold(r,v,1);
    const auto a=pose(r);hold(r,v,unsigned(std::lround(2*T/.02)));const auto b=pose(r);  // two whole cycles
    const double w=2*v[2]*T,si=w!=0?std::sin(w)/(v[2]*T):2,co=w!=0?(1-std::cos(w))/(v[2]*T):0;
    const double dx=v[0]*T*si-v[1]*T*co,dy=v[0]*T*co+v[1]*T*si;
    // The body sway (<= 18 mm, body frame) turns with the heading: |R(w)-I| x sway.
    const double tol=1e-9+std::abs(w)*.018;
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
  // Fillet re-solve accounting: the edge IK reports both solves and the sum of
  // their Newton steps and FK evaluations; its joints are the second solve's.
  {Rig r;assert(!r.m.stand(.15,r.p.now));assert(r.reachStand());walkTo(r,{0,lim[1],0});unsigned found=0;
    for(unsigned n=0;n<325&&found<4;++n){hold(r,{0,lim[1],0},1);const auto& f=simple::StopReferenceProbe::walkNominal(r.m);
      for(double roll:{-.06,.06})for(unsigned l=0;l<4;++l){const auto L=static_cast<motion::LegId>(l);
        const auto body=motion::bodyPoseFromRollPitch(f.cartesian.body.translationM,roll,0);
        motion::TiltedIkOptions o;o.seed=f.target.legs[l];o.toleranceM=1e-6;o.maxIterations=8;o.requireNominalStrip=false;
        const auto ik=motion::worldEdgeContactInverseKinematicsTilted(L,f.cartesian.feet[l],body,o);
        if(ik.status!=motion::TiltedIkResult::Status::OK||ik.base.solves!=2)continue;
        auto tilt=[&](double hip){return std::asin(std::fmin(1.,std::abs(body.rotation[2][1]*std::cos(hip)+body.rotation[2][2]*std::sin(hip))));};
        auto t=f.cartesian.feet[l];t.z+=motion::edgeContactLiftM(tilt(o.seed.hip));const auto a=motion::worldContactInverseKinematicsTilted(L,t,body,o);
        o.seed=a.base.joints;t=f.cartesian.feet[l];t.z+=motion::edgeContactLiftM(a.base.contact.axisTiltRad);const auto b=motion::worldContactInverseKinematicsTilted(L,t,body,o);
        assert(a.base.solves==1&&b.base.solves==1&&ik.base.newtonSteps==a.base.newtonSteps+b.base.newtonSteps&&ik.base.fkEvaluations==a.base.fkEvaluations+b.base.fkEvaluations);
        assert(ik.base.joints.hip==b.base.joints.hip&&ik.base.joints.upper==b.base.joints.upper&&ik.base.joints.lower==b.base.joints.lower);++found;}}
    assert(found>0);r.m.walkStop();assert(r.reachStand());}
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
  // Pre-flash (dev.17 HW: a sideways stick a hair behind centre stopped the
  // walk): a demand mainly lateral or at most 45 deg behind keeps the forward
  // walk, its contrary fore/aft held at zero; only a mostly backward one reverses.
  {Rig r;assert(!r.m.stand(.15,r.p.now));assert(r.reachStand());const V3 start{lim[0],.3*lim[1],0};walkTo(r,start);hold(r,start,300);  // steered walk
    const V3 side{-.1*lim[0],-.99*lim[1],0},diagonal{-.55*lim[0],-.6*lim[1],0},backward{-.8*lim[0],-.6*lim[1],0};
    assert(!drive(r,side)&&!std::strcmp(r.m.reason(),"WALK_JOYSTICK")&&r.m.joystickActive());
    hold(r,side,unsigned(2.5*T/.02));
    assert(simple::StopReferenceProbe::gait(r.m).advanceXM==0&&r.m.walkVelocityY()<0&&!simple::StopReferenceProbe::reverse(r.m));
    hold(r,diagonal,unsigned(T/.02));assert(simple::StopReferenceProbe::gait(r.m).advanceXM==0&&r.m.joystickActive());
    assert(!drive(r,backward));assert(!std::strcmp(r.m.reason(),"JOYSTICK_REVERSING")&&!r.m.joystickActive());
    assert(r.reachStand());}
  // Steps on every axis: each follows at its limited rate, fore/aft never crosses
  // zero inside a walk, every RAW step stays below the WALK drive per release.
  {Rig r;assert(!r.m.stand(.15,r.p.now));assert(r.reachStand());const V3 start{lim[0],.3*lim[1],0};walkTo(r,start);hold(r,start,800);  // steered walk
    double prev[3]={simple::StopReferenceProbe::stride(r.m),simple::StopReferenceProbe::gait(r.m).advanceYM,0},maxD[3]{};std::array<uint16_t,12> raw;std::copy(r.p.target,r.p.target+12,raw.begin());unsigned maxRaw=0;
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
  // dev.5 fore/aft JOYSTICK on the WALK STANDARD law. Ground HW 08/11 (dev.4):
  // a forward stick carried 0.1-0.4 mm/s of lateral (3-13 % of the range) and
  // took the 14 mm steered sway (28 mm p-p of foot excursion measured).
  using P=simple::StopReferenceProbe;
  // A. That recorded jitter, alternating sides, three cycles: the WALK STANDARD
  // law throughout (kHeight box, 1.5 mm sway, no lateral or yaw advance), no WALK STOP.
  {Rig r;assert(!r.m.stand(.15,r.p.now));assert(r.reachStand());const double jit[]={.0001,.0002,-.0003,.0004,-.0001,.0003};unsigned k=0;
    walkTo(r,{.0069,.0002,0});double maxY=0;
    for(unsigned n=0;n<unsigned(3*TW/.02);++n){if(n%12==0)assert(!drive(r,{.0069,jit[k++%6],0}));r.tick();
      const auto& g=P::gait(r.m);assert(r.m.state()==simple::State::WALKING&&&P::box(r.m)==&simple::kWalkBox&&g.swayYM==simple::kWalkSwayYM&&g.advanceYM==0&&g.yawRad==0&&!std::strcmp(r.m.reason(),"WALK_JOYSTICK"));
      maxY=std::fmax(maxY,std::abs(P::walkNominal(r.m).cartesian.body.translationM.y));}
    assert(maxY<=simple::kWalkSwayYM+1e-9&&std::abs(r.m.walkVelocityX()-.0069)<1e-12);r.m.walkStop();assert(r.reachStand());
    std::printf("JOYSTICK fore/aft with the recorded touch jitter: WALK STANDARD law, body lateral excursion %.2f mm (sway %.1f mm), no WALK STOP\n",maxY*1e3,simple::kWalkSwayYM*1e3);}
  // B. A steering demand during a fore/aft walk: WALK STOP (JOYSTICK_STEERING),
  // STANDING, then the held stick walks the steered law from its start.
  {Rig r;assert(!r.m.stand(.15,r.p.now));assert(r.reachStand());walkTo(r,{pub[0]/2,0,0});hold(r,{pub[0]/2,0,0},400);
    const V3 steer{pub[0]/2,.5*lim[1],0};assert(!drive(r,steer)&&!std::strcmp(r.m.reason(),"JOYSTICK_STEERING")&&!r.m.joystickActive());
    bool stood=false;
    for(unsigned n=0;n<4000&&!(r.m.state()==simple::State::WALKING&&&P::box(r.m)==&simple::collision::kSteer);++n){
      if(n%12==0){const char* why=drive(r,steer);assert(!why||!std::strcmp(why,"WALK_STOPPING")||!std::strcmp(why,"MOTION_BUSY"));}
      r.tick();stood|=r.m.state()==simple::State::STANDING;assert(r.m.state()!=simple::State::FAULT);}
    assert(stood&&r.m.state()==simple::State::WALKING&&&P::box(r.m)==&simple::collision::kSteer&&r.m.walkCycles()<1);r.m.walkStop();assert(r.reachStand());}
  // C. The full backward stick walks kJoystickBackM per cycle (the reversed v2 law).
  {Rig r;assert(!r.m.stand(.15,r.p.now));assert(r.reachStand());walkTo(r,{-pub[0],0,0});while(r.m.walkCycles()<1.5)hold(r,{-pub[0],0,0},1);
    assert(P::reverse(r.m)&&std::abs(r.m.walkVelocityX()+back)<1e-12&&&P::box(r.m)==&simple::kWalkBox);r.m.walkStop();assert(r.reachStand());}
  // D. Fore/aft stride changes follow kJoystickFrontalRampCycles for the full
  // 100 mm; every RAW step stays below the WALK drive per release.
  {Rig r;assert(!r.m.stand(.15,r.p.now));assert(r.reachStand());walkTo(r,{.2*pub[0],0,0});hold(r,{.2*pub[0],0,0},300);
    double prev=P::stride(r.m),maxD=0;std::array<uint16_t,12> raw;std::copy(r.p.target,r.p.target+12,raw.begin());unsigned maxRaw=0;
    const double seq[]={pub[0],.1*pub[0],.7*pub[0]};
    for(unsigned n=0;n<2100;++n){const double vx=seq[n/700];if(n%12==0)assert(!drive(r,{vx,0,0}));r.tick();assert(r.m.state()==simple::State::WALKING);
      maxD=std::fmax(maxD,std::abs(P::stride(r.m)-prev));prev=P::stride(r.m);
      for(unsigned i=0;i<12;++i){maxRaw=std::max(maxRaw,unsigned(std::abs(int(r.p.target[i])-int(raw[i]))));raw[i]=r.p.target[i];}}
    const double perRelease=simple::kStrideM[2]/simple::kJoystickFrontalRampCycles*simple::kPeriodUs/1e6/TW;
    assert(maxD<=perRelease*(1+1e-6)&&std::abs(prev-.7*pub[0]*TW)<1e-12&&maxRaw<=simple::kWalkDrive.speed*simple::kPeriodUs/1000000);
    r.m.walkStop();assert(r.reachStand());
    std::printf("JOYSTICK fore/aft stride changes: %.6f m per release (limit %.6f), max RAW step %u ticks/release\n",maxD,perRelease,maxRaw);}
  // E. STOP at five instants of a fore/aft walk: verified arrest with torque,
  // then STAND completes the interrupted WALK STANDARD gait to STANDING.
  for(unsigned when=0;when<250;when+=50){Rig r;assert(!r.m.stand(.15,r.p.now));assert(r.reachStand());const V3 v{pub[0],0,0};walkTo(r,v);hold(r,v,300+when);
    r.m.stop();for(unsigned n=0;n<150&&!r.m.stopVerified();++n){if(n%12==0)assert(drive(r,v));r.tick();}
    assert(r.m.stopVerified()&&r.m.energized()&&!r.m.joystickActive());
    assert(!r.m.stand(.12,r.p.now)&&!std::strcmp(r.m.reason(),"STAND_FROM_STOPPED_WALK"));assert(r.reachStand());}
  // F. Review 09/11 (held stick FORWARD -> DIAGONAL -> LATERAL -> FORWARD).
  // Characterizes the dev.5 contract: one law change at the first steering
  // demand (WALK STOP, JOYSTICK_STEERING), then the steered law until release;
  // back to pure fore/aft it stays steered at its own speed and its sway returns
  // to the 2 mm base (never the 14 mm of a steered demand). A law change without
  // WALK STOP is not offered by the gait (gaitRetarget: advance/yaw/swayY only).
  {Rig r;assert(!r.m.stand(.15,r.p.now));assert(r.reachStand());
    struct Seg{const char* name;V3 v;} seg[]={{"FORWARD",{pub[0],0,0}},{"DIAGONAL",{.7071*pub[0],.7071*lim[1],0}},{"LATERAL",{0,lim[1],0}},{"FORWARD",{pub[0],0,0}}};
    unsigned stops=0;double changeS=0,decayS=-1;bool walking=false;
    for(unsigned k=0;k<4;++k){bool settled=false;double t=0;
      for(unsigned n=0;n<1500;++n){if(n%12==0)drive(r,seg[k].v);r.tick();t+=.02;assert(r.m.state()!=simple::State::FAULT);
        const bool w=r.m.state()==simple::State::WALKING;if(walking&&!w){++stops;changeS=0;}if(!w)changeS+=.02;walking=w;
        if(k==3&&w&&decayS<0&&P::gait(r.m).swayYM<=simple::kJoystickSwayLowM)decayS=t;settled|=w;}
      assert(settled&&r.m.state()==simple::State::WALKING);
      const bool steered=&P::box(r.m)==&simple::collision::kSteer;
      std::printf("JOYSTICK held sequence %-8s law=%s sway=%.1f mm vx=%.2f vy=%.2f mm/s\n",seg[k].name,steered?"STEERED":"WALK_STANDARD",P::gait(r.m).swayYM*1e3,r.m.walkVelocityX()*1e3,r.m.walkVelocityY()*1e3);
      if(k==0)assert(!steered);else assert(steered);}
    assert(stops==1&&std::abs(r.m.walkVelocityX()-lim[0])<1e-12&&std::abs(P::gait(r.m).swayYM-simple::kJoystickSwayBaseYM)<1e-12&&decayS>0);
    std::printf("JOYSTICK held sequence: 1 WALK STOP (law change, %.1f s without walking), final fore/aft stays steered at %.2f mm/s, sway back under %.0f mm after %.1f s\n",changeS,lim[0]*1e3,simple::kJoystickSwayLowM*1e3,decayS);
    r.m.walkStop();assert(r.reachStand());}
  // H. Review 09/11 defect of 3afe6ce: after steering, a forward stick with the
  // recorded touch jitter kept the steered 14 mm sway (11.9-14 mm offline, the
  // dev.4 ground defect). Now the jitter is fore/aft noise in either law: the
  // lateral advance retires and the sway returns to the 2 mm base.
  {Rig r;assert(!r.m.stand(.15,r.p.now));assert(r.reachStand());const V3 diag{.7*pub[0],.7*lim[1],0};walkTo(r,diag);hold(r,diag,1000);
    const double jit[]={.0001,.0002,-.0003,.0004,-.0001,.0003};unsigned k=0;double maxSway=0;
    for(unsigned n=0;n<4000;++n){if(n%12==0)assert(!drive(r,{.95*pub[0],jit[k++%6],0}));r.tick();assert(r.m.state()==simple::State::WALKING);
      if(n>2500)maxSway=std::fmax(maxSway,P::gait(r.m).swayYM);}
    assert(&P::box(r.m)==&simple::collision::kSteer&&P::gait(r.m).advanceYM==0&&maxSway==simple::kJoystickSwayBaseYM);r.m.walkStop();assert(r.reachStand());
    std::printf("JOYSTICK forward after steering with the recorded jitter: sway %.1f mm over the last 30 s (3afe6ce: 11.9-14 mm)\n",maxSway*1e3);}
  // G. Fore/aft band sensitivity: a walk started at 19 % of lateral stays
  // WALK STANDARD; demands alternating 18/22 % change law once, never again.
  {Rig r;assert(!r.m.stand(.15,r.p.now));assert(r.reachStand());const double vx=.9*pub[0];
    walkTo(r,{vx,.19*lim[1],0});assert(&P::box(r.m)==&simple::kWalkBox);unsigned stops=0,n=0;bool w=true;
    for(;n<6000;++n){if(n%12==0)drive(r,{vx,(n/12%2?.22:.18)*lim[1],0});r.tick();const bool now=r.m.state()==simple::State::WALKING;stops+=w&&!now;w=now;}
    assert(stops==1&&w&&&P::box(r.m)==&simple::collision::kSteer);r.m.walkStop();assert(r.reachStand());
    std::printf("JOYSTICK band: 19 %% stays fore/aft; 18/22 %% alternating every 240 ms for 120 s: exactly 1 law change\n");}
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
    std::snprintf(p,sizeof p,"{\"vx\":%.12f,\"vy\":0,\"yaw_rate\":0}",simple::kStrideM[1]/simple::kWalkPeriodS);assert(!std::strcmp(send(p).reason,"ALREADY_WALKING"));
    test_clock_us=api.p.now;service.update();char status[core::kMotionStatusJsonBytes]{};assert(service.statusJson(status,sizeof(status)-640,api.p.now/1000));
    char want[256];std::snprintf(want,sizeof want,"\"walk_velocity\":{\"vx_m_s\":%.5f,\"vy_m_s\":%.5f,\"yaw_rate_rad_s\":%.5f,\"joystick\":true}",-lim[0]*.6,-lim[1]*.5,lim[2]*.5);assert(std::strstr(status,want));
    char caps[2048];assert(service.capabilitiesJson(caps,sizeof caps));
    std::snprintf(want,sizeof want,"\"joystick\":{\"vx_max_m_s\":%.5f,\"vy_max_m_s\":%.5f,\"yaw_rate_max_rad_s\":%.5f,\"envelope\":\"ELLIPSOID\",\"timeout_ms\":1000,\"ramp_cycles\":2,\"vx_back_max_m_s\":%.5f,\"frontal_band\":%.2f,\"vx_steered_max_m_s\":%.5f}",
                  pub[0],pub[1],pub[2],back,simple::kJoystickFrontalBand,lim[0]);if(!std::strstr(caps,want))std::printf("caps %s\nwant %s\n",caps,want);assert(std::strstr(caps,want));
    ack=send("{\"vx\":0,\"vy\":0,\"yaw_rate\":0,\"gait\":\"JOYSTICK\"}");assert(ack.outcome==motion::CommandOutcome::ACCEPTED&&!std::strcmp(api.m.reason(),"WALK_STOPPING"));
    assert(api.reachStand());std::printf("JOYSTICK API status %zu bytes, capabilities %zu bytes\n",std::strlen(status),std::strlen(caps));}
  std::puts("JOYSTICK PASS: vx/vy/yaw_rate sign and SE(2) proportionality, ellipsoid saturation, neutral WALK STOP, ESP deadline, rising timeout, fore/aft reversal via WALK STOP only when mostly backward, per-axis rate limits and RAW continuity, SE(2) track == closed-form law, STOP and STAND recovery, IMU board; software only, NO HARDWARE MOTION VALIDATION");
}

// H1-RCA 2026-11-08 (hardware, twice): on the fixture a steered WALK_SETTLE
// ended with +63.7/-66.5 mrad of level correction, accepted in the fillet edge
// model; the HEIGHT unwind of the joystick restart solved its steps in the
// nominal strip, every one invalid, until NATIVE_TARGET_NOT_REACHED. Plant:
// the commanded pose minus the sag of the hardware exit snapshot (exit_j, with
// its loads), servo speed in sign-magnitude, a servo whose goal moved this
// release reporting motion (the hardware WALK_SETTLE lasted until the growth
// stopped), IMU held by the fixture (open loop). Routes: (0) forward ->
// lateral -> full reversal -> restart through HEIGHT -> backward walk ->
// release -> STOP -> STAND -> PRONE; (1) reversal -> release -> PRONE. Every
// HEIGHT/PRONE_HEIGHT unwind release must retire the correction (the whole
// compensationSafe chain accepts the step), at the stand drive, inside the
// certified boxes; no fault and every state completes.
static void steeredSettleUnwind(){
  constexpr double degree=3.14159265358979323846/180,T=simple::kJoystickPeriodS;
  static const int err[12]={-2,-1,-1,0,2,3,3,-4,-1,7,5,2},load[12]={24,0,0,0,1048,1056,1056,40,0,1088,1072,1048};
  const double hp=3.9*degree/2,hr=-3.4*degree/2;
  for(unsigned route=0;route<2;++route){
    Rig r;bool plant=false;unsigned unwinds=0,settles=0;double settled=0;
    auto boxed=[](const simple::collision::Box& b,const uint16_t* raw){for(unsigned i=0;i<12;++i){const double q=(int(raw[i])-2048)*simple::kTickRad;if(q<b.lo[i]||q>b.hi[i])return false;}return true;};
    auto tick=[&]{
      r.in.quaternion[0]=std::cos(hr)*std::cos(hp);r.in.quaternion[1]=std::sin(hr)*std::cos(hp);r.in.quaternion[2]=std::cos(hr)*std::sin(hp);r.in.quaternion[3]=-std::sin(hr)*std::sin(hp);
      const bool on=plant&&r.m.energized();
      for(unsigned i=0;i<12;++i){auto& o=r.p.observed[i];const int raw=int(r.p.target[i])-(on?err[i]:0),v=raw-int(o.raw);
        o.speed=on&&v?std::max(51,std::abs(v)*50)|(v<0?0x8000:0):0;o.raw=uint16_t(raw);o.q=(raw-2048)*simple::kTickRad;o.load=on?load[i]:0;}
      const bool unwinding=r.m.state()==simple::State::MOVING&&(!std::strcmp(r.m.phase(),"HEIGHT")||!std::strcmp(r.m.phase(),"PRONE_HEIGHT"))&&
                           (simple::StopReferenceProbe::roll(r.m)!=0||simple::StopReferenceProbe::pitch(r.m)!=0);
      const double before=std::fmax(std::abs(simple::StopReferenceProbe::roll(r.m)),std::abs(simple::StopReferenceProbe::pitch(r.m)));
      std::array<uint16_t,12> previous;std::copy(r.p.target,r.p.target+12,previous.begin());
      const bool settling=!std::strcmp(r.m.phase(),"WALK_SETTLE");
      r.tick(false);assert(r.m.state()!=simple::State::FAULT);
      using namespace simple::collision;
      assert(boxed(kEntry,r.p.target)||boxed(kWalk,r.p.target)||boxed(kHeight,r.p.target)||boxed(kDeploy,r.p.target)||boxed(kLift,r.p.target)||boxed(kSteer,r.p.target));
      if(settling&&std::strcmp(r.m.phase(),"WALK_SETTLE")){++settles;settled=std::fmax(settled,before);}
      if(unwinding&&!std::strcmp(r.m.imuReason(),"WORKSPACE")){std::printf("unwind refused: phase=%s corr=%.6f,%.6f\n",r.m.phase(),simple::StopReferenceProbe::roll(r.m),simple::StopReferenceProbe::pitch(r.m));std::fflush(stdout);}
      if(unwinding&&r.m.state()==simple::State::MOVING&&!std::strcmp(r.m.imuReason(),"SLEW")){  // a retired step, never a retained one
        const double after=std::fmax(std::abs(simple::StopReferenceProbe::roll(r.m)),std::abs(simple::StopReferenceProbe::pitch(r.m)));
        assert(after<before);++unwinds;
        for(unsigned i=0;i<12;++i)assert(std::abs(int(r.p.target[i])-int(previous[i]))<=int(simple::kStandDrive.speed*simple::kPeriodUs/1000000));}
      if(unwinding)assert(std::strcmp(r.m.imuReason(),"WORKSPACE"));};
    auto until=[&](auto done,unsigned n){for(unsigned k=0;k<n&&!done();++k)tick();return done();};
    auto drive=[&](double fx,double ly){return r.m.drive(fx*simple::kJoystickStrideM/T,ly*simple::kJoystickLateralM/T,0,r.p.now);};
    auto hold=[&](double fx,double ly,double seconds){for(unsigned k=0;k<unsigned(seconds/.02);++k){if(k%12==0)assert(!drive(fx,ly));tick();}};
    assert(!r.m.stand(.12,r.p.now));assert(until([&]{return r.m.state()==simple::State::STANDING;},4000));plant=true;
    for(unsigned k=0;k<100;++k)tick();
    // dev.5: fore/aft alone would walk the WALK STANDARD law; the hardware route
    // is steered from its first step here (25 % right, above the fore/aft band).
    for(unsigned k=0;k<4000&&r.m.state()!=simple::State::WALKING;++k){if(k%12==0)drive(1,-.25);tick();}
    hold(1,-.25,1.5*T);hold(std::sqrt(.5),-std::sqrt(.5),T);hold(0,-1,T);hold(-.5,-.866,T);
    assert(!drive(-1,0)&&!std::strcmp(r.m.reason(),"JOYSTICK_REVERSING"));
    assert(until([&]{return r.m.state()==simple::State::STANDING;},6000));
    const double corr[2]={simple::StopReferenceProbe::roll(r.m),simple::StopReferenceProbe::pitch(r.m)};
    assert(corr[0]>.05&&corr[1]<-.05);  // the H1 condition: beyond the 2 deg nominal strip
    for(unsigned k=0;k<20;++k)tick();
    if(route==0){
      for(unsigned k=0;k<3000&&!(r.m.state()==simple::State::WALKING&&r.m.walkCycles()>1);++k){if(k%12==0)assert(!drive(-1,0));tick();}
      assert(r.m.state()==simple::State::WALKING&&r.m.walkVelocityX()<0);
      r.m.walkStop();assert(until([&]{return r.m.state()==simple::State::STANDING;},6000));
      r.m.stop();assert(until([&]{return r.m.stopVerified();},400));
      assert(!r.m.stand(.12,r.p.now));assert(until([&]{return r.m.state()==simple::State::STANDING;},4000));
    }else{r.m.walkStop();for(unsigned k=0;k<50;++k)tick();}
    assert(!r.m.prone(r.p.now));assert(until([&]{return r.m.proneRest();},6000));
    assert(unwinds>=40);
    std::printf("H1 steered settle unwind route %u: settle correction %+.4f/%+.4f rad (max over %u settles %.4f), %u retired unwind steps, PRONE_REST\n",
                route,corr[0],corr[1],settles,settled,unwinds);
  }
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

// PRONE rest (dev.15): the verified PRONE_LAND arrival releases the torque
// through the ordinary SAFE_OFF (never earlier, never during an interrupted
// descent); no position write after the OFF broadcast and no re-enable without
// an explicit STAND; an operator STOP during or after the rest is the usual
// latch; a STAND from the settled pose re-acquires it and re-enables through
// ACQUIRE/REFERENCE/ENABLE; outside the certified regions the existing refusal
// INITIAL_POSE_OUTSIDE_SAFE_ENTRY keeps the torque OFF. Software only.
static void proneRest(){
  auto descend=[](Rig& r,core::MotionService* service=nullptr){assert(!r.m.stand(.10,r.p.now));assert(r.reachStand());assert(!r.m.prone(r.p.now));
    const auto offs=r.p.offCalls;bool landed=false;
    for(unsigned n=0;n<8000&&!r.m.proneRest();++n){r.tick();if(service){test_clock_us=r.p.now;service->update();}
      assert(r.m.state()!=simple::State::FAULT);landed|=!std::strcmp(r.m.phase(),"PRONE_LAND");
      if(!r.m.proneRest())assert(r.p.offCalls==offs&&r.m.state()==simple::State::MOVING&&r.m.energized());}  // no OFF before the verified arrival
    assert(landed&&r.p.offCalls==offs+1&&r.m.proneRest()&&!std::strcmp(r.m.reason(),"PRONE_REST"));};
  auto ack=[](Rig& r,core::MotionService& service){test_clock_us=r.p.now;service.update();static char status[core::kMotionStatusJsonBytes];
    assert(service.statusJson(status,sizeof status,r.p.now/1000));const char* a=std::strstr(status,"\"active_command\":{");assert(a);return std::string(a,std::strcspn(a,"}"));};
  {Rig r;core::MotionService service;service.begin(&r.m,nullptr,nullptr);service.setReady(true,"READY");
    assert(!r.m.stand(.10,r.p.now));assert(r.reachStand());
    char json[200];const int n=std::snprintf(json,sizeof json,"{\"version\":1,\"id\":\"prone-1\",\"sequence\":1,\"issued_at_ms\":%lu,\"ttl_ms\":1000,\"owner\":\"dashboard\",\"type\":\"POSTURE.LIE_DOWN\",\"params\":{}}",(unsigned long)(r.p.now/1000));
    motion::MotionCommand c;const char* why=nullptr;assert(core::parseMotionJson(json,size_t(n),&c,&why));
    test_clock_us=r.p.now;{const auto a=service.command(c,r.p.now/1000);if(a.outcome!=motion::CommandOutcome::ACCEPTED)std::fprintf(stderr,"LIE_DOWN %s\n",a.reason);assert(a.outcome==motion::CommandOutcome::ACCEPTED);}
    const auto offs=r.p.offCalls;bool landed=false;
    for(unsigned k=0;k<8000&&!r.m.proneRest();++k){r.tick();test_clock_us=r.p.now;service.update();landed|=!std::strcmp(r.m.phase(),"PRONE_LAND");
      if(!r.m.proneRest())assert(r.p.offCalls==offs&&r.m.energized());}
    assert(r.p.offCalls==offs+1&&r.m.energized()&&!std::strcmp(r.m.phase(),"IDLE"));  // OFF broadcast; readback pending
    assert(landed&&r.m.proneRest()&&r.m.state()==simple::State::STOPPED);
    assert(ack(r,service).find("\"outcome\":\"ACCEPTED\"")!=std::string::npos);  // OFF readback pending: not complete, not aborted
    // Readback 13/13 with one lost OFF verification on the way: re-broadcast, then verified.
    r.p.failVerify=4;for(unsigned k=0;k<40;++k){r.tick();assert(!r.m.offVerified());}
    r.p.failVerify=-1;const auto frames=r.p.sent.size();const auto enables=r.p.enables;
    for(unsigned k=0;k<40&&!r.m.offVerified();++k)r.tick();
    assert(r.m.offVerified()&&!r.m.energized()&&r.p.offCalls>offs+1);
    for(unsigned k=0;k<5;++k)r.tick();
    const auto done=ack(r,service);
    assert(r.m.state()==simple::State::READY&&!std::strcmp(r.m.reason(),"PRONE_REST")&&done.find("\"outcome\":\"COMPLETED\",\"reason\":\"PRONE_REST\",\"id\":\"prone-1\"")!=std::string::npos);
    // Rest: no position write, no enable, torque OFF, reads continue; a neutral joystick does not take over the command.
    // (dev.17: a dashboard joystick id ends with its sequence, as the page sends it.)
    {char j[200];const int n=std::snprintf(j,sizeof j,"{\"version\":1,\"id\":\"joy-2\",\"sequence\":2,\"issued_at_ms\":%lu,\"ttl_ms\":1000,\"owner\":\"dashboard\",\"type\":\"BASE_VELOCITY\",\"params\":{\"vx\":0,\"vy\":0,\"yaw_rate\":0,\"gait\":\"JOYSTICK\"}}",(unsigned long)(r.p.now/1000));
     motion::MotionCommand c;const char* why=nullptr;assert(core::parseMotionJson(j,size_t(n),&c,&why));test_clock_us=r.p.now;assert(!std::strcmp(service.command(c,r.p.now/1000).reason,"NOT_WALKING"));
     assert(ack(r,service)==done);}
    r.p.missing=true;for(unsigned k=0;k<100;++k)r.tick();r.p.missing=false;
    for(unsigned k=0;k<400;++k){r.tick();assert(r.m.state()==simple::State::READY);}
    assert(r.p.sent.size()==frames&&r.p.enables==enables);for(unsigned i=0;i<12;++i)assert(r.p.observed[i].torque==0);
    std::printf("PRONE rest: OFF after the verified PRONE_LAND arrival, one lost readback re-broadcast, 13/13 verified, READY/PRONE_REST, ack COMPLETED, 500 releases without a frame or enable\n");}
  // Operator STOP during the rest SAFE_OFF, and after it: the ordinary latch.
  for(bool during:{true,false}){Rig r;descend(r);
    if(!during){for(unsigned k=0;k<40&&r.m.state()!=simple::State::READY;++k)r.tick();assert(r.m.state()==simple::State::READY);}
    const auto offs=r.p.offCalls,frames=r.p.frames;r.m.stop();r.m.stop();
    for(unsigned k=0;k<60;++k)r.tick();
    assert(r.m.offVerified()&&!r.m.energized()&&!r.m.proneRest()&&r.m.state()==simple::State::STOPPED&&r.p.frames==frames);
    assert(!std::strcmp(r.m.stand(.10,r.p.now),"SAFE_OFF_STAND_REFUSED"));if(during)assert(r.p.offCalls==offs);}
  // STAND from the settled pose: every joint moved by a few ticks with torque
  // OFF (inside the entry region), RAW re-acquired, REFERENCE from it, 12 enables, STANDING.
  {Rig r;descend(r);for(unsigned k=0;k<40&&r.m.state()!=simple::State::READY;++k)r.tick();
    r.p.holdObservation=true;
    // Terminal slack to the entry box, per joint: the settling the next STAND tolerates.
    double rest[12];{const auto& b=simple::collision::kEntry;for(unsigned i=0;i<12;++i)rest[i]=std::fmin(r.p.observed[i].q-b.lo[i],b.hi[i]-r.p.observed[i].q);}
    std::printf("PRONE rest terminal slack to kEntry (mrad, slot order LF RF RH LH hip/upper/lower):");for(double x:rest)std::printf(" %.1f",x*1e3);std::printf("\n");
    for(unsigned i=0;i<12;++i){auto& f=r.p.observed[i];f.raw=uint16_t(int(f.raw)+(i%2?6:-6));f.q=(int(f.raw)-2048)*simple::kTickRad;}
    for(unsigned k=0;k<20;++k)r.tick();assert(!std::strcmp(r.m.poseRegion(r.p.now),"PRONE_ENTRY"));
    double slack=1e9;{const auto& b=simple::collision::kEntry;for(unsigned i=0;i<12;++i)slack=std::fmin(slack,std::fmin(r.p.observed[i].q-b.lo[i],b.hi[i]-r.p.observed[i].q));}
    const size_t frames=r.p.sent.size();const unsigned enables=r.p.enables;std::array<uint16_t,12> settled;for(unsigned i=0;i<12;++i)settled[i]=r.p.observed[i].raw;
    assert(!r.m.stand(.10,r.p.now)&&!r.m.proneRest()&&!std::strcmp(r.m.reason(),"ACQUIRING_REAL_POSE"));
    for(unsigned k=0;k<200&&r.p.sent.size()==frames;++k)r.tick();
    assert(r.p.sent.size()==frames+1&&std::equal(settled.begin(),settled.end(),r.p.sent[frames].begin())&&r.p.sentTorqueOn[frames]==0);  // REFERENCE = settled RAW, torque OFF
    r.p.holdObservation=false;assert(r.reachStand()&&r.p.enables==enables+12);
    std::printf("PRONE rest -> STAND: settled pose (+/-6 ticks, entry-box slack %.1f mrad) re-acquired, REFERENCE = settled RAW with torque OFF, 12 enables, STANDING\n",slack*1e3);}
  // Settled outside every certified region: the existing refusal, torque never enabled.
  {Rig r;descend(r);for(unsigned k=0;k<40&&r.m.state()!=simple::State::READY;++k)r.tick();
    r.p.holdObservation=true;auto& f=r.p.observed[1];f.q+=.6;f.raw=uint16_t(2048+std::lround(f.q/simple::kTickRad));
    for(unsigned k=0;k<20;++k)r.tick();assert(!std::strcmp(r.m.poseRegion(r.p.now),"OUTSIDE_CERTIFIED"));
    const auto enables=r.p.enables,frames=r.p.frames;assert(!r.m.stand(.10,r.p.now));
    for(unsigned k=0;k<200&&!(r.m.state()==simple::State::STOPPED&&r.m.offVerified());++k)r.tick();
    // dev.5: a pre-actuation refusal, not a latched fault (entryRecovery below).
    assert(r.m.state()==simple::State::STOPPED&&!std::strcmp(r.m.reason(),"INITIAL_POSE_OUTSIDE_SAFE_ENTRY")&&r.p.enables==enables&&r.p.frames==frames&&!r.m.energized());}
  std::puts("PRONE REST PASS: automatic torque OFF only after the verified terminal, STOP latch kept, STAND from the settled pose through ACQUIRE/ENABLE; passive support NOT validated");
}

// Thermal diagnostics (dev.15): each direct confirmation of a bulk >= 70 C is
// kept with bulk, direct, servo, time and bus outcome, including a failed
// direct read; thresholds, windows and faults are unchanged.
static void temperatureChecks(){
  Rig r;assert(!r.m.stand(.10,r.p.now));assert(r.reachStand());
  r.p.bulkSlot=5;r.p.bulkTemp=94;r.p.directTemp=27;
  for(unsigned n=0;n<12&&!r.m.tempChecks();++n)r.tick();
  const uint64_t okAt=r.p.now;assert(r.m.tempChecks()==1);r.p.directTemp=-1;r.p.directOutcome=2;  // NO_RESPONSE
  for(unsigned n=0;n<12&&r.m.tempChecks()<2;++n)r.tick();
  r.p.bulkSlot=-1;for(unsigned n=0;n<60;++n)r.tick();
  const auto& a=r.m.tempCheck(0);const auto& b=r.m.tempCheck(1);
  assert(r.m.tempChecks()==2&&r.m.state()==simple::State::STANDING&&!r.m.tempSpikes()&&!r.m.hotPendingMask());
  assert(a.slot==5&&a.bulk==94&&a.direct==-1&&a.outcome==2&&b.slot==5&&b.bulk==94&&b.direct==27&&b.outcome==1&&b.stampUs==okAt&&a.stampUs>b.stampUs);
  core::MotionService service;service.begin(&r.m,nullptr,nullptr);service.setReady(true,"READY");test_clock_us=r.p.now;service.update();
  char status[core::kMotionStatusJsonBytes]{};assert(service.statusJson(status,sizeof(status)-640,r.p.now/1000));
  char expect[160];const unsigned id=actuator::slotServo(5)->bus_id;
  std::snprintf(expect,sizeof expect,"\"thermal\":{\"spikes\":0,\"hot_pending_mask\":0,\"checks\":2,\"last\":[[%u,94,-1,\"NO_RESPONSE\",%llu],[%u,94,27,\"OK\",%llu]]}",
    id,(unsigned long long)((r.p.now-a.stampUs)/1000),id,(unsigned long long)((r.p.now-okAt)/1000));
  if(!std::strstr(status,expect)){std::fprintf(stderr,"%s\n",std::strstr(status,"\"thermal\""));assert(false);}
  std::printf("THERMAL diagnostics: %s (status %zu bytes)\n",expect,std::strlen(status));
}

// dev.17: WALK after a verified STOP. The same STOP-resume admission as STAND
// (standResumeRefusal), the same resume route of the STOP origin to the WALK
// height, then WALK STANDARD with the requested stride: no ACQUIRE/ENABLE, no
// SAFE_OFF, no intermediate STAND command, no restart without a new WALK.
static void stopThenWalk(){
  auto walking=[](Rig& r,unsigned n){for(unsigned i=0;i<n&&r.m.state()!=simple::State::WALKING;++i){r.tick();
      if(r.m.state()==simple::State::FAULT){std::printf("fault=%s phase=%s\n",r.m.reason(),r.m.phase());return false;}}
    return r.m.state()==simple::State::WALKING;};
  auto stopped=[](Rig& r){r.m.stop();for(unsigned n=0;n<300&&!r.m.stopVerified();++n)r.tick();return r.m.stopVerified()&&r.m.energized();};
  // STOP during WALK45, then WALK60: the stopped gait completes with its own
  // stride (45) and envelope, HOLD at the WALK height, then WALK60.
  unsigned resumed=0;
  for(unsigned when:{60u,200u,330u}){
    Rig r;assert(!r.m.walk(1,r.p.now));assert(walking(r,3000));for(unsigned n=0;n<when;++n)r.tick();
    const auto enables=r.p.enables,off=r.p.offCalls;assert(stopped(r));
    for(unsigned n=0;n<100;++n)r.tick();
    assert(r.m.state()==simple::State::STOPPED&&r.m.stopVerified());  // nothing restarts without a command
    const char* why=r.m.standResumeRefusal(r.p.now);const char* w=r.m.walk(2,r.p.now);
    assert(!why==!w&&(!w||!std::strcmp(w,why)));  // exactly the STAND-after-STOP admission
    if(w){std::printf("WALK after STOP in WALK at %u refused like STAND: %s\n",when,w);continue;}
    assert(!std::strcmp(r.m.reason(),"STAND_FROM_STOPPED_WALK")&&!r.m.stopActive());
    bool ownStride=true;unsigned recovering=0;
    for(unsigned n=0;n<3000&&r.m.state()!=simple::State::WALKING;++n){r.tick();assert(r.m.state()!=simple::State::FAULT);
      if(r.m.state()==simple::State::MOVING&&!std::strcmp(r.m.phase(),"WALK_STANDARD")){++recovering;ownStride=ownStride&&std::abs(r.m.walkVelocityX()-simple::kStrideM[1]/simple::kWalkPeriodS)<1e-12;}}
    assert(recovering>0&&ownStride&&r.m.state()==simple::State::WALKING&&!std::strcmp(r.m.reason(),"WALK_STANDARD"));
    assert(r.m.requestedHeightM()==simple::kWalkHeightM&&std::abs(r.m.walkVelocityX()-simple::kStrideM[2]/simple::kWalkPeriodS)<1e-12);
    assert(r.p.enables==enables&&r.p.offCalls==off);
    for(unsigned n=0;n<400;++n){r.tick();assert(r.m.state()==simple::State::WALKING);}
    ++resumed;
  }
  assert(resumed>=2);
  // STOP in HOLD (STAND 150), then WALK45: HEIGHT to the WALK height, then WALK.
  {Rig r;assert(!r.m.stand(.15,r.p.now));assert(r.reachStand());for(unsigned n=0;n<50;++n)r.tick();
    const auto enables=r.p.enables,off=r.p.offCalls;assert(stopped(r));
    assert(!r.m.walk(1,r.p.now));assert(r.m.state()==simple::State::MOVING&&!std::strcmp(r.m.phase(),"HEIGHT"));
    assert(walking(r,3000)&&r.m.requestedHeightM()==simple::kWalkHeightM&&r.p.enables==enables&&r.p.offCalls==off);
    assert(std::abs(r.m.walkVelocityX()-simple::kStrideM[1]/simple::kWalkPeriodS)<1e-12);}
  // STOP while rising (HEIGHT 150 -> 60), then WALK30: same admission as STAND.
  {Rig r;assert(!r.m.stand(.15,r.p.now));assert(r.reachStand());assert(!r.m.stand(.06,r.p.now));for(unsigned n=0;n<20;++n)r.tick();
    assert(!std::strcmp(r.m.phase(),"HEIGHT"));const auto enables=r.p.enables;assert(stopped(r));
    const char* why=r.m.standResumeRefusal(r.p.now);const char* w=r.m.walk(0,r.p.now);assert(!why==!w);
    if(w)std::printf("WALK after STOP in HEIGHT refused like STAND: %s\n",w);
    else assert(walking(r,4000)&&r.p.enables==enables&&std::abs(r.m.walkVelocityX()-simple::kStrideM[0]/simple::kWalkPeriodS)<1e-12);}
  // Refusals keep their meaning: STOP in progress, bad stride, SAFE_OFF, fault.
  {Rig r;assert(!r.m.walk(1,r.p.now));assert(walking(r,3000));for(unsigned n=0;n<100;++n)r.tick();
    r.m.stop();r.tick();assert(r.m.stopActive()&&!r.m.stopVerified());
    assert(!std::strcmp(r.m.walk(1,r.p.now),"STOP_IN_PROGRESS"));
    for(unsigned n=0;n<300&&!r.m.stopVerified();++n)r.tick();assert(r.m.stopVerified());
    assert(!std::strcmp(r.m.walk(3,r.p.now),"STANDARD_STRIDE_REQUIRED")&&r.m.stopVerified());
    r.m.safeOff();assert(!std::strcmp(r.m.walk(1,r.p.now),"SAFE_OFF_IN_PROGRESS"));
    for(unsigned n=0;n<40&&!r.m.offVerified();++n)r.tick(false);assert(r.m.offVerified());
    const char* w=r.m.walk(1,r.p.now);assert(w&&r.m.offVerified());std::printf("WALK after STOP then SAFE_OFF refused: %s\n",w);}
  {Rig r;assert(!r.m.stand(.15,r.p.now));assert(r.reachStand());r.p.missing=true;
    for(unsigned i=0;i<20&&r.m.state()!=simple::State::FAULT;++i)r.tick(false);assert(r.m.state()==simple::State::FAULT);
    r.p.missing=false;for(unsigned i=0;i<40;++i)r.tick(false);
    const char* w=r.m.walk(1,r.p.now);assert(w);std::printf("WALK after a fault refused: %s\n",w);}
  std::puts("STOP->WALK: STOP in WALK/HOLD/HEIGHT then WALK STANDARD through the STAND-after-STOP admission and resume routes, stride applied at HOLD, refusals unchanged PASS (offline)");
}
// T2 (motion-perf-next): look-ahead nominal gait on alternate WALK releases.
// Alternation engages in WALK STANDARD and in steered walks, every sent
// midpoint stays within one RAW tick of the exact solution (MATDOG_T2_SHADOW),
// the T1 light compensation falls only on releases that solved the nominal,
// a WALK STOP and a missed release re-synchronise with an exact solve, STOP
// during either release kind arrests, and a release without a nominal solve
// never skips the WALK box/pointSafe/STOP proofs (a far target faults).
static void lookaheadWalk(){
  constexpr double degree=3.14159265358979323846/180;
  auto tilt=[](Rig& r,double roll,double pitch){const double cr=std::cos(roll/2),sr=std::sin(roll/2),cp=std::cos(pitch/2),sp=std::sin(pitch/2);
    r.in.quaternion[0]=cr*cp;r.in.quaternion[1]=sr*cp;r.in.quaternion[2]=cr*sp;r.in.quaternion[3]=-sr*sp;};
  for(unsigned stride:{1u,2u}){Rig r;assert(!r.m.stand(.15,r.p.now));assert(r.reachStand());assert(!r.m.walk(stride,r.p.now));
    for(unsigned n=0;n<3000&&r.m.state()!=simple::State::WALKING;++n)r.tick();tilt(r,.3*degree,.8*degree);
    const auto s0=r.m.lookaheadStats();for(unsigned n=0;n<400;++n)r.tick();const auto s1=r.m.lookaheadStats();
    // Strict alternation after the first exact release: 200/200 of 400.
    assert(s1.exact-s0.exact>=199&&s1.interp-s0.interp>=199&&s1.resync-s0.resync<=1);
    const auto t1=r.m.reuseStats();assert(t1.light>0&&t1.light<t1.full+t1.light);
    // A missed release (one period skipped) re-synchronises once, then alternates again.
    r.p.now+=simple::kPeriodUs;const auto s2=r.m.lookaheadStats();for(unsigned n=0;n<40;++n)r.tick();const auto s3=r.m.lookaheadStats();
    assert(s3.resync-s2.resync<=1&&s3.interp-s2.interp>=19);  // the bracket moves on or is rebuilt once
    // WALK STOP changes the terminal: one re-sync, then alternation to STAND.
    r.m.walkStop();const auto s4=r.m.lookaheadStats();assert(r.reachStand());const auto s5=r.m.lookaheadStats();
    assert(s5.resync-s4.resync>=1&&s5.interp-s4.interp>50&&s5.exact-s4.exact>50);
#ifdef MATDOG_T2_SHADOW
    assert(s5.maxShadowRad>0&&s5.maxShadowRad<simple::kTickRad);  // below one RAW tick on every interpolated release
#endif
  }
  // STOP on an interpolated and on an exact release: both arrest with torque.
  for(unsigned extra:{0u,1u}){Rig r;assert(!r.m.stand(.15,r.p.now));assert(r.reachStand());assert(!r.m.walk(1,r.p.now));
    for(unsigned n=0;n<3000&&r.m.state()!=simple::State::WALKING;++n)r.tick();for(unsigned n=0;n<300+extra;++n)r.tick();
    r.m.stop();for(unsigned n=0;n<200&&!r.m.stopVerified();++n)r.tick();assert(r.m.stopVerified()&&r.m.energized());}
  // Steered walk with a rate-limited retarget every 12 releases: alternation continues (retargets are tolerated).
  {Rig r;assert(!r.m.stand(.15,r.p.now));assert(r.reachStand());const double T=simple::kJoystickPeriodS;double v[3]={.5*simple::kJoystickStrideM/T,.2*simple::kJoystickLateralM/T,.1*simple::kJoystickYawRad/T};
    for(unsigned n=0;n<3000&&r.m.state()!=simple::State::WALKING;++n){if(n%12==0)r.m.drive(v[0],v[1],v[2],r.p.now);r.tick();}
    const auto s0=r.m.lookaheadStats();
    for(unsigned n=0;n<600;++n){if(n%12==0){v[1]=(n%24?.3:-.3)*simple::kJoystickLateralM/T;assert(!r.m.drive(v[0],v[1],v[2],r.p.now));}r.tick();assert(r.m.state()==simple::State::WALKING);}
    const auto s1=r.m.lookaheadStats();assert(s1.interp-s0.interp>=299&&s1.resync-s0.resync<=1);
    r.m.walkStop();assert(r.reachStand());}
  std::puts("T2 look-ahead: alternation, missed-release and WALK STOP re-sync, STOP on both release kinds, retarget tolerance PASS (software only)");
}
// P1 (review 2026-11-08): the STOP reference rate is a secant over at least
// one release period whenever the 3-deep reference history allows it. On the
// release grid (lateness < one period, skipped periods only widen the spans)
// the reference two entries back is always >= one period older, so the
// "oldest available" fallback can only act when fewer than three grid
// references exist or an off-grid reference was recorded (STAND resume after
// STOP). Direct cases: jitter 0-19 ms with a skipped release and a retarget
// every release; STOP on both release kinds; identical RAW (zero rate); stale
// history (fail closed); STOP on the first references after ENABLE; STOP at
// the first release of a STAND resume whose preceding hold release was late.
static void stopRateHistory(){
  using P=simple::StopReferenceProbe;
  struct Lcg{uint64_t s;double next(){s=s*6364136223846793005ull+1442695040888963407ull;return double(s>>11)/9007199254740992.0;}};
  auto release=[](Rig& r,uint64_t now){r.p.now=now;r.in.imuStampUs=now;r.m.inputs(r.in);r.p.publishCommandedPose();test_clock_us=now;r.m.update(now);r.m.updateTelemetry(now);};
  // The span beginStop's selection takes at the current reference (skip 1).
  auto stopSpan=[](const Rig& r){const uint64_t t[3]={P::issuedUs(r.m),P::previousUs(r.m),P::previous2Us(r.m)};uint64_t span=0;
    for(unsigned k=1;k<3;++k){if(!t[k]||t[0]<=t[k])break;span=t[0]-t[k];if(span>=simple::kPeriodUs)break;}return span;};
  auto maxPlanV=[](const Rig& r){double v=0;for(double x:P::stopPlan(r.m).v)v=std::fmax(v,std::abs(x));return v;};
  const double T=simple::kJoystickPeriodS,lim[3]={simple::kJoystickStrideM/T,simple::kJoystickLateralM/T,simple::kJoystickYawRad/T};
  // The law's joint speed stays below 0.82 rad/s (walk tilt envelope check);
  // one RAW tick of quantization over one period adds 0.077 rad/s.
  constexpr double kLawRateBound=1.0;
  // A. Jitter, skipped releases, retargets: the selected span is never below
  // one period and a STOP at a random release plans a law-plausible rate.
  {uint64_t minSpan=~0ull;unsigned spans=0;double maxV=0;
    for(uint64_t seed:{21ull,22ull,23ull,24ull,25ull,26ull}){Rig r;assert(!r.m.stand(.15,r.p.now));assert(r.reachStand());Lcg g{seed};double v[3]={lim[0]*.5,lim[1]*.5,0};  // steered from the start (dev.5: fore/aft alone walks WALK STANDARD)
      for(unsigned n=0;n<3000&&r.m.state()!=simple::State::WALKING;++n){if(n%12==0)r.m.drive(v[0],v[1],v[2],r.p.now);r.tick();}
      uint64_t grid=r.p.now;const unsigned stopAt=300+unsigned(g.next()*400);
      for(unsigned n=0;n<stopAt;++n){
        for(unsigned j=0;j<3;++j)v[j]+=lim[j]*.1*(2*g.next()-1);if(v[0]<lim[0]*.05)v[0]=lim[0]*.05;
        double norm=0;for(unsigned j=0;j<3;++j)norm+=(v[j]/lim[j])*(v[j]/lim[j]);if(norm>.9)for(unsigned j=0;j<3;++j)v[j]*=std::sqrt(.9/norm);
        r.m.drive(v[0],v[1],v[2],r.p.now);
        if(n%97==96)grid+=simple::kPeriodUs;grid+=simple::kPeriodUs;release(r,grid+uint64_t(g.next()*19000));assert(r.m.state()==simple::State::WALKING);
        if(P::previous2Us(r.m)){const uint64_t s=stopSpan(r);minSpan=std::min(minSpan,s);++spans;assert(s>=simple::kPeriodUs);}}
      r.m.stop();grid+=simple::kPeriodUs;release(r,grid+uint64_t(g.next()*19000));assert(r.m.stopActive());maxV=std::fmax(maxV,maxPlanV(r));assert(maxPlanV(r)<kLawRateBound);
      for(unsigned n=0;n<300&&!r.m.stopVerified();++n){grid+=simple::kPeriodUs;release(r,grid+uint64_t(g.next()*19000));}assert(r.m.stopVerified()&&r.m.energized());}
    std::printf("P1-A jitter/skip/retarget: %u releases, min STOP span %.1f ms (>= 20), max planned rate %.3f rad/s\n",spans,minSpan/1e3,maxV);}
  // B. Identical RAW references (HOLD): zero rate, shortest plan.
  {Rig r;assert(!r.m.stand(.15,r.p.now));assert(r.reachStand());for(unsigned n=0;n<30;++n)r.tick();
    assert(stopSpan(r)>=simple::kPeriodUs);r.m.stop();r.tick();assert(r.m.stopActive()&&maxPlanV(r)==0&&P::stopPlan(r.m).duration==.04);
    for(unsigned n=0;n<300&&!r.m.stopVerified();++n)r.tick();assert(r.m.stopVerified());}
  // C. Stale history (no release for 150 ms): fail closed, torque released.
  {Rig r;assert(!r.m.stand(.15,r.p.now));assert(r.reachStand());for(unsigned n=0;n<30;++n)r.tick();
    r.m.stop();release(r,r.p.now+150000);assert(r.m.state()==simple::State::FAULT&&!r.m.stopActive());
    assert(!std::strcmp(r.m.reason(),"MOTION_DEADLINE_LOST")||!std::strcmp(r.m.reason(),"STOP_REFERENCE_UNAVAILABLE"));
    for(unsigned n=0;n<40&&!r.m.offVerified();++n)r.tick(false);assert(r.m.offVerified()&&!r.m.energized());}
  // D. STOP on each of the first six references after ENABLE (two entries,
  // then three): either a plan with a law-plausible rate or fail closed.
  {unsigned planned=0,closed=0;
    for(unsigned k=1;k<=6;++k){Rig r;assert(!r.m.stand(.15,r.p.now));
      for(unsigned n=0;n<3000&&r.p.enables<12;++n)r.tick();assert(r.p.enables==12);  // the k-th release after the last ENABLE
      for(unsigned n=1;n<k;++n)r.tick();assert(r.m.state()==simple::State::MOVING);
      const uint64_t span=stopSpan(r);const char* phase=r.m.phase();r.m.stop();r.tick();
      if(r.m.stopActive()){++planned;assert(span>=simple::kPeriodUs&&maxPlanV(r)<kLawRateBound);for(unsigned n=0;n<300&&!r.m.stopVerified();++n)r.tick();assert(r.m.stopVerified());}
      else{++closed;assert(r.m.state()==simple::State::FAULT&&!std::strcmp(r.m.reason(),"STOP_REFERENCE_UNAVAILABLE"));  // native move: endpoint only, no in-flight reference (unchanged rule)
        if(k==1||k==6)std::printf("P1-D STOP on release %u after ENABLE in %s: %s -> SAFE_OFF\n",k,phase,r.m.reason());}}
    std::printf("P1-D STOP on the first 6 releases after ENABLE: planned=%u fail-closed=%u\n",planned,closed);}
  // E. STAND resume after STOP in WALK whose preceding hold release ran 19 ms
  // late, then STOP at the first recovery release: the only fallback case.
  {Rig r;assert(!r.m.walk(1,r.p.now));for(unsigned n=0;n<3000&&r.m.state()!=simple::State::WALKING;++n)r.tick();for(unsigned n=0;n<200;++n)r.tick();
    r.m.stop();for(unsigned n=0;n<300&&!r.m.stopVerified();++n)r.tick();assert(r.m.stopVerified());for(unsigned n=0;n<50;++n)r.tick();
    uint64_t grid=r.p.now;release(r,grid+simple::kPeriodUs+19000);  // late hold release
    const uint64_t at=grid+simple::kPeriodUs+19500;r.p.now=at;test_clock_us=at;assert(!r.m.stand(.15,at));
    release(r,grid+2*simple::kPeriodUs);  // first recovery release, 1 ms after the late one
    assert(!std::strcmp(r.m.reason(),"STAND_FROM_STOPPED_WALK")&&r.m.state()==simple::State::MOVING);
    const uint64_t span=stopSpan(r);unsigned step=0;for(unsigned i=0;i<12;++i)step=std::max(step,unsigned(std::abs(int(P::commandedRaw(r.m)[i])-int(P::previous2Raw(r.m)[i]))));
    r.m.stop();release(r,grid+3*simple::kPeriodUs);
    std::printf("P1-E resume then STOP: selected span %.1f ms, RAW step %u tick, %s, planned rate %.3f rad/s, duration %.2f s\n",span/1e3,step,
      r.m.stopActive()?"plan built":r.m.reason(),r.m.stopActive()?maxPlanV(r):0.,r.m.stopActive()?P::stopPlan(r.m).duration:0.);
    // The fallback is reached (span < one period) and is exact: every resume
    // route starts from the held RAW (recovery fade, HEIGHT from held), so the
    // secant is zero whatever its span. A non-zero step here would be the
    // first divergence that justifies a floor on the fallback span.
    assert(span<simple::kPeriodUs&&step==0&&r.m.stopActive()&&maxPlanV(r)==0);
    for(unsigned n=0;n<300&&!r.m.stopVerified();++n)r.tick();assert(r.m.stopVerified());}
  std::puts("P1 STOP rate history: span >= one period on the grid, zero rate on identical RAW, fail-closed on stale history, first references, STAND resume fallback PASS (software only)");
}
// Steered sway map (motion-perf-next) inside a steered walk: from the full
// 14 mm, 1 % takes the low band (<= 5 mm), 2.9 % stays there, 3.3 % takes the
// full amplitude again and keeps it until the demand falls under 1.5 %; the
// amplitude changes only while all four feet are down. dev.5: a NEW walk under
// kJoystickFrontalBand (ground HW: 3-13 % of touch jitter) is fore/aft and
// walks the WALK STANDARD law with its 1.5 mm sway; at the band it is steered.
static void swayMap(){
  using P=simple::StopReferenceProbe;const double T=simple::kJoystickPeriodS,vx=.8*simple::kJoystickStrideM/T,lat=simple::kJoystickLateralM/T;
  auto walking=[](Rig& r,double vx,double vy){for(unsigned n=0;n<3000&&r.m.state()!=simple::State::WALKING;++n){if(n%12==0)assert(!r.m.drive(vx,vy,0,r.p.now));r.tick();}assert(r.m.state()==simple::State::WALKING);};
  auto hold=[&](Rig& r,double vy,unsigned releases){for(unsigned n=0;n<releases;++n){if(n%12==0)assert(!r.m.drive(vx,vy*lat,0,r.p.now));r.tick();assert(r.m.state()==simple::State::WALKING);}};
  // The map itself (the steered retarget feeds it its rate-limited lateral/yaw).
  {using simple::joystickSwayYM;const double L=simple::kJoystickLateralM,B=simple::kJoystickSwayBaseYM,F=simple::kJoystickSwayYM,Lo=simple::kJoystickSwayLowM;
    assert(joystickSwayYM(0,0,B)==B&&joystickSwayYM(.029*L,0,B)>B&&joystickSwayYM(.029*L,0,B)<=Lo+1e-12&&joystickSwayYM(.033*L,0,B)==F);
    assert(joystickSwayYM(.025*L,0,F)==F&&joystickSwayYM(.01*L,0,F)<Lo);}
  // In a steered walk (review 09/11): a demand under kJoystickFrontalBand is
  // fore/aft, its lateral retired: the sway returns to the base through
  // four-contact windows only; at the band the full amplitude comes back.
  {Rig r;walking(r,vx,.5*lat);assert(&P::box(r.m)==&simple::collision::kSteer);hold(r,.5,1300);assert(std::abs(P::gait(r.m).swayYM-simple::kJoystickSwayYM)<1e-12);
    double last=P::gait(r.m).swayYM;unsigned changes=0,changesWithSwing=0;
    for(unsigned n=0;n<2600;++n){if(n%12==0)assert(!r.m.drive(vx,.19*lat,0,r.p.now));
      bool four=true;for(const auto& ph:P::walkNominal(r.m).cartesian.phases)four=four&&ph.stance;r.tick();assert(r.m.state()==simple::State::WALKING);
      const double s=P::gait(r.m).swayYM;if(s!=last){++changes;if(!four)++changesWithSwing;last=s;}}
    assert(changes>0&&changesWithSwing==0&&P::gait(r.m).advanceYM==0&&std::abs(last-simple::kJoystickSwayBaseYM)<1e-12&&&P::box(r.m)==&simple::collision::kSteer);
    hold(r,.21,1300);assert(std::abs(P::gait(r.m).swayYM-simple::kJoystickSwayYM)<1e-12);
    r.m.walkStop();assert(r.reachStand());}
  // A new walk: just under the band it is fore/aft, WALK STANDARD sway and box,
  // its lateral demand ignored; at the band it is steered with the full sway.
  const double band=simple::kJoystickFrontalBand;
  {Rig r;walking(r,vx,.999*band*lat);assert(&P::box(r.m)==&simple::kWalkBox&&P::gait(r.m).swayYM==simple::kWalkSwayYM&&P::gait(r.m).advanceYM==0);
    for(unsigned n=0;n<1300;++n){if(n%12==0)assert(!r.m.drive(vx,.999*band*lat,0,r.p.now));r.tick();assert(r.m.state()==simple::State::WALKING&&P::gait(r.m).swayYM==simple::kWalkSwayYM);}
    assert(std::abs(r.m.walkVelocityX()-vx)<1e-12&&r.m.walkVelocityY()==0);r.m.walkStop();assert(r.reachStand());}
  {Rig r;walking(r,vx,band*lat);assert(&P::box(r.m)==&simple::collision::kSteer&&std::abs(P::gait(r.m).swayYM-simple::kJoystickSwayYM)<1e-12);r.m.walkStop();assert(r.reachStand());}
  std::puts("steered sway map: low band <= 5 mm up to 2.9 %, 14 mm from 3.3 % with release under 1.5 %; in a steered walk a demand under the fore/aft band returns to the base sway through four-contact windows only; new walk under the band = WALK STANDARD sway PASS (software only)");
}
// Steered tilt envelope in the heading frame (review 2026-11-08): with
// identity heading the limit is bit for bit the old one; at a 90 deg heading
// a support-frame pitch at the polygon's pitch-up vertex is a body roll and is
// limited to the roll vertex, taken back into the support frame.
static void steeredEnvelopeHeading(){
  Rig r;const double T=simple::kJoystickPeriodS,vy=.5*simple::kJoystickLateralM/T;assert(!r.m.drive(.5*simple::kJoystickStrideM/T,vy,0,r.p.now));  // steered (dev.5)
  for(unsigned n=0;n<3000&&r.m.state()!=simple::State::WALKING;++n){if(n%12==0)r.m.drive(.5*simple::kJoystickStrideM/T,vy,0,r.p.now);r.tick();}assert(r.m.state()==simple::State::WALKING);
  double roll=.0,pitch=.05;assert(!simple::StopReferenceProbe::limitWalkTilt(r.m,roll,pitch,1,0)&&roll==0&&pitch==.05);           // inside, identity heading, untouched
  roll=.0;pitch=.10;assert(simple::StopReferenceProbe::limitWalkTilt(r.m,roll,pitch,1,0)&&roll==0&&std::abs(pitch-.07)<1e-12);     // pitch-up vertex, identity heading
  roll=.0;pitch=.05;assert(simple::StopReferenceProbe::limitWalkTilt(r.m,roll,pitch,0,1)&&std::abs(roll)<1e-12&&std::abs(pitch-.0245)<1e-12);  // heading 90 deg: a body roll, roll vertex
  roll=.0;pitch=.02;assert(!simple::StopReferenceProbe::limitWalkTilt(r.m,roll,pitch,0,1)&&roll==0&&pitch==.02);                    // inside at 90 deg too
  r.m.walkStop();assert(r.reachStand());
  std::puts("steered tilt envelope: heading-frame projection, identity bit for bit, 90 deg heading maps pitch to the roll vertex PASS (software only)");
}
// dev.5 entry recovery (ground HW 08/11/2026, dev.4): a STAND150 refused with
// INITIAL_POSE_OUTSIDE_SAFE_ENTRY latched a FAULT that STOP could not clear;
// the robot was repositioned by hand and only KEY OFF/ON made STAND possible,
// from a pose identical (+/-1 tick) to the repositioned one. Here the refusal
// is a pre-actuation STOPPED state: torque stays OFF, nothing is written, and
// READY returns by itself only when the same ACQUIRE evidence (12 fresh
// positions in a certified region, at rest, no alarm, torque OFF read again on
// every servo) holds for one full control-read cycle. The aborted STAND never
// runs again; a new explicit STAND re-acquires the pose. Software only.
static void entryRecovery(){
  // Boot-1 repositioned pose, trace matdog-motion-trace.json @116.6 s (rad,
  // slot order LF RF RH LH x hip/upper/lower): PRONE_ENTRY, accepted after reboot.
  const double placed[12]={-0.049087,1.417398,-1.59534,0.01534,1.415864,-1.583068,0.02301,1.411262,-1.596874,-0.029146,1.425068,-1.59534};
  // Rear raised (operator note): both rear UPPER 60 mrad under the entry box.
  double raised[12];std::copy(placed,placed+12,raised);
  for(unsigned s:{7u,10u})raised[s]=simple::collision::kEntry.lo[s]-.060;
  auto pose=[](Rig& r,const double* q){for(unsigned i=0;i<12;++i){auto& f=r.p.observed[i];f.q=q[i];f.raw=uint16_t(2048+std::lround(q[i]/simple::kTickRad));f.speed=0;}};
  auto ack=[](Rig& r,core::MotionService& service){test_clock_us=r.p.now;service.update();static char status[core::kMotionStatusJsonBytes];
    assert(service.statusJson(status,sizeof status,r.p.now/1000));const char* a=std::strstr(status,"\"active_command\":{");assert(a);return std::string(a,std::strcspn(a,"}"));};
  auto refused=[&](Rig& r,core::MotionService* service=nullptr){
    r.p.holdObservation=true;pose(r,raised);for(unsigned k=0;k<20;++k)r.tick();
    assert(!std::strcmp(r.m.poseRegion(r.p.now),"OUTSIDE_CERTIFIED")&&r.m.state()==simple::State::READY);
    if(service){motion::MotionCommand c;c.owner=motion::ClientOwner::DASHBOARD;c.intent=motion::Intent::STAND;c.target_height_m=.15;c.sequence=1;
      c.issued_at_ms=r.p.now/1000;c.ttl_ms=1000;std::strcpy(c.id,"entry-1");test_clock_us=r.p.now;assert(service->command(c,r.p.now/1000).outcome==motion::CommandOutcome::ACCEPTED);}
    else assert(!r.m.stand(.15,r.p.now));
    for(unsigned k=0;k<200&&!(r.m.state()==simple::State::STOPPED&&r.m.offVerified());++k){r.tick();if(service){test_clock_us=r.p.now;service->update();}}
    assert(r.m.state()==simple::State::STOPPED&&!std::strcmp(r.m.reason(),"INITIAL_POSE_OUTSIDE_SAFE_ENTRY")&&r.m.offVerified()&&!r.m.energized());
    assert(r.p.frames==0&&r.p.enables==0);for(const auto& f:r.p.observed)assert(f.torque==0);};
  // Releases until READY (bounded); 0 if it never came.
  auto readyAfter=[](Rig& r,unsigned n){for(unsigned k=1;k<=n;++k){r.tick();assert(r.p.frames==0&&r.p.enables==0);if(r.m.state()==simple::State::READY)return k;}return 0u;};
  constexpr unsigned cycle=72;  // one control read per 6 releases, 12 servos
  // 1. The field sequence: refusal, the pose stays outside, STOP pressed, repositioned.
  {Rig r;core::MotionService service;service.begin(&r.m,nullptr,nullptr);service.setReady(true,"READY");
    refused(r,&service);
    const auto a=ack(r,service);assert(a.find("\"outcome\":\"ABORTED\",\"reason\":\"INITIAL_POSE_OUTSIDE_SAFE_ENTRY\",\"id\":\"entry-1\"")!=std::string::npos);
    // dev.5 diagnostics: the status names the joint farthest outside the entry region.
    auto status=[&]{test_clock_us=r.p.now;service.update();static char s[core::kMotionStatusJsonBytes];assert(service.statusJson(s,sizeof s-640,r.p.now/1000));return std::string(s);};  // inside the bounded budget
    // (SAFE_OFF verification pauses the fast reads: one population of fresh reads first)
    for(unsigned k=0;k<12;++k)r.tick();
    assert(status().find("\"entry_violation\":{\"joint\":\"RH_UPPER\",\"outside_mrad\":-60}")!=std::string::npos);
    for(unsigned k=0;k<500;++k){r.tick();assert(r.m.state()==simple::State::STOPPED);}
    assert(!std::strcmp(r.m.stand(.15,r.p.now),"INITIAL_POSE_OUTSIDE_SAFE_ENTRY")&&r.m.state()==simple::State::STOPPED);
    assert(!std::strcmp(r.m.drive(.005,0,0,r.p.now),"INITIAL_POSE_OUTSIDE_SAFE_ENTRY")&&!std::strcmp(r.m.walk(2,r.p.now),"INITIAL_POSE_OUTSIDE_SAFE_ENTRY"));
    const auto offs=r.p.offCalls;r.m.stop();for(unsigned k=0;k<40;++k)r.tick();  // STOP only re-verifies the OFF
    assert(r.p.offCalls>offs&&r.m.offVerified()&&r.m.state()==simple::State::STOPPED&&!std::strcmp(r.m.reason(),"INITIAL_POSE_OUTSIDE_SAFE_ENTRY"));
    pose(r,placed);const unsigned k=readyAfter(r,3*cycle);
    assert(k>=cycle&&k<=2*cycle+20&&!std::strcmp(r.m.reason(),"READY")&&!std::strcmp(r.m.poseRegion(r.p.now),"PRONE_ENTRY"));
    for(unsigned n=0;n<300;++n){r.tick();assert(r.m.state()==simple::State::READY&&r.p.frames==0&&r.p.enables==0);}  // the aborted STAND never runs
    assert(ack(r,service)==a&&status().find("entry_violation")==std::string::npos);
    // A NEW explicit STAND re-acquires the placed pose and stands.
    assert(!r.m.stand(.15,r.p.now)&&!std::strcmp(r.m.reason(),"ACQUIRING_REAL_POSE"));
    for(unsigned n=0;n<200&&r.p.frames==0;++n)r.tick();
    assert(r.p.frames==1&&r.p.sentTorqueOn[0]==0);for(unsigned i=0;i<12;++i)assert(r.p.sent[0][i]==r.p.observed[i].raw);  // REFERENCE = placed RAW, torque OFF
    r.p.holdObservation=false;assert(r.reachStand()&&r.p.enables==12);
    std::printf("entry recovery: refusal -> STOPPED (no frame, no enable, torque OFF), ACK ABORTED, STOP keeps it, repositioned HW pose READY after %u releases, aborted STAND not run, new STAND reaches STANDING\n",k);}
  // 2. No STOP, refusal at REFERENCE (the pose moves after ACQUIRE): same state.
  {Rig r;r.p.holdObservation=true;pose(r,placed);for(unsigned k=0;k<20;++k)r.tick();
    assert(!r.m.stand(.15,r.p.now));for(unsigned n=0;n<40&&std::strcmp(r.m.phase(),"REFERENCE");++n)r.tick();
    assert(!std::strcmp(r.m.phase(),"REFERENCE")&&r.p.frames==0);pose(r,raised);
    for(unsigned k=0;k<200&&!(r.m.state()==simple::State::STOPPED&&r.m.offVerified());++k)r.tick();
    assert(r.m.state()==simple::State::STOPPED&&!std::strcmp(r.m.reason(),"INITIAL_POSE_OUTSIDE_SAFE_ENTRY")&&r.p.frames==0&&r.p.enables==0);
    pose(r,placed);assert(readyAfter(r,3*cycle));}
  // 3. Negative cases: each blocks READY while it lasts; READY follows once it ends.
  struct Case {const char* name;void (*on)(ScriptedObservationPort&);void (*off)(ScriptedObservationPort&);};
  const Case cases[]={
    {"feedback lost",[](ScriptedObservationPort& p){p.missing=true;},[](ScriptedObservationPort& p){p.missing=false;}},
    {"one joint 2 ticks outside kEntry",[](ScriptedObservationPort& p){auto& f=p.observed[11];f.q=simple::collision::kEntry.lo[11]-2*simple::kTickRad;f.raw=uint16_t(2048+std::lround(f.q/simple::kTickRad));},
      [](ScriptedObservationPort& p){auto& f=p.observed[11];f.q=-1.59534;f.raw=uint16_t(2048+std::lround(f.q/simple::kTickRad));}},
    {"servo still moving",[](ScriptedObservationPort& p){p.observed[4].speed=60;},[](ScriptedObservationPort& p){p.observed[4].speed=0;}},
    {"moving backwards (sign bit)",[](ScriptedObservationPort& p){p.observed[4].speed=0x8000|51;},[](ScriptedObservationPort& p){p.observed[4].speed=0x8000;}},
    {"unexpected torque",[](ScriptedObservationPort& p){p.observed[8].torque=1;},[](ScriptedObservationPort& p){p.observed[8].torque=0;}},
    {"servo status alarm",[](ScriptedObservationPort& p){p.observed[2].status=8;},[](ScriptedObservationPort& p){p.observed[2].status=0;}},
    {"servo 70 C",[](ScriptedObservationPort& p){p.observed[9].temp=70;},[](ScriptedObservationPort& p){p.observed[9].temp=30;}},
    {"voltage 3.9 V",[](ScriptedObservationPort& p){p.observed[0].voltage=39;},[](ScriptedObservationPort& p){p.observed[0].voltage=120;}}};
  for(const auto& c:cases){Rig r;refused(r);pose(r,placed);c.on(r.p);
    for(unsigned k=0;k<4*cycle;++k){r.tick();assert(r.m.state()==simple::State::STOPPED&&r.p.frames==0&&r.p.enables==0);}
    c.off(r.p);const unsigned k=readyAfter(r,3*cycle);
    if(!k)std::printf("entry case %s: no READY after the condition ended\n",c.name);assert(k>=cycle/2);  // READY needs fresh evidence again
    std::printf("entry recovery negative '%s': STOPPED for %u releases, READY %u releases after it ended\n",c.name,4*cycle,k);}
  // 4. A pose crossing an uncertified region while being placed: no READY on
  // the way, and a brief pass through PRONE_ENTRY shorter than one control
  // cycle is not enough; the final placement then needs its own full cycle.
  {Rig r;refused(r);
    double mid[12];for(unsigned i=0;i<12;++i)mid[i]=.5*(placed[i]+raised[i]);mid[1]=1.0;  // LF_UPPER far outside every box
    pose(r,mid);for(unsigned k=0;k<cycle;++k){r.tick();assert(r.m.state()==simple::State::STOPPED);}
    pose(r,placed);for(unsigned k=0;k<cycle/2;++k){r.tick();assert(r.m.state()==simple::State::STOPPED);}
    pose(r,raised);for(unsigned k=0;k<cycle;++k){r.tick();assert(r.m.state()==simple::State::STOPPED);}
    pose(r,placed);const unsigned k=readyAfter(r,3*cycle);assert(k>=cycle);
    std::printf("entry recovery: crossing an uncertified region and a %u-release pass through PRONE_ENTRY never READY; final placement READY after %u releases\n",cycle/2,k);}
  // 5. A different physical fault stays latched; a stale ACQUIRE stays a fault.
  {Rig r;refused(r);r.m.fault("SERVO_COMMUNICATION_LOST");pose(r,placed);
    for(unsigned k=0;k<4*cycle;++k){r.tick();assert(r.m.state()==simple::State::FAULT&&!std::strcmp(r.m.reason(),"SERVO_COMMUNICATION_LOST"));}
    assert(!std::strcmp(r.m.stand(.15,r.p.now),"FAULT_STOP_REQUIRED"));}
  {Rig r;r.p.holdObservation=true;pose(r,placed);r.p.observed[3].torque=1;for(unsigned k=0;k<20;++k)r.tick();assert(!r.m.stand(.15,r.p.now));
    for(unsigned k=0;k<60&&r.m.state()!=simple::State::FAULT;++k)r.tick();
    assert(r.m.state()==simple::State::FAULT&&!std::strcmp(r.m.reason(),"INITIAL_SERVO_NOT_READY"));r.p.observed[3].torque=0;
    for(unsigned k=0;k<4*cycle;++k){r.tick();assert(r.m.state()==simple::State::FAULT);}}
  // 6. SAFE_OFF requested in the refused state behaves like STOP: no operator latch.
  {Rig r;refused(r);r.m.safeOff();for(unsigned k=0;k<40;++k)r.tick();
    assert(r.m.state()==simple::State::STOPPED&&!std::strcmp(r.m.reason(),"INITIAL_POSE_OUTSIDE_SAFE_ENTRY"));pose(r,placed);assert(readyAfter(r,3*cycle));}
  // 7. Unchanged: STOP after a re-admitted READY (or at boot READY) is the ordinary SAFE_OFF latch.
  {Rig r;refused(r);pose(r,placed);assert(readyAfter(r,3*cycle));r.m.stop();for(unsigned k=0;k<60;++k)r.tick();
    assert(r.m.state()==simple::State::STOPPED&&!std::strcmp(r.m.stand(.15,r.p.now),"SAFE_OFF_STAND_REFUSED"));}
  // 8. dev.5 diagnostics: a level correction beyond the HOLD box saturates
  // (WORKSPACE) and the status names the compensationSafe check that refused.
  {Rig r;core::MotionService service;service.begin(&r.m,nullptr,nullptr);service.setReady(true,"READY");assert(!r.m.stand(.15,r.p.now));assert(r.reachStand());
    const double roll=.20;r.in.quaternion[0]=std::cos(roll/2);r.in.quaternion[1]=std::sin(roll/2);r.in.quaternion[2]=r.in.quaternion[3]=0;
    for(unsigned k=0;k<400&&std::strcmp(r.m.imuReason(),"WORKSPACE");++k)r.tick();
    assert(!std::strcmp(r.m.imuReason(),"WORKSPACE")&&r.m.imuRefusedBy()>0&&r.m.imuRefusedBy()<std::size(simple::kSafeChecks));
    test_clock_us=r.p.now;service.update();static char s[core::kMotionStatusJsonBytes];assert(service.statusJson(s,sizeof s-640,r.p.now/1000));  // inside the bounded budget
    std::printf("refusal diagnostics: status %zu bytes (bounded budget %zu)\n",std::strlen(s),sizeof s-640);
    const std::string want=std::string("\"refused_by\":\"")+simple::kSafeChecks[r.m.imuRefusedBy()]+"\"";assert(std::strstr(s,want.c_str()));
    std::printf("refusal diagnostics: HOLD 150 under 0.2 rad of board roll, WORKSPACE refused_by %s\n",simple::kSafeChecks[r.m.imuRefusedBy()]);}
  std::puts("ENTRY RECOVERY PASS: pre-actuation refusal recoverable without STOP/reset/KEY OFF, only on legitimate ACQUIRE evidence; other faults and the SAFE_OFF latch unchanged");
}
// dev.6 consolidation: WALK STANDARD 100 and the fore/aft joystick at 100 mm
// faulted WALK_IK_OR_WORKSPACE (refused_by STOP_PLAN) at release 4993, cycle
// 19.46, of a long walk on this rig (dev.4..dev.6): the T2 chord-midpoint
// reference at the swing end made the STOP secant one tick faster than the
// old kWalkTilt100 had been proved for (the level correction on its
// pitch-down boundary, as on the dev.4 ground WALK100). 160 s of each walk.
static void longWalk100(){
  const double vx=simple::kStrideM[2]/simple::kWalkPeriodS;
  for(int mode=0;mode<3;++mode){Rig r;assert(!r.m.stand(.15,r.p.now));assert(r.reachStand());
    if(mode)assert(!r.m.walk(mode==1?2:1,r.p.now));
    for(unsigned n=0;n<8000;++n){if(!mode&&n%12==0)assert(!r.m.drive(vx,0,0,r.p.now));r.tick();assert(r.m.state()!=simple::State::FAULT);}
    assert(r.m.state()==simple::State::WALKING&&r.m.walkCycles()>31);r.m.walkStop();assert(r.reachStand());}
  std::puts("long walk: JOYSTICK 100 mm, WALK STANDARD 100 and 75, 8000 releases (160 s) each without fault PASS (software only)");
}
int main(){longWalk100();proneRest();entryRecovery();temperatureChecks();standHoldTilt();lookaheadWalk();stopRateHistory();swayMap();steeredEnvelopeHeading();
  stopThenWalk();
  proneStopRecovery();
  imuSlewTime();
  joystickWalk();
  steeredSettleUnwind();
  walkLevelBoard();
  walkTxLatency();
  proneHandoverWait();
  // Exercise the production gait + IK across every join: swing lift-off and
  // touchdown, four-contact sway windows, end of the start ramp, decel start,
  // terminal touchdown and recentring end, for each stride and quarter stop.
  // Velocity is continuous everywhere (C1); acceleration too (C2) except the
  // swinging foot's own lift-off/touchdown with the trapezoidal swing, whose
  // velocity ramps start and end there by design (Gait.h swingRamp).
  for(unsigned stride=0;stride<3;++stride)for(double stop : {1.,1.25,1.5,1.75,2.}){
    const auto g=simple::walkGait(stride);
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
        const bool corner=g.swingRamp>0&&a.cartesian.phases[l].stance!=b.cartesian.phases[l].stance;
        assert(corner||(std::abs(aa.hip-ba.hip)<.002&&std::abs(aa.upper-ba.upper)<.002&&std::abs(aa.lower-ba.lower)<.002));
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
  Rig unsafe;unsafe.p.observed[0].q=.7;assert(!unsafe.m.stand(.15,unsafe.p.now));for(unsigned i=0;i<30&&unsafe.m.state()==simple::State::MOVING;++i)unsafe.tick(false);
  assert(unsafe.m.state()==simple::State::STOPPED&&!std::strcmp(unsafe.m.reason(),"INITIAL_POSE_OUTSIDE_SAFE_ENTRY")&&unsafe.p.enables==0&&unsafe.p.frames==0);
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
    assert(!r.m.prone(r.p.now));for(unsigned i=0;i<6000&&!r.m.proneRest();++i){r.tick();assert(r.m.state()!=simple::State::FAULT);}
    assert(r.m.proneRest());}
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
   reach(r,.05,1);assert(!r.m.prone(r.p.now));for(unsigned i=0;i<2600&&!(r.m.proneRest()&&r.m.state()==simple::State::READY);++i){step(r,1);assert(r.m.state()!=simple::State::FAULT);}assert(r.m.proneRest()&&r.m.state()==simple::State::READY);
   reach(r,.05,1);assert(!r.m.stand(.10,r.p.now));for(unsigned i=0;i<30;++i)step(r,1);r.m.stop();for(unsigned i=0;i<150&&!r.m.stopVerified();++i)step(r,1);assert(r.m.stopVerified()&&r.m.energized());
   std::puts("STAND height changes from loaded holds: 50<->150 and intermediate heights, WALK and PRONE from low holds, STOP mid-change");}
  std::puts("PASS software sequencing, reachable heights, gait continuity, native-target timeout, STOP, alarms, communication/deadline loss; NO HARDWARE MOTION VALIDATION");
}
