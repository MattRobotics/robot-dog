#pragma once
#include <atomic>
#include <stdint.h>
#include "../motion/MotionContract.h"
#include "../motion/Gait.h"
#include "CollisionData.h"

namespace matdog { namespace simple {
constexpr uint32_t kPeriodUs=20000, kReadTimeoutMs=2;
constexpr double kWalkPeriodS=6, kWalkRampS=6;
constexpr double kStrideM[]={.020,.030,.040};
constexpr double kMinStandM=.050;
constexpr double kTickRad=6.28318530717958647692/4096;
struct Drive { uint16_t speed; uint8_t acc; };
// STAND retains the hardware-proven calibration acceleration, with the existing
// operational speed. WALK retains the STANDARD reference's register ceilings.
constexpr Drive kStandDrive{300,8}, kWalkDrive{1000,50}, kCalibrationDrive{160,8};
enum class State : uint8_t { READY, MOVING, STANDING, WALKING, STOPPED, FAULT };
const char* name(State);
struct Feedback {
  bool valid=false; uint64_t stampUs=0,controlStampUs=0;
  uint16_t raw=0; double q=0;
  int speed=0,load=0,current=0,temp=0,voltage=0,status=0,torque=-1,goal=-1,goalSpeed=-1,acc=-1;
};
// The Controller thread alone implements this port and owns UART1. Tests can
// exercise command ordering/failure handling without asserting physical motion.
class ServoPort {
 public:
  virtual ~ServoPort()=default;
  virtual bool read(unsigned slot,bool control,Feedback&)=0;
  virtual bool resolve(unsigned slot,double q,uint16_t&)=0;
  virtual bool frame(const uint16_t raw[12],Drive)=0;
  virtual bool enable(unsigned slot)=0;
  virtual bool off()=0;  // broadcast de-escalation, thirteen installed units
  virtual bool verifyOff(unsigned slot)=0;  // slots 0..12, includes neck
};
struct Inputs {
  bool ready=false,powerOk=false,authorityOk=false,imuValid=false;
  const char* refusal="BOOTING";
  uint64_t imuStampUs=0; double quaternion[4]{1,0,0,0};
  double minRad[12]{},maxRad[12]{};
};
class SimpleMotion {
 public:
  void begin(ServoPort* p,bool powered=true) { port_=p; if(powered)enterOff(false,"BOOTING");else reason_="USB_ONLY"; }
  void inputs(const Inputs& i) { input_=i; }
  void heartbeat(uint64_t now) { heartbeatUs_=now; }
  const char* stand(double height,uint64_t now);
  const char* walk(unsigned stride,uint64_t now);
  void walkStop();
  // Atomic latch only: callable independently of HTTP and of the Motion tick.
  void stop(const char* reason="STOP_REQUESTED");
  void fault(const char* reason);
  void update(uint64_t now);
  State state() const { return state_; }
  const char* reason() const { return reason_; }
  const char* phase() const;
  bool engaged() const { return energized_||state_==State::MOVING||offPending_; }
  bool stopping() const { return offPending_||stop_.load(); }
  bool energized() const { return energized_; }
  bool offVerified() const { return offMask_==0x1fff; }
  uint16_t offMask() const { return offMask_; }
  const Feedback* feedback() const { return fb_; }
  const motion::JointTargetFrame& target() const { return goal_; }
  bool fresh(uint64_t now) const;
  bool heightValid() const { return heightValid_; }
  double heightM() const { return actualHeight_; } // encoder/FK + ground-plane MODEL
  double requestedHeightM() const { return requestedHeight_; }
  double walkCycles() const { return cycles_; }
  uint32_t releases() const { return releases_; }
  uint32_t missed() const { return missed_; }
  uint64_t maxGapUs() const { return maxGapUs_; }
  bool imuValid() const { return input_.imuValid; }
  uint64_t imuStampUs() const { return input_.imuStampUs; }
  bool stabilizationLimited() const { return imuLimited_; }
  uint64_t nextReleaseUs() const { return nextUs_; }
 private:
  enum class Phase:uint8_t { IDLE, ACQUIRE, REFERENCE, ENABLE, LOWER, ALIGN, ROUTE, HEIGHT, HOLD, WALK, WALK_HOLD };
  enum class InitialRoute:uint8_t { ENTRY, HEIGHT, WALK, ROUTE };
  bool planInitialReference();
  bool pointSafe(const motion::JointTargetFrame&) const;
  bool transitionSafe(const motion::JointTargetFrame&,const motion::JointTargetFrame&,const collision::Box&) const;
  bool send(const motion::JointTargetFrame&,Drive,uint64_t,bool holdReference=false);
  bool arrived(uint64_t) const;
  bool heightTarget(double,motion::JointTargetFrame&) const;
  void measureHeight();
  void nextStand(uint64_t);
  void beginStandSegment(const motion::JointTargetFrame&,const collision::Box&,uint64_t,bool native=false);
  void generateStand(uint64_t);
  void generateWalk(uint64_t);
  void compensate(motion::LocomotionFrame&,uint64_t);
  void startWalk(uint64_t);
  void serviceOff();
  bool serviceStop();
  void enterOff(bool fault,const char*);
  ServoPort* port_=nullptr;
  Inputs input_{};
  Feedback fb_[12]{};
  uint8_t failures_[12]{},controlFailures_[12]{};
  std::atomic<bool> stop_{false};
  std::atomic<const char*> stopReason_{"STOP_REQUESTED"};
  State state_=State::STOPPED; Phase phase_=Phase::IDLE;
  const char* reason_="BOOTING";
  bool operatorStopped_=false;
  bool energized_=false,offPending_=false,offSent_=false,faultLatched_=false;
  uint16_t offMask_=0; unsigned offNext_=0;
  uint64_t nowUs_=0,nextUs_=0,lastReleaseUs_=0,heartbeatUs_=0,goalSentUs_=0,goalDeadlineUs_=0;
  uint64_t maxGapUs_=0;
  uint32_t releases_=0,missed_=0;
  unsigned pollNext_=0,controlNext_=0,acquire_=0,enable_=0;
  motion::JointTargetFrame goal_{},reference_{},pendingGoal_{};
  motion::JointTargetFrame segmentFrom_{},segmentTo_{};
  const collision::Box* segmentBox_=nullptr;
  uint64_t segmentStartUs_=0,segmentDurationUs_=0,settleUs_=0,segmentDeadlineUs_=0;
  uint64_t imuStartUs_=0;
  bool pendingGoalValid_=false;
  uint16_t commandedRaw_[12]{};
  Drive commandedDrive_=kStandDrive;
  double requestedHeight_=.15,actualHeight_=0;
  bool heightValid_=false,walkRequested_=false;
  unsigned stride_=1,routeCell_=0,initialCell_=0;
  InitialRoute initialRoute_=InitialRoute::ENTRY;
  motion::LocomotionFrame walkPrevious_{};
  uint64_t walkStartUs_=0;
  double cycles_=0,stopCycle_=-1,stopClock_=0;
  bool walkStopping_=false,imuLimited_=false;
  double rollCorrection_=0,pitchCorrection_=0;
};
} }
