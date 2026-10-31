#pragma once
#include <atomic>
#include <stdint.h>
#include "../motion/MotionContract.h"
#include "../motion/Gait.h"
#include "CollisionData.h"
#include "ViaPlan.h"

namespace matdog { namespace simple {
constexpr uint32_t kPeriodUs=20000, kReadTimeoutMs=2;
// WALK STANDARD: one gait law, three strides per cycle. Constant clock; C2
// geometric stride ramps over one cycle at start and at WALK STOP; fore/aft
// and lateral body offsets change only with four feet down.
constexpr double kWalkPeriodS=6.5, kWalkDuty=.825, kWalkLiftM=.008;
constexpr double kWalkSwayXM=.010, kWalkSwayYM=.0035, kWalkStartCycles=1, kWalkStopCycles=1;
constexpr double kStrideM[]={.030,.045,.060};
constexpr double kMinStandM=.050;
// Nominal height, the height of commands, status and route tables: the front HIP
// axes stand 46.5 mm above it. STAND 150 is that operating reference, the pose
// qualified on the robot. base_link, the ventral plane, follows from the canonical
// geometry (48.7 mm below those axes: 147.8 mm at nominal 150) and is reported
// apart. Converted only here, at the IK and FK boundary.
constexpr double kFrontHipAboveNominalM=.0465;
double baseLinkHeightFromNominalM(double nominalM);
double nominalHeightFromBaseLinkM(double baseLinkM);
// STAND reference ceilings, below the STAND RAM profile 480/13 (0.736 rad/s,
// 1.994 rad/s^2) with room for the IMU correction. rc.10: x1.57 of rc.9
// (0.35, 0.70); 0.55 rad/s is 12% of the C018 no-load speed (4.72 rad/s at
// 12 V); peak joint acceleration moves the body at ~0.1 m/s^2 (~1% g).
constexpr double kStandSpeedRadS=.55, kStandAccelerationRadS2=1.10;
constexpr double kTickRad=6.28318530717958647692/4096;
struct Drive { uint16_t speed; uint8_t acc; };
// STAND rc.10: register ceilings above the planner (above); inside the WALK
// pair already used in operation. WALK retains the STANDARD reference's ceilings.
constexpr Drive kStandDrive{480,13}, kWalkDrive{1000,50}, kCalibrationDrive{160,8};
// PRONE: held PRONE terminal, torque ON, contact unverified; not MOVING (a new
// STAND may rise from it), never an automatic SAFE_OFF.
enum class State : uint8_t { READY, MOVING, STANDING, WALKING, STOPPED, FAULT, PRONE };
const char* name(State);
struct Feedback {
  bool valid=false; uint64_t stampUs=0,controlStampUs=0;
  uint16_t raw=0; int torqueLimit=-1; double q=0; // LOADDIAG: RAM 48 of the existing control read
  int speed=0,load=0,current=0,temp=0,voltage=0,status=0,torque=-1,goal=-1,goalSpeed=-1,acc=-1;
};
// The Controller thread alone implements this port and owns UART1. Tests can
// exercise command ordering/failure handling without asserting physical motion.
class ServoPort {
 public:
  virtual ~ServoPort()=default;
  virtual bool read(unsigned slot,bool control,Feedback&)=0;
  // Query after a failed read: RX_PENDING, including a deferred second block
  // after a successful first block. No new unanswered reply is implied.
  virtual bool readDeferred() const { return false; }
  virtual bool resolve(unsigned slot,double q,uint16_t&)=0;
  virtual bool frame(const uint16_t raw[12],Drive)=0;
  virtual bool enable(unsigned slot)=0;
  virtual bool off(unsigned first=0)=0; // OFF slots first..12 (includes neck); default SAFE_OFF is global
  virtual bool verifyOff(unsigned slot)=0;  // slots 0..12, includes neck
  // One fault event, after the SAFE_OFF broadcast; no extra servo acquisition.
  virtual void referenceFault(unsigned,const Feedback&,double,uint16_t) {}
};
struct Inputs {
  bool ready=false,powerOk=false,authorityOk=false,imuValid=false;
  const char* refusal="BOOTING";
  const char* powerReason="POWER_NOT_READY";
  uint32_t bmsAgeMs=UINT32_MAX;
  uint64_t imuStampUs=0; double quaternion[4]{1,0,0,0};
  double minRad[12]{},maxRad[12]{};
};
// LOADDIAG evidence only: Motion state at the first exit (fault or STOP) of
// the latest accepted command, frozen before SAFE_OFF. Never read by Motion.
struct ExitSnapshot {
  bool valid=false,fault=false,energized=false;
  const char *reason="",*phase="IDLE"; State state=State::STOPPED;
  uint64_t atUs=0,nowUs=0,goalSentUs=0,settleUs=0,deadlineUs=0,maxGapUs=0;
  uint32_t releases=0,missed=0; unsigned lastPolled=12; Drive drive{0,0};
  uint16_t commandedRaw[12]{}; Feedback fb[12]{};
  uint16_t loadedMask=0; const char* loadedPhase="";
};
class SimpleMotion {
 public:
  void begin(ServoPort* p,bool powered=true) { port_=p; if(powered)enterOff(false,"BOOTING");else reason_="USB_ONLY"; }
  void inputs(const Inputs& i) { input_=i; }
  const char* stand(double height,uint64_t now);
  // Experimental: arrival is not proof of ventral support; torque stays ON.
  const char* prone(uint64_t now);
  const char* walk(unsigned stride,uint64_t now);
  void walkStop();
  // Atomic latch only: callable independently of HTTP and of the Motion tick.
  void stop(const char* reason="STOP_REQUESTED");
  void fault(const char* reason);
  void update(uint64_t now);
  // Read-only model estimate, 10 Hz in cooperative slack, outside Motion.
  void updateTelemetry(uint64_t now);
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
  // Telemetry only: certified region of the measured pose, never a condition.
  const char* poseRegion(uint64_t now) const;
  bool heightValid() const { return heightValid_; }
  double heightM() const { return actualHeight_; } // encoder/FK + ground-plane MODEL
  double requestedHeightM() const { return requestedHeight_; }
  double walkCycles() const { return cycles_; }
  uint32_t releases() const { return releases_; }
  uint32_t missed() const { return missed_; }
  uint64_t maxGapUs() const { return maxGapUs_; }
  bool imuValid() const { return input_.imuValid; }
  uint64_t imuStampUs() const { return input_.imuStampUs; }
  const double* quaternion() const { return input_.quaternion; }  // BNO085 w,x,y,z (normalized)
  bool stabilizationLimited() const { return imuLimited_; }
  uint64_t nextReleaseUs() const { return nextUs_; }
  const char* powerReason() const { return input_.powerReason; }
  uint32_t bmsAgeMs() const { return input_.bmsAgeMs; }
  uint32_t maxReleaseWorkUs() const { return maxReleaseWorkUs_; }
  uint64_t offVerifiedUs() const { return offVerified()?offVerifiedUs_:0; }
  unsigned referenceFaultSlot() const { return referenceFaultSlot_; }
  const Feedback& referenceFaultFeedback() const { return referenceFaultFeedback_; }
  uint16_t referenceFaultRaw() const { return referenceFaultRaw_; }
  // LOADDIAG read-only views for @MOTION DIAG (Controller thread only).
  const ExitSnapshot& exitSnapshot() const { return exit_; }
  const uint16_t* commandedRaw() const { return commandedRaw_; }
  Drive commandedDrive() const { return commandedDrive_; }
  uint64_t goalSentUs() const { return goalSentUs_; }
  uint64_t settleUs() const { return settleUs_; }
  uint64_t deadlineUs() const { return segmentBox_?segmentDeadlineUs_:goalDeadlineUs_; }
  // Last boundary completed under a static load: joint mask and phase.
  uint16_t loadedMask() const { return loadedMask_; }
  const char* loadedPhase() const { return loadedPhase_; }
  uint64_t loadedSinceUs() const { return loadedSinceUs_; }
  // Unconfirmed hot samples rejected by a later normal sample; joints whose
  // hot window is open.
  uint32_t tempSpikes() const { return tempSpikes_; }
  uint16_t hotPendingMask() const { uint16_t m=0;for(unsigned i=0;i<12;++i)if(hotSinceUs_[i])m|=1u<<i;return m; }
 private:
  enum class Phase:uint8_t { IDLE, ACQUIRE, REFERENCE, ENABLE, LOWER, ALIGN, DEPLOY, LIFT, HEIGHT, HOLD, WALK, WALK_HOLD, PRONE_HEIGHT, PRONE_LIFT, PRONE_DEPLOY, PRONE_LAND, PRONE_WAIT };
  enum class InitialRoute:uint8_t { ENTRY, HEIGHT, WALK, LIFT, DEPLOY };
  bool readTimed(unsigned slot,bool control,Feedback&);
  bool frameTimed(const uint16_t raw[12],Drive);
  bool planInitialReference();
  bool certifiedRegion(const motion::JointTargetFrame&,InitialRoute&) const;
  bool pointSafe(const motion::JointTargetFrame&) const;
  bool transitionSafe(const motion::JointTargetFrame&,const motion::JointTargetFrame&,const collision::Box&) const;
  bool send(const motion::JointTargetFrame&,Drive,uint64_t,bool holdReference=false);
  bool arrived(uint64_t) const;
  bool settledUnderLoad(uint64_t);
  bool entersHeight(const motion::JointTargetFrame& actual) const;
  bool heightTarget(double,motion::JointTargetFrame&) const;
  void measureHeight();
  void nextStand(uint64_t);
  void beginNative(Phase,const motion::JointTargetFrame&,const collision::Box&,uint64_t,double pitch=0,double x=0,double z=0);
  bool via(const motion::JointTargetFrame&,double pitch,double x,double z);
  bool knotVia(unsigned knot);
  bool heightVias(double from,double to);
  void beginPlan(Phase,const collision::Box&,uint64_t,double speed=kStandSpeedRadS,double acc=kStandAccelerationRadS2);
  void beginProne(Phase,uint64_t);
  bool proneDisengage(motion::JointTargetFrame&) const;
  bool returningProne() const { return phase_>=Phase::PRONE_HEIGHT; }
  void beginDeploy(const motion::JointTargetFrame* measured,uint64_t);
  void beginLift(const motion::JointTargetFrame* measured,uint64_t);
  void beginHeight(const motion::JointTargetFrame& from,double fromHeight,const collision::Box&,uint64_t);
  motion::JointTargetFrame measured() const;
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
  uint64_t nowUs_=0,nextUs_=0,lastReleaseUs_=0,goalSentUs_=0,goalDeadlineUs_=0;
  uint64_t maxGapUs_=0;
  uint32_t releases_=0,missed_=0;
  uint32_t maxReleaseWorkUs_=0;
  uint64_t nextTelemetryUs_=0,offVerifiedUs_=0;
  unsigned referenceFaultSlot_=12;
  Feedback referenceFaultFeedback_{};
  uint16_t referenceFaultRaw_=0;
  ExitSnapshot exit_{};
  uint64_t loadedSinceUs_=0; uint16_t loadedMask_=0,loadedRaw_[12]{}; const char* loadedPhase_="";
  uint64_t hotSinceUs_[12]{},hotLastUs_[12]{}; uint32_t tempSpikes_=0;
  unsigned pollNext_=0,controlNext_=0,acquire_=0,enable_=0;
  unsigned referenceOffRetries_=0;
  motion::JointTargetFrame goal_{},reference_{},pendingGoal_{};
  motion::JointTargetFrame segmentTo_{};
  const collision::Box* segmentBox_=nullptr;
  ViaPlan plan_{};
  bool planned_=false;
  double nativePitch_=0,nativeX_=0,nativeZ_=0,plannedHeight_=0;
  uint64_t segmentStartUs_=0,segmentDurationUs_=0,settleUs_=0,segmentDeadlineUs_=0;
  uint64_t imuStartUs_=0,compensatedUs_=0;
  bool pendingGoalValid_=false;
  uint16_t commandedRaw_[12]{};
  Drive commandedDrive_=kStandDrive;
  double requestedHeight_=.15,actualHeight_=0;
  bool heightValid_=false,walkRequested_=false;
  unsigned stride_=1;
  InitialRoute initialRoute_=InitialRoute::ENTRY;
  motion::LocomotionFrame walkPrevious_{};
  motion::LocomotionFrame holdNominal_{};
  uint64_t walkStartUs_=0;
  double cycles_=0,stopCycle_=-1;
  bool walkStopping_=false,imuLimited_=false;
  double rollCorrection_=0,pitchCorrection_=0;
};
} }
