#pragma once
#include <atomic>
#include <stdint.h>
#include "../motion/MotionContract.h"
#include "../motion/Gait.h"
#include "CollisionData.h"
#include "SteerCollisionData.h"
#include "ViaPlan.h"
#include "StopPlan.h"

namespace matdog { namespace simple {
constexpr uint32_t kPeriodUs=20000, kReadTimeoutMs=2;
// WALK STANDARD: one gait law, three strides per cycle. Constant clock; C2
// geometric stride ramps over one cycle at start and at WALK STOP; fore/aft
// and lateral body offsets change only with four feet down.
constexpr double kWalkPeriodS=6.5, kWalkDuty=.825, kWalkLiftM=.008;
// Lower WALK posture stays in the existing HEIGHT joint box. The 2 mm
// lateral sway leaves room for STOP and the recorded loaded HIP residuals.
constexpr double kWalkHeightM=.120, kWalkSwayXM=.010, kWalkSwayYM=.002, kWalkStartCycles=1, kWalkStopCycles=1;
static constexpr const collision::Box& kWalkBox=collision::kHeight;
constexpr double kStrideM[]={.030,.045,.060};
// JOYSTICK: the same WALK law steered in SE(2). Displacement per cycle inside
// the ellipsoid (x/kJoystickStrideM)^2+(y/kJoystickLateralM)^2+(yaw/
// kJoystickYawRad)^2 <= 1, each axis rate limited to full scale over two gait
// cycles; swing targets are planned on that ramp. A steered walk keeps the
// WALK height, duty, lift and stride ramps and differs from WALK STANDARD only
// where lateral and turning steps need it: 14 mm lateral sway and stance
// contact on the tread fillet (FootContact.h), inside its own certified joint
// box kSteer and IMU tilt envelope. Offline, with the real fillet contacts and
// the URDF COM, the binding support edge is the RF-LH diagonal while RH swings;
// the lateral sway moves the COM away from it by ~0.75 mm per mm and tilts the
// stance feet (hip roll), so 15 mm already leaves the fillet limit at the IMU
// envelope boundary. At 14 mm steady commands keep >= +1.8 mm static support
// and the random multi-axis set >= +0.5 mm, a static model margin, not a
// dynamic stability claim (scripts/offline_motion/run_joystick_envelope.sh). The Controller
// ends the walk with WALK STOP when no admitted joystick command arrives for
// kJoystickTimeoutUs (ESP clock).
constexpr double kJoystickStrideM=kStrideM[1], kJoystickLateralM=.020, kJoystickYawRad=.20, kJoystickRampCycles=2;
constexpr double kJoystickSwayYM=.014;
constexpr uint64_t kJoystickTimeoutUs=1000000;
// IMU level correction slew in physical time (dev.14: 0.020 rad/s counted per
// release, slower still when releases were missed). A late release advances
// by at most kImuSlewMaxDtUs: no catch-up step after a pause.
constexpr double kImuSlewRadS=.040;  // .060 delays native arrival (NATIVE_TARGET_NOT_REACHED, STOP suite)
constexpr uint64_t kImuSlewMaxDtUs=2*kPeriodUs;
constexpr double kMinStandM=.050;
// PRONE: HEIGHT -> LIFT handover height (dev.16). At 53 mm the nominal rear
// UPPER target lay 2.2 ticks inside kLift and the recorded loaded sag (dev.15
// hardware: LH_UPPER 3, LF_UPPER 14 ticks) left the measured pose 0.3-0.8 ticks
// outside it: neither arrival nor loaded settle could hand over and the segment
// deadline released the torque at 52 mm. Lower handovers move LF_LOWER towards
// the kHeight floor instead. 52.0 mm keeps, against the worst recorded sag of
// every joint (dev.14 HOLD 69 + 23 cases, dev.15 PRONE 4 cases), >= 4.7 ticks
// inside kHeight and >= 3.5 ticks inside kLift (53 mm: -0.8). Same boxes.
constexpr double kProneHandoverM=.052;
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
// PRONE ends at rest (dev.15): after the verified PRONE_LAND arrival the
// ordinary SAFE_OFF releases the torque (STOPPED, then READY, reason PRONE_REST).
enum class State : uint8_t { READY, MOVING, STANDING, WALKING, STOPPED, FAULT };
const char* name(State);
struct Feedback {
  bool valid=false; uint64_t stampUs=0,controlStampUs=0;
  uint16_t raw=0; int torqueLimit=-1; double q=0; // LOADDIAG: RAM 48 of the existing control read
  int speed=0,load=0,current=0,temp=0,voltage=0,status=0,torque=-1,goal=-1,goalSpeed=-1,acc=-1;
};
// Direct confirmation of a bulk PresentTemperature >= 70 C by an existing read:
// bulk value, direct value (-1 without a valid reply) and the bus outcome of
// the direct read (servo::BusOutcome). Evidence only, never a decision input.
struct TempCheck { uint64_t stampUs=0; uint8_t slot=12,outcome=0; int16_t bulk=-1,direct=-1; };
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
  // Decode a commanded RAW with the same gen-1 transform as readback. No I/O.
  virtual bool decodeRaw(unsigned,uint16_t,double&) { return false; }
  virtual bool frame(const uint16_t raw[12],Drive)=0;
  // Only an explicit pre-TX cancellation, never a partial/failed write.
  virtual bool framePreempted() const { return false; }
  virtual bool enable(unsigned slot)=0;
  virtual bool off(unsigned first=0)=0; // OFF slots first..12 (includes neck); default SAFE_OFF is global
  virtual bool verifyOff(unsigned slot)=0;  // slots 0..12, includes neck
  // One fault event, after the SAFE_OFF broadcast; no extra servo acquisition.
  virtual void referenceFault(unsigned,const Feedback&,double,uint16_t) {}
  // The temperature confirmation made by the last read(), if it made one.
  virtual bool tempCheck(TempCheck&) const { return false; }
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
  // Descent to the PRONE terminal, then rest with torque OFF (proneRest). Arrival
  // is not proof of ventral or passive support.
  const char* prone(uint64_t now);
  const char* walk(unsigned stride,uint64_t now);
  // JOYSTICK base velocity (SI). Starts like walk() or refreshes the target
  // and its deadline; zero velocity is walkStop().
  const char* drive(double vx,double vy,double yawRate,uint64_t now);
  void walkStop();
  // Atomic latch only: callable independently of HTTP and of the Motion tick.
  void stop(const char* reason="STOP_REQUESTED");
  void safeOff() { safeOffRequest_.store(true,std::memory_order_release); }
  void fault(const char* reason);
  void update(uint64_t now);
  // Read-only model estimate, 10 Hz in cooperative slack, outside Motion.
  void updateTelemetry(uint64_t now);
  State state() const { return state_; }
  const char* reason() const { return reason_; }
  const char* phase() const;
  bool engaged() const { return energized_||state_==State::MOVING||offPending_; }
  bool stopping() const { return offPending_||safeOffRequest_.load()||stop_.load()||stopActive_; }
  bool stopActive() const { return stopActive_; }
  bool stopInProgress() const { return stopActive_&&!stopVerified_; }
  bool stopVerified() const { return stopVerified_; } // feedback evidence of rest, NOT contact
  bool stopSupportVerified() const { return false; }
  const char* standResumeRefusal(uint64_t now) const;
  bool stopFrameAuthorized(const uint16_t raw[12],Drive) const;
  bool busPreemptRequested() const {
    return safeOffRequest_.load()||stop_.load()||offPending_||((faultLatched_||stopActive_)&&!stopIo_);
  }
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
  // Fore/aft body velocity of the walking law (m/s), not a measurement.
  double walkVelocityX() const { return phase_==Phase::WALK?gait_.advanceXM/kWalkPeriodS:0; }
  double walkVelocityY() const { return phase_==Phase::WALK?gait_.advanceYM/kWalkPeriodS:0; }
  double walkYawRate() const { return phase_==Phase::WALK?gait_.yawRad/kWalkPeriodS:0; }
  // A joystick intent holds the walk: walking or rising to walk, not stopping.
  bool joystickActive() const { return joystick_&&(walkRequested_||(state_==State::WALKING&&!walkStopping_)); }
  uint32_t releases() const { return releases_; }
  uint32_t missed() const { return missed_; }
  uint64_t maxGapUs() const { return maxGapUs_; }
  bool imuValid() const { return input_.imuValid; }
  uint64_t imuStampUs() const { return input_.imuStampUs; }
  const double* quaternion() const { return input_.quaternion; }  // BNO085 w,x,y,z (normalized)
  bool stabilizationLimited() const { return imuLimited_; }
  // Why the correction differs from the IMU request: OK, SLEW, ENVELOPE,
  // ATTENUATED, WORKSPACE, SUPPORT_FIT or IMU_STALE; requested and applied
  // body roll/pitch (rad). Values of the latest compensation, never re-solved.
  const char* imuReason() const { return imuReason_; }
  double imuWanted(unsigned i) const { return imuWanted_[i&1]; }
  double imuApplied(unsigned i) const { return (i&1?pitchCorrection_:rollCorrection_)*correctionScale_; }
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
  uint64_t deadlineUs() const { return stopActive_?stopDeadlineUs_:(pendingGoalValid_||segmentBox_)?segmentDeadlineUs_:goalDeadlineUs_; }
  // Last boundary completed under a static load: joint mask and phase.
  uint16_t loadedMask() const { return loadedMask_; }
  const char* loadedPhase() const { return loadedPhase_; }
  uint64_t loadedSinceUs() const { return loadedSinceUs_; }
  // Unconfirmed hot samples rejected by a later normal sample; joints whose
  // hot window is open.
  uint32_t tempSpikes() const { return tempSpikes_; }
  // Torque released by the PRONE rest, not by an operator STOP: a STAND
  // re-acquires the settled pose (ACQUIRE/REFERENCE/ENABLE). Cleared by any
  // later STOP, SAFE_OFF, fault or accepted STAND.
  bool proneRest() const { return proneRest_; }
  uint16_t hotPendingMask() const { uint16_t m=0;for(unsigned i=0;i<12;++i)if(hotSinceUs_[i])m|=1u<<i;return m; }
  // Direct temperature confirmations made by the Motion reads: count and the
  // last two (latest first), whatever their outcome.
  uint32_t tempChecks() const { return tempChecks_; }
  const TempCheck& tempCheck(unsigned latest) const { return tempCheck_[(tempChecks_+1-latest)%2]; }
 private:
  friend struct StopReferenceProbe; // offline read-only verification of the generator handoff
  enum class Phase:uint8_t { IDLE, ACQUIRE, REFERENCE, ENABLE, LOWER, ALIGN, DEPLOY, LIFT, HEIGHT, HOLD, WALK, WALK_HOLD, PRONE_HEIGHT, PRONE_LIFT, PRONE_DEPLOY, PRONE_LAND, STOP };
  enum class InitialRoute:uint8_t { ENTRY, HEIGHT, WALK, LIFT, DEPLOY };
  bool readTimed(unsigned slot,bool control,Feedback&);
  bool readDeferred() const { return readInterrupted_||port_->readDeferred(); }
  bool frameTimed(const uint16_t raw[12],Drive);
  bool planInitialReference();
  bool certifiedRegion(const motion::JointTargetFrame&,InitialRoute&) const;
  bool pointSafe(const motion::JointTargetFrame&) const;
  bool transitionSafe(const motion::JointTargetFrame&,const motion::JointTargetFrame&,const collision::Box&) const;
  bool send(const motion::JointTargetFrame&,Drive,uint64_t,bool holdReference=false);
  bool arrived(uint64_t) const;
  const collision::Box* proneHandoverBox() const;
  bool settledUnderLoad(uint64_t);
  bool entersHeight(const motion::JointTargetFrame& actual,const motion::JointTargetFrame& q) const;
  bool liftEntersHeight(const motion::JointTargetFrame& actual) const;
  bool heightStartClear(const motion::JointTargetFrame& actual,const motion::JointTargetFrame& q) const;
  bool heightTarget(double,motion::JointTargetFrame&) const;
  struct WalkRecoveryStart {
    motion::JointTargetFrame held{},first{};
    double cycles=0,stopCycle=-1;
  };
  bool walkRecoveryStart(WalkRecoveryStart&) const;
  const char* standResumeRefusal(uint64_t,WalkRecoveryStart&) const;
  void measureHeight();
  void nextStand(uint64_t);
  void beginNative(Phase,const motion::JointTargetFrame&,const collision::Box&,uint64_t,double pitch=0,double x=0,double z=0);
  bool via(const motion::JointTargetFrame&,double pitch,double x,double z);
  bool knotVia(unsigned knot);
  bool heightVias(double from,double to);
  void beginPlan(Phase,const collision::Box&,uint64_t,double speed=kStandSpeedRadS,double acc=kStandAccelerationRadS2);
  void beginProne(Phase,uint64_t);
  bool returningProne() const { return phase_>=Phase::PRONE_HEIGHT&&phase_<=Phase::PRONE_LAND; }
  void proneNotCompleted(uint64_t now,const char* held);
  // A STOP in a planned route segment (the PRONE descent or the STAND_DEPLOY/
  // STAND_LIFT rise): the held RAW restarts the STAND route of the certified
  // region that holds it (proneRecovery). Native moves have no held reference.
  bool stoppedInProneDescent() const { return (stopOrigin_>=Phase::PRONE_HEIGHT&&stopOrigin_<=Phase::PRONE_LAND)||
    ((stopOrigin_==Phase::DEPLOY||stopOrigin_==Phase::LIFT)&&issuedPlanned_); }
  const char* proneRecovery(motion::JointTargetFrame& held,InitialRoute& route) const;
  void beginDeploy(const motion::JointTargetFrame* measured,uint64_t);
  void beginLift(const motion::JointTargetFrame* measured,uint64_t);
  void beginHeight(const motion::JointTargetFrame& from,double fromHeight,const collision::Box&,uint64_t);
  motion::JointTargetFrame measured() const;
  void generateStand(uint64_t);
  void generateWalk(uint64_t);
  void compensate(motion::LocomotionFrame&,uint64_t);
  double slewStep(uint64_t now);
  motion::JointTargetFrame compensationTarget(const motion::LocomotionFrame&,const motion::BodyPose&,const motion::JointTargetFrame&,bool nominalStrip=false,double edgeTilt=0);
  bool compensationSafe(const motion::JointTargetFrame&,const collision::Box&,bool stopProof=true) const;
  void compensateLevel(motion::LocomotionFrame&,const motion::Vector3&,uint64_t,bool unwind=false);
  void startWalk(uint64_t);
  void steerWalk(uint64_t);
  bool limitWalkTilt(double& roll,double& pitch) const;
  const collision::Box& walkBox() const { return steered_?collision::kSteer:kWalkBox; }
  void serviceOff();
  bool serviceStop();
  void beginStop(uint64_t);
  void generateStop(uint64_t);
  void rememberReference(uint64_t);
  bool stopTimedOut(uint64_t);
  void enterOff(bool fault,const char*);
  ServoPort* port_=nullptr;
  Inputs input_{};
  Feedback fb_[12]{};
  uint8_t failures_[12]{},controlFailures_[12]{};
  std::atomic<bool> stop_{false};
  std::atomic<bool> safeOffRequest_{false};
  std::atomic<const char*> stopReason_{"STOP_REQUESTED"};
  const char* holdReason_=nullptr; // automatic support hold (PRONE not completed); null for an operator STOP
  State state_=State::STOPPED; Phase phase_=Phase::IDLE;
  const char* reason_="BOOTING";
  bool operatorStopped_=false,proneRest_=false;
  bool stopActive_=false,stopVerified_=false,stopIo_=false,stopFrame_=false,readInterrupted_=false;
  const uint16_t* stopRaw_=nullptr;
  Drive stopDrive_=kStandDrive;
  StopPlan stopPlan_{};
  uint64_t stopStartUs_=0,stopTerminalUs_=0,stopDwellUs_=0,stopDeadlineUs_=0;
  uint16_t stopDwellRaw_[12]{};
  const collision::Box* stopBox_=nullptr;
  Phase stopOrigin_=Phase::IDLE;
  // Writer-owned samples of the accepted RAW reference (including unchanged
  // held goals). Timestamped with the ESP clock after TX, not release entry.
  uint16_t previousRaw_[12]{};
  uint64_t issuedUs_=0,previousUs_=0;
  Phase issuedPhase_=Phase::IDLE;
  bool issuedPlanned_=false;
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
  uint64_t hotSinceUs_[12]{},hotLastUs_[12]{}; uint32_t tempSpikes_=0,tempChecks_=0; TempCheck tempCheck_[2]{};
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
  // The walking law and its track. joystick_ is the live intent (deadline);
  // steered_ marks a walk started by it, through STOP and its recovery.
  motion::GaitParameters gait_{};
  motion::GaitTrack track_{};
  bool joystick_=false,steered_=false;
  double joystickGoal_[3]{},steeredCycles_=0;  // displacement per cycle: x, y (m), yaw (rad)
  uint64_t joystickDeadlineUs_=0;
  motion::LocomotionFrame walkPrevious_{};
  motion::LocomotionFrame holdNominal_{};
  uint64_t walkStartUs_=0;
  double cycles_=0,stopCycle_=-1;
  // Explicit STAND after a verified WALK STOP: the same gait completes its WALK STOP
  // from the stopped phase. Clock eased in from rest; the held RAW offset fades out.
  bool walkRecover_=false; double recoverCycles_=0; uint64_t recoverUs_=0; motion::JointTargetFrame recoverOffset_{};
  bool walkStopping_=false,imuLimited_=false;
  double rollCorrection_=0,pitchCorrection_=0,correctionScale_=0;
  const char* imuReason_="IMU_STALE";double imuWanted_[2]{};
};
} }
