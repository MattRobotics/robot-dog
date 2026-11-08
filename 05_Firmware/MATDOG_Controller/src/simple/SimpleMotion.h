#pragma once
#include <atomic>
#include <algorithm>
#include <cmath>
#include <stdint.h>
#include "../motion/MotionContract.h"
#include "../motion/Gait.h"
#include "CollisionData.h"
#include "SteerCollisionData.h"
#include "ViaPlan.h"
#include "StopPlan.h"

namespace matdog { namespace simple {
constexpr uint32_t kPeriodUs=20000, kReadTimeoutMs=2;
// WALK STANDARD v2 (gait redesign 2026-11-08): one statically stable crawl
// law (lateral sequence RH, RF, LH, LF), three strides per cycle, C2 stride
// ramps over one cycle at start and at WALK STOP. Against v1.0.1 (duty .825,
// 6.5 s, 30/45/60 mm, fore/aft sway 10 mm) three things change, each with
// its offline cause (scripts/offline_motion/walk_tilt_envelope_check.cpp):
// - the body walks kWalkComShiftXM ahead of the canonical foothold pattern.
//   The URDF COM stands 9.4 mm behind the centre of the canonical WALK120
//   contacts; centred, the stance stroke sits inside the kHeight joint range
//   (front UPPER 31 ticks from its lower bound at 100 mm, v1.0.1 15 ticks at
//   60 mm) and a 3 mm fore/aft sway keeps >= +2.5 mm static support through
//   start, steady walk and every WALK STOP (v1.0.1 +0.9 mm);
// - the swing foot moves with a trapezoidal velocity in thirds (kWalkSwingRamp):
//   peak speed and acceleration 1.5x and 4.5x the mean against 1.875x and
//   5.77x of the smootherstep, the binding joint-speed limit of a fast crawl.
//   The last third decelerates toward the touchdown, where the front UPPER
//   meets its kHeight lower bound: it lasts longer than the STOP arrest from
//   the peak speed (1.5 v/a), so the STOP proof keeps its margin there (with
//   ramps of 0.25 the IMU pitch-down envelope collapsed on that proof);
// - duty .80 lengthens the swing; the start clock eases in over one second
//   (walkClockS) and the final recentring takes kWalkSettleCycles, so the
//   four-contact body moves stay inside the qualified drive.
constexpr double kWalkPeriodS=5.0, kWalkDuty=.80, kWalkLiftM=.008;
// Lower WALK posture stays in the existing HEIGHT joint box (HIP +/-25 mrad).
// The HIP range is shared by the lateral sway, the IMU roll correction and the
// STOP arrest: the 1.5 mm lateral transfer in a 0.25 s four-contact window
// leaves >= 9 ticks for a STOP proved on a RAW secant one tick per period
// pessimistic (2 mm left -1.7 ticks there: walk_tilt_envelope_check.cpp).
constexpr double kWalkHeightM=.120, kWalkSwayXM=.003, kWalkSwayYM=.0015, kWalkStartCycles=1, kWalkStopCycles=1;
constexpr double kWalkComShiftXM=.011, kWalkSwingRamp=1./3, kWalkSettleCycles=.10;
static constexpr const collision::Box& kWalkBox=collision::kHeight;
constexpr double kStrideM[]={.050,.075,.100};
// Gait time of a walk clock started at rest: C2 ease of the rate 0->1 over the
// first second (start and STOP recovery), then real time minus 0.5 s.
inline double walkClockS(double t){const double u=std::fmin(1.,t);return t<1?u*u*u*u*(2.5+u*(-3+u)):t-.5;}
// The two walking laws (single definition for the firmware and offline tools).
motion::GaitParameters walkGait(unsigned stride);
motion::GaitParameters steerGait(const double goal[3]);
// JOYSTICK: the dev.15 steered law, unchanged by the WALK STANDARD v2 redesign
// (its own period, duty and 10 mm fore/aft sway, no COM offset, smootherstep
// swing, as certified with kSteer), steered in SE(2). Displacement per cycle inside
// the ellipsoid (x/kJoystickStrideM)^2+(y/kJoystickLateralM)^2+(yaw/
// kJoystickYawRad)^2 <= 1, each axis rate limited to full scale over two gait
// cycles; swing targets are planned on that ramp. A steered walk keeps the
// WALK height, duty, lift and stride ramps and differs from WALK STANDARD only
// where lateral and turning steps need it: up to 14 mm lateral sway and stance
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
constexpr double kJoystickPeriodS=6.5, kJoystickDuty=.825, kJoystickSwayXM=.010, kJoystickSwayBaseYM=.002;
constexpr double kJoystickStrideM=.045, kJoystickLateralM=.020, kJoystickYawRad=.20, kJoystickRampCycles=2;
// Steered lateral sway (motion-perf-next). A purely longitudinal joystick walks
// with kJoystickSwayBaseYM (2 mm, the v1.0.1 WALK sway); up to kJoystickSwayOnDemand of the
// lateral/yaw fraction of the ellipsoid the amplitude rises linearly to
// kJoystickSwayLowM, and from there any lateral or turning intent uses the
// full kJoystickSwayYM until the demand falls below kJoystickSwayOffDemand
// (hysteresis keyed on the current amplitude, no extra state). The gap is
// physical, not a tuning choice: at ~5.5 mm of body sway the stance hips tilt
// the foot axis past the 2 deg nominal strip and the real contact migrates
// ~5 mm along the tread fillet in the sway direction, so the support polygon
// follows the COM. Offline (URDF COM, real fillet contacts) a steady 6 mm
// amplitude gives -0.08 mm of static support at the first swing, 2 mm +1.07,
// 4.4 mm +2.9, 10 mm +3.0, 14 mm +5.9. The amplitude changes only with four
// feet down (steerWalk), rate limited over kJoystickSwayRampCycles, so no
// three-contact phase is crossed inside the 5-9 mm band.
constexpr double kJoystickSwayYM=.014, kJoystickSwayLowM=.005, kJoystickSwayOnDemand=.03, kJoystickSwayOffDemand=.015, kJoystickSwayRampCycles=.5;
inline double joystickSwayYM(double advanceYM,double yawRad,double currentSwayYM){
  const double y=advanceYM/kJoystickLateralM,w=yawRad/kJoystickYawRad,s=std::sqrt(y*y+w*w);
  if(s>=(currentSwayYM>kJoystickSwayLowM?kJoystickSwayOffDemand:kJoystickSwayOnDemand))return kJoystickSwayYM;
  return kJoystickSwayBaseYM+(kJoystickSwayLowM-kJoystickSwayBaseYM)*std::fmin(1.,s/kJoystickSwayOnDemand);
}
constexpr uint64_t kJoystickTimeoutUs=1000000;
// Fore/aft JOYSTICK (dev.5, ground HW 08/11/2026). A demand whose lateral and
// yaw part of the published ellipsoid stays under kJoystickFrontalBand is pure
// fore/aft and walks the WALK STANDARD v2 law itself (walkGait: period, duty,
// COM shift, swing, 1.5 mm sway, kHeight box, kWalkTilt100, nominal strip),
// its stride continuous up to the 100 mm preset forward and kJoystickBackM
// backward. Offline the v2 law keeps positive static support at 10-100 mm in
// both directions with the URDF COM; backward its support window on a forward
// COM error is narrower (zero near +2.9 mm at 50 mm, +2.3 mm at 75 mm, against
// +3.6 mm forward at 100 mm), hence the shorter backward stride. Lateral or
// yaw at or above the band use the steered law; a steering demand during a
// fore/aft walk ends it with WALK STOP (JOYSTICK_STEERING) and the held stick
// starts the steered walk from STANDING, like a reversal. On the ground a
// nearly forward stick carried 0.1-0.4 mm/s of lateral (3-13 % of the range):
// with the steered law that took the full 14 mm sway (28 mm p-p measured).
// A walk starts at the demanded stride (the v2 start ramp); a stride change
// while walking follows kJoystickFrontalRampCycles for the full 100 mm. A
// swing target is fixed at lift-off, so a fast increase leaves the stance feet
// behind the steady pattern: offline (fillet contacts, URDF COM) 20 -> 100 mm
// gave -2.91 mm of static support with 1 cycle, -0.24 with 2, +0.52 with 3
// and >= +0.83 mm with 4 over phased increases and seeded random fore/aft
// sequences (run_joystick_envelope.sh); decreases were positive with 1.
constexpr double kJoystickFrontalBand=.20, kJoystickBackM=.050, kJoystickFrontalRampCycles=4;
// WALK STANDARD IMU tilt envelopes: inner convex polygons of the kHeight box,
// gen-1 RAW, nominal support strip (including rounding) and the StopPlan over
// the full phase set of walkGait (start, steady, every WALK STOP terminal and
// recentring), by ray search (scripts/offline_motion/walk_tilt_envelope_check.cpp
// --search: STOP proved on the worst-case RAW rounding of the motion, then
// x0.85 for the runtime slew, T2 interpolation and late releases; the v1.0.1
// law passes that check at every vertex of its own envelopes). Vertices are
// roll/pitch BODY-in-support radians, not independent
// rectangular limits or a claim of physical contact. kWalkTilt75 serves the
// 50 and 75 mm strides, kWalkTilt100 the 100 mm stride.
// compensationSafe refusal codes (imuRefusedBy), in check order.
inline constexpr const char* kSafeChecks[]={"","POINT_SAFE","RAW_CONVERSION","RAW_BOX","LOAD_LIMIT","LOAD_BOX","LOAD_POINT","STOP_CLOCK","STOP_RATE","STOP_PLAN"};
struct TiltVertex { double roll,pitch; };
constexpr TiltVertex kWalkTilt75[]={{0.019019,0.000000},{0.019297,0.007993},{0.019700,0.019700},{0.019071,0.046042},{0.000000,0.066738},{-0.019000,0.045870},{-0.019700,0.019700},{-0.019297,0.007993},{-0.019019,0.000000},{-0.018722,-0.007755},{-0.018313,-0.018313},{-0.017392,-0.041987},{-0.000000,-0.081773},{0.017392,-0.041987},{0.018313,-0.018313},{0.018722,-0.007755}};
constexpr TiltVertex kWalkTilt100[]={{0.019019,0.000000},{0.019297,0.007993},{0.019678,0.019678},{0.013234,0.031951},{0.000000,0.038069},{-0.013234,0.031951},{-0.019678,0.019678},{-0.019297,0.007993},{-0.019019,0.000000},{-0.018693,-0.007743},{-0.012106,-0.012106},{-0.006313,-0.015242},{-0.000000,-0.018660},{0.006325,-0.015271},{0.012106,-0.012106},{0.018693,-0.007743}};
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
  double walkVelocityX() const { return phase_==Phase::WALK?gait_.advanceXM/period():0; }
  double walkVelocityY() const { return phase_==Phase::WALK?gait_.advanceYM/period():0; }
  double walkYawRate() const { return phase_==Phase::WALK?gait_.yawRad/period():0; }
  // Gait period of the current (or last) walk: JOYSTICK or WALK STANDARD.
  double period() const { return steered_?kJoystickPeriodS:kWalkPeriodS; }
  // Gait time t s after the start: WALK STANDARD eases in (walkClockS, its
  // first four-contact window carries the COM offset); the JOYSTICK law keeps
  // its constant clock as certified.
  double walkClock(double t) const { return steered_?t:walkClockS(t); }
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
  // dev.5 diagnostics, read-only. The compensationSafe check that refused last
  // in this release (kSafeChecks; 0 none): the first divergence behind WORKSPACE.
  uint8_t imuRefusedBy() const { return safeRefusal_; }
  // Measured joint farthest outside the entry region (12 when inside or not
  // fresh) and its signed distance (rad, < 0 under the low bound).
  unsigned entryViolation(uint64_t now,double& outsideRad) const;
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
  // Pre-actuation refusal of the measured entry pose (torque never enabled):
  // SAFE_OFF re-verified, no latch; READY again only through entryReadmitted.
  void refuseEntry();
  bool entryReadmitted(uint64_t now);
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
  // T2: the nominal frame of this release, written into the caller's frame
  // (target.valid false when a solve failed). Solves go through solveGaitInto
  // so the loop task stack never holds a temporary frame.
  void nominalRelease(motion::LocomotionFrame&,const motion::GaitParameters&,double terminal,bool clockDone);
  void compensate(motion::LocomotionFrame&,uint64_t);
  double slewStep(uint64_t now,uint64_t maxDtUs=kImuSlewMaxDtUs);
  motion::JointTargetFrame compensationTarget(const motion::LocomotionFrame&,const motion::BodyPose&,const motion::JointTargetFrame&,bool nominalStrip=false,double edgeTilt=0);
  bool compensationSafe(const motion::JointTargetFrame&,const collision::Box&,bool stopProof=true) const;
  void compensateLevel(motion::LocomotionFrame&,const motion::Vector3&,uint64_t,bool unwind=false);
  void startWalk(uint64_t);
  void steerWalk(uint64_t);
  // Displacement per cycle of the latest JOYSTICK demand on one law.
  void joystickGoal(bool frontal,double goal[3]) const;
  bool limitWalkTilt(double& roll,double& pitch,double headingCos=1,double headingSin=0) const;
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
  bool operatorStopped_=false,proneRest_=false,entryRefused_=false;
  uint64_t entrySinceUs_=0; // start of the current continuous re-admission evidence
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
  // Two accepted references before the current one: the STOP reference rate
  // is a secant over a window of at least one release period (rateFromHistory).
  uint16_t previousRaw_[12]{},previous2Raw_[12]{};
  uint64_t issuedUs_=0,previousUs_=0,previous2Us_=0;
  bool rateFromHistory(const double q[12],uint64_t atUs,unsigned skip,double vmax,double v[12]) const;
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
  // T1 (dev.16): the last ACCEPTED IMU correction as a joint offset from its
  // nominal target. Alternate WALK/HOLD releases reuse it on the new nominal
  // target; compensationSafe still runs on every release.
  motion::JointTargetFrame reuseOffset_{};Phase reusePhase_=Phase::IDLE;uint64_t reuseUs_=0;
  bool reuseLight_=false,reuseSettled_=false;
  void rememberCorrection(const motion::JointTargetFrame& accepted,const motion::JointTargetFrame& nominal,uint64_t now);
public:
  struct ReuseStats { uint32_t light=0,full=0,rejected=0; double maxShadowRad=0; };
  const ReuseStats& reuseStats() const { return reuseStats_; }
private:
  ReuseStats reuseStats_{};
  bool pendingGoalValid_=false;
  uint16_t commandedRaw_[12]{};
  Drive commandedDrive_=kStandDrive;
  double requestedHeight_=.15,actualHeight_=0;
  bool heightValid_=false,walkRequested_=false;
  unsigned stride_=1,requestedStride_=1;  // requested: applied by startWalk (dev.17)
  InitialRoute initialRoute_=InitialRoute::ENTRY;
  // The walking law and its track. joystick_ is the live intent (deadline);
  // steered_ marks a walk started by it, through STOP and its recovery.
  motion::GaitParameters gait_{};
  motion::GaitTrack track_{};
  bool joystick_=false,steered_=false;
  double joystickVel_[3]{},steeredCycles_=0;  // latest admitted demand: vx, vy (m/s), yaw rate (rad/s)
  bool joystickFrontal_=false;  // that demand is fore/aft only (kJoystickFrontalBand)
  uint64_t joystickDeadlineUs_=0;
  motion::LocomotionFrame walkPrevious_{};
  // T2 (motion-perf-next): the walking law is solved exactly at release grid
  // points two periods apart (one solve every other release) and every
  // release sends the joint-space linear interpolation of the two solutions
  // that bracket its own clock, so the reference is the law at `now` as in
  // v1.0.1 (the RAW secant STOP and compensationSafe read stays consistent
  // with wall time under late releases). Every release still runs pointSafe,
  // the WALK box, RAW, loaded and STOP proofs on what it sends. A structural
  // change of the law (terminal, STOP recovery, end of the clock) or a grid
  // point outside the bracket rebuilds both solutions; steerWalk retargets
  // stay continuous by construction and are tolerated.
  motion::LocomotionFrame walkExact_{},walkAhead_{};
  uint64_t walkExactUs_=0,walkExactGridUs_=0,walkAheadUs_=0;double walkExactTerminal_=-1;  // exact solve time, its grid point, look-ahead time
  motion::GaitParameters walkExactGait_{},walkAheadGait_{};  // laws at the two exact samples (shape check; shadow compare)
  bool walkAheadValid_=false,nominalSolved_=true;  // nominalSolved_: this release paid a nominal IK solve
  void resetLookahead(){walkAheadValid_=false;walkExactUs_=0;nominalSolved_=true;
#ifdef MATDOG_T2_SHADOW
    shadowTrack_=track_;
#endif
  }
  void lerpFrame(motion::LocomotionFrame&,double u,const motion::GaitParameters&,double terminal) const;
#ifdef MATDOG_T2_SHADOW
  motion::GaitTrack shadowTrack_{};  // test builds: the law's track sampled at every release clock
#endif
public:
  // exact: releases that solved the law; interp: releases that only
  // interpolated; resync: exact solutions rebuilt (start, missed release,
  // WALK STOP terminal). maxShadowRad (MATDOG_T2_SHADOW builds): largest
  // joint difference between the sent reference and the exact solve at the
  // release clock on the law's own track (v1.0.1 sampling).
  struct LookaheadStats { uint32_t exact=0,interp=0,resync=0; double maxShadowRad=0; };
  const LookaheadStats& lookaheadStats() const { return lookaheadStats_; }
private:
  LookaheadStats lookaheadStats_{};
  motion::LocomotionFrame holdNominal_{};
  uint64_t walkStartUs_=0;
  double cycles_=0,stopCycle_=-1;
  // Explicit STAND after a verified WALK STOP: the same gait completes its WALK STOP
  // from the stopped phase. Clock eased in from rest; the held RAW offset fades out.
  bool walkRecover_=false; double recoverCycles_=0; uint64_t recoverUs_=0; motion::JointTargetFrame recoverOffset_{};
  bool walkStopping_=false,imuLimited_=false;
  double rollCorrection_=0,pitchCorrection_=0,correctionScale_=0;
  const char* imuReason_="IMU_STALE";double imuWanted_[2]{};
  mutable uint8_t safeRefusal_=0;
};
} }
