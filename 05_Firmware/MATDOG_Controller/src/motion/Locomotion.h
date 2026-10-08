#ifndef MATDOG_MOTION_LOCOMOTION_H
#define MATDOG_MOTION_LOCOMOTION_H
#include "Gait.h"
#include "StandTransition.h"
namespace matdog { namespace motion {
// G4 coordinator states are separate from the accepted G3 gate's state type.
// The first five map explicitly by value; startup state changes remain private
// to StandTransition. Pose identifiers never enter either state enumeration.
enum class LocomotionState : uint8_t { OFF, IDLE, STAND_TRANSITION, STAND, STOPPING, GAIT_START, WALK, TROT };
struct MotionCommand { GaitParameters gait{}; double periodS=1,stampS=0; uint32_t sequence=0; };
enum class CommandStatus : uint8_t { FRESH, STALE, INVALID, ZERO, CHANGED };
class CommandWatchdog {
 public:
  bool configure(double timeoutS);
  CommandStatus accept(const MotionCommand& command,double nowS);
  CommandStatus check(double nowS);
  const MotionCommand& command() const {return command_;}
 private:
  MotionCommand command_{};
  double timeout_=0,lastNow_=0;
  bool configured_=false,received_=false,clockSeen_=false;
};
// Owns the unchanged G3 startup coordinator. No public event can forge STAND.
// Times are supplied by the caller; no device clock or actuator output exists.
class Locomotion {
 public:
  LocomotionState state() const {return state_;}
  bool apply(MotionEvent event);
  bool beginStartup(const StartupObservation& observation,const TimingSpec& timing);
  TimedStandSample nextStartup();
  bool start(const MotionCommand& command,double nowS,double timeoutS);
  CommandStatus command(const MotionCommand& command,double nowS);
  bool requestStop();
  LocomotionFrame sample(double nowS);
  GaitStatus status() const {return status_;}
  CommandStatus commandStatus() const {return commandStatus_;}
 private:
  StandTransition startup_{};
  LocomotionState state_=LocomotionState::OFF;
  GaitStatus status_=GaitStatus::STATE_ERROR;
  CommandStatus commandStatus_=CommandStatus::INVALID;
  CommandWatchdog watchdog_{};
  MotionCommand active_{};
  LocomotionFrame previous_{};
  BodyPose origin_{};
  double startTime_=0,lastTime_=0,stopCycle_=-1;
  bool stopRequested_=false,holdOnlyStop_=false,gaitActive_=false;
};
} }
#endif
