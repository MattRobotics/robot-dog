#ifndef MATDOG_MOTION_GAIT_H
#define MATDOG_MOTION_GAIT_H
#include "BodyPose.h"
#include "StandTrajectory.h"
namespace matdog { namespace motion {
enum class GaitType : uint8_t { WALK, TROT };
enum class GaitStatus : uint8_t {
  OK, INVALID_PARAMETER, NONFINITE, IK_UNREACHABLE, JOINT_LIMIT,
  CONTACT_INVALID, BRANCH_CHANGE, ILL_CONDITIONED, COLLISION,
  SUPPORT_INVALID, STATE_ERROR, CANCELLED
};
struct GaitParameters {
  GaitType type = GaitType::WALK;
  double heightM = .15, liftM = .01, duty = .8;
  // Body displacement in the instantaneous base plane per normalized cycle.
  double advanceXM = .01, advanceYM = 0, yawRad = 0;
  // WALK fore/aft COM offset: forward while a hind leg swings, backward while
  // a front leg swings, changing only inside four-contact windows. Open-loop
  // geometry, not a feedback/stabilization controller.
  double swayXM = 0;
  // WALK lateral COM offset: +Y while the right legs swing, -Y while the left
  // legs swing. It changes only inside four-contact windows (duty-0.75 cycle),
  // so every three-contact interval holds a constant offset. With a terminal
  // cycle T the walk ends at T+duty-0.75: the last window recentres to zero
  // with four contacts. Open-loop geometry, not a balance controller.
  double swayYM = 0;
  // Geometric stride ramps (cycles), C2 in the gait clock: the body progress
  // rate rises 0->1 over startCycles and, with a terminal cycle T, falls 1->0
  // over the stopCycles that end at T. Zero keeps the legacy law.
  double startCycles = 0, stopCycles = 0;
  double maxCondition = 10000;
};
struct LegPhase { double phase=0, swingProgress=0; bool stance=true, boundary=false; };
bool validGaitParameters(const GaitParameters& p);
bool validGaitPeriod(double seconds);
double gaitOffset(GaitType type, LegId leg);
LegPhase legPhase(const GaitParameters& p, LegId leg, double cycles);
// Four-contact recentring window after the terminal touchdown; 0 without sway.
double gaitSettleCycles(const GaitParameters& p);
struct CartesianSample {
  BodyPose body{};
  Vector3 bodyVelocity{}, bodyAcceleration{};
  double yawVelocity=0, yawAcceleration=0;
  Vector3 feet[4]{}, velocity[4]{}, acceleration[4]{};
  LegPhase phases[4]{};
  double cycles=0;
  double stlSmoothing=0,stlSmoothingVelocity=0,stlSmoothingAcceleration=0;
  bool valid=false;
};
// Initial anchors are canonical C4 contacts at world origin. Optional terminal
// cycle, a touchdown instant (multiple of 0.25), assembles canonical contacts
// at that cycle's body transform
// (with sway: at terminal+gaitSettleCycles, the final four-contact window).
// All values are SI. No roll/pitch or sloping terrain is represented.
CartesianSample gaitCartesian(const GaitParameters& p, double cycles,
                              double cyclesPerS, double cyclesPerS2,
                              double terminalCycle=-1);
// Four locked contacts, interpolation between geometric body configurations.
CartesianSample gaitHold(const GaitParameters& p, double anchorCycle,
                         double height, double sway, double heightVelocity=0,
                         double swayVelocity=0, double heightAcceleration=0,
                         double swayAcceleration=0);
struct LocomotionFrame {
  GaitStatus status=GaitStatus::STATE_ERROR;
  JointTargetFrame target{};
  CartesianSample cartesian{};
  LegJointAngles velocityRadS[4]{}, accelerationRadS2[4]{};
  ContactBranch branches[4]{};
  double condition[4]{}, minJointMarginRad=0, maxResidualM=0;
  uint8_t failedLeg=255;
  bool dynamicStabilityProven=false;
};
// Previous valid solution is the seed AND branch reference. nullptr anchors at
// canonical stand. An error leaves target.valid false; there is no clipping.
LocomotionFrame solveGait(const GaitParameters& p, const CartesianSample& sample,
                         const LocomotionFrame* previous=nullptr);
struct GaitAssessment {
  bool collisionFree=false, contactsValid=false, positiveWalkSupport=false;
};
// Explicit offline assessment boundary. No mesh engine is hidden in C++.
GaitStatus assessGait(LocomotionFrame& frame, GaitType type, const GaitAssessment& assessment);
} }
#endif
