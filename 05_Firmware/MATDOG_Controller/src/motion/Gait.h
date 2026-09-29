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
  // Open-loop study parameter, not a feedback/stabilization controller.
  double swayXM = 0;
  double maxCondition = 10000;
};
struct LegPhase { double phase=0, swingProgress=0; bool stance=true, boundary=false; };
bool validGaitParameters(const GaitParameters& p);
bool validGaitPeriod(double seconds);
double gaitOffset(GaitType type, LegId leg);
LegPhase legPhase(const GaitParameters& p, LegId leg, double cycles);
struct CartesianSample {
  BodyPose body{};
  Vector3 bodyVelocity{}, bodyAcceleration{};
  double yawVelocity=0, yawAcceleration=0;
  Vector3 feet[4]{}, velocity[4]{}, acceleration[4]{};
  LegPhase phases[4]{};
  double cycles=0;
  bool valid=false;
};
// Initial anchors are canonical C4 contacts at world origin. Optional terminal
// integer cycle assembles canonical contacts at that cycle's body transform.
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
