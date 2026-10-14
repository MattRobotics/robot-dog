#ifndef MATDOG_G5B_INERTIAL_INPUT_H
#define MATDOG_G5B_INERTIAL_INPUT_H
#include "StabilizationPipeline.h"
namespace matdog { namespace stabilization {
// Future REV01 input contract only. G5-B never loads REV00 masses or evaluates dynamics.
constexpr unsigned kMaxInertialLinks = 17;
enum class InertialRevision : uint8_t { UNAVAILABLE, HISTORICAL_REV00, REV01_PENDING, REV01_VERIFIED };
struct LinkInertia {
  char linkName[32]{};
  double massKg = 0;
  Vector3 comInLinkM{};
  double inertiaAtComKgM2[3][3]{}; // symmetric, link frame, physical tensor required
};
struct InertialInput {
  InertialRevision readiness = InertialRevision::UNAVAILABLE;
  char assemblyRevision[32]{}, provenance[96]{}, sourceSha256[65]{};
  double totalMassKg = 0;
  Vector3 totalComBaseM{};
  double totalInertiaAtComBaseKgM2[3][3]{};
  LinkInertia links[kMaxInertialLinks]{};
  unsigned linkCount = 0;
  double measuredAtS = 0;
};
// Future evaluator must transform CoM with BodyPose, project ALONG world gravity onto
// the supplied plane, form a convex polygon from confirmed contacts, and report signed
// edge margin. Fewer than 3 noncollinear contacts, stale support, incomplete provenance,
// nonphysical tensors, or readiness != REV01_VERIFIED must return DEPENDENCY_MISSING.
// Geometric containment alone is never dynamic stability or a load/slip measurement.
struct SupportProjectionRequest {
  InertialInput inertia{};
  BodyPose body{};
  SupportSnapshot support{};
  Vector3 gravityDirectionWorld{0, 0, -1};
  double nowS = 0, requiredMarginM = 0;
};
struct SupportProjectionResult {
  enum class Status : uint8_t { DEPENDENCY_MISSING, INVALID_INPUT, INSIDE, OUTSIDE };
  Status status = Status::DEPENDENCY_MISSING;
  Vector3 projectedComWorldM{};
  double signedMarginM = 0;
};
} }
#endif
