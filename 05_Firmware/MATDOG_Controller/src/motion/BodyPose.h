#ifndef MATDOG_MOTION_BODY_POSE_H
#define MATDOG_MOTION_BODY_POSE_H
#include "FootContact.h"
namespace matdog { namespace motion {
// Rigid world_from_base_link transform. Metres, right-handed X/Y/Z.
struct BodyPose {
  Vector3 translationM{};
  double rotation[3][3] = {{1,0,0},{0,1,0},{0,0,1}};
};
bool validBodyPose(const BodyPose& pose);
// Checked point transforms. On failure output is zeroed.
bool pointToWorld(const BodyPose& pose, Vector3 base, Vector3& world);
bool pointToBase(const BodyPose& pose, Vector3 world, Vector3& base);
// Yaw/translation are permitted; roll/pitch are unsupported by G2 contact IK.
bool supportsFlatContactIk(const BodyPose& pose);
struct WorldContactIkResult {
  enum class Status : uint8_t { OK, INVALID_POSE_OR_POINT, UNSUPPORTED_ORIENTATION, IK_FAILURE };
  Status status = Status::INVALID_POSE_OR_POINT;
  ContactIkResult baseResult{};  // G2 contact fields remain in base_link
  Vector3 achievedWorldM{};
};
WorldContactIkResult worldContactInverseKinematics(LegId leg, Vector3 worldTarget,
                      const BodyPose& pose, const ContactIkOptions& options = ContactIkOptions{});
} }
#endif
