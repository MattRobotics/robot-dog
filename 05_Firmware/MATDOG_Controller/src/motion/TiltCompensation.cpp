#include "TiltCompensation.h"
#include <cmath>
namespace matdog { namespace motion {
BodyPose bodyPoseFromRollPitch(const Vector3& t, double roll, double pitch) {
  BodyPose p; p.translationM = t;
  const double cr = std::cos(roll), sr = std::sin(roll), cp = std::cos(pitch), sp = std::sin(pitch);
  // R = Ry(pitch) * Rx(roll)
  const double r[3][3] = {{cp, sp*sr, sp*cr}, {0, cr, -sr}, {-sp, cp*sr, cp*cr}};
  for (unsigned i = 0; i < 3; ++i) for (unsigned j = 0; j < 3; ++j) p.rotation[i][j] = r[i][j];
  return p;
}
TiltCompensation compensateStand(const StandDefinition& d, double height, double roll, double pitch, const LegJointAngles seeds[4], bool strip) {
  TiltCompensation out;
  if (!std::isfinite(height) || height <= 0 || !std::isfinite(roll) || !std::isfinite(pitch) || std::abs(roll) > 1.0 || std::abs(pitch) > 1.0) return out;
  out.pose = bodyPoseFromRollPitch({0, 0, height}, roll, pitch);
  if (!validBodyPose(out.pose)) return out;
  out.minJointMarginRad = 1e9;
  for (unsigned i = 0; i < 4; ++i) {
    TiltedIkOptions o; o.seed = seeds[i]; o.requireNominalStrip = strip;
    const auto ik = worldContactInverseKinematicsTilted(static_cast<LegId>(i), d.contactsWorldM[i], out.pose, o);
    if (ik.status != TiltedIkResult::Status::OK) {
      TiltCompensation failed; failed.status = TiltCompensation::Status::IK_FAILURE; failed.failedLeg = static_cast<uint8_t>(i);
      failed.legStatus = ik.status; failed.pose = out.pose;
      return failed;  // never leak a partially solved 12-joint frame
    }
    out.target.legs[i] = ik.base.joints;
    out.contactsWorldM[i] = ik.achievedWorldM;
    const double dx = ik.achievedWorldM.x - d.contactsWorldM[i].x, dy = ik.achievedWorldM.y - d.contactsWorldM[i].y, dz = ik.achievedWorldM.z - d.contactsWorldM[i].z;
    out.maxContactDriftM = std::fmax(out.maxContactDriftM, std::sqrt(dx*dx + dy*dy + dz*dz));
    out.maxCondition = std::fmax(out.maxCondition, ik.base.condition);
    out.maxIterations = ik.base.iterations > out.maxIterations ? ik.base.iterations : out.maxIterations;
    const auto* m = legModel(static_cast<LegId>(i));
    const double q[3] = {ik.base.joints.hip, ik.base.joints.upper, ik.base.joints.lower};
    for (unsigned j = 0; j < 3; ++j) out.minJointMarginRad = std::fmin(out.minJointMarginRad, std::fmin(q[j] - m->limits[j].lower, m->limits[j].upper - q[j]));
  }
  out.target.valid = true; out.status = TiltCompensation::Status::OK;
  return out;
}
} }
