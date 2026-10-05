#include "LegKinematics.h"
#include "LegGeometryData.h"
#include "LegKinematicsInternal.h"

#include <cmath>

namespace matdog {
namespace motion {

const LegModel* legModel(LegId leg) {
  const unsigned index = static_cast<unsigned>(leg);
  return index < 4 ? &generated::kLegs[index] : nullptr;
}

const char* geometrySourceSha256() { return generated::kUrdfSha256; }

FkResult forwardKinematics(LegId leg, const LegJointAngles& q) {
  const LegModel* model = legModel(leg);
  return model ? detail::forwardWithModel(*model, q, 0.0) : FkResult{};
}

FkResult detail::forwardWithModel(const LegModel& geometry, const LegJointAngles& q,
                                   double hipFrameZ) {
  FkResult result;
  const LegModel* model = &geometry;
  if (!std::isfinite(q.hip) || !std::isfinite(q.upper) ||
      !std::isfinite(q.lower)) return result;
  const double values[3] = {q.hip, q.upper, q.lower};
  for (unsigned i = 0; i < 3; ++i) {
    if (values[i] < model->limits[i].lower || values[i] > model->limits[i].upper) {
      result.status = KinematicsStatus::JOINT_LIMIT;
      return result;
    }
  }

  // Exact reduction of T_hip Rx(h) T_upper Ry(u) T_lower Ry(l) T_foot.
  // Exporter verifies all omitted offsets/rotations and axis assumptions.
  const double ch = std::cos(q.hip), sh = std::sin(q.hip);
  const double cu = std::cos(q.upper), su = std::sin(q.upper);
  const double ct = std::cos(q.upper + q.lower);
  const double st = std::sin(q.upper + q.lower);
  const double x = model->lowerOrigin.z * su +
                   model->footOrigin.x * ct + model->footOrigin.z * st;
  const double y = model->upperOrigin.y + model->footOrigin.y;
  const double z = model->lowerOrigin.z * cu -
                   model->footOrigin.x * st + model->footOrigin.z * ct + hipFrameZ;
  result.footOriginM = {model->hipOrigin.x + x,
                       model->hipOrigin.y + ch * y - sh * z,
                       model->hipOrigin.z + sh * y + ch * z};
  const double rotation[3][3] = {
      {ct, 0.0, st}, {sh * st, ch, -sh * ct}, {-ch * st, sh, ch * ct}};
  for (unsigned r = 0; r < 3; ++r)
    for (unsigned c = 0; c < 3; ++c)
      result.baseFromFootRotation[r][c] = rotation[r][c];
  result.status = KinematicsStatus::OK;
  return result;
}

}  // namespace motion
}  // namespace matdog
