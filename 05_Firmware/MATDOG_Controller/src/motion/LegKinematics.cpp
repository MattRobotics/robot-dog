#include "LegKinematics.h"
#include "LegGeometryData.h"
#include "LegKinematicsInternal.h"
#include "MotionHotPath.h"

#include <cmath>

namespace matdog {
namespace motion {
namespace {
// Constant-initialized (no startup code): the ESP32-S3 FPU is not enabled for
// the global constructors, where a float instruction raises a coprocessor
// exception. The neighbour of a finite nonzero float f toward +inf (dir > 0) or
// -inf: the spacing in its binade [p, 2p) is p * 2^-23, half that below an
// exact power of two when moving toward zero.
constexpr float neighbour(float f, int dir) {
  const float a = f < 0 ? -f : f;
  float p = 1.0f;
  while (p > a) p *= 0.5f;
  while (p * 2.0f <= a) p *= 2.0f;
  const float spacing = p * 1.1920928955078125e-7f;
  const bool towardZero = (dir < 0) == (f > 0);
  const float d = towardZero && a == p ? spacing * 0.5f : spacing;
  return dir > 0 ? f + d : f - d;
}
constexpr float inward(double limit, bool upper) {
  const float value = static_cast<float>(limit);
  return (upper ? double(value) > limit : double(value) < limit) ? neighbour(value, upper ? -1 : 1) : value;
}
constexpr detail::ModelF modelOf(const LegModel& m) {
  detail::ModelF f{detail::toF(m.hipOrigin), detail::toF(m.upperOrigin), detail::toF(m.lowerOrigin), detail::toF(m.footOrigin), {}, {}};
  for (unsigned i = 0; i < 3; ++i) { f.lower[i] = inward(m.limits[i].lower, false); f.upper[i] = inward(m.limits[i].upper, true); }
  return f;
}
constexpr detail::ModelF kModelsF[4] = {modelOf(generated::kLegs[0]), modelOf(generated::kLegs[1]),
                                        modelOf(generated::kLegs[2]), modelOf(generated::kLegs[3])};
constexpr bool insideUrdfLimits() {
  for (unsigned l = 0; l < 4; ++l)
    for (unsigned i = 0; i < 3; ++i)
      if (double(kModelsF[l].lower[i]) < generated::kLegs[l].limits[i].lower ||
          double(kModelsF[l].upper[i]) > generated::kLegs[l].limits[i].upper) return false;
  return true;
}
static_assert(insideUrdfLimits(), "every float joint limit is rounded inward");
}  // namespace

MATDOG_MOTION_HOT const LegModel* legModel(LegId leg) {
  const unsigned index = static_cast<unsigned>(leg);
  return index < 4 ? &generated::kLegs[index] : nullptr;
}

const char* geometrySourceSha256() { return generated::kUrdfSha256; }

MATDOG_MOTION_HOT const detail::ModelF* detail::modelF(LegId leg) {
  const unsigned index = static_cast<unsigned>(leg);
  return index < 4 ? &kModelsF[index] : nullptr;
}

MATDOG_MOTION_HOT bool detail::inLimits(const ModelF& m, const JointsF& q) {
  return q.hip >= m.lower[0] && q.hip <= m.upper[0] && q.upper >= m.lower[1] && q.upper <= m.upper[1] &&
         q.lower >= m.lower[2] && q.lower <= m.upper[2];
}

KinematicsStatus detail::jointsF(LegId leg, const LegJointAngles& q, JointsF& out) {
  const LegModel* model = legModel(leg);
  if (!model || !std::isfinite(q.hip) || !std::isfinite(q.upper) || !std::isfinite(q.lower)) return KinematicsStatus::INVALID_INPUT;
  const double values[3] = {q.hip, q.upper, q.lower};
  float image[3];
  const ModelF& m = *modelF(leg);
  for (unsigned i = 0; i < 3; ++i) {
    if (values[i] < model->limits[i].lower || values[i] > model->limits[i].upper) return KinematicsStatus::JOINT_LIMIT;
    const float v = static_cast<float>(values[i]);
    image[i] = v < m.lower[i] ? m.lower[i] : v > m.upper[i] ? m.upper[i] : v;
  }
  out = {image[0], image[1], image[2]};
  return KinematicsStatus::OK;
}

FkResult forwardKinematics(LegId leg, const LegJointAngles& q) {
  FkResult result;
  detail::JointsF image;
  result.status = detail::jointsF(leg, q, image);
  if (result.status != KinematicsStatus::OK) return result;
  const auto fk = detail::forwardWithModel(*detail::modelF(leg), image, 0.0f);
  result.status = fk.status;
  result.footOriginM = detail::toD(fk.footOriginM);
  for (unsigned r = 0; r < 3; ++r)
    for (unsigned c = 0; c < 3; ++c) result.baseFromFootRotation[r][c] = fk.baseFromFootRotation[r][c];
  return result;
}

MATDOG_MOTION_HOT detail::FkF detail::forwardWithModel(const ModelF& model, const JointsF& q, float hipFrameZ) {
  FkF result;
  if (!std::isfinite(q.hip) || !std::isfinite(q.upper) || !std::isfinite(q.lower)) return result;
  if (!inLimits(model, q)) {
    result.status = KinematicsStatus::JOINT_LIMIT;
    return result;
  }

  // Exact reduction of T_hip Rx(h) T_upper Ry(u) T_lower Ry(l) T_foot.
  // Exporter verifies all omitted offsets/rotations and axis assumptions.
  const float ch = std::cos(q.hip), sh = std::sin(q.hip);
  const float cu = std::cos(q.upper), su = std::sin(q.upper);
  const float ct = std::cos(q.upper + q.lower);
  const float st = std::sin(q.upper + q.lower);
  const float x = model.lowerOrigin.z * su +
                  model.footOrigin.x * ct + model.footOrigin.z * st;
  const float y = model.upperOrigin.y + model.footOrigin.y;
  const float z = model.lowerOrigin.z * cu -
                  model.footOrigin.x * st + model.footOrigin.z * ct + hipFrameZ;
  result.footOriginM = {model.hipOrigin.x + x,
                        model.hipOrigin.y + ch * y - sh * z,
                        model.hipOrigin.z + sh * y + ch * z};
  const float rotation[3][3] = {
      {ct, 0.0f, st}, {sh * st, ch, -sh * ct}, {-ch * st, sh, ch * ct}};
  for (unsigned r = 0; r < 3; ++r)
    for (unsigned c = 0; c < 3; ++c)
      result.baseFromFootRotation[r][c] = rotation[r][c];
  result.status = KinematicsStatus::OK;
  return result;
}

}  // namespace motion
}  // namespace matdog

namespace matdog { namespace motion { namespace detail {
MATDOG_MOTION_HOT bool invert3(const float j[3][3], float inverse[3][3]) {
  float m[3][6];
  for (unsigned r = 0; r < 3; ++r) for (unsigned c = 0; c < 3; ++c) { m[r][c] = j[r][c]; m[r][3+c] = r == c ? 1.0f : 0.0f; }
  for (unsigned c = 0; c < 3; ++c) {
    unsigned p = c;
    for (unsigned r = c + 1; r < 3; ++r) if (std::abs(m[r][c]) > std::abs(m[p][c])) p = r;
    if (!(std::abs(m[p][c]) > kPivot)) return false;
    for (unsigned k = 0; k < 6; ++k) { const float t = m[p][k]; m[p][k] = m[c][k]; m[c][k] = t; }
    const float d = 1.0f / m[c][c];
    for (unsigned k = c; k < 6; ++k) m[c][k] *= d;
    for (unsigned r = 0; r < 3; ++r) if (r != c) { const float f = m[r][c]; for (unsigned k = c; k < 6; ++k) m[r][k] -= f * m[c][k]; }
  }
  for (unsigned r = 0; r < 3; ++r) for (unsigned c = 0; c < 3; ++c) { inverse[r][c] = m[r][3+c]; if (!std::isfinite(inverse[r][c])) return false; }
  return true;
}
} } }
