#ifndef MATDOG_MOTION_LEG_KINEMATICS_INTERNAL_H
#define MATDOG_MOTION_LEG_KINEMATICS_INTERNAL_H
#include "LegKinematics.h"
namespace matdog { namespace motion { namespace detail {
// Internal analytic reduction for exporter-verified X/Y/Y chains only.
// hipFrameZ is a translation AFTER hip rotation, BEFORE the planar chain.
// Public G1 callers always use zero; G2 uses it for the radial contact term.
struct AnalyticBranch { int8_t hip = 0; int8_t elbow = 0; };
FkResult forwardWithModel(const LegModel& model, const LegJointAngles& q,
                          double hipFrameZ);
IkResult inverseWithModel(const LegModel& model, const Vector3& target,
                          const IkOptions& options, double hipFrameZ,
                          AnalyticBranch* branch = nullptr);
} } }
#endif
