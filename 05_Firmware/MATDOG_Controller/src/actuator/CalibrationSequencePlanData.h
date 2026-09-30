// GENERATED FILE - DO NOT EDIT BY HAND.
//
// Produced by 06_Software/Matdog_Core/calibration/matdog_full_calibration_sequence_geometry_v5.py
//   --export-header from the geometry validation artifact(s) (sha256):
//   7fd5f4e0eb23526e01358391e6d0eae2ebb4943bacd05e60bfbac04dc252ecbf
//   c4c5a7de6632bf619ae6e68f45b1b9c363cf9430932be4bee64635279cb91d35
//   dc06b0193acca201ec7649bff037f3b7f6b3ef7409cff8033d5213e5aec87dc2
//   dcf2bb9ef2633259cf0c9fc3f9a957878128b2c63c317fed94713c2841fecadb
// Every pose below was evaluated, with every segment of its leg's 24-contact sequence up to the
// calibration guard (URDF limit + 64 ticks), on the SHA-pinned URDF and collision meshes.
// Regenerate with the tool; never patch a value (static_audit.py re-derives this file).

#ifndef MATDOG_ACTUATOR_CALIBRATION_SEQUENCE_PLAN_DATA_H
#define MATDOG_ACTUATOR_CALIBRATION_SEQUENCE_PLAN_DATA_H

#include "CalibrationSequencePlan.h"

namespace matdog {
namespace actuator {
namespace sequence_plan_data {

constexpr char kSchema[] = "matdog.full_calibration_sequence_geometry.v1";
constexpr char kArtifactSha256[] = "45a9c1e6ff889b7900bc382d84e38c369f106edc74a8bd1ff6429b1fda7b25e4";
constexpr char kUrdfSha256[] = "3890a3f0732dbed8abdc559106d7f32ee8d6e2111c8e1a06d2485bf2ffc81e59";
constexpr char kMeshManifestSha256[] = "60fff604eae0857c7c61f115bbe3922949ababdd827fab61c74e1673336c39e1";

// {leg, geometry_validated, has_rear_park, park_leg, park_joint, park_target,
//  upper_for_lower, upper_for_hip_min, upper_for_hip_max, lower_folded} - URDF q, micro-radians.
constexpr CalibrationSequencePlan kPlan = {
    kSchema, kArtifactSha256, kUrdfSha256, kMeshManifestSha256,
    {
    // LF: sequence_collision_free=True
    {calibration::Leg::LF, true, true, calibration::Leg::LH, calibration::JointKind::UPPER, 610865, 1570796, 1570796, 1483359, -1518641},
    // RF: sequence_collision_free=True
    {calibration::Leg::RF, true, true, calibration::Leg::RH, calibration::JointKind::UPPER, 610865, 1570796, 1483359, 1570796, -1518641},
    // RH: sequence_collision_free=True
    {calibration::Leg::RH, true, false, calibration::Leg::LF, calibration::JointKind::UPPER, 0, 1570796, 1570796, 1570796, -697961},
    // LH: sequence_collision_free=True
    {calibration::Leg::LH, true, false, calibration::Leg::LF, calibration::JointKind::UPPER, 0, 1570796, 1570796, 1570796, -697961},
    },
};

}  // namespace sequence_plan_data
}  // namespace actuator
}  // namespace matdog

#endif  // MATDOG_ACTUATOR_CALIBRATION_SEQUENCE_PLAN_DATA_H
