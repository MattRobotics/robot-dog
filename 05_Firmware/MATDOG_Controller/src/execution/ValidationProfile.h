#ifndef MATDOG_VALIDATION_PROFILE_H
#define MATDOG_VALIDATION_PROFILE_H
#include "ReviewedMotionRuntime.h"
#include "../calibration/CalibrationRuntimeRestore.h"
#include "../update/Sha256.h"
namespace matdog { namespace execution {
// USB-only, RAM-only staging. SHA binds the exact reviewed ASCII cards,
// including scope/generation. A hash is integrity, never hardware approval.
class ValidationProfile {
 public:
 bool card(const char* line);
 void clear(){*this=ValidationProfile{};}
 bool sealed() const{return sealed_;}
 const char* reason() const{return reason_;}
 calibration::OperationalWorkspaceEvidence workspace{};
 ReviewedMotionRuntime runtime{};
 unsigned first_knot=0,last_knot=0;
 double joint_tube_rad=0,position_tube_m=0,rotation_tube=0;
 uint16_t output_ceiling=0;
 uint32_t transaction_timeout_ms=0;
 uint16_t drive_speed=0;uint8_t drive_acceleration=0;double speed_scale=0,acceleration_scale=0,walk_tube_rad=0;
 char record_sha256[65]{};
 char review_sha256[65]{};
 char drive_proof_sha256[65]{},walk_proof_sha256[65]{},support_proof_sha256[65]{},loaded_tracking_proof_sha256[65]{},dynamics_proof_sha256[65]{};
 char corridor_proof_sha256[65]{},fixture_proof_sha256[65]{};
 motion::BodyPose fixture_pose{};
 double encoder_uncertainty_rad=0,mapping_uncertainty_rad=0;
 private:
 update::Sha256 hash_{};
 uint16_t joint_mask_=0;
 bool open_=false,sealed_=false,exec_=false,corridor_=false,walk_=false,clock_=false,fixture_=false,error_budget_=false,step_=false,drive_=false,support_=false,dynamics_=false,walk_clock_=false,route_=false,record_=false,review_=false;
 const char* reason_="REVIEWED_PROFILE_REQUIRED";
};
} }
#endif
