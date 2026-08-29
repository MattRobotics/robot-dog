/*
 * MATDOG FULL LEG CALIBRATOR V1 — generated geometry plan.
 *
 * DO NOT HAND EDIT. Regenerate/check with:
 *   python3 -B 06_Software/Matdog_Core/calibration/
 *     generate_flc_leg_plan.py [--check]
 *
 * Raw Geometry Compiler feasibility is sampled geometric evidence, not a
 * continuous swept-volume proof. The joined G12 result is an external
 * clearance classification only. Both artifacts explicitly grant ZERO
 * motion authorizations; live build/session/operator gates stay separate.
 */

#ifndef FLC_LEG_PLAN_H
#define FLC_LEG_PLAN_H

#include <stdint.h>

static const char FLC_GEOMETRY_PLAN_GENERATOR_SHA256[] = "66aae4417c8d81075700215871aadf7e30f5d5628b5acab318e4014c41ec80c0";
static const char FLC_GEOMETRY_PARKING_ARTIFACT_PATH[] = "09_Logs/Validation_Reports/Geometry_Compiler/2026-08-11_131818_MATDOG_GEOMETRY_V5_REMEDIATION_BENCHMARK_D_W4_PATH_PARKING.json";
static const char FLC_GEOMETRY_PARKING_FILE_SHA256[] = "e561e7fb98880843e590f4676e8559374c51d0e641102f89a803b3619722c4d7";
static const char FLC_GEOMETRY_PARKING_SEMANTIC_SHA256[] = "67c58430e78241af1a636cdcc22092ff855371713fc7f26bc56412f7c7181139";
static const char FLC_GEOMETRY_ENDPOINT_PROFILE_PATH[] = "09_Logs/Validation_Reports/Geometry_Compiler/2026-08-11_131818_MATDOG_GEOMETRY_V5_REMEDIATION_BENCHMARK_D_W4_ENDPOINT_PROFILE.json";
static const char FLC_GEOMETRY_ENDPOINT_FILE_SHA256[] = "dd8cb42c3b916d067f97a321c5ffcdfb013dde1f2f2a6ba71f73becff360dc0f";
static const char FLC_GEOMETRY_ENDPOINT_SEMANTIC_SHA256[] = "de205209f6015734f43af7f49146ecf60f89a74d6ce1276ce134c189a89c9f7e";
static const char FLC_GEOMETRY_COMBINED_SEMANTIC_SHA256[] = "0a772234a46afad14eb4af0999294020bb0fb8974ca0b68f3ccd780fa057db51";
static const char FLC_GEOMETRY_RUN_MANIFEST_PATH[] = "09_Logs/Validation_Reports/Geometry_Compiler/2026-08-11_131818_MATDOG_GEOMETRY_V5_REMEDIATION_BENCHMARK_D_W4_RUN_MANIFEST.json";
static const char FLC_GEOMETRY_RUN_MANIFEST_FILE_SHA256[] = "0db86e633599f63a769dbba75db3a54c1e6470e128c007eb24c465f92a428b17";
static const char FLC_GEOMETRY_RUN_MANIFEST_CONTENT_SHA256[] = "4e7172d473b5d19c79112252e50212565ef73505551bff993b6a0c927f2aaee7";
static const char FLC_GEOMETRY_SAFETY_POLICY_PATH[] = "09_Logs/Validation_Reports/Geometry_Compiler/2026-08-11_132758_MATDOG_GEOMETRY_V5_G12_FINAL_EXTERNAL_SAFETY_POLICY.json";
static const char FLC_GEOMETRY_SAFETY_POLICY_FILE_SHA256[] = "82f00a9414df06676babed1101a73e1a9d7bfa126592cdd7a425f66d5f01f1cc";
static const char FLC_GEOMETRY_SAFETY_POLICY_SEMANTIC_SHA256[] = "e5cb2a4c33082c59c6f5d381f90fcc19680cc899e326d13c9c9129932bde5d08";
static const char FLC_GEOMETRY_SOURCE_COMBINED_SHA256[] = "e4eea175e90131b1d8e55ae381fe8e88d6f966c0f87d4eee5c064d0a6967737f";
static const char FLC_GEOMETRY_URDF_SHA256[] = "3890a3f0732dbed8abdc559106d7f32ee8d6e2111c8e1a06d2485bf2ffc81e59";
static const int64_t FLC_PICORADIANS_PER_RADIAN = 1000000000000LL;
static const bool FLC_GEOMETRY_ARTIFACT_GRANTS_MOTION_AUTHORIZATION = false;

enum FlcGeometryJoint : uint8_t {
  FLC_GEOMETRY_JOINT_LF_HIP = 0,
  FLC_GEOMETRY_JOINT_LF_UPPER_LEG = 1,
  FLC_GEOMETRY_JOINT_LF_LOWER_LEG = 2,
  FLC_GEOMETRY_JOINT_RF_HIP = 3,
  FLC_GEOMETRY_JOINT_RF_UPPER_LEG = 4,
  FLC_GEOMETRY_JOINT_RF_LOWER_LEG = 5,
  FLC_GEOMETRY_JOINT_RH_HIP = 6,
  FLC_GEOMETRY_JOINT_RH_UPPER_LEG = 7,
  FLC_GEOMETRY_JOINT_RH_LOWER_LEG = 8,
  FLC_GEOMETRY_JOINT_LH_HIP = 9,
  FLC_GEOMETRY_JOINT_LH_UPPER_LEG = 10,
  FLC_GEOMETRY_JOINT_LH_LOWER_LEG = 11,
  FLC_GEOMETRY_JOINT_COUNT = 12,
  FLC_GEOMETRY_JOINT_NONE = 255
};

enum FlcParkingOutcome : uint8_t {
  FLC_NO_PARKING_REQUIRED = 0,
  FLC_PARKING_REQUIRED_1DOF = 1
};

enum FlcGeometryTargetDomain : uint8_t {
  FLC_TARGET_EXECUTABLE_URDF_DOMAIN = 0,
  FLC_TARGET_DIAGNOSTIC_OUTSIDE_URDF_LIMITS = 1
};

enum FlcGeometryPathStatus : uint8_t {
  FLC_GEOMETRY_PATH_COLLISION_FREE = 0,
  FLC_GEOMETRY_PATH_OBSTRUCTION = 1
};

enum FlcGeometryRelation : uint8_t {
  FLC_GEOMETRY_RELATION_NONE = 0,
  FLC_GEOMETRY_RELATION_SAME_BRANCH = 1,
  FLC_GEOMETRY_RELATION_CROSS_BRANCH = 2
};

enum FlcGeometryClearancePolicyResult : uint8_t {
  FLC_CLEARANCE_POLICY_PASS = 0,
  FLC_CLEARANCE_POLICY_FAIL_EXACT = 1,
  FLC_CLEARANCE_POLICY_REJECT_GEOMETRY = 2,
  FLC_CLEARANCE_POLICY_UNRESOLVED_LOWER_BOUND = 3,
  FLC_CLEARANCE_POLICY_UNRESOLVED_MISSING = 4
};

enum FlcGeometryMotionAuthorizationProvenance : uint8_t {
  FLC_MOTION_NOT_GRANTED_OFFLINE_EVIDENCE_ONLY = 0
};

struct FlcEndpointGeometryPlan {
  uint8_t canonicalEndpointIndex;
  const char *endpointId;
  FlcGeometryJoint targetJoint;
  const char *jointName;
  const char *limitSide;
  int64_t targetAnglePicoRad;
  int64_t declaredLimitPicoRad;
  FlcParkingOutcome parkingOutcome;
  FlcGeometryTargetDomain targetDomain;
  FlcGeometryPathStatus baselinePathStatus;
  FlcGeometryJoint auxiliaryJoint;
  int64_t auxiliaryAnglePicoRad;
  const char *blockingLinkA;
  const char *blockingLinkB;
  FlcGeometryRelation blockingRelation;
  FlcGeometryClearancePolicyResult clearancePolicyResult;
  FlcGeometryMotionAuthorizationProvenance motionAuthorizationProvenance;
  // Exact source sequence starts all 12 joints at q=0. During the
  // task, every masked joint must remain verified at q=0.
  uint16_t q0StartMask;
  uint16_t q0HeldDuringTaskMask;
};

static const FlcEndpointGeometryPlan FLC_ENDPOINT_GEOMETRY_PLANS[] = {
  // 0: lf_hip_joint:min
  {0, "lf_hip_joint:min", FLC_GEOMETRY_JOINT_LF_HIP, "lf_hip_joint", "min", -803055986689LL, -785398163397LL, FLC_NO_PARKING_REQUIRED, FLC_TARGET_DIAGNOSTIC_OUTSIDE_URDF_LIMITS, FLC_GEOMETRY_PATH_COLLISION_FREE, FLC_GEOMETRY_JOINT_NONE, 0LL, 0, 0, FLC_GEOMETRY_RELATION_NONE, FLC_CLEARANCE_POLICY_UNRESOLVED_LOWER_BOUND, FLC_MOTION_NOT_GRANTED_OFFLINE_EVIDENCE_ONLY, 0xFFF, 0xFFE},
  // 1: lf_hip_joint:max
  {1, "lf_hip_joint:max", FLC_GEOMETRY_JOINT_LF_HIP, "lf_hip_joint", "max", 789284248060LL, 785398163397LL, FLC_NO_PARKING_REQUIRED, FLC_TARGET_DIAGNOSTIC_OUTSIDE_URDF_LIMITS, FLC_GEOMETRY_PATH_COLLISION_FREE, FLC_GEOMETRY_JOINT_NONE, 0LL, 0, 0, FLC_GEOMETRY_RELATION_NONE, FLC_CLEARANCE_POLICY_PASS, FLC_MOTION_NOT_GRANTED_OFFLINE_EVIDENCE_ONLY, 0xFFF, 0xFFE},
  // 2: lf_upper_leg_joint:min
  {2, "lf_upper_leg_joint:min", FLC_GEOMETRY_JOINT_LF_UPPER_LEG, "lf_upper_leg_joint", "min", -909889226450LL, -916297857297LL, FLC_NO_PARKING_REQUIRED, FLC_TARGET_EXECUTABLE_URDF_DOMAIN, FLC_GEOMETRY_PATH_COLLISION_FREE, FLC_GEOMETRY_JOINT_NONE, 0LL, 0, 0, FLC_GEOMETRY_RELATION_NONE, FLC_CLEARANCE_POLICY_PASS, FLC_MOTION_NOT_GRANTED_OFFLINE_EVIDENCE_ONLY, 0xFFF, 0xFFD},
  // 3: lf_upper_leg_joint:max
  {3, "lf_upper_leg_joint:max", FLC_GEOMETRY_JOINT_LF_UPPER_LEG, "lf_upper_leg_joint", "max", 2127120025868LL, 2138028333693LL, FLC_PARKING_REQUIRED_1DOF, FLC_TARGET_EXECUTABLE_URDF_DOMAIN, FLC_GEOMETRY_PATH_OBSTRUCTION, FLC_GEOMETRY_JOINT_LH_UPPER_LEG, 610865238198LL, "lf_foot_link", "lh_foot_link", FLC_GEOMETRY_RELATION_CROSS_BRANCH, FLC_CLEARANCE_POLICY_PASS, FLC_MOTION_NOT_GRANTED_OFFLINE_EVIDENCE_ONLY, 0xFFF, 0xBFD},
  // 4: lf_lower_leg_joint:min
  {4, "lf_lower_leg_joint:min", FLC_GEOMETRY_JOINT_LF_LOWER_LEG, "lf_lower_leg_joint", "min", -1606998273389LL, -1605702911835LL, FLC_PARKING_REQUIRED_1DOF, FLC_TARGET_DIAGNOSTIC_OUTSIDE_URDF_LIMITS, FLC_GEOMETRY_PATH_OBSTRUCTION, FLC_GEOMETRY_JOINT_LF_UPPER_LEG, 1119919603363LL, "lf_hip_link", "lf_lower_leg_link", FLC_GEOMETRY_RELATION_SAME_BRANCH, FLC_CLEARANCE_POLICY_UNRESOLVED_LOWER_BOUND, FLC_MOTION_NOT_GRANTED_OFFLINE_EVIDENCE_ONLY, 0xFFF, 0xFF9},
  // 5: lf_lower_leg_joint:max
  {5, "lf_lower_leg_joint:max", FLC_GEOMETRY_JOINT_LF_LOWER_LEG, "lf_lower_leg_joint", "max", 666361254258LL, 654498469498LL, FLC_NO_PARKING_REQUIRED, FLC_TARGET_DIAGNOSTIC_OUTSIDE_URDF_LIMITS, FLC_GEOMETRY_PATH_COLLISION_FREE, FLC_GEOMETRY_JOINT_NONE, 0LL, 0, 0, FLC_GEOMETRY_RELATION_NONE, FLC_CLEARANCE_POLICY_PASS, FLC_MOTION_NOT_GRANTED_OFFLINE_EVIDENCE_ONLY, 0xFFF, 0xFFB},
  // 6: rf_hip_joint:min
  {6, "rf_hip_joint:min", FLC_GEOMETRY_JOINT_RF_HIP, "rf_hip_joint", "min", -789284248060LL, -785398163397LL, FLC_NO_PARKING_REQUIRED, FLC_TARGET_DIAGNOSTIC_OUTSIDE_URDF_LIMITS, FLC_GEOMETRY_PATH_COLLISION_FREE, FLC_GEOMETRY_JOINT_NONE, 0LL, 0, 0, FLC_GEOMETRY_RELATION_NONE, FLC_CLEARANCE_POLICY_PASS, FLC_MOTION_NOT_GRANTED_OFFLINE_EVIDENCE_ONLY, 0xFFF, 0xFF7},
  // 7: rf_hip_joint:max
  {7, "rf_hip_joint:max", FLC_GEOMETRY_JOINT_RF_HIP, "rf_hip_joint", "max", 803055986689LL, 785398163397LL, FLC_NO_PARKING_REQUIRED, FLC_TARGET_DIAGNOSTIC_OUTSIDE_URDF_LIMITS, FLC_GEOMETRY_PATH_COLLISION_FREE, FLC_GEOMETRY_JOINT_NONE, 0LL, 0, 0, FLC_GEOMETRY_RELATION_NONE, FLC_CLEARANCE_POLICY_UNRESOLVED_LOWER_BOUND, FLC_MOTION_NOT_GRANTED_OFFLINE_EVIDENCE_ONLY, 0xFFF, 0xFF7},
  // 8: rf_upper_leg_joint:min
  {8, "rf_upper_leg_joint:min", FLC_GEOMETRY_JOINT_RF_UPPER_LEG, "rf_upper_leg_joint", "min", -909889226450LL, -916297857297LL, FLC_NO_PARKING_REQUIRED, FLC_TARGET_EXECUTABLE_URDF_DOMAIN, FLC_GEOMETRY_PATH_COLLISION_FREE, FLC_GEOMETRY_JOINT_NONE, 0LL, 0, 0, FLC_GEOMETRY_RELATION_NONE, FLC_CLEARANCE_POLICY_PASS, FLC_MOTION_NOT_GRANTED_OFFLINE_EVIDENCE_ONLY, 0xFFF, 0xFEF},
  // 9: rf_upper_leg_joint:max
  {9, "rf_upper_leg_joint:max", FLC_GEOMETRY_JOINT_RF_UPPER_LEG, "rf_upper_leg_joint", "max", 2127120025868LL, 2138028333693LL, FLC_PARKING_REQUIRED_1DOF, FLC_TARGET_EXECUTABLE_URDF_DOMAIN, FLC_GEOMETRY_PATH_OBSTRUCTION, FLC_GEOMETRY_JOINT_RH_UPPER_LEG, 610865238198LL, "rf_foot_link", "rh_foot_link", FLC_GEOMETRY_RELATION_CROSS_BRANCH, FLC_CLEARANCE_POLICY_PASS, FLC_MOTION_NOT_GRANTED_OFFLINE_EVIDENCE_ONLY, 0xFFF, 0xF6F},
  // 10: rf_lower_leg_joint:min
  {10, "rf_lower_leg_joint:min", FLC_GEOMETRY_JOINT_RF_LOWER_LEG, "rf_lower_leg_joint", "min", -1606998273389LL, -1605702911835LL, FLC_PARKING_REQUIRED_1DOF, FLC_TARGET_DIAGNOSTIC_OUTSIDE_URDF_LIMITS, FLC_GEOMETRY_PATH_OBSTRUCTION, FLC_GEOMETRY_JOINT_RF_UPPER_LEG, 1119919603363LL, "rf_hip_link", "rf_lower_leg_link", FLC_GEOMETRY_RELATION_SAME_BRANCH, FLC_CLEARANCE_POLICY_UNRESOLVED_LOWER_BOUND, FLC_MOTION_NOT_GRANTED_OFFLINE_EVIDENCE_ONLY, 0xFFF, 0xFCF},
  // 11: rf_lower_leg_joint:max
  {11, "rf_lower_leg_joint:max", FLC_GEOMETRY_JOINT_RF_LOWER_LEG, "rf_lower_leg_joint", "max", 666361254258LL, 654498469498LL, FLC_NO_PARKING_REQUIRED, FLC_TARGET_DIAGNOSTIC_OUTSIDE_URDF_LIMITS, FLC_GEOMETRY_PATH_COLLISION_FREE, FLC_GEOMETRY_JOINT_NONE, 0LL, 0, 0, FLC_GEOMETRY_RELATION_NONE, FLC_CLEARANCE_POLICY_PASS, FLC_MOTION_NOT_GRANTED_OFFLINE_EVIDENCE_ONLY, 0xFFF, 0xFDF},
  // 12: rh_hip_joint:min
  {12, "rh_hip_joint:min", FLC_GEOMETRY_JOINT_RH_HIP, "rh_hip_joint", "min", -788125240354LL, -785398163397LL, FLC_NO_PARKING_REQUIRED, FLC_TARGET_DIAGNOSTIC_OUTSIDE_URDF_LIMITS, FLC_GEOMETRY_PATH_COLLISION_FREE, FLC_GEOMETRY_JOINT_NONE, 0LL, 0, 0, FLC_GEOMETRY_RELATION_NONE, FLC_CLEARANCE_POLICY_PASS, FLC_MOTION_NOT_GRANTED_OFFLINE_EVIDENCE_ONLY, 0xFFF, 0xFBF},
  // 13: rh_hip_joint:max
  {13, "rh_hip_joint:max", FLC_GEOMETRY_JOINT_RH_HIP, "rh_hip_joint", "max", 803055986689LL, 785398163397LL, FLC_NO_PARKING_REQUIRED, FLC_TARGET_DIAGNOSTIC_OUTSIDE_URDF_LIMITS, FLC_GEOMETRY_PATH_COLLISION_FREE, FLC_GEOMETRY_JOINT_NONE, 0LL, 0, 0, FLC_GEOMETRY_RELATION_NONE, FLC_CLEARANCE_POLICY_UNRESOLVED_LOWER_BOUND, FLC_MOTION_NOT_GRANTED_OFFLINE_EVIDENCE_ONLY, 0xFFF, 0xFBF},
  // 14: rh_upper_leg_joint:min
  {14, "rh_upper_leg_joint:min", FLC_GEOMETRY_JOINT_RH_UPPER_LEG, "rh_upper_leg_joint", "min", -909889226450LL, -916297857297LL, FLC_NO_PARKING_REQUIRED, FLC_TARGET_EXECUTABLE_URDF_DOMAIN, FLC_GEOMETRY_PATH_COLLISION_FREE, FLC_GEOMETRY_JOINT_NONE, 0LL, 0, 0, FLC_GEOMETRY_RELATION_NONE, FLC_CLEARANCE_POLICY_PASS, FLC_MOTION_NOT_GRANTED_OFFLINE_EVIDENCE_ONLY, 0xFFF, 0xF7F},
  // 15: rh_upper_leg_joint:max
  {15, "rh_upper_leg_joint:max", FLC_GEOMETRY_JOINT_RH_UPPER_LEG, "rh_upper_leg_joint", "max", 2127120025868LL, 2138028333693LL, FLC_NO_PARKING_REQUIRED, FLC_TARGET_EXECUTABLE_URDF_DOMAIN, FLC_GEOMETRY_PATH_COLLISION_FREE, FLC_GEOMETRY_JOINT_NONE, 0LL, 0, 0, FLC_GEOMETRY_RELATION_NONE, FLC_CLEARANCE_POLICY_PASS, FLC_MOTION_NOT_GRANTED_OFFLINE_EVIDENCE_ONLY, 0xFFF, 0xF7F},
  // 16: rh_lower_leg_joint:min
  {16, "rh_lower_leg_joint:min", FLC_GEOMETRY_JOINT_RH_LOWER_LEG, "rh_lower_leg_joint", "min", -1606998273389LL, -1605702911835LL, FLC_PARKING_REQUIRED_1DOF, FLC_TARGET_DIAGNOSTIC_OUTSIDE_URDF_LIMITS, FLC_GEOMETRY_PATH_OBSTRUCTION, FLC_GEOMETRY_JOINT_RH_UPPER_LEG, 1628973968528LL, "rh_hip_link", "rh_lower_leg_link", FLC_GEOMETRY_RELATION_SAME_BRANCH, FLC_CLEARANCE_POLICY_UNRESOLVED_LOWER_BOUND, FLC_MOTION_NOT_GRANTED_OFFLINE_EVIDENCE_ONLY, 0xFFF, 0xE7F},
  // 17: rh_lower_leg_joint:max
  {17, "rh_lower_leg_joint:max", FLC_GEOMETRY_JOINT_RH_LOWER_LEG, "rh_lower_leg_joint", "max", 666361254258LL, 654498469498LL, FLC_NO_PARKING_REQUIRED, FLC_TARGET_DIAGNOSTIC_OUTSIDE_URDF_LIMITS, FLC_GEOMETRY_PATH_COLLISION_FREE, FLC_GEOMETRY_JOINT_NONE, 0LL, 0, 0, FLC_GEOMETRY_RELATION_NONE, FLC_CLEARANCE_POLICY_PASS, FLC_MOTION_NOT_GRANTED_OFFLINE_EVIDENCE_ONLY, 0xFFF, 0xEFF},
  // 18: lh_hip_joint:min
  {18, "lh_hip_joint:min", FLC_GEOMETRY_JOINT_LH_HIP, "lh_hip_joint", "min", -803055986689LL, -785398163397LL, FLC_NO_PARKING_REQUIRED, FLC_TARGET_DIAGNOSTIC_OUTSIDE_URDF_LIMITS, FLC_GEOMETRY_PATH_COLLISION_FREE, FLC_GEOMETRY_JOINT_NONE, 0LL, 0, 0, FLC_GEOMETRY_RELATION_NONE, FLC_CLEARANCE_POLICY_UNRESOLVED_LOWER_BOUND, FLC_MOTION_NOT_GRANTED_OFFLINE_EVIDENCE_ONLY, 0xFFF, 0xDFF},
  // 19: lh_hip_joint:max
  {19, "lh_hip_joint:max", FLC_GEOMETRY_JOINT_LH_HIP, "lh_hip_joint", "max", 788125240354LL, 785398163397LL, FLC_NO_PARKING_REQUIRED, FLC_TARGET_DIAGNOSTIC_OUTSIDE_URDF_LIMITS, FLC_GEOMETRY_PATH_COLLISION_FREE, FLC_GEOMETRY_JOINT_NONE, 0LL, 0, 0, FLC_GEOMETRY_RELATION_NONE, FLC_CLEARANCE_POLICY_PASS, FLC_MOTION_NOT_GRANTED_OFFLINE_EVIDENCE_ONLY, 0xFFF, 0xDFF},
  // 20: lh_upper_leg_joint:min
  {20, "lh_upper_leg_joint:min", FLC_GEOMETRY_JOINT_LH_UPPER_LEG, "lh_upper_leg_joint", "min", -909889226450LL, -916297857297LL, FLC_NO_PARKING_REQUIRED, FLC_TARGET_EXECUTABLE_URDF_DOMAIN, FLC_GEOMETRY_PATH_COLLISION_FREE, FLC_GEOMETRY_JOINT_NONE, 0LL, 0, 0, FLC_GEOMETRY_RELATION_NONE, FLC_CLEARANCE_POLICY_PASS, FLC_MOTION_NOT_GRANTED_OFFLINE_EVIDENCE_ONLY, 0xFFF, 0xBFF},
  // 21: lh_upper_leg_joint:max
  {21, "lh_upper_leg_joint:max", FLC_GEOMETRY_JOINT_LH_UPPER_LEG, "lh_upper_leg_joint", "max", 2127120025868LL, 2138028333693LL, FLC_NO_PARKING_REQUIRED, FLC_TARGET_EXECUTABLE_URDF_DOMAIN, FLC_GEOMETRY_PATH_COLLISION_FREE, FLC_GEOMETRY_JOINT_NONE, 0LL, 0, 0, FLC_GEOMETRY_RELATION_NONE, FLC_CLEARANCE_POLICY_PASS, FLC_MOTION_NOT_GRANTED_OFFLINE_EVIDENCE_ONLY, 0xFFF, 0xBFF},
  // 22: lh_lower_leg_joint:min
  {22, "lh_lower_leg_joint:min", FLC_GEOMETRY_JOINT_LH_LOWER_LEG, "lh_lower_leg_joint", "min", -1606998273389LL, -1605702911835LL, FLC_PARKING_REQUIRED_1DOF, FLC_TARGET_DIAGNOSTIC_OUTSIDE_URDF_LIMITS, FLC_GEOMETRY_PATH_OBSTRUCTION, FLC_GEOMETRY_JOINT_LH_UPPER_LEG, 1628973968528LL, "lh_hip_link", "lh_lower_leg_link", FLC_GEOMETRY_RELATION_SAME_BRANCH, FLC_CLEARANCE_POLICY_UNRESOLVED_LOWER_BOUND, FLC_MOTION_NOT_GRANTED_OFFLINE_EVIDENCE_ONLY, 0xFFF, 0x3FF},
  // 23: lh_lower_leg_joint:max
  {23, "lh_lower_leg_joint:max", FLC_GEOMETRY_JOINT_LH_LOWER_LEG, "lh_lower_leg_joint", "max", 666361254258LL, 654498469498LL, FLC_NO_PARKING_REQUIRED, FLC_TARGET_DIAGNOSTIC_OUTSIDE_URDF_LIMITS, FLC_GEOMETRY_PATH_COLLISION_FREE, FLC_GEOMETRY_JOINT_NONE, 0LL, 0, 0, FLC_GEOMETRY_RELATION_NONE, FLC_CLEARANCE_POLICY_PASS, FLC_MOTION_NOT_GRANTED_OFFLINE_EVIDENCE_ONLY, 0xFFF, 0x7FF},
};
static const int FLC_ENDPOINT_GEOMETRY_PLAN_COUNT =
    (int)(sizeof(FLC_ENDPOINT_GEOMETRY_PLANS) /
          sizeof(FLC_ENDPOINT_GEOMETRY_PLANS[0]));

struct FlcGeometryDependency {
  FlcGeometryJoint prerequisiteJoint;
  FlcGeometryJoint targetJoint;
  uint8_t endpointIndex;
};

static const FlcGeometryDependency FLC_GEOMETRY_DEPENDENCIES[] = {
  {FLC_GEOMETRY_JOINT_LH_UPPER_LEG, FLC_GEOMETRY_JOINT_LF_UPPER_LEG, 3},
  {FLC_GEOMETRY_JOINT_LF_UPPER_LEG, FLC_GEOMETRY_JOINT_LF_LOWER_LEG, 4},
  {FLC_GEOMETRY_JOINT_RH_UPPER_LEG, FLC_GEOMETRY_JOINT_RF_UPPER_LEG, 9},
  {FLC_GEOMETRY_JOINT_RF_UPPER_LEG, FLC_GEOMETRY_JOINT_RF_LOWER_LEG, 10},
  {FLC_GEOMETRY_JOINT_RH_UPPER_LEG, FLC_GEOMETRY_JOINT_RH_LOWER_LEG, 16},
  {FLC_GEOMETRY_JOINT_LH_UPPER_LEG, FLC_GEOMETRY_JOINT_LH_LOWER_LEG, 22},
};
static const int FLC_GEOMETRY_DEPENDENCY_COUNT =
    (int)(sizeof(FLC_GEOMETRY_DEPENDENCIES) /
          sizeof(FLC_GEOMETRY_DEPENDENCIES[0]));

// Kahn order with canonical joint index as the deterministic tie-break.
static const FlcGeometryJoint FLC_GEOMETRY_TOPOLOGICAL_ORDER[] = {
  FLC_GEOMETRY_JOINT_LF_HIP, FLC_GEOMETRY_JOINT_RF_HIP, FLC_GEOMETRY_JOINT_RH_HIP, FLC_GEOMETRY_JOINT_RH_UPPER_LEG, FLC_GEOMETRY_JOINT_RF_UPPER_LEG, FLC_GEOMETRY_JOINT_RF_LOWER_LEG, FLC_GEOMETRY_JOINT_RH_LOWER_LEG, FLC_GEOMETRY_JOINT_LH_HIP, FLC_GEOMETRY_JOINT_LH_UPPER_LEG, FLC_GEOMETRY_JOINT_LF_UPPER_LEG, FLC_GEOMETRY_JOINT_LF_LOWER_LEG, FLC_GEOMETRY_JOINT_LH_LOWER_LEG
};

static const char *const FLC_GEOMETRY_JOINT_NAMES[] = {
  "lf_hip_joint", "lf_upper_leg_joint", "lf_lower_leg_joint", "rf_hip_joint", "rf_upper_leg_joint", "rf_lower_leg_joint", "rh_hip_joint", "rh_upper_leg_joint", "rh_lower_leg_joint", "lh_hip_joint", "lh_upper_leg_joint", "lh_lower_leg_joint"
};

inline bool flcGeometryStrEq(const char *a, const char *b) {
  if (a == 0 || b == 0) return false;
  while (*a && *b) {
    if (*a != *b) return false;
    ++a;
    ++b;
  }
  return *a == *b;
}

inline const char *flcGeometryJointName(FlcGeometryJoint joint) {
  const int index = (int)joint;
  if (index < 0 || index >= FLC_GEOMETRY_JOINT_COUNT) return 0;
  return FLC_GEOMETRY_JOINT_NAMES[index];
}

inline uint16_t flcGeometryJointMask(FlcGeometryJoint joint) {
  const int index = (int)joint;
  if (index < 0 || index >= FLC_GEOMETRY_JOINT_COUNT) return 0;
  return (uint16_t)(1U << index);
}

// NULL means INVALID/UNKNOWN. A known no-parking endpoint has an
// explicit row whose parkingOutcome is FLC_NO_PARKING_REQUIRED.
inline const FlcEndpointGeometryPlan *flcGeometryPlanFor(
    const char *jointName, const char *limitSide) {
  for (int i = 0; i < FLC_ENDPOINT_GEOMETRY_PLAN_COUNT; ++i) {
    const FlcEndpointGeometryPlan &row = FLC_ENDPOINT_GEOMETRY_PLANS[i];
    if (flcGeometryStrEq(row.jointName, jointName) &&
        flcGeometryStrEq(row.limitSide, limitSide)) return &row;
  }
  return 0;
}

inline const FlcEndpointGeometryPlan *flcGeometryPlanForEndpointId(
    const char *endpointId) {
  for (int i = 0; i < FLC_ENDPOINT_GEOMETRY_PLAN_COUNT; ++i) {
    if (flcGeometryStrEq(FLC_ENDPOINT_GEOMETRY_PLANS[i].endpointId,
                         endpointId)) return &FLC_ENDPOINT_GEOMETRY_PLANS[i];
  }
  return 0;
}

inline int flcGeometryDependencyCountFor(FlcGeometryJoint target) {
  int count = 0;
  for (int i = 0; i < FLC_GEOMETRY_DEPENDENCY_COUNT; ++i) {
    if (FLC_GEOMETRY_DEPENDENCIES[i].targetJoint == target) ++count;
  }
  return count;
}

inline FlcGeometryJoint flcGeometryDependencyAt(
    FlcGeometryJoint target, int ordinal) {
  if (ordinal < 0) return FLC_GEOMETRY_JOINT_NONE;
  for (int i = 0; i < FLC_GEOMETRY_DEPENDENCY_COUNT; ++i) {
    if (FLC_GEOMETRY_DEPENDENCIES[i].targetJoint != target) continue;
    if (ordinal-- == 0) return FLC_GEOMETRY_DEPENDENCIES[i].prerequisiteJoint;
  }
  return FLC_GEOMETRY_JOINT_NONE;
}

inline bool flcGeometryBuildTopologicalOrder(
    FlcGeometryJoint *out, int capacity) {
  if (out == 0 || capacity < FLC_GEOMETRY_JOINT_COUNT) return false;
  uint8_t indegree[FLC_GEOMETRY_JOINT_COUNT] = {0};
  bool emitted[FLC_GEOMETRY_JOINT_COUNT] = {false};
  for (int i = 0; i < FLC_GEOMETRY_DEPENDENCY_COUNT; ++i) {
    const int prerequisite = (int)FLC_GEOMETRY_DEPENDENCIES[i].prerequisiteJoint;
    const int target = (int)FLC_GEOMETRY_DEPENDENCIES[i].targetJoint;
    if (prerequisite < 0 || prerequisite >= FLC_GEOMETRY_JOINT_COUNT ||
        target < 0 || target >= FLC_GEOMETRY_JOINT_COUNT ||
        prerequisite == target) return false;
    ++indegree[target];
  }
  for (int outputIndex = 0; outputIndex < FLC_GEOMETRY_JOINT_COUNT;
       ++outputIndex) {
    int ready = -1;
    for (int joint = 0; joint < FLC_GEOMETRY_JOINT_COUNT; ++joint) {
      if (!emitted[joint] && indegree[joint] == 0) { ready = joint; break; }
    }
    if (ready < 0) return false;
    emitted[ready] = true;
    out[outputIndex] = (FlcGeometryJoint)ready;
    for (int i = 0; i < FLC_GEOMETRY_DEPENDENCY_COUNT; ++i) {
      if ((int)FLC_GEOMETRY_DEPENDENCIES[i].prerequisiteJoint == ready) {
        const int target = (int)FLC_GEOMETRY_DEPENDENCIES[i].targetJoint;
        if (indegree[target] == 0) return false;
        --indegree[target];
      }
    }
  }
  return true;
}

inline bool flcGeometryDependencyGraphAcyclic() {
  FlcGeometryJoint order[FLC_GEOMETRY_JOINT_COUNT];
  return flcGeometryBuildTopologicalOrder(order, FLC_GEOMETRY_JOINT_COUNT);
}

inline bool flcGeometryGeneratedTopologicalOrderValid() {
  FlcGeometryJoint derived[FLC_GEOMETRY_JOINT_COUNT];
  if (!flcGeometryBuildTopologicalOrder(derived, FLC_GEOMETRY_JOINT_COUNT))
    return false;
  for (int i = 0; i < FLC_GEOMETRY_JOINT_COUNT; ++i) {
    if (derived[i] != FLC_GEOMETRY_TOPOLOGICAL_ORDER[i]) return false;
  }
  return true;
}

#endif  // FLC_LEG_PLAN_H
