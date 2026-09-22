// Offline host tests for the calibration bootstrap geometry contract
// (src/actuator/CalibrationGeometryProfile.* + the geometry-authorised half of
// src/actuator/ActuatorWritePolicy.*).
//
// Links the REAL generated profile data, the REAL policy, the REAL arbiter and
// the REAL calibration domain model. The data under test is therefore the
// exact table the firmware would carry, reduced by
// 06_Software/Matdog_Core/calibration/matdog_calibration_geometry_export.py
// from the canonical Geometry Compiler V5 bundle
//   2026-08-11_131818_MATDOG_GEOMETRY_V5_REMEDIATION_BENCHMARK_D_W4
//
// NO TEST HERE IS HARDWARE VALIDATION. Every case proves a DECISION about a
// geometry MODEL. There is no transport in the tree that could act on one, no
// q0 has been measured on the current installation, and hardware motion
// remains compile-time blocked.

#include <cstdio>
#include <cstring>
#include <initializer_list>

#include "../../src/actuator/ActuatorWritePolicy.h"
#include "../../src/actuator/CalibrationGeometryProfileData.h"

using namespace matdog;
using namespace matdog::actuator;
using matdog::calibration::CalibrationOrigin;
using matdog::calibration::ContactSide;
using matdog::calibration::EvidenceState;
using matdog::calibration::JointIdentity;
using matdog::calibration::JointKind;
using matdog::calibration::Leg;
using matdog::core::ActuatorAuthority;
using matdog::core::ActuatorAuthorityArbiter;
using matdog::core::AuthorityClearReason;
using matdog::core::AuthorityLease;
using matdog::core::AuthorityResult;
using matdog::core::OperatingMode;

static int g_checks = 0;
static int g_failures = 0;
static const char* g_case = "";

#define CHECK(cond)                                                              \
  do {                                                                           \
    ++g_checks;                                                                  \
    if (!(cond)) {                                                               \
      ++g_failures;                                                              \
      std::printf("  FAIL [%s] %s:%d: %s\n", g_case, __FILE__, __LINE__, #cond); \
    }                                                                            \
  } while (0)

#define CHECK_EQ(actual, expected)                                        \
  do {                                                                    \
    ++g_checks;                                                           \
    const long long a_ = (long long)(actual);                             \
    const long long e_ = (long long)(expected);                           \
    if (a_ != e_) {                                                       \
      ++g_failures;                                                       \
      std::printf("  FAIL [%s] %s:%d: %s == %lld, expected %lld\n",       \
                  g_case, __FILE__, __LINE__, #actual, a_, e_);           \
    }                                                                     \
  } while (0)

#define CHECK_DECISION(actual, expected)                                    \
  do {                                                                      \
    ++g_checks;                                                             \
    const WriteDecision a_ = (actual);                                      \
    const WriteDecision e_ = (expected);                                    \
    if (a_ != e_) {                                                         \
      ++g_failures;                                                         \
      std::printf("  FAIL [%s] %s:%d: %s == %s, expected %s\n", g_case,     \
                  __FILE__, __LINE__, #actual, toString(a_), toString(e_)); \
    }                                                                       \
  } while (0)

// ---------------------------------------------------------------------------
// Fixtures
// ---------------------------------------------------------------------------

static CalibrationGeometryProfile boundProfile() {
  CalibrationGeometryProfile profile;
  profile.bind(&geometry_data::kProvenance, geometry_data::kJoints, geometry_data::kJointCount,
               geometry_data::kEndpoints, geometry_data::kEndpointCount);
  return profile;
}

static JointIdentity identity(Leg leg, JointKind kind, const char* unit) {
  JointIdentity id{};
  id.leg = leg;
  id.joint = kind;
  calibration::setPhysicalUnit(&id, unit);
  return id;
}

// The current installation, keyed exactly as MATDOG_SERVO_ALLOCATION.yaml
// keys it after the 2026-08-27 campaign.
static JointIdentity lfUpper() { return identity(Leg::LF, JointKind::UPPER, "ELR01"); }
static JointIdentity lhUpper() { return identity(Leg::LH, JointKind::UPPER, "M42"); }
static JointIdentity rhUpper() { return identity(Leg::RH, JointKind::UPPER, "ELR02"); }
static JointIdentity lfLower() { return identity(Leg::LF, JointKind::LOWER, "M33"); }

// A transform with operational provenance. Nothing in the repository produces
// one today - no q0 has been captured on the current installation - so this
// exists only to prove the gated paths are reachable at all and that the gate
// is real rather than vacuous.
static JointTransform acceptedTransform(JointIdentity id, uint16_t q0_tick, int8_t direction) {
  JointTransform t{};
  t.identity = id;
  t.geometry = geometryProvenanceTag(geometry_data::kProvenance);
  t.state = EvidenceState::PROMOTED;
  t.origin = CalibrationOrigin::LIVE_SESSION;
  t.q0_tick = q0_tick;
  t.direction = direction;
  t.present = true;
  return t;
}

static CalibrationBootstrapContext liveSession() {
  CalibrationBootstrapContext ctx{};
  ctx.session_active = true;
  ctx.origin = CalibrationOrigin::LIVE_SESSION;
  return ctx;
}

struct Harness {
  ActuatorAuthorityArbiter arbiter;
  CalibrationGeometryProfile profile;
  SafeActuatorPolicy policy;
  AuthorityLease lease{};

  Harness() {
    arbiter.reset(AuthorityClearReason::BOOT);
    profile = boundProfile();
    policy.begin(&arbiter);
    policy.bindGeometry(&profile, &geometry_data::kProvenance);
    policy.setBootstrapContext(liveSession());
    const AuthorityResult granted =
        arbiter.request(ActuatorAuthority::CALIBRATION, OperatingMode::MAINTENANCE, &lease);
    (void)granted;
  }

  WriteDecision plan(const ActuatorCommand& command) {
    ActuatorTransaction txn{};
    const WriteDecision decision =
        policy.plan(command, lease, OperatingMode::MAINTENANCE, &txn);
    policy.abort(&txn);
    return decision;
  }
};

static ActuatorCommand probe(JointIdentity joint, Leg leg, JointKind kind, ContactSide side,
                             MicroRad target) {
  ActuatorCommand c{};
  c.operation = ActuatorOperation::CALIBRATION_CONTACT_PROBE;
  c.joint = joint;
  c.endpoint_leg = leg;
  c.endpoint_joint = kind;
  c.endpoint_side = side;
  c.target_urad = target;
  return c;
}

static ActuatorCommand auxiliary(JointIdentity moving, Leg leg, JointKind kind, ContactSide side,
                                 MicroRad target) {
  ActuatorCommand c{};
  c.operation = ActuatorOperation::CALIBRATION_AUXILIARY_MOVE;
  c.joint = moving;
  c.endpoint_leg = leg;
  c.endpoint_joint = kind;
  c.endpoint_side = side;
  c.target_urad = target;
  return c;
}

static ActuatorCommand directionVerify(JointIdentity joint, int32_t delta_ticks) {
  ActuatorCommand c{};
  c.operation = ActuatorOperation::DIRECTION_VERIFY;
  c.joint = joint;
  c.delta_ticks = delta_ticks;
  return c;
}

// ---------------------------------------------------------------------------
// The compiled model itself
// ---------------------------------------------------------------------------

static void test_the_profile_is_the_canonical_v5_bundle() {
  g_case = "canonical bundle shape";
  CHECK_EQ(geometry_data::kJointCount, 12);
  CHECK_EQ(geometry_data::kEndpointCount, 24);

  int executable = 0, diagnostic = 0, parking = 0, direct = 0, pass = 0, unresolved = 0;
  for (uint8_t i = 0; i < geometry_data::kEndpointCount; ++i) {
    const GeometryEndpointRecord& e = geometry_data::kEndpoints[i];
    (e.domain == TargetDomain::EXECUTABLE_URDF_DOMAIN ? executable : diagnostic)++;
    (e.parking == ParkingOutcome::FEASIBLE_1DOF_PLAN_FOUND ? parking : direct)++;
    if (e.clearance == ClearancePolicyResult::PASS) ++pass;
    if (e.clearance == ClearancePolicyResult::UNRESOLVED_LOWER_BOUND_BELOW_THRESHOLD) {
      ++unresolved;
      // The repository's own claim, re-proven from the data rather than
      // quoted: every UNRESOLVED verdict lands on a DIAGNOSTIC endpoint, so
      // no executable target rests on unresolved clearance evidence.
      CHECK(e.domain == TargetDomain::DIAGNOSTIC_GEOMETRY_OUTSIDE_URDF_LIMITS);
    }
  }
  CHECK_EQ(executable, 8);
  CHECK_EQ(diagnostic, 16);
  CHECK_EQ(parking, 6);
  CHECK_EQ(direct, 18);
  CHECK_EQ(pass, 16);
  CHECK_EQ(unresolved, 8);

  // The eight executable endpoints are exactly the upper-leg ones: for hips
  // and lower legs the mechanism contacts BEYOND the declared URDF limit, so
  // their endpoints are measurements and not places to command.
  for (uint8_t i = 0; i < geometry_data::kEndpointCount; ++i) {
    const GeometryEndpointRecord& e = geometry_data::kEndpoints[i];
    if (e.domain == TargetDomain::EXECUTABLE_URDF_DOMAIN) {
      CHECK(e.joint == JointKind::UPPER);
    }
  }
}

static void test_the_six_parking_plans_are_exactly_the_compilers() {
  g_case = "six parking plans";
  struct Expected {
    Leg leg;
    JointKind joint;
    ContactSide side;
    Leg aux_leg;
    JointKind aux_joint;
    MicroRad aux_target;
  };
  // Copied from the canonical PATH_PARKING artifact. Two are cross-branch
  // (a front foot meeting the rear foot on the same side, parked by lifting
  // the REAR upper leg) and four are same-branch (a leg's own hip meeting its
  // own lower leg, parked by that leg's own upper joint).
  const Expected expected[] = {
      {Leg::LF, JointKind::UPPER, ContactSide::MAX_SIDE, Leg::LH, JointKind::UPPER, 610865},
      {Leg::RF, JointKind::UPPER, ContactSide::MAX_SIDE, Leg::RH, JointKind::UPPER, 610865},
      {Leg::LF, JointKind::LOWER, ContactSide::MIN_SIDE, Leg::LF, JointKind::UPPER, 1119920},
      {Leg::RF, JointKind::LOWER, ContactSide::MIN_SIDE, Leg::RF, JointKind::UPPER, 1119920},
      {Leg::RH, JointKind::LOWER, ContactSide::MIN_SIDE, Leg::RH, JointKind::UPPER, 1628974},
      {Leg::LH, JointKind::LOWER, ContactSide::MIN_SIDE, Leg::LH, JointKind::UPPER, 1628974},
  };

  CalibrationGeometryProfile profile = boundProfile();
  int matched = 0;
  for (const Expected& want : expected) {
    const GeometryEndpointRecord* got = profile.findEndpoint(want.leg, want.joint, want.side);
    CHECK(got != nullptr);
    if (got == nullptr) continue;
    CHECK(got->parking == ParkingOutcome::FEASIBLE_1DOF_PLAN_FOUND);
    CHECK(got->has_auxiliary);
    CHECK(got->auxiliary_leg == want.aux_leg);
    CHECK(got->auxiliary_joint == want.aux_joint);
    CHECK_EQ(got->auxiliary_target, want.aux_target);
    ++matched;
  }
  CHECK_EQ(matched, 6);

  // Every other endpoint is a validated direct path and carries no auxiliary.
  for (uint8_t i = 0; i < geometry_data::kEndpointCount; ++i) {
    const GeometryEndpointRecord& e = geometry_data::kEndpoints[i];
    if (e.parking == ParkingOutcome::NOT_NEEDED) CHECK(!e.has_auxiliary);
  }
}

static void test_the_legacy_hardcoded_prerequisite_poses_are_gone() {
  g_case = "no 30/50/85/90 legacy poses";
  // The superseded calibration YAML and the LF V25 era carried hardcoded
  // 30/50/85/90 degree prerequisites. The current compiler found 35.000,
  // 64.1667 and 93.3333 degrees instead. None of the legacy values may appear
  // as a parked pose, and the compiler's own note says the empty default
  // context proves the legacy prerequisites were never core truth.
  const MicroRad legacy[] = {523599, 872665, 1483530, 1570796};  // 30, 50, 85, 90 degrees
  for (uint8_t i = 0; i < geometry_data::kEndpointCount; ++i) {
    const GeometryEndpointRecord& e = geometry_data::kEndpoints[i];
    if (!e.has_auxiliary) continue;
    for (MicroRad stale : legacy) {
      const MicroRad delta =
          e.auxiliary_target > stale ? e.auxiliary_target - stale : stale - e.auxiliary_target;
      // 1 mrad is ten times the compiler's own bisection resolution: a real
      // match would be far closer than this.
      CHECK(delta > 1000);
    }
  }
}

// ---------------------------------------------------------------------------
// Identity
// ---------------------------------------------------------------------------

static void test_joints_are_keyed_by_current_physical_unit() {
  g_case = "current physical-unit identity";
  CalibrationGeometryProfile profile = boundProfile();

  struct Expected {
    Leg leg;
    JointKind joint;
    const char* unit;
    uint8_t bus_id;
  };
  const Expected expected[] = {
      {Leg::LF, JointKind::HIP, "M22", 13},   {Leg::LF, JointKind::UPPER, "ELR01", 12},
      {Leg::LF, JointKind::LOWER, "M33", 11}, {Leg::RF, JointKind::HIP, "NEW01", 23},
      {Leg::RF, JointKind::UPPER, "ELR03", 22}, {Leg::RF, JointKind::LOWER, "NEW03", 21},
      {Leg::RH, JointKind::HIP, "NEW06", 33}, {Leg::RH, JointKind::UPPER, "ELR02", 32},
      {Leg::RH, JointKind::LOWER, "NEW05", 31}, {Leg::LH, JointKind::HIP, "M43", 43},
      {Leg::LH, JointKind::UPPER, "M42", 42}, {Leg::LH, JointKind::LOWER, "M41", 41},
  };
  for (const Expected& want : expected) {
    const GeometryJointRecord* got = profile.findJoint(identity(want.leg, want.joint, want.unit));
    CHECK(got != nullptr);
    if (got != nullptr) {
      CHECK_EQ(got->bus_id, want.bus_id);
      // The URDF's motorId and the allocation's bus id agree for all twelve -
      // the exporter refuses to emit a table where they do not.
      CHECK(got->urdf_motor_direction == 1 || got->urdf_motor_direction == -1);
      // PositionOffset is 0 on every unit, so the provisioned raw centre is
      // within one tick of 2048. A PRIOR about mounting, never a q0.
      CHECK(got->provisioned_center_raw >= 2047 && got->provisioned_center_raw <= 2049);
    }
  }
}

static void test_historical_lf_v25_identity_cannot_bind_a_current_joint() {
  g_case = "historical identity rejected";
  CalibrationGeometryProfile profile = boundProfile();

  // M11 is the LF V25 archive's own unit label. On the current robot it is
  // NECK_PITCH on bus 52 - it is not in a leg at all. Presenting it as an LF
  // joint must find nothing, however plausible the slot looks.
  CHECK(profile.findJoint(identity(Leg::LF, JointKind::LOWER, "M11")) == nullptr);
  CHECK(profile.findJoint(identity(Leg::LF, JointKind::HIP, "M11")) == nullptr);

  // The slot alone is not an identity either: bus 11 is still "LF lower", but
  // the unit answering there is M33 and only M33 matches.
  CHECK(profile.findJoint(identity(Leg::LF, JointKind::LOWER, "M13")) == nullptr);
  CHECK(profile.findJoint(identity(Leg::LF, JointKind::LOWER, "M33")) != nullptr);

  // An anonymous slot matches nothing at all.
  JointIdentity anonymous{};
  anonymous.leg = Leg::LF;
  anonymous.joint = JointKind::LOWER;
  CHECK(profile.findJoint(anonymous) == nullptr);

  // And the right unit in the wrong slot is still the wrong joint.
  CHECK(profile.findJoint(identity(Leg::RF, JointKind::LOWER, "M33")) == nullptr);
}

// ---------------------------------------------------------------------------
// Provenance
// ---------------------------------------------------------------------------

static void test_geometry_provenance_must_match_exactly() {
  g_case = "provenance";
  CalibrationGeometryProfile profile = boundProfile();
  CHECK(profile.bound());
  CHECK(profile.provenanceMatches(geometry_data::kProvenance));

  // Each of the six hashes independently. A profile compiled from a different
  // URDF, different meshes, a different endpoint or parking run, a different
  // safety policy, or a different servo allocation is a profile about a
  // different robot.
  const struct {
    const char* label;
    size_t offset;
  } fields[] = {
      {"urdf", offsetof(GeometryProvenance, urdf_sha256)},
      {"mesh manifest", offsetof(GeometryProvenance, mesh_manifest_sha256)},
      {"endpoint semantic", offsetof(GeometryProvenance, endpoint_semantic_sha256)},
      {"parking semantic", offsetof(GeometryProvenance, parking_semantic_sha256)},
      {"safety policy semantic", offsetof(GeometryProvenance, safety_policy_semantic_sha256)},
      {"allocation", offsetof(GeometryProvenance, allocation_sha256)},
  };
  for (const auto& field : fields) {
    GeometryProvenance tampered = geometry_data::kProvenance;
    char* hash = reinterpret_cast<char*>(&tampered) + field.offset;
    hash[0] = (hash[0] == 'a') ? 'b' : 'a';
    CHECK(!profile.provenanceMatches(tampered));
    CHECK(!sameProvenance(geometry_data::kProvenance, tampered));
  }

  // An unbound profile matches nothing, including itself.
  CalibrationGeometryProfile unbound;
  CHECK(!unbound.bound());
  CHECK(!unbound.provenanceMatches(geometry_data::kProvenance));
  CHECK(unbound.findJoint(lfUpper()) == nullptr);
  CHECK(unbound.findEndpoint(Leg::LF, JointKind::UPPER, ContactSide::MAX_SIDE) == nullptr);

  // Partial binding is no binding.
  CalibrationGeometryProfile partial;
  partial.bind(&geometry_data::kProvenance, geometry_data::kJoints, geometry_data::kJointCount,
               nullptr, geometry_data::kEndpointCount);
  CHECK(!partial.bound());
  partial.bind(nullptr, geometry_data::kJoints, geometry_data::kJointCount,
               geometry_data::kEndpoints, geometry_data::kEndpointCount);
  CHECK(!partial.bound());
}

static void test_a_stale_profile_is_refused_by_the_policy() {
  g_case = "stale profile refused";
  Harness h;
  // A profile the policy cannot recognise: the geometry is present, but not
  // the geometry this build was made for.
  GeometryProvenance other = geometry_data::kProvenance;
  other.urdf_sha256[0] = (other.urdf_sha256[0] == 'a') ? 'b' : 'a';
  h.policy.bindGeometry(&h.profile, &other);
  CHECK_DECISION(h.plan(directionVerify(lfUpper(), 16)),
                 WriteDecision::REJECT_GEOMETRY_PROVENANCE);

  // No geometry at all is a refusal, not a free pass.
  h.policy.bindGeometry(nullptr, nullptr);
  CHECK_DECISION(h.plan(directionVerify(lfUpper(), 16)),
                 WriteDecision::REJECT_NO_GEOMETRY_PROFILE);
  // Half a binding is no binding.
  h.policy.bindGeometry(&h.profile, nullptr);
  CHECK_DECISION(h.plan(directionVerify(lfUpper(), 16)),
                 WriteDecision::REJECT_NO_GEOMETRY_PROFILE);
}

// ---------------------------------------------------------------------------
// The bootstrap envelope
// ---------------------------------------------------------------------------

static void test_direction_verify_needs_a_live_session_and_a_budget() {
  g_case = "direction verify preconditions";
  Harness h;

  // A live session but no approved excursion: geometry says what is clear,
  // the operator says how much may be used, and the operator has said nothing.
  CHECK_DECISION(h.plan(directionVerify(lfUpper(), 16)),
                 WriteDecision::REJECT_NO_ENVELOPE_BUDGET);

  // No session at all.
  h.policy.setBootstrapContext(CalibrationBootstrapContext{});
  CHECK_DECISION(h.plan(directionVerify(lfUpper(), 16)),
                 WriteDecision::REJECT_NO_CALIBRATION_SESSION);

  // A replay session authorises nothing physical, however complete it looks.
  CalibrationBootstrapContext replay = liveSession();
  replay.origin = CalibrationOrigin::HISTORICAL_REPLAY;
  replay.direction_verify_tick_budget = 64;
  h.policy.setBootstrapContext(replay);
  CHECK_DECISION(h.plan(directionVerify(lfUpper(), 16)),
                 WriteDecision::REJECT_NO_CALIBRATION_SESSION);
}

static void test_direction_verify_envelope_is_symmetric_and_bounded() {
  g_case = "direction verify envelope";
  Harness h;
  CalibrationBootstrapContext ctx = liveSession();
  ctx.direction_verify_tick_budget = 64;  // ~5.6 degrees
  h.policy.setBootstrapContext(ctx);

  // Symmetric: the same magnitude is authorised in both directions, because
  // the sign of the joint is exactly what this move exists to discover.
  CHECK_DECISION(h.plan(directionVerify(lfUpper(), 32)), WriteDecision::ACCEPT);
  CHECK_DECISION(h.plan(directionVerify(lfUpper(), -32)), WriteDecision::ACCEPT);
  CHECK_DECISION(h.plan(directionVerify(lfUpper(), 64)), WriteDecision::ACCEPT);
  CHECK_DECISION(h.plan(directionVerify(lfUpper(), -64)), WriteDecision::ACCEPT);

  // Beyond the session budget.
  CHECK_DECISION(h.plan(directionVerify(lfUpper(), 65)),
                 WriteDecision::REJECT_OUTSIDE_BOOTSTRAP_ENVELOPE);
  CHECK_DECISION(h.plan(directionVerify(lfUpper(), -65)),
                 WriteDecision::REJECT_OUTSIDE_BOOTSTRAP_ENVELOPE);
  // A zero excursion verifies nothing, so authorising it would let "the move
  // was allowed" mean "no move happened".
  CHECK_DECISION(h.plan(directionVerify(lfUpper(), 0)),
                 WriteDecision::REJECT_OUTSIDE_BOOTSTRAP_ENVELOPE);

  // A budget larger than the geometry does not enlarge the geometry. The
  // tightest compiled half-span is the lower leg's 38.18 degrees; 2048 ticks
  // is 180 degrees and must be refused on geometry alone.
  ctx.direction_verify_tick_budget = 4096;
  h.policy.setBootstrapContext(ctx);
  CHECK_DECISION(h.plan(directionVerify(lfLower(), 2048)),
                 WriteDecision::REJECT_OUTSIDE_BOOTSTRAP_ENVELOPE);
  CHECK_DECISION(h.plan(directionVerify(lfLower(), -2048)),
                 WriteDecision::REJECT_OUTSIDE_BOOTSTRAP_ENVELOPE);

  // The envelope boundary itself, straight from the profile.
  CalibrationGeometryProfile profile = boundProfile();
  const GeometryJointRecord* lower = profile.findJoint(lfLower());
  CHECK(lower != nullptr);
  if (lower != nullptr) {
    // 38.1758 degrees, the min over BOTH sides for this joint.
    CHECK_EQ(lower->clear_half_span, 666293);
    int32_t inside = 0;
    while (ticksToMicroRadMagnitude(inside + 1) <= lower->clear_half_span) ++inside;
    CHECK(profile.withinDirectionVerifyEnvelope(lfLower(), inside));
    CHECK(profile.withinDirectionVerifyEnvelope(lfLower(), -inside));
    CHECK(!profile.withinDirectionVerifyEnvelope(lfLower(), inside + 1));
    CHECK(!profile.withinDirectionVerifyEnvelope(lfLower(), -(inside + 1)));
  }

  // An unknown joint has no envelope.
  CHECK(!profile.withinDirectionVerifyEnvelope(identity(Leg::LF, JointKind::LOWER, "M11"), 8));
}

static void test_tick_to_microrad_conversion_never_rounds_in_our_favour() {
  g_case = "conservative tick conversion";
  CHECK_EQ(ticksToMicroRadMagnitude(0), 0);
  // 4096 ticks is one revolution; the conversion rounds UP, so it is at least
  // 2*pi rad and never less.
  CHECK(ticksToMicroRadMagnitude(4096) >= 6283185);
  CHECK(ticksToMicroRadMagnitude(1024) >= 1570796);
  // Sign-independent: a magnitude, not a value.
  for (int32_t ticks : {1, 7, 64, 1023, 4095}) {
    CHECK_EQ(ticksToMicroRadMagnitude(ticks), ticksToMicroRadMagnitude(-ticks));
    // Never an under-estimate of the true excursion.
    const double exact = (double)ticks * 6.283185307179586 / 4096.0 * 1e6;
    CHECK((double)ticksToMicroRadMagnitude(ticks) >= exact);
  }
}

static void test_direction_verify_does_not_need_a_transform() {
  g_case = "direction verify precedes the transform";
  Harness h;
  CalibrationBootstrapContext ctx = liveSession();
  ctx.direction_verify_tick_budget = 32;
  h.policy.setBootstrapContext(ctx);

  // The transform table is empty - no q0 and no direction exist for the
  // current installation - and this is precisely the move that measures the
  // missing half. Requiring a transform here would make the bootstrap
  // circular.
  CHECK(h.policy.transforms().empty());
  CHECK_DECISION(h.plan(directionVerify(lfUpper(), 16)), WriteDecision::ACCEPT);
}

// ---------------------------------------------------------------------------
// Endpoint plans
// ---------------------------------------------------------------------------

static void test_a_diagnostic_endpoint_can_never_become_executable() {
  g_case = "diagnostic != executable";
  Harness h;
  h.policy.transforms().admit(acceptedTransform(lfLower(), 2048, 1));

  // lf_lower:min is a real geometric contact at -92.074 degrees - 0.074
  // degrees BEYOND the declared URDF limit. It is evidence about where the
  // mechanism stops, not a place the robot may be commanded to.
  CalibrationGeometryProfile profile = boundProfile();
  const GeometryEndpointRecord* diagnostic =
      profile.findEndpoint(Leg::LF, JointKind::LOWER, ContactSide::MIN_SIDE);
  CHECK(diagnostic != nullptr);
  if (diagnostic != nullptr) {
    CHECK(diagnostic->domain == TargetDomain::DIAGNOSTIC_GEOMETRY_OUTSIDE_URDF_LIMITS);
    CHECK(!isExecutable(*diagnostic));
  }
  CHECK_DECISION(h.plan(probe(lfLower(), Leg::LF, JointKind::LOWER, ContactSide::MIN_SIDE,
                              -1600000)),
                 WriteDecision::REJECT_ENDPOINT_NOT_EXECUTABLE);

  // Every diagnostic endpoint, exhaustively - including the eight whose
  // clearance policy verdict is a clean PASS. A clean clearance on a
  // diagnostic endpoint is still not a permission.
  int refused = 0;
  for (uint8_t i = 0; i < geometry_data::kEndpointCount; ++i) {
    const GeometryEndpointRecord& e = geometry_data::kEndpoints[i];
    if (e.domain != TargetDomain::DIAGNOSTIC_GEOMETRY_OUTSIDE_URDF_LIMITS) continue;
    CHECK(!isExecutable(e));
    ++refused;
  }
  CHECK_EQ(refused, 16);

  // And an UNRESOLVED clearance blocks an otherwise executable endpoint too.
  GeometryEndpointRecord hypothetical =
      *profile.findEndpoint(Leg::LF, JointKind::UPPER, ContactSide::MIN_SIDE);
  CHECK(isExecutable(hypothetical));
  hypothetical.clearance = ClearancePolicyResult::UNRESOLVED_LOWER_BOUND_BELOW_THRESHOLD;
  CHECK(!isExecutable(hypothetical));
  hypothetical.clearance = ClearancePolicyResult::FAIL;
  CHECK(!isExecutable(hypothetical));
}

static void test_a_direct_path_is_used_only_as_the_compiler_validated_it() {
  g_case = "direct path";
  Harness h;
  h.policy.transforms().admit(acceptedTransform(lfUpper(), 2048, 1));

  // lf_upper:min is EXECUTABLE and NOT_NEEDED: the compiler validated the
  // direct q=0 -> target path with every other joint at q=0.
  const MicroRad target = -900000;  // inside [-916298, contact -910118]
  CHECK_DECISION(h.plan(probe(lfUpper(), Leg::LF, JointKind::UPPER, ContactSide::MIN_SIDE,
                              target)),
                 WriteDecision::ACCEPT);

  // With something parked, that validation no longer describes the robot.
  // This is the mirror image of requiring parking when the plan is obstructed,
  // and just as necessary.
  CalibrationBootstrapContext parked = liveSession();
  parked.auxiliary_parked = true;
  parked.parked_leg = Leg::LF;
  parked.parked_joint = JointKind::LOWER;
  parked.parked_side = ContactSide::MIN_SIDE;
  h.policy.setBootstrapContext(parked);
  CHECK_DECISION(h.plan(probe(lfUpper(), Leg::LF, JointKind::UPPER, ContactSide::MIN_SIDE,
                              target)),
                 WriteDecision::REJECT_UNEXPECTED_PARKING);
}

static void test_an_obstructed_plan_requires_its_parking_first() {
  g_case = "parking required";
  Harness h;
  h.policy.transforms().admit(acceptedTransform(lfUpper(), 2048, 1));
  h.policy.transforms().admit(acceptedTransform(lhUpper(), 2048, 1));

  // lf_upper:max is EXECUTABLE but obstructed: the LF foot meets the LH foot.
  // The compiler's plan lifts the LH upper leg to 35 degrees first.
  const MicroRad target = 2100000;  // inside [.., contact 2127120]
  CHECK_DECISION(h.plan(probe(lfUpper(), Leg::LF, JointKind::UPPER, ContactSide::MAX_SIDE,
                              target)),
                 WriteDecision::REJECT_PARKING_REQUIRED);

  // Parking a DIFFERENT endpoint's auxiliary does not satisfy this one.
  CalibrationBootstrapContext wrong = liveSession();
  wrong.auxiliary_parked = true;
  wrong.parked_leg = Leg::RF;
  wrong.parked_joint = JointKind::UPPER;
  wrong.parked_side = ContactSide::MAX_SIDE;
  h.policy.setBootstrapContext(wrong);
  CHECK_DECISION(h.plan(probe(lfUpper(), Leg::LF, JointKind::UPPER, ContactSide::MAX_SIDE,
                              target)),
                 WriteDecision::REJECT_PARKING_REQUIRED);

  // Parked as the plan says, and the probe becomes authorisable.
  CalibrationBootstrapContext right = liveSession();
  right.auxiliary_parked = true;
  right.parked_leg = Leg::LF;
  right.parked_joint = JointKind::UPPER;
  right.parked_side = ContactSide::MAX_SIDE;
  h.policy.setBootstrapContext(right);
  CHECK_DECISION(h.plan(probe(lfUpper(), Leg::LF, JointKind::UPPER, ContactSide::MAX_SIDE,
                              target)),
                 WriteDecision::ACCEPT);
}

static void test_an_auxiliary_move_must_be_the_compilers_plan() {
  g_case = "auxiliary move";
  Harness h;
  h.policy.transforms().admit(acceptedTransform(lhUpper(), 2048, 1));
  h.policy.transforms().admit(acceptedTransform(rhUpper(), 2048, -1));

  // The LF upper:max plan parks the LH upper leg at exactly 0.610865 rad.
  CHECK_DECISION(h.plan(auxiliary(lhUpper(), Leg::LF, JointKind::UPPER, ContactSide::MAX_SIDE,
                                  610865)),
                 WriteDecision::ACCEPT);
  // Unparking back to q=0 is the same plan run backwards.
  CHECK_DECISION(h.plan(auxiliary(lhUpper(), Leg::LF, JointKind::UPPER, ContactSide::MAX_SIDE, 0)),
                 WriteDecision::ACCEPT);
  // Anything between is a pose the compiler never validated.
  CHECK_DECISION(h.plan(auxiliary(lhUpper(), Leg::LF, JointKind::UPPER, ContactSide::MAX_SIDE,
                                  500000)),
                 WriteDecision::REJECT_AUXILIARY_TARGET);
  CHECK_DECISION(h.plan(auxiliary(lhUpper(), Leg::LF, JointKind::UPPER, ContactSide::MAX_SIDE,
                                  610866)),
                 WriteDecision::REJECT_AUXILIARY_TARGET);

  // The wrong joint, even a plausible one: the RH upper leg parks the RF
  // plan, never the LF one.
  CHECK_DECISION(h.plan(auxiliary(rhUpper(), Leg::LF, JointKind::UPPER, ContactSide::MAX_SIDE,
                                  610865)),
                 WriteDecision::REJECT_WRONG_AUXILIARY_JOINT);

  // An endpoint whose plan needs no parking has no auxiliary to move.
  CHECK_DECISION(h.plan(auxiliary(lhUpper(), Leg::LF, JointKind::UPPER, ContactSide::MIN_SIDE,
                                  610865)),
                 WriteDecision::REJECT_UNEXPECTED_PARKING);
}

static void test_a_probe_must_name_its_own_endpoint_and_stay_inside_it() {
  g_case = "probe target bounds";
  Harness h;
  h.policy.transforms().admit(acceptedTransform(lfUpper(), 2048, 1));

  // Moving a joint that is not the endpoint's joint is not this plan's probe.
  CHECK_DECISION(h.plan(probe(lfLower(), Leg::LF, JointKind::UPPER, ContactSide::MIN_SIDE,
                              -900000)),
                 WriteDecision::REJECT_NO_ENDPOINT_PLAN);

  // Outside the declared URDF domain.
  CHECK_DECISION(h.plan(probe(lfUpper(), Leg::LF, JointKind::UPPER, ContactSide::MIN_SIDE,
                              -1000000)),
                 WriteDecision::REJECT_TARGET_OUTSIDE_URDF_LIMITS);
  // Past the geometric contact is into the mechanism, even while still inside
  // the URDF limit.
  CalibrationGeometryProfile profile = boundProfile();
  const GeometryEndpointRecord* endpoint =
      profile.findEndpoint(Leg::LF, JointKind::UPPER, ContactSide::MIN_SIDE);
  CHECK(endpoint != nullptr);
  if (endpoint != nullptr) {
    CHECK(endpoint->contact > endpoint->declared_limit);  // -910118 > -916298
    CHECK_DECISION(h.plan(probe(lfUpper(), Leg::LF, JointKind::UPPER, ContactSide::MIN_SIDE,
                                endpoint->contact - 1)),
                   WriteDecision::REJECT_TARGET_OUTSIDE_URDF_LIMITS);
    CHECK_DECISION(h.plan(probe(lfUpper(), Leg::LF, JointKind::UPPER, ContactSide::MIN_SIDE,
                                endpoint->contact)),
                   WriteDecision::ACCEPT);
  }

  // An endpoint that does not exist in the compiled model.
  ActuatorCommand nonsense = probe(lfUpper(), Leg::LF, JointKind::UPPER, ContactSide::MIN_SIDE,
                                   -900000);
  nonsense.endpoint_side = static_cast<ContactSide>(9);
  CHECK_DECISION(h.plan(nonsense), WriteDecision::REJECT_NO_ENDPOINT_PLAN);
}

// ---------------------------------------------------------------------------
// The raw <-> q transform
// ---------------------------------------------------------------------------

static void test_q0_is_never_assumed_and_historical_q0_is_refused() {
  g_case = "q0 provenance";
  JointTransformTable table;

  // The provisioned raw centre is 2048 +/- 1 on every unit. That is a
  // servo-level guarantee about mounting, and it is NOT a q0: a transform
  // carrying it without provenance is unusable, exactly like one carrying any
  // other number.
  JointTransform assumed = acceptedTransform(lfUpper(), 2048, 1);
  assumed.state = EvidenceState::UNKNOWN;
  assumed.origin = CalibrationOrigin::NONE;
  CHECK(!assumed.usableProvenance());
  CHECK(!table.admit(assumed));
  CHECK(table.empty());

  // A replayed q0 describes a servo that is no longer in that joint.
  JointTransform replayed = acceptedTransform(lfUpper(), 2067, 1);
  replayed.origin = CalibrationOrigin::HISTORICAL_REPLAY;
  CHECK(!replayed.usableProvenance());
  CHECK(!table.admit(replayed));

  // Measured on the current installation but not yet operational calibration.
  for (EvidenceState state : {EvidenceState::MEASURED, EvidenceState::CANDIDATE,
                              EvidenceState::ACCEPTED, EvidenceState::REJECTED}) {
    JointTransform unpromoted = acceptedTransform(lfUpper(), 2050, 1);
    unpromoted.state = state;
    CHECK(!unpromoted.usableProvenance());
    CHECK(!table.admit(unpromoted));
  }

  // Direction is never defaulted to the URDF sign. Zero means "not measured".
  JointTransform undirected = acceptedTransform(lfUpper(), 2050, 0);
  CHECK(!undirected.usableProvenance());
  CHECK(!table.admit(undirected));

  // And one that does not say which model q0 was captured against. q0 is
  // measured at "the nominal URDF q=0 pose"; without the URDF that defines
  // that pose the number refers to nothing.
  JointTransform modelless = acceptedTransform(lfUpper(), 2050, 1);
  modelless.geometry = kNoGeometryProvenance;
  CHECK(!modelless.boundToGeometry());
  CHECK(!table.admit(modelless));

  // Absent, or anonymous.
  JointTransform absent = acceptedTransform(lfUpper(), 2050, 1);
  absent.present = false;
  CHECK(!table.admit(absent));
  JointTransform anonymous = acceptedTransform(lfUpper(), 2050, 1);
  std::memset(anonymous.identity.physical_unit, 0,
              sizeof(anonymous.identity.physical_unit));
  CHECK(!table.admit(anonymous));
  CHECK(table.empty());

  // A properly measured one is admitted, and applies only to its own joint.
  CHECK(table.admit(acceptedTransform(lfUpper(), 2050, 1)));
  CHECK_EQ(table.size(), 1);
  CHECK(table.find(lfUpper(), geometryProvenanceTag(geometry_data::kProvenance)) != nullptr);
  CHECK(table.find(lhUpper(), geometryProvenanceTag(geometry_data::kProvenance)) == nullptr);
  CHECK(table.find(identity(Leg::LF, JointKind::UPPER, "M11"),
                   geometryProvenanceTag(geometry_data::kProvenance)) == nullptr);
  CHECK_EQ(table.find(lfUpper(), geometryProvenanceTag(geometry_data::kProvenance))->q0_tick, 2050);
}

static void test_plan_bound_moves_fail_closed_without_a_transform() {
  g_case = "no transform";
  Harness h;
  CHECK(h.policy.transforms().empty());

  // Both plan-bound classes need the raw<->q transform to turn an angle into
  // a tick, and none exists after CALIBRATION_RESET_PENDING_FULL_RECALIBRATION.
  CHECK_DECISION(h.plan(probe(lfUpper(), Leg::LF, JointKind::UPPER, ContactSide::MIN_SIDE,
                              -900000)),
                 WriteDecision::REJECT_NO_ACCEPTED_TRANSFORM);
  CalibrationBootstrapContext parked = liveSession();
  parked.auxiliary_parked = true;
  parked.parked_leg = Leg::LF;
  parked.parked_joint = JointKind::UPPER;
  parked.parked_side = ContactSide::MAX_SIDE;
  h.policy.setBootstrapContext(parked);
  CHECK_DECISION(h.plan(auxiliary(lhUpper(), Leg::LF, JointKind::UPPER, ContactSide::MAX_SIDE,
                                  610865)),
                 WriteDecision::REJECT_NO_ACCEPTED_TRANSFORM);
}

static void test_reset_drops_the_session_and_the_transforms() {
  g_case = "reset";
  Harness h;
  CalibrationBootstrapContext ctx = liveSession();
  ctx.direction_verify_tick_budget = 32;
  ctx.auxiliary_parked = true;
  h.policy.setBootstrapContext(ctx);
  CHECK(h.policy.transforms().admit(acceptedTransform(lfUpper(), 2050, 1)));

  h.policy.reset();

  // After a fault the policy cannot know where the robot is standing, so a
  // parked auxiliary must not survive either.
  CHECK(h.policy.transforms().empty());
  CHECK(!h.policy.bootstrapContext().session_active);
  CHECK(!h.policy.bootstrapContext().auxiliary_parked);
  CHECK_EQ(h.policy.bootstrapContext().direction_verify_tick_budget, 0);
  CHECK_DECISION(h.plan(directionVerify(lfUpper(), 16)),
                 WriteDecision::REJECT_NO_CALIBRATION_SESSION);
}

// ---------------------------------------------------------------------------
// Layer separation
// ---------------------------------------------------------------------------

static void test_position_command_is_not_weakened_by_any_of_this() {
  g_case = "POSITION_COMMAND unchanged";
  Harness h;
  CalibrationBootstrapContext ctx = liveSession();
  ctx.direction_verify_tick_budget = 64;
  h.policy.setBootstrapContext(ctx);
  h.policy.transforms().admit(acceptedTransform(lfUpper(), 2048, 1));

  // A bound compiled geometry, a live session, an approved envelope and an
  // accepted transform together still authorise NOTHING for a normal position
  // command: the geometry describes the MODEL, and an accepted joint bound on
  // the current MACHINE is a different thing that does not exist.
  ActuatorCommand position{};
  position.operation = ActuatorOperation::POSITION_COMMAND;
  position.joint = lfUpper();
  position.target_tick = 2048;
  CHECK(h.policy.limits().empty());
  CHECK_DECISION(h.plan(position), WriteDecision::REJECT_NO_ACCEPTED_LIMITS);

  // And the geometry route cannot be reached by a position command even when
  // it names an endpoint.
  position.endpoint_leg = Leg::LF;
  position.endpoint_joint = JointKind::UPPER;
  position.endpoint_side = ContactSide::MIN_SIDE;
  position.target_urad = -900000;
  CHECK_DECISION(h.plan(position), WriteDecision::REJECT_NO_ACCEPTED_LIMITS);
}

static void test_the_profile_carries_no_lf_v25_numeric_evidence() {
  g_case = "LF V25 stays an oracle";
  // The LF V25 archive remains the oracle for detector behaviour - coarse
  // scout, backoff, fine, repeatability, ContactConfirmed stops advancing,
  // stall and abort handling, restore philosophy. None of its NUMBERS may
  // appear here: its q0 estimates (2067/2040/2074), its witness band (24
  // ticks) and its step sizes describe a machine that was disassembled on
  // 2026-08-27.
  for (uint8_t i = 0; i < geometry_data::kJointCount; ++i) {
    const GeometryJointRecord& j = geometry_data::kJoints[i];
    CHECK(j.provisioned_center_raw != 2067);
    CHECK(j.provisioned_center_raw != 2040);
    CHECK(j.provisioned_center_raw != 2074);
    // The URDF motorDirection is specification data the profile carries for
    // comparison. It is never a transform's direction: those start at 0.
    CHECK(j.urdf_motor_direction == 1 || j.urdf_motor_direction == -1);
  }
  JointTransform fresh{};
  CHECK_EQ(fresh.direction, 0);
  CHECK_EQ(fresh.q0_tick, 0);
  CHECK(!fresh.present);
  CHECK(!fresh.usableProvenance());
}

static void test_tostring_is_total() {
  g_case = "toString totality";
  for (TargetDomain d : {TargetDomain::EXECUTABLE_URDF_DOMAIN,
                         TargetDomain::DIAGNOSTIC_GEOMETRY_OUTSIDE_URDF_LIMITS}) {
    CHECK(std::strcmp(toString(d), "UNKNOWN") != 0);
  }
  CHECK(std::strcmp(toString(static_cast<TargetDomain>(9)), "UNKNOWN") == 0);
  for (ParkingOutcome o : {ParkingOutcome::NOT_NEEDED, ParkingOutcome::FEASIBLE_1DOF_PLAN_FOUND}) {
    CHECK(std::strcmp(toString(o), "UNKNOWN") != 0);
  }
  CHECK(std::strcmp(toString(static_cast<ParkingOutcome>(9)), "UNKNOWN") == 0);
  for (ClearancePolicyResult r :
       {ClearancePolicyResult::PASS,
        ClearancePolicyResult::UNRESOLVED_LOWER_BOUND_BELOW_THRESHOLD,
        ClearancePolicyResult::FAIL}) {
    CHECK(std::strcmp(toString(r), "UNKNOWN") != 0);
  }
  CHECK(std::strcmp(toString(static_cast<ClearancePolicyResult>(9)), "UNKNOWN") == 0);
}

int main() {
  std::printf("MATDOG calibration bootstrap geometry offline tests\n");

  test_the_profile_is_the_canonical_v5_bundle();
  test_the_six_parking_plans_are_exactly_the_compilers();
  test_the_legacy_hardcoded_prerequisite_poses_are_gone();

  test_joints_are_keyed_by_current_physical_unit();
  test_historical_lf_v25_identity_cannot_bind_a_current_joint();

  test_geometry_provenance_must_match_exactly();
  test_a_stale_profile_is_refused_by_the_policy();

  test_direction_verify_needs_a_live_session_and_a_budget();
  test_direction_verify_envelope_is_symmetric_and_bounded();
  test_tick_to_microrad_conversion_never_rounds_in_our_favour();
  test_direction_verify_does_not_need_a_transform();

  test_a_diagnostic_endpoint_can_never_become_executable();
  test_a_direct_path_is_used_only_as_the_compiler_validated_it();
  test_an_obstructed_plan_requires_its_parking_first();
  test_an_auxiliary_move_must_be_the_compilers_plan();
  test_a_probe_must_name_its_own_endpoint_and_stay_inside_it();

  test_q0_is_never_assumed_and_historical_q0_is_refused();
  test_plan_bound_moves_fail_closed_without_a_transform();
  test_reset_drops_the_session_and_the_transforms();

  test_position_command_is_not_weakened_by_any_of_this();
  test_the_profile_carries_no_lf_v25_numeric_evidence();
  test_tostring_is_total();

  std::printf("checks_run=%d failures=%d\n", g_checks, g_failures);
  if (g_failures != 0) {
    std::printf("CALIBRATION_GEOMETRY_TESTS = FAIL\n");
    return 1;
  }
  std::printf("CALIBRATION_GEOMETRY_TESTS = PASS\n");
  return 0;
}
