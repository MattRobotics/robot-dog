#include <cstdio>

#include "../../src/actuator/CalibrationGeometryProfileData.h"
#include "../../src/actuator/CalibrationQ0EvidencePreparation.h"

using namespace matdog::actuator;

static int g_checks = 0;
static int g_failures = 0;

static void check(bool ok, const char* label) {
  ++g_checks;
  if (!ok) {
    ++g_failures;
    std::printf("FAIL: %s\n", label);
  }
}

static CalibrationGeometryProfile profile() {
  CalibrationGeometryProfile p;
  p.bind(&geometry_data::kProvenance,
         geometry_data::kJoints, geometry_data::kJointCount,
         geometry_data::kEndpoints, geometry_data::kEndpointCount);
  return p;
}

static void test_snapshot_is_exact_cr2c_package() {
  check(q0_evidence_data::kCaptureSessionId == 1, "CR2-C session id");
  check(q0_evidence_data::kSampleCount == 9, "CR2-C sample count");
  check(q0_evidence_data::kStabilitySpreadTicks == 0, "CR2-C spread");
  check(q0_evidence_data::kRecordCount == 12, "CR2-C record count");

  const uint8_t buses[12] = {11,12,13,21,22,23,31,32,33,41,42,43};
  const uint16_t q0[12] = {2087,2100,1996,1985,2092,2030,2034,2042,2081,2073,2089,2035};
  for (uint8_t i = 0; i < 12; ++i) {
    check(q0_evidence_data::kRecords[i].bus_id == buses[i], "snapshot bus id");
    check(q0_evidence_data::kRecords[i].q0_tick == q0[i], "snapshot q0 tick");
  }
  check(sameProvenance(q0_evidence_data::kSourceGeometry, geometry_data::kProvenance),
        "frozen source geometry matches current CR3 geometry");
}

static void test_no_implicit_promotion_after_boot() {
  const auto p = profile();
  const auto result =
      prepareCurrentQ0Evidence(p, geometry_data::kProvenance, false);
  check(result.status ==
            Q0EvidencePreparationStatus::REJECT_CURRENT_INSTALLATION_NOT_CONFIRMED,
        "persisted evidence needs explicit installation confirmation");
  check(result.transform_count == 0, "no transform produced without confirmation");
}

static void test_capture_package_prerequisites_fail_closed() {
  const auto frozen = frozenQ0EvidencePackageFacts();
  check(frozen.formal_population_pass, "CR2-C frozen population PASS");
  check(frozen.nominal_zero_pose_confirmed, "CR2-C frozen nominal q0 confirmation");
  check(frozen.torque_off_verified_before_capture, "CR2-C frozen Torque OFF prerequisite");
  check(validateQ0EvidencePackageFacts(frozen) == Q0EvidencePreparationStatus::READY,
        "complete capture facts validate");

  auto bad = frozen;
  bad.formal_population_pass = false;
  check(validateQ0EvidencePackageFacts(bad) ==
            Q0EvidencePreparationStatus::REJECT_CAPTURE_POPULATION_NOT_PASS,
        "missing formal population PASS rejects");

  bad = frozen;
  bad.nominal_zero_pose_confirmed = false;
  check(validateQ0EvidencePackageFacts(bad) ==
            Q0EvidencePreparationStatus::REJECT_CAPTURE_Q0_POSE_NOT_CONFIRMED,
        "missing explicit q0 pose confirmation rejects");

  bad = frozen;
  bad.torque_off_verified_before_capture = false;
  check(validateQ0EvidencePackageFacts(bad) ==
            Q0EvidencePreparationStatus::REJECT_CAPTURE_TORQUE_NOT_OFF,
        "missing Torque OFF prerequisite rejects");
}

static void test_current_package_repasses_real_gates() {
  const auto p = profile();
  const auto result =
      prepareCurrentQ0Evidence(p, geometry_data::kProvenance, true);
  check(result.ready(), "current CR2-C package prepares 12 transforms");
  check(result.transform_count == 12, "all 12 transforms produced");

  for (uint8_t i = 0; i < result.transform_count; ++i) {
    const auto& t = result.transforms[i];
    check(t.present, "prepared transform present");
    check(t.state == matdog::calibration::EvidenceState::PROMOTED,
          "prepared transform is promoted only through gate");
    check(t.origin == matdog::calibration::CalibrationOrigin::LIVE_SESSION,
          "prepared transform retains live-measurement provenance");
    check(t.geometry == p.provenanceTag(), "prepared transform current geometry");
  }
}

static void test_geometry_change_invalidates_package() {
  auto p = profile();
  GeometryProvenance changed = geometry_data::kProvenance;
  changed.urdf_sha256[0] = changed.urdf_sha256[0] == 'a' ? 'b' : 'a';

  const auto result = prepareCurrentQ0Evidence(p, changed, true);
  check(result.status == Q0EvidencePreparationStatus::REJECT_SOURCE_GEOMETRY,
        "changed geometry invalidates persisted q0 evidence");
  check(result.transform_count == 0, "geometry mismatch produces no transforms");
}

int main() {
  test_snapshot_is_exact_cr2c_package();
  test_no_implicit_promotion_after_boot();
  test_capture_package_prerequisites_fail_closed();
  test_current_package_repasses_real_gates();
  test_geometry_change_invalidates_package();

  std::printf("test_cr3_q0_evidence_preparation: %d checks, %d failures\n",
              g_checks, g_failures);
  return g_failures == 0 ? 0 : 1;
}
