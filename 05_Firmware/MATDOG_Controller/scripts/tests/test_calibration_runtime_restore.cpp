// Real persisted 24/24 fixture, fake read-only storage, real codecs, record
// validation, checked mapping and admission tables. Workspace approval data is
// SYNTHETIC TEST DATA, deliberately not a production operational envelope.
#include <cstdio>
#include <cstring>
#include <vector>

#include "calibration_record_golden.h"
#include "../../src/calibration/CalibrationRuntimeRestore.h"
#include "../../src/actuator/CalibrationTargetResolver.h"

using namespace matdog;
using namespace matdog::calibration;
using namespace matdog::actuator;
static int checks = 0, failures = 0;
#define CHECK(condition) do { ++checks; if (!(condition)) { ++failures; std::printf("FAIL %d: %s\n", __LINE__, #condition); } } while (0)

namespace {
const char* kSyntheticSha = "b5311a81d5aa3297d09fca3251ec685c646f84677e074319d42c7a9fc91b8c00";
JointIdentity canonicalIdentity(uint8_t i) {
  JointIdentity identity;
  identity.leg = static_cast<Leg>(golden::kJoints[i].leg);
  identity.joint = static_cast<JointKind>(golden::kJoints[i].joint);
  setPhysicalUnit(&identity, golden::kJoints[i].unit);
  return identity;
}
class Storage : public CalibrationRecordStorage {
 public:
  std::vector<uint8_t> slot[2], marker;
  bool read_error = false, write_error = false;
  int reads = 0, writes = 0;
  StorageIoStatus get(const std::vector<uint8_t>& data, uint8_t* out, size_t capacity, size_t* size) {
    ++reads; *size = 0;
    if (read_error) return StorageIoStatus::IO_ERROR;
    if (data.empty()) return StorageIoStatus::ABSENT;
    if (data.size() > capacity) return StorageIoStatus::BUFFER_TOO_SMALL;
    std::memcpy(out, data.data(), data.size()); *size = data.size();
    return StorageIoStatus::OK;
  }
  StorageIoStatus read(CalibrationSlot s, uint8_t* out, size_t capacity, size_t* size) override {
    return get(slot[static_cast<int>(s)], out, capacity, size);
  }
  StorageIoStatus readMarker(uint8_t* out, size_t capacity, size_t* size) override { return get(marker, out, capacity, size); }
  StorageIoStatus write(CalibrationSlot s, const uint8_t* data, size_t size) override {
    ++writes; if (write_error) return StorageIoStatus::IO_ERROR;
    slot[static_cast<int>(s)].assign(data, data + size); return StorageIoStatus::OK;
  }
  StorageIoStatus writeMarker(const uint8_t* data, size_t size) override {
    ++writes; if (write_error) return StorageIoStatus::IO_ERROR;
    marker.assign(data, data + size); return StorageIoStatus::OK;
  }
  void putRecord(const CalibrationRecord& record, int index = 0) {
    slot[index].resize(kCalibrationRecordV1EncodedBytes); size_t size = 0;
    CHECK(encodeCalibrationRecord(record, slot[index].data(), slot[index].size(), &size) == CalibrationRecordStatus::OK);
  }
  void putMarker(SaveMarkerState state, uint32_t ack, uint32_t begun) {
    SaveMarker m; m.state = state; m.acknowledged_generation = ack; m.begun_generation = begun;
    marker.resize(kSaveMarkerV1Bytes); size_t size = 0;
    CHECK(encodeSaveMarker(m, marker.data(), marker.size(), &size) == SaveMarkerStatus::OK);
  }
};

RuntimeRestoreRequest facts() {
  RuntimeRestoreRequest request;
  request.expected_generation = 1;
  request.current_installation_confirmed = true;
  request.executor_idle = true;
  request.torque_off_confirmed = true;
  request.motion_permit_revoked = true;
  return request;
}

struct Scenario {
  CalibrationGeometryProfile profile = golden::boundProfile();
  Storage storage;
  CalibrationPersistenceService persistence{&storage};
  CalibrationRuntimeRestore restore;
  JointTransformTable transforms;
  ActuatorLimitTable limits;
  Scenario() {
    storage.putRecord(golden::goldenRecord());
    storage.putMarker(SaveMarkerState::IDLE, 1, 1);
    persistence.begin(NvsInitStatus::READY, 0);
  }
  RuntimeRestoreStatus run(const RuntimeRestoreRequest& request = facts()) {
    return restore.restore(persistence, profile, geometry_data::kProvenance, request, transforms, limits);
  }
  OperationalWorkspaceEvidence syntheticWorkspace() const {
    OperationalWorkspaceEvidence e;
    std::strcpy(e.artifact_sha256, kSyntheticSha);
    e.provenance = geometry_data::kProvenance;
    e.calibration_generation = 1;
    e.workspace_reviewed = e.simultaneous_configuration_checked = true;
    e.collision_clearance_checked = e.margins_reviewed = true;
    e.contact_inset_ticks = 8;
    for (uint8_t i = 0; i < kRecordJointCount; ++i) {
      e.joint[i].joint = canonicalIdentity(i);
      e.joint[i].required_min_urad = -100000;
      e.joint[i].required_max_urad = 100000;
      e.joint[i].safety_margin_urad = 10000;
    }
    return e;
  }
  OperationalAdmissionStatus qualify(const OperationalWorkspaceEvidence* e, const char* pin = kSyntheticSha) {
    return restore.qualifyOperationalLimits(profile, geometry_data::kProvenance, e, pin, facts(), transforms, limits);
  }
};

void verifiedReadAndRestore() {
  Scenario s;
  s.persistence.load(s.profile);
  CHECK(s.persistence.calibrationAvailable());
  CHECK(s.transforms.empty());
  CHECK(!s.restore.snapshot().restored);
  CHECK(!s.restore.snapshot().calibration_accepted);
  const auto bytes = s.storage.slot[0], marker = s.storage.marker;
  const int reads_before = s.storage.reads;
  CHECK(s.run() == RuntimeRestoreStatus::RESTORED);
  CHECK(s.storage.reads > reads_before); // cannot restore from cached LOAD
  CHECK(s.storage.writes == 0);
  CHECK(s.storage.slot[0] == bytes && s.storage.marker == marker);
  CHECK(s.transforms.size() == 12 && s.limits.empty());
  CHECK(s.restore.snapshot().calibration_accepted);
  CHECK(!s.restore.snapshot().operational_limits_approved && !s.restore.snapshot().motion_permit);
  CHECK(s.restore.snapshot().restored_generation == 1);
  for (uint8_t i = 0; i < kRecordJointCount; ++i) {
    const auto* t = s.transforms.find(canonicalIdentity(i), s.profile.provenanceTag());
    CHECK(t != nullptr);
    if (t == nullptr) continue;
    CHECK(t->q0_tick == golden::kJoints[i].q0);
    uint16_t raw = 0; MicroRad q = 0;
    CHECK(resolveUrdfQToRaw(s.profile, geometry_data::kProvenance, *t, 50000, &raw) == TargetResolveStatus::OK);
    CHECK((raw > t->q0_tick ? 1 : -1) == s.profile.findJoint(canonicalIdentity(i))->encoder_direction);
    CHECK(resolveRawToUrdfQ(s.profile, geometry_data::kProvenance, *t, raw, &q) == TargetResolveStatus::OK);
    CHECK(q >= 49000 && q <= 51000);
  }
  CHECK(s.run() == RuntimeRestoreStatus::RESTORED); // idempotent RAM reconstruction
  CHECK(s.storage.writes == 0);
}

void runtimePrerequisites() {
  for (int i = 0; i < 5; ++i) {
    Scenario s; auto request = facts();
    if (i == 0) request.executor_idle = false;
    if (i == 1) request.torque_off_confirmed = false;
    if (i == 2) request.motion_permit_revoked = false;
    if (i == 3) request.current_installation_confirmed = false;
    if (i == 4) request.expected_generation = 0;
    CHECK(s.run(request) != RuntimeRestoreStatus::RESTORED);
    CHECK(s.transforms.empty() && s.limits.empty());
    CHECK(s.storage.reads == 0 && s.storage.writes == 0);
  }
  Scenario s; auto request = facts(); request.expected_generation = 2;
  CHECK(s.run(request) == RuntimeRestoreStatus::REJECT_GENERATION);
  CHECK(s.transforms.empty() && s.storage.writes == 0);
  Scenario unavailable; unavailable.persistence.begin(NvsInitStatus::PARTITION_MISSING, 0);
  CHECK(unavailable.run() == RuntimeRestoreStatus::REJECT_NVS_UNAVAILABLE);
  CHECK(unavailable.storage.reads == 0);
  Scenario geometry;
  auto expected = geometry_data::kProvenance; expected.allocation_sha256[0] ^= 1;
  CHECK(geometry.restore.restore(geometry.persistence, geometry.profile, expected, facts(),
      geometry.transforms, geometry.limits) == RuntimeRestoreStatus::REJECT_GEOMETRY);
  CHECK(geometry.storage.reads == 0 && geometry.transforms.empty());
}

void persistentFailures() {
  for (int mutation = 0; mutation < 9; ++mutation) {
    Scenario s;
    switch (mutation) {
      case 0: s.storage.slot[0][27] ^= 1; break;
      case 1: s.storage.marker[12] ^= 1; break;
      case 2: s.storage.marker.clear(); break;
      case 3: s.storage.putMarker(SaveMarkerState::AWAITING_ACK, 0, 1); break;
      case 4: s.storage.putMarker(SaveMarkerState::PENDING, 0, 1); break;
      case 5: s.storage.putMarker(SaveMarkerState::IDLE, 2, 2); break;
      case 6: s.storage.read_error = true; break;
      case 7: s.storage.putRecord(golden::goldenRecord(), 1); break;
      case 8: s.storage.putRecord(golden::goldenRecord(2), 1); break;
    }
    CHECK(s.run() == RuntimeRestoreStatus::REJECT_LOAD);
    CHECK(s.transforms.empty() && s.limits.empty());
    CHECK(!s.restore.snapshot().calibration_accepted && !s.restore.snapshot().motion_permit);
    CHECK(s.storage.writes == 0);
  }
  // A previous uncertain write is not cured by a healthy-looking LOAD.
  Scenario s; s.storage.write_error = true;
  auto save = s.persistence.save(golden::goldenRecord(), s.profile);
  CHECK(save.uncertain && s.persistence.writesBlocked());
  s.storage.write_error = false; const int writes = s.storage.writes;
  CHECK(s.run() == RuntimeRestoreStatus::REJECT_UNCERTAIN_WRITE);
  CHECK(s.storage.writes == writes && s.transforms.empty());
  // Corruption after a good cached LOAD must be discovered by RESTORE.
  Scenario fresh; fresh.persistence.load(fresh.profile); fresh.storage.slot[0][30] ^= 1;
  CHECK(fresh.run() == RuntimeRestoreStatus::REJECT_LOAD);
}

void everyJointValidatedAtomically() {
  for (uint8_t i = 0; i < kRecordJointCount; ++i) {
    for (int mutation = 0; mutation < 8; ++mutation) {
      Scenario s; auto r = golden::goldenRecord(); auto& joint = r.joint[i];
      switch (mutation) {
        case 0: joint.bus_id ^= 1; break;
        case 1: joint.unit[0] = 'X'; break;
        case 2: joint.encoder_direction *= -1; break;
        case 3: joint.q0_origin = static_cast<uint8_t>(CalibrationOrigin::HISTORICAL_REPLAY); break;
        case 4: joint.q0_state = static_cast<uint8_t>(EvidenceState::ACCEPTED); break;
        case 5: joint.contact[0].witness_accepted = 0; break;
        case 6: ++joint.diagnostics.measured_span_ticks; break;
        case 7: joint.q0_sample_count = 1; break;
      }
      s.storage.putRecord(r); // correctly re-encoded CRC cannot hide semantic drift
      CHECK(s.run() == RuntimeRestoreStatus::REJECT_LOAD);
      CHECK(s.transforms.empty() && s.limits.empty() && s.storage.writes == 0);
    }
  }
  for (uint8_t i = 0; i < kRecordSha256Digests; ++i) {
    Scenario s; auto r = golden::goldenRecord(); r.digest[i][0] ^= 1; s.storage.putRecord(r);
    CHECK(s.run() == RuntimeRestoreStatus::REJECT_LOAD);
    CHECK(s.transforms.empty());
  }
  Scenario authorization; auto r = golden::goldenRecord(); r.parameters_approved = 1;
  authorization.storage.putRecord(r);
  CHECK(authorization.run() == RuntimeRestoreStatus::REJECT_LOAD);
}

void envelopeEvidenceGates() {
  Scenario s; CHECK(s.run() == RuntimeRestoreStatus::RESTORED);
  CHECK(s.qualify(nullptr) == OperationalAdmissionStatus::REJECT_MISSING_EVIDENCE);
  CHECK(s.limits.empty() && s.transforms.size() == 12);
  auto e = s.syntheticWorkspace();
  CHECK(s.qualify(&e) == OperationalAdmissionStatus::APPROVED);
  CHECK(s.limits.size() == 12 && s.restore.snapshot().approved_joints == 12);
  CHECK(!s.restore.snapshot().motion_permit);
  for (uint8_t i = 0; i < kRecordJointCount; ++i) {
    const auto id = canonicalIdentity(i);
    const auto* transform = s.transforms.find(id, s.profile.provenanceTag());
    const auto* limit = s.limits.find(id, s.profile.provenanceTag());
    CHECK(transform != nullptr && limit != nullptr);
    if (transform == nullptr || limit == nullptr) continue;
    MicroRad a = 0, b = 0;
    CHECK(resolveRawToUrdfQ(s.profile, geometry_data::kProvenance, *transform, limit->min_tick, &a) == TargetResolveStatus::OK);
    CHECK(resolveRawToUrdfQ(s.profile, geometry_data::kProvenance, *transform, limit->max_tick, &b) == TargetResolveStatus::OK);
    CHECK(a >= -90000 && a <= 90000 && b >= -90000 && b <= 90000);
  }
  for (int mutation = 0; mutation < 10; ++mutation) {
    e = s.syntheticWorkspace();
    switch (mutation) {
      case 0: e.artifact_sha256[0] = '0'; break;
      case 1: e.provenance.mesh_manifest_sha256[0] ^= 1; break;
      case 2: e.calibration_generation = 2; break;
      case 3: e.workspace_reviewed = false; break;
      case 4: e.simultaneous_configuration_checked = false; break;
      case 5: e.collision_clearance_checked = false; break;
      case 6: e.margins_reviewed = false; break;
      case 7: e.contact_inset_ticks = 0; break;
      case 8: e.schema = 2; break;
      case 9: std::memset(e.artifact_sha256, '0', 64); break;
    }
    CHECK(s.qualify(&e) != OperationalAdmissionStatus::APPROVED);
    CHECK(s.limits.empty() && !s.restore.snapshot().operational_limits_approved);
    CHECK(!s.restore.snapshot().motion_permit);
  }
  for (uint8_t i = 0; i < kRecordJointCount; ++i) {
    e = s.syntheticWorkspace(); e.joint[i].joint.physical_unit[0] = 'X';
    CHECK(s.qualify(&e) == OperationalAdmissionStatus::REJECT_JOINT_IDENTITY);
    CHECK(s.limits.empty() && s.restore.snapshot().failed_joint == i);
    e = s.syntheticWorkspace(); e.joint[i].required_min_urad = -10000000;
    CHECK(s.qualify(&e) == OperationalAdmissionStatus::REJECT_ENVELOPE);
    CHECK(s.limits.empty());
    e = s.syntheticWorkspace(); e.joint[i].safety_margin_urad = -1;
    CHECK(s.qualify(&e) == OperationalAdmissionStatus::REJECT_MARGIN_EVIDENCE);
    CHECK(s.limits.empty());
  }
  // Even full URDF limits + contact endpoints cannot replace collision evidence.
  e = s.syntheticWorkspace();
  for (uint8_t i = 0; i < kRecordJointCount; ++i) {
    e.joint[i].required_min_urad = s.profile.findJoint(canonicalIdentity(i))->urdf_lower;
    e.joint[i].required_max_urad = s.profile.findJoint(canonicalIdentity(i))->urdf_upper;
  }
  e.simultaneous_configuration_checked = false;
  CHECK(s.qualify(&e) == OperationalAdmissionStatus::REJECT_CONFIGURATION_EVIDENCE);
  e.simultaneous_configuration_checked = true; e.contact_inset_ticks = 2000;
  CHECK(s.qualify(&e) == OperationalAdmissionStatus::REJECT_CONTACT_CLEARANCE);
  CHECK(s.limits.empty());
  e = s.syntheticWorkspace(); e.contact_inset_ticks = 1;
  CHECK(s.qualify(&e) == OperationalAdmissionStatus::REJECT_MARGIN_EVIDENCE);
  CHECK(s.limits.empty());

  // An admitted transform replaced after RESTORE invalidates qualification.
  e = s.syntheticWorkspace(); auto altered = *s.transforms.find(canonicalIdentity(11), s.profile.provenanceTag());
  ++altered.q0_tick; CHECK(s.transforms.admit(altered));
  CHECK(s.qualify(&e) == OperationalAdmissionStatus::REJECT_TRANSFORM);
  CHECK(s.limits.empty() && s.restore.snapshot().failed_joint == 11);
  CHECK(s.storage.writes == 0);
}

void staleTablesRevokedAndNoAcceptanceFromLoad() {
  Scenario s; CHECK(s.run() == RuntimeRestoreStatus::RESTORED);
  auto e = s.syntheticWorkspace(); CHECK(s.qualify(&e) == OperationalAdmissionStatus::APPROVED);
  s.storage.slot[0][42] ^= 1;
  CHECK(s.run() == RuntimeRestoreStatus::REJECT_LOAD);
  CHECK(s.transforms.empty() && s.limits.empty());
  CHECK(!s.restore.snapshot().calibration_accepted);
  CalibrationRecord out = golden::goldenRecord();
  CHECK(s.persistence.readAcknowledgedRecord(s.profile, &out).load_status != LoadStatus::OK);
  CHECK(out.generation == 0 && out.calibration_accepted == 0);
  CHECK(s.persistence.readAcknowledgedRecord(s.profile, nullptr).load_status == LoadStatus::BAD_ARGUMENT);
  CHECK(s.persistence.readAcknowledgedRecord(s.profile, s.persistence.scratchRecord()).load_status == LoadStatus::BAD_ARGUMENT);
}

// Fixture commissioning limits: gen-1 contact box with a runtime inset,
// intersected with the URDF interval. Never an operational/route approval.
void commissioningLimits() {
  Scenario s;
  CHECK(s.restore.qualifyCommissioningLimits(s.profile, geometry_data::kProvenance, 8, facts(), s.transforms, s.limits) ==
        OperationalAdmissionStatus::REJECT_NOT_RESTORED);
  CHECK(s.limits.empty() && !s.restore.snapshot().commissioning_limits_active);
  CHECK(s.run() == RuntimeRestoreStatus::RESTORED);
  auto unsafe = facts(); unsafe.torque_off_confirmed = false;
  CHECK(s.restore.qualifyCommissioningLimits(s.profile, geometry_data::kProvenance, 8, unsafe, s.transforms, s.limits) ==
        OperationalAdmissionStatus::REJECT_UNSAFE_RUNTIME);
  auto stranger = facts(); stranger.current_installation_confirmed = false;
  CHECK(s.restore.qualifyCommissioningLimits(s.profile, geometry_data::kProvenance, 8, stranger, s.transforms, s.limits) ==
        OperationalAdmissionStatus::REJECT_NOT_RESTORED);
  auto generation = facts(); generation.expected_generation = 2;
  CHECK(s.restore.qualifyCommissioningLimits(s.profile, geometry_data::kProvenance, 8, generation, s.transforms, s.limits) ==
        OperationalAdmissionStatus::REJECT_GENERATION);
  for (uint16_t bad : {uint16_t(0), uint16_t(1), uint16_t(65), uint16_t(4000)}) {
    CHECK(s.restore.qualifyCommissioningLimits(s.profile, geometry_data::kProvenance, bad, facts(), s.transforms, s.limits) ==
          OperationalAdmissionStatus::REJECT_MARGIN_EVIDENCE);
    CHECK(s.limits.empty());
  }
  // The inset must exceed each joint's recorded contact repeatability.
  uint16_t repeatability = 0;
  const auto record = golden::goldenRecord();
  for (const auto& j : record.joint)
    for (const auto& c : j.contact) if (c.repeatability_ticks > repeatability) repeatability = c.repeatability_ticks;
  if (repeatability >= kCommissioningMinInsetTicks) {
    CHECK(s.restore.qualifyCommissioningLimits(s.profile, geometry_data::kProvenance, repeatability, facts(), s.transforms, s.limits) ==
          OperationalAdmissionStatus::REJECT_MARGIN_EVIDENCE);
    CHECK(s.limits.empty());
  }
  const uint16_t inset = repeatability + 1 < 8 ? 8 : repeatability + 1;
  CHECK(s.restore.qualifyCommissioningLimits(s.profile, geometry_data::kProvenance, inset, facts(), s.transforms, s.limits) ==
        OperationalAdmissionStatus::APPROVED);
  const auto& snap = s.restore.snapshot();
  CHECK(snap.commissioning_limits_active && snap.commissioning_joints == 12 && snap.commissioning_inset_ticks == inset);
  CHECK(!snap.operational_limits_approved && snap.approved_joints == 0 && !snap.motion_permit);
  CHECK(s.limits.size() == 12);
  for (uint8_t i = 0; i < kRecordJointCount; ++i) {
    const auto id = canonicalIdentity(i);
    const auto* limit = s.limits.find(id, s.profile.provenanceTag());
    const auto* t = s.transforms.find(id, s.profile.provenanceTag());
    const auto* g = s.profile.findJoint(id);
    CHECK(limit != nullptr && t != nullptr && g != nullptr);
    if (!limit || !t || !g) continue;
    const auto& j = record.joint[i];
    const uint16_t lo = j.contact[0].fine_tick_2 < j.contact[1].fine_tick_2 ? j.contact[0].fine_tick_2 : j.contact[1].fine_tick_2;
    const uint16_t hi = j.contact[0].fine_tick_2 > j.contact[1].fine_tick_2 ? j.contact[0].fine_tick_2 : j.contact[1].fine_tick_2;
    CHECK(limit->min_tick >= lo + inset && limit->max_tick + inset <= hi);
    // Never outside the URDF interval either.
    MicroRad qa = 0, qb = 0;
    CHECK(resolveRawToUrdfQ(s.profile, geometry_data::kProvenance, *t, limit->min_tick, &qa) == TargetResolveStatus::OK);
    CHECK(resolveRawToUrdfQ(s.profile, geometry_data::kProvenance, *t, limit->max_tick, &qb) == TargetResolveStatus::OK);
    CHECK(qa >= g->urdf_lower && qa <= g->urdf_upper && qb >= g->urdf_lower && qb <= g->urdf_upper);
  }
  // A reviewed qualification replaces the commissioning scope, and vice versa.
  auto e = s.syntheticWorkspace(); CHECK(s.qualify(&e) == OperationalAdmissionStatus::APPROVED);
  CHECK(!s.restore.snapshot().commissioning_limits_active && s.restore.snapshot().approved_joints == 12);
  CHECK(s.restore.qualifyCommissioningLimits(s.profile, geometry_data::kProvenance, inset, facts(), s.transforms, s.limits) ==
        OperationalAdmissionStatus::APPROVED);
  CHECK(s.restore.snapshot().approved_joints == 0 && !s.restore.snapshot().operational_limits_approved);
  // A later rejection revokes every old limit; invalidate clears the scope.
  CHECK(s.restore.qualifyCommissioningLimits(s.profile, geometry_data::kProvenance, 1, facts(), s.transforms, s.limits) ==
        OperationalAdmissionStatus::REJECT_MARGIN_EVIDENCE);
  CHECK(s.limits.empty() && !s.restore.snapshot().commissioning_limits_active);
  CHECK(s.restore.qualifyCommissioningLimits(s.profile, geometry_data::kProvenance, inset, facts(), s.transforms, s.limits) ==
        OperationalAdmissionStatus::APPROVED);
  s.restore.invalidate(s.transforms, s.limits);
  CHECK(s.limits.empty() && !s.restore.snapshot().commissioning_limits_active && s.restore.snapshot().commissioning_joints == 0);
}
}  // namespace

int main() {
  verifiedReadAndRestore(); runtimePrerequisites(); persistentFailures();
  everyJointValidatedAtomically(); envelopeEvidenceGates(); staleTablesRevokedAndNoAcceptanceFromLoad();
  commissioningLimits();
  std::printf("CalibrationRuntimeRestore: %d checks, %d failures (offline; no hardware or approved production bounds)\n", checks, failures);
  return failures == 0 ? 0 : 1;
}
