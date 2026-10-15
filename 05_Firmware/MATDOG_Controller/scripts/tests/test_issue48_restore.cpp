// Issue #48: conditional RAM-only RESTORE. Real golden 24/24 record, fake
// read-only storage. Proves (final operator decision 2026-10-14): AUTOMATIC
// boot RESTORE with live evidence and no pin/attestation; RAM suspension =>
// PENDING; manual needs confirmation;
// live raw outside the recorded span refuses atomically; no storage writes;
// idempotence; nothing here grants limits/permit/authority.
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

#include "calibration_record_golden.h"
#include "../../src/calibration/CalibrationRuntimeRestore.h"
#include "../../src/calibration/RestoreAttestationPolicy.h"
#include "../../src/network/RestoreRequestJson.h"

using namespace matdog;
using namespace matdog::calibration;
using namespace matdog::actuator;
static int checks = 0, failures = 0;
#define CHECK(c) do { ++checks; if (!(c)) { ++failures; std::printf("FAIL %d: %s\n", __LINE__, #c); } } while (0)

namespace {
class Storage : public CalibrationRecordStorage {
 public:
  std::vector<uint8_t> slot[2], marker;
  int reads = 0, writes = 0;
  StorageIoStatus get(const std::vector<uint8_t>& d, uint8_t* out, size_t cap, size_t* size) {
    ++reads; *size = 0;
    if (d.empty()) return StorageIoStatus::ABSENT;
    if (d.size() > cap) return StorageIoStatus::BUFFER_TOO_SMALL;
    std::memcpy(out, d.data(), d.size()); *size = d.size(); return StorageIoStatus::OK;
  }
  StorageIoStatus read(CalibrationSlot s, uint8_t* out, size_t cap, size_t* size) override { return get(slot[static_cast<int>(s)], out, cap, size); }
  StorageIoStatus readMarker(uint8_t* out, size_t cap, size_t* size) override { return get(marker, out, cap, size); }
  StorageIoStatus write(CalibrationSlot, const uint8_t*, size_t) override { ++writes; return StorageIoStatus::IO_ERROR; }
  StorageIoStatus writeMarker(const uint8_t*, size_t) override { ++writes; return StorageIoStatus::IO_ERROR; }
  void putRecord(const CalibrationRecord& r) {
    slot[0].resize(kCalibrationRecordV1EncodedBytes); size_t n = 0;
    CHECK(encodeCalibrationRecord(r, slot[0].data(), slot[0].size(), &n) == CalibrationRecordStatus::OK);
  }
  void putMarker(uint32_t ack) {
    SaveMarker m; m.state = SaveMarkerState::IDLE; m.acknowledged_generation = ack; m.begun_generation = ack;
    marker.resize(kSaveMarkerV1Bytes); size_t n = 0;
    CHECK(encodeSaveMarker(m, marker.data(), marker.size(), &n) == SaveMarkerStatus::OK);
  }
};
struct Rig {
  CalibrationGeometryProfile profile = golden::boundProfile();
  Storage storage;
  CalibrationPersistenceService persistence{&storage};
  CalibrationRuntimeRestore restore;
  JointTransformTable transforms;
  ActuatorLimitTable limits;
  explicit Rig(bool record = true) {
    if (record) { storage.putRecord(golden::goldenRecord()); storage.putMarker(1); }
    persistence.begin(NvsInitStatus::READY, 0);
    persistence.load(profile);
  }
  RuntimeRestoreRequest facts() const {
    RuntimeRestoreRequest r; r.expected_generation = 1; r.current_installation_confirmed = true;
    r.executor_idle = r.torque_off_confirmed = r.motion_permit_revoked = true; return r;
  }
};
RestoreFacts goodFacts(const RecordInspection& in) {
  RestoreFacts f;
  f.maintenance = f.runtime_idle = f.nvs_ready = f.record_valid_acknowledged = true;
  f.record_inspection_ok = in.ok; f.record_generation = in.generation; f.record_fingerprint = in.fingerprint;
  f.preflight_complete = true; f.preflight_pass = 12; f.torque_off = 12;
  f.live_positions_inside_record = true;
  return f;
}
LivePositionEvidence midSpan(const RecordInspection& in) {
  LivePositionEvidence live; live.present = true;
  for (unsigned i = 0; i < kRecordJointCount; ++i) live.raw[i] = static_cast<uint16_t>((in.contact_min[i] + in.contact_max[i]) / 2);
  return live;
}
}  // namespace

int main() {
  // --- inspection: read-only, deterministic fingerprint, no publication ---
  Rig rig;
  RecordInspection in;
  const int writes0 = rig.storage.writes;
  CHECK(rig.restore.inspect(rig.persistence, rig.profile, geometry_data::kProvenance, &in) == RuntimeRestoreStatus::RESTORED);
  CHECK(in.ok && in.generation == 1 && in.fingerprint != 0 && in.calibration_accepted);
  CHECK(rig.transforms.empty());                 // inspect publishes nothing
  CHECK(!rig.restore.snapshot().restored);
  CHECK(rig.restore.snapshot().restore_attempts == 0);
  RecordInspection again;
  rig.restore.inspect(rig.persistence, rig.profile, geometry_data::kProvenance, &again);
  CHECK(again.fingerprint == in.fingerprint);
  CalibrationRecord altered = golden::goldenRecord(); altered.joint[4].q0_tick ^= 1;
  CHECK(calibrationRecordFingerprint(altered) != calibrationRecordFingerprint(golden::goldenRecord()));
  for (unsigned i = 0; i < kRecordJointCount; ++i) CHECK(in.contact_min[i] <= in.contact_max[i]);
  std::printf("ISSUE48 golden gen=%lu fingerprint=%016llx\n", (unsigned long)in.generation, (unsigned long long)in.fingerprint);

  // --- policy matrix ---
  InstallationAttestation suspended; suspended.auto_restore_enabled = false;  // RAM maintenance suspension
  InstallationAttestation pinned;  // default configuration: automatic, nothing pinned
  const InstallationAttestation& none = pinned;
  CHECK(pinned.auto_restore_enabled && !pinned.present && pinned.record_fingerprint == 0);
  RestoreRequestFacts boot;
  RestoreRequestFacts manual; manual.trigger = RestoreTrigger::OPERATOR_MANUAL; manual.requested_generation = 1; manual.operator_confirmed_installation = true;
  const RestoreFacts ok = goodFacts(in);
  auto decide = [&](RestoreFacts f, const InstallationAttestation& a, const RestoreRequestFacts& q) { return evaluateRestore(f, a, q); };

  CHECK(decide(ok, pinned, boot).decision == RestoreDecision::ALLOW_AUTOMATIC);   // no pin, no confirmation
  { auto a = pinned; a.record_fingerprint ^= 1; a.generation = 7; CHECK(decide(ok, a, boot).decision == RestoreDecision::ALLOW_AUTOMATIC); }  // informational only
  CHECK(decide(ok, suspended, boot).decision == RestoreDecision::PENDING_OPERATOR_REVIEW);
  CHECK(decide(ok, suspended, boot).reason == RestoreReason::AUTO_RESTORE_DISABLED);
  CHECK(decide(ok, suspended, manual).decision == RestoreDecision::ALLOW_OPERATOR);  // maintenance can still restore by hand
  CHECK(decide(ok, none, manual).decision == RestoreDecision::ALLOW_OPERATOR);
  { auto q = manual; q.operator_confirmed_installation = false; CHECK(decide(ok, none, q).reason == RestoreReason::OPERATOR_CONFIRMATION_REQUIRED); }
  { auto q = manual; q.requested_generation = 2; CHECK(decide(ok, none, q).reason == RestoreReason::GENERATION_MISMATCH); }
  // Every precondition refuses even with a pinned attestation (fail closed).
  { auto f = ok; f.maintenance = false; CHECK(decide(f, pinned, boot).reason == RestoreReason::NOT_MAINTENANCE); }
  { auto f = ok; f.runtime_idle = false; CHECK(decide(f, pinned, manual).reason == RestoreReason::RUNTIME_NOT_IDLE); }
  { auto f = ok; f.nvs_ready = false; CHECK(decide(f, pinned, boot).decision == RestoreDecision::REFUSED); }
  { auto f = ok; f.write_outcome_uncertain = true; CHECK(decide(f, pinned, boot).reason == RestoreReason::WRITE_OUTCOME_UNCERTAIN); }
  { auto f = ok; f.record_valid_acknowledged = false; CHECK(decide(f, pinned, boot).reason == RestoreReason::NO_VALID_ACKNOWLEDGED_RECORD); }
  { auto f = ok; f.record_inspection_ok = false; CHECK(decide(f, pinned, manual).reason == RestoreReason::RECORD_REJECTED); }
  { auto f = ok; f.preflight_complete = false; CHECK(decide(f, pinned, boot).decision == RestoreDecision::PENDING_OPERATOR_REVIEW);
    CHECK(decide(f, pinned, manual).decision == RestoreDecision::REFUSED); }
  { auto f = ok; f.preflight_pass = 11; CHECK(decide(f, pinned, boot).reason == RestoreReason::PREFLIGHT_FAILED);
    CHECK(decide(f, none, manual).reason == RestoreReason::PREFLIGHT_FAILED); }  // confirmation cannot override
  { auto f = ok; f.torque_off = 11; CHECK(decide(f, pinned, manual).reason == RestoreReason::TORQUE_NOT_OFF); }
  { auto f = ok; f.live_positions_inside_record = false; f.live_position_failed_joint = 7;
    const auto v = decide(f, pinned, manual); CHECK(v.reason == RestoreReason::LIVE_POSITION_OUTSIDE_RECORD && v.failed_joint == 7); }
  { auto f = ok; f.already_restored = true; f.restored_generation = 1; CHECK(decide(f, pinned, boot).decision == RestoreDecision::ALREADY_RESTORED);
    CHECK(decide(f, none, manual).decision == RestoreDecision::ALREADY_RESTORED); }
  { auto f = ok; f.preflight_complete = false; f.preflight_pass = 0; f.torque_off = 0;
    CHECK(decide(f, pinned, boot).reason == RestoreReason::PREFLIGHT_NOT_COMPLETE); }  // automatic boot waits for its own preflight
  { auto f = ok; f.live_positions_inside_record = false; f.live_position_failed_joint = 3;  // remounted servo => maintenance
    const auto v = decide(f, pinned, boot); CHECK(v.decision == RestoreDecision::REFUSED && v.reason == RestoreReason::LIVE_POSITION_OUTSIDE_RECORD && v.failed_joint == 3); }
  for (int d = 0; d <= 4; ++d) CHECK(std::strcmp(toString(static_cast<RestoreDecision>(d)), "UNKNOWN") != 0);
  for (int r = 0; r <= 15; ++r) CHECK(std::strcmp(toString(static_cast<RestoreReason>(r)), "UNKNOWN") != 0);

  // --- live-position gate inside restore(): atomic, fail closed ---
  {
    Rig r;
    auto live = midSpan(in);
    live.raw[5] = static_cast<uint16_t>(in.contact_max[5] + in.repeatability[5] + live.extra_margin_ticks + 1);
    CHECK(firstLivePositionMismatch(in, live) == 5);
    CHECK(r.restore.restore(r.persistence, r.profile, geometry_data::kProvenance, r.facts(), r.transforms, r.limits, &live)
          == RuntimeRestoreStatus::REJECT_LIVE_POSITION);
    CHECK(r.transforms.empty() && !r.restore.snapshot().restored && r.restore.snapshot().failed_joint == 5);
    live.raw[5] = static_cast<uint16_t>(in.contact_max[5] + in.repeatability[5] + live.extra_margin_ticks);  // boundary inside
    CHECK(firstLivePositionMismatch(in, live) == kRecordJointCount);
    live.raw[0] = 4096; CHECK(firstLivePositionMismatch(in, live) == 0);
    LivePositionEvidence absent; CHECK(firstLivePositionMismatch(in, absent) == 0);  // no evidence = not inside
  }
  {
    Rig r;
    const auto live = midSpan(in);
    const int w = r.storage.writes;
    CHECK(r.restore.restore(r.persistence, r.profile, geometry_data::kProvenance, r.facts(), r.transforms, r.limits, &live)
          == RuntimeRestoreStatus::RESTORED);
    CHECK(r.transforms.size() == kRecordJointCount);
    CHECK(r.limits.size() == 0);                             // RESTORE never approves limits
    CHECK(!r.restore.snapshot().operational_limits_approved);
    CHECK(!r.restore.snapshot().commissioning_limits_active);
    CHECK(!r.restore.snapshot().motion_permit);
    CHECK(r.storage.writes == w);                            // never writes NVS
    // Idempotent: a second identical RESTORE gives the same table.
    CHECK(r.restore.restore(r.persistence, r.profile, geometry_data::kProvenance, r.facts(), r.transforms, r.limits, &live)
          == RuntimeRestoreStatus::RESTORED);
    CHECK(r.transforms.size() == kRecordJointCount);
  }
  {
    Rig empty(false);  // no record at all
    RecordInspection none_in;
    CHECK(empty.restore.inspect(empty.persistence, empty.profile, geometry_data::kProvenance, &none_in) != RuntimeRestoreStatus::RESTORED);
    CHECK(!none_in.ok && none_in.fingerprint == 0);
    auto f = goodFacts(none_in); f.record_valid_acknowledged = false;
    CHECK(evaluateRestore(f, pinned, boot).decision == RestoreDecision::REFUSED);
  }
  {
    Rig wrong;  // different geometry provenance refuses inspection
    GeometryProvenance other = geometry_data::kProvenance; other.urdf_sha256[0] = other.urdf_sha256[0] == 'a' ? 'b' : 'a';
    RecordInspection x;
    CHECK(wrong.restore.inspect(wrong.persistence, wrong.profile, other, &x) == RuntimeRestoreStatus::REJECT_GEOMETRY && !x.ok);
  }
  // --- Dashboard body parser: strict, bounded, no implicit confirmation ---
  {
    using matdog::network::parseRestoreRequestJson;
    auto parse = [](const char* body, uint32_t* g, bool* c) { const char* why = nullptr; return parseRestoreRequestJson(body, std::strlen(body), g, c, &why); };
    uint32_t g = 0; bool c = false;
    CHECK(parse("{\"generation\":1,\"confirm\":\"CONFIRM_CURRENT_INSTALLATION\"}", &g, &c) && g == 1 && c);
    CHECK(parse(" { \"confirm\" : \"CONFIRM_CURRENT_INSTALLATION\" , \"generation\" : 4294967295 } ", &g, &c) && g == 4294967295u && c);
    CHECK(parse("{\"generation\":1,\"confirm\":\"yes\"}", &g, &c) && g == 1 && !c);            // parsed, NOT confirmed
    CHECK(!parse("{\"generation\":0,\"confirm\":\"CONFIRM_CURRENT_INSTALLATION\"}", &g, &c));
    CHECK(!parse("{\"generation\":4294967296,\"confirm\":\"CONFIRM_CURRENT_INSTALLATION\"}", &g, &c));
    CHECK(!parse("{\"generation\":01,\"confirm\":\"CONFIRM_CURRENT_INSTALLATION\"}", &g, &c));
    CHECK(!parse("{\"generation\":-1,\"confirm\":\"CONFIRM_CURRENT_INSTALLATION\"}", &g, &c));
    CHECK(!parse("{\"generation\":1}", &g, &c));
    CHECK(!parse("{\"generation\":1,\"generation\":2}", &g, &c));
    CHECK(!parse("{\"generation\":1,\"confirm\":\"CONFIRM_CURRENT_INSTALLATION\",\"x\":1}", &g, &c));
    CHECK(!parse("{\"generation\":1,\"confirm\":\"CONFIRM_CURRENT_INSTALLATION\"}x", &g, &c));
    CHECK(!parse("{\"generation\":1,\"confirm\":\"CONFIRM_\\u0043URRENT_INSTALLATION\"}", &g, &c));  // escapes refused
    CHECK(!parse("", &g, &c));
    std::string big(300, ' '); big[0] = '{'; CHECK(!parseRestoreRequestJson(big.c_str(), big.size(), &g, &c, nullptr));
  }
  CHECK(rig.storage.writes == writes0);
  std::printf("ISSUE48_RESTORE: %d checks, %d failures (offline; no NVS/servo/authority side effect)\n", checks, failures);
  return failures ? 1 : 0;
}
