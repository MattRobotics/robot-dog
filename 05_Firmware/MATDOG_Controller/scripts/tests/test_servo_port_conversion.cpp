// Controller::SimpleServoPort conversions (2026-11-14): the production slot(),
// decodeRaw() and resolve() bodies, copied verbatim from SimpleMotionIntegration.cpp
// by run_servo_port_conversion_tests.sh, against the per-call lookup they replace
// (transforms()/limits().find + the full resolveRawObservation/resolveRadiansToRaw)
// on the robot's gen-1 tables (golden 24/24 record, restore + qualifyCommissioningLimits
// inset 5). Every result must be identical for the 12 joints: the whole RAW domain,
// a quarter-tick q grid, every micro-radian rounding boundary, the URDF limits and
// non-finite values, before and after each change of the tables or the profile
// (clear, re-admission in another order, a new q0, a narrowed limit, a cleared,
// rebound or different geometry). Software only.
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>
#include "calibration_record_golden.h"
#include "CalibrationRuntimeRestore.h"
#include "CalibrationTargetResolver.h"
#include "ActuatorWritePolicy.h"
#include "CalibrationGeometryProfileData.h"
namespace {
using namespace matdog; using namespace matdog::calibration;
class Storage : public CalibrationRecordStorage {
 public:
  std::vector<uint8_t> slot[2], marker;
  StorageIoStatus get(const std::vector<uint8_t>& data, uint8_t* out, size_t capacity, size_t* size) {
    *size = 0; if (data.empty()) return StorageIoStatus::ABSENT; if (data.size() > capacity) return StorageIoStatus::BUFFER_TOO_SMALL;
    std::memcpy(out, data.data(), data.size()); *size = data.size(); return StorageIoStatus::OK; }
  StorageIoStatus read(CalibrationSlot s, uint8_t* out, size_t capacity, size_t* size) override { return get(slot[static_cast<int>(s)], out, capacity, size); }
  StorageIoStatus readMarker(uint8_t* out, size_t capacity, size_t* size) override { return get(marker, out, capacity, size); }
  StorageIoStatus write(CalibrationSlot, const uint8_t*, size_t) override { return StorageIoStatus::IO_ERROR; }
  StorageIoStatus writeMarker(const uint8_t*, size_t) override { return StorageIoStatus::IO_ERROR; }
};
}
namespace matdog { namespace core {
struct MiniPolicy {
  actuator::JointTransformTable t; actuator::ActuatorLimitTable l;
  actuator::JointTransformTable& transforms() { return t; }
  actuator::ActuatorLimitTable& limits() { return l; }
};
struct Controller {
  actuator::CalibrationGeometryProfile geometry_profile_; MiniPolicy actuator_policy_;
  struct SimpleServoPort {
    Controller* owner = nullptr;
    bool resolve(unsigned, double, uint16_t&);
    bool decodeRaw(unsigned, uint16_t, double&);
#include "production_simple_servo_port_slot_decl.inc"
  } port;
};
#include "production_simple_servo_port_conversion.inc"
} }
using namespace matdog; using namespace matdog::calibration; using namespace matdog::actuator;
static unsigned long checks = 0, failures = 0;
static void check(bool ok, const char* step, const char* what, unsigned slot, double x) {
  ++checks; if (ok) return; if (++failures <= 20) std::printf("FAIL %s %s slot=%u at %.17g\n", step, what, slot, x);
}
// The per-call lookup the port replaced (the 2026-11-14 bodies before the slot binding).
static bool lookupDecode(core::Controller& c, unsigned i, uint16_t raw, double& q) {
  if (i >= 12 || raw >= 4096) return false; JointIdentity id; slotIdentity(i, &id);
  const auto* tr = c.actuator_policy_.t.find(id, c.geometry_profile_.provenanceTag()); MicroRad urad = 0;
  if (!tr || resolveRawObservation(c.geometry_profile_, geometry_data::kProvenance, *tr, raw, &urad) != TargetResolveStatus::OK) return false;
  q = microRadToRadians(urad); return true;
}
static bool lookupResolve(core::Controller& c, unsigned i, double q, uint16_t& raw) {
  if (i >= 12) return false; JointIdentity id; slotIdentity(i, &id);
  const auto* tr = c.actuator_policy_.t.find(id, c.geometry_profile_.provenanceTag());
  const auto* lim = c.actuator_policy_.l.find(id, c.geometry_profile_.provenanceTag());
  return tr && lim && resolveRadiansToRaw(c.geometry_profile_, geometry_data::kProvenance, *tr, q, &raw) == TargetResolveStatus::OK && lim->contains(raw);
}
static unsigned long okDecode = 0, okResolve = 0;
static void sweep(core::Controller& c, const char* step) {
  for (unsigned i = 0; i < 13; ++i) {  // slot 12 does not exist: both refuse
    for (unsigned r = 0; r < 4101; ++r) {
      double a = -7, b = -7; const bool x = c.port.decodeRaw(i, uint16_t(r), a), y = lookupDecode(c, i, uint16_t(r), b);
      check(x == y && (!x || std::memcmp(&a, &b, sizeof a) == 0), step, "decodeRaw", i, r); okDecode += x;
    }
    std::vector<double> qs;
    for (double q = -4.0; q <= 4.0; q += 6.283185307179586 / 4096 / 4) qs.push_back(q);
    for (int k = -2100; k <= 2100; ++k) { const double m = (k + .5) * 6283186.0 / 4096;
      for (double u : {std::floor(m), std::ceil(m)}) for (double e : {-.4e-6, .4e-6}) qs.push_back(u / 1e6 + e); }
    JointIdentity id; slotIdentity(i, &id);
    if (const auto* j = c.geometry_profile_.findJoint(id))
      for (MicroRad lim : {j->urdf_lower, j->urdf_upper}) for (int d = -2; d <= 2; ++d) for (double e : {-.49e-6, 0., .49e-6}) qs.push_back((lim + d) / 1e6 + e);
    for (double s : {double(NAN), double(INFINITY), -double(INFINITY), 2000., -2000., 2000.000001, -2000.000001, 0., -0.}) qs.push_back(s);
    uint64_t x = 0x9e3779b97f4a7c15ull + i;
    for (unsigned n = 0; n < 4000; ++n) { x = x * 6364136223846793005ull + 1442695040888963407ull; qs.push_back(-3.5 + 7.0 * double(x >> 11) / double(1ull << 53)); }
    for (double q : qs) {
      uint16_t a = 0xffff, b = 0xffff; const bool u = c.port.resolve(i, q, a), v = lookupResolve(c, i, q, b);
      check(u == v && (!u || a == b), step, "resolve", i, q); okResolve += u;
    }
  }
}
int main() {
  static core::Controller c; c.port.owner = &c; c.geometry_profile_ = golden::boundProfile();
  Storage storage; CalibrationPersistenceService persistence{&storage}; CalibrationRuntimeRestore restore;
  storage.slot[0].resize(kCalibrationRecordV1EncodedBytes); size_t size = 0;
  encodeCalibrationRecord(golden::goldenRecord(), storage.slot[0].data(), storage.slot[0].size(), &size);
  SaveMarker m; m.state = SaveMarkerState::IDLE; m.acknowledged_generation = 1; m.begun_generation = 1;
  storage.marker.resize(kSaveMarkerV1Bytes); encodeSaveMarker(m, storage.marker.data(), storage.marker.size(), &size);
  persistence.begin(NvsInitStatus::READY, 0);
  RuntimeRestoreRequest f; f.expected_generation = 1; f.current_installation_confirmed = true; f.executor_idle = true; f.torque_off_confirmed = true; f.motion_permit_revoked = true;
  const bool restored = restore.restore(persistence, c.geometry_profile_, geometry_data::kProvenance, f, c.actuator_policy_.t, c.actuator_policy_.l) == RuntimeRestoreStatus::RESTORED &&
      restore.qualifyCommissioningLimits(c.geometry_profile_, geometry_data::kProvenance, 5, f, c.actuator_policy_.t, c.actuator_policy_.l) == OperationalAdmissionStatus::APPROVED;
  check(restored && c.actuator_policy_.t.size() == 12 && c.actuator_policy_.l.size() == 12, "fixture", "golden restore", 0, 0);
  sweep(c, "restored"); sweep(c, "restored_again");
  JointTransform t[12]; JointLimit l[12];
  for (unsigned i = 0; i < 12; ++i) { JointIdentity id; slotIdentity(i, &id);
    t[i] = *c.actuator_policy_.t.find(id, c.geometry_profile_.provenanceTag()); l[i] = *c.actuator_policy_.l.find(id, c.geometry_profile_.provenanceTag()); }
  auto revision = [&] { return c.geometry_profile_.revision() * 1000003ull + c.actuator_policy_.t.revision() * 1009ull + c.actuator_policy_.l.revision(); };
  auto changed = [&](unsigned long long before, const char* step) { check(revision() != before, step, "revision changed", 0, 0); sweep(c, step); };
  unsigned long long r = revision(); c.actuator_policy_.t.clear(); changed(r, "transforms_cleared");
  r = revision(); for (int i = 11; i >= 0; --i) { c.actuator_policy_.t.admit(t[i]); } changed(r, "transforms_reversed_order");
  r = revision(); { JointTransform q0 = t[0]; q0.q0_tick = uint16_t(q0.q0_tick + 7); c.actuator_policy_.t.admit(q0); } changed(r, "slot0_new_q0");
  r = revision(); c.actuator_policy_.t.admit(t[0]); changed(r, "slot0_q0_restored");
  r = revision(); c.actuator_policy_.l.clear(); changed(r, "limits_cleared");
  r = revision(); for (int i = 11; i >= 0; --i) { c.actuator_policy_.l.admit(l[i]); } changed(r, "limits_reversed_order");
  r = revision(); { JointLimit n = l[4]; n.max_tick = uint16_t(n.max_tick - 40); c.actuator_policy_.l.admit(n); } changed(r, "slot4_limit_narrowed");
  r = revision(); c.geometry_profile_.clear(); changed(r, "profile_cleared");
  r = revision(); c.geometry_profile_ = golden::boundProfile(); sweep(c, "profile_rebound");
  static GeometryProvenance other = geometry_data::kProvenance; other.urdf_sha256[0] = other.urdf_sha256[0] == '0' ? '1' : '0';
  r = revision(); c.geometry_profile_.bind(&other, geometry_data::kJoints, geometry_data::kJointCount, geometry_data::kEndpoints, geometry_data::kEndpointCount); changed(r, "other_geometry");
  r = revision(); c.geometry_profile_.bind(&geometry_data::kProvenance, geometry_data::kJoints, geometry_data::kJointCount, geometry_data::kEndpoints, geometry_data::kEndpointCount); changed(r, "geometry_bound_again");
  std::printf("SERVO_PORT_CONVERSION %s checks=%lu failures=%lu (decode ok %lu, resolve ok %lu; 13 slots, 12 table/profile states)\n",
              failures ? "FAIL" : "PASS", checks, failures, okDecode, okResolve);
  return failures ? 1 : 0;
}
