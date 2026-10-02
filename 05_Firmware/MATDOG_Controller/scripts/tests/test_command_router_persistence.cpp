// Real USB command dispatch + persistence parser/handler. Only platform I/O
// and unrelated hardware service methods are simulated. Gate, Q0 session,
// promotion, service and marker/A/B store are the firmware implementations.
#include <Arduino.h>
#include <cstring>
#include <utility>
// Test-only access sets adverse cached states without issuing a bus command.
#define private public
#include "../../src/core/CommandRouter.h"
#include "../../src/core/ControllerService.h"
#include "../../src/calibration/FirstMotionExecutor.h"
#include "calibration_persistence_fixture.h"
#undef private

using namespace persistence_fixture;
using namespace matdog::core;
static int checks = 0, failures = 0;
#define CHECK(x) do { ++checks; if (!(x)) { ++failures; std::printf("FAIL %d: %s\n", __LINE__, #x); } } while (0)

class RouterStorage : public MemStorage {
 public:
  bool fail_next_marker = false;
  bool persist_on_error = false;
  StorageIoStatus writeMarker(const uint8_t* bytes, size_t length) override {
    if (!fail_next_marker) return MemStorage::writeMarker(bytes, length);
    fail_next_marker = false;
    if (persist_on_error) MemStorage::writeMarker(bytes, length);
    else ++marker_writes;
    return StorageIoStatus::IO_ERROR;
  }
};

struct Fixture {
  Scenario scenario;
  RouterStorage storage;
  CalibrationPersistenceService persistence{&storage};
  CalibrationQ0CaptureSession q0;
  ActuatorAuthorityArbiter authority;
  OperatingModeManager mode;
  CalibrationManager calibration;
  CalibrationMotionPermit permit;
  CalibrationMotionAuthorizationState authorization;
  actuator::SafeActuatorPolicy policy;
  FirstMotionExecutor first_motion;
  FullLegCalibrationExecutor full_leg;
  FullLegRunState run;
  servo::ServoBus bus;
  servo::ServoCensus census;
  servo::ServoPreflight preflight;
  servo::SafeOffResult safe_off = servo::SafeOffResult::VERIFIED_OFF;
  SystemState system;
  ControllerService telemetry;
  CommandRouter router;

  Fixture() {
    calibration.begin(&authority);
    policy.begin(&authority);
    policy.bindGeometry(&scenario.profile, &actuator::geometry_data::kProvenance);
    persistence.begin(NvsInitStatus::READY, 0);
    persistence.load(scenario.profile);
    CommandRouter::Modules m{};
    m.servo_bus = &bus; m.servo_census = &census; m.servo_preflight = &preflight;
    m.system_state = &system; m.operating_mode = &mode; m.authority = &authority;
    m.calibration = &calibration; m.q0_capture = &q0; m.actuator_policy = &policy;
    m.geometry_profile = &scenario.profile; m.motion_permit = &permit;
    m.motion_authorization = &authorization; m.first_motion = &first_motion;
    m.full_leg_calibration = &full_leg; m.full_leg_run = &run;
    m.full_leg_evidence = &scenario.evidence; m.persistence = &persistence;
    m.first_motion_safe_off_result = &safe_off;
    m.service = &telemetry; telemetry.begin(m); router.begin(m);
    CHECK(startCapture(q0)); CHECK(finishCapture(q0, scenario.golden));
    CHECK(command("@CALIBRATION Q0 PROMOTE CONFIRM_CURRENT_INSTALLATION").find("PROMOTE=OK") != std::string::npos);
  }
  std::string command(const char* line) {
    Serial.output.clear();
    for (const char* p = line; *p; ++p) Serial.input.push_back(*p);
    Serial.input.push_back('\n');
    router.update(millis());  // actual framing, normalization and handleLine()
    return Serial.output;
  }
  void awaiting2() {
    CHECK(command("@CALIBRATION PERSIST SAVE CONFIRM_SAVE_FULL_CALIBRATION").find("WRITTEN_AWAITING_ACK generation=1") != std::string::npos);
    CHECK(command("@CALIBRATION PERSIST ACK 1").find("ACK=OK") != std::string::npos);
    CHECK(command("@CALIBRATION PERSIST SAVE CONFIRM_SAVE_FULL_CALIBRATION").find("WRITTEN_AWAITING_ACK generation=2") != std::string::npos);
  }
  int writes() const { return storage.slot_writes + storage.marker_writes; }
};

void parser_and_read_only() {
  Fixture f; f.awaiting2();
  const char* malformed[] = {
    "@CALIBRATION PERSISTACK 2", "@CALIBRATION PERSISTSAVE CONFIRM_SAVE_FULL_CALIBRATION",
    "@CALIBRATION PERSIST", "@CALIBRATION PERSIST ACK", "@CALIBRATION PERSIST ACK -2",
    "@CALIBRATION PERSIST ACK 0", "@CALIBRATION PERSIST ACK 4294967296",
    "@CALIBRATION PERSIST ACK 9999999999999999999999999999999",
    "@CALIBRATION PERSIST ACK 2X", "@CALIBRATION PERSIST ACK +2",
    "@CALIBRATION PERSIST ACK 2 EXTRA", "@CALIBRATION PERSIST STATUS EXTRA",
    "@CALIBRATION PERSIST SAVE CHECK EXTRA", "@CALIBRATION PERSIST SAVE",
    "@CALIBRATION PERSIST SAVE CONFIRM_DISCARD", "@CALIBRATION PERSIST RECONCILE",
    "@CALIBRATION PERSIST RECONCILE ADOPT", "@CALIBRATION PERSIST RECONCILE ADOPT 0",
    "@CALIBRATION PERSIST RECONCILE ADOPT -1", "@CALIBRATION PERSIST RECONCILE ADOPT 4294967296",
    "@CALIBRATION PERSIST RECONCILE ADOPT 1X",
    "@CALIBRATION PERSIST RECONCILE ADOPT CONFIRM_DISCARD 1",
    "@CALIBRATION PERSIST RECONCILE CONFIRM_DISCARD ADOPT 1",
    "@CALIBRATION PERSIST RECONCILE ADOPT 1 CONFIRM_DISCARD EXTRA",
    "@CALIBRATION PERSIST RECONCILE DECLARE_NOTHING CONFIRM_DISCARD EXTRA",
    "@CALIBRATION PERSIST ACK 2 A B C D E F G", "@CALIBRATION PERSIST\tACK 2"
  };
  uint8_t marker[kSaveMarkerV1Bytes]; std::memcpy(marker, f.storage.marker, sizeof(marker));
  const int writes = f.writes();
  for (const char* cmd : malformed) {
    const auto out = f.command(cmd);
    CHECK(out.find("ACK=OK") == std::string::npos);
    CHECK(out.find("=REFUSED") != std::string::npos || out.find("UNKNOWN_COMMAND") != std::string::npos);
    CHECK(f.writes() == writes); CHECK(std::memcmp(marker, f.storage.marker, sizeof(marker)) == 0);
  }
  CHECK(!CommandRouter::isPersistCommand(String("@CALIBRATION PERSISTACK 2")));
  // Even a direct handler call cannot bypass the dispatch boundary check.
  f.router.handlePersistCommand(String("@CALIBRATION PERSISTACK 2"));
  CHECK(f.writes() == writes);
  CHECK(f.command("@CALIBRATION PERSIST RECONCILE ADOPT 1").find("DISCARD_NOT_CONFIRMED") != std::string::npos);
  CHECK(f.command("@CALIBRATION PERSIST RECONCILE DECLARE_NOTHING CONFIRM_DISCARD").find("DECLARE_REFUSED_ACKNOWLEDGED_INTACT") != std::string::npos);
  CHECK(f.command("@CALIBRATION PERSIST ACK 4294967295").find("WRONG_GENERATION") != std::string::npos);
  CHECK(f.command("@CALIBRATION PERSIST STATUS").find("AWAITING_ACK_GENERATION=2") != std::string::npos);
  CHECK(f.command("@CALIBRATION PERSIST SAVE CHECK").find("CHECK_REFUSED") != std::string::npos);
  CHECK(f.writes() == writes);
  CHECK(f.command("  @calibration persist   ack 2  ").find("ACK=OK") != std::string::npos);
  const int acknowledged_writes = f.writes();
  CHECK(f.command("@CALIBRATION PERSIST ACK 2").find("ALREADY_ACKNOWLEDGED") != std::string::npos);
  CHECK(f.command("@CALIBRATION PERSIST STATUS").find("ACKNOWLEDGED_GENERATION=2") != std::string::npos);
  CHECK(f.command("@CALIBRATION PERSIST SAVE CHECK").find("CHECK_OK") != std::string::npos);
  CHECK(f.writes() == acknowledged_writes);
  CHECK(f.authority.current() == ActuatorAuthority::NONE); CHECK(!f.permit.active());
  CHECK(f.policy.transforms().size() == 12); CHECK(router_test::hardware_calls == 0);
}

void identical_recapture_through_real_commands() {
  Fixture f;
  CHECK(f.command("@CALIBRATION PERSIST SAVE CHECK").find("CHECK_OK") != std::string::npos);
  const uint32_t first = f.q0.status().promoted_capture_session_id;
  CHECK(f.command("@CALIBRATION Q0 CAPTURE 9 16 CONFIRM_Q0_POSE").find("Q0=STARTED") != std::string::npos);
  CHECK(f.q0.status().capture_session_id != first); CHECK(f.q0.status().promoted_capture_session_id == 0);
  CHECK(f.q0.markCensusStarted()); CHECK(f.q0.submitCensus(goodCensus()));
  CHECK(f.q0.markPreflightStarted()); CHECK(f.q0.submitPreflight(goodPreflight()));
  CHECK(f.command("@CALIBRATION PERSIST SAVE CHECK").find("CHECK_REFUSED") != std::string::npos);
  CHECK(finishCapture(f.q0, f.scenario.golden));
  CHECK(f.command("@CALIBRATION PERSIST SAVE CHECK").find("REASON=Q0_NOT_PROMOTED") != std::string::npos);
  CHECK(f.command("@CALIBRATION Q0 PROMOTE").find("UNKNOWN_COMMAND") != std::string::npos);
  CHECK(f.q0.status().promoted_capture_session_id == 0);
  CHECK(f.command("@CALIBRATION Q0 PROMOTE CONFIRM_CURRENT_INSTALLATION").find("PROMOTE=OK") != std::string::npos);
  CHECK(f.q0.status().promoted_capture_session_id == f.q0.status().capture_session_id);
  CHECK(f.command("@CALIBRATION PERSIST SAVE CHECK").find("CHECK_OK") != std::string::npos);
  CHECK(f.writes() == 0); CHECK(router_test::hardware_calls == 0);
}

void authorization_gates() {
  // Each state is injected independently. The actual handler checks the real
  // module state before the persistence service can issue any storage write.
  const char* commands[] = {"@CALIBRATION PERSIST SAVE CONFIRM_SAVE_FULL_CALIBRATION",
                            "@CALIBRATION PERSIST ACK 2",
                            "@CALIBRATION PERSIST RECONCILE ADOPT 1 CONFIRM_DISCARD"};
  for (unsigned gate = 0; gate < 13; ++gate) {
    for (unsigned operation = 0; operation < 3; ++operation) {
      Fixture f;
      // SAVE starts from writable empty storage; ACK/RECONCILE from A/1 + B/2.
      // Otherwise an earlier AWAITING_ACK refusal could mask a missing SAVE gate.
      if (operation != 0) f.awaiting2();
      switch (gate) {
        case 0: f.mode.setMode(OperatingMode::RUN); break;
        case 1: f.calibration.status_.state = SessionState::ACTIVE; break;
        case 2: f.run.armed = true; break;
        case 3: f.first_motion.status_.state = FirstMotionState::MONITORING; break;
        case 4: f.full_leg.status_.step = FullLegStep::SAFE_OFF_ALL; break;
        case 5: CHECK(startCapture(f.q0)); break;
        case 6: f.bus.scan_state_ = servo::ScanState::RUNNING; break;
        case 7: f.census.state_ = servo::ServoCensus::State::RUNNING; break;
        case 8: f.preflight.state_ = servo::ServoPreflight::State::RUNNING; break;
        case 9: f.authority.owner_ = ActuatorAuthority::CALIBRATION; break;
        case 10: f.permit.active_ = true; break;
        case 11: f.authorization.operator_authorized = true; break;
        case 12: f.authorization.token = {1, 1, 1}; break;
      }
      const int writes = f.writes();
      CHECK(f.command(commands[operation]).find("=OK") == std::string::npos);
      CHECK(f.writes() == writes);
      CHECK(f.command("@CALIBRATION PERSIST STATUS").find("PERSIST=STATUS") != std::string::npos);
      CHECK(f.command("@CALIBRATION PERSIST SAVE CHECK").find("CHECK_REFUSED") != std::string::npos);
      CHECK(f.writes() == writes);
    }
  }
  // SAVE-specific prerequisites use current evidence as well.
  for (unsigned gate = 0; gate < 5; ++gate) {
    Fixture f;
    switch (gate) {
      case 0: f.scenario.evidence.reset(); break;
      case 1: f.q0.reset(); break;
      case 2: f.policy.transforms().clear(); break;
      case 3: f.first_motion.status_.state = FirstMotionState::COMPLETE;
              f.safe_off = servo::SafeOffResult::UNVERIFIED_NO_RESPONSE; break;
      case 4: f.persistence.begin(NvsInitStatus::PARTITION_MISSING, 0); break;
    }
    CHECK(f.command("@CALIBRATION PERSIST SAVE CHECK").find("CHECK_REFUSED") != std::string::npos);
    CHECK(f.command("@CALIBRATION PERSIST SAVE CONFIRM_SAVE_FULL_CALIBRATION").find("SAVE=REFUSED") != std::string::npos);
    CHECK(f.writes() == 0);
  }
  CHECK(router_test::hardware_calls == 0);
}
void uncertain_reconciliation_through_handler() {
  for (bool persist_on_error : {false, true}) {
    Fixture f; f.awaiting2();
    uint8_t acknowledged[kCalibrationRecordV1EncodedBytes];
    std::memcpy(acknowledged, f.storage.data[0], sizeof(acknowledged));
    f.storage.fail_next_marker = true; f.storage.persist_on_error = persist_on_error;
    CHECK(f.command("@CALIBRATION PERSIST RECONCILE ADOPT 1 CONFIRM_DISCARD").find("WRITES_BLOCKED=1") != std::string::npos);
    CHECK(f.persistence.writesBlocked());
    uint8_t marker[kSaveMarkerV1Bytes]; std::memcpy(marker, f.storage.marker, sizeof(marker));
    const int writes = f.writes();
    CHECK(f.command("@CALIBRATION PERSIST ACK 2").find("REASON=WRITES_BLOCKED") != std::string::npos);
    CHECK(f.command("@CALIBRATION PERSIST SAVE CONFIRM_SAVE_FULL_CALIBRATION").find("REASON=WRITES_BLOCKED") != std::string::npos);
    CHECK(f.command("@CALIBRATION PERSIST RECONCILE ADOPT 2 CONFIRM_DISCARD").find("REASON=WRITES_BLOCKED") != std::string::npos);
    CHECK(f.command("@CALIBRATION PERSIST STATUS").find("WRITES_BLOCKED=1") != std::string::npos);
    CHECK(f.writes() == writes);
    CHECK(std::memcmp(marker, f.storage.marker, sizeof(marker)) == 0);
    CHECK(std::memcmp(acknowledged, f.storage.data[0], sizeof(acknowledged)) == 0);
    CHECK(router_test::hardware_calls == 0);
  }
}
int main() {
  parser_and_read_only(); identical_recapture_through_real_commands(); authorization_gates();
  uncertain_reconciliation_through_handler();
  std::printf("test_command_router_persistence: %d checks, %d failures\n", checks, failures);
  return failures ? 1 : 0;
}
