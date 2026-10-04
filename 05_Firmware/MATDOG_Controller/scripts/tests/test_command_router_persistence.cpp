// Real USB command dispatch + persistence parser/handler. Only platform I/O
// and unrelated hardware service methods are simulated. Gate, Q0 session,
// promotion, service and marker/A/B store are the firmware implementations.
#include <Arduino.h>
#include <algorithm>
#include <cstring>
#include <utility>
#include <vector>
#include "router_nvs_stub.h"
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
static const char* framing_case = "";
#define CHECK(x) do { ++checks; if (!(x)) { ++failures; std::printf("FAIL %d [%s]: %s\n", __LINE__, framing_case, #x); } } while (0)

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
  CalibrationPersistenceService persistence;
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
  network::WifiManager wifi;
  CommandRouter router;

  explicit Fixture(CalibrationRecordStorage* backend = nullptr)
      : persistence(backend == nullptr ? &storage : backend) {
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
    m.wifi = &wifi; m.service = &telemetry; telemetry.begin(m); router.begin(m);
    CHECK(startCapture(q0)); CHECK(finishCapture(q0, scenario.golden));
    CHECK(command("@CALIBRATION Q0 PROMOTE CONFIRM_CURRENT_INSTALLATION").find("PROMOTE=OK") != std::string::npos);
  }
  // Explicit length is essential: a C-string loop cannot inject an embedded NUL.
  // Every raw chunk corresponds to one real CommandRouter::update() call.
  std::string raw(const char* bytes, size_t length) {
    Serial.output.clear();
    for (size_t i = 0; i < length; ++i) Serial.input.push_back(bytes[i]);
    router.update(millis());  // actual framing, normalization and handleLine()
    return Serial.output;
  }
  std::string raw(const std::string& bytes) { return raw(bytes.data(), bytes.size()); }
  std::string command(const char* line) { return raw(std::string(line) + '\n'); }
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
  f.full_leg.recovery_witness_ = true;
  CHECK(f.command("@CALIBRATION Q0 CAPTURE 9 16 CONFIRM_Q0_POSE").find("EXISTING_EVIDENCE_REQUIRES_EXPLICIT_DISCARD") != std::string::npos);
  CHECK(f.q0.status().promoted_capture_session_id == first);
  CHECK(f.full_leg.recovery_witness_);
  CHECK(f.scenario.evidence.legsPresent() == 4);
  CHECK(f.command("@CALIBRATION EVIDENCE DISCARD").find("UNKNOWN_COMMAND") != std::string::npos);
  CHECK(f.scenario.evidence.legsPresent() == 4);
  CHECK(f.command("@CALIBRATION EVIDENCE DISCARD CONFIRM_NEW_Q0").find("DISCARD=OK scope=RAM_ONLY NVS=UNCHANGED") != std::string::npos);
  CHECK(f.scenario.evidence.legsPresent() == 0);
  CHECK(!f.full_leg.recovery_witness_);
  CHECK(f.policy.transforms().size() == 12); // discard is not an unpromotion
  CHECK(f.q0.status().promoted_capture_session_id == first);
  CHECK(f.command("@CALIBRATION Q0 CAPTURE 9 16 CONFIRM_Q0_POSE").find("Q0=STARTED") != std::string::npos);
  CHECK(f.q0.status().capture_session_id != first); CHECK(f.q0.status().promoted_capture_session_id == 0);
  CHECK(f.q0.markCensusStarted()); CHECK(f.q0.submitCensus(goodCensus()));
  CHECK(f.q0.markPreflightStarted()); CHECK(f.q0.submitPreflight(goodPreflight()));
  CHECK(f.command("@CALIBRATION PERSIST SAVE CHECK").find("CHECK_REFUSED") != std::string::npos);
  CHECK(finishCapture(f.q0, f.scenario.golden));
  CHECK(f.command("@CALIBRATION PERSIST SAVE CHECK").find("CHECK_REFUSED") != std::string::npos);
  CHECK(f.command("@CALIBRATION Q0 PROMOTE").find("UNKNOWN_COMMAND") != std::string::npos);
  CHECK(f.q0.status().promoted_capture_session_id == 0);
  CHECK(f.command("@CALIBRATION Q0 PROMOTE CONFIRM_CURRENT_INSTALLATION").find("PROMOTE=OK") != std::string::npos);
  CHECK(f.q0.status().promoted_capture_session_id == f.q0.status().capture_session_id);
  CHECK(f.command("@CALIBRATION PERSIST SAVE CHECK").find("CHECK_REFUSED") != std::string::npos); // old 24/24 cannot be combined with this capture
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
        case 12: f.authorization.token = {false, 1, 1, 1}; break;
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

// P3a.2: byte-level USB framing. The same cases run with memory storage and
// through the real CalibrationRecordNvsBackend + a simulated platform NVS API.
constexpr size_t kMaxUsbLine = 95;
const char* const kMutatingCommands[] = {
    "@CALIBRATION PERSIST SAVE CONFIRM_SAVE_FULL_CALIBRATION",
    "@CALIBRATION PERSIST ACK 2",
    "@CALIBRATION PERSIST RECONCILE ADOPT 1 CONFIRM_DISCARD",
};
const char* const kSuccessfulReplies[] = {"WRITTEN_AWAITING_ACK generation=1", "ACK=OK", "RECONCILE=OK"};

// Capture the actual objects' bytes, without manufacturing an expected state.
// Router receive state is excluded: it is the only state allowed to change.
std::vector<uint8_t> calibrationBytes(const Fixture& f) {
  std::vector<uint8_t> bytes;
  const auto append = [&bytes](const auto& object) {
    const auto* first = reinterpret_cast<const uint8_t*>(&object);
    bytes.insert(bytes.end(), first, first + sizeof(object));
  };
  append(f.calibration); append(f.q0); append(f.policy); append(f.scenario.evidence);
  append(f.run); append(f.first_motion); append(f.full_leg);
  append(f.authority); append(f.permit); append(f.authorization); append(f.safe_off);
  append(f.persistence.snapshot());
  return bytes;
}

struct FramingSnapshot {
  std::vector<uint8_t> calibration;
  bool present[2];
  std::vector<uint8_t> slot[2];
  bool marker_present;
  std::vector<uint8_t> marker;
  int writes;
  router_nvs_test::State nvs;

  explicit FramingSnapshot(const Fixture& f)
      : calibration(calibrationBytes(f)), marker_present(f.storage.marker_present),
        marker(f.storage.marker, f.storage.marker + f.storage.marker_size),
        writes(f.writes()), nvs(router_nvs_test::state) {
    for (unsigned i = 0; i < 2; ++i) {
      present[i] = f.storage.present[i];
      slot[i].assign(f.storage.data[i], f.storage.data[i] + f.storage.size[i]);
    }
  }
  void checkUnchanged(const Fixture& f) const {
    CHECK(calibrationBytes(f) == calibration);
    CHECK(f.writes() == writes);
    for (unsigned i = 0; i < 2; ++i) {
      CHECK(f.storage.present[i] == present[i]);
      CHECK(f.storage.size[i] == slot[i].size());
      CHECK(std::equal(slot[i].begin(), slot[i].end(), f.storage.data[i]));
    }
    CHECK(f.storage.marker_present == marker_present);
    CHECK(f.storage.marker_size == marker.size());
    CHECK(std::equal(marker.begin(), marker.end(), f.storage.marker));
    const auto& actual = router_nvs_test::state;
    CHECK(actual.blobs == nvs.blobs);
    CHECK(actual.namespace_present == nvs.namespace_present);
    CHECK(actual.set_calls == nvs.set_calls);
    CHECK(actual.commit_calls == nvs.commit_calls);
    CHECK(actual.rw_open_calls == nvs.rw_open_calls);
    CHECK(actual.contract_errors == 0);
    CHECK(router_test::hardware_calls == 0);
  }
};

template <typename Test>
void withFramingFixture(bool nvs, const Test& test) {
  router_nvs_test::reset();
  if (nvs) {
    CalibrationRecordNvsBackend backend;
    CHECK(backend.begin() == NvsInitStatus::READY);
    Fixture f(&backend);
    test(f);
  } else {
    Fixture f;
    test(f);
  }
  CHECK(router_nvs_test::state.contract_errors == 0);
}

void checkReceiveReset(const Fixture& f) {
  CHECK(f.router.line_len_ == 0);
  CHECK(f.router.line_error_ == CommandRouter::LineError::NONE);
  CHECK(std::all_of(std::begin(f.router.line_buf_), std::end(f.router.line_buf_),
                    [](char c) { return c == 0; }));
}

void checkSuccessfulMutation(Fixture& f, unsigned operation, bool nvs, const std::string& raw) {
  const int writes = f.writes();
  const unsigned sets = router_nvs_test::state.set_calls;
  const unsigned commits = router_nvs_test::state.commit_calls;
  CHECK(f.raw(raw).find(kSuccessfulReplies[operation]) != std::string::npos);
  const unsigned expected = operation == 0 ? 3 : 1;  // two markers + slot, or marker only
  CHECK(f.writes() - writes == (nvs ? 0 : static_cast<int>(expected)));
  CHECK(router_nvs_test::state.set_calls - sets == (nvs ? expected : 0));
  CHECK(router_nvs_test::state.commit_calls - commits == (nvs ? expected : 0));
  checkReceiveReset(f);
}

void valid_usb_framing_limits() {
  CHECK(CommandRouter::kLineBufSize == 96);  // pinned contract, no buffer growth
  framing_case = "valid USB line boundary / CRLF";
  for (bool nvs : {false, true}) {
    for (unsigned operation = 0; operation < 3; ++operation) {
      for (bool maximum : {false, true}) {
        for (bool crlf : {false, true}) {
          withFramingFixture(nvs, [&](Fixture& f) {
            if (operation != 0) f.awaiting2();
            std::string payload(kMutatingCommands[operation]);
            if (maximum) payload.append(kMaxUsbLine - payload.size(), ' ');
            const FramingSnapshot before(f);
            // CRLF split across update() calls: CR adds no byte to the line.
            if (crlf) {
              CHECK(f.raw(payload + '\r').empty());
              before.checkUnchanged(f);
              checkSuccessfulMutation(f, operation, nvs, "\n");
            } else {
              checkSuccessfulMutation(f, operation, nvs, payload + '\n');
            }
          });
        }
      }
    }
  }
}

struct InvalidUsbLine {
  const char* name;
  std::string payload;  // deliberately no LF; embedded NULs are kept by length
  const char* error;
};

std::vector<InvalidUsbLine> invalidUsbLines(const std::string& command) {
  const std::string maximum = command + std::string(kMaxUsbLine - command.size(), ' ');
  const std::string nul(1, '\0');
  return {
      {"overflow by one byte", maximum + ' ', "ERROR=COMMAND_LINE_OVERFLOW\n"},
      {"original overflow EXTRA", maximum + "EXTRA", "ERROR=COMMAND_LINE_OVERFLOW\n"},
      {"very long line", maximum + std::string(4096, 'X') + command, "ERROR=COMMAND_LINE_OVERFLOW\n"},
      {"original internal NUL", command + nul + "EXTRA", "ERROR=COMMAND_LINE_NUL\n"},
      {"NUL first byte", nul + command, "ERROR=COMMAND_LINE_NUL\n"},
      {"multiple NUL bytes", command + nul + nul + command + nul, "ERROR=COMMAND_LINE_NUL\n"},
      {"NUL then overlong suffix", command + nul + std::string(512, ' ') + command, "ERROR=COMMAND_LINE_NUL\n"},
      {"overflow then NUL", maximum + 'X' + nul + command, "ERROR=COMMAND_LINE_OVERFLOW\n"},
      {"NUL before another mutating command", command + nul + kMutatingCommands[0], "ERROR=COMMAND_LINE_NUL\n"},
      {"invalid line with CRLF", command + nul + "EXTRA\r", "ERROR=COMMAND_LINE_NUL\n"},
  };
}

void invalid_usb_lines_and_recovery() {
  for (bool nvs : {false, true}) {
    for (unsigned operation = 0; operation < 3; ++operation) {
      for (const auto& invalid : invalidUsbLines(kMutatingCommands[operation])) {
        framing_case = invalid.name;
        for (bool fragmented : {false, true}) {
          withFramingFixture(nvs, [&](Fixture& f) {
            if (operation != 0) f.awaiting2();
            const FramingSnapshot before(f);
            std::string reply;
            if (fragmented) {
              // Cuts immediately before/after the NUL and the capacity boundary.
              std::vector<size_t> cuts{1, std::strlen(kMutatingCommands[operation]),
                                      std::strlen(kMutatingCommands[operation]) + 1,
                                      kMaxUsbLine, kMaxUsbLine + 1, invalid.payload.size()};
              std::sort(cuts.begin(), cuts.end());
              size_t offset = 0;
              for (size_t end : cuts) {
                if (end <= offset || end > invalid.payload.size()) continue;
                CHECK(f.raw(invalid.payload.data() + offset, end - offset).empty());
                before.checkUnchanged(f);
                offset = end;
              }
              reply = f.raw("\n", 1);
            } else {
              reply = f.raw(invalid.payload + '\n');
            }
            CHECK(reply == invalid.error);  // no parser/handler output, only framing error
            before.checkUnchanged(f);
            checkReceiveReset(f);
            // The next line is usable; STATUS is read-only even after a bad line.
            CHECK(f.raw("@CALIBRATION PERSIST STATUS\n").find("PERSIST=STATUS") != std::string::npos);
            before.checkUnchanged(f);
            // The rejected mutation was eligible, and the same valid command now works.
            checkSuccessfulMutation(f, operation, nvs, std::string(kMutatingCommands[operation]) + '\n');
          });
        }
      }
    }
  }
}

void usb_multiple_lines_and_begin_reset() {
  for (bool nvs : {false, true}) {
    for (const auto& invalid : invalidUsbLines(kMutatingCommands[0])) {
      framing_case = invalid.name;
      withFramingFixture(nvs, [&](Fixture& f) {
        const FramingSnapshot before(f);
        // All three lines are drained by ONE update(), with a valid line immediately
        // following the invalid one. CHECK exercises the real SAVE gate without writing.
        const auto out = f.raw(invalid.payload + "\n@CALIBRATION PERSIST STATUS\n"
                               "@CALIBRATION PERSIST SAVE CHECK\n");
        CHECK(out.find(invalid.error) == 0);
        CHECK(out.find("PERSIST=STATUS") != std::string::npos);
        CHECK(out.find("SAVE=CHECK_OK") != std::string::npos);
        CHECK(out.find("WRITTEN_AWAITING_ACK") == std::string::npos);
        before.checkUnchanged(f);
        checkReceiveReset(f);
      });
      withFramingFixture(nvs, [&](Fixture& f) {
        const FramingSnapshot before(f);
        CHECK(f.raw(invalid.payload).empty());  // invalid state remains latched without LF
        before.checkUnchanged(f);
        f.router.begin(f.router.modules_);
        checkReceiveReset(f);
        CHECK(f.raw("@CALIBRATION PERSIST STATUS\n").find("PERSIST=STATUS") != std::string::npos);
        before.checkUnchanged(f);
      });
    }
    framing_case = "begin clears valid unfinished prefix / existing controls";
    withFramingFixture(nvs, [&](Fixture& f) {
      const FramingSnapshot before(f);
      CHECK(f.raw(kMutatingCommands[0], std::strlen(kMutatingCommands[0])).empty());
      f.router.begin(f.router.modules_);
      checkReceiveReset(f);
      CHECK(f.raw("\r@CALIBRATION PERSIST STATUS\r").empty());  // bare CR never dispatches
      CHECK(f.raw("\n").find("PERSIST=STATUS") != std::string::npos);
      // Other controls retain their existing parser meaning: they are not removed
      // to reconstruct a mutating token, and this malformed confirmation is refused.
      CHECK(f.raw(std::string(kMutatingCommands[0]) + '\x01' + '\n').find("=REFUSED") != std::string::npos);
      before.checkUnchanged(f);
      CHECK(f.raw("\n\r\n").empty());  // empty LF/CRLF lines do nothing
      checkReceiveReset(f);
    });
  }
  framing_case = "";
}

void network_quiet_reservation() {
  Fixture f;
  f.wifi.config_busy_.store(true);
  const int writes = f.writes();
  CHECK(f.command("@MODE RUN").find("NETWORK_CONFIG_BUSY") != std::string::npos);
  CHECK(f.mode.mode() == OperatingMode::MAINTENANCE);
  CHECK(f.command("@CALIBRATION PERSIST SAVE CONFIRM_SAVE_FULL_CALIBRATION").find("NETWORK_CONFIG_BUSY") != std::string::npos);
  CHECK(f.command("@CALIBRATION Q0 CAPTURE 9 16 CONFIRM_Q0_POSE").find("NETWORK_CONFIG_BUSY") != std::string::npos);
  CHECK(f.command("@CALIBRATION PERSIST STATUS").find("PERSIST=STATUS") != std::string::npos);
  CHECK(f.command("@WIFI AP KEYY never-print-this-secret").find("never-print-this-secret") == std::string::npos);
  CHECK(f.command("@WIFI STATUS").find("WIFI_STATE") != std::string::npos);
  CHECK(f.writes() == writes);
  f.wifi.config_busy_.store(false);
}

int main() {
  network_quiet_reservation();
  parser_and_read_only(); identical_recapture_through_real_commands(); authorization_gates();
  uncertain_reconciliation_through_handler();
  valid_usb_framing_limits(); invalid_usb_lines_and_recovery(); usb_multiple_lines_and_begin_reset();
  std::printf("test_command_router_persistence: %d checks, %d failures\n", checks, failures);
  return failures ? 1 : 0;
}
