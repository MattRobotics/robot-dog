// Offline host tests for the G2 servo population / census classification
// and the hardware-profile expectation derivation.
//
// No device I/O, no hardware, no Arduino runtime: this links the REAL
// firmware sources (src/servo/ServoPopulation.cpp, src/core/Availability.cpp)
// rather than a host-side copy of the rules, so the thing under test is the
// thing that ships. That is why those translation units were made
// Arduino-free in G2.
//
// No test framework is pulled in for this (handoff: "do not add a
// heavyweight test framework just for this") — a CHECK macro and a
// pass/fail tally, mirroring the style/outputs of the existing Python OTA
// suite. Build and run with scripts/tests/run_host_tests.sh.

#include <cstdio>
#include <cstring>
#include <vector>

#include "../../src/config/HardwareProfile.h"
#include "../../src/core/Availability.h"
#include "../../src/servo/ServoPopulation.h"

using namespace matdog;
using servo::CensusResult;
using servo::CensusVerdict;
using servo::CurrentConfig;
using servo::IdClassification;

static int g_checks = 0;
static int g_failures = 0;
static const char* g_case = "";

#define CHECK(cond)                                                            \
  do {                                                                         \
    ++g_checks;                                                                \
    if (!(cond)) {                                                             \
      ++g_failures;                                                            \
      std::printf("  FAIL [%s] %s:%d: %s\n", g_case, __FILE__, __LINE__, #cond); \
    }                                                                          \
  } while (0)

#define CHECK_EQ(actual, expected)                                             \
  do {                                                                         \
    ++g_checks;                                                                \
    const long a_ = (long)(actual);                                            \
    const long e_ = (long)(expected);                                          \
    if (a_ != e_) {                                                            \
      ++g_failures;                                                            \
      std::printf("  FAIL [%s] %s:%d: %s == %ld, expected %ld\n", g_case,      \
                  __FILE__, __LINE__, #actual, a_, e_);                        \
    }                                                                          \
  } while (0)

// Runs a census over the full canonical range with the given responder set.
static CensusResult census(const std::vector<int>& observed) {
  return servo::classifyObserved(observed.data(), (int)observed.size(),
                                 (int)observed.size(), servo::kCanonicalScanLo,
                                 servo::kCanonicalScanHi);
}

// The 13 servos physically installed in the robot today.
static std::vector<int> installedNow() {
  return {11, 12, 13, 21, 22, 23, 31, 32, 33, 41, 42, 43, 51};
}

static bool listContains(const uint8_t* ids, uint8_t count, uint8_t id) {
  for (uint8_t i = 0; i < count; ++i) {
    if (ids[i] == id) return true;
  }
  return false;
}

// --- 1. canonical allocated count = 17 -------------------------------------
static void test_canonical_allocated_count() {
  g_case = "canonical_allocated_count";
  CHECK_EQ(servo::canonicalAllocatedCount(), 17);
  CHECK_EQ(servo::kCanonicalServoCount, 17);

  // Every canonical bus ID is unique — a duplicated row would silently
  // corrupt every count below.
  for (uint8_t i = 0; i < servo::kCanonicalServoCount; ++i) {
    for (uint8_t j = (uint8_t)(i + 1); j < servo::kCanonicalServoCount; ++j) {
      CHECK(servo::kCanonicalServos[i].bus_id != servo::kCanonicalServos[j].bus_id);
    }
  }
}

// --- 2. expected-now count = 13 --------------------------------------------
static void test_expected_now_count() {
  g_case = "expected_now_count";
  CHECK_EQ(servo::expectedNowCount(), 13);
  CHECK_EQ(servo::absentByDesignCount(), 4);

  // The 12 leg servos + neck rotation, individually.
  for (int id : installedNow()) {
    const servo::CanonicalServo* e = servo::findCanonical((uint8_t)id);
    CHECK(e != nullptr);
    if (e != nullptr) CHECK(e->current_config == CurrentConfig::INSTALLED);
  }
}

// --- 3. 52,53,54,55 are absent by design -----------------------------------
static void test_head_servos_absent_by_design() {
  g_case = "head_servos_absent_by_design";
  for (uint8_t id : {52, 53, 54, 55}) {
    const servo::CanonicalServo* e = servo::findCanonical(id);
    CHECK(e != nullptr);  // still canonically ALLOCATED
    if (e != nullptr) CHECK(e->current_config == CurrentConfig::ABSENT_BY_DESIGN);

    // Silent -> healthy, not a failure.
    CHECK(servo::classifyId(id, false, servo::kCanonicalScanLo,
                            servo::kCanonicalScanHi) ==
          IdClassification::ABSENT_BY_DESIGN);
  }
}

// --- 4. all expected present, no unexpected -> PASS ------------------------
static void test_healthy_current_population_passes() {
  g_case = "healthy_current_population_passes";
  const CensusResult c = census(installedNow());

  // The exact healthy powered census the handoff specifies.
  CHECK_EQ(c.canonical_allocated, 17);
  CHECK_EQ(c.expected_now, 13);
  CHECK_EQ(c.present_expected, 13);
  CHECK_EQ(c.absent_by_design, 4);
  CHECK_EQ(c.missing_expected, 0);
  CHECK_EQ(c.unexpected_id, 0);
  CHECK_EQ(c.absent_by_design_present, 0);
  CHECK_EQ(c.not_probed, 0);
  CHECK(!c.truncated);
  CHECK(c.verdict == CensusVerdict::PASS);
}

// --- 5. one expected servo missing -> MISSING_EXPECTED + mismatch ----------
static void test_one_expected_servo_missing() {
  g_case = "one_expected_servo_missing";
  std::vector<int> observed = installedNow();
  observed.erase(observed.begin() + 4);  // drop ID 22 (RF_UPPER)

  const CensusResult c = census(observed);
  CHECK_EQ(c.present_expected, 12);
  CHECK_EQ(c.missing_expected, 1);
  CHECK_EQ(c.absent_by_design, 4);   // unchanged: still healthy
  CHECK_EQ(c.unexpected_id, 0);
  CHECK_EQ(c.missing_id_count, 1);
  CHECK(listContains(c.missing_ids, c.missing_id_count, 22));
  CHECK(c.verdict == CensusVerdict::PROFILE_MISMATCH);

  CHECK(servo::classifyId(22, false, servo::kCanonicalScanLo,
                          servo::kCanonicalScanHi) ==
        IdClassification::MISSING_EXPECTED);
}

// --- 6. all four absent-by-design absent is NOT a failure ------------------
static void test_absent_by_design_is_not_a_failure() {
  g_case = "absent_by_design_is_not_a_failure";
  const CensusResult c = census(installedNow());

  // The whole point of the gate: 13 of 17 responding is a PASS.
  CHECK(c.verdict == CensusVerdict::PASS);
  CHECK_EQ(c.absent_by_design, 4);
  CHECK_EQ(c.missing_expected, 0);

  // And they must not have been quietly counted as present either.
  CHECK(c.present_expected != c.canonical_allocated);
}

// --- 7. an absent-by-design servo appears -> surfaced, not ignored ---------
static void test_absent_by_design_servo_appears() {
  g_case = "absent_by_design_servo_appears";
  std::vector<int> observed = installedNow();
  observed.push_back(53);  // HEAD_ROTATION physically fitted without the config saying so

  const CensusResult c = census(observed);
  CHECK_EQ(c.absent_by_design_present, 1);
  CHECK_EQ(c.absent_by_design, 3);
  CHECK_EQ(c.present_expected, 13);
  CHECK_EQ(c.unexpected_id, 0);  // it IS canonically allocated, so not "unknown"
  CHECK(listContains(c.absent_by_design_present_ids,
                     c.absent_by_design_present_id_count, 53));
  CHECK(c.verdict == CensusVerdict::PROFILE_MISMATCH);

  CHECK(servo::classifyId(53, true, servo::kCanonicalScanLo,
                          servo::kCanonicalScanHi) ==
        IdClassification::ABSENT_BY_DESIGN_PRESENT);
}

// --- 8. an unallocated ID appears -> UNEXPECTED_ID -------------------------
static void test_unallocated_id_appears() {
  g_case = "unallocated_id_appears";
  std::vector<int> observed = installedNow();
  observed.push_back(37);  // not in the MATDOG allocation at all

  const CensusResult c = census(observed);
  CHECK_EQ(c.unexpected_id, 1);
  CHECK_EQ(c.unexpected_id_count, 1);
  CHECK(listContains(c.unexpected_ids, c.unexpected_id_count, 37));
  CHECK_EQ(c.present_expected, 13);
  CHECK_EQ(c.missing_expected, 0);
  CHECK(c.verdict == CensusVerdict::PROFILE_MISMATCH);

  CHECK(servo::classifyId(37, true, servo::kCanonicalScanLo,
                          servo::kCanonicalScanHi) == IdClassification::UNEXPECTED_ID);
  // A silent unallocated ID is the normal case for most of the address
  // space and must not be reported as an anomaly.
  CHECK(servo::classifyId(37, false, servo::kCanonicalScanLo,
                          servo::kCanonicalScanHi) == IdClassification::NOT_PROBED);
}

// --- 9. multiple unexpected IDs -> correct structured result ---------------
static void test_multiple_unexpected_ids() {
  g_case = "multiple_unexpected_ids";
  std::vector<int> observed = installedNow();
  observed.push_back(14);
  observed.push_back(37);
  observed.push_back(49);

  const CensusResult c = census(observed);
  CHECK_EQ(c.unexpected_id, 3);
  CHECK_EQ(c.unexpected_id_count, 3);
  CHECK(listContains(c.unexpected_ids, c.unexpected_id_count, 14));
  CHECK(listContains(c.unexpected_ids, c.unexpected_id_count, 37));
  CHECK(listContains(c.unexpected_ids, c.unexpected_id_count, 49));
  CHECK_EQ(c.present_expected, 13);
  CHECK(c.verdict == CensusVerdict::PROFILE_MISMATCH);

  // Combined anomaly: a missing expected servo AND unexpected responders.
  std::vector<int> mixed = {11, 12, 13, 21, 22, 23, 31, 32, 33, 41, 42, 99};
  const CensusResult m = census(mixed);
  CHECK_EQ(m.present_expected, 11);
  CHECK_EQ(m.missing_expected, 2);  // 43 and 51
  CHECK_EQ(m.unexpected_id, 1);     // 99
  CHECK_EQ(m.absent_by_design, 4);
  CHECK(listContains(m.missing_ids, m.missing_id_count, 43));
  CHECK(listContains(m.missing_ids, m.missing_id_count, 51));
  CHECK(m.verdict == CensusVerdict::PROFILE_MISMATCH);
}

// --- 10. canonical allocation and expected-now stay distinct ---------------
static void test_canonical_and_expected_now_remain_distinct() {
  g_case = "canonical_and_expected_now_remain_distinct";
  CHECK(servo::canonicalAllocatedCount() != servo::expectedNowCount());
  CHECK_EQ(servo::canonicalAllocatedCount() - servo::expectedNowCount(),
           servo::absentByDesignCount());

  // A census result carries BOTH, so no consumer can conflate them.
  const CensusResult c = census(installedNow());
  CHECK(c.canonical_allocated != c.expected_now);
  CHECK_EQ(c.present_expected + c.missing_expected, c.expected_now);
  CHECK_EQ(c.absent_by_design + c.absent_by_design_present,
           servo::absentByDesignCount());

  // A full 17-responder bus is NOT the healthy current configuration — it
  // means four servos exist that the declared configuration says do not.
  std::vector<int> all17;
  for (uint8_t i = 0; i < servo::kCanonicalServoCount; ++i) {
    all17.push_back(servo::kCanonicalServos[i].bus_id);
  }
  const CensusResult a = census(all17);
  CHECK_EQ(a.present_expected, 13);
  CHECK_EQ(a.absent_by_design_present, 4);
  CHECK(a.verdict == CensusVerdict::PROFILE_MISMATCH);
}

// --- 11. USB_ONLY expected hardware semantics unchanged --------------------
static void test_usb_only_semantics_unchanged() {
  g_case = "usb_only_semantics_unchanged";
  constexpr config::ProfileExpectations usb =
      config::expectationsFor(config::HardwareProfile::USB_ONLY);

  CHECK(!usb.servo_power_available);
  CHECK(!usb.battery_available);
  CHECK(!usb.led_rail_powered);
  CHECK(std::strcmp(config::toString(config::HardwareProfile::USB_ONLY), "USB_ONLY") == 0);

  CHECK(core::expectedStateForServoBus(usb) == core::ExpectedState::EXPECTED_OFFLINE);
  CHECK(core::expectedStateForBattery(usb) == core::ExpectedState::EXPECTED_OFFLINE);
  CHECK(core::expectedStateForLedRail(usb) == core::ExpectedState::EXPECTED_UNPOWERED);
  CHECK(core::detectedStateForLedRail(usb) == core::DetectedState::UNPOWERED);

  // The V0.1-validated behaviour that must not regress: an unpowered servo
  // bus / BMS that does not answer is a PASS, not a fault.
  core::AvailabilityStatus servo_bus;
  servo_bus.init = core::InitializationState::INITIALIZED;
  servo_bus.detected = core::DetectedState::NO_RESPONSE;
  servo_bus.expected = core::expectedStateForServoBus(usb);
  CHECK(core::classify(servo_bus) == core::Classification::PASS);

  core::AvailabilityStatus bms;
  bms.init = core::InitializationState::INITIALIZED;
  bms.detected = core::DetectedState::NO_RESPONSE;
  bms.expected = core::expectedStateForBattery(usb);
  CHECK(core::classify(bms) == core::Classification::PASS);

  core::AvailabilityStatus led;
  led.init = core::InitializationState::DEFERRED;
  led.detected = core::detectedStateForLedRail(usb);
  led.expected = core::expectedStateForLedRail(usb);
  CHECK(core::classify(led) == core::Classification::PASS);
}

// --- 12. ROBOT_POWERED makes DALY/servo/LED expected as appropriate --------
static void test_robot_powered_expectations() {
  g_case = "robot_powered_expectations";
  constexpr config::ProfileExpectations pwr =
      config::expectationsFor(config::HardwareProfile::ROBOT_POWERED);

  CHECK(pwr.servo_power_available);
  CHECK(pwr.battery_available);
  CHECK(pwr.led_rail_powered);
  CHECK(std::strcmp(config::toString(config::HardwareProfile::ROBOT_POWERED),
                    "ROBOT_POWERED") == 0);

  CHECK(core::expectedStateForServoBus(pwr) == core::ExpectedState::REQUIRED);
  CHECK(core::expectedStateForBattery(pwr) == core::ExpectedState::REQUIRED);
  // OPTIONAL, never REQUIRED: a dead status ring must not fault the robot.
  CHECK(core::expectedStateForLedRail(pwr) == core::ExpectedState::OPTIONAL);
  CHECK(core::detectedStateForLedRail(pwr) == core::DetectedState::UNKNOWN);

  // The same unchanged NO_RESPONSE now classifies differently — this is the
  // whole point of reusing the V0.1 Availability model instead of replacing it.
  core::AvailabilityStatus servo_bus;
  servo_bus.init = core::InitializationState::INITIALIZED;
  servo_bus.detected = core::DetectedState::NO_RESPONSE;
  servo_bus.expected = core::expectedStateForServoBus(pwr);
  CHECK(core::classify(servo_bus) == core::Classification::FAULT);

  core::AvailabilityStatus bms;
  bms.init = core::InitializationState::INITIALIZED;
  bms.detected = core::DetectedState::NO_RESPONSE;
  bms.expected = core::expectedStateForBattery(pwr);
  CHECK(core::classify(bms) == core::Classification::FAULT);

  core::AvailabilityStatus led;
  led.init = core::InitializationState::INITIALIZED;
  led.detected = core::DetectedState::NO_RESPONSE;
  led.expected = core::expectedStateForLedRail(pwr);
  CHECK(core::classify(led) == core::Classification::DEGRADED);

  // An ONLINE servo bus passes under both profiles.
  servo_bus.detected = core::DetectedState::ONLINE;
  CHECK(core::classify(servo_bus) == core::Classification::PASS);
}

// --- 13. fail-closed: a partial scan can never report PASS ----------------
static void test_partial_scan_is_fail_closed() {
  g_case = "partial_scan_is_fail_closed";
  // Only the left-front leg was probed. Every other canonical ID was never
  // asked, so nothing can be concluded — and the 12 unprobed servos must
  // NOT be reported as missing.
  const std::vector<int> observed = {11, 12, 13};
  const CensusResult c =
      servo::classifyObserved(observed.data(), (int)observed.size(),
                              (int)observed.size(), 11, 13);

  CHECK_EQ(c.present_expected, 3);
  CHECK_EQ(c.missing_expected, 0);
  CHECK_EQ(c.not_probed, 14);
  CHECK(c.verdict == CensusVerdict::RANGE_INCOMPLETE);
  CHECK(c.verdict != CensusVerdict::PASS);

  // An anomaly observed inside a partial range still outranks
  // incompleteness: a real positive finding does not become inconclusive
  // just because the rest of the bus was not scanned.
  const std::vector<int> stray = {11, 12, 13, 19};
  const CensusResult s =
      servo::classifyObserved(stray.data(), (int)stray.size(), (int)stray.size(), 11, 19);
  CHECK_EQ(s.unexpected_id, 1);
  CHECK(s.verdict == CensusVerdict::PROFILE_MISMATCH);
}

// --- 14. fail-closed: a truncated responder list can never report PASS ----
static void test_truncated_scan_is_fail_closed() {
  g_case = "truncated_scan_is_fail_closed";
  const std::vector<int> observed = installedNow();
  // The underlying scan found more responders than its buffer could hold.
  const CensusResult c =
      servo::classifyObserved(observed.data(), (int)observed.size(),
                              (int)observed.size() + 5, servo::kCanonicalScanLo,
                              servo::kCanonicalScanHi);
  CHECK(c.truncated);
  CHECK(c.verdict == CensusVerdict::RANGE_INCOMPLETE);
  CHECK(c.verdict != CensusVerdict::PASS);

  // Degenerate inputs must not crash or claim success.
  const CensusResult empty = servo::classifyObserved(nullptr, 0, 0,
                                                     servo::kCanonicalScanLo,
                                                     servo::kCanonicalScanHi);
  CHECK_EQ(empty.present_expected, 0);
  CHECK_EQ(empty.missing_expected, 13);
  CHECK(empty.verdict == CensusVerdict::PROFILE_MISMATCH);

  // A default-constructed result is NOT_RUN, never PASS.
  const CensusResult fresh;
  CHECK(fresh.verdict == CensusVerdict::NOT_RUN);
}

int main() {
  std::printf("MATDOG G2 servo population / hardware profile offline tests\n");

  test_canonical_allocated_count();
  test_expected_now_count();
  test_head_servos_absent_by_design();
  test_healthy_current_population_passes();
  test_one_expected_servo_missing();
  test_absent_by_design_is_not_a_failure();
  test_absent_by_design_servo_appears();
  test_unallocated_id_appears();
  test_multiple_unexpected_ids();
  test_canonical_and_expected_now_remain_distinct();
  test_usb_only_semantics_unchanged();
  test_robot_powered_expectations();
  test_partial_scan_is_fail_closed();
  test_truncated_scan_is_fail_closed();

  std::printf("checks_run=%d failures=%d\n", g_checks, g_failures);
  if (g_failures != 0) {
    std::printf("SERVO_POPULATION_TESTS = FAIL\n");
    return 1;
  }
  std::printf("SERVO_POPULATION_TESTS = PASS\n");
  return 0;
}
