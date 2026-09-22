// Offline host tests for the MATDOG_C018_V1 persistent profile contract and
// the leg-servo selection the H0 preflight runs over
// (src/servo/ServoProfile.*, src/servo/ServoProfileData.h,
//  src/servo/ServoPopulation.*).
//
// Links the REAL generated register table, reduced by
// 06_Software/Matdog_Core/config/matdog_servo_profile_export.py from the
// reviewed MATDOG_ST3215_C018_V1.yaml. The twenty values under test are the
// exact ones the firmware carries; none is retyped here.
//
// NO TEST HERE IS HARDWARE VALIDATION. Everything proves a CONTRACT and a
// comparison. The preflight service that drives these against a real bus is
// device-only (it holds a ServoBus) and is exercised by H0, not by this suite.

#include <cstdio>
#include <cstring>
#include <initializer_list>

#include "../../src/servo/ServoPopulation.h"
#include "../../src/servo/ServoProfileData.h"

using namespace matdog::servo;

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

#define CHECK_EQ(actual, expected)                                       \
  do {                                                                   \
    ++g_checks;                                                          \
    const long long a_ = (long long)(actual);                            \
    const long long e_ = (long long)(expected);                          \
    if (a_ != e_) {                                                      \
      ++g_failures;                                                      \
      std::printf("  FAIL [%s] %s:%d: %s == %lld, expected %lld\n",      \
                  g_case, __FILE__, __LINE__, #actual, a_, e_);          \
    }                                                                    \
  } while (0)

// ---------------------------------------------------------------------------
// The compiled contract
// ---------------------------------------------------------------------------

static void test_the_persistent_profile_is_the_canonical_twenty() {
  g_case = "persistent profile shape";
  CHECK_EQ(profile_data::kPersistentRegisterCount, 20);
  CHECK(std::strcmp(profile_data::kProfileId, "MATDOG_C018_V1") == 0);

  uint8_t previous = 0;
  for (uint8_t i = 0; i < profile_data::kPersistentRegisterCount; ++i) {
    const ProfileRegister& reg = profile_data::kPersistentRegisters[i];
    // Sorted and unique, so the verifier walks the bus in address order and
    // no register is checked twice or skipped.
    if (i > 0) CHECK(reg.address > previous);
    previous = reg.address;
    CHECK(reg.width == 1 || reg.width == 2);
    CHECK(reg.expected <= (reg.width == 1 ? 0xFFu : 0xFFFFu));
    CHECK(reg.name != nullptr && reg.name[0] != '\0');
  }
}

static void test_runtime_ram_state_is_not_part_of_the_persistent_profile() {
  g_case = "runtime state stays out";
  // TorqueLimit, GoalSpeed and Acc are written per motion. Treating any of
  // them as a provisioning contract would make a motion setting look like a
  // fact about the servo - the YAML's own runtime_policy says they are not,
  // and the exporter refuses to emit one.
  for (uint8_t i = 0; i < profile_data::kPersistentRegisterCount; ++i) {
    const char* name = profile_data::kPersistentRegisters[i].name;
    CHECK(std::strcmp(name, "TorqueLimit") != 0);
    CHECK(std::strcmp(name, "GoalSpeed") != 0);
    CHECK(std::strcmp(name, "Acc") != 0);
    CHECK(std::strcmp(name, "GoalPosition") != 0);
  }
}

static void test_the_servo_invariants_are_a_separate_layer() {
  g_case = "invariants";
  const ServoInvariants& inv = profile_data::kInvariants;
  CHECK_EQ(inv.model_address, 0x03);
  CHECK_EQ(inv.model_expected, 777);          // ST-3215-C018
  CHECK_EQ(inv.position_offset_address, 0x1F);
  CHECK_EQ(inv.position_offset_expected, 0);  // all 17 units, 2026-08-27
  CHECK_EQ(inv.baud_address, 0x06);
  CHECK_EQ(inv.baud_expected, 0);             // 1 Mbps, verification only
  CHECK_EQ(inv.raw_center, 2048);
  CHECK_EQ(inv.center_tolerance_ticks, 1);

  // The invariants are NOT part of the twenty-register delta set: a unit is
  // "provisioned" by the twenty, and separately "the right kind of unit,
  // correctly zeroed" by these.
  for (uint8_t i = 0; i < profile_data::kPersistentRegisterCount; ++i) {
    const uint8_t addr = profile_data::kPersistentRegisters[i].address;
    CHECK(addr != inv.model_address);
    CHECK(addr != inv.position_offset_address);
    CHECK(addr != inv.baud_address);
  }
}

// ---------------------------------------------------------------------------
// Comparison semantics
// ---------------------------------------------------------------------------

static void test_a_failed_read_is_never_a_match() {
  g_case = "register comparison";
  const ProfileRegister reg = {0x0D, 1, 70, "MaxTemperature"};
  CHECK(checkRegister(reg, 70) == RegisterCheck::MATCH);
  CHECK(checkRegister(reg, 69) == RegisterCheck::MISMATCH);
  CHECK(checkRegister(reg, 0) == RegisterCheck::MISMATCH);
  // A negative observation is a failed read, not a value to compare. It must
  // never be silently treated as 0 - which would be a MISMATCH by luck here,
  // and a MATCH for any register whose expected value happens to be 0.
  CHECK(checkRegister(reg, -1) == RegisterCheck::NO_ANSWER);

  const ProfileRegister zero_expected = {0x17, 1, 0, "I"};
  CHECK(checkRegister(zero_expected, 0) == RegisterCheck::MATCH);
  CHECK(checkRegister(zero_expected, -1) == RegisterCheck::NO_ANSWER);
}

static void test_an_unread_register_can_never_produce_a_match_verdict() {
  g_case = "fold is fail-closed";
  // All twenty matched.
  ProfileVerdict verdict = ProfileVerdict::NOT_RUN;
  for (int i = 0; i < 20; ++i) verdict = foldRegisterCheck(verdict, RegisterCheck::MATCH);
  CHECK(verdict == ProfileVerdict::MATCH);

  // One register did not answer: the unit is INCOMPLETE, never MATCH. An
  // unread register is never assumed to hold its expected value.
  verdict = ProfileVerdict::NOT_RUN;
  for (int i = 0; i < 19; ++i) verdict = foldRegisterCheck(verdict, RegisterCheck::MATCH);
  verdict = foldRegisterCheck(verdict, RegisterCheck::NO_ANSWER);
  CHECK(verdict == ProfileVerdict::INCOMPLETE);
  // And it cannot recover afterwards.
  verdict = foldRegisterCheck(verdict, RegisterCheck::MATCH);
  CHECK(verdict == ProfileVerdict::INCOMPLETE);

  // A proven disagreement is worse news than a missing read, and is sticky.
  verdict = foldRegisterCheck(ProfileVerdict::NOT_RUN, RegisterCheck::MISMATCH);
  CHECK(verdict == ProfileVerdict::MISMATCH);
  verdict = foldRegisterCheck(verdict, RegisterCheck::MATCH);
  CHECK(verdict == ProfileVerdict::MISMATCH);
  verdict = foldRegisterCheck(verdict, RegisterCheck::NO_ANSWER);
  CHECK(verdict == ProfileVerdict::MISMATCH);

  // INCOMPLETE then MISMATCH still ends MISMATCH.
  verdict = foldRegisterCheck(ProfileVerdict::NOT_RUN, RegisterCheck::NO_ANSWER);
  verdict = foldRegisterCheck(verdict, RegisterCheck::MISMATCH);
  CHECK(verdict == ProfileVerdict::MISMATCH);

  // NOT_READ is treated exactly like NO_ANSWER: no evidence either way.
  CHECK(foldRegisterCheck(ProfileVerdict::NOT_RUN, RegisterCheck::NOT_READ) ==
        ProfileVerdict::INCOMPLETE);
}

static void test_position_offset_is_twos_complement() {
  g_case = "offset decode";
  CHECK_EQ(decodePositionOffset(0), 0);
  CHECK_EQ(decodePositionOffset(1), 1);
  CHECK_EQ(decodePositionOffset(85), 85);      // the provisioner's golden NEW01 value
  CHECK_EQ(decodePositionOffset(0xFFFF), -1);
  CHECK_EQ(decodePositionOffset(0x8000), -32768);
  CHECK_EQ(decodePositionOffset(0x7FFF), 32767);

  // The decoders genuinely disagree, which is why the encoding is pinned.
  // Sign-magnitude would read 0x8055 as -85; two's complement reads -32683.
  const uint16_t raw = 0x8055;
  const int16_t sign_magnitude = static_cast<int16_t>(-(raw & 0x7FFF));
  CHECK(decodePositionOffset(raw) != sign_magnitude);
  CHECK_EQ(decodePositionOffset(raw), -32683);
}

// ---------------------------------------------------------------------------
// The twelve the preflight runs over
// ---------------------------------------------------------------------------

static void test_the_leg_selection_is_exactly_twelve() {
  g_case = "leg selection";
  CHECK_EQ(kLegServoCount, 12);

  int found = 0;
  for (uint8_t i = 0; i < kCanonicalServoCount; ++i) {
    if (isLegServo(kCanonicalServos[i].bus_id)) ++found;
  }
  CHECK_EQ(found, kLegServoCount);

  // Head and neck are excluded by ADDRESS, not by name.
  for (int id : {51, 52, 53, 54, 55}) CHECK(!isLegServo(static_cast<uint8_t>(id)));
  for (int id : {11, 12, 13, 21, 22, 23, 31, 32, 33, 41, 42, 43}) CHECK(isLegServo(static_cast<uint8_t>(id)));
  // Nothing outside the allocation sneaks in.
  for (int id : {0, 1, 10, 14, 20, 24, 44, 50, 99}) CHECK(!isLegServo(static_cast<uint8_t>(id)));

  // Enumeration is stable, complete and terminates.
  uint8_t seen_ids[kLegServoCount] = {0};
  for (uint8_t i = 0; i < kLegServoCount; ++i) {
    const CanonicalServo* servo = legServoAt(i);
    CHECK(servo != nullptr);
    if (servo == nullptr) continue;
    seen_ids[i] = servo->bus_id;
    CHECK(isLegServo(servo->bus_id));
    CHECK(servo->current_config == CurrentConfig::INSTALLED);
    // The EXPECTED physical unit is carried for every leg joint - the
    // preflight reports it as configuration, never as an observation.
    CHECK(servo->physical_unit != nullptr && servo->physical_unit[0] != '\0');
  }
  CHECK(legServoAt(kLegServoCount) == nullptr);

  for (uint8_t i = 0; i < kLegServoCount; ++i) {
    for (uint8_t j = static_cast<uint8_t>(i + 1); j < kLegServoCount; ++j) {
      CHECK(seen_ids[i] != seen_ids[j]);
    }
  }
}

static void test_expected_units_match_the_allocation() {
  g_case = "expected physical units";
  // The 2026-08-27 allocation, as the preflight will report it. These are
  // EXPECTED values from configuration; an ST3215 exposes no unit serial, so
  // nothing the servo can say confirms them.
  struct Expected {
    uint8_t bus_id;
    const char* joint;
    const char* unit;
  };
  const Expected expected[] = {
      {11, "LF_LOWER", "M33"},  {12, "LF_UPPER", "ELR01"}, {13, "LF_HIP", "M22"},
      {21, "RF_LOWER", "NEW03"}, {22, "RF_UPPER", "ELR03"}, {23, "RF_HIP", "NEW01"},
      {31, "RH_LOWER", "NEW05"}, {32, "RH_UPPER", "ELR02"}, {33, "RH_HIP", "NEW06"},
      {41, "LH_LOWER", "M41"},  {42, "LH_UPPER", "M42"},   {43, "LH_HIP", "M43"},
  };
  for (const Expected& want : expected) {
    const CanonicalServo* got = findCanonical(want.bus_id);
    CHECK(got != nullptr);
    if (got == nullptr) continue;
    CHECK(std::strcmp(got->joint, want.joint) == 0);
    CHECK(std::strcmp(got->physical_unit, want.unit) == 0);
  }

  // M11 is the LF V25 archive's own label. On the current robot it is the
  // neck, and the table says so.
  const CanonicalServo* neck = findCanonical(52);
  CHECK(neck != nullptr);
  if (neck != nullptr) {
    CHECK(std::strcmp(neck->physical_unit, "M11") == 0);
    CHECK(std::strcmp(neck->joint, "NECK_PITCH") == 0);
    CHECK(!isLegServo(neck->bus_id));
  }
}

static void test_tostring_is_total() {
  g_case = "toString totality";
  for (RegisterCheck c : {RegisterCheck::NOT_READ, RegisterCheck::NO_ANSWER,
                          RegisterCheck::MATCH, RegisterCheck::MISMATCH}) {
    CHECK(std::strcmp(toString(c), "UNKNOWN") != 0);
  }
  CHECK(std::strcmp(toString(static_cast<RegisterCheck>(9)), "UNKNOWN") == 0);
  for (ProfileVerdict v : {ProfileVerdict::NOT_RUN, ProfileVerdict::MATCH,
                           ProfileVerdict::MISMATCH, ProfileVerdict::INCOMPLETE}) {
    CHECK(std::strcmp(toString(v), "UNKNOWN") != 0);
  }
  CHECK(std::strcmp(toString(static_cast<ProfileVerdict>(9)), "UNKNOWN") == 0);
}

int main() {
  std::printf("MATDOG C018 persistent profile + leg selection offline tests\n");

  test_the_persistent_profile_is_the_canonical_twenty();
  test_runtime_ram_state_is_not_part_of_the_persistent_profile();
  test_the_servo_invariants_are_a_separate_layer();

  test_a_failed_read_is_never_a_match();
  test_an_unread_register_can_never_produce_a_match_verdict();
  test_position_offset_is_twos_complement();

  test_the_leg_selection_is_exactly_twelve();
  test_expected_units_match_the_allocation();
  test_tostring_is_total();

  std::printf("checks_run=%d failures=%d\n", g_checks, g_failures);
  if (g_failures != 0) {
    std::printf("SERVO_PROFILE_TESTS = FAIL\n");
    return 1;
  }
  std::printf("SERVO_PROFILE_TESTS = PASS\n");
  return 0;
}
