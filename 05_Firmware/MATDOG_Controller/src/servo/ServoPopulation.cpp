#include "ServoPopulation.h"

namespace matdog {
namespace servo {

namespace {

bool inRange(int value, int lo, int hi) { return value >= lo && value <= hi; }

// Appends to one of the fixed-size evidence lists, saturating rather than
// overflowing. The COUNTERS are always exact; only the ID list can
// saturate, and when it does the caller marks the result truncated so the
// verdict can never be PASS on incomplete evidence.
bool appendId(uint8_t* ids, uint8_t* count, uint8_t id) {
  if (*count >= kMaxReportedIds) return false;
  ids[*count] = id;
  (*count)++;
  return true;
}

}  // namespace

const char* toString(IdClassification c) {
  switch (c) {
    case IdClassification::PRESENT_EXPECTED:         return "PRESENT_EXPECTED";
    case IdClassification::ABSENT_BY_DESIGN:         return "ABSENT_BY_DESIGN";
    case IdClassification::MISSING_EXPECTED:         return "MISSING_EXPECTED";
    case IdClassification::ABSENT_BY_DESIGN_PRESENT: return "ABSENT_BY_DESIGN_PRESENT";
    case IdClassification::UNEXPECTED_ID:            return "UNEXPECTED_ID";
    case IdClassification::NOT_PROBED:               return "NOT_PROBED";
  }
  return "UNKNOWN";
}

const char* toString(CensusVerdict v) {
  switch (v) {
    case CensusVerdict::PASS:             return "PASS";
    case CensusVerdict::PROFILE_MISMATCH: return "PROFILE_MISMATCH";
    case CensusVerdict::RANGE_INCOMPLETE: return "RANGE_INCOMPLETE";
    case CensusVerdict::NOT_RUN:          return "NOT_RUN";
  }
  return "UNKNOWN";
}

uint8_t canonicalAllocatedCount() { return kCanonicalServoCount; }

uint8_t expectedNowCount() {
  uint8_t n = 0;
  for (uint8_t i = 0; i < kCanonicalServoCount; ++i) {
    if (kCanonicalServos[i].current_config == CurrentConfig::INSTALLED) ++n;
  }
  return n;
}

uint8_t absentByDesignCount() {
  return static_cast<uint8_t>(kCanonicalServoCount - expectedNowCount());
}

const CanonicalServo* findCanonical(uint8_t bus_id) {
  for (uint8_t i = 0; i < kCanonicalServoCount; ++i) {
    if (kCanonicalServos[i].bus_id == bus_id) return &kCanonicalServos[i];
  }
  return nullptr;
}

IdClassification classifyId(uint8_t bus_id, bool responded, int scan_lo, int scan_hi) {
  const CanonicalServo* entry = findCanonical(bus_id);

  if (entry == nullptr) {
    // Not allocated by the MATDOG design at all. Only meaningful if it
    // actually answered — an unallocated ID staying silent is the normal
    // case for 200+ addresses and is not evidence of anything.
    return responded ? IdClassification::UNEXPECTED_ID : IdClassification::NOT_PROBED;
  }

  // A canonical ID that was never probed cannot be called absent. Note the
  // asymmetry: a RESPONSE is proof of presence regardless of the declared
  // range (it can only have come from a real probe), but SILENCE only
  // means something if we actually asked.
  if (!responded && !inRange(bus_id, scan_lo, scan_hi)) {
    return IdClassification::NOT_PROBED;
  }

  if (entry->current_config == CurrentConfig::INSTALLED) {
    return responded ? IdClassification::PRESENT_EXPECTED
                     : IdClassification::MISSING_EXPECTED;
  }

  // ABSENT_BY_DESIGN in the current configuration. Silence is the healthy
  // outcome; a response means the robot's real hardware no longer matches
  // the declared configuration and must be surfaced, never ignored.
  return responded ? IdClassification::ABSENT_BY_DESIGN_PRESENT
                   : IdClassification::ABSENT_BY_DESIGN;
}

CensusResult classifyObserved(const int* observed_ids,
                              int observed_len,
                              int observed_count,
                              int scan_lo,
                              int scan_hi) {
  CensusResult r;
  r.canonical_allocated = canonicalAllocatedCount();
  r.expected_now = expectedNowCount();
  r.scan_lo = scan_lo;
  r.scan_hi = scan_hi;

  if (scan_lo > scan_hi) {
    const int tmp = scan_lo;
    scan_lo = scan_hi;
    scan_hi = tmp;
    r.scan_lo = scan_lo;
    r.scan_hi = scan_hi;
  }

  // The scan buffer held fewer entries than the scan actually found: the
  // responder list is incomplete evidence.
  if (observed_ids == nullptr) {
    observed_len = 0;
    observed_count = 0;
  }
  int usable = observed_count;
  if (usable > observed_len) {
    usable = observed_len;
    r.truncated = true;
  }
  if (usable < 0) usable = 0;

  // --- Pass 1: every canonical ID, against the observed set --------------
  for (uint8_t i = 0; i < kCanonicalServoCount; ++i) {
    const uint8_t id = kCanonicalServos[i].bus_id;

    bool responded = false;
    for (int j = 0; j < usable; ++j) {
      if (observed_ids[j] == static_cast<int>(id)) {
        responded = true;
        break;
      }
    }

    switch (classifyId(id, responded, scan_lo, scan_hi)) {
      case IdClassification::PRESENT_EXPECTED:
        r.present_expected++;
        break;
      case IdClassification::ABSENT_BY_DESIGN:
        r.absent_by_design++;
        break;
      case IdClassification::MISSING_EXPECTED:
        r.missing_expected++;
        if (!appendId(r.missing_ids, &r.missing_id_count, id)) r.truncated = true;
        break;
      case IdClassification::ABSENT_BY_DESIGN_PRESENT:
        r.absent_by_design_present++;
        if (!appendId(r.absent_by_design_present_ids,
                      &r.absent_by_design_present_id_count, id)) {
          r.truncated = true;
        }
        break;
      case IdClassification::NOT_PROBED:
        r.not_probed++;
        break;
      case IdClassification::UNEXPECTED_ID:
        // Unreachable: this loop only walks canonical IDs.
        break;
    }
  }

  // --- Pass 2: responders that are not in the canonical table -------------
  for (int j = 0; j < usable; ++j) {
    const int id = observed_ids[j];
    if (id < 0 || id > 255) continue;
    if (findCanonical(static_cast<uint8_t>(id)) != nullptr) continue;
    r.unexpected_id++;
    if (!appendId(r.unexpected_ids, &r.unexpected_id_count, static_cast<uint8_t>(id))) {
      r.truncated = true;
    }
  }

  // --- Verdict ------------------------------------------------------------
  // Precedence is deliberate: an anomaly that WAS observed outranks an
  // incomplete range, because it is a positive finding and remains true
  // whatever the unscanned part of the bus contains. Only when nothing
  // anomalous was seen does incompleteness decide, and then it is
  // fail-closed — never PASS.
  const bool anomaly =
      r.missing_expected > 0 || r.absent_by_design_present > 0 || r.unexpected_id > 0;

  const bool range_covers_canonical =
      scan_lo <= kCanonicalScanLo && scan_hi >= kCanonicalScanHi;

  if (anomaly) {
    r.verdict = CensusVerdict::PROFILE_MISMATCH;
  } else if (!range_covers_canonical || r.not_probed > 0 || r.truncated) {
    r.verdict = CensusVerdict::RANGE_INCOMPLETE;
  } else {
    r.verdict = CensusVerdict::PASS;
  }

  return r;
}

}  // namespace servo
}  // namespace matdog
