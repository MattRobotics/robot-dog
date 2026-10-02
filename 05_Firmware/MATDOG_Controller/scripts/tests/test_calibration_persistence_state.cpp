// Offline tests for the pure persistence classification
// (src/calibration/CalibrationPersistenceState.*): what the two record slots and
// the save marker, read together after a reboot, allow. Named cases from the
// P2.4 brief, plus an exhaustive sweep of small inputs checking the safety
// properties that must hold for EVERY combination.

#include <cstdio>
#include <cstring>
#include <initializer_list>

#include "../../src/calibration/CalibrationPersistenceState.h"

using namespace matdog::calibration;

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

namespace {

SlotReport slot(SlotState s, uint32_t gen = 0) {
  SlotReport r;
  r.state = s;
  r.generation_hint = gen;
  return r;
}
SlotReport valid(uint32_t gen) { return slot(SlotState::VALID, gen); }
SlotReport absent() { return slot(SlotState::ABSENT); }
SlotReport corrupt(uint32_t gen = 0) { return slot(SlotState::CORRUPT, gen); }
SlotReport foreign(uint32_t gen) { return slot(SlotState::INCOMPATIBLE, gen); }

MarkerReport marker(SaveMarkerState st, uint32_t c, uint32_t b) {
  MarkerReport r;
  r.state = MarkerObservation::VALID;
  r.marker.state = st;
  r.marker.completed_generation = c;
  r.marker.begun_generation = b;
  return r;
}
MarkerReport done(uint32_t c, uint32_t b) { return marker(SaveMarkerState::COMPLETED, c, b); }
MarkerReport pending(uint32_t c, uint32_t b) { return marker(SaveMarkerState::PENDING, c, b); }
MarkerReport mstate(MarkerObservation o) {
  MarkerReport r;
  r.state = o;
  return r;
}

PersistenceInputs in(SlotReport a, SlotReport b, MarkerReport m) {
  PersistenceInputs i;
  i.slot[0] = a;
  i.slot[1] = b;
  i.marker = m;
  return i;
}

PersistenceAssessment cls(SlotReport a, SlotReport b, MarkerReport m) {
  return classifyPersistence(in(a, b, m));
}

void expectNotServed(const PersistenceAssessment& a) {
  CHECK(!a.record_available);
  CHECK(!a.save_allowed);
  CHECK(a.reconciliation_required || a.cls == PersistenceClass::IO_ERROR ||
        a.cls == PersistenceClass::MARKER_INCOMPATIBLE);
}

void test_healthy_and_empty() {
  g_case = "healthy / empty";
  PersistenceAssessment a = cls(absent(), absent(), mstate(MarkerObservation::ABSENT));
  CHECK(a.cls == PersistenceClass::NEVER_INITIALIZED_OR_ERASED);
  CHECK(!a.record_available && a.save_allowed && !a.reconciliation_required);

  a = cls(valid(1), absent(), done(1, 1));
  CHECK(a.cls == PersistenceClass::CONSISTENT);
  CHECK(a.record_available && a.save_allowed && a.confirmed_record_intact);
  CHECK(a.confirmed_slot == CalibrationSlot::A);
  CHECK(a.confirmed_generation == 1);

  a = cls(valid(1), valid(2), done(2, 2));
  CHECK(a.cls == PersistenceClass::CONSISTENT);
  CHECK(a.confirmed_slot == CalibrationSlot::B);

  // Older neighbour damaged / foreign / absent is fine: the confirmed one is intact.
  CHECK(cls(corrupt(), valid(2), done(2, 2)).cls == PersistenceClass::CONSISTENT);
  CHECK(cls(foreign(1), valid(2), done(2, 2)).cls == PersistenceClass::CONSISTENT);
  CHECK(cls(absent(), valid(2), done(2, 2)).cls == PersistenceClass::CONSISTENT);

  // COMPLETED with nothing confirmed.
  a = cls(absent(), absent(), done(0, 1));
  CHECK(a.cls == PersistenceClass::NOTHING_CONFIRMED);
  CHECK(!a.record_available && a.save_allowed);
}

void test_confirmed_generation_lost() {
  g_case = "confirmed generation lost";
  // The brief's fundamental rule: marker attests G, only G-1 survives.
  PersistenceAssessment a = cls(valid(2), absent(), done(3, 3));
  CHECK(a.cls == PersistenceClass::CONFIRMED_GENERATION_LOST);
  CHECK(a.older_record_survives);
  CHECK(!a.confirmed_record_intact);
  expectNotServed(a);
  CHECK(a.allowed_actions & kReconcileAdoptBit);   // the operator may adopt G-1 explicitly
  CHECK(a.allowed_actions & kReconcileDeclareBit);

  a = cls(valid(2), corrupt(3), done(3, 3));  // G corrupt, G-1 valid
  CHECK(a.cls == PersistenceClass::CONFIRMED_GENERATION_LOST);
  CHECK(a.older_record_survives);

  a = cls(corrupt(3), corrupt(2), done(3, 3));  // both corrupt
  CHECK(a.cls == PersistenceClass::CONFIRMED_GENERATION_LOST);
  CHECK(!a.older_record_survives);
  CHECK(!(a.allowed_actions & kReconcileAdoptBit));
  CHECK(a.allowed_actions & kReconcileDeclareBit);
  expectNotServed(a);

  a = cls(absent(), absent(), done(3, 3));  // wiped slots, marker intact: not "never used"
  CHECK(a.cls == PersistenceClass::CONFIRMED_GENERATION_LOST);
  expectNotServed(a);
}

void test_pending() {
  g_case = "pending";
  PersistenceAssessment a = cls(valid(1), absent(), pending(1, 2));
  CHECK(a.cls == PersistenceClass::PENDING_RECORD_ABSENT);
  CHECK(a.confirmed_record_intact && a.pending_generation == 2);
  expectNotServed(a);

  a = cls(valid(1), corrupt(2), pending(1, 2));
  CHECK(a.cls == PersistenceClass::PENDING_RECORD_ABSENT);

  a = cls(valid(1), valid(2), pending(1, 2));  // written, never confirmed: NOT promoted
  CHECK(a.cls == PersistenceClass::PENDING_RECORD_PRESENT);
  expectNotServed(a);

  a = cls(absent(), absent(), pending(0, 1));  // first ever SAVE interrupted
  CHECK(a.cls == PersistenceClass::PENDING_RECORD_ABSENT);
  expectNotServed(a);
  a = cls(valid(1), absent(), pending(0, 1));
  CHECK(a.cls == PersistenceClass::PENDING_RECORD_PRESENT);
  expectNotServed(a);

  // PENDING plus the confirmed record lost.
  a = cls(absent(), valid(2), pending(1, 2));
  CHECK(a.cls == PersistenceClass::CONFIRMED_GENERATION_LOST);
  expectNotServed(a);
}

void test_record_ahead_and_conflict() {
  g_case = "ahead / conflict";
  PersistenceAssessment a = cls(valid(1), valid(2), done(1, 1));
  CHECK(a.cls == PersistenceClass::RECORD_AHEAD_OF_MARKER);
  expectNotServed(a);
  a = cls(valid(1), valid(3), pending(1, 2));  // 3 was never begun
  CHECK(a.cls == PersistenceClass::RECORD_AHEAD_OF_MARKER);
  a = cls(valid(2), absent(), done(0, 1));  // marker says nothing confirmed, yet 2 > begun 1
  CHECK(a.cls == PersistenceClass::RECORD_AHEAD_OF_MARKER);

  // A leftover at or below begun is a discarded attempt, not an anomaly.
  a = cls(valid(1), valid(2), done(1, 2));
  CHECK(a.cls == PersistenceClass::CONSISTENT);
  CHECK(a.confirmed_generation == 1);

  a = cls(valid(2), valid(2), done(2, 2));
  CHECK(a.cls == PersistenceClass::RECORD_GENERATION_CONFLICT);
  expectNotServed(a);
  CHECK(!(a.allowed_actions & kReconcileAdoptBit));  // ambiguous: only declare
  a = cls(valid(2), valid(2), pending(1, 2));
  CHECK(a.cls == PersistenceClass::RECORD_GENERATION_CONFLICT);
  // Equal generations nobody speaks about are leftovers.
  a = cls(valid(1), valid(1), done(3, 3));
  CHECK(a.cls == PersistenceClass::CONFIRMED_GENERATION_LOST);
}

void test_marker_problems() {
  g_case = "marker problems";
  PersistenceAssessment a = cls(valid(1), valid(2), mstate(MarkerObservation::ABSENT));
  CHECK(a.cls == PersistenceClass::MARKER_MISSING);
  expectNotServed(a);
  a = cls(corrupt(), absent(), mstate(MarkerObservation::ABSENT));
  CHECK(a.cls == PersistenceClass::MARKER_MISSING);  // debris without a marker is not "never used"
  a = cls(valid(1), valid(2), mstate(MarkerObservation::CORRUPT));
  CHECK(a.cls == PersistenceClass::MARKER_CORRUPT);
  expectNotServed(a);
  CHECK(cls(absent(), absent(), mstate(MarkerObservation::CORRUPT)).cls == PersistenceClass::MARKER_CORRUPT);
  a = cls(valid(1), absent(), mstate(MarkerObservation::INCOMPATIBLE));
  CHECK(a.cls == PersistenceClass::MARKER_INCOMPATIBLE);
  expectNotServed(a);
  CHECK(a.allowed_actions == 0);  // this build must not replace another schema's marker
}

void test_io_errors_and_incompatible_records() {
  g_case = "io / incompatible";
  PersistenceAssessment a = cls(valid(1), slot(SlotState::IO_ERROR), done(1, 1));
  CHECK(a.cls == PersistenceClass::IO_ERROR);
  expectNotServed(a);
  CHECK(a.allowed_actions == 0);
  a = cls(valid(1), absent(), mstate(MarkerObservation::IO_ERROR));
  CHECK(a.cls == PersistenceClass::IO_ERROR);
  a = cls(valid(1), absent(), mstate(MarkerObservation::UNREAD));
  CHECK(a.cls == PersistenceClass::IO_ERROR);
  a = cls(slot(SlotState::UNREAD), absent(), done(1, 1));
  CHECK(a.cls == PersistenceClass::IO_ERROR);
  CHECK(PersistenceAssessment().cls == PersistenceClass::IO_ERROR);  // default is fail-closed
  CHECK(!PersistenceAssessment().save_allowed && !PersistenceAssessment().record_available);

  a = cls(foreign(2), valid(1), done(2, 2));  // confirmed generation present but foreign
  CHECK(a.cls == PersistenceClass::RECORD_INCOMPATIBLE);
  expectNotServed(a);
  a = cls(valid(1), foreign(2), done(1, 1));  // foreign record above the marker
  CHECK(a.cls == PersistenceClass::RECORD_INCOMPATIBLE);
  a = cls(valid(2), foreign(1), done(2, 2));  // foreign older neighbour: harmless
  CHECK(a.cls == PersistenceClass::CONSISTENT);
}

void test_determinism() {
  g_case = "determinism";
  const PersistenceInputs i = in(valid(1), corrupt(2), pending(1, 2));
  const PersistenceAssessment a = classifyPersistence(i);
  const PersistenceAssessment b = classifyPersistence(i);
  CHECK(std::memcmp(&a, &b, sizeof(a)) == 0 || (a.cls == b.cls && a.allowed_actions == b.allowed_actions));
  CHECK(std::strcmp(toString(PersistenceClass::CONFIRMED_GENERATION_LOST), "CONFIRMED_GENERATION_LOST") == 0);
  CHECK(std::strcmp(toString(ReconciliationAction::ADOPT_VALID_RECORD), "ADOPT_VALID_RECORD") == 0);
  CHECK(std::strcmp(toString(ReconciliationStatus::NOT_REQUIRED), "NOT_REQUIRED") == 0);
}

void test_plan_reconciliation() {
  g_case = "plan reconciliation";
  SaveMarker m;
  // PENDING_RECORD_PRESENT: adopt the new one.
  PersistenceInputs i = in(valid(1), valid(2), pending(1, 2));
  CHECK(planReconciliation(i, ReconciliationAction::ADOPT_VALID_RECORD, 2, &m) == ReconciliationStatus::OK);
  CHECK(m.state == SaveMarkerState::COMPLETED && m.completed_generation == 2 && m.begun_generation == 2);
  // ... or the previous one; the discarded generation 2 stays below begun.
  CHECK(planReconciliation(i, ReconciliationAction::ADOPT_VALID_RECORD, 1, &m) == ReconciliationStatus::OK);
  CHECK(m.completed_generation == 1 && m.begun_generation == 2);
  // A generation with no valid record cannot be adopted.
  CHECK(planReconciliation(i, ReconciliationAction::ADOPT_VALID_RECORD, 3, &m) == ReconciliationStatus::GENERATION_NOT_VALID);
  CHECK(planReconciliation(i, ReconciliationAction::ADOPT_VALID_RECORD, 0, &m) == ReconciliationStatus::GENERATION_NOT_VALID);
  // Declare nothing confirmed.
  CHECK(planReconciliation(i, ReconciliationAction::DECLARE_NOTHING_CONFIRMED, 0, &m) == ReconciliationStatus::OK);
  CHECK(m.completed_generation == 0 && m.begun_generation == 2);
  // Not required / not allowed / bad arguments.
  const PersistenceInputs healthy = in(valid(1), absent(), done(1, 1));
  CHECK(planReconciliation(healthy, ReconciliationAction::DECLARE_NOTHING_CONFIRMED, 0, &m) == ReconciliationStatus::NOT_REQUIRED);
  CHECK(planReconciliation(i, ReconciliationAction::ADOPT_VALID_RECORD, 2, nullptr) == ReconciliationStatus::BAD_ARGUMENT);
  CHECK(planReconciliation(i, static_cast<ReconciliationAction>(9), 0, &m) == ReconciliationStatus::ACTION_NOT_ALLOWED);
  const PersistenceInputs io = in(valid(1), slot(SlotState::IO_ERROR), done(1, 1));
  CHECK(planReconciliation(io, ReconciliationAction::DECLARE_NOTHING_CONFIRMED, 0, &m) == ReconciliationStatus::NOT_REQUIRED);
  const PersistenceInputs foreign_marker = in(valid(1), absent(), mstate(MarkerObservation::INCOMPATIBLE));
  CHECK(planReconciliation(foreign_marker, ReconciliationAction::DECLARE_NOTHING_CONFIRMED, 0, &m) == ReconciliationStatus::NOT_REQUIRED);
  // Ambiguous generations: adopt refused.
  const PersistenceInputs dup = in(valid(2), valid(2), done(2, 2));
  CHECK(planReconciliation(dup, ReconciliationAction::ADOPT_VALID_RECORD, 2, &m) == ReconciliationStatus::ACTION_NOT_ALLOWED);
  CHECK(planReconciliation(dup, ReconciliationAction::DECLARE_NOTHING_CONFIRMED, 0, &m) == ReconciliationStatus::OK);
  // Declare after a lost confirmation raises begun above everything seen.
  const PersistenceInputs lost = in(valid(2), absent(), done(3, 3));
  CHECK(planReconciliation(lost, ReconciliationAction::DECLARE_NOTHING_CONFIRMED, 0, &m) == ReconciliationStatus::OK);
  CHECK(m.completed_generation == 0 && m.begun_generation == 3);
  // Declare on a marker-less partition holding debris with no generation.
  const PersistenceInputs debris = in(corrupt(), absent(), mstate(MarkerObservation::ABSENT));
  CHECK(planReconciliation(debris, ReconciliationAction::DECLARE_NOTHING_CONFIRMED, 0, &m) == ReconciliationStatus::OK);
  CHECK(m.completed_generation == 0 && m.begun_generation == 1);
}

// ---- exhaustive sweep ------------------------------------------------------

constexpr uint32_t kMaxGen = 4;

int gSlotOptions(SlotReport* out) {
  int n = 0;
  out[n++] = slot(SlotState::ABSENT);
  out[n++] = slot(SlotState::CORRUPT, 0);
  out[n++] = slot(SlotState::IO_ERROR);
  out[n++] = slot(SlotState::UNREAD);
  for (uint32_t g = 1; g <= kMaxGen; ++g) {
    out[n++] = slot(SlotState::CORRUPT, g);
    out[n++] = slot(SlotState::INCOMPATIBLE, g);
    out[n++] = slot(SlotState::VALID, g);
  }
  return n;
}

int gMarkerOptions(MarkerReport* out) {
  int n = 0;
  out[n++] = mstate(MarkerObservation::UNREAD);
  out[n++] = mstate(MarkerObservation::ABSENT);
  out[n++] = mstate(MarkerObservation::IO_ERROR);
  out[n++] = mstate(MarkerObservation::CORRUPT);
  out[n++] = mstate(MarkerObservation::INCOMPATIBLE);
  for (uint32_t c = 0; c <= kMaxGen; ++c) {
    for (uint32_t b = 0; b <= kMaxGen; ++b) {
      for (SaveMarkerState st : {SaveMarkerState::COMPLETED, SaveMarkerState::PENDING}) {
        SaveMarker mm;
        mm.state = st;
        mm.completed_generation = c;
        mm.begun_generation = b;
        if (validateSaveMarker(mm) != SaveMarkerStatus::OK) continue;
        out[n++] = marker(st, c, b);
      }
    }
  }
  return n;
}

bool anySlot(const PersistenceInputs& i, SlotState s) { return i.slot[0].state == s || i.slot[1].state == s; }

PersistenceInputs withMarker(const PersistenceInputs& i, const SaveMarker& m) {
  PersistenceInputs r = i;
  r.marker = marker(m.state, m.completed_generation, m.begun_generation);
  return r;
}

void test_exhaustive_properties() {
  g_case = "exhaustive";
  SlotReport slots[40];
  MarkerReport markers[80];
  const int ns = gSlotOptions(slots);
  const int nm = gMarkerOptions(markers);
  long total = 0, healthy = 0, reconcilable = 0;
  int shown = 0;

  for (int a = 0; a < ns; ++a) {
    for (int b = 0; b < ns; ++b) {
      for (int m = 0; m < nm; ++m) {
        const PersistenceInputs i = in(slots[a], slots[b], markers[m]);
        const PersistenceAssessment r = classifyPersistence(i);
        ++total;
        const int before = g_failures;

        const bool unreadable = anySlot(i, SlotState::UNREAD) || anySlot(i, SlotState::IO_ERROR) ||
                                i.marker.state == MarkerObservation::UNREAD ||
                                i.marker.state == MarkerObservation::IO_ERROR;
        const bool marker_valid = i.marker.state == MarkerObservation::VALID;
        const uint32_t C = marker_valid ? i.marker.marker.completed_generation : 0;
        const uint32_t B = marker_valid ? i.marker.marker.begun_generation : 0;
        const bool is_pending = marker_valid && i.marker.marker.state == SaveMarkerState::PENDING;

        // P5: unreadable storage says nothing and allows nothing.
        if (unreadable) {
          CHECK(r.cls == PersistenceClass::IO_ERROR);
          CHECK(!r.save_allowed && !r.record_available && r.allowed_actions == 0);
        }
        // P1: a record is served only when the marker confirms exactly it.
        if (r.record_available) {
          CHECK(r.cls == PersistenceClass::CONSISTENT);
          CHECK(marker_valid && !is_pending && C > 0);
          CHECK(r.confirmed_record_intact && r.confirmed_generation == C);
          CHECK(i.slot[static_cast<int>(r.confirmed_slot)].state == SlotState::VALID);
          CHECK(i.slot[static_cast<int>(r.confirmed_slot)].generation_hint == C);
          ++healthy;
        }
        CHECK(r.record_available == (r.cls == PersistenceClass::CONSISTENT));
        // P2/P3: a new SAVE only from an empty, "nothing confirmed" or healthy state.
        CHECK(r.save_allowed == (r.cls == PersistenceClass::NEVER_INITIALIZED_OR_ERASED ||
                                 r.cls == PersistenceClass::NOTHING_CONFIRMED ||
                                 r.cls == PersistenceClass::CONSISTENT));
        if (is_pending) CHECK(!r.save_allowed && !r.record_available);
        if (r.save_allowed) {
          CHECK((marker_valid && !is_pending) ||
                (i.marker.state == MarkerObservation::ABSENT && i.slot[0].state == SlotState::ABSENT &&
                 i.slot[1].state == SlotState::ABSENT));
          // never overwrite on top of damage the marker speaks about
          if (marker_valid) {
            for (int s = 0; s < 2; ++s) {
              if (i.slot[s].state == SlotState::VALID) CHECK(i.slot[s].generation_hint <= B);
            }
          }
        }
        // P4: the marker attests C and no valid record has it => not healthy, not saveable.
        if (marker_valid && C > 0 && !unreadable) {
          const bool has = (i.slot[0].state == SlotState::VALID && i.slot[0].generation_hint == C) ||
                           (i.slot[1].state == SlotState::VALID && i.slot[1].generation_hint == C);
          if (!has) CHECK(!r.save_allowed && !r.record_available);
        }
        // P6: NEVER_INITIALIZED only for a truly empty partition.
        if (r.cls == PersistenceClass::NEVER_INITIALIZED_OR_ERASED) {
          CHECK(i.marker.state == MarkerObservation::ABSENT && i.slot[0].state == SlotState::ABSENT &&
                i.slot[1].state == SlotState::ABSENT);
        }
        // P7: reconciliation is offered exactly where the state needs it.
        CHECK((r.allowed_actions != 0) == r.reconciliation_required);
        CHECK(!(r.reconciliation_required && (r.save_allowed || r.record_available)));

        // P8: every offered action leads to a state that never serves anything unconfirmed.
        if (r.reconciliation_required) {
          ++reconcilable;
          uint32_t begun_seen = B;
          for (int s = 0; s < 2; ++s) {
            if (i.slot[s].generation_hint > begun_seen) begun_seen = i.slot[s].generation_hint;
          }
          SaveMarker pm;
          if (r.allowed_actions & kReconcileDeclareBit) {
            CHECK(planReconciliation(i, ReconciliationAction::DECLARE_NOTHING_CONFIRMED, 0, &pm) == ReconciliationStatus::OK);
            CHECK(validateSaveMarker(pm) == SaveMarkerStatus::OK);
            CHECK(pm.state == SaveMarkerState::COMPLETED && pm.completed_generation == 0);
            CHECK(pm.begun_generation >= begun_seen);
            const PersistenceAssessment after = classifyPersistence(withMarker(i, pm));
            CHECK(after.cls == PersistenceClass::NOTHING_CONFIRMED);
            CHECK(after.save_allowed && !after.record_available);
          }
          for (uint32_t g = 1; g <= kMaxGen; ++g) {
            const ReconciliationStatus st = planReconciliation(i, ReconciliationAction::ADOPT_VALID_RECORD, g, &pm);
            bool valid_g = false;
            for (int s = 0; s < 2; ++s) {
              valid_g |= i.slot[s].state == SlotState::VALID && i.slot[s].generation_hint == g;
            }
            const bool both_g = i.slot[0].state == SlotState::VALID && i.slot[1].state == SlotState::VALID &&
                                i.slot[0].generation_hint == g && i.slot[1].generation_hint == g;
            if (!valid_g || !(r.allowed_actions & kReconcileAdoptBit)) {
              CHECK(st != ReconciliationStatus::OK);
              continue;
            }
            if (both_g) {
              CHECK(st != ReconciliationStatus::OK);  // two valid records, one generation: ambiguous
              continue;
            }
            CHECK(st == ReconciliationStatus::OK);
            if (st != ReconciliationStatus::OK) continue;
            CHECK(validateSaveMarker(pm) == SaveMarkerStatus::OK);
            CHECK(pm.completed_generation == g && pm.begun_generation >= begun_seen);
            const PersistenceAssessment after = classifyPersistence(withMarker(i, pm));
            CHECK(after.cls == PersistenceClass::CONSISTENT);
            CHECK(after.record_available && after.confirmed_generation == g);
          }
        } else {
          SaveMarker pm;
          CHECK(planReconciliation(i, ReconciliationAction::DECLARE_NOTHING_CONFIRMED, 0, &pm) ==
                ReconciliationStatus::NOT_REQUIRED);
        }

        if (g_failures != before && shown++ < 5) {
          std::printf("    inputs: A=%d/%u B=%d/%u marker=%d c=%u b=%u st=%d -> %s\n",
                      (int)i.slot[0].state, i.slot[0].generation_hint, (int)i.slot[1].state,
                      i.slot[1].generation_hint, (int)i.marker.state, C, B,
                      marker_valid ? (int)i.marker.marker.state : -1, toString(r.cls));
        }
      }
    }
  }
  CHECK(total > 5000);
  CHECK(healthy > 0);
  CHECK(reconcilable > 0);
  std::printf("  exhaustive sweep: %ld inputs, %ld serve a record, %ld need reconciliation\n", total,
              healthy, reconcilable);
}

}  // namespace

int main() {
  test_healthy_and_empty();
  test_confirmed_generation_lost();
  test_pending();
  test_record_ahead_and_conflict();
  test_marker_problems();
  test_io_errors_and_incompatible_records();
  test_determinism();
  test_plan_reconciliation();
  test_exhaustive_properties();
  std::printf("calibration persistence state: %d checks, %d failures\n", g_checks, g_failures);
  return g_failures == 0 ? 0 : 1;
}
