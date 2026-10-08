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
  r.marker.acknowledged_generation = c;
  r.marker.begun_generation = b;
  return r;
}
MarkerReport done(uint32_t c, uint32_t b) { return marker(SaveMarkerState::IDLE, c, b); }
MarkerReport pending(uint32_t c, uint32_t b) { return marker(SaveMarkerState::PENDING, c, b); }
MarkerReport awaiting(uint32_t c, uint32_t b) { return marker(SaveMarkerState::AWAITING_ACK, c, b); }
MarkerReport mstate(MarkerObservation o) {
  MarkerReport r;
  r.state = o;
  return r;
}

PersistenceInputs withMarkerOf(PersistenceInputs i, const SaveMarker& m) {
  i.marker = marker(m.state, m.acknowledged_generation, m.begun_generation);
  return i;
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
  CHECK(a.record_available && a.save_allowed && a.acknowledged_record_intact);
  CHECK(a.acknowledged_slot == CalibrationSlot::A);
  CHECK(a.acknowledged_generation == 1);

  a = cls(valid(1), valid(2), done(2, 2));
  CHECK(a.cls == PersistenceClass::CONSISTENT);
  CHECK(a.acknowledged_slot == CalibrationSlot::B);

  // Older neighbour damaged / foreign / absent is fine: the acknowledged one is intact.
  CHECK(cls(corrupt(), valid(2), done(2, 2)).cls == PersistenceClass::CONSISTENT);
  CHECK(cls(foreign(1), valid(2), done(2, 2)).cls == PersistenceClass::CONSISTENT);
  CHECK(cls(absent(), valid(2), done(2, 2)).cls == PersistenceClass::CONSISTENT);

  // IDLE with nothing acknowledged.
  a = cls(absent(), absent(), done(0, 1));
  CHECK(a.cls == PersistenceClass::NOTHING_ACKNOWLEDGED);
  CHECK(!a.record_available && a.save_allowed);
}

void test_acknowledged_generation_lost() {
  g_case = "acknowledged generation lost";
  // The brief's fundamental rule: marker attests G, only G-1 survives.
  PersistenceAssessment a = cls(valid(2), absent(), done(3, 3));
  CHECK(a.cls == PersistenceClass::ACKNOWLEDGED_GENERATION_LOST);
  CHECK(a.older_record_survives);
  CHECK(!a.acknowledged_record_intact);
  expectNotServed(a);
  CHECK(a.allowed_actions & kReconcileAdoptBit);   // the operator may adopt G-1 explicitly
  CHECK(a.allowed_actions & kReconcileDeclareBit);

  a = cls(valid(2), corrupt(3), done(3, 3));  // G corrupt, G-1 valid
  CHECK(a.cls == PersistenceClass::ACKNOWLEDGED_GENERATION_LOST);
  CHECK(a.older_record_survives);

  a = cls(corrupt(3), corrupt(2), done(3, 3));  // both corrupt
  CHECK(a.cls == PersistenceClass::ACKNOWLEDGED_GENERATION_LOST);
  CHECK(!a.older_record_survives);
  CHECK(!(a.allowed_actions & kReconcileAdoptBit));
  CHECK(a.allowed_actions & kReconcileDeclareBit);
  expectNotServed(a);

  a = cls(absent(), absent(), done(3, 3));  // wiped slots, marker intact: not "never used"
  CHECK(a.cls == PersistenceClass::ACKNOWLEDGED_GENERATION_LOST);
  expectNotServed(a);
}

void test_pending() {
  g_case = "pending";
  PersistenceAssessment a = cls(valid(1), absent(), pending(1, 2));
  CHECK(a.cls == PersistenceClass::PENDING_RECORD_ABSENT);
  CHECK(a.acknowledged_record_intact && a.pending_generation == 2);
  expectNotServed(a);

  a = cls(valid(1), corrupt(2), pending(1, 2));
  CHECK(a.cls == PersistenceClass::PENDING_RECORD_ABSENT);

  a = cls(valid(1), valid(2), pending(1, 2));  // written, never acknowledged: NOT promoted
  CHECK(a.cls == PersistenceClass::PENDING_RECORD_PRESENT);
  expectNotServed(a);

  a = cls(absent(), absent(), pending(0, 1));  // first ever SAVE interrupted
  CHECK(a.cls == PersistenceClass::PENDING_RECORD_ABSENT);
  expectNotServed(a);
  a = cls(valid(1), absent(), pending(0, 1));
  CHECK(a.cls == PersistenceClass::PENDING_RECORD_PRESENT);
  expectNotServed(a);

  // PENDING plus the acknowledged record lost.
  a = cls(absent(), valid(2), pending(1, 2));
  CHECK(a.cls == PersistenceClass::ACKNOWLEDGED_GENERATION_LOST);
  expectNotServed(a);
}

void test_awaiting_ack() {
  g_case = "awaiting ack";
  // A/1 acknowledged, B/2 written and verified, marker AWAITING_ACK: not promoted,
  // not served, previous generation protected, ACK allowed.
  PersistenceAssessment a = cls(valid(1), valid(2), awaiting(1, 2));
  CHECK(a.cls == PersistenceClass::AWAITING_ACK);
  CHECK(a.ack_allowed && !a.save_allowed && !a.record_available && a.reconciliation_required);
  CHECK(a.acknowledged_generation == 1 && a.awaiting_generation == 2 && a.pending_generation == 0);
  CHECK(a.acknowledged_record_intact && a.acknowledged_slot == CalibrationSlot::A);
  CHECK((a.allowed_actions & kReconcileAdoptBit) && (a.allowed_actions & kReconcileDeclareBit));

  // First SAVE with no previous generation.
  a = cls(valid(1), absent(), awaiting(0, 1));
  CHECK(a.cls == PersistenceClass::AWAITING_ACK && a.ack_allowed && !a.acknowledged_record_intact);
  a = cls(absent(), valid(1), awaiting(0, 1));
  CHECK(a.cls == PersistenceClass::AWAITING_ACK);
  expectNotServed(a);

  // The verified record vanished: the marker's claim cannot be honoured.
  a = cls(valid(1), absent(), awaiting(1, 2));
  CHECK(a.cls == PersistenceClass::AWAITING_ACK_RECORD_LOST && !a.ack_allowed);
  expectNotServed(a);
  a = cls(valid(1), corrupt(2), awaiting(1, 2));
  CHECK(a.cls == PersistenceClass::AWAITING_ACK_RECORD_LOST);
  a = cls(absent(), absent(), awaiting(0, 1));
  CHECK(a.cls == PersistenceClass::AWAITING_ACK_RECORD_LOST);
  a = cls(valid(1), foreign(2), awaiting(1, 2));  // intact but foreign: not acknowledgeable
  CHECK(a.cls == PersistenceClass::RECORD_INCOMPATIBLE && !a.ack_allowed);

  // A discarded older attempt may survive in the other slot (nothing acknowledged
  // yet, so there is no acknowledged slot to take its place): it must not block the
  // ACK of the in-flight generation, and a record beyond begun_generation still does.
  a = cls(valid(2), valid(4), awaiting(0, 4));
  CHECK(a.cls == PersistenceClass::AWAITING_ACK && a.ack_allowed && a.awaiting_generation == 4);
  a = cls(valid(2), valid(5), awaiting(0, 4));
  CHECK(a.cls == PersistenceClass::RECORD_AHEAD_OF_MARKER && !a.ack_allowed);

  // A lost acknowledged generation takes priority over the awaiting one.
  a = cls(absent(), valid(2), awaiting(1, 2));
  CHECK(a.cls == PersistenceClass::ACKNOWLEDGED_GENERATION_LOST && !a.ack_allowed);
  expectNotServed(a);
  a = cls(corrupt(1), valid(2), awaiting(1, 2));
  CHECK(a.cls == PersistenceClass::ACKNOWLEDGED_GENERATION_LOST);

  // Anything above the awaited generation, or two copies of it, is not acknowledgeable.
  a = cls(valid(1), valid(3), awaiting(1, 2));
  CHECK(a.cls == PersistenceClass::RECORD_AHEAD_OF_MARKER && !a.ack_allowed);
  a = cls(valid(2), valid(2), awaiting(1, 2));
  CHECK(a.cls == PersistenceClass::RECORD_GENERATION_CONFLICT && !a.ack_allowed);

  // planAcknowledgment: only the awaited generation, never a neighbour.
  SaveMarker m;
  const PersistenceInputs w = in(valid(1), valid(2), awaiting(1, 2));
  CHECK(planAcknowledgment(w, 2, &m) == AckPlanStatus::OK);
  CHECK(m.state == SaveMarkerState::IDLE && m.acknowledged_generation == 2 && m.begun_generation == 2);
  CHECK(planAcknowledgment(w, 1, &m) == AckPlanStatus::GENERATION_MISMATCH);
  CHECK(planAcknowledgment(w, 3, &m) == AckPlanStatus::GENERATION_MISMATCH);
  CHECK(planAcknowledgment(w, 0, &m) == AckPlanStatus::BAD_ARGUMENT);
  CHECK(planAcknowledgment(w, 2, nullptr) == AckPlanStatus::BAD_ARGUMENT);
  // After the ACK: idempotent; an older or unknown generation is refused.
  const PersistenceInputs acked = in(valid(1), valid(2), done(2, 2));
  CHECK(planAcknowledgment(acked, 2, &m) == AckPlanStatus::ALREADY_ACKNOWLEDGED);
  CHECK(planAcknowledgment(acked, 1, &m) == AckPlanStatus::NOT_AWAITING);
  CHECK(planAcknowledgment(acked, 3, &m) == AckPlanStatus::NOT_AWAITING);
  CHECK(planAcknowledgment(in(absent(), absent(), mstate(MarkerObservation::ABSENT)), 1, &m) == AckPlanStatus::NOT_AWAITING);
  // PENDING: the record is incomplete, whatever is on disk.
  CHECK(planAcknowledgment(in(valid(1), valid(2), pending(1, 2)), 2, &m) == AckPlanStatus::RECORD_NOT_VALID);
  CHECK(planAcknowledgment(in(valid(1), absent(), pending(1, 2)), 2, &m) == AckPlanStatus::RECORD_NOT_VALID);
  CHECK(planAcknowledgment(in(valid(1), valid(2), pending(1, 2)), 1, &m) == AckPlanStatus::GENERATION_MISMATCH);
  // AWAITING_ACK whose record is lost / foreign.
  CHECK(planAcknowledgment(in(valid(1), absent(), awaiting(1, 2)), 2, &m) == AckPlanStatus::RECORD_NOT_VALID);
  CHECK(planAcknowledgment(in(valid(1), foreign(2), awaiting(1, 2)), 2, &m) == AckPlanStatus::RECORD_NOT_VALID);
  // Unresolved states.
  CHECK(planAcknowledgment(in(absent(), valid(2), awaiting(1, 2)), 2, &m) == AckPlanStatus::RECONCILIATION_REQUIRED);
  CHECK(planAcknowledgment(in(valid(1), valid(2), mstate(MarkerObservation::ABSENT)), 2, &m) == AckPlanStatus::RECONCILIATION_REQUIRED);
  CHECK(planAcknowledgment(in(valid(1), valid(2), mstate(MarkerObservation::INCOMPATIBLE)), 2, &m) == AckPlanStatus::RECONCILIATION_REQUIRED);
  CHECK(planAcknowledgment(in(valid(1), slot(SlotState::IO_ERROR), awaiting(1, 2)), 2, &m) == AckPlanStatus::STORAGE_UNUSABLE);

  // Reconciliation respects verified vs acknowledged.
  // ADOPT the awaited generation == an operator ACK; ADOPT the previous one discards it.
  CHECK(planReconciliation(w, ReconciliationAction::ADOPT_VALID_RECORD, 2, &m) == ReconciliationStatus::OK);
  CHECK(m.state == SaveMarkerState::IDLE && m.acknowledged_generation == 2 && m.begun_generation == 2);
  CHECK(planReconciliation(w, ReconciliationAction::ADOPT_VALID_RECORD, 1, &m) == ReconciliationStatus::OK);
  CHECK(m.acknowledged_generation == 1 && m.begun_generation == 2);
  CHECK(classifyPersistence(in(valid(1), valid(2), done(m.acknowledged_generation, m.begun_generation))).cls ==
        PersistenceClass::CONSISTENT);
  CHECK(planReconciliation(w, ReconciliationAction::DECLARE_NOTHING_ACKNOWLEDGED, 0, &m) == ReconciliationStatus::OK);
  CHECK(m.acknowledged_generation == 0 && m.begun_generation == 2);
  // Corrupt / foreign / absent records are never adoptable.
  const PersistenceInputs lost = in(valid(1), corrupt(2), awaiting(1, 2));
  CHECK(planReconciliation(lost, ReconciliationAction::ADOPT_VALID_RECORD, 2, &m) == ReconciliationStatus::GENERATION_NOT_VALID);
  CHECK(planReconciliation(lost, ReconciliationAction::ADOPT_VALID_RECORD, 1, &m) == ReconciliationStatus::OK);
  CHECK(m.acknowledged_generation == 1 && m.begun_generation == 2);
  const PersistenceInputs frn = in(valid(1), foreign(2), awaiting(1, 2));
  CHECK(planReconciliation(frn, ReconciliationAction::ADOPT_VALID_RECORD, 2, &m) == ReconciliationStatus::GENERATION_NOT_VALID);
  // First SAVE pending ACK: adopt it, or declare nothing.
  const PersistenceInputs first = in(valid(1), absent(), awaiting(0, 1));
  CHECK(planReconciliation(first, ReconciliationAction::ADOPT_VALID_RECORD, 1, &m) == ReconciliationStatus::OK);
  CHECK(m.acknowledged_generation == 1 && m.begun_generation == 1);
  CHECK(planReconciliation(first, ReconciliationAction::DECLARE_NOTHING_ACKNOWLEDGED, 0, &m) == ReconciliationStatus::OK);
  CHECK(m.acknowledged_generation == 0 && m.begun_generation == 1);
  CHECK(classifyPersistence(withMarkerOf(first, m)).cls == PersistenceClass::NOTHING_ACKNOWLEDGED);
  CHECK(std::strcmp(toString(PersistenceClass::AWAITING_ACK), "AWAITING_ACK") == 0);
  CHECK(std::strcmp(toString(AckPlanStatus::GENERATION_MISMATCH), "GENERATION_MISMATCH") == 0);
}

void test_record_ahead_and_conflict() {
  g_case = "ahead / conflict";
  PersistenceAssessment a = cls(valid(1), valid(2), done(1, 1));
  CHECK(a.cls == PersistenceClass::RECORD_AHEAD_OF_MARKER);
  expectNotServed(a);
  a = cls(valid(1), valid(3), pending(1, 2));  // 3 was never begun
  CHECK(a.cls == PersistenceClass::RECORD_AHEAD_OF_MARKER);
  a = cls(valid(2), absent(), done(0, 1));  // marker says nothing acknowledged, yet 2 > begun 1
  CHECK(a.cls == PersistenceClass::RECORD_AHEAD_OF_MARKER);

  // A leftover at or below begun is a discarded attempt, not an anomaly.
  a = cls(valid(1), valid(2), done(1, 2));
  CHECK(a.cls == PersistenceClass::CONSISTENT);
  CHECK(a.acknowledged_generation == 1);

  a = cls(valid(2), valid(2), done(2, 2));
  CHECK(a.cls == PersistenceClass::RECORD_GENERATION_CONFLICT);
  expectNotServed(a);
  CHECK(!(a.allowed_actions & kReconcileAdoptBit));  // ambiguous: only declare
  a = cls(valid(2), valid(2), pending(1, 2));
  CHECK(a.cls == PersistenceClass::RECORD_GENERATION_CONFLICT);
  // Equal generations nobody speaks about are leftovers.
  a = cls(valid(1), valid(1), done(3, 3));
  CHECK(a.cls == PersistenceClass::ACKNOWLEDGED_GENERATION_LOST);
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

  a = cls(foreign(2), valid(1), done(2, 2));  // acknowledged generation present but foreign
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
  CHECK(std::strcmp(toString(PersistenceClass::ACKNOWLEDGED_GENERATION_LOST), "ACKNOWLEDGED_GENERATION_LOST") == 0);
  CHECK(std::strcmp(toString(ReconciliationAction::ADOPT_VALID_RECORD), "ADOPT_VALID_RECORD") == 0);
  CHECK(std::strcmp(toString(ReconciliationStatus::NOT_REQUIRED), "NOT_REQUIRED") == 0);
}

void test_plan_reconciliation() {
  g_case = "plan reconciliation";
  SaveMarker m;
  // PENDING_RECORD_PRESENT: adopt the new one.
  PersistenceInputs i = in(valid(1), valid(2), pending(1, 2));
  CHECK(planReconciliation(i, ReconciliationAction::ADOPT_VALID_RECORD, 2, &m) == ReconciliationStatus::OK);
  CHECK(m.state == SaveMarkerState::IDLE && m.acknowledged_generation == 2 && m.begun_generation == 2);
  // ... or the previous one; the discarded generation 2 stays below begun.
  CHECK(planReconciliation(i, ReconciliationAction::ADOPT_VALID_RECORD, 1, &m) == ReconciliationStatus::OK);
  CHECK(m.acknowledged_generation == 1 && m.begun_generation == 2);
  // A generation with no valid record cannot be adopted.
  CHECK(planReconciliation(i, ReconciliationAction::ADOPT_VALID_RECORD, 3, &m) == ReconciliationStatus::GENERATION_NOT_VALID);
  CHECK(planReconciliation(i, ReconciliationAction::ADOPT_VALID_RECORD, 0, &m) == ReconciliationStatus::GENERATION_NOT_VALID);
  // Declare nothing acknowledged.
  CHECK(planReconciliation(i, ReconciliationAction::DECLARE_NOTHING_ACKNOWLEDGED, 0, &m) == ReconciliationStatus::OK);
  CHECK(m.acknowledged_generation == 0 && m.begun_generation == 2);
  // Not required / not allowed / bad arguments.
  const PersistenceInputs healthy = in(valid(1), absent(), done(1, 1));
  CHECK(planReconciliation(healthy, ReconciliationAction::DECLARE_NOTHING_ACKNOWLEDGED, 0, &m) == ReconciliationStatus::NOT_REQUIRED);
  CHECK(planReconciliation(i, ReconciliationAction::ADOPT_VALID_RECORD, 2, nullptr) == ReconciliationStatus::BAD_ARGUMENT);
  CHECK(planReconciliation(i, static_cast<ReconciliationAction>(9), 0, &m) == ReconciliationStatus::ACTION_NOT_ALLOWED);
  const PersistenceInputs io = in(valid(1), slot(SlotState::IO_ERROR), done(1, 1));
  CHECK(planReconciliation(io, ReconciliationAction::DECLARE_NOTHING_ACKNOWLEDGED, 0, &m) == ReconciliationStatus::NOT_REQUIRED);
  const PersistenceInputs foreign_marker = in(valid(1), absent(), mstate(MarkerObservation::INCOMPATIBLE));
  CHECK(planReconciliation(foreign_marker, ReconciliationAction::DECLARE_NOTHING_ACKNOWLEDGED, 0, &m) == ReconciliationStatus::NOT_REQUIRED);
  // Ambiguous generations: adopt refused.
  const PersistenceInputs dup = in(valid(2), valid(2), done(2, 2));
  CHECK(planReconciliation(dup, ReconciliationAction::ADOPT_VALID_RECORD, 2, &m) == ReconciliationStatus::ACTION_NOT_ALLOWED);
  CHECK(planReconciliation(dup, ReconciliationAction::DECLARE_NOTHING_ACKNOWLEDGED, 0, &m) == ReconciliationStatus::OK);
  // Declare after a lost acknowledgment raises begun above everything seen.
  const PersistenceInputs lost = in(valid(2), absent(), done(3, 3));
  CHECK(planReconciliation(lost, ReconciliationAction::DECLARE_NOTHING_ACKNOWLEDGED, 0, &m) == ReconciliationStatus::OK);
  CHECK(m.acknowledged_generation == 0 && m.begun_generation == 3);
  // Declare on a marker-less partition holding debris with no generation.
  const PersistenceInputs debris = in(corrupt(), absent(), mstate(MarkerObservation::ABSENT));
  CHECK(planReconciliation(debris, ReconciliationAction::DECLARE_NOTHING_ACKNOWLEDGED, 0, &m) == ReconciliationStatus::OK);
  CHECK(m.acknowledged_generation == 0 && m.begun_generation == 1);
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
      for (SaveMarkerState st : {SaveMarkerState::IDLE, SaveMarkerState::PENDING, SaveMarkerState::AWAITING_ACK}) {
        SaveMarker mm;
        mm.state = st;
        mm.acknowledged_generation = c;
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
  r.marker = marker(m.state, m.acknowledged_generation, m.begun_generation);
  return r;
}

void test_exhaustive_properties() {
  g_case = "exhaustive";
  SlotReport slots[40];
  MarkerReport markers[120];
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
        const uint32_t C = marker_valid ? i.marker.marker.acknowledged_generation : 0;
        const uint32_t B = marker_valid ? i.marker.marker.begun_generation : 0;
        const bool is_pending = marker_valid && i.marker.marker.state != SaveMarkerState::IDLE;  // PENDING or AWAITING_ACK
        const bool is_awaiting = marker_valid && i.marker.marker.state == SaveMarkerState::AWAITING_ACK;

        // P5: unreadable storage says nothing and allows nothing.
        if (unreadable) {
          CHECK(r.cls == PersistenceClass::IO_ERROR);
          CHECK(!r.save_allowed && !r.record_available && r.allowed_actions == 0);
        }
        // P1: a record is served only when the marker confirms exactly it.
        if (r.record_available) {
          CHECK(r.cls == PersistenceClass::CONSISTENT);
          CHECK(marker_valid && !is_pending && C > 0);
          CHECK(r.acknowledged_record_intact && r.acknowledged_generation == C);
          CHECK(i.slot[static_cast<int>(r.acknowledged_slot)].state == SlotState::VALID);
          CHECK(i.slot[static_cast<int>(r.acknowledged_slot)].generation_hint == C);
          ++healthy;
        }
        CHECK(r.record_available == (r.cls == PersistenceClass::CONSISTENT));
        // P9 (P2.4.1): an ACK is possible in exactly one class, and only for the awaited generation.
        CHECK(r.ack_allowed == (r.cls == PersistenceClass::AWAITING_ACK));
        if (r.ack_allowed) CHECK(is_awaiting && !r.save_allowed && !r.record_available && r.awaiting_generation == B);
        if (is_awaiting) CHECK(!r.save_allowed && !r.record_available);
        {
          for (uint32_t g = 0; g <= kMaxGen + 1; ++g) {
            SaveMarker am;
            const AckPlanStatus ast = planAcknowledgment(i, g, &am);
            CHECK((ast == AckPlanStatus::OK) == (r.ack_allowed && g == B && g != 0));
            if (ast == AckPlanStatus::OK) {
              CHECK(am.state == SaveMarkerState::IDLE && am.acknowledged_generation == g && am.begun_generation == g);
              const PersistenceAssessment after = classifyPersistence(withMarker(i, am));
              CHECK(after.cls == PersistenceClass::CONSISTENT && after.acknowledged_generation == g);
            }
            // Never an acknowledgment of a generation without a valid record.
            if (ast == AckPlanStatus::OK || ast == AckPlanStatus::ALREADY_ACKNOWLEDGED) {
              bool has = false;
              for (int s = 0; s < 2; ++s) has |= i.slot[s].state == SlotState::VALID && i.slot[s].generation_hint == g;
              CHECK(has);
            }
          }
        }
        // P2/P3: a new SAVE only from an empty, "nothing acknowledged" or healthy state.
        CHECK(r.save_allowed == (r.cls == PersistenceClass::NEVER_INITIALIZED_OR_ERASED ||
                                 r.cls == PersistenceClass::NOTHING_ACKNOWLEDGED ||
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

        // P8: every offered action leads to a state that never serves anything unacknowledged.
        if (r.reconciliation_required) {
          ++reconcilable;
          uint32_t begun_seen = B;
          for (int s = 0; s < 2; ++s) {
            if (i.slot[s].generation_hint > begun_seen) begun_seen = i.slot[s].generation_hint;
          }
          SaveMarker pm;
          if (r.allowed_actions & kReconcileDeclareBit) {
            CHECK(planReconciliation(i, ReconciliationAction::DECLARE_NOTHING_ACKNOWLEDGED, 0, &pm) == ReconciliationStatus::OK);
            CHECK(validateSaveMarker(pm) == SaveMarkerStatus::OK);
            CHECK(pm.state == SaveMarkerState::IDLE && pm.acknowledged_generation == 0);
            CHECK(pm.begun_generation >= begun_seen);
            const PersistenceAssessment after = classifyPersistence(withMarker(i, pm));
            CHECK(after.cls == PersistenceClass::NOTHING_ACKNOWLEDGED);
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
            CHECK(pm.acknowledged_generation == g && pm.begun_generation >= begun_seen);
            const PersistenceAssessment after = classifyPersistence(withMarker(i, pm));
            CHECK(after.cls == PersistenceClass::CONSISTENT);
            CHECK(after.record_available && after.acknowledged_generation == g);
          }
        } else {
          SaveMarker pm;
          CHECK(planReconciliation(i, ReconciliationAction::DECLARE_NOTHING_ACKNOWLEDGED, 0, &pm) ==
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
  test_acknowledged_generation_lost();
  test_pending();
  test_awaiting_ack();
  test_record_ahead_and_conflict();
  test_marker_problems();
  test_io_errors_and_incompatible_records();
  test_determinism();
  test_plan_reconciliation();
  test_exhaustive_properties();
  std::printf("calibration persistence state: %d checks, %d failures\n", g_checks, g_failures);
  return g_failures == 0 ? 0 : 1;
}
