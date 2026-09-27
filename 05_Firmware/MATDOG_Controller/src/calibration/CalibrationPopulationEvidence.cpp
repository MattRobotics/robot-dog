#include "CalibrationPopulationEvidence.h"

#include <string.h>

#include "../servo/ServoProfileData.h"

namespace matdog {
namespace calibration {

namespace {

constexpr int32_t kRawTickMin = 0;
constexpr int32_t kRawTickMax = 4095;

bool sameText(const char* a, const char* b) {
  if (a == nullptr || b == nullptr) return false;
  return strcmp(a, b) == 0;
}

bool semanticIdentityFromCanonicalImpl(const servo::CanonicalServo& canonical,
                                       JointIdentity* out) {
  if (out == nullptr || canonical.joint == nullptr ||
      canonical.physical_unit == nullptr ||
      canonical.current_config != servo::CurrentConfig::INSTALLED) {
    return false;
  }

  const char* joint = canonical.joint;
  Leg leg;
  if (strncmp(joint, "LF_", 3) == 0) {
    leg = Leg::LF;
  } else if (strncmp(joint, "RF_", 3) == 0) {
    leg = Leg::RF;
  } else if (strncmp(joint, "RH_", 3) == 0) {
    leg = Leg::RH;
  } else if (strncmp(joint, "LH_", 3) == 0) {
    leg = Leg::LH;
  } else {
    return false;
  }

  const char* kind = joint + 3;
  JointKind joint_kind;
  if (strcmp(kind, "HIP") == 0) {
    joint_kind = JointKind::HIP;
  } else if (strcmp(kind, "UPPER") == 0) {
    joint_kind = JointKind::UPPER;
  } else if (strcmp(kind, "LOWER") == 0) {
    joint_kind = JointKind::LOWER;
  } else {
    return false;
  }

  *out = JointIdentity{};
  out->leg = leg;
  out->joint = joint_kind;
  setPhysicalUnit(out, canonical.physical_unit);
  return out->valid() && out->unitKnown();
}

bool censusCompleteForFormalEvidence(const servo::CensusResult& census) {
  const uint8_t expected_now = servo::expectedNowCount();
  const uint8_t absent_by_design = servo::absentByDesignCount();
  return census.scan_lo <= servo::kCanonicalScanLo &&
         census.scan_hi >= servo::kCanonicalScanHi &&
         census.not_probed == 0 &&
         !census.truncated &&
         census.canonical_allocated == servo::canonicalAllocatedCount() &&
         census.expected_now == expected_now &&
         static_cast<uint16_t>(census.present_expected) + census.missing_expected ==
             expected_now &&
         static_cast<uint16_t>(census.absent_by_design) +
                 census.absent_by_design_present ==
             absent_by_design &&
         census.missing_id_count == census.missing_expected &&
         census.unexpected_id_count == census.unexpected_id &&
         census.absent_by_design_present_id_count ==
             census.absent_by_design_present;
}

uint16_t busAnomalyCount(const servo::CensusResult& census) {
  const uint32_t count = static_cast<uint32_t>(census.unexpected_id) +
                         static_cast<uint32_t>(census.absent_by_design_present);
  return count > 0xFFFFu ? 0xFFFFu : static_cast<uint16_t>(count);
}

bool censusMissingLeg(const servo::CensusResult& census) {
  for (uint8_t i = 0; i < census.missing_id_count; ++i) {
    if (servo::isLegServo(census.missing_ids[i])) return true;
  }
  return false;
}

bool recordMatchesCanonical(const servo::JointPreflightRecord& record,
                            const servo::CanonicalServo& canonical) {
  return record.expected_bus_id == canonical.bus_id &&
         sameText(record.joint, canonical.joint) &&
         sameText(record.expected_physical_unit, canonical.physical_unit);
}

bool recordQualifies(const servo::JointPreflightRecord& record,
                     const servo::CanonicalServo& canonical) {
  if (!recordMatchesCanonical(record, canonical)) return false;
  if (record.result != servo::JointPreflightResult::PASS) return false;
  if (record.observed_bus_id != canonical.bus_id) return false;
  if (record.model != servo::profile_data::kInvariants.model_expected) return false;
  if (!record.position_offset_read || record.position_offset != 0) return false;
  if (record.profile != servo::ProfileVerdict::MATCH) return false;
  if (record.profile_registers_read != servo::profile_data::kPersistentRegisterCount) return false;
  if (record.profile_mismatch_count != 0) return false;
  if (record.torque_enable != 0) return false;
  if (record.present_position < kRawTickMin || record.present_position > kRawTickMax) return false;
  return true;
}

}  // namespace

bool semanticIdentityFromCanonical(const servo::CanonicalServo& canonical,
                                   JointIdentity* out) {
  return semanticIdentityFromCanonicalImpl(canonical, out);
}

PopulationEvidenceBuildResult buildCurrentLegPopulationEvidence(
    const servo::CensusResult& census,
    const servo::PreflightResult& preflight,
    const PopulationEvidenceBuildContext& context) {
  PopulationEvidenceBuildResult out{};

  // A cached 12/12 preflight is not formal current calibration evidence.
  if (!context.current_observation_bundle) {
    out.status = PopulationEvidenceBuildStatus::REJECT_SOURCE_NOT_CURRENT;
    return out;
  }

  out.evidence.evaluated = true;
  out.evidence.origin = CalibrationOrigin::LIVE_SESSION;
  out.evidence.session_ms = context.session_ms;

  if (!censusCompleteForFormalEvidence(census)) {
    out.status = PopulationEvidenceBuildStatus::REJECT_CENSUS_INCOMPLETE;
    return out;
  }

  out.evidence.unexpected_count = busAnomalyCount(census);
  if (out.evidence.unexpected_count != 0) {
    out.status = PopulationEvidenceBuildStatus::REJECT_BUS_ANOMALY;
    return out;
  }

  // ID 51 is intentionally outside the twelve leg-calibration slots. A
  // current census may miss the neck without claiming a leg H1 failure.
  if (censusMissingLeg(census)) {
    out.status = PopulationEvidenceBuildStatus::REJECT_LEG_MISSING_IN_CENSUS;
    return out;
  }

  if (!preflight.complete ||
      preflight.joints_evaluated != servo::kLegPreflightCount ||
      preflight.pass_count != servo::kLegPreflightCount ||
      preflight.no_response_count != 0 ||
      preflight.mismatch_count != 0 ||
      preflight.incomplete_count != 0) {
    out.status = PopulationEvidenceBuildStatus::REJECT_PREFLIGHT_INCOMPLETE;
    return out;
  }

  bool identity_mismatch = false;
  bool qualification_failure = false;
  bool duplicate_slot = false;
  uint16_t seen_slots = 0;

  for (uint8_t i = 0; i < servo::kLegPreflightCount; ++i) {
    const servo::JointPreflightRecord& record = preflight.joints[i];
    const servo::CanonicalServo* canonical = servo::findCanonical(record.expected_bus_id);

    if (canonical == nullptr || !servo::isLegServo(record.expected_bus_id) ||
        canonical->current_config != servo::CurrentConfig::INSTALLED) {
      identity_mismatch = true;
      continue;
    }

    JointIdentity identity{};
    if (!semanticIdentityFromCanonical(*canonical, &identity)) {
      identity_mismatch = true;
      continue;
    }

    const uint8_t slot = legSlotIndex(identity.leg, identity.joint);
    if (slot >= kLegServoSlotCount) {
      identity_mismatch = true;
      continue;
    }

    const uint16_t bit = static_cast<uint16_t>(1u << slot);
    if ((seen_slots & bit) != 0) {
      duplicate_slot = true;
      out.rejected_slots |= bit;
      continue;
    }
    seen_slots |= bit;

    if (!recordMatchesCanonical(record, *canonical)) {
      identity_mismatch = true;
      out.rejected_slots |= bit;
      continue;
    }

    if (!recordQualifies(record, *canonical)) {
      qualification_failure = true;
      out.rejected_slots |= bit;
      continue;
    }

    out.evidence.observed_mask |= bit;
    ++out.qualified_slots;
  }

  if (duplicate_slot) {
    out.status = PopulationEvidenceBuildStatus::REJECT_DUPLICATE_SLOT;
    return out;
  }
  if (identity_mismatch) {
    out.status = PopulationEvidenceBuildStatus::REJECT_IDENTITY_MISMATCH;
    return out;
  }
  if (qualification_failure || out.qualified_slots != kLegServoSlotCount) {
    out.status = PopulationEvidenceBuildStatus::REJECT_JOINT_QUALIFICATION;
    return out;
  }

  if (!populationIsCurrentPass(out.evidence)) {
    out.status = PopulationEvidenceBuildStatus::REJECT_JOINT_QUALIFICATION;
    return out;
  }

  out.status = PopulationEvidenceBuildStatus::PASS;
  return out;
}

const char* toString(PopulationEvidenceBuildStatus status) {
  switch (status) {
    case PopulationEvidenceBuildStatus::NOT_EVALUATED:
      return "NOT_EVALUATED";
    case PopulationEvidenceBuildStatus::REJECT_SOURCE_NOT_CURRENT:
      return "REJECT_SOURCE_NOT_CURRENT";
    case PopulationEvidenceBuildStatus::REJECT_CENSUS_INCOMPLETE:
      return "REJECT_CENSUS_INCOMPLETE";
    case PopulationEvidenceBuildStatus::REJECT_BUS_ANOMALY:
      return "REJECT_BUS_ANOMALY";
    case PopulationEvidenceBuildStatus::REJECT_LEG_MISSING_IN_CENSUS:
      return "REJECT_LEG_MISSING_IN_CENSUS";
    case PopulationEvidenceBuildStatus::REJECT_PREFLIGHT_INCOMPLETE:
      return "REJECT_PREFLIGHT_INCOMPLETE";
    case PopulationEvidenceBuildStatus::REJECT_IDENTITY_MISMATCH:
      return "REJECT_IDENTITY_MISMATCH";
    case PopulationEvidenceBuildStatus::REJECT_JOINT_QUALIFICATION:
      return "REJECT_JOINT_QUALIFICATION";
    case PopulationEvidenceBuildStatus::REJECT_DUPLICATE_SLOT:
      return "REJECT_DUPLICATE_SLOT";
    case PopulationEvidenceBuildStatus::PASS:
      return "PASS";
  }
  return "UNKNOWN";
}

}  // namespace calibration
}  // namespace matdog
