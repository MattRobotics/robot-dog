# Milestone I registry schemas

All files are UTF-8 RFC 4180-style CSV with one header row. IDs are stable,
case-sensitive and unique within their registry. Multi-value ID fields use
semicolon separators; vector fields use space-separated scalars. Empty means
unknown/not applicable only when the row's status or classification says so.

## Shared enums

Claim classifications are defined in the source policy. Confidence is one of
`HIGH`, `MEDIUM`, `LOW`, `NOT_APPLICABLE`. Profile/evidence status values are
`validated`, `partially-validated`, `software-ready`, `hardware-pending`,
`blocked`, `unknown`. Decision status is `OPEN` or `CLOSED`; this foundation
contains only evidence-consistent open decisions.

`foundation_expectations.json` is the versioned acceptance manifest. It
contains immutable repository pins, exact registry ID sets, canonical counts
and distributions, conflict/decision/unresolved states, critical provenance,
joint directions, profile derivation constants and document metrics. Updating
a registry and this baseline together is a review-visible semantic change.

## Registry contracts

- `source_manifest.csv`: `source_id, repository, ref, path, sha256,
  source_class, authority, scope, temporal_status, parse_status,
  interpretation_status, notes`. Robot-dog hashes are checked against the
  current repository. External hashes bind the pinned ref. Authority and scope
  are mandatory. The M11 historical blob is explicitly text-only and
  non-parseable.
- `source_claim_registry.csv`: exactly the mandatory fields `claim_id,
  domain, statement, classification, confidence, source_repository,
  source_ref, source_path, source_locator, units, applies_to, supersedes,
  conflicts_with, notes`.
- `source_conflict_registry.csv`: conflicting claim IDs, resolution rule,
  status, class and sources. Every `conflicts_with` edge must be covered.
- `joint_registry.csv`: the exact 12 URDF revolute joints, topology, origin,
  RPY, axis, lower/upper/effort/velocity fields, units, servo association,
  custom `motorType`/`motorId`/`motorDirection`/`armature` values and profile
  IDs.
- `frame_registry.csv`: root, joint and nominal foot frames, transforms,
  axes/units, status, source joint, source link and provenance. Planned frames
  have no materialized source joint/link.
- `servo_mapping_registry.csv`: one row per canonical joint/servo with sign,
  raw/digital zero fields and direction evidence.
- `calibration_registry.csv`: one row per 24 MIN/MAX profiles, including
  order, joint role, allowed motor IDs, prerequisite targets, reverse restore
  order, home/visual zero, modeled limit, guard, software/hardware states,
  observed contact and explicit operational-safe flag.
- `limit_registry.csv`: URDF, mechanical-contact and operational-safe-limit
  rows. `is_operational_safe_limit=true` requires `limit_type=operational_safe`
  plus evidence and a non-unknown status.
- `decision_registry.csv`: `decision_id, question, why_it_matters,
  required_evidence, blocked_work, owner, target_milestone, status`.
- `unresolved_registry.csv`: open proof gaps, required evidence, impact and
  owning decision.

## Validator-enforced frozen boundaries

The accepted claim IDs and classification distribution must match the
expectation manifest; designated `C-UNKNOWN-*` rows must remain `UNKNOWN`
until this schema and its tests are deliberately updated with new evidence.
XGo source rows may support only
`XGOLITE_ARCHITECTURAL_REFERENCE` claims in architecture/interface domains.
The exact conflict, decision and unresolved ID/status/category sets are
data-driven and validated. Paths under `MattRobotics/robot-dog` must exist
locally; paths for other repositories are ref-qualified and are not resolved
against this worktree.

The validator invocation is:

```bash
PYTHONDONTWRITEBYTECODE=1 \
PYTHONPYCACHEPREFIX=/tmp/matdog-milestone-i-pycache \
python3 06_Software/Matdog_Core/milestone_i/validate_foundation.py --check
```
