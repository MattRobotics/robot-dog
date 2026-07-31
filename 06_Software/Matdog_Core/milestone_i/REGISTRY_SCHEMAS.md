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

`foundation_expectations.json` is the versioned acceptance inventory. It
contains immutable repository pins, exact registry ID sets, canonical counts
and distributions, conflict/decision/unresolved states, critical provenance,
joint directions, parsed profile values and document metrics. It is not the
sole evidence authority: code-owned enums and compatibility, pinned Git blobs,
critical locator baselines, robot-dog historical values and the static
NormaCore parser remain independent gates.

## Registry contracts

- `source_manifest.csv`: `source_id, repository, ref, path, sha256,
  source_class, authority, scope, temporal_status, parse_status,
  interpretation_status, notes`. Every hash-bearing row is checked against
  `ref:path` in the matching explicit local Git repository. The sole exception
  is the declared hashless upstream external reference. Authority and scope
  are mandatory. The M11 historical blob is explicitly text-only and
  non-parseable.
- `source_claim_registry.csv`: exactly the mandatory fields `claim_id,
  domain, statement, classification, confidence, source_repository,
  source_ref, source_path, source_locator, units, applies_to, supersedes,
  conflicts_with, notes, line_start, line_end,
  expected_excerpt_sha256`. The final three fields are mandatory for
  code-designated critical claims and blank for ordinary claims.
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
validated. Frame, authority, temporal, parse/interpretation,
conflict/decision/unresolved enums and the source class + authority + scope
compatibility matrix are code-owned. Every hash-bearing source path is
resolved at its pinned Git ref; no source is read from mutable worktree bytes.

The validator proves repository/ref/path/blob identity, excerpt identity,
numeric/mapping/formula/enum/count conformance (`MACHINE_VERIFIED`). Whether a
verified excerpt is semantically sufficient for its claim remains a human
review judgment (`HUMAN_REVIEWED`).

The validator invocation is:

```bash
PYTHONDONTWRITEBYTECODE=1 \
PYTHONPYCACHEPREFIX=/tmp/matdog-milestone-i-pycache \
python3 06_Software/Matdog_Core/milestone_i/validate_foundation.py --check \
  --robot-dog-repo /path/to/robot-dog \
  --normacore-repo /path/to/norma-core \
  --xgolite-repo /path/to/xgolite-low-level-reconstruction
```
