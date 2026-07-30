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

## Registry contracts

- `source_manifest.csv`: `source_id, repository, ref, path, sha256,
  source_class, authority, scope, temporal_status, notes`. Robot-dog hashes are
  checked against the current repository. External hashes bind the pinned ref.
- `source_claim_registry.csv`: exactly the mandatory fields `claim_id,
  domain, statement, classification, confidence, source_repository,
  source_ref, source_path, source_locator, units, applies_to, supersedes,
  conflicts_with, notes`.
- `source_conflict_registry.csv`: conflicting claim IDs, resolution rule,
  status, class and sources. Every `conflicts_with` edge must be covered.
- `joint_registry.csv`: the exact 12 URDF revolute joints, topology, origin,
  axis/limits/units, servo association and profile IDs.
- `frame_registry.csv`: root, joint and nominal foot frames, transforms,
  axes/units, status and provenance.
- `servo_mapping_registry.csv`: one row per canonical joint/servo with sign,
  raw/digital zero fields and direction evidence.
- `calibration_registry.csv`: one row per 24 MIN/MAX profiles, separating
  software status, hardware status, modeled limit, guard and observed contact.
- `limit_registry.csv`: URDF, mechanical-contact and operational-safe-limit
  rows. `is_operational_safe_limit=true` requires `limit_type=operational_safe`
  plus evidence and a non-unknown status.
- `decision_registry.csv`: `decision_id, question, why_it_matters,
  required_evidence, blocked_work, owner, target_milestone, status`.
- `unresolved_registry.csv`: open proof gaps, required evidence, impact and
  owning decision.

## Validator-enforced frozen boundaries

Claims `C-UNKNOWN-*` must remain `UNKNOWN` until this schema and its tests are
deliberately updated with new evidence. XGo source rows may support only
`XGOLITE_ARCHITECTURAL_REFERENCE` claims in architecture/interface domains.
The exact required conflict pairs and open-decision evidence are data-driven
and validated. Paths under `MattRobotics/robot-dog` must exist locally; paths
for other repositories are ref-qualified and are not resolved against this
worktree.

The validator invocation is:

```bash
PYTHONDONTWRITEBYTECODE=1 \
PYTHONPYCACHEPREFIX=/tmp/matdog-milestone-i-pycache \
python3 06_Software/Matdog_Core/milestone_i/validate_foundation.py --check
```
