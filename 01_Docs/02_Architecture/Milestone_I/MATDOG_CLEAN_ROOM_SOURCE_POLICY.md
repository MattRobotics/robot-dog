# MATDOG Milestone I clean-room source policy

Status: `CURRENT_CANONICAL` for Milestone I.0-I.1.  Date: 2026-07-30.

## Scope and invariants

This policy governs the documentary and machine-readable MATDOG foundation.
It does not authorize hardware access, Station execution, motion, gait, IK,
firmware, a serial adapter, or Milestone I.2.  A claim is no stronger than its
row in `source_claim_registry.csv`, even when prose uses a convenient name.

The authoritative operational NormaCore repository for this milestone is
`MattRobotics/norma-core`, not the external upstream `norma-core/norma-core`.
The upstream is reference-only and its PR #4 is unrelated to MATDOG.

## Classification vocabulary

| Classification | Meaning |
|---|---|
| `MATDOG_VERIFIED` | Directly supported by a canonical MATDOG file or identified, versioned MATDOG hardware evidence. |
| `MATDOG_DERIVED` | Reproducible result derived only from verified MATDOG CAD, URDF, mapping, or calibration data. |
| `HARDWARE_OBSERVATION` | A physical measurement or event; it is not generalized automatically. |
| `NORMACORE_MATDOG_FORK_MAIN_FACT` | Behavior present at the pinned `MattRobotics/norma-core` main commit. |
| `NORMACORE_EXPERIMENTAL_PR` | Behavior or validation present only in pinned `MattRobotics/norma-core` PR #4. |
| `NORMACORE_UPSTREAM_REFERENCE` | Code or architecture observed in external `norma-core/norma-core`; never automatically canonical for MATDOG. |
| `NORMACORE_GENERIC_REFERENCE` | Generic SO101/ElRobot behavior; never automatically valid for MATDOG. |
| `XGOLITE_ARCHITECTURAL_REFERENCE` | Transferable abstract boundary or pattern; never an XGo physical constant. |
| `CORROBORATED` | Supported from more than one source but not proved canonical. |
| `UNKNOWN` | Not demonstrated. |
| `DECISION_REQUIRED` | A future choice that may not be selected implicitly. |
| `SUPERSEDED` | Historical information that is no longer operational. |

Allowed confidence values are `HIGH`, `MEDIUM`, `LOW`, and
`NOT_APPLICABLE`. Confidence is independent of classification.

## Source and document state

Source rows also carry one of `CURRENT_CANONICAL`, `CURRENT_SUPPORTING`,
`HISTORICAL`, `SUPERSEDED`, `HARDWARE_EVIDENCE`, `EXPERIMENTAL`, or
`UNKNOWN_STATUS`. A historical file is preserved, not retroactively edited.
Every numerical claim records units; blank units mean not applicable, never
an implicit unit.

## Promotion and conflict rules

- `UNKNOWN`, `CORROBORATED`, vendor evidence, or architecture references may
  be promoted only by a new identified primary source and an explicit registry
  change. The validator freezes designated unknown claims.
- Conflicting claims remain linked in both the claim and conflict registries.
- A mechanical contact is not an operational safe limit. A URDF joint limit
  is not proof of contact, and independent joint limits do not prove a
  collision-free configuration.
- XGoLite geometry, frames, signs, zeros, offsets, IDs, registers, timing,
  gains, bus details, and servo characteristics are prohibited as MATDOG
  physical facts.
- Generic NormaCore EEPROM/reset/freeze behavior is not a MATDOG calibration
  contract. The MATDOG contact path is RAM-only.

## Command-position semantics

ST3215 `GOAL_POSITION` is always an unsigned `0..4095` command. A circular
signed tick delta may be used only as a local mathematical difference. It may
not be serialized or wrapped into a command. Visual zero, raw encoder,
digital zero, EEPROM Position Offset, post-recenter `q=0`, mechanical contact,
URDF limit, travel guard, prerequisite pose, calibration corridor,
first-stand envelope, operational safe limit, and configuration-dependent
collision boundary are distinct concepts.

## Change rule

Generated registries are reviewed data, not a replacement for evidence. Any
later change must update sources, claims, conflicts/decisions, validator tests,
acceptance counts, and handoff together. No listed decision is resolved merely
to make validation pass.
