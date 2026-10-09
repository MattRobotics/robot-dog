# MATDOG logs and evidence

> **CHRONOLOGY AND EVIDENCE — NOT CURRENT-STATE AUTHORITY**
>
> Start with the [root README](../README.md) for today's hardware, firmware baseline, validation
> status and immediate milestone. Active architecture contracts live in
> [ARCHITECTURE.md](../01_Docs/02_Architecture/ARCHITECTURE.md).

This directory preserves dated engineering decisions, development chronology, calibration records
and validation evidence. A result remains evidence for its recorded date and scope; it does not
automatically describe the robot after a later rebuild or architecture change.

## Navigation

| Area | Purpose |
|---|---|
| [History index](HISTORY_INDEX.md) | One chronology, June–October 2026: phases, status, documents and available evidence |
| [Open items](OPEN_ITEMS.md) | Blockers and residuals; checkpoint after PR #40 (2026-10-08), updated after PR #41 and for C2 (2026-10-09) |
| [Commit provenance map](COMMIT_PROVENANCE_MAP.md) | PR #37–#41 merge SHAs; original SHA to recreated SHA for PR-2 and PR-3 (PR #38 merged with original SHAs; PR #41 is documentation only) |
| [External archives](EXTERNAL_ARCHIVES.md) | Deleted branches (last tips), remaining tags, the non-public archive status and recovery steps; items without evidence stay `EXTERNAL_ARCHIVE_PENDING_VERIFICATION` |
| [ST3215 evidence index](ST3215_EVIDENCE_INDEX.md) | Canonical navigation for bench QC, source survey, provisioning and frozen-tool evidence |
| [`Validation_Reports/`](Validation_Reports/README.md) | Dated hardware, software and model validation evidence; each claim is limited to its stated scope |
| [`Calibration/`](Calibration/) | Calibration records and the 2026-08-27 reset notice; pre-reset values are historical |
| [`Calibration_Sessions/`](Calibration_Sessions/) | Raw and derived records from dated calibration sessions |
| [`Development_Log/`](Development_Log/README.md) | Chronological handoffs and milestone records; their “next steps” are historical |
| [`Architecture_Decisions/`](Architecture_Decisions/README.md) | Dated ADRs; consult the canonical architecture before treating an older decision as active |
| [Historical index](Historical/README.md) | Explicitly superseded material and preserved engineering archives |

## Reading rules

- Validation means only that the documented test passed within its recorded scope.
- A dated handoff records what was current then; it does not own today's milestone.
- ST3215 provisioning of 17 allocated units is not a statement that 17 servos are physically
  installed today.
- Historical calibration and archived tools provide evidence or engineering reference, never
  hardware-motion authorization.
- Keep evidence at its existing path so hashes, manifests and inbound links remain intact.
