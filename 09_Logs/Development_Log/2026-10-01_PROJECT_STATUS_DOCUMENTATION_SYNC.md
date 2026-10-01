# MATDOG — Current-status documentation synchronization (2026-10-01)

**Scope:** documentation-only synchronization after successful TRUE Full Calibration 24/24 and
merge of PR #35 to `main` (`1fd0f5afc3cc737d1ac82183b4ce204dcd01c402`). No new firmware, flash, mechanical measurement or gait
validation was performed as part of this synchronization.

- Hardware-validated firmware: `dfcecb670d0565d2db1a8152b6cd7ad230bdb87d` (BUILD_ID `dfcecb670d05`, `ROBOT_POWERED`).
- Full Calibration: LF/RF/RH/LH, each 6/6 on first attempt; 24/24 accepted; SAFE_OFF 13/13.
- Evidence and full 24-endpoint metrology: [hardware report](../Validation_Reports/Full_Calibration_24_Contact_Hardware_2026-10-01/README.md).
- Current-state documents synchronized: root README, architecture roadmap, Controller README,
  Controller VALIDATION overview, firmware index and Development Gates update date.
- Already current and not rewritten: Controller CHANGELOG, Development Gates substantive criteria,
  2026-09-30 calibration development log (section 8e) and the complete hardware report.
- Historical sections, frozen tool reports and archived failed-session evidence retain their
  historical status and provenance.

**Open:** Calibration Persistence V1 (RAM-only results, no boot restore), telemetry-integrity
investigation (23 isolated temperature readings, 39 held-speed diagnostic events), LOWER MAX
scout margins (RF +6 ticks, LH +13 ticks), a reviewed q0 refinement decision, and operational
workspace/envelope approval. **No stand or gait hardware authorization.**

The offline gait branch/worktree (`feat/gait-engine-offline-v1`, `b06558f`) is not modified.
