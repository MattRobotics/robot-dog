# Architecture decisions — index

Dated decision records. Each ADR keeps the text it had when it was written; later status changes
are added as annotations, never by rewriting the original decision.
Current architecture: [`ARCHITECTURE.md`](../../01_Docs/02_Architecture/ARCHITECTURE.md).
Chronology: [`HISTORY_INDEX.md`](../HISTORY_INDEX.md).

| ADR | Date | Subject | Status today | Note |
|---|---|---|---|---|
| [ADR-001](ADR-001_Quadruped_Control_Architecture.md) | 2026-06 | Quadruped control architecture: NormaCore Station as sole ST3215 bus owner, Waveshare adapter, 12 servos | **SUPERSEDED** (banner dated 2026-08-27) | Replaced by the ESP32-S3 Controller as operational bus owner, with 17 allocated servo slots (13 physically installed today: 12 leg + neck ID 51). Preserved as the historical decision. |
| [ADR-002](ADR-002_MATDOG_Repository_and_Station_Integration.md) | 2026-06-25 | `robot-dog` is the MATDOG reference repository; boundary with NormaCore Station | **Accepted, partially superseded** (annotation 2026-10-08) | Still valid: `robot-dog` is the source of truth for MATDOG. Superseded: direct servo-bus control by NormaCore Station. See the "Status update" section of the ADR. |
| [ADR-003](ADR-003_URDF_REV00_Kinematic_Baseline.md) | 2026-06-30 | URDF REV00 kinematic baseline | **Accepted** | Canonical robot description used by geometry, kinematics and motion code. |
| [ADR-004](ADR-004_Third-Party_Reverse_Engineering_Material_Boundary.md) | 2026-10-08 | Third-party reverse-engineering material stays out of MATDOG history | **Accepted** (annotation 2026-10-09: branches deleted, exposure not guaranteed removed) | Boundary applied by PR #39 and PR #40. Deferred material is recorded by path and SHA-256 only. Not to be weakened without a separate owner decision. |

Related dated decisions kept outside the ADR series:

- [Calibration reset, 2026-08-27](../Calibration/MATDOG_CALIBRATION_RESET_2026-08-27.md) — all
  calibration recorded before the physical rebuild is historical.
- [Historical index](../Historical/README.md) — the three events (Station to ESP32-S3, 12 servos to
  17 allocated slots, hardware decisions frozen).
