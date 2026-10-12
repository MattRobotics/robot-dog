# Prone recovery — evidence, 12 October 2026

The historical physical stand to approximately 150 mm is documented. The actual initial natural belly-supported folded pose is **UNKNOWN** as a measured twelve-joint state. This audit found no complete coherent folded-start readback. It does not equate that physical pose with `q=0`, calibration fold prerequisites or `REST_GROUND`.

Only these two evidence documents were written. No hardware, historical executor, NVS/EEPROM, torque or motion operation was used. No Full Calibration was reopened, and the generation-1 record is preserved. No applicable `AGENTS.md` was located in the searched ancestors/trees. The search covered the Motion worktree/local Git blobs and the named MATDOG historical handoff/download archive; absence of a capture is bounded by this scope.

## Three distinct records

1. **Physical success declaration, recorded 22 September.** Commit `58b97392dc13efa30e166fe3802efa677dcba011`, [CALIBRATION_BOOTSTRAP.md](../../../05_Firmware/MATDOG_Controller/CALIBRATION_BOOTSTRAP.md#L723), source-commit lines 684–686 (current corresponding lines 723–725): “already executed the C4-C 51-frame stand trajectory on real hardware” and “from near q=0 to roughly 150 mm body height, on the ground, four-foot support, stable Torque-ON hold.” [H0_LEG_PREFLIGHT_RUNBOOK.md](../../../05_Firmware/MATDOG_Controller/H0_LEG_PREFLIGHT_RUNBOOK.md#L542), lines 542–545, repeats the successful stand and current-installation revalidation requirement. This establishes a repository historical declaration of success, without a measured initial/final twelve-joint capture, event timestamp or measured base_link-to-plane height. The old encoder-direction prose in that commit is not evidence for the current mapping.
2. **Separate failed supported C5 attempt, 9 July.** Archive `/home/matteo-manicardi/MATDOG/archive/matdog-download-artifacts/2026-07-26_cleanup/README_C5_HANDOFF_NEXT_CHAT.md`, lines 91–99 and 191–248, records 121 delivered steps over 24.24 s, plausible LF/RH/LH final poses and RF failure. RF HIP readback was −0.829883606246 rad, beyond its ±0.785398163397 URDF range; RF LOWER folded upward and collided with HIP. Delivery completion and Torque-ON hold did not prove physical stand success. The initial 12 raw readings were explicitly a return to *zero calibration*, not a measured natural prone fold. The archived executor assumes visual zero and bridges directly in circular encoder ticks to the 100 mm C4 frame; this bridge is not admissible in the new architecture.
3. **G3.5 `REST_GROUND`, exact static CAD model.** Read-only Git source `e1704719979789cd9c9f18741e4546725558797e:09_Logs/Validation_Reports/G35_Pose_Audit/pose_library.json`, SHA-256 `746f4393a2f9de5dc532c6b48e083f2089e3cc2fa25f381dc4ef9d08f9903a58`, matches the current [PoseReferenceData.h](../../../05_Firmware/MATDOG_Controller/src/motion/PoseReferenceData.h#L6). No excluded G3.5 artifact was imported. The source report lines 17, 31–43 and 79–86 state model-only evidence, body support with all feet clear, real contact height not established and no validated connection to STAND in the bounded search. That search does not prove physical impossibility.

## Exact REST_GROUND geometry, not a physical capture

Semantic radians, ordered HIP / UPPER / LOWER:

| Leg | HIP | UPPER | LOWER | Upper segment inclination | Lower segment inclination | Directed segment angle |
|---|---:|---:|---:|---:|---:|---:|
| LF | +0.260000000000 | 1.958362087164 | 0.588021958282 | 21.4219° | 8.9810° | 31.3142° |
| RF | −0.260000000000 | 1.958362087164 | 0.588021958282 | 21.4219° | 8.9810° | 31.3142° |
| RH | 0 | 2.048195210429 | 0.455068935851 | 27.3529° | 11.5703° | 38.9300° |
| LH | 0 | 2.048195210429 | 0.455068935851 | 27.3529° | 11.5703° | 38.9300° |

These new metrics use the exact current URDF pivot-to-pivot chain: upper vector `Rx(h)Ry(u)·(0,0,−0.09)`, lower vector `Rx(h)Ry(u+l)·(0.107,foot_y,−0.0499)`. Inclination is `asin(|vz|/|v|)` against the level model plane; directed angle is `acos(a·b/(|a||b|))`. They describe joint-pivot segments, not visual mesh long axes. They do not establish that both physical links lie approximately parallel to the floor. The internal knee endpoint angles are 148.6858° front and 141.0700° rear. No natural-fold joint vector was invented.

The model body rotation is identity and `base_link.z = 1.0999563076780766e−16 m`, derived from its mesh minimum, numerically zero. Only the body supports the robot. Lowest foot mesh vertices are approximately 45.62 mm above the plane at the front and 17.30 mm at the rear. The minimum tested non-adjacent mesh separation is 0.661635 mm; this is neither assembled clearance approval nor a motion envelope. CAD/URDF mass is 2.48 kg, not a hardware measurement.

All **19/19** frozen canonical source pins match current file bytes: URDF, 17 collision meshes and contact geometry YAML. Exact paths and hashes are in [prone_evidence.json](prone_evidence.json). This identifies the model pose reproducibly, while leaving its equality to the actual natural physical fold **UNKNOWN**.

## Reusable C4-C and current mapping

[C4-C report](../C4_rest_to_stand_trajectory/2026-07-08_190405_C4C_contact_locked_rest_to_stand_trajectory.json), SHA-256 `4b667a5995e55122b9578c39042914a373aa6bb4e0be84de11381e7f2398b7b2`, is a 51-sample contact-locked **100→150 mm** rise component, with `offline_only=true` and `command_eligible=false`. Its own policy rejects direct q0 interpolation. It does not contain belly-supported acquisition. Its semantic rise can be reused subject to current full V5 collision/contact/continuity and operational checks.

The original C4 commit `7345831` used URDF SHA `5e28da3dba10fd3f2ea6ebf6f5d6271157bda0b12b82d92aedbe3031643089ef`; current SHA is `3890a3f0732dbed8abdc559106d7f32ee8d6e2111c8e1a06d2485bf2ffc81e59`. XML comparison confirms all 16 joint elements are equal and all 17 link elements are equal after collision-path relocation. Original LFS mesh content IDs differ from current collision asset hashes, so this comparison does not certify identical collision surfaces or waive current V5 validation.

The July raw ticks are not current operational targets. [27 August assembly reset](../../Calibration/MATDOG_CALIBRATION_RESET_2026-08-27.md), lines 13–55, records removed/remounted horns and links plus reprovisioning. [ServoPopulation.h](../../../05_Firmware/MATDOG_Controller/src/servo/ServoPopulation.h#L74) distinguishes semantic bus slot from expected physical unit, whose identity the bus cannot confirm. The [1 October q0 reference](../Full_Calibration_24_Contact_Hardware_2026-10-01/q0_promoted.json) is current calibration evidence, **not an observation of the folded pose**. Runtime RESTORE, calibration acceptance and a motion permit remain separate. Existing Full Calibration need not be repeated to obtain a pose observation.

## Missing observations for identifying the physical start

- A coherent timestamped capture of all 12 actual encoders in the current natural belly-supported fold, with verified generation-1 runtime mapping, units, freshness and uncertainty.
- Observed base_link orientation and position relative to a measured support plane. BNO085 orientation alone does not provide base height.
- An independent contact/support witness for belly and feet, plus plane normal and relevant fixture conditions. IMU and encoders alone cannot establish ground contact.
- Provenance tying the historical successful start/end readings and timing to an installation, if such a capture can be recovered.

`physical_readiness = false`. These evidence distinctions authorize no hardware action and establish no autonomous prone-to-STAND route.
