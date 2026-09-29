# MATDOG G3.5 offline pose audit

This toolset reads canonical URDF collision geometry and inertials. It never
imports vendor libraries, connects to a device, or produces actuator commands.
The compact C++ records contain semantic URDF radians only. Static pose evidence
does not authorize startup, a transition, or physical actuation.

## Resume state

The accepted G3 start was `87c3e914e068260170e7c19c5d664f727f5b03ce`. The
workstream was interrupted twice.

- First resume: Git had no staged/tracked changes or G3.5 commits. Thirteen
  analysis files, two draft pure C++ support files and eleven JSON reports were
  present and retained. The disposable `/tmp` environment, XGO archive and Ghidra
  exports had expired and were reconstructed; fresh exports matched every
  preserved action-handler hash. This resume committed the XGO trace (`024e626`),
  the mesh model (`274d8ad`), the contact-mode routes (`51319b3`) and the pose
  support layer (`821c266`).
- Second resume: state was reconstructed from Git and the filesystem without
  modification, every valid artifact was kept, and the closing evidence was added
  (`rest_connectivity.py`, `connectivity.py`, `lower_envelope.py`,
  `artifact_manifest.py`, `validate.py`, `build_report.py`, extended tests).
  `rest_connectivity.py` was needed because the original body-only connection
  searches seeded only one of six valid body-only candidates.

`research_checkpoint.md` records the older interruption. `revalidation.json`
records whole-robot sample rechecks.

## Reproduce without hardware

From the repository root, create an isolated Python environment:

```sh
python3 -m venv --system-site-packages /tmp/matdog-g35-env
/tmp/matdog-g35-env/bin/pip install -r 06_Software/Matdog_Core/pose_audit/requirements.txt
```

Use `/tmp/matdog-g35-env/bin/python` for the commands below, in order. Paths
are relative to `06_Software/Matdog_Core/pose_audit/`:

1. `survey.py --stage rest`
2. `survey.py --stage envelope`
3. `contact_modes.py`
4. `expanded_survey.py`
5. `survey.py --stage transitions`
6. `transfer_search.py`, then `support_transfer.py`
7. `route_extension.py`, then `equivalent_stand.py`
8. `diagnostics.py`, `revalidate.py`, `policy_report.py`; then run
   `diagnostics.py --sources equivalent_stand --output equivalent_stand_diagnostics`
   and `revalidate.py --sources equivalent_stand --output equivalent_stand_revalidation`
9. `pose_library.py`, `pose_export.py`, `retarget.py`
10. `rest_connectivity.py` (body-only start connectivity), `lower_envelope.py`, `connectivity.py`, `transition_summary.py`
11. `render.py`
12. `validate.py` (runs and records every gate below), then `build_report.py`,
    then `artifact_manifest.py`
13. `artifact_manifest.py --check` and `test_pose_audit.py`, once more after the
    report and manifest are final

Expensive historical search outputs are preserved as inputs. Their original
provenance describes the generator revision used at the time; later model
validation is separately recorded rather than relabeling old runs. Numeric
results should agree at the stated tolerances. Byte identity is expected with
the pinned environment, not across arbitrary BLAS/FCL versions. All generators
use deterministic grids or fixed random seeds. No time-dependent seeds occur.

The standard-library-only embedded freshness check is:

```sh
python3 06_Software/Matdog_Core/pose_audit/pose_export.py --check
bash 05_Firmware/MATDOG_Controller/scripts/tests/run_motion_host_tests.sh
python3 05_Firmware/MATDOG_Controller/scripts/static_audit.py
```

The static audit invokes the complete existing host runner, including G1/G2/G3
oracles and the new C++ pose policy test. It only prints its aggregate result
when all captured subprocesses succeed. `validation_results.json` records the
executed gates, including revalidation of the preserved samples, the full host
suite, the ASan/UBSan build and run of `test_pose_support.cpp`, and
`git diff --check`. `build_report.py` reads every number in `REPORT.md` from the
saved artifacts and the canonical URDF, so the report is regenerated, not edited.
The manifest indexes `REPORT.md` and `validation_results.json`, so its check and
the final test rerun happen after both are final and are not stored in an artifact.

## XGO source reconstruction

The original checkout remains untouched. Read-only Git objects supplied the
archive at commit `a1b34a8594e5bc76c76b1e3ddf89a3aef2b98298`; use `git archive`
into a new temporary directory, never checkout/reset the original worktree.
The source cache and Ghidra project paths are those explicitly referenced by
that repository, pinned in the JSON evidence inventories.

Copy `xgolite_g2_segment_aware.gpr` and `.rep` to a disposable directory before
using Ghidra. Run `PoseExport.java` with `-process xgolite_app_v4.3.7.bin
-noanalysis -readOnly`. Firmware must never be executed. `xgo_audit.py` accepts
`--xgo`, `--exports`, `--cache`, and `--firmware` to locate these read-only inputs.
Its optional vendor corpus was extracted from the referenced CM4/CM5 ZIP files
for textual inspection only; inventory hashes preserve exactly what was read.
The resulting action-handler text extracts are committed so temporary files
are not needed to review the conclusions.

`evidence_closure.py` compares pinned historical/current official motion PDFs
and records command-slot semantics. It requires network access to immutable
GitHub source URLs, but executes no downloaded code. `dimensions.py` parses the
hash-matching primary family XML, explicitly correcting the conflicting derived
H2 summary without modifying the XGO repository. Exact Lite dimensions remain
unknown where no authoritative physical geometry was recovered.

## Contact and collision contracts

- G1 still targets the `foot_link` origin.
- G2 still uses the canonical eccentric finite-cylinder reference.
- Offline mesh IK holds that reference's XY and solves actual lowest mesh Z.
  It uses previous valid joint angles as the next numerical seed.
- Inclined edge contact and migration of the mesh support patch are recorded;
  these are not asserted to be G2 strip-contact locking or proven no-slip motion.
- Penetration/undeclared-contact tolerance: 1 micrometre. Declared patch band:
  10 micrometres. Neither is a physical compliance or actuator limit.
- All 17 meshes are watertight. All 120 nonadjacent pairs are tested using FCL
  triangles plus closed-solid containment. Sixteen direct URDF adjacency pairs
  are explicitly excluded for assembly/joint interfaces. No claim is made about
  internal adjacent-link clearances. See `collision_policy.json`.
- Reported support is the convex hull of intended mesh patches. COM uses all
  17 URDF masses/inertial offsets and is always **CAD/URDF MODEL COM**.
- Full 3D distance to the lowest patch, vertical error, and tessellation-dependent
  patch-centroid displacement are different diagnostics. Do not substitute the
  centroid for the canonical G2 contact reference.

## Artifacts and limits

All results are in `09_Logs/Validation_Reports/G35_Pose_Audit/`.
`REPORT.md` answers the 22 requested questions; `artifact_manifest.json` indexes
hashes and cross-checks recorded provenance. `pose_library.json` retains both compact validated static targets and
clearly marked research records. Only six validated static records enter
`PoseReferenceData.h`. No new motion state or startup route is enabled.

The validated transition graph (`connectivity.json`) unions only saved routes that
passed the full offline policy. A missing edge means no validated route was found
by the bounded searches, never a proof of impossibility; singleton nodes are
untested, not proven disconnected. BODY_SUPPORT `REST_GROUND` is not connected to
the stand component and is not authorized as a startup or transition source.

Finite search failures do not prove global impossibility. Sampled route success
does not certify continuous collision clearance, friction, contact loads, rate
limits, or physical support transfer. G3 startup remains explicitly gated.
