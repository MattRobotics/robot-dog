# MATDOG handoff — LF_LOWER_M11_MIN V28R PASS

Remote truth was rechecked before publication on 2026-07-31.

## Hardware result

`LF_LOWER_M11_MIN` completed with contacts `3094 / 3092`, spread `2`,
`Done 14/14`, verified global torque OFF and serial free after Station stop.

## Remote state observed before this checkpoint

```text
MattRobotics/norma-core main: 32e3222c87016b7f5d7c1c1da497a4cea3e7b80a
norma-core PR #4: open draft, head b06cc2bf2e36fb5bbaae12e48c5998c7668862ef, workflow success
norma-core PR #5: open draft, head a87a8aecf0cc9a49c770bb213036a7670aa9a3a8, workflow success
MattRobotics/robot-dog main: a48395ab1e4801baf328af5db54ce5ccbaac69f3
robot-dog PR #3: open draft, head cc70718b4bfefc46349551561aabd5636bd2eef8, workflow success
```

The remote did not yet contain the local source chain ending at
`5c52a93c2f889c556fb2f66cfce70f8843354f58` or the V28R evidence.

## Next mandatory sequence

1. Publish this evidence checkpoint on a draft branch/PR only.
2. Materialize the validated NormaCore source chain on a separate draft branch.
3. Run independent CI with 87 ST3215 tests, zero Rust warnings, viewer build,
   Station release build and RAM-only audits.
4. Only after CI succeeds, derive the offline gate and a separate guarded runner
   for `LF_LOWER_M11_MAX`.

No merge to `main` and no hardware run are authorized by this handoff.
