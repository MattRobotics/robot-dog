# MATDOG upstream synchronization — 2026-08-04

## Integrated upstream revision

```text
upstream repository: norma-core/norma-core
upstream main: 78f4bc9678a1db3bc3c63f2ebc1ac175b98c9206
fork base before synchronization: 7b3af210e40cfc9376eff6e362d9678235e2589a
staging merge: bc880a2defbab6881580b38480eb6d93ef5a379e
```

The upstream revision adds the self-contained example:

```text
software/station/examples/arm2dog-py/
```

The reviewed upstream delta contains exactly eleven new files under that directory. It does not modify:

- `software/drivers/st3215/**`;
- `tools/matdog/**` other than this fork-local synchronization record;
- the LF V25 implementation or persistent-profile contracts;
- the durable MATDOG workflows;
- `release/matdog-lf-calibrator-v25`.

## Arm2Dog assessment

`arm2dog-py` bridges an ST3215 leader arm to a Yahboom Dogzilla Lite follower arm through two Station instances. It is an optional example and is not imported, started or enabled by the MATDOG calibrator.

Its documented hardware limitations are material:

- follower servo telemetry is a command echo, not verified physical position;
- no follower current or moving-status telemetry is available;
- no default per-servo collision limits are applied;
- the startup sequence commands a rate-limited arm home pose;
- direction and safe-range values are rig-specific and require supervised validation.

Therefore the example is retained unchanged as upstream reference code but is outside the MATDOG RF calibration path. It must not be used to command MATDOG legs or as a substitute for the Station-owned ST3215 calibration safety contract.

## Required compatibility gate

Before this synchronization reaches `main`, the combined staging tree must pass:

- MATDOG native calibrator offline contract;
- ST3215 Rust tests and format checks;
- MATDOG observer authority contract;
- MATDOG Python runner/profile tests;
- Station release build.

The immutable LF V25 release branch remains byte-identical and is not part of the synchronization.
