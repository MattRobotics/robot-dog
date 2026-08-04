# MATDOG repository cleanup and LF V25 closeout

**Date:** 2026-08-04  
**Status:** final remote closeout

## Result

The LF V25 milestone is complete and frozen. Repository presentation has been consolidated around the actual validated state:

```text
LF V25: validated on hardware
RF/RH/LH: not yet mechanically calibrated
all-leg persistent profile: not yet complete
```

## Canonical refs

```text
robot-dog/main
norma-core/main
norma-core/release/matdog-lf-calibrator-v25
```

The release branch preserves the exact reviewed LF V25 source. Future-leg development starts from merged `norma-core/main` and must not rewrite the release branch.

## Historical development

Superseded versioned experiments remain available through their closed pull requests as audit history. Their remote working branches and temporary workflow definitions are not retained as operational choices.

## Workflow policy

Only durable current CI and successful LF V25 evidence are retained. Failed, cancelled, incomplete, duplicate and superseded workflow runs may be removed from GitHub Actions.

## Public repository policy

`robot-dog` documents MATDOG itself. Unrelated private research sources are excluded from the public repository, documentation and machine-readable baseline.

## Next milestone

```text
generalize merged V25 architecture to RF
→ supervised RF six-contact calibration
→ RF affine gate
→ transactional RF freeze
```
