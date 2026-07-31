# V28R runner provenance

Original local runner:

```text
~/Downloads/MATDOG_LF_LOWER_M11_MIN_HARDWARE_HANDOFF_V28R.sh
```

The verified terminal transcript proves that this runner:

- checked the V27R marker and SHA-256;
- checked the Station binary and MATDOG UI bundle;
- required the exact explicit arm string `LF_LOWER_M11_MIN`;
- kept HIP profiles blocked;
- started Station only for the supervised V28R run;
- stopped Station in a controlled manner;
- reported the serial free after shutdown.

The exact runner source bytes were not available to the GitHub connector during
this online checkpoint. This file therefore freezes its identity and observed
behavior without fabricating a byte-identical script. The original script must
remain in the local verification archive/download archive until its exact bytes
can be imported and hashed in a later evidence-only commit.
