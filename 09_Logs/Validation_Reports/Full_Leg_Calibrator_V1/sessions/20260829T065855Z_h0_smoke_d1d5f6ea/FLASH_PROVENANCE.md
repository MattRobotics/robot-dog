# H0 flash provenance — 2026-08-29

This session validated a specific image on real hardware. Recording what was
flashed matters more than recording that a flash happened.

| Field | Value |
|---|---|
| Candidate commit | `ceb5e445ef03280983838870e789ba626c9c8944` |
| Worktree at build | clean (`BUILD_WORKTREE_DIRTY=NO` read back from the device) |
| Build stage | `H0_ESP32_ONLY`, H3 bootstrap **denied** |
| **Flashed binary SHA256** | `ebee9d2f203be7ab165b0647d289208c63687db82a7b9329691df25af7a64cc0` |
| FQBN | `esp32:esp32:esp32s3:USBMode=hwcdc,CDCOnBoot=cdc,UploadMode=default,CPUFreq=240,FlashMode=qio,FlashSize=16M,PartitionScheme=app3M_fat9M_16MB,DebugLevel=none,PSRAM=opi` |
| Port | `usb-Espressif_USB_JTAG_serial_debug_unit_14:C1:9F:22:75:94-if00` |
| Result | **29/29 gates PASS** |
| Servo power | **OFF for the entire session**, never touched |
| Servo motion | none — `motion_actually_attempted=false` |
| EEPROM writes | 0 |
| Broadcast writes | 0 |
| Final torque state | `OFF_VERIFIED` |

Verified before flashing that the binary arduino-cli actually writes (its cache
copy) was byte-identical to the exported artifact that was hashed; esptool then
reported `Hash of data verified` for all four flashed regions.

## Why the binary hash is not the same as an earlier published value

At the time of this session the firmware still printed `BUILD_DATE=__DATE__` and
`BUILD_TIME=__TIME__`, so the compile instant was baked into the image and two
clean builds of the SAME commit produced different SHA256s. The hash above is
therefore the hash of *the image that was actually flashed in this session*, not
a value reproducible from the commit alone.

That defect is fixed after this session: build metadata is now derived from the
commit and `SOURCE_DATE_EPOCH` pins the toolchain's macro expansion, so two cold
builds of one commit are byte-identical. See
`05_Firmware/Full_Leg_Calibrator_V1/tools/check_reproducible_build.sh`.

## First connect attempt failed — see the sibling session

`../20260829T065640Z_h0_smoke_3da4b210/` records a genuine FAIL:
`firmware READY marker was not received`. The host runner assumed the ESP32-S3
resets when the port is opened. It does not with this FQBN: two consecutive
opens both reported `BOOT_SESSION_ID=BAFA9902` and emitted nothing at all. The
firmware prints its banner once in `setup()` and never waits for a host, so the
old connect path could only succeed inside the ~1.5 s window after a reset.

That session is preserved deliberately. The PASS above was obtained by
re-flashing the identical verified binary and connecting immediately, which won
the race. The runner has since been changed to a deterministic `@STATUS`
handshake so connecting no longer depends on catching the banner.
