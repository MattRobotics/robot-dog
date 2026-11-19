# ACQ_PRECONFIG_v2: preparazione del registro 85 (`acq_check.py usb_boot settle_A_0623`)

- **B, usb_boot:** 2026-11-19T06:08:33.557+00:00 `MOTION_ACC_CEILING verified=220 target=220` (4.753 s dall'inizio del log, ultimo uptime None ms)
- **S, prima STAND (settle_A_0623):** `@MOTION STAND 120` → `MOTION_ACC_CEILING verified=220 target=220`: **5.0 ms** (18/11 senza patch: 131 ms; STAND successive 12 ms)
- **Release saltate dal boot prima del primo comando:** 0 (stato READY, release 44024, now_ms 885977, da poller.jsonl)

Verdetti: B PASS, S PASS. W = `cand_vs_ref.py` §3.
