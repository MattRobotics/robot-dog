# `30c894a` — qualifica hardware del 18/11/2026: l'immagine di rollback della Stable `v1.0.2`

**Contenuto:**
- `REPORT_HW.md` è una copia invariata del rapporto della sessione del 18/11 (11:17Z–12:36Z, fixture).
- Originale locale: `~/MATDOG/verification-artifacts/MATDOG_1_0_2_ACQV2_30C894A_HW_20261118/`. Sigillo `SHA256SUMS.txt` `2ca75b3e…`, 604 file, verificato il 19/11; qui come `SHA256SUMS.local.txt`.

**Identità:**
- `1.0.2-dev.7`, build `30c894a0d430`, sorgente `30c894a0d4303d497d750cccb5fd12f347c61d7d`;
- APP 1 779 312 B, SHA256 `03c863cc433622f672c11cae45f46694c4aa6f78ba0136cc1b7bc818a2e4b340`.

**Esito del 18/11 (VERIFIED HARDWARE):**
- identità, NVS gen 1 e calibrazione 12/12 conformi;
- catena 140/140, STOP USB 14/14;
- ACQUIRE v2 PASS dopo il reset del flash;
- **Motion Timing P0 allora aperto:** 20 761 µs, 2 overrun.

**Ruolo oggi:** è l'immagine di rollback della Stable [`v1.0.2`](../Motion_1314bc4_HW_2026-11-19/RELEASE_MANIFEST.md#4-rollback).
- Ha la stessa tabella delle partizioni, lo stesso sdkconfig e gli stessi schemi NVS.
- Il 18/11 sera Fable l'ha riscritta sul robot con lo script qualificato `812a2e83…` (`digest matched`).
- Si trova in `~/MATDOG/backups/esp32/app_images/30c894a/` (sigillo `8a1aa6d3…`).

**Nota:** i percorsi del rapporto che puntano a `../MATDOG_1_0_2_ACQV2_CLEAN_20261118/…` e alle sottocartelle `qon/`, `usb_boot/`, `flashlog/` si riferiscono alle cartelle locali sull'Asus, non al repository.
