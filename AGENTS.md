# AGENTS.md — MATDOG

> **Baseline attuale (20/11/2026): installata `1.0.3-dev.2` (TROT58, build di sviluppo) su `main`; Stable pubblicata [`v1.0.2`](https://github.com/MattRobotics/robot-dog/releases/tag/v1.0.2).**
> - **Installato:** `1.0.3-dev.2`, build `3152e880c3cc`, commit `3152e88`, APP SHA256 `3c8774a9f9d0facfc0cb65af67ad4745c10bd9b2a2a9e906cbb8b0d9802f8c68`. Nessun tag né release.
> - **Qualifica:** solo sulla fixture, il 19/11 sera. KEEP, C1–C10 PASS ([rapporto](09_Logs/Validation_Reports/TROT58_HW_2026-11-19/REPORT.md)). Locomozione a terra **non** qualificata.
> - **Stable/Latest:** `v1.0.2` (`1314bc4`, APP `dd16bd62…`), invariata; è il rollback ([dossier](09_Logs/Validation_Reports/Motion_1314bc4_HW_2026-11-19/README.md)).
>   - Motion Timing P0 CLOSED e congelato ([ADR-008](09_Logs/Architecture_Decisions/ADR-008_Motion_Timing_P0_Closure_and_Freeze.md)).
> - **Stable storica:** `v1.0.1`, immutabile.
> - **Silenzi DALY:** causa UNKNOWN, RCA congelata.
> - **Prossima milestone:** Ground Locomotion & Gait Performance ([OPEN_ITEMS](09_Logs/OPEN_ITEMS.md)).
> - **Sessioni hardware** ([ADR-007](09_Logs/Architecture_Decisions/ADR-007_Motion_Timing_Integration_and_Heap_Margin.md)):
>   - al massimo 2 client HTTP;
>   - controllo di memoria M0 prima di Motion;
>   - processi solo dal registro (`session_proc.py`), mai uccisi per pattern;
>   - criteri fissati prima della prova e mai rilassati dopo.
> - **Regole:** `main`, tag, binario verificato e firmware installato sono identità distinte. Nessun flash, merge o riscrittura di calibrazione senza mandato.
> - **Storico:** gli stati del 10, 16, 17 e 18/11 sono nella [cronologia](09_Logs/HISTORY_INDEX.md) e in [OPEN_ITEMS](09_Logs/OPEN_ITEMS.md).
> - **Gemello MuJoCo V1:** offline, in `06_Software/Matdog_MuJoCo`. Non è una qualifica hardware.


Punto d'ingresso: [`README.md`](README.md). Storia fino al 26/10/2026: diario V2 in [`09_Logs/Firmware_History/`](09_Logs/Firmware_History/README.md). Architettura: [`ARCHITECTURE.md`](01_Docs/02_Architecture/ARCHITECTURE.md). Sicurezza flash: [`FLASH_LAYOUT_SAFETY_NOTICE.md`](05_Firmware/MATDOG_Controller/FLASH_LAYOUT_SAFETY_NOTICE.md).

## Non confondere queste identità
`main` (sorgenti), il tag `archive/rc5-source-checkpoint-20261026` (checkpoint storico del 26/10; le successive RCA dev.15/16 e dev.17 sono nella storia di `main`), il **commit sorgente validato**, una **APP compilata** (identificata da manifest + SHA256) e il **firmware installato sul robot** sono cose diverse. Un commit non riproduce bit a bit una APP (la build incorpora SHA Git, flag dirty, build ID e ora). Non dedurre l'APP installata da `main`, da un branch o da un nome file: richiede una verifica del dispositivo.

## Vincoli di dominio
- L'**ESP32‑S3** è l'unica autorità su Motion, ServoBus e sicurezza runtime; browser/host inviano solo intenti semantici.
- Preserva calibrazioni (gen‑1), NVS, geometria/URDF, mappatura dei giunti e polarità, STOP/SAFE_OFF e limiti fisici ed elettrici. Non indebolirli per far passare un test o un obiettivo di prestazione.

## Come lavorare
- Patch minime, verifiche proporzionate al rischio; leggi prima la documentazione e il codice esistenti.
- Distingui sempre: prova hardware (HW‑LOG/HW‑REPORT), **VERIFIED OFFLINE**, IMPLEMENTED, UNKNOWN. Nessun risultato offline va presentato come hardware.
- Nessun flash, reset distruttivo, scrittura NVS/EEPROM o merge firmware senza richiesta esplicita. Non pubblicare credenziali, `*.local.h`, identità TLS, dump o backup.
