# AGENTS.md — MATDOG

> **Baseline attuale (10/11/2026).**
> - **Firmware installato:** build di sviluppo `1.0.2-dev.6` ACC 220. Sorgente `450d2c7`, in `main` dalla PR #67; build `450d2c793dcb`; APP SHA256 `82122e06b6708a9d687f4525f612182c227134f3abcac27f32ed7e266be04256`.
> - **Validazione:** fixture Q1–Q8 qualificata; a terra verifica positiva ma fuori protocollo ([dossier HW](09_Logs/Validation_Reports/Dev6_ACC220_HW_2026-11-10/REPORT.md)).
> - **Stable storica:** [`v1.0.1`](https://github.com/MattRobotics/robot-dog/releases/tag/v1.0.1) (`1.0.1-dev.17`), immutabile.
> - **Gemello MuJoCo V1:** offline, in `06_Software/Matdog_MuJoCo` (tag `matdog-mujoco-v1-20261110`). Non è una qualifica hardware.
> - **Regole:** `main`, tag, binario verificato e firmware installato sono identità distinte. Nessun flash, merge o riscrittura di calibrazione senza mandato.
> - **Problemi residui:** [OPEN_ITEMS](09_Logs/OPEN_ITEMS.md).


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
