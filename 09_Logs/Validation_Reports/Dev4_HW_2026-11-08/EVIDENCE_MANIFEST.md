# dev.4 — manifest delle evidenze hardware (08/11/2026)

Tutti gli SHA256 sono calcolati sui **byte originali**. Report: [REPORT.md](REPORT.md).

## A. Prova a terra — pacchetto originale dell'operatore (versione senza video)

[`raw/MATDOG_DEV4_GROUND_VALIDATION_2026-11-08_NO_VIDEO.zip`](raw/MATDOG_DEV4_GROUND_VALIDATION_2026-11-08_NO_VIDEO.zip): **416.169 byte**, SHA256 **`ec7c55402d767dd9cde1b36bfbcf66eb8faa46a2cf0579d9c099bb966ad1d720`**, pubblicato byte-identico. `unzip -t` senza errori, 9 elementi; `MANIFEST_SHA256.txt` interno **5/5 OK**. Origine dichiarata dal README del pacchetto: allegati inviati dall'operatore in una conversazione ChatGPT, non prelevati dall'ASUS. Estratto sull'ASUS in `~/MATDOG/verification-artifacts/MATDOG_1_0_2_DEV4_GROUND_20261108/`.

| File nell'archivio | Byte | SHA256 |
|---|---:|---|
| `evidence/01_initial_entry_fault/matdog-motion-trace.json` (boot 1, 30 campioni in FAULT) | 293.680 | `db204421f457d556cc2b13e2870e19daa82e023cd05836d27d1a5b8aa4f3edda` |
| `evidence/01_initial_entry_fault/1000083725.jpg` (pubblicato anche in [`screenshots/`](screenshots/1000083725.jpg)) | 117.961 | `b3e7086328a176d48087c5ef1276b6b98df686a3d3b7548c9ba6c94853a232b8` |
| `evidence/02_ground_walk100/matdog-motion-trace (1).json` (STAND150, WALK100, WALK STOP) | 1.962.620 | `3a810799586215beb7e2dcb175a7804aec581b504df2c10557b2c8f2622f8cd0` |
| `evidence/03_ground_joystick_and_recovery/matdog-motion-trace (2).json` | 2.564.371 | `746377ba385f2d6993da04fa908ec178165a1471cd2a5b5de32e84f0923603b9` |
| `evidence/03_ground_joystick_and_recovery/matdog-motion-trace (3).json` | 2.514.607 | `812d15df99446fd6376bc6605e9a601ad0ca2a255f157605245f45bd4bfa5fae` |
| `README.md` / `INDEX.json` / `MANIFEST_SHA256.txt` | 4.412 / 3.347 / 630 | `6bcc472e…1fc6` / `654b53f0…909b` / `6d1d144d…936f` |
| `OPERATOR_NOTES.md` (pubblicato in chiaro come [OPERATOR_OBSERVATIONS.md](OPERATOR_OBSERVATIONS.md), identico) | 1.784 | `f8dc8f253a1357c0aa8b6c61c772894a915e0862707b986761064e34f4bd4d7f` |

**Video:** l'originale `1000083730.mp4` non è sull'ASUS e non ha un hash nel pacchetto. Il 09/11 è arrivata una copia ricompressa via WhatsApp, `VID-20261109-WA0000.mp4`: 6.128.555 B, SHA256 `1c75f27b7410824748b6dad3fd9903a139f0d50896a15ee988ff8551525d86ee`, 478×850, 24,94 s. È conservata solo sull'ASUS, non pubblicata (interno di un'abitazione); nessun URL pubblico.

Controllo privacy: JSON senza password, SSID, token, cookie, CSRF o email (`admin_session` è un booleano). Lo screenshot JPEG non contiene EXIF (solo JFIF e profilo ICC) e mostra la dashboard, la barra di stato del telefono e l'indirizzo predefinito dell'AP del robot `192.168.4.1` (rete locale non instradabile).

## B. Fixture — H0 e H1-RCA — originali ASUS

Cartella originale: `~/MATDOG/verification-artifacts/MATDOG_1_0_2_DEV4_H1RCA_PREP_20261108/` (`EVIDENCE_H0.sha256` e `EVIDENCE_H1RCA_RETEST.sha256` OK al momento della pubblicazione). Copia byte-identica dei file senza identificativi del dispositivo o dell'host: [`raw/MATDOG_DEV4_FIXTURE_H0_H1RCA_2026-11-08.zip`](raw/MATDOG_DEV4_FIXTURE_H0_H1RCA_2026-11-08.zip), **262.492 byte**, SHA256 **`774a374582d19d9c8b082122aa37533c21a92b58de65737a6dade1cac5b47a14`**.

| File (nello ZIP) | Byte | SHA256 |
|---|---:|---|
| `h1rca/dashboard-trace-41.json` (sessione 1) | 2.518.160 | `8bf3ac8ea3cbd13a533bb77d39a1279a834abae1b57bea8be9d7b1790a9a4b4f` |
| `h1rca/dashboard-trace-42.json` (sessione 2, tre srotolamenti fuori fascia) | 2.519.545 | `977deb5dac1abc59cc03e662e595fb00bb292d272987c56666b9662d30ba4b00` |
| `h1rca/extracted/final_diag_timing.txt` (`@MOTION DIAG`/`TIMING`, sessione 1) | 9.808 | `2c733904393a49708147ea15670852db772f60ebe7d062ededc712086d0311de` |
| `h1rca/extracted/final_diag_timing_t42.txt` (`@MOTION DIAG`/`TIMING`, sessione 2) | 20.162 | `b46b395e49c0d1a85ad76d9ded80d9b645b7cfcc303fba2739553297c0402524` |
| `H1RCA_RETEST_REPORT.md` | 3.904 | `6c98e860f431434ba5113be624154ef7de059c8f8380df6fd3d1e069f499b87b` |
| `EVIDENCE_H1RCA_RETEST.sha256` | 835 | `cb6ca3c71f2aaa4b674b807a4e08063776cb33c3911ae7f2321f0cb78e44c969` |
| `flash/boot-status.txt` (primo `/status` dev.4) | 1.892 | `5a0c818359813228dc6e237ae5385533df7b67d8831c8d8e4a9f6d74a6b30e6e` |
| `flash/device-layout.log` (layout in sola lettura) | 409 | `4725f00c42a0d874547b740416a0afb381e444c946bbc149712aeba35f207c2f` |
| `h0/preflight.json` (stato dopo il boot) | 9.261 | `2029e82774796642fed031f3c1380f5b662ad21528d12fd523e03a942ee2f04e` |

**Conservati solo sull'ASUS** (contengono il MAC del dispositivo, percorsi locali o il log USB completo):

| File | Byte | SHA256 |
|---|---:|---|
| `H0_REPORT.md` | — | `250a5d20a5cf8ff91c40d485ee59522cf6074bb3dca4803192652122e05d8e1e` |
| `EVIDENCE_H0.sha256` (21 file di flash/boot/preflight) | — | `32b4765546f73251205b1ee5d143d2833b71c78c60561c84abe0458b294db228` |
| `h1rca/serial.log` (registro USB no-reset completo) | 1.522.368 | `ef5c7a412c1ad36e112ce2cb3c43875422448f9931271a5b8ab4794d929cffe2` |
| `h1rca/http.jsonl` (watcher HTTP 1 Hz) | 7.761.223 | `88f58335f944d51797e3a18a36a04f2d4d7f10b3bbeae63716c3a6c53a710686` |
| `h1rca/events.jsonl` / `h1rca/watch_events.jsonl` | 1.330 / 248 | `c3e90caf…4464` / `21a65c8a…3eed` |

## C. Derivati pubblicati in chiaro

| File | SHA256 | Origine |
|---|---|---|
| [TRACE_METRICS.csv](TRACE_METRICS.csv) | `e81774ba4a187147f5424600c7c7018b32f5e9d6bf5a36d6140cdfc3adb329f1` | `python3 -I tools/ground_metrics.py <evidence> --fk <fk_feet>` sulle tre tracce del boot 2 |
| [COMMANDS_RETAINED.csv](COMMANDS_RETAINED.csv) | — | comandi deduplicati per sequenza; due righe del boot 1 ricostruite da screenshot e `active_command` |
| [`tools/ground_metrics.py`](tools/ground_metrics.py), [`tools/fk_feet.cpp`](tools/fk_feet.cpp) | — | `fk_feet` si compila con `g++ -std=c++17 -O2 -I src tools/fk_feet.cpp src/motion/{FootContact,LegKinematics,LegInverseKinematics}.cpp` dalla cartella del firmware |
