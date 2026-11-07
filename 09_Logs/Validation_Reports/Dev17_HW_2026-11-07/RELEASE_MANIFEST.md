# MATDOG v1.0.1 — manifest firmware e release

**Release GitHub `v1.0.1` = promozione stabile dell'esatto binario `1.0.1-dev.17` installato e collaudato sul MATDOG attuale.** Il binario non è stato ricompilato per la release: la sua stringa interna resta `1.0.1-dev.17` e il build ID `17db0efe2324`. Il tag `v1.0.1` identifica la promozione, non una versione interna diversa.

## 1. Catena di identità

| # | Elemento | Valore |
|---|---|---|
| 1 | Firmware realmente installato | `MATDOG Controller 1.0.1-dev.17`, letto via HTTP dopo il boot del 07/11/2026 (`firmware_version`, `build_id`, `ota_running_build_id`) |
| 2 | Commit sorgente dell'APP | **`17db0efe23242ae8932b5549546a9758d9aa4645`** (branch di sviluppo `matdog/dev17-stabilization`; integrato nella PR #64 tramite il merge `deb07dd`) |
| 3 | Build ID | `17db0efe2324` |
| 4 | APP SHA256 | **`14e055a0bfa2adcab8f87df29ece0a0cf01a6e9e13b99aee1c94bda59ba84a83`** |
| 5 | Dimensione / profilo | 1.707.808 byte; `ROBOT_POWERED`, `SOURCE_STATE=CLEAN`, `GIT_DIRTY=0`, `OTA_INGEST_ENABLED=0`, `MOTION_STACK=MATDOG_SIMPLE_V1`, FQBN `esp32:esp32:esp32s3:…PartitionScheme=custom…PSRAM=opi` (manifest di build originale, `BUILD_UTC=2026-11-07T18:04:17Z`) |
| 6 | Slot / offset | `app0` @ `0x010000`, layout `MATDOG_16M_2x5M_NVS_V1`, tabella `8f756ecb719c4894b9c23c26bcc171e1d01ae8cda69882950944d5ce264946e7`; bootloader e sdkconfig identici a dev.16 |
| 7 | Flash e verifica | 07/11/2026 18:32:20–18:33:09 UTC, una `write-flash` 0x010000–0x1b0f20, `Hash of data verified`, `verify-flash` `digest matched`, `APPLICATION_ONLY_FLASH = PASS` ([evidenze](EVIDENCE_MANIFEST.md#b-flash-app-only-boot-e-audit-idle--originali-asus)) |
| 8 | Boot e calibrazione | READY/RUN, census 13/13, NVS gen-1 `VALID_ACKNOWLEDGED` (ack gen 1), nessuna ricalibrazione o restore, SAFE_OFF 8191 |
| 9 | Diagnostiche hardware | audit IDLE 91 s (160/160 HTTP 200, 50,00 Hz, 0 perse/overrun) e sessione operatore `(33)`–`(36)` ([REPORT](REPORT.md)) |
| 10 | Commit su `main` | merge della PR #64; lo SHA esatto è il commit puntato dal tag `v1.0.1` ed è riportato nel manifest allegato alla Release (`MATDOG_v1.0.1_RELEASE_MANIFEST.txt`) e nelle Release Notes. È un commit di integrazione: **non** è il commit da cui è stata compilata l'APP |
| 11 | Tag GitHub | `v1.0.1` (annotato), su `main` |
| 12 | Asset della Release | vedi §2 |
| 13 | Rollback | dev.16 `1.0.1-dev.16`, build `e0ddbb1791d0`, sorgente `e0ddbb1` (su `main` dopo il merge), APP 1.694.048 B, SHA256 `73952eec1ddb29976941b7afd16149b15f790a2e8c46cb2067d1f72a8424138d`, conservata in sola lettura sull'ASUS (`~/MATDOG/build-output/dev16-stabilization-20261107/build-e0ddbb1/…`) insieme allo script qualificato `flash-qualified-app0-dev16.sh`; VERIFIED HARDWARE il 07/11/2026 ([report](../Dev16_HW_2026-11-07/REPORT.md)) |

Verifica pre-merge: tra `17db0ef` e la HEAD della PR i file eseguibili sono identici (`src/` tree `d9069c27…`, `MATDOG_Controller.ino` `12b4b1ec…`); cambiano solo `CHANGELOG.md`, `README.md`, `AGENTS.md` e `09_Logs/`. `main` era antenato della PR, quindi il merge non sovrascrive nulla. La candidata superata `7503bfc` (APP `f49b531a…`) **non** è rilasciata.

## 2. Asset della Release `v1.0.1`

| Asset | Contenuto |
|---|---|
| `MATDOG_v1.0.1_app0_ROBOT_POWERED_fw-1.0.1-dev.17_17db0efe2324.bin` | APP originale, byte-identica a quella installata (SHA256 `14e055a0…4a83`, 1.707.808 B) |
| `MATDOG_v1.0.1_build_manifest_17db0efe2324.txt` | `matdog_build_manifest.txt` originale della build, byte-identico |
| `MATDOG_v1.0.1_RELEASE_MANIFEST.txt` | questa catena di identità con lo SHA di `main`/tag |
| `SHA256SUMS.txt` | SHA256 di tutti gli asset |

L'APP è stata controllata prima della pubblicazione: nessuna credenziale Wi-Fi/OTA locale presente nei byte (`PRIVATE_TLS_CONFIGURATION_SHA256=ABSENT`).

## 3. Installazione APP-only e rollback (solo questo robot)

Prerequisiti: robot sostenuto, coppia OFF, SAFE_OFF 8191, BMS regolare, USB-C diretto all'ESP32-S3.

1. `sha256sum` dell'asset `.bin` = `14e055a0…4a83`.
2. Ingresso in ROM con un solo reset e lettura MAC (`14:c1:9f:22:75:94`).
3. Lettura **in sola lettura** di tabella e otadata con `05_Firmware/MATDOG_Controller/scripts/verify_application_partition.py`: tabella `8f756ecb…`, slot attivo `app0` @ `0x010000`. Se lo slot attivo non è `app0`, fermarsi.
4. Una sola scrittura dell'APP in `app0` (`0x010000`), poi `verify-flash` indipendente. Mai bootloader, tabella, otadata, NVS (0xfe0000–0xff0000), EEPROM servo o `app1`. Procedura completa: [FLASH_AND_TEST.md](../Dev17_Candidate_2026-11-07/FLASH_AND_TEST.md) e `scripts/flash_app_only.sh`.
5. Boot con watchdog-reset, poi solo HTTP: `firmware_version=1.0.1-dev.17`, `build_id=17db0efe2324`, READY/RUN, census 13/13, NVS gen-1 `VALID_ACKNOWLEDGED`, SAFE_OFF 8191.

**Rollback:** stessa procedura con l'APP dev.16 (`73952eec…`). Il backup completo del 10/09/2026 contiene firmware **e NVS di quella data**: solo per ripristino catastrofico, mai come rollback ordinario (sovrascriverebbe la calibrazione gen-1).
