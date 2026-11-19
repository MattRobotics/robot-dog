# MATDOG v1.0.2 — manifest firmware e release

**La release GitHub `v1.0.2` promuove a Stable l'esatto binario `1.0.2-dev.7` / `1314bc411c58` installato e collaudato sul MATDOG il 19/11/2026.**
- Il binario non è stato ricompilato per la release: la stringa interna resta `1.0.2-dev.7`.
- Il tag `v1.0.2` identifica la promozione, non una versione interna diversa.
- La Stable precedente `v1.0.1` resta immutabile.

## 1. Catena di identità

| # | Elemento | Valore |
|---|---|---|
| 1 | Firmware realmente installato | `SOURCE_SIGNATURE build_id=1314bc411c58 version=1.0.2-dev.7 profile=ROBOT_POWERED`, `GIT_DIRTY=NO`, `ota_running_build_id=1314bc411c58`. Letto via USB dopo il boot del 19/11 (06:08Z) e di nuovo alle 07:24Z e 07:33Z |
| 2 | Commit sorgente dell'APP | **`1314bc411c589c4216e95629f4665d9d01f05781`**. Pubblicato prima sul branch `cand/ksteer-slow-recert-20261118`, eliminato dal remoto il 19/11 dopo l'inclusione in `main` ([inventario](../../EXTERNAL_ARCHIVES.md#9-remote-branches-deleted-on-2026-11-19)). Discende da `main` `6838e4d` senza divergenze; la linea è nel §5 |
| 3 | Build ID | `1314bc411c58` |
| 4 | APP SHA256 | **`dd16bd6215d836f8dae6389df7b13b8fcdcff20bdec06ca9948a34ae9b0dd82a`** |
| 5 | Dimensione e profilo | 1 777 952 B. Manifest di build originale: `ROBOT_POWERED`, `SOURCE_STATE=CLEAN`, `GIT_DIRTY=0`, `OTA_INGEST_ENABLED=0`, `MOTION_STACK=MATDOG_SIMPLE_V1`, `MOTION_AUTHORITY=RUNTIME_GOVERNED`, `BOOT_MOTION_AUTHORIZED=0`, `PRIVATE_TLS_CONFIGURATION_SHA256=ABSENT`, `BUILD_UTC=2026-11-19T05:27:14Z`; FQBN `esp32:esp32:esp32s3:…PartitionScheme=custom…PSRAM=opi` |
| 6 | Slot e offset | `app0` @ `0x010000`, layout `MATDOG_16M_2x5M_NVS_V1`. Tabella `8f756ecb719c4894b9c23c26bcc171e1d01ae8cda69882950944d5ce264946e7` e sdkconfig `534f3457…` identici a `30c894a`; `CAL_RECORD_SCHEMA=1`, `CAL_MARKER_SCHEMA=2` |
| 7 | Flash e verifica | 19/11/2026, ingresso ROM alle 06:07:24Z, una sola scrittura di app0 (cancellazione `0x010000–0x1c2fff`). Esito: `Hash of data verified`, `Verification successful (digest matched)`, `APPLICATION_ONLY_FLASH = PASS`. Prima del flash, `rom_checks.sh` aveva verificato che l'installato fosse `30c894a` con digest corrispondente |
| 8 | Boot e calibrazione | NVS gen 1 `VALID_ACKNOWLEDGED`. Calibrazione: `CALIBRATION_RESTORE_DECISION … generation=1 fingerprint=be8d5902bb4e37da restored=1 mapped=12/12 torque=UNCHANGED_OFF`. Inoltre census 13/13, `MOTION_ACC_CEILING verified=220`, SAFE_OFF 8191; nessun panic né `Coprocessor exception` |
| 9 | Prove hardware | Catena A/B 8/8 (140 passi), dashboard D1/D2, blocco D 27/27, S9 15/15 ([README](README.md)) |
| 10 | Commit su `main` | Merge della PR di consolidamento del 19/11. Lo SHA esatto è nel manifest allegato alla Release (`MATDOG_v1.0.2_RELEASE_MANIFEST.txt`). Il merge contiene `1314bc4` nella sua storia e non cambia alcun file eseguibile del firmware. In `05_Firmware/` cambiano solo file di documentazione: `README.md`, `MATDOG_Controller/README.md` e `MATDOG_Controller/CHANGELOG.md`. L'albero `MATDOG_Controller/src/`, lo sketch e gli script sono identici a `1314bc4` |
| 11 | Tag GitHub | `v1.0.2`, annotato, **sul commit firmware `1314bc4`**, non sul merge né su un commit di sola documentazione. Questo differisce da `v1.0.1`, il cui tag puntava al merge |
| 12 | Asset della Release | §2 |
| 13 | Rollback | §4 |

**Il binario pubblicato contiene:**
- tre percorsi della toolchain Arduino-ESP32 dell'host di build (stringhe `__FILE__` del core), come ogni build di questa macchina;
- nessuna credenziale: password dell'AP, token amministratore e SSID dell'host sono assenti (verificato il 19/11 senza stamparli).

## 2. Asset della Release `v1.0.2`

| Asset | Contenuto |
|---|---|
| `MATDOG_v1.0.2_app0_ROBOT_POWERED_fw-1.0.2-dev.7_1314bc411c58.bin` | APP originale, byte-identica a quella installata (`dd16bd62…`, 1 777 952 B) |
| `MATDOG_v1.0.2_build_manifest_1314bc411c58.txt` | `matdog_build_manifest.txt` originale della build, byte-identico (`be466734…`) |
| `MATDOG_v1.0.2_RELEASE_MANIFEST.txt` | Questa catena di identità, con lo SHA di `main` dopo il merge e il bundle Git |
| `SHA256SUMS.txt` | SHA256 di tutti gli asset |

**Origine dell'APP:**
- backup locale `~/MATDOG/backups/esp32/app_images/1314bc4/` (sigillo `SHA256SUMS` `312fe371…`);
- identica alla build `~/MATDOG/build-output/motion-integrated-20261119/1314bc4/artifacts/ROBOT_POWERED-ota0/` e all'immagine scritta e verificata il 19/11.

## 3. Installazione APP-only (solo questo robot)

**Prerequisiti:** robot sostenuto, coppie OFF, SAFE_OFF 8191, BMS regolare, USB-C diretto all'ESP32-S3.

1. Verificare che `sha256sum` dell'asset `.bin` dia `dd16bd62…82a`.
2. Entrare in ROM con un solo reset e leggere il MAC: `14:c1:9f:22:75:94`.
3. Leggere tabella e otadata **in sola lettura**: tabella `8f756ecb…`, slot attivo `app0` @ `0x010000`. Se lo slot attivo non è `app0`, fermarsi.
4. Una sola scrittura dell'APP in `0x010000` (fino a `0x1c2120`), poi `verify-flash`. **Mai** bootloader, tabella, otadata, NVS (`0xfe0000–0xff0000`), EEPROM servo o `app1`.
5. Boot, poi verifica:
   - identità: `1.0.2-dev.7`, `1314bc411c58`;
   - READY;
   - census 13/13, NVS gen 1, calibrazione `be8d5902bb4e37da` 12/12;
   - `MOTION_ACC_CEILING verified=220`, SAFE_OFF 8191.

**Script qualificato:** `flash-qualified-app0-1.0.2-dev.7-1314bc4.sh` (`d24920b0…`), con il gate offline `…_DRY_GATES_OFFLINE.sh` (`9b6fa15e…`). Sono conservati sull'Asus nel pacchetto `MATDOG_1_0_2_MOTION_INTEGRATED_1314BC4_20261119/preflash/`.

**Procedura completa:** [HW_PROCEDURE.md](../Motion_Integrated_1314bc4_2026-11-19/HW_PROCEDURE.md).

## 4. Rollback

**Immagine di rollback: `30c894a`**
- `1.0.2-dev.7`, build `30c894a0d430`, sorgente `30c894a0d4303d497d750cccb5fd12f347c61d7d` (nella storia di `main` dopo il merge);
- APP 1 779 312 B, SHA256 `03c863cc433622f672c11cae45f46694c4aa6f78ba0136cc1b7bc818a2e4b340`.

**Dove si trova:**
- backup `~/MATDOG/backups/esp32/app_images/30c894a/` (sigillo `8a1aa6d3…`);
- script qualificato `flash-qualified-app0-1.0.2-dev.7-30c894a.sh` (`812a2e83…`), in `MATDOG_1_0_2_ACQV2_CLEAN_20261118/preflash/`.

**Qualifica:**
- VERIFIED HARDWARE il 18/11 ([rapporto](../Motion_30c894a_HW_2026-11-18/README.md)): catena 140/140, P0 allora aperto;
- riscritta da Fable sul robot il 18/11 sera con `digest matched`;
- il gate offline del rollback è PASS dal 19/11.

**Compatibilità:** stessa tabella, stesso layout, stesso sdkconfig e stessi schemi NVS di calibrazione. È uno scambio della sola APP: NVS e calibrazione gen 1 restano intatte.

**Ripristino catastrofico:** il backup integrale cifrato della flash ([10_Backups/ESP32](../../../10_Backups/ESP32/README.md)) contiene anche la NVS della sua data. Serve solo per il recupero, mai come rollback ordinario: sovrascriverebbe la calibrazione.

## 5. Linea dei commit firmware da `main` (`6838e4d`, 10/11) a `1314bc4`

| Commit | Contenuto | Stato hardware |
|---|---|---|
| `06b6db0` e precedenti | Branch operativo unificato del 14–17/11: T1/T2, T2R1M, E1/E1b, P3a/P3b, P5, F3 (FAST 29,6 mm/s), IRAM G1/G2s, Patch A, riarmo STOP | Leggi Motion qualificate tramite le build diagnostiche del 15–16/11 |
| `85bad0e` | PlanSplit V2, STOP prima del primo frame (ancora corretta), writer HTTP senza printf, `validDefinition` | 18/11 mattina su fixture: catena 8/8, P0 aperto |
| `30c894a` | ACQUIRE v2: registro 85 preparato a riposo in RUN | 18/11: catena 140/140, P0 aperto (20 761 µs, 2 overrun); è il rollback |
| `eb1ba28` | P1 minima: ricalcolo dell'alzata del raccordo 10 → 50 µm | Solo offline, poi incluso in `1314bc4` |
| `05e1214` | Nucleo cinematico FP32 (API double invariata) | Solo offline, poi incluso in `1314bc4` |
| `1314bc4` | `kSteer` SLOW ricertificato: massimo di RF UPPER 0,963456 → 1,004874 rad, certificato FCL 120/120 + 12/12 | **VERIFIED HARDWARE 19/11: Stable `v1.0.2`** |
