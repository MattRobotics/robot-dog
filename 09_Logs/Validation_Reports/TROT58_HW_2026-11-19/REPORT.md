# MATDOG `1.0.3-dev.2` (TROT58): flash APP-only e qualifica sulla fixture — 19/11/2026 sera

**Esito: KEEP.** Criteri C1–C10 tutti PASS — **VERIFIED HARDWARE sulla fixture**, zampe libere, operatore presente.
- **Criteri:** fissati prima della prova nella procedura del kit.
- **Rollback a `1314bc4`:** pronto (gate DRY PASS), non usato.

**Perimetro:**
- solo fixture;
- nessuna prova a terra nella sessione;
- nessun altro firmware;
- nessuna scrittura di NVS, EEPROM, calibrazioni o limiti.

**Non qualificati:**
- la stabilità a terra e la velocità reale di qualunque modalità (NOT VALIDATED);
- TROT58 indietro, laterale e in rotazione, perché il firmware lo pubblica solo in avanti.

**Prova a terra successiva.** Dopo la chiusura della sessione l'operatore ha fatto una prova esplorativa a terra. Non fa parte di questo rapporto né di C1–C10; il suo trace è conservato in locale e sarà analizzato a parte.

## 1. Identità

| Oggetto | Valore |
|---|---|
| Firmware installato | `MATDOG Controller 1.0.3-dev.2`, build `3152e880c3cc`, `ROBOT_POWERED`, `SOURCE_STATE=CLEAN`, `OTA_INGEST=0`, `BUILD_UTC 2026-11-19T19:21:37Z` (= ora del commit) |
| Commit sorgente | [`3152e880c3cc2453ba1513a652b730670e0d0dd5`](https://github.com/MattRobotics/robot-dog/commit/3152e880c3cc2453ba1513a652b730670e0d0dd5), un commit sopra `7c7c4e2`, in `main` con lo SHA originale |
| APP | 1 784 928 B, SHA256 **`3c8774a9f9d0facfc0cb65af67ad4745c10bd9b2a2a9e906cbb8b0d9802f8c68`**. Ricostruita alla stessa radice: identica byte per byte |
| Manifest di build | SHA256 `bb9843b5bc9f490e0d10734fb6845cb61799573f7461a3b9b47f4c1b9649fd26` |
| Invariati rispetto a `1314bc4` | Bootloader, tabella delle partizioni (`8f756ecb719c4894b9c23c26bcc171e1d01ae8cda69882950944d5ce264946e7`), `sdkconfig`. Scritta solo app0 (`0x010000..0x1c3c60`) |
| Memoria | `.iram0.text` e `.dram0.data` invariati; `.dram0.bss` +48 B; `.flash.text` +4 228 B; `.flash.rodata` +2 752 B |
| NVS e calibrazione | Gen 1 `VALID_ACKNOWLEDGED`, fingerprint `be8d5902bb4e37da`, restore 12/12, invariate prima e dopo il flash |
| Rollback | Stable [`v1.0.2`](https://github.com/MattRobotics/robot-dog/releases/tag/v1.0.2) (`1.0.2-dev.7`, `1314bc4`, APP `dd16bd6215d836f8dae6389df7b13b8fcdcff20bdec06ca9948a34ae9b0dd82a`) |

**Contenuto del commit:** TROT come quarta modalità WALK sullo stesso generatore `Gait` e sulla stessa pipeline Motion.
- **Legge TROT58:** duty 0,60; 39 / 49 / 58 mm ogni 0,9 s (43,3 / 54,4 / 64,4 mm/s nominali); quota 115 mm; alzata 8,5 mm; drive {1950,220}; inviluppo IMU proprio; solo avanti.
- **Crawl** SLOW, STANDARD e FAST: invariati.
- **Non toccati:** STOP, StopPlan, box, protezioni, NVS, calibrazione, mapping, geometria e limiti.
- **Prima della prova:** verifiche **VERIFIED OFFLINE** (suite host, gate, QEMU, gemello) nel messaggio del commit e nel [CHANGELOG](../../../05_Firmware/MATDOG_Controller/CHANGELOG.md).
- **WALK FAST 40:** studiato e non integrato.

## 2. Sequenza (UTC, 19/11/2026)

| Passo | Ora | Esito |
|---|---|---|
| S0 | 20:49 | Robot spento; porta assente; nessun processo di sessione; APP e manifest come le pin; gate DRY del flash e del rollback PASS |
| S1 | 20:49:55 | Accensione; registratore USB aperto 0,7 s dopo, in sola lettura; boot completo |
| Finestra QON | → 20:57 | Gate DALY PASS (212 letture, 0 errori); `MOTION_ACC_CEILING verified=220` |
| S2 | 20:57 | Identità installata letta sul dispositivo (USB e HTTP): `1314bc411c58`, `1.0.2-dev.7`. Preflight PASS |
| Autorizzazione | — | Autorizzazione esplicita dell'operatore al flash APP-only di `3152e88` |
| S3 | 21:00 | Ingresso in ROM. `verify-flash` dell'installato: esattamente `1314bc4`. Scrittura di app0 con hash verificato, poi `verify-flash` indipendente: digest corrispondente |
| S4 | 21:01:36 | Boot registrato: `3152e880c3cc`, `1.0.3-dev.2`, `GIT_DIRTY=NO`; ACC 220; NVS e calibrazione come sopra; coppie OFF; nessun panic. Gate DALY PASS (214 letture, 0 errori) |
| S5 | 21:09 | Preflight PASS: 0 release perse e 0 overrun dal boot. `capabilities` come attese |
| S6 | 21:09:33–21:24:49 | Catena A/B della `v1.0.2`: 8/8 prove, 140/140 passi PASS |
| S7 | 21:25:47–21:28:50 | Blocco T TROT58: 37/37 PASS |
| S8 | 21:30:25–21:34:01 | Dashboard D1 (unico client HTTP): joystick, WALK TROT58, STOP HTTP: PASS |
| S9 | 21:34:35 | PRONE_REST, SAFE_OFF 8191, coppie OFF, nessun fault; stream BMS OFF; nessun processo di sessione aperto |

## 3. Criteri C1–C10

| # | Criterio | Esito | Evidenza |
|---|---|---|---|
| C1 | Flash e boot | **PASS** | Digest corrispondente (scrittura e verifica indipendente); nessun panic; ACC 220; NVS e calibrazione invariate; nessuna scrittura NVS |
| C2 | Identità e tabella | **PASS** | `3152e880c3cc`, `1.0.3-dev.2`. `walk_modes` 4 righe. TROT: 0,9 s, 39/49/58 mm, 64,4 mm/s, 115 mm, `lift_mm` 8.5, 1950/220, `hardware_qualified=false`, joystick indietro, laterale e imbardata 0. SLOW, STANDARD e FAST con valori identici a `1314bc4` |
| C3 | Catena A/B | **PASS** | 8/8 prove, ogni passo PASS, 0 abort, STOP verificato in ogni prova |
| C4 | Timing P0 (catena e blocco T) | **PASS** | Massimo 11 629 µs (catena) e 11 113 µs (blocco T), budget 16 000; 0 release ≥ 15 ms; 0 overrun; 0 release operative perse, prima STAND compresa (9 053 µs). A parte, come nella `v1.0.2`: 1 release IDLE saltata a riposo, nel passo A1 di `ab_A`, subito dopo un dump TIMING, a coppie OFF |
| C5 | Blocco T | **PASS** | 37/37 (M0 + T0..T35) |
| C6 | Legge TROT58 | **PASS** | T4 `active=TROT`, 0,05444 m/s; T8 0,06444; T16 TROT dopo la ripresa; T25 0,06444 con STANDARD selezionato |
| C7 | Indipendenza delle andature | **PASS** | T23 e T29 `WALK_MODE_NEXT_WALK`; T25 legge TROT, T31 legge STANDARD; il WALK successivo prende la scelta (T27) |
| C8 | STOP in TROT | **PASS** | T10 2,2 s e T14 2,4 s, verificati a coppia ON, ripresa ammessa; T11 STAND 120 in 4,8 s; T15 WALK ripreso in TROT |
| C9 | Salute | **PASS** | Nessun FAULT; BMS fresco a ogni passo; temperatura massima 60 °C; tensione minima 10,9 V; giunti freschi |
| C10 | Memoria | **PASS** | `heap_min_free` 39 772 B (soglia 24 576) |

## 4. Misure

**Catena contro `1314bc4`:** stessa catena `0623` del 19/11 mattina, stessi profili HTTP ([confronto](analysis/cand_vs_ref_2109.txt)).

| Prova | Massimo µs `3152e88` | Massimo µs `1314bc4` |
|---|---:|---:|
| settle A / B | 11 147 / 11 067 | 10 928 / 9 936 |
| blocchi A+B, B / A | 10 028 / 11 629 | 10 822 / 10 548 |
| blocchi E+F+G, A / B | 10 961 / 10 264 | 11 012 / 10 979 |
| blocco H, B / A | 10 644 / 11 419 | 9 883 / 10 829 |

Release 10–15 ms: 50 (profilo A) e 9 (profilo B). Release ≥ 15 ms: 0 in entrambi.

**STOP:**
- **USB in movimento:** 14/14 verificati in 2,0–3,0 s, sempre con ripresa ammessa. Sono 12 nella catena (B4, F4, H5, H7, H12, H14) e 2 in TROT58 (T10, T14).
- **USB a riposo:** 2/2.
- **HTTP dalla dashboard:** 1/1. Dopo circa 10 s di WALK TROT58 avviato dal pulsante, lo STOP ha dato `HTTP 200 · ACCEPTED`, poi `STOP · COMPLETED · STOPPED_SUPPORT_UNVERIFIED` (arresto verificato a coppia ON). Ripresa STAND 120 fino a STAND_REACHED, poi PRONE fino a PRONE_REST con coppie OFF sui 13 servo.

**Dashboard D1** ([contatori](analysis/d1_counters.txt)):
- **Sequenza:** STAND 120 · selettore TROT58 (`WALK_MODE_SELECTED`) · joystick avanti a fondo corsa tenuto dall'operatore per circa 16 s (64,4 mm/s), poi rilascio con COMPLETED · STAND_REACHED · WALK TROT58 e STOP HTTP · STAND · PRONE · selettore WALK (SLOW).
- **Fra i due bordi a riposo:** 10 789 release, 0 perse, 0 overrun, massimo ≤ 11 629 µs, heap minimo invariato.

**Correzione IMU in TROT58:** sempre `SUPPORT_FIT`, applicata per intero.

| Passo | Rollio / beccheggio (mrad) |
|---|---|
| T5 | 23,3 / −42,2 |
| T9 | 24,7 / −34,6 |
| T13 | 15,2 / −35,7 |
| T17 | 22,5 / −48,2 |
| T19 | 25,5 / −32,9 |

Il limite in beccheggio verso il basso è −127,5 mrad. In ripresa (T11) la correzione è `SLEW`.

**Fine sessione:** 98 514 release dal boot, 1 persa, quella IDLE di A1.

## 5. Osservazioni

- **Heap minimo:** 39 772 B contro 44 592 della sessione di `1314bc4`. È già al minimo nel primo campione della catena, con i due client connessi; poi non scende più. L'heap statico è cresciuto di soli 48 B. È variabilità fra sessioni, sopra soglia.
- **Temperature:** picco di 60 °C (blocco H, profilo A), sotto i 65 °C del criterio. I picchi di un solo campione sono già noti sulla Stable.
- **Dashboard:** dal selettore TROT58, cliccare il preset WALK già conservato non riporta l'andatura a WALK, perché non cambia nulla; il selettore «WALK» lo fa. È solo usabilità.
- **Strumenti:** l'intestazione delle tabelle A/B generate cita ancora la build `30c894a0d430`. È testo fisso dello strumento; i dati sono di questa sessione.
- **Baseline a riposo prima del flash:** `1314bc4` aveva 5 release IDLE perse su 21 839; il candidato, dopo il boot, 0 su 21 712.

## 6. Evidenze e archivi

**Originali integrali**, solo locali, sulla workstation dell'operatore: log seriali, registrazioni HTTP, preflight, flashlog, catena, blocco T, analisi.
- **Cartella:** `~/MATDOG/verification-artifacts/MATDOG_1_0_3_TROT58_HW_20261119/`.
- **Sigillo:** `SHA256SUMS.txt` `ee1eea59696b87642756cb1f540913de8abd831fff6a88b2cd7d7d00d52ba243`, 518 file, verificato alla chiusura. Copia: [`SHA256SUMS.local.txt`](SHA256SUMS.local.txt).
- **Non pubblicati:** log grezzi, credenziali, dump NVS, backup privati.

| Archivio locale verificato | Percorso | SHA256 |
|---|---|---|
| Seconda copia della sessione: ripristino di prova identico byte per byte, sigillo interno OK, nessuna credenziale | `~/MATDOG/backups/verification-artifacts/session-20261119/MATDOG_1_0_3_TROT58_HW_20261119.tar.gz` (1 681 121 B) | `ca6e59aaa3301444868307521b25d405e7445a09e80de0e92ab77a17c5358edd` |
| Immagine installata (bin, ELF, map, manifest, sdkconfig) | `~/MATDOG/backups/esp32/app_images/3152e88/` | `SHA256SUMS` `0f0798782508cee2ad041a0a2b4d4f7cb073a85ef3adbaaec2b9cc888c3c3eac` |
| Bundle Git del commit sorgente (richiede `7c7c4e2`) | `~/MATDOG/backups/git/robot-dog-trot58-3152e88-20261119.bundle` | `8d28616a730341f99300d510fc242a9c8e2b6b74bdf04121313867b9f50acdd2` |
| Kit della prova (procedura e criteri, script di flash e rollback, strumenti) | `~/MATDOG/verification-artifacts/MATDOG_MOTION_VNEXT_INTEGRATED_20261119/package/` | sigillo `c0b0878a6cb4485678f3902718d37b64d2f6c16a220e26881ee733c7274ef906` |

**Copie qui.** I file [`analysis/cand_vs_ref_2109.txt`](analysis/cand_vs_ref_2109.txt) e [`analysis/d1_counters.txt`](analysis/d1_counters.txt) sono copie identiche degli originali sigillati: i loro hash compaiono in `SHA256SUMS.local.txt`.

**Stesso disco.** Originali e seconde copie stanno sullo stesso disco fisico: non è dichiarata una copia indipendente.

## 7. Che cosa resta

| Voce | Stato |
|---|---|
| TROT58 e crawl a terra | **NOT VALIDATED.** Serve la qualifica a terra, supervisionata, con anticaduta e progressione prudente; prima il COM reale e l'offset di montaggio dell'IMU |
| Prova esplorativa a terra del 19/11 sera | Fuori qualifica; analisi separata ([OPEN_ITEMS](../../OPEN_ITEMS.md)) |
| TROT58 indietro, laterale e in rotazione | Non implementato in questa build |
| Dashboard D2 (dashboard più watcher) | Non eseguita: non fa parte di C1–C10 |
| Release | Nessuna: `1.0.3-dev.2` è una build di sviluppo. La Stable pubblicata resta `v1.0.2` |
