# MATDOG Motion Controller — notte 08→09/11/2026: candidata `1.0.2-dev.5`

Mandato autonomo offline. **Nessun flash, nessun accesso hardware, nessun merge.** Il robot resta su `1.0.2-dev.4` (`4358f13234f9`, VALIDATED HARDWARE nel perimetro di [Dev4_HW_2026-11-08](../Dev4_HW_2026-11-08/REPORT.md)), che è la baseline di tutte le analisi. Classi: **VH** verificato su hardware (solo dev.4), **VO** verificato offline, **INV** implementato non validato su hardware, **UNKNOWN**.

## 1. Stato GitHub

| | Commit | Stato |
|---|---|---|
| Inizio | `560936956b6a` (branch `feat/motion-perf-next-20261107`, PR #67 Draft, 38 commit, 51 file) | dev.4 sorgente `4358f13`, installata |
| Evidenze hardware dev.4 | `59cc22a` | pubblicate prima di qualsiasi modifica firmware |
| Fasi | `789f342` (A) · `d36ce04` (B) · `c8fc628` (stack) · `268c8d7` (sonda costi) · `3b70d4d` (E) | ognuna con regressioni |
| Identità candidata | `3afe6ce10a2e` (versione, CHANGELOG, strumento digest) | `1.0.2-dev.5`, sorgente della build |

PR [#67](https://github.com/MattRobotics/robot-dog/pull/67) resta **OPEN e DRAFT**; nessun merge, nessun force push.

## 2. Evidenze hardware pubblicate (VH, dev.4)

[Dev4_HW_2026-11-08](../Dev4_HW_2026-11-08/REPORT.md): flash APP-only + H0; fix H1-RCA sulla fixture (tre srotolamenti fuori fascia, ~1,8 s, nessun fault); prova a terra (STAND150, WALK STANDARD 100 due volte con 0 release perse, joystick, STOP e ripresa, quote, PRONE). Pacchetto dell'operatore pubblicato byte-identico (SHA256 `ec7c5540…d720`, 5/5 checksum interni OK). **Limiti:** il video WALK100 non è arrivato all'ASUS (pacchetto `NO_VIDEO`, nessun hash): non analizzato. Tracce campionate a ~0,35 s; nessun `@MOTION DIAG` della prova a terra.

## 3. Problemi corretti

### A. Posa d'ingresso rifiutata → nessun power-cycle (`789f342`) — VO

**Causa verificata (tracce + sorgente):** `fault("INITIAL_POSE_OUTSIDE_SAFE_ENTRY")` in ACQUIRE/REFERENCE, prima di qualsiasi abilitazione, impostava `faultLatched_`, che nessun percorso azzera. Dopo il riposizionamento la posa era `PRONE_ENTRY` e **identica entro 1 tick** a quella accettata dopo KEY OFF/ON.

**Correzione:** il rifiuto pre-attuazione diventa uno stato STOPPED non latched (SAFE_OFF riverificato, nessun frame, nessuna abilitazione). Torna READY **da solo** quando l'evidenza di ACQUIRE regge per un ciclo completo di letture di controllo (~1,4 s): 12 posizioni fresche in una regione certificata, ferme, senza allarmi, temperatura e tensione nei limiti, coppia OFF riletta su ogni servo. Fino ad allora STAND, WALK e joystick sono rifiutati. Lo STAND abortito non riparte mai, il successivo richiede un comando esplicito. Uno STOP in quello stato riverifica solo l'OFF. Il feedback non fresco a fine ACQUIRE resta un fault latched, ora con un motivo proprio (`SERVO_FEEDBACK_STALE`). Gli altri fault e il latch SAFE_OFF dopo READY sono invariati.

**Regressione `entryRecovery`** (posa reale del boot 1). Positivi: rifiuto, STOP, riposizionamento, READY dopo 78 release, nuovo STAND fino a STANDING. Negativi (nessun READY finché dura la condizione):
- feedback perso;
- un giunto 2 tick fuori dalla box;
- servo in movimento, anche con velocità in bit di segno;
- coppia inattesa;
- allarme di stato servo;
- 70 °C;
- 3,9 V;
- attraversamento di una regione non certificata;
- un altro fault (resta latched);
- coppia ON in ACQUIRE (resta latched).

Fallisce sul sorgente dev.4.

### B. Joystick avanti/indietro = legge WALK STANDARD; nessuno sway da jitter (`d36ce04`) — VO

**Causa verificata:** `joystickSwayYM` passava a 14 mm di sway sopra il 3 % di richiesta laterale/yaw. A terra uno stick quasi frontale portava 0,1–0,4 mm/s di laterale, cioè il 3–13 % del fondo scala. Escursione laterale media dei piedi misurata dai giunti: **28,3 mm p-p** contro **2,8–2,9 mm** in WALK STANDARD.

**Correzione:** il Controller classifica come solo avanti/indietro una richiesta con laterale+yaw sotto `kJoystickFrontalBand` (0,20 dell'ellissoide); ignora laterale e yaw e cammina con `walkGait` stesso:
- periodo 5,0 s, duty 0,80, COM shift 11 mm, swing trapezoidale, sway 1,5 mm;
- box `kHeight`, inviluppo `kWalkTilt100`, fascia nominale;
- passo continuo fino a **100 mm avanti** e **50 mm indietro**.

Una sterzata durante un passo frontale fa WALK STOP (`JOYSTICK_STEERING`, anche nell'ACK); la leva tenuta riparte sterzando da STANDING. La legge sterzata resta invariata.

**Prove offline:**
- Supporto statico della v2 positivo a 10–100 mm in entrambi i versi (COM URDF).
- Inviluppo `kWalkTilt100` con prova STOP: PASS a 10–100 mm in entrambi i versi, 187 330 controlli ciascuno.
- Cambi di passo a metà camminata: supporto −2,91 mm con rampa di 1 ciclo, quindi rampa di **4 cicli** (≥ +0,83 mm).
- Gate joystick su 43 scenari (frontali, jitter registrato, accelerazioni a fasi diverse, sequenze casuali): supporto minimo **+0,96 mm**, clearance 2,18 mm.

### C. Stack della loop task (`c8fc628`) — VO

**Causa verificata:** grafo di chiamata Xtensa (`-fcallgraph-info=su`) su tutte le 89 unità. Il percorso statico peggiore da `loop()` valeva **15 952 B su 16 384**, attraverso `HttpTransport::serviceRequest` (7 408 B). Il frame veniva da una sola istruzione: `resp = HttpTransportResponse{}` materializzava sullo stack un temporaneo di 5 376 B di testo più lo snapshot. **Correzione:** costruzione in place. `serviceRequest` scende a **416 B**; il percorso peggiore a **11 024 B**, ora attraverso la riga di comando USB.

### E. Diagnostica della prima causa (`3b70d4d`) — VO

- `imu_correction.refused_by`: il controllo di `compensationSafe` che ha rifiutato (POINT_SAFE … STOP_PLAN). Nel caso H1 avrebbe detto subito POINT_SAFE. Esempio offline: HOLD150 con 0,2 rad di rollio → `WORKSPACE refused_by STOP_PLAN`.
- `entry_violation {joint, outside_mrad}` durante il rifiuto d'ingresso: dice all'operatore quale gamba riposizionare.
- Entrambi visibili in dashboard. Snapshot invariato a 1552 B.

## 4. Prestazioni nominali (legge, non misura)

| | dev.4 | dev.5 |
|---|---|---|
| Joystick avanti | 45 mm / 6,5 s = **6,9 mm/s**, legge sterzata | 100 mm / 5,0 s = **20 mm/s**, legge WALK STANDARD |
| Joystick indietro | 6,9 mm/s | **10 mm/s** (50 mm / 5 s) |
| Laterale / rotazione | 3,08 mm/s / 1,76 °/s | **invariati** (vedi §6) |
| Sway con stick quasi frontale | 14 mm (28 mm p-p misurati) | 1,5 mm (WALK STANDARD) |
| Supporto statico minimo offline (gate joystick) | +0,57 mm | +0,96 mm |
| Passo RAW massimo per release | 13–22 tick (sterzata) | 16 tick (avanti a 20 mm/s, come WALK100) |
| Rampa di passo in camminata | 2 cicli × 6,5 s | 4 cicli × 5 s (frontale); da fermo, rampa di avvio v2 di 1 ciclo |

## 5. Budget temporale (proxy host, NON tempo ESP32-S3)

Chiamate libm per release (doppia precisione software sull'S3), sonda `motion_cost_probe`:

| Scenario | dev.4 p99 / max | dev.5 p99 / max |
|---|---|---|
| Joystick avanti con il jitter registrato | 864 / 883 (8 IK, 23 FK) | **390 / 396** (4 IK, 15 FK) |
| Joystick sterzato | 822 / 879 | 822 / 879 (invariato) |
| WALK100 | 430 / 473 | 430 / 473 (invariato) |

A terra (VH, dev.4) la WALK100 aveva 0,6–2,3 % di overrun e 0 perse; il joystick 9,5–20,9 % e 0,12–0,65 perse/s. **Atteso (non verificato):** il joystick avanti/indietro con il profilo di costo della WALK100. Il percorso sterzato resta il più costoso: è al limite geometrico e una sua ottimizzazione richiede una legge nuova (§6).

## 6. Non modificato, con evidenza

- **Laterale e rotazione:** varianti laterale 25 mm, yaw 0,25 rad, rampa sterzata di 1 ciclo, periodo 5,5 o 5,0 s. Tutte falliscono offline: raggi IMU oltre i 12° del raccordo, supporto negativo, oppure `WALK_IK_OR_WORKSPACE` dalla box o dalla prova STOP ([`evidence/steered_limits`](evidence/steered_limits/SUMMARY.txt)). Migliorarle richiede una legge sterzata nuova, la ricertificazione di `kSteer` o un inviluppo IMU ridotto.
- **Drive ST3215 `{1000,50}`:** la WALK100 usa già l'83 % della velocità (1,27 di 1,534 rad/s) e il 76 % dell'accelerazione (5,81 di 7,67 rad/s²). Le leggi sono limitate da geometria, box e prova STOP, non dal profilo. Non esistono dati di corrente, temperatura o frenata per profili più spinti. **Invariato**, nessun profilo aggressivo preparato.

## 7. Regressioni (sorgente finale)

| Regressione (sorgente finale, offline) | Esito | Nuovo o confronto con la dev.4 |
|---|---|---|
| SimpleMotion completa | PASS | nuovi `entryRecovery` (posa HW del boot 1, 11 casi negativi, diagnostica) e casi joystick frontali A–E; mappa sway, inviluppo in heading e rotta H1 aggiornati al nuovo contratto; H1 raggiunge ancora +64/−66 mrad e lo srotola ([riepilogo](evidence/simple_motion_dev5_final_summary.txt)) |
| `entryRecovery` contro la dev.4 | FAIL sulla dev.4 | riproduce il FAULT latched |
| STOP support | PASS | invariato |
| PRONE (replay dev.15) | 4/4 `PRONE_REST` | invariato |
| Dashboard e contratto HTTP | PASS: 6175 controlli, DOM 21 buste, browser layout, viewcube, STOP durante il caricamento del Twin | sequenza dinamica: 1 WALK STOP `JOYSTICK_STEERING` alla prima sterzata, poi nessuno stop in 4 cambi di direzione sterzati |
| Gate inviluppo joystick, 43 scenari | PASS: supporto minimo +0,96 mm, clearance 2,18 mm, inclinazione piede 11,44° | dev.4: 32 scenari, +0,57 mm, 2,18 mm, 11,44° ([dev.5](evidence/joystick_envelope_dev5_final.log), [dev.4](evidence/joystick_envelope_dev4_baseline.log)) |
| Supporto WALK v2, passi continui, avanti e indietro, errore COM −4/0/+3,6 mm | positivo con il COM URDF; indietro negativo a +3,6 mm | [matrice](evidence/v2_support_matrix.txt) |
| Inviluppo `kWalkTilt100` con prova STOP, 10–100 mm, avanti e indietro | 10/10 PASS (187 330 controlli ciascuno) | [matrice](evidence/v2_tilt_envelope_matrix.txt) |
| Varianti sterzate (laterale, yaw, rampa, periodo) | FAIL | motivo per cui laterale e yaw restano invariati ([riepilogo](evidence/steered_limits/SUMMARY.txt)) |
| Digest dei frame, 10 scenari | identici alla dev.4 | [dev.4](evidence/frame_digest_dev4.txt) / [dev.5](evidence/frame_digest_dev5.txt) |
| Sonda costi | joystick frontale p99 864 → 390 | [dev.4](evidence/cost_probe_dev4.txt) / [dev.5](evidence/cost_probe_dev5.txt) |
| Stack Xtensa, 89 unità | `loop()` 15.952 → 11.024 B | [dev.4](evidence/stack_worst_loop_dev4.txt) / [dev.5](evidence/stack_worst_loop_dev5_build.txt) |
| Trasporto: HTTP Motion, fence, AP-only, slack-exempt | PASS | invariati |
| Audit statico (366 file, host test inclusi) | PASS | le mutazioni USB storiche sono ancora rilevate |
| Budget del JSON di stato con i nuovi campi | 4543 B in HOLD WORKSPACE con `refused_by`; stato d'ingresso rifiutato nel budget | budget limitato 4736 B, riserva di 640 B intatta (asserito nei test) |
| Build ROBOT_POWERED CLEAN | PASS | §8 |

Digest dei frame delle leggi invariate (HOLD, WALK75/100 con STOP, joystick sterzato a regime, casuale, con jitter, WALK STOP e STOP): **identici bit a bit alla dev.4** con lo stesso strumento. Test di trasporto `ap_transport`, `ap_ingress`, `motion_transport_mode` e `motion_safety_boundaries`: falliscono **identici sulla dev.4** (preesistenti, non toccati).

## 8. Candidata

| | |
|---|---|
| Sorgente | `3afe6ce10a2e8390874cdb7bb5c5a6cb66be6044`, `FW_VERSION=1.0.2-dev.5`, build ID `3afe6ce10a2e` |
| Manifest | `SOURCE_STATE=CLEAN`, `GIT_DIRTY=0`, `HARDWARE_PROFILE=ROBOT_POWERED`, `OTA_INGEST=0`, `MOTION_AUTHORIZED=0`, `BOOT_MOTION_AUTHORIZED=0`, `BUILD_UTC=2026-11-08T23:33:08Z` |
| APP | `~/MATDOG/build-output/gait-redesign-20261108/build-3afe6ce/artifacts/ROBOT_POWERED-ota0/MATDOG_Controller.ino.bin`, **1.725.712 B** (+2.688 B rispetto alla dev.4), SHA256 **`da6d0ad6b5493f385c680c38eb6dc1868dcb549b5c822c44f1b6889fea8d4cc0`**; RAM 138.012 B (+24 B) |
| Layout | `MATDOG_16M_2x5M_NVS_V1`, tabella `8f756ecb…` invariata; scrittura solo `app0` 0x010000–0x1b5510 (cancellazione fino a 0x1b6000), NVS mai toccata; slot al 32,9 % |
| Stack | percorso statico peggiore da `loop()` **11.024 B** su 16.384 (dev.4: 15.952 B); `serviceRequest` 416 B (dev.4: 7.408 B) |
| Flash (preparato, NON eseguito) | `~/MATDOG/verification-artifacts/MATDOG_1_0_2_DEV5_FLASH_PREP_20261109/flash-qualified-app0-1.0.2-dev.5.sh` (SHA256 `f3cddaba…e7f5`), stesso schema qualificato della dev.4; senza `MATDOG_DEV5_FLASH_AUTHORIZED=YES_ROBOT_SUPPORTED` risponde REFUSE (verificato); piano di scrittura, SHA, dimensione, manifest, tabella e backup verificati offline senza invocare esptool |
| Rollback | `rollback-assets.sha256` **6/6 OK**: dev.5, dev.4 installata (`3e7ee77d…`, script `MATDOG_1_0_2_DEV4_FLASH_PREP_20261108/`), dev.17 Stable `14e055a0…`, dev.3 `cef30d8c…`, dev.16 `73952eec…`, backup integrale `5cbba0b9…` (nessun nuovo backup necessario) |
| Differenze dalla dev.4 | `SimpleMotion.{h,cpp}` (ingresso, joystick frontale, diagnostica), `MotionService.{h,cpp}` (ACK, capabilities, diagnostica), `MotionDashboard.h` (testi, limiti del joystick), `HttpTransport.cpp` (reset in place), `BuildConfig.h`; legge sterzata, box, inviluppi, geometria, calibrazione, STOP e drive invariati |

## 9. Rischi residui e collaudo minimo

**Rischi INV:**
- legge v2 all'indietro mai eseguita sull'hardware; la sua finestra di supporto su un errore del COM in avanti è più stretta;
- cambi di passo continui e transizione `JOYSTICK_STEERING` su hardware;
- dinamica reale a 20 mm/s guidata dal joystick (la WALK100 è VH a terra);
- stack e tempi su ESP32-S3 (solo analisi statica e proxy);
- il READY automatico dipende dalla lettura della velocità dei servo mossi a mano.

**Collaudo minimo** (operatore, robot supervisionato):
1. **H0** flash APP-only: identità, NVS, census, SAFE_OFF.
2. **Ingresso:** accensione col posteriore sollevato, STAND150. Atteso: `POSA D'INGRESSO NON RICONOSCIUTA`, `entry_violation` che nomina la gamba, nessun FAULT. Riposizionare: READY in ~1,5 s senza STOP. STAND150 fino a STANDING. Ripetere premendo STOP prima del riposizionamento.
3. **Fixture:**
   - joystick avanti a fondo scala per 2 cicli, poi rilascio;
   - indietro a fondo scala;
   - avanti con piccoli scostamenti laterali del dito: nessuno sway;
   - diagonale durante l'avanti: `JOYSTICK_STEERING`, poi ripartenza sterzata;
   - inversione;
   - ripetere la sequenza H1 (srotolamento sterzato).
4. **Terra:**
   - joystick avanti ~40 s, confrontando overrun, perse e stack con la WALK100 della dev.4;
   - STOP durante il passo frontale, poi STAND;
   - PRONE.

Osservare `loop_stack_min_free` (atteso molto sopra i 1696 B della dev.4) e `refused_by` in caso di saturazione.

## 10. Verdetto

**READY FOR SUPERVISED FLASH** (APP-only, robot supervisionato, **prima sulla fixture**, poi a terra).

Motivi:
- Tutte le verifiche offline pertinenti sono PASS.
- Le leggi non modificate sono identiche bit a bit alla dev.4.
- Il percorso ad alto rischio della dev.4, cioè lo stack della loop task sotto una richiesta HTTP, scende di ~4,9 KB nel caso peggiore statico.
- I comportamenti nuovi riusano leggi già certificate: per il joystick frontale la WALK STANDARD v2, VH a 100 mm sul pavimento.
- Il nuovo stato d'ingresso non comanda mai attuatori.
- Rollback alla dev.4 pronto e verificato.

**Prima causa concreta di rischio residuo**, non qualificabile offline: la legge v2 **all'indietro** e i **cambi di passo** in camminata non sono mai stati eseguiti sull'hardware, e la loro margine statica dipende dal COM reale. Per questo l'ordine del collaudo al §9 parte dalla fixture con lo stick a metà corsa.

Non è una release stabile: la dev.5 resta IMPLEMENTED NOT VALIDATED HARDWARE finché il collaudo del §9 non è completato. La decisione di flash, l'attivazione dei servo e il merge restano all'operatore.
