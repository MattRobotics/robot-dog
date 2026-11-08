# MATDOG — Pre-flash readiness e correzioni operative — candidata `1.0.2-dev.3` — 2026-11-08

Sostituisce `1.0.2-dev.2` (`c68842e`, APP `0d0a7cbe…`, mai flashata). Installata resta Stable `1.0.1-dev.17`. Legge WALK v2, inviluppi IMU, geometria, box, calibrazione, STOP, deadman e drive **invariati**. Legenda: VH = verificato su hardware (tracce dev.17), VO = verificato offline, UNKNOWN = serve misura HW.

## 0. Esito

| | |
|---|---|
| Candidata unica | `ROBOT_POWERED` CLEAN, sorgente **`94a260c`**, `FW_VERSION=1.0.2-dev.3`, APP **1.722.976 B**, SHA256 **`cef30d8c6c835fffee38ec5e2db9394efda35175aeafdf0b63ddcacf44316f37`**, RAM 137.988 B, tabella `8f756ecb…` invariata (§6) |
| Corretti offline | joystick dinamico (causa del ritorno al centro e del WALK STOP su dev.17); cascata di resync T2 in ritardo; costo della ricerca StopPlan; refresh joystick durante l'arresto per inversione (WALK_STOPPING/503) |
| Serve misura HW | picchi ×1,7–2 a lavoro identico (C1); trascinamento al distacco a WALK 75/100; COM reale senza Jetson; offset statico della correzione IMU (−24…−29 mrad) — ciascuno con misura e criterio nel collaudo |
| Verdetto | **READY FOR FLASH APP-only supervisionato**: nessun difetto software concreto noto e correggibile offline resta aperto. Gli aperti sono quantificati; nessuno si risolve offline senza toccare legge o inviluppi (§3) |

## 1. Budget Motion, release perse e MOTION_BUDGET_UNAVAILABLE

| Osservazione dev.17 (VH) | Causa | Stato |
|---|---|---|
| WALK STANDARD: 310 overrun / 3287 release (9,4 %), 1 persa, max 33,4 ms | release tipiche sotto 20 ms; overrun = **stesso lavoro** ×1,7–2 (C1) | C1 **UNKNOWN** (sotto) |
| Joystick: 474 / 2353 (20,1 %), 62 perse (1,3/s), max 47,2 ms | release piena tipica ~29,5 ms: **strutturale** | ridotta in dev.1 (A1/A2/T2/T1b, −37/−38 % proxy libm); legge v2 a parità di costo: WALK100 234 contro WALK60 237 chiamate libm/release, joystick 307 contro 310 (VO, proxy non tempo Xtensa) |
| — | resync T2 in ritardo a cascata con jitter (ogni resync = un solve completo in più) | **corretto** `72bc90c`: 16–76 → 2 resync per 2000 release (VO) |
| — | ricerca StopPlan da 2 passi a ogni release | **corretto** `72bc90c`: parte dal primo multiplo ammissibile (1,5 max\|v\|/amax), stessi piani (2 M casi, 0 differenze); WALK100 255 → 234 libm/release |
| HOLD: 0 / 4285 | — | — |
| `MOTION_BUDGET_UNAVAILABLE`: 0 tra i comandi conservati | gate 5 ms: i refresh joystick sono esenti finché il joystick è attivo, con o senza cambio di direzione (`HttpTransport.cpp` `joystickRefresh`); l'unica finestra non esente durante una pressione era l'arresto per inversione | **corretto** `86aba6f`: l'ACK dice `JOYSTICK_REVERSING` e la pagina tace fino a STANDING |

C1 resta UNKNOWN: il JSON non la discrimina. Misure esistenti per separarla, raccolte nel collaudo: `timing.max_work`/`first_miss` (READ, NOMINAL, COMPENSATE, FRAME, `externalUs`, `sinceReleaseUs`), istogrammi D11 (`work_hist`, READ, lateness), dump USB `@MOTION TIMING`. READ in crescita → bus/UART; NOMINAL+COMPENSATE in crescita a READ costante → CPU/cache. **Overrun** (release oltre 20 ms, la successiva resta sulla griglia) e **release perse** (periodi saltati) sono riportati separatamente; il 9,4 % non è una soglia accettata. STOP lane, deadline, deadman: invariati.

## 2. Joystick dinamico (addendum) — `86aba6f`

| Anello | Difetto (dev.17) | Correzione |
|---|---|---|
| Controller `drive()` | qualunque componente indietro, anche minima, durante un passo in avanti = inversione → WALK STOP: leva a destra un filo sotto l'orizzontale ferma il passo | inverte solo una domanda **prevalentemente** contraria (parte avanti/indietro > laterale + yaw nell'ellissoide); laterale/diagonale continuano con la parte contraria tenuta a zero da `steerWalk` (comportamento esistente) |
| Dashboard `render()` | con `joyAllowed` falso chiamava `joyRelease()` (neutro = WALK STOP) anche per un poll in ritardo (>1,5 s senza stato), uno snapshot non fresco, un'oscillazione di `ready` o l'arresto per inversione del Controller | la presa finisce solo su fatti del Controller (fault, STOP, fine del suo passo joystick che non sia un'inversione) o su link perso (nessuno stato da 1,5 s **e** nessun refresh accettato entro la deadline di 1 s del Controller). Premere richiede ancora telemetria fresca e ammessa |
| Pad | zona morta solo radiale: una spinta laterale portava un segno avanti/indietro (sceglieva l'ordine delle zampe di un nuovo passo) | banda morta assiale avanti/indietro (stessa `JOY_DEAD`) |
| Inversione completa | rilascio forzato, knob al centro, nuova pressione dopo STANDING | WALK STOP unico (`JOYSTICK_REVERSING`), leva tenuta e silenziosa durante arresto e assestamento, ripartenza all'indietro a STANDING con l'intento tenuto |
| Controlli tenuti | disabilitati durante la presa | restano abilitati: `pointerup`/`pointercancel` arrivano sempre |

Latenza intento → riferimento: refresh ≤ 240 ms + HTTP (dev.17: mediana 57, p90 96, max 182 ms) + una release (20 ms); il primo piede nella nuova direzione alla prossima partenza di swing (≤ 1/4 di ciclo = 1,6 s); fondo scala 13 s (`kJoystickRampCycles` 2). **Rampa invariata**: 1,5 cicli fallisce il gate dell'inviluppo joystick (`random_3`, 146 raggi di tilt fuori), come 1 ciclo (dev.1). Regressione end-to-end: DOM (pagina reale) avanti → avanti-destra → destra un filo dietro → avanti-sinistra con stati reali del Controller, poll in ritardo, link perso, inversione, rilascio; le stesse buste HTTP ripetute in ordine attraverso gate HTTP, sessione, parser, MotionService e SimpleMotion (8 cicli, 4 cambi di direzione, **0 WALK STOP**; 1 per l'inversione); Chrome reale mouse e touch con il dito fuori dal pad (pointer capture).

## 3. Robustezza WALK e compensazione IMU

**COM (URDF con Jetson; robot del 30/10 senza Jetson, 2,807 kg).** Supporto statico WALK100 v2 contro errore del COM in x (`run_walk_support_check.sh --com-error-x-mm`, VO):

| dx (mm) | −8 | −6 | −4 | −2 | 0 | +2 | +4 | +6 |
|---|---|---|---|---|---|---|---|---|
| v2 min (avvio / regime / stop) | −2,31 | −1,15 | 0,01 (0,50/2,19/0,01) | 1,17 | **2,12** (2,12/3,13/2,13) | 0,94 | −0,24 (−0,24/0,72/−0,14) | −1,43 |
| v1.0.1 WALK60 min | −3,71 | −2,56 | −1,42 | −0,27 | 0,88 | 2,02 | 3,17 | 4,32 |

Finestra v2: dx ∈ (−4, +3,6) mm, dy ±3 mm, limitata da avvio e WALK STOP (a regime ≥ +0,72 mm su ±4 mm); centrata a −0,2 mm sul COM URDF. v1.0.1 richiedeva dx > −1,5 mm. Evidenza dev.17 (WALK45, correzione non limitata, VH): pitch richiesto in quattro appoggi −23,2 mrad; swing posteriore −14,7/−15,3 (Δ ≈ +8,5), anteriore −24,8/−26,0 (Δ ≈ −2,2): il corpo si muove ~4× di più quando si alza una posteriore, come previsto per il COM URDF dietro il centro dei contatti. La rimozione del Jetson (175 g) sposta il COM di ±3 mm per qualunque montaggio entro ±55 mm. **Decisione: `kWalkComShiftXM` 11 mm invariato.** Verifica HW: correzione IMU per gamba in swing a WALK 50/75 a terra (collaudo, Metriche).

**Trascinamento al distacco (~5 mm osservati su v1.0.1).** Lo swing v2 avanza in orizzontale come u² (rampa trapezoidale) mentre il sollevamento 64·lift·u³(1−u)³ parte come u³. Corsa orizzontale di riferimento prima che il sollevamento raggiunga h (VO):

| h | v1.0.1 WALK60 | v2 WALK50 | v2 WALK75 | v2 WALK100 |
|---|---|---|---|---|
| 1 mm | 1,5 | 2,4 | 3,6 | 4,8 |
| 2 mm | 3,3 | 4,3 | 6,5 | 8,6 |
| 3 mm | 5,4 | 6,3 | 9,4 | 12,5 |

I ~5 mm osservati corrispondono al distacco reale a ~3 mm di sollevamento di riferimento (cedevolezza): WALK50 resta vicino a v1.0.1, WALK100 può arrivare a ~2,3×. Correzione provata e **non adottata**: sollevamento 16·lift·u²(1−u)² (WALK100 a 3 mm: 12,5 → 8,0 mm) porta l'accelerazione di giunto a WALK100 da 76 % a 94 % del drive e richiede di rigenerare `kWalkTilt75/100`: non è minima né a basso rischio. Esito → collaudo H3 per passo, con arresto su inciampo e scelta del preset a terra; prossimo ciclo se confermato.

**Inviluppi IMU contro dev.17 (convenzione del codice).** HOLD 100/120: pitch richiesto −24…−29 mrad, roll −2…−3,5 (VH). v1.0.1 WALK60 (limite −8,5 mrad) era `ENVELOPE` nel **100 %** dei campioni con −8,4 applicati; WALK45 (limite −53) mai. Candidata: WALK50/75 limite −81,8 mrad (nessuna limitazione attesa), WALK100 −18,7 mrad (limitazione attesa spesso, residuo ~5–10 mrad, meglio di WALK60). Il lato +2,2° (+38 mrad) non è mai stato avvicinato: non è restrittivo. **Nessun allargamento.** L'origine dell'offset (pendenza del pavimento o zero IMU) si separa con un inclinometro in H1; una ricalibrazione non è in questo ciclo.

**Continuità** (avvio, WALK STOP, STANDING, recupero dopo STOP): coperta da SimpleMotion (passo RAW per release ≤ drive), digest (passo RAW max 12–20 tick, 0 frame fuori box), STOP support e contratto HTTP (STOP → STAND, STOP → WALK75): PASS.

## 4. STOP, jitter e riferimenti

T2: la resync in ritardo risolve la legge al proprio `now` (come v1.0.1) e ancora il bracket al punto di griglia: nessun frame indietro di `lateness`, nessuna cascata. La rate dello STOP usa la secante tra riferimenti trasmessi distanti ≥ 1 periodo (dev.1 B4); la prova STOP usa l'arrotondamento RAW peggiore (ceil(\|Δq\|/tick)). STOP in swing e in quattro appoggi per gamba: suite STOP support PASS; digest `WALK75_STOP_at300` verificato in 2,02 s; stress 36 semi con jitter e STOP: 0 frame fuori box. Ripresa dopo STOP: PASS (contratto HTTP).

## 5. Diagnostica per il collaudo (inventario; nulla aggiunto)

Distribuzione release: `command_timing.work_hist`, READ e lateness (D11); overrun e perse: `overrun_releases`, `missed_releases`, `command_timing.overruns/missed`, `max_gap_us`; lateness: `first_miss_lateness_us`; partizioni READ/NOMINAL/COMPENSATE/FRAME, lavoro esterno, servizi: `timing.max_work`, `first_miss`, `*_meta`; stack: `loop_stack_min_free`, `httpd_stack_min_free`; tracking/feedback/fault: `joints[]` (`raw`, `commanded_raw`, `goal`, `age_ms`, `control_age_ms`), `stale`, `fault`, `reason`; risposta HTTP: `Server-Timing` e tempi nella traccia; IMU: `imu_correction` wanted/applied/reason. Il nuovo motivo d'ACK `JOYSTICK_REVERSING` è un chiarimento del protocollo, non un osservabile.

## 6. Provenienza, regressioni, identità

Commit su `feat/motion-perf-next-20261107` (PR #67, Draft): `72bc90c` T2 + StopPlan · `9bb806c` strumento supporto/COM · `86aba6f` joystick dinamico + test · `94a260c` identità `1.0.2-dev.3` · commit documentale successivo.

| Regressione (sorgente `94a260c`) | Esito |
|---|---|
| SimpleMotion (incluso il nuovo criterio d'inversione) | PASS |
| Dashboard: fixture 79, DOM 21 buste / 89 comandi, contratto 5824 controlli, Chrome reale (layout con presa e passata dinamica, ViewCube, STOP durante il Twin) | PASS (il controllo `trace_input_ms_agrees` è fallito una volta su 1 ms di timestamp, poi 3/3 PASS: instabile, non legato al joystick) |
| STOP support; PRONE 4/4 | PASS |
| Gate inviluppo joystick, 32 scenari | PASS: supporto min 0,57 mm, clearance 2,18 mm, tilt 11,44° (prima 11,63°); 23 scenari identici a dev.2, gli 8 random cambiano per il criterio d'inversione (margine di tilt minimo 6,3 mrad in `random_3`, prima 26,2) |
| Digest dei frame | bit-identici ai precedenti (HOLD, WALK75/100 con STOP, joystick steady/random); stress 36 semi: 0 frame fuori box |
| Cost probe host, A/B con il sorgente dev.17 (`40728b7`) | trig + inv-trig + sqrt per release: WALK 312 (WALK60) → **212** (WALK100, −32 %), joystick 408 → **283** (−31 %), HOLD 97 → 97; tempo host medio 17,5 → 13,4 µs e 20,3 → 15,4 µs. Proxy soft-double, **non** tempo ESP32-S3: il beneficio sul robot si misura nel collaudo |
| Static audit (366 file) | PASS (le righe FAIL nel log sono le mutazioni storiche iniettate e rilevate) |

| Identità | |
|---|---|
| Build | `~/MATDOG/build-output/gait-redesign-20261108/build-94a260c/artifacts/ROBOT_POWERED-ota0/`, manifest `SOURCE_STATE=CLEAN`, `GIT_DIRTY=0`, `OTA_INGEST=0`, `MOTION_AUTHORIZED=0` |
| APP | 1.722.976 B (+1.936 B contro dev.2), SHA256 `cef30d8c…`, verificata su file e manifest; stringhe `1.0.2-dev.3` e `94a260cd3b4e` nel binario |
| Compatibilità APP-only | layout `MATDOG_16M_2x5M_NVS_V1`, tabella `8f756ecb…` = installata; scrittura solo `app0` 0x010000–0x1b4a60 (erase fino a 0x1b5000); NVS 0xfe0000–0xff0000, bootloader, otadata, `app1`, EEPROM servo non toccati |
| Flash preparato (non eseguito) | `~/MATDOG/verification-artifacts/MATDOG_1_0_2_DEV3_FLASH_PREP_20261108/flash-qualified-app0-1.0.2-dev.3.sh` (stesso schema qualificato di dev.17; senza `MATDOG_DEV3_FLASH_AUTHORIZED=YES_ROBOT_SUPPORTED`: REFUSE, verificato) |
| Rollback | dev.17 `14e055a0…` (1.707.808 B, `build-17db0ef`), dev.16 `73952eec…`, backup integrale `5cbba0b9…`: `rollback-assets.sha256` 4×OK |

Collaudo: [`HW_TEST_PLAN.md`](HW_TEST_PLAN.md).
