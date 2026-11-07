# MATDOG dev.15 — RCA del timing Motion (20 ms) — 07/11/2026

**Natura:** analisi offline del firmware realmente installato e delle tracce hardware del 06/11/2026. **Nessun flash, reboot, movimento, scrittura NVS/EEPROM o modifica firmware.** La patch indicata al §10 è una raccomandazione, **non implementata**.

Classificazione usata: **VERIFIED HARDWARE** (misura nelle tracce del robot), **VERIFIED OFFLINE** (codice/ELF/test host riproducibili), **IMPLEMENTED NOT VALIDATED**, **HYPOTHESIS**, **UNKNOWN**.

Riferimenti al codice: percorsi relativi a `05_Firmware/MATDOG_Controller/`, al commit `7997ad4` (identici nel merge `e5b0644` di questo branch, §1). Riferimenti alle tracce: `trace(N)` = `matdog-motion-trace (N).json` dell'archivio in [`raw/`](raw/), con `now_ms` del campione.

## 0. Risposte brevi

| Domanda | Risposta | Classe |
|---|---|---|
| Dove si consumano i 20 ms? | Nel **calcolo numerico**: NOMINAL (IK nominale del gait) + COMPENSATE (IK inclinata per la correzione IMU, fit del supporto, prove di sicurezza). In WALK la mediana delle release è 24,3 ms, di cui NOMINAL+COMPENSATE 21,6 ms; l'I/O servo (READ+FRAME) vale ~2,8 ms. | VERIFIED HARDWARE |
| Il budget è superabile anche nel caso migliore? | **No nel WALK sterzato:** la release joystick più veloce osservata (firma `1/8/12/23`) dura **28,2 ms**; la WALK45 più veloce (`1/4/8/15`) **18,4 ms**. La release è strutturalmente oltre il periodo. | VERIFIED HARDWARE |
| Prima divergenza verificabile | La **prima release con compensazione IMU** (DISENGAGE, 2 solve / 8 IK) supera 20 ms: trace(9) `now_ms` 675882, **20,04 ms** (COMPENSATE 14,23 ms). Prima di essa (IDLE, ACQUIRE) ≤ 4,9 ms. | VERIFIED HARDWARE |
| Causa principale | Calcolo double in **software** (ESP32-S3 senza FPU double; primitive `__adddf3/__muldf3/__divdf3` in ROM) di due catene IK per release, eseguito ogni 20 ms dal commit `2a630a0` che ha **rimosso il dimezzamento del solve IMU in WALK** introdotto in dev.7. | VERIFIED OFFLINE (meccanismo e regressione); quota esatta per singola operazione HYPOTHESIS |
| HTTP 503 | Effetto diretto dell'overrun: un comando che richiede slack attende al massimo 100 ms una release che lasci ≥ 5 ms prima della successiva; con release > 15 ms la finestra non arriva. Non è starvation di HTTPD, né lock, né servo/IK. | VERIFIED OFFLINE (percorso) + coerente con VERIFIED HARDWARE |
| Patch minima | (T0) correggere le etichette invertite della diagnostica ed esportare lateness/esterni; (T1) ripristinare il riuso, a release alterne, dell'ultimo offset di correzione accettato, **mantenendo ogni controllo di sicurezza a ogni release**. | raccomandazione, non implementata |
| Il timing rende il robot lento? | **No, oggi no**: il clock del gait è tempo reale (`SimpleMotion.cpp:1057`). La lentezza deriva dalle costanti del gait (periodo 6,5 s, stop di 1 ciclo, rampa joystick di 2 cicli). Il timing diventa però il limite per qualunque accelerazione. | VERIFIED OFFLINE |
| UNKNOWN principale | Perché lo **stesso lavoro deterministico** in HOLD impiega da 7,4 a 33,1 ms (×4,5) sul robot. | UNKNOWN |

## 1. Sorgente analizzato e provenienza

| Elemento | Valore | Verifica |
|---|---|---|
| Firmware installato | `1.0.1-dev.15`, build `7997ad4d4497` | `/status` di sessione: `build_id=7997ad4d4497` (VERIFIED HARDWARE) |
| APP | 1.691.328 byte, SHA256 `b81d3a4ffaa27a7690b196e77e7174c0cc7851563fcfd64cd3bddb40e62f9e13`, app0 `0x010000` | `esptool verify-flash` "digest matched" (log di sessione locale) |
| Manifest di build | `SOURCE_COMMIT=7997ad4d4497cd2675421d832362b3a0c144a46d`, `SOURCE_STATE=CLEAN`, `GIT_DIRTY=0`, `APP_SHA256` uguale | `SHA256SUMS` della build: tutti OK (locale ASUS) |
| Sorgente pubblicato | merge `e5b0644` su `docs/dev15-hardware-validation-20261106`; 2° genitore = `7997ad4` | `git diff 7997ad4 e5b0644 -- 05_Firmware 06_Software` → solo `CHANGELOG.md` (conflitto risolto a mano, non compilato); tree `src`, `MATDOG_Controller.ino`, `scripts`, `host_bridge`, `viewer` **identici** a `7997ad4` |
| `main` | `3f7d7ca`, linea dev.7 + documentazione | **non** contiene dev.15; non è l'APP installata |
| Configurazione installata | `CONFIG_COMPILER_OPTIMIZATION_SIZE=y` (‑Os), loop Arduino su core 1, `ARDUINO_LOOP_STACK` sostituito da `SET_LOOP_TASK_STACK_SIZE(16 KB)` (`MATDOG_Controller.ino:6`), I-cache 16 KB / D-cache 32 KB, 240 MHz | `sdkconfig` della build installata (locale) |

Una ricompilazione del tree pubblicato **non** è dichiarata byte-identica all'APP installata (il timestamp di compilazione del core differisce).

## 2. Evidenze usate e mancanti

- **Archivio** `raw/MATDOG_DEV15_TIMING_RCA_INPUTS_2026-11-07.zip`, SHA256 `d0e52dfdb9d8a0a40df22d97def5d45c8dc824f74235d90c1db0b662678a6dc5` (= atteso); 15 JSON con SHA identici a [EVIDENCE_MANIFEST.md](EVIDENCE_MANIFEST.md). Contenuto rivisto campo per campo: solo telemetria, nomi giunto, etichette comando, ID casuali di istanza browser. Nessuna credenziale, IP privato, cookie o dato personale.
- **Script riproducibile** [`tools/timing_rca_extract.py`](tools/timing_rca_extract.py) (sola lettura; ricalcola ogni tabella dei §5–§8).
- **Probe host** [`tools/rca_host_probe.patch`](tools/rca_host_probe.patch): strumentazione **solo di analisi** sul rig `test_simple_motion.cpp` (non va mai applicata a un firmware). Applicata a una copia pulita di `7997ad4` riproduce i risultati del §5.3.
- **ELF installato** (`MATDOG_Controller.ino.elf`, locale): tabella simboli.
- **Mancanti:** nessun dump USB `@MOTION TIMING` nella sessione dev.15 (`CommandRouter.cpp:266-278` stampa lateness, visit gap e tempi esterni, che il JSON HTTP **non** espone); nessuna traccia per-release continua; nessuna misura del tempo STOP→frame di arresto.

## 3. Percorso di una release

```
core 1, loopTask prio 1 (MATDOG_Controller.ino:12-15: loop(){controller.update(); delay(1);})
Controller::update  (Controller.cpp:1047-1233)
 ├─ serviceStopLane()                                   :1049   STOP da HTTPD (core 0) applicato qui
 ├─ updateSimpleMotion()  ──► SimpleMotion::update      :1050   release se now ≥ nextUs_
 ├─ router USB, IMU/BMS, LED, census, calibrazione       :1053-1098 (tempi "esterni")
 ├─ updateSimpleMotion()                                 :1100
 ├─ wifi snapshot, OTA, LED status, HttpTransport::update:1222   UNA richiesta HTTP servita qui (core 1)
 ├─ updateSimpleMotion()                                 :1223
 └─ telemetria solo se nextUs_ > now+5 ms                :1225-1226

SimpleMotion::update (SimpleMotion.cpp:1082-1274)  — una release
 1. scheduling: skipped=(now-nextUs_)/20ms; missed_+=skipped; nextUs_ avanza   :1097
    gap > 100 ms con coppia ON → FAULT MOTION_DEADLINE_LOST                    :1099
 2. READ: 2 letture rapide (+1 lettura di controllo ogni 6 release)            :1107-1124, :1223-1232
 3. HOLD:  compensate() → compensateLevel()                                    :1269-1273
    WALK:  generateWalk() [NOMINAL]                                            :1260, :1041-1074
             solveGait()  = IK nominale 4 zampe                                :1065
             compensate() [COMPENSATE] → supportLevelTarget (FK supporto)      :949
                 compensationTarget = IK inclinata 4 zampe (8 con fillet)      :970, :869-881
                 compensationSafe  = pointSafe, box, RAW, StopPlan::build      :883-926
 4. send() [CONVERT]: resolve RAW 12 giunti; target identico → nessun frame   :560-580
    frameTimed() [FRAME]: SYNC_WRITE 12 servo                                  :133-140
```

Task sugli altri core: HTTPD, lifecycle HTTP, rete e IMU sono fissati sul **core 0** (`HttpTransport.cpp:195,327,342`; `WifiManager.cpp:71-72`; `Bno085Imu.cpp:29-30`). Sul core 1 restano loopTask, IDLE1 e il task eventi Arduino (`CONFIG_ARDUINO_EVENT_RUNNING_CORE=1`, attivo solo su eventi Wi-Fi). Codice Motion e libm sono in flash mappata (`0x420xxxxx`), eseguiti tramite l'I-cache da 16 KB condivisa dai due core (VERIFIED OFFLINE, ELF + sdkconfig).

## 4. Che cosa misurano i contatori

| Grandezza | Definizione nel codice | Rappresenta la deadline? |
|---|---|---|
| `max_release_work_us` / `command_timing.max_work_us` | tempo **di parete** da inizio a fine `SimpleMotion::update` per una release; `sum(sezioni)==workUs` (`MotionTiming.h:84-93`) | sì per la durata della release; include preemption e stalli (`MotionTiming.h:7-8`) |
| sezioni READ…REST | tempo di parete per sezione, annidamento RAII (`MotionTiming.h:53-63`) | sì; **non** separano CPU, attesa I/O e stalli |
| `missed_releases` | numero di **periodi interi** saltati: `(now-nextUs_)/20000` (`SimpleMotion.cpp:1097`) | **no**: una release da 27 ms seguita da una leggera non incrementa il contatore (trace(9)/(10): 0 perse con release da 26,96 ms) |
| `max_gap_us` | intervallo massimo tra inizi di release (`:1095`) | sì (max 51,16 ms; soglia di fault 100 ms) |
| `first_miss_lateness_us` | ritardo all'inizio della **prima release che ha saltato un periodo**, cioè l'accumulo dei superamenti precedenti | sì, ma è cumulativo |
| works `solves/ik/newton/fk` | contatori manuali; **non** contano `StopPlan::build`, `pointSafe`, `resolve`; FK di `compensateLevel` fissato a 3 (`SimpleMotion.cpp:950`) | parziale |

**Difetto diagnostico D1 — VERIFIED OFFLINE + confermato sui dati.** `captureStatus` mette `firstMiss` nell'indice 0 e `maximum` nell'indice 1 (`MotionService.cpp:139`), ma `formatStatus` serializza l'indice 0 come `"max_work"` e l'1 come `"first_miss"` (`MotionService.cpp:180`). Nelle tracce l'array `first_miss` somma a `command_timing.max_work_us` in **3152/3152** coppie (traccia, record). La Dashboard eredita l'inversione (`MotionDashboard.h:148`); il dump USB è corretto (`CommandRouter.cpp:267-269`). `WINDOW_METRICS.csv` usa `command_timing` e non è affetto. In questo documento i record sono rietichettati correttamente.

## 5. Costo delle sezioni

### 5.1 Release che precedono un massimo o una prima perdita (campione meno distorto), mediane in µs — VERIFIED HARDWARE

| Fase | n | READ | NOMINAL | COMPENSATE | CONVERT | FRAME | REST | Totale | > 20 ms |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| WALK_STANDARD | 65 | 1596 | 9417 | 12170 | 889 | 1172 | 118 | **24319** | 48/65 |
| HOLD | 16 | 1734 | 0 | 10252 | 886 | 0 | 134 | 14521 | 7/16 |
| HEIGHT | 30 | 1624 | 0 | 6851 | 930 | 1170 | 158 | 11688 | 0/30 |
| PRONE_HEIGHT | 14 | 1648 | 882 | 8738 | 952 | 1170 | 174 | 13584 | 0/14 |
| DISENGAGE | 7 | 1594 | 939 | 8775 | 938 | 1169 | 224 | 13187 | 0/7 |

Fonte: tutte le tracce, record rietichettati, `tools/timing_rca_extract.py`. IDLE (trace(8)): massimo 4,93 ms; ACQUIRE: READ 4,0 ms + CONVERT 0,58 ms = 4,8 ms.

### 5.2 Stessa firma di lavoro, min/mediana/max (release corrente, µs) — VERIFIED HARDWARE

| Fase, firma `solves/ik/newton/fk` | n | NOMINAL | COMPENSATE | Totale |
|---|---:|---|---|---|
| WALK joystick `1/8/12/23` | 25 | 10101 / 17153 / 19229 | 13302 / 16038 / 27066 | **28217** / 37519 / 49586 |
| WALK45 `1/4/8/15` | 18 | 6055 / 9854 / 13073 | 8520 / 14444 / 17889 | **18430** / 30546 / 34319 |
| HOLD `1/4/4/11` | 16 | 0 | **7354 / 28244 / 33109** | 10628 / 31510 / 38125 |
| HEIGHT `1/4/4/8` | 20 | 0 | 5864 / 11672 / 14234 | 11471 / 16458 / 21294 |

I record del §5.2 sono estremi (massimi o prime perdite): le mediane sono distorte verso l'alto, i **minimi** sono limiti superiori affidabili del costo reale.

**Frequenza effettiva (de-duplicata tra finestre sovrapposte):** WALK_STANDARD **45,1 Hz** (292,8 s, 4,91 periodi persi/s), HOLD 49,4 Hz (0,59/s), tutte le altre fasi 50,0 Hz. — VERIFIED HARDWARE

### 5.3 Ripartizione del calcolo sul rig host — VERIFIED OFFLINE

Rig `test_simple_motion.cpp`, produzione `SimpleMotion` invariata, IMU inclinata costante; i contatori di lavoro del rig **coincidono esattamente** con quelli del robot (HOLD `1/4/4/11`, WALK45 `1/4/8/15`, joystick `1/8/12/23`). Tempo host (double hardware x86) per release:

| Scenario | host µs/release | solveGait | compensationTarget | supportLevelTarget | compensationSafe (di cui StopPlan) | pointSafe |
|---|---:|---:|---:|---:|---:|---:|
| HOLD100 inclinato | 7,9 | — | 5,27 (≈ 65 %) | 1,01 | 0,35 (0,13) | 0,22 |
| WALK45 inclinato | 21,2 | 8,86 (42 %) | 6,09 (29 %) | 1,06 | 1,99 (1,05) | 0,53 |
| JOYSTICK inclinato | 23,1 | 9,86 | 7,02 | 1,04 | 1,87 (0,95) | 0,53 |

Chiamate libm per release (`run_motion_cost_probe.sh`, invariato): WALK45 242 sin/cos, 64 trig inverse, 94,5 sqrt; JOYSTICK 334,5 / 74,3 / 99,6; HOLD 106 / 15 / 57,7.

**Coerenza robot/host (HYPOTHESIS ben supportata):** il calcolo minimo osservato sul robot (NOMINAL+COMPENSATE+CONVERT del record più veloce) è HOLD 8,3 ms, WALK45 15,5 ms, joystick 25,3 ms: rispettivamente ~1050×, ~730× e ~1100× il tempo host dello stesso scenario. Un rapporto quasi costante tra scenari con contenuto numerico diverso indica un costo **proporzionale al calcolo**, compatibile con aritmetica double emulata (nell'ELF installato le primitive sono in ROM: `__adddf3@0x40002184`, `__muldf3@0x40002418`, `__divdf3@0x40002250`) e build ‑Os. Il costo per singola operazione su questa scheda **non è stato misurato**.

## 6. Prima divergenza verificabile

1. **Prima release oltre il periodo (per sessione) — VERIFIED HARDWARE.** È la prima che esegue la compensazione IMU con contact-IK:
   - sessione 1: trace(9) `now_ms` 675882, fase DISENGAGE, **20,041 ms** (READ 2,22; NOMINAL 1,81; **COMPENSATE 14,23**; CONVERT 1,50), firma `2/8/6/14`; IMU `ATTENUATED`;
   - sessione 2: trace(17) `now_ms` 214585, DISENGAGE, **23,905 ms** (COMPENSATE 18,34), firma `2/8/8/16`.
   Nelle fasi STAND fuori da HOLD/WALK, `compensate()` esegue un solve di prova e, se la scala cambia di oltre il 2 %, un secondo solve (`SimpleMotion.cpp:1016-1036`): durante la rampa quintica di 1 s dopo l'abilitazione (`:1029`), e quando l'attenuazione vicino ai limiti cambia, sono quindi tipici 2 solve per release (firme `2/8/…` in DISENGAGE, STAND_DEPLOY, HEIGHT, PRONE_*).
2. **Primo periodo perso — VERIFIED HARDWARE.** Arriva dopo un accumulo di superamenti:
   - sessione 1: trace(11) `now_ms` 1010845, WALK_STANDARD: release da **33,456 ms** (NOMINAL 9,86; COMPENSATE 17,34) → la successiva parte in ritardo di 29,75 ms;
   - sessione 2: trace(17) `now_ms` 227214, **HOLD**: release da **30,009 ms** (COMPENSATE 26,55) → ritardo 28,895 ms.
3. **Tempo fuori release — UNKNOWN, limitato.** Per i 16 primi-periodi-persi distinti vale `l_N + O_N = lateness + 20 ms − W_N` = **12,5–21,7 ms** (ritardo accumulato più lavoro del loop tra due release). Il JSON non permette di separare i due termini.

Nessun comando HTTP specifico è associato ai record di massimo: le tracce non lo consentono. Questo report **non** attribuisce un massimo a un comando.

## 7. Causa principale, concause, ipotesi

**Causa principale — VERIFIED OFFLINE + VERIFIED HARDWARE.** Il contenuto numerico di una release WALK (IK nominale `solveGait` + IK inclinata `compensationTarget` + fit del supporto + prova di sicurezza) costa da solo **18–28 ms** sul robot nel caso migliore e ~24 ms in mediana, oltre i 20 ms. Il calcolo è in double con emulazione software e build ‑Os.

**Regressione che l'ha resa strutturale — VERIFIED OFFLINE (git).** dev.7 (`4634de8`, `SimpleMotion.cpp:538-545`) applicava in WALK, a release alterne, l'offset articolare dell'ultima correzione risolta ("WALK budget: every other release keeps the joint offset…"). Su hardware dev.7 aveva 0 release perse in tre camminate (CHANGELOG). Il commit `2a630a0` (05/11, "run WALK120 with shared IMU leveling and continuous recovery") ha rimosso il meccanismo, introducendo al suo posto il fit del supporto e la prova STOP a ogni release. Da quel momento ogni release WALK registra `solves=1`.

**Concause:**
- C1 — **Inflazione del tempo di parete fino a ×2 (WALK) e ×4,5 (HOLD) a lavoro contato identico** — VERIFIED HARDWARE (§5.2). Il percorso HOLD è deterministico sul rig: anche con rampa di inclinazione fino a 0,24 rad, `StopPlan::build` gira al massimo 5 iterazioni e il costo host resta ~7,5–8 µs (VERIFIED OFFLINE). L'ipotesi "iterazioni StopPlan variabili" è quindi **falsificata** come spiegazione principale. Cause candidate (HYPOTHESIS, non discriminate): stalli di I-cache/flash per contesa con il core 0 (Wi-Fi, HTTPD, invio degli asset del Twin), interrupt sul core 1, differenze di percorso non riprodotte dal rig.
- C2 — **Joystick con fillet:** 8 IK invece di 4 (`compensationTarget` usa la variante edge-contact, `SimpleMotion.cpp:874-875`): +50 % di calcolo — VERIFIED HARDWARE (firme) + OFFLINE.
- C3 — **HOLD risolve ogni 20 ms anche se il risultato viene scartato** (`imu_correction.reason=WORKSPACE` frequente in HOLD, `SimpleMotion.cpp:970-978`): un solve completo per release senza nuovo comando — VERIFIED OFFLINE + coerente con HW.
- C4 — **HTTP servito sul core Motion** una volta per loop, con 3 chiamate `updateSimpleMotion` per loop (`Controller.cpp:1050,1100,1222-1223`): un loop può contenere fino a 3 release pesanti; `loop_max_ms` arriva a 105–110 ms ≈ 3 × 35 ms (trace(12)–(14), VERIFIED HARDWARE). La lettura di stato costa ~0,6–0,7 ms (`read_service_max_us`), con un caso isolato da 10,8 ms (trace(21)): **non** è la causa dell'overrun, ma ne subisce la latenza.

**Escluso o non supportato:**
- letture UART bloccanti: READ 1,5–4,1 ms, entro il budget dichiarato (`SimpleMotion.cpp:1107-1108`) — VERIFIED HARDWARE;
- ritrasmissione di target identici: già evitata (`SimpleMotion.cpp:572-574`), FRAME=0 nella maggior parte dei record HOLD — VERIFIED OFFLINE + HW;
- contesa mutex/lock nel percorso Motion: `resolve`/`decodeRaw` non usano lock (`SimpleMotionIntegration.cpp:45-64`); nessun mutex nel percorso release — VERIFIED OFFLINE.

## 8. HTTP 503 `MOTION_BUDGET_UNAVAILABLE`

**Percorso — VERIFIED OFFLINE.**
1. HTTPD (core 0) chiama `dispatch`. Con Motion attiva (`twin_deferred_`) e richiesta non esente, il budget scende a 100 ms (`HttpTransport.cpp:426-427`, `HttpTransport.h:156`). Sono esenti le letture di stato, login/sessione e la velocità zero (WALK STOP); il refresh JOYSTICK è esente solo se il joystick è già attivo (`HttpTransport.cpp:402-415`, `:551`).
2. Il loop (core 1) prende la richiesta. Se `motionSlack(now,5000)` è falso, la rimette in coda **senza** eseguirla (`HttpTransport.cpp:551-555`); `slack` richiede `nextUs_ > now + 5 ms` (`MotionService.h:54-55`).
3. Scaduti i 100 ms, HTTPD abbandona e risponde `MOTION_BUDGET_UNAVAILABLE` (`HttpTransport.cpp:488`), con `elapsed_ms` 101–102.

Poiché `nextUs_` viene fissato all'inizio della release, uno slack ≥ 5 ms esiste solo dopo una release di durata < ~15 ms (meno il ritardo accumulato). Con le release HOLD/WALK del §5 questa finestra può non presentarsi in 100 ms.

**Osservato — VERIFIED HARDWARE (3 rifiuti distinti, finestre sovrapposte de-duplicate):**

| Comando | Stato | Lavoro max nel campione | `elapsed_ms` | Latenza client |
|---|---|---|---|---|
| WALK STANDARD 45 mm | STANDING/HOLD | 33,99 ms | 102 | 154,7 ms |
| STAND 100 mm | STANDING/HOLD | 34,92 ms | 101 | 135,9 ms |
| JOYSTICK (0,0 / 3,1 mm/s) | WALKING | 48,76 ms | 101 | 115,9 ms |

Contatore `transport.busy` sempre 0: nessun `MAILBOX_BUSY`, quindi nessuna contesa del mailbox. Incrementa `late`.

**Conclusione:** il 503 è la **conseguenza deterministica dell'overrun Motion** tramite la regola dei 5 ms di slack. Non indica starvation di HTTPD, lock, timeout del servo o un difetto IK. Il comando **non** ha raggiunto il Motion executor. Per il JOYSTICK rifiutato, lo stato `joystickActive()` al momento della ricezione è UNKNOWN.

**STOP:** usa una corsia separata, senza mailbox (`HttpTransport.cpp:1371-1386`, budget 600 ms `HttpTransport.h:177`). Il Controller la serve all'inizio del loop e dentro le transazioni bus prerilasciabili (`Controller.cpp:1041-1045`, `ServoBus.cpp:314,394`). ACK STOP osservato lato client: 20–83 ms (n=5, VERIFIED HARDWARE). Il tempo fino al frame d'arresto **non è misurato** (UNKNOWN). Durante NOMINAL/COMPENSATE non ci sono punti di prelazione: l'applicazione può attendere la fine del calcolo in corso (fino a ~45 ms osservati).

## 9. Difetti di timing confermati

| ID | Difetto | Classe |
|---|---|---|
| D1 | Chiavi JSON `timing.max_work`/`timing.first_miss` invertite (`MotionService.cpp:139` vs `:180`), etichette Dashboard errate | VERIFIED OFFLINE + dati |
| D2 | Release WALK oltre 20 ms anche nel caso migliore (WALK45 18,4 ms, joystick 28,2 ms); WALK a 45,1 Hz effettivi | VERIFIED HARDWARE |
| D3 | Rimozione in `2a630a0` del solve IMU a release alterne (dev.7), senza sostituto di budget | VERIFIED OFFLINE |
| D4 | `missed_releases` non conta i superamenti di deadline < 1 periodo accumulato; manca un contatore di overrun e un istogramma | VERIFIED OFFLINE |
| D5 | 503 sistematici per i comandi che richiedono slack mentre HOLD/WALK superano ~15 ms | VERIFIED OFFLINE + HW |
| D6 | Margine verso `MOTION_DEADLINE_LOST` (gap > 100 ms, `SimpleMotion.cpp:1099`) ridotto a ~2× (max gap 51,16 ms): un ulteriore aumento del calcolo porterebbe a fault con SAFE_OFF durante il cammino | VERIFIED HW (margine) / HYPOTHESIS (rischio) |

Osservazione non di timing: `loop_stack_min_free` scende a **2240 B** su 16 KB in trace(21)/(22) (prima 4144 B). Da tenere sotto controllo in qualunque patch.

## 10. Patch minima consigliata (NON implementata)

**T0 — Diagnostica, nessun effetto sul comportamento.**
- Correggere la mappatura indice→chiave in `MotionService.cpp:139/180`.
- Aggiungere al record HTTP i campi già presenti in `MotionTiming::Record` (`MotionTiming.h:20-22`): `latenessUs`, `visitGapUs`, `outsideVisitUs`, `externalUs[9]`.
- Aggiungere un contatore `overruns` (release con `workUs > kPeriodUs`).
Costo: pochi byte di snprintf su HTTPD; nessun nuovo task o framework.

**T1 — Budget: ripristinare il riuso a release alterne dell'offset di correzione IMU, in WALK e HOLD.**
- Release "piena": come oggi (`supportLevelTarget` + `compensationTarget` + `compensationSafe`); si memorizza l'offset articolare accettato (target compensato − nominale).
- Release "leggera": target = nominale corrente + ultimo offset accettato. Si eseguono **invariati** `pointSafe`, `contains(walkBox()/kHeight)` e `compensationSafe(..., stopProof)`, con la secante reale per `StopPlan`. Se un controllo fallisce, si esegue subito il solve completo; mai un bypass.
- Si forza il solve completo a ogni confine di fase, al recupero dopo STOP (`walkRecover_`), al retarget del joystick e quando l'IMU non è fresca.
- Lo slew in tempo fisico resta invariato: con un solve ogni 40 ms, `dt` = 40 ms rientra nel limite `kImuSlewMaxDtUs = 2 × periodo` (`SimpleMotion.h:44-45`).

Effetto atteso (HYPOTHESIS, da verificare): sul rig la release leggera elimina `compensationTarget` + `supportLevelTarget`, cioè ~34 % del calcolo WALK45 e ~75 % del HOLD. WALK45: coppie di release < 40 ms → periodi persi verso 0 (precedente dev.7). HOLD: release leggere ~4–5 ms → finestre di slack → 503 attesi a zero. Joystick: il caso pesante (≥ 28 ms) resta; riduzione, **non** eliminazione.

**Da non fare come primo passo:** ridurre controlli, box, limiti, prove STOP o freschezza del feedback; alzare la soglia di 100 ms o lo slack di 5 ms; cambiare il periodo del gait insieme al timing.

**Passo successivo, dopo i dati T0 (non minimo):** pianificazione a 25 Hz con interpolazione lineare in spazio giunti a 50 Hz. Le box certificate sono convesse, quindi una combinazione convessa di due target nelle box resta nelle box: proprietà da dimostrare per tutti i predicati, non solo per le box. In alternativa: ‑O2 o IRAM per le TU calde, `float` nei cicli IK interni, un task Motion dedicato.

## 11. Regressioni essenziali

**Offline (prima di qualsiasi build candidata):**
- Test unitario D1: l'array `max_work` deve sommare a `max_work_us`.
- `test_simple_motion`: alternanza piena/leggera; contatori `solves` 1/0; controlli di sicurezza eseguiti a **ogni** release; STOP che prevale in entrambe le release; recupero STOP; confini di fase; slew invariato.
- Differenza massima (mrad) tra target leggero e target risolto a ogni release, su WALK45/WALK60/joystick/HOLD inclinato.
- `run_motion_cost_probe.sh` e la patch di analisi: costo per tipo di release.
- Inviluppi e certificati: `run_joystick_envelope.sh`, `certify_steer_box.py`, suite PRONE/STOP esistenti.
- `static_audit` e catene di stack (16 KB, minimo osservato 2240 B).
- `test_http_fence_contention`, `test_motion_http_transport`: ammissione dei comandi con slack.

**Hardware (solo con autorizzazione esplicita di sessione):**
- IDLE di riferimento; STAND100/150 e HOLD 60 s con la Dashboard attiva; tavola inclinata in HOLD; WALK45 per 3 cicli; joystick avanti, strafe, yaw per 60 s ciascuno.
- STOP in HOLD e in WALK; WALK STANDARD e STAND inviati da HOLD (verifica dei 503).
- Criteri: 0 periodi persi in WALK45; `overruns` e lateness registrati; nessun 503 per STAND/WALK da HOLD; STOP ACK ≤ baseline (20–83 ms); `loop_stack_min_free` ≥ baseline.

## 12. Rischi per STOP, sicurezza e calibrazione

- **STOP:** T1 accorcia il calcolo e quindi l'attesa prima del prossimo punto di prelazione. La corsia STOP non viene toccata. Rischio: un offset riusato senza prova STOP; è escluso dalla specifica (prova a ogni release).
- **Sicurezza geometrica:** il target leggero non è la soluzione IK esatta della posa inclinata corrente. Deve restare nelle box e nei limiti certificati, verificato a ogni release; l'errore di contatto atteso è dell'ordine di variazione del nominale in 20 ms (stimato < 0,2 mm a 6,9 mm/s, HYPOTHESIS da misurare offline).
- **Calibrazione/NVS:** nessun impatto; nessuna scrittura. Mapping, limiti, geometria e collisioni invariati.
- **Certificati:** l'ampiezza della correzione applicata cambia nel tempo; gli inviluppi joystick e WALK vanno rieseguiti.

## 13. Impatto sulle prestazioni e previsione condizionata

**Limiti numerici imposti dal software — VERIFIED OFFLINE (`SimpleMotion.h`):**

| Costante | Valore | Effetto |
|---|---|---|
| `kWalkPeriodS` (`:16`) | 6,5 s | periodo del ciclo gait |
| `kStrideM` (`:21`) | 30/45/60 mm | vx 4,6/6,9/9,2 mm/s |
| joystick (`:38`) | 45 mm, 20 mm, 0,20 rad per ciclo | vx 6,9 mm/s, vy 3,1 mm/s, yaw 1,76°/s |
| `kJoystickRampCycles` (`:38`) | 2 cicli | **13 s** per arrivare al fondo scala |
| `kWalkStartCycles`/`kWalkStopCycles` (`:19`) | 1 / 1 | 6,5 s di accelerazione e 6,5 s di decelerazione |
| `gaitSettleCycles` (`Gait.cpp:200`) | duty − 0,75 = 0,075 cicli | 0,49 s |
| `walkStop()` (`SimpleMotion.cpp:556`) | attende il quarto di ciclo successivo | 0–1,63 s |

WALK STOP → STAND = 0–1,63 + 6,5 + 0,49 s + assestamento = **7,0–8,6 s**, coerente con i 7,3–8,7 s misurati. È la legge del gait, **non** il timing. Il clock del gait usa il tempo reale (`SimpleMotion.cpp:1057`): i periodi persi abbassano la frequenza di campionamento (45 Hz in WALK), non la velocità.

**Limiti fisici dimostrati:** nessuno per la velocità. Unico dato quantitativo: il margine **statico** del supporto per lo sway joystick da 14 mm, +0,55 mm (scenario casuale) / +1,85 mm (steady), da modello offline ([Joystick_Motion_2026-11-05/REPORT.md](../Joystick_Motion_2026-11-05/REPORT.md) §"Steering sway sweep").

**Stima dinamica (HYPOTHESIS: pendolo lineare, sway sinusoidale a frequenza di ciclo, altezza COM ≈ 0,12 m):** spostamento ZMP ≈ A·(2π/T)²·h/g con A = 14 mm:

| Fattore | Via periodo (stride invariata) | Spostamento ZMP | Confronto con +0,55 / +1,85 mm |
|---|---|---:|---|
| 1× | T = 6,5 s | 0,16 mm | dentro |
| 2× | T = 3,25 s | 0,64 mm | oltre lo scenario casuale |
| 3× | T = 2,17 s | 1,43 mm | quasi tutto il margine steady |
| 5× | T = 1,30 s | 3,97 mm | oltre |

**Percorso plausibile (non dimostrato sicuro):**
- **2×** (~14 mm/s): combinazione stride 60 mm e T ≈ 4,3 s (ZMP ≈ 0,37 mm), `kWalkStopCycles` 0,5 e rampa joystick 1 ciclo; recertificazione degli inviluppi; T1 prima. **Confidenza media-bassa.**
- **3×** (~21 mm/s, obiettivo di #52): richiede un nuovo profilo di sway consapevole dello ZMP o uno stride maggiore certificato, oltre al calcolo ricondotto entro 20 ms (T1 più un passo successivo). **Confidenza bassa.**
- **5×** (~35 mm/s): fuori dal regime quasi statico del crawl a duty 0,825; servono un gait diverso, il tracking dei servo misurato e 50 Hz rispettati. **Confidenza molto bassa.**

Con gait più veloci il costo per release non cala: le iterazioni di `StopPlan` crescono con la velocità articolare e ogni periodo perso produce un salto di target maggiore. Il timing va risolto **prima** di accelerare.

Lo "sbilanciamento all'indietro" all'avvio è compatibile con lo sway longitudinale `kWalkSwayXM = 10 mm` durante la rampa d'avvio (`SimpleMotion.h:19`): **HYPOTHESIS**, da verificare tracciando offline la X del corpo nominale nei primi 6,5 s.

## 14. UNKNOWN e strumento minimo

| UNKNOWN | Strumento minimo |
|---|---|
| Separazione CPU / stallo cache / preemption dentro NOMINAL e COMPENSATE | Contatori PMU Xtensa (istruzioni eseguite, cicli di I-cache miss) letti solo attorno alle due sezioni del record massimo, più T0. Oppure A/B hardware: stessa prova HOLD inclinata con Dashboard in polling vs osservazione USB HostLink a radio inattiva |
| Tempo fuori release `O_N` | T0 (campi `externalUs` e `outsideVisitUs` già raccolti, non esportati) oppure `@MOTION TIMING` via USB |
| Costo per operazione double sulla scheda | microbenchmark in un'APP di diagnostica, solo con flash autorizzato |
| Latenza STOP→frame d'arresto | timestamp di applicazione dello STOP nel record STOP |
| Stato `joystickActive()` del JOYSTICK rifiutato | reason del refusal più specifico (solo diagnostico) |
| Causa dei 10,8 ms di `read_service_max_us` (trace(21)) e dei `TIMEOUT_800MS` del polling | T0 e log transport per richiesta |

## 15. Riproduzione

```
sha256sum raw/MATDOG_DEV15_TIMING_RCA_INPUTS_2026-11-07.zip   # d0e52dfd…a6dc5
mkdir /tmp/rca && cd /tmp/rca && unzip <percorso>/raw/MATDOG_DEV15_TIMING_RCA_INPUTS_2026-11-07.zip && sha256sum -c SHA256SUMS
python3 -I <dossier>/tools/timing_rca_extract.py /tmp/rca
# rig host: copia pulita di 05_Firmware/MATDOG_Controller a 7997ad4
patch -p1 < <dossier>/tools/rca_host_probe.patch && bash scripts/offline_motion/run_motion_cost_probe.sh
```
