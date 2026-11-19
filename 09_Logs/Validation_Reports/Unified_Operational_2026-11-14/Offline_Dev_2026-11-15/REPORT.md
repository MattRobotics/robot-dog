# MATDOG — sviluppo offline 15/11/2026: Timing P0, RCA DALY, locomozione avanzata

> Copia nel repository del rapporto in `~/MATDOG/verification-artifacts/MATDOG_1_0_2_OFFLINE_DEV_20261115/REPORT.md`. I percorsi relativi `timing/`, `loco/` e `bms/` sono in quella cartella, sigillata con `SHA256SUMS.txt`.

Mandato autonomo, solo offline. MATDOG è rimasto spento: nessun flash, reset o movimento.

**Baseline:**
- branch operativo `integ/matdog-unified-operational-20261114`, HEAD `cfc788a` (firmware `539b646`);
- installato `8530d3a11ac9` (diagnostica dev.8, APP `75fdd3ac…0c4d4b`);
- validazione hardware: `MATDOG_1_0_2_DIAG8530_MOTION_HW_20261115/`.

**Categorie di stato:** VERIFIED HARDWARE, VERIFIED OFFLINE, IMPLEMENTED NOT VALIDATED, HYPOTHESIS, UNKNOWN. Nessun risultato offline è presentato come hardware.

## 0. Risposte

### TIMING

- **Quale costo è stato eliminato?** Gli stalli di fetch delle istruzioni del percorso cinematico caldo.
  - Sei funzioni, scelte per miss evitati per byte, sono passate dalla flash (I-cache da 16 KB) all'IRAM: `legModel`, `edgeContactLiftM`, `pointToWorld`, `contactForwardKinematics`, `contactInverseKinematics`, `forwardWithModel`.
  - Il PMU del 15/11 indica lo stallo istruzioni su memoria come causa dominante: 0,47–0,50 dei cicli N+C.
- **Quanto è stato ridotto offline?** In una traccia QEMU della sequenza fixture del 14/11, sul binario del candidato, i miss della I-cache simulata (16 KB a 8 vie) per release cambiano così:
  - yaw: −36 % medi (−40 % al netto della stampa del rig) e −45 % al massimo;
  - ripresa: −43 % medi (−46 %) e −50 % al massimo.

  È VERIFIED OFFLINE come miss simulati del solo core Motion, non come tempo.
- **Quanto spazio occupa?** Build di produzione ROBOT_POWERED CLEAN:
  - `.iram0.text` +4 628 B, flash −4 240 B;
  - **heap interno −4 608 B**: minimo previsto circa 33,5 KB contro i 38,1 KB misurati il 15/11.
- **Residuo previsto?** È un modello, non una misura: lo stallo è proporzionale ai miss.
  - WALK −1,6–1,9 ms e RECOVERY −3,0–3,3 ms medi per release;
  - picco da 39,7 a circa 31 ms.

  Il criterio dev.8 §8.3 (0 perse, 0 overrun, lavoro massimo ≤ 16 ms) **resta lontano**. Il residuo è calcolo: soluzione nominale e compensazione piena nella stessa release, double software in ROM (§2.2).
- **Cosa manca per chiudere P0?**
  1. Misura PMU sul chip del candidato diagnostico integrato `e321e86`, con la stessa suite fixture.
  2. Una riduzione di calcolo che oggi non esiste a semantica esatta (§2.2): serve una decisione su un cambio di semantica misurato e qualificato con i gate STOP, per esempio la ripresa su griglia T2 invece della soluzione esatta a ogni release.
  3. Correzione del protocollo §8.3 per il gap della prima attivazione (§2.3).

### BMS

- **Prima divergenza:** per 440–576 s non arriva alcun byte nella FIFO RX di UART2 (`rx_bytes=0`, `stale_bytes=0`), mentre la FIFO TX si svuota a ogni poll (218 poll da 8 B). È l'unica osservabile firmware.
- **Difetto firmware dimostrato?** **No.** Il replay offline del driver di produzione lo esclude: niente desincronizzazione, fail-closed, rientro corretto.
  - **`8e20c63` è equivalente sul filo:** in Arduino-ESP32 3.3.11 `flush()` è `uartFlushTxOnly(true)`, solo un'attesa di TX inattivo. Prima e dopo la modifica nessun registro UART viene scritto in modo diverso, e la forma d'onda su GPIO15, l'unico ingresso dell'XY-017, è identica.
  - Anche il drenaggio limitato e il gate `availableForWrite` sono letture.
- **Patch giustificata:** nessuna.
- **Misura che distingue:** analizzatore logico su GPIO15, GPIO16 e sul bus A/B (con un ricevitore RS485 in sola ascolto), durante un guasto 2–5 min dopo un'accensione a freddo, più una prova A/B del traffico Wi-Fi. Procedura in [`LOGIC_ANALYZER_PROCEDURE.md`](LOGIC_ANALYZER_PROCEDURE.md).

### LOCOMOZIONE (gemello MuJoCo, legge firmware attuale `cfc788a`, modello di contatto `dev7_ground`)

- **FAST 40 è fattibile?** **No, nelle condizioni attuali.**
  - Vincolo dominante: la prova STOP alla frenata qualificata (ACC 84). L'arresto sincrono porta HIP/UPPER fuori dal box. A duty fisso la velocità del piede in swing vale v/(1−duty) per qualunque falcata, e lo spazio di frenata cresce con v².
  - **Capacità credibile:** circa 30 mm/s utili, con la legge FAST attuale, una volta risolta la clearance (§4.3).
- **Velocità utile prevista dal gemello** (μ 0,8):

  | Movimento | Utile | % della legge |
  |---|---:|---:|
  | Avanti SLOW | 9,7 mm/s | 49 % |
  | Avanti STANDARD | 19,7 mm/s | 66 % |
  | Avanti FAST | 21,9 mm/s | 68 % |
  | Indietro STANDARD / FAST | −15,1 / −18,1 mm/s | 91 / 90 % |
  | Laterale | | 36–65 % |
  | Yaw | | 50–65 % |
  | Diagonale | | 48–56 % |

  Fortemente dipendente dall'attrito: con μ 0,6 l'avanti sale all'83–90 %, con μ 1,0 scende al 25–47 %.
- **Sidestep, rotazione, sway, clearance:** **nessun miglioramento implementato sulla legge operativa.**
  - **In avanti, causa verificata sulla legge attuale con il controfattuale:** se la zampa in swing non tocca il suolo, l'avanti arriva al 93–98 %, la clearance diventa positiva e l'escursione laterale del corpo dimezza.
  - **Nei cammini sterzati:** al contatto in swing si aggiunge il margine di supporto della legge sterzata. Senza contatto il robot del gemello cade, perché la zampa «in swing» porta l'8–9 % del peso (§4.6).
  - **Leva che lo risolve nel gemello:** compensazione della cedevolezza in appoggio (C·JᵀF) più sollevata del corpo (V7). Dipende da una cedevolezza hardware ancora UNKNOWN e fa fallire per costruzione i gate rigidi; serve un porting manuale su `539b646` (§4.5).
- **Protezioni verificate:** STOP, StopPlan, box, limiti, Patch A, FAST `05039ca` e riarmo `7878be0` sono invariati. Le uniche modifiche di questo mandato sono di posizionamento in memoria (§5).
- **Nuove capacità:** nessuna implementata come funzione operativa. CRAWL C2, V7 e TROT restano prototipi del 13/11 su base dev.7.

### INTEGRAZIONE

Vedi §5 e §6.

## 1. Provenienza e metodo

- **Stato iniziale verificato:** worktree operativo pulito, HEAD `cfc788a` uguale a origin, `main` `6838e4d`, stash vuoto.
- **Candidati su branch e worktree separati,** mai sul branch operativo:
  - Timing: `cand/timing-iram-20261115`;
  - diagnostico integrato: `cand/diag-8530d3a-iram-20261115`;
  - BMS e locomozione: nessuna modifica firmware.
- **Diagnostica `8530d3a`:** importata dal bundle verificato come `refs/preserve/20261114/diag-8530d3a`.
- **Strumenti esistenti riusati:**
  - rig QEMU `qreal` della chiusura P0 (replay fixture 14/11, conversioni gen-1 reali), con il clock CCOUNT adattato a `539b646`;
  - tracer e simulatore I-cache della dev.8 (`icache_sim`, `icache_replay`);
  - `run_qemu_motion.sh`, `static_audit`, `build.sh`;
  - gemello `fw_bridge` + `vnext_eval.py`.
- **Strumenti nuovi, solo analisi in sola lettura:** `timing/tools/rank_iram.py`, `eval_iram_sets.py`, `loco/summarize.py` e un wrapper che rende tollerante la sola stampa di `m6_gait.show()`.
- **Baseline del rig sull'albero esportato:** digest `1f96723695b8a124`, identico alla corsa B2 del 14/11; costi per classe identici.

## 2. Timing P0

### 2.1 A1 — IRAM mirata (candidato G1)

**Memoria misurata (ELF installato `8530d3a`):**
- I-cache da 16 KB: l'IRAM parte da 0x40374000.
- `.iram0.text` 82 587 B; `.dram0.dummy` 67 328 B.
- Ogni byte di IRAM è un byte in meno di heap interno, perché SRAM1 è condivisa. `heap_min_free` (`MALLOC_CAP_INTERNAL`) era 38 136 B il 15/11.
- Le routine double di libgcc sono in ROM e non passano dalla cache. libm (`asin`, `atan`, `hypot`, `sqrt`, `cos`) e il codice applicativo sono in flash.

**Traccia** (QEMU `in_asm,exec,nochain` filtrato sulla flash, in streaming verso `icache_sim`, 120 release di yaw e 150 di ripresa):
- codice flash toccato per release: 46,6 KB (yaw) e 56,5 KB (ripresa) contro 16 KB di cache;
- istruzioni in flash per release: circa 129 k e 191 k, su circa 864 k e 1 348 k totali. Il resto gira in ROM, cioè in virgola mobile software.

**Selezione per miss evitati per byte** (`rank_iram.py`, `eval_iram_sets.py`; baseline pulita 2 241 / 3 331 miss per release yaw / ripresa):

| Insieme | Byte di testo | Yaw | Ripresa |
|---|---:|---:|---:|
| **G1** (6 funzioni dense) | **4 381** | **−41 %** | **−48 %** |
| G2 (11 funzioni) | 9 085 | −51 % | −59 % |
| S6 (le 6 della dev.8) | 15 241 | −54 % | −63 % |
| G3 (14 funzioni) | 20 307 | −60 % | −69 % |

G1 dà circa l'80 % del beneficio di S6 con il 29 % dei byte. Le funzioni libm sono dense di miss ma stanno in librerie precompilate: spostarle richiederebbe di cambiare lo script di link del core, quindi sono escluse.

**Implementazione** (`5eff171`, `e7b4266`, `f349af9`):
- `src/motion/MotionHotPath.h` definisce `MATDOG_MOTION_HOT` come `__attribute__((section(".iram1.motion_hot")))` solo con `ESP_PLATFORM`; nessun header di piattaforma, come impone il gate di purezza.
- L'attributo è davanti alle sei definizioni. Nessuna istruzione, valore o firma cambia.

**Build di produzione** (ROBOT_POWERED, CLEAN, stesso script):

| Voce | Base `cfc788a` | Candidato |
|---|---:|---:|
| APP (B) | 1 773 504 | 1 773 888 |
| `.iram0.text` | 81 967 | 86 595 (+4 628) |
| `.flash.text` | 1 054 500 | 1 050 260 (−4 240) |
| Inizio heap DRAM | 0x3fcbaf60 | 0x3fcbc160 (+4 608 B) |

**Effetto misurato offline** (traccia sul binario reale del candidato, stessa sequenza):

| Finestra | Miss medi base | Miss medi candidato | Δ medio | Miss max base | Miss max candidato | Δ max |
|---|---:|---:|---:|---:|---:|---:|
| Yaw | 2 483 | 1 589 | −36 % | 4 401 | 2 408 | −45 % |
| Ripresa | 3 562 | 2 022 | −43 % | 4 596 | 2 286 | −50 % |

**Da verificare sul chip:** la riduzione reale di `istall_mem`, di overrun e di perse con il PMU, e l'heap minimo. Il modello trascura il traffico del core 0 e la contesa MSPI con la PSRAM: un miss che scompare in IRAM scompare anche da quel traffico, quindi il beneficio reale può essere maggiore, ma non è misurato.

### 2.2 A2 — coincidenza NOMINAL + COMPENSATE

Fatti (rig QEMU, replay fixture 14/11):

| Classe | Soluzione nominale | Compensazione piena | Entrambe | Picco |
|---|---|---|---|---|
| WALK | 527 release su 1 051 | 620 | **96 (9 %)** | 2,14 M istruzioni |
| RECOVERY | tutte | tutte | **214 su 214** | 2,2 M istruzioni |

- **Dipendenza temporale:** il look-ahead T2 si risolve alla prima release di ogni bracket. `lerpFrame` usa la soluzione «ahead» anche per stato, rami e appoggi, quindi non si può rinviare. Anticiparla di una release la calcolerebbe con parametri vecchi di una release, cioè con risultati diversi.
- **Riuso:** la compensazione leggera T1 richiede `reuseSettled_`, cioè una correzione che non scorre. Nei cammini sterzati la correzione è in ENVELOPE e cambia a ogni release (HW 15/11: 309 campioni ENVELOPE nel blocco D), quindi la compensazione è piena sempre e il bilanciamento T2 («nessuna release porta entrambe») fallisce quando cade la soluzione nominale.
- **Ripresa:** con `walkRecover_` la soluzione esatta all'orologio addolcito è voluta e T1 è escluso. Entrambe in ogni release per progetto: da qui il picco HW di 39,67 ms.
- **Lavoro duplicato a semantica esatta:**
  - il riuso dei RAW (B1) vale < 1 % dopo B2;
  - seed IK diversi o il bracket in ripresa cambierebbero i risultati entro la tolleranza del solutore, e quindi i RAW.
  - **Nessuna riorganizzazione minima trovata.** Non ho modificato nulla.
- **Decisione richiesta all'operatore:** chiudere P0 secondo §8.3 richiede di ridurre il calcolo della ripresa e dei cammini sterzati con semantica cambiata e misurata. Le due opzioni:
  - ripresa su griglia T2;
  - compensazione leggera con correzione in movimento entro il passo di slew.

  Ciascuna andrebbe qualificata con i gate STOP. Non l'ho implementata: cambia la semantica Motion.

### 2.3 A3 — gap di 147 ms: causa trovata

**VERIFIED HARDWARE** (log del 15/11) e codice.
- **Il meccanismo:** prima di un'attivazione da coppie OFF, `Controller::prepareSimpleMotion()` esegue in modo sincrono nel loop `configureAccCeiling(12 servo, 220)`.
- **Prima attivazione dopo l'accensione dei servo:** il registro 85 (volatile, 50 di fabbrica) richiede la scrittura con verifica, poi `kAccCeilingSettleMs = 100` (assestamento voluto, misurato il 10/11 e bloccato in `static_audit`), poi 12 riletture.
- **Log:** tra l'eco `>>> @MOTION STAND` e `MOTION_ACC_CEILING verified=220` passano 133 ms in A2 (prima STAND), contro 6–11 ms in A8, C1, D1, E1 (solo letture).
- **Effetto:** la release successiva parte con 127 ms di ritardo (6 slot) in ACQUIRE, a coppie OFF. Nessuna conseguenza sul moto.
- **Esclusi:** cache fredde, `7878be0`, NOMINAL/COMPENSATE, allocazioni.
- **Nessuna modifica:** è un passo di sicurezza voluto.
- **Correzione proposta al criterio** (non applicata): in §8.3 della dev.8, `missed_releases` e `max_gap_us` «invariati dall'avvio» vanno valutati dopo la prima attivazione. In alternativa il firmware li conterebbe separatamente quando la release è ritardata a coppie OFF. È una modifica di contatore, da decidere esplicitamente.

### 2.4 A4 — regressioni del candidato

| Prova | Base `cfc788a` | Candidato | Esito |
|---|---|---|---|
| Replay fixture 14/11, rig `qreal` (1 819 frame, host e Xtensa) | `1f96723695b8a124` | `5eff171` e `f349af9`: `1f96723695b8a124` | **identico**; le sei funzioni sono a 0x4037xxxx (IRAM) nell'ELF del rig |
| `run_qemu_motion.sh`, 18 scenari | 18/18 SAME | `5eff171`: 18/18 SAME, digest uguali alla base 18/18; `f349af9`: 18/18 SAME, digest uguali alla base 18/18 | **identico** |
| `run_motion_convergence_tests` (gate di purezza e provenienza) | PASS (14/11) | `5eff171`: FAIL (`esp_attr.h`, 3 pin); `e7b4266`: FAIL (3 pin); `f349af9`: **PASS** | ri-pin documentato; i controlli negativi falliscono ancora |
| `static_audit` (comprende `run_host_tests`) | PASS (chiusura P0, 14/11) | `5eff171`: FAIL (2 rilievi dello stesso gate); `f349af9`: **`STATIC_AUDIT = PASS`** (`HOST_TESTS = PASS`, `MOTION_CONVERGENCE_SOURCE = PASS`) | PASS |
| Build ROBOT_POWERED CLEAN, layout | PASS, tabella `8f756ecb` | PASS, stessa tabella, APP 1 773 888 B su 5 MiB | — |
| Test host (`run_host_tests`, compresi in `static_audit`) | PASS | vedi `static_audit` | — |
| Servo plant, STOP matrix, frame digest host | — | **non rieseguiti**: strumenti solo host, e sull'host l'attributo è vuoto, quindi il codice compilato è quello della base | equivalenza per costruzione, non per misura |

- **Equivalenza:** stesse istruzioni, stessi frame RAW, stessi rifiuti e STOP in tutti gli scenari QEMU. Il costo CCOUNT in QEMU è invariato (235 545 contro 235 621 medi, differenza dello 0,03 % di icount), come atteso: QEMU non modella la cache.
- **Memoria:** heap interno −4,6 KB; stack minimo del task Motion nel rig 7 496 B contro 7 560.

## 3. RCA DALY (fase B)

### 3.1 Osservazione e prima divergenza

Guasti: 06/11 (dev.14, risolto con un ciclo di alimentazione), 08/11, 09/11 (due), 14/11 (576 s), 15/11 (440 s).

**Firma** (D1, VERIFIED HARDWARE il 15/11):
- poll ogni 2 s, `rx_bytes=0`, `stale_bytes=0`, `first_byte=NONE`;
- rientro autonomo, con il primo frame a 128/129 byte e il primo byte puntuale a 17 ms;
- Motion bloccato in fail-closed.

La FIFO TX dell'ESP32 si è svuotata per 218 poll (1 744 B, molto oltre i 128 B della FIFO): altrimenti il gate `availableForWrite ≥ 8` avrebbe fermato i poll e il `fail_streak`. Questo dimostra che la periferica UART ha trasmesso. **Non** dimostra che il segnale sia arrivato al bus né che il BMS l'abbia ricevuto.

### 3.2 Analisi differenziale prima e dopo `8e20c63`

`8e20c63` è lo snapshot rc.11 r2 del 30/10. La modifica era già sull'hardware con le build rc di fine ottobre; `f838f67` committato ha ancora il `flush()`.

| Modifica | Effetto sul filo (codice core 3.3.11) | Esito |
|---|---|---|
| `flush()` rimosso dopo `write()` | `flush()` = `uartFlushTxOnly(uart,true)`: busy-wait su `uart_ll_is_tx_idle`, nessuna scrittura di registro, RX non toccato. Il TX è spinto dall'hardware in entrambi i casi. La sola differenza è che la CPU non aspetta 8,34 ms, quindi `markSent()` e `rx_len_=0` avvengono 8,34 ms prima: la finestra di 750 ms parte prima, ininfluente su risposte a 11–84 ms | **equivalente sul filo** |
| Drenaggio dello stale limitato a 129 B | sola lettura della FIFO RX, prima della richiesta; nel guasto stale = 0 | nessun effetto |
| Gate `availableForWrite() ≥ 8` | sola lettura dello stato FIFO TX | nessun effetto |
| XY-017 a direzione automatica | il suo unico ingresso logico è GPIO15, che ha la stessa forma d'onda nei due casi | non può dipendere dalla modifica |
| 9600 baud, 8 B in 8,34 ms, risposta a 11–84 ms | nessuna sovrapposizione richiesta-risposta possibile, con o senza `flush()` | — |
| Echo del bridge | escluso: nelle letture normali `rx_bytes=129` esatti, non 137 | — |
| Wi-Fi AP | già attivo il 17/10 in sessioni senza guasti | non è una novità di fine ottobre |

La correlazione temporale con `8e20c63`, cioè nessun guasto prima e tutti dopo, è quindi **senza meccanismo sul filo**. Va trattata come coincidenza o come effetto di un cambiamento non firmware dello stesso periodo.

### 3.3 Riproduzione offline

`MATDOG_1_0_2_DIAG8530_MOTION_HW_20261115/bms_rca/repro`: il `DalyBms` di produzione con lo stub UART esistente, ASan e UBSan.
- **Scenari:** silenzio prolungato (218 poll), risposta 128/129 (primo byte mancante, ultimo mancante, ultimo tardivo), risposte tardive, 10 cicli di guasto e rientro.
- **Esito:** `DALY_OUTAGE_REPRO PASS`. Il driver non si blocca, non si desincronizza, resta fail-closed e rientra al primo frame completo.

### 3.4 Esito

- **Causa UNKNOWN, circoscritta:** la divergenza è a valle della FIFO TX e a monte della FIFO RX dell'ESP32, cioè pin, bridge, bus o BMS.
- **Nessun difetto firmware dimostrato, nessuna patch.** Non ho alzato timeout, non ho aggiunto reset e non ho toccato BMS, KEY o MOS.
- **Misura definitiva:** [`LOGIC_ANALYZER_PROCEDURE.md`](LOGIC_ANALYZER_PROCEDURE.md). GPIO15 (classe 1), RO di un ricevitore RS485 in sola ascolto sul bus (classi 2–3) e GPIO16 (classe 4) durante un guasto. A/B col traffico Wi-Fi (H2) e conteggio dei byte del frame 128/129 sul bus e al pin. `@BMS STREAM ON` dà il D1 di ogni poll senza transazioni aggiuntive.

## 4. Locomozione (fase C)

**Base di misura.** Gemello MuJoCo con `fw_bridge` compilato sui sorgenti firmware di `cfc788a` (`loco/build_base`, `SOURCE.txt`).
- `vnext_eval.py` è invariato, così come il modello di contatto `dev7_ground`.
- μ 0,8 di riferimento, con sensibilità a 0,6 e 1,0.
- Legge FAST presa dalla tabella del firmware (2,5 s, 80 mm, 105 mm).
- Ogni numero è **VERIFIED OFFLINE nel gemello**: il modello attuatore è identificato sulla dev.7, l'attrito del piede non è validato, il cedimento statico del LOWER è nella banda HW.

### 4.1 Baseline attuale: velocità comandata contro realizzata (μ 0,8)

| Scenario | Legge | Gemello | % | Moto indesiderato | Escursione laterale corpo p2p (gemello / legge) | Roll / pitch p2p | Clearance p10/p50/p90 | Swing trascinato |
|---|---|---|---:|---|---|---|---|---:|
| fwd_slow | 20,0 mm/s | 9,7 | 49 % | — | 8,2 / 3,7 mm | 29 / 25 mrad | 1,4 / 1,6 / 2,5 mm | 54 % |
| fwd_standard | 30,1 | 19,7 | 66 % | — | 7,6 / 2,7 | 29 / 26 | 0,9 / 1,2 / 3,2 | 46 % |
| fast_fwd | 32,0 | 21,9 | 68 % | yaw −0,16 °/s | 10,8 / 2,1 | 18 / 24 | 0,0 / 0,8 / 1,3 | 55 % |
| back_standard | −16,6 | −15,1 | 91 % | — | 2,5 / 1,6 | 32 / 19 | 2,1 / 2,3 / 2,5 | 48 % |
| fast_back | −20,0 | −18,1 | 90 % | — | 2,0 / 1,1 | 30 / 21 | 0,1 / 0,3 / 0,8 | 52 % |
| left_slow | 4,0 | 2,6 | 65 % | yaw +0,30 °/s | 25,1 / 23,8 | 41 / 29 | 0,5 / 1,8 / 4,1 | 49 % |
| left_standard | 6,7 | 4,2 | 62 % | yaw +0,45 °/s | 22,9 / 21,6 | 45 / 36 | 0,6 / 1,5 / 3,7 | 52 % |
| fast_left / fast_right | 4,0 / −5,6 | 1,4 / −2,9 | 36 / 51 % | yaw ±0,4 °/s | 17 / 17 | 30 / 24 | 0,4 / 0,6 / 1,2 | 62 % |
| yaw_standard | 2,29 °/s | 1,49 | 65 % | 1,1 / 1,4 mm/s | 22,9 / 20,0 | 50 / 29 | 0,8 / 1,8 / 3,8 | 53 % |
| fast_yaw | 1,83 °/s | 0,92 | 50 % | 0,7 / 0,6 mm/s | 17 / 17 | 31 / 21 | 0,4 / 0,6 / 1,4 | 64 % |
| diag_standard | 13,7 (x) | 7,7 | 56 % | — | 22,8 / 21,5 | 42 / 34 | 0,5 / 0,8 / 2,0 | 59 % |
| fast_diag | 16,0 (x) | 7,7 | 48 % | — | 20,9 / 16,8 | 29 / 24 | 0,3 / 0,4 / 0,5 | 71 % |
| modes_cycle (S→STD→F) | 21,2 | 13,5 | 64 % | — | 12,5 / 3,3 | 39 / 45 | 0,1 / 1,4 / 3,2 | 55 % |

- **Attrito:** fwd_standard vale il 90 % a μ 0,6 e il 47 % a μ 1,0; fast_fwd il 90 % e il 39 %. Il vincolo è il trascinamento: più attrito frena di più il piede che striscia.
- **Fault e rifiuti:** nessuno in alcuno scenario.

### 4.2 Prima divergenza: controfattuale `swing_leg_contact=0` sulla legge attuale

Solo diagnostico, mai una configurazione: tutta la parte distale della zampa in swing non collide col suolo.

| Scenario | Base | Senza contatto in swing | Escursione laterale corpo |
|---|---:|---:|---|
| fwd_slow | 49 % | **98 %** | 8,2 → 4,3 mm |
| fwd_standard | 66 % | **96 %** | 7,6 → 4,1 mm |
| fast_fwd | 68 % | **93 %** | 10,8 → 6,7 mm |
| left_standard / yaw_standard / fast_left | 62 / 65 / 36 % | **non valido**: il robot si ribalta (roll p2p fino a 2π) | — |

**Lettura.** La perdita di velocità utile viene dal **contatto della zampa in swing col suolo**, non dalla cinematica della legge. È la causa già verificata sul gemello della dev.7: gli appoggi solo-P cedono 10–20 tick sotto carico, il treppiede scende verso l'angolo in swing e la zampa in swing tocca, trascina e frena. Ora è confermata sulla legge `539b646`, cioè con Patch A, rampa FAST e B2.

### 4.3 C3 — FAST 40 mm/s

Le quattro velocità da distinguere:

| Velocità | FAST attuale (32) | Legge F40 (dev.7, 13/11) |
|---|---|---|
| Richiesta / comandata | 32 mm/s (80 mm in 2,5 s) | 40 mm/s |
| Realizzata nel gemello (μ 0,8) | 21,9 mm/s (68 %) | 32,6 mm/s (81 %) |
| Gemello senza contatto in swing | 29,8 mm/s (93 %) | — |
| Reale sul terreno | **UNKNOWN**, mai misurata | **UNKNOWN** |

**Vincolo dominante (offline, 14/11, confermato).** Alla frenata qualificata (ACC 84), lo STOP sincrono dura 0,16–0,30 s, fissato dall'UPPER più veloce. In tutte le 396 configurazioni a 40 mm/s che rispettano supporto, ZMP, cattura e drive, LF_HIP, LF_UPPER o LH_HIP escono dal box certificato.

**Perché cambiare falcata non basta.** In una camminata a quattro tempi con duty d, la velocità media del piede in swing vale v/(1−d), qualunque sia la falcata:
- d 0,825 → 183 mm/s a 32 mm/s e 229 mm/s a 40 mm/s (+25 %);
- lo spazio percorso durante l'arresto cresce come v² (+56 %);
- abbassare il duty verso 0,75 ridurrebbe la velocità dello swing (160 mm/s) ma annulla le finestre a quattro appoggi in cui il COM si sposta. Con la legge attuale, ogni 0,5 mm di sway in meno costa circa 0,37 mm di margine di supporto e di cattura (§4.4).

**Leve che cambierebbero il vincolo, nessuna disponibile oggi:**
1. una frenata sotto carico più alta, qualificata sull'HW;
2. un margine HIP maggiore, che richiede box nuovi e quindi una nuova prova CAD;
3. una legge con swing più lento a parità di velocità del corpo, che richiede duty più basso e quindi margine di supporto che oggi manca.

Il what-if ad ACC 150/220 (14/11) non trova configurazioni che rispettino insieme supporto, cattura, HIP e drive.

**Esito: FAST 40 non fattibile nelle condizioni attuali.** Capacità credibile: **circa 30 mm/s utili con la legge FAST attuale**, se si elimina il contatto della zampa in swing (§4.5). Oggi la velocità utile non è limitata dalla legge ma dal trascinamento del piede.

### 4.4 C1 — sway del corpo nella marcia frontale

Supporto statico e cattura allo STOP contro sway laterale comandato (gate `walk_support_check`, modello rigido, legge attuale):

| Modo | Sway (ampiezza) | Supporto min | Cattura STOP min |
|---|---|---:|---:|
| SLOW | 1,5 mm (attuale) | 2,12 mm | 1,41 mm |
| SLOW | 0 | 1,00 | 0,31 |
| STANDARD | 1,2 mm (attuale) | 1,99 | 1,51 |
| STANDARD | 0 | 1,05 | 0,59 |
| FAST | 1,0 mm (attuale) | 1,71 | 1,41 |
| FAST | 0 | 0,96 | 0,67 |

- **Spostamento minimo del COM:** con tre appoggi la legge attuale è già allo sway minimo compatibile con il criterio di cattura usato per il FAST (≥ 1,41 mm, il margine di SLOW). Ogni 0,5 mm in meno costa circa 0,37 mm di supporto e di cattura.
- **Il dondolio non è comandato:**
  - nel gemello l'escursione laterale del corpo vale 7,6 mm p2p in STANDARD, contro 2,7 mm di legge;
  - senza contatto in swing scende a 4,1 mm; il resto viene dal cedimento degli appoggi;
  - roll e pitch p2p valgono 18–29 mrad in avanti.
- **Esito:** ridurre lo sway comandato peggiorerebbe margini qualificati per togliere al massimo circa 1 mm di un dondolio da 7,6 mm. **Non lo propongo.** La riduzione utile passa dalla clearance (§4.5).

### 4.5 C2 — clearance effettiva e cedevolezza

- **Clearance mediana effettiva** (legge attuale, μ 0,8): 1,6 mm (SLOW), 1,2 mm (STANDARD), 0,8 mm (FAST), con piede trascinato nel 46–55 % dello swing. Il `liftM` di legge è 6–8 mm.
- **Leva verificata nel gemello** (13/11, base dev.7, `REPORT_VNEXT_LOCO.md` §4): compensazione della cedevolezza in appoggio, `ref = IK − C·JᵀF` su UPPER e LOWER, più una sollevata del corpo durante lo swing di 2–3 mm (V6f20/V7). Con questa leva l'avanti arriva al 99–101 % e la clearance al picco vale 5,5/4,6 mm in SLOW e 1,8/0,5 mm in FAST.
- **Perché non è integrabile oggi:**
  1. **C sull'hardware è UNKNOWN:**
     - dalle HOLD esiste solo una stima condivisa, C ≈ 0,036 rad/N·m, nell'intervallo [0,031; 0,039] solo se l'attrito è ≤ 0,27 N·m;
     - UPPER, LOWER e HIP separatamente sono UNKNOWN;
     - con 2/3 di C il gemello perde il 27 % in SLOW.
  2. **Gate rigidi:** il test host `WalkBoardPlant` e il gate joystick geometrico modellano un robot rigido. Con la compensazione il riferimento comandato si discosta dalla posa reale prevista di 10–20 tick, quindi i gate falliscono per costruzione. **Proposta di correzione del criterio** (non applicata):
     - i gate di box e prova STOP devono valutare sia il riferimento comandato (limiti servo) sia la posa reale prevista con C misurata (collisione e supporto);
     - va decisa e qualificata dopo la misura di C.
  3. **Porting:** il generatore delle patch vNext si ancora al testo dev.7 + F40T, e il `Gait.h` attuale non ha quelle ancore. Il principio va portato a mano su `539b646` senza la riga FAST 40.
- **Identificabile dai log esistenti:** solo la C condivisa. Non ci sono log ad alta frequenza con riferimento e retroazione sincroni sotto carico per giunto: nello stato HTTP `commanded_raw` ha un'età di circa 0,7 s.

### 4.6 C4 e C5 — laterale, diagonale, rotazione, nuove abilità

**Misure separate** (gemello, legge attuale, §4.1):

| Scenario | Comandato | Realizzato | % | Moto indesiderato | Escursione laterale corpo |
|---|---|---|---:|---|---|
| left_standard | 6,7 mm/s | 4,2 mm/s | 62 % | rotazione indesiderata +0,45 °/s | 22,9 mm p2p (21,6 di legge) |
| fast_left | 4,0 mm/s | 1,4 mm/s | 36 % | rotazione indesiderata +0,36 °/s | — |
| yaw_standard | 2,29 °/s | 1,49 °/s | 65 % | traslazione indesiderata 1,1 / 1,4 mm/s | 23 mm p2p |
| fast_yaw | 1,83 °/s | 0,92 °/s | 50 % | — | — |
| diag_standard | 13,7 mm/s in x | 7,7 mm/s | 56 % | — | 22,8 mm p2p |
| fast_diag | 16,0 mm/s in x | 7,7 mm/s | 48 % | — | 20,9 mm p2p |

- **Controfattuale sterzato (`swing_leg_contact=0`): non valido come misura, ma informativo.**
  - Il robot del gemello si ribalta (roll p2p 2π in left_standard e yaw_standard; 150 mrad e piedi a 30–220 mm dal suolo in fast_left).
  - Nei cammini sterzati della legge attuale la zampa «in swing» porta l'8–9 % del peso: con appoggi cedevoli il treppiede da solo non regge il corpo.
- **Prima causa verificabile di sidestep e rotazione poco efficaci:**
  - margine di supporto della legge sterzata insufficiente sotto cedevolezza;
  - contatto della zampa in swing.
- **Perché aumentare `kJoystickLateralM` o `kJoystickYawRad` non risolve:**
  - lo sway sterzato è già al limite della prova STOP: con i termini vNext pieni, il gate joystick va in `STOP_PLAN`;
  - l'escursione laterale del corpo è già di 17–25 mm p2p, quasi tutta di legge;
  - aumentare i passi porta le zampe più vicine ai box sterzati;
  - **non lo propongo.**
- **Strada credibile:**
  - un CRAWL a tre appoggi con margine statico ampio. Prototipo C2 del 13/11 (dev.7): supporto statico 4,61 mm contro 2,1, clearance 9 mm, laterale 77 %, rotazione 80 %, STOP 9/9 nel gemello;
  - come **modo distinto**, senza toccare SLOW, sulla base attuale, dopo la misura di C, perché C2 include la compensazione;
  - sono necessari box sterzati e prove STOP nuove: niente CRAWL senza nuova certificazione.
- **C5, nuove abilità:**
  - sidestep, rotazione sul posto, diagonali e cambi di direzione esistono già come regime sterzato del WALK a quattro tempi: vanno resi efficaci, non duplicati;
  - nessuna nuova capacità implementata in questo mandato;
  - TROT resta fuori: il 13/11 non era fattibile (9,4 mm/s su 40, fault alla WALK STOP) e servo di posizione solo-P non garantiscono stabilità dinamica.

## 5. Integrazione

### 5.1 Commit e sorgenti

| Riferimento | Contenuto | Stato |
|---|---|---|
| `cand/timing-iram-20261115` = **`f349af9`** su `cfc788a` | `5eff171` IRAM G1 (6 funzioni); `e7b4266` attributo senza header, per il gate di purezza; `f349af9` ri-pin documentato dei 3 sorgenti nel gate di convergenza | candidato offline, **non pubblicato sul branch operativo** |
| `cand/diag-8530d3a-iram-20261115` = **`e321e86`** su `8530d3a` | la diagnostica installata più gli stessi 3 commit, cherry-pick puliti | **unico candidato diagnostico integrato, NON installato** |
| `refs/preserve/20261114/diag-8530d3a` | `8530d3a` importato dal bundle verificato | conservazione |
| Branch operativo | solo documentazione (§7) | — |

**Perché l'IRAM non è nel branch operativo.** Le regressioni sono positive (§2.4), ma il beneficio sul chip non è misurato e il costo di heap (−4,6 KB) non è verificato sull'hardware. Il perimetro non è ancora «chiaramente dimostrato» come chiede il mandato.

### 5.2 Differenze rispetto al firmware installato (`8530d3a`)

- **Candidato diagnostico `e321e86`:** sei funzioni Motion in IRAM, stesso codice macchina, stessi frame RAW.
- **Codice invariato:** gait, IK, STOP, StopPlan, box, limiti, calibrazioni, NVS, BMS, Wi-Fi.
- **Memoria:** heap interno −4,6 KB.

### 5.3 Vincoli hardware realmente mancanti

1. PMU sul chip del candidato IRAM: `istall_mem`, overrun, perse, picco, heap minimo.
2. Il filo RS485 e UART durante un guasto DALY.
3. La cedevolezza in appoggio per giunto, sotto carico a terra: prerequisito della clearance.
4. Attrito piede-pavimento e velocità reale sul terreno.
5. Frenata sotto carico sopra ACC 84: solo se si vuole riaprire il FAST 40.

## 6. Piano essenziale del prossimo collaudo

Un'unica sessione, con operatore presente e autorizzazione esplicita.

1. **Accensione a freddo con acquisizione passiva DALY** ([`LOGIC_ANALYZER_PROCEDURE.md`](LOGIC_ANALYZER_PROCEDURE.md)): analizzatore su GPIO15, GPIO16 e bus prima del KEY ON, più `@BMS STREAM ON`. Robot fermo per 20 minuti, con le fasi A/B del traffico Wi-Fi. Se compare un guasto, la classe 1–4 è decisa.
2. **Un flash APP-only di `e321e86`**, con la procedura e gli script di rollback esistenti. Rollback a `8530d3a`, la cui APP è verificata.
3. **Verifica passiva:** boot, identità, heap minimo (atteso circa 4,6 KB sotto il 15/11), D1, SAFE_OFF.
4. **La stessa suite fixture A–G** (`timing_tools/fixture_suite.py`, invariata) e `analyze_diag.py`.
   - Confronto con il 15/11 per classe: `istall_mem`, IPC, muro medio, overrun, perse, picco.
   - Criteri: quelli del 15/11, nessuna regressione funzionale. La riduzione di `istall_mem` va misurata, non presunta.
   - La sequenza parte dopo la prima attivazione (§2.3).
5. **Opzionale, se si autorizza una sessione a terra:** HOLD a 80, 120 e 150 mm con registrazione ad alta frequenza di riferimento e retroazione per giunto, per identificare C per giunto. È la condizione per portare la compensazione della cedevolezza.

## 7. Evidenze e riproducibilità

Cartella `~/MATDOG/verification-artifacts/MATDOG_1_0_2_OFFLINE_DEV_20261115/`. `SHA256SUMS.txt` sigilla tutti i file.

**`timing/`**
- `qrig/`: rig, `run.sh`, `trace.sh`; output `out/{base,cand,final}`.
- `trace_base/`, `trace_cand/`: `icache_sim.txt`, `iram_sets.txt` e `rank.txt`.
- **Esclusi dal sigillo e rigenerabili con gli script:**
  - i `dump.bin` (80 MB ciascuno);
  - le directory `build/` delle compilazioni QEMU;
  - gli alberi esportati da Git (`timing/tree_*`, `loco/v7try`).
- `tools/`: `icache_sim`, `icache_replay` (dev.8), `rank_iram.py`, `eval_iram_sets.py`.
- `qemu18_{base,cand,final}/` e le console.
- `static_audit_cand*.txt`.

**[`LOGIC_ANALYZER_PROCEDURE.md`](LOGIC_ANALYZER_PROCEDURE.md)**

**`loco/`**
- `build_base/SOURCE.txt`;
- `base/mu{06,08,10}/*.json` e `cf/mu08/*.json`, risultati di `vnext_eval.py`;
- `summarize.py`, `run_*_battery.sh`, `walk_support_check_base.txt`;
- `make_v7p_on_current.py`, il tentativo di generazione che si è fermato sugli ancoraggi.

**Build** (`~/MATDOG/build-output/offline-dev-20261115/`)
- `base-cfc788a`, `cand-iram-5eff171`, `cand-iram-final` (`f349af9`), `diag-iram-e321e86`;
- ognuna con ELF, map, manifest e `sdkconfig`.

**Git**
- bundle `~/MATDOG/backups/git/robot-dog-offline-dev-20261115-candidates.bundle`, SHA256 `c1e87724f5af30860d7d77adf90bc440a5c7023d23b66fec3eb8252ec613684c`, prerequisito `cfc788a`;
- branch locali `cand/timing-iram-20261115` (`f349af9`) e `cand/diag-8530d3a-iram-20261115` (`e321e86`);
- ref `refs/preserve/20261114/diag-8530d3a`.

**Registro di lavoro:** `WORKLOG.md`.
