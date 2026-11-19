# MATDOG dev.8 — P0 timing Motion: analisi offline e build diagnostica (12/11/2026)

> **Evidenza storica, recuperata il 19/11/2026.** Il rapporto non descrive lo stato corrente. È superato dalla chiusura di Motion Timing P0 sulla Stable `v1.0.2` (`1314bc4`, 19/11): massimo 12 729 µs su 16 000, 0 overrun e 0 release perse ([ADR-008](../../Architecture_Decisions/ADR-008_Motion_Timing_P0_Closure_and_Freeze.md)).
>
> - Proviene dal commit locale `76c5f8f` del ramo `diag/dev8-p0-timing-20261112`, che non è mai stato pubblicato.
> - Sono importati solo questo rapporto e `extracts/`, identici all'originale salvo questa nota e i riferimenti a `tools/`, resi testo semplice.
> - Gli strumenti `tools/` (banco QEMU, simulatore di I-cache, script di sessione, patch di strumentazione) e la build diagnostica `1.0.2-dev.8` `be9198d` non sono nel repository. Restano nell'archivio Git locale ([EXTERNAL_ARCHIVES §10](../../EXTERNAL_ARCHIVES.md#10-local-worktrees-and-branches-consolidated-on-2026-11-19)).
> - La build `be9198d` non è mai stata installata. La diagnostica installata dopo è la `0526cf6`.

Ripresa dal checkpoint hardware dev.7 dell'11/11: `09_Logs/Validation_Reports/MATDOG_DEV7_HW_2026-11-11/NEXT_SESSION_HANDOFF.md`, `OPEN_ITEMS.md`, `HW_REPORT.md`. Sono nel commit di documentazione `c3b67d0` del branch `integ/motion-vnext-battery-p1-dev7-20261111`, non in questo branch, che parte dal sorgente `b561062`. Nessun flash, push, merge o intervento sul robot in questa sessione. La dev.7 `b561062` resta la baseline hardware installata e il rollback.

Classi: **VH** verificato su hardware (dati dell'11/11); **VO** verificato offline; **UNKNOWN** non determinabile con i dati disponibili.

Strumenti e uscite: `tools/` (tracciatore QEMU, simulatore di I-cache, `analyze_p0.py`), [`extracts/p0_analysis.txt`](extracts/p0_analysis.txt) (tutte le tabelle di questo report, rigenerabili). Tracce grezze QEMU: `~/MATDOG/verification-artifacts/MATDOG_1_0_2_DEV8_P0_DIAG_20261112/` (vedi §9).

## 0. Esito in breve

- **Prima divergenza osservabile:** in entrambe le finestre critiche la prima release oltre 20 ms è una release che porta **sia** la solve nominale **sia** la compensazione IMU completa; la prima release persa segue due release consecutive di questo tipo. Ma il loro lavoro deterministico, misurato sul codice Xtensa reale, è 0,6–1,1 M istruzioni: **2,6–4,7 ms** a 240 MHz con un ciclo per istruzione. Sull'hardware le stesse release hanno impiegato 25–45 ms. La divergenza non è una sezione o una solve in più: è il **costo per istruzione**, circa **25–28 ns (6–7 cicli a 240 MHz)** in tutte le release di camminata e di ripresa.
- **Causa dimostrata (VO + VH):** il calcolo deterministico spiega solo il 15–25 % del tempo a muro di NOMINAL+COMPENSATE. Il resto è un fattore per istruzione: minimo 4–5 cicli, mediano 8–10 cicli, a parità di lavoro contato.
- **Dato ancora mancante (UNKNOWN):** quanta parte di quel fattore è stallo di memoria (cache flash), interrupt, altri task del core Motion o una frequenza reale diversa da 240 MHz. Nessuna misura esistente li separa; serve una misura sull'hardware.
- **Patch minima:** **nessuna patch operativa**, perché la causa non è ancora dimostrata. Ho preparato la **build diagnostica `1.0.2-dev.8`**, oggi `be9198d` (§10: la prima, `290b663`, non avrebbe contato nulla), che misura esattamente quel dato, e lo script APP-only con i pin. Non è installata.
- **Difetto trovato nella sonda esistente** (VO): `MATDOG_DIAG_PREEMPTION` costava ~270 k istruzioni per release, perché `uxTaskGetSystemState` scansiona lo stack libero di ogni task. Corretto nella build diagnostica: ora costa ~1,7 k istruzioni per release.
- **Stato al 12/11/2026 (§12):** il flash diagnostico non è autorizzato. Build operativa e integrazione F40 sono congelate fino al prossimo handoff MuJoCo. dev.7 `b561062` resta installata.

## 1. Provenienza (VH/VO)

| Voce | Verifica | Esito |
|---|---|---|
| Firmware installato | APP SHA256 `28744982…53d4` nel report dev.7 = `build-b561062/artifacts/ROBOT_POWERED-ota0/MATDOG_Controller.ino.bin` = copia nell'archivio (`firmware_identity/SHA256SUMS.txt`) | identici |
| Manifest | `SOURCE_COMMIT=b5610620d0ce…`, `SOURCE_STATE=CLEAN`, `ROBOT_POWERED`, `FW_VERSION=1.0.2-dev.7`, `CPUFreq=240`, `PSRAM=opi` | coerente |
| Branch | `integ/motion-vnext-battery-p1-dev7-20261111` locale = `origin` = `c3b67d0`; dopo `b561062` solo `09_Logs/` (31 file, 0 in `05_Firmware`) | coerente |
| Worktree dev.7 | `~/MATDOG/worktrees/matdog-integrated-dev7-20261111`, pulito | coerente |
| Evidenze | `MATDOG_1_0_2_DEV7_HW_20261111T1849Z.tar.gz`: `sha256sum -c` OK | integre |
| Sorgente analizzato | `git archive b561062` in una directory temporanea | esatto |

Configurazione di memoria e CPU della build installata (sdkconfig della variante `qio_opi` di Arduino ESP32 3.3.11, fatti di configurazione):
- CPU 240 MHz; `CONFIG_PM_ENABLE` non impostato, quindi nessun DFS;
- I-cache **16 KB**, 8 vie, linee da 32 B, condivisa dai due core; D-cache 32 KB; flash QIO 80 MHz;
- libm (`sin`, `cos`, `atan2`, kernel `__ieee754_*`) in **flash** (0x420d…); primitive double software (`__adddf3`, `__muldf3`…) in **ROM**;
- codice Motion in flash: 107 KB (`simple` 47 KB, `motion` 60 KB), libm 11 KB;
- `CONFIG_ESP_WIFI_IRAM_OPT` e `CONFIG_LWIP_IRAM_OPTIMIZATION` disattivati: Wi-Fi e lwIP girano dalla flash sul core 0;
- task del firmware tutti sul core 0 (`md_imu`, `md_network`, `md_http_life`, HTTPD); sul core 1 restano `loopTask` (Motion, priorità 1), `arduino_events`, `ipc1` e IDLE1.

## 2. Prima divergenza osservabile (VH, `usb/http.jsonl`)

Sezioni in µs. «prev» indica la release precedente; work = solve/IK/Newton/FK; meta = ritardo, tempo dalla fine della release precedente, lavoro esterno del loop.

**Yaw STANDARD (comando armato alle 19:44:37):**

| Ora | Record | READ | NOMINAL | COMPENSATE | CONVERT | FRAME | prev NOM/CMP | work | meta |
|---|---|---:|---:|---:|---:|---:|---|---|---|
| 19:44:38 | prima release > 20 ms (25,3 ms) | 1861 | 8776 | 12288 | 1031 | 1177 | 5308 / 7450 | 1/4/4/11 | 550 / 2629 / 1370 |
| 19:44:39 | **prima persa** | 1583 | 9743 | 13043 | 983 | 1176 | **10248 / 13075** | 1/8/12/23 | **21691** / 1513 / 2197 |
| 19:44:41 | massima | 1901 | 14420 | 24351 | 1404 | 1280 | 561 / 18102 | 1/7/11/21 | 20239 / 1281 / 1406 |

**Camminata sterzata STANDARD (19:47:49–19:48:10):** prima persa alle 19:48:01 con ritardo 28,6 ms, dopo una release con NOMINAL 17,4 + COMPENSATE 10,4 ms; massima alle 19:48:03: 43,8 ms = NOMINAL 14,3 + COMPENSATE 24,6 ms (1/7/11/21, raccordo su tre zampe).

**Ripresa STAND_FROM_STOPPED_WALK (19:44:56–19:45:01):**
- prima persa con ritardo 20,5 ms già alla seconda release;
- la prima release segue 25,8 ms di lavoro esterno del loop (ammissione di STAND con solve del gait, fuori dalla release);
- massima 45,5 ms = NOMINAL 18,3 + COMPENSATE 22,9 ms.

**Sezioni di I/O, stabili** (tutti i record e le release precedenti):
- READ: mediana 2,1 ms, p90 3,8 ms;
- FRAME: 1,17–1,34 ms;
- CONVERT: mediana 1,2 ms.

L'attesa UART non cresce con il problema.

## 3. Calcolo effettivo contro tempo a muro (VO + VH)

### 3.1 Ricostruzione delle release critiche

- Le due sequenze sono state rigiocate sul **codice di produzione di `b561062`** nel banco esistente (`qemu_motion`), con gli ingressi registrati:
  - altezza 110/120 mm;
  - STANDARD, yaw a fondo scala 2,29 °/s, STOP, 4 s fermo, STAND 120;
  - giro della leva avanti → diagonali → laterale → indietro → avanti;
  - correzione IMU applicata ≈ −37 mrad in pitch con motivo ENVELOPE, come sull'HW;
  - un periodo di ritardo del feedback e cedimento sotto carico.
- **Sezioni misurate in istruzioni:** CCOUNT sotto `-icount shift=3`, × 1,5625. Un orologio di timing diverso solo nel banco: `tools/motion_timing_ccount_clock.patch`.
- **Frame identici:**
  - digest RAW host = Xtensa in QEMU;
  - identici anche con la diagnostica attiva e spenta.
- **Limite del banco:** nello scenario A, dopo la ripresa, WALK_SETTLE non si completa (`NATIVE_TARGET_NOT_REACHED` dopo 15 s), perché l'ingresso di inclinazione è costante. Succede dopo le finestre analizzate e non le tocca. Sull'HW WALK_SETTLE si è completato in 3 s.

### 3.2 Istruzioni per release (NOMINAL + COMPENSATE)

| Segmento | Release | p50 | max | Struttura |
|---|---:|---:|---:|---|
| Yaw STANDARD | 496 | 391 k | 1 099 k | solve nominale nel 50 %, compensazione completa nel 64 %, entrambe nel 15 % |
| Giro della leva STANDARD | 3 198 | 373 k | 1 123 k | entrambe nell'11 % |
| Ripresa STAND_FROM_STOPPED_WALK | 205 | 631 k | 1 106 k | **entrambe nel 100 %** (T1/T2 esclusi in ripresa, come nella RCA D1) |
| Release con entrambe (camminata) | 414 | 633 k | 1 123 k | — |

A 240 MHz e un ciclo per istruzione, il massimo è 4,7 ms. Sull'HW la release con NOMINAL+COMPENSATE minima osservata in camminata supera già i 5 ms.

### 3.3 Un solo fattore per istruzione riproduce l'istogramma HW

Ho applicato alle istruzioni QEMU un solo fattore in ns per istruzione, sommando l'I/O misurato sull'HW (4,55 ms), e l'ho confrontato con `work_hist` dello stesso comando.

| Segmento | Fattore | Cicli a 240 MHz | Distanza L1 dall'istogramma HW |
|---|---:|---:|---:|
| Ripresa | 24,7 ns | 5,9 | **0,10** |
| Yaw | 27,4 ns | 6,6 | 0,32 |
| Giro della leva | 27,9 ns | 6,7 | 0,25 |

L'HW ha più dispersione del modello: vedi §3.4.

### 3.4 Stesso lavoro, tempo diverso

Per classe di lavoro contato, COMPENSATE in QEMU ha una dispersione di ±15 %. Sull'HW, con lo stesso lavoro, varia di un fattore 2,2–2,6:

| Lavoro | HW min / mediana / max | QEMU p50 | ns per istruzione (HW min – mediana) |
|---|---|---:|---|
| 1/4/4/11 | 6,0 / 13,6 / 15,5 ms | 319 k | 18,8 – 42,7 |
| 1/4/5/12 | 6,5 / 13,2 / 17,0 ms | 355 k | 18,4 – 37,2 |
| 1/4/8/15 | 7,1 / 15,8 / 19,6 ms | 413 k | 17,2 – 38,3 |
| 1/7/11/21 | 11,8 / 20,1 / 24,6 ms | 603 k | 19,5 – 33,3 |
| 1/8/12/23 | 13,0 / 23,5 / 26,9 ms | 654 k | 20,0 – 35,9 |

**Cosa ne segue, misurato:**
1. **Pavimento di 4–5 cicli per istruzione.** Una prelazione occasionale non può alzare il minimo: c'è un costo sistematico per istruzione.
2. **Una componente variabile di pari ordine,** non deterministica rispetto al codice: dipende dallo stato della macchina, cioè contesa di cache con il core 0, interrupt o task.

### 3.5 Impronta di codice contro I-cache (VO)

- **Metodo:** log QEMU di ogni blocco eseguito in flash, durante 120 release di yaw e 150 di ripresa, passato a un simulatore di cache LRU a 8 vie con linee da 32 B (`tools/icache_sim.cpp`).
- **Ingresso del simulatore:** solo il flusso di istruzioni del core Motion, senza il traffico del core 0. È un limite inferiore.

| Finestra | Codice flash toccato per release | Miss 16 KB | Miss 32 KB | Miss 64 KB |
|---|---|---:|---:|---:|
| Yaw | 39 KB (p50), 52 KB max | 2 092 (p50), 3 732 max | 915 | 91 |
| Ripresa | 49 KB (p50) | 2 781 (p50), 3 635 max | 1 306 | 138 |

- Il codice eseguito in una release è **2,4–3 volte la I-cache**. È un fatto del percorso di codice. Il costo reale di un miss sull'ESP32-S3 non è misurato.
- Senza misura non lo attribuisco come causa:
  - a ~1 µs per linea, i miss simulati valgono 2–3 ms per release, una parte dei 6–25 ms di eccesso;
  - mancano i miss indotti dal core 0, i miss della D-cache e gli stalli di pipeline (moltiplicazioni e divisioni della double software).

## 4. Causa: cosa è dimostrato e cosa manca

| Ipotesi | Stato | Prova |
|---|---|---|
| Più lavoro numerico del previsto (solve in più) | **esclusa come spiegazione sufficiente** (VO) | istruzioni QEMU per release: 15–25 % del tempo a muro a 1 ciclo per istruzione |
| Struttura che concentra il lavoro (solve nominale + compensazione completa nella stessa release; 100 % in ripresa) | **dimostrata, ma è l'amplificatore e non la causa** (VO + VH) | §2, §3.2 |
| Costo per istruzione 4–10 cicli | **dimostrato** (VO + VH) | §3.3, §3.4 |
| Stallo su cache flash (I e/o D) | **plausibile, non dimostrata** | §3.5; configurazione §1 |
| Interrupt sul core 1 | **non misurata** | — |
| Prelazione da task sul core 1 | **improbabile come causa del pavimento, non misurata** | non sposta il minimo (§3.4); sul core 1 non c'è nessun task del firmware |
| Frequenza reale < 240 MHz | **non misurata** (PM disattivato) | nessun dato HW di CCOUNT |
| Sonda `MATDOG_DIAG_PREEMPTION` esistente | **perturbazione dimostrata** (VO): ~270 k istruzioni per release, ~3,3 ms emulati di finestra in più nell'autotest | corretta nella dev.8 |

**Dato mancante, preciso:** per release, sull'HW, nelle sezioni NOMINAL e COMPENSATE:
- cicli (CCOUNT) contro µs, cioè la frequenza reale;
- istruzioni eseguite;
- cicli di stallo istruzioni e dati;
- numero di interrupt;
- tempo degli altri task del core 1.

## 5. Build diagnostica `1.0.2-dev.8` (preparata, NON installata)

| Voce | Valore |
|---|---|
| Branch / worktree | `diag/dev8-p0-timing-20261112` / `~/MATDOG/worktrees/matdog-dev8-p0-timing-20261112` (da `b561062`, solo locale) |
| Commit | **`be9198dc2ae197f66333d1d967767fd6e0336092`** (CLEAN, ROBOT_POWERED, OTA 0), BUILD_ID `be9198dc2ae1`. Sostituisce `290b663` (§10) |
| APP | 1 765 376 B, SHA256 `babd7e9b43cbe9e73787109c0e3cb994cad275a90a966e3627cd6776acc4610b` |
| Artefatti | `~/MATDOG/build-output/dev8-p0-timing-20261112/build-be9198d/artifacts/ROBOT_POWERED-ota0/` (+ `SHA256SUMS.txt`) |
| Tabella partizioni | `8f756ecb…` = installata |
| Script APP-only | `~/MATDOG/verification-artifacts/MATDOG_1_0_2_DEV8_P0_DIAG_20261112/flash/flash-qualified-app0-1.0.2-dev.8-be9198d.sh` (SHA256 `cf7e3e83…9ddd`), autorizzazione `MATDOG_DEV8_DIAG_FLASH_AUTHORIZED=YES_ROBOT_SUPPORTED`; quello di `290b663` è spostato in `flash/superseded/`, non eseguibile |
| Rollback | script dev.7 esistente (`MATDOG_DEV7_FLASH_AUTHORIZED`), APP `28744982…` |
| Sessione | `PROCEDURE.md`, registratori e `analyze_diag.py` nella stessa directory di verifica |

**Scopo, esattamente:** separare sull'HW, per release e per sezione, i termini del tempo a muro: istruzioni, stalli di memoria, interrupt ed eccezioni, altri task e frequenza reale. Non cambia nient'altro.

**Cosa misura** (`src/simple/DiagBuild.h`, `MotionTiming.h`):
- CCOUNT a ogni cambio di sezione;
- i due contatori di prestazione Xtensa del core Motion, riprogrammati a ogni release con una **rotazione su 7 release**. 7 è primo rispetto a 2 (T2), 6 (lettura di controllo), 12 (aggiornamento del joystick), 25 (stampa IMU ogni 500 ms) e 50 (polling HTTP). I gruppi:
  - 0 — istruzioni / cicli a ogni livello; i cicli devono coincidere con CCOUNT (autoverifica);
  - 1 — stalli istruzioni / stalli dati;
  - 2 — stalli istruzioni su memoria / su moltiplicazione e divisione iterative;
  - 3 — stalli dati su memoria / bolle di pipeline;
  - 4 — interrupt / eccezioni di finestra;
  - 5 — cicli / istruzioni **sopra il livello 0**: ISR, gestori di eccezione, sezioni critiche;
  - 6 — stalli istruzioni / dati sopra il livello 0;
- il tempo degli altri task del core 1, diviso fra task fissi sul core (esatto) e senza affinità (limite superiore);
- somme dall'avvio per classe (WALK, RECOVERY = STAND_FROM_STOPPED_WALK, OTHER) e per gruppo, con µs e contatori di NOMINAL, COMPENSATE e CONVERT, più la release più pesante di ogni classe con tutte le sezioni.

**Lettura:**
- comando `@MOTION TIMING` (righe `MOTION_PMU`, `MOTION_PMU_HEAVIEST`, `MOTION_PMU_TASK`);
- resta **rifiutato con Motion attivo**: va letto a coppie OFF, in PRONE_REST;
- nessun campo nuovo nel JSON HTTP, nella dashboard o nello snapshot.

**Costo misurato:**
- QEMU (`290b663`, stesso numero di letture per release): +1,7 k istruzioni per release (p50; 0,4 % di una release di camminata), su HW ~40 µs;
- frame dello stack del percorso release identici a `b561062`, salvo `switchPart` (32 → 48 B) più `pmuRead` (48 B): al massimo ~100 B sulle foglie, contro 10 572 B minimi liberi della loop task misurati sulla dev.7;
- RAM statica +7,6 KB (`.dram0.data` +4 304 B, `.bss` +3 272 B); `.iram0.text` +620 B.

**Invariati:** legge Motion, IK, STOP/StopPlan, watchdog, box, limiti, tolleranze, deadline, controlli di collisione, NVS, calibrazione, Battery P1, contratti JSON. Sono identici i digest RAW in QEMU e il digest dei frame host (§7).

**Correzione inclusa della sonda di prelazione:**
- i task del core Motion sono elencati una volta per comando, in `arm()` fuori dalle release (`arm()` gira nella loop task: `stand()` e `prone()` arrivano dal router USB e dalla mailbox HTTP, entrambi serviti dal loop);
- una release legge solo i loro contatori, con `vTaskGetInfo` senza scansione dello stack;
- autotest QEMU con un task occupato 3 ms ogni 10 ms sul core Motion: quota 0,266 (`290b663`) e 0,345 (`be9198d`); il valore dipende dall'allineamento in fase, con un atteso di circa 0,30;
- un task creato dopo `arm()` non viene contato.

## 6. Patch minima e confronto prima/dopo

**Patch operativa: nessuna.** La causa del fattore per istruzione non è dimostrata. Una patch scelta ora sceglierebbe tra cache, interrupt e frequenza senza misura, e il mandato lo esclude. Per le stesse ragioni non ho toccato:
- T1/T2 in ripresa, perché cambiano il riferimento RAW (RCA D1);
- periodo, contatori e soglie;
- gait, PID, calibrazione e Battery P1.

**Confronto prima/dopo, solo della build diagnostica rispetto a `b561062`:**

| Voce | `b561062` | `be9198d` diag |
|---|---|---|
| Frame RAW (QEMU, 2 scenari, 7 760 release) | `70c0bd72…` / `af4d46bc…` | identici |
| Istruzioni per release (camminata, p50) | 389,6 k | +1,7 k |
| Costo della sonda di prelazione per release | ~270 k istruzioni (se attivata) | ~1,7 k |
| APP | 1 757 472 B | 1 765 376 B |

**Patch condizionate all'esito della misura HW**, da scrivere solo dopo la misura, una alla volta:

| Esito della misura | Patch candidata | Stima offline |
|---|---|---|
| Stalli istruzioni su memoria ≥ 40 % dei cicli in NOMINAL+COMPENSATE | spostare in IRAM le funzioni più calde (`solveGaitInto`, `contactForwardKinematics`, `contactInverseKinematicsGroundNormal`, `inverseWithModel`, `forwardWithModel`, `pointToWorld`: 14,9 KB) | miss simulati −55 % in camminata (2 065 → 949) e −60 % in ripresa (2 852 → 1 130); con le prime 10 funzioni (26,6 KB) −68/−72 %. Costo: stessa quantità di RAM interna (heap minimo libero attuale 69 KB) |
| Stalli dati su memoria dominanti | tabelle e costanti calde in DRAM | da misurare |
| Frequenza misurata < 240 MHz | correzione della configurazione | — |
| Altri task / interrupt ≥ 20 % del tempo a muro | affinità o priorità del task che interviene | — |
| Stalli bassi e CPI di base ≥ 3 (pipeline, moltiplicazioni e divisioni) | lavoro sulla matematica: fuori dal perimetro P0 minimo | — |

## 7. Test e regressioni (VO)

| Prova | Esito |
|---|---|
| `run_simple_motion_tests.sh` (host, ASan/UBSan) sull'albero diagnostico | PASS |
| QEMU P0, diagnostica attiva / spenta / `b561062`: digest RAW | identici (2 scenari, 7 760 release) |
| QEMU `run_qemu_motion.sh --diag` (autotest della sonda) | quota 0,266 (`b561062`: 0,201, finestra gonfiata dalla sonda) |
| Build `ROBOT_POWERED` CLEAN, layout gate | PASS, `APPLICATION_SIZE_STATUS=OK` |
| Frame dello stack del percorso release | identici |
| Script APP-only dev.8: rifiuto senza autorizzazione; gate a secco su copia troncata prima di esptool | PASS (`WRITE_RANGE 0x010000..0x1be890`, NVS intatta) |
| Script di rollback dev.7: rifiuto e gate a secco | PASS (APP `28744982…`) |
| `run_motion_frame_digest.sh` diag contro `b561062` | identico (17 righe, `diff` vuoto) |
| `static_audit.py` (374 file sorgente) | `STATIC_AUDIT = PASS`; mutanti USB rilevati come previsto |
| `run_servo_plant.sh` (6 seed, 11,1 e 9,6 V, tre modi) | 48/48 righe con 0 fault, STOP 1296/1296, `missed=0` |
| **`be9198d`**: build `ROBOT_POWERED` CLEAN, frame dello stack, identità nell'APP | PASS (§5) |
| **`be9198d`**: QEMU P0 con diagnostica attiva, digest RAW | identici (2 scenari, 7 760 release) |
| **`be9198d`**: autotest della sonda di prelazione in QEMU | quota 0,345 |
| **`be9198d`**: script APP-only (rifiuto, gate a secco) e rollback dev.7 | PASS (`WRITE_RANGE 0x010000..0x1bf000`) |
| **`be9198d`**: `static_audit.py` | `FAIL (1 finding)`: `IMU_SEQLOCK torn=1` in `IMU_FEED`, test host TSan multithread. **Non è una regressione:** sorgenti IMU e test identici a `b561062`; su 10 ripetizioni letture inconsistenti in 7 su `b561062` e in 2 su `be9198d`. Tutte le altre suite passano. Problema preesistente. **Verifica mirata in §11:** nessuna lettura con dati mescolati; il test conta un campione coerente più vecchio di una pubblicazione (design a due slot), innocuo per Motion |

## 8. Criteri per l'hardware

### 8.1 Sessione diagnostica dev.8 (solo con l'autorizzazione dell'operatore)

La procedura completa è in `~/MATDOG/verification-artifacts/MATDOG_1_0_2_DEV8_P0_DIAG_20261112/PROCEDURE.md`. In breve:
1. Robot sostenuto in PRONE_REST, SAFE_OFF 8191, coppie OFF; autorizzazione esplicita dell'operatore.
2. Flash APP-only; boot con watchdog-reset e registratore USB aperto subito; banner `1.0.2-dev.8` / `be9198dc2ae1`; preflight 8/8.
3. `@MOTION TIMING` di base, accodato in automatico a coppie OFF: nessuna riga `MOTION_PMU`.
4. A terra, dalla dashboard:
   - STAND 110;
   - STANDARD yaw orario a fondo scala per 10 s;
   - STOP; 4 s fermo;
   - STAND 120 (ripresa) fino a STAND_REACHED;
   - la stessa cosa con yaw antiorario;
   - PRONE fino a PRONE_REST.
5. `@MOTION TIMING` finale, accodato in automatico.
6. Analisi con `analyze_diag.py`; rollback a `b561062`.

Durante la prova la sessione non invia comandi Motion. Il registratore accetta solo `@MOTION TIMING` e `QUIT`.

### 8.2 Regole di decisione

Le applica `analyze_diag.py` alle classi WALK e RECOVERY, sezioni NOMINAL+COMPENSATE, sul delta fra due dump.

**Autoverifiche:**
- cicli PMU a ogni livello / CCOUNT tra 0,98 e 1,02, altrimenti la configurazione dei contatori non è validata e il verdetto è INCONCLUSIVE;
- clock CCOUNT/µs tra 238 e 242 MHz, altrimenti la causa è la frequenza;
- istruzioni per release tra 0,7 e 1,3 volte i riferimenti QEMU: medie WALK 0,40–0,44 M, RECOVERY 0,58–0,67 M.

**Scomposizione dell'eccesso** (cicli che non emettono istruzioni = 1 − IPC), in quote dei cicli:
- stallo istruzioni su memoria (gruppo 2);
- stallo dati su memoria (gruppo 3);
- moltiplicazioni e divisioni iterative (gruppo 2);
- bolle (gruppo 3);
- cicli sopra il livello 0 (gruppo 5);
- altri task, fissi e senza affinità.

**Verdetto:**
- **causa dominante** se una voce vale almeno metà dell'eccesso; la patch si sceglie allora dalla tabella del §6;
- altrimenti «nessuna causa singola».

Avvertenza: non è documentato come l'ESP32-S3 esponga al core i miss della cache esterna. Se gli stalli di memoria risultano ~0 mentre i cicli restano molto sopra le istruzioni e i cicli PMU coincidono con CCOUNT, il tempo si perde in un modo che i contatori non classificano. Il verdetto è allora «nessuna causa singola» e non si sceglie una patch.

### 8.3 Qualifica del risultato (la futura patch, dev.9)

Stessa sequenza a terra della dev.7, nella stessa configurazione (dashboard aperta):
- STANDARD sterzato con yaw, almeno 10 s per verso;
- almeno tre STOP in camminata, di cui uno in yaw e uno nel giro della leva, ciascuno seguito da STAND_FROM_STOPPED_WALK fino a STAND_REACHED;
- giro completo della leva per almeno 60 s.

**Non si escludono le riprese:** contano tutti i comandi armati della sequenza.

**PASS solo se tutte le condizioni sono vere:**
- per ogni comando (`command_timing`): `missed = 0`, `overruns = 0`, `work_hist` vuoto da 20 ms in su;
- `max_work_us ≤ 16 000`, cioè almeno il 20 % di margine sui 20 ms;
- nell'intera sequenza `missed_releases` e `overrun_releases` dall'avvio invariati;
- nessun `MOTION_DEADLINE_LOST`;
- `max_gap_us` invariato;
- ogni STOP verificato con coppia ON nei tempi della dev.7 (2–3 s); ogni ripresa a STAND_REACHED; nessun fault né SAFE_OFF;
- offline PASS: `test_simple_motion`, `servo_plant`, `static_audit`, digest dei frame identico alla dev.7 (la patch non cambia il riferimento), QEMU, e DOM della dashboard se toccata.

Il periodo di 20 ms, i contatori, le soglie, il watchdog e le protezioni restano quelli della dev.7.

## 10. Revisione pre-hardware (12/11/2026, dopo questo report)

1. **Eventi PMU per ESP32-S3**
   - **Fonte:** `libperfmon` di Arduino ESP32 3.3.11, disassemblata.
   - **Registri** (ERI): PM0 `0x101080`, PMCTRL0 `0x101100`, PMSTAT0 `0x101180`, PMG `0x101000`. Il core ha 2 contatori (`XCHAL_NUM_PERF_COUNTERS`).
   - **Campi di PMCTRL:** KRNLCNT al bit 3, TRACELEVEL ai bit 7:4, SELECT ai bit 12:8, MASK ai bit 31:16.
   - **Semantica:** nell'uso di riferimento di ESP-IDF (`xtensa_perfmon_exec` e la documentazione di `tracelevel`), **KRNLCNT=1 conta solo quando il livello di interrupt è > TRACELEVEL**.
   - **`290b663` non contava:** usava KRNLCNT=1 + TRACELEVEL=15 su tutti i contatori, quindi ogni somma PMU sarebbe stata 0 sul robot. Corretto in `be9198d`: KRNLCNT=0 + TRACELEVEL=15 per contare a ogni livello, KRNLCNT=1 + TRACELEVEL=0 per il solo livello > 0.
   - **Maschere:** `INSN_ALL` = 0x8DFF (categorie disgiunte, un'istruzione contata una volta), `CYCLES` = 0x1, maschere di stallo dall'header `xt_perf_consts.h`.
   - **Controllo sul robot:** i cicli PMU del gruppo 0 devono coincidere con CCOUNT.
2. **Contaminazioni**
   - **Interrupt:** separati con i gruppi 5 e 6, che contano solo sopra il livello 0. Le eccezioni di finestra girano a livello EXCM e vi rientrano: il loro numero è nel gruppo 4.
   - **Altri task:** sul core 1 restano loopTask, arduino_events e ipc1; il timer FreeRTOS non ha affinità. Fissi e flottanti sono separati.
   - **Rotazione:** con 5 gruppi era agganciata in fase alla stampa IMU dei 500 ms, perché l'interrupt USB gira sul core 1. Ora è su 7 release.
3. **`@MOTION TIMING`**
   - **Persistenza:** le somme sono dall'avvio, mai azzerate da `arm()`, STOP, SAFE_OFF o PRONE.
   - **Gate e concorrenza:** il comando è rifiutato con Motion attivo (`engaged()`); in PRONE_REST energized è falso. Il dump gira nella stessa loop task delle release: nessuna release può modificarlo mentre stampa.
   - **Precedente su HW:** il dump esistente ha già funzionato il 30/10/2026 dopo PRONE (valori `%llu` corretti, `CONFIG_NEWLIB_NANO_FORMAT` disattivato) ed è stato rifiutato con Motion attivo il 31/10.
   - **Effetto della stampa:** la stampa ritarda le release a riposo successive. I delta HTTP si prendono quindi prima del dump.
4. **Identità**
   - **Manifest `be9198d`:** CLEAN, ROBOT_POWERED, OTA 0, FQBN pinnato, tabella `8f756ecb…`.
   - **APP:** contiene `1.0.2-dev.8` e `be9198dc2ae1`.
   - **Script APP-only:** rifiuta senza autorizzazione; i gate a secco passano.
   - **Rollback dev.7:** rifiuta senza autorizzazione; i gate a secco passano (APP `28744982…`).
5. **Registratori**
   - **USB:** apertura senza reset, allowlist `@MOTION TIMING` e `QUIT`.
   - **HTTP:** GET a 1 Hz; accoda `@MOTION TIMING` solo a coppie OFF con SAFE_OFF verificato, all'inizio e dopo ogni periodo a coppie ON.
   - **Script di sessione:** ingresso in ROM e boot uguali alla dev.7.
   - **Analizzatore:** collaudato su dati sintetici nel formato esatto del firmware.
6. **Avvio contro prove**
   - **Prove:** una prova è il delta tra due dump; il primo dump è la base.
   - **Copertura:** tutte le release armate della classe sono incluse.
   - **Contatori HTTP dall'avvio:** perse e overrun letti nella finestra fra i dump, con 3 s di guardia dopo ogni dump.
   - **Per comando:** `command_timing` di ogni comando armato della finestra.

**Verifiche ripetute su `be9198d`:**
- build CLEAN;
- frame dello stack;
- digest RAW QEMU identici, con diagnostica attiva: 2 scenari, 7 760 release;
- autotest della sonda (0,345);
- gate degli script;
- `static_audit`: un solo finding, `IMU_SEQLOCK` instabile anche su `b561062` (§7, verifica mirata in §11).

## 11. Verifica mirata del finding `IMU_SEQLOCK torn` (12/11/2026, prima del flash)

**Verdetto (VO):** il seqlock non è difettoso. In tutte le prove nessuna lettura ha restituito dati mescolati. Il test conta come «torn» anche un campione coerente che è **una pubblicazione più vecchio** della lettura precedente (`s.seq<last`). È una proprietà reale del design a due slot, ma innocua per Motion. Il FAIL compare solo con l'host sotto carico. La sessione diagnostica non aggiunge rischi rispetto alla dev.7 installata, che usa lo stesso codice.

**Perimetro:** sono identici in `b561062` e `be9198d`:
- `src/imu/ImuFeed.h` (`SeqSlot`);
- `src/imu/Bno085Imu.*`;
- `src/core/SimpleMotionIntegration.cpp`;
- `scripts/tests/test_imu_feed.cpp`.

### 11.1 Meccanismo, riprodotto in modo deterministico

1. Il lettore (Motion, core 1) carica `cur_`, che indica lo slot A con il campione n.
2. Prima che il lettore legga il `seq` dello slot A, lo scrittore (task `md_imu`, core 0) completa n+1 nello slot B e lo rende corrente.
3. Lo scrittore scrive poi n+2 nello slot A, fino al `seq` pari, ma non ha ancora aggiornato `cur_` (`ImuFeed.h:65–66`).
4. Il lettore trova lo slot A completo e coerente e restituisce n+2.
5. La lettura successiva segue `cur_`, che indica ancora n+1, e restituisce n+1. Dopo l'aggiornamento di `cur_` torna n+2.

Conseguenze:
- Ogni campione restituito è completo.
- La sequenza torna indietro **al più di una pubblicazione**: c'è un solo scrittore ed esiste al più uno slot completo ma non ancora corrente.
- Il commento di `ImuFeed.h` («the latest COMPLETED sample») è più forte di ciò che il codice garantisce.

### 11.2 Prove su host

Strumenti in `tools/imu_seqlock/`, uscita in [`extracts/imu_seqlock_check.txt`](extracts/imu_seqlock_check.txt). Classi:
- **dati mescolati:** i campi di un campione non concordano;
- **indietro di 1:** il campione è coerente ma più vecchio di una pubblicazione rispetto alla lettura precedente.

| Prova | Letture | Dati mescolati | Indietro di 1 | Indietro di più | Esito |
|---|---|---|---|---|---|
| Interleaving deterministico sul `SeqSlot` di produzione | 3 | 0 | 4→3→4 | 0 | riprodotto |
| Classificatore, TSan, 20 run | 8,18 M | 0 | 0 | 0 | 0 avvisi TSan |
| Classificatore, `-O2`, 200 run | 514,7 M | 0 | 12 | 0 | – |
| `test_imu_feed` sezione E, 20 run in sequenza | – | 0 | 0 | 0 | 0 FAIL, 0 avvisi TSan |
| `test_imu_feed` sezione E, 24 run concorrenti su 8 CPU | – | 0 | 12 | 0 | 9 FAIL, 0 avvisi TSan |

Una prima ripetizione concorrente ha dato 14 FAIL su 24, con 22 eventi, tutti «indietro di 1». La frequenza dipende dal carico dell'host: spiega i 7/10 e 2/10 del §7, misurati mentre girava la campagna QEMU.

### 11.3 Sul target (VO, disassemblato dell'ELF `be9198d`)

Estratto: [`extracts/imu_seqlock_target_disasm.txt`](extracts/imu_seqlock_target_disasm.txt), ELF `cd99554c…`.
- **Barriere:** ogni accesso atomico, nel lettore e nello scrittore, è racchiuso da `memw` (seq_cst di GCC su Xtensa). Il protocollo vale quindi anche fra core 0 e core 1.
- **Condizione per l'evento:** il lettore deve restare fermo nella sua finestra di 8 istruzioni, tra il caricamento di `cur_` e quello di `seq` (`0x4201e87b–0x4201e896`), per almeno un'intera pubblicazione Rotation Vector. Le pubblicazioni arrivano a 50 Hz, ognuna con un proprio trasferimento SHTP. Il lettore deve poi riprendere nella finestra di 7 istruzioni dello scrittore, tra lo store di `seq` e quello di `cur_` (`0x4201fa1e–0x4201fa31`).
- **Frequenza attesa:** rara. Non è misurata sul robot.

### 11.4 Effetto su Motion se accade

- **Freschezza:** la lettura «indietro» restituisce il campione corrente, con la sua età normale. I limiti di 200 ms (ammissione) e 600 ms (`IMU_STALE`) non cambiano. Motion non usa `seq` (`SimpleMotionIntegration.cpp:164–176`).
- **Compensazione:** per una release vede l'assetto del Rotation Vector precedente (un periodo di 20 ms prima), poi di nuovo il più recente. La correzione è proporzionale (guadagno 0,5, zona morta 5 mrad, limite ±20 mrad), limitata in slew e senza integrale (`SimpleMotion.cpp:1404–1420`). L'effetto è dello stesso ordine del normale jitter fra IMU e Motion, entrambi a 50 Hz e asincroni.
- **Tempo:** `read()` resta limitata a 8 tentativi. Il timing non cambia.

### 11.5 Decisione

**Compatibile con una breve acquisizione diagnostica supervisionata** (§8.1). Non serve alcuna modifica prima del flash.

**Per la prossima candidata operativa** (non ora: serve solo ad avere `static_audit` verde). Due opzioni, entrambe verificate offline (`tools/imu_seqlock/mono_check.cpp`, `prod_fail.cpp`, in coda a `extracts/imu_seqlock_check.txt`):
- **(a) Rilettura di `cur_` dopo la copia:** le letture diventano monotone (0 indietro, 0 dati mescolati). Però sotto lo stesso stress host fallisce più spesso: 24 letture senza campione su 289 M, contro 7 su 829 M di `read()` attuale; circa 10 volte tanto, e un run precedente dava 18/309 M contro 2/759 M. A 50 Hz sul target sono trascurabili entrambe. Una lettura fallita a coppie ON è un fault `IMU_STALE` (`SimpleMotion.cpp:1627`). Il contatore `snapshotReadFailures()` non è esposto, quindi non esiste un dato hardware.
- **(b) Nessuna modifica al firmware (consigliata):** il test e il commento di `ImuFeed.h` dichiarano il contratto reale (mai dati mescolati, al più un campione indietro), e un test deterministico copre l'interleaving del §11.1. Il test continua a rilevare dati mescolati e regressioni di più di un campione.

## 12. Stato di coordinamento (12/11/2026, decisione dell'operatore)

**Il flash di `be9198d` non è autorizzato.** DIAG_FLASH_READY (§11.5) resta un esito tecnico, non un'autorizzazione. La causa del fattore per istruzione resta **UNKNOWN** (§4). Nessuna analisi offline separa stallo di memoria, interrupt e frequenza: serve la sessione §8.1.

**Conservato:**
- **dev.7 `b561062`**: installata, baseline hardware e rollback. APP `28744982…53d4` ricontrollato. `b561062` è sul branch remoto `integ/motion-vnext-battery-p1-dev7-20261111` (`c3b67d0`).
- **diag `be9198d`**: APP `babd7e9b…610b`, `SHA256SUMS.txt` della build OK. Script di flash e di rollback invariati. In `PROCEDURE.md` della sessione c'è solo una nota «SOSPESA».
- **Ref locali** `refs/preserve/20261112/`: `dev7-installed-b561062`, `dev8-diag-be9198d` e `dev8-docs-*`, l'ultimo per i commit di documentazione, incluso questo. I commit dev.8 sono solo locali, senza push.
- **Copia d'archivio**: `MATDOG_1_0_2_DEV8_P0_DIAG_20261112.tar.gz` con `.sha256`. Contiene la sessione, gli artefatti della build `be9198d` e un bundle git `b561062..` del branch.

**Congelato fino al prossimo handoff MuJoCo:**
- la creazione della nuova build operativa (dev.9);
- l'integrazione della F40 attuale: MuJoCo lavora ancora su sollevamento del piede, cambi di modalità, laterale e yaw;
- la patch IRAM e l'opzione (b) di §11.5;
- l'aggiornamento del piano e di `OPEN_ITEMS` con la candidata.

**Obiettivo finale invariato:** una candidata integrata con Motion a 20 ms secondo i criteri di §8.3, FAST 40 mm/s qualificato e gait migliorato. SLOW 20, STANDARD 30, STOP e calibrazioni restano invariati.

**Analisi offline nel frattempo**, solo P0 e senza sovrapposizione con MuJoCo:
- **Fatta, spazio della patch IRAM candidata (§6)**:
  - Nell'ELF `be9198d` le sei funzioni misurano 15 393 B, literal pool esclusi.
  - L'IRAM usata è 83,7 KB su 358 KB della regione condivisa, quindi il linker non è il limite.
  - Il costo è 1:1 sull'heap interno, perché la DRAM comincia dopo l'IRAM. Con il minimo libero dev.7 di 69 KB restano circa 54 KB con sei funzioni e circa 42 KB con dieci.
  - Il margine si giudica sul minimo heap misurato nella sessione diagnostica, prima di scegliere quante funzioni spostare.
- **Escluso ora**: tutto ciò che dipende dalla legge F40, cioè passo, swing, laterale, yaw e cambi di modalità.
- **Al handoff MuJoCo**: tracciatore QEMU (§3.2) sulla legge consegnata, con istruzioni per release in camminata, ripresa, STOP e cambi di modalità. Il risultato va confrontato con la base dev.7 (§3.2) e con il budget di §8.3, prima di integrarla.

## 9. Archivio e cosa non è stato fatto

- **Tracce grezze QEMU** in `~/MATDOG/verification-artifacts/MATDOG_1_0_2_DEV8_P0_DIAG_20261112/qemu/`, con `SHA256SUMS.txt`:
  - per release, su `b561062`, con la diagnostica attiva e con la diagnostica spenta;
  - il dump degli accessi di I-cache (79 MB);
  - le uscite del simulatore.
- **Non fatto:**
  - nessun flash, push, merge o modifica al robot;
  - nessuna modifica a gait, PID servo, calibrazione, Battery P1, periodo, contatori o soglie;
  - la dev.8 diagnostica non è un candidato operativo;
  - la dev.7 resta installata e resta la baseline.
