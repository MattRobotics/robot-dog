# Chiusura strutturale del Motion Timing P0 — sviluppo offline, 15/11/2026

> Copia nel repository del rapporto. Le prove sono locali, non nel repository: `~/MATDOG/verification-artifacts/MATDOG_1_0_2_P0_STRUCTURAL_20261115/` (i percorsi `A_iram/`, `B_t2/`, `D_d2/` sono relativi a quella cartella), sigillate in `SHA256SUMS` (sha256 `e7050ce84ccae8ed29a1ca34e7765ea3b46f6280eade3fa56bb2c4fd96a820ca`). Codice di ricerca e diagnostiche: branch locali, bundle `backups/git/robot-dog-p0-structural-20261115.bundle` (sha256 `ccc33c0e…`).

Mandato offline: nessun flash, nessun Motion. Il robot è rimasto acceso con la diagnostica `e321e86` installata (PRONE_REST, SAFE_OFF) e non è stato toccato.

Le istruzioni Xtensa contate dal rig QEMU **non sono millisecondi**: le proiezioni sull'hardware sono un modello, dichiarato come tale. **P0 non è chiuso** e non può esserlo su base simulata.

## Esito

| # | Punto | Esito | Stato |
|---|---|---|---|
| 1 | IRAM G1 sul branch operativo | merge `f3b0d48` di `f349af9`, patch identica byte per byte a quella di `e321e86`; build CLEAN `f3b0d48cd56d`, sezioni identiche; `static_audit` PASS; docs `0c57b1b`, push; `main` invariato | VERIFIED OFFLINE + HW (tramite `e321e86`) |
| 2 | RCA del calcolo residuo | ogni picco WALK/RECOVERY è la somma di: una soluzione nominale (~850 k istr.), una compensazione piena con IK sul bordo del piede sotto `ENVELOPE` (~1 060 k), `compensationSafe` (~200 k) e fit (84 k). Totale ~2,25 M istruzioni. In RECOVERY la coincidenza è sistematica (soluzione esatta a ogni release); in WALK solo mentre la correzione scorre | VERIFIED OFFLINE |
| 3 | Candidato migliore | **T2R1M** `0fb22b0`: 4 istruzioni di logica in `SimpleMotion.cpp`, nessuno stato nuovo. La ripresa usa il bracket T2 esistente; la release leggera T1 vale anche mentre la correzione scorre (WALK e ripresa). WALK max 2 255 → 1 430 k istr. (−37 %); RECOVERY media 1 214 → 727 k (−40 %), p99 1 840 → 1 239 k (−33 %), max 2 278 → 2 156 k (−5 %) | IMPLEMENTED NOT VALIDATED |
| 4 | Regressioni | 64 scenari host: stesse sequenze stato/causa, stessi esiti (47 STAND_REACHED, 1 fault previsto), 0 NATIVE_TARGET_NOT_REACHED, nessun gate o limite toccato. Costi misurati: errore IMU medio in WALK +18 % (massimo invariato); seconda differenza del RAW in ripresa 8 → 11 tick; posa tenuta dopo STOP fino a 22 tick diversa (stessa durata); in 47 scenari 1 release `WORKSPACE` in più alla fine della ripresa | VERIFIED OFFLINE |
| 5 | Memoria e costo | nessuno stato nuovo (header invariati); build: solo `.flash.text` +56 B, IRAM/DRAM/BSS identiche; stack minimo del task Motion nel rig invariato (7 392 B) | VERIFIED OFFLINE |
| 6 | Target 16 ms | **non raggiungibile con T2/T1**. Il budget del modello è ~1,0–1,27 M istruzioni per release. Restano sopra: compensazione piena con IK di bordo (~1,4 M); release di resync (~2,0–2,16 M); seconda release dopo il resync (~1,8 M); ALIGN/STAND_DEPLOY (~2,0 M) | modello |
| 7 | D2 | candidato distinto `4e3a740` (`8429bc0` + correzione dei test): lettura read-only del registro di stato UART2 per periodo di poll; test host e suite `night_recovery` PASS; build CLEAN APP `1c6eb07c…`; `static_audit` PASS | IMPLEMENTED NOT VALIDATED |
| 8 | Piano fixture | una sola sessione, un solo flash APP-only della diagnostica integrata `46b3175` (build CLEAN, APP `57508746…`, non flashata); criteri dichiarati prima (§8) | proposta |

## 1. IRAM G1 integrato sul branch operativo

**Provenienza.** La patch IRAM di `f349af9` (su `cfc788a`) è identica byte per byte a quella della diagnostica installata `e321e86` (su `8530d3a`). Verificato con `git diff`, righe `index` escluse. L'immagine installata era stata verificata sul dispositivo il 15/11: `build_id e321e8668139`, `verify-flash` con digest uguale ad APP `b88c448d…`.

**Integrazione.**
- Merge `--no-ff` `f3b0d48` di `cand/timing-iram-20261115`: i tre commit originali (`5eff171`, `e7b4266`, `f349af9`) restano nella storia con i loro hash.
- Firmware e script dopo il merge identici a `f349af9`.
- La diagnostica installata resta il branch locale separato `cand/diag-8530d3a-iram-20261115`.

**Verifiche.**
- Build CLEAN `f3b0d48cd56d`, APP `a6baacf5d5abc46fc0deb20a6be1664b89f3631c4b0e9c20fbdf74cd64619b99`, 1 773 888 B, **non flashata**.
- Sezioni identiche alla build di `f349af9` (IRAM 86 595 B, `.flash.text` 1 050 260 B). I 119 byte diversi sono identità e hash.
- `static_audit` PASS (`HOST_TESTS` PASS, gate dei sorgenti Motion PASS).

**Pubblicazione.** Docs `0c57b1b` (CHANGELOG, HANDOFF), push del branch operativo. `main` `6838e4d` invariato.

## 2. RCA del calcolo residuo NOMINAL + COMPENSATE

**Strumento.** Il rig QEMU Xtensa esistente: Motion di produzione, conversioni gen-1 reali, `-icount 3`, ×3,088 = istruzioni. Due aggiunte, solo di misura:
- accumulatori di cicli inclusivi per gruppo di funzioni (`ScratchAcc.h`), inseriti in una copia di misura e mai nel firmware;
- 45 scenari di ripresa, oltre al replay fixture del 14/11:
  - ripresa dopo STOP in 8 istanti del ciclo, per ciascun modo;
  - STOP dentro la ripresa;
  - inclinazione in movimento;
  - cammini sterzati;
  - lag/sag;
  - WALK dopo STOP.

La strumentazione è inerte: il digest del replay fixture è identico a quello offline (`1f96723695b8a124`). Anche i build «shadow», che confrontano con la soluzione esatta, sono inerti: 0 differenze di comportamento su ~154 700 release per run (`B_t2/shadow_inert_check.txt`).

**Baseline `f3b0d48`** (migliaia di istruzioni per release, 46 scenari QEMU):

| Classe | Media | p99 | Max | Composizione del massimo |
|---|---|---|---|---|
| WALK | 784 | 1 580 | 2 255 | soluzione nominale 852 + IK di compensazione **1 067** + `compensationSafe` 176 (di cui prova STOP 153) + fit 84 + invio 14 |
| RECOVERY | 1 214 | 1 840 | 2 278 | 856 + 1 068 + 215 (STOP 167) + 83 + 14 |
| OTHER | 370 | 968 | 2 043 | costruzione del piano di segmento ~1 140 + 2 IK di compensazione ~800 |

**Prima causa: la coincidenza.**
- **RECOVERY:** `walkRecover_` impone la soluzione esatta a ogni release (`nominalRelease`) ed esclude il riuso T1 (`compensateLevel`). Ogni release paga quindi nominale + compensazione piena, più un secondo `compensationSafe` con prova STOP dopo la sfumatura del RAW tenuto.
- **WALK:** il T2 risolve la legge una release su due. Il T1 riusa la correzione sulla release della soluzione **solo se la correzione è assestata**. Quando scorre (inviluppo `ENVELOPE` che cambia nei cammini sterzati), la compensazione piena cade sulla release della soluzione.

**Seconda causa: l'IK di compensazione sul bordo.** Nei cammini sterzati (`edgeTiltRad > 0`) la compensazione usa il modello di contatto sul fillet: 8 IK, 12 passi di Newton, 23 FK per release. Costa ~1 060 k istruzioni, contro una media di 428 k. Con `compensationSafe` e fit arriva da sola a ~1,38–1,44 M istruzioni.

**Terza causa, fuori dal perimetro T2/T1.** In OTHER i picchi sono esattamente 2 release per ogni STAND dalla posa d'ingresso o da PRONE, all'inizio di ALIGN e di STAND_DEPLOY: la costruzione del piano e la compensazione cadono nella stessa release.

**Dove restano i picchi con il candidato** (T2R1M, per posizione dopo l'ultimo resync del bracket T2; `B_t2/resync_position_t2r1m.txt`):

| RECOVERY, posizione | Che cosa paga | Release | Media | Max | > 1,5 M |
|---|---|---|---|---|---|
| 0 | resync: soluzione esatta a `now` + compensazione piena (nessuna correzione riusabile) | 53 | 1 235 | 2 156 | 3 |
| 1 | look-ahead + compensazione leggera | 52 | 607 | 676 | 0 |
| 2 | look-ahead + compensazione piena (la release leggera precedente esclude il riuso) | 52 | 1 231 | 1 808 | 3 |
| 3 | interpolata + compensazione piena | 52 | 773 | 1 311 | 0 |
| ≥ 4, con soluzione | soluzione + compensazione leggera (sempre) | 5 288 | 666 | 1 119 | 0 |
| ≥ 4, interpolata | interpolazione + compensazione piena | 5 345 | 779 | 1 440 | 0 |

- In WALK il massimo (1 430 k) è una release interpolata con compensazione piena sul bordo: il pavimento dell'IK di bordo. Le release con soluzione sono tutte leggere, con un massimo di 1 087 k.
- La posizione 2 dipende dal calendario del T2 esistente: dopo un resync risolve alle release 0, 1, 2, 4, 6…
  - Esiste già in WALK alla partenza, uguale nel baseline (1 023 k di media).
  - Un calendario «pigro» (0, 1, 3, 5…) la eliminerebbe. Non l'ho implementato: non abbassa alcun massimo di classe (il massimo RECOVERY è il resync, quello WALK il pavimento dell'IK di bordo) e cambierebbe il T2 di tutto il WALK. Resta un vincolo documentato (§6).

## 3. Candidati, implementati separatamente

| Candidato | Commit | Modifica | Righe (con commenti) |
|---|---|---|---|
| **T2R** | `bb5520b` | la ripresa usa il bracket T2 esistente. In `nominalRelease` il percorso esatto resta solo per il frame terminale; `cyc()` segue l'orologio di ripresa (`recoverCycles_ + walkClockS(t)/period()`, `t ≥ 0`). La prima release è il resync esatto, lo stesso frame da cui nasce il ponte del RAW tenuto | +11 −4 |
| **T2R1** | `4757fa0` | T2R + T1 in ripresa con la regola della correzione assestata (velocità di slew invariata). La prova STOP della release leggera è quella che `generateWalk` fa già sul target finale | +7 −2 |
| **T1M** | `a62171c` | alternativa separata, su `f3b0d48`: in WALK la release leggera vale anche mentre la correzione scorre; HOLD e ripresa invariati | +9 −1 |
| **T2R1M** | `0fb22b0` | combinazione misurata di T2R1 e della regola T1M, ripresa compresa | merge di due righe |

Nessuno scheduler nuovo, nessuna nuova legge, nessun limite o gate toccato. La regola del 2026-11-08 resta: dopo una release leggera la correzione avanza al massimo di un periodo, per non gonfiare la secante della prova STOP.

Il T1 è stato prima implementato e misurato da solo (T1M, su `f3b0d48`, senza T2R); la combinazione T2R1M è un candidato distinto, misurato a parte. Il diff completo di T2R1M rispetto a `f3b0d48` è +26 −6 righe in un solo file. Le 4 istruzioni cambiate sono: la condizione `reusable`, l'argomento della prova STOP sulla release leggera, la condizione del percorso esatto, la lambda `cyc()`.

**Nota sul soggetto del commit `0fb22b0`.** Il soggetto dice «nessuna release di ripresa o di cammino porta entrambe le soluzioni», ma questo vale solo a regime. La release di resync e la seconda dopo di essa portano ancora soluzione + compensazione piena (tabella del §2). Il commit non è stato riscritto: è referenziato dalle misure e dalla diagnostica integrata.

**Scelta** (criteri del mandato, punto C):

| | T2R | T2R1 | T1M | **T2R1M** |
|---|---|---|---|---|
| WALK media / p99 / max (k istr.) | = baseline | 784 / 1 581 / 2 257 | 721 / 1 150 / 1 430 | **721 / 1 149 / 1 430** |
| RECOVERY media / p99 / max | 1 015 / 1 738 / 2 269 | 754 / 1 388 / 2 272 | 1 215 / 1 854 / 2 282 | **727 / 1 239 / 2 156** |
| Errore IMU WALK media / max (mrad) | 0,784 / 33,14 | 0,784 / 33,14 | 0,925 / 32,97 | 0,926 / 32,97 |
| Errore IMU RECOVERY media / max | 0,520 / 16,46 | 0,474 / 17,05 | 0,507 / 17,16 | 0,494 / 15,45 |
| Shadow T2 in ripresa / T1 in WALK (mrad) | 4,09 / = baseline | 4,09 / 1,88 | 0 / 2,07 | 4,09 / 2,07 |
| STOP: durata / posa tenuta (max diff.) | 0 / 18 tick | 1 / 21 tick | 1 / 22 tick | 1 / 22 tick |
| Logica cambiata | 2 istruzioni | 2 | 1 | 4 |
| Memoria | nessuno stato | nessuno | nessuno | nessuno |

T2R1M dà la riduzione più grande in entrambe le classi, l'errore IMU in ripresa più basso e lo stesso costo di qualità in WALK di T1M. Le modifiche restano semplici e senza memoria. Il costo di qualità è misurato al §4 ed è il prezzo dichiarato della scelta.

## 4. Confronto quantitativo e regressioni

**Metodo.**
- **Host:** 64 scenari (replay P0, 18 scenari di `run()`, 45 di ripresa). Release allineate sull'orologio simulato; baseline contro candidato (`B_t2/cmp_*`, `seg_*`, `imu_track_all.txt`, `shadow_by_class.txt`).
- **QEMU:** 46 scenari (P0 + ripresa) per il costo.

**Esiti e protezioni** (punti del mandato):

| Verifica | Baseline | T2R1M |
|---|---|---|
| Sequenze stato/causa diverse | — | **0** su 64 scenari |
| Esiti finali diversi | — | **0** |
| STAND_REACHED a fine scenario | 47 | 47 |
| Fault | 1 (`SERVO_FEEDBACK_STALE`, scenario di latch previsto) | lo stesso |
| NATIVE_TARGET_NOT_REACHED | 0 | 0 |
| `WALK_IK_OR_WORKSPACE`, `STOP_RECOVERY_PATH_NOT_CLEAR` | 0 | 0 |
| Ripresa dopo yaw (R4 sterzati, R6 ripresa del WALK) | STAND_REACHED | STAND_REACHED, stesse sequenze |

**Target, RAW, traiettorie** (T2R1M contro baseline, tick; 1 tick = 1,534 mrad):
- RAW diverso in 99 966 release su 154 711. Massimo |ΔRAW| 22 tick in ripresa e 35 negli altri casi.
  - È la divergenza ad anello chiuso dovuta a una correzione IMU diversa, non un errore rispetto a un riferimento. L'errore rispetto al riferimento è lo shadow, sotto.
  - Il caso peggiore è `JOYSTICK_HELD_SEQUENCE`: correzione applicata diversa fino a 19,6 mrad.
- Velocità massima invariata: 28 tick/release in ripresa, 31 negli altri casi.
- Seconda differenza in ripresa 8 → 11 tick, altrove 14 → 14. È l'interpolazione T2 in ripresa: T2R da solo dà lo stesso 8 → 11.

**Shadow** (scarto dalla soluzione esatta, mrad):
- T2 in ripresa ≤ 4,09 (baseline 0, che risolve esatto a ogni release).
- T1 in WALK 1,88 → 2,07; T1 in ripresa ≤ 1,59.
- Lo shadow T2 in WALK (57,5 mrad in `JOYSTICK_HELD_SEQUENCE`) è identico nel baseline. È preesistente e fuori perimetro: nei passaggi di joystick il riferimento esatto non è unico.

**Compensazione IMU** (|voluta − applicata|, mrad, norma su roll e pitch):

| Classe | Baseline media / p99 / max | T2R1M media / p99 / max | Differenza |
|---|---|---|---|
| WALK | 0,784 / 7,55 / 33,14 | 0,926 / 9,46 / 32,97 | media +18 %, p99 +25 %, max −0,5 % |
| RECOVERY | 0,521 / 3,22 / 17,02 | 0,494 / 3,16 / 15,45 | media −5 %, max −9 % |
| OTHER | 6,388 / 20,00 / 45,78 | 6,399 / 20,00 / 45,78 | invariato |

- Release WALK con `SLEW` 1 693 → 2 461.
- Frazione leggera WALK 39,7 % → 49,9 %; ripresa 0 → 49,4 %.
- La correzione, mentre scorre, avanza al massimo di un passo ogni due release: è il costo di qualità dichiarato.

**STOP in ogni fase** (R1: 8 istanti × 3 modi; R2: STOP dopo 1–80 release di ripresa; 8 STOP in sequenza; latch di fault):
- Tutti gli STOP finiscono come nel baseline. Durata uguale, con 1 release di differenza in un solo scenario (8 STOP in sequenza).
- La posa tenuta differisce fino a 22 tick (FAST STOP3/STOP7, sequenza 8/8; 14 tick per lo STOP a metà ripresa).
- **Meccanismo**, verificato sullo scenario FAST STOP7: lo STOP estrapola la secante dell'ultimo passo del RAW. Con T1M la correzione contribuisce a quella secante solo se l'ultima release era piena.
  - Giunto 10: secante 21 → 19 tick/release, distanza d'arresto 84 → 67 tick.
  - La prova STOP di `compensationSafe` gira anche sulle release leggere, sulla stessa secante.
- La ripresa dopo lo STOP finisce in STAND_REACHED come nel baseline. Assestamento ±3 release.

**Workspace e collision box.**
- Nessun fault di workspace.
- Limiti IMU `WORKSPACE`: WALK 4 → 10 (FAST, scenario 4) e OTHER 240 → 287.
- **Meccanismo dei +47 in OTHER:** 1 release in `WALK_SETTLING` alla fine di una ripresa, in 47 scenari.
  - Succede quando la compensazione della release terminale della ripresa è leggera: non consuma il tempo di slew. La compensazione piena dell'ingresso in `WALK_HOLD`, nella stessa release, prova quindi un passo.
  - `compensationSafe` lo rifiuta e tiene il target accettato: lo stesso valore che il baseline tiene in quella release.
  - Effetto: correzione applicata identica, causa diagnostica `WORKSPACE` invece di `SLEW`, STAND_REACHED alla stessa release.
  - Non è un gate che fallisce. L'ho lasciato così e documentato.

**Suite e audit.**

| Verifica | Baseline `f3b0d48` | T2R1 | T1M | T2R1M |
|---|---|---|---|---|
| `run_host_tests` e `static_audit` | PASS | PASS (host) | PASS (host) | PASS (`static_audit` completo) |
| `simple_motion`, `stop_support`, `prone`, `night_recovery` | PASS | PASS | PASS | PASS |
| `simple_dashboard` con `jsdom` e `three` disponibili | PASS (contratto 6 112 controlli) | — | — | PASS (6 115) |
| altre 7 suite Motion rimandate da `static_audit` | FAIL | FAIL, identico | FAIL, identico | FAIL, identico |

Le 7 suite falliscono già sul baseline per cause preesistenti: sono test vecchi che compilano sorgenti rimossi (`src/execution/*`, `src/motion/Locomotion.cpp`, `MotionLaw.h`…), `esp_timer.h` o un include generato (`production_simple_servo_port_frame.inc`). Le cause sono identiche riga per riga.

`simple_dashboard` nel runner delle suite fallisce su tutti gli alberi perché `jsdom` non è sul `NODE_PATH`. L'ho rieseguito su copie d'archivio pulite di baseline `0c57b1b`, T2R1M e diagnostica `5672323` (stessi sorgenti firmware di `46b3175`), con `jsdom` e `three` raggiungibili (`B_t2/dashboard/`):
- fixture (88 controlli), DOM (92 comandi), joystick e contratto: PASS su tutti e tre;
- i 3 test in browser reale non sono stati eseguiti (richiedono Chrome);
- i +3 controlli di T2R1M e della diagnostica sono tutti lo stesso controllo per tick («no fault, no STOP», 306 → 309) nello scenario `joystick_dynamic_reverse`. La ripartenza all'indietro dopo l'inversione arriva 3 release più tardi (`contract_check_diff_base_vs_t2r1m.txt`).

## 5. Memoria e costo computazionale

| | `f3b0d48` | T2R1M `0fb22b0` | Differenza |
|---|---|---|---|
| APP (CLEAN, non flashata) | 1 773 888 B, `a6baacf5…` | 1 773 952 B, `da2af466…` | +64 B |
| `.iram0.text` | 86 595 | 86 595 | 0 |
| `.dram0.data` / `.dram0.bss` | 23 820 / 118 096 | 23 820 / 118 096 | 0 |
| `.flash.text` | 1 050 260 | 1 050 316 | +56 |
| Stato nuovo | — | nessuno (header invariati) | — |
| Stack minimo del task Motion (rig) | 7 392 B | 7 392 B | 0 |

Costo per release (QEMU, migliaia di istruzioni; il numero di release sopra 1,2 / 1,5 M istruzioni è conteggiato su tutti i 46 scenari):

| Classe | Baseline media / p99 / max | T2R1M media / p99 / max | > 1,2 M | > 1,5 M |
|---|---|---|---|---|
| WALK | 784 / 1 580 / 2 255 | 721 / 1 149 / 1 430 | 1 265 → 95 | 201 → 0 |
| RECOVERY | 1 214 / 1 840 / 2 278 | 727 / 1 239 / 2 156 | 4 606 → 131 | 326 → 6 |
| OTHER | 370 / 968 / 2 043 | 370 / 968 / 2 045 | invariato | invariato |

**Proiezione sull'hardware.** È un modello, non una misura, calibrato sull'hardware del 15/11 (`e321e86`):
- basso: N+C / 102,5 k istruzioni per ms (IPC 0,40–0,44) + 3,6 ms per release fuori da N+C;
- alto: istruzioni × 16,0 µs/k, il rapporto del picco baseline.

| Classe | Baseline media; max (basso..alto) | T2R1M media; max (basso..alto) |
|---|---|---|
| WALK | 11,0 ms; 25,4..36,1 ms | 10,4 ms; 17,3..22,9 ms |
| RECOVERY | 15,1 ms; 25,5..36,4 ms | 10,4 ms; 24,3..34,5 ms |
| OTHER | 7,0 ms; 23,0..32,7 ms | invariato |

Riferimento misurato su `e321e86` il 15/11: picco 36,11 ms, 323 overrun, 10 release perse.

## 6. Vincoli che impediscono il target di 16 ms

Il budget del modello per una release di 16 ms è: N+C ≤ (16 − 3,6) × 102,5 k ≈ **1,27 M istruzioni** (basso), oppure ≤ **1,0 M** (alto). Restano sopra, con T2R1M:

1. **Compensazione piena con IK di bordo** (cammini sterzati, `edgeTiltRad > 0`): ~1,06 M + `compensationSafe` ~0,2 M + fit 84 k ≈ **1,38–1,44 M** su ogni release interpolata che la porta. È il massimo WALK (1 430 k), cioè 17,3–22,9 ms nel modello. Nessuna redistribuzione T2/T1 la elimina: una release su due deve aggiornare la correzione.
2. **Release di resync del T2** (inizio ripresa, partenza, **ogni release persa**): soluzione esatta + compensazione piena, **2,0–2,16 M** sotto `ENVELOPE`. Sull'hardware ogni release saltata riapre il bracket. In un cammino sterzato una perdita può quindi generare la successiva. È un'inferenza dal codice e dai costi QEMU, da verificare con la strumentazione della prossima sessione.
3. **Seconda release dopo il resync** (calendario T2 preesistente 0, 1, 2, 4…): soluzione + compensazione piena, fino a **1,81 M** in ripresa.
4. **OTHER, ALIGN e STAND_DEPLOY:** costruzione del piano (~1,14 M) + 2 IK di compensazione (~0,8 M) nella stessa release, **~2,0 M** (23–33 ms nel modello). È fuori dal perimetro T2/T1.
5. **Il tempo fuori da N+C** (~3,6 ms per release sull'hardware) consuma da solo un quarto del budget.

Per i 16 ms servirebbe ridurre il costo dell'IK di bordo e dei picchi di resync e di ALIGN/DEPLOY. Sono lavori diversi da T2/T1; nessuno è stato avviato, come chiede il mandato (niente ottimizzazioni casuali).

## 7. Diagnostica DALY D2 (candidato distinto, non installato)

Branch `cand/daly-d2-20261115` = `4e3a740`, da `0c57b1b`: `8429bc0` (D2) + `4e3a740` (correzione dei soli test, sotto). Worktree `~/MATDOG/worktrees/matdog-daly-d2-20261115`.

**Che cosa legge.** A ogni `DalyBms::update()` fa una sola lettura del registro di stato della UART2 dell'ESP32-S3 (`UART2.status`: campi `rxd`, `txd`, `rxfifo_cnt`, `txfifo_cnt`; `soc/uart_struct.h` dell'IDF 5.5). La lettura non ha effetti su FIFO, interrupt o linea. Per ogni periodo di poll (da richiesta a richiesta) conta:
- campioni;
- campioni con RX basso e fronti di RX;
- campioni con TX basso a FIFO TX vuota, almeno 12 ms dopo la richiesta;
- campioni con FIFO RX hardware non vuota, e il suo massimo.

Il periodo concluso si pubblica alla richiesta successiva in due forme:
- come riga finale `d2 …` di `@BMS STATUS`, dopo D1: il parser della guardia la ignora (test aggiunto);
- come chiavi `bms_d2_*` in `/status`.

**Che cosa non fa.**
- Nessuna transazione sul bus, nessuna scrittura di registro.
- Non cambia polling, scadenze, protocollo, configurazione DALY, salute, LED o potenza del Motion.
- Rispetta le regole DALY di `static_audit`: una sola scrittura `bms_uart_`, nessuna altra via UART2.

**Come si legge durante un silenzio (`rx_bytes=0`).** Sono indizi lato ESP, non una localizzazione certa:

| Osservazione | Significato | Limite |
|---|---|---|
| RX sempre alto, FIFO RX sempre vuota | al pin non arriva nulla: compatibile con DALY muto, bus occupato con RO a riposo o ricevitore dello XY-017 disabilitato | non distingue questi casi |
| RX basso per buona parte dei campioni | l'uscita RO tiene la linea bassa: bus o bridge in stato anomalo | non dice quale dei due |
| FIFO RX non vuota mentre `rx_bytes` resta 0 | byte arrivati alla UART ma non consegnati: percorso RX del driver ESP bloccato, che riaprirebbe l'ipotesi firmware | — |
| TX basso a riposo | linea TX dell'ESP non a riposo: potrebbe tenere lo XY-017 in trasmissione | non osserva il bus |

I campioni sono alla frequenza del loop: si vedono solo livelli sostenuti, mentre impulsi brevi e livello del bus RS485 restano non osservabili. D2 non pretende di identificare il punto fisico del guasto. Riduce le possibilità lato ESP e decide se un'ipotesi firmware va riaperta.

**Verifiche.**
- `test_daly_async` con la stand-in del registro: contabilità D2 corretta, nessun byte TX in più, PASS.
- Test del parser della guardia di calibrazione: PASS.
- **Difetto trovato e corretto, solo nei test.**
  - Il difetto è emerso eseguendo le suite Motion rimandate sulla diagnostica integrata: `night_recovery` falliva in compilazione.
  - Causa: l'harness pre-enable (`run_preenable_bus_tests.sh`, eseguito due volte da quella suite) compila `DalyBms.cpp`, ma la stand-in di `soc/uart_struct.h` stava solo in `daly_async_stub`.
  - Il baseline passa la suite. `static_audit` non la esegue (DEFERRED), quindi la build `8429bc0` (APP `11f6af51…`) era passata.
  - Correzione `4e3a740`: la stand-in passa in `uart_status_stub`, sul percorso di include di entrambe le compilazioni. I sorgenti firmware sono invariati; la suite `night_recovery` completa ora passa.
- Build CLEAN `4e3a740c07c1`, APP `1c6eb07ccbf6fc2bc4e920e05efa0532b5bedc7ef9bb500004be558000ac2e39`, 1 774 384 B, **non flashata**. Sezioni identiche alla build di `8429bc0`; 123 byte diversi, cioè identità e hash. Differenze da `f3b0d48`:
  - +496 B in totale: `.flash.text` +292, `.flash.rodata` +208, `.dram0.bss` +48;
  - IRAM invariata.
- `static_audit` PASS (`D_d2/static_audit_4e3a740.txt`).

## 8. Piano della successiva validazione fixture (una sola sessione)

**Immagine.** Un solo flash APP-only della diagnostica integrata `cand/diag-p0-t2r1m-d2-20261115` = `46b3175`, che contiene:
- operativo `f3b0d48` (IRAM G1);
- candidato T2R1M;
- strumentazione Timing dev.8 di `8530d3a`;
- D2, con la correzione dei test (`46b3175` = `5672323` + la stessa modifica di `4e3a740`).

Verifiche:
- build CLEAN `46b31757eb81`, APP `57508746449ff5872aaf144227460e63fdf80f8311d98275c617aee7fbbf80b2`, 1 782 416 B, `1.0.2-dev.8`, **non flashata**; sezioni identiche alla build di `5672323` (119 byte diversi: identità e hash);
- `static_audit` PASS (`D_d2/static_audit_diag_46b3175.txt`);
- suite Motion `simple_motion`, `stop_support`, `prone`, `night_recovery`: PASS (`D_d2/motion_suites_diag_46b3175.txt`);
- `simple_dashboard` PASS (§4, eseguito su `5672323`: stessi sorgenti firmware).

La build precedente `5672323` (APP `ffd5c882…`) è superata: la sua suite `night_recovery` falliva per il difetto dei test del §7.

**Differenza dall'immagine installata `e321e86`** (`D_d2/diag_5672323_provenance.txt`, `D_d2/sections_e321e86_vs_46b3175.txt`):
- nei sorgenti firmware solo T2R1M (`SimpleMotion.cpp`, identico alla patch misurata), D2 (`DalyBms`, `CommandRouter`, `ControllerService`, `HttpTransport`) e 5 righe di commento in `BuildConfig.h`;
- le patch Timing e D2 sono identiche, riga per riga, a quelle di `8530d3a` e `8429bc0`, e la correzione dei test a quella di `4e3a740`;
- sezioni: IRAM e `.dram0.data` identiche, `.dram0.bss` +48 B, `.flash.text` +360 B, `.flash.rodata` +208 B.

Il confronto con la sessione del 15/11 pomeriggio misura quindi T2R1M (e D2) e nient'altro.

Rollback: `e321e86` (installato) o `8530d3a`, script pinnati già verificati.

**Sequenza.**
1. **Cattura DALY a freddo, 20 min passivi**, con D1 e D2 attraverso `@BMS STREAM ON`. Se compare un silenzio, D2 registra livelli e FIFO.
2. **Preflight e flash APP-only**, con gate e criteri del 15/11 invariati (150 poll OK, preflight 8/8).
3. **La stessa suite fixture A–G del 15/11** (65 passi, stessi controlli), per il confronto diretto con `8530d3a` (mattina) ed `e321e86` (pomeriggio). La ripresa dopo STOP è nei passi B6, D15, D19 e F5; i cammini sterzati in D2–D11.
4. **Misure Timing:** `analyze_diag` per classe e blocco. Si rilevano picchi, overrun, perse, gap, IPC, `istall_mem`, heap, e le sequenze di perse consecutive (vincolo 2 del §6).

**Criteri, dichiarati prima.**
- **P0:** chiuso solo con 0 perse, 0 overrun e massimo ≤ 16 ms (criterio dev.8 §8.3, invariato). Per il §6 l'esito atteso resta **non chiuso**. La sessione misura quanto della riduzione QEMU arriva sull'hardware. L'attesa ACC della prima STAND resta conteggiata e riportata a parte.
- **Regressione:** ogni passo deve finire come il 15/11.
- **Qualità IMU:** confronto della correzione voluta e applicata nei cammini sterzati. Atteso: aumento dell'errore medio (+18 % offline), nessun aumento del massimo.
- **STOP durante la ripresa:** la suite non lo contiene. Aggiungerlo richiede una versione nuova dello strumento, da qualificare offline prima della sessione.

## 9. Branch, immagini e prove

| Oggetto | Dove | Stato |
|---|---|---|
| Branch operativo | `integ/matdog-unified-operational-20261114` (origin) | `f3b0d48` IRAM G1, docs `0c57b1b` e il commit di questo rapporto |
| `main` | `6838e4d` | invariato |
| T2R, T2R1 | `research/p0-t2-recovery-20261115` (`bb5520b`, `4757fa0`) | locale + bundle |
| T1M | `research/p0-t1m-20261115` (`a62171c`) | locale + bundle |
| T2R1M | `research/p0-t2r1m-20261115` (`0fb22b0`) | locale + bundle |
| D2 | `cand/daly-d2-20261115` (`8429bc0`, `4e3a740`) | locale + bundle |
| Diagnostica integrata | `cand/diag-p0-t2r1m-d2-20261115` (`46b3175`) | locale + bundle |
| Diagnostica installata | `cand/diag-8530d3a-iram-20261115` (`e321e86`) | invariata sul robot |

Prove: questa cartella. `B_t2/` contiene strumenti, rig, alberi, uscite host e QEMU, confronti e suite; `D_d2/` gli audit; `A_iram/` l'audit di `f3b0d48`. Sigilli in `SHA256SUMS`.

Nessun flash, nessun Motion, nessuna scrittura NVS, nessuna modifica a `main`, limiti o calibrazioni.
