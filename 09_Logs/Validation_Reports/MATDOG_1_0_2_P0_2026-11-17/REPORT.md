# MATDOG 1.0.2 — Motion Timing P0 e HTTP: rapporto del 17/11/2026

Registro della giornata: [issue #70](https://github.com/MattRobotics/robot-dog/issues/70). Evidenze locali, sigilli e decisioni: [`EVIDENCE_MANIFEST.md`](EVIDENCE_MANIFEST.md). Ripresa: [`NEXT_SESSION.md`](NEXT_SESSION.md). Patch non applicate: [`patches/`](patches/).

**P0 resta APERTO.** Nessun risultato qui sotto dimostra un massimo ≤ 16 ms con zero overrun e zero release operative perse.

## Classificazione usata

| Etichetta | Significato |
|---|---|
| **VERIFIED HARDWARE** | misurato sul robot, con la build indicata |
| **VERIFIED OFFLINE** | dimostrato su banchi host o QEMU ESP32-S3; non dice nulla del comportamento sul chip oltre a quanto il banco modella |
| **IMPLEMENTED NOT VALIDATED** | presente nel candidato locale, mai installato |
| **HISTORICAL** | risultato di sessioni precedenti, citato per contesto |
| **HYPOTHESIS** | spiegazione o stima non misurata |
| **UNKNOWN** | non determinato |

## 1. Stato del robot e provenienza

| Oggetto | Identità | Stato |
|---|---|---|
| **Firmware installato** | `6be46906d069` (`GIT_SHA` `6be46906d069787f3305aa7661d2c55fa118e78c`, `1.0.2-dev.8`, diagnostica F3 + IRAM G2s), APP `be678d83…3b68` | VERIFIED HARDWARE: identità letta via USB senza reset il 17/11 (APP per associazione col manifest). Robot spento a fine giornata |
| Branch operativo | `integ/matdog-unified-operational-20261114`, sorgente `202ecbc` (= remoto prima di questo commit) | nessun commit di sorgenti firmware il 17/11 |
| **Candidato locale** | `202ecbc` + 4 file modificati **non committati** (`validDefinition()`, PlanSplit V2, correzione STOP → STAND), diff `7e2d3d70…` | IMPLEMENTED NOT VALIDATED; build DIRTY, mai installata |
| **Patch HTTP** | scrittore di stato in `MotionService.cpp`, diff `22188f11…` | VERIFIED OFFLINE; isolata, **non** nel worktree |

**Il firmware installato non è il candidato.** In particolare non contiene la correzione STOP → STAND (§6).

## 2. Obiettivo Motion

- Periodo 20 ms (50 Hz).
- **Massimo ≤ 16 000 µs per release, zero overrun, zero release operative perse**, con le protezioni reali.
- Verifica finale con la dashboard attiva: STAND, WALK STANDARD, RECOVERY e STOP.

## 3. Cronologia del 17/11

Orari locali.

| Ora | Lavoro | Esito |
|---|---|---|
| 08:24 | `validDefinition()` | esatta, −95 % sulla funzione; insufficiente da sola |
| 08:50 | Attribuzione di REST | REST = 4 IK di contatto; target F6 duplicato provato di `pendingGoal_`; nessuna patch |
| 10:13 | Preparazione della qualifica operativa | test dashboard PASS; immagini APP installate salvate; bloccata sull'identità CLEAN |
| 11:12 | A. Riuso trigonometrico FK → Jacobiano | esatto, COMPENSATE −6 %; insufficiente |
| 12:33 | B. Jacobiano X/Y/Y | non bit per bit, COMPENSATE −9…−11 %; insufficiente |
| 16:09 | PlanSplit V2 | piano preparato in k−1; equivalenza offline; idonea |
| 18:01 | Correzione STOP → STAND (difetto trovato dalla revisione indipendente di V2) | idonea |
| 18:40 | Consolidamento nel worktree | `FINAL.diff` `7e2d3d70`; gate PASS |
| 19:28 | Analisi temporale Motion/HTTP | caso B: i picchi coincidono con le risposte di stato HTTP |
| 19:43–20:20 | **Prova A/B sul robot** (`6be4690`) e acquisizione DALY | HTTP causa dominante delle release ≥ 15 ms |
| 22:02 | Scrittore di stato senza `printf` | identico byte per byte, −51 % istruzioni |

## 4. `validDefinition()` — VERIFIED OFFLINE, IMPLEMENTED NOT VALIDATED

- **Modifica:** `StandTrajectory::validDefinition()` non ricalcola ciò che la definizione già garantisce. Pin delle sorgenti aggiornato.
- **Costo:** −95 % sulla funzione (35 446 → 1 708 istruzioni, QEMU), pari a −2,8…−3,3 % sulle release F6/H18. Stima sul chip ≈ 0,3–0,6 ms (HYPOTHESIS).
- **Equivalenza:** target RAW, stati, STOP e transizioni identici in tutti i banchi.

## 5. PlanSplit V2 — VERIFIED OFFLINE, IMPLEMENTED NOT VALIDATED

- **Modifica:** la preparazione del piano HEIGHT/PRONE_HEIGHT pendente (vias, `ViaPlan::plan`, prova box/`pointSafe`) si sposta nell'ultima release di unwind k−1, dopo il suo TX.
  - L'attivazione resta nella release k, con dati freschi e la stessa epoca.
  - Nessuna release, latenza, coda o stato in più (un solo marcatore da 4 B).
- **Equivalenza:** RAW, piano, STOP, rifiuti e fault identici in 26 casi mirati, nei digest dei frame, nelle matrici STOP e nel gate delle secanti.
- **Tempo** (HYPOTHESIS): ≈ 89–91 k CCOUNT tolti dalla release di avvio. Avvio F6/H18 da 20,8 ms a ≈ 15,4–15,7 ms, stima tarata su release che si sovrappongono a una risposta HTTP.
- **Memoria:** flash +568 B; IRAM e heap invariati.

## 6. Correzione STOP → STAND — VERIFIED OFFLINE, IMPLEMENTED NOT VALIDATED

- **Difetto, preesistente e presente anche nel firmware installato.** Uno STOP servito proprio nella release di avvio di una partenza pendente, prima o al posto del suo primo TX, lascia un'ancora di ripresa mai comandata. La ripresa con STAND salta fino a **555 tick** (0,85 rad): 555, 354, 138, 403 e 353 tick nei casi riprodotti. Produce anche due fault spuri: `STAND_CLEARANCE_UNAVAILABLE` e, nella catena di due STOP, `NATIVE_TARGET_NOT_REACHED`.
- **Correzione:** 6 righe in `beginStop()`. Ripristinano l'ancora HOLD dal primo via del piano e la correzione IMU a zero.
- **Verifica:**
  - i 24 casi del difetto coincidono bit per bit con uno STOP servito prima dell'attivazione; gli altri 49 sono identici;
  - QEMU è identico all'host;
  - matrici STOP 0/1/2, gate delle secanti e digest dei frame identici;
  - controllo del box STOP su 5 208 configurazioni: 0 rifiuti.
- **Fino alla qualifica sul robot:** dopo uno STOP vicino a un comando STAND/WALK/PRONE, **non riprendere con STAND**; passare da SAFE_OFF.

## 7. Analisi Motion/HTTP

Fonte: registrazioni hardware del 15–16/11, 10 avvii, 6 build. È un'analisi offline di dati hardware.

- **22 coppie di release** con lo stesso lavoro (soluzioni, IK, Newton, FK): COMPENSATE della release più pesante ×1,94 in mediana (1,11–2,23), più lenta in 22/22.
- **PMU:** COMPENSATE ha in media IPC 0,54 e 24 % di cicli fermi sul fetch istruzioni; nelle release più pesanti 0,32 e 54–55 %. Interrupt normali, nessun altro task sul core Motion.
- **Coincidenza:** 24 delle 87 release più pesanti sono la prima dopo una lettura di stato del watcher HTTP (caso: 1,74; p ≈ 8,5 × 10⁻²¹).
- **Meccanismo** (HYPOTHESIS):
  - una sola I-cache da 16 KB condivisa dai due core;
  - HTTPD, lwIP e Wi-Fi che eseguono da flash sul core 0;
  - una release Motion tocca 46–57 KB di codice in flash.
- **Decisione:** nessuna redistribuzione del lavoro Motion; serve una misura A/B dell'HTTP a firmware invariato.

## 8. Prova A/B — VERIFIED HARDWARE (`6be4690`, fixture, 17/11)

**Disegno**
- Stessi quattro strumenti qualificati del 16/11 (settle, blocchi A+B, E+F+G con FAST, H con STOP in RECOVERY), ciascuno nei profili A e B, ordine ABBA.
- Profilo A: carico HTTP del 16/11, 2,86 letture di stato/s.
- Profilo B: 0,91 letture/s.
- **140/140 passi PASS**, nessun fault o abort; tutti gli STOP via USB verificati.

| Misura | A | B |
|---|---:|---:|
| Release ≥ 15 ms | **168** | **25** (−85 %) |
| Release 10–15 ms | 2 877 | 1 930 (−33 %) |
| Massimo | **20 449 µs** | **19 786 µs** |
| Overrun (> 20 ms) | 1 | 0 |
| Finestre con massimo ≥ 16 ms (su 28) | 21 | 8 |

- **Significatività:** probabilità di 25 o meno per caso, se il profilo non contasse: 1,6 × 10⁻²⁷. Massimo per finestra più alto in A in 26 finestre su 28.
- **PMU**, a parità di istruzioni per release, attesa sul fetch istruzioni in COMPENSATE (A contro B):
  - WALK 0,265 contro 0,236;
  - RECOVERY 0,282 contro 0,236.
- **Coincidenze:** tutte e 10 le release `MAX_WORK` con numero noto cadono su una risposta di stato in corso; 7 sono la prima release dopo la lettura.
- **Dunque** l'HTTP è la causa dominante delle release ≥ 15 ms. Ma nel profilo B una sola risposta sovrapposta porta ancora le release a due soluzioni a 16,2–16,8 ms e un WALK a 19,8 ms: **ridurre il polling non basta per P0**.
- **Release perse:**
  - durante un comando: 6, tutte in ACQUIRE della **prima STAND dopo il boot** (123 ms, max_gap 142,6 ms), indipendenti dal profilo;
  - a riposo: 3, dopo dump diagnostici.
- **Termico:** un campione spurio a 90 °C su RH_HIP (26 °C prima e dopo), filtrato dal firmware; nessun fault.
- **DALY:** nessun silenzio dall'uptime 282 s alla chiusura (1 800 s). I primi 282 s dopo l'accensione non sono stati registrati.

## 9. Scrittore HTTP di stato — VERIFIED OFFLINE, non installato

- **Dove sta il costo.** `/api/motion/status` acquisisce lo snapshot sul task loop (core di Motion, fra due release; HW fino a 5,7 ms). Poi `formatStatus` lo serializza su HTTPD, sull'altro core e **in parallelo alle release**:
  - 70 chiamate `snprintf`, 372 argomenti, ≈ 4,8 KB;
  - 94 297 CCOUNT ≈ 291 k istruzioni (QEMU);
  - `_svfprintf_r`, `_dtoa_r` e i loro helper stanno in flash.
- **Modifica:** un solo file, `MotionService.cpp`, +91 −1. `formatStatus` usa un piccolo scrittore esatto per le sole conversioni che impiega; il resto passa ancora per `snprintf`. Stesse stringhe di formato, quindi stesso schema.
- **Equivalenza:** identico byte per byte.
  - Risposta reale del robot.
  - Snapshot casuali: 200 000 su host, 8 000 sul target newlib.
  - Scrittore contro `snprintf` a ogni troncamento: 7 063 936 confronti su host e 252 966 sul target, 0 differenze.
  - 24 fixture della dashboard.
- **Costo:**
  - `formatStatus` 94 297 → 46 116 CCOUNT, **−51 % istruzioni**;
  - codice in flash raggiungibile per risposta 22 541 → 7 698 B, **−66 %**.
- **Memoria:** heap interno, IRAM e DRAM **invariati**; APP +2 800 B; stack di `formatStatus` −585 B.
- **Gate:**
  - `static_audit` PASS (`MOTION_HTTP_TRANSPORT` 97 controlli);
  - suite dashboard PASS su baseline e candidato, compreso il contratto da 6 116 controlli e lo STOP durante il caricamento del Twin;
  - suite Motion 4/4 PASS.
- **Effetto sui massimi Motion: UNKNOWN** fino alla prova sul robot. QEMU non modella la cache condivisa, la flash né il secondo core.

## 10. Problemi aperti

| Problema | Stato |
|---|---|
| **P0** | APERTO. Il massimo con carico HTTP resta > 16 ms (VERIFIED HARDWARE, 17/11). L'effetto dello scrittore è UNKNOWN |
| Prima ACQUIRE dopo il boot: 123 ms e 6 release perse | VERIFIED HARDWARE, distinto dall'HTTP, non toccato. Da solo impedisce di dichiarare zero release operative perse |
| DALY BMS intermittente | HISTORICAL: silenzi entro ≈ 335 s dall'accensione. Causa UNKNOWN; manca la misura sul bus RS485 durante un silenzio. Nessuna anomalia il 17/11 dopo 282 s |
| Qualifica STOP sul robot | la correzione STOP → STAND non è installata. STOP/ripresa dalla dashboard con la correzione non qualificati sul robot. Frenata fisica e supporto non qualificati |
| Margine di heap con più client HTTP | invariato dalle modifiche; al massimo 2 client nelle sessioni (ADR-007) |

## 11. Decisione tecnica sulla prossima APP

**Composizione:** `202ecbc` + `FINAL.diff` + `HTTP_CANDIDATE.diff` = `COMPOSED_vs_202ecbc.diff`, cioè 5 file.

| File | Contenuto |
|---|---|
| `StandTrajectory.cpp`, `motion_final_release_sources.json` | `validDefinition()` e pin |
| `SimpleMotion.h`, `SimpleMotion.cpp` | PlanSplit V2 e correzione STOP → STAND |
| `MotionService.cpp` | scrittore di stato |

**Escluse per ora (DEFER):**
- ottimizzazioni A e B del Jacobiano: si escludono a vicenda;
- riuso di `pendingGoal_`;
- dashboard a 1 Hz.

Motivo: le release lente sono dominate dall'HTTP. Inserirle nella stessa APP confonderebbe la misura.

**Condizioni di collaudo, fissate prima della prova**
1. Commit locale autorizzato per l'identità CLEAN; build, `static_audit` e pacchetto di flash qualificato; rollback all'immagine installata `6be4690` pronto.
2. Accensione con gate DALY: QON + 6 min e 150 poll validi.
3. Stessa catena A/B del 17/11 (profili A e B) più un blocco con la dashboard reale. Dopo uno STOP dalla dashboard, ripresa STAND solo su questa build.
4. **P0 chiuso solo** con massimo ≤ 16 000 µs, 0 overrun e 0 release operative perse in ogni prova, dashboard compresa. La perdita in ACQUIRE della prima STAND **conta**.
5. Effetto dello scrittore: release ≥ 15 ms nel profilo A confrontate con le 168 del 17/11, con intervallo di Poisson al 95 %.

**P0 non è dichiarato concluso.**
