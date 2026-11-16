# ADR-007 — Ottimizzazioni Motion Timing: integrazione senza strumentazione e margine di heap interno

Data: 2026-11-16. Stato: **Accepted** sul branch `integ/matdog-unified-operational-20261114`.

## Contesto

- Dal 14/11 il lavoro su Motion Timing P0 si valida con build **diagnostiche** `1.0.2-dev.8`: contatori PMU, `@MOTION TIMING` esteso, D2 DALY e D3.
- Le modifiche Motion vere e proprie (T2R1M, E1/E1b, P3a/P3b, P5, F3, IRAM) sono state provate sul chip solo dentro quelle build.
- Il 16/11 la sessione su `cb9ed85` (F3 + IRAM G2) si è fermata alla soglia di heap interno di 24 KB.
  - G2 costa 10 240 B di heap (spazio della IRAM).
  - Il transitorio di lwIP e Wi-Fi in heap interno cresce col numero di client HTTP contemporanei: ≈ 21 KB con 1 client, 26,8–36,2 KB con 2, 50,4 KB con 3.
- La variante G2s (−6 144 B) è stata qualificata su `6be4690` con 2 client: margine 1 096 B.

## Decisione

1. **Integrazione per cherry-pick tracciati, mai sostituendo l'albero.**
   - Nel branch operativo entrano solo le modifiche alla legge Motion e ai test, con `cherry picked from` nel messaggio.
   - La strumentazione diagnostica (PMU, `DiagBuild.h`, timing esteso, D2, D3, note di composizione in `BuildConfig.h`) resta fuori. È conservata come patch e bundle.
   - L'identità installata resta quella della build diagnostica: un sorgente integrato non diventa «installato» finché la sua APP non è letta dal dispositivo.
2. **Ogni placement IRAM è limitato dal margine di heap interno.**
   - Si misura il costo statico dall'ELF (`_heap_start`).
   - Si qualifica sul chip con lo stesso numero di client HTTP previsto nell'uso.
   - Sotto 24 KB di `heap_min_free` (minimo storico, somma per regione) o con un'allocazione fallita, la prova si ferma e si torna al rollback.
3. **Sessioni hardware: al massimo 2 client HTTP**, cioè lo strumento di prova e un watcher.
   - I processi si avviano e si chiudono solo dal registro con PID reale (`timing_tools/session_proc.py`, `run_acquisition.py`).
   - Ogni prova comincia con il controllo M0 della memoria prima di qualunque comando Motion (`memgate.py`).
   - Nessuna dashboard aperta durante le prove.
4. **Nessuna protezione cambia per far spazio.** STOP, limiti, calibrazioni e STOP proof restano quelli validati: si riduce l'insieme IRAM (G2 → G2s), non le protezioni.

## Conseguenze

- Il branch operativo contiene la legge Motion di `6be4690` (VERIFIED HARDWARE tramite quella build). La build operativa integrata `33134ae` è soltanto VERIFIED OFFLINE.
- Dall'ELF, l'heap statico della build operativa è maggiore di 8 168 B rispetto alla diagnostica. Il margine con 2 client e la dashboard va misurato sul chip prima di dichiararla adeguata.
- Con 3 o più client contemporanei nessuna build ha un margine garantito. Ridurre l'uso di RAM interna del trasporto HTTP (buffer di lwIP o Wi-Fi in PSRAM) è una decisione futura, non presa qui.
- Riferimenti: [rapporto del 16/11](../Validation_Reports/Unified_Operational_2026-11-14/P0_F3_G2s_2026-11-16/REPORT.md), [RCA del MEMSTOP](../Validation_Reports/Unified_Operational_2026-11-14/P0_F3_G2s_2026-11-16/sources/RCA_MEMSTOP_G2s_20261116.md).
