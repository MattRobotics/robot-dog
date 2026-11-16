# Prove offline degli strumenti (16/11/2026 sera)

Simulatore: `sim/sim_device.py`, copia di quello del 16/11 (`MATDOG_1_0_2_DIAG_46B3175_HW_20261116/sim`) con build `cb9ed85565f6`, campi `heap_free`/`heap_min_free` e calo programmabile del minimo (`SIM_DROP_AT` secondi dopo la prima richiesta delle capabilities, cioè dall'avvio di uno strumento di prova). Registra i comandi USB ricevuti. Ogni caso gira in `cases/<caso>/` su una copia di `../hw_next/tools`. Mai il robot.

| Caso | Condizione | Esito |
|---|---|---|
| T1 `t1_nominal` | blocco H completo, minimo 39 424 B | **20/20 PASS** (M0 + 19 passi). M0 dopo 20 s: 39 424 B, margine 14 848 B, 26 campioni del watcher. `ACQUISITION_CLOSED`, nessun processo rimasto |
| T2 `t2_m0_low` | minimo a 20 000 B 3 s dopo l'avvio | **M0 MEMORY_GATE**: Abort prima di ogni comando Motion (0 comandi Motion ricevuti), robot non toccato. Chiusura verificata. (Prima versione: la guardia arrivava prima di M0; corretta, M0 controlla la soglia a ogni ciclo) |
| T3 `t3_stray` | un `http_watch.py` non registrato attivo, come il 16/11 | **REFUSE**: `UNREGISTERED pid … http_watch.py`; nessuna acquisizione, 0 comandi USB |
| T4 `t4_guard_in_motion` | minimo a 11 144 B 35 s dopo l'avvio (durante Motion) | M0 PASS; `heap_guard` → SIGTERM in H7 → Abort → **STOP inviato** da `safe_end` (protezione conservata). Chiusura verificata |
| T5 `t5_cold` | `cold_start.sh` (fasi da 3 s) | registratore e watcher registrati con PID reale; `run_acquisition` **rifiuta** finché la registrazione cold è aperta; `stop` del watcher (PID) e del registratore (`QUIT`) verificati; `check --expect-none` PASS |
| T6 `t6_boot` | registratore di boot lanciato dal registro (`--also usb_session.py`, segue l'`exec`) + watcher | `check` PASS con entrambi; `stop` verificati (QUIT per l'USB); nessun processo rimasto |
