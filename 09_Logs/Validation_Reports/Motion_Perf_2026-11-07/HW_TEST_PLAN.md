# Piano di collaudo hardware — candidata 1.0.2-dev.1 (motion-perf-next)

**Stato:** proposta. Nessun flash, riavvio o movimento è stato eseguito in questa sessione; il robot è spento su fixture con dev.17 (`v1.0.1`) installato. Ogni passo sotto richiede una decisione supervisionata esplicita, operatore presente, STOP della Dashboard e KEY OFF del BMS raggiungibili, supporto anticaduta per i primi WALK.

**Rollback:** la Stable `v1.0.1` (APP `14e055a0…`, 1.707.808 B) e dev.16 (`73952eec…`) restano gli unici binari qualificati; il flash APP-only del candidato usa lo stesso script (`flash_app_only.sh`) e non tocca NVS gen-1, partizioni, bootloader.

## Cosa deve dimostrare il collaudo (metriche nel JSON `/api/motion/status` già esportate)

| # | Prova | Metriche da leggere | Esito atteso (modello) | Esito che boccia |
|---|---|---|---|---|
| H0 | Boot, READY/IDLE 90 s con Dashboard in polling | `releases`, `missed_releases`, `overrun_releases`, `max_release_work_us`, `loop_stack_min_free` (se esportato via USB `@MOTION TIMING`) | 0 missed/overrun, max ≤ 5 ms, NVS gen-1 `VALID_ACKNOWLEDGED`, census 13/13 | qualunque fault, versione ≠ `1.0.2-dev.1` |
| H1 | STAND100, HOLD 60 s con coppia ON | come dev.17 (34): 0 missed, ≤ 2 overrun | HOLD invariato (T1 già in dev.17) | overrun > dev.17 |
| H2 | WALK STANDARD 45 mm, 3 cicli, WALK STOP | `command_timing.max_work_us`, `overruns`, `timing.max_work[0..5]` = sezioni della **release di massimo** del comando (un massimo, non una misura per release) e `[6..11]` = sezioni della release che l'ha preceduta | nel record di massimo NOMINAL resta ~6 ms (è una release che ha risolto la legge) ma la release **precedente** deve mostrare NOMINAL ≈ 0,5 ms quando non ha risolto (prima: ~6 ms su ogni release); overrun % < 9,4 % (dev.17 traccia 35); `max_work_us` < 33 ms; nessun fault. La distribuzione per release si ottiene solo dal dump USB `@MOTION TIMING` | fault `WALK_IK_OR_WORKSPACE`, overrun % ≥ dev.17, vibrazione nuova |
| H3 | WALK STANDARD 60 mm, 3 cicli, WALK STOP | idem | idem; zampa LF senza vibrazione come dev.17 | idem |
| H4 | STOP durante WALK45 (due volte: a metà swing e a quattro appoggi) | ACK STOP, `stop.motion_verified`, tempo fino a `STOPPED` | come dev.17; nessun peggioramento del tempo di arresto | STOP non verificato, SAFE_OFF inatteso |
| H5 | Joystick avanti costante 60 s, poi avanti+laterale 60 s, poi rilascio | `max_work_us`, `overruns`, `missed`, firma `solves/ik/newton/fk` (massimi del comando) | `max_work_us` < 47 ms e «previous» con NOMINAL ≈ 0,5 ms sulle release senza soluzione; missed/s ≪ 1,3 (traccia 36) | missed/s ≥ dev.17, fault |
| H6 | Joystick con comando variato ogni ~1 s per 60 s (retarget continui) | idem | quota di release leggere T1 più alta: `max_work` stabile | fault |
| H7 | STOP → WALK diretto, PRONE finale, SAFE_OFF 13/13 | come dev.17 (36) | invariato | qualunque regressione |

Tutte le prove alla quota WALK 120 mm, stride 45/60, cadenza 6,5 s: la legge del gait, i box, i limiti e il profilo servo **non sono cambiati** in questa candidata, quindi i certificati geometrici di v1.0.1 restano validi.

**Revisione 08/11/2026 (T2 sotto retarget e jitter):** il riferimento inviato è la legge all'istante della release (interpolazione fra due soluzioni esatte a cavallo di `now`), come nella Stable; la stima della rate per la prova di arresto e per `beginStop` è letta su almeno un periodo (`dt ≥ 20 ms`). Da osservare in H4/H5: ogni STOP termina `STOPPED/STOP` con `motion_verified=true`; nessun `STOP_REFERENCE_OUTSIDE_BOUNDS`.

## Misure aggiuntive utili (non bloccanti)

- Dump USB `@MOTION TIMING` in WALK e joystick: lateness per release, `outsideVisitUs`, `externalUs[]` per separare lavoro Motion e lavoro del loop (ipotesi C1 «stesso lavoro, tempo ×2» ancora UNKNOWN).
- Stack minimo del loop per fase (dev.17: 2.060 B in traccia 36). I frame statici del candidato sono riportati nel REPORT; la misura reale è solo su hardware.
- Confronto A/B dev.17 ↔ candidata sulla **stessa** sequenza di comandi (stesse quote, stessi tempi), finestre non sovrapposte.

## Cosa NON autorizza questo piano

- Nessuna cadenza > 1×, nessuna modifica del profilo servo `kWalkDrive{1000,50}`, nessuno sway continuo: il modello (`run_walk_tilt_envelope_check.sh`) mostra che ≥ 1,5× richiede un profilo drive più alto (≥ 1500/100) e una qualifica hardware dedicata (corrente, termica ID21/ID31, tracking).
- Nessuna ricalibrazione, scrittura NVS, cambio geometria.
