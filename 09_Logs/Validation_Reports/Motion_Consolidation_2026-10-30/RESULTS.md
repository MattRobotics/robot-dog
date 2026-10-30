# Risultati verificati — consolidamento dev.4

Tutto offline; C++ produzione con mock servo e ASAN/UBSAN, DOM con risposte reali MotionService, HTTPD estratto/mirrored secondo il singolo test. Nessun test reale di durata, termica, Wi-Fi, motori o appoggio.

| Verifica | Esito e prova |
|---|---|
| SimpleMotion generale | PASS: sequenze STAND, WALK, native target, STOP, fault/comunicazione/deadline; `tests/test_simple_motion.log` |
| WALK STOP, identità, ACK senza I/O, completamento HOLD/torque ON | PASS cinque punti ciclo, idempotenza; JSON lungo4283B; `tests/test_walk_stop_observation.log` |
| PRONE | PASS3 quote,23 residui,30 STOP/fault, STOP duranteTX,600s terminale virtuale; max8tick/frame; `tests/test_prone_descent.log` |
| Replay hardware50/100/150 | PASS69/69 terminali; minimi per giunto/fase in log e `margin-summary.json`; nessuna nuova acquisizione hardware |
| STAND carico e timing | PASS73 controlli carico,18 timing; stessi profili480/13; `test_stand_load_completion.log`, `test_stand_profile_timing.log` |
| Termica rc.11 r2 | PASS67 controlli conferma,10 scenari nelle5 fasi PRONE; status/tensione continuano fault immediato; log termici |
| HOLD | PASS345 controlli,6h virtuali, HEIGHT nominale e fault; `test_operational_hold.log` |
| HTTP concorrente | PASS709 controlli per esecuzione,2 esecuzioni; produzione dispatch/update/admission/STOP lane con thread host; `http-fence.log` |
| Sessione esente slack | PASS0 failure; `http-tests.log` |
| Dashboard | PASS25 fixture,35 comandi nella sessione principale + race rinnovo/STAND/STOP separata,9 envelope,159 controlli parser/origin/CSRF/service; `dashboard-final.log` |
| Budget generale e Twin parent | PASS24 DOM budget,32 Twin; embedded0 richieste proprie/4 frame; standalone500ms invariato |
| Asset Twin | PASS42 controlli asset,50 invio limitato; gzip230690B; `twin-assets.log`, `twin-send.log`; due asset geometrici identici in `twin-geometry-preserved.json` |
| Viewer | TypeScript typecheck exit0; Vite exit0; bundle grande segnalato, nessuna modifica alle mesh; `viewer-typecheck.log`, `viewer-build.log` |
| Restore/gen1 | PASS974 runtime restore +500 persistence +143 issue48,17905tick accettati nei12span registrati; `restore.log` |
| Logger | PASS replay offline12 margini, stale esclusi, GET-only, nessun STOP temporizzato; `observer.log` |
| Integrità |164 file protetti identici a rc.11;207 sorgenti rc.11 manifest invariati; diff e status rc.11 identici al pacchetto precedente; SimpleMotion/CommandRouter/PortalPage identici a dev.3; `integrity.json`; SHA del rollback rc.11 confermato, checksum del pacchetto dev.3 precedente tutti validi |

Il log HOLD conserva una frase storica «full PRONE descent remains blocked»: quel test riguarda HOLD/HEIGHT, **non verifica né descrive l'attuale PRONE**. L'esito PRONE è dato dai test dedicati sopra. Non usare quella frase come risultato di questa candidata.

## Verifiche non completate e fallimenti intermedi

- `test_motion_http_transport.py` legacy **non esegue i test**: estrattore obsoleto cerca `Controller::stopPreempt`, assente nell'architettura corrente. È un errore dell'harness, non un PASS né prova di bug firmware. Conservato in `http-tests.log`. Copertura pertinente fornita dai709 controlli concorrenti attuali e159 contratto/CSRF; non equivale a socket ESP32 reali.
- Durante sviluppo il test JSON con buffer4096 è fallito: corretta la capacità a5120 e ripetuti tutti gli host test interessati con exit0. Regex asset aggiornata al bundle nuovo; prima esecuzione non era un PASS. I log finali elencati sopra sono gli esiti effettivamente verificati.
- Nessuna misura reale del timeout5s, RTT, stack/WCET, fluidità WebGL, slip, contatti o appoggio PRONE. La build non colma queste lacune.
- Nessuna nuova route C: confronto quantitativo in `stand-study/summary.json`, traiettorie in `stand-study/route.csv`, script riproducibile nel worktree. Esito EXCLUDED, non qualificazione.

## Riproduzione offline

Dal firmware nel worktree isolato, impostare `MATDOG_TEST_OUTPUT` a una nuova directory e usare `scripts/tests/run_operational_host_tests.sh` con: `test_simple_motion test_walk_stop_observation test_prone_descent test_prone_hardware_replay test_stand_load_completion test_stand_profile_timing test_temperature_confirm test_prone_temperature test_operational_hold`.

Dashboard: `scripts/tests/run_simple_dashboard_tests.sh` con Node22 e jsdom locali (`NODE`/`NODE_PATH`). Test Python: `test_twin_assets.py`, `test_twin_bounded_send.py`, `test_slack_exempt_session.py`, `test_motion_observer.py`; il test concorrente è `test_http_fence_contention.py`. Il collector non è stato collegato al robot: utilizzato solo `--replay-json` nel test. Per build, comando e manifest in BUILD.md. Non usare script di flash o vecchi logger con comandi temporizzati.
