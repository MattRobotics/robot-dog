# Verifiche concluse — offline, nessun VERIFIED HARDWARE nuovo

| Verifica | Risultato completato | Evidenza nel pacchetto |
|---|---|---|
| Provenienza rc.11 | 207 header/C++ compilati e INO confrontati; APP originale hash corretto; baseline invariata | provenance.json / baseline-source-sha256.json |
| Preservazione | 164 file protetti identici; termica, STOP, STAND, FK/IK e compensazione invariati | protected-files.json / diff |
| Prima integrazione senza correzione | 50: 23/23 FAULT durante PRONE_LIFT; 100/150: 46/46 terminale ma margini 1–2 tick | replay-original/test_prone_hardware_replay.log |
| Replay finale | 69/69 terminale torque ON, zero rifiuti/fault; minimi per giunto >4 tick | tests-final/test_prone_hardware_replay.log |
| PRONE nominale | 50/100/150, 23 vecchi residui100; max 8 tick/frame, torque ON, budget letture preservato | tests-corrected/test_prone_descent.log |
| STOP/fault PRONE | 30 scenari in 5 fasi, STOP in TX, feedback perso, power/native/voltage; nessun recupero | stesso log |
| Terminale prolungato | 600 s virtuali a ciascuna altezza nominale, goal fissi e coppia ON; non prova motori/temperatura | stesso log |
| Termica rc.11 r2 | 67 check, finestre/gap/isteresi, STOP, falsi spike, conferma e re-arm | tests-corrected/test_temperature_confirm.log |
| Termica durante PRONE | 10 scenari: 98→24 isolato filtrato e 98 persistente confermato nelle 5 fasi; OFF dopo fault | tests-final/test_prone_temperature.log |
| SimpleMotion/STAND | Suite precedente; carico 73 check, timing 18 check; ammissione PRONE | tests-corrected/test_simple_motion.log, test_stand_load_completion.log, test_stand_profile_timing.log, test_prone_admission.log |
| HOLD | 345 check sulla prima integrazione; sola modifica successiva nel planner PRONE | tests/test_operational_hold.log |
| HEIGHT100 discendente | 23/23 a 50 timeout attesi; 23/23 a 53 completano dentro entrambe box | tests/test_prone_height_residuals.log |
| Traiettoria corretta | Limiti/box, velocità/accelerazione/continuità; 89 appoggi statici mesh, 69 endpoint Controller | geometry/summary.json, path.csv, static_path_samples.json, replay_endpoint_rest.json |
| Dashboard | Fixture25, DOM33 comandi/9 envelope, Twin32, budget24, contratto159: tutti PASS | dashboard-final.log |
| Restore gen1 | Runtime974, persistence500, gen1 143, sweep17905 tick: tutti PASS | restore-tests.log |
| Build | exit0, CLEAN, APP1629408B, layout OK | build.log / candidate/matdog_build_manifest.txt |

Tutti i processi finali di test/build sono terminati con exit 0. L'iniziale dashboard-tests.log ha exit 1 nel test Twin: il checkout base f838 conteneva il vecchio mobileMain.ts, mentre la rc.11 ha il nuovo sorgente. La copia **identica** dal worktree rc.11 riallinea il test agli asset già installati; l'intera suite dashboard-final.log è poi passata. Nessun cambiamento aggiuntivo agli asset firmware TwinAssets.

Il test HOLD ereditato stampa ancora la frase storica «full PRONE descent remains blocked»: descrive il vecchio perimetro, non l'esito PRONE della nuova suite. I valori MARGIN stampati dopo FAULT/IDLE nella prova originale non sono utilizzati come minimi validi.

Non eseguiti: flash, accesso hardware, timing ESP32 reale, nuova durata termica, discesa sotto carico reale, misura dell'appoggio, trasferimento posteriore, assestamento libero, concorrenza/socket HTTPD, static audit completo. Nessun PASS dichiarato per queste attività.
