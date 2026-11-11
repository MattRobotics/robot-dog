# Chiusura sessione dev.7 HW — 11/11/2026

File aggiunti dopo `MANIFEST_SHA256.txt` (19:56Z, che resta valido per i file che elenca); l'inventario completo è `INVENTORY_SHA256.txt`.

- **Fine prove:** 19:49:36Z, PRONE_REST, SAFE_OFF verificato, mask 8191, coppie OFF (`usb/http.jsonl`, `http_watch.console`).
- **Spegnimento fisico (KEY OFF):** ultimo `/status` 200 alle 19:55:32.972Z; `E BOD: Brownout detector was triggered` nel seriale alle 19:55:33.297Z; HTTP `No route to host` / timeout dalle 19:55:36Z; nessun `/dev/ttyACM*` presente dopo la chiusura.
- **Registratori:** `ctl.py usb QUIT` alle 19:55:54Z, `usb/events.jsonl` termina con `session_end`, console `USB SESSION END (Motion untouched)`; `http_watch.py` fermato con SIGTERM (ultimo record 19:55:55Z, tutti i JSONL validi e terminati da newline).
- **Disturbo IMU 19:34:34–36 (HOLD 110 mm):** chiarito dall'operatore, riposizionamento manuale del robot per la lunghezza del cavo USB (~1 m), anche in altri HOLD della seconda parte a terra. Chiuso, da non riaprire.
- **"360°" dell'operatore:** è il giro completo della leva in STANDARD (avanti → diagonali → laterali → indietro → avanti, 19:47:49–19:49:05Z), non una rotazione di 360° del corpo; lo yaw a terra è 2,29 °/s per 9–13 s per verso (`ground_directions.txt`).
- **Aggiunti alla chiusura:** `ground_directions.txt`, `firmware_identity/`, `offline_repro_tools/`, `INVENTORY_SHA256.txt`, `TIME_RANGES.md`.
- **Report canonico:** `09_Logs/Validation_Reports/MATDOG_DEV7_HW_2026-11-11/HW_REPORT.md` nel repository; `HW_REPORT.md` qui è la bozza scritta a fine prove (stessi esiti, meno dettaglio).
