# Intervalli temporali (UTC, 11/11/2026)

| Intervallo | Fase | Fonte principale |
|---|---|---|
| 18:49–18:50 | Preflight sulla dev.6 installata, gate offline | `pre/` |
| 18:50:58 | Ingresso ROM (usb-reset, read-mac) | `flash/rom-entry*.{log,txt}` |
| 18:51:11–18:52:01 | Flash APP-only app0 + verify-flash | `flash/flash.log`, `flash/flash-*-utc.txt` |
| 18:52:12 | Reset watchdog, apertura registratore USB | `flash/boot-*`, `flash/recorder-open-utc.txt` |
| 18:52:12–19:55:33 | `usb/serial.log` (46 500 righe) | |
| 18:52:27–19:55:55 | `usb/http.jsonl` (4 551 record: motion 1 Hz, /status 5 s) | |
| 18:52:12–19:55:54 | `usb/events.jsonl` (61 eventi, fino a `session_end`) | |
| 18:56:26–19:29:26 | `usb/watch_events.jsonl` (58 DIAG automatici su WORKSPACE persistente) | |
| 18:52:30–18:56 | H0 | `h0/` |
| 18:56–18:57 | F1 STAND / STOP / STAND | `f1_summary_a.txt` |
| 18:58–18:59 | F2 SLOW joystick + STOP | `f2_summary.txt` |
| 18:59:30–19:05:17 | F3a SLOW | `f3a_summary.txt` |
| 19:05:20–19:10:21 | F3b STANDARD | `f3b_summary.txt` |
| 19:10:25–19:15:11 | F3c FAST | `f3c_summary.txt` |
| 19:15:30–19:19:17 | F4a cambi di modo | `f4a_summary.txt` |
| 19:19:20–19:28:39 | F6 timing con dashboard/tab | `f6_summary.txt` |
| 18:52:30–19:28:39 | Fixture completa | `fixture_full_summary.txt` |
| 19:28:32–19:30:41 | HOLD 105 mm sulla fixture | `m1_flex.txt` |
| 19:30:42–19:30:54 | PRONE sulla fixture, trasferimento a terra a coppie OFF | `ground_summary.txt` |
| 19:31:14–19:35:41 | Terra, blocco 1 (M1 + camminate + STOP FAST indietro) | `ground_summary.txt`, `ground_timeline.txt`, `m1_flex.txt` |
| 19:39:30–19:49:36 | Terra, blocco 2 (STOP avanti nei tre modi, laterali, yaw, indietro, cambi di modo, PRONE) | `ground2_summary.txt`, `ground_directions.txt` |
| 19:55:33 | KEY OFF (brownout) | `usb/serial.log` |
| 19:55:54 | Chiusura registratori | `usb/events.jsonl` |
