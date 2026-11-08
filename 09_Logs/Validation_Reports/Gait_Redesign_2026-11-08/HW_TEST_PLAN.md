# Collaudo supervisionato breve — candidata `1.0.2-dev.3` (WALK STANDARD v2 + correzioni pre-flash)

**Da eseguire solo dopo autorizzazione esplicita del flash APP-only.** Installata oggi: Stable `1.0.1-dev.17` (APP `14e055a0…`); rollback APP-only verso dev.17 (o dev.16 `73952eec…`) con la stessa procedura. Backup integrale già verificato: `~/MATDOG/backups/esp32/matdog_esp32s3_fullflash_2026-09-10.bin` (`5cbba0b9…`), nessun nuovo backup richiesto. Drive `{1000,50}` invariato. Robot sempre supervisionato, mano libera sul KEY OFF, STOP in dashboard (o `Esc`) e `@STOP` via USB pronti.

## Interruzione immediata (STOP; se non basta, KEY OFF)

Qualunque `FAULT`; SAFE_OFF non richiesto; inizio di ribaltamento o perdita d'appoggio; piede che si impunta e fa inciampare o ruotare il corpo; temperatura servo > 60 °C; release perse in HOLD (> 0) o ripetute in WALK (> 2 nello stesso ciclo); joystick che torna al centro con il dito ancora premuto o WALK STOP non richiesto (H6); qualunque differenza di identità/NVS in H0.

## Fasi

| Fase | Azione | Osservabili | Esito atteso / arresto |
|---|---|---|---|
| H0 | Robot sostenuto, SAFE_OFF 8191 e coppia OFF verificati via HTTP. `sha256sum -c ~/MATDOG/verification-artifacts/MATDOG_1_0_2_DEV3_FLASH_PREP_20261108/rollback-assets.sha256`; ingresso in ROM (unico reset) `esptool … --before usb-reset --after no-reset read-mac`; layout/otadata in sola lettura; `MATDOG_DEV3_FLASH_AUTHORIZED=YES_ROBOT_SUPPORTED …/flash-qualified-app0-1.0.2-dev.3.sh` (una sola scrittura in `app0` 0x010000–0x1b4a60, poi `verify-flash`); avvio `--before no-reset --after watchdog-reset read-mac`, da qui **solo HTTP** | 4×OK; `APPLICATION_ONLY_FLASH=PASS`; `/status`: `FW_VERSION=1.0.2-dev.3`, `GIT_SHA=94a260cd3b4e…`; NVS gen-1 `VALID_ACKNOWLEDGED` invariato; census 13/13; IDLE 60 s con `overrun_releases` e `missed_releases` = 0 | qualunque differenza: rollback APP-only |
| H1 | STAND 100 → HOLD 60 s; STAND 150 → HOLD 30 s. Inclinometro (livella o telefono) sul pavimento e sul dorso | `imu_correction` wanted/applied in HOLD (dev.17: pitch −24…−29 mrad); `overrun_releases` = 0 in HOLD | separa pendenza del pavimento da offset IMU (non si ricalibra in questa sessione) |
| H2 | **Sospeso** (piedi liberi): WALK 50 → 2 cicli → WALK STOP | `timing.work_hist`, `command_timing` (overrun, missed, max work per fase), lag `goal`/`raw` per giunto al distacco | nessun fault; nessuna release persa ripetuta |
| H3 | **A terra**: WALK 50 → 3 cicli → WALK STOP; poi WALK 75; poi WALK 100 (ciascuno 3 cicli). Video laterale basso sui piedi; striscia di carta sotto una zampa | trascinamento al distacco per passo (previsto, riferimento a 3 mm di sollevamento: 6,3 / 9,4 / 12,5 mm; v1.0.1 WALK60 5,4 mm, osservato ~5 mm); oscillazione all'avvio e al WALK STOP (fasi limitanti del COM); `imu_correction.reason` (WALK100: `ENVELOPE` atteso spesso, come WALK60 su dev.17); WALK STOP → STANDING (atteso ~6,6 s) | inciampo/rotazione → fermarsi a quel passo e tenere il precedente come preset a terra |
| H4 | STOP operatore in WALK 100 (o nel passo più lungo accettato in H3): a metà swing anteriore, a metà swing posteriore, in finestra a quattro appoggi | piano alla release successiva; `stop.motion_verified` ≈ 2 s; nessun SAFE_OFF; resta in piedi | SAFE_OFF o caduta |
| H5 | Recupero: STAND dopo STOP; WALK 50 diretto dopo STOP | completamento del passo interrotto, nessun enable/OFF | fault in recupero |
| H6 | **Joystick dinamico, una sola pressione** (da STANDING): avanti fino a passo regolare (≥ 1 ciclo da 6,5 s), poi senza rilasciare avanti-destra (≥ 1 ciclo), destra (anche un filo sotto l'orizzontale), avanti-sinistra (≥ 1 ciclo). Poi indietro a fondo, sempre tenendo. Rilascio. Infine rotazione con i pulsanti | per tutta la parte laterale/diagonale: `walk_velocity.joystick=true`, `reason=WALK_JOYSTICK`, nessun comando «JOYSTICK neutro/rilascio» nel log, knob sotto il dito; direzione che cambia gradualmente (fondo scala 13 s). Indietro: **un solo** WALK STOP (`JOYSTICK_REVERSING`, ACK con la stessa ragione), assestamento, STANDING, ripartenza all'indietro con la leva tenuta. Rilascio: un solo neutro, WALK STOP | knob al centro con dito premuto, WALK STOP non richiesto, fault |
| H7 | PRONE da STANDING; STAND dopo PRONE; comandi dashboard (STAND, WALK, WALK STOP, STOP) uno per tipo; PRONE finale, coppia OFF | esiti `active_command`; `PRONE_REST` con OFF 13/13 | — |

## Metriche (dev.17 contro candidata)

Dopo ogni fase scaricare la traccia della dashboard («Traccia»). Per fase, con [`tools/trace_phase_analysis.py`](../Motion_Perf_2026-11-07/tools/trace_phase_analysis.py) (sola lettura): release, **overrun** (release oltre 20 ms) e **release perse** (periodi saltati), riportati **separatamente**, lavoro massimo e partizioni (READ / NOMINAL / COMPENSATE / FRAME, lavoro esterno), istogrammi D11 (`command_timing.work_hist`, READ, lateness), minimo stack del loop. Riferimento dev.17 (VERIFIED HARDWARE): WALK STANDARD 310 overrun su 3287 release (9,4 %), 1 persa; joystick 474 / 2353 (20,1 %), 62 perse (1,3/s); HOLD 0 / 4285. A fine H3 e H6 anche il dump USB `@MOTION TIMING`.

Lettura: il 9,4 % **non** è una soglia di accettazione. Obiettivo del budget: overrun e perse **inferiori** a dev.17 in WALK e joystick a parità di fase; overrun uguali o superiori senza release perse né fault non fanno arretrare il collaudo, ma lasciano il budget non validato. Se gli overrun restano ×1,7–2 a lavoro identico (stessa firma solve/IK), il READ separa la causa: READ cresce → bus/UART; NOMINAL + COMPENSATE crescono a READ costante → CPU/cache (C1, UNKNOWN).

COM (H3, WALK 50/75 a terra, dove la correzione non è limitata): media della correzione IMU richiesta raggruppata per gamba in swing. Inclinazioni simili con zampe anteriori e posteriori in swing → COM nella finestra; inclinazione verso la zampa anteriore sollevata dominante → COM più avanti dell'URDF; posteriore dominante → più indietro. Il passo dei dati è la sola telemetria esistente.

Joystick (H6): ritardo intento → riferimento dal `sent_ms` del comando al primo campione con `walk_velocity` cambiato (atteso: refresh ≤ 240 ms + HTTP mediana 57 ms + una release).

## Accettazione

H0–H7 senza fault; release perse non superiori a dev.17 per fase; nessun trascinamento che faccia inciampare al passo scelto come preset a terra; H6 senza neutri né WALK STOP non richiesti. Rifiuto → rollback APP-only a `1.0.1-dev.17`. Un esito positivo qualifica WALK STANDARD v2 e il joystick dinamico con il drive attuale; **non** qualifica WALK FAST.
