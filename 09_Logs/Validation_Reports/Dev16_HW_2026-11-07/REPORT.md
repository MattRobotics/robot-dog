# MATDOG 1.0.1-dev.16 — validazione hardware del 07/11/2026

**Stato: VERIFIED HARDWARE nel perimetro provato; RELEASE STABLE NON ANCORA QUALIFICATA.** Questo documento aggiorna, non riscrive retroattivamente, il rapporto [candidato offline](../Dev16_Candidate_2026-11-07/REPORT.md). Prove supervisionate dall'operatore via dashboard Wi-Fi, con diagnostiche originali numerate (23)–(32) e osservazioni fisiche. [Manifest e limiti](EVIDENCE_MANIFEST.md) · [metriche per finestra](TRACE_WINDOW_METRICS.csv). Nessun comando/flash è stato eseguito dall'assistente.

## 1. Provenienza reale

- Firmware **installato sul robot** dopo flash APP-only: `1.0.1-dev.16`, build `e0ddbb1791d0`, sorgente commit `e0ddbb1791d0726d757fb17e10c4ba2d67daf103` (commit patch PRONE `115032e`), APP di **1.694.048 byte**, SHA256 `73952eec1ddb29976941b7afd16149b15f790a2e8c46cb2067d1f72a8424138d`.
- Flash eseguito da Claude Code su ESP32-S3 MAC `14:c1:9f:22:75:94`, APP-only a `0x010000`, digest readback corrispondente, `APPLICATION_ONLY_FLASH=PASS`. NVS/otadata/bootloader/EEPROM servo non scritti. Esecuzione e preflight riportati dall'agente; identità della dev.16 corroborata dagli screenshot e dall'operazione.
- Boot iniziale: READY/RUN, census 13/13, NVS gen-1 `VALID_ACKNOWLEDGED`, SAFE_OFF 13/13, nessun movimento autonomo; osservazione 77,6 s, ~50 Hz, 0 missed/overrun, lavoro massimo 4,77 ms. Il backend APP installato non coincide automaticamente con il futuro HEAD di `main`.
- Branch PR #64: `docs/dev15-hardware-validation-20261106` (contiene sorgenti dev.15 e dev.16 e report), **non mergiato**. APP/ELF/manifest qualificati conservati localmente sull'ASUS, non asset di Release GitHub.

## 2. Risultati Motion (VERIFIED HARDWARE)

| Prova | Risultato reale / diagnostica |
|---|---|
| IDLE (24) | READY, SAFE_OFF 8191, 0 missed/overrun, 4,745 ms max lavoro, stack minimo 5.456 B; 40,51 s osservati. |
| STAND150 da PRONE_ENTRY + PRONE (23) | `STAND_REACHED` 10,709 s, `PRONE_REST` 12,385 s, senza fault e SAFE_OFF finale 13/13. Primo riscontro che la nuova consegna PRONE supera il blocco della dev.15. |
| STAND100 + HOLD (25), HOLD prolungato (26) | STAND100 11,992 s, ~98,5 mm stimati, HOLD 59 s senza overrun nuovi; ulteriori 130,9 s HOLD senza incrementi missed/overrun. STACK minimo 4.208 B. |
| STAND150 + HOLD (29) | da quota già elevata, STAND150 3,382 s e ~148,9 mm, stabile e senza vibrazioni osservate. Nella finestra di HOLD150 sono comparsi 2 overrun, **0 periodi persi**. Corrente totale riferita ~0,4–0,5 A. |
| WALK45 (30) | START/STOP completati; camminata fisicamente più rettilinea e con sway laterale minore rispetto al joystick, secondo operatore. WALK STOP→STAND ~7,124 s. Il comando WALK aveva 125 overrun; nella finestra campionata (30) 127 overrun e **0 missed**. |
| PRONE da STAND100 (31) | STAND_REACHED, poi PRONE `COMPLETED / PRONE_REST` in 11,975 s; torque OFF 13/13, nessun fault, 0 missed e 3 nuovi overrun nella finestra. Nessuna caduta prematura osservata. |
| Sequenze miste e STOP (32) | 16 comandi registrati: quattro PRONE `PRONE_REST` (11,975; 13,505; 12,509; 13,570 s), tre STOP da movimenti diversi `STOPPED_SUPPORT_UNVERIFIED` (1,944; 2,332; 2,372 s), STAND100/150, WALK45/60; un PRONE interrotto volontariamente da STOP (NON è un fallimento). Tutte le 16 richieste HTTP inviate e conservate sono HTTP 200. Nessun fault; ultima posa READY / PRONE_REST, `safe_off_mask=8191`, `torque_active=false`. Nella finestra (32) **+7 missed, +154 overrun**. |
| Spegnimento finale | Operatore conferma KEY OFF sul BMS: discharge MOS interno ha tolto alimentazione **sia a ESP32-S3 sia ai motori e ai carichi** nel cablaggio attuale. Nessuna procedura addizionale richiesta. |

**Limiti della prova:** `STOPPED_SUPPORT_UNVERIFIED` conferma uno STOP del Controller, **non** certifica contatto a terra; durante STOP può rimanere coppia ON. Per le prove positive di PRONE è stata osservata la discesa fino all'appoggio e poi OFF; l'appoggio passivo non è misurato da un sensore. La `height_estimate_mm` deriva da encoder/IMU, non da un misuratore esterno.

**Limite metodologico dei JSON:** i 240 campioni e 48 comandi massimi sono buffer a finestra; comandi/poll_errors conservati possono essere precedenti alla finestra dei campioni. I campioni dei file derivano da sessioni sovrapposte; **non sommare i delta tra file**. Il campo `command_timing.overruns` è per comando e può azzerarsi al cambio comando; `overrun_releases` e `missed_releases` sono cumulativi dal boot. Per i delta usare primo/ultimo campione della singola finestra; `poll_errors` è limitato in memoria e non rappresenta necessariamente il totale storico.

## 3. Trasporto HTTP e dashboard — problemi APERTI

- Da terminale ASUS→AP: ICMP **60/60 ricevuti**, min/avg/max RTT 1,930/27,890/111,961 ms; picchi 105–112 ms ricorrenti. La mancanza di packet loss non prova latenza di trasporto stabile.
- `GET /status` in serie, con dashboard aperta: ripetuti `HTTP 503` immediati, `HTTP 200` anche a ~1,9 s, una risposta HTTP 200 incompleta al timeout 2 s (1.409/1.899 byte). Nel sorgente l'endpoint generale è sottoposto al controllo di slack Motion, con soglia 5 ms: rifiuti possibili anche senza missed release.
- `GET /api/motion/status`: **20/20 HTTP 200**, tempi ~0,025–0,141 s, dimensione ~4.928–4.935 byte, nessun timeout. È esente da quel particolare gate; il test non dimostra affidabilità di tutte le route o di tutte le condizioni di carico.
- Nel browser: molti `TIMEOUT_800MS`, `poll_skipped`, banner intermittenti `MOTION_BUDGET_UNAVAILABLE` e `DATI NON AGGIORNATI`. Distinguere errori di `/status`, polling Motion, scheduling HTTP e connessione radio. **Non occultare fault reali, non ridurre priorità STOP**.
- Gli alert e l'ultimo comando variabili fanno **saltare verticalmente i controlli del joystick**; impossibile mantenere pressione stabile. Il codice dashboard ha `pointerleave` nel rilascio oltre agli eventi di cattura. Correzione UI: messaggi in slot fisso/toast senza reflow, input stabile con `pointerup`/`pointercancel`/lost capture e watchdog/deadman invariati.
- Twin: Reset vista +X verificato; l'operatore ha osservato rotazione grafica durante STAND senza corrispondente yaw fisico, correlata a variazione quaternion IMU in assestamento; origine esatta non ancora dimostrata. **Refresh con torque ON lascia Twin vuoto**: `MotionDashboard.h` rinvia inizializzazione iframe fino a `safe_off_verified` e `!torque_active`; scelta da riprogettare senza caricare asset pesanti durante Motion se interferiscono con timing.
- Nuovo cockpit richiesto: Twin 3D sempre in alto a tutta larghezza, joystick e STAND/PRONE/WALK/WALK STOP nello stesso pannello; eliminare badge dev.15 e prose joystick; quota STAND **50–150 mm passo 5 mm**; selettore stride **30/45/60 mm/ciclo** con tartaruga→lepre (non confondere stride con velocità reale); **ViewCube CAD con triade, click facce e trascinamento**, Reset vista invariato; STOP sempre visibile; nessuna logica safety critica nel browser.

## 4. Motion timing / prestazioni — problemi APERTI

- T0/T1 fisicamente funzionali; in HOLD100 nessun nuovo overrun/missed in 59 s + ulteriori 131 s, mentre in HOLD150 due overrun e zero missed. La variabilità di durata della stessa fase HOLD (precedente dev.15: ~7,4→33,1 ms, ~4,5×) ha **causa non ancora provata**; considerare I-cache, preemption, altri percorsi solo dopo misure per-release (completo vs leggero). Non aggiungere profiling permanente se non necessario.
- WALK45 e joystick hanno ancora overrun; (27) joystick yaw +54 missed/+504 overrun; (30) WALK +0 missed/+127 overrun; (32) sequenze e STOP +7 missed/+154 overrun. Queste sono finestre distinte, non addendi. T1 migliora ma non soddisfa sempre 20 ms.
- Gait lenta: WALK45 velocità comandata ~6,92 mm/s, WALK STOP→STAND ~7–9 s; joystick aumenta sway laterale. Obiettivo futuro fino a 5× **non qualificato**; affrontare dopo stabilità operativa, senza eliminare guardie fisiche.
- Risultati termici: picchi isolati incoerenti (ID21/ID31, compreso campione 108 °C) che tornano in pochi centinaia di ms a ~25–35 °C; mancata conferma di surriscaldamento reale **non dimostra che il canale telemetria sia corretto**. Conservare le protezioni termiche.

## 5. Recupero operativo STOP → WALK: richiesta concreta

**VERIFIED HARDWARE (operatore):** gli STOP durante WALK, salita STAND e discesa PRONE sono reattivi e completati; ma per avviare nuovamente WALK dopo STOP il cockpit richiede prima un comando STAND. Se l'ultimo valore STAND selezionato è 150 mm, MATDOG sale a 150 e poi ridiscende alla quota WALK ~120: transizione superflua, lenta e inefficiente.

**REQUISITO da implementare**, non comportamento già presente: comando WALK esplicito deve poter partire da STOP completato con supporto/posa/feedback certificati, eseguendo **internamente** l'eventuale ENTRY/PRIME e passaggio alla quota nominale WALK, senza costringere a STAND150. Se la posa non è ammissibile, rifiuto motivato; non trasformare STOP in restart automatico né bypassare supporto, limiti, collisioni e deadman. ESP32-S3 unica autorità della transizione. Test da fixture/supervisione dopo patch.

## 6. Decisione di fine sessione

**Dev.16: hardware validata nel perimetro STAND/PRONE/WALK/STOP provato, NON ancora release stabile.** Non ripetere prove PRONE già positive senza causa. Priorità correttive: (1) HTTP/UI controllo affidabile e Twin refresh; (2) STOP→WALK diretto qualificato; (3) timing locomozione e RCA variazione HOLD; (4) UX cockpit; (5) velocità gait successiva. Nessuna patch, flash, merge, tag o release eseguita in questa sessione. Robot spento fisicamente con KEY OFF come riferito dall'operatore.
