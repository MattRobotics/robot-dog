# MATDOG — Piano prestazioni post-dev.15 (target fino a 5×)

**Data:** 06/11/2026. **Natura:** piano tecnico e target verificabili, **non implementazione**, non autorizzazione a flash. Riferimenti: [report hardware](REPORT.md), [window metrics](WINDOW_METRICS.csv), [video audit](VIDEO_AUDIT.md), [issue Motion #52](https://github.com/MattRobotics/robot-dog/issues/52).

## Baseline misurata / NON misurata

- Il firmware **realmente installato** è dev.15 `7997ad4d4497`; sorgente `7997ad4` ancora locale e assente da GitHub main.
- Periodo di release Motion impostato 20 ms (50 Hz); lavoro massimo osservato **49,586 ms** (più di 2,4 volte il budget) e **51,159 ms max gap** in uno snapshot cumulativo della sessione joystick. Centinaia di missed release per finestra di ~84 s; `COMPENSATE` massimo **32,408 ms o 33,109 ms** in altre finestre. Serve prima una attribuzione nello **stesso release** su build esatta (READ, NOMINAL, COMPENSATE, CONVERT, FRAME, REST, numero solve/IK).
- Log VIDEO: oscillazione del marker laterale ~6,5 s; **non** prova periodo esatto del gait né velocità a terra. Esiste un ritardo di assestamento `WALK_SETTLE→STAND` ~7,3–8,7 s dopo rilascio joystick/WALK STOP, distinto dalla latenza di invio e accettazione del comando.
- Limiti **comandati** dev.15 riferiti nella sessione: `vx` ~6,9 mm/s, `vy` ~3,1 mm/s, `yaw` ~1,76°/s. Nessuna velocità effettiva al suolo in mm/s misurata. **Non** inferire velocità e coppia massime ST3215 da questi tre valori.
- Livellamento IMU: roll desiderato fino ~4,8°, applicato ~1,9° nella prova; pitch fino ~14,0° desiderato e ~13,5° applicato. Non considerare queste correzioni equivalenti al livellamento fisico globale.

## Quantificazione 2×, 3×, 5× (esempi di scala, NON setpoint autorizzati)

| Fattore richiesto | vx massimo comandato equivalente | vy massimo comandato equivalente | yaw massimo comandato equivalente | Periodo visivo marker se scalato linearmente |
|---|---:|---:|---:|---:|
| baseline dev.15 | ~6,9 mm/s | ~3,1 mm/s | ~1,76°/s | ~6,5 s |
| **2×** | ~13,8 mm/s | ~6,2 mm/s | ~3,52°/s | ~3,25 s |
| **3×** | ~20,7 mm/s | ~9,3 mm/s | ~5,28°/s | ~2,17 s |
| **5×** | ~34,5 mm/s | ~15,5 mm/s | ~8,80°/s | ~1,30 s |

**Attenzione:** moltiplicare il limite di comando NON implica aumento identico di velocità al suolo. Il marker video ha componenti di sway e traslazione prospettica; la legge di scalatura del periodo è soltanto una stima obiettivo, non un modello cinematico validato. La velocità reale richiede prova di distanza a terra misurata e tempo effettivo.

## Ordine dei lavori, con primo criterio di successo

### P0 — PRONE fisicamente sicuro e ripristinabile

1. Confrontare la prima divergenza tra due PRONE `ABORTED` a 52–53 mm e due `PRONE_REST` riusciti, sul sorgente locale esatto `7997ad4`. Identificare predicato bloccante (arrived, loaded settlement, feedback fresco, next box, deadline) usando i JSON (15),(18),(22) e codice, non ipotesi su servomotori.
2. Evitare che fallimento di completamento comporti **caduta di ~50 mm** prima del supporto del corpo: conservare via STOP/SAFE_OFF la possibilità di interrompere l'energia, ma non trattare torque OFF da quota sospesa come normale esito PRONE. La soluzione deve rispettare safety fisica e possibilità d'intervento, non semplicemente mantenere coppia indefinitamente in un fault.
3. Recovery supervisionato quando misure RAW/postura e regione certificata ammettono un riavvio STAND; mai cancellare fault latched o ignorare collisioni senza prova.
4. Test offline del caso storico + prova hardware sulla fixture sostenuta, senza ripetere cadute non necessarie.

### P1 — Motion 20 ms sostenibile e comandi raggiungibili

> **RCA eseguita il 07/11/2026:** [TIMING_RCA_2026-11-07.md](TIMING_RCA_2026-11-07.md). Sintesi: il calcolo double software di IK nominale + IK di compensazione supera i 20 ms (WALK45 ≥ 18,4 ms, joystick ≥ 28,2 ms); regressione `2a630a0` (rimosso il solve IMU a release alterne di dev.7); 503 = assenza di slack di 5 ms. Patch minima raccomandata T0 + T1, non implementata.

1. Profilare una release reale con attribuzione **prima deadline saltata**: READ, NOMINAL, COMPENSATE, CONVERT, FRAME, REST, numero solve, stile di letture UART; verificare `maxWorkUs`, `maxGapUs`, `missedReleases`, heap/stack e latenza STOP prima/dopo patch.
2. Cercare il lavoro ripetuto o non necessario in `COMPENSATE` (risoluzione IK, verifiche duplicate, calcolo nel HOLD, aggiornamenti che non cambiano i target). Priorità a caching/cadenza corretta e diff minimi; **non** rimuovere limiti, feedback stale, termica, collisioni o STOP.
3. Distinguere CPU/servo bus da **HTTP 503 `MOTION_BUDGET_UNAVAILABLE`** dopo ~100 ms: verificare dispatch, lock contesi, starvation e motivo preciso di rifiuto. Il comando deve arrivare all'autorità Motion senza deadline artificiali spurie.
4. **Target ingegneristico**: nessuna missed release nelle prove equivalenti a quelle già fallite, nessun picco >20 ms e margine utile (orientativamente p99 lavoro <=15 ms, se test adeguato); non dichiarare successo solo perché un contatore resta a zero mentre esistono work>20 ms. Dati tracciati per release e STOP sempre prioritario.

### P2 — Movimento utile, non solo servo cap più alto

1. Misurare baseline **vera** su pavimento piano: velocità al suolo e distanza/ciclo, stride e step frequency, durata di sway vs foot swing/stance, tracking RAW per giunto, corrente/termica BMS e slip/piedi.
2. Eliminare trasferimenti laterali *eccessivi* e precarico all'indietro solo se support polygon, posizione di contatto e margini IK/collisione restano validi. Nessuna diminuzione cieca del sway basata su pixel.
3. Ridurre `WALK_SETTLE→STAND` da ~8,4 s verso **<3 s** come primo target, poi valutare ~1,5–2 s su robot stabile; mantenere **stop del comando immediato** al rilascio joystick e deadman. Distinguere stop logico, touchdown e postura nominale; evitare sequenze redundantemente lunghe per cambiare direzione.
4. Incrementare con gradini **2×, 3× e fino a 5×** solo dove tracking, contatto/supporto, alimentazione, correnti, workspace e timing dimostrano margine, senza alterare limiti hardware a priori. Target iniziale allineato alla issue #52: velocità longitudinale utile ≥20–25 mm/s, poi ~35–40 mm/s dove fisicamente ammissibile. **Non** proclamare 5× prima di misurare la velocità reale.

### P3 — ROLL e qualità percepita

Verificare margini workspace a quote 50/100/120/150 mm e nella posa con quattro appoggi; se i vincoli `WORKSPACE` sono eccessivamente conservativi, correggere **solo** il modello/allocazione dimostrati errati. Lo swing nel gait non è prova di libertà identica di livellamento statico. Mantenere rate limit sensati e osservare risposta su tavola spenta/inclinata.

### P4 — HostLink e cervello Jetson

Quando Motion core è affidabile: riprendere USB HostLink senza DTR/RTS e `--no-stub` solo per flash; validare comando/ACK, deadman e recupero ai distacchi con Jetson come planner alto livello. **Jetson non deve possedere bus servo, IK safety o autorità Motion**. Nessuna funzione AI rende accettabile un robot lento/incontrollabile.

## Cose da preservare

Firmware sul device e NVS gen-1, 12 ID più collo51, conversioni joint↔servo, URDF REV01-FIX, limiti/collisioni, feed-back servo, STOP, watchdog, BMS, LED, Digital Twin e proporzionalità già verificata. No reflash/merge automatico dalla PR documentale. **Una sola RCA e patch circoscritta per sottosistema**, con test pertinente, prima di rivalutare altri colli di bottiglia.
