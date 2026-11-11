# MATDOG `1.0.2-dev.7` — validazione hardware supervisionata dell'11/11/2026 (fixture e pavimento)

Firmware installato e provato: **`1.0.2-dev.7`**, build `b5610620d0ce`, sorgente CLEAN `b5610620d0cef40cdc34bd62bf6256857bc9cfde`, APP SHA256 `28744982e293d5a56a0246c7ccacd083d1a0040f70ece29da4ac72fbfe6053d4` (1 757 472 B), app0 `0x010000`, profilo `ROBOT_POWERED`. Candidata integrata Motion vNext (joystick multidirezionale, `a7f5c2c`) + Battery P1. **dev.7 è VALIDATED HARDWARE nel perimetro provato qui sotto, NOT STABLE.** La Stable resta `v1.0.1` (dev.17).

I commit successivi a `b561062` su questo branch contengono solo documentazione (`09_Logs/`): non cambiano il firmware installato.

Evidenze e SHA256: [EVIDENCE_INDEX.md](EVIDENCE_INDEX.md). Punti aperti: [OPEN_ITEMS.md](OPEN_ITEMS.md). Ripresa: [NEXT_SESSION_HANDOFF.md](NEXT_SESSION_HANDOFF.md). Piano seguito: `PREFLIGHT_CARD.md` (fuori repository, vedi indice).

Classi: **VH** = verificato su hardware; **VO** = verificato solo offline; **UNKNOWN** = non determinabile dai dati disponibili.

Fonti dei numeri: telemetria HTTP a 1 Hz (`/api/motion/status`) e ogni 5 s (`/status`), log seriale USB. Una durata «verificato dopo 2,0 s» ha la risoluzione di 1 s del campionamento.

## 1. Configurazione e condizioni

| Voce | Valore | Classe |
|---|---|---|
| Massa e COM | **Senza Jetson Orin** (il modello URDF REV01-FIX la include: 175 g dentro `base_link`, 2,928 kg totali); circa 20 g di cavi in più vicino al baricentro (dichiarazione dell'operatore). La posizione della Jetson nel `base_link` non è nelle evidenze: lo spostamento del COM rispetto al modello è UNKNOWN (circa 0,06 mm per ogni mm di distanza della Jetson dal COM) | VH (configurazione) / UNKNOWN (COM) |
| Batteria | Inizio 12,1 V, cella 4050 mV, SOC 97,5 %; fine 11,9 V, cella min 3971 mV, SOC 92,1 %; allarmi DALY sempre `0000 0000 0000 0000` | VH |
| Fixture | Zampe libere, corpo trattenuto | VH (dichiarazione) |
| Terra | Pavimento. Cavo USB di circa 1 m: l'operatore ha riposizionato a mano il robot in STAND/HOLD con le coppie ON (es. 19:34:34–36Z, pitch voluto fino a −197 mrad). Uso dell'anticaduta non registrato nei log | VH / UNKNOWN (anticaduta) |
| Comandi | Tutti i comandi Motion dalla dashboard, dall'operatore. Il registratore USB ha inviato solo diagnostica in lettura (`@MOTION DIAG` automatici, `QUIT`) | VH |

## 2. Flash APP-only e H0 (robot sulla fixture) — VH

| Controllo | Esito |
|---|---|
| Prima del flash | `1.0.2-dev.6` / `450d2c793dcb`, preflight 8/8, 12,1 V |
| Gate offline | PASS (script con pin su `b561062`; intervallo di scrittura `0x010000..0x1bd120`; rollback `450d2c7`) |
| Layout | tabella `8f756ecb…`, app0 attiva, rollback abilitato |
| Flash (18:51:11–18:52:01Z) | 1 757 472 B scritti in `0x010000`, «Hash of data verified», `verify-flash` digest matched, `APPLICATION_ONLY_FLASH = PASS`. Nessuna scrittura di bootloader, partizioni, NVS o EEPROM servo |
| Boot (watchdog-reset) | banner `1.0.2-dev.7` / `b5610620d0ce`, un solo boot, nessun panic |
| H0 | preflight 8/8, READY, BMS OK, SAFE_OFF verificato mask 8191, coppia OFF; `bms_remaining_estimate_ah` assente (Battery P1) |

## 3. Fixture F0–F6 (18:52–19:30Z) — VH

| Prova | Comandi e direzioni effettivamente eseguiti | Esito e evidenza determinante |
|---|---|---|
| F0 identità | sola lettura | **PASS** |
| F1 STAND/STOP/STAND | STAND 120, STOP, STAND | **PASS** — USB `MOTION_ACC_CEILING verified=220 target=220`; STOP verificato in 2,0 s con coppia ON (18:56:38); ripresa a STAND_REACHED |
| F2 SLOW joystick + STOP | SLOW avanti 9,5 mm/s, STOP (Esc) | **PASS** — STOP verificato in 2,0 s, coppia ON (18:59:10); servo 1000/50 |
| F3a SLOW | laterale ±, yaw ±1,83 °/s, indietro −5,0 mm/s, STOP in sterzata; WALK preset 20 mm/s fuori piano alle 19:00:53 | **PASS** — STOP verificati 19:00:55 e 19:04:57 |
| F3b STANDARD | laterale +2,9/−3,4 mm/s, yaw ±2,29 °/s, **indietro −12,0 mm/s (primo su HW)**, avanti 14,2 mm/s, STOP in sterzata | **PASS** — STOP verificato 19:10:06; servo 1800/180; margine HIP ≥ 10,3 tick |
| F3c FAST | laterale +2,7/−2,4 mm/s, yaw ±1,83 °/s, **indietro −10,6 mm/s (primo su HW)**, avanti 32,0 mm/s (massimo), laterale 4,8 mm/s, STOP | **PASS** — STOP verificato 19:15:00; servo 1950/220; carico massimo 388 ‰ |
| F4a cambio modo in WALK (preset) | SLOW→STANDARD, STANDARD→FAST, FAST→SLOW, FAST→STANDARD, STOP in transizione | **PARZIALE.** SLOW→STANDARD in 8 s e STANDARD→FAST in 4 s: PASS. STOP durante il cambio pendente FAST→SLOW (19:19:06): verificato, PASS. **FAST→SLOW e FAST→STANDARD: FAIL**, mai completati (clock e quota cambiano, forma e drive restano FAST a 16 mm/s; nessun fault). Vedi §6 e OPEN_ITEMS P1 |
| F4b selezione a leva tenuta | — | **NON ESEGUITO** |
| F5 avviso batteria | — | **NON APPLICABILE** (nessun avviso spontaneo). A fine sessione, a coppie OFF, `led_render_deferred=NO` e anello aggiornato |
| F6 timing | SLOW diagonale indietro-sinistra, laterale, diagonale avanti-destra; FAST avanti 32 mm/s; con tab e finestre cambiate | **PASS** — perse ≤ 2,6 ogni 30 s, overrun 4–8 % (circa 22 % il 10/11), picchi 31–41 ms |

Totale fixture: **7 STOP su 7 verificati con coppia ON**, 0 fault, 0 SAFE_OFF; release perse 0→146 (soprattutto nelle riprese dopo STOP), overrun 3645, lavoro massimo per release 46,69 ms (F3a/F3b); temperatura massima confermata da due campioni consecutivi 40 °C (ID 42). Su fixture la correzione IMU è in WORKSPACE (voluta +57/−66 mrad, applicata ±32): inclinazione della fixture, già vista il 10/11.

Trasferimento: STAND 105 mm in HOLD sulla fixture (19:28:32–19:30:41Z), PRONE (19:30:42), PRONE_REST a coppie OFF (19:30:54), robot posato a terra; STAND con acquisizione della posa reale alle 19:31:14. Nessun riavvio, nessun latch.

## 4. Terra (19:31–19:49Z) — VH

### 4.1 M1 — cedimento statico delle anche (raw − goal, mediana in HOLD)

| Quota | LF | RF | RH | LH | Quota stimata |
|---|---:|---:|---:|---:|---:|
| 120 mm (19:31:26–19:32:00) | +1 | −2 | −2 | −3 | 118,7 mm |
| 110 mm (19:32:03–19:32:38) | +1 | −2 | −2 | −1 | 108,6 mm |
| 105 mm (19:32:41–19:33:17) | +1 | −2 | −2 | −1 | 103,6 mm |
| 120 mm a fine blocco 1 (19:35:41–19:39:03) | −4 | +2 | −4 | +1 | 118,6 mm |

Massimo **4 tick**. Il modello `servo_plant` usa per lo slot 9 un cedimento di 7 tick a 1,0×; i fallimenti dello STOP in avanti nel modello compaiono da 1,13× in su. Il dato misurato è circa 0,3–0,6×. Vale solo da fermi: il cedimento dinamico durante il trasferimento di peso non è misurabile a 1 Hz (**UNKNOWN**).

### 4.2 Direzioni provate (da `extracts/ground_directions.txt`)

| Modo | Direzioni (massimi registrati) |
|---|---|
| SLOW | avanti 20,0 mm/s; indietro −10,0 mm/s |
| STANDARD | avanti 30,0 mm/s; indietro −16,7 mm/s; sinistra 6,7 mm/s; destra 6,7 mm/s; yaw ±2,29 °/s (9–13 s per verso); diagonali avanti-destra, avanti-sinistra, indietro-destra; giro completo della leva avanti → diagonali → laterali → indietro → avanti (19:47:49–19:49:05) |
| FAST | avanti 32,0 mm/s; avanti-destra 16,1/4,8 mm/s; destra 4,8 mm/s; indietro −20,0 mm/s |

Non provati a terra: FAST sinistra e yaw in FAST/SLOW; SLOW laterale; una camminata continua di 2 minuti (la più lunga è di circa 76 s, con inversioni).

### 4.3 STOP a terra: 7 su 7 verificati con coppia ON

| Ora (Z) | Modo e direzione al momento dello STOP | Verificato dopo |
|---|---|---|
| 19:35:30 | FAST indietro −20 mm/s | 2,0 s |
| 19:42:19 | SLOW avanti | 2,0 s |
| 19:42:50 | STANDARD avanti 30 mm/s | 3,0 s |
| 19:43:28 | FAST avanti 32 mm/s | 2,0 s |
| 19:44:49 | STANDARD yaw orario 2,29 °/s | 3,0 s |
| 19:47:24 | SLOW, durante il WALK STOP dopo il cambio FAST→SLOW | 2,0 s |
| 19:49:06 | STANDARD avanti 30 mm/s | 2,0 s |

Ogni STOP è stato ripreso con STAND (STAND_FROM_STOPPED_WALK → WALK_SETTLING → STAND_REACHED). Nessun SAFE_OFF e nessun fault. Osservazione dell'operatore su uno STOP in avanti in STANDARD: tre piedi in appoggio e uno leggermente sollevato, posa mantenuta. Lo STOP su tre contatti non è certificato offline: qui è stato osservato su hardware in un caso, non qualificato in generale.

### 4.4 Cambi di modo durante il WALK (preset) a terra

SLOW→STANDARD in 9 s (19:46:44), STANDARD→FAST in 4 s (19:46:58), **FAST→SLOW completato in 5 s** (19:47:15→19:47:20). Non provati a terra: FAST→STANDARD e i cambi a leva tenuta.

### 4.5 Margini, carichi e temperature

- Margine HIP minimo dalla box kHeight: **5,3 tick** (SLOW, LH e LF), criterio ≥ 4. Nei tratti sterzati vale la box kSteer: il margine kHeight non si applica.
- Carico massimo: upper 388 ‰, lower 288 ‰ (FAST avanti).
- Temperatura massima confermata da due campioni consecutivi: 37 °C (ID 11, 19:48:50); 36 °C a fine sessione. Letture isolate fino a 150 °C (§6).

## 5. Chiusura

PRONE da STAND (19:49:22), PRONE_REST alle 19:49:36: SAFE_OFF verificato, mask 8191, coppie OFF. Spegnimento con la chiave alle 19:55:33Z (brownout nel seriale). Registratori chiusi alle 19:55:54. **PASS** (equivalente di F7 tramite PRONE_REST).

## 6. Anomalie

| # | Anomalia | Dati | Classe |
|---|---|---|---|
| A1 | **Release perse a terra oltre il criterio operativo** (> 16 in 30 s) | Fino a **43 in 30 s** (19:44:31–19:45:01: yaw in STANDARD, STOP, ripresa). Release perse a terra 146→328: 101 nelle camminate sterzate col joystick, 54 nelle riprese STAND_FROM_STOPPED_WALK, 27 altrove. Overrun a fine sessione 6702. Release più lunga a terra 45,47 ms (19:45:00, ripresa: nominal 18,3 ms, compensate 22,9 ms); compensate massimo 24,57 ms (19:48:03, camminata sterzata). In entrambe una sola chiamata solve, 7–8 IK, 11–12 Newton, 21–23 FK: tempo misurato a muro, prelazione inclusa. Picco dall'avvio 46,69 ms (fixture). `max_gap` 140,8 ms solo nel primo ACQUIRE a coppie OFF (18:56), poi invariato; nessun `MOTION_DEADLINE_LOST` | VH (sintomo) / UNKNOWN (causa) |
| A2 | **Cambio da FAST verso un modo più lento pendente indefinitamente sulla fixture** | Sulla fixture FAST→SLOW e FAST→STANDARD mai completati; a terra FAST→SLOW in 5 s. Durante FAST→STANDARD la correzione applicata era dentro l'inviluppo STANDARD in 29 campioni e fuori in 15. Offline (stesso output su `450d2c7` e `b561062`): FAST→SLOW pendente per 80 s in 3 dei 16 punti della griglia di inclinazione statica, FAST→STANDARD sempre completato. Ipotesi non verificata: correlazione con la fase della finestra a quattro contatti | VH (sintomo) / VO (logica preesistente) / UNKNOWN (causa sulla fixture) |
| A3 | Release perse nelle riprese dopo STOP | Fixture: 14–23 per ripresa (riferimento 10/11: 6–12); terra: 54 in tutto | VH |
| A4 | **Letture isolate di temperatura** | 28 picchi scartati in sessione; 105 riletture dirette registrate, **26 hanno ripetuto il valore alto** (71–150 °C, es. ID 21 a 150 °C alle 19:14:37, ID 31 a 117 °C alle 19:35:03, ID 13 a 98 °C alle 19:49:32), mentre il campione successivo era 29–36 °C. Nessuna finestra calda ha superato la conferma `kHotC=70 °C` per `kHotConfirmUs=500 ms` (`SimpleMotion.cpp:41–42`) | VH (sintomo) / UNKNOWN (causa) |
| A5 | Spostamento laterale del baricentro marcato nelle camminate sterzate (osservazione dell'operatore) | Ampiezza di progetto: SLOW 10, STANDARD 9, FAST 7,5 mm, contro 1,25–1,5 mm in avanti. In FAST, 6,5–7 mm davano supporto negativo offline e 10 mm faceva fallire STOP_PLAN all'avvio sterzato | VH (osservazione) / VO (vincolo) |
| A6 | STANDARD (30 mm/s) troppo vicino a FAST (32 mm/s) (osservazione dell'operatore) | Richiesta: STANDARD a circa 25 mm/s | richiesta |

## 7. Riepilogo per classe

- **VERIFIED HARDWARE (questa build, questa configurazione senza Jetson):** flash APP-only e identità; STAND/STOP/ripresa; SLOW/STANDARD/FAST avanti e indietro, laterali, yaw e diagonali entro le velocità di §3 e §4.2; 14 STOP verificati su 14 (7 fixture, 7 terra), di cui 4 in avanti a terra nei tre modi; cambi di modo verso l'alto; FAST→SLOW a terra; PRONE con SAFE_OFF verificato; cedimento statico delle anche ≤ 4 tick; Battery P1 (pacco e cella in dashboard, nessuna capacità residua stimata, rendering LED rinviato a coppie ON).
- **VERIFIED OFFLINE (non provato o non riproducibile su HW):** la logica di cambio modo pendente è identica a `450d2c7`; i risultati di `servo_plant` sugli STOP nelle finestre di sway e sulla finestra COM all'indietro (2,59 / 2,89 / 2,81 mm) restano quelli di [AUDIT_FIXES.md](../Motion_vNext_Multidirectional_2026-11-11/AUDIT_FIXES.md).
- **UNKNOWN:** cedimento dinamico delle anche; COM reale senza Jetson; causa dei picchi di tempo e delle letture di temperatura; causa del cambio da FAST pendente sulla fixture; uso dell'anticaduta; F4b; camminata continua di 2 minuti; comportamento con la Jetson montata.
