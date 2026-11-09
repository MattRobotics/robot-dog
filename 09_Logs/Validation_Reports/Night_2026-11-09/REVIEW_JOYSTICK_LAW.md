# Review pre-flash dev.5 — legge del joystick dopo una sterzata (09/11/2026)

Revisione indipendente della candidata `1.0.2-dev.5` (`3afe6ce`). Tutto offline: robot spento, nessun flash, nessun merge. Il binario CLEAN della dev.5 resta intatto come riferimento.

## 1. Sequenza AVANTI → DIAGONALE → LATERALE → AVANTI con lo stick sempre premuto — confermata

Regressione deterministica `joystickWalk` F (`test_simple_motion.cpp`), su `3afe6ce` e sulla patch, risultato identico. Comandi ogni 240 ms come la pagina, 30 s per tratto.

| Tratto | Legge attiva | Sway Y | vx / vy | Transizione |
|---|---|---|---|---|
| AVANTI | WALK STANDARD (5,0 s) | 1,5 mm | 20,00 / 0 mm/s | — |
| DIAGONALE | sterzata (6,5 s) | 14 mm | 6,54 / 1,01 mm/s (saturata sull'ellissoide sterzato) | `JOYSTICK_STEERING`: WALK STOP, 6,3 s dal comando alla nuova camminata (0,8 s senza camminare) |
| LATERALE | sterzata | 14 mm | 0 / 3,08 mm/s | nessuno stop |
| AVANTI | **sterzata** | rientra sotto 5 mm in 20 s, poi 2 mm | **6,92** / 0 mm/s | nessuno stop |

**Confermato:** dopo la sterzata il tratto finale resta sulla legge sterzata, a 6,9 mm/s invece di 20, fino al rilascio. Rilasciando e ripremendo, la camminata riparte sulla WALK STANDARD.

## 2. Difetto più grave emerso dalla stessa analisi — corretto

Nel tratto AVANTI dopo una sterzata, con il **jitter del dito registrato a terra** (0,1–0,4 mm/s laterali), la dev.5 tiene lo sway a **11,9–14 mm** per tutti gli 80 s. È il difetto misurato sulla dev.4 (28 mm p-p). Causa: la banda frontale (20 %) veniva applicata solo per scegliere la legge a inizio camminata. Dentro una camminata sterzata il jitter restava una domanda laterale e la mappa dello sway la portava al pieno.

**Patch minima** (`SimpleMotion::joystickGoal`, una riga più commento): anche nella legge sterzata, una richiesta sotto la banda vale come solo avanti/indietro, con laterale e yaw a zero. La legge sterzata li ritira con i propri ratei già certificati: avanzamento laterale limitato e sway cambiato solo con quattro appoggi. Nessuna nuova legge, stato o protezione.

- Con la patch, nello stesso scenario: sway **2,0 mm**, avanzamento laterale 0.
- Conseguenza voluta: una sterzata deliberata sotto il 20 % viene ignorata in entrambe le leggi. La regola è unica: sotto la banda la richiesta è solo avanti/indietro.

## 3. Passaggio sterzata → WALK STANDARD senza WALK STOP — non esiste con la macchina attuale

- `gaitRetarget` accetta solo variazioni di avanzamento, yaw e sway laterale. Da sterzata a v2 cambiano anche periodo (6,5 → 5,0 s), duty (0,825 → 0,80), sway longitudinale (10 → 3 mm), COM shift (0 → 11 mm), profilo di swing, box (`kSteer` → `kHeight`) e inviluppo IMU: il cambio viene rifiutato.
- Scambio diretto dei parametri allo stesso ciclo, scegliendo l'istante migliore: il vettore piede–corpo salta di almeno **4,6 mm** (i piedi in appoggio dovrebbero scivolare) e un giunto di **21 tick**. Il limite del drive WALK è 20 tick per release, quello STAND 9,6.
- Un passaggio continuo richiede un pianificatore di transizione nuovo (spostamento del corpo nelle finestre a quattro appoggi, rampa del periodo, cambio del duty, consegna tra box e inviluppi) più la sua certificazione. È un ridisegno: **non implementato**.

**Alternativa minima, da decidere (non implementata):** passaggio simmetrico con WALK STOP. In camminata sterzata, una richiesta solo avanti/indietro **persistente** (ad esempio ≥ 2 s, per non fermarsi quando si passa dritti da destra a sinistra) chiude la camminata e la leva tenuta riparte sulla WALK STANDARD. Costi: ~6,3 s di pausa a ogni raddrizzamento, il robot si ferma quando l'operatore raddrizza, uno stato di persistenza in più. Oggi lo stesso risultato si ottiene rilasciando e ripremendo lo stick.

## 4. Sensibilità della soglia del 20 %

- Sulla pagina la frazione laterale vale |sin θ| dello stick: il 20 % corrisponde a θ ≈ 11,5° dalla verticale. Il jitter registrato a terra arriva al 13 % (θ ≈ 7,5°).
- Regressione `joystickWalk` G: una camminata avviata al 19 % resta WALK STANDARD. Richieste alternate 18/22 % ogni 240 ms per 120 s producono **un solo** cambio di legge, perché dentro la camminata sterzata non si torna indietro.
- Con la patch, nella camminata sterzata il 18 % non produce sway: niente oscillazioni di legge né di sway attorno alla soglia.

## 5. I quattro test di trasporto che falliscono anche sulla dev.4

| Test | Causa | Rilevanza operativa e STOP |
|---|---|---|
| `test_ap_transport` («cookie protections») | Lo stub di test salva gli header in una mappa: il secondo `Set-Cookie` (`md_device`, aggiunto il 2026-10-11, `baec349`) sovrascrive `md_session`. Il firmware invia entrambi con `HttpOnly; SameSite=Strict`, `md_session` con `Max-Age=300` | nessuna; la fixture del test va aggiornata |
| `test_ap_ingress` | errore di link: il test non compila `Hmac256.cpp`, richiesto da `PortalSecurity.cpp` | nessuna; lista dei sorgenti del test obsoleta |
| `test_motion_transport_mode` | cerca nel sorgente un testo (`operatingMode() != OperatingMode::MAINTENANCE ||`) che non esiste più | nessuna; ancoraggio testuale obsoleto |
| `test_motion_safety_boundaries` | legge `src/execution/ValidationProfile.cpp`, file rimosso | nessuna; percorso obsoleto |

Il trasporto e lo STOP sono coperti da test aggiornati e PASS: HTTP Motion (97 controlli), contesa del fence con la corsia STOP (723), AP-only (134), contratto dashboard e corsia STOP (6175), STOP support. I quattro test non nascondono un difetto operativo, ma andrebbero riparati a parte.

## 6. Esito

| | |
|---|---|
| Problema segnalato | **confermato**: dopo una sterzata il passo frontale resta sulla legge sterzata (6,9 mm/s) fino al rilascio |
| Comportamento desiderato | avanti/indietro coerente con WALK STANDARD anche dopo una sterzata |
| Correzione continua | **non esiste** senza un pianificatore di transizione nuovo (§3): ridisegno, non implementato; decisione proposta: tenere il rilascio-e-ripressione oppure il WALK STOP simmetrico con persistenza |
| Difetto emerso e corretto | jitter dopo una sterzata → sway 14 mm: **patch minima** `5e0faa8`, sway 2,0 mm |
| Test | SimpleMotion (nuovi F, G, H e mappa sway; H e la mappa falliscono su `3afe6ce`), STOP, PRONE 4/4, dashboard 6175, gate joystick 43 PASS (supporto minimo +0,64 mm in `random_3` sterzato, dev.4 +0,57; dev.5 +0,96 in un caso frontale; inclinazione piede 11,65° < 12°), digest invariati per HOLD, WALK75/100, WALK75 STOP e joystick sterzato a regime; diversi per costruzione nei casi sterzati casuali, 0 frame fuori box |
| **Candidata per il prossimo flash** | **`1.0.2-dev.6`**: sorgente `2c53ecfad6645d23655647c6df86142ec24b7a0d`, build CLEAN `ROBOT_POWERED`, APP **1.725.744 B**, SHA256 **`eef32e6133b504f658c5bc76bb41fb2281c3486ecae1f66ed0ffd253acf214c1`**, RAM 138.012 B, tabella `8f756ecb…` invariata, scrittura `app0` 0x010000–0x1b5530; script `~/MATDOG/verification-artifacts/MATDOG_1_0_2_DEV6_FLASH_PREP_20261109/flash-qualified-app0-1.0.2-dev.6.sh` (REFUSE senza `MATDOG_DEV6_FLASH_AUTHORIZED=YES_ROBOT_SUPPORTED`, controlli verificati offline), rollback 7/7 OK |
| Riferimento conservato | dev.5 `3afe6ce`, APP `da6d0ad6…`, mai flashata, superata dalla dev.6 |
