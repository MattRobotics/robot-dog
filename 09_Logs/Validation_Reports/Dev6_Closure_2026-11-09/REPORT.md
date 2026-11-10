# MATDOG 1.0.2-dev.6 — Chiusura tecnica offline e preparazione collaudo

**Data:** 09/11/2026. **Branch:** `feat/gait-redesign-20261108`, pubblicato su `feat/motion-perf-next-20261107`. **PR:** #67, OPEN e DRAFT.

**Installato sul robot:** `1.0.2-dev.4` (`4358f13`). Nessun flash, reset, scrittura NVS/EEPROM, calibrazione o merge.

**Classi di evidenza:**
- **VO** = verificato offline;
- **VH** = verificato su hardware;
- **INH** = implementato, non validato su hardware.

## Verdetto

**READY FOR SUPERVISED FLASH** per la qualifica in SLOW sulla fixture, poi STANDARD e FAST (§9).

- **Fault residuo di STANDARD:** era un difetto software di passaggio di consegne. È corretto e il caso originale ora passa (§3). Nel modello non resta alcun fault riproducibile.
- **STANDARD e FAST** restano **INH**: non sono «qualificate».
- **Motion budget:** **non risolto**. La causa dei picchi hardware non è dimostrata (§7).

**Candidata unica:** `70abcff54cd3e6e1573385efae16d78a5460f9ee`, build `70abcff54cd3`, APP `67d55e1a9d104fc75c9b1d805e2a290022d8bbcf0d3967d6bc232925b46ca834` (§11).

## 1. Interfaccia: un solo selettore (VO)

La riga del passo 🐢/🐇 è eliminata. Resta un solo selettore: **SLOW 20 mm/s | STANDARD 25 mm/s | FAST 32 mm/s**. Le etichette sono calcolate dalla tabella `kWalkModes` letta dalle capabilities.

**Comportamento:**
- **Default:** SLOW al boot; la scelta è volatile.
- **Cambio modalità:** invia subito `GAIT.SELECT`. Durante un WALK vale solo per il prossimo, e la dashboard mostra «In corso: X · prossimo WALK: Y».
- **WALK con selezione in volo:** WALK resta cliccabile mentre è in volo solo `GAIT.SELECT`; il click **attende l'ACK**.
- **Selezione non accettata:** se `GAIT.SELECT` non è ACCEPTED non parte alcun WALK, compare il messaggio e il selettore torna alla scelta del Controller. Se la selezione del Controller è diversa dalla scelta, la pagina la richiede una volta.
- **Dopo uno STOP:** il Controller rifiuta `GAIT.SELECT` (`STOP_LATCHED`, verificato sul sorgente: `MotionService::command`) e la ripresa usa la modalità della camminata fermata. La pagina non invia un WALK per una modalità diversa.
- **Invariati:** STOP sempre accessibile; joystick sempre SLOW, dichiarato in guida. Nessun comando nuovo: `GAIT.SELECT` e `BASE_VELOCITY` come prima. I preset non usati dalla dashboard restano nel firmware.

**Test:**
- **DOM:** «STANDARD poi WALK immediato» produce `GAIT.SELECT`, poi `BASE_VELOCITY` con vx = 0,075/5,0 (preset 1). Con STOP latched, il rifiuto `STOP_LATCHED` non produce alcun WALK e il selettore torna su SLOW.
- **Contratto:** la stessa coppia, catturata dalla pagina, attraversa parser reale, sessione/CSRF, MotionService e SimpleMotion e cammina **STANDARD 75 mm / 3,0 s = 25 mm/s** alla quota 110. Il rifiuto con STOP latched è nella fixture del servizio reale.
- **Esiti:** fixture 84, DOM 92 comandi, contratto 6 112 controlli, browser (layout, ViewCube, STOP durante il caricamento del Twin): tutto PASS.
- **Nota:** una prima esecuzione del controllo browser `trace_input_ms_agrees` è fallita mentre tre simulazioni del plant occupavano la CPU. Rieseguita a macchina scarica: PASS. È un controllo di timing sensibile al carico.

## 2. Tabella effettiva

| Dashboard | GAIT.SELECT | Comando WALK | Preset nel Controller | Gait avviato | Quota | Drive |
|---|---|---|---|---|---|---|
| SLOW 20 | `SLOW` | `BASE_VELOCITY` vx = 0,100/5 | 2 | 100 mm / 5,0 s = 20 mm/s | 120 | {1000,50} |
| STANDARD 25 | `STANDARD` | vx = 0,075/5 | 1 | 75 mm / 3,0 s = 25 mm/s | 110 | {1800,180} |
| FAST 32 | `FAST` | vx = 0,100/5 | 2 | 80 mm / 2,5 s = 32 mm/s | 105 (alzata 6) | {1950,220} |

Le leggi cinematiche non sono cambiate.

## 3. Fault residuo STANDARD: RCA chiusa

**Riproduzione deterministica** del caso originale: frenata simulata 5 600–8 400 step/s², modalità STANDARD, scenario 8 STOP con ripresa, seed fisso. Sulla build precedente 1 fault a 11,1 V e 1 a 9,6 V, entrambi a c0,06: `WALK_IK_OR_WORKSPACE`, refused_by `POINT_SAFE`.

**Traccia strumentata (copia scratch):**
1. Dopo lo STOP, la ripresa passa da WALK_HOLD (fase 11).
2. Lì la correzione IMU **non è limitata dall'inviluppo WALK**. Il fit d'appoggio sulle gambe che inseguono in ritardo la porta a roll/pitch −30/−95 mrad.
3. Il WALK parte con −22/−70 mrad applicati; l'inviluppo di STANDARD è −17 mrad.
4. Al primo passo `compensationTarget` non trova soluzione IK (bersaglio non valido, giunti 7 e 10). Il retry alla correzione precedente fallisce anch'esso → fault.

**Prima causa: riferimento di ripresa scorretto.** È un difetto software di passaggio di consegne: si avvia una camminata fuori dalla regione che i suoi inviluppi certificano. Non è un limite fisico e non dipende da un'ipotesi del plant: il plant ha solo generato la correzione grande. A terra, cedimento e ritardo producono lo stesso effetto.

**Patch minima** (`8ba9d50`):
- `startWalk` parte solo se la correzione mantenuta sta dentro l'inviluppo fore/aft della camminata in attesa. Altrimenti l'HOLD limita la correzione richiesta a quell'inviluppo, senza la zona morta di 1 mrad, e ritenta la partenza a ogni release (motivo `WALK_IMU_SETTLING`).
- Box, prova STOP, tolleranze e timeout invariati.

**Prima/dopo:**
- caso originale: **1 → 0 fault**, 48/48 STOP con ripresa a 11,1 e 9,6 V;
- nuova regressione deterministica in `test_simple_motion` (tavola a 30 mrad di roll durante la ripresa): `WALK_IMU_SETTLING` osservato, STANDARD parte a −9,1/0,1 mrad dentro l'inviluppo, 1 500 release senza fault.

## 4. Regressioni prima mancanti, ora eseguite sul sorgente finale

| Prova | Esito |
|---|---|
| Plant default, 3 modalità × 3 scenari × 2 tensioni × 6 seed | **0 fault**, 48/48 STOP con ripresa per cella STOP |
| Plant frenata 5 600–8 400 (STANDARD) | 0 fault (prima 1) |
| Plant frenata 3 000–5 600 (3 modalità) | 0 fault, uscita dal box 0,0 tick, fine ≤ 2,2 tick |
| Matrice di perturbazioni (60 celle) | 0 fault |
| Version repro R1–R4 (difetti storici) | assenti: sway 1,5 mm, nessuno STOP, WALK100 per 8 000 release |
| Partenze in pendenza ±30 mrad (SLOW/STANDARD/FAST sui preset 2/1/2) | 75/75 senza fault |
| Suite SimpleMotion (ASan/UBSan) | PASS (inclusa la nuova regressione di ripresa e la latch delle modalità) |
| Test host (contratto HTTP) | PASS (contratto HTTP 97 controlli) |
| Audit statico | **PASS** (370 file) |

## 5. QEMU sull'ultimo sorgente (VO)

Questi controlli sono stati eseguiti su questo sorgente (`8ba9d50` + scenari; il sorgente motion è lo stesso della build).
- **Scenari:** Espressif QEMU ESP32-S3, `-icount`; 9 scenari. Coprono SLOW 100, STANDARD 75, **FAST 80** (preset della dashboard), STOP in 8 fasi, ripresa, IMU oltre l'inviluppo, joystick, fault latched, ritardo e cedimento.
- **Equivalenza host/Xtensa: 9/9 SAME.** Ordine degli eventi identico: frame, digest RAW, passo massimo, stato, motivo, primo fault ed exit.
- **Legge conservata:** i digest di SLOW, joystick, STOP, IMU, fault e ritardo/cedimento SLOW sono **identici a quelli della dev.6 precedente**. STANDARD 75 e FAST 80 hanno digest nuovi, perché la legge è diversa: non vanno confrontati con le vecchie leggi.
- **Esito:** nessun crash. Pila minima libera della task Motion 7 320 B su 16 KB.
- **Lavoro deterministico per release:** massimo ~0,78 M cicli (FAST con ritardo/cedimento), medio ~0,22–0,25 M.
- **Limite di copertura:** STOP + ripresa di STANDARD/FAST sono coperti nel plant host, non in QEMU.

## 6. Double contro float: non applicato, con motivo

**Profilo QEMU (Xtensa) di WALK100 SLOW:** chiamate per release moltiplicate per i cicli misurati per chiamata.

| Voce | Cicli per release (media) |
|---|---|
| Aritmetica double soft-float (+, −, ×, ÷, confronti) | ≈ 255 k |
| Trascendentali libm (sin/cos/atan2/asin/acos/atan/sqrt/hypot) | ≈ 143 k |

**Massimo misurato:** ~0,8 M cicli per release, cioè ≈ 3,3 ms a 240 MHz (≈ 4 ms per JOYSTICK).

**Conclusione:** anche eliminando *tutto* il costo double, il risparmio massimo per release è di pochi millisecondi. Non spiega i picchi hardware di 30–48 ms.

**Perché non applico la variante float:**
- il guadagno sui picchi è irrilevante;
- il rischio tocca prove di sicurezza (IK, prova STOP, conversione RAW, box).

Il benchmark comparativo completo (differenze RAW, IK e margini) non l'ho eseguito: senza un beneficio sui picchi non giustifica una modifica alla precisione. Resta un'ottimizzazione possibile in futuro, solo dopo aver identificato la causa dei picchi.

## 7. Motion budget: stato reale (non risolto)

**Attribuito offline:** il calcolo deterministico vale ≤ ~4 ms per release (stima da QEMU, non da cronometro).

**Non attribuito:** la differenza fino a 30–48 ms. Restano ipotesi non dimostrate:
- cache flash (accessi a codice o costanti in flash);
- preemption da altre task o ISR;
- attese UART.

**Banco QEMU con carico concorrente:** non svolto. Un task sintetico non riproduce cache flash né Wi-Fi reale, e senza intervalli misurati sull'hardware il risultato non sarebbe attribuibile.

**`MATDOG_DIAG_PREEMPTION`:** misura il tempo delle altre task con `uxTaskGetSystemState` due volte per release.
- **Perturbazione propria:** sospensione dello scheduler e scansione delle task (~decine di µs, piccola rispetto ai picchi).
- **Cosa non vede:** non attribuisce le ISR e non misura gli stalli di cache.

**Misura hardware minima:** build diagnostica in sessione dedicata (non la candidata) che registri per release, quando l'intervallo supera 25 ms:
- tempo delle altre task;
- `MAX_WORK` per sezione (READ, COMPENSATE, NOMINAL, CONVERT, FRAME);
- durata delle letture UART.

Se le altre task e la UART non spiegano il picco, il sospetto principale diventa la cache flash, da verificare spostando in IRAM le funzioni calde.

## 8. Margini per le tre velocità (invariati, VO salvo dove indicato)

| | SLOW 20 | STANDARD 25 (75 mm) | FAST 32 (80 mm) |
|---|---|---|---|
| Prova STOP a 8 400 step/s² (frenata misurata a vuoto) | PASS | PASS | PASS |
| Supporto statico / capture point | 2,12 / 1,41 mm | 2,22 / 1,84 mm | 1,71 / 1,41 mm |
| Inviluppo IMU, pitch-down / roll | −12,8 / ±19 mrad | −17,1 / ±9,1 mrad | −34,2 / ±11,8 mrad |
| Tracking nel plant, massimo | 66–72 tick | 108–163 | 131–159 |
| ZMP durante lo STOP | esce in modo transitorio in tutte le modalità (sollevamento di bordo, non caduta) | idem | idem |

**Non coperto dal plant:** non accoppia corpo, contatto e IMU, quindi questi margini **non dimostrano la stabilità**. ACC 180/220 non sono qualificate; 8 400 è un dato a vuoto. L'alzata di FAST è 6 mm: la luce reale dal terreno va verificata.

## 9. Limiti fisici da misurare sulla fixture (ordine minimo)

1. Flash, boot in sola lettura, SLOW di regressione (WALK100, STOP, PRONE).
2. **Sulla fixture, zampe libere:**
   - STANDARD 25, poi FAST 32;
   - registrare goal/present, corrente e temperatura;
   - 8 STOP per modalità: distanza d'arresto reale contro piano, ritardo contro il plant.
3. STOP sotto carico, a terra con supporto: frenata equivalente ≥ 5 600 step/s².
4. **A terra:** STANDARD poi FAST. Registrare la correzione IMU richiesta e applicata, `WALK_IMU_SETTLING`, e la luce del piede FAST.
5. In una sessione separata, la build diagnostica del budget (§7).

## 10. Fattibilità futura TROT > 40 mm/s (solo valutazione)

Il limite di 32 mm/s appartiene al WALK a quattro tempi staticamente stabile, alla sua prova STOP e al suo supporto. **Non è la velocità massima del robot.**

1. **Cinematica e workspace.** Le coppie diagonali alternate sono compatibili con le IK e con le box esistenti. Il passo per gamba di un trot con duty ~0,5 è simile a quello del crawl. Con swing di ~0,3–0,4 s si ottengono 50–60 mm/s con picchi di velocità articolare ~2–3 rad/s, sotto la velocità a vuoto (4,3 rad/s). Le accelerazioni ~30–50 rad/s² richiedono ACC ben oltre 50, mai misurata.
2. **Oltre 40 mm/s:** teoricamente sì per i limiti articolari. Il vincolo diventa la frenata e la stabilità dinamica, non la cinematica.
3. **Modifiche minime al generatore:**
   - fase per coppie diagonali (duty ≈ 0,5–0,6);
   - traiettoria del corpo senza sway di supporto statico;
   - riferimenti ZMP o capture point al posto del criterio di poligono statico.

   Restano riusabili IK di contatto, legge di swing e T2.
4. **Protezioni riusabili:** box certificate, `compensationSafe`, StopPlan con prova sulla secante RAW, SAFE_OFF, `exit_cause`, modello del servo, QEMU.
5. **Stabilità dinamica con due appoggi:**
   - ZMP o capture point rispetto al segmento diagonale;
   - recupero a passo (stepping) invece di un arresto immediato;
   - sensibilità a ritardo e cedimento.
6. **Qualifica dello STOP:**
   - in stance diagonale, arrestare solo dopo il prossimo touchdown (transizione a quattro appoggi), non nel mezzo;
   - lo STOP immediato in swing va provato con capture point dentro il poligono della fase successiva;
   - niente fasi di volo nella prima versione.
7. **IMU senza duplicare l'autorità Motion:** IMU e feedback sono ingressi del generatore, che sposta il piede o il corpo; lo stesso `compensationSafe` decide. Nessun secondo controllore.

**Percorso proposto:**
1. Trot lento a ~20–25 mm/s con quattro appoggi brevi tra le coppie (walking trot) e STOP al touchdown.
2. Qualifica sulla fixture della frenata ad ACC 100–150.
3. Trot a duty 0,5 verso 40–50 mm/s.

## 11. Identità della candidata unica

| Campo | Valore |
|---|---|
| Versione | `1.0.2-dev.6` |
| Commit | `70abcff54cd3e6e1573385efae16d78a5460f9ee` |
| Build ID | `70abcff54cd3`, CLEAN `ROBOT_POWERED` |
| APP | 1 738 032 B, SHA256 `67d55e1a9d104fc75c9b1d805e2a290022d8bbcf0d3967d6bc232925b46ca834` |
| Script APP-only (preparato, **mai eseguito**) | `~/MATDOG/verification-artifacts/MATDOG_1_0_2_DEV6CLOSE_FLASH_PREP_20261109/`, variabile `MATDOG_DEV6CLOSE_FLASH_AUTHORIZED` |

**Build storiche, conservate e mai flashate:** `4e62b81` (`d8ba3f14…`), `5159c7f` (`bce9a022…`), `3803960` (`05935919…`). Rollback dev.4 e backup completo verificati via SHA256.

## 12. PR #67

OPEN e DRAFT, aggiornata a questo report.
