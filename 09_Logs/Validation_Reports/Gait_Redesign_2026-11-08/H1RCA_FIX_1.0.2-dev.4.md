# Fix H1-RCA — unwind della correzione IMU nel modello di contatto fillet — candidata `1.0.2-dev.4` — 2026-11-08

Autorizzazione: patch minima, regressioni pertinenti e build offline. **Nessun flash, reset o comando hardware.** Installata sul robot resta `1.0.2-dev.3`. Classi: VH = verificato su hardware, VO = verificato offline.

## 1. Diff (sorgente `004685e`, `SimpleMotion::compensateLevel`)

```diff
-  const auto q=compensationTarget(f,body,goal_.valid?goal_:f.target,true,walk||phase_==Phase::WALK_HOLD?gait_.edgeTiltRad:0);
+  const double edgeTilt=walk||phase_==Phase::WALK_HOLD?gait_.edgeTiltRad:unwind?kEdgeContactMaxTiltRad:0;
+  const auto q=compensationTarget(f,body,goal_.valid?goal_:f.target,true,edgeTilt);
```

Più 7 righe di commento. Modello, limite (`kEdgeContactMaxTiltRad`, 12°) e funzione IK (`worldEdgeContactInverseKinematicsTilted`) sono quelli già usati dal passo joystick. Cambia solo il caso `unwind=true` fuori da WALK e WALK_SETTLE, cioè lo srotolamento pendente di `HEIGHT` e `PRONE_HEIGHT`. HOLD, WALK STANDARD, WALK joystick, WALK_SETTLE e i riferimenti dopo lo srotolamento restano invariati.

## 2. Prima divergenza risolta

| | Evidenza |
|---|---|
| Robot (VH, due volte) | Uno WALK_SETTLE joystick sulla fixture accetta +63,7/−66,5 mrad nel modello fillet. `HEIGHT` (ripartenza) srotola nel modello della fascia nominale (2°): ogni bersaglio è invalido, `compensationSafe` rifiuta al **primo controllo, `pointSafe(q)`**. La correzione resta ferma per 16 s, il goal non cambia (`goal_sent_us` precedente a HEIGHT) e scatta `NATIVE_TARGET_NOT_REACHED`. Tracking ≤ 7 tick, nessuna anomalia di tempi o bus |
| Causa | Un riferimento accettato in un dominio (EDGE_BIASED) viene ritirato passo per passo in un dominio più stretto (NOMINAL_STRIP). Fuori fascia nessun passo intermedio è rappresentabile: è un **deadlock geometrico**, non un limite fisico, perché la correzione zero è valida e raggiungibile |
| Correzione | I passi dello srotolamento si risolvono nel modello in cui la correzione era stata accettata. Dentro la fascia il sollevamento fillet vale zero e i giunti sono identici a dev.3 (bit a bit, §4). Fuori fascia il contatto sul raccordo è continuo fino al bordo della fascia |

## 3. Catena di sicurezza completa a ogni passo (VO)

Banco H1, strumentazione scratch non presente nel repository. Il plant usa:
- il riferimento comandato meno il cedimento dello snapshot hardware `exit_j`, con i carichi reali;
- la velocità dei servo in segno-modulo;
- l'IMU trattenuta dalla fixture (anello aperto).

Lo WALK_SETTLE del banco termina a **+65,4/−66,5 mrad** (robot +63,7/−66,5; con velocità in segno-modulo pura il banco arriva esattamente a +63,7/−66,5), sul confine dove la StopPlan rifiuta la crescita, come sul robot. File per passo: [`h1rca/unwind_steps_dev4_bench.csv`](h1rca/unwind_steps_dev4_bench.csv).

| Srotolamento | Partenza | Accettati / rifiutati | Passi su raccordo, poi fascia | Margine min box (residuo di carico + ritardo pesato sull'età) | Ritardo max | StopPlan max | Passo RAW max |
|---|---|---|---|---|---|---|---|
| avvio joystick | +31,3/−31,3 | 41/0 | 0 (identico a dev.3) | 8,3 tick | 12 tick | 0,12 s | 2 tick |
| **ripartenza dopo inversione** | +64,6/−65,7 | **84/0** | 40 (asse fino a 4,05°) | 8,3 tick | 12 tick | 0,12 s | 2 tick |
| **STAND dopo STOP** | +62,4/−64,6 | **82/0** | 37 | 3,3 tick | 15 tick | 0,24 s | 4 tick |
| PRONE | +0,8/−0,8 | 2/0 | 0 | 8,3 tick | 12 tick | 0,12 s | 2 tick |

Ogni passo accettato supera tutti i controlli di `compensationSafe`:
- `pointSafe` e box `kHeight`;
- conversione e decodifica RAW (convenzione del banco);
- limiti servo con residuo di carico e margine di ritardo;
- fallback `kLift`;
- prova STOP con `rateFromHistory` e `StopPlan`.

Il primo passo dista 2–3 tick dall'ultimo RAW di WALK_SETTLE; i passi successivi sono ≤ 2–4 tick per rilascio, sotto il drive stand di 9 tick per rilascio. Tutti i frame inviati sono dentro i box certificati. Nessun controllo successivo all'IK rifiuta: non c'è una nuova divergenza.

## 4. Regressione e confronto con dev.3 (VO)

| Scenario banco H1 | dev.3 | dev.4 |
|---|---|---|
| avanti → laterale → inversione → ripartenza → indietro → rilascio → STOP → STAND → PRONE | FAULT `NATIVE_TARGET_NOT_REACHED` (766 rifiuti, codice `pointSafe`) | completa fino a PRONE_REST, 0 rifiuti |
| inversione → rilascio → PRONE | bloccata `PRONE_NOT_COMPLETED_SUPPORT_HELD` | PRONE_REST |
| inversione → rilascio → STOP → STAND 120 | FAULT | STANDING |
| inclinazione minore (1,0/−0,8°) | FAULT | completa |
| IMU in piano, con e senza cedimento (correzioni dentro la fascia) | completa | completa, **3517 e 3397 frame identici bit a bit a dev.3** |
| tutte le sequenze H1, fino al punto di blocco di dev.3 | — | frame identici bit a bit |

Test di repository `steeredSettleUnwind` (`test_simple_motion.cpp`, stesso plant, due percorsi). Verifica:
- nessun fault;
- ogni rilascio di srotolamento riduce la correzione (mai `WORKSPACE`);
- passo RAW entro il drive stand;
- ogni frame dentro un box certificato;
- conclusione degli stati: indietro, STANDING, STOP verificato, STANDING, PRONE_REST.

Contro il sorgente dev.3 lo stesso test fallisce al primo passo di HEIGHT (`WORKSPACE` a +65,4/−66,5 mrad).

| Regressione (sorgente `4358f13`) | Esito | Confronto dev.3 |
|---|---|---|
| SimpleMotion (incluso il nuovo `steeredSettleUnwind`) | PASS | il nuovo test fallisce su dev.3 |
| STOP support; PRONE 4/4 | PASS | invariato |
| Dashboard e joystick end-to-end (fixture 79, DOM 21 buste, contratto 5824 controlli) | PASS | invariato |
| Gate inviluppo joystick, 32 scenari | PASS (0,57 mm / 2,18 mm / 11,44°) | log **identico** a dev.3 |
| Inviluppi IMU WALK75/WALK100 (187 330 verifiche ciascuno) | 0 fallimenti | invariato |
| Digest dei frame, 9 scenari (incluso lo stress con jitter e STOP) | completi, 0 frame fuori box | **digest identici** a dev.3 |
| Static audit (366 file) | PASS | invariato |

## 5. Limiti residui

- **Solo offline.** Il plant non riproduce compliance, attrito né rumore della velocità servo; la conversione RAW è quella lineare del banco, non la calibrazione gen-1 del robot (sul robot `resolve`/`decodeRaw` restano nella catena e la applicano).
- La crescita della correzione in WALK_SETTLE sulla fixture (anello aperto) **non** è modificata: la fixture la porta a ~3,7°. A terra resta piccola; su una pendenza oltre ~2° è legittima. dev.4 la ritira senza deadlock.
- Durante la HOLD successiva a WALK_SETTLE con correzione oltre la fascia, la HOLD mantiene il riferimento di WALK_SETTLE (`WORKSPACE`, comportamento dev.3 invariato) finché un comando di postura non la srotola.
- Margine minimo osservato 3,3 tick (LH_HIP, il giunto con il cedimento maggiore): positivo, nessuna protezione aggirata.

## 6. Candidata

| | |
|---|---|
| Sorgente | `4358f13` (fix `004685e` + identità), `FW_VERSION=1.0.2-dev.4`, manifest `SOURCE_STATE=CLEAN`, `GIT_DIRTY=0`, `MOTION_AUTHORIZED=0`, ROBOT_POWERED, OTA ingest 0 |
| APP | `~/MATDOG/build-output/gait-redesign-20261108/build-4358f13/artifacts/ROBOT_POWERED-ota0/MATDOG_Controller.ino.bin`, **1.723.024 B** (+48 B), SHA256 **`3e7ee77d934bb681efcf4b7ff534447aab9d08db599df2ab45601efb19ed9c66`**, RAM 137.988 B |
| Compatibilità | layout `MATDOG_16M_2x5M_NVS_V1`, tabella `8f756ecb…` invariata; scrittura solo `app0` 0x010000–0x1b4a90 |
| Flash (preparato, non eseguito) | `~/MATDOG/verification-artifacts/MATDOG_1_0_2_DEV4_FLASH_PREP_20261108/flash-qualified-app0-1.0.2-dev.4.sh` (stesso schema qualificato; senza `MATDOG_DEV4_FLASH_AUTHORIZED=YES_ROBOT_SUPPORTED`: REFUSE, verificato) |
| Rollback | dev.3 installata `cef30d8c…`, Stable dev.17 `14e055a0…`, dev.16 `73952eec…`, backup integrale `5cbba0b9…`: `rollback-assets.sha256` 5×OK |
| Prova hardware suggerita | stessa H1 sulla fixture (STAND 120, joystick avanti → laterale → indietro tenendo la leva): attesa ripartenza all'indietro senza FAULT, `imu_correction` da ~64 mrad a 0 in ~1,7 s in HEIGHT; acquisizione con gli stessi strumenti read-only H1 |
