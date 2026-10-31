# MATDOG — Recovery Motion e semplificazione (17/10/2026)

Base: `8e1673b` (installato, ROBOT_POWERED) → `7d954de` (solo docs/qualifica). Robot spento:
nessun accesso hardware, flash, comando servo, NVS/EEPROM/BMS. Evidenze **solo offline**.

## 1. Root cause STAND (codice)

Percorso: `POST /api/motion/command` → `MotionService::command` → `MotionCoordinator::accept`
→ `preArmCheck` → `Controller::armOperationalSession` → `MotionSession::request` (clamp 0x104)
→ **ACCEPTED** (= arming avviato, non movimento) → `Controller` loop → `MotionSession::update`
(PRIME 12 goal, ENERGIZE 12 torque, ARM) → executor.

- `PRIME_BACKEND_OR_POLICY` nasceva in `MotionSession.cpp:157`: **un'unica stringa** per
  quattro esiti diversi (rifiuto policy al plan, rifiuto al commit, backend `VERIFIED_NOT_APPLIED`,
  backend `UNCERTAIN`/nessuna risposta), senza giunto né fase. Revoca **al primo** errore.
- La policy MOTION per questo stato è dimostrata accettante: stessi controlli di `request()`
  (limiti, transform, geometria, lease adottata, modo RUN), nessun bootstrap calibrazione attivo;
  la nuova regressione riproduce clamp `260` e passa policy+commit. Quindi il rifiuto reale è
  nel **backend ServoBus** (scrittura goal/torque con readback): unico tratto mai eseguito su
  hardware nel percorso operativo (FIRST_MICRO_MOTION: "GoalPosition/PRESENT coherence" BLOCKED),
  diverso dal percorso calibrazione già funzionante (timeout 100 ms, solo goal word). Quale dei
  due esiti backend e su quale giunto **non è ricostruibile**: il firmware lo scartava e il logger
  USB era fermo (09:47) prima della prova.
- Clamp mask `260 = 0x104` = giunti 2 e 8 (LOWER LF e LOWER LH) appoggiati oltre il limite
  ripiegato, primati 2..12 tick verso l'interno: comportamento previsto, non la causa.
- Nessun conflitto Motion Authority/legacy: lease MOTION adottata, epoch/generation coerenti
  (un mismatch avrebbe dato `SESSION_CHANGED`/`MODE_POWER_FAULT`, non questo motivo).
- `INPUT_LOST_OR_EXPIRED` **non** era una perdita di presenza: dopo la revoca, il permit cade
  (`SAFE_OFF_REQUIRED`), `MotionService::refresh` lo legge come gate e `disconnected()`
  chiamava `executor.stop("INPUT_LOST_OR_EXPIRED")` anche con executor OFF, sovrascrivendo il
  motivo. Il comando veniva chiuso ABORTED con `OPERATOR_GO_REQUIRED`. Poi `confirmSafeOff`
  scriveva `SAFE_OFF_VERIFIED`, cancellando l'ultima traccia nella sessione.
- HTTP ACCEPTED era coerente (arming avviato) ma l'esito finale era etichettato male.

## 2. Correzioni

- `MotionSession`: motivi distinti `PRIME|ENERGIZE _POLICY_REJECTED / _COMMIT_REJECTED /
  _BACKEND_NOT_APPLIED / _BACKEND_NO_RESPONSE`, giunto e decisione/risultato; fino a 3
  tentativi per lo stesso giunto/target solo per esiti backend (goal con torque OFF e
  torque-enable sono idempotenti e verificati da readback, ri-pianificati via policy; un rifiuto
  di policy non si ritenta mai); `lastRevoke()` sopravvive a SAFE_OFF_VERIFIED.
- `MotionCoordinator/MotionService`: un arming fallito chiude il comando ABORTED con **la sua**
  causa; `disconnected()` ferma solo un executor attivo (niente rietichettatura).
- Telemetria: `local_session.{last_revoke,failure,failed_joint}`; riga USB
  `OPERATIONAL_SESSION=REVOKED reason=… joint=… detail=…` (buffer statico, non allocante).
- Status Motion: `heartbeat:true|false` (questo poll ha rinnovato la presenza?).

## 3. Dashboard

STAND / WALK (STANDARD 20/30/40 mm, FAST 60 mm sperimentale) / WALK STOP / STOP nella prima
scheda, causa effettiva separata (RETE · AUTENTICAZIONE · STALE · HTTP · CONTROLLER · FAULT),
diagnostica in `<details>`. Nessuna checkbox presenza. Poll = heartbeat di comunicazione:
continua in background, ripresa immediata su visibilitychange/focus/pageshow/online, watchdog.
Pagina nascosta durante WALK → WALK STOP controllato. Sessione: rinnovo automatico
(dispositivo ricordato) quando `heartbeat:false`; un POST `AUTH_FAILED` (mai eseguito) viene
reinviato una sola volta, stesso id/sequence, dopo il rinnovo. Deadman 3 s invariato.

## 4. Twin

Nessun campo IP: la Dashboard sonda con un'immagine `/api/bridge/ping.gif` gli host
configurati (CSP `img-src`/`frame-src` = stessa allowlist 192.168.4.2/192.168.1.149:8765) e
incornicia il primo raggiungibile; stato esplicito "HOST NON TROVATO". Bridge:
`--source auto` (default) usa lo **status HTTP read-only** del robot (192.168.4.1) quando
raggiungibile — USB mai aperta — altrimenti seriale. Sorgente HTTP: nessuna scrittura, nessun
heartbeat (poll anonimo), STOP via bridge rifiutato (`HTTP_SOURCE_READONLY`); telemetria
passata tal quale (nessun LIVE simulato). Le mesh restano sull'host.

## 5. Batteria

DALY 0xD2 espone la capacità residua (registro 48, 0,1 Ah, già decodificato): mostrata come
"valore BMS" (non validato su MATDOG). In più **STIMA = SOC × 9,0 Ah** (nominale configurata,
Zeee 3S 9000 mAh) etichettata STIMA. SOC, tensione pacco, cella minima, corrente. Solo display:
nessuna soglia/protezione/scrittura BMS cambia.

## 6. WALK / FAST / stack

- `kStandOnlyCandidate` rimosso (ADR-006): WALK è legato a runtime se il box certificato ± tube
  sta nei limiti live; FAST offerto solo se il **suo** box ci sta. Capability:
  `hardware_qualified:{stand,walk,fast}=false`.
- Stack (ELF, `entry a1,N`, catene dirette MATDOG note dal loop): **17 648 → 12 704 B** su
  16 384 (stencil/sample/frame STL in storage statico; solver non rientrante, solo loop).
  WALK operativo reale non usa la catena STL (`stlContactReference=false`): ≈ 9,7 KiB. Uscite
  STL bit-identiche alla baseline. Margine runtime (high-water) da misurare al primo WALK.
- FAST: margine certificato 6,158 mm vs 6 mm richiesti **a box ± 6 mrad**. Target sempre nel
  box; giunto misurato fuori box ± tube → stop. Condizione residua non dimostrata: overshoot
  servo oltre il goal (1200/100) < ≈ 45 mrad (≈ 6 mm sul LOWER) durante latenza di rilevamento
  + SAFE_OFF. Serve risposta a gradino misurata in fixture. FAST resta opt-in, etichettato.

## 7. Test (offline)

Vedi `CANDIDATE.md` per esiti e identità binario.
