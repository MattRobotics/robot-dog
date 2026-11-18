# P1 minimo, classificazione degli scostamenti, `random_3` (18/11/2026, offline)

Mandato «MATDOG — Decisione su patch P1».
- **Come ho lavorato:** solo copie e un worktree nuovo.
- **Non toccati:** worktree operativo (`30c894a`, pulito), robot, porte seriali, flash, artefatti di `30c894a` e la sessione A/B di Fable.
- **Non fatto:** nessun push, nessuna build.

## 1. Diff funzionale minimo: solo `Gait.cpp`

**`P1_MIN.diff`** (`3850f777…`), 2 file:
- in `solveGaitInto` la soglia passa da `1e-5` a `5e-5`, con il commento che ne dà la derivazione;
- pin di `Gait.cpp` in `motion_final_release_sources.json`.

**Invariati:** `TiltedContactIk.cpp/.h` (20 µm) e `FootContact.h`.

**Equivalenza:** i frame sono identici bit per bit a P1 completo e alla variante solo-`Gait.cpp` in 21 esecuzioni su 21 (7 scenari × 3 fasi del rumore IMU).
- La soglia della compensazione resta 20 µm, quindi l'errore d'alzata dei frame compensati misurato dall'inviluppo resta quello della baseline (20,5–20,9 µm).
- L'aumento a 23–44 µm visto con P1 completo veniva dalla modifica a `TiltedContactIk`, ora esclusa.

## 2. Gli scostamenti fino a 8 tick

**Scomposizione esatta.** Per ogni articolazione: `ΔRAW = ΔNOM + ΔCOMP`, cioè bersaglio nominale prima della compensazione IMU più contributo della compensazione, con l'arrotondamento incluso. Frame allineati per ingresso di fase; 3 fasi del rumore.

| Componente | Misura | Esito |
|---|---|---|
| **Cinematica diretta (nominale)** | ≤ **0,485 tick**, solo nel WALK sterzato; STAND, HEIGHT, PRONE e WALK non sterzato identici | Entro mezzo tick, come derivato |
| **Compensazione IMU** | Tutti i frame con ≥ 3 tick (267: 232 WALK, 23 HOLD, 12 WALK_SETTLE), massimo 8 tick. In ognuno la correzione applicata è diversa (fino a **3,9 mrad**) | Diverso cammino della correzione, vedi sotto |
| **Sfasamento temporale** | Nessuno sul nominale (0 frame spiegati da uno spostamento di 1–2 release). Durate delle fasi identiche, salvo il rientro HEIGHT dopo il WALK sterzato: −2 release (40 ms) in 2 casi su 21, perché parte da una correzione tenuta diversa di 1,3 mrad | Effetto della compensazione |

**Perché la correzione diverge:**
- la stima del piano d'appoggio usa le posizioni misurate. Con P1 queste differiscono sistematicamente, sotto il tick, in ogni soluzione sterzata;
- la banda morta di 1 mrad, lo slew e il limite dell'inviluppo rendono la correzione tenuta dipendente dalla storia.

**Sensibilità della baseline:**
- una perturbazione IMU di 1e-7 rad cambia 12 frame su 38 154, di 1 tick;
- un encoder letto 1 tick fuori per 120 ms cambia 9 frame, al massimo di 2 tick, con correzione diversa per al più 0,8 mrad, ed è transitorio.

**Assenza di peggioramento** (somma sulle 3 fasi del rumore, baseline → P1):

| Grandezza | Baseline | P1 |
|---|---:|---:|
| Errore livellamento ∣voluto−applicato∣ nel WALK, media / massimo | 1,289 / 18,52 mrad | 1,290 / 18,52 mrad |
| Stesso errore in HOLD, media | 15,698 mrad | 15,695 mrad |
| Stesso errore in HEIGHT, media | 9,268 mrad | 9,250 mrad |
| Saturazioni dell'inviluppo (ENVELOPE) nel WALK | 21 582 | 21 576 |
| Rifiuti WORKSPACE in HOLD | 3 524 | 3 521 |
| Passo massimo e seconda differenza massima del comando, per fase | – | identici |
| Seconda differenza del nominale nel WALK | 8,627 tick | 8,640 tick |
| Inversioni di 1 tick nel WALK | 3 489 | 3 492 |

**Contatto** (inviluppo del joystick con il raccordo reale):
- errore d'alzata nominale ≤ 50 µm per costruzione, sotto mezzo tick: un tick è ≥ 100 µm al piede;
- penetrazione del modello +0,01…+0,03 mm (massimo 0,214 → 0,232 SLOW; 0,222 → 0,235 STANDARD): il piede è leggermente più premuto, **non c'è perdita di contatto**;
- distanza dal suolo delle parti diverse dal piede: SLOW 2,27 → 2,24 mm, STANDARD 2,31 → 2,27 mm; requisito ≥ 2 mm;
- supporto identico negli scenari deterministici.

**Suite su P1 minimo:**
- `run_simple_motion_tests.sh`: PASS.
- `run_stop_secant_gate.sh`: PASS, con secanti peggiori e margini identici.
- Matrice di perturbazioni: identica salvo una riga casuale (0 fault, 0 riprese fallite).
- Inviluppo del joystick STANDARD: **PASS**, margine d'inclinazione minimo 14,3 mrad come la baseline.
- `static_audit` sul commit: vedi §4.

**Esito: nessuna regressione funzionale.**

## 3. `random_3` (preesistente nella baseline)

**Dove e cosa** (copia diagnostica dello strumento, `random3/`):
- **Modo:** solo **SLOW** (modo 0). La stessa sequenza passa in STANDARD (+14,3 mrad) e FAST (+15,8 mrad); l'insieme STANDARD completo passa.
- **Stato:** SLOW sterzato, passo 40 mm, oscillazione laterale 9 mm, nessuna rotazione; piede **anteriore destro (RF)** in appoggio; 6 release consecutive (circa 120 ms).
- **Correzione:** raggio 8 del poligono certificato `kSteerTilt`, cioè **beccheggio +55 mrad**, il massimo in quella direzione.
- **Articolazione:** **RF UPPER** quantizzata a 0,9649–0,9664 rad contro il massimo del box `kSteer` (0,96346): **1,4–3,0 mrad oltre (1–2 tick)**. Limite URDF gen-1 a 2,10 rad, lontano.
- **Con P1 minimo:** stessa articolazione e stesso raggio, 4 release, −1,42 mrad. P1 non lo crea e non lo corregge.

**Protezione coinvolta:**
- `kSteer` è un **certificato di collisione** (`certify_steer_box.py`): l'unione degli inviluppi osservati alla certificazione, allargata di 40 mrad, con ≥ 3 mm fra tutte le 120 coppie di parti non adiacenti (+3 mm ventrale) per qualunque combinazione interna al box;
- sul robot lo fa rispettare `compensationSafe`: la correzione verrebbe rifiutata (correzione tenuta, `WORKSPACE`) oppure, se anche quella tenuta uscisse dal box per il moto nominale, il WALK si fermerebbe con `WALK_IK_OR_WORKSPACE`.

**Rilevanza fisica:**
- **collisione: no.** 3 mrad sulla coscia spostano i punti del segmento di ≤ 0,6 mm, contro una distanza certificata ≥ 3 mm;
- **disponibilità: possibile, solo in SLOW sterzato,** con una correzione di beccheggio vicina a +55 mrad (≈ 3°, discesa o fixture inclinata) in quella fase del passo;
- SLOW non è il modo operativo.

**Origine** (bisezione dei 66 commit di legge dopo la certificazione di `kSteer`):

| Commit | Margine di `random_3` |
|---|---:|
| `9a6d105` (certificazione) | +41,5 mrad |
| `d36ce04` | +6,3 |
| `5472b65` | +1,7 |
| `23577ad` | +0,1 |
| `b4ce6d5` (seme E1) e successivi | **−3,0** |

- E1 non cambia la verifica: sposta di pochissimo gli stati visitati, e il margine era già a zero.
- **Causa: la legge sterzata SLOW è evoluta dopo la certificazione, e né `kSteer` né `kSteerTilt` sono stati ricertificati.**
- Limite non rilassato.

**Possibili correzioni** (decisione dell'operatore, nessuna applicata):
- ricertificare `kSteer` per la legge SLOW attuale con lo strumento esistente, che produce una nuova prova di collisione;
- oppure restringere il bordo di beccheggio di `kSteerTilt` per lo SLOW.

## 4. Commit locale isolato

| Voce | Valore |
|---|---|
| Commit | `eb1ba28d9ad32a794ec6afc1e2adff80c01c02fe`, padre `30c894a`, 2 file +5/−3 |
| Branch e worktree | branch locale `cand/p1-edge-lift-halftick-20261118`, worktree `~/MATDOG/worktrees/matdog-p1-edge-lift-20261118` |
| Worktree operativo | invariato (`30c894a`, pulito) |
| Remoto | invariato (`06b6db0`), nessun push |
| Build e flash | nessuna |
| `static_audit` sul commit pulito | **PASS** (exit 0); gate di convergenza delle sorgenti Motion PASS con il pin aggiornato; righe di sotto-test con fallimento atteso identiche all'audit CLEAN di `30c894a`; albero pulito dopo l'audit |

## 5. Beneficio misurato di P1

| Misura | Effetto |
|---|---|
| Ricalcoli dell'alzata nel WALK sterzato | −45 % (B3), −52 % (D8), −75 % (YS), −63 % (YF) |
| QEMU (CCOUNT deterministico, non WCET HW): release sterzate sopra 320 k, misurate su P1 minimo | B3 25 → 5, D8 44 → 17 (stessi digest dei frame di P1 completo) |
| QEMU: NOMINAL medio | −5 % |
| Massimo | **invariato**: le release del trasferimento laterale (8 IK) spostano ≥ ½ tick |
| WALK non sterzato | identico |

**Non chiude Motion Timing P0.**
