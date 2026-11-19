# MATDOG Motion — candidato integrato P1 + FP32 (`05e1214`), 18/11/2026

Mandato «Motion FP32 e consolidamento delle ottimizzazioni». Lavoro solo offline:
- **Non toccati:** robot, porte seriali, dashboard, flash, worktree operativo (`30c894a`, pulito), worktree e backup di Fable.
- **Push:** nessuno.

## 0. Esito

| Voce | Valore |
|---|---|
| Candidato | `05e1214b4010f65794d8e038366d5a3e36251bb8` = `30c894a` + P1 minima (`eb1ba28`) + nucleo FP32 (`05e1214`) |
| Branch / worktree | `cand/motion-fp32-20261118`, `~/MATDOG/worktrees/matdog-motion-fp32-20261118` (locale, non pubblicato) |
| Build CLEAN | APP `75ac737d7ee65bcb9bc1c2cb43722b742b64fb8a4af78492a54eb2b2057257c5`, 1 777 952 B (33,9 % di app0); `BUILD_ID=05e1214b4010`, `SOURCE_STATE=CLEAN`, `GIT_DIRTY=0`, ROBOT_POWERED, OTA ingest 0 |
| `static_audit` | **PASS** sul commit pulito; righe dei sotto-test con fallimento atteso identiche a `eb1ba28`; albero pulito dopo |
| Flash | **non eseguito**. Pacchetto APP-only pronto, gate offline PASS, rifiuto senza autorizzazione verificato |
| Rollback | `30c894a` col suo script già usato il 18/11 (`812a2e83…`); gate offline PASS; backup `app_images/30c894a` verificato |
| `random_3` | Patch separata e minima `1314bc4` (branch `cand/ksteer-slow-recert-20261118`), **non** inclusa nella build |
| Blocker offline | Nessuno. Uno trovato e corretto prima della build: panic al boot (§2.4) |

## 1. Matrice delle modifiche

| Modifica | Stato | Nella build |
|---|---|---|
| ACQUIRE v2, T1/T2, PlanSplit V2, fix STOP→STAND, writer HTTP, validDefinition, IRAM G2s, F3 | Già integrate (`30c894a`, HW 18/11) | Sì, invariate |
| P1 minima: ricalcolo dell'alzata 10 → 50 µm, solo `Gait.cpp` (`eb1ba28`) | Nuova, validata offline | Sì |
| Nucleo FP32 (`05e1214`) | Nuova, validata offline | Sì |
| `kSteer` ricertificato per SLOW (`1314bc4`) | Nuova, validata offline | **No**: patch separata, decisione dell'operatore |
| P1 completa (`TiltedContactIk` a 50 µm) | Scartata | No |
| W1 (seme HIP previsto) | Scartata (18/11) | No |
| Varianti della regola del 2 % e della tolleranza di Newton | Scartate (inversioni di 1 tick, accoppiamento con la sonda) | No |
| W2, B, tuning HTTP | Non incorporate (sospese o aperte) | No |

## 2. Conversione FP32

### 2.1 Confine

**In float** (`detail::` in `LegKinematicsInternal.h`):
- FK e IK analitica;
- modello di contatto del piede;
- trasformazioni della posa del corpo;
- Newton su terreno inclinato e il suo Jacobiano;
- soluzione per gamba del gait e differenziale (condizionamento);
- fit del piano d'appoggio per il livellamento;
- trigonometria di `bodyPoseFromRollPitch`.

**In double, per necessità dimostrata:**
- **API pubblica e frame dei giunti (`JointTargetFrame`).** Conversione RAW, box di collisione, `compensationSafe` e prova `StopPlan` leggono gli stessi double di prima.
  - Le rotte certificate di PRONE/STAND restano bit per bit.
- **Clock del gait (`cycles`) e traslazioni nel mondo.** Crescono con la camminata: in float, dopo 1 000 cicli la fase avrebbe una risoluzione di 6·10⁻⁵ cicli.
  - Le differenze nel mondo si fanno in double e solo il vettore nel corpo, limitato, passa in float.
- **Timestamp, IMU (quaternione), comandi, NVS e calibrazione:** invariati.

### 2.2 Criteri numerici ri-derivati

Tutti in `LegKinematicsInternal.h`, ciascuno con la sua derivazione. Sono tolleranze di convergenza e di arrotondamento: nessun margine meccanico cambia.

| Criterio | Double | FP32 | Derivazione e significato fisico |
|---|---|---|---|
| Residuo IK/contatto (default `IkOptions`/`TiltedIkOptions`) | 1e-9 m | **1e-6 m** | Residuo float ≤ ~3e-7 m (misurato 5,1e-8 su 78 200 casi del 07/11); 1/50 dei 50 µm di mezzo tick al piede |
| Guardie di dominio (raggio, coseno del gomito) | 1e-12 | 1e-6 | Arrotondamento float del coseno ~3e-7; il bersaglio resta verificato dal residuo |
| Bordo dei limiti articolari | 1e-12 rad | 1e-6 rad | Errore di `acos`/`atan2` float ≤ ~5e-7 rad (6,5e-4 tick); limiti float arrotondati verso l'interno |
| Pivot singolare (Jacobiani 3×3) | 1e-12 | 1e-6 m/rad | Pivot operativi 0,03–0,2; arrotondamento di un pivot nullo ~1e-8 |
| Degenerazione asse/normale | 1e-9 | 1e-5 | Asse entro 1e-5 rad dalla normale: impossibile nei limiti articolari |
| Ortonormalità della rotazione | 1e-10 | 1e-5 | Errore peggiore di una rotazione arrotondata in float ~1,5e-6; un punto a 0,3 m si sposta ≤ 3 µm |
| Test: recupero del livellamento (`test_stop_support`) | 1e-7 rad | 2e-5 rad | 1 µm di residuo su un triangolo d'appoggio ≥ 0,1 m; misurato 1,6e-6 rad, 1/50 della banda morta di 1 mrad |

**Invariati:**
- soglia di P1 (5e-5 m) e soglia del raccordo nella compensazione (2e-5 m);
- 8 iterazioni di Newton;
- regola del 2 %;
- limiti URDF e operativi, box, `pointSafe`, `compensationSafe`, prova STOP, conversione RAW.

### 2.3 Codice macchina Xtensa (ELF del firmware, `xtensa/`)

**Nucleo** (FK, IK, contatto, Newton, Jacobiano, `invert3`, `solve3`):
- 0 chiamate a routine double software;
- 0 chiamate alla libm double;
- istruzioni FPU single (`mul.s`, `madd.s`, …) e libm float.

**Libreria di sistema:**
- `sqrtf` usa la sequenza hardware `sqrt0.s`/`maddn.s`;
- la divisione float chiama `__divsf3` in ROM.

**Funzioni Motion considerate**, `30c894a` → `05e1214`:

| Misura | `30c894a` | `05e1214` |
|---|---|---|
| Istruzioni | 11 778 | 8 714 (−26 %) |
| Chiamate double software | 797 | 320 (solo al confine API e nella legge del gait) |
| Chiamate libm double | 63 | 1 |

**Memoria (ELF):**
- `.iram0.text` 92 743 → 89 323 B (−3 420);
- `.dram0.bss` −288 B;
- `.flash.text` +588 B, `.flash.rodata` +1 328 B;
- **`_heap_start` 3 872 B più basso: ~3,9 KB di heap interno in più.** Con G2s il margine era ~1 KB.

**Stack (QEMU):** minimo libero del task Motion da 7 476 a 8 228–8 420 B.

### 2.4 Difetto trovato e corretto: panic al boot

- **Sintomo:** il primo boot in QEMU dell'immagine FP32 andava in `Coprocessor exception` dentro `do_global_ctors`.
- **Causa:** i limiti float del modello erano calcolati con `nextafter` in un costruttore globale. Sull'ESP32-S3 la FPU non è abilitata in quella fase; sul robot sarebbe stato un ciclo di riavvii.
- **Correzione:** tabella `constexpr`, senza codice all'avvio.
  - I limiti risultano identici a quelli calcolati con `nextafter`.
  - Uno `static_assert` prova che tutti i 24 limiti sono dentro quelli URDF.
- **Verifica sull'ELF finale:** nessun costruttore globale usa la FPU (`fp_audit_p1fp32_ctors.txt`).

## 3. Differenze cinematiche, qualità e sicurezza

**Banco host:** 7 scenari (fixture S, sterzate B3/D8/YS/YF, H, E) × 3 fasi del rumore IMU, 38 153 frame per variante.

| Confronto | Frame identici | Diversi | Ampiezza massima |
|---|---|---|---|
| FP32 contro double (`30c894a`) | 38 093 | 51 di compensazione + 9 senza nominale (HEIGHT) | **1 tick**, a un bordo di mezzo tick |
| P1+FP32 contro P1 | 38 087 | 65 + 8 | 1 tick |

**Andamento e continuità** (FP32 contro double):
- nominale invariato (< 0,0005 tick);
- correzione IMU applicata diversa al massimo di 0,061 mrad;
- nessuno sfasamento e durate delle fasi identiche;
- passo e seconda differenza massimi identici per fase.

**IMU:**
- errore di livellamento (medio e massimo, ogni fase) identico entro 0,003 mrad;
- conteggi di ENVELOPE, WORKSPACE, SLEW e ATTENUATED identici. Le differenze fra baseline e P1+FP32 sono tutte di P1 e già classificate nel rapporto P1.

**Contatto** (inviluppo del joystick con il raccordo reale): supporto minimo, distanza dal suolo delle parti non-piede e inclinazione compensata del piede identici a P1 in double.
- SLOW: 0,34 mm / 2,24 mm / 9,47°.
- STANDARD: 0,30 mm / 2,27 mm / 9,41°.
- FAST: 0,95 mm / 2,32 mm / 7,55°.

**STOP e regressioni:**
- `run_stop_secant_gate.sh` PASS, con secanti peggiori e margini identici riga per riga (es. STANDARD preset 2: 30,00 contro 31,36, +0,36).
- Matrice di perturbazioni: 0 fault e 0 riprese fallite (2 righe con RAW massimo −1).
- `run_simple_motion_tests`, `run_prone_tests`, `run_stop_support_tests`, `run_preenable_bus_tests`, `run_operational_host_tests`: PASS.
- 7 suite storiche non compilano già sulla baseline (sorgenti rimossi da tempo): preesistenti, invariate.

**Inviluppo del joystick:**
- STANDARD e FAST: **PASS**;
- SLOW: FAIL **solo** per `random_3`, preesistente e identico a P1 in double (−1,4 mrad, 4 release). Vedi §5.

## 4. Beneficio misurato (QEMU ESP32-S3, CCOUNT deterministico)

Il CCOUNT di QEMU **non** è un tempo hardware e qui non viene convertito in millisecondi.

Variazioni fra corse diverse di ±1 k: interrupt del tick FreeRTOS che cadono in release diverse.

**Base e P1:**
- B3/D8/H/E sono le corse della revisione delle tolleranze, con lo stesso banco;
- S (fixture STAND/HEIGHT/PRONE) è stato aggiunto qui per la classe 1.

| Scenario | Variante | Release più pesante | NOMINAL WALK medio / max | Release > 280 k | > 320 k |
|---|---|---:|---:|---:|---:|
| B3 sterzato STANDARD | base / P1 / FP32 / P1+FP32 | 403 600 / 403 614 / 248 106 / 248 085 | 106 066 / 101 097 / 22 514 / 22 140 – max 285 533 → 40 197 | 163 / 152 / 0 / 0 | 25 / 5 / 0 / 0 |
| D8 sterzato andata/ritorno | idem | 439 865 / 442 115 / 248 099 / 248 085 | 105 196 / 99 408 / 22 010 / 21 598 – max 287 714 → 40 190 | 230 / 198 / 0 / 0 | 44 / 17 / 0 / 0 |
| H WALK, STOP, ripresa | idem | 403 657 / 403 670 / 248 787 / 248 098 | 97 238 / 97 230 / 32 063 / 32 084 | 140 / 140 / 0 / 0 | 6 / 6 / 0 / 0 |
| E WALK con cambi di modo | idem | 403 625 / 404 303 / 248 113 / 248 085 | 84 446 / 84 442 / 17 962 / 17 970 | 116 / 116 / 0 / 0 | 9 / 9 / 0 / 0 |
| S fixture inclinata | idem | 528 863 / 528 863 / 268 840 / 269 555 | – | 224 / 224 / 0 / 0 | 190 / 190 / 0 / 0 |

**Classi P0** (release più pesante di ciascuna classe):

| Classe | Base | P1 | FP32 | P1+FP32 |
|---|---:|---:|---:|---:|
| 1. Doppia compensazione, senza piano di segmento | 354 231 | 354 181 | 91 790 | **91 067 (−74 %)** |
| 1b. Doppia compensazione con piano di segmento | 528 877 | 528 877 | 269 583 | 268 853 (−49 %) |
| 2. NOMINAL + COMPENSATE | 439 865 | 442 115 | 108 492 | **110 857 (−75 %)** |
| 3. WALK sterzato con più di 4 IK | 439 865 | 442 115 | 110 281 | **110 857 (−75 %)** |

**Attribuzione:**
- **P1** riduce la frequenza dei picchi sterzati in double (> 320 k: B3 25 → 5, D8 44 → 17) e non il massimo.
- **FP32** porta a zero le release sopra 280 k in ogni scenario e riduce i massimi di classe del 74–75 %.
- **Combinazione:** sopra FP32, P1 vale ancora −1,5…−1,9 % sul NOMINAL medio sterzato. Ogni IK costa ora ~1/8, quindi il beneficio assoluto di P1 è piccolo; resta il minor numero di IK con qualità invariata.

**Cosa resta pesante** (lavoro double invariato per mandato):
- Release che costruiscono un piano di segmento: STAND_DEPLOY/LIFT, PRONE, NOMINAL 170–205 k di `ViaPlan` e prova STOP; release totale ~248–270 k.
- Nel WALK, `compensationSafe` (~60 k: decodifica RAW e prova `StopPlan`), su release totali di ~110 k.

## 5. `random_3` (preesistente, risolto con una patch separata)

**Diagnosi** (evidenze conservate in `MATDOG_1_0_2_P0_TOLERANCE_20261118`):
- solo SLOW sterzato;
- RF UPPER supera il massimo di `kSteer` (0,96346 rad) di 1 tick al vertice di beccheggio +55 mrad di `kSteerTilt`;
- la bisezione mostra che la legge SLOW è evoluta dopo la certificazione (`9a6d105` +41,5 mrad … `b4ce6d5` −3,0) senza ricertificare `kSteer`.

**Verifica con il certificatore CAD/collisioni esistente**: `certify_steer_box.py`, FCL, ≥ 3 mm fra le 120 coppie non adiacenti, +3 mm ventrali, tubo di 40 mrad.
- **Raccolta:** l'inviluppo SLOW del candidato con la procedura dell'11/11 (STANDARD/FAST): 97 scenari su una copia scratch.
- **Regime sterzato** (79 scenari): sta dentro `kSteer` in ogni giunto, **tranne** il massimo di RF UPPER, 1,42 mrad oltre.

| Box candidato | Esito | Limite inferiore certificato |
|---|---|---|
| `kSteer` attuale (dev.15) | PASS | 3,035 mm |
| Inviluppo completo ± tubo (anche i tratti frontali, box più stretto in altri giunti) | PASS | 3,001 mm |
| Unione `kSteer` ∪ inviluppo sterzato ± tubo (4 limiti allargati) | PASS | 3,008 mm |
| **Minimo: solo RF UPPER max → 1,004874 rad** (inviluppo + 40 mrad), gli altri 23 limiti invariati | **PASS** 120/120 + 12/12 | **3,085 mm** |

**Patch `1314bc4`** (`static_audit` PASS sul commit pulito, righe identiche a `05e1214`):
- contenuto: `SteerCollisionData.h` (un limite più `kSteerProof`) ed evidenze in `09_Logs/Validation_Reports/Motion_SteerSlow_Recert_2026-11-18`;
- invariati: `kSteerTilt`, `compensationSafe`, prova STOP e gli altri box;
- inviluppo del joystick: SLOW **PASS** (`random_3` da −1,4 a +33,9 mrad), STANDARD e FAST PASS;
- prova STOP a secante e perturbazioni: PASS.

**SLOW sterzato:** non va qualificato come operativo senza questa patch (oppure va delimitato al solo SLOW frontale).

## 6. Margine computazionale: che cosa permetterebbe in futuro

Non ho cambiato velocità, accelerazioni, ampiezze o traiettorie. Su base QEMU, da confermare sull'hardware:

- **WALK STANDARD/FAST.**
  - Il massimo di classe scende a ~111 k e la metà è la prova STOP.
  - Il limite di velocità oggi è fisico, non di calcolo: margine del secante STOP +0,36 tick in STANDARD preset 2, supporto statico in FAST, FAST 40 già escluso.
  - Il margine di calcolo permetterebbe di togliere le scorciatoie nate per il budget: la parentesi T2 con interpolazione, il riuso T1 della correzione. Così ogni release riceverebbe la compensazione esatta (livellamento più fedele e meno inversioni di 1 tick). Va dimostrato con lo stesso banco prima di proporlo.
- **STAND e transizioni di altezza.**
  - La sonda d'attenuazione e le prove di scala costano ~1/4.
  - Diventa possibile compensare ogni release dei segmenti senza l'eccezione P3b sulla release di piano.
  - Restano pesanti le release che costruiscono il piano (~250–270 k): candidato futuro è distribuire il piano su più release.
- **Memoria.** I ~3,9 KB di heap interno recuperati riaprono spazio per IRAM mirata (le funzioni escluse da G2s) senza tornare al MEMORY_STOP del 16/11.

## 7. Piano breve di qualifica hardware

**Prerequisiti:**
- A/B DALY di Fable concluso e valutato;
- robot su fixture con l'operatore presente;
- autorizzazione al flash nella sessione attiva.

**Passi:**
1. **Identità, sola lettura.** Leggere l'immagine installata **prima** di tutto: può non essere `30c894a`.
2. **Flash APP-only** con `preflash/flash-qualified-app0-1.0.2-dev.7-05e1214.sh` (`MATDOG_FP32_05E1214_FLASH_AUTHORIZED=YES_ROBOT_SUPPORTED`), con la procedura ROM del 18/11. Poi:
   - `verify-flash`;
   - boot pulito: nessun panic né `Coprocessor exception` nel log;
   - identità `05e1214` e manifest;
   - NVS gen-1 e calibrazione identiche;
   - `MOTION_ACC_CEILING verified=220`;
   - heap libero non inferiore a `30c894a`.
3. **Catena fixture del 18/11** (ABBA 140 passi, `cand_vs_ref.py` con riferimento 1811), sotto carico HTTP operativo (status del dashboard). Criteri pre-registrati, non modificabili:
   - release ≤ 16 ms, 0 release operative perse, 0 overrun;
   - distribuzione per classe P0 contro `30c894a`.
4. **STOP USB** 14/14 e **STOP HTTP**; quest'ultimo resta NOT TESTED senza accesso al dashboard.
5. **Modi WALK su fixture:** SLOW solo frontale (sterzato solo con `1314bc4` approvata e ricompilata), STANDARD e FAST. Conteggi ENVELOPE e WORKSPACE confrontati con `30c894a`.
6. **Ciclo a freddo.** Il gate DALY resta separato (Fable).
7. **Rollback** a `30c894a` se un criterio fallisce, senza rilassarlo: script `812a2e83…`, backup verificato.

## 8. Identità e file

**Commit e build:**

| Voce | Valore |
|---|---|
| Commit | `eb1ba28` (P1), `05e1214` (FP32); `1314bc4` (`kSteer`, branch separato). Nessun push |
| Build root | `~/MATDOG/build-output/motion-fp32-20261118/05e1214/artifacts/ROBOT_POWERED-ota0/` (ELF, map, manifest, sdkconfig) |
| Backup immagine | `~/MATDOG/backups/esp32/app_images/05e1214` (SHA256SUMS verificato) |

**Contenuto di questa cartella:**

| Percorso | Contenuto |
|---|---|
| `checks/` | `static_audit` di `05e1214` e `1314bc4` |
| `build/` | Log della build |
| `xtensa/` | Audit FPU |
| `logs/` | Classificazioni e tabelle |
| `regress/` | Suite |
| `random3/` | Raccolte e certificati |
| `qemu/`, `out/`, `rig/`, `trees/` | Banchi e risultati |
| `preflash/` | Pacchetto APP-only e controlli |
