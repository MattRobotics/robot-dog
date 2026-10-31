# MATDOG — Handoff per la sessione Astra (Motion) dopo il consolidamento REV01-FIX

Data: 2026-10-31. Documento unico di passaggio di consegne. Offline: nessun flash, nessuna calibrazione, nessuna scrittura NVS/EEPROM, nessuna modifica al firmware operativo, nessun merge.

Classi di prova: **VH** = verificato su hardware, **VO** = verificato offline, **H** = ipotesi, **U** = ignoto.

## 1. Identità del repository (verificata sul remoto)

| Voce | Valore |
|---|---|
| Repository | `https://github.com/MattRobotics/robot-dog` |
| PR | [#57](https://github.com/MattRobotics/robot-dog/pull/57), **draft**, non unita |
| Branch | `geometry/rev01-fix-integration-20261030` |
| Base (`origin/main`, intatto) | `9b2c42ff6a2867d4fa986edcd3a0a02d29634385` |
| Commit precedenti del branch | `aa89dd4` geometria REV01-FIX · `0f8f7ff` base di calibrazione storica · `820d80b` proprietà fisiche · `7d04939` chiusura |
| **Commit di contenuto di questo intervento (su remoto, verificato con `git ls-remote`)** | `423de3a` workbook approvato (byte esatti) + rimozione archivio URDF Studio · `bfdf7b83380ec2d75021bb759a2338b92a1d9312` allineamento tabella del workbook |
| Questo documento | Il commit che lo aggiunge è la punta del branch; il suo SHA è nel messaggio di chiusura della sessione (un file non può contenere il proprio hash). Verifica: `git ls-remote origin refs/heads/geometry/rev01-fix-integration-20261030` |
| Branch remoti esistenti | `main`, `geometry/rev01-fix-integration-20261030`, `matdog/rc5-source-checkpoint-20261026`. **Non esiste alcun branch `dev5` sul remoto.** |

## 2. Modello canonico (URDF e manifest)

Percorso: `03_CAD/URDF/matt_robodog_rev00/` (il nome `rev00` è conservato di proposito; la revisione si riconosce dagli hash).

- URDF operativo: `matt_robodog_rev00.urdf`, SHA-256 `ce63034e727e82ee82267fb97cea01cd6e171e850cb2d42cb42cf763ee3706f7`.
- Manifest `SHA256SUMS.txt`: SHA-256 `ff0b858f38dae8cd531e658a8ce69bb1ec8c6e040d38e77cc7b7f4454795ce49` (19 righe: URDF, 17 STL, workbook). Fissato dal test `test_matdog_geometry_v5_immutability.py`.
- Unica geometria: 17 STL in `meshes/collision/`; **ogni `<visual>` e ogni `<collision>` punta alla stessa STL** (34 riferimenti, 17 file unici, origin 0/0/0, scala 0.001) — verificato in questo intervento. Non esistono `meshes/*.stl` estetiche (rimosse in `aa89dd4`).
- Rimosso in questo intervento: `urdf_studio/matt_robodog_rev00.usp` (27 MB, LFS). Conteneva le vecchie STL estetiche dettagliate, il `base_link` REV00 da 5,2 MB e un URDF non aggiornato: una seconda sorgente geometrica, non referenziata da nessun codice o manifest. Recuperabile dalla storia Git (commit `1d53e20`).
- Variazioni rispetto a REV00 (cinematica invariata): `base_link` REV01-FIX, quattro `hip_joint` +2,2 mm in Z (LF/RF 0,0487 m, RH/LH 0,0287 m), 17 blocchi `<inertial>`, percorsi visual. Assi, limiti, motorId, direzioni: invariati.

Manifest:

| File | SHA-256 |
|---|---|
| `matt_robodog_rev00.urdf` | `ce63034e727e82ee82267fb97cea01cd6e171e850cb2d42cb42cf763ee3706f7` |
| `meshes/collision/base_link.stl` | `c2eadf6d6db1b03379cf256b542dadbcd8d63076179bc84ebd7169634e9f0097` |
| `meshes/collision/lf_foot_link.stl` | `e4d35cd4107bd7fad2d3fe511926ec4fc5211f15f978581e469787266ebe5357` |
| `meshes/collision/lf_hip_link.stl` | `b2e430aa99998791879f1ab80c46787a310c80f77a3df9116d1084e0a0e27d97` |
| `meshes/collision/lf_lower_leg_link.stl` | `bbb40400571fab554395d5671ad86b1b106ec3fe24fce733ef0be179a739508a` |
| `meshes/collision/lf_upper_leg_link.stl` | `0830fc10d8873b6a45f2f58c2ca61f08a7f38092984a96b911ac6beed274c30c` |
| `meshes/collision/lh_foot_link.stl` | `e4d35cd4107bd7fad2d3fe511926ec4fc5211f15f978581e469787266ebe5357` |
| `meshes/collision/lh_hip_link.stl` | `5350010e3623ec36301d2c0592e3209e1b06da83e823215caf55cca5aafe86da` |
| `meshes/collision/lh_lower_leg_link.stl` | `bbb40400571fab554395d5671ad86b1b106ec3fe24fce733ef0be179a739508a` |
| `meshes/collision/lh_upper_leg_link.stl` | `0830fc10d8873b6a45f2f58c2ca61f08a7f38092984a96b911ac6beed274c30c` |
| `meshes/collision/rf_foot_link.stl` | `18dfa7908e7410cc2920d5d37b0c0cce13cb2c683f765e7d073b3d5777c00d33` |
| `meshes/collision/rf_hip_link.stl` | `83a7622f990228e6c2b6bb0431e682fc462a76feb3ce2fe190b7553be879dc33` |
| `meshes/collision/rf_lower_leg_link.stl` | `6b40148edad22670c420d1bcca2325c8befd72e27f4390569a292250df3e96c8` |
| `meshes/collision/rf_upper_leg_link.stl` | `fbf43f047a943188a6c7b9a7a3a45763c22ddb35704a517b7b47b03dcb68cb91` |
| `meshes/collision/rh_foot_link.stl` | `18dfa7908e7410cc2920d5d37b0c0cce13cb2c683f765e7d073b3d5777c00d33` |
| `meshes/collision/rh_hip_link.stl` | `04b205e9be3683b5238a035970b56ee319e4e7fca55ba7860a19fdb549c4efea` |
| `meshes/collision/rh_lower_leg_link.stl` | `6b40148edad22670c420d1bcca2325c8befd72e27f4390569a292250df3e96c8` |
| `meshes/collision/rh_upper_leg_link.stl` | `fbf43f047a943188a6c7b9a7a3a45763c22ddb35704a517b7b47b03dcb68cb91` |
| `reference/Coordinate_Cinematiche_matt_robodog_rev00.xlsx` | `f376b42fef824732acae8e6569dbaed38c1c37f40ef4228ed869fb1332db1827` |

## 3. Provenienza delle 17 STL e pipeline di decimazione

- 16 STL di gamba: baseline di collisione approvata il 2026-08-10, invariate (nessun hash cambiato rispetto a REV00). Per upper, lower e piede, LH ha la stessa STL di LF e RH la stessa di RF (hash uguali nel manifest); le quattro STL di HIP sono tutte diverse.
- `base_link.stl`: export CAD REV01-FIX, 36.832 triangoli, SHA-256 `c2eadf6d6db1b03379cf256b542dadbcd8d63076179bc84ebd7169634e9f0097` (sostituisce il REV00 da 22.044 triangoli, `0a485e7a…`, conservato solo nella base storica). Otto perni motore corretti a gioco nominale 0,25 mm; distanza minima di superficie misurata 0,203 mm. Totale del set: 134.484 triangoli (6.725.628 byte).
- **Sorgente unica = URDF canonico + 17 STL.** Il rendering del Digital Twin non ha una geometria propria:
  1. `viewer/.../scripts/sync_canonical_assets.mjs` (sul branch PR) copia in `public/canonical/` (gitignored, rigenerato ogni volta) l'URDF così com'è e solo le STL referenziate, e scrive `CANONICAL_PROVENANCE.json`. Eseguito in questo intervento: URDF staged `ce63034e…` = canonico, 17 mesh.
  2. `scripts/build_mobile_twin_assets.py` (esiste sul lineage `dev5`, non su `main`) decima per clustering di vertici su griglia (`npm run build:twin` usa `--cells 3`) e produce `matdog_collision.mtw` (formato MTW1). È solo geometria di visualizzazione, mai modello di collisione.
  3. `package_mobile_twin.py` + `scripts/generate_twin_assets.py` generano `src/network/TwinAssets.h` (HTML, JS, MTW e URDF in gzip, con ETag SHA-256).
  - Prova in questo intervento (strumento ufficiale di dev5 estratto in sola lettura con `git show df43feb:…`, eseguito sulle 17 STL canoniche, output fuori dal repo): lossless 134.484 triangoli · 1 mm 52.320 · 2 mm 27.777 · **3 mm 16.973 triangoli, 88.351 B gzip (griglia usata da `npm run build:twin`; il `TwinAssets.h` di dev5 contiene la versione REV00 da 84.108 B gzip)** · 4 mm 11.607 · 6 mm 7.120. Dati in `data/MOBILE_TWIN_BUDGET_canonical_REV01FIX.json`.
  - **Stato nel firmware**: `TwinAssets.h` di dev5 (e dell'installato) incorpora ancora l'URDF REV00 (`3890a3f0…`) e la mesh decimata dal `base_link` REV00. Va rigenerato col sistema esistente (non a mano) quando dev5 integra REV01-FIX; la sonda locale lo ha già fatto una volta (§8).

## 4. Massa, CoM, inerzie (URDF = workbook, verificati 17/17 senza differenze)

Configurazione nominale unica **con Jetson Orin** (175 g dentro `base_link`), report Solid Edge 2026-10-30. Il `base_link` comprende cinque ST3215 (4 HIP + neck yaw); il neck yaw non è ancora un giunto URDF e non si aggiunge massa.

- Somma dei 17 link **2,928 kg**; assieme Solid Edge 2,937 kg; **residuo 9 g non allocato** (= 1 g di risoluzione dei report, documentato, non risolvibile con le evidenze). CoM aggregato modello `[0,688; −0,275; 17,286] mm` vs assieme `[0,78; −0,59; 17,02] mm` (scarto max 0,42 mm).
- Pesata reale 2026-10-30: **2,807 kg senza Jetson** — evidenza storica della configurazione assemblata quel giorno, non una seconda configurazione; il CAD senza Jetson (2,762 kg) è 45 g sotto la pesata (ipotesi cablaggio, H). Distinta calcolata con Jetson: 2,9432 kg.
- Le anche posteriori ereditano le proprietà di LF (differenza stimata sulle mesh: 0,21 mm CoM, 0,8 % inerzia). RF = riflessione in Y di LF; LH = frame di LF.
- **Nessuna validazione dinamica su hardware (U).** Nessun dato inerziale per il robot reale senza Jetson.

Tensori al CoM nel frame del link (kg·m², convenzione URDF; CoM in mm):

| Link | Massa [kg] | CoM x / y / z [mm] | Ixx | Ixy | Ixz | Iyy | Iyz | Izz |
|---|---|---|---|---|---|---|---|---|
| `base_link` | 1.828 | -6.93 / -0.44 / 36.57 | 1.8855e-03 | -1.426e-05 | -2.560e-04 | 1.3608e-02 | -1.160e-05 | 1.4054e-02 |
| `lf_hip_link` | 0.050 | 3.66 / 29.82 / 6.94 | 4.0225e-05 | -1.315e-06 | -1.140e-06 | 1.8958e-05 | -4.464e-06 | 4.5428e-05 |
| `lf_upper_leg_link` | 0.152 | 1.22 / 0.14 / -45.03 | 1.7935e-04 | -6.266e-08 | 0.000e+00 | 1.7365e-04 | 0.000e+00 | 2.8446e-05 |
| `lf_lower_leg_link` | 0.059 | 30.75 / -1.18 / -19.58 | 2.1336e-05 | 6.042e-07 | 1.669e-05 | 1.0167e-04 | -3.688e-07 | 1.0344e-04 |
| `lf_foot_link` | 0.014 | -0.74 / 0.00 / 15.72 | 8.9210e-07 | 0.000e+00 | -5.562e-09 | 8.4305e-07 | 0.000e+00 | 8.9320e-07 |
| `rf_hip_link` | 0.050 | 3.66 / -29.82 / 6.94 | 4.0225e-05 | 1.315e-06 | -1.140e-06 | 1.8958e-05 | 4.464e-06 | 4.5428e-05 |
| `rf_upper_leg_link` | 0.152 | 1.22 / -0.14 / -45.03 | 1.7935e-04 | 6.266e-08 | 0.000e+00 | 1.7365e-04 | 0.000e+00 | 2.8446e-05 |
| `rf_lower_leg_link` | 0.059 | 30.75 / 1.18 / -19.58 | 2.1336e-05 | -6.042e-07 | 1.669e-05 | 1.0167e-04 | 3.688e-07 | 1.0344e-04 |
| `rf_foot_link` | 0.014 | -0.74 / 0.00 / 15.72 | 8.9210e-07 | 0.000e+00 | -5.562e-09 | 8.4305e-07 | 0.000e+00 | 8.9320e-07 |
| `rh_hip_link` | 0.050 | 3.66 / -29.82 / 6.94 | 4.0225e-05 | 1.315e-06 | -1.140e-06 | 1.8958e-05 | 4.464e-06 | 4.5428e-05 |
| `rh_upper_leg_link` | 0.152 | 1.22 / -0.14 / -45.03 | 1.7935e-04 | 6.266e-08 | 0.000e+00 | 1.7365e-04 | 0.000e+00 | 2.8446e-05 |
| `rh_lower_leg_link` | 0.059 | 30.75 / 1.18 / -19.58 | 2.1336e-05 | -6.042e-07 | 1.669e-05 | 1.0167e-04 | 3.688e-07 | 1.0344e-04 |
| `rh_foot_link` | 0.014 | -0.74 / 0.00 / 15.72 | 8.9210e-07 | 0.000e+00 | -5.562e-09 | 8.4305e-07 | 0.000e+00 | 8.9320e-07 |
| `lh_hip_link` | 0.050 | 3.66 / 29.82 / 6.94 | 4.0225e-05 | -1.315e-06 | -1.140e-06 | 1.8958e-05 | -4.464e-06 | 4.5428e-05 |
| `lh_upper_leg_link` | 0.152 | 1.22 / 0.14 / -45.03 | 1.7935e-04 | -6.266e-08 | 0.000e+00 | 1.7365e-04 | 0.000e+00 | 2.8446e-05 |
| `lh_lower_leg_link` | 0.059 | 30.75 / -1.18 / -19.58 | 2.1336e-05 | 6.042e-07 | 1.669e-05 | 1.0167e-04 | -3.688e-07 | 1.0344e-04 |
| `lh_foot_link` | 0.014 | -0.74 / 0.00 / 15.72 | 8.9210e-07 | 0.000e+00 | -5.562e-09 | 8.4305e-07 | 0.000e+00 | 8.9320e-07 |

## 5. Workbook approvato

- Input approvato dall'operatore (ZIP `MATDOG_REV01_FIX_ALLINEAMENTO_2026-10-31.zip`, checksum del pacchetto verificato OK): SHA-256 **`68e64ce08ce7fc035d8652d9e2bc22c73ea6a8ffab56cd17898543087ba9a546`**. Versione presente nel commit `423de3a` byte per byte.
- **Workbook canonico attuale** (commit `bfdf7b8`): SHA-256 **`f376b42fef824732acae8e6569dbaed38c1c37f40ef4228ed869fb1332db1827`**. Differisce dall'approvato in **una sola parte**, `xl/tables/table1.xml`: le intestazioni di `Joint_Parameters` riga 7 (colonne U, W, X) erano state rinominate nel foglio ("Tetto a vuoto 3S [ RPM ]", "Budget coppia 3S [kg·cm]/[N·m]") ma non nella definizione della tabella (`Speed [rpm]`, `Torque`, `Effort`); Excel richiede che coincidano e avrebbe proposto la riparazione. Fogli, formule, valori, stili e grafico sono byte-identici all'approvato. Per tornare ai byte approvati: `git revert bfdf7b8` (poi riallineare manifest e pin del test).
- 7 fogli: `Joint_Parameters`, `Reference_Notes`, `Inertial_Properties`, `CAD_Source_Data`, `Mass_Reconciliation`, `Weight_Evidence_2026-10-30`, `ST3215_3S_Reference` (con grafico RPM stimati vs coppia, serie `F4:F34` su categorie `E4:E34`). Fogli eliminati/accorpati: `Mesh_Transforms`, `Link_Engineering`, `Mass_Summary`, `Engineering_Weight_Source`, `URDF_Audit`, `Physical_Property_Method`, `Physical_Evidence_Log` (riferimenti documentali aggiornati; versioni precedenti in Git: `7d04939`, SHA-256 `dbfe1e25…`, e `0f8f7ff`).
- Verifiche (VO, strutturali, `tools/check_workbook_openxml.py`): ZIP integro; 20 parti ben formate; content-type e relazioni coerenti; nomi/ID foglio unici; righe e celle ordinate; nessun merge sovrapposto; nessun riferimento a fogli inesistenti né errore; tabella `JointParametersTable` coerente con le intestazioni; grafico con riferimenti validi e popolati; 342 formule, valori memorizzati uguali al ricalcolo con valutatore proprio (0 differenze). Coerenza con l'URDF: 16 giunti (origini, assi, limiti) e 17 link (massa, CoM, tensore) senza differenze.
- **Non eseguita: apertura in Excel o in un'applicazione compatibile (non disponibile qui).** Resta aperta; il grafico ha cache dei punti vuota (`ptCount 0`) e Excel lo ricostruisce all'apertura. Il file non ha `calcPr` (nessun `fullCalcOnLoad`): non serve, perché i valori memorizzati coincidono col ricalcolo.

## 6. 29 RPM attuali vs riferimento progettuale 40 RPM

**Decisione progettuale del workbook (da qualificare, non ancora adottata):** 40 RPM (4,18879 rad/s) come tetto di progetto *quasi a vuoto* a 11,1 V, derivato linearmente dai 45 RPM a vuoto dichiarati da Feetech a 12 V (41,6 RPM per sola proporzione di tensione, arrotondato a 40). Curva coppia–velocità: pendenza 45/30 = 1,5 RPM/(kg·cm) dai dati di targa a 12 V (30 kg·cm di stallo), estesa a 11,1 V per ipotesi; a 9,2 kg·cm ≈ **27,8 RPM** (non 40). È una stima preliminare, **non misurata sul robot (H)**. 29 RPM non è né confermato né scartato come vincolo definitivo.

**Dove compaiono oggi i 29 RPM** (`velocity="3.03687289847"` rad/s, `effort="0.902244"` N·m — 0,9022118 N·m è la conversione esatta di 9,2 kg·cm):

| # | Dipendenza | Natura | Azione per Astra |
|---|---|---|---|
| 1 | `matt_robodog_rev00.urdf` righe 401–570: 12 `<limit>` dei giunti rotoidali | limite operativo del modello | **non modificato qui**; cambiarlo solo con decisione esplicita |
| 2 | `historical_calibration_basis/rev00/matt_robodog_rev00.urdf` (stessi 12 limiti) | base storica, immutabile | non toccare |
| 3 | `matdog_calibration_basis.py --check` | il delta ammesso è: 4 hip Z, percorsi visual, `base_link`, 17 `<inertial>`. **Qualsiasi modifica ai `<limit>` fa fallire il controllo (e la `static_audit`)** | cambiare il limite richiede di estendere esplicitamente il delta dichiarato, con revisione (i limiti di velocità non entrano nella catena di calibrazione, ma il controllo li protegge) |
| 4 | `03_CAD/URDF/matt_robodog_rev00/README.md` | semantica dei limiti (aggiornata con il riferimento 40 RPM) | allineare dopo la decisione |
| 5 | Workbook: `Joint_Parameters` U8:U23 = `ST3215_3S_Reference!B7` (40), `ST3215_3S_Reference!B9` = 29 "limite URDF vigente", `Reference_Notes` C29–C31, C63 | disallineamento dichiarato fra candidato (40) e URDF (29) | aggiornare B9 solo dopo la qualificazione |
| 6 | dev5 `TwinAssets.h` e Twin sul telefono | incorpora l'URDF REV00 (limiti inclusi): solo visualizzazione | si rigenera col sistema esistente |
| 7 | Bundle Geometry V5, `CalibrationGeometryProfileData.h`, piano di sequenza, evidenze `Motion_*`/`Preflight_*`/`Runtime_Commissioning_*`, certificati di ingresso | legati per SHA all'URDF REV00 (che contiene i 29 RPM) | provenienza storica; non riscrivere (vedi §7) |
| 8 | `06_Software/Matdog_Core/motion/dynamics/leg_torque_estimate.py` (solo dev5) | legge `effort` dall'URDF; il log `Issue52_Realtime_2026-10-15/leg_torque_estimate.log` usò massa **2,480 kg** (inerzie REV00) | rieseguire con le inerzie nuove (2,928 kg) |

**Cosa non ho trovato (importante):** nessun file sorgente di firmware o software nei commit `df43feb` (dev5), `6c5cffc` (motion-operational), `ac10251` (rc11-integration), `9cfd86b` (sonda) contiene la stringa "29 rpm" o `3.0368…` al di fuori dell'URDF e del README. Il Motion di dev5 non legge l'attributo `velocity` dell'URDF: usa limiti propri in `src/simple/SimpleMotion.h` — `kStandSpeedRadS=0,55`, `kStandAccelerationRadS2=1,10`, registro servo `kStandDrive{480,13}` (≈ 7 RPM), `kWalkDrive{1000,50}` (≈ 14,6 RPM), `kCalibrationDrive{160,8}` — con commento "0,55 rad/s è il 12 % della velocità a vuoto C018 (4,72 rad/s a 12 V)" cioè 45 RPM. Se i "contratti Motion" a 29 RPM citati dall'operatore sono altrove (documenti, schede, altri branch), non sono nel codice che ho analizzato: **Astra deve confermare quale contratto intende** prima di cambiare qualcosa. La mappa sopra è completa solo per i branch citati.

## 7. Certificazioni Motion ancora riferite a REV00

Sul branch PR (cioè su `main`) non c'è Motion. Su dev5 (`df43feb`) i dati seguenti sono legati all'URDF REV00 (`3890a3f0…`) e/o al `base_link` REV00; **tutti restano validi per il robot e il firmware installati, ma non sono stati ricertificati su REV01-FIX**:

- `src/simple/CollisionData.h`: scatole `kEntry`, `kWalk`, `kHeight`, `kDeploy`, `kLift` e relativi `k*Proof` (hash di prove FCL continue). Generato da `scripts/recover_simple_collision_data.py` da `TOLERANCE_V1_20261018`, `SIMPLE_OPT_20261022`, `SIMPLE_TRAJ_V2_20261022` (in `verification-artifacts/`). Conseguenza misurata sulla sonda (`9cfd86b`): con REV01-FIX `SimpleMotion::stand(0.15)` viene **rifiutato** (l'IK a 150 mm esce dal box `kHeight` calcolato su REV00): fail-closed (VO).
- `src/motion/PairedProneData.h`, first-stride, tabelle PRONE e `PRONE_EVIDENCE` (`Motion_Recovery_2026-10-12`); certificati di ingresso/regione di camminata (`Preflight_Review_2026-10-15`, `Runtime_Commissioning_2026-10-14`, `Issue52_Realtime_2026-10-15`), `Motion_Night_Run_2026-10-12`; test `paired_prone/test_frozen_geometry.py`.
- Strumenti FCL di ricertificazione già esistenti: `06_Software/Matdog_Core/motion/paired_prone/certify_*.py` (python-fcl), `certify_operational_walk_region.py`, `certify_measured_entry_region.py`, `certify_operational_tube.py`, `certify_suspended_first_stride.py`, `walk_after_stand.py`.
- Box di recupero all'avvio (±10 tick) e relativo certificato CAD (396 PASS, 708 box, clearance ≥ 3,1 mm): emessi su REV00, il generatore non è nel repo. Controllo a campione (non il certificato): 114 pose, clearance minima identica su REV00 e REV01-FIX.
- **Catena di calibrazione: non toccare.** Resta legata a REV00 per SHA (profilo geometrico, piano di sequenza, evidenze q0, riferimento di avvio) e al tag storico **`3713f4ddc43b204e`**, che il record NVS gen-1 porta con sé. Rigenerarla porterebbe il tag a `c42b4c22…` e il gen-1 a `RECORD_INCOMPATIBLE` (rilettura fail-closed del recupero all'avvio). Opzione 1 approvata il 2026-10-31: la base storica è in `historical_calibration_basis/rev00/` e `matdog_calibration_basis.py --check` verifica il delta. Dettagli: `09_Logs/Validation_Reports/REV01_FIX_Integration_2026-10-30/calibration_chain/CALIBRATION_BASIS_DECISION.md`.

## 8. Stato effettivo di dev5 e firmware installato

| Voce | Valore | Classe |
|---|---|---|
| Installato sul robot | `1.0.1-dev.5`, APP `c8b27d672eca145ad8291639fa6762d2279854fe57e93631c29d7506f2cd5bfb` (1.634.240 B), BUILD_UTC 2026-10-30T14:56:30Z, flashato il 30/10 con readback identico, NVS gen-1 `6d03b9dd…` byte-identico prima/dopo | VH (letto il 30/10; **non riletto oggi**, nessun accesso al robot) |
| Sorgente compilato | commit `df43feb94cdba8cd4c15efd6afa638d37a00db4c`, branch **locale** `dev5/prone-stand-20261030` (worktree `worktrees/matdog-dev5-prone-stand-20261030`), pulito (0 file modificati verificato oggi), **non pubblicato, non unito** | VO |
| Checkpoint | `verification-artifacts/MATDOG_DEV5_CHECKPOINT_20261030/` (APP, manifest, ELF, bundle git `fdd0434..df43feb`, tar sorgente, NVS) | — |
| Rollback qualificato | rc.11 r2 `fc0a7141…`; immagine dev.4 `4bfcde09…` | VH |
| Cosa NON contiene l'installato | nulla di PR #57: URDF REV00 (`kUrdfSha256` `3890a3f0…` in `LegGeometryData.h`), `base_link` REV00, quote HIP 46,5/26,5 mm, inerzie REV00 (2,480 kg), limiti 29 RPM nell'URDF incorporato nel Twin | VO |
| Sonda locale | `verify/rev01fix-on-dev5-20261030` @ `9cfd86b` ("PROBE, not for flash", non pubblicata): REV01-FIX applicata su dev5. Build esterna OK (1.582.608 B, 30,2 % dello slot; **non** flashata), `TwinAssets.h` rigenerato col generatore ufficiale (HTML/JS bit-identici, cambiano solo mesh e URDF; test twin 42+50 PASS; viewer 85/85). `stand(0.15)` rifiutato come in §7 | VO |
| Evidenze hardware dev5 (30/10) | PRONE→STAND senza STOP: PASS da USB e da dashboard, 0 missed. Sequenza non pianificata STAND150/WALK 45 mm/WALK STOP/STAND100/PRONE-da-100/STOP: **41 missed releases**, poi nessuna analisi di attribuzione completa (ipotesi HTTPD/Wi-Fi/client sul core 0, mailbox del Controller esclusa, H). Compensate 19,5 ms vs 9,5 ms a pari lavoro. Robot lasciato STOPPED, prono, coppia OFF | VH |
| Prossima prova HW già preparata | `verification-artifacts/MATDOG_DEV5_WALK_PREP_20261030/PROCEDURE_WALK_TIMING.md` (WALK solo da USB, telefono fuori dall'AP, DIAG 2 Hz). Non eseguita | — |

## 9. Attività necessarie per STAND, return-to-PRONE e WALK STANDARD

Non esiste ancora un ramo che unisca REV01-FIX e dev5. L'ordine proposto (decisioni di Astra/operatore):

**A. Preparazione (offline)**
1. Decidere la base: attendere il merge di PR #57 su `main`, oppure partire dal commit di questo branch. Integrare in dev5 tramite una nuova branch (non modificare `dev5/prone-stand-20261030`): la sonda `9cfd86b` mostra i file toccati. File con pin dell'URDF REV00 da aggiornare con gli strumenti ufficiali: `LegGeometryData.h` (generatore corretto `matdog_motion_geometry_export.py`, recuperato dal blob storico verificato `5406f73` come strumento esterno, non presente in `main`/`dev5`; `matdog_calibration_geometry_export.py` genera invece `CalibrationGeometryProfileData.h`), `scripts/tests/motion_integration_manifest_pr2.json`, `test_twin_assets.py`, `docs/MOTION_ASSET_BUDGET.json`, `kinematics/*.yaml`, `matdog_urdf_fk.py`, `paired_prone/test_frozen_geometry.py`, `TwinAssets.h` (rigenerato da `npm run build:twin` + `generate_twin_assets.py`). **Non** toccare i file della catena di calibrazione né il tag `3713f4ddc43b204e`.
2. Decidere i limiti di velocità: 29 RPM, 40 RPM o altro, con la curva come stima da qualificare (§6). Ricordare che 40 RPM non vale insieme a 9,2 kg·cm (≈ 27,8 RPM) e che il Motion di dev5 usa già limiti propri molto più bassi. Se si modifica l'URDF, estendere esplicitamente il delta del controllo di base (§6, riga 3).
3. Rieseguire la stima di coppia (`leg_torque_estimate.py`) con le nuove inerzie.

**B. Ricertificazione geometrica (offline, strumenti FCL ufficiali)**
4. Rigenerare le prove continue sulle 17 STL canoniche (`certify_*.py`; evidenze `TOLERANCE_V1`, `SIMPLE_OPT`, `SIMPLE_TRAJ_V2`) e poi `recover_simple_collision_data.py` → `CollisionData.h`; rigenerare PairedProne/first-stride se usati.
5. Verificare con `static_audit.py` e con la suite Motion (nel branch PR la suite Motion è `DEFERRED`: non c'è). Nessuna scorciatoia, whitelist o eccezione permanente.

**C. STAND (0–150 mm secondo la semantica del progetto; minimo `kMinStandM` 50 mm)**
6. Con la nuova `kHeight`, `stand(0.15)` non deve più essere rifiutato; dimostrare 50/100/150 mm offline sul modello con i nuovi dati inerziali.
7. Prima prova hardware sempre **supervisionata**, corpo sostenuto: lo STOP dai 150 mm fa cadere il corpo (fallimento già osservato; il fall-arrest lo ha trattenuto). Nessun flash né test dinamico automatico.

**D. Return-to-PRONE**
8. Ricertificare tabelle PRONE e ingresso (`PairedProneData.h`, `certify_measured_entry_region.py`, `certify_commissioning_entry.py`); il terminale PRONE di dev5 mantiene la coppia ON e non fa SAFE_OFF automatico. Non lanciare PRONE da 100/150 mm senza sostegno del corpo.

**E. WALK STANDARD**
9. Ricertificare `kWalk`/`kLift`/`kDeploy` e la regione di camminata (`certify_operational_walk_region.py`); stride 30/45/60 mm, periodo 6,5 s, duty 0,825, lift 8 mm come definiti in `SimpleMotion.h` (nessuna riprogettazione implicita).
10. Chiudere prima il problema di temporizzazione: WALK costa ≈ 1,5–1,6× STAND in calcolo sull'host (H: oltre 20 ms sul target); la prova USB-only preparata in `MATDOG_DEV5_WALK_PREP_20261030` è il passo supervisionato iniziale.
11. Valutare 40 RPM come obiettivo progettuale **dipendente dal carico**, non come 40 RPM a 9,2 kg·cm; mantenere STOP e limiti fisici.

## 10. Vincoli da preservare (non toccati in questo intervento)

Calibrazioni dei 12 servo; record NVS gen-1 e tag `3713f4ddc43b204e`; offset, direzioni encoder, ID servo; tick q0 e `kStartupReference`; certificati hardware e STOP; base storica `historical_calibration_basis/rev00/`; Geometry V5 (kernel, bundle, artefatti); firmware installato e branch dev5.

## 11. Verifiche eseguite in questo intervento

| Verifica | Esito |
|---|---|
| Pacchetto ZIP (CRC, `SHA256SUMS` del pacchetto) | OK |
| Workbook: OpenXML, formule, grafico, tabella (`tools/check_workbook_openxml.py`) | PASS sul canonico; **FAIL sull'approvato verbatim** per il difetto della tabella (3 colonne), corretto in `bfdf7b8` |
| Workbook vs URDF (16 giunti, 17 link) | 0 differenze |
| Manifest `SHA256SUMS.txt` (`sha256sum -c`) | 19/19 OK |
| `test_matdog_geometry_v5_immutability` + `test_matdog_calibration_basis` | 19/19 OK sul commit `423de3a`; rieseguiti sul commit finale (vedi messaggio di chiusura) |
| `matdog_calibration_basis.py --check` | PASS |
| `static_audit.py` | `STATIC_AUDIT = PASS` (exit 0) sul commit `423de3a`; rieseguito sul commit finale (vedi messaggio di chiusura) |
| Test cinematici (urdf_fk, IK, contatto, stance) | OK; `test_matdog_leg_fk_live` fallisce come prima di PR #57 (richiede dati fuori dal repo) |
| Staging del viewer da URDF canonico | URDF staged `ce63034e…`, 17 mesh |
| Decimazione con lo strumento ufficiale su 17 STL canoniche | OK (§3) |
| Non eseguito | Excel/LibreOffice; npm test del viewer (`node_modules` assente, nessuna installazione); `static_audit` e suite Motion di dev5; qualsiasi prova su hardware; campagne V5 complete (geometria invariata) |

## 12. Puntatori

`09_Logs/Validation_Reports/`: `REV01_FIX_Integration_2026-10-30/` (geometria, catena di calibrazione) · `REV01_FIX_Physical_Properties_2026-10-31/` (inerzie, strumenti, input) · `REV01_FIX_Alignment_2026-10-31/` (questo documento, `input/`, `tools/`, `data/`). `verification-artifacts/`: `MATDOG_DEV5_CHECKPOINT_20261030/`, `MATDOG_DEV5_HW_20261030/`, `MATDOG_DEV5_RCA_20261030/`, `MATDOG_DEV5_WALK_PREP_20261030/`, `MATDOG_REV01_COLLISION_OFFLINE_20261030/`.
