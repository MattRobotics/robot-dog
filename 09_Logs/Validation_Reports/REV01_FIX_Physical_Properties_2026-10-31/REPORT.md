# MATDOG REV01-FIX — Integrazione proprietà fisiche (Solid Edge 2026-10-30)

Data: 2026-10-31. Solo offline. Nessun flash, cattura q0, calibrazione, scrittura NVS/EEPROM. Nessuna validazione dinamica hardware: le proprietà sono dati CAD, non misure di dinamica sul robot.

## 1. Esito

| Area | Classe | Sintesi |
|---|---|---|
| 17 blocchi `<inertial>` nell'URDF | VERIFIED OFFLINE (dati CAD) | masse, CoM e tensori dai report Solid Edge; definiti positivi, simmetrici, disuguaglianza triangolare, riproducono i momenti CAD stampati entro l'arrotondamento |
| Workbook canonico (un solo file) | VERIFIED OFFLINE | formule preservate (127/129 identiche, 2 modificate per la nuova configurazione), ricalcolo coerente |
| Geometria / mesh / cinematica | invariate | URDF privato degli `<inertial>`: albero canonico identico (digest `d66fa1cc…`); 0 STL modificate; q=0 V5 12/12 |
| Catena di calibrazione gen-1 | invariata | nessun file della catena toccato; tag `3713f4ddc43b204e` e record gen-1 riconosciuti (test host) |
| Resto del bilancio massa (9 g) | UNKNOWN (origine non risolvibile con le evidenze) | vedi §4 |

## 2. Input e integrità

Il manifest `input/SHA256SUMS.json` è stato verificato per tutti gli 8 file (hash e dimensioni) prima dell'uso. Gli originali sono in `input/` (report Solid Edge, workbook dei pesi, documento di pesata). Nulla è stato modificato.

## 3. Configurazione fisica canonica

Un solo modello nominale, **con Jetson Orin** (175 g, già nel CAD dentro `base_link`): assieme Solid Edge 2,937 kg. La pesata del 30/10/2026 (2,807 kg, senza Jetson) è conservata come evidenza storica della configurazione montata quel giorno, non come seconda configurazione. La massa della Jetson non è stata aggiunta di nuovo.

## 4. Riconciliazione dei totali

| Voce | Valore |
|---|---|
| Somma delle 17 masse per-link (base 1,828 + 4 × 0,275) | 2,928 kg |
| Assieme Solid Edge | 2,937 kg |
| Residuo | **9 g** |
| Volume: somma dei 17 link vs assieme | 1 970 534,788 vs 1 970 535,851 mm³ → differenza 1,06 mm³ |

- **Non è geometria mancante:** i volumi chiudono a 1,06 mm³ (≈1,6 mg a densità 1,5 g/cm³, contro 9 g di residuo), quindi nessun solido manca dai 17 link.
- **Origine:** la risoluzione di 1 g dei report. Con 5 valori distinti replicati sulle 4 zampe l'errore massimo da arrotondamento è 0,5 + 16 × 0,5 = **8,5 g**; il residuo di 9 g lo supera di 0,5 g. Quindi serve troncamento invece di arrotondamento, oppure masse non identiche tra le zampe. Con le evidenze fornite (solo LF) non si può distinguere: non è stata applicata alcuna correzione né scala, non sono stati inventati componenti.
- **Controllo di plausibilità:** una singola massa mancante di 9 g non spiega l'offset del CoM (richiederebbe y ≈ −103 mm, fuori dall'ingombro); è coerente con arrotondamenti distribuiti.
- **Pesata vs CAD:** CAD senza Jetson = 2,937 − 0,175 = 2,762 kg; misurato 2,807 kg → +45 g. Il workbook dei pesi calcola 2,7682 kg senza Jetson (distinta bottom-up) → 38,8 g. L'ipotesi "cablaggi interni nel corpo centrale" resta un'ipotesi, non una misura; nessuna massa è stata aggiunta. Osservazione: il workbook dei pesi conta 5 alloggiamenti motore nel gruppo base; l'evidenza non dice se la massa CAD di `base_link` includa il quinto: UNKNOWN.
- **Distinta bottom-up con Jetson:** 2,9432 kg, 6,2 g sopra l'assieme CAD (0,21 %).

## 5. Ricostruzione dei tensori

- Report in frame di link (verificato: il CoM dell'anca LF coincide con il CoM già usato nell'URDF REV00).
- I momenti "rispetto agli assi globali" sono riferiti all'**origine CAD**, non al CoM: confermato (es. base: Ixx_CoM = 0,433 − m·z² = 0,188 kg·dm²). Sono usati solo come controllo.
- Tensore al CoM: `I = Rᵀ diag(m·K²) R` con assi principali ortonormalizzati (SVD; errore di ortogonalità ≤ 9e-4, determinante +1,000 ± 0,0007), K a 0,01 mm. Unità convertite in kg, m, kg·m².
- Il metodo riproduce a 12 cifre i valori REV00 dell'anca e (a parità di massa) del lower leg.
- Controllo sui momenti stampati: riportando i tensori all'origine CAD l'errore massimo è 2,7e-6 (base), 4,8e-6 (anca), 2,4e-6 (upper), 4,0e-6 (lower), 4,4e-6 (piede) kg·m², cioè sotto i 5e-6 dell'arrotondamento dei report (0,0005 kg·dm²). Il valore "0.000" del piede è arrotondamento (il tensore ricostruito dà 4e-6 all'origine), non inerzia nulla.
- Segno dei prodotti: i prodotti stampati sono ∫xy dm ecc.; i termini fuori diagonale del tensore URDF ne sono l'opposto (verificato su tutte e 5 le parti).
- Positività (autovalori > 0), simmetria, disuguaglianza triangolare dei momenti principali: verificate per tutti i 17 blocchi.

## 6. Simmetrie (verificate, non assunte)

Confronto delle mesh di collisione nei frame locali (densità uniforme; serve solo per le trasformazioni, non per le masse):

- **RF = riflessione Y di LF** per hip, upper, lower (CoM ≤ 0,0002 mm, inerzia ≤ 8e-6 relativo).
- **RH:** upper e lower = riflessione Y di LF (esatto). **LH:** upper, lower = frame di LF (esatto).
- **Piedi:** identici nelle quattro zampe.
- **Anche posteriori:** le mesh RH/LH differiscono da LF (volume −0,35 %, CoM 0,21 mm, inerzia 0,8 % come stima da mesh). Le anche posteriori ereditano le proprietà CAD di LF (RH riflessa in Y): incertezza stimata sotto la risoluzione di 1 g (2 %), documentata, non misurata sul CAD.

## 7. Tabella dei 17 link

### Per-link table (SI; CoM in the link frame; tensor about the CoM in link axes)

| link | m [kg] | CoM x,y,z [mm] | Ixx | Iyy | Izz | Ixy | Ixz | Iyz | rule |
|---|---|---|---|---|---|---|---|---|---|
| base_link | 1.828 | -6.93, -0.44, 36.57 | 1.88547e-03 | 1.36083e-02 | 1.40540e-02 | -1.4261e-05 | -2.5599e-04 | -1.1598e-05 | CAD direct |
| lf_hip_link | 0.050 | 3.66, 29.82, 6.94 | 4.02252e-05 | 1.89581e-05 | 4.54281e-05 | -1.3146e-06 | -1.1395e-06 | -4.4640e-06 | CAD direct |
| lf_upper_leg_link | 0.152 | 1.22, 0.14, -45.03 | 1.79348e-04 | 1.73652e-04 | 2.84456e-05 | -6.2663e-08 | 0.0000e+00 | 0.0000e+00 | CAD direct |
| lf_lower_leg_link | 0.059 | 30.75, -1.18, -19.58 | 2.13364e-05 | 1.01668e-04 | 1.03445e-04 | 6.0418e-07 | 1.6689e-05 | -3.6883e-07 | CAD direct |
| lf_foot_link | 0.014 | -0.74, 0.00, 15.72 | 8.92099e-07 | 8.43046e-07 | 8.93205e-07 | 0.0000e+00 | -5.5621e-09 | 0.0000e+00 | CAD direct |
| rf_hip_link | 0.050 | 3.66, -29.82, 6.94 | 4.02252e-05 | 1.89581e-05 | 4.54281e-05 | 1.3146e-06 | -1.1395e-06 | 4.4640e-06 | Y reflection of LF |
| rf_upper_leg_link | 0.152 | 1.22, -0.14, -45.03 | 1.79348e-04 | 1.73652e-04 | 2.84456e-05 | 6.2663e-08 | 0.0000e+00 | 0.0000e+00 | Y reflection of LF |
| rf_lower_leg_link | 0.059 | 30.75, 1.18, -19.58 | 2.13364e-05 | 1.01668e-04 | 1.03445e-04 | -6.0418e-07 | 1.6689e-05 | 3.6883e-07 | Y reflection of LF |
| rf_foot_link | 0.014 | -0.74, 0.00, 15.72 | 8.92099e-07 | 8.43046e-07 | 8.93205e-07 | 0.0000e+00 | -5.5621e-09 | 0.0000e+00 | identical foot geometry |
| rh_hip_link | 0.050 | 3.66, -29.82, 6.94 | 4.02252e-05 | 1.89581e-05 | 4.54281e-05 | 1.3146e-06 | -1.1395e-06 | 4.4640e-06 | Y reflection of LF |
| rh_upper_leg_link | 0.152 | 1.22, -0.14, -45.03 | 1.79348e-04 | 1.73652e-04 | 2.84456e-05 | 6.2663e-08 | 0.0000e+00 | 0.0000e+00 | Y reflection of LF |
| rh_lower_leg_link | 0.059 | 30.75, 1.18, -19.58 | 2.13364e-05 | 1.01668e-04 | 1.03445e-04 | -6.0418e-07 | 1.6689e-05 | 3.6883e-07 | Y reflection of LF |
| rh_foot_link | 0.014 | -0.74, 0.00, 15.72 | 8.92099e-07 | 8.43046e-07 | 8.93205e-07 | 0.0000e+00 | -5.5621e-09 | 0.0000e+00 | identical foot geometry |
| lh_hip_link | 0.050 | 3.66, 29.82, 6.94 | 4.02252e-05 | 1.89581e-05 | 4.54281e-05 | -1.3146e-06 | -1.1395e-06 | -4.4640e-06 | same left-side frame as LF |
| lh_upper_leg_link | 0.152 | 1.22, 0.14, -45.03 | 1.79348e-04 | 1.73652e-04 | 2.84456e-05 | -6.2663e-08 | 0.0000e+00 | 0.0000e+00 | same left-side frame as LF |
| lh_lower_leg_link | 0.059 | 30.75, -1.18, -19.58 | 2.13364e-05 | 1.01668e-04 | 1.03445e-04 | 6.0418e-07 | 1.6689e-05 | -3.6883e-07 | same left-side frame as LF |
| lh_foot_link | 0.014 | -0.74, 0.00, 15.72 | 8.92099e-07 | 8.43046e-07 | 8.93205e-07 | 0.0000e+00 | -5.5621e-09 | 0.0000e+00 | identical foot geometry |

### Change vs previous REV00 blocks

| link | m old → new [kg] | ΔCoM [mm] | Ixx old → new | Iyy old → new | Izz old → new |
|---|---|---|---|---|---|
| base_link | 1.2820 → 1.828 | -12.77, 0.21, 8.84 | 1.4727e-03 → 1.8855e-03 | 8.8701e-03 → 1.3608e-02 | 9.4991e-03 → 1.4054e-02 |
| lf_hip_link | 0.0500 → 0.050 | 0.00, 0.00, 0.00 | 4.0225e-05 → 4.0225e-05 | 1.8958e-05 → 1.8958e-05 | 4.5428e-05 → 4.5428e-05 |
| lf_upper_leg_link | 0.1725 → 0.152 | 0.14, -0.01, 0.00 | 2.0544e-04 → 1.7935e-04 | 1.9918e-04 → 1.7365e-04 | 3.1113e-05 → 2.8446e-05 |
| lf_lower_leg_link | 0.0595 → 0.059 | 0.00, 0.00, 0.00 | 2.1517e-05 → 2.1336e-05 | 1.0253e-04 → 1.0167e-04 | 1.0432e-04 → 1.0344e-04 |
| lf_foot_link | 0.0175 → 0.014 | -0.22, 0.00, 0.24 | 1.1453e-06 → 8.9210e-07 | 1.2027e-06 → 8.4305e-07 | 1.1484e-06 → 8.9320e-07 |
| rf_hip_link | 0.0500 → 0.050 | 0.00, 0.00, 0.00 | 4.0225e-05 → 4.0225e-05 | 1.8958e-05 → 1.8958e-05 | 4.5428e-05 → 4.5428e-05 |
| rf_upper_leg_link | 0.1725 → 0.152 | 0.14, 0.01, 0.00 | 2.0544e-04 → 1.7935e-04 | 1.9918e-04 → 1.7365e-04 | 3.1113e-05 → 2.8446e-05 |
| rf_lower_leg_link | 0.0595 → 0.059 | 0.00, 0.00, 0.00 | 2.1517e-05 → 2.1336e-05 | 1.0253e-04 → 1.0167e-04 | 1.0432e-04 → 1.0344e-04 |
| rf_foot_link | 0.0175 → 0.014 | -0.22, 0.00, 0.24 | 1.1453e-06 → 8.9210e-07 | 1.2027e-06 → 8.4305e-07 | 1.1484e-06 → 8.9320e-07 |
| rh_hip_link | 0.0500 → 0.050 | 0.00, 0.00, 0.00 | 4.0225e-05 → 4.0225e-05 | 1.8958e-05 → 1.8958e-05 | 4.5428e-05 → 4.5428e-05 |
| rh_upper_leg_link | 0.1725 → 0.152 | 0.14, 0.01, 0.00 | 2.0544e-04 → 1.7935e-04 | 1.9918e-04 → 1.7365e-04 | 3.1113e-05 → 2.8446e-05 |
| rh_lower_leg_link | 0.0595 → 0.059 | 0.00, 0.00, 0.00 | 2.1517e-05 → 2.1336e-05 | 1.0253e-04 → 1.0167e-04 | 1.0432e-04 → 1.0344e-04 |
| rh_foot_link | 0.0175 → 0.014 | -0.22, 0.00, 0.24 | 1.1453e-06 → 8.9210e-07 | 1.2027e-06 → 8.4305e-07 | 1.1484e-06 → 8.9320e-07 |
| lh_hip_link | 0.0500 → 0.050 | 0.00, 0.00, 0.00 | 4.0225e-05 → 4.0225e-05 | 1.8958e-05 → 1.8958e-05 | 4.5428e-05 → 4.5428e-05 |
| lh_upper_leg_link | 0.1725 → 0.152 | 0.14, -0.01, 0.00 | 2.0544e-04 → 1.7935e-04 | 1.9918e-04 → 1.7365e-04 | 3.1113e-05 → 2.8446e-05 |
| lh_lower_leg_link | 0.0595 → 0.059 | 0.00, 0.00, 0.00 | 2.1517e-05 → 2.1336e-05 | 1.0253e-04 → 1.0167e-04 | 1.0432e-04 → 1.0344e-04 |
| lh_foot_link | 0.0175 → 0.014 | -0.22, 0.00, 0.24 | 1.1453e-06 → 8.9210e-07 | 1.2027e-06 → 8.4305e-07 | 1.1484e-06 → 8.9320e-07 |

previous REV00 blocks: mass 2.4800 kg, CoM [mm] [9.571, -0.336, 7.032], I about CoM (base axes) diag [0.01593, 0.0293, 0.03647], off-diag xy,xz,yz [1.1e-05, -0.00064, 1e-05], principal [0.03649, 0.0293, 0.01591] kg m2

new nominal blocks: mass 2.9280 kg, CoM [mm] [0.688, -0.275, 17.286], I about CoM (base axes) diag [0.01592, 0.03337, 0.03899], off-diag xy,xz,yz [-2e-05, 6.1e-05, 4e-06], principal [0.03899, 0.03337, 0.01592] kg m2


Modello nominale completo (con Jetson), nel frame di `base_link`, URDF REV01-FIX: **2,928 kg**, CoM **[0,688, −0,275, 17,286] mm**; assieme Solid Edge: 2,937 kg, CoM [0,78, −0,59, 17,02] mm → scarto CoM **0,42 mm**; momenti principali entro 1 % (raggi di girazione 115,39/106,76/73,74 vs 115,56/106,96/73,98 mm). Il modello precedente (REV00) era 2,480 kg con CoM z ≈ 6–7 mm (6,0 mm nel workbook REV00; 7,0 mm ricalcolato sulle posizioni HIP REV01-FIX): la differenza principale è la massa del base_link (1,282 → 1,828 kg, batteria/Jetson inclusi nel CAD nominale).

## 8. URDF e compatibilità con la base storica di calibrazione

- Cambiano solo i 17 `<inertial>` (e i relativi commenti di provenienza). URDF: `5677babd…` → `ce63034e727e82ee82267fb97cea01cd6e171e850cb2d42cb42cf763ee3706f7`.
- Prova che la geometria non è cambiata: URDF senza `<inertial>` → albero canonico identico prima/dopo (digest `d66fa1cc84a39da41a5e3a20e2dd16a98658d017a539c7f615843ded2509c2e7`); 0 STL modificate; coordinate HIP, assi, limiti, motorId invariati; V5 q=0 12/12 identico. Il Geometry Compiler V5 completo **non** è stato rieseguito: i suoi input geometrici (superfici di collisione, cinematica) sono identici; è cambiato solo lo SHA del file URDF.
- `matdog_calibration_basis.py`: il delta ammesso ora include **solo** i blocchi `<inertial>` dei 17 link elencati in `BASIS.json`, con controlli strutturali (figli esatti, rpy 0, attributi) e fisici (massa in (0, 5] kg, tensore simmetrico definito positivo, disuguaglianza triangolare, CoM dentro il bounding box della mesh di collisione, raggio di girazione ≤ diagonale). Continua a rifiutare modifiche a joint, assi, limiti, motori, collisioni, mesh, altri elementi: 16 test di mutazione, due dei quali hanno rivelato e corretto difetti reali del checker (eccezione su mesh illeggibile; divisione con massa non positiva).
- Input REV00 storici e relativi hash invariati; tag `3713f4ddc43b204e`, record NVS gen-1 e RESTORE preservati (file della catena byte-identici).
- Dati derivati dall'URDF operativo aggiornati con i generatori/strumenti esistenti: `SHA256SUMS.txt` (solo la riga dell'URDF), `LegGeometryData.h` rigenerato con l'exporter ufficiale (**cambia solo `kUrdfSha256`**; coordinate cinematiche identiche), pin in `matdog_urdf_fk.py`, `MATDOG_BODY_STANCE_GEOMETRY.yaml`, `MATDOG_FOOT_CONTACT_GEOMETRY.yaml`, `motion_integration_manifest_pr2.json` (4 pin), test di immutabilità (hash del manifest), README del viewer. Il test sul testo URDF è stato riscritto come test semantico sul digest dell'URDF privato degli `<inertial>`. Il test host gen-1 non fissa più l'hash operativo.
- Le evidenze hardware REV00 non sono state riemesse come certificazioni REV01.

## 9. Workbook

Un solo workbook canonico, `03_CAD/URDF/matt_robodog_rev00/reference/Coordinate_Cinematiche_matt_robodog_rev00.xlsx`, aggiornato modificando direttamente l'XML (stili, tabelle, unioni e formule dei fogli non toccati sono intatti). SHA-256 del workbook finale `dbfe1e2581f1c1e3f3a43d29b6e21a4756e0462d579c3d80a6fa8c5a5c2c7da2` (registrato in `SHA256SUMS.txt` del modello, il cui hash è `47b48b22727f613e75db49ff5447236fbd4a665d065c4912a9ee6b2b767a18ab`). Lo storico è in Git (commit `0f8f7ff`, SHA-256 della versione precedente `381675b4c4fae0305ed2a2e882c0b9b43f513d9f0e53afff41e60801d798b13f`); nessuna copia concorrente.

- `Inertial_Properties`: 17 righe nuove, totale e confronto con l'assieme (formule). È l'unica sorgente delle masse: `Link_Engineering` e `Mesh_Transforms` ora leggono le masse da qui con formule (prima erano numeri duplicati).
- `Mesh_Transforms`/`Link_Engineering`: percorsi mesh `meshes/collision/<link>.stl` (visual = collision).
- `Joint_Parameters`: quote HIP Z 48,7 mm (LF/RF) e 28,7 mm (RH/LH); le formule in metri si ricalcolano (0,0487 / 0,0287).
- `CAD_Source_Data`, `Mass_Summary`, `Mass_Reconciliation`, `Physical_Property_Method`, `URDF_Audit`, `Reference_Notes`: dati e testi aggiornati; i confronti CAD/reale usano la configurazione senza Jetson.
- Nuovi fogli: `Weight_Evidence_2026-10-30` (il workbook dei pesi fornito, formule e valori invariati) e `Physical_Evidence_Log` (hash e posizione delle evidenze correnti e storiche).
- Rimossi soltanto contenuti superati e duplicati (valori REV00 di massa/inerzia, percorsi `meshes/*.stl`, quote HIP 46,5/26,5). Mantenuti `Engineering_Weight_Source` (allocazione componenti/ID motore, storica) e tutti gli ID servo e limiti.
- Formule: 129 originali, 127 identiche, 2 modificate in `Mass_Reconciliation` (B10, B11: il confronto reale usa CAD senza Jetson); 185 formule nuove in totale. Ricalcolo con valutatore proprio (validato prima sul workbook originale: 129/129 e sul workbook dei pesi: 80/80 valori in cache riprodotti) → nessuna differenza dai valori memorizzati; `fullCalcOnLoad` attivo per Excel. Non è disponibile LibreOffice/Excel: la verifica con un'applicazione Excel reale NON è stata eseguita.
- Controllo di integrità OpenXML finale (2026-10-31, strutturale, senza applicazione): ZIP integro, 26 parti uniche, tutte le parti XML ben formate, 13 fogli con id e nomi unici, relazioni/content-type coerenti, righe e celle in ordine, nessun riferimento a fogli inesistenti né valori d'errore, 314 formule i cui valori memorizzati coincidono col ricalcolo. Ha rilevato un difetto già presente nel file originale: merge sovrapposti in `Mass_Summary` (A1:F1/A1:B1, A2:F2/A2:B2, A13:F13/A13:F14), che Excel avrebbe segnalato da riparare; rimossi i soli merge ridondanti (nessun valore o formula cambiato). SHA-256 workbook finale `dbfe1e2581f1c1e3f3a43d29b6e21a4756e0462d579c3d80a6fa8c5a5c2c7da2`.

> **Superato dal 2026-10-31 (allineamento, `../REV01_FIX_Alignment_2026-10-31/REPORT.md`).** Il workbook descritto in questa sezione (13 fogli, SHA-256 `dbfe1e25…`) è stato sostituito dal workbook consolidato a 7 fogli approvato dall'operatore (SHA-256 `68e64ce0…`). I fogli `Mesh_Transforms`, `Link_Engineering`, `Mass_Summary`, `Engineering_Weight_Source`, `URDF_Audit`, `Physical_Property_Method` e `Physical_Evidence_Log` citati sopra non esistono più nel file corrente; la versione descritta qui resta recuperabile da Git (commit `7d04939`). I valori di massa, CoM e inerzia nell'URDF non sono cambiati.

## 10. Evidenze storiche preservate
`input/` (originali), Git (workbook precedente, URDF precedente, commit `0f8f7ff`), `03_CAD/URDF/historical_calibration_basis/rev00/` (input REV00 della calibrazione), i log hardware esistenti, `calibration_chain/` della cartella REV01_FIX_Integration.

## 11. Criticità aperte
1. Residuo di 9 g: origine non risolvibile con le evidenze (§4).
2. Anche posteriori: proprietà ereditate da LF con incertezza stimata da mesh.
3. Precisione: le masse hanno risoluzione di 1 g (fino al 3,5 % sul piede da 14 g); le inerzie ne ereditano l'incertezza.
4. Configurazione reale senza Jetson: non esistono proprietà inerziali del robot attualmente assemblato senza Jetson (la posizione della Jetson nel base_link non è nell'evidenza).
5. Nessuna validazione dinamica hardware; Startup Recovery, STAND 150 mm, certificazioni Motion dev5: invariati e fuori mandato.
6. Workbook non aperto in Excel/LibreOffice (vedi §9).
