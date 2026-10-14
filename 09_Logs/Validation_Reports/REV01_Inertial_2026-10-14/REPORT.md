# MATDOG REV01 — Mass budget e CoM offline

Base `main` verificata: `fdd04345c39b50137cc7cdc50fb08cd592b8c2af`. Branch separata `feat/matdog-rev01-inertial-model`.

| Stato | Significato e risultato |
|---|---|
| IMPLEMENTED | Estrazione, verifica SI/tensori, budget parametrico, aggregazione, proiezione C4-E e mapping candidato PR #50. |
| OFFLINE VALIDATED | 35 test deterministici; oracle NumPy indipendente su 100 pose; replay di 51 frame C4-C; probe C++17 sul contratto reale PR #50. |
| ASSUMED | Configurazione CAD REV00 storica; simmetrie locali dichiarate nel workbook; accessori rigidamente solidali al parent solo dopo indicazione esplicita. |
| PENDING MEASUREMENT | REV01, configurazione reale, inventario, masse/CoM/inerzie degli accessori e riconciliazione per-link. |
| HARDWARE UNVERIFIED | Nessun test fisico, seriale, flash o movimento. Ogni uscita resta command_eligible=false. |

## Risultato e fonti

URDF ed Excel coincidono sui 17 link: 5 report CAD diretti (base e LF), 12 derivazioni locali per simmetria/copia. Tutti i tensori sono simmetrici, definiti positivi e rispettano le disuguaglianze triangolari dei momenti principali.

Il modello storico contiene 2,480 kg. L’Excel registra anche 2,3894 kg da distinta pesi e 2,297 kg da una pesata fisica storica. Il divario CAD–pesata è 183 g (7,379% della massa CAD); configurazione, data, strumento e allocazione per-link della pesata non sono disponibili. La pesata resta un’evidenza storica riportata, non una verifica REV01. Non è stata applicata alcuna scala globale alle inerzie.

REV00 README Physical Data Status says CoM/inertia are not yet authoritative; URDF and workbook contain a historical CAD model. Original sources preserved.

Gli originali `physprop__*.txt` e `weight_breakdown_summary.txt` sono nominati dal workbook ma non presenti nel repository. La provenienza è una trascrizione storica. `CAD_Source_Data` non specifica l’unità delle inerzie globali arrotondate: questi campi non alimentano il calcolo. Il segno dei prodotti CAD va riconfermato nel nuovo export; i termini URDF sono già elementi del tensore.

Posa numerica esplicita q=0 (non una stance di contatto): massa 2.480000000 kg; CoM base [9.571040323, -0.336008065, 5.969645161] mm.

Inerzia equivalente al CoM, assi base, kg·m²:

```text
[0.0160474887521, 1.11203269457e-05, -0.00062985856494]
[1.11203269457e-05, 0.0294155239945, 1.04314906881e-05]
[-0.00062985856494, 1.04314906881e-05, 0.0364725474863]
```

L’Excel `Physical_Property_Method` conserva il confronto storico del CoM ricostruito [9,571; −0,336; 5,970] mm con il CAD completo [9,480; −0,710; 5,910] mm. La differenza massima di circa 0,374 mm è una verifica interna CAD storica, non una misura dell’assemblaggio attuale. Il CoM aggregato cambia con la posa dei joint.

## Estratto dei 17 link

Masse in kg; CoM in m nel frame locale. I sei termini del tensore e gli autovalori completi sono in `historical_source_audit.json`, senza arrotondamento editoriale.

| Link | Massa kg | CoM locale [m] | Evidenza storica |
|---|---:|---|---|
| base_link | 1.282000 | (0.00584, -0.00065, 0.02773) | DIRECT CAD |
| lf_hip_link | 0.050000 | (0.00366, 0.02982, 0.00694) | DIRECT CAD |
| lf_upper_leg_link | 0.172500 | (0.00108, 0.00015, -0.04503) | DIRECT CAD |
| lf_lower_leg_link | 0.059500 | (0.03075, -0.00118, -0.01958) | DIRECT CAD |
| lf_foot_link | 0.017500 | (-0.00052, 0.0, 0.01548) | DIRECT CAD |
| rf_hip_link | 0.050000 | (0.00366, -0.02982, 0.00694) | DERIVED BY SYMMETRY |
| rf_upper_leg_link | 0.172500 | (0.00108, -0.00015, -0.04503) | DERIVED BY SYMMETRY |
| rf_lower_leg_link | 0.059500 | (0.03075, 0.00118, -0.01958) | DERIVED BY SYMMETRY |
| rf_foot_link | 0.017500 | (-0.00052, 0.0, 0.01548) | DERIVED BY SYMMETRY |
| rh_hip_link | 0.050000 | (0.00366, -0.02982, 0.00694) | DERIVED BY SYMMETRY |
| rh_upper_leg_link | 0.172500 | (0.00108, -0.00015, -0.04503) | DERIVED BY SYMMETRY |
| rh_lower_leg_link | 0.059500 | (0.03075, 0.00118, -0.01958) | DERIVED BY SYMMETRY |
| rh_foot_link | 0.017500 | (-0.00052, 0.0, 0.01548) | DERIVED BY SYMMETRY |
| lh_hip_link | 0.050000 | (0.00366, 0.02982, 0.00694) | DERIVED BY SYMMETRY |
| lh_upper_leg_link | 0.172500 | (0.00108, 0.00015, -0.04503) | DERIVED BY SYMMETRY |
| lh_lower_leg_link | 0.059500 | (0.03075, -0.00118, -0.01958) | DERIVED BY SYMMETRY |
| lh_foot_link | 0.017500 | (-0.00052, 0.0, 0.01548) | DERIVED BY SYMMETRY |

## Budget REV01 e misure mancanti

Il template ha 10 gruppi di componenti e 17 revisioni di link. Sono aperti 116 campi obbligatori. Massa totale REV01, CoM REV01 e inerzia REV01 sono **PENDING_MEASUREMENT**; i 2,480 kg sono esclusivamente il riferimento storico, non un subtotal verificato né un limite inferiore.

Compilare il [template operativo](../../../06_Software/Matdog_Core/inertial_rev01/MEASUREMENTS.md) e la copia del [budget parametrico](../../../06_Software/Matdog_Core/inertial_rev01/data/mass_budget_rev01.json). `missing_measurements.json` separa i 116 campi del budget iniziale dai campi di acquisizione CAD/robot e provenienza del template.

Per elettronica, BNO085, cablaggi, supporto antenna posteriore, antenna/cavo, portafusibile, contatti di ricarica, configurazione batteria/alimentazione, assieme neck yaw e altri componenti occorrono disposizione, parent, massa, CoM, tensore al CoM, orientamento ed evidenza di copertura. Nessuna massa, densità o posizione è stata attribuita agli accessori.

Ogni riga deve diventare `absent`, `included_in_link`, `add` o `replace_link`, con evidenza. `add` è ammesso solo per contenuti disgiunti dal link. Per rimuovere/sostituire componenti già inglobati nel CAD si misura e sostituisce il link completo; niente masse negative o sottrazioni da proprietà ignote. Un accessorio articolato non è rappresentabile come addizione rigida durante il suo movimento: serve una futura revisione di topologia e del limite 17 link.

Le decisioni per tutti i link (`retain_historical_offline` oppure `replace_complete_link`) e la revisione dell’inventario impediscono di trasformare il completamento dei soli accessori in una verifica dell’intero robot. Anche un budget interamente numerico resta `REV01_PENDING`.

## Sensibilità del CoM

Per un accessorio disgiunto di massa m al punto r, con baseline M,C nella stessa posa/frame:

```text
C_new = C + m/(M+m) * (r-C)
dC_new/dm = M/(M+m)^2 * (r-C)
dC_new/dr = m/(M+m) * E
```

Finché r e m restano PENDING_MEASUREMENT non è possibile un ranking fisico né una previsione in mm. `accessory_sensitivity.json` riporta lo stato per ogni riga e, con budget completo, lo spostamento incrementale e le derivate rispetto allo scenario privo della singola addizione. Sul solo riferimento storico M=2,480 kg, il coefficiente infinitesimo per unità di braccio è 1/M = 0,403225806 kg⁻¹. Un incremento di massa al CoM non sposta C; la posizione lungo l’asse posteriore produce uno spostamento solo dopo che il suo braccio è misurato. Per più accessori si sommano i momenti di massa con il denominatore totale, non gli spostamenti individuali. Le incertezze di massa/posizione restano da fornire per propagare un’incertezza del CoM.

| Rapporto parametrico μ=m/M (nessun accessorio reale) | Frazione del braccio che diventa spostamento μ/(1+μ) |
|---:|---:|
| 0.001 | 0.000999001 |
| 0.010 | 0.009900990 |
| 0.050 | 0.047619048 |
| 0.100 | 0.090909091 |

Per un link sostituito il confronto è la differenza fra le due aggregazioni complete alla stessa posa. Non si usa la formula di sola addizione. Il modulo `sensitivity` e i test a differenze finite verificano le derivate; i numeri inventati nei test sono fixture matematiche, separati dai dati robot.

## CoM e poligono C4-E

Riuso in sola lettura di `_convex_hull`, `_polygon_signed_margin` e `_evaluate_frame` originali. Le 51 pose C4-C usano i joint semantici in radianti e il CoM dei 17 link trasformato in world. Il peggiore margine storico al centro è 93.663988 mm; riproducendo la vecchia scatola ±20 mm attorno al nuovo CoM storico è 73.663988 mm. Il replay riproduce anche tutti i margini originali C4-E entro 1e-12 m.

La scatola ±20 mm è soltanto il confronto storico, non l’incertezza misurata REV01. La proiezione generale è lungo la gravità world sul piano esplicito, non lungo la normale. Contatti insufficienti/collineari, non coplanari o gravità parallela al piano sono rifiutati. La routine valuta geometria offline: non osserva carico, aderenza, freshness né contatti reali confermati. Non produce un’approvazione di stabilità dinamica. La valutazione del CoM effettivo REV01 si abilita nel runner solo con budget numerico completo e continua a dichiarare HARDWARE UNVERIFIED.

## Compatibilità futura PR #50

Contratto letto al commit `0529e44303467c454b07eb7f0e24bedd07182f08`; header SHA-256 `d9310c92b52fb3d4f95372192b6be7bc8b87720a7601e235188fc6edb4a4be28`. Probe C++17 compilato ed eseguito contro l’header autentico e i suoi include transitivi, estratti in una directory temporanea da oggetti Git locali. Nessuna modifica a G5-B.

Il mapping conserva 17 link: addizioni rigide vengono aggregate al parent; inerzie per-link al CoM negli assi del link; inerzia totale al CoM negli assi base. Le capacità stringa includono il terminatore NUL e sono controllate in byte UTF-8. Il payload storico usa `HISTORICAL_REV00`, quello numerico candidato `REV01_PENDING`. Il template incompleto non viene serializzato in numeri C++ zero. `measuredAtS=0` significa indisponibile e non è un timestamp di misura fresco.

Il campo aggregato dipende dalla posa, che il contratto C++ non trasporta: il companion JSON conserva tutti i joint e la preimmagine SHA-256 canonica del payload senza hash. Un futuro consumer dovrà rigenerare l’aggregato per la posa, validare provenienza/configurazione/timestamp e implementare i gate di contatto e freshness. Il contratto richiede DEPENDENCY_MISSING finché readiness != REV01_VERIFIED; il probe preserva tale default. Non è stata implementata un’integrazione runtime né una qualifica REV01.

## Riproduzione e isolamento

```bash
python3 -B -m unittest discover -s 06_Software/Matdog_Core/inertial_rev01/tests -v
python3 -B 06_Software/Matdog_Core/inertial_rev01/run_validation.py
```

Il runner necessita di NumPy per l’oracle indipendente e di g++ per il probe host, già disponibili nell’ambiente usato. Non installa dipendenze e non scarica niente. Per il probe deve esistere localmente l’oggetto Git PR50 fissato sopra. I calcoli core e l’estrattore OOXML usano solo la libreria standard. Gli output deterministici possono essere rigenerati in una directory temporanea con `--report-dir`; non contengono timestamp di esecuzione.

Modifiche limitate a nuove directory `06_Software/Matdog_Core/inertial_rev01` e `09_Logs/Validation_Reports/REV01_Inertial_2026-10-14`. URDF/STL/Excel REV00, C4-E, firmware, calibrazione, Motion, G5-B e PR #47/#49/#50 sono preservati. Nessun merge. Hash delle fonti, estratto workbook con celle/formule/cache, risultato dei test e lista precisa dei campi mancanti sono negli artefatti JSON.
