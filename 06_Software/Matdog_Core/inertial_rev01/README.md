# MATDOG REV01 — calcolo inerziale offline

IMPLEMENTED e OFFLINE VALIDATED per i calcoli. REV01 rimane PENDING MEASUREMENT e
HARDWARE UNVERIFIED. Gli input REV00 sono una baseline CAD storica, con simmetrie
ASSUMED per l’assemblaggio attuale. Nessun risultato genera autorità Motion.

Il codice usa solamente nuove directory e importa in sola lettura la FK URDF e le
funzioni del poligono C4-E esistenti. Non scrive URDF, STL, Excel, firmware,
calibrazione o G5-B. Non ha comandi seriali, flash o attuatori.

## Riproduzione

Dalla root del worktree isolato:

```bash
python3 -B -m unittest discover -s 06_Software/Matdog_Core/inertial_rev01/tests -v
python3 -B 06_Software/Matdog_Core/inertial_rev01/run_validation.py
```

Il core e l’estrattore OOXML richiedono solo Python standard. Il runner completo
richiede NumPy per l’oracle indipendente e g++ per il probe C++17. Nell’ambiente di
questa revisione sono già disponibili; il runner non installa o scarica niente.
Deve essere già disponibile l’oggetto Git della PR #50
`0529e44303467c454b07eb7f0e24bedd07182f08`. Se mancano dipendenze/oggetti locali il
runner fallisce, senza dichiarare PASS o ripiegare su un header imitato.

Gli output predefiniti sono in
`09_Logs/Validation_Reports/REV01_Inertial_2026-10-14/`.
Per confrontare replay indipendenti usare `--report-dir /tmp/<directory-nuova>`.
`--oracle-python` permette di usare un interprete con NumPy già installato.
`--compiler` seleziona un compilatore C++17 host. Il probe materializza soltanto gli
header G5-B e Motion in un TemporaryDirectory e compila un programma host che
assegna i campi del contratto. Non compila né rilascia il firmware.

## Input parametrico

`data/mass_budget_rev01.json` è l’unico input operativo del budget. Per preparare
una configurazione senza alterare il template originale, copiarlo in una nuova
directory di lavoro e passarlo con `--budget <file.json> --report-dir <directory>`.
Gli output JSON e REPORT.md vengono rigenerati soltanto dopo tutti i controlli.
I file candidati REV01 non vengono prodotti per input incompleti.

`PENDING_MEASUREMENT` è un valore esplicito, diverso da zero o assenza. Un budget
incompleto restituisce `complete=false`, `result=null` e l’elenco dei campi
mancanti. Nessun subtotal parziale diventa massa totale o CoM REV01.

Le unità accettate sono esattamente kg, m, rad, kg*m^2. Gli array sono coordinate
di tre numeri e matrici 3×3. I booleani, valori non finiti, masse non positive,
link ignoti e tensori non fisici sono rifiutati. Non si stimano densità né forme.
La simmetria e la fisicità sono verificate con tolleranza relativa 1e-12 alla
scala del tensore. Dopo la verifica si media la coppia simmetrica soltanto per
rimuovere l’errore numerico di rotazione; asimmetrie superiori sono rifiutate.

Per ciascun componente scegliere una disposizione:

| Disposizione | Campi necessari | Effetto |
|---|---|---|
| `PENDING_MEASUREMENT` | Dati ancora da fornire | Blocca il risultato completo |
| `absent` | `evidence` dell’assenza in questa configurazione | Nessuna massa aggiunta |
| `included_in_link` | `parent_link`, `evidence`, `coverage_note` | Già coperto dal link, nessuna addizione |
| `add` | Parent, massa, CoM, tensore, orientamento, evidence, coverage_note | Addizione disgiunta, rigidamente solidale al parent |
| `replace_link` | Gli stessi dati, riferiti all’intero link nuovo | Sostituzione completa del link storico |

Per `add` / `replace_link`:

```text
mass_kg: massa totale installata della riga (inclusi elementi dichiarati nella copertura)
com_in_parent_m: CoM espresso negli assi e nell’origine del parent/link sostituito
inertia_at_com_component_kg_m2: tensore al CoM negli assi del componente
component_rpy_in_parent_rad: R_parent_from_component = Rz(yaw) Ry(pitch) Rx(roll)
parent_link: uno dei 17 link canonici
coverage_note: elenco dei componenti coperti e prova che non sono contati altrove
evidence: riferimento export/misura con revisione e hash
```

Il CoM è già nel parent: non viene ruotato una seconda volta. Il tensore viene
ruotato con `R I R^T`. Se l’export fornisce il tensore negli assi parent, indicare
esplicitamente rpy=[0,0,0] con evidenza; non copiare quello zero senza verificare.

La review di ciascuno dei 17 link deve essere `retain_historical_offline` oppure
`replace_complete_link`, con evidenza. La seconda richiede esattamente una riga
`replace_link` dello stesso link. `inventory_review` e
`assembly_configuration_description` definiscono la copertura dell’intero robot.
Le righe `included_in_link` devono essere riferite alla versione del link scelta.

Per modifiche/rimozioni da un assieme storico, usare un export completo del link
aggiornato. Non sono ammesse masse negative né sostituzioni basate sulla sola
differenza di peso. Più aggiunte rigide possono essere aggregate nel parent
mantenendo 17 link, con CoM locale e tensore aggiornati. Un neck yaw articolato
non può diventare una massa rigida per tutte le pose: la futura topologia e il
contratto limitato a 17 link devono essere rivisti separatamente.

Anche con input tutti numerici il risultato è sempre `REV01_PENDING`,
`hardware_verified=false`, `command_eligible=false`. Il software non promuove
automaticamente una revisione a VERIFIED.

## Metodo e limiti

Alla posa semantica esplicita dei 12 joint:

```text
c_i^base = R_base_from_link c_i^link + t_base_from_link
I_i^base = R_base_from_link I_i^link R_base_from_link^T
M = sum(m_i)
C = sum(m_i c_i^base) / M
I_C = sum(I_i^base + m_i (||c_i^base-C||² E - (c_i^base-C)(c_i^base-C)^T))
```

Si tratta dell’inerzia equivalente a posa congelata. Non è la matrice di massa
articolata, non include inerzia riflessa degli attuatori non fornita e non prova
stabilità dinamica. La posa q=0 è una fixture numerica, non un comando/stance.

La proiezione geometrica offline trasforma C con la BodyPose e interseca la retta
lungo gravità world con il piano esplicito. Riusa la convex hull e il margine
signed di C4-E in coordinate ortonormali del piano. Il margine è la distanza
euclidea minima ai segmenti del bordo, con segno C4-E. Per punti esterni vicino a
un vertice può differire da un futuro margine di semipiani/rette infinite:
questa convenzione va concordata col futuro consumer G5-B.

I contatti del replay sono riferimenti G2/strip nominale presenti in C4-C, non
origini dei foot_link né misure di carico. Freshness, contatti confermati, attrito,
compliance e acquisizione non sono osservati. Il confronto della scatola ±20 mm
riproduce C4-E e non assegna un’incertezza REV01.

`contract.py` produce un companion candidato con posa e hash canonico. Non è un
loader runtime. I 17 link sono nello stesso ordine canonico dell’URDF. Le stringhe
rispettano le capacità C++ in byte UTF-8 più NUL. `measuredAtS=0` indica dato non
misurato/fresco; la futura misura richiede una mappatura di dominio temporale.
Il C++ non contiene la posa: il futuro consumer dovrà ricalcolare l’aggregato
alla posa corrente e validare tutte le dipendenze di `InertialInput.h`.

Il modulo `sensitivity()` calcola spostamento esatto e derivate per un’addizione
disgiunta quando massa e CoM sono disponibili nella stessa posa/frame. Per più
accessori usare un’unica aggregazione. Per sostituzioni confrontare le due
configurazioni complete; non sommare spostamenti da formule di addizione.

## Artefatti

- [Template misure](MEASUREMENTS.md) e `data/measurement_intake_template.json`.
- [Report](../../../09_Logs/Validation_Reports/REV01_Inertial_2026-10-14/REPORT.md).
- `historical_source_audit.json`: 17 link, tensori, autovalori, provenienza, celle
  workbook con formula e cache separate; cache non ricalcolate o certificate.
- `mass_budget_result.json`, `missing_measurements.json`: risultato chiuso sui
  dati mancanti e lista puntuale.
- `accessory_sensitivity.json`: campi mancanti e derivate per ogni addizione;
  per input completi il riferimento è lo scenario senza quella singola riga.
- `historical_c4e_com_replay.json`: 51 pose e margini del CoM storico.
- `pr50_candidate_historical.json`: mapping numerico storico, non idoneo runtime.
- `validation.json`: test, oracle indipendente, contratto e hash delle fonti.

Non sono state modificate le PR #47/#49/#50, le checkout Motion o Claude Code.
Il lavoro parte dal `main` verificato fdd04345c39b50137cc7cdc50fb08cd592b8c2af.
