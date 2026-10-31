# MATDOG — pacchetto di input per allineamento repository
Data: 2026-10-31. NON è un firmware installabile e NON è una release firmware.

## Input approvato
- `INPUT/Coordinate_Cinematiche_matt_robodog_rev00.xlsx`
- SHA-256 del file: `68e64ce08ce7fc035d8652d9e2bc22c73ea6a8ffab56cd17898543087ba9a546`
- 7 fogli; modello nominale con Jetson; proprietà dei 17 link; 40 RPM limite progettuale stimato quasi a vuoto a 11,1 V; curva coppia-velocità preliminare e grafico Excel.

## Sorgente canonica Git — NON duplicare STL dal pacchetto
Repository: https://github.com/MattRobotics/robot-dog
PR: https://github.com/MattRobotics/robot-dog/pull/57
Head verificato prima di questo pacchetto: `7d04939a58b8afb190797518e8685ebe3f125367` (può cambiare: ricontrollare il branch prima delle scritture).
Branch PR: `geometry/rev01-fix-integration-20261030`.
Path workbook da sostituire nel branch PR:
`03_CAD/URDF/matt_robodog_rev00/reference/Coordinate_Cinematiche_matt_robodog_rev00.xlsx`
Le 17 STL canoniche già versionate via Git LFS sono in:
`03_CAD/URDF/matt_robodog_rev00/meshes/collision/`.
URDF operativo: `03_CAD/URDF/matt_robodog_rev00/matt_robodog_rev00.urdf`.
La base di calibrazione storica REV00, tag NVS gen-1 e relativi artefatti non vanno riscritti.

## Mandato di aggiornamento repository
1. Prima di scrivere, controllare branch, head, worktree, manifest e diff.
2. Sostituire solo il workbook canonico con i byte dell'input approvato (file della cartella INPUT); verificare lo SHA.
3. Aggiornare la riga del workbook nel manifest SHA256SUMS e i test/pin che dipendono davvero dall'hash del workbook; aggiornare le descrizioni dei fogli nel README e i riferimenti documentali obsoleti. Non cambiare fogli/formule approvate senza evidenza di difetto.
4. Confermare che la geometria canonica è ancora una STL per link, condivisa da visual e collision, e che il viewer genera staging da quella. Nel dev5 operativo, il TwinAssets è un artefatto *generato/decimato*, non una nuova sorgente geometrica da mantenere.
5. Il workbook usa 40 RPM come riferimento progettuale a vuoto nominale, NON garantito sotto carico, 45 RPM a vuoto a 12 V come dato di targa, curva stimata. Il firmware e l'URDF correnti riportano ancora 29 RPM: non cambiare i limiti esecutivi e i dati certificati in questa integrazione documentale. Mappare la dipendenza e consegnarla ad Astra come intervento Motion separato.
6. Verifiche proporzionate: integrità ZIP/OpenXML, formule/riferimenti, grafico, 17 link, manifesto, test di immutabilità, calibration basis e static_audit. Non rieseguire inutilmente l'intero V5, geometria e inerzie non cambiano.
7. Commit/push soltanto sul branch della PR #57, mantenendola draft; nessun merge, flash, SAVE NVS o modifica a dev5 installato.
8. Consegna finale: SHA del nuovo commit, SHA workbook canonico, stato test, differenze ancora aperte, identificativo certo per handoff Astra.

## Nota su STAND/WALK, per successiva sessione Astra
Dopo il consolidamento Git, usare la geometria REV01-FIX e i dati inerziali del commit definitivo; distinguere la build locale di dev5 dal firmware realmente installato. Ricertificare i box e le traiettorie geometriche dipendenti da REV00 con strumenti ufficiali; raggiungere STAND (0-150mm secondo semantica del progetto), return-to-PRONE e WALK STANDARD; valutare 40 RPM come obiettivo progettuale dipendente dal carico, non come 40 RPM a 9,2 kg.cm, mantenendo STOP e limiti fisici. Nessun flash o test dinamico automatico senza collaudo supervisionato iniziale.
