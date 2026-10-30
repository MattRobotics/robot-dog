# MATDOG — Integrazione REV01-FIX (base_link + quote HIP) e unificazione Visual/Collision

Data: 2026-10-30. Solo offline: nessun flash, NVS, calibrazione servo, offset EEPROM, limite meccanico o kernel V5 modificato. Nessuna prova hardware (nessun VERIFIED HARDWARE).

## Esito

| Area | Classe | Sintesi |
|---|---|---|
| Geometria V5 (q=0, pin/HIP, compilatore completo) | VERIFIED OFFLINE | PASS, nessuna nuova collisione né regressione |
| Modello canonico (URDF, mesh, manifest) | VERIFIED OFFLINE | 17/17 link con una sola STL per visual e collision |
| `LegGeometryData.h` / FK-IK C++ vs URDF | VERIFIED OFFLINE | coerenti (scarto FK 2.8e-17 m su 16 pose) |
| Viewer / Digital Twin | VERIFIED OFFLINE | test e typecheck verdi; twin rigenerato sul ramo `dev5` (sonda locale) |
| Catena di calibrazione firmware (`Calibration*Data.h`, yaml, audit) | IMPLEMENTED NOT VALIDATED | NON aggiornata, ancora legata all'URDF REV00 (vedi criticità) |
| Motion certificato di `dev5` (box/tabelle) | UNKNOWN | richiede ricertificazione, non eseguibile offline qui |

## Input

- `base_link_collision(rev01-fix).stl`: SHA-256 `c2eadf6d6db1b03379cf256b542dadbcd8d63076179bc84ebd7169634e9f0097`, 1841684 byte, 36832 triangoli.
- Sorgenti V5 identiche a `main`, `origin/main` e `dev5` (kernel non modificato). Base di integrazione: `origin/main` 9b2c42f.
- URDF integrato: SHA-256 `5677babd320aa990a70709d12027fcac64723fe1dc3fd756108972f24ad0eb73`.
- `LegGeometryData.h`: SHA-256 `114d261c12ca563dcedb…` (vedi `data/verify_integration.txt` per l'hash completo).

## A. Gate q=0 (kernel V5, stessa configurazione della verifica REV01)

REV00 12/12 `SEPARATED_NARROW`; REV01 (prima della correzione) 4 `INTERSECTING` (tutti i BASE/HIP); **REV01-FIX 12/12 `SEPARATED_NARROW`**, nessuna intersezione BASE/HIP. BASE contro gli altri 12 link: `SEPARATED_AABB`, nessuna intersezione. Dettaglio e witness triangle in `data/q0_comparison.json`.

## B. Giochi perno/sede HIP (misurati sulle STL)

Quote perni e sedi coincidono con il CAD dichiarato. Tutti gli otto accoppiamenti: gioco **assiale 0.250 mm**, gioco **radiale 0.250 mm** misurato sui vertici (raggio sede − raggio perno). La distanza minima di superficie tra le mesh è **0.203–0.204 mm** perché i due cilindri sono facettati diversamente: 0.25 mm minimi effettivi sulla mesh NON sono dimostrati, sono nominali. Dettaglio per perno in `data/pin_clearance_REV01_FIX.json`.

## C. Geometry Compiler V5 completo (24 endpoint, path obstruction attiva)

- REV00: hash semantico `6f797a32…` (riproduce la corsa precedente). Modello integrato: `eae0f5f6…`.
- 22 endpoint su 24 identici bit a bit. RH hip minimo e LH hip massimo cambiano di 6.8e-5 rad (< risoluzione di bisezione 1e-4 rad), con margine 2.66 mrad oltre il limite dichiarato.
- Stesse 6 ostruzioni di percorso con gli stessi id e stessi 8 contatti già interni ai limiti dichiarati dei giunti upper-leg di REV00. Nessuna nuova collisione né regressione.
- Il modello integrato è riproducibile: stessi endpoint della corsa sul candidato validato, cambia solo la provenienza.

## Integrazione

- `base_link.stl` collision (= visual) sostituita con REV01-FIX; quattro origin HIP a Z 0.0487 (LF/RF) e 0.0287 (RH/LH); offset anteriore/posteriore ancora 20.0 mm.
- Ogni `<visual>` e `<collision>` punta a `meshes/collision/<link>.stl` (scale e origin invariati). Le 17 STL solo estetiche in `meshes/` sono rimosse con `git rm` (recuperabili dalla storia).
- Il test su testo URDF ora prova che le uniche differenze rispetto al digest storico sono le quattro Z e i path visual.
- `LegGeometryData.h` rigenerato con `matdog_motion_geometry_export.py` (recuperato dai rami `integration/matdog-controller-v0.2.0` ecc., SHA `d0e22b49…` = manifest; non incluso in `main` perché «deferred»). Il diff cambia solo le quattro Z e l'hash URDF.
- Pin aggiornati: `matdog_urdf_fk.py`, `MATDOG_BODY_STANCE_GEOMETRY.yaml`, `MATDOG_FOOT_CONTACT_GEOMETRY.yaml`, `motion_integration_manifest_pr2.json`, `SHA256SUMS.txt`, test di immutabilità (19 voci), README del modello e del viewer.
- Test che assumevano due insiemi di mesh o attesi REV00 aggiornati con valori derivati dal modello (scostamento esatto +2.2 mm, o ×cos(pitch) per le chiusure mondo), senza allentare le tolleranze.

## Regressione (modello integrato vs baseline `origin/main`)

Baseline: tutti i test geometria/cinematica passano tranne `test_matdog_leg_fk_live` (già in fallimento prima; richiede dati non nel repo). Dopo le correzioni dei test: stessi esiti, più l'audit statico che passa in baseline e dopo l'integrazione segnala 3 problemi nella catena di calibrazione (sotto). Viewer: 59/59 test, typecheck OK. Convergenza motion: PASS. `tools/verify_integration.py`: PASS su tutti i controlli strutturali.

Su `dev5` (sonda locale, non pubblicata): build firmware esterna OK (1582608 B, 30.2% dello slot, non flashata), `TwinAssets.h` rigenerato con il generatore ufficiale (HTML/JS bit-identici, solo mesh e URDF cambiano; 42+50 controlli twin PASS, 234931 B), viewer 85/85.

## Criticità aperte

1. **Catena di calibrazione firmware non aggiornata.** `Calibration{GeometryProfile,Q0Evidence,SequencePlan}Data.h`, `MATDOG_JOINT_CALIBRATION.yaml`, `matdog_full_calibration_sequence_geometry_v5.py` e il pin in `static_audit.py` restano legati all'URDF REV00 (3890a3f0…) e alle evidenze del robot. Rigenerarli riassocierebbe calibrazione e record NVS a un URDF senza evidenza hardware, fuori mandato. La static audit segnala 3 problemi (export del piano di sequenza «computed on another URDF / mesh set», export del profilo geometrico «input drift», mutation test collegato). Decisione e riqualificazione necessarie prima di compilare un firmware da questo modello per il robot.
2. **Motion certificato di `dev5`.** Con la nuova geometria, `SimpleMotion::stand(0.15)` viene rifiutato (l'IK a 150 mm esce dal box certificato `collision::kHeight`, calcolato su REV00); `PairedProneData.h`, first-stride e box `kEntry/kWalk/kLift` sono pure certificati su REV00. Serve la ricertificazione con gli strumenti FCL ufficiali (non disponibili qui). Il comportamento è fail-closed.
3. **Dati dinamici storici di `base_link`** (massa 1.282 kg, CoM, inerzia) e `reference/Coordinate_Cinematiche_matt_robodog_rev00.xlsx` (quote HIP REV00): servono i valori aggiornati di Solid Edge; non sono stati inventati.
4. **Smoke test browser del viewer non ripetuto** (atteso 134 484 + 20 = 134 504 triangoli, non misurato).
5. Distanza minima di superficie perno/sede 0.203 mm vs 0.25 nominale (sezione B).
