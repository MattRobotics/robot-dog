> Pre-decision analysis (2026-10-31). Option 1 was approved and implemented: see CALIBRATION_BASIS_DECISION.md. Paths below refer to scratch copies.

# MATDOG — Catena di calibrazione vs REV01-FIX: analisi dei 3 fallimenti `static_audit`

Data: 2026-10-31. Solo offline. Nessun merge, flash, NVS, calibrazione, commit o push. Il worktree della PR #57 è invariato (HEAD `aa89dd4`, status pulito); il checkout principale è invariato.
Tutte le rigenerazioni sono state eseguite su **copie isolate** (`rehearsal_repo/`, `control_rev00_repo/`) fuori dal repository; nessun file generato è stato portato nel repo.

Classificazione: VERIFIED OFFLINE salvo dove indicato. Nessun VERIFIED HARDWARE.

## 1. I tre fallimenti

| # | Finding | Causa precisa |
|---|---|---|
| F1 | `matdog_full_calibration_sequence_geometry_v5.py --export-header` → REFUSED "computed on another URDF / mesh set" | I 4 `*_validate.json` (2026-09-30) registrano URDF `3890a3f0…` e mesh manifest `60fff604…`; il `SHA256SUMS.txt` corrente è `46bd1d1f…` (cambiato per la base REV01-FIX e per la rimozione delle 17 STL visual). Il tool confronta il manifest corrente con quello degli artefatti. |
| F2 | `matdog_calibration_geometry_export.py --check` → "input drift" | Il RUN_MANIFEST del bundle canonico (`…REMEDIATION_BENCHMARK_D_W4`) registra gli hash di 18 input; 16 STL collision coincidono, **URDF e `base_link.stl` no** (gli unici due file cambiati da REV01-FIX). |
| F3 | `test_static_audit_safe_actuator.py` (mutation test) | **Derivato da F1**: il caso baseline "24-contact sequence" si aspetta zero finding e riceve il REFUSED di F1; due mutazioni sono rilevate ma il messaggio non contiene "exporter's output" perché l'exporter rifiuta prima. |

## 2. Le tre classi di dato (cosa è cosa)

**Classe 1 — calibrazioni fisiche dei 12 servo (non toccate, non rigenerabili offline).**
Record NVS gen-1 (tag `3713f4ddc43b204e`), tick q0 misurati, `encoder_direction`, unità/bus ID, `MATDOG_SERVO_ALLOCATION.yaml`, `kStartupReference` (12 q0 misurati), `CalibrationQ0EvidenceData.h`.
Sono dati di spazio-giunto. Il diff completo URDF REV00→integrato (16 giunti, 17 link: origin, axis, limit, motorId, direction, inertial, collision) mostra **solo** 4 traslazioni `hip origin Z +2,2 mm` + percorsi visual; nessun asse, limite, motorId, direzione o altra mesh cambia. Quindi i tick q0 non dipendono da questa revisione: *derivazione statica*, non misura hardware.

**Classe 2 — dati/certificati geometrici derivati da REV00 (rigenerabili con la pipeline).**
Profilo 24 endpoint (`CalibrationGeometryProfileData.h`), parking, policy di sicurezza G12, piano di sequenza (`CalibrationSequencePlanData.h`), box di tolleranza del recupero all'avvio (`startupPositionInBand`).

**Classe 3 — controlli SHA di provenienza.**
Segnalano *staleness* reale ("dato calcolato su un altro modello"), non un'incompatibilità fisica con il robot. Ma il rebinding ha una conseguenza dura sulla Classe 1 (§4).

## 3. Per ciascun fallimento

### F1 — piano di sequenza
- **Dipendenza geometrica effettiva:** solo i 4 booleani `sequence_collision_free`; le pose (1570796/1024 tick, parcheggio 610865 µrad ≈ 35°, fold) sono costanti V25/V5.
- **Prova (rehearsal):** i 4 `validate` rigenerati sul modello integrato danno `ALL_SEQUENCES_COLLISION_FREE=True`; **53/53 segmenti con verdetto e `base_static` identici**. Controllo: lo stesso tool su REV00 riproduce i 4 artefatti del 2026-09-30 **bit a bit** (0 differenze numeriche), quindi le differenze su REV01-FIX sono geometria, non deriva dello strumento.
- **Cambia solo l'evidenza di clearance** (non gating): durante i HIP sweep delle gambe posteriori la clearance minima verso la base scende (RH 0,187→0,088 mm; LH 0,214→0,087 mm; HIP_MIN_SEARCH 0,307→0,137 / 0,292→0,201 mm), mentre migliora sulle anteriori (LF 0,075→0,187 mm). Il minimo globale resta RF HIP_MAX_SEARCH (0,026→0,038 mm).
- **Header rigenerato:** diff vs committato = solo `kArtifactSha256`, `kUrdfSha256`, `kMeshManifestSha256`; tutte le righe numeriche identiche.
- **Evidenza hardware nuova:** nessuna per il contenuto.
- **Correzione minima:** aggiornare la costante `EXPECTED_URDF_SHA256` del tool (1 riga) e il pin dell'URDF degli artefatti in `static_audit.py:2326` (1 riga); rieseguire `validate` ×4 in una **nuova** cartella (le 4 del 2026-09-30 restano come storia) e `--export-header`. Vincolo: **atomico con F2** — piano e profilo devono avere gli stessi urdf/mesh hash, altrimenti il firmware risponde `REJECT_NO_SEQUENCE_PLAN` (verificato: con un solo header rigenerato 7 check del test write-policy falliscono, fail-closed).

### F2 — profilo geometrico
- **Dipendenza geometrica effettiva (rehearsal con bundle V5 completo C/W1, 12/12 q0, oracolo G4/G7 PASS):**
  - 22 endpoint su 24 identici bit a bit; parking (outcome, target), `clearance_policy` e domini dei 24 endpoint identici;
  - cambiano solo **LH HIP max** e **RH HIP min** (endpoint diagnostici *oltre* il limite URDF, margine 2,66 mrad dentro): contatto 788125→788057 µrad, clear 788057→787989 µrad; e `clear_half_span` di LH/RH HIP 788057→787989 µrad.
  - Δ = 68 µrad = **0,044 tick**: nel dominio dei tick la decisione `ticksToMicroRadMagnitude(n) ≤ half_span` è identica (513 tick entra, 514 no, con entrambi i valori).
- **Header rigenerato:** diff vs committato = bundle ID, 5 digest (urdf, mesh manifest, endpoint, parking, policy; l'allocazione è invariata) e questi 4 numeri.
- **La pipeline non gira "così com'è" su un nuovo modello:** il runner, l'oracolo G4 e il tool di policy fissano il riferimento G4 congelato su REV00 (4 costanti in 3 file: `full_runner:73`, `g4_oracle:42,45`, `safety_policy:32`; il runner si ferma subito con "G4 replay URDF SHA differs"). Serve un nuovo riferimento G4 Phase 1B (tool esistente `matdog_geometry_compiler.py`, 13 min) e l'aggiornamento di quei 4 pin (sorgenti "canonical": il loro sha cambia nel nuovo manifest; kernel e compilatore V5 non toccati). Bundle C/W1: 3886 s.
- **Evidenza hardware nuova:** nessuna per questi numeri.
- **Correzione minima (solo tecnica):** nuovo G4 → 4 pin → bundle → policy → `BUNDLE_ID` e `SAFETY_POLICY` dell'exporter → header; più il marker del bundle in `static_audit.py:3591`.

### F3 — mutation test
- Nessuna causa propria. Nel rehearsal, con F1 rigenerato e il test **invariato**: `SAFE_ACTUATOR_AUDIT_MUTATION_TESTS = PASS` (le mutazioni continuano a essere rilevate: nessun indebolimento).

## 4. Accoppiamenti nascosti emersi (non segnalati oggi dall'audit)

Rigenerando gli header, la `static_audit` del rehearsal ha mostrato che il tag del profilo è usato come **identità di calibrazione**:

1. **Tag `3713f4ddc43b204e` = FNV dei 6 digest del profilo corrente** (verificato: ricalcolato = uguale). È cablato in `StartupRecoveryReference.h` (`kStartupReferenceGeometry`), `scripts/matdog_startup_reference.py:13`, `test_calibration_record.cpp:105` + golden, pin letterale in `static_audit.py:6179`, e nei log hardware 2026-10-01/10-06 (promozione q0 12/12).
2. Con il profilo rigenerato il tag diventa `c42b4c227b6085a4`: `startupReferenceMatches()` è falso (recupero all'avvio fail-closed) e 3 check di `test_actuator_write_policy` falliscono (rehearsal).
3. **Il record NVS gen-1 porta i 6 digest e il tag.** `validateCalibrationRecord` → `PROVENANCE_MISMATCH` → classe `INCOMPATIBLE` → `RECORD_INCOMPATIBLE`: il record non viene cancellato ma non è più usabile; RESTORE gen-1 (issue #48) non disponibile; servirebbe nuova Full Calibration su hardware. È il comportamento voluto dal firmware ("cambiato l'URDF, il confronto non descrive più questo robot").
4. Precedente: `CalibrationQ0EvidenceData.h` ha già un'allocazione diversa dal profilo corrente (`574dfd6e…` vs `d9c66134…`): le evidenze congelate mantengono la loro provenienza storica e non sono state riassociate.
5. I box del recupero all'avvio (±10 tick; RF LOWER 0…367 con RF UPPER 1014…1034 e RH UPPER 388…408) sono scritti a mano; il certificato CAD che li giustifica (396 PASS, 708 box, clearance ≥3,1 mm) non ha il generatore nel repo. Controllo a campione (NON il certificato): 114 pose nei box, clearance minima per box **identica** su REV00 e REV01-FIX (LOWER 0,762 mm per la coppia RF/RH lower; UPPER 7,207 mm; REAR 12,573 mm). Nessun segnale di peggioramento, ma il certificato non è riemettibile.

## 5. Opzioni (decisione tua)

- **Opzione 1 — raccomandata: separare "base di calibrazione" da "modello operativo".** La catena resta byte-identica e dichiara la base storica REV00 (hash noti); gli input storici cambiati (URDF REV00, `base_link` REV00, `SHA256SUMS` REV00: ~1,9 MB LFS, non duplicati ma versioni diverse) vengono conservati sotto un percorso "historical" e i controlli di provenienza li verificano lì, con la stessa severità. Si aggiunge un check che verifica a macchina il delta REV00→REV01-FIX (solo 4 Z hip + base_link; assi/limiti/motorId/altre 16 mesh identici). gen-1, tag, recupero all'avvio e Q0 restano validi; nessuna nuova restrizione. Costo: modifica a exporter/tool/audit (risoluzione degli input), da rivedere; è una scelta architetturale, non un'eccezione, ma tocca l'audit: serve la tua approvazione esplicita.
- **Opzione 2 — rebinding completo (provato fattibile offline).** Seguire §3 + aggiornare i pin del tag (§4) + nuova cattura read-only q0 e SAVE gen-2 su hardware prima di qualsiasi flash. gen-1 resterebbe in flash come `INCOMPATIBLE`. Da fare solo se vuoi che la catena si sposti su REV01-FIX.
- **Opzione 3 — lineage nel firmware** (accettare record il cui tag sta in una lista compilata di predecessori attestati). Cambia un gate di sicurezza; non la raccomando adesso.

**Non unire la PR finché non scegli.** Una build con la catena rigenerata *non* va flashata senza l'Opzione 2 completa.

## 6. Cosa non è stato fatto
Nessuna modifica al repo o alla PR; nessun artefatto del rehearsal importato; nessun certificato riscritto; nessuna modifica a calibrazioni, NVS, mapping, limiti, firmware installato; kernel V5 invariato. STAND 150 mm / tabelle Motion dev5 restano lavoro separato (sessione Astra).

## Artefatti
`calibration_chain/` — `compare_seq.py`, `compare_ctl.py`, `startup_box_clearance.py`, `out/` (sequenze, header rigenerati, log static_audit del rehearsal), `rehearsal_repo/09_Logs/Validation_Reports/Geometry_Compiler/REHEARSAL_REV01FIX_*` (G4, bundle C/W1, policy).
