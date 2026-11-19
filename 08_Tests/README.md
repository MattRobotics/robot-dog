# MATDOG Tests — punto d'accesso ai test attivi

**Che cosa c'è qui:** questa cartella non contiene gli script. Indica, per ogni categoria, **dove si trova lo script reale** e **dove finiscono le evidenze**.

**Perché gli script non sono qui:** stanno vicino al codice che provano. Spostarli romperebbe percorsi, import e comandi qualificati, per esempio i pin di sorgente di `static_audit.py` e gli script di flash.

**Regole:**
- Un test passato vale nel perimetro e alla data registrati nel suo rapporto.
- Un risultato offline non è un risultato hardware.

Stato corrente: Stable `v1.0.2` ([qualifica](../09_Logs/Validation_Reports/Motion_1314bc4_HW_2026-11-19/README.md)).

## Dove sono i test

Percorsi relativi alla radice del repository. `FW` = `05_Firmware/MATDOG_Controller`.

| Categoria | Script reale | Note per l'esecuzione |
|---|---|---|
| Test host del firmware (unità e integrazione) | `FW/scripts/tests/run_host_tests.sh`, più le singole suite `FW/scripts/tests/run_*` | `bash scripts/tests/run_host_tests.sh` dalla cartella `FW`. Dura ≈ 2,5 min; il file non è eseguibile |
| Suite Motion mirate | `FW/scripts/tests/run_simple_motion_tests.sh`, `run_stop_support_tests.sh`, `run_prone_tests.sh`, `run_calibration_restore_tests.sh` | `bash …` |
| Audit statico (pin di sorgente, confini, regole) | `FW/scripts/static_audit.py` | `python3 scripts/static_audit.py "$(pwd)"` con il percorso **assoluto** dello sketch. Dura ≈ 6 min |
| Gate offline della legge Motion | `FW/scripts/offline_motion/`: `run_stop_secant_gate.sh`, `run_joystick_envelope.sh`, `run_perturbation_matrix.sh`, `run_stop_matrix.sh`, `run_walk_support_check.sh`, `run_walk_tilt_envelope_check.sh`, `run_motion_frame_digest.sh` | Banchi host deterministici |
| Banco QEMU ESP32-S3 | `FW/scripts/offline_motion/run_qemu_motion.sh`, `FW/scripts/offline_motion/qemu_motion/` | Conteggi CCOUNT, **non** tempi hardware |
| Certificati CAD e collisioni (FCL) | `FW/scripts/offline_motion/certify_steer_box.py` | Richiede l'ambiente del certificatore (sull'Asus) |
| Dashboard (DOM) | `FW/scripts/tests/run_simple_dashboard_tests.sh`, `test_simple_dashboard_dom.cjs`, `test_dashboard_layout_browser.cjs`, `test_portal_dom.cjs` | Serve jsdom |
| Gemello MuJoCo (offline) | `06_Software/Matdog_MuJoCo/` (script `m1`…`m6`, [README](../06_Software/Matdog_MuJoCo/README.md)) | Simulazione: **non** è una qualifica hardware |
| Collaudo hardware sulla fixture | Strumenti pubblicati del 16/11: [`timing_tools/`](../09_Logs/Validation_Reports/Unified_Operational_2026-11-14/timing_tools/README.md). Le copie qualificate per la `v1.0.2` restano sull'Asus, con i loro hash nei sigilli; i selettori del 19/11 sono in [`completion/`](../09_Logs/Validation_Reports/Motion_1314bc4_HW_2026-11-19/completion/) | Solo con mandato e operatore presente. Al massimo 2 client HTTP, controllo M0, processi da `session_proc.py` ([ADR-007](../09_Logs/Architecture_Decisions/ADR-007_Motion_Timing_Integration_and_Heap_Margin.md)) |
| Flash APP-only e verifica della partizione | `FW/scripts/flash_app_only.sh`, `FW/scripts/verify_application_partition.py` | Solo con mandato esplicito. Non è un test: serve all'installazione verificata |

## Dove finiscono le evidenze

| Tipo | Dove |
|---|---|
| Rapporti, analisi testuali, manifest e sigilli | `09_Logs/Validation_Reports/<Argomento>_<data>/` ([indice](../09_Logs/Validation_Reports/README.md)) |
| Originali integrali (log seriali, registrazioni di stato, QEMU, binari) | Sull'Asus, `~/MATDOG/verification-artifacts/<cartella sigillata>/`. Percorsi e hash sono nei manifest delle sessioni ([v1.0.2](../09_Logs/Validation_Reports/Motion_1314bc4_HW_2026-11-19/EVIDENCE_MANIFEST.md)) |
| Foto, video, screenshot, rendering | [`07_Media/`](../07_Media/README.md), collegati dal rapporto pertinente senza duplicarli |
| Decisioni | [`09_Logs/Architecture_Decisions/`](../09_Logs/Architecture_Decisions/README.md) |

## Storico (giugno 2026)

Prima base di ingegneria validata: URDF REV00 ([rapporto](../09_Logs/Validation_Reports/2026-06-30_URDF_REV00_Kinematic_Validation.md)).

Ordine di sviluppo dei test previsto allora: struttura URDF → zero meccanico → conversione encoder-radianti → moto di un giunto → FK e IK di una zampa → posa STAND → traiettoria del gait → locomozione hardware.

L'elenco originale delle categorie comprendeva anche i test di compatibilità con la Station, oggi storica. Resta come riferimento del metodo, non come stato corrente.
