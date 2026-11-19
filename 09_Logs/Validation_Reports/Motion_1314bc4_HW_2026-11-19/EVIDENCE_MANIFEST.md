# Stable `v1.0.2` — manifest delle evidenze (19/11/2026)

**Tre principi:**
- Gli SHA256 sono calcolati sui **byte originali**, senza modifiche.
- Gli originali integrali (log seriali, JSONL di stato, QEMU, certificati, script di qualifica, binari) restano **solo sull'Asus**.
- Nel repository ci sono i rapporti, le analisi testuali e i sigilli che elencano ogni file locale con il suo hash. **Un file elencato solo in un sigillo locale non si recupera dal repository pubblico.**

## A. Copie pubblicate in questa cartella

Sono 29 file, tutti **byte-identici** alla voce del rispettivo sigillo locale (verificato il 19/11).

| Cartella del repository | Originale sull'Asus (`~/MATDOG/verification-artifacts/`) | Sigillo |
|---|---|---|
| `qualification/` (rapporto, 10 analisi) | `MATDOG_1_0_2_MOTION_1314BC4_HW_20261119/` | `SHA256SUMS.txt` `3dbec5d3…`, 416 file → `qualification/SHA256SUMS.local.txt` |
| `completion/` (rapporto, piano, 2 selettori, `runs.log`, 8 analisi) | `MATDOG_1_0_2_MOTION_1314BC4_HW_BLOCKD_S9_20261119/` | `SHA256SUMS.txt` `9850a66a…`, 146 file → `completion/SHA256SUMS.local.txt` |
| [`../BMS_DALY_B2_2026-11-19/`](../BMS_DALY_B2_2026-11-19/README.md) | `MATDOG_BMS_DALY_QON_20261119/` | `SHA256SUMS.txt` `1c8303fc…`, 8 file |
| [`../Motion_30c894a_HW_2026-11-18/`](../Motion_30c894a_HW_2026-11-18/README.md) | `MATDOG_1_0_2_ACQV2_30C894A_HW_20261118/` | `SHA256SUMS.txt` `2ca75b3e…`, 604 file |

**Controllo privacy** prima della pubblicazione: nessun percorso dell'host, nome dell'adattatore Wi-Fi, SSID dell'host, password, token o cookie nei file pubblicati.
- Escluso `tools_used.sha256` della sessione B2, perché contiene percorsi assoluti dell'host. Il suo hash `f084df6a…` è nel sigillo locale.
- I log seriali e le registrazioni di stato non sono pubblicati.

## B. Originali locali (Asus): sigilli verificati il 19/11

**Esito:** 29 cartelle sigillate del 17–19/11 su 29 risultano integre a `sha256sum -c` (elenco completo in [`LOCAL_SEALS_VERIFIED_20261119.txt`](LOCAL_SEALS_VERIFIED_20261119.txt)).

| Cartella | Contenuto | SHA256 del sigillo |
|---|---|---|
| `MATDOG_1_0_2_MOTION_INTEGRATED_1314BC4_20261119/` | Build CLEAN, pacchetto APP-only, gate, QEMU, regressioni, procedura HW | `a9355b23…` (704 file) |
| `MATDOG_1_0_2_MOTION_1314BC4_HW_20261119/` | Flash, boot, catena A/B, D1/D2, analisi | `3dbec5d3…` (416) |
| `MATDOG_1_0_2_MOTION_1314BC4_HW_BLOCKD_S9_20261119/` | Blocco D, S9, chiusura | `9850a66a…` (146) |
| `MATDOG_BMS_DALY_QON_20261119/` | Avvio a freddo B2 di `30c894a` (Fable) | `1c8303fc…` (8) |
| `MATDOG_1_0_2_MOTION_FP32_20261118/` | Candidato FP32, QEMU, `random_3` | `31d79d73…` (3 212) |
| `MATDOG_1_0_2_P0_TOLERANCE_20261118/` | Revisione tolleranze, P1 | `8f3780e0…` (12 161) |
| `MATDOG_1_0_2_ACQV2_CLEAN_20261118/` | Pacchetto `30c894a` e script di rollback | `eb95181b…` (88) |
| `MATDOG_1_0_2_ACQV2_30C894A_HW_20261118/` | Qualifica HW di `30c894a` | `2ca75b3e…` (604) |
| `MATDOG_1_0_2_OPCAND_85BAD0E_HW_20261118/` | Qualifica HW di `85bad0e` | `13d8d88d…` (466, file `SHA256SUMS_20261118T0732Z.txt`) |
| `MATDOG_BMS_DALY_AB_FABLE_20261118/` | A/B DALY | `3ab9f106…` (63) |
| Altre 19 cartelle del 17–18/11 (campagna P0, HTTP A/B, RCA DALY) | Elencate con il loro sigillo in `LOCAL_SEALS_VERIFIED_20261119.txt` | — |

**Una sola cartella senza sigillo:** `MATDOG_1_0_2_SESSION_20261118_SUMMARY/` (riepilogo della sessione del 18/11). Non è stata modificata; hash dei suoi due file:
- `SESSION_SUMMARY.md` `0afe418c…`;
- `SESSION_SUMMARY_ADDENDUM.md` `e3fd8449…`.

## C. Immagini firmware (Asus)

| File | SHA256 |
|---|---|
| `~/MATDOG/backups/esp32/app_images/1314bc4/MATDOG_Controller.ino.bin` (APP Stable) | `dd16bd6215d836f8dae6389df7b13b8fcdcff20bdec06ca9948a34ae9b0dd82a` |
| `…/1314bc4/matdog_build_manifest.txt` | `be4667340b1dfe31136afde3321a65f8b1fb614de5a34a5377523f19ed397c46` |
| `…/1314bc4/MATDOG_Controller.ino.elf` | `126cf6b08d97c9e41eb75b9c65af4f757919529f05081c4280dad74a89dee847` |
| `…/1314bc4/MATDOG_Controller.ino.map` | `b02e213b2f9a8adcf32680e1ae6b07eb643cf61cfa63b81d566b849fa293f72c` |
| `…/1314bc4/sdkconfig` | `534f3457aaf3ffbc82c45dc21d6b3ac57388c82d0d9a24bd2b8e6cadc3e48d9d` |
| `…/1314bc4/SHA256SUMS` (sigillo della cartella) | `312fe37189bfbaf6a8c5e07f66e5513780c8d72e220baf7a7d83535ae64116f7` |
| `~/MATDOG/backups/esp32/app_images/30c894a/MATDOG_Controller.ino.bin` (rollback) | `03c863cc433622f672c11cae45f46694c4aa6f78ba0136cc1b7bc818a2e4b340` |
| `…/30c894a/SHA256SUMS` | `8a1aa6d36217bf10a2ea9d3d66dd17588aee4887d264d870e406a81ed124712c` |

**Script di flash qualificati:**
- `flash-qualified-app0-1.0.2-dev.7-1314bc4.sh` `d24920b0ebe216a8b5fb4aaeacae795a949852d66fee15d879d4e40871ef388e`;
- gate offline `9b6fa15e7b55f84437587b53a20342954283f68d9cb0388df834d00341debfbf`;
- rollback `flash-qualified-app0-1.0.2-dev.7-30c894a.sh` `812a2e830225c4513016f959e1e035df7e5f440e53e8142c68db9e800e4c643b`.

## D. Seconda copia (Asus)

Cartella `~/MATDOG/backups/verification-artifacts/session-20261119/`, verificata il 19/11 con `gzip -t`, `tar -t` e `sha256sum -c`. Sigillo `SHA256SUMS.txt` `9ce3947a2ba2ee08b35ae21c3dbcd71b7826dfde737085d4ce564fb0c4c021c3`.

| Archivio | SHA256 |
|---|---|
| `MATDOG_1_0_2_MOTION_INTEGRATED_1314BC4_20261119.tar.gz` | `fe2f655351b98629071fdfa2a6c4ac3a912bbf6993fd9177e8a66b1c3719bceb` |
| `MATDOG_1_0_2_MOTION_1314BC4_HW_20261119.tar.gz` | `ae0cb802992acff2e1a48a1aed1eed0a907a24e2e125eb79e7cd59bd92a6e7af` |
| `MATDOG_1_0_2_MOTION_1314BC4_HW_BLOCKD_S9_20261119.tar.gz` | `953b8a5cbc0f01373aab82f402b9ab2883f4e2e7f2d576714664d1b9029ac86e` |
| `MATDOG_BMS_DALY_QON_20261119.tar.gz` | `ecd5499b2df09702524d6cd1a73a18f2f8dc125100b9fed378ea86dcbf967ef6` |
| `MATDOG_firmware_1314bc4_artifacts.tar.gz` (APP, ELF, map, manifest, sdkconfig) | `ad68dfd6134c134dd91602d8b5e93ead140b1cc4258975b4289200c2b08bc5f7` |

**Sessione del 18/11:** archivi e bundle sono in [`../Motion_FP32_2026-11-18/LOCAL_ARCHIVES.sha256.txt`](../Motion_FP32_2026-11-18/LOCAL_ARCHIVES.sha256.txt). Il bundle completo `robot-dog-motion-candidates-20261118-full.bundle` (`50dba781…`) è stato riverificato il 19/11.

**Bundle Git della Stable:** `~/MATDOG/backups/git/robot-dog-stable-v1.0.2-20261119.bundle`, 43 031 153 B, SHA256 `3f800b97b70b7967ca2713895700fd84e6f3d51a5e5c407e13b33e51525a15bd`, `git bundle verify` okay.
- Contiene `main` al merge (`052d52e`), i tag `v1.0.1` e `v1.0.2` e le punte dei sette branch eliminati dal remoto il 19/11.
- Inventario dei branch: `~/MATDOG/backups/git/deleted-remote-branches-20261119.txt`, SHA256 `074d821cd79ccf6c14869b64ccbc80109cccfd5e30b8b3eb84b46660f573ebfc`.

**Kit della release:** `~/MATDOG/backups/releases/v1.0.2/` (asset, note, testo della PR e dei commenti alle issue). Sigillo `LOCAL_SHA256SUMS.txt` `33870b443326c51b16346f6f616d768429543ced20e51b6c0b88b123139ef051`.

**Limite:** originali e seconda copia stanno sullo **stesso disco** dell'Asus. Non è dichiarata una copia fisica indipendente; i sottoinsiemi pubblicati sono anche su GitHub.

## E. Che cosa non è nel Git e perché

- **Log seriali, JSONL di stato a 1–2 Hz, console degli strumenti, registri di processo:** decine di MB per sessione, con percorsi e identificativi dell'host. I sigilli ne pubblicano gli hash.
- **ELF, map e cartelle QEMU:** decine o centinaia di MB, ricostruibili dal sorgente solo in modo non bit-identico.
- **Copie degli strumenti HW** usate il 19/11 (`hw_session/`, 30 file): identiche, con `cmp`, a quelle del pacchetto `MATDOG_1_0_2_MOTION_INTEGRATED_1314BC4_20261119/hw_session/`. Contengono percorsi dell'host, per cui restano locali con il loro hash nei sigilli. La versione pubblica precedente, del 16/11, è in [`timing_tools/`](../Unified_Operational_2026-11-14/timing_tools/README.md). Gli strumenti nuovi del 19/11 sono invece pubblicati:
  - i due selettori di passi, in `completion/`;
  - gli script di analisi, in sola lettura: `completion/analysis/steer_check.py` e `qualification/analysis/sections_from_ab_tables.py`.
