# `1.0.2-dev.6` ACC 220 — manifest delle evidenze hardware (10/11/2026)

**Le evidenze sono conservate sull'ASUS e non sono copiate nel repository.** Fa eccezione la copia byte-identica del rapporto di chiusura, [`REPORT_HW_SESSION.md`](REPORT_HW_SESSION.md).

Gli SHA256 sono calcolati sui byte originali e verificati il 10/11/2026 alle 21:45Z circa. La sintesi è in [REPORT.md](REPORT.md).

## A. Cartella della sessione

`~/MATDOG/verification-artifacts/MATDOG_1_0_2_DEV6_ACC220_HW_20261110/`

| Manifest | Stato |
|---|---|
| `SHA256SUMS` (13 497 B, SHA256 `35a98771bfbc480eaca66222c76240839e5f816833a161be91c3dacd706d6515`) | 136 file, `sha256sum -c` tutti OK; copre l'intera cartella |
| `closure/SHA256SUMS_pre_chiusura_2102Z.txt` (12 976 B, SHA256 `aad1dd4c9118dd32b221909e36e4f9c2ff10bcf5ba2ae0b7af7a75d2d449905d`) | Manifesto delle 21:02Z (131 file). I log grezzi qui sotto sono invariati rispetto a esso |
| `EVIDENCE.sha256` | **Assente.** Da chiudere a cura del responsabile firmware; non è stato generato in questa integrazione |

| File | Byte | SHA256 |
|---|---:|---|
| `REPORT_HW_SESSION.md` (= la copia in questa cartella) | 16 410 | `6b55f7d522b8dc305197dcdc5041f3bd0232182aa6b3d2b407a34d0e52a9eeab` |
| `READY_REPORT.md` (runbook) | 16 999 | `f5f333de354aef6509f607d3c48aa3242236df579f10d78042edeaa68e811086` |
| `HANDOFF_JOYSTICK_MULTIDIREZIONALE.md` | 5 453 | `e825b9fee349f0acd3a1c081b85de6bc28449a7830f920bf017aecff11cbec1b` |
| `pre/offline_gates_1913.txt` | 1 175 | `66f51c87f90b37ac2f04b787bc1cd11985515a3ce7306d208013ab6d76644009` |
| `flash/flash.log` | 155 420 | `0c259d5d61e3a5434a89a5bd9954934c8a373483c58b19f9dc869e27563f9a90` |
| `h0/preflight_h0.json` | 9 403 | `1bef1910081da7e8ad96dbbf5fbc0722f40136fce380f9b5ca506384a247d0b8` |
| `usb/serial.log` | 3 255 201 | `af661bd4c2e4cd2b54142c9e063111d3ce3c1b2b9a913daf7ee290613f0085e2` |
| `usb/http.jsonl` (stato Motion a 1 Hz, `/status` ogni 5 s) | 16 293 434 | `b9355916adea91138b63617b78f817a476243c02b37ad85d4970b667197ae63c` |
| `usb/events.jsonl` (comandi USB) | 11 078 | `f72d9fc9a4875fbf2a63896e9e73ffb9e01831f626c6664e52e8e0a3c132109e` |
| `usb/summary_full_session.txt` (rigenerato alla chiusura) | 19 102 | `133d4ba61ef2a6288e997b076a2c480d7ae21c84a57a5348fa83243664517fb9` |
| `terra/dashboard_trace_47.json` (19:47:35–19:48:58Z) | 2 557 974 | `f4f7601f1891e38e2584ee1330c0bce74a0f990118580d3fafdfa553025bf314` |
| `terra/dashboard_trace_48.json` (19:57:23–19:58:47Z) | 2 563 282 | `8109a49aef2da12906c21de0ccdc3b36a375c2b9bcf977dea6f8a9c55fc0ebf8` |
| `terra/hip_margins_dashboard_traces.txt` | 8 000 | `979249fa68f8726e790c83aa8c09f36fd6b4d90a7c5fb0f9cc49dc3d08db74d0` |
| `terra/summary_ground.txt` (rigenerato alla chiusura) | 10 289 | `f97a4cc84ac0d3f380b5bd422f38f7886dfa8fcf9d08e84a47e209c10d16d186` |
| `closure/offline_gates_post_session.txt` (20/20 PASS) | 1 175 | `66f51c87f90b37ac2f04b787bc1cd11985515a3ce7306d208013ab6d76644009` |
| `closure/integrity_checks.txt` | 2 221 | `61a07f9a1a4a1f4e53d39dcddf91dffe737411dc27cae235a6647a1dda98b7ed` |
| `closure/summary_fixture_q1_q7.txt` | 12 457 | `9bda1c5e719740c9ac5eace5da64197d7e3c6e2ca4cd9191a9230112bade22ef` |

La cartella `rehearsal/` contiene la prova generale contro un dispositivo **simulato**, con la stessa identità di build. Non è un'evidenza hardware.

## B. Build e flash

| Elemento | Percorso sull'ASUS | SHA256 |
|---|---|---|
| APP flashata | `~/MATDOG/build-output/gait-redesign-20261108/build-450d2c7/artifacts/ROBOT_POWERED-ota0/MATDOG_Controller.ino.bin` (1 751 040 B) | `82122e06b6708a9d687f4525f612182c227134f3abcac27f32ed7e266be04256` |
| Manifest della build | `…/ROBOT_POWERED-ota0/matdog_build_manifest.txt` (`SOURCE_COMMIT=450d2c793dcb76d08ea341eb5c4b1e50215cee40`, `BUILD_ID=450d2c793dcb`, `SOURCE_STATE=CLEAN`, `GIT_DIRTY=0`, `LAYOUT_ID=MATDOG_16M_2x5M_NVS_V1`, tabella delle partizioni `8f756ecb…`) | `df74c7ba613aa0c5bf7f9da7a04ff54c7ee923606d196981d41aaa6bd9baa31c` |
| Script di flash qualificato | `~/MATDOG/verification-artifacts/MATDOG_1_0_2_DEV6_ACC220_OPERATIONAL_20261110/flash/flash-qualified-app0-1.0.2-dev.6-450d2c7.sh` (rifiuta senza `MATDOG_DEV6_ACC220_FLASH_AUTHORIZED`) | `41df0d6af4ab9fe534bcae0c386fd8b772b27697e01a36f19b2fee0afc45c7b3` |
| Asset di rollback | `…/MATDOG_1_0_2_DEV6_ACC220_OPERATIONAL_20261110/flash/rollback-assets.sha256` (12/12 OK) | `139224e591f3aa6c1b304bb77c3e418abef125103eba6f6cd10e039a8b60a61e` |
| Cartella della build operativa | `~/MATDOG/verification-artifacts/MATDOG_1_0_2_DEV6_ACC220_OPERATIONAL_20261110/` (audit, piano HW, regressioni) | `SHA256SUMS` 58/58 OK |

**Controllo privacy della copia:** il rapporto copiato non contiene indirizzi IP, MAC, SSID, password, token né credenziali.
