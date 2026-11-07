# Dev.17 — manifest delle evidenze hardware (07/11/2026)

Tutti gli SHA256 sono calcolati sui **byte originali**, senza modifiche. Report: [REPORT.md](REPORT.md). Manifest firmware/release: [RELEASE_MANIFEST.md](RELEASE_MANIFEST.md).

## A. Sessione operatore — archivio fornito nella conversazione

[`raw/MATDOG_DEV17_HARDWARE_VALIDATION_2026-11-07.zip`](raw/MATDOG_DEV17_HARDWARE_VALIDATION_2026-11-07.zip): **444.642 byte**, SHA256 **`b1edd6cf49bf6d50c62ee4bbefd1b007249e6a133ee0bafd37ddbf9b661565d3`** (uguale al valore atteso dichiarato dall'operatore). `unzip -t` senza errori, 12 elementi; `SHA256SUMS.txt` interno **11/11 OK**. Pubblicato byte-identico; i JSON restano solo dentro lo ZIP (convenzione di dev.16).

| File nell'archivio | Byte | SHA256 |
|---|---:|---|
| `raw/matdog-motion-trace (33).json` | 1.432.982 | `61bd2c3105b8313bf12551758b255e91975b87cf442fdb36641533218e29ae0e` |
| `raw/matdog-motion-trace (34).json` | 2.337.823 | `8f486608699eff662376dc6916ed5d00d6205d225fe06f907fed46ba7a0dc8e0` |
| `raw/matdog-motion-trace (35).json` | 2.447.495 | `9ad95474d0033d8179ea0edd4fc5897407ac9dea63b110a7e771d6b845e9abbb` |
| `raw/matdog-motion-trace (36).json` | 2.513.388 | `6ee4f8492ccb30aecf3682ab9596abae833bf49dc222ec9789cae6a0f2001b44` |
| `screenshots/image(20261107-184722).png` | 70.288 | `007e2daa9cf77c28b9555ed0d4e320a6fcdd94174cebf65153e7a33141f2c27b` |
| `screenshots/image(20261107-185216).png` | 58.031 | `c69234c051fc56336eefa3044feaba9bd72fd211d0a17159d00980b2b7e1cef2` |
| `TRACE_METRICS.csv` (pubblicato anche in chiaro, identico) | 1.205 | `d54c1829fc83807c2e6a5ab7ff73269ec687c9d72eb43e132007a59236d69cae` |
| `COMMANDS_RETAINED.csv` (pubblicato anche in chiaro, identico) | 5.973 | `6d4ec7b419161e1703646ceefbb697f72b4e275ab59d7cadcb77e4ef7f984ba5` |
| `OPERATOR_OBSERVATIONS.md` (pubblicato anche in chiaro, identico) | 1.562 | `7beceea8a38e3ba34d5e139ed0f72207ae8c510d37085852e4219543ca4ae1c3` |
| `README.md` / `REPORT.md` (derivati dell'operatore, inglese) | 3.915 / 5.354 | `abb682c8…e012` / `630bbd9a…6c43` |

Controllo privacy prima della pubblicazione: nessuna password, SSID, indirizzo IP, email, user agent, token, cookie o CSRF; `admin_session` è solo un booleano; i due PNG mostrano solo la dashboard e non contengono metadati (solo `IHDR`/`sBIT`/`IDAT`/`IEND`). Le metriche di `TRACE_METRICS.csv` sono state ricalcolate dai JSON originali e coincidono (perse/overrun, lavoro max, stack minimo, stato finale).

## B. Flash APP-only, boot e audit IDLE — originali ASUS

Cartella originale: `~/MATDOG/verification-artifacts/MATDOG_DEV17_FLASH_PREP_20261107/`, `EVIDENCE.sha256` **26/26 OK** al momento della promozione. Copia byte-identica dei 24 registri che non contengono percorsi locali né identificativi dell'host: [`raw/MATDOG_DEV17_FLASH_BOOT_IDLE_2026-11-07.zip`](raw/MATDOG_DEV17_FLASH_BOOT_IDLE_2026-11-07.zip), **18.869 byte**, SHA256 **`25f63459923906e51c073ec89679e1c24ac2af44051e411141546f09651e3393`** (include lo stesso `EVIDENCE.sha256` per la verifica incrociata).

| Registro (nello ZIP) | Byte | SHA256 |
|---|---:|---|
| `preflight-status-1.txt` (dev.16 prima del flash) | 1.898 | `8f6fe78e4c866451e68aae605099c5b7a70d408c1aff2b0138062652507b61f2` |
| `preflight-motion-final.json` | 4.658 | `12dd51d5c8846edeb0c96b0788d0849f0ea944287b5a9c8a4de79fef0abc2cf5` |
| `device-layout.log` (layout/otadata in sola lettura) | 409 | `4725f00c42a0d874547b740416a0afb381e444c946bbc149712aeba35f207c2f` |
| `boot-status.txt` (primo `/status` dev.17) | 1.895 | `817512c577e40f78ad34513a8f2878b2c093353703d55b9b8496154f6e45af35` |
| `boot-motion.json` | 4.717 | `000285ee9ff96f068d87a6a91514571b9f0b4ee77b895285a14e4f6fceb71224` |
| `idle-audit.jsonl` (160 richieste) | 135.315 | `0061dd5263ea627e0c9aa2abe7960f8fd04e0121f8a780a6e53a986c2f2e3cdd` |
| `idle-audit-summary.txt` | 1.307 | `7011e501b7b8818f423252a66716590292c26e2d90652084aabfdee7601a5bef` |

**Conservati solo sull'ASUS** (contengono percorsi locali o il nome dell'adattatore Wi-Fi dell'host; SHA in `EVIDENCE.sha256` o qui):

| File | SHA256 |
|---|---|
| `flash.log` (151.700 B, output completo di esptool) | `a344c655081e050ac338f7309c898c330fe2992b8f2cd0b8ca5db8258318ef2e` |
| `preflight-assets.txt` | `65702f95ebfd57a7fe8d60505c066c061cab45042e45b741a0df4a4a11231cb9` |
| `session-notes.md` | `3861078ac3d7caebe3bf2a83d8d7b2c1e883c263f2cbbf9200dbeb4a2f45752c` |
| `flash-qualified-app0-dev17.sh` (script eseguito) | `57477b2b2fe39639abce44ab06a35d4727530e8f9f51e68bd1379cacaaae78ed` |
| `rollback-assets.sha256` | `78cc471e9b432461a5f09358e07357413af09ceada651c32fc56a32ed8feb52e` |

Righe di `flash.log` copiate verbatim (unica sostituzione: `/home/<utente>` → `~`):

```
APPLICATION_SHA256    = 14e055a0bfa2adcab8f87df29ece0a0cf01a6e9e13b99aee1c94bda59ba84a83
APPLICATION_OFFSET    = 0x010000 (partition 'app0')
APPLICATION_SIZE      = 1707808
PARTITION_TABLE_SHA256 = 8f756ecb719c4894b9c23c26bcc171e1d01ae8cda69882950944d5ce264946e7 (build manifest == table installed on the device)
WRITE_RANGE           = 0x010000 .. 0x1b0f20 (partition 'app0')
ERASE_RANGE           = 0x010000 .. 0x1b1000 (sector rounded; MATDOG NVS 0xfe0000..0xff0000 untouched)
SOURCE_COMMIT         = 17db0efe23242ae8932b5549546a9758d9aa4645
Writing application partition only (no bootloader/partition-table/boot_app0/NVS)...
Wrote 1707808 bytes at 0x00010000 in 38.3 seconds (356.3 kbit/s).
Hash of data verified.
Verifying 0x1a0f20 (1707808) bytes at 0x00010000 in flash against '~/MATDOG/build-output/dev17-stabilization-20261107/build-17db0ef/artifacts/ROBOT_POWERED-ota0/MATDOG_Controller.ino.bin'...
Verification successful (digest matched).
APPLICATION_ONLY_FLASH = PASS
```

Il backup completo del 10/09/2026 (SHA `5cbba0b9…fd32`) e l'APP dev.16 restano sull'ASUS, verificati 3/3 con la candidata: non sono duplicati qui. Il backup contiene firmware e NVS di quella data: serve solo per un ripristino catastrofico.

## C. Regola di lettura

Le tracce sono buffer finiti (ultimi 240 campioni, ultimi 48 comandi) e in parte sovrapposti: non sommare finestre diverse, non contare due volte lo stesso `TIMEOUT_3500MS`, non attribuire un ACK a comandi usciti dal buffer. `VERIFIED HARDWARE` vale solo per ciò che è stato osservato o registrato in questa sessione, su questo robot.

## D. Screenshot pubblicati in chiaro (aggiunta 07/11/2026, dopo la release)

Estratti con `unzip -p` dallo ZIP dell'operatore, **senza modificarne i byte** e con nome descrittivo; gli originali restano nello ZIP e il suo `SHA256SUMS.txt` è invariato.

| File pubblicato | Originale nello ZIP | Byte | SHA256 |
|---|---|---:|---|
| [`screenshots/matdog-motion-cockpit-ready-idle-twin-viewcube-2026-11-07.png`](screenshots/matdog-motion-cockpit-ready-idle-twin-viewcube-2026-11-07.png) | `screenshots/image(20261107-184722).png` | 70.288 | `007e2daa9cf77c28b9555ed0d4e320a6fcdd94174cebf65153e7a33141f2c27b` |
| [`screenshots/matdog-motion-cockpit-stand100-reached-twin-front-2026-11-07.png`](screenshots/matdog-motion-cockpit-stand100-reached-twin-front-2026-11-07.png) | `screenshots/image(20261107-185216).png` | 58.031 | `c69234c051fc56336eefa3044feaba9bd72fd211d0a17159d00980b2b7e1cef2` |

Contenuto: solo la dashboard Motion Cockpit (stato, Twin, ViewCube, comandi, STOP e l'etichetta «Amministratore connesso»); nessun indirizzo, nome utente, URL, SSID o dato personale; nessun metadato PNG oltre a `IHDR`/`sBIT`/`IDAT`/`IEND`.
