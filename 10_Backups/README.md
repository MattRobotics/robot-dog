# MATDOG — backup pubblici e recupero integrale locale

Questa cartella distingue ciò che è **recuperabile da GitHub** da ciò che esiste **solo sulla workstation dell'operatore** (Asus). Il metodo dei backup integrali cifrati della flash è in [`ESP32/README.md`](ESP32/README.md).

## Pubblico (GitHub)

| Materiale | Dove | Note |
|---|---|---|
| Backup integrali **cifrati** della flash ESP32 | Release `backup-esp32-*`: ciphertext e manifest | **Non sono firmware.** Contengono anche la NVS della loro data: servono solo per il recupero catastrofico, mai come rollback ordinario |
| Manifest e ricevute di quei backup | [`ESP32/`](ESP32/) | Senza segreti |
| APP rilasciate | Release [`v1.0.1`](https://github.com/MattRobotics/robot-dog/releases/tag/v1.0.1) e [`v1.0.2`](https://github.com/MattRobotics/robot-dog/releases/tag/v1.0.2): APP originale, manifest di build, manifest di release, `SHA256SUMS.txt` | Solo `app0`, senza NVS. Installazione e rollback: [manifest della v1.0.2](../09_Logs/Validation_Reports/Motion_1314bc4_HW_2026-11-19/RELEASE_MANIFEST.md) |

## Solo locale (Asus)

| Materiale | Percorso | Indice con gli hash |
|---|---|---|
| Immagini APP con ELF, map, manifest e sdkconfig | `~/MATDOG/backups/esp32/app_images/<commit>/`. Stable `v1.0.2`: `1314bc4`, sigillo `312fe371…`. Rollback: `30c894a`, sigillo `8a1aa6d3…` | [Manifest delle evidenze v1.0.2](../09_Logs/Validation_Reports/Motion_1314bc4_HW_2026-11-19/EVIDENCE_MANIFEST.md#c-immagini-firmware-asus) |
| Evidenze integrali delle sessioni (log seriali, registrazioni di stato, QEMU, certificati, script) | `~/MATDOG/verification-artifacts/<cartella sigillata>/` | Manifest di ogni sessione; per la linea `1.0.2`, la sezione 8 di [`EXTERNAL_ARCHIVES.md`](../09_Logs/EXTERNAL_ARCHIVES.md) |
| Seconde copie delle evidenze | `~/MATDOG/backups/verification-artifacts/session-<data>/` (tar.gz con `SHA256SUMS.txt`) | Idem |
| Bundle Git | `~/MATDOG/backups/git/*.bundle`, con `.sha256` | Bundle della `v1.0.2`: hash nel `MATDOG_v1.0.2_RELEASE_MANIFEST.txt` della Release e nel [manifest delle evidenze](../09_Logs/Validation_Reports/Motion_1314bc4_HW_2026-11-19/EVIDENCE_MANIFEST.md#d-seconda-copia-asus) |
| Kit della release `v1.0.2` (asset, note, testo della PR e dei commenti alle issue) | `~/MATDOG/backups/releases/v1.0.2/`, con `LOCAL_SHA256SUMS.txt` | Idem |
| Inventario dei branch remoti eliminati il 19/11 | `~/MATDOG/backups/git/deleted-remote-branches-20261119.txt`, con `.sha256` | [EXTERNAL_ARCHIVES §9](../09_Logs/EXTERNAL_ARCHIVES.md#9-remote-branches-deleted-on-2026-11-19) |
| Archivio di tutti i riferimenti Git locali al 19/11 (125: branch, `refs/preserve`, tag), prima della pulizia dei worktree | `~/MATDOG/backups/git/robot-dog-local-refs-archive-20261119.bundle` (storia completa), con `.sha256` e `.inventory.txt` | [EXTERNAL_ARCHIVES §10](../09_Logs/EXTERNAL_ARCHIVES.md#10-local-worktrees-and-branches-consolidated-on-2026-11-19) |
| Contenuto non committato dei 60 worktree rimossi e del checkout `github/robot-dog` (61 cartelle: patch, file, ignorati significativi) | `~/MATDOG/backups/worktrees-archive-20261119/`, con `MANIFEST.json` e `SHA256SUMS.txt` | Idem |

## Regole

- **Mai nel repository pubblico:** immagini flash in chiaro, NVS, credenziali, `*.local.h`, identità TLS o dump.
- **Stesso disco, nessuna copia indipendente.** Una seconda cartella, partizione o filesystem dello stesso dispositivo fisico non è una copia indipendente. Per gli archivi locali della linea `1.0.2` non ne è dichiarata una.
- **Nessun backup esistente si modifica, sovrascrive o cancella.** Le nuove copie vanno in nuove cartelle datate, ciascuna con il suo `SHA256SUMS`.
