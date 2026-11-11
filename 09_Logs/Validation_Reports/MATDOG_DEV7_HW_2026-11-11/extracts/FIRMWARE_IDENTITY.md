# Firmware installato sul robot a fine sessione (11/11/2026, 19:55Z)

| Voce | Valore |
|---|---|
| Versione | `1.0.2-dev.7` |
| Commit sorgente | `b5610620d0cef40cdc34bd62bf6256857bc9cfde` (branch `integ/motion-vnext-battery-p1-dev7-20261111`) |
| BUILD_ID | `b5610620d0ce` (SOURCE_STATE CLEAN, GIT_DIRTY 0) |
| APP | `MATDOG_Controller.ino.bin`, 1 757 472 B, SHA256 `28744982e293d5a56a0246c7ccacd083d1a0040f70ece29da4ac72fbfe6053d4` |
| Profilo / slot | `ROBOT_POWERED`, app0 `0x010000` (scrittura `0x010000..0x1bd120`) |
| Tabella partizioni | SHA256 `8f756ecb719c4894b9c23c26bcc171e1d01ae8cda69882950944d5ce264946e7` (invariata) |
| Prova dell'installazione | `../flash/flash.log`: hash dati verificato, `verify-flash` digest matched, `APPLICATION_ONLY_FLASH = PASS`; banner al boot `1.0.2-dev.7` / `b5610620d0ce` (`../usb/serial.log`, `../h0/`) |

Commit successivi sullo stesso branch che toccano solo `09_Logs/` (documentazione) **non** cambiano il firmware installato: l'identità è il commit `b561062` e lo SHA256 dell'APP qui sopra.

## Copie conservate qui

- `installed_b561062/`: APP, `matdog_build_manifest.txt`, `partitions.bin`, `bootloader.bin` (solo identità, mai scritti), `MATDOG_Controller.ino.elf` (simboli per decodificare eventuali backtrace). Originali in `~/MATDOG/build-output/integrated-dev7-20261111/build-b561062/artifacts/ROBOT_POWERED-ota0/`.
- `rollback_450d2c7/`: APP `1.0.2-dev.6` / `450d2c793dcb`, SHA256 `82122e06b6708a9d687f4525f612182c227134f3abcac27f32ed7e266be04256`, suo manifest e script APP-only `flash-qualified-app0-1.0.2-dev.6-450d2c7.sh` (variabile `MATDOG_DEV6_ACC220_FLASH_AUTHORIZED=YES_ROBOT_SUPPORTED`; SCRIPT_DIR = worktree `matdog-acc85-runtime-proposal`).
- Script APP-only della dev.7: `../flash/flash-qualified-app0-1.0.2-dev.7-b561062.sh`.

Non copiati di proposito: `merged.bin` da 16 MB (mai usarlo: sovrascrive l'NVS), `.map`, `sdkconfig`.

## Backup dell'intera flash (non copiato qui: contiene l'NVS)

`~/MATDOG/backups/esp32/matdog_esp32s3_fullflash_2026-09-10.bin`, 16 777 216 B, SHA256 `5cbba0b9c5500d0c95247b9b7e7173a29f934b8b13f6800cc9f583374d67fd32` (verificato l'11/11 sera). Solo per ripristino di emergenza.

Nessuna scrittura di firmware, NVS, EEPROM servo, calibrazioni o partizioni dopo il flash APP delle 18:51Z.
