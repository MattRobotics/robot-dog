# Build dev.4 — offline, exit0

- Sorgente CLEAN `babb8a5a23da8f6d544cf8209c175be99cd8299e`, build `babb8a5a23da`.
- Versione `1.0.1-dev.4`, UTC `2026-10-30T11:51:14Z`.
- APP **1632704 byte**, SHA256 **`4bfcde09628506b02a2d51cb76930456758eb2dbfb7f78dc6300593d6cd0d3ba`**.
- Layout `MATDOG_16M_2x5M_NVS_V1`, partizioni SHA `8f756ecb719c4894b9c23c26bcc171e1d01ae8cda69882950944d5ce264946e7`. Slot5MiB, occupazione31,1%.
- ROBOT_POWERED; OTA ingest0; SUPERVISED_VALIDATION_CAPABLE0; BOOT_MOTION_AUTHORIZED0; autorità runtime governata. Nessuna autorizzazione hardware implicita.
- Configurazione TLS privata ABSENT.
- `207` header/C++ compilati identici al worktree tolto solo il prefisso Arduino#line1. SketchINO identico tolte le righe/prototipi generati e newline finale aggiunto. Manifest verificato exit0; `compiled-source-sha256.json`, `manifest-verify.log`, `provenance.json`. La parola VERIFIED_HARDWARE_PROFILE nell'output del verificatore indica un controllo offline sul flag del manifest, non un test hardware.
- Una sola invocazione build firmware, nessun tentativo fallito. Viewer compilato separatamente offline prima del firmware.
- Due warning esterni SCServo già presenti: parametroACC inutilizzato e indice−23 inReadMode. Non corretti né qualificati innocui; audit statico completo non eseguito.
- Report compilatore:1632555byte sketch,133084byte globali; dimensione APP file1632704byte.

Comando eseguito da `05_Firmware/MATDOG_Controller`:

```sh
MATDOG_BUILD_ROOT=/home/matteo-manicardi/MATDOG/build-output/matdog-motion-consolidation-dev4-20261030 MATDOG_PROFILE=ROBOT_POWERED MATDOG_OTA_INGEST_VALIDATION=0 MATDOG_SUPERVISED_VALIDATION=0 bash scripts/build.sh --jobs 3
```

Build/cache/output esterni ai worktree. `candidate/` contiene APP, manifest e tabella partizioni come evidenza; installazione futura soloAPP, non scrivere la tabella. ELF/map e altri prodotti restano in `/home/matteo-manicardi/MATDOG/build-output/matdog-motion-consolidation-dev4-20261030/artifacts/ROBOT_POWERED-ota0`.
