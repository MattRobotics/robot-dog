# Sessione hardware `1314bc411c58`: flash APP-only e qualifica della release Motion integrata (preparata il 19/11/2026, NON eseguita)

Deriva dalla procedura eseguita il 18/11 su `30c894a` (`../../MATDOG_1_0_2_ACQV2_CLEAN_20261118/hw_session/PROCEDURE.md`).

**Cosa resta uguale:** strumenti, fixture, catena e criteri.

**Cosa cambia:**
- le pin di identità e di rollback, in `rom_entry.sh`, `rom_checks.sh`, `rom_checks_before_rollback.sh`, `fixture_suite.py` e `cand_vs_ref.py`;
- il riferimento di confronto, che diventa la catena `1148` del 18/11 su `30c894a` (`MATDOG_REF=1811b`, default).

I criteri sono fissati prima della prova e non si modificano dopo.

**Cartella di lavoro:** copiare questa cartella e lavorare solo nella copia.

```bash
cp -a ~/MATDOG/verification-artifacts/MATDOG_1_0_2_MOTION_INTEGRATED_1314BC4_20261119/hw_session ~/MATDOG/verification-artifacts/MATDOG_1_0_2_MOTION_1314BC4_HW_<AAAAMMGG>
```

## 0. Prerequisiti

**Controllo DALY a freddo di Fable:**
- deve essere concluso e valutato dall'operatore;
- il candidato contiene il firmware BMS di `30c894a` invariato;
- se l'esito impone un'azione sul BMS, questa sessione si rimanda.

**Autorizzazione al flash:** esplicita, nella sessione attiva, per questo firmware. Quella del flash A/B BMS non vale.

**Robot:** sulla fixture, PRONE, zampe libere, STOP raggiungibile, operatore presente.

**Immagine installata:** letta prima di tutto (S2). Il 18/11 sera era `30c894a`; se oggi è diversa, ci si ferma e si decide con l'operatore.

## 1. Identità

| Oggetto | Valore |
|---|---|
| Installato atteso (rollback) | `30c894a0d430` (`1.0.2-dev.7`), APP `03c863cc433622f672c11cae45f46694c4aa6f78ba0136cc1b7bc818a2e4b340`, 1 779 312 B; script `flash-qualified-app0-1.0.2-dev.7-30c894a.sh` (`812a2e83…`), usato sul robot il 18/11 (Fable, digest corrispondente) |
| Candidato | `1314bc411c589c4216e95629f4665d9d01f05781` (branch `cand/ksteer-slow-recert-20261118`), `BUILD_ID=1314bc411c58`, `SOURCE_STATE=CLEAN` |
| APP del candidato | `dd16bd6215d836f8dae6389df7b13b8fcdcff20bdec06ca9948a34ae9b0dd82a`, 1 777 952 B, app0 `0x010000` |
| Scrittura | Sola app0 `0x010000..0x1c2120`, cancellazione fino a `0x1c3000`; bootloader, tabella delle partizioni (`8f756ecb…`) e sdkconfig identici a `30c894a`; NVS (`0xfe0000..0xff0000`) mai toccate |

## 2. Sequenza

**S0, robot spento:**
```bash
python3 -I session_proc.py check --expect-none
bash ../preflash/flash-qualified-app0-1.0.2-dev.7-1314bc4_DRY_GATES_OFFLINE.sh | tail -1
bash qon_recorder.sh qon 7
```

**S1, operatore:**
- KEY ON (alimenta insieme MCU e rail servo);
- dongle sull'AP;
- gate DALY: QON + 6 min e 150 poll validi.

**S2, preflight dell'installato:**
```bash
EXPECT_FW=1.0.2-dev.7 EXPECT_BUILD=30c894a0d430 python3 -I preflight.py pre/preflight_30c894a.json
```
Poi `session_proc.py stop qon_usb` e `check --expect-none`.

**S3, flash unico** (PRONE_REST, SAFE_OFF 8191, coppie OFF):
```bash
export MATDOG_INTEG_1314BC4_FLASH_AUTHORIZED=YES_ROBOT_SUPPORTED
bash rom_entry.sh && bash rom_checks.sh      # verifica che l'installato sia esattamente 30c894a
bash ../preflash/flash-qualified-app0-1.0.2-dev.7-1314bc4.sh 2>&1 | tee flashlog/flash.log
```

**S4, boot registrato:**
```bash
session_proc.py start usb_boot_usb usb usb_boot usb_boot.console --also usb_session.py -- bash boot_and_record.sh usb_boot
bash boot_reads.sh usb_boot 7
```
Attesi:
- nessun panic e nessuna `Coprocessor exception`: il nucleo FP32 usa la FPU, i costruttori globali no;
- `MOTION_ACC_CEILING verified=220` al READY senza comando;
- NVS gen 1 `VALID_ACKNOWLEDGED`, calibrazione `be8d5902bb4e37da` 12/12.

**S5, preflight del candidato:**
```bash
EXPECT_FW=1.0.2-dev.7 EXPECT_BUILD=1314bc411c58 python3 -I preflight.py pre/preflight_1314bc4.json
```
Riportare l'heap libero: atteso non inferiore a `30c894a`, con ~3,9 KB in più dalla IRAM liberata.

**S6, catena A/B** (dashboard chiusa, 2 client dello strumento, `@STOP` USB):
```bash
bash run_ab_chain.sh <HHMM>
python3 -I ab_compare.py <HHMM>
python3 -I cand_vs_ref.py <HHMM>                    # contro 30c894a, catena 1148 del 18/11
MATDOG_REF=1811 python3 -I cand_vs_ref.py <HHMM>    # contro 85bad0e
python3 -I acq_check.py usb_boot settle_A_<HHMM>
```
Copre:
- START/STAND (settle) e cambi di HEIGHT;
- WALK STANDARD, preset e joystick sterzato (yaw B3/D8/D10, laterale D4/D6, indietro D2);
- FAST (E/F/G);
- RECOVERY e ripresa (H);
- PRONE;
- STOP USB in ogni prova.

**S7, ciclo di alimentazione a freddo** (operatore, PRONE_REST, SAFE_OFF): `bash qon_recorder.sh qon_cold 7`, poi KEY OFF per almeno 10 s e KEY ON. È anche un secondo avvio a freddo registrato del firmware BMS di `30c894a`.

**S8, dashboard D1 e D2:** come il 18/11 (S8 della procedura `30c894a`).
- **L'operatore esegue l'accesso amministratore**: Claude non legge né inserisce il token.
- Lo STOP della dashboard è lo STOP HTTP.
- Al massimo 2 client HTTP.
```bash
python3 -I acq_check.py usb_boot settle_A_<HHMM> qon_cold
python3 -I cand_vs_ref.py <HHMM> dash2
```

**S9, facoltativo, solo se l'operatore vuole qualificare lo SLOW sterzato** (patch `kSteer`): una sequenza joystick SLOW laterale e yaw entro i limiti SLOW, con STOP USB. Non fa parte della catena standard. Senza questo passo lo SLOW sterzato resta certificato solo offline.

**S10, chiusura:**
- PRONE_REST, SAFE_OFF 8191, coppie OFF;
- `@BMS STREAM OFF`;
- `session_proc.py check --expect-none` → `SAFE_TO_POWER_OFF`.

## 3. Misure da riportare

Per ogni prova (catena 8/8, D1, D2):
- tempo massimo delle release e istogramma;
- distribuzione per classe P0 (doppia compensazione, NOMINAL+COMPENSATE, WALK sterzato con doppie IK) contro `30c894a` 1148;
- overrun;
- release operative perse, prima STAND compresa;
- `missed_releases` dal boot, riportate a parte;
- STOP USB e HTTP: verificato e tempo;
- qualità della compensazione: motivi IMU (ENVELOPE, WORKSPACE, SLEW, ATTENUATED) contro `30c894a`;
- continuità dei frame: salto RAW massimo e seconda differenza per fase, dai log della catena;
- heap minimo e stack.

## 4. Criteri (immutati rispetto al 18/11)

**P0 chiuso solo se, in ogni prova (catena 8/8, D2, D1 via `dash_counters.py`):**
- massimo ≤ 16 000 µs;
- 0 overrun;
- 0 release operative perse, prima STAND compresa.

**ACQ** (`acq_check.py`, invariato):
- **B:** riga `verified=220` senza comando dell'host nei 2 s precedenti, in `usb_boot` e in `qon_cold`;
- **S:** prima STAND sotto 50 ms;
- **W:** 0 release perse nella prima finestra di `settle_A`.

**Sicurezza:** STOP verificato in ogni prova (USB nella catena, HTTP dalla dashboard), nessun fault, heap minimo ≥ 24 KB.

**Riferimento:** il QEMU non è un tempo hardware. Il candidato ha in QEMU 0 release sopra 280 k cicli e classi P0 −74/−75 % rispetto a `30c894a`; solo questa sessione dice se il limite di 16 ms è raggiunto.

## 5. Arresti e rollback

**Arresti** come il 18/11: FAULT, silenzio DALY o `BMS TIMEOUT`, anomalia meccanica, memoria sotto soglia, stato `stale`. Si fa STOP, nessuna ripresa automatica.

**Un P0 non chiuso non è motivo di rollback.**

**Rollback a `30c894a`** solo per un criterio di sicurezza proprio del candidato:
```bash
export MATDOG_INTEG_1314BC4_FLASH_AUTHORIZED=YES_ROBOT_SUPPORTED
bash rom_entry.sh && bash rom_checks_before_rollback.sh   # verifica che l'installato sia esattamente 1314bc4
export MATDOG_ACQV2_30C894A_FLASH_AUTHORIZED=YES_ROBOT_SUPPORTED
bash ../../MATDOG_1_0_2_ACQV2_CLEAN_20261118/preflash/flash-qualified-app0-1.0.2-dev.7-30c894a.sh 2>&1 | tee flashlog/rollback.log
```
Immagine di rollback anche in `~/MATDOG/backups/esp32/app_images/30c894a` (SHA256SUMS verificato il 19/11).

## 6. Nessuna scrittura NVS

Come il 18/11: nessuna EEPROM, calibrazione, NVS o limite. Il candidato scrive solo il registro 85 in RAM dei servo al READY (Lock 1, coppie OFF), come `30c894a`.
