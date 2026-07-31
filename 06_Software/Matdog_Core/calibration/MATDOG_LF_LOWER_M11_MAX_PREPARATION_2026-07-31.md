# MATDOG — preparazione LF_LOWER_M11_MAX

**Data:** 2026-07-31  
**Stato:** offline preparation complete; hardware not authorized or executed  
**Dipendenza:** checkpoint `LF_LOWER_M11_MIN` V28R e NormaCore PR draft #6.

## Sorgente NormaCore vincolata

```text
branch: matdog/lf-lower-m11-min-v28r-alignment
head:   fff8e8989ca945bb56982ab5f626e3b45ba8b2dd
base:   32e3222c87016b7f5d7c1c1da497a4cea3e7b80a
```

CI indipendente conclusa con successo:

```text
MATDOG LF LOWER M11 MIN V28R Source CI: 30647874565
MATDOG Native Calibrator Offline Check: 30647874515
Station Viewer PR Check: 30647874525
```

Nessun merge in `main` è stato eseguito.

## Profilo riconfermato dal sorgente e dai test

```text
profile: LF_LOWER_M11_MAX
probe motor: M11
probe direction: tick decrescenti
home: 2048
baseline: 1984
URDF MAX: 1621
guard: 1557
contact acceptance corridor: 1557 .. 1685
```

Prerequisite:

```text
M42 = 2389
M13 = 2048
M12 = 3072  # LF upper orizzontale
M11 = unico probing joint
```

Il test Rust canonico `lf_lower_profiles_use_horizontal_upper_and_exact_unsigned_numbers`
conferma questi numeri. I profili HIP restano bloccati.

## Artefatti preparati

### Offline gate

```text
MATDOG_LF_LOWER_M11_MAX_OFFLINE_GATE_V29.sh
```

Il gate:

- richiede l'esatto head NormaCore sopra indicato;
- richiede working tree pulito e Station non attiva;
- verifica i token del profilo e i divieti RAM-only;
- esegue test mirati e l'intera suite ST3215;
- compila il viewer e verifica la UI MATDOG Auto Calibrate-only;
- compila Station release;
- genera un marker con SHA-256 del binario;
- dichiara esplicitamente `hardware_started=false` e `serial_opened=false`.

### Hardware runner preparato ma hard-disabled

```text
MATDOG_LF_LOWER_M11_MAX_HARDWARE_RUNNER_V29.sh
```

In questa task il runner supporta esclusivamente:

```text
--prepare
```

Verifica marker, head, binario, configurazione e seriale libera, quindi genera:

```text
RUN_CONTRACT.env
STATION_LAUNCH_COMMAND.disabled
SHA256SUMS
```

Non avvia Station, non apre la seriale e non invia comandi. Termina deliberatamente
con exit code `78`. L'attivazione del comando richiederà una task hardware
supervisionata separata e una nuova autorizzazione esplicita.

## Contratto permanente

- Station unico proprietario seriale;
- nessun `pyserial` parallelo;
- `GoalPosition` unsigned standard `0..4095`;
- sole scritture RAM: TorqueEnable, Acc, GoalPosition, GoalSpeed, TorqueLimit;
- vietati EEPROM, Position Offset, LOCK, Reset/ResetCalibration, RegWrite/Action,
  Save/Freeze;
- una sola armatura esplicita per avvio Station;
- Reset e Save disabilitati nella UI MATDOG;
- robot sostenuto, zampe libere, operatore presente e arresto fisico disponibile
  prima di una futura attivazione;
- nessun merge automatico in `main`.
