# MATDOG — NormaCore Station: riferimento canonico

**Stato:** validato su hardware
**Data validazione:** 2026-07-21
**Scopo:** evitare di ripetere nelle chat future l’analisi dello startup, della configurazione, del bus ST3215, di NormFS e della telemetria Station.

Questo documento è il riferimento operativo canonico per l’uso di NormaCore Station con MATDOG.

---

## 1. Regole permanenti

1. **NormaCore Station è l’unico proprietario della seriale ST3215.**
2. Non aprire `/dev/ttyACM0` tramite `pyserial` o altri processi mentre Station è attiva.
3. Lo startup Station è considerato hardware **read-only** soltanto quando:
   - non sono attivi client capaci di accodare comandi;
   - la coda `commands` non riceve nuovi elementi;
   - non è attiva una modalità di mirroring;
   - non viene avviata alcuna procedura di auto-calibrazione.
4. Il watcher MATDOG segue esclusivamente:
   - `st3215/inference`
5. Il watcher read-only:
   - non importa `send_commands`;
   - non costruisce `DriverCommand`;
   - non accoda elementi nella coda `commands`;
   - non abilita torque;
   - non invia target;
   - non scrive RAM;
   - non scrive EEPROM.
6. `GOAL_POSITION` ST3215 resta unsigned standard.
7. Non usare signed-wrap.

---

## 2. Percorsi canonici

Repository MATDOG:

```text
~/MATDOG/github/robot-dog
```

Repository NormaCore:

```text
~/norma-core
```

Binario Station release validato:

```text
~/norma-core/target/release/station
```

Configurazione Station dedicata a MATDOG:

```text
~/MATDOG/runtime/station/station.yaml
```

Contenuto configurazione MATDOG validato:

```yaml
drivers:
  st3215:
    enabled: true
    current-threshold: 500
    deadband: 40
  system-info: true
```

Log runtime read-only:

```text
~/MATDOG/runtime/station/station_readonly_preflight.log
```

PID file runtime read-only:

```text
~/MATDOG/runtime/station/station_readonly_preflight.pid
```

Directory dati isolata per preflight read-only:

```text
~/MATDOG/runtime/station/station_data_readonly_preflight
```

---

## 3. Bus ST3215 MATDOG

Bus seriale canonico:

```text
5B14114953
```

Dispositivo Linux rilevato:

```text
/dev/ttyACM0
```

Symlink persistente:

```text
/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B14114953-if00
```

Adapter USB:

```text
VID:PID = 1a86:55d3
QinHeng Electronics USB Single Serial
```

L’utente operativo deve appartenere al gruppo:

```text
dialout
```

---

## 4. Mappa motori attesa

Station deve rilevare esattamente i seguenti 12 ID:

```text
11 12 13
21 22 23
31 32 33
41 42 43
```

Mapping MATDOG:

| Zampa | Hip | Upper | Lower |
|---|---:|---:|---:|
| LF | M13 | M12 | M11 |
| RF | M23 | M22 | M21 |
| RH | M33 | M32 | M31 |
| LH | M43 | M42 | M41 |

Un ID mancante o inatteso blocca la procedura.

---

## 5. Cosa fa Station allo startup

Con `drivers.st3215.enabled: true`, Station:

1. crea le queue NormFS;
2. registra `st3215/rx`;
3. registra `st3215/tx`;
4. registra `st3215/meta`;
5. registra `st3215/inference`;
6. individua gli adapter USB ST3215 compatibili;
7. apre il bus seriale;
8. esegue discovery tramite `Ping`;
9. legge configurazione e RAM dei servo;
10. pubblica lo stato aggregato su `st3215/inference`.

Lo startup non genera automaticamente:

- `Write`;
- `RegWrite`;
- `SyncWrite`;
- `Action`;
- `Reset`;
- `AutoCalibrate`;
- abilitazione torque;
- target posizione;
- scritture EEPROM.

Le funzioni di scrittura esistono nel driver, ma vengono eseguite soltanto in seguito alla ricezione di un nuovo comando.

---

## 6. Semantica della coda `commands`

Il driver ST3215 sottoscrive la queue NormFS:

```text
commands
```

Flusso comando:

```text
commands
  -> StationCommandsPack
  -> STC_ST3215_COMMAND
  -> st3215/tx
  -> worker della porta seriale
  -> process_command()
```

### Proprietà critica verificata

Con NormFS `0.1.0-beta.1`, `subscribe()`:

- registra un callback nella queue in memoria;
- notifica soltanto i **nuovi enqueue**;
- non effettua automaticamente il replay degli elementi storici;
- non rilegge il WAL storico all’atto della sottoscrizione.

Di conseguenza, la presenza di file storici nella directory `commands` non provoca da sola la riesecuzione dei vecchi comandi.

Resta comunque preferibile usare una directory dati isolata per i preflight read-only.

---

## 7. Avvio canonico read-only

Prima dell’avvio:

- robot meccanicamente stabile;
- nessun client command-producing attivo;
- nessun teleop;
- nessun executor;
- nessuna auto-calibrazione;
- nessun altro proprietario della seriale;
- torque atteso OFF.

Avvio locale e isolato:

```bash
cd ~/MATDOG/runtime/station

RUST_LOG=info \
~/norma-core/target/release/station \
  --config ~/MATDOG/runtime/station/station.yaml \
  --normfs-base-folder \
    ~/MATDOG/runtime/station/station_data_readonly_preflight \
  --tcp 127.0.0.1:8888
```

Proprietà dell’avvio:

- TCP esposto soltanto su localhost;
- porta NormFS TCP `8888`;
- interfaccia web non avviata;
- directory dati separata;
- driver ST3215 attivo;
- telemetria attiva;
- nessun comando generato automaticamente.

Verifica listener:

```bash
ss -ltnp | grep '127.0.0.1:8888'
```

Nel log devono comparire:

```text
NormFS server listening on 127.0.0.1:8888
New ST3215 port detected: /dev/ttyACM0
Successfully opened ST3215 port: /dev/ttyACM0
```

Devono inoltre essere rilevati tutti i 12 ID MATDOG.

---

## 8. Configurazione inference predefinita

Quando la chiave YAML `inference` è assente, Station avvia la configurazione predefinita `normvla`.

Nel preflight validato sono comparsi messaggi come:

```text
No inference configuration found, using default normvla config
Skip: no_bus (... torque=false)
```

Questo non ha inviato comandi e non ha abilitato torque.

Il writer shared-memory predefinito può creare:

```text
/dev/shm/normvla
```

Per il watcher MATDOG questa inference non è necessaria, ma nel comportamento corrente di Station viene avviata automaticamente quando `inference` non è specificata.

---

## 9. Semantica reale di `st3215/inference`

`st3215/inference` contiene uno stato aggregato del bus.

Quando Station riceve una nuova lettura di un singolo servo:

1. aggiorna solo il `MotorState` di quel servo;
2. conserva gli ultimi stati noti degli altri servo;
3. serializza nuovamente l’intero `InferenceState`;
4. pubblica un nuovo frame aggregato.

Conseguenza:

- un nuovo frame non implica che tutti i 12 motori abbiano un timestamp nuovo;
- molti motori possono comparire con lo stesso timestamp del frame precedente;
- timestamp uguale significa campione aggregato duplicato, non regressione;
- timestamp inferiore significa regressione reale e deve bloccare;
- ogni motore deve comunque avanzare almeno una volta durante una finestra multi-frame.

Regola corretta:

```text
new_timestamp < previous_timestamp  -> HARD BLOCK
new_timestamp = previous_timestamp  -> duplicate aggregato ammesso
new_timestamp > previous_timestamp  -> nuovo campione motore
```

La barriera dopo un comando resta invece stretta:

```text
command success
+
campione del motore con timestamp strettamente maggiore
```

---

## 10. Watcher read-only MATDOG

Script canonico:

```text
06_Software/Matdog_Core/calibration/
matdog_endstop_station_readonly_watch.py
```

Queue seguita:

```text
st3215/inference
```

Comando validato:

```bash
cd ~/MATDOG/github/robot-dog

python3 \
  06_Software/Matdog_Core/calibration/\
matdog_endstop_station_readonly_watch.py \
  --server 127.0.0.1 \
  --bus-serial 5B14114953 \
  --frames 40 \
  --frame-timeout 5
```

Schema report corrente:

```text
matdog.endstop.station_readonly_watch.v2
```

Il watcher deve riportare:

```json
{
  "all_motors_advanced": true,
  "all_status_zero": true,
  "all_torque_disabled": true,
  "command_api_available": false,
  "motor_commands_sent": false,
  "eeprom_writes_sent": false,
  "timestamps_strictly_increasing": true
}
```

---

## 11. Risultato hardware validato del 2026-07-21

Validazione eseguita con:

```text
server: 127.0.0.1
bus: 5B14114953
requested_frames: 40
received_frames: 40
```

Esito:

```text
PASS
```

Condizioni verificate:

- 12/12 motori presenti;
- set ID esatto;
- Station `app_start_id` stabile;
- tutti i motori hanno avanzato il proprio timestamp;
- nessuna regressione timestamp;
- status zero su tutti i servo;
- torque disabilitato su tutti i servo;
- nessun comando motore inviato;
- nessuna scrittura EEPROM;
- nessuna API di comando disponibile nel watcher.

I numerosi `duplicate_frame_count` osservati sono coerenti con la pubblicazione aggregata di Station.

---

## 12. Condizioni di HARD BLOCK

Bloccare immediatamente la procedura in caso di:

- bus `5B14114953` assente;
- ID servo mancante;
- ID servo inatteso;
- torque attivo su almeno un servo;
- status non zero;
- timestamp motore regressivo;
- motore che non avanza durante una finestra multi-frame;
- cambio di `app_start_id`;
- timeout stream;
- processo command-producing inatteso;
- secondo proprietario della seriale;
- listener Station esposto su interfacce di rete non previste;
- comando presente nella coda `commands` durante un preflight read-only.

---

## 13. Arresto Station

Quando Station è stata avviata in foreground:

```text
Ctrl+C
```

Quando è stata avviata tramite PID file:

```bash
kill "$(cat \
  ~/MATDOG/runtime/station/station_readonly_preflight.pid)"
```

Dopo l’arresto verificare:

```bash
ss -ltnp | grep ':8888'
```

Nessun listener deve rimanere attivo.

---

## 14. Checklist per le chat future

Prima di rifare qualunque analisi su NormaCore Station:

1. leggere questo documento;
2. verificare che i percorsi non siano cambiati;
3. verificare il bus seriale;
4. verificare la configurazione MATDOG;
5. controllare soltanto eventuali differenze introdotte da nuovi commit NormaCore;
6. non ripetere da zero l’audit di `subscribe()`, del driver o dello startup se le versioni non sono cambiate.

Ripetere l’audit completo soltanto se cambia almeno uno tra:

- versione NormaCore;
- versione NormFS;
- implementazione `St3215Driver::new`;
- implementazione `St3215Port::worker_loop`;
- implementazione `NormFS::subscribe`;
- configurazione Station MATDOG;
- hardware USB/seriale;
- pipeline `st3215/inference`.

---

## 15. Versioni validate

```text
NormFS: 0.1.0-beta.1
Station binary: release build del repository ~/norma-core
Watcher: matdog.endstop.station_readonly_watch.v2
Bus seriale: 5B14114953
Validazione hardware: 2026-07-21
```
