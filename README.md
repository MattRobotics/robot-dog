# MATDOG — quadrupede robotico di Matt Robotics

MATDOG è un robot a quattro zampe progettato da zero. Il Controller ESP32-S3 trasforma i comandi della dashboard in traiettorie e comandi coordinati ai servo; raccoglie feedback e gestisce STOP e protezioni. Il Digital Twin mostra la posa misurata. L'obiettivo successivo è il comando direzionale continuo, senza affidare al browser il controllo degli attuatori.

**Al 05/11/2026 il robot ha installata la dev.14.** STAND, PRONE e WALK STANDARD sono stati osservati sul robot reale; la stabilizzazione IMU è già implementata e attiva, ma non ancora adeguata per ampiezza e velocità. Il flash è verificato; la validazione Motion è **parziale**. Non è una release stable.

[Report hardware dev.14](09_Logs/Validation_Reports/Dev14_HW_2026-11-05/REPORT.md) · [codice esatto installato, tag checkpoint](https://github.com/MattRobotics/robot-dog/tree/checkpoint/matdog-dev14-hw-20261105) · [problemi aperti](09_Logs/OPEN_ITEMS.md)

## Hardware e architettura

| Parte | Configurazione effettiva |
|---|---|
| Controller | ESP32-S3, unica autorità su Motion, bus servo e sicurezza runtime |
| Attuatori | 12 ST3215 delle zampe più collo ID51: 13 installati, 17 posizioni canoniche allocate |
| Sensori | IMU BNO085, BMS DALY, feedback di posizione, carico, tensione e temperatura dei servo |
| Interfaccia | Dashboard web dell'ESP32; comandi semantici STAND, PRONE, WALK STANDARD e STOP; Twin in sola lettura |
| Motion | Stack SIMPLE: gait, cinematica inversa, compensazione IMU, frame coordinati e feedback asincrono |
| Geometria | URDF operativo REV01-FIX; mapping e calibrazione persistente gen-1 preservati |
| Alto livello | Jetson Orin Nano Super previsto per integrazioni successive; nessuna autorità diretta browser → servo |

## Cosa funziona e cosa resta da completare

| Funzione | Stato reale della dev.14 |
|---|---|
| STAND e PRONE | Transizioni osservate a terra, incluso PRONE → STAND150 e HOLD120. La prima prova STAND150 partiva dalla stessa posa sulla fixture. |
| WALK STANDARD | WALK30 e WALK60 realmente eseguiti; appoggi regolari riferiti dall'operatore. Restano release perse persistenti e picchi superiori al periodo nominale di20ms. |
| Stabilizzazione IMU | Correzione pitch/roll implementata e attiva. Pitch quasi assente a150mm; a100mm pitch e roll reagiscono, ma risposta lenta ed escursione laterale insufficiente secondo operatore. Ampiezza geometrica e velocità di risposta sono problemi distinti. |
| STOP | Arresto controllato e coppia mantenuta osservati da STAND e WALK. Recupero WALK operativo ma lungo; recupero dopo STOP in PRONE intermedia incompleto. |
| PRONE terminale | Coppia ON; appoggio non verificato. Nessun rilascio automatico della coppia qualificato soltanto dalla posa PRONE. |
| Digital Twin | Segue la posa; Reset vista non ripristina ancora il riferimento frontale grafico +X richiesto. Nessun reset fisico IMU per correggere la UI. |
| RESTORE NVS | Automatico gen-1 operativo; boot dev.14 con record integro e READY. Percorso manuale dashboard e manutenzione da qualificare separatamente. |
| LED | Nessuna anomalia riferita nell'ultima sessione; il difetto intermittente storico resta aperto. |

Le osservazioni fisiche non sono una misura di contatto o stabilità certificata. I test offline non equivalgono a prove hardware. Cause dei picchi temporali e delle letture termiche isolate restano da discriminare; non vanno nascoste riducendo le protezioni.

## Quattro riferimenti da non confondere

| Linea | Significato |
|---|---|
| **main** | Sorgente attualmente integrato, basato sulla linea dev.7 più documentazione. Questo closeout non integra il firmware dev.14 o dev.15. Una build di main non è l'APP installata. |
| **dev.14** | Firmware realmente installato e parzialmente validato. Sorgente conservato dal tag annotato [checkpoint/matdog-dev14-hw-20261105](https://github.com/MattRobotics/robot-dog/tree/checkpoint/matdog-dev14-hw-20261105). |
| **dev.15** | Sviluppo Motion direzionale in corso, separato e **NON validato hardware**; questo documento non ne attesta compilazione o qualifica. |
| **dev.7** | [Pre-release storica pubblicata](https://github.com/MattRobotics/robot-dog/releases/tag/matdog-controller-v1.0.1-dev.7), con [report del31/10](09_Logs/Validation_Reports/Motion_dev7_HW_2026-10-31/REPORT.md). Non è più il firmware installato e non viene promossa a stable. |

Identità dev.14: commit `4ff2aadf4ed907d39f2b3ad4f9e6af54db5decee`, build `4ff2aadf4ed9`, profilo ROBOT_POWERED, OTA ingest0. APP SHA256: `71028f95a8aee4b4ff7237f9420eec6aeeb0efa0aee5c00c00ca5a7bb947bd22`. Il tag conserva il sorgente; non è una nuova GitHub Release e non garantisce una ricompilazione bit-identica.

## Priorità aperte

- [#52 — Motion](https://github.com/MattRobotics/robot-dog/issues/52): budget temporale, IMU, recuperi STOP e tracking.
- [#62 — Dashboard, Twin e LED](https://github.com/MattRobotics/robot-dog/issues/62): Reset vista +X e difetto LED intermittente.
- [#48 — RESTORE e manutenzione](https://github.com/MattRobotics/robot-dog/issues/48): percorso manuale, rifiuti motivati e sospensione dell'automatismo.
- [#46 — Stable futura](https://github.com/MattRobotics/robot-dog/issues/46): accettazione e pubblicazione dopo qualifica del binario effettivo.

## Orientarsi nel repository

- [Architettura](01_Docs/02_Architecture/ARCHITECTURE.md), [firmware](05_Firmware/README.md), [software e Twin](06_Software), [CAD](03_CAD/README.md), [elettronica e alimentazione](04_Electronics/README.md).
- [Report e log](09_Logs/README.md), [cronologia](09_Logs/HISTORY_INDEX.md), [problemi aperti](09_Logs/OPEN_ITEMS.md).
- [Storia firmware fino al26/10](09_Logs/Firmware_History/README.md) e [snapshot documentali precedenti](09_Logs/Historical/Dev14_Documentation_Closeout_2026-11-05/README.md). Documenti originali conservati; le loro condizioni sono riferite alle rispettive date.

## Operazioni e sicurezza

Non usare il solo nome di un branch per scegliere un firmware da installare: servono artefatto, manifest, SHA e procedura APP-only supervisionata. La pubblicazione di sorgenti o documentazione non autorizza flash o movimenti. Nessun movimento al boot; l'ammissione Motion è governata a runtime dall'ESP32.

Con caricatore collegato il bus può restare alimentato anche con KEY OFF. Preservare STOP, limiti, NVS e mapping; nessuna scrittura EEPROM/offset servo fuori da procedure autorizzate. [Avvertenze flash e relativo contesto storico](05_Firmware/MATDOG_Controller/FLASH_LAYOUT_SAFETY_NOTICE.md) · [stati di alimentazione](04_Electronics/MATDOG_POWER_STATES_AND_CHARGING.md).

Per agenti AI: [AGENTS.md](AGENTS.md).
