# MATDOG — Closeout hardware non-motion — 2026-10-11

**Verdetto: BASELINE HARDWARE NON-MOTION VALIDATED; MOTION NOT VALIDATED.**

Chiusura del perimetro non-motion sulla branch `feat/wifi-finalization-20261011`;
non è accettazione generale di release, né autorizzazione di calibrazione, torque,
gait, locomozione, flash, OTA o merge su main. La chiusura incorpora output originali incollati dall’operatore, recuperati
dalla conversazione e verificati per integrità; dichiara sotto la provenienza. Nessuna nuova misura hardware è
stata eseguita per scrivere questo report.

## Identità e perimetro

- Robot MATDOG ricostruito: ESP32-S3, BNO085, DALY, bus ST3215 (13 installati:
  12 zampe + collo ID 51; 17 ID canonici, 4 assenti per design), 12 WS2812 GPIO47.
- Firmware installato dichiarato dall'operatore:
  `b08322dedc5564c080f691dc98047511ef2b953f`; build breve visibile nello screenshot reale
  `b08322dedc55`. Questo SHA resta distinto dal successivo HEAD documentale.
- Profilo `ROBOT_POWERED`, layout `MATDOG_16M_2x5M_NVS_V1`, `OTA_INGEST=0`.
  Manifest di release locale verificato documentalmente: E2.
- Applicazione dichiarata flashata APP-ONLY, `app0@0x010000`, SHA-256
  `ccbce2ff6ffee9a5e132bad8646f379fb31fdd305c0d577df992e85e2ebf4b43`.
  Il pacchetto riferisce `APPLICATION_ONLY_FLASH = PASS` e
  `Verification successful (digest matched)`: contenuto confermato nella
  trascrizione integrale originale recuperata E9, non log automatico nativo Asus.
- Il binario build corrente è stato ricompilato nella successiva analisi LAN:
  non è assunto identico al binario installato. Non ricompilato o riflashato nel closeout.

## Provenienza e criteri di lettura

PASS significa successo nel solo controllo indicato; la colonna evidenza distingue
misura registrata, osservazione operatore e verifica offline. DEFERRED significa
rinviato deliberatamente; NOT TESTED significa che manca una prova di quel requisito.
I FAIL storici restano registrati anche se un controllo successivo è positivo.

| ID | Fonte reale | Che cosa sostiene e che cosa non sostiene |
|---|---|---|
| E1 | Locale: `~/MATDOG/verification-artifacts/closeout-20261011/MATDOG_HW_CLOSEOUT_INPUT.md` e `visuals/LOCAL_ONLY_wifi_sta_actual_2026-10-11.jpg` | Input di contesto dell'operatore, non attestazione firmata. Screenshot reale ispezionato: build breve b08322d, risultato positivo STA e COMMITTED; non prova flash, NVS o misure di sicurezza. Originali Wi-Fi esclusi dal repository per privacy. |
| E2 | `~/MATDOG/verification-artifacts/wifi-release-b08322d/release.txt` | Identità candidato, profilo/layout/OTA, hash e piano APP0. Riporta esplicitamente `FLASH=NOT_RUN`: da solo NON prova l'installazione. Estratto pubblico in [evidenze](HW_Closeout_2026-10-11/evidence-excerpts.txt). |
| E3 | `~/MATDOG/verification-artifacts/wifi-sta-stop-fix-20261011/offline-checks.txt`, `baseline-regression.log`; commit b08322d in Git | Regressione riprodotta offline, poi `WIFI_STA_TEST: 2165 checks, 0 failures`, host/static audit PASS; motion execution suites DEFERRED. Il rifiuto pre-flash TREE_NOT_CLEAN era atteso prima del commit: non è fallimento hardware finale. |
| E4 | `~/MATDOG/verification-artifacts/MATDOG_WIFI_FINALIZATION_20261011/hardware-20261011/preflight-usb.log` | Vecchia build 837f523: READY, MAINTENANCE, NONE, moduli online, NVS gen1 integra. Estratto con numeri di riga e hash del file in [evidenze](HW_Closeout_2026-10-11/evidence-excerpts.txt). Non è una lettura post-flash b08322d. |
| E5 | [LED hardware 2026-10-10](MATDOG_LED_RMT_DMA_DIRTY_FRAME_HW_VALIDATION_2026-10-10.md) | RMT DMA attivo, osservazione 10 min con AP e zero client, nessuna anomalia osservata su fee9383. Non estendere durata/condizioni alla build finale. |
| E6 | [Audit dev.3 2026-10-06](MATDOG_V0_2_DEV3_THERMAL_DELTA_AUDIT_2026-10-06.md) | Census 13/13, preflight 12/12 e torque off storici; SAVE/ACK/LOAD attraverso reset, nessun vero power-cycle. |
| E7 | [Power/charging closeout 2026-09-24](../Development_Log/2026-09-24_MATDOG_POWER_CHARGING_USB_VALIDATION_CLOSEOUT.md) e [power canonico](../../04_Electronics/MATDOG_POWER_STATES_AND_CHARGING.md) | KEY OFF senza charger/USB e alimentazione del bus dal charger comune: evidenza storica, non nuova misura. |
| E8 | `~/MATDOG/verification-artifacts/lan-maintenance-feasibility-b08322d/feasibility.md`; policy sorgente b08322d | Analisi architetturale, nessuna implementazione LAN o prova hardware LAN. |
| E9 | [Appendice integrativa](HW_Closeout_2026-10-11/MATDOG_2026-10-11_EVIDENCE_APPENDIX.md) e tre originali LOCAL_ONLY nel supplemento locale | SHA256SUMS 5/5 PASS; trascrizione integrale output flash e risposte seriali post-flash/STA incollate dall’operatore e recuperate dalla conversazione. Contenuto disponibile e tracciabile, non cattura automatica nativa Asus; nessun nuovo test. |

I report locali `MATDOG_WIFI_FINALIZATION_20261011/RAPPORTO_HARDWARE.md`,
`FLASH_LOG.md` e `HARDWARE_VALIDATION_MATRIX.md` descrivono tentativi precedenti
arrestati (backup fallito e poi USB senza risposte); NON sono log del flash finale.
Non cancellati né riscritti. I dati finali riferiti da E1 sono successivi.
Il supplemento E9 recupera integralmente il contenuto finale flash/post-flash/STA:
la precedente lacuna di contenuto è chiusa. APP-ONLY, digest e NVS finale sono
**PASS documentati nell'output originale dell’operatore**. Non esiste una cattura
automatica nativa Asus di questi testi; gli hash verificano gli allegati ricevuti,
non una nuova verifica indipendente del dispositivo. Registrazione automatica
per sessioni future è un miglioramento di processo, non un test hardware pendente.

## Matrice finale

| Controllo | Esito | Evidenza / limite |
|---|---|---|
| Identità sorgente installata b08322d | PASS | E9 output flash SOURCE_COMMIT completo; E1 build breve nello screenshot; E2 coerente |
| APP-ONLY app0, digest matched | PASS (output operatore) | E9 trascrizione integrale: digest matched, write/erase APP0, NVS esclusa; E2 piano/manifest |
| ROBOT_POWERED / layout V1 / OTA_INGEST=0 | PASS | E9 output originale conferma profilo/layout/OTA; E2 candidato, E4 storico |
| READY, MAINTENANCE, authority NONE, motion_allowed=NO | PASS (output operatore) | E9 risposte seriali finali; E4 storico. `power_state=RUN` è stato elettrico, non `mode=RUN` né permit |
| BNO085, DALY, bus servo online | PASS (output operatore) | E9 risposte finali; E4 sulla build precedente. Non è qualifica completa sensori/attuatori |
| Census 13/13, 4 assenti per design; preflight zampe 12/12, torque off | PASS (storico) | E4/E6; non presentato come nuova scansione post-flash |
| NVS VALID_ACKNOWLEDGED, generazione 1 integra dopo flash | PASS (output operatore) | E9 finale: gen1 integra, SLOT_A=VALID, SLOT_B=ABSENT, DEGRADED=0; RECONCILIATION_REQUIRED=0 è E1 riferito/E4 storico, non campo stampato in E9 |
| AP e Dashboard sei tab a 192.168.4.1 | PASS (output operatore) | E9 ADMIN_AP, portal LISTENING senza fault; E1 sei tab e screenshot tab Wi-Fi |
| STA_STOP prima della patch | FAIL (storico) | E1 riferisce UNEXPECTED_ROLE_EVENT STA_STOP e AP perso; E3 riproduce regressione offline |
| Fix STA_STOP e successiva prova STA temporanea | PASS | E3 offline, E9 seriale originale CONNECTED/COMMITTED: seq=1, associated=YES, attempts=1, elapsed_ms=7900, reason=0, rollback=NO; E1 screenshot coerente |
| Profilo primary/credenziali persistite e ritorno AP | PASS (output operatore) | E9 COMMITTED, profilo index=0 enabled=YES, source NVS riferita E1 e ritorno AP; non prova persistenza su power-cycle. connected=NO in MAINTENANCE è atteso |
| LED barra SOC, avanzamento charging e breathing successivo | PASS (osservazione operatore) | E1; escursione verde poco evidente. Trasporto storico E5; mockup non probatori |
| DALY telemetria sana | PASS (osservazione operatore) | E1 prima del flash: SOC 98.9%, pack 11.9 V, min/max 3972..3985 mV, delta 13 mV, nessun allarme; non certifica FULL/termination |
| Charging termination/cutoff e qualifica unattended/dock | NOT TESTED | Nessuna piena qualifica; E7 osservazioni limitate di charging manuale |
| KEY config su vero ciclo BMS; discrepanza MOS scarica | NOT TESTED | E1/E7; KEY OFF riferito con discharge_mos ON da investigare, nessuna forzatura MOS |
| LED SOC <=30 ambra / <=15 rosso e maggiore escursione charging | DEFERRED | Implementazione/validazione richieste per fase successiva, solo simulazione SOC |
| Dashboard LAN in MAINTENANCE | DEFERRED | E8: non implementata; AP-only attuale |
| STA automatica in RUN; retry/router roaming e prove negative hardware complete | NOT TESTED | Offline non equivale a hardware; nessun RUN per accesso LAN |
| Durata/revoca sessione admin, accessi non autorizzati, CSRF server-side | NOT TESTED | Login visivo non basta; threat model e prove approfondite mancanti |
| OTA ingest/update e rollback automatico hardware | NOT TESTED | OTA_INGEST=0, nessun aggiornamento OTA qualificato |
| Calibrazione operativa, envelope, motion permit | DEFERRED | NVS valido non ammette trasformazione o autorità; RESTORE=NOT_IMPLEMENTED |
| Motion execution suites | DEFERRED | E3 e ADR-004; nessun PASS dedotto dal riepilogo host |
| Torque operativo, gait e locomozione reali | NOT TESTED | Nessuna esecuzione o autorizzazione in questa validazione non-motion |

## Correzione STA_STOP

Il commit `b08322d` rende esplicita la proprietà del restart STA provocato dalla
configurazione bandwidth: registra gli ACK attesi stop/start e serializza il
worker fino al loro completamento prima di associazione, commit o ritorno AP.
Non ignora indiscriminatamente gli eventi imprevisti. Il diff sorgente e la
regressione E3 sostengono la correzione; la prova positiva E9 e lo screenshot E1 confermano il risultato
operatore. L'interruzione temporanea AP durante «Prova e salva» è intenzionale.
IP DHCP e SSID domestici non sono riportati qui.

## Limiti e prossimo gate

Persistenza gen1 integra significa evidenza salvata, **non motion permit**.
Lo stato operativo precedente `STALE_PENDING_FULL_RECALIBRATION` è riferito da E1;
non è sostituito dal verdetto del record. `MOTION_AUTHORIZED=0` e
`RESTORE=NOT_IMPLEMENTED` restano invarianti. Nessun restore automatico.

La chiusura non risolve gli acceptance blocker storici dev.3 per scope estranei,
le marginature/calibrazioni/limiti operativi o i residui termici. Non promuove
main a immagine flashabile. Breathing, LAN, charging/MOS e security restano aperti
nel registro canonico [OPEN_ITEMS](../OPEN_ITEMS.md#closeout-2026-10-11--fase-successiva).
Handoff operativo: [Motion 2026-10-12](../Development_Log/2026-10-12_MOTION_ENABLEMENT_HANDOFF.md).

## Privacy e conservazione

Solo estratti testuali selezionati senza SSID/router IP/MAC/segreti sono pubblicati.
Nessun raw flash, binario, backup, dump NVS, cookie, token o credenziale aggiunto.
[Media](../../07_Media/Dashboard/2026-10-11/README.md): due mockup illustrativi
pubblicati, entrambi i file Wi-Fi esclusi e conservati localmente. L'esclusione
dello screenshot reale limita l'audit pubblico della prova STA ma protegge la privacy.
