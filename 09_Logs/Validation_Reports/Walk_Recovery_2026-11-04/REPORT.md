# dev.12 — recupero WALK mediante il terminale esistente

IMPLEMENTED / VERIFIED OFFLINE. Base esatta dev.11:
`804f6ffd994d0a080f2cb2ef31687d7ed6614fa4`.
Installato dichiarato dall'operatore: dev.10 `ea17b7f1`; nessuna operazione sul dispositivo.
Worktree dedicato `matdog-dev12-walk-recovery-20261104`, branch
`codex/dev12-walk-recovery-20261104`. Dev.11 e rollback restano preservati.

## Implementazione e differenze rispetto a Opus

`git apply --check` della proposta Opus è passato sulla baseline pulita;
la proposta non era integrata. Ripresa di `gaitCartesian(..., terminal)`
solo dopo STOP verificato e STAND esplicito, mantenendo coppia e RAW:
clock raccordato C2 in un secondo, offset RAW svanito nello stesso intervallo,
swing nominale completato, terminale precedente conservato, recentraggio,
WALK_SETTLE, STAND150 canonico e infine quota richiesta. Stato MOVING,
nessun intento WALK, ENABLE o OFF implicito. STOP/SAFE_OFF/fault restano prioritari.
STOP durante WALK_SETTLE riconosce il riferimento mantenuto come origine WALK;
il fallback SAFE_OFF per riferimenti davvero indisponibili resta attivo.

Adattamenti mirati: eliminato il `continue` che saltava MotionService nelle
otto finestre stance/swing; status e JSON ammessi, comando accettato e reale
COMPLETED/STAND_REACHED con nessun enable/OFF sono ora verificati. Rimosso
il ciclo duplicato aggiunto da Opus. Rafforzati arrivo del secondo recupero,
quota finale e fault di alimentazione/IMU durante recupero. HTTP usa i veri
parser/servizio e gli envelope della dashboard per 45/60 mm; socket e scheduling
restano simulati. Riutilizzato il risultato locale dell'ammissione in `stand()`:
un solve nominale anziché due, nessuna cache o stato persistente aggiuntivo.
La dashboard dev.11 cambia solo motivo di rifiuto e avviso sul recupero con
possibile avanzamento e contatto non verificato; nessun nuovo controllo o polling.
`TiltedContactIk.cpp`, geometria, calibrazioni, NVS e BNO085 non sono modificati.

## Verifiche ed evidenze

Artefatti esterni: `~/MATDOG/build-output/dev12-walk-recovery-20261104/evidence/`.
Suite STOP e SimpleMotion con ASan/UBSan e Werror: PASS. STOP verifica 38
completamenti e altri otto via MotionService, RAW C0, box e passo <=20 tick,
STOP durante WALK STOP/WALK_SETTLE/recupero, priorità STOP/SAFE_OFF e fault,
quote 50/100/150 mm; STAND, PRONE e WALK STOP ordinari preservati.
Dashboard DOM, viewer parent feed (47), budget DOM (28), contratto HTTP/UI
(191): PASS. Il gate generale comprende HTTP production-handler (97).
Audit generale: STATIC_AUDIT=PASS, 357 sorgenti; host gate PASS.
Build, SHA e stack dell'ELF effettivo: vedere la ricevuta esterna finale,
senza confonderli con prove hardware.

Probe Opus originale: UBSan rileva `mainProbe()` int senza return finale,
exit 1. Correzione solo nello scratch: return 0 e inizializzazione degli array
storici del confronto; probe completo con sanitizer termina exit 0. Non si
considera il precedente crash equivalente a PASS. Probe dei frame: exit 0,
77 recuperi per passo (52 da swing); a 45/60 mm, rispettivamente:
avanzamento massimo 54,0/68,7 mm, durata massima 13,8 s, passo RAW 10/12 tick,
errore nominale del piede stance <=1,64/1,74 mm nel raccordo e
<=0,265/0,254 mm dopo. Sono campioni simulati, non limiti fisici garantiti.

## Costo e limiti temporali

Benchmark host O2 osservato, senza sanitizer: captureStatus mediano
0,687 us baseline contro 9,948 us finale; ammissione 0,319 contro 9,643 us.
Ogni snapshot indipendente ricontrolla dati vivi e usa un solve nominale;
nessun solve di ammissione nei release del recupero. Il secondo solve al
comando era lavoro duplicato misurabile ed è stato eliminato. I contatori
per release con roll 0,04 rad coincidono per WALK baseline, WALK finale e
recupero: massimi 2 solve compensazione, 8 IK, 16 Newton, 24 FK, oltre al
singolo solve nominale del gait. Il solver sperimentale è escluso.

Con clock host nel timer del mock: max_release_work_us baseline STOP/WALK
47/46, finale STOP/WALK 46/33 e 46 dopo il recupero (una esecuzione; rumore
scheduler incluso). Non sono tempi ESP32 né WCET. La suite con clock
puramente scriptato restituisce zero e non misura CPU. Il problema #52 non
è risolto o qualificato da queste misure. L'ELF finale deve essere ispezionato
per stack; high-water e max_release_work reali restano da misurare sul robot.

## Qualifica successiva

GO alla revisione pre-flash offline se build/manifest/layout/audit finali sono
positivi. Nessuna autorizzazione a flash o accettazione hardware implicita.
Il contatto fisico resta NON VERIFICATO: encoder e fase nominale non misurano
supporto, attrito, scivolamento, touchdown anticipato, stabilità statica o carico.
L'avanzamento è conseguenza dello STAND richiesto dall'operatore, mai di STOP.

Collaudo successivo, previa autorizzazione separata: verifica identità APP;
robot sostenuto, STOP in stance e swing e attesa immobile prima di STAND;
poi a terra con spazio libero e sostegno pronto, passo45 nelle otto finestre,
quindi passo60. Solo dopo: STOP durante WALK STOP e WALK_SETTLE, secondo STOP
nel recupero e nuova richiesta STAND, quote richieste, SAFE_OFF e regressione
STAND/PRONE/WALK STOP. Registrare RAW, motivo STAND_REACHED, coppia, IMU,
feedback, max_release_work, missed release e minimo libero stack; interrompere
su fault reale o appoggio/moto inatteso. Non generare fault distruttivi sul robot.
