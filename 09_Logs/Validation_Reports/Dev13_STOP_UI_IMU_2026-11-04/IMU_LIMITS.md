# STAND IMU: comportamento verificato e limite del livellamento

Base operativa installata: dev.12 4d0679cfc652, APP94e6e15e9b8d.
VERIFIED OFFLINE, nessuna nuova prova hardware. Nessuna estensione runtime
IMU è inclusa: il livellamento completo richiesto NON è implementato.

Rotation Vector BNO08550Hz → snapshot normalizzato (normtol.001, age<=200ms,
task vivo) → roll/pitch da R^T ez, stessi assi base_link, nessun offset →
SimpleMotion::compensate → body pose → IK quattro contatti → resolver RAW.
HOLD esegue questo percorso ogni20ms partendo da holdNominal_, non dal target
già compensato. Il controllo è chiuso sul tilt misurato ma applica un OFFSET
proporzionale assoluto: Kp.5, deadband.005rad, cap±.020rad, slew.020rad/s,
inserimentoC2 1s e attenuazione geometrica (room<=.025rad). WALK resta identico.

Il segno oppone l'errore. Nel modello ideale piccolo-angolo theta=beta+c,
c=-.5*alpha*(theta-d), quindi theta=(beta+.5*alpha*d)/(1+.5*alpha): resta
almeno circa2/3 della perturbazione senza saturazione, di più con attenuazione.
Il comportamento attuale è stabilizzazione limitata, non annullamento del tilt.
Nei test di produzione, input IMU±.01..±.16rad a50/100/150mm produce massimi
effettivi roll .004951/.005139/.003942rad, pitch .002740/.003022/.002205rad.
Passo RAW massimo osservato1tick, entro limite test10tick/20ms. Stimolo lento
.04*sin(n*.004)rad, nessun fault nel modello scriptato, ritorno al riferimento
orizzontale esatto, nessun accumulo integrale o doppio release allo stesso tempo.
Queste prove non sono un modello dinamico del robot o della tavola.

La legge corrente mantiene anchor canonici sul piano mondiale z0 e il wrapper
worldContactInverseKinematicsTilted assume normale mondiale+Z. Un piano inclinato
ha normale diversa; una tavola mobile cambia anche gli anchor mondiali. La sola
IMU del corpo non misura questi contatti. Non è giustificato aumentare guadagno,
cap o room, né accumulare errori per simulare un livellamento non dimostrato.

Envelope separato con IK di produzione (tol1e-6/8iter), limiti gen1 e kHeight,
continuazione0.01gradi, body livellato, quattro anchor ruotati rigidamente
attorno all'origine della tavola e altezza verticale invariata:

| Quota mm | Roll gradi | Pitch gradi | Diagonale simmetrica, componente gradi |
|---|---|---|---|
| 50 | ±.92 | -.97..+.63 | .37 |
| 100 | ±13.50 | -16.68..+19.55 | vedere envelope completo |
| 150 | ±.66 | -.57..+.47 | .27 |

A100mm il contatto cilindrico esce da NOMINAL_STRIP oltre circa±1.97gradi di
roll: il range IK maggiore non è un appoggio nominale qualificato. L'envelope
body-tilt/anchor orizzontali è diverso (roll±1.33/1.84/1.05gradi rispettivamente).
Sono percorsi geometrici campionati, NON range autorizzati su hardware, né
limiti universali o prova di attrito/supporto. Geometria esatta installata
invariata, non rigenerata da un URDF storico. Nominale50 è dentro entrambi
kHeight/kLift: l'ipotesi iniziale di un bug box HOLD50 è stata smentita e la
patch proposta è stata ritirata prima del commit.

Minimo seguito tecnico: fornire al compensatore esistente normale e quattro
anchor coerenti e verificati del piano; riusare l'IK ground-normal già parametrica,
aggiornando gli anchor se il piano si muove. Poi provare il circuito e il budget
reale sotto carico. Nessun secondo controller d'assetto o import G5-B/G5-C.

Evidenze esterne: build-output/dev13-stop-ui-stand-20261104/evidence/imu/
e imu-simple-final.log. Suite esistente SimpleMotion ASan/UBSan/Werror PASS;
STAND, STOP, WALK, PRONE e stale/fault/preemption preservati. L'IK installata
ha già Jacobiano analitico e riuso residuale; header storico central-difference
non descrive più l'implementazione. Nessuna patch solver sperimentale importata.
