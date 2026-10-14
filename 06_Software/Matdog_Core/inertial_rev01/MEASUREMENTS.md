# MATDOG REV01 — template operativo di acquisizione

Stato iniziale: PENDING MEASUREMENT, HARDWARE UNVERIFIED. Questo documento prepara
i dati che l’operatore fornirà in seguito. Nessuna acquisizione hardware viene
eseguita dal codice o da questa revisione.

Compilare `data/measurement_intake_template.json` in una copia di lavoro, allegando
gli export originali. Conservare sempre valore grezzo, unità, configurazione,
metodo, incertezza, data/operatore, revisione e SHA-256 dell’evidenza. Separare il
CAD dalla misura fisica. Un dato non disponibile resta `PENDING_MEASUREMENT`.

## 1. Identificare la configurazione

| Informazione da fornire | Stato |
|---|---|
| Revisione CAD Solid Edge e configurazione/esploso selezionato | PENDING_MEASUREMENT |
| Inventario installato: schede, BNO085, harness, supporti, antenna, fusibile, contatti, viteria, accessori | PENDING_MEASUREMENT |
| Batteria presente/assente, alimentazione esterna e cavi inclusi nella pesata | PENDING_MEASUREMENT |
| Neck yaw: presente/assente, parti già incluse e rigidità/articolazione rispetto al body | PENDING_MEASUREMENT |
| Modifiche ai singoli link rispetto a REV00, con distinta disgiunta | PENDING_MEASUREMENT |
| Configurazione del robot durante pesata e CAD, stessa posa e stessa copertura | PENDING_MEASUREMENT |

La voce elettronica non deve inglobare il BNO085, cablaggi o portafusibile quando
questi sono già misurati in righe autonome. Dividere `other_components` in nuove
righe con ID unici per parti indipendenti. Un componente assente necessita di
evidenza dell’assenza; assente e ignoto non sono equivalenti.

## 2. Export Solid Edge per-link

Fornire 17 report, uno per ogni link indicato nel JSON. In ogni report richiedere:

| Campo | Unità/convenzione richiesta |
|---|---|
| Massa dell’assieme completo | kg, senza arrotondare a tre decimali |
| CoM | m nel frame del relativo link REV00 |
| Origine, X/Y/Z e registrazione del frame CAD → URDF | Frame destrorso; X avanti, Y sinistra, Z alto per base |
| Tensore completo al CoM | 3×3 kg·m² negli assi del link; indicare origine del tensore |
| Se vengono esportati assi/momenti principali | Momenti kg·m², oppure raggi di girazione con unità esplicita; matrice di orientamento e convenzione righe/colonne |
| Composizione dell’assieme | Parti/viteria/elettronica incluse ed esclusioni nominate |
| Materiali/densità o masse assegnate nel CAD | Valori solo se forniti e verificati; evidenza della densità/override massa |
| Report originale e hash | Nome file, revisione, data, SHA-256 |

Per nuovi accessori fornire anche parent di montaggio, CoM espresso nel parent e
orientamento del tensore rispetto al parent. Non serve inventare una mesh per
calcolare le proprietà se il CAD produce direttamente massa, CoM e tensore.

Se l’export è in mm: coordinate ×0,001; kg·mm² ×0,000001. Se i momenti sono
g·mm²: ×0,000000001. Non applicare la scala STL al tensore URDF già in SI.
Una densità sconosciuta non è un motivo per scegliere un materiale plausibile.

Con colonne di R uguali agli assi principali espressi nel link:

```text
I_link_at_com = R_link_from_principal diag(J1,J2,J3) R_link_from_principal^T
Jk = m Kk²  (K in metri)
```

Se il CAD memorizza gli assi sulle righe si usa la trasposta appropriata.
Verificare ortonormalità e destrorsità della matrice; chiedere più precisione per
assi arrotondati. I prodotti di inerzia CAD non sono automaticamente gli elementi
fuori diagonale del tensore. Riportare la convenzione e verificare segni/principali
con un export non arrotondato. Preferire il tensore al CoM per evitare cancellazione
numerica quando si trasla da un’origine lontana.

Le copie/simmetrie LF→RF/RH/LH storiche non sono prova che cablaggi e accessori reali
siano simmetrici. Per REV01 acquisire i link separatamente o documentare l’esatta
identità con evidenza. La riflessione storica Y usa M=diag(1,−1,1) e I'=M I M^T;
non è una rotazione fisica né una riflessione X per le gambe posteriori.

## 3. Export CAD dell’assieme completo

Fornire la posa semantica dei 12 joint in radianti URDF, la massa, CoM base e
tensore al CoM in assi base. La verifica per-link/assi paralleli deve essere
confrontata col CAD completo nella stessa posa e configurazione. Conservare le
differenze e le tolleranze motivate dalla precisione dell’export, senza scegliere
una soglia che assorba una distinta incompleta.

I 183 g fra il CAD storico e la vecchia pesata non si distribuiscono uniformemente.
Occorre una riconciliazione nominativa di parti assenti, presenti, sostituite o
misurate diversamente. Non riscalare globalmente CoM o inerzie.

## 4. Dati del robot assemblato, da fornire dall’operatore

| Dato | Dettaglio richiesto |
|---|---|
| Massa totale ripetuta | Letture grezze, unità, numero di ripetizioni, bilancia/risoluzione/accuratezza, tara e cavi sostenuti esternamente |
| Masse dei componenti modificati/aggiunti | Montaggio effettivo, viteria/connessioni comprese, copertura disgiunta |
| Posa e frame di riferimento | Joint semantici rad, origine/assi base e registrazione nel world di misura |
| Contatti di supporto reali | Posizioni e identificazione del contatto effettivo; non inferire dal solo foot_link |
| Reazioni statiche | Forze/letture per supporto, ripetizioni, piano/orientamento e carichi esterni |
| CoM X/Y indipendente | Metodo, letture, registrazione CAD e incertezza |
| CoM Z indipendente | Metodo specifico e incertezza; una sola pesata su piano orizzontale non osserva la quota |
| Cablaggi e accessori | Pose reali e distribuzione; parti flessibili/configurazioni variabili dichiarate |
| Confronto CAD ↔ reale | Scarti per-link/totali e motivazione della configurazione |

Per dati statici già acquisiti su un piano orizzontale, senza carichi esterni e
con reazioni verticali F_i, il controllo del CoM proiettato è
`C_xy = Σ(F_i p_i_xy) / ΣF_i`. Non determina C_z o il tensore inerziale. I contatti
G2 nominali e una lettura IMU non sostituiscono questi dati. Il template non
autorizza movimenti, flash, alimentazione, prove in carico o accesso seriale.

## 5. Trascrivere e rieseguire offline

1. Compilare la copia del template di misure, mantenendo separati CAD e robot.
2. Compilare la copia del budget: disposition, parent, campi SI, evidence e
   coverage_note per ciascuna riga; review di tutti i 17 link e inventario globale.
3. Rimuovere dal computo solo righe esplicitamente assenti o già incluse con
   evidenza. Usare `replace_link` per modifiche di assiemi storici.
4. Eseguire il runner con `--budget` e `--report-dir` verso nuove directory.
5. Esaminare massa/CoM/inerzia, differenze CAD/robot, sensibilità e margini per posa.

Il risultato numerico completo resta `REV01_PENDING`: nessun script imposta
`REV01_VERIFIED`. La futura qualifica richiede riconciliazione delle misure reali,
provenienza completa, approvazione della revisione, osservabilità del supporto e
gestione dei timestamp compatibile con G5-B. Queste attività sono ancora
PENDING MEASUREMENT e HARDWARE UNVERIFIED.
