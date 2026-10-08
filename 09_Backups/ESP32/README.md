# MATDOG — backup integrali ESP32

Il metodo standard per i backup integrali della flash è una copia cifrata
GPG/AES-256 archiviata come asset versionato su GitHub Releases del repository
pubblico `MattRobotics/robot-dog`. Gli originali A/B rimangono locali e immutati.
Un supporto fisico locale distinto resta un'alternativa valida. Una seconda
partizione/directory/filesystem dello stesso dispositivo fisico non è una
copia indipendente.

## Cifratura e verifica locale

1. Riutilizzare un'acquisizione A/B già verificata, senza riacquisire flash
   per pubblicare il backup. Lavorare in una directory di sessione esterna
   al checkout; non copiare mai immagini flash in chiaro nel repository.
2. Cifrare `read-a-16m.bin` con GPG simmetrico, AES-256, S2K iterato/salato
   SHA-256. Inserire la password esclusivamente tramite pinentry interattivo
   sicuro sul desktop dell'operatore; conservarla sotto il suo controllo.
   Non usare argomenti password, variabili d'ambiente, file password, chat,
   stdin di tool Codex o log. Disabilitare la cache delle password simmetriche.
3. Decifrare tramite lo stesso pinentry e confrontare l'intero flusso recuperato
   con l'originale, verificando anche dimensione e SHA-256. Verificare l'esito
   GPG, compresa l'integrità del messaggio. È possibile consumare lo stdout
   binario in un verificatore locale senza scrivere un file flash recuperato.
   Non stampare byte recuperati nei log.
4. Creare un manifest senza segreti: identificativo/data dell'acquisizione,
   firmware installato, dimensione/hash originale, dimensione/hash cifrato,
   metodo e parametri di cifratura, riferimento GitHub.

## Pubblicazione e seconda copia verificata

Pubblicare soltanto ciphertext e manifest in una Release dedicata con tag
univoco; il tag è un indice dell'archivio, non identifica automaticamente il
firmware contenuto nella flash. Identità del firmware e checkpoint Git sono
espliciti nel manifest. Non sostituire asset esistenti con `--clobber`.

Dopo la pubblicazione, confermare Release e asset tramite GitHub, scaricare
entrambi in una nuova directory esterna al checkout e confrontare tutti i
byte con i file caricati. Per il ciphertext verificare dimensione e SHA-256,
poi decifrare la copia scaricata e ripetere il confronto byte/dimensione/hash
con l'originale. Un upload non confermato o una decifratura fallita impongono
STOP; non dichiarare un backup verificato.

Registrare un receipt con URL Release/asset, identificativi remoti, dimensioni,
SHA-256 del ciphertext remoto, SHA-256 del plaintext recuperato, esiti e data
UTC della verifica. Versionare manifest, receipt e documentazione, senza
password/chiavi, directory private GPG o file temporanei. La disponibilità
della password resta responsabilità dell'operatore: il solo ciphertext senza
credenziale di recupero disponibile non è una copia recuperabile.

Se Releases non è disponibile, il fallback autorizzato è un ciphertext in
questa directory versionata con manifest e receipt, verificato dopo download
dal remoto con gli stessi controlli. Non inserire mai backup in chiaro nella
cronologia Git, neppure temporaneamente.

Il receipt positivo soddisfa il requisito di seconda copia indipendente
dall'ASUS. Non autorizza ingresso ROM, scrittura, reset o movimento. La
continuità dell'alimentazione e le evidenze correnti sulle periferiche restano
gate separati del runbook M0.4.
