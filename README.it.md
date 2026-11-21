# goodix-milan-driver

*[English](README.md)*

Driver Linux per il lettore di impronte SPI **Goodix GXFP3200** (famiglia Goodix "Milan F",
ID chip `0x2202xx`) montato sui portatili **ASUSPRO P5440FA / ExpertBook P5440**, che finora
funzionava solo su Windows.

È un driver per [libfprint](https://fprint.freedesktop.org/), caricato come modulo TOD dalla
libfprint di Ubuntu: tutto il resto funziona senza modifiche, cioè `fprintd`, le Impostazioni
di GNOME (Accesso con impronta digitale), la schermata di blocco, GDM e `sudo` tramite
`pam_fprintd`.

> **Stato: sperimentale.** Usato ogni giorno sul portatile dell'autore. Provato su
> Ubuntu 26.04 con libfprint 1.95.1+tod1. Le segnalazioni da altri computer sono benvenute.

## Hardware supportato

Per sapere se il tuo portatile ha questo sensore:

```
ls /sys/bus/spi/devices/ | grep GXFP3200
```

Se compare `spi-GXFP3200:00`, il driver dovrebbe funzionare. All'avvio il driver controlla
l'ID del chip e accetta solo `0x2202xx` (sensore da 108 x 88 pixel). Gli altri chip Milan
sono descritti in [docs/PROTOCOL.md](docs/PROTOCOL.md) ma non sono stati provati.

## Installazione (Ubuntu / Debian)

```
git clone https://github.com/enBeg-Code/goodix-milan-driver.git
cd goodix-milan-driver
sudo ./install.sh                # oppure: sudo ./install.sh --pam-retries
```

Lo script installa le dipendenze (`libfprint-2-tod-dev`, `fprintd`, `libpam-fprintd`),
compila e installa il modulo nella cartella TOD di libfprint, aggiunge una regola udev che
collega `spidev` al sensore a ogni avvio, riavvia `fprintd` e attiva l'impronta in PAM.

Ubuntu concede un solo tentativo con l'impronta prima di chiedere la password. Con
`--pam-retries` lo script installa un profilo PAM che ne concede 3; è consigliato, perché un
sensore così piccolo rifiuta circa un appoggio su dieci.

Poi registra un dito:

```
fprintd-enroll -f right-index-finger      # 12 appoggi, spostando un po' il dito ogni volta
fprintd-verify
```

oppure usa *Impostazioni → Utenti → Accesso con impronta digitale*.

Le distribuzioni la cui libfprint non supporta i moduli TOD (per esempio Fedora e Arch)
richiedono di compilare il driver dentro libfprint: per ora non è disponibile.

## Disinstallazione

```
sudo ./uninstall.sh
```

Le impronte registrate restano in `/var/lib/fprint` finché non le cancelli con
`fprintd-delete <utente>`.

## Come funziona

- **Protocollo**: semplici letture e scritture di registri via SPI; un'immagine si acquisisce
  con una breve sequenza di comandi e si legge da una coda a blocchi di 4 KiB, quindi basta
  il buffer predefinito di `spidev`. Descrizione completa in [docs/PROTOCOL.md](docs/PROTOCOL.md).
- **Immagini**: 108 x 88 pixel a 12 bit; il dito si ottiene come differenza tra un frame e
  un'immagine di riferimento del sensore vuoto, aggiornata ogni volta che il sensore è libero.
- **Riconoscimento**: il sensore è troppo piccolo per il riconoscimento a minuzie di libfprint
  (NBIS trova 1-2 minuzie per immagine), quindi il driver ne ha uno proprio
  (`src/gm-match.c`, in C puro): punti caratteristici di tipo ORB con verifica geometrica
  RANSAC. Il modello è fatto di 12 immagini; una verifica richiede circa 0,1 s.
- **Precisione** (una persona, 4 dita, 50 immagini): nessun falso accettato, circa 7-12% di
  falsi rifiuti per appoggio. Servono più dati: vedi *Contribuire*.

## Sicurezza

Lo sblocco con l'impronta è una comodità. Il driver non rileva dita finte e la sua
precisione è stata misurata su pochi dati. La password resta sempre utilizzabile in
alternativa. I modelli delle impronte sono salvati da `fprintd` in `/var/lib/fprint`,
leggibili solo da root.

## Risoluzione dei problemi

- `journalctl -u fprintd -b` mostra i messaggi del driver; per più dettagli:
  `sudo systemctl stop fprintd` e poi `sudo G_MESSAGES_DEBUG=all /usr/libexec/fprintd`.
- `ls /sys/bus/spi/devices/spi-GXFP3200:00/spidev` deve mostrare `spidev0.0` (o simile).
  Se non c'è, riapplica la regola:
  `sudo udevadm trigger --action=add /sys/bus/spi/devices/spi-GXFP3200:00`.
- Un solo programma alla volta può usare il sensore: ferma `fprintd` prima di usare gli strumenti.

## Contribuire

In `tools/` ci sono script Python per provare il sensore, raccogliere immagini e valutare il
riconoscimento (`pip install numpy opencv-python-headless pillow`):

```
sudo systemctl stop fprintd
sudo python3 tools/gmsensor.py                 # ID del chip e un frame
sudo python3 tools/collect.py right-index 20   # raccolta guidata in ./dataset
make libgmmatch.so && python3 tools/eval_matcher.py dataset
```

Le immagini raccolte sono dati biometrici: tienile private e non allegarle a issue o pull
request. Sono invece molto utili i numeri di precisione prodotti da `eval_matcher.py`.

Possibili sviluppi: rilevare il dito con la modalità a interrupt del sensore (FDT) invece del
polling, calibrazione da OTP, supporto ad altri chip Milan, integrazione del driver in
libfprint.

## Riconoscimenti

Sviluppato da **enBeg**, che ha guidato il progetto, messo a disposizione l'hardware e
provato ogni passaggio sul portatile reale.

Il reverse engineering, l'analisi del protocollo e la maggior parte del codice sono stati
realizzati con l'aiuto di **[Claude](https://www.anthropic.com/claude)**, un assistente di
intelligenza artificiale sviluppato da Anthropic, lavorando in modo interattivo con enBeg.
Lo dichiariamo apertamente perché chi usa o esamina il progetto sappia come è stato fatto;
come per qualsiasi codice, rivedilo prima di fidarti.

## Sostieni il progetto

Se questo driver ti è utile, puoi sostenere il progetto con una donazione:

[![Dona con PayPal](https://img.shields.io/badge/Dona-PayPal-blue.svg)](https://paypal.me/enbeg)

Le donazioni sono del tutto libere e non comportano alcun obbligo per nessuna delle due
parti: non danno diritto ad assistenza, nuove funzioni o garanzie. Il driver è e resterà
gratuito.

## Note legali

Questo progetto non è affiliato, autorizzato né approvato da Goodix o ASUS. Il nome "Goodix"
è usato solo per identificare l'hardware. Il protocollo è stato ricavato osservando il
comportamento del dispositivo e del driver Windows al solo scopo di ottenere
l'interoperabilità (direttiva UE 2009/24/CE, art. 6). Il repository non contiene codice,
firmware o file binari dei produttori.

## Licenza

[LGPL-2.1-or-later](LICENSE), la stessa licenza di libfprint.
