# OV7670 → ESP32-C3 Super Mini → PC : video 100×100 a colori

Due programmi:

| file | cosa fa |
|---|---|
| `firmware/ov7670_stream/ov7670_stream.ino` | acquisisce i frame dalla OV7670 in bit-banging, li riduce a 100×100 RGB565 e li spedisce sulla USB CDC |
| `viewer/ov7670_viewer.py` | legge la seriale, ricostruisce i frame e li mostra in una finestra sul PC |

---

## 1. Cablaggio

| OV7670 | ESP32-C3 | note |
|---|---|---|
| SCL / SIOC | GPIO20 | anche RX0 di UART0 |
| SDA / SIOD | GPIO21 | **condiviso con D0**, anche TX0 di UART0 |
| VSYNC | GPIO6 | |
| HREF | GPIO7 | |
| PCLK | GPIO1 | |
| XCLK | GPIO0 | generato dall'ESP32 con LEDC |
| D7 | GPIO5 | |
| D6 | GPIO4 | |
| D5 | GPIO3 | |
| D4 | GPIO10 | |
| D3 | GPIO9 | **strapping pin** |
| D2 | GPIO8 | **strapping pin** |
| D1 | GPIO2 | **strapping pin** |
| D0 | GPIO21 | **condiviso con SIOD** |
| 3V3 / GND | 3V3 / GND | |

### D0 e SIOD sullo stesso pin

Tutta la configurazione SCCB avviene in `setup()`. Subito dopo il firmware
chiama `Wire.end()` e riporta GPIO21 a ingresso puro, così la linea diventa D0
e la cattura può partire. Non ci sono altre scritture SCCB durante lo streaming.

I comandi runtime (mirror, flip, luminosità, contrasto, color bar, reinit)
riaprono l'I2C solo per il tempo delle poche scritture necessarie
(`applyToCamera()` in `handleCommands()`) e poi ripristinano D0. In quel breve
istante il driver push-pull di D0 della camera e la linea SCCB open-drain si
contendono il pin: dura qualche millisecondo e in pratica funziona, ma se vuoi
essere tranquillo metti **una resistenza da 1 kΩ in serie fra D0 della camera e
GPIO21**. A 3 MHz di PCLK non disturba la lettura e limita la corrente di
contesa. Se preferisci non correre rischi, semplicemente non usare quei comandi.

### GPIO2 / GPIO8 / GPIO9 sono strapping pin

Sono usati come D1/D2/D3. All'accensione l'ESP32-C3 li campiona per decidere la
modalità di boot: se la camera li tiene bassi durante il reset la scheda può
finire in *download mode* e non partire. Se succede:

* alimenta la camera **dopo** il boot dell'ESP32 (per esempio con il 3V3 della
  camera su un pin GPIO libero o con un interruttore), oppure
* metti resistenze in serie da 1 kΩ su D1/D2/D3, oppure
* tieni GPIO9 a 3V3 con un pull-up da 10 kΩ (più forte dell'uscita della camera
  non è, ma spesso basta il pull-up interno più la serie).

### UART0 non è utilizzabile

GPIO20/21 sono i pin di UART0, quindi la console è su **USB CDC**. Il firmware
chiude UART0 (`Serial0.end()`) per liberare i pin. I messaggi del bootloader
ROM sulla GPIO21 durante il reset non si possono evitare, ma sono innocui.

---

## 2. Firmware: compilazione

Arduino IDE, core **ESP32 by Espressif 3.x**:

* Board: `ESP32C3 Dev Module` (o `ESP32-C3 Super Mini`)
* **USB CDC On Boot: Enabled** ← obbligatorio
* CPU Frequency: **160 MHz** (il loop di cattura è tarato su questa)
* Flash Size / Partition Scheme: i default vanno bene

Se `USB CDC On Boot` resta disabilitato lo sketch compila comunque (usa un
oggetto `HWCDC` proprio) ma emette un warning.

### Come funziona la cattura

L'ESP32-C3 non ha la periferica parallela LCD_CAM, quindi il bus dati va letto
in bit-banging. Le scelte che rendono la cosa affidabile:

1. **XCLK = 12 MHz** generato con LEDC. La camera è in QQVGA (160×120) RGB565
   con divisore PCLK /4, quindi **PCLK = 3 MHz**: ~333 ns per byte, cioè ~53
   cicli di CPU a 160 MHz. Il loop di lettura ne usa una ventina.
2. **PCLK, D0…D7, HREF e VSYNC stanno tutti nei GPIO 0…21**, quindi una singola
   lettura del registro `GPIO_IN` restituisce insieme il clock e il dato. La
   macro `BUS_BYTE()` ricompone il byte in 11 istruzioni sfruttando il fatto che
   D2…D4 sono su GPIO8…10 consecutivi e D5…D7 su GPIO3…5 consecutivi.
3. **`COM10` bit 5**: il PCLK non oscilla durante il blanking orizzontale, così
   ogni fronte di salita è un byte valido e la sincronizzazione è banale.
4. **Mascheramento predittivo delle interruzioni.** In QQVGA il periodo di HREF
   è ~523 µs di cui solo ~107 µs sono dati. Spegnere le interruzioni per tutta
   l'attesa di HREF le terrebbe spente per l'80% del tempo e affamerebbe l'USB;
   aspettare HREF a interruzioni attive fa invece perdere i primi byte ogni
   volta che una ISR è in esecuzione quando HREF sale (~2% delle righe →
   righe visibilmente strappate). Il firmware quindi **misura il periodo di
   HREF** e spegne le interruzioni solo 40 µs prima del fronte previsto:
   interruzioni spente il 28% del tempo e nessuna riga strappata. Entrambi gli
   estremi della misura sono presi a interruzioni spente, quindi la stima non ha
   jitter da ISR; parte bassa e cresce al primo valore misurato (sottostimare
   costa solo una sezione critica più lunga, mai una riga persa). Se la stima
   diventasse troppo grande, trovare HREF già alto è un segnale netto: il frame
   viene scartato e la stima tagliata del 12.5%, con rientro in 3-4 frame.
   Ogni loop di attesa ha un contatore di guardia, quindi una camera scollegata
   non blocca il chip. Le stime correnti si leggono col tasto `i`
   (`gapHref`, `gapFirst`).
5. **Riduzione a 100×100 riga per riga**, durante il blanking orizzontale
   (~24 µs disponibili, ne servono ~6): serve solo un buffer di riga da 320 byte
   più i 20 000 byte del frame di uscita, non un frame QQVGA intero.
6. **Trasmissione sovrapposta alla cattura**: il ring buffer di TX della CDC è
   da 28 KB, quindi un frame ci entra tutto; `Serial.write()` ritorna subito e
   l'ISR USB svuota il buffer mentre il frame successivo viene acquisito. Se il
   PC non legge abbastanza in fretta il frame viene **scartato** invece di
   bloccare la cattura: lo stream resta integro e la latenza bassa.

### Parametri regolabili (in cima allo sketch)

| define | default | significato |
|---|---|---|
| `XCLK_HZ` | `12000000` | 12 MHz → PCLK 3 MHz, ~15 fps. 16 MHz → ~20 fps, 20 MHz → ~25 fps (limite del bit-bang e della banda USB) |
| `CLKRC_PRESCALER` | `0x00` | divisore interno della camera: `0x01` dimezza tutto |
| `CROP_SQUARE` | `1` | `1` = ritaglia le 120 colonne centrali → pixel quadrati, nessuna deformazione. `0` = usa tutti i 160 px → FOV pieno ma immagine schiacciata |
| `MVFP_BASE` | `0x07` | `+0x20` mirror, `+0x10` flip verticale |
| `TX_RING_BYTES` | `28 KB` | ring buffer di trasmissione USB |

Alzare `XCLK_HZ` aumenta il frame rate ma riduce il margine del loop di
lettura: se compaiono righe sfasate o colori sbagliati, torna a 12 MHz.

---

## 3. Viewer Python

```bash
pip install -r viewer/requirements.txt        # pyserial e numpy bastano
python viewer/ov7670_viewer.py                # porta rilevata automaticamente
python viewer/ov7670_viewer.py --list         # elenca le porte
python viewer/ov7670_viewer.py --port COM7 --scale 5
```

La porta viene cercata per VID Espressif (`0x303A`), poi fra le CDC native
(`ttyACM*`, `usbmodem*`). Il viewer **non tocca DTR/RTS**, altrimenti la CDC
nativa del C3 rischia di far ripartire il chip in bootloader.

Un thread legge la seriale, verifica i checksum, decodifica RGB565 → RGB888 con
numpy e tiene pronto **solo il frame più recente**: se la GUI è più lenta della
camera si salta avanti invece di accumulare latenza (contatore `tardivi`).

Backend grafico: OpenCV se installato (più fluido), altrimenti Tkinter con
Pillow, altrimenti Tkinter puro via PPM+base64 — quindi funziona anche con solo
`pyserial` e `numpy`.

### Tasti

| tasto | effetto |
|---|---|
| `q` / `ESC` | esci |
| `w` | salva l'immagine corrente in `snapshots/` |
| `m` | mirror orizzontale |
| `f` | flip verticale |
| `b` | color bar di test della camera |
| `+` `-` | luminosità |
| `[` `]` | contrasto |
| `p` | pausa/riprendi lo streaming |
| `i` | stampa lo stato del firmware sul terminale |

### Opzioni

`--scale N` zoom · `--backend auto|cv2|tk` · `--swap` inverte l'ordine dei byte
RGB565 · `--no-osd` niente testo sull'immagine · `--stats S` intervallo delle
statistiche sul terminale (`0` per disattivarle) · `--save-dir DIR`.

---

## 4. Protocollo

Ogni pacchetto: header di 20 byte, payload, checksum del payload a 16 bit.
Interi little-endian, checksum = somma dei byte troncata a 16 bit.

| offset | size | campo |
|---|---|---|
| 0 | 4 | magic `"OVC1"` |
| 4 | 1 | versione = 1 |
| 5 | 1 | tipo: 1 = frame, 2 = testo |
| 6 | 1 | formato: 1 = RGB565 big-endian |
| 7 | 1 | riservato |
| 8 | 2 | width |
| 10 | 2 | height |
| 12 | 4 | numero di sequenza |
| 16 | 2 | lunghezza payload |
| 18 | 2 | checksum dei byte 0…17 |
| 20 | N | payload |
| 20+N | 2 | checksum del payload |

Un frame 100×100 RGB565 occupa 20 022 byte in tutto. Il numero di sequenza è il
contatore dei frame **acquisiti**, quindi dai buchi il viewer capisce quanti
frame sono stati scartati. Il parser si risincronizza sempre cercando il magic e
scartando gli header con checksum sbagliato, quindi anche riattaccando la
seriale a metà frame riparte da solo.

I pacchetti di tipo 2 (testo) sono i messaggi diagnostici del firmware e vengono
stampati sul terminale: usano lo stesso framing per non rompere il flusso
binario.

---

## 5. Se qualcosa non va

**"OV7670 NON rilevata via SCCB"** (messaggio all'avvio) — controlla SIOC/SIOD,
l'alimentazione a 3V3 e che il modulo abbia i pull-up. Alcuni cloni non
rispondono alla lettura dei registri pur accettando le scritture: se l'immagine
arriva comunque, ignora il messaggio.

**Nessun frame, contatore `bad` che sale** — manca XCLK (controlla GPIO0) oppure
VSYNC/HREF non sono collegati. Prova `i` per lo stato.

**Immagine con i colori invertiti / verdastra** — l'ordine dei byte RGB565 è
sfasato di uno in modo uniforme: prova `--swap`.

**Righe alternate con colori sbagliati** — il tuo modulo ignora `COM10` bit 5 e
il PCLK continua a oscillare durante il blanking, quindi l'allineamento cambia
riga per riga. Prova a scrivere `COM10 = 0x00` e ad abbassare `XCLK_HZ`, oppure
sostituisci il modulo: senza il gating del PCLK il bit-banging non ha un
riferimento affidabile.

**Righe sfasate o "strappate"** — il loop di lettura non sta al passo: abbassa
`XCLK_HZ` e verifica che la CPU sia a 160 MHz. Con `i` controlla `gapHref`: a
XCLK 12 MHz deve stabilizzarsi intorno a 520 µs (scala inversamente con
`XCLK_HZ`); se oscilla molto, alza `GUARD_US` nello sketch.

**Tutto nero o tutto bianco** — è l'esposizione automatica: aspetta un paio di
secondi, poi regola con `+`/`-`. Premi `b` per le color bar: se le barre sono
nitide e stabili, il bus dati e la catena di trasmissione sono a posto e il
problema è ottico o di esposizione.

**Frame rate basso con molti `tardivi`** — la GUI o l'USB non stanno al passo:
riduci `--scale`, installa OpenCV, oppure abbassa `XCLK_HZ`.
