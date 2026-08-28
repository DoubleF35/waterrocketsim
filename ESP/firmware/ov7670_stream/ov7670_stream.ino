/* ============================================================================
 *  OV7670 -> ESP32-C3 Super Mini -> USB CDC : streaming video 100x100 RGB565
 * ============================================================================
 *
 *  L'ESP32-C3 non ha la periferica LCD_CAM/I2S parallela, quindi il bus dati
 *  della camera viene letto in bit-banging. Per farlo in modo affidabile:
 *
 *   - XCLK viene generato con LEDC a XCLK_HZ (default 12 MHz);
 *   - la camera e' configurata in QQVGA (160x120) RGB565: il PCLK esce diviso
 *     per 4, cioe' XCLK/4 = 3 MHz -> ~333 ns per byte, ~53 cicli di CPU a
 *     160 MHz. Larghissimo margine per il loop di lettura;
 *   - PCLK e' bloccato durante il blanking orizzontale (COM10 bit 5) cosi'
 *     ogni fronte di salita di PCLK corrisponde ad un byte valido;
 *   - le interruzioni sono disabilitate solo per la durata di una riga
 *     (~130 us), non per l'intero frame: lo stack USB continua a girare;
 *   - PCLK e tutti i D0..D7 stanno nei GPIO 0..21, quindi una sola lettura
 *     del registro GPIO_IN restituisce insieme il clock e il dato.
 *
 *  CABLAGGIO (come richiesto)
 *  -------------------------
 *    SCL/SIOC -> GPIO20        D7 -> GPIO5      D3 -> GPIO9
 *    SDA/SIOD -> GPIO21        D6 -> GPIO4      D2 -> GPIO8
 *    VSYNC    -> GPIO6         D5 -> GPIO3      D1 -> GPIO2
 *    HREF     -> GPIO7         D4 -> GPIO10     D0 -> GPIO21  (!)
 *    PCLK     -> GPIO1
 *    XCLK     -> GPIO0
 *
 *  ATTENZIONE 1 - D0 e SIOD condividono GPIO21. Tutta la configurazione SCCB
 *  avviene in setup(); subito dopo Wire viene chiuso (Wire.end()) e GPIO21
 *  diventa un ingresso puro per D0. Per i comandi runtime che devono
 *  riscrivere i registri esiste sccbSession(), che riapre l'I2C, scrive e
 *  ripristina D0 (vedi cmdApply()).
 *
 *  ATTENZIONE 2 - GPIO20/21 sono i pin di UART0. La console e' quindi su USB
 *  CDC: in Arduino IDE selezionare "USB CDC On Boot: Enabled". Il codice
 *  chiude comunque UART0 per liberare i pin.
 *
 *  ATTENZIONE 3 - GPIO2, GPIO8 e GPIO9 sono strapping pin del C3 e qui sono
 *  usati come D1, D2, D3. Se la camera li tiene bassi durante il reset la
 *  scheda puo' entrare in download mode. Vedi README (resistenze in serie
 *  oppure alimentare la camera dopo il boot).
 *
 *  Board: "ESP32C3 Dev Module" / ESP32-C3 Super Mini, core ESP32 Arduino 3.x
 *  ========================================================================= */

#include <Arduino.h>
#include <Wire.h>
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "soc/gpio_reg.h"

/* ---------------------------------------------------------------- parametri */

#define XCLK_HZ            12000000UL  /* 12 MHz -> PCLK 3 MHz, ~15 fps.
                                        * 16 MHz -> PCLK 4 MHz, ~20 fps
                                        * 20 MHz -> PCLK 5 MHz, ~25 fps (al
                                        * limite del bit-bang e della banda
                                        * USB: alzare solo se serve).        */
#define CLKRC_PRESCALER    0x00        /* divisore interno: 0x00=/1, 0x01=/2 */

#define OUT_W              100
#define OUT_H              100
#define CROP_SQUARE        1           /* 1 = ritaglia il centro 120x120 del
                                        * sensore (pixel quadrati, nessuna
                                        * deformazione, FOV orizzontale
                                        * ridotto).
                                        * 0 = usa tutti i 160 px di larghezza
                                        * (FOV pieno, immagine schiacciata). */

#define MVFP_BASE          0x07        /* +0x20 = mirror, +0x10 = flip verticale */

#define TX_RING_BYTES      (28 * 1024) /* ring buffer di trasmissione USB CDC  */
#define STREAM_ENABLED_AT_BOOT 1

/* ------------------------------------------------------------------ i pin  */

#define PIN_SIOC   20
#define PIN_SIOD   21
#define PIN_VSYNC   6
#define PIN_HREF    7
#define PIN_PCLK    1
#define PIN_XCLK    0
#define PIN_D0     21
#define PIN_D1      2
#define PIN_D2      8
#define PIN_D3      9
#define PIN_D4     10
#define PIN_D5      3
#define PIN_D6      4
#define PIN_D7      5

#define M_VSYNC  (1UL << PIN_VSYNC)
#define M_HREF   (1UL << PIN_HREF)
#define M_PCLK   (1UL << PIN_PCLK)

/* Estrazione del byte dal registro GPIO_IN. Le maschere sono raggruppate a
 * mano (11 istruzioni invece di 23) sfruttando il fatto che D2..D4 stanno su
 * GPIO8..10 consecutivi e D5..D7 su GPIO3..5 consecutivi. Se si cambia il
 * cablaggio vanno riscritte: gli static_assert qui sotto lo ricordano.      */
static_assert(PIN_D0 == 21 && PIN_D1 == 2 && PIN_D2 == 8 && PIN_D3 == 9 &&
              PIN_D4 == 10 && PIN_D5 == 3 && PIN_D6 == 4 && PIN_D7 == 5,
              "Cablaggio dati diverso: aggiornare la macro BUS_BYTE()");

#define BUS_BYTE(w) ((uint8_t)( (((w) >> 21) & 0x01u)   /* D0  <- GPIO21    */ \
                              | (((w) >>  1) & 0x02u)   /* D1  <- GPIO2     */ \
                              | (((w) >>  6) & 0x1Cu)   /* D2..D4 <- GPIO8..10 */ \
                              | (((w) <<  2) & 0xE0u))) /* D5..D7 <- GPIO3..5  */

/* Puntatore diretto al registro di ingresso GPIO: una load APB, nessuna
 * chiamata di funzione.                                                     */
static volatile uint32_t *const GPIO_IN_PTR = (volatile uint32_t *)GPIO_IN_REG;

/* --------------------------------------------------------------- la seriale */

#if ARDUINO_USB_CDC_ON_BOOT
  #define USBOUT Serial
#else
  #warning "Consigliato: Tools > USB CDC On Boot: Enabled (GPIO20/21 sono UART0)"
  static HWCDC UsbCdc;
  #define USBOUT UsbCdc
#endif

/* ------------------------------------------------------------------ buffer */

#define SRC_W        160
#define SRC_H        120
#define LINE_BYTES   (SRC_W * 2)

static uint8_t  lineBuf[LINE_BYTES + 8];
static uint8_t  frameOut[OUT_W * OUT_H * 2];
static uint16_t xMap[OUT_W];        /* offset in byte dentro lineBuf         */
static uint8_t  srcRowOf[OUT_H];    /* riga sorgente per ogni riga di uscita */

static uint32_t seqCounter   = 0;
static uint32_t framesSent   = 0;
static uint32_t framesDroppedTx = 0;
static uint32_t framesBad    = 0;
static bool     streaming    = STREAM_ENABLED_AT_BOOT;
static uint8_t  mvfpValue    = MVFP_BASE;
static uint8_t  brightValue  = 0x00;   /* reg 0x55, signed-magnitude          */
static uint8_t  contrastValue= 0x40;   /* reg 0x56                            */
static bool     colorBarOn   = false;

/* ==========================================================================
 *  SCCB (I2C) - usabile SOLO quando GPIO21 non e' in uso come D0
 * ========================================================================== */

#define OV7670_ADDR 0x21

/* registri usati */
#define REG_GAIN     0x00
#define REG_BLUE     0x01
#define REG_RED      0x02
#define REG_VREF     0x03
#define REG_COM1     0x04
#define REG_AECHH    0x07
#define REG_COM3     0x0c
#define REG_COM4     0x0d
#define REG_AECH     0x10
#define REG_CLKRC    0x11
#define REG_COM7     0x12
#define REG_COM8     0x13
#define REG_COM9     0x14
#define REG_COM10    0x15
#define REG_HSTART   0x17
#define REG_HSTOP    0x18
#define REG_VSTART   0x19
#define REG_VSTOP    0x1a
#define REG_MVFP     0x1e
#define REG_HREF_R   0x32
#define REG_TSLB     0x3a
#define REG_COM11    0x3b
#define REG_COM12    0x3c
#define REG_COM13    0x3d
#define REG_COM14    0x3e
#define REG_EDGE     0x3f
#define REG_COM15    0x40
#define REG_COM16    0x41
#define REG_BRIGHT   0x55
#define REG_CONTRAS  0x56
#define REG_GFIX     0x69
#define REG_SCAL_XSC 0x70
#define REG_SCAL_YSC 0x71
#define REG_SCAL_DCW 0x72
#define REG_SCAL_PDV 0x73
#define REG_RGB444   0x8c
#define REG_SCAL_PDL 0xa2

static bool sccbWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(OV7670_ADDR);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

static bool sccbRead(uint8_t reg, uint8_t *val) {
  /* SCCB e' a due fasi: scrittura indirizzo + STOP, poi lettura. */
  Wire.beginTransmission(OV7670_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission() != 0) return false;
  delayMicroseconds(200);
  if (Wire.requestFrom((uint8_t)OV7670_ADDR, (uint8_t)1) != 1) return false;
  *val = Wire.read();
  return true;
}

/* ==========================================================================
 *  Tabella di inizializzazione: RGB565 + QQVGA 160x120
 * ========================================================================== */

static const uint8_t initRegs[][2] = {
  /* --- timing di base / finestra VGA (poi ridotta dal DCW) --------------- */
  { REG_TSLB,    0x04 },
  { REG_COM7,    0x04 },            /* uscita RGB, formato VGA               */
  { REG_HSTART,  0x13 }, { REG_HSTOP, 0x01 }, { REG_HREF_R, 0xb6 },
  { REG_VSTART,  0x02 }, { REG_VSTOP, 0x7a }, { REG_VREF,   0x0a },

  /* --- gamma ------------------------------------------------------------- */
  { 0x7a, 0x20 }, { 0x7b, 0x10 }, { 0x7c, 0x1e }, { 0x7d, 0x35 },
  { 0x7e, 0x5a }, { 0x7f, 0x69 }, { 0x80, 0x76 }, { 0x81, 0x80 },
  { 0x82, 0x88 }, { 0x83, 0x8f }, { 0x84, 0x96 }, { 0x85, 0xa3 },
  { 0x86, 0xaf }, { 0x87, 0xc4 }, { 0x88, 0xd7 }, { 0x89, 0xe8 },

  /* --- AGC / AEC (per ora spente, riaccese in coda) ---------------------- */
  { REG_COM8,    0xe0 },
  { REG_GAIN,    0x00 }, { REG_AECH, 0x00 },
  { REG_COM4,    0x40 },
  { REG_COM9,    0x18 },            /* limite guadagno 4x                    */
  { 0xa5, 0x05 }, { 0xab, 0x07 },
  { 0x24, 0x95 }, { 0x25, 0x33 }, { 0x26, 0xe3 },
  { 0x9f, 0x78 }, { 0xa0, 0x68 }, { 0xa1, 0x03 }, { 0xa6, 0xd8 },
  { 0xa7, 0xd8 }, { 0xa8, 0xf0 }, { 0xa9, 0x90 }, { 0xaa, 0x94 },

  /* --- registri "magici" del datasheet ----------------------------------- */
  { 0x0e, 0x61 }, { 0x0f, 0x4b }, { 0x16, 0x02 }, { 0x21, 0x02 },
  { 0x22, 0x91 }, { 0x29, 0x07 }, { 0x33, 0x0b }, { 0x35, 0x0b },
  { 0x37, 0x1d }, { 0x38, 0x71 }, { 0x39, 0x2a },
  { REG_COM12,   0x78 }, { 0x4d, 0x40 }, { 0x4e, 0x20 },
  { REG_GFIX,    0x00 }, { 0x6b, 0x0a }, { 0x74, 0x10 },
  { 0x8d, 0x4f }, { 0x8e, 0x00 }, { 0x8f, 0x00 }, { 0x90, 0x00 },
  { 0x91, 0x00 }, { 0x96, 0x00 }, { 0x9a, 0x00 },
  { 0xb0, 0x84 }, { 0xb1, 0x0c }, { 0xb2, 0x0e }, { 0xb3, 0x82 },
  { 0xb8, 0x0a },

  /* --- bilanciamento del bianco ------------------------------------------ */
  { 0x43, 0x0a }, { 0x44, 0xf0 }, { 0x45, 0x34 }, { 0x46, 0x58 },
  { 0x47, 0x28 }, { 0x48, 0x3a }, { 0x59, 0x88 }, { 0x5a, 0x88 },
  { 0x5b, 0x44 }, { 0x5c, 0x67 }, { 0x5d, 0x49 }, { 0x5e, 0x0e },
  { 0x6c, 0x0a }, { 0x6d, 0x55 }, { 0x6e, 0x11 }, { 0x6f, 0x9f },
  { 0x6a, 0x40 }, { REG_BLUE, 0x40 }, { REG_RED, 0x60 },

  /* --- matrice colore / edge / denoise ----------------------------------- */
  { 0x4f, 0xb3 }, { 0x50, 0xb3 }, { 0x51, 0x00 }, { 0x52, 0x3d },
  { 0x53, 0xa7 }, { 0x54, 0xe4 }, { 0x58, 0x9e },
  { REG_COM16,   0x08 }, { REG_EDGE, 0x00 },
  { 0x75, 0x05 }, { 0x76, 0xe1 }, { 0x4c, 0x00 }, { 0x77, 0x01 },
  { REG_COM13,   0xc0 }, { 0x4b, 0x09 }, { 0xc9, 0x60 },
  { REG_COM16,   0x38 }, { 0x34, 0x11 },
  { REG_COM11,   0x02 }, { 0xa4, 0x88 }, { 0x96, 0x00 }, { 0x97, 0x30 },
  { 0x98, 0x20 }, { 0x99, 0x30 }, { 0x9a, 0x84 }, { 0x9b, 0x29 },
  { 0x9c, 0x03 }, { 0x9d, 0x4c }, { 0x9e, 0x3f }, { 0x78, 0x04 },
  { 0x79, 0x01 }, { 0xc8, 0xf0 }, { 0x79, 0x0f }, { 0xc8, 0x00 },
  { 0x79, 0x10 }, { 0xc8, 0x7e }, { 0x79, 0x0a }, { 0xc8, 0x80 },
  { 0x79, 0x0b }, { 0xc8, 0x01 }, { 0x79, 0x0c }, { 0xc8, 0x0f },
  { 0x79, 0x0d }, { 0xc8, 0x20 }, { 0x79, 0x09 }, { 0xc8, 0x80 },
  { 0x79, 0x02 }, { 0xc8, 0xc0 }, { 0x79, 0x03 }, { 0xc8, 0x40 },
  { 0x79, 0x05 }, { 0xc8, 0x30 }, { 0x79, 0x26 },

  /* --- formato RGB565 ---------------------------------------------------- */
  { REG_COM7,    0x04 },            /* RGB                                   */
  { REG_RGB444,  0x00 },            /* niente RGB444                         */
  { REG_COM1,    0x00 },
  { REG_COM15,   0xd0 },            /* RGB565, range 00..FF                  */
  { REG_COM9,    0x6a },            /* limite guadagno 128x                  */

  /* --- QQVGA 160x120 via DCW + divisore PCLK /4 -------------------------- */
  { REG_COM3,    0x04 },            /* DCW enable                            */
  { REG_COM14,   0x1a },            /* scaling PCLK on, divisore manuale /4  */
  { REG_SCAL_XSC, 0x3a },
  { REG_SCAL_YSC, 0x35 },
  { REG_SCAL_DCW, 0x22 },           /* downsample /4 in H e in V             */
  { REG_SCAL_PDV, 0xf2 },           /* clock di uscita /4                    */
  { REG_SCAL_PDL, 0x02 },

  /* --- sincronismi: PCLK fermo durante il blanking orizzontale ----------- */
  { REG_COM10,   0x20 },
};

/* Scrive tutta la configurazione. Presuppone Wire attivo. */
static void cameraConfigure() {
  sccbWrite(REG_COM7, 0x80);        /* reset software */
  delay(120);
  sccbWrite(REG_CLKRC, CLKRC_PRESCALER);
  delay(10);

  for (size_t i = 0; i < sizeof(initRegs) / sizeof(initRegs[0]); i++) {
    sccbWrite(initRegs[i][0], initRegs[i][1]);
    delayMicroseconds(300);
  }

  sccbWrite(REG_MVFP,   mvfpValue);
  sccbWrite(REG_BRIGHT, brightValue);
  sccbWrite(REG_CONTRAS, contrastValue);
  sccbWrite(REG_SCAL_XSC, 0x3a);
  sccbWrite(REG_SCAL_YSC, colorBarOn ? 0xb5 : 0x35);
  sccbWrite(REG_COM8,   0xe7);      /* AGC + AWB + AEC automatici */
  delay(60);
}

/* ==========================================================================
 *  Gestione pin: SCCB attivo <-> bus dati attivo
 * ========================================================================== */

static const uint8_t busPins[] = { PIN_VSYNC, PIN_HREF, PIN_PCLK,
                                   PIN_D0, PIN_D1, PIN_D2, PIN_D3,
                                   PIN_D4, PIN_D5, PIN_D6, PIN_D7 };

static void busPinsAsInput() {
  for (size_t i = 0; i < sizeof(busPins); i++) {
    gpio_num_t g = (gpio_num_t)busPins[i];
    gpio_reset_pin(g);
    gpio_set_direction(g, GPIO_MODE_INPUT);
    gpio_set_pull_mode(g, GPIO_FLOATING);   /* i pin sono pilotati dalla camera */
  }
}

static void sccbBegin() {
  Wire.begin(PIN_SIOD, PIN_SIOC, 100000);
  Wire.setTimeOut(50);
  delay(2);
}

static void sccbEnd() {
  Wire.end();
  /* SIOC resta a riposo alto, SIOD torna a essere D0 */
  gpio_reset_pin((gpio_num_t)PIN_SIOC);
  gpio_set_direction((gpio_num_t)PIN_SIOC, GPIO_MODE_INPUT);
  gpio_set_pull_mode((gpio_num_t)PIN_SIOC, GPIO_PULLUP_ONLY);
  busPinsAsInput();
  delayMicroseconds(200);
}

/* ------------------------------------------------------------------- XCLK  */

static void startXclk(uint32_t hz) {
  ledc_timer_config_t t = {};
  t.speed_mode      = LEDC_LOW_SPEED_MODE;
  t.duty_resolution = LEDC_TIMER_1_BIT;
  t.timer_num       = LEDC_TIMER_0;
  t.freq_hz         = hz;
  t.clk_cfg         = LEDC_AUTO_CLK;
  ledc_timer_config(&t);

  ledc_channel_config_t c = {};
  c.gpio_num   = PIN_XCLK;
  c.speed_mode = LEDC_LOW_SPEED_MODE;
  c.channel    = LEDC_CHANNEL_0;
  c.timer_sel  = LEDC_TIMER_0;
  c.duty       = 1;                 /* 50% con risoluzione 1 bit */
  c.hpoint     = 0;
  ledc_channel_config(&c);
}

/* ------------------------------------------------- tabelle di riscalatura  */

static void buildMaps() {
#if CROP_SQUARE
  const uint16_t cropW = SRC_H;     /* 120 colonne centrali -> pixel quadrati */
#else
  const uint16_t cropW = SRC_W;     /* FOV pieno, immagine schiacciata        */
#endif
  const uint16_t x0 = (SRC_W - cropW) / 2;
  for (uint16_t x = 0; x < OUT_W; x++)
    xMap[x] = (uint16_t)((x0 + (uint32_t)x * cropW / OUT_W) * 2u);
  for (uint16_t y = 0; y < OUT_H; y++)
    srcRowOf[y] = (uint8_t)((uint32_t)y * SRC_H / OUT_H);
}

/* ==========================================================================
 *  Protocollo verso il PC
 *
 *    offset  size  campo
 *      0      4    magic "OVC1"
 *      4      1    versione = 1
 *      5      1    tipo: 1 = frame, 2 = testo
 *      6      1    formato: 1 = RGB565 big-endian, 0 = testo
 *      7      1    riservato
 *      8      2    width   (LE)
 *     10      2    height  (LE)
 *     12      4    numero di sequenza del frame (LE)
 *     16      2    lunghezza payload (LE)
 *     18      2    checksum dei byte 0..17 (LE)
 *     20      N    payload
 *   20+N      2    checksum del payload (LE)
 * ========================================================================== */

#define HDR_LEN    20
#define PKT_FRAME  1
#define PKT_TEXT   2
#define FMT_RGB565 1

static size_t  txRingBytes = 256;
static uint8_t hdr[HDR_LEN];

static inline void putU16(uint8_t *p, uint16_t v) { p[0] = v & 0xff; p[1] = v >> 8; }
static inline void putU32(uint8_t *p, uint32_t v) {
  p[0] = v & 0xff; p[1] = (v >> 8) & 0xff; p[2] = (v >> 16) & 0xff; p[3] = v >> 24;
}

static uint16_t sum16(const uint8_t *d, size_t n) {
  uint32_t s = 0;
  while (n--) s += *d++;
  return (uint16_t)s;
}

/* Scrittura bloccante con scadenza, usata solo se il ring buffer USB e'
 * troppo piccolo per contenere un frame intero.                            */
static bool writeAll(const uint8_t *d, size_t n, uint32_t deadline) {
  while (n) {
    int room = USBOUT.availableForWrite();
    if (room > 0) {
      size_t chunk = ((size_t)room < n) ? (size_t)room : n;
      size_t w = USBOUT.write(d, chunk);
      d += w; n -= w;
      if (w) continue;
    }
    if ((int32_t)(millis() - deadline) >= 0) return false;
    vTaskDelay(1);
  }
  return true;
}

static bool sendPacket(uint8_t type, uint8_t fmt, uint16_t w, uint16_t h,
                       const uint8_t *payload, uint16_t len) {
  const size_t total = HDR_LEN + len + 2;

  hdr[0] = 'O'; hdr[1] = 'V'; hdr[2] = 'C'; hdr[3] = '1';
  hdr[4] = 1; hdr[5] = type; hdr[6] = fmt; hdr[7] = 0;
  putU16(hdr + 8,  w);
  putU16(hdr + 10, h);
  putU32(hdr + 12, seqCounter);
  putU16(hdr + 16, len);
  putU16(hdr + 18, sum16(hdr, 18));

  uint8_t tail[2];
  putU16(tail, sum16(payload, len));

  /* Se il ring buffer USB puo' contenere un frame intero si scarta il frame
   * quando lo spazio non basta, invece di bloccare la cattura: il flusso
   * resta integro, la latenza bassa e la trasmissione si sovrappone alla
   * cattura del frame successivo (la svuota l'ISR USB).                     */
  if (txRingBytes >= total + 64 && (size_t)USBOUT.availableForWrite() < total)
    return false;

  const uint32_t deadline = millis() + 300;
  return writeAll(hdr, HDR_LEN, deadline) &&
         writeAll(payload, len, deadline) &&
         writeAll(tail, 2, deadline);
}

static void sendText(const char *s) {
  size_t n = strlen(s);
  if (n > 200) n = 200;
  sendPacket(PKT_TEXT, 0, 0, 0, (const uint8_t *)s, (uint16_t)n);
}

/* ==========================================================================
 *  Cattura di un frame
 * ========================================================================== */

#define SPIN_VSYNC  8000000UL     /* ~ 0.5 s */
#define SPIN_HREF    600000UL     /* ~ 40 ms */
#define SPIN_LINE    150000UL     /* failsafe dentro la riga (~10 ms)       */

static inline bool waitLevel(uint32_t mask, bool high, uint32_t spins) {
  while (spins--) {
    bool now = (*GPIO_IN_PTR & mask) != 0;
    if (now == high) return true;
  }
  return false;
}

/* La finestra critica.
 *
 * In QQVGA il periodo di HREF e' ~523 us (a XCLK 12 MHz) di cui solo ~107 us
 * sono dati: aspettare HREF dentro la sezione critica terrebbe le interruzioni
 * spente per l'80% del tempo e affamerebbe l'USB. Aspettarlo fuori, invece, fa
 * perdere i primi byte ogni volta che una ISR e' in esecuzione quando HREF sale
 * (~2% delle righe: righe visibilmente strappate).
 *
 * Soluzione: si misura il periodo di HREF e si spengono le interruzioni solo
 * GUARD_US prima del fronte previsto. Entrambi gli estremi della misura sono
 * presi a interruzioni spente, quindi la stima non ha jitter da ISR. Parte
 * bassa e cresce: sottostimare costa solo una sezione critica piu' lunga,
 * mai una riga persa. Se invece diventa troppo grande, trovare HREF gia'
 * alto e' un segnale netto: si scarta il frame e si taglia la stima del
 * 12.5%, cosi' si rientra in 3-4 frame.                                   */
#define GUARD_US   40u
#define GAP_MAX_US 30000u

static uint32_t gapFirstUs = GUARD_US;   /* VSYNC basso -> prima riga  */
static uint32_t gapLineUs  = GUARD_US;   /* periodo di HREF            */

static inline void learnGap(uint32_t *gap, uint32_t measured) {
  if (measured == 0 || measured >= GAP_MAX_US) return;
  if (measured > *gap) *gap = measured;                  /* sali subito   */
  else                 *gap = (*gap * 15u + measured) / 16u;  /* scendi piano */
}

static bool captureFrame() {
  /* VSYNC e' attivo alto e impulsa nel blanking verticale: aspetto la sua
   * salita e poi la discesa, che marca l'inizio esatto del frame.          */
  if (!waitLevel(M_VSYNC, false, SPIN_VSYNC)) return false;
  if (!waitLevel(M_VSYNC, true,  SPIN_VSYNC)) return false;
  if (!waitLevel(M_VSYNC, false, SPIN_VSYNC)) return false;

  uint32_t  tRef = micros();
  uint32_t *gap  = &gapFirstUs;
  uint8_t   nextOut = 0;

  for (uint16_t y = 0; y < SRC_H; y++) {
    /* Attesa a interruzioni attive fino a poco prima del fronte previsto. */
    const uint32_t target = tRef + (*gap > GUARD_US ? *gap - GUARD_US : 0u);
    const uint32_t limit  = tRef + GAP_MAX_US;
    for (;;) {
      const uint32_t now = micros();
      if ((int32_t)(now - target) >= 0 || (int32_t)(now - limit) >= 0) break;
    }

    uint8_t       *p     = lineBuf;
    uint8_t *const pend  = lineBuf + LINE_BYTES;
    uint32_t       spins = SPIN_LINE;
    uint32_t       w;
    bool           ok    = true;
    uint32_t       tRise;

    /* Sezione critica: ~GUARD_US di attesa piu' i ~107 us della riga. */
    portDISABLE_INTERRUPTS();

    /* Se HREF e' gia' alto il fronte e' passato mentre aspettavamo: la stima
     * era troppo grande. Segnale netto, senza ambiguita'.                  */
    const bool late = (*GPIO_IN_PTR & M_HREF) != 0;
    while (!(*GPIO_IN_PTR & M_HREF)) {              /* inizio riga */
      if (!--spins) { ok = false; break; }
    }
    tRise = micros();
    if (ok) {
      /* Con COM10 bit5 il PCLK e' fermo (basso) durante il blanking
       * orizzontale, quindi il primo fronte di salita dopo HREF e' il byte 0.
       * Questo controllo costa ~7 cicli e non fa nulla nel caso normale, ma
       * garantisce che il primo campionamento cada su un ciclo di PCLK
       * completo anche se il modulo ignorasse COM10 bit5.                  */
      while (*GPIO_IN_PTR & M_PCLK) { if (!--spins) { ok = false; break; } }
    }
    if (ok) {
      do {
        while (!((w = *GPIO_IN_PTR) & M_PCLK)) { if (!--spins) { ok = false; break; } }
        if (!ok) break;
        *p++ = BUS_BYTE(w);
        while (*GPIO_IN_PTR & M_PCLK) { if (!--spins) { ok = false; break; } }
      } while (ok && p < pend);
    }

    portENABLE_INTERRUPTS();

    if (late) {
      /* -12.5% a ogni ritardo: da una sovrastima si rientra in 3-4 frame. */
      *gap = (*gap > 8u * GUARD_US) ? (*gap - *gap / 8u) : GUARD_US;
      return false;
    }
    if (!ok) return false;

    learnGap(gap, tRise - tRef);
    tRef = tRise;              /* riferimento privo di jitter da ISR */
    gap  = &gapLineUs;

    /* Da qui siamo nel blanking orizzontale: ~400 us per il riscalamento
     * della riga, che ne richiede circa 6.                                */
    waitLevel(M_HREF, false, SPIN_HREF);

    if (nextOut < OUT_H && y == srcRowOf[nextOut]) {
      uint8_t *dst = frameOut + (uint32_t)nextOut * (OUT_W * 2);
      for (uint16_t x = 0; x < OUT_W; x++) {
        const uint8_t *s = lineBuf + xMap[x];
        *dst++ = s[0];
        *dst++ = s[1];
      }
      nextOut++;
    }
  }
  return nextOut == OUT_H;
}

/* ==========================================================================
 *  Comandi dal PC (un carattere ciascuno)
 * ========================================================================== */

static void applyToCamera(void (*writes)()) {
  sccbBegin();
  writes();
  sccbEnd();
}

static void wMvfp()     { sccbWrite(REG_MVFP, mvfpValue); }
static void wBright()   { sccbWrite(REG_BRIGHT, brightValue); }
static void wContrast() { sccbWrite(REG_CONTRAS, contrastValue); }
static void wColorBar() {
  sccbWrite(REG_SCAL_XSC, 0x3a);
  sccbWrite(REG_SCAL_YSC, colorBarOn ? 0xb5 : 0x35);
}
static void wReinit()   { cameraConfigure(); }

static void reportStatus() {
  char msg[200];
  snprintf(msg, sizeof(msg),
           "stream=%d xclk=%luHz out=%dx%d sent=%lu droppedTx=%lu bad=%lu "
           "mvfp=0x%02X bright=0x%02X contrast=0x%02X bars=%d ringTx=%u "
           "gapHref=%luus gapFirst=%luus",
           (int)streaming, (unsigned long)XCLK_HZ, OUT_W, OUT_H,
           (unsigned long)framesSent, (unsigned long)framesDroppedTx,
           (unsigned long)framesBad, mvfpValue, brightValue, contrastValue,
           (int)colorBarOn, (unsigned)txRingBytes,
           (unsigned long)gapLineUs, (unsigned long)gapFirstUs);
  sendText(msg);
}

static void handleCommands() {
  while (USBOUT.available()) {
    int c = USBOUT.read();
    switch (c) {
      case 's': case 'S':
        streaming = !streaming;
        sendText(streaming ? "streaming on" : "streaming off");
        break;
      case 'b': case 'B':
        colorBarOn = !colorBarOn;
        applyToCamera(wColorBar);
        sendText(colorBarOn ? "color bar on" : "color bar off");
        break;
      case 'm': case 'M':
        mvfpValue ^= 0x20;
        applyToCamera(wMvfp);
        sendText("mirror toggled");
        break;
      case 'v': case 'V':
        mvfpValue ^= 0x10;
        applyToCamera(wMvfp);
        sendText("vflip toggled");
        break;
      case '+':
        /* 0x55 e' in modulo+segno: bit7 = 1 significa negativo */
        if (brightValue & 0x80) brightValue = (brightValue & 0x7f) > 8
                                            ? (uint8_t)(brightValue - 8) : 0x00;
        else                    brightValue = (brightValue < 0x60)
                                            ? (uint8_t)(brightValue + 8) : 0x60;
        applyToCamera(wBright);
        sendText("brightness +");
        break;
      case '-':
        if (brightValue & 0x80) { if ((brightValue & 0x7f) < 0x60) brightValue += 8; }
        else if (brightValue >= 8) brightValue -= 8;
        else brightValue = 0x88;
        applyToCamera(wBright);
        sendText("brightness -");
        break;
      case ']':
        if (contrastValue < 0x80) contrastValue += 8;
        applyToCamera(wContrast);
        sendText("contrast +");
        break;
      case '[':
        if (contrastValue > 0x10) contrastValue -= 8;
        applyToCamera(wContrast);
        sendText("contrast -");
        break;
      case 'r': case 'R':
        applyToCamera(wReinit);
        sendText("camera reinit");
        break;
      case 'i': case 'I': case '?':
        reportStatus();
        break;
      default:
        break;   /* newline e altro: ignorati */
    }
  }
}

/* ========================================================================== */

static void configureUsb() {
  USBOUT.end();
  delay(20);
  USBOUT.setTxBufferSize(0);                /* elimina l'eventuale ring da 256 B */
  size_t got = USBOUT.setTxBufferSize(TX_RING_BYTES);
  txRingBytes = got ? got : TX_RING_BYTES;
  USBOUT.setRxBufferSize(512);
  USBOUT.setTxTimeoutMs(10);
  USBOUT.begin();
  delay(50);
  /* Verifica reale dello spazio disponibile: se il ring e' rimasto piccolo
   * sendPacket() passa automaticamente alla scrittura bloccante.           */
  int room = USBOUT.availableForWrite();
  if (room > 0 && (size_t)room < txRingBytes) txRingBytes = (size_t)room;
}

void setup() {
  /* UART0 vive su GPIO20/21: va spenta prima di toccare SCCB e D0. */
#if ARDUINO_USB_CDC_ON_BOOT
  Serial0.end();
#else
  Serial.end();
#endif
  gpio_reset_pin((gpio_num_t)PIN_SIOC);
  gpio_reset_pin((gpio_num_t)PIN_SIOD);

  configureUsb();
  buildMaps();

  startXclk(XCLK_HZ);
  delay(50);

  sccbBegin();
  uint8_t pid = 0, ver = 0;
  bool found = sccbRead(0x0a, &pid) && sccbRead(0x0b, &ver);
  cameraConfigure();
  sccbEnd();

  char msg[120];
  snprintf(msg, sizeof(msg),
           "OV7670 %s (PID=0x%02X VER=0x%02X) xclk=%lu out=%dx%d ringTx=%u",
           found ? "trovata" : "NON rilevata via SCCB", pid, ver,
           (unsigned long)XCLK_HZ, OUT_W, OUT_H, (unsigned)txRingBytes);
  sendText(msg);
}

void loop() {
  handleCommands();

  if (!streaming) { vTaskDelay(10); return; }

  if (captureFrame()) {
    seqCounter++;
    if (sendPacket(PKT_FRAME, FMT_RGB565, OUT_W, OUT_H,
                   frameOut, (uint16_t)sizeof(frameOut))) framesSent++;
    else                                                  framesDroppedTx++;
  } else {
    framesBad++;
    /* Nessun sincronismo: la camera potrebbe essere scollegata o senza XCLK. */
    vTaskDelay(20);
  }

  /* Lascia girare il task idle (watchdog) e lo stack USB. */
  vTaskDelay(1);
}
