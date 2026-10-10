/*
  Heltec WiFi LoRa 32 V3 (ESP32-S3 + SX1262) POCSAG transceiver

  !! Needs the 433/470-510 MHz hardware variant. The 868/915 MHz board has its
  !! matching network and filter for that band and will be deaf/weak at 439.9875 MHz.

  TX: RadioLib PagerClient (direct-mode frequency stepping), same as the T3 sketch.
  RX: the SX1262 has no direct-mode data/clock pins, so RadioLib's PagerClient
      cannot receive on it. Instead the GFSK packet engine is used:
        sync word   = POCSAG frame sync codeword (last 3 bytes, see SYNC_LEN)
        packet      = fixed length, RX_BATCHES batches (16 codewords + next FSC each)
        CRC/whitening off, RX continuous (chip re-arms itself after every packet)
      and pocsag_rx.h decodes the codewords (BCH check/correct, address, text).

  Pins (Heltec V3):
    SX1262: NSS 8, SCK 9, MOSI 10, MISO 11, RST 12, BUSY 13, DIO1 14
            DIO2 = internal RF switch, DIO3 = TCXO supply (1.8 V)
    OLED  : SDA 17, SCL 18, RST 21, powered via Vext (GPIO36, LOW = on)
    LED   : 35

  Serial monitor 115200, line ending "Newline". Type  RIC:message  to transmit.
  Library deps: RadioLib, U8g2.
*/

#include <SPI.h>
#include <Wire.h>
#include <RadioLib.h>
#include <U8x8lib.h>
#include <stdarg.h>
#include <Preferences.h>
#include <Ticker.h>
#include "pocsag_rx.h"

#define PIN_SCK    9
#define PIN_MISO  11
#define PIN_MOSI  10
#define PIN_NSS    8
#define PIN_RST   12
#define PIN_BUSY  13
#define PIN_DIO1  14
#define PIN_SDA   17
#define PIN_SCL   18
#define PIN_OLED_RST 21
#define PIN_VEXT  36
#define PIN_LED   35

static const float    FREQ_MHZ      = 439.9875;
static const int32_t  DEFAULT_CORR_HZ = 0;     // used until a correction is saved with /save
static const uint16_t BAUD          = 1200;     // 512 / 1200 / 2400
static const bool     INVERT        = false;    // same meaning as on the T3 sketch
// Defaults, used until changed at runtime (/ric /mon /pwr) and stored with /save
static const uint32_t DEFAULT_RIC   = 604552;   // DAPNET RIC 0604552 - NO leading 0 (octal!)
static const bool     DEFAULT_MON   = false;    // true = show every RIC on the channel
static const int8_t   DEFAULT_PWR   = 10;       // dBm
static const char*    BOARD_NAME    = "Heltec WiFi LoRa 32 V3 / SX1262";
static const int8_t   PWR_MIN = -9, PWR_MAX = 22;
static const int      FREQ_STEP_HZ  = 30;       // float frequency granularity at 440 MHz
static const bool     DEFAULT_LED   = true;     // status LED patterns, /led 0|1
static const float    RX_BW_KHZ     = 14.6;     // SX126x double-sided BW; 19.5 if far off frequency
static const uint8_t  RX_BATCHES    = 3;        // batches captured per packet (max 3: 200 B <= 255 B)
static const uint8_t  SYNC_LEN      = 3;        // use last 3 bytes of FSC so a few bits lost while
                                                // the chip restarts between packets don't matter

// frequency correction in Hz, adjustable at runtime (see /help), stored in NVS
static int32_t  corrHz = DEFAULT_CORR_HZ;
static uint32_t myRic  = DEFAULT_RIC;
static bool     monAll = DEFAULT_MON;
static int8_t   txPwr  = DEFAULT_PWR;
static bool     ledEnabled = DEFAULT_LED;

// ---------------- status LED ----------------
// Driven from an esp_timer (Ticker), so it keeps blinking while loop() is blocked
// inside pager.transmit() or the /cw carrier.
//   TX / carrier    : fast blink 5 Hz for the whole transmission
//   RX for own RIC  : 3 blinks          RX for other RIC (monitor mode): 1 blink
//   idle            : 20 ms heartbeat flash every 5 s (receiver is alive)
static Ticker ledTicker;
static volatile int  ledToggles = 0;     // remaining edges, -1 = run until ledIdle()
static volatile bool ledState   = false;
static void ledWrite(bool on) { ledState = on; digitalWrite(PIN_LED, on ? HIGH : LOW); }
static void ledTick() {                  // timer context: no detach here, just go quiet
  if (ledToggles == 0) { if (ledState) ledWrite(false); return; }
  ledWrite(!ledState);
  if (ledToggles > 0) ledToggles--;
}
static void ledStart(int toggles, uint32_t halfPeriodMs) {
  ledTicker.detach();
  ledWrite(false);
  if (!ledEnabled) return;
  ledToggles = toggles;
  ledTick();                             // first edge immediately
  ledTicker.attach_ms(halfPeriodMs, ledTick);
}
static void ledBlink(int count, uint32_t periodMs) { ledStart(count * 2, periodMs / 2); }
static void ledBusy() { ledStart(-1, 100); }
static void ledIdle() { ledTicker.detach(); ledToggles = 0; ledWrite(false); }
static bool ledQuiet() { return ledToggles == 0; }
static Preferences prefs;
static inline float curFreq() { return FREQ_MHZ + corrHz / 1e6f; }

SX1262 radio = new Module(PIN_NSS, PIN_DIO1, PIN_RST, PIN_BUSY);
PagerClient pager(&radio);

// ---------------- logging: Serial + OLED ----------------
U8X8_SSD1306_128X64_NONAME_HW_I2C oled(PIN_OLED_RST, PIN_SCL, PIN_SDA);
static const uint8_t LOG_W = 16, LOG_H = 8;
static uint8_t logBuf[LOG_W * LOG_H];
U8X8LOG oledLog;
static bool oledOk = false;

static void logf(const char* fmt, ...) {
  char buf[256];
  va_list ap; va_start(ap, fmt); vsnprintf(buf, sizeof(buf), fmt, ap); va_end(ap);
  Serial.print(buf);
  if (oledOk) oledLog.print(buf);
}

// JSON helpers for the machine-readable @-lines (see protocol notes further down)
static void jsonStr(String& o, const char* s) {
  o += '"';
  for (; *s; s++) {
    char c = *s;
    if (c == '"' || c == '\\') { o += '\\'; o += c; }
    else if ((uint8_t)c < 0x20) { char b[8]; snprintf(b, sizeof b, "\\u%04x", (uint8_t)c); o += b; }
    else o += c;
  }
  o += '"';
}
static void emitEvt(const char* type, const String& json) {
  Serial.print('@'); Serial.print(type); Serial.print(' '); Serial.println(json);
}
static void emitMsg(const char* type, const char* msg) {
  String j = "{\"msg\":"; jsonStr(j, msg); j += "}"; emitEvt(type, j);
}

static void oledInit() {
  pinMode(PIN_VEXT, OUTPUT);
  digitalWrite(PIN_VEXT, LOW);        // power the OLED
  delay(100);
  oled.setBusClock(400000);
  oled.begin();                       // also pulses OLED RST (GPIO21)
  oled.setFont(u8x8_font_chroma48medium8_r);
  oledLog.begin(oled, LOG_W, LOG_H, logBuf);
  oledLog.setRedrawMode(0);
  oledOk = true;
}

static void check(int16_t st, const char* what) {
  if (st != RADIOLIB_ERR_NONE) {
    logf("ERR %s: %d\n", what, st);
    while (true) { delay(1000); }
  }
}

// ---------------- POCSAG receive ----------------
static const size_t PKT_LEN = 64 + 68 * (RX_BATCHES - 1);   // first FSC is consumed as sync word
static volatile bool pktFlag = false;
static uint32_t lastPktMs = 0;

static void IRAM_ATTR onPacket() { pktFlag = true; }

static void onMessage(uint32_t ric, uint8_t func, const char* text, bool numeric, int fixes, int bad) {
  if (ric == myRic) ledBlink(3, 300); else ledBlink(1, 150);
  logf("RX %lu f%u%s\n%s\n", (unsigned long)ric, func, bad ? " ERR" : (fixes ? " fix" : ""),
       text[0] ? text : "(tone)");
  float rssi = radio.getRSSI();
  Serial.printf("   [%s, %d corrected, %d bad codewords, RSSI %.1f dBm]\n",
                numeric ? "numeric" : "alpha", fixes, bad, rssi);
  String j = "{\"ric\":" + String(ric) + ",\"func\":" + String(func) + ",\"text\":";
  jsonStr(j, text);
  j += ",\"numeric\":" + String(numeric ? "true" : "false") + ",\"fixes\":" + String(fixes) +
       ",\"bad\":" + String(bad) + ",\"rssi\":" + String(rssi, 1) + "}";
  emitEvt("RX", j);
}
pocsag::Decoder decoder(onMessage);

// (re)apply RX configuration - pager.begin()/transmit() change deviation & mode
static void startRx() {
  radio.standby();
  check(radio.setFrequency(curFreq()), "setFrequency");
  check(radio.setBitRate(BAUD / 1000.0), "setBitRate");
  check(radio.setFrequencyDeviation(4.5), "setFreqDev");
  check(radio.setRxBandwidth(RX_BW_KHZ), "setRxBandwidth");
  check(radio.setDataShaping(RADIOLIB_SHAPING_NONE), "setDataShaping");
  check(radio.setEncoding(RADIOLIB_ENCODING_NRZ), "setEncoding");
  check(radio.setCRC(0), "setCRC");
  check(radio.setWhitening(false), "setWhitening");

  // chip treats high frequency as '1', POCSAG uses low frequency as '1'
  // -> with INVERT=false look for the inverted sync word and invert the payload
  uint32_t fsc = INVERT ? pocsag::FSC : ~pocsag::FSC;
  uint8_t sync[4] = { (uint8_t)(fsc >> 24), (uint8_t)(fsc >> 16), (uint8_t)(fsc >> 8), (uint8_t)fsc };
  check(radio.setSyncWord(sync + (4 - SYNC_LEN), SYNC_LEN), "setSyncWord");
  check(radio.setPreambleLength(0), "setPreambleLength");   // 0 -> preamble detector off:
                                                           // sync search runs continuously,
                                                           // so batch 2,3.. (no preamble) are caught
  check(radio.fixedPacketLengthMode(PKT_LEN), "fixedPacketLengthMode");
  radio.setPacketReceivedAction(onPacket);
  check(radio.startReceive(), "startReceive");             // RX continuous
}

static inline uint32_t be32(const uint8_t* p) {
  return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) | ((uint32_t)p[2] << 8) | p[3];
}

static void processPacket() {
  uint8_t buf[PKT_LEN];
  int16_t st = radio.readData(buf, PKT_LEN);
  lastPktMs = millis();
  if (st != RADIOLIB_ERR_NONE) { Serial.printf("[RX] readData %d\n", st); return; }
  if (!INVERT) for (size_t i = 0; i < PKT_LEN; i++) buf[i] = ~buf[i];

  size_t off = 0;
  for (uint8_t b = 0; b < RX_BATCHES; b++) {
    if (b > 0) {                       // next batch must start with FSC, else transmission ended
      uint32_t s = be32(buf + off); off += 4;
      if (__builtin_popcount(s ^ pocsag::FSC) > 3) { decoder.flush(); return; }
    }
    for (uint8_t i = 0; i < 16; i++, off += 4) decoder.feed(be32(buf + off), i);
  }
  // message may continue in the next packet (chip is still in RX continuous)
}

// ---------------- runtime config + serial protocol ----------------
// Human-readable log lines go to Serial as before. Lines starting with '@' are
// machine-readable JSON events for the web console (webui/pocsag_console.html):
//   @CFG {...}   current configuration (after boot and after every change)
//   @RX  {...}   received message        @TX {...}  transmit result
//   @CW  {...}   carrier on/off          @OK {...}  @ERR {...}
static void emitCfg() {
  char b[320];
  snprintf(b, sizeof b,
    "{\"board\":\"%s\",\"freq\":%.4f,\"corr\":%ld,\"eff\":%.5f,\"baud\":%u,\"inv\":%s,"
    "\"ric\":%lu,\"mon\":%s,\"pwr\":%d,\"pmin\":%d,\"pmax\":%d,\"step\":%d,\"led\":%s}",
    BOARD_NAME, FREQ_MHZ, (long)corrHz, curFreq(), BAUD, INVERT ? "true" : "false",
    (unsigned long)myRic, monAll ? "true" : "false", txPwr, PWR_MIN, PWR_MAX, FREQ_STEP_HZ,
    ledEnabled ? "true" : "false");
  emitEvt("CFG", String(b));
}

static void applyFreq() {
  radio.standby();
  check(pager.begin(curFreq(), BAUD, INVERT, 4500), "pager.begin");   // TX + RX base freq
  startRx();
  logf("F %.5f\ncorr %+ld Hz\n", curFreq(), (long)corrHz);
  emitCfg();
}

static void applyFilter() {
  decoder.setFilter(myRic, monAll ? 0 : 0x1FFFFF);
  logf("RIC %lu%s\n", (unsigned long)myRic, monAll ? " +mon" : "");
  emitCfg();
}

// kind: 'a' alphanumeric, 'n' numeric, 't' tone only
static void doTx(uint32_t ric, char kind, String text) {   // by value: RadioLib wants String&
  if (ric > 2097151UL) { emitMsg("ERR", "RIC out of range (max 2097151)"); return; }
  radio.standby();
  int16_t st;
  ledBusy();
  if (kind == 't')      st = pager.sendTone(ric);
  else if (kind == 'n') st = pager.transmit(text, ric, RADIOLIB_PAGER_BCD);
  else                  st = pager.transmit(text, ric, RADIOLIB_PAGER_ASCII);
  ledIdle();
  startRx();
  logf("TX %lu %s\n", (unsigned long)ric, st == RADIOLIB_ERR_NONE ? "ok" : "FAIL");
  String j = "{\"ric\":" + String(ric) + ",\"kind\":\"" + String(kind) + "\",\"ok\":" +
             (st == RADIOLIB_ERR_NONE ? "true" : "false") + ",\"code\":" + String(st) + ",\"text\":";
  jsonStr(j, text.c_str());
  j += "}";
  emitEvt("TX", j);
}

// "RIC:text" -> true + parts
static bool splitRicText(const String& s, uint32_t& ric, String& text) {
  int sep = s.indexOf(':');
  if (sep <= 0) return false;
  ric = (uint32_t)s.substring(0, sep).toInt();
  text = s.substring(sep + 1);
  return true;
}

static void help() {
  Serial.println(F(
    "Commands:\n"
    "  RIC:text      transmit alphanumeric page\n"
    "  /n RIC:digits transmit numeric page (0-9 * U space - ( ) )\n"
    "  /tone RIC     transmit tone-only page\n"
    "  /f            show frequency and correction\n"
    "  /u [Hz]       tune up   (default 100 Hz)\n"
    "  /d [Hz]       tune down (default 100 Hz)\n"
    "  /c Hz         set absolute correction, e.g. /c -1250\n"
    "  /cw [s]       unmodulated carrier for measuring (default 5 s, max 30)\n"
    "  /ric N        set own RIC (receive filter)\n"
    "  /mon 0|1      1 = show all RICs on the channel\n"
    "  /pwr dBm      TX power\n"
    "  /led 0|1      status LED patterns off/on\n"
    "  /save         store correction, RIC, monitor, power and LED in flash\n"
    "  /cfg          print configuration (@CFG line)\n"
    "  /cls          clear the OLED\n"
    "  /help\n"
    "Note: RadioLib takes the frequency as float -> effective steps ~30 Hz at 440 MHz."));
}

static void handleCommand(String cmd) {
  cmd.trim();
  String arg;
  int sp = cmd.indexOf(' ');
  if (sp > 0) { arg = cmd.substring(sp + 1); arg.trim(); cmd = cmd.substring(0, sp); }
  cmd.toLowerCase();
  long n = arg.length() ? arg.toInt() : 0;

  if (cmd == "/u")      { corrHz += arg.length() ? labs(n) : 100; applyFreq(); }
  else if (cmd == "/d") { corrHz -= arg.length() ? labs(n) : 100; applyFreq(); }
  else if (cmd == "/c") { corrHz = n; applyFreq(); }
  else if (cmd == "/f") { logf("F %.5f\ncorr %+ld Hz\n", curFreq(), (long)corrHz); emitCfg(); }
  else if (cmd == "/cfg") { emitCfg(); }
  else if (cmd == "/ric") {
    if (n <= 0 || n > 2097151L) { emitMsg("ERR", "RIC must be 1..2097151"); return; }
    myRic = (uint32_t)n; applyFilter();
  }
  else if (cmd == "/mon") { monAll = (n != 0); applyFilter(); }
  else if (cmd == "/pwr") {
    txPwr = (int8_t)constrain(n, (long)PWR_MIN, (long)PWR_MAX);
    radio.standby();
    check(radio.setOutputPower(txPwr), "setOutputPower");
    startRx();
    logf("PWR %d dBm\n", txPwr);
    emitCfg();
  }
  else if (cmd == "/save") {
    prefs.putInt("corrHz", corrHz);
    prefs.putUInt("ric", myRic);
    prefs.putBool("mon", monAll);
    prefs.putChar("pwr", txPwr);
    prefs.putBool("led", ledEnabled);
    logf("saved\n");
    emitMsg("OK", "settings saved");
  }
  else if (cmd == "/n" || cmd == "/tone") {
    uint32_t ric; String text;
    if (cmd == "/tone") { ric = (uint32_t)n; text = ""; }
    else if (!splitRicText(arg, ric, text)) { emitMsg("ERR", "use /n RIC:digits"); return; }
    doTx(ric, cmd == "/tone" ? 't' : 'n', text);
  }
  else if (cmd == "/cw") {
    long secs = constrain(arg.length() ? n : 5, 1, 30);
    logf("CW %.5f %lds\n", curFreq(), secs);
    emitEvt("CW", "{\"on\":true,\"secs\":" + String(secs) + "}");
    radio.standby();
    check(radio.setFrequency(curFreq()), "setFrequency");
    check(radio.transmitDirect(), "transmitDirect");               // continuous carrier
    ledBusy();
    delay(secs * 1000);
    ledIdle();
    startRx();
    logf("CW end\n");
    emitEvt("CW", "{\"on\":false}");
  }
  else if (cmd == "/led") {
    ledEnabled = (n != 0);
    if (ledEnabled) ledBlink(2, 200); else ledIdle();
    emitCfg();
  }
  else if (cmd == "/cls") {
    if (oledOk) {
      oledLog.print('\f');            // u8log: clear log buffer, cursor to top-left
      oled.clearDisplay();             // blank the panel now (log redraws on next newline)
    }
    Serial.println("OLED cleared");
    emitMsg("OK", "display cleared");
  }
  else { help(); emitMsg("ERR", "unknown command"); }
}

// ---------------- setup / loop ----------------
void setup() {
  Serial.begin(115200);
  delay(300);
  pinMode(PIN_LED, OUTPUT);
  oledInit();

  SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI, PIN_NSS);

  prefs.begin("pocsag", false);
  corrHz = prefs.getInt("corrHz", DEFAULT_CORR_HZ);
  myRic  = prefs.getUInt("ric", DEFAULT_RIC);
  monAll = prefs.getBool("mon", DEFAULT_MON);
  txPwr  = prefs.getChar("pwr", DEFAULT_PWR);
  ledEnabled = prefs.getBool("led", DEFAULT_LED);

  radio.tcxoVoltage = 1.8;            // Heltec V3: TCXO powered from DIO3 at 1.8 V
  ConfigFSK_t cfg;
  cfg.frequency = curFreq();
  cfg.bitRate = BAUD / 1000.0;
  cfg.frequencyDeviation = 4.5;
  cfg.receiverBandwidth = RX_BW_KHZ;
  cfg.power = txPwr;
  check(radio.beginFSK(cfg), "radio.beginFSK");
  check(radio.setDio2AsRfSwitch(true), "setDio2AsRfSwitch");

  check(pager.begin(curFreq(), BAUD, INVERT, 4500), "pager.begin");

  decoder.setFilter(myRic, monAll ? 0 : 0x1FFFFF);
  startRx();

  logf("%.4f MHz\ncorr %+ld Hz\n%u bps %s\nRIC %lu\nListening...\n", curFreq(), (long)corrHz,
       BAUD, INVERT ? "inv" : "norm", (unsigned long)myRic);
  Serial.println("Send 'RIC:message' to transmit, /help for commands.");
  emitCfg();
  ledBlink(2, 200);                  // booted
}

void loop() {
  // heartbeat: short flash every 5 s while nothing else is showing
  static uint32_t lastBeat = 0;
  if (millis() - lastBeat > 5000) {
    lastBeat = millis();
    if (ledQuiet()) ledBlink(1, 40);
  }

  if (pktFlag) { pktFlag = false; processPacket(); }

  // no follow-up packet within ~1 packet time -> close any open message
  const uint32_t pktMs = (PKT_LEN + 4) * 8 * 1000UL / BAUD;
  if (decoder.active() && millis() - lastPktMs > pktMs + 500) decoder.flush();

  if (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    line.trim();
    if (line.length() == 0) return;
    if (line.startsWith("/")) { handleCommand(line); return; }
    uint32_t ric; String text;
    if (splitRicText(line, ric, text)) doTx(ric, 'a', text);
    else { Serial.println("Format: RIC:message"); emitMsg("ERR", "format is RIC:message"); }
  }
}
