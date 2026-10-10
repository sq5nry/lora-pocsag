/*
  ESP32 + SX1278 (e.g. Ra-02) POCSAG transceiver using RadioLib PagerClient

  Receive: prints decoded messages for the own RIC (or all RICs with /mon 1)
  Transmit: type  RIC:message  in the serial monitor (115200 baud)

  Board: LILYGO T3 V1.6.1 (SX1278 module U22), pins per T3_V1.6.1 schematic.
  All LoRa lines are hard-wired on the PCB, no jumpers needed:
    SX1278   ESP32
    SCK   -> IO5
    MISO  -> IO19
    MOSI  -> IO27
    NSS   -> IO18   (via R61 0R)
    RESET -> IO23   (via R60 0R)
    DIO0  -> IO26
    DIO1  -> IO33   (via R16 22R) bit clock DCLK in direct mode - REQUIRED for RX
    DIO2  -> IO32   (via R15 22R) demodulated data in direct mode - REQUIRED for RX
  OLED (SSD1306 128x64, I2C 0x3C): SDA IO21 / SCL IO22 (via R13/R12 22R), no reset line.

  Library deps: RadioLib, U8g2 (both in the Arduino Library Manager).
  All log output goes to Serial AND to a scrolling 16x8 text console on the OLED.
*/

#include <SPI.h>
#include <Wire.h>
#include <RadioLib.h>
#include <U8x8lib.h>
#include <stdarg.h>
#include <Preferences.h>
#include <Ticker.h>
#include "ble_link.h"          // BLE serial (Nordic UART), needs NimBLE-Arduino 2.x

#define PIN_SCK   5
#define PIN_MISO  19
#define PIN_MOSI  27
#define PIN_NSS   18
#define PIN_RST   23
#define PIN_DIO0  26
#define PIN_DIO1  33
#define PIN_DIO2  32
#define PIN_SDA   21
#define PIN_SCL   22
#define PIN_LED   25        // green user LED via R20 2k, active high

// DAPNET channel; adjust to your own use
static const float    FREQ_MHZ      = 439.9875;
// Crystal error correction in Hz, adjustable at runtime (/u /d /c, see /help), stored with /save.
// DEFAULT_CORR_HZ is used until a value has been saved. Example: carrier 3.2 kHz high -> -3200
static const int32_t  DEFAULT_CORR_HZ = 0;
static const uint16_t BAUD          = 1200;    // 512, 1200 or 2400
static const bool     INVERT        = false;   // flip if nothing decodes on either side
// Defaults, used until changed at runtime (/ric /mon /pwr) and stored with /save
static const uint32_t DEFAULT_RIC   = 604552;  // DAPNET RIC 0604552 (from DMR ID 2604552). NO leading 0 - that would be octal!
static const bool     DEFAULT_MON   = false;   // true = show every RIC on the channel
static const int8_t   DEFAULT_PWR   = 10;      // dBm, PA_BOOST
static const char*    BOARD_NAME    = "LilyGO T3 V1.6.1 / SX1278";
static const int8_t   PWR_MIN = 2, PWR_MAX = 17;
static const int      FREQ_STEP_HZ  = 61;      // synthesizer resolution
static const bool     DEFAULT_LED   = true;    // status LED patterns, /led 0|1

static int32_t  corrHz = DEFAULT_CORR_HZ;
static uint32_t myRic  = DEFAULT_RIC;
static bool     monAll = DEFAULT_MON;
static int8_t   txPwr  = DEFAULT_PWR;
static bool     ledEnabled = DEFAULT_LED;
static const bool DEFAULT_BLE = true;            // BLE console on/off, /ble 0|1
static bool     bleEnabled = DEFAULT_BLE;
static uint32_t blePin     = 0;                  // 6-digit pairing PIN, /blepin; default derived from MAC
BleLink ble;
DualOut out(ble);                                // everything we print: USB serial + paired BLE phone

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

SX1278 radio = new Module(PIN_NSS, PIN_DIO0, PIN_RST, PIN_DIO1);
PagerClient pager(&radio);

// ---------------- logging: Serial + OLED ----------------
// U8x8 = text-only, no frame buffer; U8X8LOG gives a scrolling terminal.
// HW I2C ctor args: (reset, clock, data)
U8X8_SSD1306_128X64_NONAME_HW_I2C oled(U8X8_PIN_NONE, PIN_SCL, PIN_SDA);
static const uint8_t LOG_W = 16, LOG_H = 8;   // 8x8 font -> 16 cols x 8 rows
static uint8_t logBuf[LOG_W * LOG_H];
U8X8LOG oledLog;
static bool oledOk = false;

// printf-style log to both outputs. Call only from task context, never from an ISR.
static void logf(const char* fmt, ...) {
  char buf[192];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);
  out.print(buf);
  if (oledOk) oledLog.print(buf);   // wraps long lines, scrolls on '\n'
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
  out.print('@'); out.print(type); out.print(' '); out.println(json);
}
static void emitMsg(const char* type, const char* msg) {
  String j = "{\"msg\":"; jsonStr(j, msg); j += "}"; emitEvt(type, j);
}

// Scan an I2C bus, print every device found, return first OLED-like address (0x3C/0x3D) or 0
static uint8_t i2cScan(int sda, int scl) {
  Wire.end();
  Wire.begin(sda, scl);
  Wire.setClock(100000);
  delay(5);
  uint8_t oledAddr = 0;
  out.printf("[I2C] scan SDA=%d SCL=%d:", sda, scl);
  for (uint8_t a = 0x08; a < 0x78; a++) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) {
      out.printf(" 0x%02X", a);
      if (!oledAddr && (a == 0x3C || a == 0x3D)) oledAddr = a;
    }
  }
  out.println(oledAddr ? "" : "  (no OLED)");
  return oledAddr;
}

static void oledInit() {
  // T3 V1.6.1: OLED D/C# and CS# tied to GND -> 0x3C; RES# has its own RC reset.
  Wire.begin(PIN_SDA, PIN_SCL);
  delay(50);                         // let the OLED RC reset (~5 ms) and its LDO settle

  // Diagnostic I2C scan - shows what is actually on the bus
  out.print("[I2C] scan SDA=21 SCL=22:");
  int found = 0;
  for (uint8_t a = 0x08; a < 0x78; a++) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) { out.printf(" 0x%02X", a); found++; }
  }
  out.println(found ? "" : " nothing found");

  Wire.beginTransmission(0x3C);
  uint8_t e = Wire.endTransmission();
  if (e != 0) out.printf("[OLED] no ACK at 0x3C (err %u) - initialising anyway\n", e);

  // Initialise regardless of the probe result; writes to a missing display are harmless.
  oled.setBusClock(400000);          // U8g2 sets the Wire clock itself on every transfer
  oled.begin();
  oledOk = true;
  oled.setFont(u8x8_font_chroma48medium8_r);
  oledLog.begin(oled, LOG_W, LOG_H, logBuf);
  oledLog.setRedrawMode(0);          // 0: redraw only on newline (cheaper), 1: every char
}

static void check(int16_t st, const char* what) {
  if (st != RADIOLIB_ERR_NONE) {
    logf("ERR %s: %d\n", what, st);
    while (true) { delay(1000); }
  }
}

static void startRx() {
  // also sets RX frequency (from pager.begin), 4.5 kHz deviation and DIO2 back to input
  check(pager.startReceive(PIN_DIO2, myRic, monAll ? 0 : 0x1FFFFF), "pager.startReceive");
}

// Before any TX: in continuous mode the SX1278 FSK-modulates whatever is on DIO2.
// After RX, DIO2 is a floating ESP input and the deviation is 4.5 kHz, so the carrier
// would wobble randomly. Pin DIO2 low and set minimum deviation (what pager.begin() does).
static void prepTx() {
  radio.standby();
  pinMode(PIN_DIO2, OUTPUT);
  digitalWrite(PIN_DIO2, LOW);
  radio.setFrequencyDeviation(-1);   // -> lowest setting (0.6 kHz), pager steps the carrier itself
}

// ---------------- runtime config + serial protocol ----------------
// Human-readable log lines go to Serial as before. Lines starting with '@' are
// machine-readable JSON events for the web console (webui/pocsag_console.html):
//   @CFG {...}   current configuration (after boot and after every change)
//   @RX  {...}   received message        @TX {...}  transmit result
//   @CW  {...}   carrier on/off          @OK {...}  @ERR {...}
static void emitCfg() {
  char b[420];
  snprintf(b, sizeof b,
    "{\"board\":\"%s\",\"freq\":%.4f,\"corr\":%ld,\"eff\":%.5f,\"baud\":%u,\"inv\":%s,"
    "\"ric\":%lu,\"mon\":%s,\"pwr\":%d,\"pmin\":%d,\"pmax\":%d,\"step\":%d,\"led\":%s,"
    "\"ble\":%s,\"blename\":\"%s\"}",
    BOARD_NAME, FREQ_MHZ, (long)corrHz, curFreq(), BAUD, INVERT ? "true" : "false",
    (unsigned long)myRic, monAll ? "true" : "false", txPwr, PWR_MIN, PWR_MAX, FREQ_STEP_HZ,
    ledEnabled ? "true" : "false", bleEnabled ? "true" : "false", ble.name().c_str());
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
  radio.standby(); startRx();
  logf("RIC %lu%s\n", (unsigned long)myRic, monAll ? " +mon" : "");
  emitCfg();
}

// kind: 'a' alphanumeric, 'n' numeric, 't' tone only
static void doTx(uint32_t ric, char kind, String text) {   // by value: RadioLib wants String&
  if (ric > 2097151UL) { emitMsg("ERR", "RIC out of range (max 2097151)"); return; }
  prepTx();
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
  out.println(F(
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
    "  /ble 0|1      Bluetooth LE console off/on\n"
    "  /blepin NNNNNN  BLE pairing PIN (6 digits)\n"
    "  /bleforget    forget all paired phones\n"
    "  /save         store correction, RIC, monitor, power, LED and BLE settings in flash\n"
    "  /cfg          print configuration (@CFG line)\n"
    "  /cls          clear the OLED\n"
    "  /help\n"
    "Note: SX1278 synth step is 61 Hz; float frequency adds ~30 Hz granularity."));
}

static void bleStart() {
  if (ble.begin(ble.name().length() ? ble.name() : BleLink::defaultName(), blePin))
    logf("BLE %s\nPIN %06lu\n", ble.name().c_str(), (unsigned long)blePin);
  else
    logf("BLE start failed\n");
}

static void handleCommand(String cmd);
// one input line from USB serial or BLE
static void processLine(String line) {
  line.trim();
  if (line.length() == 0) return;
  if (line.startsWith("/")) { handleCommand(line); return; }
  uint32_t ric; String text;
  if (splitRicText(line, ric, text)) doTx(ric, 'a', text);
  else { out.println("Format: RIC:message"); emitMsg("ERR", "format is RIC:message"); }
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
    prefs.putBool("ble", bleEnabled);
    prefs.putUInt("blepin", blePin);
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
    prepTx();
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
  else if (cmd == "/ble") {
    bleEnabled = (n != 0);
    if (bleEnabled) bleStart(); else { ble.end(); logf("BLE off\n"); }
    emitCfg();
  }
  else if (cmd == "/blepin") {
    if (n < 100000 || n > 999999) { emitMsg("ERR", "PIN must have 6 digits"); return; }
    blePin = (uint32_t)n;
    ble.setPin(blePin);
    logf("BLE PIN %06lu\n", (unsigned long)blePin);
    emitMsg("OK", "PIN changed; /save to keep it, /bleforget to make paired phones pair again");
  }
  else if (cmd == "/bleforget") {
    ble.forgetBonds();
    emitMsg("OK", "paired phones forgotten");
  }
  else if (cmd == "/cls") {
    if (oledOk) {
      oledLog.print('\f');            // u8log: clear log buffer, cursor to top-left
      oled.clearDisplay();             // blank the panel now (log redraws on next newline)
    }
    out.println("OLED cleared");
    emitMsg("OK", "display cleared");
  }
  else { help(); emitMsg("ERR", "unknown command"); }
}

void setup() {
  Serial.begin(115200);
  delay(200);
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, LOW);
  oledInit();

  // T3 uses non-default SPI pins; the default VSPI pins (SCK 18, MOSI 23) would clash
  // with NSS and RESET here, so the bus must be started explicitly before the radio.
  SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI, PIN_NSS);

  prefs.begin("pocsag", false);
  corrHz = prefs.getInt("corrHz", DEFAULT_CORR_HZ);
  myRic  = prefs.getUInt("ric", DEFAULT_RIC);
  monAll = prefs.getBool("mon", DEFAULT_MON);
  txPwr  = prefs.getChar("pwr", DEFAULT_PWR);
  ledEnabled = prefs.getBool("led", DEFAULT_LED);
  bleEnabled = prefs.getBool("ble", DEFAULT_BLE);
  blePin     = prefs.getUInt("blepin", BleLink::defaultPin());
  const float f = curFreq();

  // Recent RadioLib uses a config struct; older releases take
  // radio.beginFSK(freq, ...) arguments instead.
  ConfigFSK_t cfg;
  cfg.frequency = f;
  check(radio.beginFSK(cfg), "radio.beginFSK");
  check(radio.setOutputPower(txPwr), "setOutputPower");

  // begin(base freq, bitrate, invert, shift Hz)
  check(pager.begin(f, BAUD, INVERT, 4500), "pager.begin");

  startRx();
  logf("%.4f MHz\ncorr %+ld Hz\n%u bps %s\nRIC %lu\nListening...\n", f, (long)corrHz, BAUD,
       INVERT ? "inv" : "norm", (unsigned long)myRic);
  out.println("Send 'RIC:message' to transmit, /help for commands.");
  if (bleEnabled) bleStart();
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

  // --- receive ---
  // 2 batches is enough for short/medium alphanumeric messages
  if (pager.available() >= 2) {
    String msg;
    uint32_t addr = 0;
    int16_t st = pager.readData(msg, 0, &addr);
    if (st == RADIOLIB_ERR_NONE) {
      logf("RX %lu:\n%s\n", (unsigned long)addr, msg.c_str());
      if (addr == myRic) ledBlink(3, 300); else ledBlink(1, 150);
      // RadioLib's pager RX doesn't expose function bits / error counts
      String j = "{\"ric\":" + String(addr) + ",\"func\":null,\"text\":";
      jsonStr(j, msg.c_str());
      j += ",\"numeric\":null,\"fixes\":null,\"bad\":null,\"rssi\":null}";
      emitEvt("RX", j);
    } else {
      logf("RX err %d\n", st);
      emitEvt("ERR", "{\"msg\":\"RX decode error\",\"code\":" + String(st) + "}");
    }
  }

  // --- transmit ---
  if (Serial.available()) processLine(Serial.readStringUntil('\n'));

  // --- BLE console ---
  String bl;
  if (ble.readLine(bl)) processLine(bl);
  static bool bleWasConnected = false;
  if (ble.connected() != bleWasConnected) {
    bleWasConnected = ble.connected();
    logf(bleWasConnected ? "BLE connected\n" : "BLE disconnected\n");
    if (bleWasConnected) emitCfg();               // greet the phone with the current state
  }
}
