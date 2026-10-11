/*
  lora-pocsag firmware: POCSAG pager transmitter/receiver on 70 cm with LoRa boards.

  One sketch for both boards; the board is picked automatically (see board.h):
    LilyGO T3 V1.6.1      (ESP32    + SX1278)  Arduino board: "TTGO LoRa32-OLED" or "ESP32 Dev Module"
    Heltec WiFi LoRa 32 V3 (ESP32-S3 + SX1262) Arduino board: "Heltec WiFi LoRa 32(V3)"

  Libraries (Library Manager): RadioLib, U8g2, NimBLE-Arduino (2.x)

  Control: USB serial 115200 (line ending "Newline") or Bluetooth LE (Nordic UART),
  or the web console (webui/pocsag_console.html). Type /help for commands.
  Lines starting with '@' are JSON events for the web console:
    @CFG config   @RX received page   @TX transmit result   @CW carrier on/off
    @RSSI live signal level   @LOGEND end of /log replay   @OK / @ERR
*/

#include <Wire.h>
#include <U8x8lib.h>
#include <stdarg.h>
#include <Preferences.h>
#include <Ticker.h>
#include "board.h"
#include "radio.h"
#include "msg_log.h"
#include "ble_link.h"

// ---------------- defaults (used until changed at runtime and stored with /save) ----------------
static const float    FREQ_MHZ        = 439.9875;   // DAPNET channel
static const int32_t  DEFAULT_CORR_HZ = 0;
static const uint32_t DEFAULT_RIC     = 123456;     // placeholder; set yours with /ric. Never a leading 0 (octal!)
static const bool     DEFAULT_MON     = false;      // true = show pages for every RIC
static const int8_t   DEFAULT_PWR     = 10;         // dBm
static const bool     DEFAULT_LED     = true;
static const bool     DEFAULT_BLE     = true;
static const char*    DEFAULT_CALL    = "XX9YYY";   // placeholder until set with /call
static const uint8_t  DEFAULT_TXMAX   = 10;         // s, transmit time limit
static const uint8_t  TXMAX_MIN = 2, TXMAX_MAX = 60;

// ---------------- runtime settings ----------------
static int32_t  corrHz     = DEFAULT_CORR_HZ;
static uint32_t myRic      = DEFAULT_RIC;
static bool     monAll     = DEFAULT_MON;
static int8_t   txPwr      = DEFAULT_PWR;
static bool     ledEnabled = DEFAULT_LED;
static bool     bleEnabled = DEFAULT_BLE;
static uint32_t blePin     = 0;                     // default derived from MAC
static char     callsign[12] = "XX9YYY";
static bool     autoId     = true;
static float    rxBw       = BW_DEFAULT;
static uint8_t  txMax      = DEFAULT_TXMAX;

static Preferences prefs;
static PocsagRadio radio;
static MsgLog      msgLog;
BleLink ble;
DualOut out(ble);                                   // USB serial + paired BLE phone

static inline float curFreq() { return FREQ_MHZ + corrHz / 1e6f; }
static inline uint32_t rxMask() { return monAll ? 0 : 0x1FFFFF; }

// ---------------- status LED ----------------
// Driven from an esp_timer (Ticker), so it keeps blinking while loop() is blocked in a transmission.
//   TX / carrier: 5 Hz blink   RX own RIC: 3 blinks   RX other RIC: 1 blink   idle: flash every 5 s
static Ticker ledTicker;
static volatile int  ledToggles = 0;     // remaining edges, -1 = until ledIdle()
static volatile bool ledState   = false;
static void ledWrite(bool on) { ledState = on; digitalWrite(PIN_LED, on ? HIGH : LOW); }
static void ledTick() {
  if (ledToggles == 0) { if (ledState) ledWrite(false); return; }
  ledWrite(!ledState);
  if (ledToggles > 0) ledToggles--;
}
static void ledStart(int toggles, uint32_t halfPeriodMs) {
  ledTicker.detach();
  ledWrite(false);
  if (!ledEnabled) return;
  ledToggles = toggles;
  ledTick();
  ledTicker.attach_ms(halfPeriodMs, ledTick);
}
static void ledBlink(int count, uint32_t periodMs) { ledStart(count * 2, periodMs / 2); }
static void ledBusy() { ledStart(-1, 100); }
static void ledIdle() { ledTicker.detach(); ledToggles = 0; ledWrite(false); }
static bool ledQuiet() { return ledToggles == 0; }

// ---------------- OLED + logging ----------------
U8X8_SSD1306_128X64_NONAME_HW_I2C oled(PIN_OLED_RST, PIN_SCL, PIN_SDA);
static const uint8_t LOG_W = 16, LOG_H = 8;
static uint8_t oledBuf[LOG_W * LOG_H];
U8X8LOG oledLog;
static bool oledOk = false;

static void logf(const char* fmt, ...) {
  char buf[256];
  va_list ap; va_start(ap, fmt); vsnprintf(buf, sizeof(buf), fmt, ap); va_end(ap);
  out.print(buf);
  if (oledOk) oledLog.print(buf);
}

static void oledInit() {
  if (PIN_VEXT >= 0) { pinMode(PIN_VEXT, OUTPUT); digitalWrite(PIN_VEXT, LOW); }   // Heltec: power the OLED
  Wire.begin(PIN_SDA, PIN_SCL);
  delay(100);                          // OLED reset / supply settle
  oled.setBusClock(400000);            // U8g2 sets the Wire clock itself on every transfer
  oled.begin();
  oled.setFont(u8x8_font_chroma48medium8_r);
  oledLog.begin(oled, LOG_W, LOG_H, oledBuf);
  oledLog.setRedrawMode(0);            // redraw on newline
  oledOk = true;
}

// ---------------- JSON helpers ----------------
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

static void check(int16_t st, const char* what) {
  if (st != RADIOLIB_ERR_NONE) {
    logf("ERR %s: %d\n", what, st);
    while (true) { delay(1000); }
  }
}
static void warnIf(int16_t st, const char* what) {
  if (st != RADIOLIB_ERR_NONE) {
    char m[64]; snprintf(m, sizeof m, "%s failed, code %d", what, st);
    logf("ERR %s %d\n", what, st);
    emitMsg("ERR", m);
  }
}

static void emitCfg() {
  String bws = "[";
  for (size_t i = 0; i < BW_COUNT; i++) { if (i) bws += ','; bws += String(BW_LIST[i], 1); }
  bws += "]";
  char b[720];
  snprintf(b, sizeof b,
    "{\"board\":\"%s\",\"freq\":%.4f,\"corr\":%ld,\"eff\":%.5f,\"baud\":%u,\"inv\":%s,"
    "\"ric\":%lu,\"mon\":%s,\"pwr\":%d,\"pmin\":%d,\"pmax\":%d,\"step\":%d,\"led\":%s,"
    "\"ble\":%s,\"blename\":\"%s\",\"call\":\"%s\",\"autoid\":%s,\"pnote\":\"%s\","
    "\"bw\":%.1f,\"bwss\":%s,\"txmax\":%u,\"txmin\":%u,\"txmaxmax\":%u,\"logn\":%u,\"bws\":",
    BOARD_NAME, FREQ_MHZ, (long)corrHz, curFreq(), POCSAG_BAUD, POCSAG_INVERT ? "true" : "false",
    (unsigned long)myRic, monAll ? "true" : "false", txPwr, PWR_MIN, PWR_MAX, FREQ_STEP_HZ,
    ledEnabled ? "true" : "false", bleEnabled ? "true" : "false", ble.name().c_str(),
    callsign, autoId ? "true" : "false", PWR_NOTE,
    rxBw, BW_SINGLE_SIDED ? "true" : "false", txMax, TXMAX_MIN, TXMAX_MAX, MsgLog::LOG_N);
  emitEvt("CFG", String(b) + bws + "}");
}

// ---------------- received pages ----------------
static void emitRx(const LogEntry& e, long age, bool replay) {
  String j = "{\"id\":" + String(e.id) + ",\"ric\":" + String(e.ric) + ",\"func\":" +
             (e.func < 0 ? String("null") : String(e.func)) + ",\"text\":";
  jsonStr(j, e.text);
  j += ",\"numeric\":" + String(e.numeric < 0 ? "null" : (e.numeric ? "true" : "false"));
  j += ",\"fixes\":" + (e.fixes < 0 ? String("null") : String(e.fixes));
  j += ",\"bad\":" + (e.bad < 0 ? String("null") : String(e.bad));
  j += ",\"rssi\":" + (e.rssi10 == INT16_MIN ? String("null") : String(e.rssi10 / 10.0f, 1));
  j += ",\"age\":" + (age < 0 ? String("null") : String(age));
  if (replay) j += ",\"replay\":true";
  j += "}";
  emitEvt("RX", j);
}

static void onPage(const RxPage& p) {
  if (p.ric == myRic) ledBlink(3, 300); else ledBlink(1, 150);
  logf("RX %lu%s\n%s\n", (unsigned long)p.ric, p.bad > 0 ? " ERR" : (p.fixes > 0 ? " fix" : ""),
       p.text[0] ? p.text : "(tone)");
  const LogEntry& e = msgLog.add(p.ric, p.func, p.text, p.numeric, p.fixes, p.bad, p.rssi);
  emitRx(e, 0, false);
}

static void replayLog() {
  for (uint8_t i = 0; i < msgLog.count(); i++) {
    const LogEntry& e = msgLog.at(i);
    emitRx(e, msgLog.ageSec(e), true);
  }
  emitEvt("LOGEND", "{\"n\":" + String(msgLog.count()) + "}");
}

// ---------------- transmit with time limit ----------------
static Ticker txWatch;
static void txWatchFire() { radio.abortTx(); }    // timer context: only sets a flag

// kind: 'a' alphanumeric, 'n' numeric, 't' tone only
static void doTx(uint32_t ric, char kind, String text) {   // by value: RadioLib wants String&
  if (ric == 0 || ric > 2097151UL) { emitMsg("ERR", "RIC out of range (1..2097151)"); return; }
  if (kind == 'a' && autoId && callsign[0]) {               // identify the station
    String up = text; up.toUpperCase();
    if (up.indexOf(callsign) < 0) text = String(callsign) + " " + text;
  }
  float air = pocsagAirtime(ric, kind, text.length());
  if (air > txMax) {
    char m[96]; snprintf(m, sizeof m, "page would take %.1f s on air, limit is %u s (/txmax)", air, txMax);
    emitMsg("ERR", m);
    return;
  }
  ledBusy();
  txWatch.once_ms((uint32_t)(txMax * 1000UL + 500), txWatchFire);   // hard stop if anything hangs
  uint32_t t0 = millis();
  int16_t st = radio.transmit(ric, kind, text);
  txWatch.detach();
  ledIdle();
  float took = (millis() - t0) / 1000.0f;
  if (st == RADIOLIB_ERR_TX_TIMEOUT) logf("TX %lu ABORTED\n", (unsigned long)ric);
  else logf("TX %lu %s\n", (unsigned long)ric, st == RADIOLIB_ERR_NONE ? "ok" : "FAIL");
  String j = "{\"ric\":" + String(ric) + ",\"kind\":\"" + String(kind) + "\",\"ok\":" +
             (st == RADIOLIB_ERR_NONE ? "true" : "false") + ",\"code\":" + String(st) +
             ",\"airtime\":" + String(took, 2) + ",\"text\":";
  jsonStr(j, text.c_str());
  j += "}";
  emitEvt("TX", j);
  if (st == RADIOLIB_ERR_TX_TIMEOUT) emitMsg("ERR", "transmission stopped by the time limit");
}

// ---------------- live RSSI ----------------
static uint32_t rssiUntil = 0, rssiLast = 0;
static void serviceRssi() {
  if (!rssiUntil) return;
  if ((int32_t)(millis() - rssiUntil) > 0) { rssiUntil = 0; emitEvt("RSSI", "{\"on\":false}"); return; }
  if (millis() - rssiLast < 250) return;
  rssiLast = millis();
  emitEvt("RSSI", "{\"dbm\":" + String(radio.rssiNow(), 1) + "}");
}

// ---------------- commands ----------------
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
    "  RIC:text        transmit alphanumeric page\n"
    "  /n RIC:digits   transmit numeric page (0-9 * U space - ( ) )\n"
    "  /tone RIC       transmit tone-only page\n"
    "  /txmax s        transmit time limit, 2..60 s (pages that would take longer are refused)\n"
    "  /f              show frequency and correction\n"
    "  /u [Hz] /d [Hz] tune up / down (default 100 Hz)\n"
    "  /c Hz           set absolute correction, e.g. /c -1250\n"
    "  /cw [s]         unmodulated carrier for measuring (default 5 s, capped by /txmax)\n"
    "  /bw kHz         receive filter bandwidth (chip units, see /cfg bws)\n"
    "  /rssi 0|1       stream live signal level for 2 minutes\n"
    "  /ric N          own RIC (receive filter)\n"
    "  /mon 0|1        1 = show pages for every RIC\n"
    "  /pwr dBm        TX power (see /cfg pmin..pmax)\n"
    "  /call CALL      station callsign (/call - to clear)\n"
    "  /autoid 0|1     put the callsign in front of text pages\n"
    "  /log            replay stored received pages   /logclear  forget them\n"
    "  /led 0|1        status LED patterns\n"
    "  /ble 0|1        Bluetooth LE console   /blepin NNNNNN   /bleforget\n"
    "  /save           store all settings in flash\n"
    "  /cfg            print configuration (@CFG line)\n"
    "  /cls            clear the OLED\n"
    "  /help"));
}

static void bleStart() {
  if (ble.begin(ble.name().length() ? ble.name() : BleLink::defaultName(), blePin))
    logf("BLE %s\nPIN %06lu\n", ble.name().c_str(), (unsigned long)blePin);
  else
    logf("BLE start failed\n");
}

static void handleCommand(String cmd);
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

  if (cmd == "/u" || cmd == "/d" || cmd == "/c") {
    if (cmd == "/u") corrHz += arg.length() ? labs(n) : 100;
    else if (cmd == "/d") corrHz -= arg.length() ? labs(n) : 100;
    else corrHz = n;
    warnIf(radio.setFreq(curFreq()), "set frequency");
    logf("F %.5f\ncorr %+ld Hz\n", curFreq(), (long)corrHz);
    emitCfg();
  }
  else if (cmd == "/f") { logf("F %.5f\ncorr %+ld Hz\n", curFreq(), (long)corrHz); emitCfg(); }
  else if (cmd == "/cfg") { emitCfg(); }
  else if (cmd == "/ric" || cmd == "/mon") {
    if (cmd == "/ric") {
      if (n <= 0 || n > 2097151L) { emitMsg("ERR", "RIC must be 1..2097151"); return; }
      myRic = (uint32_t)n;
    } else monAll = (n != 0);
    warnIf(radio.setFilter(myRic, rxMask()), "set filter");
    logf("RIC %lu%s\n", (unsigned long)myRic, monAll ? " +mon" : "");
    emitCfg();
  }
  else if (cmd == "/pwr") {
    txPwr = (int8_t)constrain(n, (long)PWR_MIN, (long)PWR_MAX);
    if (txPwr != n) {
      char m[64]; snprintf(m, sizeof m, "power limited to %d dBm (board range %d..%d)", txPwr, PWR_MIN, PWR_MAX);
      emitMsg("ERR", m);
    }
    warnIf(radio.setPower(txPwr), "set power");
    logf("PWR %d dBm\n", txPwr);
    emitCfg();
  }
  else if (cmd == "/bw") {
    float want = arg.toFloat();
    if (want <= 0) { emitMsg("ERR", "use /bw kHz, allowed values in /cfg bws"); return; }
    rxBw = bwNearest(want);
    warnIf(radio.setRxBw(rxBw), "set bandwidth");
    logf("BW %.1f kHz\n", rxBw);
    emitCfg();
  }
  else if (cmd == "/txmax") {
    if (n < TXMAX_MIN || n > TXMAX_MAX) { emitMsg("ERR", "time limit must be 2..60 s"); return; }
    txMax = (uint8_t)n;
    emitCfg();
  }
  else if (cmd == "/rssi") {
    if (n) { rssiUntil = millis() + 120000UL; if (!rssiUntil) rssiUntil = 1; }
    else if (rssiUntil) { rssiUntil = 0; emitEvt("RSSI", "{\"on\":false}"); }
  }
  else if (cmd == "/log") { replayLog(); }
  else if (cmd == "/logclear") { msgLog.clear(); emitMsg("OK", "stored pages cleared"); }
  else if (cmd == "/save") {
    prefs.putInt("corrHz", corrHz);
    prefs.putUInt("ric", myRic);
    prefs.putBool("mon", monAll);
    prefs.putChar("pwr", txPwr);
    prefs.putBool("led", ledEnabled);
    prefs.putBool("ble", bleEnabled);
    prefs.putUInt("blepin", blePin);
    prefs.putString("call", callsign);
    prefs.putBool("autoid", autoId);
    prefs.putFloat("bw", rxBw);
    prefs.putUChar("txmax", txMax);
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
    long secs = constrain(arg.length() ? n : 5, 1L, (long)txMax);
    logf("CW %.5f %lds\n", curFreq(), secs);
    emitEvt("CW", "{\"on\":true,\"secs\":" + String(secs) + "}");
    int16_t st = radio.carrierOn();
    if (st == RADIOLIB_ERR_NONE) { ledBusy(); delay(secs * 1000); ledIdle(); }
    warnIf(st, "carrier");
    warnIf(radio.carrierOff(), "restart receiver");
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
  else if (cmd == "/bleforget") { ble.forgetBonds(); emitMsg("OK", "paired phones forgotten"); }
  else if (cmd == "/call") {
    String c = arg; c.trim(); c.toUpperCase();
    if (c == "-") c = "";
    bool ok = c.length() <= 10;
    for (size_t i = 0; ok && i < c.length(); i++) ok = isalnum((unsigned char)c[i]) || c[i] == '/';
    if (!ok) { emitMsg("ERR", "callsign: up to 10 letters, digits or /"); return; }
    strncpy(callsign, c.c_str(), sizeof(callsign) - 1); callsign[sizeof(callsign) - 1] = 0;
    logf("CALL %s\n", callsign[0] ? callsign : "(none)");
    emitCfg();
  }
  else if (cmd == "/autoid") { autoId = (n != 0); emitCfg(); }
  else if (cmd == "/cls") {
    if (oledOk) { oledLog.print('\f'); oled.clearDisplay(); }
    out.println("OLED cleared");
    emitMsg("OK", "display cleared");
  }
  else { help(); emitMsg("ERR", "unknown command"); }
}

// ---------------- setup / loop ----------------
void setup() {
  Serial.begin(115200);
  delay(200);
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, LOW);
  oledInit();

  prefs.begin("pocsag", false);
  corrHz     = prefs.getInt("corrHz", DEFAULT_CORR_HZ);
  myRic      = prefs.getUInt("ric", DEFAULT_RIC);
  monAll     = prefs.getBool("mon", DEFAULT_MON);
  txPwr      = (int8_t)constrain((int)prefs.getChar("pwr", DEFAULT_PWR), PWR_MIN, PWR_MAX);
  ledEnabled = prefs.getBool("led", DEFAULT_LED);
  bleEnabled = prefs.getBool("ble", DEFAULT_BLE);
  blePin     = prefs.getUInt("blepin", BleLink::defaultPin());
  prefs.getString("call", callsign, sizeof(callsign));
  autoId     = prefs.getBool("autoid", true);
  rxBw       = bwNearest(prefs.getFloat("bw", BW_DEFAULT));
  txMax      = (uint8_t)constrain((int)prefs.getUChar("txmax", DEFAULT_TXMAX), TXMAX_MIN, TXMAX_MAX);

  msgLog.begin();
  check(radio.begin(curFreq(), txPwr, rxBw, myRic, rxMask(), onPage), "radio init");

  logf("%.4f MHz\ncorr %+ld Hz\nBW %.1f kHz\nRIC %lu%s\n", curFreq(), (long)corrHz, rxBw,
       (unsigned long)myRic, monAll ? " +mon" : "");
  if (bleEnabled) bleStart();
  if (!strcmp(callsign, DEFAULT_CALL)) logf("%s\nset /call!\n", callsign);
  else logf("%s\n", callsign[0] ? callsign : "no callsign");
  out.println("Send 'RIC:message' to transmit, /help for commands.");
  emitCfg();
  ledBlink(2, 200);
}

void loop() {
  static uint32_t lastBeat = 0;
  if (millis() - lastBeat > 5000) {
    lastBeat = millis();
    if (ledQuiet()) ledBlink(1, 40);
  }

  radio.poll();
  serviceRssi();
  msgLog.service();

  if (Serial.available()) processLine(Serial.readStringUntil('\n'));

  String bl;
  if (ble.readLine(bl)) processLine(bl);
  static bool bleWasConnected = false;
  if (ble.connected() != bleWasConnected) {
    bleWasConnected = ble.connected();
    logf(bleWasConnected ? "BLE connected\n" : "BLE disconnected\n");
    if (bleWasConnected) emitCfg();
    else rssiUntil = 0;                         // stop streaming when the phone leaves
  }
}
