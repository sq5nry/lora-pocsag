/*
  lora-pocsag firmware: POCSAG pager transmitter/receiver on 70 cm with LoRa boards.

  One sketch for all boards; the profile follows the board chosen in the Arduino IDE (board.h):
    LilyGO T3 V1.6.1      (ESP32    + SX1278)  Arduino board: "TTGO LoRa32-OLED"
    Heltec WiFi LoRa 32 V3 (ESP32-S3 + SX1262) Arduino board: "Heltec WiFi LoRa 32(V3)"
    ESP32 DevKit + Ebyte E22-400M30S (SX1268)  Arduino board: "ESP32 Dev Module"

  Libraries (Library Manager): RadioLib, U8g2, NimBLE-Arduino (2.x)

  Control: USB serial 115200 (line ending "Newline") or Bluetooth LE (Nordic UART),
  or the web console (webui/pocsag_console.html). Type /help for commands.
  Lines starting with '@' are JSON events for the web console:
    @CFG config   @RX received page   @TX transmit result   @CW carrier on/off
    @RSSI live signal level   @LOGEND end of /log replay   @CAL board-to-board tuning   @OK / @ERR
    @DAP DAPNET link status

  DAPNET: with WiFi set (/wifi) and a registered transmitter (/dapnet name, /dapnet key), /dapnet 1
  logs the board into the DAPNET core; pages routed to it go out in its assigned time slots.
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
#include "dapnet.h"

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
static const uint32_t DEFAULT_DEST    = 123456;     // destination RIC offered in the web console
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
static uint32_t lastDest   = DEFAULT_DEST;          // last RIC a page was sent to, kept by /save
static bool     dapOn      = false;                 // DAPNET transmitter link

static Preferences prefs;
static PocsagRadio radio;
static MsgLog      msgLog;
BleLink ble;
DualOut out(ble);                                   // USB serial + paired BLE phone
static Dapnet dap;
static volatile bool dapChanged = false;
static uint16_t dapRev = 0;                        // bumped on every DAPNET/WiFi setting change

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

// Flash the whole display negative and back (SSD1306 "inverse display" command), e.g. as a
// pager-style alert. Run from loop(), never from a timer: it talks I2C.
static uint8_t  flashLeft = 0;
static uint32_t flashAt = 0;
static void oledInvert(bool on) {
  if (!oledOk) return;
  u8x8_t* u = oled.getU8x8();
  u8x8_cad_StartTransfer(u);
  u8x8_cad_SendCmd(u, on ? 0xA7 : 0xA6);     // 0xA7 inverse, 0xA6 normal
  u8x8_cad_EndTransfer(u);
}
static void oledFlash(uint8_t times) { flashLeft = times * 2; flashAt = millis() - 1000; }
static void serviceFlash() {
  if (!flashLeft || millis() - flashAt < 250) return;
  flashAt = millis();
  flashLeft--;
  oledInvert(flashLeft & 1);                  // odd = negative; ends on normal
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
    "\"bw\":%.1f,\"bwss\":%s,\"txmax\":%u,\"txmin\":%u,\"txmaxmax\":%u,\"logn\":%u,\"dest\":%lu,\"bws\":",
    BOARD_NAME, FREQ_MHZ, (long)corrHz, curFreq(), POCSAG_BAUD, POCSAG_INVERT ? "true" : "false",
    (unsigned long)myRic, monAll ? "true" : "false", txPwr, PWR_MIN, PWR_MAX, FREQ_STEP_HZ,
    ledEnabled ? "true" : "false", bleEnabled ? "true" : "false", ble.name().c_str(),
    callsign, autoId ? "true" : "false", PWR_NOTE,
    rxBw, BW_SINGLE_SIDED ? "true" : "false", txMax, TXMAX_MIN, TXMAX_MAX, MsgLog::LOG_N,
    (unsigned long)lastDest);
  String d = ",\"dap\":" + String(dapOn ? "true" : "false") + ",\"wssid\":";
  jsonStr(d, dap.ssid.c_str());
  d += ",\"wpass\":" + String(dap.pass.length() ? "true" : "false") + ",\"dsrv\":";
  jsonStr(d, (dap.host + ":" + String(dap.port)).c_str());
  d += ",\"dname\":";
  jsonStr(d, dap.name.c_str());
  d += ",\"dkey\":" + String(dap.key.length() ? "true" : "false") + ",\"drev\":" + String(dapRev);
  emitEvt("CFG", String(b) + bws + d + "}");
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
  if (p.ric == myRic && !p.text[0]) oledFlash(5);   // tone-only page for us: flash the display
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

static int16_t txPage(uint32_t ric, char kind, String& text, uint8_t func, bool dapnet);

// kind: 'a' alphanumeric, 'n' numeric, 't' tone only
static void doTx(uint32_t ric, char kind, String text) {   // by value: RadioLib wants String&
  if (ric == 0 || ric > 2097151UL) { emitMsg("ERR", "RIC out of range (1..2097151)"); return; }
  lastDest = ric;
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
  txPage(ric, kind, text, RADIOLIB_PAGER_FUNC_AUTO, false);
}

// key up, send, report; dapnet = page relayed from DAPNET (no ERR toasts for the console user)
static int16_t txPage(uint32_t ric, char kind, String& text, uint8_t func, bool dapnet) {
  ledBusy();
  txWatch.once_ms((uint32_t)(txMax * 1000UL + 500), txWatchFire);   // hard stop if anything hangs
  uint32_t t0 = millis();
  int16_t st = radio.transmit(ric, kind, text, func);
  txWatch.detach();
  ledIdle();
  float took = (millis() - t0) / 1000.0f;
  const char* pfx = dapnet ? "DTX" : "TX";
  if (st == RADIOLIB_ERR_TX_TIMEOUT) logf("%s %lu ABORTED\n", pfx, (unsigned long)ric);
  else logf("%s %lu %s\n", pfx, (unsigned long)ric, st == RADIOLIB_ERR_NONE ? "ok" : "FAIL");
  String j = "{\"ric\":" + String(ric) + ",\"kind\":\"" + String(kind) + "\",\"ok\":" +
             (st == RADIOLIB_ERR_NONE ? "true" : "false") + ",\"code\":" + String(st) +
             ",\"airtime\":" + String(took, 2);
  if (func != RADIOLIB_PAGER_FUNC_AUTO) j += ",\"func\":" + String(func);
  if (dapnet) j += ",\"src\":\"dapnet\"";
  j += ",\"text\":";
  jsonStr(j, text.c_str());
  j += "}";
  emitEvt("TX", j);
  if (st == RADIOLIB_ERR_TX_TIMEOUT && !dapnet) emitMsg("ERR", "transmission stopped by the time limit");
  return st;
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

// ---------------- board-to-board frequency alignment ----------------
// Base board: /calbeacon sends short LoRa packets on the channel.
// Board to tune: /calsync listens in LoRa mode; the chip reports each packet's frequency error,
// the correction is moved and the error measured again, learning the chip's sign and scale
// from the first step (secant method), until the boards agree within ~120 Hz.
static void emitCal(const String& json) { emitEvt("CAL", json); }

static void calBeacon(long count) {
  count = constrain(count, 5L, 60L);
  logf("CAL beacon x%ld\n", count);
  emitCal("{\"role\":\"beacon\",\"state\":\"start\",\"count\":" + String(count) + "}");
  ledBusy();
  int16_t st = radio.calEnter(curFreq());
  if (!st) st = radio.calBeacon(callsign, (uint8_t)count);
  int16_t st2 = radio.calExit(curFreq());
  ledIdle();
  warnIf(st, "calibration beacon");
  warnIf(st2, "restart receiver");
  emitCal("{\"role\":\"beacon\",\"state\":\"done\",\"ok\":" + String(st == 0 ? "true" : "false") + "}");
  logf("CAL beacon end\n");
}

static void calSync(long waitSec) {
  waitSec = constrain(waitSec, 5L, 120L);
  const int32_t startCorr = corrHz;
  logf("CAL sync, waiting\n");
  emitCal("{\"role\":\"sync\",\"state\":\"listen\",\"wait\":" + String(waitSec) + "}");
  ledBusy();
  int16_t st = radio.calEnter(curFreq());
  // Secant method: start by assuming "measured error = Hz to move", then learn the real
  // ratio (sign and scale) from how the error changed after each step. Converges in 2-5
  // rounds even if the chip reports the error with the opposite sign or a wrong scale.
  float err = 0, gain = 1.0f, prevErr = 0, prevStep = 0;
  bool havePrev = false;
  uint8_t got = 0;
  bool done = false, heard = false;
  for (uint8_t round = 0; !st && round < 6; round++) {
    st = radio.calMeasure(curFreq(), 4, round == 0 ? waitSec * 1000UL : 3000UL, 1500UL, err, got);
    if (st || !got) break;
    heard = true;
    emitCal("{\"role\":\"sync\",\"state\":\"round\",\"round\":" + String(round + 1) +
            ",\"err\":" + String(err, 0) + ",\"n\":" + String(got) + ",\"corr\":" + String(corrHz) + "}");
    if (fabsf(err) < 120.0f) { done = true; break; }        // aligned within ~2 synth steps
    if (havePrev && fabsf(prevStep) > 50.0f) {
      float g = (prevErr - err) / prevStep;                 // change of measured error per Hz moved
      if (fabsf(g) > 0.2f && fabsf(g) < 5.0f) gain = g;
    }
    float step = constrain(err / gain, -40000.0f, 40000.0f);
    int32_t before = corrHz;
    corrHz += (int32_t)lroundf(step);
    prevStep = (float)(corrHz - before);
    prevErr = err; havePrev = true;
  }
  int16_t st2 = radio.calExit(curFreq());
  ledIdle();
  warnIf(st, "calibration receive");
  warnIf(st2, "restart receiver");
  if (!heard) {
    corrHz = startCorr;
    emitCal("{\"role\":\"sync\",\"state\":\"done\",\"ok\":false,\"msg\":\"no calibration signal heard\"}");
    logf("CAL: nothing heard\n");
    return;
  }
  String j = "{\"role\":\"sync\",\"state\":\"done\",\"ok\":" + String(done ? "true" : "false") +
             ",\"corr\":" + String(corrHz) + ",\"change\":" + String(corrHz - startCorr) +
             ",\"residual\":" + String(err, 0);
  if (!done) j += ",\"msg\":\"signal lost before the error was below 120 Hz\"";
  j += "}";
  emitCal(j);
  logf("CAL %+ld Hz\nres %.0f Hz\n", (long)(corrHz - startCorr), err);
  emitCfg();
}

// ---------------- DAPNET ----------------
static void emitDap() {
  uint32_t left = 0;
  int8_t slot = dap.slotNow(&left);
  String j = "{\"on\":" + String(dapOn ? "true" : "false") + ",\"state\":\"" + dap.stateName() +
             "\",\"online\":" + String(dap.state() == Dapnet::ONLINE ? "true" : "false") +
             ",\"slots\":\"" + dap.slotText() + "\",\"slot\":" + String(slot) +
             ",\"q\":" + String(dap.queued()) + ",\"rx\":" + String(dap.rxCount) +
             ",\"tx\":" + String(dap.txCount) + ",\"drop\":" + String(dap.dropCount);
  if (WiFi.status() == WL_CONNECTED)
    j += ",\"ip\":\"" + WiFi.localIP().toString() + "\",\"wrssi\":" + String(WiFi.RSSI());
  j += "}";
  emitEvt("DAP", j);
}

static void dapApply() {                       // (re)start or stop after a settings change
  if (dapOn) dap.start(); else dap.stop();
  dapChanged = true;
}

// send the next DAPNET page if it fits into the current slot
static void serviceDapTx() {
  if (!dapOn) return;
  const DapPage* p = dap.peek();
  if (!p) return;
  String text = p->text;
  char kind;
  if (p->func == 0) kind = 'n';
  else if (p->func == 1 || text.length() == 0) kind = 't';
  else kind = 'a';
  float air = pocsagAirtime(p->ric, kind, text.length());
  if (air > txMax) {
    logf("DTX %lu too long\n", (unsigned long)p->ric);
    dap.pop(); dap.dropCount++;
    return;
  }
  if (!dap.slotOpen(air)) return;
  uint32_t ric = p->ric;
  uint8_t func = p->func;
  dap.pop();
  if (txPage(ric, kind, text, kind == 't' ? RADIOLIB_PAGER_FUNC_AUTO : func, true) == RADIOLIB_ERR_NONE)
    dap.txCount++;
  dapChanged = true;
}

static void dapCommand(String arg) {
  String sub = arg, val;
  int sp = arg.indexOf(' ');
  if (sp > 0) { sub = arg.substring(0, sp); val = arg.substring(sp + 1); val.trim(); }
  sub.toLowerCase();
  if (sub == "" ) { logf("DAPNET %s\n", dapOn ? dap.stateName() : "off"); emitDap(); return; }
  if (sub == "1" || sub == "on" || sub == "0" || sub == "off") {
    dapOn = (sub == "1" || sub == "on");
    if (dapOn && !dap.configured()) emitMsg("ERR", "set /wifi, /dapnet name and /dapnet key first");
    dapApply();
  }
  else if (sub == "name") {
    if (val.length() < 3 || val.length() > 20) { emitMsg("ERR", "use /dapnet name <transmitter name>"); return; }
    val.toLowerCase(); dap.name = val; if (dapOn) dapApply();
  }
  else if (sub == "key") {
    if (val.length() < 4 || val.indexOf(' ') >= 0) { emitMsg("ERR", "use /dapnet key <auth key>"); return; }
    dap.key = val; if (dapOn) dapApply();
    emitMsg("OK", "auth key set");
  }
  else if (sub == "server") {
    int c = val.lastIndexOf(':');
    String h = c > 0 ? val.substring(0, c) : val;
    long pt = c > 0 ? val.substring(c + 1).toInt() : 43434;
    if (!h.length() || pt <= 0 || pt > 65535) { emitMsg("ERR", "use /dapnet server host[:port]"); return; }
    dap.host = h; dap.port = (uint16_t)pt; if (dapOn) dapApply();
  }
  else { emitMsg("ERR", "use /dapnet [0|1|name N|key K|server host:port]"); return; }
  dapRev++;
  emitCfg();
}

static void wifiCommand(String arg) {
  if (arg == "-") { dap.ssid = ""; dap.pass = ""; }
  else {
    // "SSID|PASSWORD" (SSID may contain spaces), or "SSID PASSWORD" split at the last space
    if (!arg.length()) { emitMsg("ERR", "use /wifi SSID PASSWORD (or /wifi - to clear)"); return; }
    int sp = arg.indexOf('|');
    if (sp < 0) sp = arg.lastIndexOf(' ');
    if (sp > 0) { dap.ssid = arg.substring(0, sp); dap.pass = arg.substring(sp + 1); }
    else { dap.ssid = arg; dap.pass = ""; }
  }
  if (dapOn) dapApply();
  dapRev++;
  emitMsg("OK", dap.ssid.length() ? "WiFi set" : "WiFi cleared");
  emitCfg();
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
    "  /calbeacon [n]  send n calibration packets (base board, default 40, ~6 s)\n"
    "  /calsync [s]    tune to a base board's /calbeacon, wait up to s seconds (default 30)\n"
    "  /rssi 0|1       stream live signal level for 2 minutes\n"
    "  /ric N          own RIC (receive filter)\n"
    "  /mon 0|1        1 = show pages for every RIC\n"
    "  /pwr dBm        TX power (see /cfg pmin..pmax)\n"
    "  /call CALL      station callsign (/call - to clear)\n"
    "  /autoid 0|1     put the callsign in front of text pages\n"
    "  /log            replay stored received pages   /logclear  forget them\n"
    "  /led 0|1        status LED patterns\n"
    "  /ble 0|1        Bluetooth LE console   /blepin NNNNNN   /bleforget\n"
    "  /wifi SSID PASS WiFi for DAPNET (SSID|PASS if the SSID has spaces; /wifi - clears)\n"
    "  /dapnet [0|1]   DAPNET transmitter link on/off; no argument = status\n"
    "  /dapnet name N  transmitter name as registered   /dapnet key K  its auth key\n"
    "  /dapnet server host[:port]  (default dapnet.afu.rwth-aachen.de:43434)\n"
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
  else if (cmd == "/cfg") { emitCfg(); emitDap(); }
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
    prefs.putUInt("dest", lastDest);
    prefs.putBool("dapon", dapOn);
    prefs.putString("wssid", dap.ssid);
    prefs.putString("wpass", dap.pass);
    prefs.putString("daphost", dap.host);
    prefs.putUShort("dapport", dap.port);
    prefs.putString("dapname", dap.name);
    prefs.putString("dapkey", dap.key);
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
  else if (cmd == "/calbeacon") { calBeacon(arg.length() ? n : 40); }
  else if (cmd == "/calsync")   { calSync(arg.length() ? n : 30); }
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
  else if (cmd == "/wifi")   { wifiCommand(arg); }
  else if (cmd == "/dapnet") { dapCommand(arg); }
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
  logf("%s\n", BOARD_NAME);                 // shows which board profile was compiled in

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
  lastDest   = prefs.getUInt("dest", DEFAULT_DEST);
  dapOn      = prefs.getBool("dapon", false);
  dap.ssid   = prefs.getString("wssid", "");
  dap.pass   = prefs.getString("wpass", "");
  dap.host   = prefs.getString("daphost", dap.host);
  dap.port   = prefs.getUShort("dapport", dap.port);
  dap.name   = prefs.getString("dapname", "");
  dap.key    = prefs.getString("dapkey", "");
  txMax      = (uint8_t)constrain((int)prefs.getUChar("txmax", DEFAULT_TXMAX), TXMAX_MIN, TXMAX_MAX);

  msgLog.begin();
  int16_t rs = radio.begin(curFreq(), txPwr, rxBw, myRic, rxMask(), onPage);
#if defined(RADIO_SX126X)
  if (rs == RADIOLIB_ERR_CHIP_NOT_FOUND) {       // show what is on the SPI bus before halting
    char l1[20], l2[20], l3[20];
    sx126xProbe(l1, l2, l3, sizeof l1);
    logf("chip not found\n%s\n%s\n%s\n", l1, l2, l3);
  }
#endif
  check(rs, "radio init");

  logf("%.4f MHz\ncorr %+ld Hz\nBW %.1f kHz\nRIC %lu%s\n", curFreq(), (long)corrHz, rxBw,
       (unsigned long)myRic, monAll ? " +mon" : "");
  if (bleEnabled) bleStart();
  dap.onLog = [](const char* l) { logf("%s\n", l); };
  dap.onChange = []() { dapChanged = true; };
  if (dapOn) dap.start();
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
  dap.service();
  serviceDapTx();
  static uint32_t dapEmitted = 0;
  if (dapChanged && millis() - dapEmitted > 300) { dapChanged = false; dapEmitted = millis(); emitDap(); }
  serviceRssi();
  serviceFlash();
  msgLog.service();

  if (Serial.available()) processLine(Serial.readStringUntil('\n'));

  String bl;
  if (ble.readLine(bl)) processLine(bl);
  static bool bleWasConnected = false;
  if (ble.connected() != bleWasConnected) {
    bleWasConnected = ble.connected();
    logf(bleWasConnected ? "BLE connected\n" : "BLE disconnected\n");
    if (bleWasConnected) { emitCfg(); emitDap(); }
    else rssiUntil = 0;                         // stop streaming when the phone leaves
  }
}
