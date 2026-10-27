/*
  DAPNET transmitter client: the board logs into the DAPNET core like a hotspot and sends the
  pages DAPNET routes to it, inside the time slots DAPNET assigns.

  Protocol: the legacy line protocol of the DAPNET core (as used by DAPNETGateway / UniPager),
  TCP to dapnet.afu.rwth-aachen.de:43434, lines end in CR LF.
    board  -> core   [lora-pocsag v1.0 <transmitter name> <auth key>]
    core   -> board  2:<time>               time sync, the first one also means "logged in"
                     board answers the same line + ":0000", then "+"
                     3...                   board answers "+"
                     4:<hex digits>         allowed slots, e.g. 4:37BF = slots 3, 7, 11, 15; "+"
                     7...                   login rejected; retry with growing back-off
                     #NN T:S:RIC:F:text     page: NN message id (hex), T type (5 numeric, 6 alpha),
                                            S speed (1 = 1200 bps), RIC in hex, F function 0..3;
                                            board answers "#<NN+1> +" (or "-" if malformed)
  Slots: time is cut into 16 slots of 6.4 s (102.4 s cycle) counted from the Unix epoch (UTC);
  a page may only go out if it ends inside one of the allowed slots. The ESP32 clock comes
  from NTP.
*/
#pragma once
#include <Arduino.h>
#include <WiFi.h>
#include <time.h>
#include <sys/time.h>

struct DapPage {
  uint32_t ric;
  uint8_t  type;       // 5 numeric, 6 alphanumeric
  uint8_t  func;       // POCSAG function bits 0..3
  uint32_t queuedMs;   // millis() when it arrived
  char     text[84];   // DAPNET limits pages to 80 characters
};

class Dapnet {
 public:
  static const uint8_t  QLEN        = 24;
  static const uint32_t SLOT_MS     = 6400;
  static const uint32_t TIME_MSG_MS = 15000;   // time broadcasts older than this are stale
  enum State : uint8_t { OFF, NO_CONFIG, WIFI, CLOCK, CONNECT, LOGIN, ONLINE, RETRY };

  // ---- configuration (kept in flash by the sketch) ----
  String   ssid, pass, host = "dapnet.afu.rwth-aachen.de", name, key;
  uint16_t port = 43434;

  void (*onLog)(const char* line) = nullptr;   // short status lines for OLED + console
  void (*onChange)() = nullptr;                // state, slots or queue changed

  bool configured() const { return ssid.length() && name.length() && key.length() && host.length(); }

  void start() {
    stopNet();
    if (!configured()) { setState(NO_CONFIG); return; }
    WiFi.mode(WIFI_STA);
    WiFi.setSleep(true);                       // needed when Bluetooth runs at the same time
    WiFi.setAutoReconnect(true);
    WiFi.begin(ssid.c_str(), pass.c_str());
    clockStarted_ = false;
    failCount_ = 0;
    setState(WIFI);
  }

  void stop() { stopNet(); WiFi.disconnect(true); WiFi.mode(WIFI_OFF); setState(OFF); }

  void service() {
    if (st_ == OFF || st_ == NO_CONFIG) return;
    if (WiFi.status() != WL_CONNECTED) {
      if (st_ != WIFI) { closeTcp(); setState(WIFI); log("WiFi lost"); }
      return;
    }
    switch (st_) {
      case WIFI:
        if (!clockStarted_) { configTzTime("UTC0", "pool.ntp.org", "time.google.com"); clockStarted_ = true; }
        logf("WiFi %s", WiFi.localIP().toString().c_str());
        setState(CLOCK);
        break;
      case CLOCK:
        if (clockValid()) setState(CONNECT);
        break;
      case CONNECT: {
        logf("DAPNET connect");
        client_.setTimeout(5);
        if (!client_.connect(host.c_str(), port, 5000)) { log("DAPNET no connection"); retry(); break; }
        client_.setNoDelay(true);
        String nm = name; nm.toLowerCase();
        char login[160];
        snprintf(login, sizeof login, "[lora-pocsag v1.0 %s %s]\r\n", nm.c_str(), key.c_str());
        client_.print(login);
        lineLen_ = 0;
        stateSince_ = lastRx_ = millis();
        setState(LOGIN);
        break;
      }
      case LOGIN:
      case ONLINE:
        readLines();
        if (st_ != LOGIN && st_ != ONLINE) break;            // a line made us drop the link
        if (!client_.connected()) { log("DAPNET link lost"); retry(); break; }
        if (st_ == LOGIN && millis() - stateSince_ > 30000) { log("DAPNET no answer"); retry(); }
        else if (millis() - lastRx_ > 600000UL) { log("DAPNET silent, reconnect"); retry(); }
        break;
      case RETRY:
        if ((int32_t)(millis() - retryAt_) >= 0) setState(CONNECT);
        break;
      default: break;
    }
  }

  // ---- state for the console ----
  State state() const { return st_; }
  const char* stateName() const {
    static const char* n[] = { "off", "not configured", "joining WiFi", "setting clock",
                               "connecting", "logging in", "online", "waiting to retry" };
    return n[st_];
  }
  bool clockValid() const { return time(nullptr) > 1700000000L; }
  bool haveSlots() const { return haveSlots_; }
  uint16_t slotMask() const { return slots_; }
  String slotText() const {                    // e.g. "37BF"
    String s;
    for (uint8_t i = 0; i < 16; i++) if (slots_ & (1u << i)) s += "0123456789ABCDEF"[i];
    return s;
  }
  // current slot 0..15 and ms left in it; -1 without a valid clock
  int8_t slotNow(uint32_t* msLeft = nullptr) const {
    if (!clockValid()) return -1;
    struct timeval tv; gettimeofday(&tv, nullptr);
    uint64_t ms = (uint64_t)tv.tv_sec * 1000ULL + tv.tv_usec / 1000;
    if (msLeft) *msLeft = SLOT_MS - (uint32_t)(ms % SLOT_MS);
    return (int8_t)((ms / SLOT_MS) % 16);
  }
  uint32_t rxCount = 0, txCount = 0, dropCount = 0;

  // ---- queue ----
  uint8_t queued() const { return qn_; }
  // next page to send, or nullptr; stale time broadcasts are dropped on the way
  const DapPage* peek() {
    while (qn_) {
      const DapPage& p = q_[qh_];
      if (isTimeMsg(p) && millis() - p.queuedMs > TIME_MSG_MS) { pop(); dropCount++; continue; }
      return &p;
    }
    return nullptr;
  }
  void pop() { if (qn_) { qh_ = (qh_ + 1) % QLEN; qn_--; changed(); } }
  // may a transmission of airtimeS seconds start now and end inside an allowed slot?
  bool slotOpen(float airtimeS) const {
    if (!haveSlots_ || !slots_) return false;
    uint32_t left;
    int8_t s = slotNow(&left);
    if (s < 0 || !(slots_ & (1u << s))) return false;
    return airtimeS * 1000.0f + 150.0f < (float)left;   // 150 ms for keying the PA and margin
  }

  // public for tests: feed one received line (without CR/LF)
  void handleLine(char* l) {
    lastRx_ = millis();
    switch (l[0]) {
      case '+': break;
      case '-': log("DAPNET reports error"); break;
      case '2':
        if (st_ == LOGIN) { setState(ONLINE); failCount_ = 0; log("DAPNET online"); }
        send(l); send(":0000\r\n"); send("+\r\n");
        break;
      case '3': send("+\r\n"); break;
      case '4': {
        uint16_t m = 0;
        const char* p = strchr(l, ':');
        for (p = p ? p + 1 : l + 1; *p; p++) {
          char c = toupper(*p);
          if (c >= '0' && c <= '9') m |= 1u << (c - '0');
          else if (c >= 'A' && c <= 'F') m |= 1u << (c - 'A' + 10);
        }
        slots_ = m; haveSlots_ = true;
        logf("slots %s", slotText().c_str());
        send("+\r\n");
        changed();
        break;
      }
      case '7':
        log("DAPNET login refused");
        send("+\r\n");
        retry();
        break;
      case '#': parsePage(l); break;
      default: send("-\r\n"); break;
    }
  }

 private:
  WiFiClient client_;
  State    st_ = OFF;
  bool     clockStarted_ = false, haveSlots_ = false, fullLogged_ = false;
  uint16_t slots_ = 0;
  uint32_t stateSince_ = 0, lastRx_ = 0, retryAt_ = 0;
  uint8_t  failCount_ = 0;
  char     line_[300];
  size_t   lineLen_ = 0;
  DapPage  q_[QLEN];
  uint8_t  qh_ = 0, qn_ = 0;

  static bool isTimeMsg(const DapPage& p) {
    return (p.type == 5 && p.func == 0) || (p.type == 6 && p.func == 3 && !strncmp(p.text, "XTIME=", 6));
  }
  void setState(State s) { if (s != st_) { st_ = s; stateSince_ = millis(); changed(); } }
  void changed() { if (onChange) onChange(); }
  void log(const char* s) { if (onLog) onLog(s); }
  void logf(const char* fmt, ...) {
    char b[64]; va_list ap; va_start(ap, fmt); vsnprintf(b, sizeof b, fmt, ap); va_end(ap); log(b);
  }
  void send(const char* s) { if (client_.connected()) client_.print(s); }
  void closeTcp() { client_.stop(); haveSlots_ = false; slots_ = 0; }
  void stopNet() { closeTcp(); qn_ = 0; qh_ = 0; }
  void retry() {
    static const uint32_t BACKOFF[] = { 2000, 4000, 8000, 10000, 20000, 60000, 120000, 240000, 480000, 600000 };
    closeTcp();
    retryAt_ = millis() + BACKOFF[failCount_];
    if (failCount_ < 9) failCount_++;
    setState(RETRY);
  }

  void readLines() {
    int budget = 1024;                          // don't starve loop() on a flood
    while (client_.available() && budget-- > 0) {
      char c = (char)client_.read();
      if (c == '\r') continue;
      if (c == '\n') {
        line_[lineLen_] = 0;
        if (lineLen_) handleLine(line_);
        lineLen_ = 0;
        if (st_ != LOGIN && st_ != ONLINE) return;
      } else if (lineLen_ < sizeof(line_) - 1) line_[lineLen_++] = c;
    }
  }

  // "#NN T:S:RIC:F:text"
  void parsePage(char* l) {
    char* e;
    unsigned id = strtoul(l + 1, &e, 16);
    char reply[16];
    snprintf(reply, sizeof reply, "#%02X ", (id + 1) & 0xFF);
    char* p = e;
    while (*p == ' ') p++;
    DapPage pg;
    pg.type = (uint8_t)strtoul(p, &e, 10);       if (*e != ':') goto bad; p = e + 1;
    strtoul(p, &e, 10);                          if (*e != ':') goto bad; p = e + 1;   // speed
    pg.ric = strtoul(p, &e, 16);                 if (*e != ':') goto bad; p = e + 1;
    pg.func = (uint8_t)strtoul(p, &e, 10);       if (*e != ':') goto bad; p = e + 1;
    if (pg.ric == 0 || pg.ric > 2097151UL || pg.func > 3) goto bad;
    strncpy(pg.text, p, sizeof(pg.text) - 1); pg.text[sizeof(pg.text) - 1] = 0;
    pg.queuedMs = millis();
    send(reply); send("+\r\n");
    rxCount++;
    if (qn_ == QLEN) { dropCount++; if (!fullLogged_) log("DAPNET queue full"); fullLogged_ = true; return; }
    fullLogged_ = false;
    q_[(qh_ + qn_) % QLEN] = pg; qn_++;
    changed();
    return;
  bad:
    send(reply); send("-\r\n");
  }
};
