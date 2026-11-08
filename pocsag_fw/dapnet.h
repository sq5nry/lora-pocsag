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
  WiFi: up to MAX_NETS known networks. When not connected the board scans in the background,
  tries the known networks it can see, strongest first, then the ones it can't see (hidden SSIDs),
  each for up to 12 s, and starts over after 30 s if none worked. Nothing here blocks loop().
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
  static const uint8_t MAX_NETS = 8;
  struct Net { String ssid, pass; };
  Net      nets[MAX_NETS];
  uint8_t  netCount = 0;
  String   host = "dapnet.afu.rwth-aachen.de", name, key;
  uint16_t port = 43434;

  int8_t findNet(const String& ssid) const {
    for (uint8_t i = 0; i < netCount; i++) if (nets[i].ssid == ssid) return i;
    return -1;
  }
  // add a network, or change the password of a known one; false if the list is full
  bool addNet(const String& ssid, const String& pass) {
    int8_t i = findNet(ssid);
    if (i >= 0) { nets[i].pass = pass; return true; }
    if (netCount >= MAX_NETS) return false;
    nets[netCount].ssid = ssid; nets[netCount].pass = pass; netCount++;
    return true;
  }
  bool removeNet(uint8_t i) {
    if (i >= netCount) return false;
    for (uint8_t k = i; k + 1 < netCount; k++) nets[k] = nets[k + 1];
    netCount--; nets[netCount] = Net();
    return true;
  }
  void clearNets() { while (netCount) removeNet(netCount - 1); }
  // network being joined (while searching) or in use (when connected), "" otherwise
  String wifiSsid() const {
    if (WiFi.status() == WL_CONNECTED) return WiFi.SSID();
    if (st_ == WIFI && wph_ == W_JOIN && ci_ < nc_) return nets[cand_[ci_]].ssid;
    return "";
  }
  bool wifiSearching() const { return st_ == WIFI && wph_ == W_SCAN; }

  void (*onLog)(const char* line) = nullptr;   // short status lines for OLED + console
  void (*onChange)() = nullptr;                // state, slots or queue changed

  bool configured() const { return netCount && name.length() && key.length() && host.length(); }

  void start() {
    stopNet();
    if (!configured()) { setState(NO_CONFIG); return; }
    WiFi.persistent(false);                    // networks live in our own settings, not the SDK's
    WiFi.mode(WIFI_STA);
    WiFi.setSleep(true);                       // needed when Bluetooth runs at the same time
    WiFi.setAutoReconnect(false);              // network choice is ours (serviceWifi)
    WiFi.disconnect(false);
    clockStarted_ = false;
    failCount_ = 0;
    wph_ = W_IDLE; wifiAt_ = millis();
    setState(WIFI);
  }

  void stop() { stopNet(); WiFi.scanDelete(); WiFi.disconnect(true); WiFi.mode(WIFI_OFF); setState(OFF); }

  void service() {
    if (st_ == OFF || st_ == NO_CONFIG) return;
    if (WiFi.status() != WL_CONNECTED) {
      if (st_ != WIFI) {
        closeTcp(); log("WiFi lost");
        wph_ = W_IDLE; wifiAt_ = millis() + 1000;
        setState(WIFI);
      }
      serviceWifi();
      return;
    }
    switch (st_) {
      case WIFI:
        if (!clockStarted_) { configTzTime("UTC0", "pool.ntp.org", "time.google.com"); clockStarted_ = true; }
        logf("WiFi %s", WiFi.SSID().c_str());
        logf("IP %s", WiFi.localIP().toString().c_str());
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
    static const char* n[] = { "off", "not configured", "looking for WiFi", "setting clock",
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
  // WiFi search: scan -> try candidates in order -> wait -> scan again
  enum WifiPhase : uint8_t { W_IDLE, W_SCAN, W_JOIN };
  WifiPhase wph_ = W_IDLE;
  uint32_t wifiAt_ = 0;                        // W_IDLE: when to scan; W_JOIN: when the try started
  uint8_t  cand_[MAX_NETS], nc_ = 0, ci_ = 0;

  void serviceWifi() {
    switch (wph_) {
      case W_IDLE:
        if ((int32_t)(millis() - wifiAt_) < 0) return;
        WiFi.disconnect(false);
        WiFi.scanNetworks(true);                 // async: returns at once, poll scanComplete()
        wph_ = W_SCAN; changed();
        return;
      case W_SCAN: {
        int16_t n = WiFi.scanComplete();
        if (n == WIFI_SCAN_RUNNING) return;
        // candidates: known networks in range, strongest first, then the rest (maybe hidden)
        bool used[MAX_NETS] = {};
        int16_t rssi[MAX_NETS];
        nc_ = 0;
        for (int16_t i = 0; i < n; i++) {
          int8_t k = findNet(WiFi.SSID(i));
          if (k < 0) continue;
          int16_t r = WiFi.RSSI(i);
          if (used[k]) {                         // same network on several access points: keep the best
            for (uint8_t j = 0; j < nc_; j++) if (cand_[j] == k && r > rssi[j]) rssi[j] = r;
            continue;
          }
          used[k] = true; cand_[nc_] = k; rssi[nc_] = r; nc_++;
        }
        for (uint8_t a = 1; a < nc_; a++)        // insertion sort by signal
          for (uint8_t b = a; b > 0 && rssi[b] > rssi[b - 1]; b--) {
            uint8_t tc = cand_[b]; cand_[b] = cand_[b - 1]; cand_[b - 1] = tc;
            int16_t tr = rssi[b]; rssi[b] = rssi[b - 1]; rssi[b - 1] = tr;
          }
        uint8_t seen = nc_;
        for (uint8_t k = 0; k < netCount; k++) if (!used[k]) cand_[nc_++] = k;
        WiFi.scanDelete();
        if (!seen) log("WiFi: no known network seen");
        ci_ = 0;
        if (nc_) beginCand(); else { wph_ = W_IDLE; wifiAt_ = millis() + 30000; }
        return;
      }
      case W_JOIN:
        if (millis() - wifiAt_ < 12000) return;
        if (++ci_ < nc_) beginCand();
        else { log("WiFi: none joined"); wph_ = W_IDLE; wifiAt_ = millis() + 30000; changed(); }
        return;
    }
  }
  void beginCand() {
    const Net& n = nets[cand_[ci_]];
    WiFi.disconnect(false);
    WiFi.begin(n.ssid.c_str(), n.pass.length() ? n.pass.c_str() : nullptr);
    logf("WiFi try %s", n.ssid.c_str());
    wph_ = W_JOIN; wifiAt_ = millis();
    changed();
  }
  bool     clockStarted_ = false, haveSlots_ = false, fullLogged_ = false;
  uint16_t slots_ = 0;
  uint32_t stateSince_ = 0, lastRx_ = 0, retryAt_ = 0;
  uint8_t  failCount_ = 0;
  char     line_[300];
  size_t   lineLen_ = 0;
  DapPage  q_[QLEN];
  uint8_t  qh_ = 0, qn_ = 0;

  // DAPNET time broadcasts (DAPNET core SkyperProtocol): 2504 numeric HHMMSS DDMMYY (UTC),
  // 200 / 208 Swissphone "XTIME=" (UTC / local), 216 / 224 AlphaPoc "YYYYMMDDHHMMSS..." (UTC / local)
  static bool isTimeMsg(const DapPage& p) {
    switch (p.ric) { case 200: case 208: case 216: case 224: case 2504: return true; }
    return !strncmp(p.text, "XTIME=", 6) || !strncmp(p.text, "YYYYMMDDHHMMSS", 14);
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
