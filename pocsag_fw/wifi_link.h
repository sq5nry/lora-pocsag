/*
  WiFi for DAPNET and the board's own web console.

  Station: up to MAX_NETS known networks. While not connected the board scans in the background,
  tries the known networks it can see, strongest first, then the ones it can't see (hidden
  SSIDs), each for up to 12 s, and starts over after 30 s if none worked.

  Access point: when no known network has been joined for apAfterMs (at once if none are known),
  the board also opens its own network apName / apPass (192.168.4.1), so the web console can be
  reached in the field. While a phone or laptop is connected to it the search pauses (a scan
  would disturb it), except for one round right after the network list changes; once a known network is joined and nobody uses the access point, it closes.
  If nobody connects for apWindowMs it closes too and stays closed until the next start or a
  settings change (saves the battery); with no known networks WiFi is then switched off.

  Nothing here blocks loop().
*/
#pragma once
#include <Arduino.h>
#include <WiFi.h>

class WifiLink {
 public:
  static const uint8_t MAX_NETS = 8;
  struct Net { String ssid, pass; };
  Net     nets[MAX_NETS];
  uint8_t netCount = 0;

  bool     apEnabled = false;                  // set by the sketch (web console on, password set)
  String   apName = "POCSAG", apPass;          // WPA2 needs 8..63 characters
  uint32_t apAfterMs = 45000;
  uint32_t apWindowMs = 600000;                // close the access point after 10 min unused

  void (*onLog)(const char* line) = nullptr;   // short status lines for OLED + console
  void (*onChange)() = nullptr;                // state changed

  // ---- known networks ----
  int8_t findNet(const String& ssid) const {
    for (uint8_t i = 0; i < netCount; i++) if (nets[i].ssid == ssid) return i;
    return -1;
  }
  bool addNet(const String& ssid, const String& pass) {       // false if the list is full
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

  // ---- control ----
  void start() {
    if (on_) return;
    on_ = true;
    WiFi.persistent(false);                    // networks live in our own settings, not the SDK's
    WiFi.mode(WIFI_STA);
    WiFi.setSleep(true);                       // needed when Bluetooth runs at the same time
    WiFi.setAutoReconnect(false);              // network choice is ours
    WiFi.disconnect(false);
    wasUp_ = false; apOn_ = false; apExpired_ = false; asleep_ = false;
    downSince_ = millis();
    wph_ = W_IDLE; wifiAt_ = millis();
    changed();
  }
  void stop() {
    if (!on_) return;
    on_ = false;
    if (apOn_) { WiFi.softAPdisconnect(true); apOn_ = false; }
    WiFi.scanDelete();
    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
    wasUp_ = false;
    changed();
  }
  // the list or the access point settings changed: search again, drop a network that was removed
  void reconsider() {
    if (!on_) return;
    apExpired_ = false;
    tryNow_ = true;                              // one search round even with devices on our AP
    if (asleep_) { asleep_ = false; WiFi.mode(WIFI_STA); downSince_ = millis(); }
    if (up() && findNet(WiFi.SSID()) < 0) WiFi.disconnect(false);
    if (!up()) { wph_ = W_IDLE; wifiAt_ = millis(); }
    if (apOn_ && !apWanted()) apStop();
    changed();
  }

  // ---- state ----
  bool active() const { return on_; }
  bool up() const { return on_ && WiFi.status() == WL_CONNECTED; }
  bool apOn() const { return apOn_; }
  bool asleep() const { return asleep_; }      // on, but radio off: nothing to join, AP window over
  uint8_t apClients() const { return apOn_ ? WiFi.softAPgetStationNum() : 0; }
  bool searching() const { return on_ && !up() && wph_ == W_SCAN; }
  // network in use, or being tried; "" otherwise
  String ssid() const {
    if (up()) return WiFi.SSID();
    if (on_ && wph_ == W_JOIN && ci_ < nc_) return nets[cand_[ci_]].ssid;
    return "";
  }

  void service() {
    if (!on_ || asleep_) return;
    bool upNow = WiFi.status() == WL_CONNECTED;
    if (upNow != wasUp_) {
      wasUp_ = upNow;
      if (upNow) {
        logf("WiFi %s", WiFi.SSID().c_str());
        logf("IP %s", WiFi.localIP().toString().c_str());
      } else {
        log("WiFi lost");
        downSince_ = millis();
        wph_ = W_IDLE; wifiAt_ = millis() + 1000;
      }
      changed();
    }
    if (upNow) {
      tryNow_ = false;
      if (apOn_ && apClients() == 0) apStop();         // a known network is back, AP not needed
      return;
    }
    if (!apOn_ && apWanted() && (netCount == 0 || millis() - downSince_ > apAfterMs)) apStart();
    if (apOn_) {
      if (apClients() > 0) {                                // someone uses the AP: don't scan under them,
        apUsed_ = millis();                                 // unless the network list just changed
        if (!tryNow_) return;
      }
      if (millis() - apUsed_ > apWindowMs) {                // nobody came: close it until next start
        apStop(); apExpired_ = true;
        log("AP closed, unused");
        if (!netCount) { WiFi.mode(WIFI_OFF); asleep_ = true; log("WiFi off"); changed(); return; }
      }
    }
    serviceWifi();
  }

 private:
  enum WifiPhase : uint8_t { W_IDLE, W_SCAN, W_JOIN };
  bool      on_ = false, wasUp_ = false, apOn_ = false, apExpired_ = false, asleep_ = false;
  bool      tryNow_ = false;                           // network list changed: search once regardless
  uint32_t  apUsed_ = 0;                               // AP opened or last seen with a client
  WifiPhase wph_ = W_IDLE;
  uint32_t  wifiAt_ = 0, downSince_ = 0;               // W_IDLE: when to scan; W_JOIN: try start
  uint8_t   cand_[MAX_NETS], nc_ = 0, ci_ = 0;

  void changed() { if (onChange) onChange(); }
  void log(const char* s) { if (onLog) onLog(s); }
  void logf(const char* fmt, ...) {
    char b[64]; va_list ap; va_start(ap, fmt); vsnprintf(b, sizeof b, fmt, ap); va_end(ap); log(b);
  }
  bool apWanted() const { return apEnabled && !apExpired_ && apPass.length() >= 8 && apPass.length() <= 63; }
  void apStart() {
    WiFi.mode(WIFI_AP_STA);
    if (!WiFi.softAP(apName.c_str(), apPass.c_str())) { log("AP start failed"); apEnabled = false; return; }
    apOn_ = true; apUsed_ = millis();
    logf("AP %s", apName.c_str());
    logf("IP %s", WiFi.softAPIP().toString().c_str());
    changed();
  }
  void apStop() {
    WiFi.softAPdisconnect(true);
    WiFi.mode(WIFI_STA);
    apOn_ = false;
    log("AP off");
    changed();
  }

  void serviceWifi() {
    switch (wph_) {
      case W_IDLE:
        if ((int32_t)(millis() - wifiAt_) < 0) return;
        if (!netCount) { wifiAt_ = millis() + 5000; return; }
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
        else { log("WiFi: none joined"); wph_ = W_IDLE; wifiAt_ = millis() + 30000; tryNow_ = false; changed(); }
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
};
