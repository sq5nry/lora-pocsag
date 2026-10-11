/*
  Message memory: the last LOG_N received pages, kept in RAM and written to flash (NVS)
  a little while after the last change, so they survive a reboot without wearing the flash.

  Every entry gets an id = (boot number << 16) | sequence, so the web console can tell
  which pages it has already shown. Age is only known for pages from the current boot.
*/
#pragma once
#include <Arduino.h>
#include <Preferences.h>

struct LogEntry {
  uint32_t id;
  uint32_t ms;          // millis() when received (valid only if boot == current boot)
  uint32_t ric;
  int16_t  rssi10;      // dBm * 10, INT16_MIN = unknown
  int16_t  fixes, bad;  // -1 unknown
  uint16_t boot;
  int8_t   func;        // -1 unknown
  int8_t   numeric;     // -1 unknown
  char     text[102];  // pagers rarely show more than 80 characters
};

class MsgLog {
 public:
  static const uint8_t LOG_N = 20;   // ~2.5 KB blob: fits the default 20 KB NVS partition comfortably

  void begin() {
    prefs_.begin("pocsaglog", false);
    boot_ = prefs_.getUShort("boot", 0) + 1;
    prefs_.putUShort("boot", boot_);
    seq_ = 0;
    count_ = prefs_.getUChar("count", 0);
    head_  = prefs_.getUChar("head", 0);
    if (count_ > LOG_N || head_ >= LOG_N ||
        prefs_.getBytes("data", buf_, sizeof buf_) != sizeof buf_) { count_ = 0; head_ = 0; }
  }

  // store a page, returns the stored entry (for its id)
  const LogEntry& add(uint32_t ric, int8_t func, const char* text, int8_t numeric,
                      int16_t fixes, int16_t bad, float rssi) {
    LogEntry& e = buf_[head_];
    e.id = ((uint32_t)boot_ << 16) | (seq_++ & 0xFFFF);
    e.ms = millis(); e.boot = boot_; e.ric = ric; e.func = func; e.numeric = numeric;
    e.fixes = fixes; e.bad = bad;
    e.rssi10 = isnan(rssi) ? INT16_MIN : (int16_t)lroundf(rssi * 10);
    strncpy(e.text, text ? text : "", sizeof e.text - 1); e.text[sizeof e.text - 1] = 0;
    head_ = (head_ + 1) % LOG_N;
    if (count_ < LOG_N) count_++;
    dirtySince_ = millis(); dirty_ = true;
    return e;
  }

  uint8_t count() const { return count_; }
  // i = 0 oldest .. count-1 newest
  const LogEntry& at(uint8_t i) const { return buf_[(head_ + LOG_N - count_ + i) % LOG_N]; }
  // seconds since reception, or -1 if it came from an earlier boot
  long ageSec(const LogEntry& e) const { return e.boot == boot_ ? (long)((millis() - e.ms) / 1000) : -1; }

  void clear() { count_ = 0; head_ = 0; dirty_ = true; dirtySince_ = 0; }

  // call from loop(): writes to flash 20 s after the last change
  void service() {
    if (dirty_ && millis() - dirtySince_ > 20000) {
      prefs_.putBytes("data", buf_, sizeof buf_);
      prefs_.putUChar("count", count_);
      prefs_.putUChar("head", head_);
      dirty_ = false;
    }
  }

 private:
  Preferences prefs_;
  LogEntry buf_[LOG_N];
  uint8_t head_ = 0, count_ = 0;
  uint16_t boot_ = 0;
  uint32_t seq_ = 0, dirtySince_ = 0;
  bool dirty_ = false;
};
