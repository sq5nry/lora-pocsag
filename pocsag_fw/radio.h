/*
  POCSAG radio layer: same interface for both boards.

    SX1278 (T3)    : TX and RX with RadioLib PagerClient (RX in direct mode, DIO1 clock / DIO2 data)
    SX1262 (Heltec), SX1268 (E22): TX with PagerClient; RX via the GFSK packet engine +
                     pocsag_rx.h decoder, because the SX126x has no direct-mode outputs.

  TX timeout: PagerClient::transmit() blocks and can't be cancelled, so the chip class is wrapped
  in TxGuard. Once abortTx is set (from a timer), the next per-bit transmitDirect() call puts the
  chip in standby and every later one is ignored, so RF stops within one bit period.
*/
#pragma once
#include <Arduino.h>
#include <SPI.h>
#include <RadioLib.h>
#include "board.h"
#include "pocsag_rx.h"

#define POCSAG_BAUD    1200
#define POCSAG_SHIFT   4500      // Hz
#define POCSAG_INVERT  false     // flip if your pagers / other boards can't decode

struct RxPage {
  uint32_t ric;
  int8_t   func;         // -1 = unknown (T3: RadioLib doesn't report it)
  const char* text;
  int8_t   numeric;      // -1 unknown, 0 alpha, 1 numeric
  int16_t  fixes, bad;   // -1 unknown
  float    rssi;         // NAN unknown
};
typedef void (*PageCb)(const RxPage&);

template <class R> class TxGuard : public R {
 public:
  using R::R;
  volatile bool abortTx = false;
  bool aborted = false;
  int16_t transmitDirect(uint32_t frf = 0) override {
    if (abortTx) {
      if (!aborted) { aborted = true; R::standby(); }
      return RADIOLIB_ERR_TX_TIMEOUT;
    }
    return R::transmitDirect(frf);
  }
};

// Airtime of a page as RadioLib sends it: 18 preamble words + 17 words per batch, 32 bit each
static inline float pocsagAirtime(uint32_t ric, char kind, size_t len) {
  size_t sym = (kind == 'n') ? 4 : 7;
  size_t blocks = (kind == 't') ? 0 : (len * sym + 19) / 20;
  size_t batches = (2 * (ric & 7) + blocks + 16) / 16;
  return (18 + 17 * batches) * 32.0f / POCSAG_BAUD;
}

class PocsagRadio {
 public:
  // freq MHz, power dBm, bw kHz (chip units, see board.h), receive filter
  int16_t begin(float freq, int8_t pwr, float bw, uint32_t ric, uint32_t mask, PageCb cb);
  int16_t setFreq(float freq)              { freq_ = freq; chip_.standby();
                                             int16_t s = pager_.begin(freq_, POCSAG_BAUD, POCSAG_INVERT, POCSAG_SHIFT);
                                             return s ? s : startRx(); }
  int16_t setFilter(uint32_t ric, uint32_t mask) { ric_ = ric; mask_ = mask; return restartRx(); }
  int16_t setPower(int8_t pwr)             { pwr_ = pwr; chip_.standby(); int16_t s = chip_.setOutputPower(pwr);
                                             int16_t r = startRx(); return s ? s : r; }
  int16_t setRxBw(float bw)                { bw_ = bw; return restartRx(); }
  float   rxBw() const                     { return bw_; }
  // blocking page transmission; kind 'a' alpha, 'n' numeric, 't' tone;
  // func = POCSAG function bits 0..3, or automatic from the kind
  int16_t transmit(uint32_t ric, char kind, String& text, uint8_t func = RADIOLIB_PAGER_FUNC_AUTO);
  void    abortTx()                        { chip_.abortTx = true; }   // safe from a timer callback
  int16_t carrierOn();
  int16_t carrierOff()                     { return startRx(); }
  float   rssiNow();
  void    poll();                          // call from loop()

  // ---- board-to-board frequency alignment (LoRa mode, see calEnter) ----
  // Temporarily switch the chip to LoRa: its demodulator measures the frequency error of each
  // received packet, which FSK reception can't do on both chips. calExit() restores POCSAG.
  int16_t calEnter(float freq);
  int16_t calExit(float freq)              { return begin(freq, pwr_, bw_, ric_, mask_, cb_); }
  int16_t calBeacon(const char* call, uint8_t count);
  // average error (Hz) over up to n beacon packets on freq; firstWaitMs for the first packet,
  // nextWaitMs for each further one. got = packets used.
  int16_t calMeasure(float freq, uint8_t n, uint32_t firstWaitMs, uint32_t nextWaitMs, float& err, uint8_t& got);

 private:
  int16_t startRx();
  int16_t restartRx()                      { chip_.standby(); return startRx(); }
  float freq_ = 439.9875f, bw_ = BW_DEFAULT;
  int8_t pwr_ = 10;
  uint32_t ric_ = 0, mask_ = 0x1FFFFF;
  PageCb cb_ = nullptr;

#if defined(BOARD_T3_V161)
  TxGuard<SX1278> chip_{new Module(PIN_NSS, PIN_DIO0, PIN_RST, PIN_DIO1)};
  PagerClient pager_{&chip_};
  int lastAvail_ = 0;
  uint32_t availSince_ = 0;
  void prepTx() {
    // In continuous mode the SX1278 FSK-modulates whatever is on DIO2. After RX that is a
    // floating ESP input and the deviation is 4.5 kHz -> pin it low, minimum deviation.
    chip_.standby();
    pinMode(PIN_DIO2, OUTPUT);
    digitalWrite(PIN_DIO2, LOW);
    chip_.setFrequencyDeviation(-1);
  }
#elif defined(RADIO_SX126X)
  TxGuard<RADIO_CHIP> chip_{new Module(PIN_NSS, PIN_DIO1, PIN_RST, PIN_BUSY)};
  PagerClient pager_{&chip_};
  static const uint8_t RX_BATCHES = 3;    // batches per packet (max 3: 200 B <= 255 B)
  static const uint8_t SYNC_LEN   = 3;    // last 3 bytes of FSC: tolerant to the restart gap
  static const size_t  PKT_LEN    = 64 + 68 * (RX_BATCHES - 1);
  static volatile bool pktFlag_;
  static PocsagRadio*  self_;
  uint32_t lastPktMs_ = 0;
  pocsag::Decoder decoder_{&PocsagRadio::onDecoded};
  static void onPacket();     // ISR, defined out of line: IRAM_ATTR on an in-class (inline)
                              // function breaks the ESP32-S3 link ("dangerous relocation: l32r")
  static void onDecoded(uint32_t ric, uint8_t func, const char* text, bool numeric, int fixes, int bad);
  void prepTx() { chip_.standby(); }
#endif
};

// ======================================================================================
#if defined(BOARD_T3_V161)

int16_t PocsagRadio::begin(float freq, int8_t pwr, float bw, uint32_t ric, uint32_t mask, PageCb cb) {
  freq_ = freq; bw_ = bw; ric_ = ric; mask_ = mask; cb_ = cb; pwr_ = pwr;
  // T3 uses non-default SPI pins; default VSPI (SCK 18, MOSI 23) would clash with NSS/RESET
  SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI, PIN_NSS);
  ConfigFSK_t c;
  c.frequency = freq;
  int16_t s = chip_.beginFSK(c);                  if (s) return s;
  s = chip_.setOutputPower(pwr);                  if (s) return s;
  s = pager_.begin(freq_, POCSAG_BAUD, POCSAG_INVERT, POCSAG_SHIFT); if (s) return s;
  return startRx();
}

int16_t PocsagRadio::startRx() {
  // RxBw is not touched by PagerClient; RadioLib's FSK default would be 125 kHz
  int16_t s = chip_.setRxBandwidth(bw_);          if (s) return s;
  // also restores RX frequency, 4.5 kHz deviation and DIO2 as input
  return pager_.startReceive(PIN_DIO2, ric_, mask_);
}

int16_t PocsagRadio::transmit(uint32_t ric, char kind, String& text, uint8_t func) {
  prepTx();
  chip_.abortTx = false; chip_.aborted = false;
  int16_t s;
  if (kind == 't')      s = pager_.sendTone(ric);
  else if (kind == 'n') s = pager_.transmit(text, ric, RADIOLIB_PAGER_BCD, func);
  else                  s = pager_.transmit(text, ric, RADIOLIB_PAGER_ASCII, func);
  bool aborted = chip_.aborted;
  chip_.abortTx = false;
  startRx();
  return aborted ? RADIOLIB_ERR_TX_TIMEOUT : s;
}

int16_t PocsagRadio::carrierOn() {
  prepTx();
  int16_t s = chip_.setFrequency(freq_);          if (s) return s;
  return chip_.transmitDirect();
}

float PocsagRadio::rssiNow() { return chip_.getRSSI(false, true); }   // skipReceive: keep direct RX running

void PocsagRadio::poll() {
  // RadioLib buffers whole batches. Read once the transmission is over (no new batch for
  // ~1.5 batch times) or the buffer is getting full (direct buffer holds ~3.7 batches).
  int a = pager_.available();
  if (a != lastAvail_) { lastAvail_ = a; availSince_ = millis(); }
  if (a <= 0) return;
  if (a < 3 && millis() - availSince_ < 700) return;
  String msg;
  uint32_t addr = 0;
  int16_t s = pager_.readData(msg, 0, &addr);
  lastAvail_ = pager_.available(); availSince_ = millis();
  if (s != RADIOLIB_ERR_NONE || !cb_) return;
  RxPage p = { addr, -1, msg.c_str(), -1, -1, -1, rssiNow() };
  cb_(p);
}

// ======================================================================================
#elif defined(RADIO_SX126X)

// --------------------------------------------------------------------------------------
// Wiring check for SX126x boards, used when begin() fails with RADIOLIB_ERR_CHIP_NOT_FOUND.
// Resets the chip by hand, waits for BUSY, reads the version string at register 0x0320 over
// raw SPI and fills three short lines (16 chars, OLED width):
//   l1 "BUSY r0 1 ok"    BUSY level during reset, after release, and whether it went low
//   l2 "ver SX1268 V2D"  printable version text (dots for non-printable bytes)
//   l3 "53 58 31 32 S80" first 4 bytes in hex + SPI status byte
// Reading: 00 00 .. = chip unpowered or MISO held low; FF FF .. = MISO open or NSS not
// reaching the chip; BUSY stuck at 1 = BUSY/RST wiring or no supply; other text = other chip.
static void sx126xProbe(char* l1, char* l2, char* l3, size_t n) {
  pinMode(PIN_NSS, OUTPUT);  digitalWrite(PIN_NSS, HIGH);
  pinMode(PIN_BUSY, INPUT);
  pinMode(PIN_RST, OUTPUT);  digitalWrite(PIN_RST, LOW);
  delay(2);
  int bRst = digitalRead(PIN_BUSY);
  digitalWrite(PIN_RST, HIGH);
  delay(1);
  int bRel = digitalRead(PIN_BUSY);
  uint32_t t0 = millis();
  while (digitalRead(PIN_BUSY) && millis() - t0 < 200) delay(1);
  bool ready = !digitalRead(PIN_BUSY);
  delay(5);
  uint8_t v[16];
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  digitalWrite(PIN_NSS, LOW);
  SPI.transfer(0x1D);                               // ReadRegister
  SPI.transfer(0x03); SPI.transfer(0x20);           // address 0x0320
  uint8_t st = SPI.transfer(0x00);                  // status byte
  for (int i = 0; i < 16; i++) v[i] = SPI.transfer(0x00);
  digitalWrite(PIN_NSS, HIGH);
  SPI.endTransaction();
  char txt[17];
  for (int i = 0; i < 16; i++) txt[i] = (v[i] >= 0x20 && v[i] < 0x7F) ? (char)v[i] : '.';
  txt[16] = 0;
  snprintf(l1, n, "BUSY r%d %d %s", bRst, bRel, ready ? "ok" : "STUCK");
  snprintf(l2, n, "%.16s", txt);
  snprintf(l3, n, "%02X %02X %02X %02X S%02X", v[0], v[1], v[2], v[3], st);
}

volatile bool PocsagRadio::pktFlag_ = false;
PocsagRadio*  PocsagRadio::self_ = nullptr;

void IRAM_ATTR PocsagRadio::onPacket() { pktFlag_ = true; }

int16_t PocsagRadio::begin(float freq, int8_t pwr, float bw, uint32_t ric, uint32_t mask, PageCb cb) {
  freq_ = freq; bw_ = bw; ric_ = ric; mask_ = mask; cb_ = cb; pwr_ = pwr; self_ = this;
  SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI, PIN_NSS);
  chip_.tcxoVoltage = TCXO_VOLTAGE;
#if defined(PIN_RXEN)
  // external RX switch / LNA enable: RadioLib raises it in RX and drops it in TX and standby.
  // TX side of the switch is DIO2 (setDio2AsRfSwitch below).
  chip_.setRfSwitchPins(PIN_RXEN, RADIOLIB_NC);
#endif
  ConfigFSK_t c;
  c.frequency = freq;
  c.bitRate = POCSAG_BAUD / 1000.0f;
  c.frequencyDeviation = POCSAG_SHIFT / 1000.0f;
  c.receiverBandwidth = bw;
  c.power = pwr;
  int16_t s = chip_.beginFSK(c);                  if (s) return s;
  s = chip_.setDio2AsRfSwitch(true);              if (s) return s;
  s = pager_.begin(freq_, POCSAG_BAUD, POCSAG_INVERT, POCSAG_SHIFT); if (s) return s;
  return startRx();
}

// (re)apply RX configuration - pager.begin()/transmit() change deviation & mode
int16_t PocsagRadio::startRx() {
  decoder_.setFilter(ric_, mask_);
  int16_t s;
  if ((s = chip_.setFrequency(freq_))) return s;
  if ((s = chip_.setBitRate(POCSAG_BAUD / 1000.0f))) return s;
  if ((s = chip_.setFrequencyDeviation(POCSAG_SHIFT / 1000.0f))) return s;
  if ((s = chip_.setRxBandwidth(bw_))) return s;
  if ((s = chip_.setDataShaping(RADIOLIB_SHAPING_NONE))) return s;
  if ((s = chip_.setEncoding(RADIOLIB_ENCODING_NRZ))) return s;
  if ((s = chip_.setCRC(0))) return s;
  if ((s = chip_.setWhitening(false))) return s;
  // chip treats high frequency as '1', POCSAG uses low frequency as '1'
  // -> with POCSAG_INVERT=false look for the inverted sync word and invert the payload
  uint32_t fsc = POCSAG_INVERT ? pocsag::FSC : ~pocsag::FSC;
  uint8_t sync[4] = { (uint8_t)(fsc >> 24), (uint8_t)(fsc >> 16), (uint8_t)(fsc >> 8), (uint8_t)fsc };
  if ((s = chip_.setSyncWord(sync + (4 - SYNC_LEN), SYNC_LEN))) return s;
  // preamble length 0 -> preamble detector off: sync search runs continuously,
  // so batches 2, 3, ... (no preamble in front of them) are caught too
  if ((s = chip_.setPreambleLength(0))) return s;
  if ((s = chip_.fixedPacketLengthMode(PKT_LEN))) return s;
  chip_.setPacketReceivedAction(onPacket);
  return chip_.startReceive();                     // RX continuous
}

int16_t PocsagRadio::transmit(uint32_t ric, char kind, String& text, uint8_t func) {
  prepTx();
  chip_.abortTx = false; chip_.aborted = false;
  int16_t s;
  if (kind == 't')      s = pager_.sendTone(ric);
  else if (kind == 'n') s = pager_.transmit(text, ric, RADIOLIB_PAGER_BCD, func);
  else                  s = pager_.transmit(text, ric, RADIOLIB_PAGER_ASCII, func);
  bool aborted = chip_.aborted;
  chip_.abortTx = false;
  chip_.standby();
  startRx();
  return aborted ? RADIOLIB_ERR_TX_TIMEOUT : s;
}

int16_t PocsagRadio::carrierOn() {
  prepTx();
  int16_t s = chip_.setFrequency(freq_);          if (s) return s;
  return chip_.transmitDirect();
}

float PocsagRadio::rssiNow() { return chip_.getRSSI(false); }   // instantaneous

void PocsagRadio::onDecoded(uint32_t ric, uint8_t func, const char* text, bool numeric, int fixes, int bad) {
  if (!self_ || !self_->cb_) return;
  RxPage p = { ric, (int8_t)func, text, (int8_t)(numeric ? 1 : 0), (int16_t)fixes, (int16_t)bad,
               self_->chip_.getRSSI(true) };
  self_->cb_(p);
}

static inline uint32_t be32(const uint8_t* p) {
  return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) | ((uint32_t)p[2] << 8) | p[3];
}

void PocsagRadio::poll() {
  if (pktFlag_) {
    pktFlag_ = false;
    uint8_t buf[PKT_LEN];
    int16_t st = chip_.readData(buf, PKT_LEN);
    lastPktMs_ = millis();
    if (st == RADIOLIB_ERR_NONE) {
      if (!POCSAG_INVERT) for (size_t i = 0; i < PKT_LEN; i++) buf[i] = ~buf[i];
      size_t off = 0;
      bool ended = false;
      for (uint8_t b = 0; b < RX_BATCHES && !ended; b++) {
        if (b > 0) {                     // next batch must start with FSC, else transmission ended
          uint32_t s = be32(buf + off); off += 4;
          if (__builtin_popcount(s ^ pocsag::FSC) > 3) { decoder_.flush(); ended = true; break; }
        }
        for (uint8_t i = 0; i < 16; i++, off += 4) decoder_.feed(be32(buf + off), i);
      }
    }
  }
  // no follow-up packet within ~1 packet time -> close any open message
  const uint32_t pktMs = (PKT_LEN + 4) * 8 * 1000UL / POCSAG_BAUD;
  if (decoder_.active() && millis() - lastPktMs_ > pktMs + 500) decoder_.flush();
}
#endif

// ======================================================================================
// Board-to-board alignment, common to both chips
static const char CAL_TAG[] = "PCAL ";              // beacon payload: "PCAL <callsign>"

int16_t PocsagRadio::calEnter(float freq) {
  chip_.standby();
  ConfigLoRa_t c;
  c.frequency = freq;
  c.bandwidth = 125.0f;        // captures offsets up to about +-30 kHz
  c.spreadingFactor = 7;       // ~50 ms per beacon packet
  c.codingRate = 5;
  c.syncWord = RADIOLIB_LORA_SYNC_WORD_PRIVATE;
  c.power = pwr_;
  c.preambleLength = 8;
  int16_t s = chip_.begin(c);
#if defined(RADIO_SX126X)
  if (!s) s = chip_.setDio2AsRfSwitch(true);
#endif
  return s;
}

int16_t PocsagRadio::calBeacon(const char* call, uint8_t count) {
  char msg[24];
  snprintf(msg, sizeof msg, "%s%s", CAL_TAG, call && call[0] ? call : "NOCALL");
  int16_t s = RADIOLIB_ERR_NONE;
  for (uint8_t i = 0; i < count; i++) {
    s = chip_.transmit((const uint8_t*)msg, strlen(msg));
    if (s) break;
    delay(100);
  }
  return s;
}

int16_t PocsagRadio::calMeasure(float freq, uint8_t n, uint32_t firstWaitMs, uint32_t nextWaitMs,
                                float& err, uint8_t& got) {
  err = 0; got = 0;
  int16_t s = chip_.setFrequency(freq);
  if (s) return s;
  float sum = 0;
  uint32_t deadline = millis() + firstWaitMs;
  uint8_t buf[32];
  while (got < n && (int32_t)(millis() - deadline) < 0) {
    uint32_t left = deadline - millis();
    s = chip_.receive(buf, sizeof buf, left > 1000 ? 1000 : left);   // short slices keep it responsive
    if (s == RADIOLIB_ERR_NONE && memcmp(buf, CAL_TAG, sizeof CAL_TAG - 1) == 0) {
      sum += chip_.getFrequencyError();
      got++;
      deadline = millis() + nextWaitMs;      // next packet should follow soon
    }
  }
  if (got) err = sum / got;
  return RADIOLIB_ERR_NONE;
}
