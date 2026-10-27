/*
  Minimal POCSAG codeword decoder (BCH(31,21) + parity check, up to 2-bit correction,
  numeric and alphanumeric messages). Fed one 32-bit codeword at a time together with
  its index (0..15) inside the batch, so address frame position is known.
*/
#pragma once
#include <stdint.h>
#include <string.h>

namespace pocsag {

static const uint32_t FSC  = 0x7CD215D8UL;   // frame sync codeword
static const uint32_t IDLE = 0x7A89C197UL;   // idle codeword

// BCH(31,21) syndrome, generator x^10+x^9+x^8+x^6+x^5+x^3+1 (0x769)
inline uint32_t bchSyndrome(uint32_t cw) {
  uint32_t v = cw >> 1;                       // drop even-parity bit
  for (int i = 30; i >= 10; i--)
    if (v & (1UL << i)) v ^= (0x769UL << (i - 10));
  return v & 0x3FF;
}
inline bool parityOk(uint32_t cw) { return (__builtin_popcount(cw) & 1) == 0; }
inline bool valid(uint32_t cw)    { return bchSyndrome(cw) == 0 && parityOk(cw); }

// 0 = clean, 1 = corrected, -1 = uncorrectable (cw unchanged)
inline int correct(uint32_t& cw) {
  if (valid(cw)) return 0;
  for (int i = 0; i < 32; i++) {
    uint32_t t = cw ^ (1UL << i);
    if (valid(t)) { cw = t; return 1; }
  }
  for (int i = 0; i < 32; i++)
    for (int j = i + 1; j < 32; j++) {
      uint32_t t = cw ^ (1UL << i) ^ (1UL << j);
      if (valid(t)) { cw = t; return 1; }
    }
  return -1;
}

class Decoder {
 public:
  // ric, function bits (0..3), text (tone-only -> ""), numeric?, corrected+bad codeword count
  typedef void (*Callback)(uint32_t ric, uint8_t func, const char* text, bool numeric, int fixes, int bad);

  explicit Decoder(Callback cb) : cb_(cb) {}

  void setFilter(uint32_t addr, uint32_t mask) { fAddr_ = addr; fMask_ = mask; }
  bool active() const { return active_; }

  void feed(uint32_t cw, uint8_t idxInBatch) {
    int r = correct(cw);
    if (r < 0) {                      // uncorrectable: count it, keep bits if inside a message
      if (active_) { bad_++; addBits((cw >> 11) & 0xFFFFF); }
      return;
    }
    if (r > 0 && active_) fixes_++;

    if (cw == IDLE) { flush(); return; }

    if ((cw & 0x80000000UL) == 0) {   // address codeword
      flush();
      uint32_t ric = (((cw >> 13) & 0x3FFFFUL) << 3) | (idxInBatch >> 1);
      uint8_t func = (cw >> 11) & 0x03;
      if ((ric & fMask_) == (fAddr_ & fMask_)) start(ric, func, r > 0);
    } else if (active_) {             // message codeword
      addBits((cw >> 11) & 0xFFFFF);
    }
  }

  // A whole batch (16 codewords after an FSC). Noise that happened to look like an FSC gives
  // mostly uncorrectable words (about 3 in 4), a real batch even at the edge of reception
  // mostly correctable ones: fewer than 8 good words -> treat as noise, end any message.
  // Returns false for noise.
  bool feedBatch(const uint32_t* cw) {
    uint8_t good = 0;
    for (uint8_t i = 0; i < 16; i++) { uint32_t t = cw[i]; if (correct(t) >= 0) good++; }
    if (good < 8) { flush(); return false; }
    for (uint8_t i = 0; i < 16; i++) feed(cw[i], i);
    return true;
  }

  // end the current message (idle, new address, end of transmission)
  void flush() {
    if (!active_) return;
    active_ = false;
    // strip trailing padding / control chars
    while (len_ > 0 && ((uint8_t)text_[len_ - 1] < 0x20 || (numeric_ && text_[len_ - 1] == ' '))) len_--;
    for (size_t i = 0; i < len_; i++) if ((uint8_t)text_[i] < 0x20) text_[i] = '.';
    text_[len_] = 0;
    if (cb_) cb_(ric_, func_, text_, numeric_, fixes_, bad_);
  }

 private:
  void start(uint32_t ric, uint8_t func, bool fixed) {
    active_ = true; ric_ = ric; func_ = func;
    numeric_ = (func == 0);            // DAPNET/common usage: 0 numeric, 3 alpha
    len_ = 0; acc_ = 0; nacc_ = 0; fixes_ = fixed ? 1 : 0; bad_ = 0;
  }

  // 20 data bits, transmitted MSB first; characters are packed LSB first
  void addBits(uint32_t bits20) {
    const uint8_t w = numeric_ ? 4 : 7;
    for (int i = 19; i >= 0; i--) {
      acc_ |= ((bits20 >> i) & 1) << nacc_;
      if (++nacc_ == w) {
        char c;
        if (numeric_) c = "0123456789*U -)("[acc_ & 0x0F];
        else          c = (acc_ < 0x7F) ? (char)acc_ : '.';   // control chars handled in flush()
        if (len_ < sizeof(text_) - 1) text_[len_++] = c;
        acc_ = 0; nacc_ = 0;
      }
    }
  }

  Callback cb_;
  uint32_t fAddr_ = 0, fMask_ = 0;
  bool active_ = false, numeric_ = false;
  uint32_t ric_ = 0; uint8_t func_ = 0;
  char text_[256]; size_t len_ = 0;
  uint32_t acc_ = 0; uint8_t nacc_ = 0;
  int fixes_ = 0, bad_ = 0;
};

}  // namespace pocsag
