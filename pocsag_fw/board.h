/*
  Board profiles, picked from the board selected in the Arduino IDE:
    "TTGO LoRa32-OLED"        -> LilyGO T3 V1.6.1 (SX1278)
    "ESP32 Dev Module"        -> ESP32 DevKit + Ebyte E22-400M30S (SX1268)
    any ESP32-S3 board        -> Heltec WiFi LoRa 32 V3 (SX1262)
    any other plain ESP32     -> LilyGO T3 V1.6.1
  To force a profile regardless of the IDE board, uncomment one line below.
*/
#pragma once
// #define BOARD_T3_V161
// #define BOARD_HELTEC_V3
// #define BOARD_E22_DEVKIT

#if !defined(BOARD_T3_V161) && !defined(BOARD_HELTEC_V3) && !defined(BOARD_E22_DEVKIT)
  #if defined(CONFIG_IDF_TARGET_ESP32S3)
    #define BOARD_HELTEC_V3
  #elif defined(ARDUINO_ESP32_DEV)
    #define BOARD_E22_DEVKIT
  #elif defined(CONFIG_IDF_TARGET_ESP32)
    #define BOARD_T3_V161
  #else
    #error "Unknown board: uncomment a BOARD_... line in board.h"
  #endif
#endif

#if defined(BOARD_T3_V161)
  // ---- LilyGO T3 V1.6.1, ESP32-PICO-D4 + SX1278 (pins checked against T3_V1.6.1 schematic)
  #define BOARD_NAME   "LilyGO T3 V1.6.1 / SX1278"
  #define PIN_SCK      5
  #define PIN_MISO     19
  #define PIN_MOSI     27
  #define PIN_NSS      18     // via R61 0R
  #define PIN_RST      23     // via R60 0R
  #define PIN_DIO0     26
  #define PIN_DIO1     33     // direct-mode bit clock (DCLK), via R16 22R
  #define PIN_DIO2     32     // direct-mode data, via R15 22R
  #define PIN_SDA      21     // OLED D1
  #define PIN_SCL      22     // OLED D0
  #define PIN_OLED_RST U8X8_PIN_NONE   // RC reset on the board
  #define PIN_VEXT     -1
  #define PIN_LED      25     // green, active high
  #define PWR_MIN      2
  #define PWR_MAX      17
  #define PWR_NOTE     "17 dBm (50 mW) is the SX1278 PA_BOOST limit for continuous use. " \
                       "The chip's 20 dBm mode is rated for 1% duty cycle only, too little for long pages."
  #define FREQ_STEP_HZ 61
  // SX127x RxBw is single-sided: total filter width is twice the value
  #define BW_SINGLE_SIDED true
  #define BW_DEFAULT   15.6f
  static const float BW_LIST[] = { 5.2f, 6.3f, 7.8f, 10.4f, 12.5f, 15.6f, 20.8f, 25.0f,
                                   31.3f, 41.7f, 50.0f, 62.5f, 83.3f, 100.0f, 125.0f };

#elif defined(BOARD_HELTEC_V3)
  // ---- Heltec WiFi LoRa 32 V3, ESP32-S3 + SX1262 (433/470-510 MHz variant needed)
  #define BOARD_NAME   "Heltec WiFi LoRa 32 V3 / SX1262"
  #define RADIO_CHIP   SX1262
  #define PIN_SCK      9
  #define PIN_MISO     11
  #define PIN_MOSI     10
  #define PIN_NSS      8
  #define PIN_RST      12
  #define PIN_BUSY     13
  #define PIN_DIO1     14
  #define PIN_SDA      17
  #define PIN_SCL      18
  #define PIN_OLED_RST 21
  #define PIN_VEXT     36     // LOW = OLED powered
  #define PIN_LED      35     // white, active high
  #define TCXO_VOLTAGE 1.8f   // TCXO supplied from DIO3
  #define PWR_MIN      -9
  #define PWR_MAX      22
  #define PWR_NOTE     "22 dBm (160 mW) is the SX1262 maximum. If your board variant is matched for 470-510 MHz, " \
                       "expect a few dB less at 440 MHz. Draws about 120 mA while transmitting."
  #define FREQ_STEP_HZ 30
  // SX126x RxBw is the total (double-sided) filter width
  #define BW_SINGLE_SIDED false
  #define BW_DEFAULT   14.6f
  static const float BW_LIST[] = { 7.3f, 9.7f, 11.7f, 14.6f, 19.5f, 23.4f, 29.3f, 39.0f, 46.9f, 58.6f };

#elif defined(BOARD_E22_DEVKIT)
  // ---- ESP32 DevKit (ESP32-WROOM-32) + Ebyte E22-400M30S (SX1268 + 1 W PA + LNA),
  //      wired as the Meshtastic DIY "E-byte Type 1" schematic, plus one hand-wired change:
  //      E22 DIO2 (pin 8) bridged to TXEN (pin 7), ESP32 IO13 NOT connected to TXEN.
  //      The SX1268 then switches the PA on by itself whenever it transmits (DIO2 as RF switch);
  //      RXEN (LNA path) is driven by RadioLib from IO14. Module VCC from +5 V, not 3.3 V.
  #define BOARD_NAME   "ESP32 DevKit + E22-400M30S / SX1268"
  #define RADIO_CHIP   SX1268
  #define PIN_SCK      5
  #define PIN_MISO     19
  #define PIN_MOSI     27
  #define PIN_NSS      18
  #define PIN_RST      23
  #define PIN_BUSY     32
  #define PIN_DIO1     33     // IRQ
  #define PIN_RXEN     14     // E22 RXEN; TXEN comes from DIO2
  #define PIN_SDA      21
  #define PIN_SCL      22
  #define PIN_OLED_RST U8X8_PIN_NONE
  #define PIN_VEXT     -1
  #define PIN_LED      2      // DevKit on-board LED (no LED on the E22 schematic)
  #define TCXO_VOLTAGE 1.8f   // E22 TCXO from DIO3; if radio init fails, try 0 (crystal)
  #define PWR_MIN      -9
  #define PWR_MAX      22
  #define PWR_NOTE     "Setting is the SX1268 output; the E22 PA adds about 8 dB, so 22 gives about 1 W (30 dBm) " \
                       "with the module on 5 V. Full power draws roughly 0.6 A and heats the module on long pages. " \
                       "Never transmit without an antenna."
  #define FREQ_STEP_HZ 30
  #define BW_SINGLE_SIDED false
  #define BW_DEFAULT   14.6f
  static const float BW_LIST[] = { 7.3f, 9.7f, 11.7f, 14.6f, 19.5f, 23.4f, 29.3f, 39.0f, 46.9f, 58.6f };
#endif

// SX126x boards share one radio implementation (radio.h)
#if defined(BOARD_HELTEC_V3) || defined(BOARD_E22_DEVKIT)
  #define RADIO_SX126X
#endif

static const size_t BW_COUNT = sizeof(BW_LIST) / sizeof(BW_LIST[0]);

// nearest allowed bandwidth for this chip
static inline float bwNearest(float v) {
  float best = BW_LIST[0];
  for (size_t i = 0; i < BW_COUNT; i++) if (fabsf(BW_LIST[i] - v) < fabsf(best - v)) best = BW_LIST[i];
  return best;
}
