# lora-pocsag
70cm LoRA as POCSAG pager

<img src="media/P_20261010_225427.jpg" width="400" alt="LilyGO T3 V1.6.1 (SX1278) and Heltec WiFi LoRa 32 V3 (SX1262) running the POCSAG firmware, status on their OLEDs">

**[Open the web console](https://sq5nry.github.io/lora-pocsag/webui/pocsag_console.html)** to connect to a board over USB or Bluetooth (Chrome or Edge; on Android use Bluetooth).

![Web console: pager-style message list with send, frequency and receiver controls](media/ui.png)

## Firmware

One Arduino sketch, `pocsag_fw/`, for both boards. The board is detected from the chip type
(ESP32 → LilyGO T3 V1.6.1 with SX1278, ESP32-S3 → Heltec WiFi LoRa 32 V3 with SX1262); to force
one, uncomment the line at the top of `pocsag_fw/board.h`.

1. Arduino IDE with the **esp32 by Espressif** board package.
2. Libraries from the Library Manager: **RadioLib**, **U8g2**, **NimBLE-Arduino** (2.x).
3. Board: *TTGO LoRa32-OLED* (or *ESP32 Dev Module*) for the T3, *Heltec WiFi LoRa 32(V3)* for the Heltec.
   If uploads fail, lower the upload speed to 460800 or 115200.
4. Open `pocsag_fw/pocsag_fw.ino` and upload.

After the first start, set your callsign and RIC (web console, or serial at 115200 baud):

```
/call YOURCALL
/ric 1234567
/save
```

Type `/help` for all commands: frequency correction and carrier for calibration, receive filter
width, transmit power and time limit, monitor mode for all RICs, stored pages, Bluetooth.

### Matching the frequency of two boards

Crystal boards (like the T3) can be several kHz off, while POCSAG between two boards stops
working at roughly 3 kHz of difference. To align a board to a reference (ideally the Heltec,
which has a TCXO): on the board to tune run `/calsync` (or *Tune to another board* in the web
console's Calibration panel), then on the reference run `/calbeacon` (*Send calibration signal*).
The boards briefly switch to LoRa, whose receiver measures the frequency error of each packet;
the tuned board corrects itself in a few rounds to within about 100 Hz. Save with `/save`.

The Heltec V3 must be the 433/470–510 MHz variant; the 868/915 MHz board is matched for that band.
