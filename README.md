# lora-pocsag
70cm LoRA as POCSAG pager

<img src="media/P_20261010_225427.jpg" width="400" alt="LilyGO T3 V1.6.1 (SX1278) and Heltec WiFi LoRa 32 V3 (SX1262) running the POCSAG firmware, status on their OLEDs">

**[Open the web console](https://sq5nry.github.io/lora-pocsag/webui/pocsag_console.html)** to connect to a board over USB or Bluetooth (Chrome or Edge; on Android use Bluetooth).

On iPhone and iPad, Safari and every other browser lack Web Bluetooth and Web Serial. Use the free
[Bluefy – Web BLE Browser](https://apps.apple.com/app/id1492822055) from the App Store instead:
open the console link in Bluefy, tap *Connect Bluetooth*, and enter the PIN from the board's
display when iOS asks to pair. USB doesn't work on iOS. On a Mac, Chrome or Edge work as on any
desktop. Or let the board serve the console itself over WiFi (see *Web console on the board*),
which works in any browser.

![Web console: pager-style message list with send, frequency and receiver controls](media/ui.png)

## Firmware

One Arduino sketch, `pocsag_fw/`, for both boards (and a home-made E22 board, see below). The
board profile follows the board chosen in the Arduino IDE: *ESP32 PICO-D4* → LilyGO T3 V1.6.1
(SX1278), *Heltec WiFi LoRa 32(V3)* → Heltec V3 (SX1262), *ESP32 Dev Module* → ESP32 DevKit + E22
(SX1268). To force one, uncomment its line at the top of `pocsag_fw/board.h`. The board name is
shown on the display and serial at start-up.

1. Arduino IDE with the **esp32 by Espressif** board package.
2. Libraries from the Library Manager: **RadioLib**, **U8g2**, **NimBLE-Arduino** (2.x).
3. Board and partition scheme (the firmware with WiFi, web console and Bluetooth is about 1.4 MB,
   more than the default 1.25 MB app partition of 4 MB boards):

   | Board | *Tools → Board* | *Tools → Partition Scheme* |
   |---|---|---|
   | LilyGO T3 V1.6.1 | *ESP32 PICO-D4* | *Minimal SPIFFS (Large APPS with OTA)* |
   | Heltec WiFi LoRa 32 V3 | *Heltec WiFi LoRa 32(V3)* | default (8 MB flash, enough space) |
   | ESP32 DevKit + E22 | *ESP32 Dev Module* | *Minimal SPIFFS (1.9MB APP with OTA/190KB SPIFFS)* |

   *TTGO LoRa32-OLED* also selects the T3 profile, but that board entry has no Partition Scheme
   menu, so the firmware no longer fits ("Sketch too big"). The T3 V1.6.1 is an ESP32-PICO-D4,
   so *ESP32 PICO-D4* is the right entry anyway. Stored settings survive the switch (the settings
   area is at the same place in both layouts).
   If uploads fail, lower the upload speed to 460800 or 115200.
4. Open `pocsag_fw/pocsag_fw.ino` and upload.

After the first start, set your callsign and RIC (web console, or serial at 115200 baud):

```
/call YOURCALL
/ric 1234567
/save
```

Type `/help` for all commands: frequency correction and carrier for calibration, receive filter
width, transmit power and time limit, monitor mode for all RICs, stored pages, Bluetooth, DAPNET.

### ESP32 DevKit + Ebyte E22-400M30S (1 W)

A third, home-made board: an ESP32-WROOM DevKit with an E22-400M30S module (SX1268 with a
1 W PA and an LNA), wired as the Meshtastic DIY "E-byte Type 1" schematic with one change.

| E22 pin | ESP32 | |
|---|---|---|
| 19 NSS | IO18 | |
| 18 SCK | IO5 | |
| 17 MOSI | IO27 | |
| 16 MISO | IO19 | |
| 15 NRST | IO23 | |
| 14 BUSY | IO32 | |
| 13 DIO1 | IO33 | interrupt |
| 6 RXEN | IO14 | LNA path, switched by the firmware |
| 7 TXEN | – | **bridged to pin 8 (DIO2)**, not to an ESP32 pin |
| 9, 10 VCC | +5 V | full power needs 5 V and about 0.6 A |
| GND | GND | |

With TXEN on DIO2 the SX1268 turns its PA on by itself only while it transmits, so a firmware
hang can never leave the PA keyed. The OLED (SSD1306, I²C) goes on SDA IO21 / SCL IO22.

Build it with the board set to *ESP32 Dev Module*; that selects the E22 profile. The power setting (−9…22 dBm) is the
SX1268 output; the PA adds about 8 dB, so 22 gives about 1 W. Never transmit without an antenna.

If the display shows `ERR radio init: -2` (chip not found), the firmware reads the chip's
version register by hand and shows the result:

- `BUSY ... STUCK` – BUSY or NRST wiring, or the module has no supply
- `00 00 00 00` – module unpowered or MISO held low
- `FF FF FF FF` – MISO open, or NSS not reaching the module
- `SX1268 ...` – the chip answers; check the wiring of the other SPI lines and BUSY

If the chip is found but init still fails, set `TCXO_VOLTAGE` to `0` in `board.h`.

### DAPNET

The board can work as a DAPNET transmitter: it logs into the DAPNET core over WiFi and sends the
pages DAPNET routes to it, in the time slots DAPNET assigns (16 slots of 6.4 s; the clock comes
from NTP). Local pages from the console still go out at once.

1. Register at [hampager.de](https://hampager.de) (amateur licence needed): a user account with
   your RIC, and a **transmitter** (support ticket "New DAPNET Transmitter"; a home node is type
   *personal*). You get a transmitter name and an **auth key** – not your user password.
2. In the web console, *Board → DAPNET*: add your WiFi networks (up to 8: home, work, a phone
   hotspot…), the transmitter name and auth key, then tick *Relay pages from DAPNET* and
   *Save settings to board*. Wherever the board is, it joins the strongest known network in range,
   and also tries the others in case they are hidden. Or over serial:

   ```
   /wifi add MyNetwork|wifi-password
   /wifi add Phone hotspot|other-password
   /dapnet name sq5nry
   /dapnet key <auth key>
   /dapnet 1
   /save
   ```

3. `/wifi` lists the known networks, `/wifi del <name>` removes one, `/wifi clear` all of them.
   `/dapnet` shows the link state; the console shows it live: online, assigned slots, pages
   received / sent / waiting. Relayed pages show up in the message list as *SENT via DAPNET*.

Pages longer than the transmit time limit (`/txmax`) are dropped, and DAPNET time broadcasts
older than 15 s are skipped, as DAPNETGateway does. If the T3 or E22 build is too big for flash
with WiFi, the web console and Bluetooth together, see the board table above (*Minimal SPIFFS*).

A transmitter that others use is still your station: it has to run under your licence and the
rules for your country. In Poland a small home hotspot is usually run under the personal licence;
a wide-area transmitter left unattended (e.g. the 1 W E22 on a mast) normally needs its own UKE
permit for an unattended station. Check before going on air.

### Web console on the board

The board also serves the console itself: open `http://<board address>/` in any browser (Safari on
an iPhone included), log in, and the page talks to the board over WiFi. It is on by default:

- **Login:** `admin` and an 8-digit password of its own for each board, shown on the display at
  start (like the Bluetooth PIN) and by `/web`. Change them in *Board → Web console*, or with
  `/web user <name>` and `/web pass <8 to 63 characters>` (`/web pass -` goes back to the board's
  own), then `/save`. `/web 0` turns the web console off.
- **At home:** the board joins one of its known WiFi networks (*Board → WiFi*, or
  `/wifi add`). `/web` prints the address; the display shows it too.
- **Without a known network:** the board opens its own WiFi, at start if it knows no networks,
  otherwise after 45 s without one. The network has the board's Bluetooth name (`POCSAG-xxxx`),
  its password is the web password, and the console is at `http://192.168.4.1/`. While a phone is
  connected the board stops looking for other networks; back on a known network and with nobody
  connected, it closes its own. If nobody connects within 10 minutes it closes it too, to save
  the battery, and with no known networks it turns WiFi off; the next start, or `/web ap 1`, opens
  it again. `/web ap 0` never opens it.

Notes:

- The page from the board uses plain HTTP, so the password crosses the network unencrypted: fine
  at home or on the board's own WiFi, not on public networks.
- Browsers allow USB and Bluetooth only on HTTPS pages, so the page from the board can't offer
  them; use the GitHub Pages console for those.
- The page is built into the firmware. After changing `webui/pocsag_console.html`, run
  `python3 tools/make_web_page.py` to update `pocsag_fw/web_page.h`, then rebuild.

### Matching the frequency of two boards

Crystal boards (like the T3) can be several kHz off, while POCSAG between two boards stops
working at roughly 3 kHz of difference. To align a board to a reference (ideally the Heltec,
which has a TCXO): on the board to tune run `/calsync` (or *Tune to another board* in the web
console's Calibration panel), then on the reference run `/calbeacon` (*Send calibration signal*).
The boards briefly switch to LoRa, whose receiver measures the frequency error of each packet;
the tuned board corrects itself in a few rounds to within about 100 Hz. Save with `/save`.

The Heltec V3 must be the 433/470–510 MHz variant; the 868/915 MHz board is matched for that band.

## How reception works

From antenna to bits, simulated on a weak signal (Eb/N0 15 dB, ±7.8 kHz receive filter, 900 Hz
frequency error). Part 1 follows the signal through the radio chip to complex baseband (I/Q):
spectrum, I and Q, the spinning phasor and its phase. Part 2 turns the phase into bits:
discriminator, data filter and slicer, then the bit synchroniser that recovers the 1200 Hz clock
during the preamble. All of this runs inside the SX1278 / SX126x; the firmware takes over at the
bits (frame sync, error correction, decoding).

![POCSAG receiver from antenna to bits: block diagram, baseband spectrum, I/Q, phasors, phase, discriminator output, data filter and slicer, bit clock recovery](media/pocsag_receiver_steps.png)
