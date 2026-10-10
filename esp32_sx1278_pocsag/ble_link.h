/*
  BLE serial link (Nordic UART Service) for the POCSAG sketches.
  Library: NimBLE-Arduino 2.x (Library Manager: "NimBLE-Arduino" by h2zero).

  - Same text protocol as USB serial: commands in, log + @-events out.
  - Passkey pairing (6 digits, MITM, bonding). Nothing is sent or accepted until
    the peer is paired and the link is encrypted, so nobody nearby can transmit
    pages under your callsign.
  - Works with Web Bluetooth (Chrome on Android / desktop) and apps such as
    nRF Connect or "Serial Bluetooth Terminal".
*/
#pragma once
#include <Arduino.h>
#include <NimBLEDevice.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

#define NUS_SERVICE_UUID "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define NUS_RX_UUID      "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"   // phone -> board, write
#define NUS_TX_UUID      "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"   // board -> phone, notify

class BleLink {
 public:
  // Unique per board, derived from the MAC: name "POCSAG-ABCD", default PIN 6 digits
  static String defaultName() {
    uint64_t mac = ESP.getEfuseMac();
    char b[16]; snprintf(b, sizeof b, "POCSAG-%04X", (unsigned)((mac >> 32) & 0xFFFF));
    return String(b);
  }
  static uint32_t defaultPin() {
    uint64_t mac = ESP.getEfuseMac();
    return 100000UL + (uint32_t)((mac ^ (mac >> 24)) % 900000UL);
  }

  bool begin(const String& name, uint32_t pin) {
    if (active_) return true;
    if (!rxq_) rxq_ = xQueueCreate(512, 1);
    name_ = name;
    if (!NimBLEDevice::init(name.c_str())) return false;
    NimBLEDevice::setMTU(185);
    NimBLEDevice::setSecurityAuth(true, true, true);          // bonding, MITM, LE secure connections
    NimBLEDevice::setSecurityIOCap(BLE_HS_IO_DISPLAY_ONLY);   // board "displays" the PIN (OLED/serial)
    NimBLEDevice::setSecurityPasskey(pin);

    NimBLEServer* srv = NimBLEDevice::createServer();
    srv->setCallbacks(&srvCb_, false);
    srv->advertiseOnDisconnect(true);
    NimBLEService* svc = srv->createService(NUS_SERVICE_UUID);
    tx_ = svc->createCharacteristic(NUS_TX_UUID, NIMBLE_PROPERTY::NOTIFY);
    NimBLECharacteristic* rx = svc->createCharacteristic(NUS_RX_UUID,
        NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR |
        NIMBLE_PROPERTY::WRITE_ENC | NIMBLE_PROPERTY::WRITE_AUTHEN);
    rx->setCallbacks(&chrCb_);
    svc->start();

    NimBLEAdvertising* adv = NimBLEDevice::getAdvertising();
    adv->enableScanResponse(true);                            // name goes in the scan response
    adv->addServiceUUID(NUS_SERVICE_UUID);
    adv->setName(name.c_str());
    active_ = adv->start();
    return active_;
  }

  void end() {
    if (!active_) return;
    active_ = false; authed_ = false;
    NimBLEServer* srv = NimBLEDevice::getServer();
    if (srv && conn_ != BLE_HS_CONN_HANDLE_NONE) srv->disconnect(conn_);
    NimBLEDevice::stopAdvertising();
    delay(50);
    NimBLEDevice::deinit(true);
    tx_ = nullptr; conn_ = BLE_HS_CONN_HANDLE_NONE; line_ = "";
  }

  void setPin(uint32_t pin) { if (active_) NimBLEDevice::setSecurityPasskey(pin); }
  void forgetBonds()        { if (active_) NimBLEDevice::deleteAllBonds(); }

  bool active()    const { return active_; }
  bool connected() const { return active_ && authed_; }
  const String& name() const { return name_; }

  // one complete received line (without \r\n), call from loop()
  bool readLine(String& out) {
    if (!rxq_) return false;
    uint8_t c;
    while (xQueueReceive(rxq_, &c, 0) == pdTRUE) {
      if (c == '\n' || c == '\r') {
        if (rxLine_.length()) { out = rxLine_; rxLine_ = ""; return true; }
      } else if (rxLine_.length() < 300) rxLine_ += (char)c;
    }
    return false;
  }

  // outgoing text; buffered per line and sent as notifications (call from loop context)
  void write(const uint8_t* d, size_t n) {
    if (!connected()) return;
    for (size_t i = 0; i < n; i++) {
      line_ += (char)d[i];
      if (d[i] == '\n' || line_.length() >= 400) sendLine();
    }
  }

 private:
  void sendLine() {
    if (!tx_ || !connected()) { line_ = ""; return; }
    const size_t chunk = (mtu_ > 23 ? mtu_ - 3 : 20);
    const uint8_t* p = (const uint8_t*)line_.c_str();
    size_t left = line_.length();
    while (left && connected()) {
      size_t n = left < chunk ? left : chunk;
      int tries = 0;
      while (!tx_->notify(p, n, conn_)) {                    // out of buffers: wait a little
        if (++tries > 30) { line_ = ""; return; }
        delay(5);
      }
      p += n; left -= n;
    }
    line_ = "";
  }

  struct SrvCb : public NimBLEServerCallbacks {
    BleLink* l;
    explicit SrvCb(BleLink* o) : l(o) {}
    void onConnect(NimBLEServer*, NimBLEConnInfo& ci) override {
      l->conn_ = ci.getConnHandle(); l->authed_ = false; l->mtu_ = 23;
      NimBLEDevice::startSecurity(ci.getConnHandle());       // ask the phone to pair right away
    }
    void onDisconnect(NimBLEServer*, NimBLEConnInfo&, int) override {
      l->authed_ = false; l->conn_ = BLE_HS_CONN_HANDLE_NONE;
    }
    void onMTUChange(uint16_t mtu, NimBLEConnInfo&) override { l->mtu_ = mtu; }
    void onAuthenticationComplete(NimBLEConnInfo& ci) override {
      l->authed_ = ci.isEncrypted() && ci.isAuthenticated();
      if (!l->authed_) NimBLEDevice::getServer()->disconnect(ci.getConnHandle());
    }
  };
  struct ChrCb : public NimBLECharacteristicCallbacks {
    BleLink* l;
    explicit ChrCb(BleLink* o) : l(o) {}
    void onWrite(NimBLECharacteristic* c, NimBLEConnInfo& ci) override {
      if (!(ci.isEncrypted() && ci.isAuthenticated())) return;   // belt and braces
      NimBLEAttValue v = c->getValue();
      for (size_t i = 0; i < v.size(); i++) xQueueSend(l->rxq_, &v.data()[i], 0);
    }
  };

  SrvCb srvCb_{this};
  ChrCb chrCb_{this};
  NimBLECharacteristic* tx_ = nullptr;
  QueueHandle_t rxq_ = nullptr;
  volatile bool active_ = false, authed_ = false;
  volatile uint16_t conn_ = BLE_HS_CONN_HANDLE_NONE;
  volatile uint16_t mtu_ = 23;
  String name_, line_, rxLine_;
};

// Print sink that goes to USB serial and, when a paired phone is connected, to BLE
class DualOut : public Print {
 public:
  explicit DualOut(BleLink& b) : ble_(b) {}
  size_t write(uint8_t c) override { Serial.write(c); ble_.write(&c, 1); return 1; }
  size_t write(const uint8_t* b, size_t n) override { Serial.write(b, n); ble_.write(b, n); return n; }
 private:
  BleLink& ble_;
};
