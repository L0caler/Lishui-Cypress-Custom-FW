#include "ble_log.h"

#include <Arduino.h>
#include <NimBLEDevice.h>

namespace blelog {

// Nordic UART Service: the phone writes to RX and subscribes to TX.
constexpr const char *NUS_SERVICE = "6E400001-B5A3-F393-E0A9-E50E24DCCA9E";
constexpr const char *NUS_RX = "6E400002-B5A3-F393-E0A9-E50E24DCCA9E";
constexpr const char *NUS_TX = "6E400003-B5A3-F393-E0A9-E50E24DCCA9E";
// Name and password from platformio.ini.  Neutral name, nothing on the
// link until the password: someone connecting by chance sees nothing.
#ifndef BLE_NAME
#define BLE_NAME "ESP32"
#endif
#ifndef BLE_PASSWORD
#error "set BLE_PASSWORD in platformio.ini"
#endif
static_assert(sizeof(BLE_PASSWORD) > 1, "BLE_PASSWORD must not be empty");
constexpr const char *DEVICE_NAME = BLE_NAME;
constexpr uint8_t MAX_WRONG = 3;  // wrong passwords, then the link is dropped
// Power: the radio's average draw is set by how often it transmits.
constexpr uint16_t ADV_MIN = 800, ADV_MAX = 1600;  // advertise every 0.5..1 s (x 0.625 ms)
constexpr uint16_t CONN_MIN = 40, CONN_MAX = 80;   // connection interval 50..100 ms (x 1.25 ms)
constexpr uint16_t CONN_TIMEOUT = 400;             // 4 s (x 10 ms)
constexpr int8_t TX_POWER_DBM = -12;  // the ESP32's lowest: next to the bike only

// Text waiting to be notified.  Full: new text is dropped and counted, so a
// slow phone never stalls the control of the link.
constexpr size_t BUF_SIZE = 2048;
static uint8_t buf[BUF_SIZE];
static size_t head = 0, tail = 0;  // head: write, tail: read
static uint32_t dropped = 0;

static NimBLECharacteristic *txChar = nullptr;
static volatile bool subscribed = false;
static volatile uint16_t mtu = 23;
static bool started = false;
static volatile bool connected = false;
static volatile uint16_t connHandle = 0;
static bool unlocked = false;  // main task only
static uint8_t wrong = 0;

static size_t used() { return (head + BUF_SIZE - tail) % BUF_SIZE; }

// Received text, from the BLE task, until the main loop takes whole lines.
// Fixed buffer: no heap use inside the critical sections.
constexpr size_t RX_MAX = 256;  // more without a line end: thrown away
static char rxBuf[RX_MAX];
static size_t rxLen = 0;
static portMUX_TYPE rxMux = portMUX_INITIALIZER_UNLOCKED;

class Tee : public Print {
 public:
  size_t write(uint8_t c) override { return write(&c, 1); }
  size_t write(const uint8_t *p, size_t n) override {
    Serial.write(p, n);
    if (!started || !subscribed || !unlocked) return n;
    for (size_t i = 0; i < n; i++) {
      size_t next = (head + 1) % BUF_SIZE;
      if (next == tail) {
        dropped += n - i;
        break;
      }
      buf[head] = p[i];
      head = next;
    }
    return n;
  }
};
static Tee tee;

Print &log() { return tee; }

class ServerCallbacks : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer *server, NimBLEConnInfo &info) override {
    // Power: a packet every 50..100 ms instead of the phone's usual
    // 7.5..30 ms.  Plenty for a text log at 4 lines a second.
    server->updateConnParams(info.getConnHandle(), CONN_MIN, CONN_MAX, 0, CONN_TIMEOUT);
    mtu = info.getMTU();
    connHandle = info.getConnHandle();
    connected = true;
    Serial.println("BLE: phone connected, waiting for the password");
  }
  void onDisconnect(NimBLEServer *, NimBLEConnInfo &, int reason) override {
    subscribed = false;  // loop() empties the buffer and locks, on the main task
    connected = false;
    if (!started) return;  // end(): shutting down
    Serial.printf("BLE: phone disconnected (reason %d), advertising again\n", reason);
    NimBLEDevice::startAdvertising();
  }
  void onMTUChange(uint16_t m, NimBLEConnInfo &) override { mtu = m; }
};

class TxCallbacks : public NimBLECharacteristicCallbacks {
  void onSubscribe(NimBLECharacteristic *, NimBLEConnInfo &, uint16_t subValue) override {
    subscribed = subValue != 0;
  }
};

class RxCallbacks : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic *c, NimBLEConnInfo &) override {
    std::string v = c->getValue();
    portENTER_CRITICAL(&rxMux);
    if (rxLen + v.size() > RX_MAX) rxLen = 0;
    if (v.size() <= RX_MAX) {
      memcpy(rxBuf + rxLen, v.data(), v.size());
      rxLen += v.size();
    }
    portEXIT_CRITICAL(&rxMux);
  }
};

void begin() {
  NimBLEDevice::init(DEVICE_NAME);
  NimBLEDevice::setPower(TX_POWER_DBM);
  NimBLEDevice::setMTU(185);  // the phone may ask for less

  NimBLEServer *server = NimBLEDevice::createServer();
  server->setCallbacks(new ServerCallbacks());
  NimBLEService *svc = server->createService(NUS_SERVICE);
  txChar = svc->createCharacteristic(NUS_TX, NIMBLE_PROPERTY::NOTIFY);
  txChar->setCallbacks(new TxCallbacks());
  NimBLECharacteristic *rx =
      svc->createCharacteristic(NUS_RX, NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR);
  rx->setCallbacks(new RxCallbacks());
  server->start();

  NimBLEAdvertising *adv = NimBLEDevice::getAdvertising();
  adv->setName(DEVICE_NAME);
  adv->addServiceUUID(NUS_SERVICE);
  adv->enableScanResponse(true);
  adv->setMinInterval(ADV_MIN);
  adv->setMaxInterval(ADV_MAX);
  adv->setPreferredParams(CONN_MIN, CONN_MAX);
  adv->start();
  started = true;
}

void end() {
  if (!started) return;
  started = false;
  subscribed = false;
  NimBLEDevice::deinit(true);  // stop the stack and the radio, free the memory
}

// The next line, whatever it is.
static bool takeLine(std::string &line) {
  char out[RX_MAX];
  size_t n = 0;
  bool got = false;
  portENTER_CRITICAL(&rxMux);
  size_t end = 0;
  while (end < rxLen && rxBuf[end] != '\r' && rxBuf[end] != '\n') end++;
  if (end < rxLen) {
    got = true;
    n = end;
    memcpy(out, rxBuf, n);
    while (end < rxLen && (rxBuf[end] == '\r' || rxBuf[end] == '\n')) end++;
    memmove(rxBuf, rxBuf + end, rxLen - end);
    rxLen -= end;
  }
  portEXIT_CRITICAL(&rxMux);
  if (got) line.assign(out, n);
  return got;
}

static std::string trim(const std::string &s) {
  size_t a = s.find_first_not_of(" \t"), b = s.find_last_not_of(" \t");
  return a == std::string::npos ? std::string() : s.substr(a, b - a + 1);
}

bool readLine(std::string &line) {
  if (!started) return false;
  while (takeLine(line)) {
    if (unlocked) return true;
    // Locked: the only thing that counts is the password.  No answer to
    // anything else, so the link gives nothing away.
    if (trim(line) == BLE_PASSWORD) {
      unlocked = true;
      wrong = 0;
      Serial.println("BLE: unlocked");
      tee.println("ok");
    } else if (++wrong >= MAX_WRONG) {
      Serial.println("BLE: wrong password, disconnecting");
      wrong = 0;
      NimBLEDevice::getServer()->disconnect(connHandle);
    }
  }
  return false;
}

void loop() {
  if (!started) return;
  if (!connected && unlocked) {
    unlocked = false;  // every new connection needs the password again
    wrong = 0;
  }
  if (!subscribed || !unlocked) {
    head = tail = 0;
    return;
  }
  if (dropped) {
    uint32_t d = dropped;
    dropped = 0;
    tee.printf("[BLE: %lu bytes dropped]\n", (unsigned long)d);
  }
  // A few chunks per call, each as big as the connection allows.
  size_t chunk = mtu > 3 ? mtu - 3 : 20;
  for (int i = 0; i < 4 && used(); i++) {
    uint8_t out[256];
    size_t n = used();
    if (n > chunk) n = chunk;
    if (n > sizeof(out)) n = sizeof(out);
    for (size_t k = 0; k < n; k++) out[k] = buf[(tail + k) % BUF_SIZE];
    txChar->setValue(out, n);
    if (!txChar->notify()) break;  // stack busy: try again next loop
    tail = (tail + n) % BUF_SIZE;
  }
}

}  // namespace blelog
