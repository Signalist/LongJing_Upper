#include "Globals.h"
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

// ============================================================
// 通信对象和接收缓冲
// ============================================================

#if ENABLE_BLE
static QueueHandle_t bleCommands = nullptr;
BLEServer* bleServer = NULL;
BLECharacteristic* txCharacteristic = NULL;
BLECharacteristic* rxCharacteristic = NULL;
bool bleDeviceConnected = false;
#endif

#if WIFI_MODE != WIFI_MODE_OFF
WiFiServer wifiServer(WIFI_TCP_PORT);
WiFiClient wifiClient;
String wifiRxBuffer = "";
#endif

String serialRxBuffer = "";

// ============================================================
// BLE 函数
// ============================================================

#if ENABLE_BLE

void bleSendText(const String& msg) {
  // 直接发送，不查询 Notify 订阅状态
  if (txCharacteristic != NULL) {
    txCharacteristic->setValue(msg.c_str());
    txCharacteristic->notify();
  }
}

class ServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* server) {
    bleDeviceConnected = true;
  }

  void onDisconnect(BLEServer* server) {
    bleDeviceConnected = false;
    server->getAdvertising()->start();
  }
};

class RxCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* characteristic) {
    String cmd = characteristic->getValue();
    cmd.trim();

    if (cmd.length() == 0) {
      return;
    }

    if (cmd.length() > COMMAND_MAX_LENGTH || !bleCommands) {
      bleSendText("ERR, COMMAND_TOO_LONG_OR_UNAVAILABLE"); return;
    }
    char pending[COMMAND_MAX_LENGTH + 1] = {};
    cmd.toCharArray(pending, sizeof(pending));
    if (xQueueSend(bleCommands, pending, 0) != pdTRUE) bleSendText("ERR, COMMAND_QUEUE_FULL");
  }
};

void setupBLE() {
  bleCommands = xQueueCreate(8, COMMAND_MAX_LENGTH + 1);
  if (!bleCommands) { Serial.println("BLE queue allocation failed"); return; }
  BLEDevice::init(DEVICE_NAME);

  bleServer = BLEDevice::createServer();
  bleServer->setCallbacks(new ServerCallbacks());

  BLEService* service = bleServer->createService(SERVICE_UUID);

  txCharacteristic = service->createCharacteristic(
    TX_CHAR_UUID,
    BLECharacteristic::PROPERTY_NOTIFY
  );
  txCharacteristic->addDescriptor(new BLE2902());

  rxCharacteristic = service->createCharacteristic(
    RX_CHAR_UUID,
    BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR
  );
  rxCharacteristic->setCallbacks(new RxCallbacks());

  service->start();

  BLEAdvertising* advertising = BLEDevice::getAdvertising();
  advertising->addServiceUUID(SERVICE_UUID);
  advertising->setScanResponse(true);
  advertising->start();

  Serial.println("BLE started.");
}

#else

void setupBLE() {
  Serial.println("BLE disabled.");
}

#endif

// ============================================================
// WiFi 函数
// ============================================================

void setupWiFi() {
#if WIFI_MODE == WIFI_MODE_OFF

  Serial.println("WiFi disabled.");

#elif WIFI_MODE == WIFI_MODE_ROUTER

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_ROUTER_SSID, WIFI_ROUTER_PASS);

  Serial.print("Connecting to router WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(300);
    Serial.print(".");
  }
  Serial.println();

  wifiServer.begin();

  Serial.print("WiFi router connected, IP=");
  Serial.println(WiFi.localIP());

#elif WIFI_MODE == WIFI_MODE_PHONE

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_PHONE_SSID, WIFI_PHONE_PASS);

  Serial.print("Connecting to phone hotspot");
  while (WiFi.status() != WL_CONNECTED) {
    delay(300);
    Serial.print(".");
  }
  Serial.println();

  wifiServer.begin();

  Serial.print("WiFi phone hotspot connected, IP=");
  Serial.println(WiFi.localIP());

#elif WIFI_MODE == WIFI_MODE_AP

  WiFi.mode(WIFI_AP);
  WiFi.softAP(WIFI_AP_SSID, WIFI_AP_PASS);

  wifiServer.begin();

  Serial.print("WiFi AP started, IP=");
  Serial.println(WiFi.softAPIP());

#endif
}

// Reject an oversized line in its entirety; do not execute its trailing suffix.
static bool feedCommandLine(char c, String& buffer, bool& discard, uint8_t source) {
  if (c == '\n' || c == '\r') {
    if (discard) { discard = false; buffer = ""; return false; }
    if (buffer.length()) {
      String command = buffer;
      buffer = "";
      handleCommand(command, source);
      return true;
    }
  } else if (!discard) {
    if (buffer.length() >= COMMAND_MAX_LENGTH || c == 0) {
      buffer = ""; discard = true;
      sendReply(source, "ERR, COMMAND_TOO_LONG_OR_INVALID");
    } else buffer += c;
  }
  return false;
}

void handleWiFiCommand() {
#if WIFI_MODE != WIFI_MODE_OFF
  static bool discard = false;
  if (!wifiClient || !wifiClient.connected()) {
    WiFiClient newClient = wifiServer.available();
    if (newClient) {
      wifiClient = newClient;
      wifiRxBuffer = "";
      discard = false;
      broadcastMsg("WiFi client connected");
      wifiClient.println("Connected to Upper3.0-FOC");
    }
  }
  if (wifiClient && wifiClient.connected()) {
    for (uint8_t n = 0; n < 64 && wifiClient.available(); ++n) {
      if (feedCommandLine(wifiClient.read(), wifiRxBuffer, discard, 2)) break;
    }
  }
#endif
}

void handleSerialCommand() {
  static bool discard = false;
  for (uint8_t n = 0; n < 64 && Serial.available(); ++n) {
    if (feedCommandLine(Serial.read(), serialRxBuffer, discard, 0)) break;
  }
}


void handleBLECommand() {
#if ENABLE_BLE
  char pending[COMMAND_MAX_LENGTH + 1];
  if (bleCommands && xQueueReceive(bleCommands, pending, 0) == pdTRUE) handleCommand(String(pending), 1);
#endif
}
