#include "Globals.h"
#if ENABLE_BLE
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
struct BleQueuedCommand { char text[128]; };
static QueueHandle_t bleCommandQueue = nullptr;
#endif

// ============================================================
// 通信对象和接收缓冲
// ============================================================

#if ENABLE_BLE
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

    // BLE callbacks run on another task. Only the main loop may access motors/SPI.
    BleQueuedCommand queued = {};
    if (cmd.length() >= sizeof(queued.text)) {
      sendReply(1, "ERR, BLE_COMMAND_TOO_LONG");
      return;
    }
    cmd.toCharArray(queued.text, sizeof(queued.text));
    if (!bleCommandQueue || xQueueSend(bleCommandQueue, &queued, 0) != pdTRUE) {
      sendReply(1, "ERR, BLE_COMMAND_QUEUE_FULL");
    }
  }
};

void handleBLECommand() {
  BleQueuedCommand queued;
  if (bleCommandQueue && xQueueReceive(bleCommandQueue, &queued, 0) == pdTRUE) {
    handleCommand(String(queued.text), 1);
  }
}

void setupBLE() {
  bleCommandQueue = xQueueCreate(8, sizeof(BleQueuedCommand));
  if (!bleCommandQueue) {
    Serial.println("ERR, BLE_COMMAND_QUEUE_ALLOCATION");
    return;
  }
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

void handleBLECommand() {}

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

void handleWiFiCommand() {
#if WIFI_MODE != WIFI_MODE_OFF

  if (!wifiClient || !wifiClient.connected()) {
    WiFiClient newClient = wifiServer.available();

    if (newClient) {
      wifiClient = newClient;
      wifiRxBuffer = "";
      broadcastMsg("WiFi client connected");
      wifiClient.println("Connected to Upper1.1-FOC");
    }
  }

  if (wifiClient && wifiClient.connected()) {
    while (wifiClient.available()) {
      char c = wifiClient.read();

      if (c == '\n' || c == '\r') {
        if (wifiRxBuffer.length() > 0) {
          String cmd = wifiRxBuffer;
          wifiRxBuffer = "";
          cmd.trim();
          handleCommand(cmd, 2);
        }
      } else {
        wifiRxBuffer += c;
      }
    }
  }

#endif
}


void handleSerialCommand() {
  while (Serial.available()) {
    char c = Serial.read();

    if (c == '\n' || c == '\r') {
      if (serialRxBuffer.length() > 0) {
        String cmd = serialRxBuffer;
        serialRxBuffer = "";
        cmd.trim();
        handleCommand(cmd, 0);
      }
    } else {
      serialRxBuffer += c;
    }
  }
}
