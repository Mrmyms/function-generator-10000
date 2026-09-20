#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// --- PINOUT & SERIAL DEFINITIONS ---
#define UART_STM_RX 16   // ESP32 RX2 connects to STM32 TX (e.g. PA9)
#define UART_STM_TX 17   // ESP32 TX2 connects to STM32 RX (e.g. PA10)
#define UART_BAUD   115200

// --- BLE UUIDs ---
#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHAR_COMMAND_UUID   "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define CHAR_TELEMETRY_UUID "1c95d5e3-d8f7-413a-bf3d-7a2e5d7be87e"

static BLEServer* pServer = nullptr;
static BLECharacteristic* pCmdChar = nullptr;
static BLECharacteristic* pTelemChar = nullptr;
static bool deviceConnected = false;
static bool oldDeviceConnected = false;

// --- PACKETS & PROTOCOL (PACKED) ---
struct __attribute__((packed)) ChannelPacket {
  char     waveform;
  float    frequency;
  uint8_t  amplitude;
  int8_t   offset;
  uint8_t  enabled;
  uint8_t  dutyCycle;
  float    sweepTarget;
  float    sweepTimeMs;
  uint8_t  modType;
  float    modFreq;
  uint8_t  modDepth;
  uint8_t  burstEnabled;
  uint32_t burstCycles;
  uint8_t  trigger;
};

// Compact CRC8 calculation
uint8_t calcCRC8(const uint8_t *data, size_t len) {
  uint8_t crc = 0;
  while (len--) {
    crc ^= *data++;
    for (uint8_t j = 0; j < 8; j++) crc = (crc & 0x80) ? (crc << 1) ^ 0x07 : (crc << 1);
  }
  return crc;
}

// Forward command packet to STM32 over UART2
void sendPacketToSTM(uint8_t cmd, const uint8_t* payload, size_t len) {
  uint8_t header[4];
  header[0] = 0xAA; // Sync byte 1
  header[1] = 0x55; // Sync byte 2
  header[2] = cmd;  // Command ID
  header[3] = (uint8_t)len;

  uint8_t crc = calcCRC8(payload, len);
  crc ^= calcCRC8(header, 4);

  Serial2.write(header, 4);
  if (len > 0 && payload != nullptr) {
    Serial2.write(payload, len);
  }
  Serial2.write(crc);

  Serial.printf("[ESP32 -> STM] Sent CMD 0x%02X (%d bytes, CRC 0x%02X)\n", cmd, (int)len, crc);
}

// Ping-Pong test with STM32
static uint32_t pingSeq = 0;
static uint32_t lastPingTime = 0;
static uint32_t pingSentMicros = 0;

void sendPingToSTM() {
  uint32_t payload[2];
  payload[0] = ++pingSeq;
  payload[1] = millis();
  pingSentMicros = micros();
  sendPacketToSTM(0x00, (const uint8_t*)payload, sizeof(payload));
}

// BLE Callbacks
class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
      Serial.println(">>> Web Bluetooth Client Connected! <<<");
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
      Serial.println(">>> Web Bluetooth Client Disconnected. Advertising restarted. <<<");
    }
};

class CommandCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      std::string rxValue = pCharacteristic->getValue();
      if (rxValue.length() == 0) return;

      uint8_t cmd = (uint8_t)rxValue[0];
      const uint8_t* data = (const uint8_t*)rxValue.data() + 1;
      size_t len = rxValue.length() - 1;

      Serial.printf("[BLE RX] Command 0x%02X (Length: %d bytes)\n", cmd, (int)len);
      // Relay command to STM32 via high-speed UART
      sendPacketToSTM(cmd, data, len);
    }
};

void setup() {
  Serial.begin(115200);
  Serial2.begin(UART_BAUD, SERIAL_8N1, UART_STM_RX, UART_STM_TX);
  delay(500);

  Serial.println("\n==============================================");
  Serial.println("  Function Generator 10000 - ESP32 BLE Gateway");
  Serial.println("  Wireless Link: Web Bluetooth (Vercel) <-> STM32");
  Serial.println("==============================================");

  // Initialize BLE Device
  BLEDevice::init("FuncGen-10000");
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create BLE Service
  BLEService *pService = pServer->createService(SERVICE_UUID);

  // Create Command Characteristic (Web App writes commands here)
  pCmdChar = pService->createCharacteristic(
                      CHAR_COMMAND_UUID,
                      BLECharacteristic::PROPERTY_WRITE |
                      BLECharacteristic::PROPERTY_WRITE_NR
                    );
  pCmdChar->setCallbacks(new CommandCallbacks());

  // Create Telemetry Characteristic (ESP32 notifies status to Web App)
  pTelemChar = pService->createCharacteristic(
                      CHAR_TELEMETRY_UUID,
                      BLECharacteristic::PROPERTY_READ |
                      BLECharacteristic::PROPERTY_NOTIFY
                    );
  pTelemChar->addDescriptor(new BLE2902());

  pService->start();

  // Start Advertising
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06); // functions that help with iPhone connections
  pAdvertising->setMinPreferred(0x12);
  BLEDevice::startAdvertising();

  Serial.println("BLE Advertising active as 'FuncGen-10000' (Waiting for Web App connection...)");
}

void loop() {
  // 1. Process UART responses from STM32
  static uint8_t rxState = 0;
  static uint8_t rxCmd = 0;
  static uint8_t rxLen = 0;
  static uint8_t rxBuf[64];
  static uint8_t rxIdx = 0;

  while (Serial2.available()) {
    uint8_t b = Serial2.read();
    if (rxState == 0) {
      if (b == 0xAA) rxState = 1;
    } else if (rxState == 1) {
      if (b == 0x55) rxState = 2;
      else rxState = 0;
    } else if (rxState == 2) {
      rxCmd = b;
      rxState = 3;
    } else if (rxState == 3) {
      rxLen = b;
      rxIdx = 0;
      rxState = (rxLen == 0) ? 5 : 4;
    } else if (rxState == 4) {
      rxBuf[rxIdx++] = b;
      if (rxIdx >= rxLen || rxIdx >= sizeof(rxBuf)) rxState = 5;
    } else if (rxState == 5) {
      uint8_t expectedCrc = b;
      rxState = 0;

      if (rxCmd == 0x80) { // PONG Response from STM32
        uint32_t rttUs = micros() - pingSentMicros;
        Serial.printf("⚡ [STM32 -> ESP32] PONG! Link Active! Latency: %.2f ms (RTT: %u µs)\n", rttUs / 1000.0f, rttUs);

        // Notify Web App if connected
        if (deviceConnected && pTelemChar != nullptr) {
          uint8_t telem[6];
          telem[0] = 0x80;
          telem[1] = (uint8_t)(rttUs & 0xFF);
          telem[2] = (uint8_t)((rttUs >> 8) & 0xFF);
          pTelemChar->setValue(telem, 3);
          pTelemChar->notify();
        }
      } else if (rxCmd == 0x81) { // ACK from STM32
        Serial.println("✔ [STM32 -> ESP32] ACK: Command executed by STM32");
      }
    }
  }

  // 2. Periodic Ping test to STM32 every 2 seconds
  if (millis() - lastPingTime >= 2000) {
    lastPingTime = millis();
    sendPingToSTM();
  }

  // 3. Handle BLE disconnect & reconnect advertising
  if (!deviceConnected && oldDeviceConnected) {
    delay(500); // give the bluetooth stack the chance to get things ready
    pServer->startAdvertising(); // restart advertising
    Serial.println("Advertising restarted.");
    oldDeviceConnected = deviceConnected;
  }
  if (deviceConnected && !oldDeviceConnected) {
    oldDeviceConnected = deviceConnected;
  }

  delay(10);
}
