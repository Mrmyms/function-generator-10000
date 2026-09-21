/*
 * ==================================================================================================
 * FUNCTION GENERATOR 10000 - ESP32 BLE WIRELESS GATEWAY FIRMWARE
 * ==================================================================================================
 * Hardware:  ESP32-WROOM-32 (NodeMCU-32S / ESP32 DevKit v1)
 * Purpose:   High-Performance Wireless Bridge (Web Bluetooth <---> STM32 Nucleo UART)
 *
 * Wiring to STM32 NUCLEO-H503RB:
 *   - ESP32 TX2 (GPIO 17) ----> STM32 Pin D0 (PB15_ALT1 RX)
 *   - ESP32 RX2 (GPIO 16) <---- STM32 Pin D1 (PB14 TX)
 *   - Common Ground: GND  <---> GND
 *
 * Core Subsystems:
 *   1. Web Bluetooth (BLE GATT Server)
 *      - Device Name: "FuncGen-10000"
 *      - Service: 4fafc201-1fb5-459e-8fcc-c5c9c331914b
 *      - Command Characteristic (Write): beb5483e-36e1-4688-b7f5-ea07361b26a8
 *      - Telemetry Characteristic (Notify): 1c95d5e3-d8f7-413a-bf3d-7a2e5d7be87e
 *   2. High-Speed Inter-Board UART Link
 *      - Serial2 @ 115200 bps 8N1
 *      - Framed Binary Protocol with CRC8 error verification (0xAA 0x55 <cmd> <len> <payload> <crc>)
 *   3. Autonomous Link Monitoring
 *      - 1 Hz hardware ping-pong measuring microsecond round-trip latency (RTT)
 *      - Automated STM32 offline/online state detection
 *   4. Visual Status LED Feedback (GPIO 2 / LED_BUILTIN)
 *   5. USB Serial Diagnostic Console (115200 bps)
 * ==================================================================================================
 */

#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// --- PINOUT & HARDWARE CONFIGURATION ---
#define UART_STM_RX     16    // ESP32 RX2 connects to STM32 TX (Pin D1 / PB14)
#define UART_STM_TX     17    // ESP32 TX2 connects to STM32 RX (Pin D0 / PB15)
#define UART_BAUD       115200

#ifndef LED_BUILTIN
#define LED_BUILTIN     2
#endif
#define LED_STATUS_PIN  LED_BUILTIN

// --- BLE GATT UUIDs ---
#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHAR_COMMAND_UUID   "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define CHAR_TELEMETRY_UUID "1c95d5e3-d8f7-413a-bf3d-7a2e5d7be87e"

// BLE Server & Characteristics
static BLEServer*         pServer     = nullptr;
static BLECharacteristic* pCmdChar    = nullptr;
static BLECharacteristic* pTelemChar  = nullptr;
static bool deviceConnected           = false;
static bool oldDeviceConnected        = false;

// --- PACKET COUNTERS & TELEMETRY ---
static uint32_t totalBleCmdsReceived  = 0;
static uint32_t totalPacketsSentToStm = 0;
static uint32_t totalAcksReceived     = 0;
static uint32_t totalNacksReceived    = 0;
static uint32_t totalCrcErrors        = 0;

// STM32 Link State
static uint32_t pingSeq               = 0;
static uint32_t lastPingTime          = 0;
static uint32_t pingSentMicros        = 0;
static bool     stm32Connected        = false;
static uint32_t lastStm32PongTime     = 0;
static uint32_t lastRttUs             = 0;
static uint32_t minRttUs              = 999999;
static uint32_t maxRttUs              = 0;
static uint8_t  stm32Flags            = 0;
static uint8_t  stm32FwVersion        = 0;
static uint8_t  stm32HwId             = 0;

// LED Activity Timer
static volatile unsigned long ledActivityOffUntil = 0;

// --- CRC8 CALCULATION (POLYNOMIAL 0x07) ---
uint8_t calcCRC8(const uint8_t *data, size_t len) {
  uint8_t crc = 0;
  while (len--) {
    crc ^= *data++;
    for (uint8_t j = 0; j < 8; j++) {
      crc = (crc & 0x80) ? (crc << 1) ^ 0x07 : (crc << 1);
    }
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

  totalPacketsSentToStm++;

  // Quick blink LED on activity
  ledActivityOffUntil = millis() + 80;
}

// Send periodic Heartbeat Ping to STM32
void sendPingToSTM() {
  uint32_t payload[2];
  payload[0] = ++pingSeq;
  payload[1] = millis();
  pingSentMicros = micros();
  sendPacketToSTM(0x00, (const uint8_t*)payload, sizeof(payload));
}

// --- BLE CALLBACKS ---
class MyServerCallbacks: public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) override {
    deviceConnected = true;
    Serial.println("\n>>> [BLE] Web Bluetooth Client Connected! <<<");
  }

  void onDisconnect(BLEServer* pServer) override {
    deviceConnected = false;
    Serial.println("\n>>> [BLE] Web Bluetooth Client Disconnected. <<<");
  }
};

class CommandCallbacks: public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) override {
    uint8_t* rxData = pCharacteristic->getData();
    size_t totalLen = pCharacteristic->getLength();
    if (totalLen == 0 || rxData == nullptr) return;

    totalBleCmdsReceived++;

    uint8_t cmd = rxData[0];
    const uint8_t* data = (totalLen > 1) ? (rxData + 1) : nullptr;
    size_t len = (totalLen > 1) ? (totalLen - 1) : 0;

    Serial.printf("[BLE RX -> STM32] Forwarding CMD 0x%02X (Payload: %d bytes)\n", cmd, (int)len);

    // Relay command directly to STM32 via high-speed UART
    sendPacketToSTM(cmd, data, len);
  }
};

// --- SETUP HARDWARE ---
void setup() {
  pinMode(LED_STATUS_PIN, OUTPUT);
  digitalWrite(LED_STATUS_PIN, LOW);

  Serial.begin(115200);
  Serial2.begin(UART_BAUD, SERIAL_8N1, UART_STM_RX, UART_STM_TX);
  delay(200);

  Serial.println("\n===========================================================");
  Serial.println("   FUNCTION GENERATOR 10000 - ESP32 BLE WIRELESS GATEWAY   ");
  Serial.println("   Web Bluetooth (Chrome/Edge/Vercel) <---> STM32 Nucleo   ");
  Serial.println("===========================================================");
  Serial.printf(" - UART2 RX: GPIO %d (connect to STM32 Pin D1 / PB14 TX)\n", UART_STM_RX);
  Serial.printf(" - UART2 TX: GPIO %d (connect to STM32 Pin D0 / PB15 RX)\n", UART_STM_TX);
  Serial.printf(" - Baudrate: %d bps 8N1\n", UART_BAUD);
  Serial.printf(" - Status LED: GPIO %d\n", LED_STATUS_PIN);
  Serial.println("-----------------------------------------------------------");

  // Initialize BLE Device
  BLEDevice::init("FuncGen-10000");
  BLEDevice::setMTU(256); // Support unfragmented long packets (equations & waveforms)

  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create BLE GATT Service
  BLEService *pService = pServer->createService(SERVICE_UUID);

  // 1. Command Characteristic (Web App writes commands)
  pCmdChar = pService->createCharacteristic(
                      CHAR_COMMAND_UUID,
                      BLECharacteristic::PROPERTY_WRITE |
                      BLECharacteristic::PROPERTY_WRITE_NR
                    );
  pCmdChar->setCallbacks(new CommandCallbacks());

  // 2. Telemetry Characteristic (ESP32 sends push notifications)
  pTelemChar = pService->createCharacteristic(
                      CHAR_TELEMETRY_UUID,
                      BLECharacteristic::PROPERTY_READ |
                      BLECharacteristic::PROPERTY_NOTIFY
                    );
  pTelemChar->addDescriptor(new BLE2902());

  // Start BLE Service
  pService->start();

  // Start BLE Advertising
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06); // Settings to assist iOS / mobile connections
  pAdvertising->setMinPreferred(0x12);
  BLEDevice::startAdvertising();

  Serial.println("✔ BLE Advertising ACTIVE as 'FuncGen-10000'");
  Serial.println("✔ Waiting for Web App to pair via Web Bluetooth...");
  Serial.println("===========================================================\n");
  Serial.flush();
}

// --- UART PACKET PARSER FOR STM32 RESPONSES ---
void processStm32Uart() {
  static uint8_t rxState = 0;
  static uint8_t rxCmd = 0;
  static uint8_t rxLen = 0;
  static uint8_t rxBuf[128];
  static uint8_t rxIdx = 0;

  while (Serial2.available()) {
    uint8_t b = Serial2.read();

    if (rxState == 0) {
      if (b == 0xAA) rxState = 1;
    } else if (rxState == 1) {
      if (b == 0x55) rxState = 2;
      else rxState = (b == 0xAA) ? 1 : 0; // Prevent loss of sync on double 0xAA
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

      // Verify CRC8 integrity
      uint8_t header[4] = { 0xAA, 0x55, rxCmd, rxLen };
      uint8_t calculatedCrc = calcCRC8(rxBuf, rxLen) ^ calcCRC8(header, 4);

      if (calculatedCrc == expectedCrc) {
        // Activity blink
        ledActivityOffUntil = millis() + 80;

        // -------------------------------------------------------------
        // CMD 0x80: PONG Response from STM32 (Handshake OK)
        // -------------------------------------------------------------
        if (rxCmd == 0x80) {
          lastStm32PongTime = millis();
          stm32Connected = true;
          lastRttUs = micros() - pingSentMicros;

          if (lastRttUs < minRttUs) minRttUs = lastRttUs;
          if (lastRttUs > maxRttUs) maxRttUs = lastRttUs;

          if (rxLen >= 3) {
            stm32HwId = rxBuf[2];
          }
          if (rxLen >= 4) {
            stm32Flags = rxBuf[3];
          }
          if (rxLen >= 5) {
            stm32FwVersion = rxBuf[4];
          }
        }
        // -------------------------------------------------------------
        // CMD 0x81: ACK from STM32 (Command executed successfully)
        // -------------------------------------------------------------
        else if (rxCmd == 0x81) {
          totalAcksReceived++;
          Serial.println("✔ [STM32 -> ESP32] ACK: Command executed by STM32 silicon!");
        }
        // -------------------------------------------------------------
        // CMD 0x82: NACK from STM32 (Syntax or execution error)
        // -------------------------------------------------------------
        else if (rxCmd == 0x82) {
          totalNacksReceived++;
          Serial.println("✘ [STM32 -> ESP32] NACK: STM32 rejected command / formula!");
        }
      } else {
        totalCrcErrors++;
        Serial.printf("⚠️ [STM32 UART] CRC Error! Expected 0x%02X, Calculated 0x%02X\n", expectedCrc, calculatedCrc);
      }
    }
  }
}

// --- USB SERIAL DIAGNOSTIC CLI ---
void processUsbCli() {
  static char cmdLine[64];
  static uint8_t cmdIdx = 0;

  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\r' || c == '\n') {
      if (cmdIdx == 0) continue;
      cmdLine[cmdIdx] = '\0';

      if (strcmp(cmdLine, "help") == 0) {
        Serial.println("\n--- ESP32 BLE Gateway Commands ---");
        Serial.println("  status : Print live BLE, UART, and latency statistics");
        Serial.println("  ping   : Trigger immediate hardware ping to STM32");
        Serial.println("  reset  : Reset RTT min/max statistics");
        Serial.println("----------------------------------\n");
      } else if (strcmp(cmdLine, "status") == 0) {
        Serial.println("\n--- ESP32 Gateway Status ---");
        Serial.printf("  BLE Client:    %s\n", deviceConnected ? "CONNECTED (Active Session)" : "DISCONNECTED (Advertising)");
        Serial.printf("  STM32 Link:    %s\n", stm32Connected ? "ONLINE (UART Link Synchronized)" : "OFFLINE (No response to Ping)");
        if (stm32Connected) {
          Serial.printf("  STM32 HW ID:   0x%02X (Firmware v%d.0)\n", stm32HwId, stm32FwVersion);
          Serial.printf("  RTT Latency:   Current: %.2f ms (%u us) | Min: %.2f ms | Max: %.2f ms\n",
                        lastRttUs / 1000.0f, lastRttUs, minRttUs / 1000.0f, maxRttUs / 1000.0f);
        }
        Serial.printf("  BLE Cmds RX:   %lu\n", totalBleCmdsReceived);
        Serial.printf("  STM Pkts TX:   %lu\n", totalPacketsSentToStm);
        Serial.printf("  STM ACKs RX:   %lu\n", totalAcksReceived);
        Serial.printf("  STM NACKs RX:  %lu\n", totalNacksReceived);
        Serial.printf("  CRC Errors:    %lu\n", totalCrcErrors);
        Serial.printf("  Free Heap:     %u bytes\n", ESP.getFreeHeap());
        Serial.println("-----------------------------\n");
      } else if (strcmp(cmdLine, "ping") == 0) {
        Serial.println("Sending manual ping to STM32...");
        sendPingToSTM();
      } else if (strcmp(cmdLine, "reset") == 0) {
        minRttUs = 999999;
        maxRttUs = 0;
        Serial.println("RTT statistics reset.");
      } else {
        Serial.println("Unknown command. Type 'help' for available commands.");
      }

      cmdIdx = 0;
    } else {
      if (cmdIdx < sizeof(cmdLine) - 1) {
        cmdLine[cmdIdx++] = c;
      }
    }
  }
}

// --- MAIN LOOP ---
void loop() {
  // 1. Process UART responses from STM32
  processStm32Uart();

  // 2. Process USB Serial CLI commands
  processUsbCli();

  // 3. Periodic Handshake Ping to STM32 every 1 second
  if (millis() - lastPingTime >= 1000) {
    lastPingTime = millis();
    sendPingToSTM();
  }

  // 4. Evaluate STM32 Handshake Timeout (2.5s without response = Offline)
  if (millis() - lastStm32PongTime > 2500) {
    stm32Connected = false;
    lastRttUs = 0;
  }

  // 5. Send Unified Dual-Board Telemetry to Web App every 1 second
  static uint32_t lastTelemTime = 0;
  if (deviceConnected && pTelemChar != nullptr && (millis() - lastTelemTime >= 1000)) {
    lastTelemTime = millis();
    uint8_t telem[8];
    telem[0] = 0x80;                          // Telemetry command ID
    telem[1] = 0x01;                          // ESP32 status: 1 = Online
    telem[2] = stm32Connected ? 0x01 : 0x00;  // STM32 status: 1 = Online, 0 = Offline
    telem[3] = (uint8_t)(lastRttUs & 0xFF);   // RTT microseconds (Little Endian)
    telem[4] = (uint8_t)((lastRttUs >> 8) & 0xFF);
    telem[5] = (uint8_t)((lastRttUs >> 16) & 0xFF);
    telem[6] = (uint8_t)((lastRttUs >> 24) & 0xFF);
    telem[7] = stm32Flags;                    // Active channels flag
    pTelemChar->setValue(telem, sizeof(telem));
    pTelemChar->notify();
  }

  // 6. Handle BLE Disconnect & Reconnect Advertising cleanly
  if (!deviceConnected && oldDeviceConnected) {
    delay(500); // Allow BLE stack to stabilize
    pServer->startAdvertising();
    Serial.println("[BLE] Advertising restarted.");
    oldDeviceConnected = deviceConnected;
  }
  if (deviceConnected && !oldDeviceConnected) {
    oldDeviceConnected = deviceConnected;
  }

  // 7. Visual Status LED Feedback:
  // - Traffic/Activity: Brief blink OFF (ledActivityOffUntil)
  // - BLE Connected + STM32 Online: Solid ON
  // - BLE Connected + STM32 Offline: Fast blink (200 ms)
  // - BLE Disconnected + STM32 Online: Slow heartbeat pulse (80 ms every 1.5s)
  // - BLE Disconnected + STM32 Offline: Slow blink (500 ms)
  if (millis() < ledActivityOffUntil) {
    digitalWrite(LED_STATUS_PIN, LOW);
  } else if (deviceConnected) {
    if (stm32Connected) {
      digitalWrite(LED_STATUS_PIN, HIGH);
    } else {
      // Alert: Connected to phone/web, but STM32 cable is offline!
      digitalWrite(LED_STATUS_PIN, (millis() % 200 < 100) ? HIGH : LOW);
    }
  } else {
    // BLE Disconnected
    if (stm32Connected) {
      // STM32 is ready, waiting for BLE client: short heartbeat
      digitalWrite(LED_STATUS_PIN, (millis() % 1500 < 80) ? HIGH : LOW);
    } else {
      // Both offline: slow blink
      digitalWrite(LED_STATUS_PIN, (millis() % 1000 < 500) ? HIGH : LOW);
    }
  }

  delay(5);
}
