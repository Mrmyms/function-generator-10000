#include <Arduino.h>

#ifndef LED_BUILTIN
#define LED_BUILTIN PA5
#endif

// Test USART1 on D0 (RX: PB_15_ALT1) and D1 (TX: PB_14) using PinName constructor
Uart Serial1((PinName)PB_15_ALT1, (PinName)PB_14);

uint8_t calcCRC8(const uint8_t *data, size_t len) {
  uint8_t crc = 0;
  while (len--) {
    crc ^= *data++;
    for (uint8_t j = 0; j < 8; j++) crc = (crc & 0x80) ? (crc << 1) ^ 0x07 : (crc << 1);
  }
  return crc;
}

void sendPacketToESP(uint8_t cmd, const uint8_t* payload, size_t len) {
  uint8_t header[4] = {0xAA, 0x55, cmd, (uint8_t)len};
  uint8_t crc = calcCRC8(payload, len) ^ calcCRC8(header, 4);
  Serial1.write(header, 4);
  if (len > 0 && payload != nullptr) Serial1.write(payload, len);
  Serial1.write(crc);
}

void setup() {
  Serial.begin(115200);  // ST-LINK USB to PC (COM7)
  Serial1.begin(115200); // Pins D0 & D1 to ESP32
  pinMode(LED_BUILTIN, OUTPUT);

  Serial.println("\n--- STM32 Handshake Responder Test Started ---");
  Serial.println("Listening for ESP32 PING on D0... Responding with PONG on D1...");
}

void loop() {
  static uint8_t rxState = 0;
  static uint8_t rxCmd = 0;
  static uint8_t rxLen = 0;
  static uint8_t rxBuf[64];
  static uint8_t rxIdx = 0;

  while (Serial1.available()) {
    uint8_t b = Serial1.read();
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

      uint8_t header[4] = {0xAA, 0x55, rxCmd, rxLen};
      uint8_t crc = calcCRC8(rxBuf, rxLen) ^ calcCRC8(header, 4);

      if (crc == expectedCrc) {
        if (rxCmd == 0x00) { // PING received from ESP32
          uint8_t pongPayload[6];
          pongPayload[0] = (rxLen > 0) ? rxBuf[0] : 0;
          pongPayload[1] = (rxLen > 1) ? rxBuf[1] : 0;
          pongPayload[2] = 0x54; // Hardware ID STM32
          pongPayload[3] = 0x00; // Channels OFF
          pongPayload[4] = 0x01; // FW v1.0
          pongPayload[5] = 0x00;
          sendPacketToESP(0x80, pongPayload, sizeof(pongPayload));

          Serial.printf("⚡ [STM32] PING received! PONG sent to ESP32! (Seq %d)\n", pongPayload[0]);
          digitalWrite(LED_BUILTIN, HIGH);
          delay(40);
          digitalWrite(LED_BUILTIN, LOW);
        }
      }
    }
  }
}
