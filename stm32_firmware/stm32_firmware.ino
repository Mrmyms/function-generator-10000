/*
 * ==================================================================================
 * Function Generator 10000 - STM32 MB1814 (NUCLEO-H503RB / H533RE) Firmware
 * Hardware: ARM Cortex-M33 @ 250 MHz
 * Outputs: Dual 12-bit DACs at 1 MSPS (1,000,000 Samples Per Second)
 *   - Channel 1: Pin PA4 (DAC1_CH1)
 *   - Channel 2: Pin PA5 (DAC1_CH2)
 * UART Inter-Communication with ESP32 WROOM:
 *   - STM32 RX: Pin PA10 (Arduino Pin D0)  <- ESP32 TX2 (GPIO 17)
 *   - STM32 TX: Pin PA9  (Arduino Pin D1)  -> ESP32 RX2 (GPIO 16)
 *   - Baudrate: 115200 bps
 * ==================================================================================
 */

#include <Arduino.h>
#include <math.h>

#define DAC_CH1_PIN PA4
#define DAC_CH2_PIN PA5
#define LED_PIN     LED_BUILTIN

#define TABLE_SIZE 1024
#define SAMPLE_RATE_HZ 1000000.0f  // 1 MSPS (1 Mega-sample/sec)
const float PHASE_MULT = 4294967296.0f / SAMPLE_RATE_HZ;

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

struct ChannelRuntime {
  ChannelPacket cfg;
  uint32_t basePhaseInc = 0;
  uint32_t modPhaseInc  = 0;
  float    sweepStep    = 0.0f;
  uint32_t phaseAcc     = 0;
  uint32_t modPhaseAcc  = 0;
  float    sweepFreq    = 1000.0f;
  bool     sweepUp      = true;
  uint32_t burstCount   = 0;
  bool     burstActive  = true;
};

static ChannelRuntime chRun[2];
static uint16_t sineTable[TABLE_SIZE];
static uint16_t customTable[2][64];

// CRC8
uint8_t calcCRC8(const uint8_t *data, size_t len) {
  uint8_t crc = 0;
  while (len--) {
    crc ^= *data++;
    for (uint8_t j = 0; j < 8; j++) crc = (crc & 0x80) ? (crc << 1) ^ 0x07 : (crc << 1);
  }
  return crc;
}

// Waveform sample lookup (12-bit: -2048 to +2047)
static inline int16_t getWaveSample(char w, uint16_t idx, uint8_t duty, uint8_t cIdx) {
  switch (w) {
    case 's': return (int16_t)sineTable[idx] - 2048;
    case 'q': return (idx < (uint16_t)((duty * TABLE_SIZE) / 100)) ? 2047 : -2048;
    case 't': {
      uint16_t half = idx & (TABLE_SIZE / 2 - 1);
      int16_t val = (half < (TABLE_SIZE / 4)) ? (half * 8) : (4095 - (half - (TABLE_SIZE / 4)) * 8);
      return (idx < (TABLE_SIZE / 2)) ? (val - 2048) : (2047 - val);
    }
    case 'a': return (int16_t)((idx * 4) - 2048);
    case 'n': return (int16_t)(rand() % 4096) - 2048;
    case 'c': {
      uint8_t c_pos = (idx * 64) / TABLE_SIZE;
      return (int16_t)(customTable[cIdx][c_pos] * 16) - 2048;
    }
    default: return (int16_t)sineTable[idx] - 2048;
  }
}

// Compute next 12-bit sample for DAC
static inline uint16_t nextSample(uint8_t cIdx, ChannelRuntime &cr) {
  ChannelPacket &cfg = cr.cfg;
  if (!cfg.enabled) return 2048;

  if (cfg.burstEnabled) {
    if (cr.burstActive && cr.burstCount >= cfg.burstCycles) cr.burstActive = false;
    if (!cr.burstActive) return 2048;
  }

  uint32_t currentPhaseInc = cr.basePhaseInc;
  float currentAmpMod = 1.0f;

  if (cfg.modType == 1 && cr.modPhaseInc > 0) { // AM
    uint16_t mIdx = (cr.modPhaseAcc >> 22) & (TABLE_SIZE - 1);
    float mVal = (float)((int16_t)sineTable[mIdx] - 2048) / 2048.0f;
    currentAmpMod = constrain(1.0f + (mVal * (cfg.modDepth / 100.0f)), 0.0f, 2.0f);
  } else if (cfg.modType == 2 && cr.modPhaseInc > 0) { // FM
    uint16_t mIdx = (cr.modPhaseAcc >> 22) & (TABLE_SIZE - 1);
    float mVal = (float)((int16_t)sineTable[mIdx] - 2048) / 2048.0f;
    float devHz = cfg.frequency * (cfg.modDepth / 100.0f);
    currentPhaseInc = (uint32_t)(constrain(cfg.frequency + (mVal * devHz), 0.1f, 200000.0f) * PHASE_MULT);
  }

  uint32_t oldPhase = cr.phaseAcc;
  cr.phaseAcc += currentPhaseInc;
  if (cfg.burstEnabled && cr.phaseAcc < oldPhase) cr.burstCount++;
  cr.modPhaseAcc += cr.modPhaseInc;

  if (cfg.sweepTimeMs > 0 && cfg.sweepTarget > 0) {
    cr.sweepFreq += cr.sweepUp ? cr.sweepStep : -cr.sweepStep;
    if (cr.sweepUp && cr.sweepFreq >= cfg.sweepTarget) cr.sweepUp = false;
    else if (!cr.sweepUp && cr.sweepFreq <= cfg.frequency) cr.sweepUp = true;
    cr.basePhaseInc = (uint32_t)(cr.sweepFreq * PHASE_MULT);
  }

  uint16_t idx0 = (cr.phaseAcc >> 22) & (TABLE_SIZE - 1);
  uint16_t idx1 = (idx0 + 1) & (TABLE_SIZE - 1);
  uint8_t frac  = (cr.phaseAcc >> 14) & 0xFF;

  int16_t raw0 = getWaveSample(cfg.waveform, idx0, cfg.dutyCycle, cIdx);
  int16_t raw1 = getWaveSample(cfg.waveform, idx1, cfg.dutyCycle, cIdx);
  int16_t rawInterp = raw0 + (int16_t)(((raw1 - raw0) * frac) >> 8);

  int32_t s = (int32_t)(rawInterp * (cfg.amplitude / 100.0f) * currentAmpMod) + ((int32_t)cfg.offset * 16) + 2048;
  return (uint16_t)constrain(s, 0, 4095);
}

// Fast 1 MSPS synthesis trigger (1 µs interval)
HardwareTimer *timer1M = nullptr;

void timer1M_callback() {
  uint16_t s1 = nextSample(0, chRun[0]);
  uint16_t s2 = nextSample(1, chRun[1]);

  analogWrite(DAC_CH1_PIN, s1);
  analogWrite(DAC_CH2_PIN, s2);
}

// Send packet to ESP32 over UART
void sendPacketToESP(uint8_t cmd, const uint8_t* payload, size_t len) {
  uint8_t header[4] = {0xAA, 0x55, cmd, (uint8_t)len};
  uint8_t crc = calcCRC8(payload, len) ^ calcCRC8(header, 4);

  Serial1.write(header, 4);
  if (len > 0 && payload != nullptr) Serial1.write(payload, len);
  Serial1.write(crc);
}

void setup() {
  pinMode(LED_PIN, OUTPUT);
  analogWriteResolution(12); // Full 12-bit DAC mode (0..4095)

  // UART to ESP32 on Serial1 (PA9 TX, PA10 RX)
  Serial1.begin(115200);

  // USB CDC Serial for PC diagnostics
  Serial.begin(115200);

  // Precalculate 12-bit Sine Table (1024 points)
  for (uint16_t i = 0; i < TABLE_SIZE; i++) {
    sineTable[i] = (uint16_t)(2047.5f + 2047.5f * sinf(2.0f * PI * i / TABLE_SIZE));
  }

  for (int c = 0; c < 2; c++) {
    for (int i = 0; i < 64; i++) customTable[c][i] = sineTable[i * 16];
    chRun[c].cfg.enabled = 1;
    chRun[c].cfg.frequency = 1000.0f;
    chRun[c].cfg.amplitude = 100;
    chRun[c].cfg.waveform = 's';
    chRun[c].basePhaseInc = (uint32_t)(1000.0f * PHASE_MULT);
  }

  // Setup Hardware Timer at 1 MHz (1 µs period = 1 MSPS)
  TIM_TypeDef *Instance = TIM6;
  timer1M = new HardwareTimer(Instance);
  timer1M->setOverflow(1, MICROSEC_FORMAT); // 1 µs = 1 MSPS
  timer1M->attachInterrupt(timer1M_callback);
  timer1M->resume();

  digitalWrite(LED_PIN, HIGH);
}

void loop() {
  // UART Packet Receiver from ESP32
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

      if (rxCmd == 0x00) { // PING request from ESP32 -> Respond PONG!
        digitalWrite(LED_PIN, !digitalRead(LED_PIN)); // Toggle activity LED
        sendPacketToESP(0x80, rxBuf, rxLen); // Echo payload back in PONG
      } else if (rxCmd == 0x01 || rxCmd == 0x02) { // Channel 1 or 2 config
        uint8_t cIdx = rxCmd - 1;
        if (rxLen >= sizeof(ChannelPacket)) {
          ChannelPacket pkt;
          memcpy(&pkt, rxBuf, sizeof(ChannelPacket));
          ChannelRuntime &cr = chRun[cIdx];
          cr.cfg = pkt;
          cr.basePhaseInc = (uint32_t)(pkt.frequency * PHASE_MULT);
          cr.modPhaseInc  = (uint32_t)(pkt.modFreq * PHASE_MULT);
          cr.sweepFreq    = pkt.frequency;
          cr.sweepUp      = true;
          if (pkt.sweepTimeMs > 0 && pkt.sweepTarget > 0) {
            float fSteps = (pkt.sweepTimeMs * 0.001f) * SAMPLE_RATE_HZ;
            cr.sweepStep = (fSteps > 0) ? fabsf(pkt.sweepTarget - pkt.frequency) / fSteps : 0.0f;
          }
          if (pkt.burstEnabled && pkt.trigger) {
            cr.burstCount = 0;
            cr.burstActive = true;
          }
          // Send ACK
          uint8_t ackPayload[1] = {cIdx};
          sendPacketToESP(0x81, ackPayload, 1);
        }
      } else if (rxCmd == 0x03) { // Phase offset
        if (rxLen >= 4) {
          int32_t po;
          memcpy(&po, rxBuf, sizeof(po));
          float poDeg = (float)constrain(po, 0, 360);
          chRun[1].phaseAcc = chRun[0].phaseAcc + (uint32_t)((poDeg / 360.0f) * 4294967296.0f);
          sendPacketToESP(0x81, nullptr, 0);
        }
      } else if (rxCmd >= 0x10 && rxCmd <= 0x13) { // AWG chunks (16 samples)
        uint8_t chunk = rxCmd - 0x10;
        if (rxLen >= 17) {
          uint8_t cIdx = rxBuf[0];
          if (cIdx == 1 || cIdx == 2) {
            for (int i = 0; i < 16; i++) {
              customTable[cIdx - 1][chunk * 16 + i] = rxBuf[1 + i];
            }
            sendPacketToESP(0x81, nullptr, 0);
          }
        }
      }
    }
  }
}
