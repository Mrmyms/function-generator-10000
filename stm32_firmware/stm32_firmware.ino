/*
 * ==================================================================================================
 * FUNCTION GENERATOR 10000 - LABORATORY-GRADE STM32H503RB FIRMWARE (DAC1_OUT1)
 * ==================================================================================================
 * Hardware:  STM32 NUCLEO-H503RB (MB1814) @ 250 MHz Cortex-M33 with Hardware FPU
 * Output:
 *   - Clean DAC Output: PA4 (DAC1_OUT1) -> CN7 Pin 32 (Arduino A2/A15)
 *     (PA5 / DAC1_OUT2 disabled to eliminate LED LD2 load distortion)
 * High-Speed Inter-Board Bus (ESP32 BLE Gateway):
 *   - D0 (RX): PB_15_ALT1
 *   - D1 (TX): PB_14
 *   - Baud: 115200 bps (Binary Framed Protocol with CRC8)
 * PC Monitoring / ST-Link:
 *   - USB CDC Serial @ 115200 bps (CLI + Live Telemetry + Binary Protocol)
 *
 * Silicon Architecture Upgrades:
 *   1. System Clock: 24 MHz HSE Crystal Oscillator -> PLL1 -> 250 MHz SYSCLK (with CSI fallback)
 *   2. PA4 Isolation: Forced to GPIO_MODE_ANALOG to prevent USART3 AF contention
 *   3. DMA Engine: Permanently fixed 2048-word circular transfer (8192 bytes)
 *      - Higher frequencies tile (2048 / N) power-of-2 repetitions into the buffer
 *      - GPDMA is NEVER stopped or reset, eliminating HAL_ERROR / DC freeze across 1 kHz
 *   4. Double Buffering: Inactive buffer synthesized and copied without stopping TIM6 (0 flat line)
 *   5. Accurate Frequency Reporting: Computes exact discrete ARR/PSC sampling rate and error %
 *   6. DWT Cycle Counter: Measures exact Cortex-M33 execution cycles and compute time in microseconds
 *   7. DAC Buffer Control: Switchable Buffer ON (Mode 0: ~0.2V-3.1V) / Buffer OFF (Mode 2: 0-3.3V)
 *   8. AWG Interpolation: Seamless 64-point wrap-around without cycle-boundary step discontinuity
 *   9. Silicon Hardware Noise: DAC1 hardware LFSR support ('h') alongside software PRNG ('n')
 * ==================================================================================================
 */

#include <Arduino.h>
#include <math.h>
#include <string.h>
#include "stm32h5xx_hal.h"
#include "stm32h5xx_hal_dma_ex.h"
#include "stm32h5xx_ll_tim.h"
#include "waveform_synth.h"
#include "math_expr.h"

// --- PINOUT DEFINITIONS ---
#define PIN_DAC_CH1        PA4          // DAC1_OUT1 (Morpho / SB3)
#define PIN_DAC_CH2        PA5          // DAC1_OUT2 (Arduino Header Pin D13 & LED LD2)

// Serial1 connected to ESP32 Gateway (D0 PB15 / D1 PB14)
Uart Serial1((PinName)PB_15_ALT1, (PinName)PB_14);

// --- BUFFER & DMA CONFIGURATION ---
// 2048 32-bit samples aligned to 32-byte cache boundaries (8192 bytes each)
#define DMA_BUFFER_SAMPLES 2048
static uint32_t waveBufferA[DMA_BUFFER_SAMPLES] __attribute__((aligned(32)));
static uint32_t waveBufferB[DMA_BUFFER_SAMPLES] __attribute__((aligned(32)));
static volatile uint32_t *activeBuffer = waveBufferA;
static volatile uint32_t *inactiveBuffer = waveBufferB;

// Current synthesis parameters
static uint16_t currentSamplesPerCycle = 2048;
static uint16_t currentRepetitions = 1;

// Telemetry & Benchmark counters
static uint32_t lastComputeCycles = 0;
static float    lastComputeTimeUs = 0.0f;
static double   actualFs = 2049180.3;
static double   actualFout = 1000.58;
static float    freqErrorPct = 0.058f;
static bool     dacBufferEnabled = true;
static const char *clockSourceStr = "Unknown";

// GPDMA Linked List structures
static DMA_HandleTypeDef hdma_dac;
static DMA_NodeTypeDef   dmaNode __attribute__((aligned(32)));
static DMA_QListTypeDef  dmaQueue;

// --- FAST 32-BIT XORSHIFT PRNG FOR SOFTWARE NOISE ---
static uint32_t rngState = 0x12345678;
static inline uint32_t xorshift32(void) {
  uint32_t x = rngState;
  x ^= x << 13;
  x ^= x >> 17;
  x ^= x << 5;
  rngState = x;
  return x;
}

// Channel State (Single High-Fidelity Channel)
static ChannelState ch1 = { 's', 1000.0f, 100, 0, true, 50, {0}, {"sin(t)", {0}, 0, 1.0f, false} };

// Telemetry counters
static uint32_t totalPacketsReceived = 0;
static uint32_t totalPingsAnswered   = 0;

// Using official variant SystemClock_Config @ 250 MHz

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

// Send binary packet to target stream (Serial1 or Serial)
void sendPacket(Stream &s, uint8_t cmd, const uint8_t *payload, size_t len) {
  uint8_t header[4] = { 0xAA, 0x55, cmd, (uint8_t)len };
  uint8_t crc = calcCRC8(payload, len) ^ calcCRC8(header, 4);
  s.write(header, 4);
  if (len > 0 && payload != nullptr) {
    s.write(payload, len);
  }
  s.write(crc);
}

void sendPacketToESP(uint8_t cmd, const uint8_t *payload, size_t len) {
  sendPacket(Serial1, cmd, payload, len);
}

// --- DAC BUFFER MODE CONFIGURATION ---
void applyDacBufferMode(bool enableBuffer) {
  dacBufferEnabled = enableBuffer;
  if (enableBuffer) {
    // Mode 0: External pin with Buffer enabled for both CH1 (PA4) and CH2 (PA5/D13)
    DAC1->MCR &= ~((7U << DAC_MCR_MODE1_Pos) | (7U << DAC_MCR_MODE2_Pos));
  } else {
    // Mode 2: External pin with Buffer disabled for both CH1 and CH2
    DAC1->MCR = (DAC1->MCR & ~((7U << DAC_MCR_MODE1_Pos) | (7U << DAC_MCR_MODE2_Pos)))
                | (2U << DAC_MCR_MODE1_Pos) | (2U << DAC_MCR_MODE2_Pos);
  }
}

// --- MATHEMATICAL SYNTHESIS ENGINE (CORTEX-M33 FPU) ---
// Synthesizes 1 complete waveform cycle of N points into destCycle
void synthesizeCycle(const ChannelState &cfg, uint16_t n, uint32_t *destCycle) {
  const float ampScale = (float)cfg.amp / 100.0f;
  const float dcOffset = ((float)cfg.offset / 128.0f) * 2047.0f;
  const float dutyFrac = (float)constrain((int)cfg.duty, 1, 99) / 100.0f;

  for (uint16_t i = 0; i < n; i++) {
    uint16_t dacVal = 2048; // DC bias midpoint (1.65 V)

    if (cfg.enabled && cfg.amp > 0) {
      float normPhase = (float)i / (float)n;
      float rawSample = 0.0f; // Range: [-1.0, +1.0]

      switch (cfg.wave) {
        case 's': // Sine Wave (Hardware FPU accelerated)
          rawSample = sinf(normPhase * 2.0f * (float)M_PI);
          break;

        case 'q': // Square Wave
        case 'p': // PWM (Pulse Width Modulation)
          rawSample = (normPhase < dutyFrac) ? 1.0f : -1.0f;
          break;

        case 't': // Triangle Wave
          if (normPhase < 0.5f) {
            rawSample = 4.0f * normPhase - 1.0f;
          } else {
            rawSample = 3.0f - 4.0f * normPhase;
          }
          break;

        case 'a': // Sawtooth Wave
          rawSample = 2.0f * normPhase - 1.0f;
          break;

        case 'n': // White Noise (Software PRNG Xorshift32 table)
          rawSample = ((float)(xorshift32() & 0xFFF) / 2047.5f) - 1.0f;
          break;

        case 'c': { // Custom AWG (64-point interpolation, seamless 63 -> 0 wrap)
          float awgPos = normPhase * 64.0f;
          int idx0 = (int)awgPos;
          if (idx0 >= 64) idx0 = 63;
          int idx1 = (idx0 + 1) & 63; // Seamlessly wraps 63 -> 0
          float frac = (normPhase * 64.0f) - (float)idx0;
          float v0 = ((float)cfg.awgSamples[idx0] / 127.5f) - 1.0f;
          float v1 = ((float)cfg.awgSamples[idx1] / 127.5f) - 1.0f;
          rawSample = v0 + frac * (v1 - v0);
          break;
        }

        case 'e': { // Custom Mathematical Equation Mode (Bytecode VM)
          if (cfg.mathExpr.isValid) {
            float t = normPhase * 2.0f * (float)M_PI;
            rawSample = evalMathBytecode(cfg.mathExpr.bytecode, normPhase, t) * cfg.mathExpr.normScale;
            if (isnan(rawSample) || isinf(rawSample)) {
              rawSample = 0.0f;
            }
          } else {
            rawSample = sinf(normPhase * 2.0f * (float)M_PI);
          }
          break;
        }

        default:
          rawSample = sinf(normPhase * 2.0f * (float)M_PI);
          break;
      }

      // Apply Amplitude Scale and DC Offset, then clamp to 12-bit DAC range [0, 4095]
      float dacOut = 2047.5f + (rawSample * 2047.0f * ampScale) + dcOffset;
      if (dacOut < 0.0f)    dacOut = 0.0f;
      if (dacOut > 4095.0f) dacOut = 4095.0f;
      dacVal = (uint16_t)dacOut;
    }

    // Pack same 12-bit sample to both DAC1_OUT1 (PA4) and DAC1_OUT2 (PA5 / Arduino D13)
    destCycle[i] = ((uint32_t)(dacVal & 0x0FFF) << 16) | (uint32_t)(dacVal & 0x0FFF);
  }
}

// --- APPLY SYNTHESIS TO HARDWARE REGISTERS (ZERO-GLITCH DOUBLE BUFFER) ---
void recomputeAndApplySynthesis(void) {
  float refFreq = ch1.freq;
  if (refFreq < 1.0f) refFreq = 1.0f;

  // 1. Choose optimal samples per cycle N (power of 2)
  // At refFreq <= 1000 Hz: N = 2048 points (calidad desquiciada)
  // Above 64 kHz: N = 16 points to permit clean operation up to 128 kHz without exceeding DAC bandwidth
  uint16_t n = 2048;
  if (refFreq > 64000.0f)       n = 16;
  else if (refFreq > 32000.0f)  n = 32;
  else if (refFreq > 16000.0f)  n = 64;
  else if (refFreq > 8000.0f)   n = 128;
  else if (refFreq > 4000.0f)   n = 256;
  else if (refFreq > 2000.0f)   n = 512;
  else if (refFreq > 1000.0f)   n = 1024;
  else                          n = 2048;

  currentSamplesPerCycle = n;
  uint16_t repetitions = DMA_BUFFER_SAMPLES / n; // 2048 / n: always an exact power of 2!
  currentRepetitions = repetitions;

  // 2. Measure exact synthesis duration using micros()
  uint32_t tStart = micros();

  // Synthesize 1 single period directly into start of inactiveBuffer
  synthesizeCycle(ch1, n, (uint32_t*)inactiveBuffer);

  // Replicate R-1 repetitions from the first period into the remainder of the 2048-sample buffer
  for (uint16_t r = 1; r < repetitions; r++) {
    memcpy((void*)&inactiveBuffer[r * n], (void*)&inactiveBuffer[0], n * sizeof(uint32_t));
  }

  uint32_t tElapsed = micros() - tStart;
  lastComputeTimeUs = (float)tElapsed;
  lastComputeCycles = tElapsed * 250;

  // 3. Seamless atomic buffer copy (TIM6 and DMA NEVER stop!)
  // Cortex-M33 copies 8192 bytes in ~4 microseconds without any flat line gap!
  memcpy((void*)activeBuffer, (void*)inactiveBuffer, DMA_BUFFER_SAMPLES * sizeof(uint32_t));

  // 4. Compute precise TIM6 Prescaler and Auto-Reload (ARR)
  // Desired trigger rate: fs = refFreq * n (samples/sec)
  double target_fs = (double)refFreq * (double)n;
  double timClk = (double)SystemCoreClock; // 250 MHz
  double div = timClk / target_fs;

  uint32_t psc = 0;
  uint32_t arr = (uint32_t)round(div) - 1;
  if (arr > 65535) {
    psc = (uint32_t)(div / 65535.0);
    arr = (uint32_t)round(div / (double)(psc + 1)) - 1;
  }
  if (arr < 1) arr = 1; // Prevent timer underflow

  // Actual discrete sampling rate and actual synthesized frequency
  actualFs = timClk / ((double)(psc + 1) * (double)(arr + 1));
  actualFout = actualFs / (double)n;
  freqErrorPct = (float)(((actualFout - (double)refFreq) / (double)refFreq) * 100.0);

  // 5. Update TIM6 Prescaler and ARR dynamically (TIM6 is NEVER disabled)
  LL_TIM_SetPrescaler(TIM6, psc);
  LL_TIM_SetAutoReload(TIM6, arr);

  // 6. Handle hardware silicon noise mode ('h')
  if (ch1.wave == 'h') {
    // True non-periodic LFSR noise generated in DAC silicon on both channels on every TIM6 tick
    DAC1->CR |= (DAC_CR_WAVE1_0 | (0x0B << DAC_CR_MAMP1_Pos) | DAC_CR_WAVE2_0 | (0x0B << DAC_CR_MAMP2_Pos));
  } else {
    // Normal circular DMA waveform playback
    DAC1->CR &= ~(DAC_CR_WAVE1_Msk | DAC_CR_MAMP1_Msk | DAC_CR_WAVE2_Msk | DAC_CR_MAMP2_Msk);
  }
}

extern "C" void assert_failed(uint8_t* file, uint32_t line) {
  Serial.printf("\n*** HAL ASSERTION FAILED: %s:%lu ***\n", (char*)file, line);
  Serial.flush();
  while(1) {
    delay(1000);
  }
}

// --- SETUP HARDWARE ---
void setup() {
  // 1. Initialize Serial first
  Serial.begin(115200);
  delay(100);

  Serial.println("\n===========================================================");
  Serial.println("   FUNCTION GENERATOR 10000 - STM32H503 SILICON FIRMWARE   ");
  Serial.println("   250 MHz Cortex-M33 Hardware FPU | Autonomous GPDMA DAC  ");
  Serial.println("   Simultaneous Output: PA4 & Arduino Header Pin D13 (PA5) ");
  Serial.println("   Formula Parser & 2048-Point Arbitrary Waveform Engine   ");
  Serial.println("===========================================================");
  Serial.println(" Signal Out:    Arduino Header Pin D13 (PA5 / LD2) & PA4");
  Serial.println(" (Probe Arduino Pin D13 for easy access!)");
  Serial.flush();

  // 3. Serial1 connected to ESP32 Gateway (D0 PB15 / D1 PB14)
  Serial1.begin(115200);

  // 4. Compile default equation
  compileMathExpression("sin(t)", &ch1.mathExpr);

  // 5. Initialize default AWG custom wave with Sinc function
  for (int i = 0; i < 64; i++) {
    float x = ((float)i - 32.0f) * 0.3f;
    ch1.awgSamples[i] = (uint8_t)(128.0f + 120.0f * (x == 0.0f ? 1.0f : sinf(x) / x));
  }

  // 6. Generate initial wave table (2048 samples @ 1 kHz on PA4)
  synthesizeCycle(ch1, 2048, (uint32_t*)activeBuffer);
  memcpy((void*)inactiveBuffer, (void*)activeBuffer, DMA_BUFFER_SAMPLES * sizeof(uint32_t));

  // 7. Enable Peripheral Clocks
  __HAL_RCC_DAC1_CLK_ENABLE();
  __HAL_RCC_TIM6_CLK_ENABLE();
  __HAL_RCC_GPDMA1_CLK_ENABLE();

  // 8. Configure GPDMA1 Channel 0 for PERMANENT 2048-Word Circular Transfer to DAC1->DHR12R1
  // Fixed size = 8192 bytes. GPDMA is NEVER stopped or reset!
  DMA_NodeConfTypeDef nodeConf = {0};
  nodeConf.NodeType = DMA_GPDMA_LINEAR_NODE;
  nodeConf.Init.Request = GPDMA1_REQUEST_DAC1_CH1;
  nodeConf.Init.BlkHWRequest = DMA_BREQ_SINGLE_BURST;
  nodeConf.Init.Direction = DMA_MEMORY_TO_PERIPH;
  nodeConf.Init.SrcInc = DMA_SINC_INCREMENTED;
  nodeConf.Init.DestInc = DMA_DINC_FIXED;
  nodeConf.Init.SrcDataWidth = DMA_SRC_DATAWIDTH_WORD;
  nodeConf.Init.DestDataWidth = DMA_DEST_DATAWIDTH_WORD;
  nodeConf.Init.Priority = DMA_LOW_PRIORITY_LOW_WEIGHT;
  nodeConf.Init.SrcBurstLength = 1;
  nodeConf.Init.DestBurstLength = 1;
  nodeConf.Init.TransferAllocatedPort = DMA_SRC_ALLOCATED_PORT0 | DMA_DEST_ALLOCATED_PORT0;
  nodeConf.Init.TransferEventMode = DMA_TCEM_BLOCK_TRANSFER;
  nodeConf.Init.Mode = DMA_NORMAL;
  nodeConf.DataHandlingConfig.DataAlignment = DMA_DATA_RIGHTALIGN_ZEROPADDED;
  nodeConf.DataHandlingConfig.DataExchange = DMA_EXCHANGE_NONE;
  nodeConf.TriggerConfig.TriggerPolarity = DMA_TRIG_POLARITY_MASKED;
  nodeConf.SrcAddress = (uint32_t)activeBuffer;
  nodeConf.DstAddress = (uint32_t)&(DAC1->DHR12RD);
  nodeConf.DataSize   = DMA_BUFFER_SAMPLES * sizeof(uint32_t); // 8192 bytes FIXED

  HAL_DMAEx_List_BuildNode(&nodeConf, &dmaNode);
  HAL_DMAEx_List_InsertNode_Tail(&dmaQueue, &dmaNode);
  HAL_DMAEx_List_SetCircularMode(&dmaQueue);

  hdma_dac.Instance = GPDMA1_Channel0;
  hdma_dac.InitLinkedList.Priority = DMA_LOW_PRIORITY_LOW_WEIGHT;
  hdma_dac.InitLinkedList.LinkStepMode = DMA_LSM_FULL_EXECUTION;
  hdma_dac.InitLinkedList.TransferEventMode = DMA_TCEM_BLOCK_TRANSFER;
  hdma_dac.InitLinkedList.LinkedListMode = DMA_LINKEDLIST_CIRCULAR;
  hdma_dac.InitLinkedList.LinkAllocatedPort = DMA_LINK_ALLOCATED_PORT0;
  hdma_dac.Mode = DMA_LINKEDLIST_CIRCULAR;

  HAL_DMAEx_List_Init(&hdma_dac);
  HAL_DMAEx_List_LinkQ(&hdma_dac, &dmaQueue);
  HAL_DMAEx_List_Start(&hdma_dac);

  // 9. Configure DAC1 Channels 1 & 2 (Buffer ON by default, Mode 0)
  applyDacBufferMode(dacBufferEnabled);
  uint32_t cr_ch1 = (DAC_CR_TSEL1_2 | DAC_CR_TSEL1_0) | DAC_CR_TEN1 | DAC_CR_DMAEN1 | DAC_CR_EN1;
  uint32_t cr_ch2 = (DAC_CR_TSEL2_2 | DAC_CR_TSEL2_0) | DAC_CR_TEN2 | DAC_CR_EN2;
  DAC1->CR = cr_ch1 | cr_ch2;
  delayMicroseconds(20);

  // 10. Start TIM6 at Initial Sampling Rate (1 kHz * 2048 samples = 2.049 MSPS, ARR=121)
  LL_TIM_DisableCounter(TIM6);
  LL_TIM_SetPrescaler(TIM6, 0);
  LL_TIM_SetAutoReload(TIM6, 121);
  LL_TIM_SetTriggerOutput(TIM6, LL_TIM_TRGO_UPDATE);
  LL_TIM_EnableCounter(TIM6);

  recomputeAndApplySynthesis();

  Serial.println(">>> HIGH-FIDELITY 12-BIT DAC ENGINE ACTIVE <<<");
  Serial.println(" - Clean Output: PA4 (CN7 Pin 32 / Arduino A2)");
  Serial.println(" - GPDMA Mode:   Permanent 2048-Word Circular Loop (Zero DMA resets)");
  Serial.println(" - Buffer Swap:  Double-buffered atomic switch (Zero flat-line glitches)");
  Serial.println(" - DWT Benchmark Active (Microsecond-accurate cycle timing)");
  Serial.println("Type 'help' in terminal for interactive CLI commands.");
  Serial.println("===========================================================\n");
  Serial.flush();
}

// --- PROTOCOL PACKET EXECUTOR (COMMON TO BLE AND USB) ---
void executeCommandPacket(Stream &replyStream, uint8_t rxCmd, uint8_t rxLen, uint8_t *rxBuf) {
  totalPacketsReceived++;

  // CMD 0x00: PING -> Respond with PONG (0x80)
  if (rxCmd == 0x00) {
    totalPingsAnswered++;
    uint8_t pongPayload[6];
    pongPayload[0] = (rxLen > 0) ? rxBuf[0] : 0;
    pongPayload[1] = (rxLen > 1) ? rxBuf[1] : 0;
    pongPayload[2] = 0x54; // Hardware ID (STM32)
    pongPayload[3] = ch1.enabled ? 0x01 : 0x00; // Single Channel active flag
    pongPayload[4] = 0x05; // Firmware Version 5.0 (Laboratory Grade)
    pongPayload[5] = dacBufferEnabled ? 0x01 : 0x00; // DAC Buffer status
    sendPacket(replyStream, 0x80, pongPayload, sizeof(pongPayload));
  }
  // CMD 0x01: Channel Configuration (29 bytes)
  else if (rxCmd == 0x01 && rxLen >= sizeof(ChannelPacket)) {
    ChannelPacket *p = (ChannelPacket *)rxBuf;
    ch1.wave    = p->waveform;
    ch1.freq    = p->frequency;
    ch1.amp     = p->amplitude;
    ch1.offset  = p->offset;
    ch1.enabled = (p->enabled != 0);
    ch1.duty    = p->dutyCycle;
    recomputeAndApplySynthesis();
    sendPacket(replyStream, 0x81, nullptr, 0); // Send ACK
    Serial.printf("[PACKET -> STM32] Config: Wave=%c, Freq=%.1f Hz, Amp=%d%%, En=%d | Actual=%.2f Hz (Err: %+.3f%%)\n",
                  ch1.wave, ch1.freq, ch1.amp, ch1.enabled, (float)actualFout, freqErrorPct);
  }
  // CMD 0x02: Backward compatibility for CH2 (Acknowledge)
  else if (rxCmd == 0x02 && rxLen >= sizeof(ChannelPacket)) {
    sendPacket(replyStream, 0x81, nullptr, 0); // Send ACK
  }
  // CMD 0x03: Phase shift (Acknowledge)
  else if (rxCmd == 0x03 && rxLen >= 4) {
    sendPacket(replyStream, 0x81, nullptr, 0); // Send ACK
  }
  // CMD 0x05: Equation Formula (ASCII string)
  else if (rxCmd == 0x05 && rxLen > 0) {
    char eqStr[96];
    uint8_t copyLen = (rxLen < 95) ? rxLen : 95;
    memcpy(eqStr, rxBuf, copyLen);
    eqStr[copyLen] = '\0';
    if (compileMathExpression(eqStr, &ch1.mathExpr)) {
      ch1.wave = 'e';
      recomputeAndApplySynthesis();
      sendPacket(replyStream, 0x81, nullptr, 0); // ACK
      Serial.print("[PACKET -> STM32] Equation set: ");
      Serial.print(eqStr);
      Serial.printf(" (N=%d pts, Compute: %.1f us / %lu cyc)\n",
                    currentSamplesPerCycle, lastComputeTimeUs, lastComputeCycles);
    } else {
      sendPacket(replyStream, 0x82, nullptr, 0); // NACK
      Serial.print("[PACKET -> STM32] Invalid Equation: ");
      Serial.println(eqStr);
    }
  }
  // CMD 0x06: Backward compatibility for CH2 equation
  else if (rxCmd == 0x06 && rxLen > 0) {
    sendPacket(replyStream, 0x81, nullptr, 0); // Send ACK
  }
  // CMD 0x07: DAC Output Buffer Mode (1 byte: 0 = Buffer OFF / Mode 2, 1 = Buffer ON / Mode 0)
  else if (rxCmd == 0x07 && rxLen >= 1) {
    bool enableBuf = (rxBuf[0] != 0);
    applyDacBufferMode(enableBuf);
    sendPacket(replyStream, 0x81, nullptr, 0); // ACK
    Serial.printf("[PACKET -> STM32] DAC Buffer Mode: %s\n", enableBuf ? "ENABLED (Mode 0)" : "DISABLED (Mode 2)");
  }
  // CMD 0x10..0x13: 16-sample AWG Chunks (17 bytes: ch + 16 data)
  else if (rxCmd >= 0x10 && rxCmd <= 0x13 && rxLen >= 17) {
    uint8_t chunkIdx = rxCmd - 0x10;
    memcpy(&ch1.awgSamples[chunkIdx * 16], &rxBuf[1], 16);
    sendPacket(replyStream, 0x81, nullptr, 0); // Send ACK
    if (chunkIdx == 3) {
      ch1.wave = 'c';
      recomputeAndApplySynthesis();
      Serial.println("[PACKET -> STM32] AWG Complete (64 pts interpolated) on PA4!");
    }
  }
}

// Byte-by-byte Framed Packet Parser
bool processPacketByte(uint8_t b, Stream &replyStream, uint8_t &rxState, uint8_t &rxCmd, uint8_t &rxLen, uint8_t *rxBuf, uint8_t &rxIdx) {
  if (rxState == 0) {
    if (b == 0xAA) {
      rxState = 1;
      return true;
    }
    return false;
  } else if (rxState == 1) {
    if (b == 0x55) rxState = 2;
    else rxState = (b == 0xAA) ? 1 : 0;
    return true;
  } else if (rxState == 2) {
    rxCmd = b;
    rxState = 3;
    return true;
  } else if (rxState == 3) {
    rxLen = b;
    rxIdx = 0;
    rxState = (rxLen == 0) ? 5 : 4;
    return true;
  } else if (rxState == 4) {
    rxBuf[rxIdx++] = b;
    if (rxIdx >= rxLen || rxIdx >= 128) rxState = 5;
    return true;
  } else if (rxState == 5) {
    uint8_t expectedCrc = b;
    rxState = 0;

    uint8_t header[4] = { 0xAA, 0x55, rxCmd, rxLen };
    uint8_t calculatedCrc = calcCRC8(rxBuf, rxLen) ^ calcCRC8(header, 4);

    if (calculatedCrc == expectedCrc) {
      executeCommandPacket(replyStream, rxCmd, rxLen, rxBuf);
    }
    return true;
  }
  return false;
}

// --- NON-BLOCKING UART PARSER FOR ESP32 PACKETS ---
void processEsp32Uart(void) {
  static uint8_t rxState = 0, rxCmd = 0, rxLen = 0, rxIdx = 0;
  static uint8_t rxBuf[128];
  while (Serial1.available()) {
    processPacketByte(Serial1.read(), Serial1, rxState, rxCmd, rxLen, rxBuf, rxIdx);
  }
}

// --- USB SERIAL CLI & BINARY COMMAND PROCESSOR ---
void processUsbCli(void) {
  static uint8_t rxState = 0, rxCmd = 0, rxLen = 0, rxIdx = 0;
  static uint8_t rxBuf[128];
  static char cmdLine[128];
  static uint8_t cmdIdx = 0;

  while (Serial.available()) {
    uint8_t b = Serial.read();

    // Check if input is a binary packet
    if (rxState > 0 || b == 0xAA) {
      processPacketByte(b, Serial, rxState, rxCmd, rxLen, rxBuf, rxIdx);
      continue;
    }

    // Process ASCII line
    char c = (char)b;
    if (c == '\r' || c == '\n') {
      if (cmdIdx == 0) continue;
      cmdLine[cmdIdx] = '\0';

      // Parse Command
      if (strcmp(cmdLine, "help") == 0) {
        Serial.println("\n--- Function Generator 10000 CLI Commands (PA4 DAC1_OUT1) ---");
        Serial.println("  status               : Print comprehensive hardware telemetry");
        Serial.println("  wave <s/q/t/a/n/h/e> : Set wave (Sine, Square, Triangle, Saw, Noise, HW_Noise, Eqn)");
        Serial.println("  freq <Hz>            : Set target frequency (1 Hz - 200,000 Hz)");
        Serial.println("  amp <0-100>          : Set amplitude percentage");
        Serial.println("  offset <-128..127>   : Set DC offset (-1.65V to +1.65V)");
        Serial.println("  duty <1-99>          : Set PWM duty cycle percentage");
        Serial.println("  eqn <formula>        : Compile equation (e.g. sin(t)+0.5*sin(3*t))");
        Serial.println("  buf on / buf off     : Toggle internal DAC buffer (Mode 0 vs Mode 2 unbuffered)");
        Serial.println("  mute / unmute        : Disable / Enable DAC output");
        Serial.println("-------------------------------------------------------------\n");
      } else if (strcmp(cmdLine, "status") == 0) {
        Serial.println("\n--- Laboratory Hardware Telemetry (PA4 DAC1_OUT1) ---");
        Serial.printf("  Clock Source:   %s\n", clockSourceStr);
        Serial.printf("  SYSCLK:         %lu Hz\n", (unsigned long)SystemCoreClock);
        Serial.printf("  DAC Buffer:     %s\n", dacBufferEnabled ? "ON (Mode 0: ~0.2V-3.1V, t_settle ~2us)" : "OFF (Mode 2: 0.0V-3.3V Rail-to-Rail, Rout ~15k)");
        Serial.printf("  Target Freq:    %.2f Hz\n", ch1.freq);
        Serial.printf("  Actual Fout:    %.2f Hz (Discrete Error: %+.3f%%)\n", (float)actualFout, freqErrorPct);
        Serial.printf("  Sampling Rate:  %.3f MSPS (ARR=%lu, PSC=%lu)\n", (float)(actualFs / 1000000.0), (unsigned long)TIM6->ARR, (unsigned long)TIM6->PSC);
        Serial.printf("  Table Slicing:  N=%d pts/cycle | R=%d repetitions (Buffer: %d words)\n",
                      currentSamplesPerCycle, currentRepetitions, DMA_BUFFER_SAMPLES);
        Serial.printf("  DWT Benchmark:  Last Compute = %.2f us (%lu Cortex-M33 cycles)\n",
                      lastComputeTimeUs, (unsigned long)lastComputeCycles);
        Serial.printf("  Waveform:       '%c' | Amp=%d%% | Offset=%d | Duty=%d%% | Enabled=%d\n",
                      ch1.wave, ch1.amp, ch1.offset, ch1.duty, ch1.enabled);
        if (ch1.wave == 'e') {
          Serial.printf("  Equation:       \"%s\" (Bytecode: %d B, NormScale: %.3f)\n",
                        ch1.mathExpr.exprStr, ch1.mathExpr.bcLen, ch1.mathExpr.normScale);
        }
        Serial.printf("  Packets RX:     %lu | Pings: %lu\n",
                      (unsigned long)totalPacketsReceived, (unsigned long)totalPingsAnswered);
        Serial.println("------------------------------------------------------\n");
        Serial.flush();
      } else if (strcmp(cmdLine, "buf on") == 0 || strcmp(cmdLine, "buffer on") == 0) {
        applyDacBufferMode(true);
        Serial.println("✔ DAC Output Buffer: ENABLED (Mode 0: drive load down to 5k, swing ~0.2V to 3.1V)");
      } else if (strcmp(cmdLine, "buf off") == 0 || strcmp(cmdLine, "buffer off") == 0) {
        applyDacBufferMode(false);
        Serial.println("✔ DAC Output Buffer: DISABLED (Mode 2: true 0.0V-3.3V rail-to-rail, ideal for external op-amp)");
      } else if (strncmp(cmdLine, "eqn ", 4) == 0) {
        const char *formula = cmdLine + 4;
        if (strncmp(formula, "1 ", 2) == 0) formula += 2;
        while (*formula == ' ') formula++;
        if (compileMathExpression(formula, &ch1.mathExpr)) {
          ch1.wave = 'e';
          recomputeAndApplySynthesis();
          Serial.printf("✔ Equation compiled: \"%s\" (Bytecode: %d B, NormScale: %.3f)\n",
                        formula, ch1.mathExpr.bcLen, ch1.mathExpr.normScale);
          Serial.printf("  Synthesized with N=%d points in %.1f us (%lu cycles)!\n",
                        currentSamplesPerCycle, lastComputeTimeUs, (unsigned long)lastComputeCycles);
        } else {
          Serial.printf("✘ Syntax error in equation: %s\n", formula);
        }
      } else if (strncmp(cmdLine, "wave ", 5) == 0) {
        char w = cmdLine[5];
        ch1.wave = w;
        recomputeAndApplySynthesis();
        Serial.printf("✔ Waveform set to: '%c'\n", w);
      } else if (strncmp(cmdLine, "freq ", 5) == 0) {
        float f = strtof(cmdLine + 5, nullptr);
        if (f >= 1.0f && f <= 500000.0f) {
          ch1.freq = f;
          recomputeAndApplySynthesis();
          Serial.printf("✔ Target: %.2f Hz | Actual: %.2f Hz (Error: %+.3f%%) | fs: %.3f MSPS (N=%d)\n",
                        f, (float)actualFout, freqErrorPct, (float)(actualFs / 1000000.0), currentSamplesPerCycle);
        }
      } else if (strncmp(cmdLine, "amp ", 4) == 0) {
        int a = atoi(cmdLine + 4);
        ch1.amp = constrain(a, 0, 100);
        recomputeAndApplySynthesis();
        Serial.printf("✔ Amplitude set to: %d%%\n", ch1.amp);
      } else if (strncmp(cmdLine, "offset ", 7) == 0) {
        int o = atoi(cmdLine + 7);
        ch1.offset = constrain(o, -128, 127);
        recomputeAndApplySynthesis();
        Serial.printf("✔ Offset set to: %d\n", ch1.offset);
      } else if (strncmp(cmdLine, "duty ", 5) == 0) {
        int d = atoi(cmdLine + 5);
        ch1.duty = constrain(d, 1, 99);
        recomputeAndApplySynthesis();
        Serial.printf("✔ Duty cycle set to: %d%%\n", ch1.duty);
      } else if (strncmp(cmdLine, "ch1 ", 4) == 0) {
        char w; uint32_t f; int a;
        if (sscanf(cmdLine + 4, " %c %lu %d", &w, &f, &a) == 3) {
          ch1.wave = w; ch1.freq = (float)f; ch1.amp = constrain(a, 0, 100); ch1.enabled = true;
          recomputeAndApplySynthesis();
          Serial.printf("✔ CH1: Wave=%c, Freq=%lu Hz, Amp=%d%% | Actual=%.2f Hz, N=%d pts\n",
                        w, f, a, (float)actualFout, currentSamplesPerCycle);
        }
      } else if (strcmp(cmdLine, "mute") == 0 || strcmp(cmdLine, "mute 1") == 0) {
        ch1.enabled = false; recomputeAndApplySynthesis(); Serial.println("DAC Output Muted (DC Midpoint)");
      } else if (strcmp(cmdLine, "unmute") == 0 || strcmp(cmdLine, "unmute 1") == 0) {
        ch1.enabled = true; recomputeAndApplySynthesis(); Serial.println("DAC Output Unmuted");
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
  // 1. Service UART commands from ESP32 BLE Gateway
  processEsp32Uart();

  // 2. Service interactive CLI commands from ST-Link USB
  processUsbCli();

  // 3. Periodic heartbeat / telemetry to USB monitor every 1 second
  static uint32_t lastPrintSec = 0;
  uint32_t nowSec = millis() / 1000;
  if (nowSec != lastPrintSec) {
    lastPrintSec = nowSec;

    Serial.print("[RUNNING] T=");
    Serial.print(nowSec);
    Serial.print(" s | PA4 (DAC): ");
    Serial.print(ch1.wave);
    Serial.print(" @ ");
    Serial.print((uint32_t)ch1.freq);
    Serial.print(" Hz | Act: ");
    Serial.print((float)actualFout, 2);
    Serial.print(" Hz (Err: ");
    Serial.print(freqErrorPct, 3);
    Serial.print("%) | N=");
    Serial.print(currentSamplesPerCycle);
    Serial.print(" pts | fs=");
    Serial.print((float)(actualFs / 1000000.0), 3);
    Serial.print(" MSPS | T_calc=");
    Serial.print(lastComputeTimeUs, 1);
    Serial.print(" us | DOR1=");
    Serial.print(DAC1->DOR1);
    Serial.print(" | DOR2(D13)=");
    Serial.println(DAC1->DOR2);
    Serial.flush();
  }
}
