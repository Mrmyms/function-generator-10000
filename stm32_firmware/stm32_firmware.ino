/*
 * ==================================================================================================
 * FUNCTION GENERATOR 10000 - DDS ARCHITECTURE (Fixed 1 MSPS + Wavetable + Interpolation)
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
 * DDS Architecture:
 *   1. Fixed 1.000 MSPS sampling rate (TIM6 ARR=249, PSC=0 from 250 MHz)
 *   2. 2048-point high-resolution wavetable (one perfect cycle, 0..2*PI)
 *   3. 32-bit phase accumulator with linear interpolation between samples
 *   4. Phase increment quantized so DMA buffer contains exact integer cycles
 *      -> Eliminates discontinuity at circular DMA wrap point
 *   5. Double-buffered: Inactive buffer synthesized, then memcpy'd while DMA runs
 *   6. GPDMA permanently circular, NEVER stopped or reset
 *   7. Frequency changes: only re-fill buffer + swap (instant, no glitch)
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
#define PIN_DAC_CH2        PA5          // DAC1_OUT2 (Arduino D13 / LED LD2)

// Serial1 connected to ESP32 Gateway (D0 PB15 / D1 PB14)
Uart Serial1((PinName)PB_15_ALT1, (PinName)PB_14);

// ===========================================================================
// DDS CORE PARAMETERS
// ===========================================================================

// Wavetable: 2048 points of one perfect cycle, pre-computed for each waveform
#define WAVETABLE_SIZE     2048
#define WAVETABLE_MASK     (WAVETABLE_SIZE - 1)
#define WAVETABLE_BITS     11           // log2(2048)

// DMA output buffer: 2048 32-bit samples, circular, double-buffered swap
#define DMA_BUFFER_SAMPLES 2048

// Phase accumulator: 32-bit unsigned, wraps at 2^32
// Upper 11 bits = wavetable index, next 21 bits = fractional interpolation
#define FIXED_SAMPLE_RATE  1000000UL    // 1.000 MSPS (exactly)
#define PHASE_FRAC_BITS    21           // 32 - WAVETABLE_BITS

// --- BUFFERS (cache-aligned) ---
static uint16_t wavetable[WAVETABLE_SIZE] __attribute__((aligned(32)));  // 12-bit DAC values [0..4095]

static uint32_t waveBufferA[DMA_BUFFER_SAMPLES] __attribute__((aligned(32)));
static uint32_t waveBufferB[DMA_BUFFER_SAMPLES] __attribute__((aligned(32)));
static volatile uint32_t *activeBuffer   = waveBufferA;
static volatile uint32_t *inactiveBuffer = waveBufferB;

// DDS state
static volatile uint32_t phaseIncrement  = 0;   // Phase step per DAC sample
static uint32_t          numCyclesInBuf  = 1;   // How many complete waveform cycles in DMA buffer

// Channel State
static ChannelState ch1 = { 's', 1000.0f, 100, 0, true, 50, {0}, {"sin(t)", {0}, 0, 1.0f, false} };

// Telemetry
static double   actualFout = 1000.0;
static float    freqErrorPct = 0.0f;
static bool     dacBufferEnabled = true;
static uint32_t lastComputeCycles = 0;
static float    lastComputeTimeUs = 0.0f;
static uint16_t currentSamplesPerCycle = 1000;
static uint16_t currentRepetitions = 1;
static const char *clockSourceStr = "Unknown";

static uint32_t totalPacketsReceived = 0;
static uint32_t totalPingsAnswered   = 0;
static uint32_t totalDmaUnderruns    = 0;
static uint32_t lastDmaRemainingCbr1 = 0;
static bool     dmaHardwareActive    = false;

// GPDMA Linked List structures
static DMA_HandleTypeDef hdma_dac;
static DMA_NodeTypeDef   dmaNode __attribute__((aligned(32)));
static DMA_QListTypeDef  dmaQueue;

// --- FAST 32-BIT XORSHIFT PRNG ---
static uint32_t rngState = 0x12345678;
static inline uint32_t xorshift32(void) {
  uint32_t x = rngState;
  x ^= x << 13;
  x ^= x >> 17;
  x ^= x << 5;
  rngState = x;
  return x;
}

// ===========================================================================
// HARDWARE HEALTH MONITOR
// ===========================================================================
uint8_t getSamplingHealthByte(void) {
  uint8_t health = 0;

  // Bit 1: Check DAC underrun flag
  if (DAC1->SR & DAC_SR_DMAUDR1) {
    DAC1->SR = DAC_SR_DMAUDR1;
    totalDmaUnderruns++;
  } else {
    health |= 0x02;
  }

  // Bit 0: Check GPDMA activity
  uint32_t currentCbr1 = GPDMA1_Channel0->CBR1;
  if (currentCbr1 != lastDmaRemainingCbr1) {
    dmaHardwareActive = true;
    lastDmaRemainingCbr1 = currentCbr1;
  }
  if (dmaHardwareActive) health |= 0x01;

  // Bit 2: TIM6 running
  if (LL_TIM_IsEnabledCounter(TIM6)) health |= 0x04;

  // Bit 3: Channel enabled
  if (ch1.enabled) health |= 0x08;

  // Bit 4: Valid amplitude
  if (ch1.amp > 0 && ch1.amp <= 100) health |= 0x10;

  return health;
}

// ===========================================================================
// CRC8 & PACKET FRAMING
// ===========================================================================
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

void sendPacket(Stream &s, uint8_t cmd, const uint8_t *payload, size_t len) {
  uint8_t header[4] = { 0xAA, 0x55, cmd, (uint8_t)len };
  uint8_t crc = calcCRC8(payload, len) ^ calcCRC8(header, 4);
  s.write(header, 4);
  if (len > 0 && payload != nullptr) s.write(payload, len);
  s.write(crc);
}

void sendPacketToESP(uint8_t cmd, const uint8_t *payload, size_t len) {
  sendPacket(Serial1, cmd, payload, len);
}

// ===========================================================================
// DAC BUFFER MODE
// ===========================================================================
void applyDacBufferMode(bool enableBuffer) {
  dacBufferEnabled = enableBuffer;
  uint32_t savedCr = DAC1->CR;
  DAC1->CR &= ~(DAC_CR_EN1);
  delayMicroseconds(2);
  if (enableBuffer) {
    DAC1->MCR &= ~(7U << DAC_MCR_MODE1_Pos);
  } else {
    DAC1->MCR = (DAC1->MCR & ~(7U << DAC_MCR_MODE1_Pos))
                | (2U << DAC_MCR_MODE1_Pos);
  }
  delayMicroseconds(2);
  DAC1->CR = savedCr;
}

// ===========================================================================
// WAVETABLE BUILDER - Compute one perfect waveform cycle (2048 points)
// ===========================================================================
void rebuildWavetable(void) {
  const float ampScale = (float)ch1.amp / 100.0f;
  const float dcOffset = ((float)ch1.offset / 128.0f) * 2047.0f;
  const float dutyFrac = (float)constrain((int)ch1.duty, 1, 99) / 100.0f;

  for (uint16_t i = 0; i < WAVETABLE_SIZE; i++) {
    float normPhase = (float)i / (float)WAVETABLE_SIZE;  // [0.0 .. 1.0)
    float rawSample = 0.0f;

    if (ch1.enabled && ch1.amp > 0) {
      switch (ch1.wave) {
        case 's': // Sine
          rawSample = sinf(normPhase * 2.0f * (float)M_PI);
          break;

        case 'q': // Square
        case 'p': // PWM
          rawSample = (normPhase < dutyFrac) ? 1.0f : -1.0f;
          break;

        case 't': // Triangle
          if (normPhase < 0.5f)
            rawSample = 4.0f * normPhase - 1.0f;
          else
            rawSample = 3.0f - 4.0f * normPhase;
          break;

        case 'a': // Sawtooth
          rawSample = 2.0f * normPhase - 1.0f;
          break;

        case 'n': // Software noise (randomized table)
          rawSample = ((float)(xorshift32() & 0xFFF) / 2047.5f) - 1.0f;
          break;

        case 'c': { // Custom AWG (64 points upsampled with interpolation)
          float awgPos = normPhase * 64.0f;
          int idx0 = (int)awgPos;
          if (idx0 >= 64) idx0 = 63;
          int idx1 = (idx0 + 1) & 63;
          float frac = awgPos - (float)idx0;
          float v0 = ((float)ch1.awgSamples[idx0] / 127.5f) - 1.0f;
          float v1 = ((float)ch1.awgSamples[idx1] / 127.5f) - 1.0f;
          rawSample = v0 + frac * (v1 - v0);
          break;
        }

        case 'e': { // Equation (Bytecode VM)
          if (ch1.mathExpr.isValid) {
            float t = normPhase * 2.0f * (float)M_PI;
            rawSample = evalMathBytecode(ch1.mathExpr.bytecode, normPhase, t) * ch1.mathExpr.normScale;
            if (isnan(rawSample) || isinf(rawSample)) rawSample = 0.0f;
          } else {
            rawSample = sinf(normPhase * 2.0f * (float)M_PI);
          }
          break;
        }

        default:
          rawSample = sinf(normPhase * 2.0f * (float)M_PI);
          break;
      }
    }

    // Scale, offset, clamp to 12-bit
    float dacOut = 2047.5f + (rawSample * 2047.0f * ampScale) + dcOffset;
    if (dacOut < 0.0f)    dacOut = 0.0f;
    if (dacOut > 4095.0f) dacOut = 4095.0f;
    wavetable[i] = (uint16_t)dacOut;
  }
}

// ===========================================================================
// DDS BUFFER FILL - Phase Accumulator + Linear Interpolation
// ===========================================================================
// Fills the DMA buffer with exactly numCycles complete waveform cycles,
// using the phase accumulator to walk the wavetable with sub-sample precision.
// Linear interpolation between adjacent wavetable entries ensures smooth output.

void fillDmaBufferDDS(uint32_t *dest, uint32_t numSamples, uint32_t phaseInc) {
  uint32_t phase = 0;  // Start at phase 0 (exact cycle alignment)

  const bool isNoise  = (ch1.wave == 'n');
  const bool isHwNoise = (ch1.wave == 'h');

  if (isHwNoise) {
    // Hardware LFSR noise: midpoint value, DAC hardware adds noise
    for (uint32_t i = 0; i < numSamples; i++) {
      dest[i] = (2048U << 16) | 2048U;
    }
    return;
  }

  if (isNoise) {
    // Software noise: real-time random (independent of wavetable)
    const float ampScale = (float)ch1.amp / 100.0f;
    const float dcOffset = ((float)ch1.offset / 128.0f) * 2047.0f;
    for (uint32_t i = 0; i < numSamples; i++) {
      float raw = ((float)(xorshift32() & 0xFFF) / 2047.5f) - 1.0f;
      float dacOut = 2047.5f + (raw * 2047.0f * ampScale) + dcOffset;
      if (dacOut < 0.0f)    dacOut = 0.0f;
      if (dacOut > 4095.0f) dacOut = 4095.0f;
      uint16_t val = (uint16_t)dacOut;
      dest[i] = ((uint32_t)val << 16) | (uint32_t)val;
    }
    return;
  }

  // DDS with linear interpolation from wavetable
  for (uint32_t i = 0; i < numSamples; i++) {
    // Upper 12 bits = wavetable index, next 20 bits = fractional part
    uint32_t idx0 = (phase >> PHASE_FRAC_BITS) & WAVETABLE_MASK;
    uint32_t idx1 = (idx0 + 1) & WAVETABLE_MASK;
    uint32_t frac = (phase >> (PHASE_FRAC_BITS - 12)) & 0xFFF;  // 12-bit fraction [0..4095]

    // Linear interpolation: y = y0 + (y1 - y0) * frac / 4096
    int32_t y0 = (int32_t)wavetable[idx0];
    int32_t y1 = (int32_t)wavetable[idx1];
    int32_t interpolated = y0 + (((y1 - y0) * (int32_t)frac) >> 12);

    // Clamp
    if (interpolated < 0)    interpolated = 0;
    if (interpolated > 4095) interpolated = 4095;

    uint16_t dacVal = (uint16_t)interpolated;
    dest[i] = ((uint32_t)dacVal << 16) | (uint32_t)dacVal;

    phase += phaseInc;
  }
}

// ===========================================================================
// DDS FREQUENCY SOLVER - Compute optimal phase increment for clean DMA loop
// ===========================================================================
// Key constraint: the DMA buffer (2048 samples) must contain an EXACT integer
// number of waveform cycles so that circular DMA wraps without discontinuity.
//
// numCycles = round(freq * DMA_BUFFER_SAMPLES / SAMPLE_RATE)
// phaseIncrement = (numCycles * 2^32) / DMA_BUFFER_SAMPLES
// actualFout = numCycles * SAMPLE_RATE / DMA_BUFFER_SAMPLES

void computePhaseIncrement(void) {
  double freq = (double)ch1.freq;
  if (freq < 0.1) freq = 0.1;
  if (freq > 500000.0) freq = 500000.0;

  // How many complete cycles fit in the DMA buffer?
  double rawCycles = freq * (double)DMA_BUFFER_SAMPLES / (double)FIXED_SAMPLE_RATE;
  uint32_t nCycles = (uint32_t)round(rawCycles);
  if (nCycles < 1) nCycles = 1;
  // Cap at half the buffer (Nyquist: at least 2 samples per cycle)
  if (nCycles > DMA_BUFFER_SAMPLES / 2) nCycles = DMA_BUFFER_SAMPLES / 2;

  numCyclesInBuf = nCycles;

  // Phase increment so that exactly nCycles complete in DMA_BUFFER_SAMPLES
  // phaseInc = nCycles * 2^32 / DMA_BUFFER_SAMPLES
  double incDouble = (double)nCycles * 4294967296.0 / (double)DMA_BUFFER_SAMPLES;
  phaseIncrement = (uint32_t)round(incDouble);

  // Actual output frequency
  actualFout = (double)nCycles * (double)FIXED_SAMPLE_RATE / (double)DMA_BUFFER_SAMPLES;
  freqErrorPct = (float)(fabs(actualFout - freq) / freq * 100.0);

  // Points per cycle
  currentSamplesPerCycle = (uint16_t)(DMA_BUFFER_SAMPLES / nCycles);
  currentRepetitions = (uint16_t)nCycles;
}

// ===========================================================================
// FULL SYNTHESIS PIPELINE: Wavetable + DDS + Double-Buffer Swap
// ===========================================================================
void recomputeAndApplySynthesis(void) {
  uint32_t tStart = DWT->CYCCNT;

  // 1. Rebuild wavetable for current waveform shape
  rebuildWavetable();

  // 2. Compute optimal phase increment
  computePhaseIncrement();

  // 3. Handle hardware noise mode
  if (ch1.wave == 'h') {
    DAC1->CR |= (DAC_CR_WAVE1_0 | (0x0B << DAC_CR_MAMP1_Pos));
  } else {
    DAC1->CR &= ~(DAC_CR_WAVE1_Msk | DAC_CR_MAMP1_Msk);
  }

  // 4. Fill inactive buffer using DDS phase accumulator + interpolation
  fillDmaBufferDDS((uint32_t *)inactiveBuffer, DMA_BUFFER_SAMPLES, phaseIncrement);

  // 5. Atomic swap: copy inactive -> active (DMA keeps running)
  memcpy((void *)activeBuffer, (const void *)inactiveBuffer, DMA_BUFFER_SAMPLES * sizeof(uint32_t));

  // 6. Benchmark
  uint32_t elapsed = DWT->CYCCNT - tStart;
  lastComputeCycles = elapsed;
  lastComputeTimeUs = (float)elapsed / 250.0f;  // 250 MHz
}

// ===========================================================================
// ASSERT HANDLER
// ===========================================================================
extern "C" void assert_failed(uint8_t* file, uint32_t line) {
  Serial.printf("\n*** HAL ASSERT: %s:%lu ***\n", (char*)file, line);
  while(1) { delay(1000); }
}

// ===========================================================================
// SETUP
// ===========================================================================
void setup() {
  Serial.begin(115200);
  delay(100);

  Serial.println("\n===========================================================");
  Serial.println("   FUNCTION GENERATOR 10000 - DDS ENGINE v6.0              ");
  Serial.println("   250 MHz Cortex-M33 | Fixed 1.000 MSPS | Phase Accum     ");
  Serial.println("   2048-Point Wavetable + Linear Interpolation              ");
  Serial.println("   Circular DMA (2048 samples) + Double-Buffer Swap        ");
  Serial.println("   Output: PA4 (DAC1_OUT1) -> CN7 Pin 32 (Arduino A2)      ");
  Serial.println("===========================================================");

  // 1. Serial1 for ESP32 Gateway
  Serial1.begin(115200);

  // 2. Enable DWT Cycle Counter for benchmarking
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

  // 3. Detect clock source
  uint32_t cfgr = RCC->CFGR1;
  uint32_t sws = (cfgr >> 3) & 0x07;
  switch (sws) {
    case 0: clockSourceStr = "HSI (64 MHz Internal)"; break;
    case 1: clockSourceStr = "CSI (4 MHz Fallback)"; break;
    case 2: clockSourceStr = "HSE (24 MHz Crystal)"; break;
    case 3: clockSourceStr = "PLL1 (250 MHz via HSE)"; break;
    default: clockSourceStr = "Unknown"; break;
  }
  Serial.print("Clock Source: "); Serial.println(clockSourceStr);
  Serial.print("SYSCLK: "); Serial.print((unsigned long)SystemCoreClock); Serial.println(" Hz");

  // 4. Compile default equation
  compileMathExpression("sin(t)", &ch1.mathExpr);

  // 5. Initialize AWG with sinc function
  for (int i = 0; i < 64; i++) {
    float x = ((float)i - 32.0f) * 0.3f;
    ch1.awgSamples[i] = (uint8_t)(128.0f + 120.0f * (x == 0.0f ? 1.0f : sinf(x) / x));
  }

  // 6. PA4 isolation
  GPIO_InitTypeDef gpio = {0};
  gpio.Pin  = GPIO_PIN_4;
  gpio.Mode = GPIO_MODE_ANALOG;
  gpio.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &gpio);

  // 7. Build wavetable + fill DMA buffer
  rebuildWavetable();
  computePhaseIncrement();
  fillDmaBufferDDS(waveBufferA, DMA_BUFFER_SAMPLES, phaseIncrement);
  memcpy(waveBufferB, waveBufferA, DMA_BUFFER_SAMPLES * sizeof(uint32_t));

  // 8. Enable peripheral clocks
  __HAL_RCC_DAC1_CLK_ENABLE();
  __HAL_RCC_TIM6_CLK_ENABLE();
  __HAL_RCC_GPDMA1_CLK_ENABLE();

  // 9. Configure GPDMA1 Channel 0: 2048-Word Circular Transfer
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
  nodeConf.SrcAddress = (uint32_t)waveBufferA;
  nodeConf.DstAddress = (uint32_t)&(DAC1->DHR12RD);
  nodeConf.DataSize   = DMA_BUFFER_SAMPLES * sizeof(uint32_t);  // 8192 bytes

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

  // 10. Configure DAC1
  applyDacBufferMode(dacBufferEnabled);
  uint32_t cr_ch1 = (DAC_CR_TSEL1_2 | DAC_CR_TSEL1_0) | DAC_CR_TEN1 | DAC_CR_DMAEN1 | DAC_CR_EN1;
  uint32_t cr_ch2 = (DAC_CR_TSEL2_2 | DAC_CR_TSEL2_0) | DAC_CR_TEN2 | DAC_CR_EN2;
  DAC1->CR = cr_ch1 | cr_ch2;
  delayMicroseconds(20);

  // 11. Start TIM6 at fixed 1.000 MSPS: 250 MHz / (249+1) = 1,000,000 Hz
  LL_TIM_DisableCounter(TIM6);
  LL_TIM_SetPrescaler(TIM6, 0);
  LL_TIM_SetAutoReload(TIM6, 249);
  LL_TIM_EnableARRPreload(TIM6);
  LL_TIM_SetTriggerOutput(TIM6, LL_TIM_TRGO_UPDATE);
  LL_TIM_EnableCounter(TIM6);

  Serial.println("\n>>> DDS ENGINE ACTIVE <<<");
  Serial.println(" - Sampling:    1.000 MSPS (Fixed TIM6 ARR=249)");
  Serial.println(" - Wavetable:   2048 points per cycle");
  Serial.println(" - DMA Buffer:  2048 samples (circular, never reset)");
  Serial.println(" - Interpolation: Linear (sub-sample precision)");
  Serial.println(" - Phase Accum: 32-bit (0.000233 Hz resolution)");
  Serial.println("Type 'help' for CLI commands.");
  Serial.println("===========================================================\n");
}

// ===========================================================================
// PROTOCOL PACKET EXECUTOR (COMMON TO BLE AND USB)
// ===========================================================================
void executeCommandPacket(Stream &replyStream, uint8_t rxCmd, uint8_t rxLen, uint8_t *rxBuf) {
  totalPacketsReceived++;

  // CMD 0x00: PING -> PONG (0x80)
  if (rxCmd == 0x00) {
    totalPingsAnswered++;
    uint8_t pongPayload[6];
    pongPayload[0] = (rxLen > 0) ? rxBuf[0] : 0;
    pongPayload[1] = (rxLen > 1) ? rxBuf[1] : 0;
    pongPayload[2] = 0x54; // Hardware ID
    pongPayload[3] = getSamplingHealthByte();
    pongPayload[4] = 0x06; // Firmware v6.0 (DDS)
    pongPayload[5] = dacBufferEnabled ? 0x01 : 0x00;
    sendPacket(replyStream, 0x80, pongPayload, sizeof(pongPayload));
  }
  // CMD 0x01: Channel Configuration (9+ bytes)
  else if (rxCmd == 0x01 && rxLen >= 9) {
    ChannelPacket *p = (ChannelPacket *)rxBuf;
    ch1.wave    = p->waveform;
    ch1.freq    = p->frequency;
    ch1.amp     = p->amplitude;
    ch1.offset  = p->offset;
    ch1.enabled = (p->enabled != 0);
    ch1.duty    = p->dutyCycle;
    recomputeAndApplySynthesis();

    // Rich ACK (11 bytes)
    uint8_t ackPayload[11];
    ackPayload[0] = 0x01;
    ackPayload[1] = getSamplingHealthByte();
    ackPayload[2] = (uint8_t)(totalDmaUnderruns & 0xFF);
    ackPayload[3] = (uint8_t)(currentSamplesPerCycle & 0xFF);
    ackPayload[4] = (uint8_t)((currentSamplesPerCycle >> 8) & 0xFF);
    uint32_t fActualInt = (uint32_t)round(actualFout);
    ackPayload[5] = (uint8_t)(fActualInt & 0xFF);
    ackPayload[6] = (uint8_t)((fActualInt >> 8) & 0xFF);
    ackPayload[7] = (uint8_t)((fActualInt >> 16) & 0xFF);
    ackPayload[8] = (uint8_t)((fActualInt >> 24) & 0xFF);
    ackPayload[9] = (uint8_t)ch1.wave;
    ackPayload[10] = dacBufferEnabled ? 0x01 : 0x00;
    sendPacket(replyStream, 0x81, ackPayload, sizeof(ackPayload));

    if (Serial) {
      Serial.print("[DDS] Config: ");
      Serial.print(ch1.wave);
      Serial.print(" @ ");
      Serial.print(ch1.freq);
      Serial.print(" Hz -> Act=");
      Serial.print((float)actualFout, 2);
      Serial.print(" Hz (");
      Serial.print(currentSamplesPerCycle);
      Serial.print(" pts x ");
      Serial.print(numCyclesInBuf);
      Serial.print(" cyc, Err=");
      Serial.print(freqErrorPct, 4);
      Serial.println("%)");
    }
  }
  // CMD 0x02: CH2 backward compat
  else if (rxCmd == 0x02 && rxLen >= sizeof(ChannelPacket)) {
    sendPacket(replyStream, 0x81, nullptr, 0);
  }
  // CMD 0x03: Phase shift compat
  else if (rxCmd == 0x03 && rxLen >= 4) {
    sendPacket(replyStream, 0x81, nullptr, 0);
  }
  // CMD 0x05: Equation Formula
  else if (rxCmd == 0x05 && rxLen > 0) {
    char eqStr[96];
    uint8_t copyLen = (rxLen < 95) ? rxLen : 95;
    memcpy(eqStr, rxBuf, copyLen);
    eqStr[copyLen] = '\0';
    if (compileMathExpression(eqStr, &ch1.mathExpr)) {
      ch1.wave = 'e';
      recomputeAndApplySynthesis();

      uint8_t ackPayload[11];
      ackPayload[0] = 0x05;
      ackPayload[1] = getSamplingHealthByte();
      ackPayload[2] = (uint8_t)(totalDmaUnderruns & 0xFF);
      ackPayload[3] = (uint8_t)(currentSamplesPerCycle & 0xFF);
      ackPayload[4] = (uint8_t)((currentSamplesPerCycle >> 8) & 0xFF);
      uint32_t fActualInt = (uint32_t)round(actualFout);
      ackPayload[5] = (uint8_t)(fActualInt & 0xFF);
      ackPayload[6] = (uint8_t)((fActualInt >> 8) & 0xFF);
      ackPayload[7] = (uint8_t)((fActualInt >> 16) & 0xFF);
      ackPayload[8] = (uint8_t)((fActualInt >> 24) & 0xFF);
      ackPayload[9] = 'e';
      ackPayload[10] = dacBufferEnabled ? 0x01 : 0x00;
      sendPacket(replyStream, 0x81, ackPayload, sizeof(ackPayload));

      if (Serial) {
        Serial.print("[DDS] Equation: ");
        Serial.println(eqStr);
      }
    } else {
      sendPacket(replyStream, 0x82, nullptr, 0);
      if (Serial) {
        Serial.print("[DDS] Bad equation: ");
        Serial.println(eqStr);
      }
    }
  }
  // CMD 0x06: CH2 equation compat
  else if (rxCmd == 0x06 && rxLen > 0) {
    sendPacket(replyStream, 0x81, nullptr, 0);
  }
  // CMD 0x07: DAC Buffer Mode
  else if (rxCmd == 0x07 && rxLen >= 1) {
    bool enableBuf = (rxBuf[0] != 0);
    applyDacBufferMode(enableBuf);
    uint8_t ackPayload[2] = { 0x07, (uint8_t)(enableBuf ? 1 : 0) };
    sendPacket(replyStream, 0x81, ackPayload, sizeof(ackPayload));
    if (Serial) {
      Serial.print("[DDS] DAC Buffer: ");
      Serial.println(enableBuf ? "ON (Mode 0)" : "OFF (Mode 2)");
    }
  }
  // CMD 0x10..0x13: AWG Chunks (17 bytes: ch + 16 data)
  else if (rxCmd >= 0x10 && rxCmd <= 0x13 && rxLen >= 17) {
    uint8_t chunkIdx = rxCmd - 0x10;
    memcpy(&ch1.awgSamples[chunkIdx * 16], &rxBuf[1], 16);
    uint8_t ackPayload[3] = { rxCmd, (uint8_t)chunkIdx, getSamplingHealthByte() };
    sendPacket(replyStream, 0x81, ackPayload, sizeof(ackPayload));
    if (chunkIdx == 3) {
      ch1.wave = 'c';
      recomputeAndApplySynthesis();
      if (Serial) {
        Serial.println("[DDS] AWG Complete (64pts -> 2048 wavetable)!");
      }
    }
  }
  else {
    uint8_t nackPayload[2] = { rxCmd, 0xFF };
    sendPacket(replyStream, 0x82, nackPayload, sizeof(nackPayload));
  }
}

// ===========================================================================
// PACKET PARSER (byte-by-byte framed protocol)
// ===========================================================================
bool processPacketByte(uint8_t b, Stream &replyStream, uint8_t &rxState, uint8_t &rxCmd, uint8_t &rxLen, uint8_t *rxBuf, uint8_t &rxIdx) {
  if (rxState == 0) {
    if (b == 0xAA) { rxState = 1; return true; }
    return false;
  } else if (rxState == 1) {
    if (b == 0x55) rxState = 2;
    else rxState = (b == 0xAA) ? 1 : 0;
    return true;
  } else if (rxState == 2) {
    rxCmd = b; rxState = 3; return true;
  } else if (rxState == 3) {
    rxLen = b; rxIdx = 0;
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
    } else {
      uint8_t nackPayload[3] = { rxCmd, 0xEE, expectedCrc };
      sendPacket(replyStream, 0x82, nackPayload, sizeof(nackPayload));
      if (Serial) {
        Serial.print("[UART] CRC Mismatch! Exp=0x");
        Serial.print(expectedCrc, HEX);
        Serial.print(" Calc=0x");
        Serial.println(calculatedCrc, HEX);
      }
    }
    return true;
  }
  return false;
}

// --- NON-BLOCKING UART PARSER FOR ESP32 PACKETS ---
void processEsp32Uart(void) {
  static uint8_t rxState = 0, rxCmd = 0, rxLen = 0, rxIdx = 0;
  static uint8_t rxBuf[128];
  static uint32_t lastByteTimeMs = 0;
  if (rxState > 0 && (millis() - lastByteTimeMs > 35)) {
    rxState = 0; rxIdx = 0;
  }
  while (Serial1.available()) {
    lastByteTimeMs = millis();
    processPacketByte(Serial1.read(), Serial1, rxState, rxCmd, rxLen, rxBuf, rxIdx);
  }
}

// --- USB SERIAL CLI & BINARY COMMAND PROCESSOR ---
void processUsbCli(void) {
  static uint8_t rxState = 0, rxCmd = 0, rxLen = 0, rxIdx = 0;
  static uint8_t rxBuf[128];
  static char cmdLine[128];
  static uint8_t cmdIdx = 0;
  static uint32_t lastUsbByteTimeMs = 0;

  if (rxState > 0 && (millis() - lastUsbByteTimeMs > 40)) {
    rxState = 0; rxIdx = 0;
  }

  while (Serial.available()) {
    lastUsbByteTimeMs = millis();
    uint8_t b = Serial.read();

    if (rxState > 0 || b == 0xAA) {
      processPacketByte(b, Serial, rxState, rxCmd, rxLen, rxBuf, rxIdx);
      continue;
    }

    char c = (char)b;
    if (c == '\r' || c == '\n') {
      if (cmdIdx == 0) continue;
      cmdLine[cmdIdx] = '\0';

      if (strcmp(cmdLine, "help") == 0) {
        Serial.println("\n--- Function Generator 10000 DDS CLI ---");
        Serial.println("  status               : Hardware telemetry");
        Serial.println("  wave <s/q/t/a/n/h/e> : Set waveform");
        Serial.println("  freq <Hz>            : Set frequency (0.1 - 500k Hz)");
        Serial.println("  amp <0-100>          : Set amplitude %");
        Serial.println("  offset <-128..127>   : Set DC offset");
        Serial.println("  duty <1-99>          : Set duty cycle %");
        Serial.println("  eqn <formula>        : Compile equation");
        Serial.println("  buf on / buf off     : Toggle DAC buffer");
        Serial.println("  mute / unmute        : Toggle output");
        Serial.println("----------------------------------------\n");
      } else if (strcmp(cmdLine, "status") == 0) {
        Serial.println("\n--- DDS Hardware Telemetry ---");
        Serial.print("  Clock:       "); Serial.println(clockSourceStr);
        Serial.print("  SYSCLK:      "); Serial.print((unsigned long)SystemCoreClock); Serial.println(" Hz");
        Serial.print("  HCLK:        "); Serial.print((unsigned long)HAL_RCC_GetHCLKFreq()); Serial.println(" Hz");
        Serial.print("  PCLK1:       "); Serial.print((unsigned long)HAL_RCC_GetPCLK1Freq()); Serial.println(" Hz");
        Serial.println("  Sampling:    1.000 MSPS (Fixed TIM6 ARR=249 PSC=0)");
        Serial.print("  DAC Buffer:  "); Serial.println(dacBufferEnabled ? "ON (Mode 0)" : "OFF (Mode 2)");
        Serial.print("  Target Freq: "); Serial.print(ch1.freq, 2); Serial.println(" Hz");
        Serial.print("  Actual Fout: "); Serial.print((float)actualFout, 4); Serial.print(" Hz (Err: "); Serial.print(freqErrorPct, 4); Serial.println("%)");
        Serial.print("  Phase Inc:   "); Serial.print(phaseIncrement); Serial.print(" (0x"); Serial.print(phaseIncrement, HEX); Serial.println(")");
        Serial.print("  Cycles/Buf:  "); Serial.print(numCyclesInBuf); Serial.print(" ("); Serial.print(currentSamplesPerCycle); Serial.println(" pts/cycle)");
        Serial.print("  Wavetable:   "); Serial.print(WAVETABLE_SIZE); Serial.println(" points");
        Serial.print("  DMA Buffer:  "); Serial.print(DMA_BUFFER_SAMPLES); Serial.println(" samples (circular, 16384 bytes)");
        Serial.print("  Compute:     "); Serial.print(lastComputeTimeUs, 2); Serial.print(" us ("); Serial.print((unsigned long)lastComputeCycles); Serial.println(" cycles)");
        Serial.print("  Waveform:    '"); Serial.print(ch1.wave); Serial.print("' | Amp="); Serial.print(ch1.amp); Serial.print("% | Offset="); Serial.print(ch1.offset); Serial.print(" | Duty="); Serial.print(ch1.duty); Serial.print("% | En="); Serial.println(ch1.enabled);
        if (ch1.wave == 'e') {
          Serial.print("  Equation:    \""); Serial.print(ch1.mathExpr.exprStr); Serial.print("\" (BC: "); Serial.print(ch1.mathExpr.bcLen); Serial.println(" B)");
        }
        Serial.print("  Packets:     "); Serial.print(totalPacketsReceived); Serial.print(" RX | "); Serial.print(totalPingsAnswered); Serial.print(" Pings | "); Serial.print(totalDmaUnderruns); Serial.println(" UDR");
        Serial.println("----------------------------------------------\n");
      } else if (strcmp(cmdLine, "buf on") == 0 || strcmp(cmdLine, "buffer on") == 0) {
        applyDacBufferMode(true);
        Serial.println("✔ DAC Buffer: ENABLED (Mode 0: ~0.2V-3.1V)");
      } else if (strcmp(cmdLine, "buf off") == 0 || strcmp(cmdLine, "buffer off") == 0) {
        applyDacBufferMode(false);
        Serial.println("✔ DAC Buffer: DISABLED (Mode 2: 0.0V-3.3V)");
      } else if (strncmp(cmdLine, "eqn ", 4) == 0) {
        const char *formula = cmdLine + 4;
        if (strncmp(formula, "1 ", 2) == 0) formula += 2;
        while (*formula == ' ') formula++;
        if (compileMathExpression(formula, &ch1.mathExpr)) {
          ch1.wave = 'e';
          recomputeAndApplySynthesis();
          Serial.print("✔ Equation: \""); Serial.print(formula); Serial.print("\" | "); Serial.print(lastComputeTimeUs, 1); Serial.println(" us");
        } else {
          Serial.print("✘ Syntax error: "); Serial.println(formula);
        }
      } else if (strncmp(cmdLine, "wave ", 5) == 0) {
        ch1.wave = cmdLine[5];
        recomputeAndApplySynthesis();
        Serial.print("✔ Waveform: '"); Serial.print(ch1.wave); Serial.println("'");
      } else if (strncmp(cmdLine, "freq ", 5) == 0) {
        float f = strtof(cmdLine + 5, nullptr);
        if (f >= 0.1f && f <= 500000.0f) {
          ch1.freq = f;
          recomputeAndApplySynthesis();
          Serial.print("✔ Freq: "); Serial.print(f, 2);
          Serial.print(" Hz -> Act: "); Serial.print((float)actualFout, 4);
          Serial.print(" Hz (Err: "); Serial.print(freqErrorPct, 4);
          Serial.print("%) | "); Serial.print(currentSamplesPerCycle);
          Serial.print(" pts x "); Serial.print(numCyclesInBuf);
          Serial.println(" cyc");
        }
      } else if (strncmp(cmdLine, "amp ", 4) == 0) {
        ch1.amp = constrain(atoi(cmdLine + 4), 0, 100);
        recomputeAndApplySynthesis();
        Serial.print("✔ Amplitude: "); Serial.print(ch1.amp); Serial.println("%");
      } else if (strncmp(cmdLine, "offset ", 7) == 0) {
        ch1.offset = constrain(atoi(cmdLine + 7), -128, 127);
        recomputeAndApplySynthesis();
        Serial.print("✔ Offset: "); Serial.println(ch1.offset);
      } else if (strncmp(cmdLine, "duty ", 5) == 0) {
        ch1.duty = constrain(atoi(cmdLine + 5), 1, 99);
        recomputeAndApplySynthesis();
        Serial.print("✔ Duty: "); Serial.print(ch1.duty); Serial.println("%");
      } else if (strncmp(cmdLine, "ch1 ", 4) == 0) {
        char w; uint32_t f; int a;
        if (sscanf(cmdLine + 4, " %c %lu %d", &w, &f, &a) == 3) {
          ch1.wave = w; ch1.freq = (float)f; ch1.amp = constrain(a, 0, 100); ch1.enabled = true;
          recomputeAndApplySynthesis();
          Serial.print("✔ CH1: "); Serial.print(w); Serial.print(" @ "); Serial.print(f);
          Serial.print(" Hz -> Act="); Serial.print((float)actualFout, 2); Serial.println(" Hz");
        }
      } else if (strcmp(cmdLine, "mute") == 0 || strcmp(cmdLine, "mute 1") == 0) {
        ch1.enabled = false; recomputeAndApplySynthesis(); Serial.println("Output Muted");
      } else if (strcmp(cmdLine, "unmute") == 0 || strcmp(cmdLine, "unmute 1") == 0) {
        ch1.enabled = true; recomputeAndApplySynthesis(); Serial.println("Output Unmuted");
      } else {
        Serial.println("Unknown command. Type 'help'.");
      }

      cmdIdx = 0;
    } else {
      if (cmdIdx < sizeof(cmdLine) - 1) {
        cmdLine[cmdIdx++] = c;
      }
    }
  }
}

// ===========================================================================
// MAIN LOOP
// ===========================================================================
void loop() {
  // 1. Service ESP32 BLE Gateway
  processEsp32Uart();

  // 2. Service USB CLI
  processUsbCli();

  // 3. Heartbeat every 1 second
  static uint32_t lastPrintSec = 0;
  uint32_t nowSec = millis() / 1000;
  if (nowSec != lastPrintSec) {
    lastPrintSec = nowSec;
    Serial.print("[DDS] T=");
    Serial.print(nowSec);
    Serial.print("s | ");
    Serial.print(ch1.wave);
    Serial.print(" @ ");
    Serial.print((uint32_t)ch1.freq);
    Serial.print(" Hz | Act: ");
    Serial.print((float)actualFout, 2);
    Serial.print(" Hz | 1.000 MSPS | ");
    Serial.print(numCyclesInBuf);
    Serial.print("cyc x ");
    Serial.print(currentSamplesPerCycle);
    Serial.print("pts | H:0x");
    Serial.print(getSamplingHealthByte(), HEX);
    Serial.print(" | UDR:");
    Serial.print(totalDmaUnderruns);
    Serial.print(" | DOR1=");
    Serial.print(DAC1->DOR1);
    Serial.print(" | D13=");
    Serial.println(DAC1->DOR2);
  }
}
