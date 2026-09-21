#ifndef WAVEFORM_SYNTH_H
#define WAVEFORM_SYNTH_H

#include <stdint.h>
#include <stdbool.h>
#include "math_expr.h"

// Maximum table size for DAC DMA circular synthesis: 2048 points for ultra-pure fidelity!
#define MAX_TABLE_SIZE 2048

// --- PROTOCOL PACKETS (MATCHING ESP32 & WEB APP) ---
struct __attribute__((packed)) ChannelPacket {
  char     waveform;      // 's'=Sine, 'q'=Square, 't'=Triangle, 'a'=Saw, 'p'=PWM, 'n'=Noise, 'c'=AWG, 'e'=Equation
  float    frequency;     // Target frequency in Hz (1.0 to 500,000.0 Hz)
  uint8_t  amplitude;     // 0 to 100%
  int8_t   offset;        // -128 to 127 (-1.65V to +1.65V)
  uint8_t  enabled;       // 1 = Active, 0 = Muted
  uint8_t  dutyCycle;     // 1 to 99%
  float    sweepTarget;   // Reserved
  float    sweepTimeMs;   // Reserved
  uint8_t  modType;       // Reserved
  float    modFreq;       // Reserved
  uint8_t  modDepth;      // Reserved
  uint8_t  burstEnabled;  // Reserved
  uint32_t burstCycles;   // Reserved
  uint8_t  trigger;       // Reserved
};

// Internal Channel State
struct ChannelState {
  char           wave;           // 's', 'q', 't', 'a', 'p', 'n', 'c', 'e'
  float          freq;
  uint8_t        amp;
  int8_t         offset;
  bool           enabled;
  uint8_t        duty;
  uint8_t        awgSamples[64];
  MathExpression mathExpr;
};

#endif // WAVEFORM_SYNTH_H
