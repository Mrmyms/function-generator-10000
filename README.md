# ⚡ FUNCTION GENERATOR 10000 (FG-10000)

> **Commercial Laboratory-Grade Arbitrary Function & Signal Generator**  
> Powered by **STM32 ARM Cortex-M33 @ 250 MHz** (GPDMA 12-bit DAC Engine up to **2.049 MSPS**) + **ESP32 BLE GATT Gateway** + **Nothing OS Aesthetic Web App**.

---

## 🚀 Overview

The **Function Generator 10000** is an open-architecture, high-precision arbitrary waveform generator designed for laboratory experiments, audio engineering, RF synthesis, and hardware debugging. It couples an ultra-fast hardware direct memory access (GPDMA) DAC core on an STM32 with a wireless Bluetooth Low Energy (BLE) gateway and a futuristic web interface inspired by the Nothing OS design language.

### Key Capabilities
- **Sampling Rate:** Up to **2.049 MSPS** (2048 samples/cycle @ 1.000 kHz, 1024 @ 2 kHz, down to 32 @ 64 kHz).
- **True Dual-Mirror 12-Bit Analog Output:**
  - **Arduino Header Pin D13 (`PA5` / `DAC1_OUT2`)**: Direct breadboard/scope probing with adjacent GND.
  - **Morpho Header Pin `PA4` (`DAC1_OUT1`)**: High-bandwidth direct analog tap.
- **Microcontroller Core:** STM32 NUCLEO-H503RB (ARM Cortex-M33 running at **250 MHz SYSCLK**).
- **Wireless Link:** ESP32-WROOM BLE GATT server (256-byte MTU) with binary UART packet bridge.
- **Web App Interface (Nothing OS):**
  - Dot-matrix typography (`NDOT 57`, `Space Grotesk`).
  - CRT Phosphor Oscilloscope Visualizer with beam persistence.
  - Nothing Glyph Matrix sound-reactive ambient indicator.
  - Interactive Mathematical Expression Evaluator (`sin`, `cos`, `exp`, `tan`, `abs`, `noise(f)`).
  - 64-Point Touch & Mouse AWG Waveform Painter.
  - Frequency Sweeps (Linear & Logarithmic) and Burst Waveform Generation.

---

## 🏗️ System Architecture

```
+-------------------------------------------------------------+
|               NOTHING OS WEB APP (CHROME / EDGE)            |
|  - Web Bluetooth API (GATT Service: 4fafc201-...)           |
|  - Real-Time CRT Oscilloscope Simulator                     |
|  - Expression Parser VM (Real-time Math -> Waveform Table)  |
|  - Interactive 64-Point Canvas AWG Painter                  |
+-------------------------------------------------------------+
                              │
                     [ BLE 5.0 Wireless Link ]
                              │
                              ▼
+-------------------------------------------------------------+
|                     ESP32 WROOM-32 GATEWAY                  |
|  - BLE GATT Server ("FuncGen-10000")                        |
|  - MTU 256 Auto-Negotiation & Reassembly Buffer             |
|  - Hardware UART2 Bridge @ 115,200 Baud                     |
|    * GPIO 16 (RX2) <--- STM32 D1 (PB14 TX)                  |
|    * GPIO 17 (TX2) ---> STM32 D0 (PB15 RX)                  |
+-------------------------------------------------------------+
                              │
               [ High-Speed Binary UART Protocol ]
                              │
                              ▼
+-------------------------------------------------------------+
|                STM32 NUCLEO-H503RB (CORTEX-M33)             |
|  - 250 MHz System Clock, Hardware FPU                       |
|  - TIM6 Hardware Trigger Auto-Reload Register               |
|  - GPDMA1 Channel 0 Circular Buffer (Zero CPU overhead)     |
|  - Simultaneous Dual 12-Bit DAC Write (DAC1->DHR12RD):      |
|    * Pin D13 / PA5 (DAC1_OUT2) -> Probe Out & User LED      |
|    * Pin PA4 (DAC1_OUT1)       -> High-Speed Analog Out     |
+-------------------------------------------------------------+
```

---

## 🔌 Hardware Pinout & Wiring

### 1. Inter-Board Communication (ESP32 <-> STM32)

| ESP32 Pin | STM32 NUCLEO-H503RB Pin | Function |
| :--- | :--- | :--- |
| **GPIO 17 (TX2)** | **Pin D0 (`PB15` RX)** | Commands from ESP32 to STM32 |
| **GPIO 16 (RX2)** | **Pin D1 (`PB14` TX)** | Telemetry & ACKs to ESP32 |
| **GND** | **GND** | Common reference ground |

*(Both microcontrollers operate natively at 3.3V logic levels. No level shifters required).*

### 2. Analog Outputs (Oscilloscope Probes)

| Output | Physical Board Location | Signal Characteristics |
| :--- | :--- | :--- |
| **Primary DAC Out** | **Arduino Pin `D13`** (`PA5`) | 0.0V to 3.3V full-scale, buffered |
| **Auxiliary DAC Out**| **Morpho Header Pin `PA4`** | 0.0V to 3.3V full-scale, buffered |
| **Ground Reference** | **Adjacent GND Pin** | Probe alligator clip ground |

---

## 📁 Repository Structure

```
function-generator-10000/
├── esp32_firmware/
│   └── esp32_firmware.ino       # ESP32 BLE GATT server & UART transceiver
├── stm32_firmware/
│   └── stm32_firmware.ino       # STM32 250 MHz GPDMA DAC waveform engine
├── web_app/
│   ├── index.html               # Nothing OS single-page web application
│   ├── package.json             # Deployment metadata
│   └── vercel.json              # Vercel deployment configuration
├── flash_esp32.ps1              # Flash script for ESP32 DevKit
├── upload_esp.ps1               # Automated ESP32 compilation & flash utility
├── start_web_server.ps1         # Local HTTP dev server launcher
├── test_binary_packet.ps1       # Protocol packet generator & validator
└── README.md                    # System documentation
```

---

## 💻 Quick Start Guide

### 1. Flashing the ESP32
```powershell
powershell -ExecutionPolicy Bypass -File .\upload_esp.ps1
```

### 2. Flashing the STM32 NUCLEO-H503RB
Using Arduino IDE or `arduino-cli`:
```powershell
arduino-cli compile --fqbn STMicroelectronics:stm32:Nucleo_64:pnum=NUCLEO_H503RB .\stm32_firmware
arduino-cli upload -p COM7 --fqbn STMicroelectronics:stm32:Nucleo_64:pnum=NUCLEO_H503RB .\stm32_firmware
```
*(Or drop the generated `.bin` file directly onto the `NOD_H503RB` USB drive).*

### 3. Running the Web App
Start the local server or deploy directly to Vercel:
```powershell
powershell -ExecutionPolicy Bypass -File .\start_web_server.ps1
```
Open `http://localhost:8088/` in Google Chrome or Microsoft Edge, click **"CONNECT GENERATOR"**, pair with **`FuncGen-10000`**, and probe **Pin D13** on your oscilloscope!

---

## 📜 License
MIT License. Built for precision audio, RF synthesis, and modern lab automation.
