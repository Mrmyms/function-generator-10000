# ⚡ Function Generator 10000

Generador de Funciones de Laboratorio de **Doble Canal a 1 MSPS (1,000,000 muestras/segundo)** con DACs reales de 12 bits, controlado inalámbricamente mediante **Web Bluetooth API** desde una App Web moderna desplegable en **Vercel**.

---

## 🏗️ Arquitectura del Sistema

```
+-------------------------------------------------------------+
|                     WEB APP (VERCEL)                        |
|  - Alojada en Vercel (HTTPS Nativo Obligatorio)             |
|  - Web Bluetooth API (Google Chrome / Edge / Opera)         |
|  - Selector de Formas de Onda, Sweep, PWM, Ráfagas y AWG    |
+-------------------------------------------------------------+
                              |
                     [ BLE Inalámbrico ]
                              |
                              v
+-------------------------------------------------------------+
|                     ESP32 WROOM (GATEWAY)                   |
|  - Servidor BLE GATT ("FuncGen-10000")                      |
|  - Recepción de paquetes de comando y telemetría            |
|  - Puente UART Hardware de alta velocidad a 115,200 baud    |
+-------------------------------------------------------------+
                              |
               [ UART Serial 3.3V (3 cables) ]
                              |
                              v
+-------------------------------------------------------------+
|                STM32 MB1814 (NUCLEO-H503RB / H533RE)        |
|  - Núcleo ARM Cortex-M33 @ 250 MHz                          |
|  - Hardware Timer (TIM6) disparando a 1 MHz (1 µs)          |
|  - Salida Dual DAC de 12 bits a 1 MSPS:                     |
|      * CANAL 1: Pin PA4 (DAC1_CH1)                          |
|      * CANAL 2: Pin PA5 (DAC1_CH2)                          |
+-------------------------------------------------------------+
```

---

## 🔌 Conexión Física (Cableado)

### 1. Entre ESP32 WROOM y STM32 MB1814 (Solo 3 cables)

| ESP32 WROOM | STM32 MB1814 (Nucleo) | Descripción |
| :--- | :--- | :--- |
| **TX2 (GPIO 17)** | **PA10 (Pin D0)** | Comandos de ESP32 hacia STM32 |
| **RX2 (GPIO 16)** | **PA9 (Pin D1)** | Respuestas / Telemetría hacia ESP32 |
| **GND** | **GND** | Tierra de referencia común |

*(Nota: Ambos microcontroladores operan a nivel lógico de **3.3V**, por lo que se conectan de forma directa sin conversores de nivel).*

### 2. Salidas Analógicas del Generador (STM32)

* **Canal 1 (0 a 3.3V)**: Pin **`PA4`** (`DAC1_CH1` de 12 bits)
* **Canal 2 (0 a 3.3V)**: Pin **`PA5`** (`DAC1_CH2` de 12 bits)
* **GND**: Conectar a la tierra de la sonda / osciloscopio

---

## 📂 Estructura del Proyecto

```
Function generator 10000/
├── esp32_firmware/
│   └── esp32_firmware.ino       # Firmware ESP32 (BLE GATT Server + UART Bridge)
├── stm32_firmware/
│   └── stm32_firmware.ino       # Firmware STM32 (Timer 1MHz + Dual DAC 12-bit 1MSPS)
├── web_app/
│   ├── index.html               # App Web Frontend (Web Bluetooth, Live Visualizer, AWG)
│   ├── package.json             # Manifiesto para despliegue en Vercel
│   └── vercel.json              # Configuración de URLs y headers para Vercel
└── README.md                    # Documentación del proyecto
```

---

## 🚀 Puesta en Marcha

### 1. Cargar el Firmware al ESP32 WROOM
Con el ESP32 conectado al puerto USB (`/dev/cu.usbserial-110`):

```bash
arduino-cli compile --fqbn esp32:esp32:esp32 "esp32_firmware"
arduino-cli upload -p /dev/cu.usbserial-110 --fqbn esp32:esp32:esp32 "esp32_firmware"
```

### 2. Cargar el Firmware al STM32 MB1814
Puedes compilar y cargar mediante:
* **STM32duino (Arduino IDE)**: Selecciona la placa *Nucleo-64* -> *Nucleo H503RB* o *H533RE*.
* **STM32CubeIDE**: Importa el código de `stm32_firmware/stm32_firmware.ino`.
* **Drag-and-Drop**: Si tu Nucleo monta una unidad flash ST-Link, puedes compilar a binario `.bin` y arrastrarlo.

### 3. Desplegar la App Web en Vercel
La carpeta `web_app/` está lista para subirse a Vercel con un solo comando:

```bash
cd web_app
npx vercel
```
O simplemente sube esta carpeta a un repositorio de GitHub y conéctalo en tu panel de Vercel.

---

## 📡 Prueba de Intercomunicación (Ping-Pong)

1. Conecta los 3 cables (TX, RX y GND) entre el ESP32 y el STM32.
2. Abre el Monitor Serial del ESP32 a 115,200 baudios:
   * Verás al ESP32 enviando pings periódicos: `[ESP32 -> STM] Sent CMD 0x00`.
   * El STM32 responderá inmediatamente alternando su LED de usuario:
     `⚡ [STM32 -> ESP32] PONG! Link Active! Latency: 0.85 ms`.
3. Abre la App Web en Chrome o Edge, pulsa **"Connect BLE"**, selecciona **"FuncGen-10000"** y tendrás control total inalámbrico.
