#!/bin/bash
# Script para compilar y flashear el ESP32 en macOS / Linux
echo "⚡ Buscando puerto serie del ESP32..."
PORT=$(ls /dev/cu.usbserial* /dev/cu.wchusbserial* /dev/cu.SLAB_USBtoUART* 2>/dev/null | head -n 1)

if [ -z "$PORT" ]; then
    echo "❌ No se detectó automáticamente el ESP32. Puertos disponibles:"
    ls /dev/cu.*
    echo "Uso: ./upload_esp.sh /dev/cu.tu_puerto"
    if [ -n "$1" ]; then
        PORT="$1"
    else
        exit 1
    fi
fi

echo "🚀 Compilando y subiendo firmware a $PORT ..."
arduino-cli compile --fqbn esp32:esp32:esp32 ./esp32_firmware
arduino-cli upload -p "$PORT" --fqbn esp32:esp32:esp32 ./esp32_firmware
