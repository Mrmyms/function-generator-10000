#!/bin/bash
# Script para compilar y flashear el STM32 NUCLEO-H503RB en macOS / Linux
echo "⚡ Buscando NUCLEO-H503RB..."

# Opción A: Por puerto serie ST-LINK
PORT=$(ls /dev/cu.usbmodem* 2>/dev/null | head -n 1)

# Opción B: Si está montado como unidad USB de almacenamiento
VOL="/Volumes/NOD_H503RB"

echo "🔨 Compilando firmware STM32 (250 MHz Cortex-M33)..."
arduino-cli compile --fqbn STMicroelectronics:stm32:Nucleo_64:pnum=NUCLEO_H503RB --output-dir ./stm32_firmware/build ./stm32_firmware

if [ -d "$VOL" ]; then
    echo "💾 Disco Nucleo detectado en $VOL. Copiando binario por Drag-and-Drop..."
    cp ./stm32_firmware/build/*.bin "$VOL/"
    echo "✅ Firmware flasheado con éxito vía almacenamiento USB!"
elif [ -n "$PORT" ]; then
    echo "🚀 Subiendo por ST-LINK en $PORT ..."
    arduino-cli upload -p "$PORT" --fqbn STMicroelectronics:stm32:Nucleo_64:pnum=NUCLEO_H503RB ./stm32_firmware
    echo "✅ Firmware subido con éxito!"
else
    echo "⚠️ Conecta la placa Nucleo por USB. Puertos encontrados:"
    ls /dev/cu.*
fi
