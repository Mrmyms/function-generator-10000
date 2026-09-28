#!/bin/bash
# Script para compilar y flashear el STM32 NUCLEO-H503RB en macOS / Linux
echo "[INFO] Buscando NUCLEO-H503RB..."

# Opción A: Por puerto serie ST-LINK
PORT=$(ls /dev/cu.usbmodem* 2>/dev/null | head -n 1)

# Opción B: Si está montado como unidad USB de almacenamiento
VOL="/Volumes/NOD_H503RB"

echo "[INFO] Compilando firmware STM32 (250 MHz Cortex-M33)..."
arduino-cli compile --fqbn STMicroelectronics:stm32:Nucleo_64:pnum=NUCLEO_H503RB --output-dir ./stm32_firmware/build ./stm32_firmware

CLI="/Applications/STMicroelectronics/STM32Cube/STM32CubeProgrammer/STM32CubeProgrammer.app/Contents/Resources/bin/STM32_Programmer_CLI"

if [ -x "$CLI" ]; then
    echo "[INFO] Flasheando directamente por SWD con STM32CubeProgrammer..."
    if "$CLI" -c port=SWD -w ./stm32_firmware/build/stm32_firmware.ino.bin 0x08000000 -v -rst; then
        echo "[OK] Firmware STM32 flasheado y verificado con exito via SWD!"
        exit 0
    fi
fi

if [ -d "$VOL" ]; then
    echo "[INFO] Disco Nucleo detectado en $VOL. Copiando binario por Drag-and-Drop..."
    cat ./stm32_firmware/build/*.bin > "$VOL/firmware.bin" 2>/dev/null || true
    sync 2>/dev/null || true
    echo "[OK] Firmware STM32 flasheado con exito via almacenamiento USB!"
elif [ -n "$PORT" ]; then
    echo "[INFO] Subiendo por ST-LINK en $PORT ..."
    export PATH="/Applications/STMicroelectronics/STM32Cube/STM32CubeProgrammer/STM32CubeProgrammer.app/Contents/Resources/bin:$PATH"
    if arduino-cli upload -p "$PORT" --fqbn STMicroelectronics:stm32:Nucleo_64:pnum=NUCLEO_H503RB,upload_method=swdMethod ./stm32_firmware; then
        echo "[OK] Firmware STM32 subido con exito!"
    else
        echo "[ERROR] Error al subir por ST-LINK."
        exit 1
    fi
else
    echo "[WARN] Conecta la placa Nucleo por USB. Puertos encontrados:"
    ls /dev/cu.*
    exit 1
fi
