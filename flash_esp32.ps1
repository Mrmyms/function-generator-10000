$ProgressPreference = 'SilentlyContinue'
$ErrorActionPreference = 'Stop'

Write-Host "==========================================================" -ForegroundColor Cyan
Write-Host " Flasher para ESP32 - Function Generator 10000" -ForegroundColor Cyan
Write-Host "==========================================================" -ForegroundColor Cyan

# 1. Buscar puertos COM físicos USB (excluyendo ST-Link de STM32 y puertos virtuales Bluetooth)
$ports = Get-PnpDevice -Class Ports -Status OK -ErrorAction SilentlyContinue | Where-Object { 
    $_.FriendlyName -notmatch 'STLink' -and 
    $_.FriendlyName -notmatch 'Bluetooth' -and
    $_.InstanceId -match '^USB'
}

# Si no hay puerto activo, verificar si hay un dispositivo desconectado o fantasma
if (-not $ports) {
    Write-Host "[!] No se detectó un puerto USB serie activo para el ESP32." -ForegroundColor Yellow
    
    $phantom = Get-PnpDevice -Class Ports -ErrorAction SilentlyContinue | Where-Object { 
        $_.FriendlyName -match 'CH340|CP210|FTDI|Silicon Labs|USB-SERIAL'
    }
    
    if ($phantom) {
        Write-Host "    Se detectó un dispositivo previo desconectado: $($phantom[0].FriendlyName)" -ForegroundColor Yellow
        Write-Host "    --> Conecta el cable USB del ESP32 al PC y vuelve a ejecutar este script." -ForegroundColor Cyan
    } else {
        Write-Host "    Conecta tu ESP32 mediante cable USB al PC." -ForegroundColor Yellow
    }

    Write-Host "`nPuertos detectados actualmente en el sistema:" -ForegroundColor Gray
    Get-PnpDevice -Class Ports -Status OK | Select-Object FriendlyName, InstanceId | Format-Table -AutoSize
    exit 1
}

$comPort = $null
foreach ($p in $ports) {
    if ($p.FriendlyName -match '\((COM\d+)\)') {
        $comPort = $matches[1]
        Write-Host "[+] ESP32 detectado en: $comPort ($($p.FriendlyName))" -ForegroundColor Green
        break
    }
}

if (-not $comPort) {
    Write-Host "[!] No se pudo extraer el puerto COM." -ForegroundColor Red
    exit 1
}

$cli = 'C:\Users\myms1\.gemini\antigravity-ide\scratch\bin\arduino-cli.exe'
$sketchDir = 'C:\proyectos\function-generator-10000\esp32_firmware'

Write-Host "[-] Flasheando firmware al ESP32 en $comPort..." -ForegroundColor Yellow
& $cli upload -p $comPort --fqbn esp32:esp32:esp32 $sketchDir

if ($LASTEXITCODE -eq 0) {
    Write-Host "`n==========================================================" -ForegroundColor Green
    Write-Host " [EXITO] ¡ESP32 flasheado exitosamente!" -ForegroundColor Green
    Write-Host " Servidor Web Bluetooth 'FuncGen-10000' listo y activo." -ForegroundColor Cyan
    Write-Host " Enlace UART2 activo con STM32 (GPIO 16 RX / GPIO 17 TX)." -ForegroundColor Cyan
    Write-Host "==========================================================" -ForegroundColor Green
} else {
    Write-Host "`n[ERROR] Falló la carga. Si tu ESP32 tiene botón BOOT, mantenlo presionado durante la conexión." -ForegroundColor Red
}
