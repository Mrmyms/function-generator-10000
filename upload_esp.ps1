$esptool = "C:\Users\myms1\AppData\Local\Arduino15\packages\esp32\tools\esptool_py\5.3.1\esptool.exe"
$binFile = "c:\proyectos\function-generator-10000\esp32_firmware\build\esp32_firmware.ino.merged.bin"
$port = "COM8"

Write-Host "==========================================================" -ForegroundColor Cyan
Write-Host " Flasheo ESP32 con Deteccion Automatica" -ForegroundColor Cyan
Write-Host " Realiza esta combinacion en tu ESP32:" -ForegroundColor Yellow
Write-Host " 1. Manten presionado el boton BOOT" -ForegroundColor Yellow
Write-Host " 2. Pulsa y suelta el boton EN (o RST)" -ForegroundColor Yellow
Write-Host " 3. Suelta el boton BOOT" -ForegroundColor Yellow
Write-Host "==========================================================" -ForegroundColor Cyan

$success = $false
for ($attempt = 1; $attempt -le 20; $attempt++) {
    Write-Host "[-] Intento $attempt de 20: buscando modo descarga en $port..." -ForegroundColor Gray
    
    # Intentar flashear directamente
    & $esptool --chip esp32 --port $port --baud 921600 --before default_reset --after hard_reset write_flash 0x0 $binFile
    if ($LASTEXITCODE -eq 0) {
        $success = $true
        break
    }
    
    # Probar con no_reset si ya entro por botones
    & $esptool --chip esp32 --port $port --baud 921600 --before no_reset --after hard_reset write_flash 0x0 $binFile
    if ($LASTEXITCODE -eq 0) {
        $success = $true
        break
    }

    Start-Sleep -Milliseconds 600
}

if ($success) {
    Write-Host "`n==========================================================" -ForegroundColor Green
    Write-Host " [EXITO] ¡ESP32 flasheado con exito!" -ForegroundColor Green
    Write-Host " Firmware con control de LED listo y corriendo." -ForegroundColor Green
    Write-Host "==========================================================" -ForegroundColor Green
} else {
    Write-Host "`n[ERROR] No se detecto el ESP32 en modo descarga tras varios intentos." -ForegroundColor Red
}
