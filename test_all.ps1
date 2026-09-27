Write-Host "==========================================================" -ForegroundColor Cyan
Write-Host " PRUEBA INTEGRAL: FUNCTION GENERATOR 10000" -ForegroundColor Cyan
Write-Host "==========================================================" -ForegroundColor Cyan

# 1. Test ESP32 on COM10 & UART Link to STM32
Write-Host "`n[1/2] Probando ESP32 en COM10 y enlace UART con STM32..." -ForegroundColor Yellow
$p10 = New-Object System.IO.Ports.SerialPort "COM10", 115200, "None", 8, "One"
$p10.ReadTimeout = 2000
$p10.DtrEnable = $true
$p10.RtsEnable = $true

try {
    $p10.Open()
    Write-Host "  -> COM10 abierto correctamente. Esperando inicio de sesion..." -ForegroundColor Green
    Start-Sleep -Milliseconds 1500
    $null = $p10.ReadExisting()

    Write-Host "  -> Consultando estado inicial del enlace ESP32 <-> STM32..." -ForegroundColor Green
    $p10.WriteLine("status")
    Start-Sleep -Milliseconds 600
    $st = $p10.ReadExisting()
    Write-Host $st -ForegroundColor Cyan

    Write-Host "  -> Enviando comando 'set freq 5000' (Cambio a 5000 Hz)..." -ForegroundColor Yellow
    $p10.WriteLine("set freq 5000")
    Start-Sleep -Milliseconds 500
    $ack1 = $p10.ReadExisting()
    Write-Host $ack1 -ForegroundColor Green

    Write-Host "  -> Enviando comando 'set wave q' (Cambio a Onda Cuadrada)..." -ForegroundColor Yellow
    $p10.WriteLine("set wave q")
    Start-Sleep -Milliseconds 500
    $ack2 = $p10.ReadExisting()
    Write-Host $ack2 -ForegroundColor Green

    Write-Host "  -> Enviando comando 'set buf on' (Habilitar DAC Buffer Modo 0)..." -ForegroundColor Yellow
    $p10.WriteLine("set buf on")
    Start-Sleep -Milliseconds 500
    $ack3 = $p10.ReadExisting()
    Write-Host $ack3 -ForegroundColor Green

    Write-Host "  -> Consultando estado final (Verificando conteo de ACKs del STM32)..." -ForegroundColor Green
    $p10.WriteLine("status")
    Start-Sleep -Milliseconds 600
    $st2 = $p10.ReadExisting()
    Write-Host $st2 -ForegroundColor Cyan

} catch {
    Write-Host "  -> Error en COM10: $_" -ForegroundColor Red
} finally {
    if ($p10.IsOpen) {
        $p10.Close()
        Write-Host "  -> COM10 cerrado." -ForegroundColor Green
    }
}

# 2. Test STM32 on COM7 Live Telemetry
Write-Host "`n[2/2] Leyendo telemetria en vivo del STM32 NUCLEO-H503RB en COM7..." -ForegroundColor Yellow
$p7 = New-Object System.IO.Ports.SerialPort "COM7", 115200, "None", 8, "One"
$p7.ReadTimeout = 2000
$p7.DtrEnable = $true
$p7.RtsEnable = $true

try {
    $p7.Open()
    Write-Host "  -> COM7 abierto correctamente. Capturando 4 segundos de salida..." -ForegroundColor Green
    $endTime = (Get-Date).AddSeconds(4)
    while ((Get-Date) -lt $endTime) {
        Start-Sleep -Milliseconds 250
        $data = $p7.ReadExisting()
        if ($data.Length -gt 0) {
            Write-Host -NoNewline $data -ForegroundColor Gray
        }
    }
} catch {
    Write-Host "  -> Error en COM7: $_" -ForegroundColor Red
} finally {
    if ($p7.IsOpen) {
        $p7.Close()
        Write-Host "`n  -> COM7 cerrado." -ForegroundColor Green
    }
}

Write-Host "`n==========================================================" -ForegroundColor Cyan
Write-Host " FIN DE LA PRUEBA INTEGRAL" -ForegroundColor Cyan
Write-Host "==========================================================" -ForegroundColor Cyan
