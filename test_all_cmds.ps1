$port = New-Object System.IO.Ports.SerialPort "COM10", 115200, "None", 8, "One"
$port.ReadTimeout = 2000
$port.DtrEnable = $true
$port.RtsEnable = $true

try {
    $port.Open()
    Start-Sleep -Milliseconds 1200
    $null = $port.ReadExisting()

    Write-Host "`n>>> [1/4] CMD 0x01: set freq 10000" -ForegroundColor Yellow
    $port.WriteLine("set freq 10000")
    Start-Sleep -Milliseconds 500
    Write-Host $port.ReadExisting() -ForegroundColor Green

    Write-Host "`n>>> [2/4] CMD 0x01: set wave q" -ForegroundColor Yellow
    $port.WriteLine("set wave q")
    Start-Sleep -Milliseconds 500
    Write-Host $port.ReadExisting() -ForegroundColor Green

    Write-Host "`n>>> [3/4] CMD 0x07: set buf on" -ForegroundColor Yellow
    $port.WriteLine("set buf on")
    Start-Sleep -Milliseconds 500
    Write-Host $port.ReadExisting() -ForegroundColor Green

    Write-Host "`n>>> [4/4] CMD 0x01: set wave s" -ForegroundColor Yellow
    $port.WriteLine("set wave s")
    Start-Sleep -Milliseconds 500
    Write-Host $port.ReadExisting() -ForegroundColor Green

    Write-Host "`n>>> Estado del enlace (Verificando conteo acumulado de ACKs):" -ForegroundColor Cyan
    $port.WriteLine("status")
    Start-Sleep -Milliseconds 500
    Write-Host $port.ReadExisting() -ForegroundColor Cyan

} finally {
    if ($port.IsOpen) { $port.Close() }
}
