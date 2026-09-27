$port = New-Object System.IO.Ports.SerialPort "COM10", 115200, "None", 8, "One"
$port.ReadTimeout = 2000
$port.DtrEnable = $true
$port.RtsEnable = $true

try {
    $port.Open()
    Start-Sleep -Milliseconds 1000
    $null = $port.ReadExisting()

    $commands = @("set wave q", "set freq 12000", "set buf on", "set wave t", "set freq 2500")

    foreach ($cmd in $commands) {
        Write-Host "`n>>> Enviando: $cmd" -ForegroundColor Yellow
        $port.WriteLine($cmd)
        Start-Sleep -Milliseconds 300
        $data = $port.ReadExisting()
        Write-Host $data -ForegroundColor Green
    }

    Write-Host "`n>>> Verificando estado final del enlace..." -ForegroundColor Cyan
    $port.WriteLine("status")
    Start-Sleep -Milliseconds 400
    Write-Host $port.ReadExisting() -ForegroundColor Cyan

} finally {
    if ($port.IsOpen) { $port.Close() }
}
