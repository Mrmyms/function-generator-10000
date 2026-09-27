$port = New-Object System.IO.Ports.SerialPort "COM10", 115200, "None", 8, "One"
$port.ReadTimeout = 2000
$port.DtrEnable = $true
$port.RtsEnable = $true

try {
    $port.Open()
    Start-Sleep -Milliseconds 1200
    $null = $port.ReadExisting()

    Write-Host "[+] Enviando 'set freq 5000'..." -ForegroundColor Yellow
    $port.WriteLine("set freq 5000")
    
    $endTime = (Get-Date).AddSeconds(1.5)
    while ((Get-Date) -lt $endTime) {
        Start-Sleep -Milliseconds 100
        $data = $port.ReadExisting()
        if ($data) {
            Write-Host -NoNewline $data
        }
    }

    Write-Host "`n[+] Consultando status..." -ForegroundColor Yellow
    $port.WriteLine("status")
    Start-Sleep -Milliseconds 400
    Write-Host $port.ReadExisting()

} finally {
    if ($port.IsOpen) { $port.Close() }
}
