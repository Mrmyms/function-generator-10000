$port = New-Object System.IO.Ports.SerialPort "COM10", 115200, "None", 8, "One"
$port.ReadTimeout = 1000
$port.DtrEnable = $false
$port.RtsEnable = $false

try {
    $port.Open()
    Write-Host "[+] COM10 abierto."
    
    # Pulse EN to reboot ESP32 into normal execution mode
    $port.DtrEnable = $false
    $port.RtsEnable = $true
    Start-Sleep -Milliseconds 150
    $port.RtsEnable = $false
    Start-Sleep -Milliseconds 2500

    $resp = $port.ReadExisting()
    Write-Host "ESP32 Boot Log:`n$resp"

    Write-Host "[+] Enviando 'status'..."
    $port.WriteLine("status")
    Start-Sleep -Milliseconds 800
    $resp = $port.ReadExisting()
    Write-Host "Status Response:`n$resp"

    Write-Host "[+] Enviando 'ping'..."
    $port.WriteLine("ping")
    Start-Sleep -Milliseconds 800
    $resp = $port.ReadExisting()
    Write-Host "Ping Response:`n$resp"
} catch {
    Write-Host "Error: $_"
} finally {
    if ($port.IsOpen) {
        $port.Close()
        Write-Host "`n[+] COM10 cerrado."
    }
}
