$port = New-Object System.IO.Ports.SerialPort "COM9", 115200, "None", 8, "One"
$port.ReadTimeout = 1000
$port.DtrEnable = $false
$port.RtsEnable = $false

try {
    $port.Open()
    # Pulse EN (RTS) to reset chip into normal run mode
    $port.RtsEnable = $true
    Start-Sleep -Milliseconds 100
    $port.RtsEnable = $false
    Start-Sleep -Milliseconds 3000

    $resp = $port.ReadExisting()
    Write-Host "ESP32 Boot Log:`n$resp"

    # Send status
    $port.WriteLine("status")
    Start-Sleep -Milliseconds 600
    $resp = $port.ReadExisting()
    Write-Host "Status Response:`n$resp"
} catch {
    Write-Host "Error: $_"
} finally {
    if ($port.IsOpen) {
        $port.Close()
    }
}
