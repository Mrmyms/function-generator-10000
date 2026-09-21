$port = New-Object System.IO.Ports.SerialPort "COM7", 115200, "None", 8, "One"
$port.ReadTimeout = 1000
$port.DtrEnable = $true
$port.RtsEnable = $true

try {
    $port.Open()
    Start-Sleep -Milliseconds 800
    
    # Flush existing buffer
    $null = $port.ReadExisting()

    # Send status command
    Write-Host ">>> Sending 'status' command to STM32..."
    $port.WriteLine("status")
    Start-Sleep -Milliseconds 500
    $resp = $port.ReadExisting()
    Write-Host $resp

    # Send phase shift: 90 degrees
    Write-Host ">>> Sending 'phase 90' to STM32..."
    $port.WriteLine("phase 90")
    Start-Sleep -Milliseconds 400
    $resp = $port.ReadExisting()
    Write-Host $resp

    # Send noise test: CH1 to Noise @ 5 kHz, 100%
    Write-Host ">>> Sending 'ch1 n 5000 100' (LFSR White Noise) to STM32..."
    $port.WriteLine("ch1 n 5000 100")
    Start-Sleep -Milliseconds 400
    $resp = $port.ReadExisting()
    Write-Host $resp

    # Send status again to verify updated silicon state
    Write-Host ">>> Querying updated 'status'..."
    $port.WriteLine("status")
    Start-Sleep -Milliseconds 500
    $resp = $port.ReadExisting()
    Write-Host $resp

} catch {
    Write-Host "Error: $_"
} finally {
    if ($port.IsOpen) {
        $port.Close()
    }
}
