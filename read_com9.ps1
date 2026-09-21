$port = New-Object System.IO.Ports.SerialPort "COM9", 115200, "None", 8, "One"
$port.ReadTimeout = 1000
$port.DtrEnable = $true
$port.RtsEnable = $true

try {
    $port.Open()
    Write-Host "COM9 opened successfully."
    $null = $port.ReadExisting()
    
    $port.WriteLine("ping")
    Start-Sleep -Milliseconds 400
    $port.WriteLine("status")
    Start-Sleep -Milliseconds 800

    $data = $port.ReadExisting()
    Write-Host $data
} catch {
    Write-Host "Error: $_"
} finally {
    if ($port.IsOpen) {
        $port.Close()
        Write-Host "`nCOM9 closed."
    }
}
