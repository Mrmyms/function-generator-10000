$port = New-Object System.IO.Ports.SerialPort "COM7", 115200, "None", 8, "One"
$port.ReadTimeout = 1000
$port.DtrEnable = $true
$port.RtsEnable = $true

try {
    $port.Open()
    Write-Host "COM7 opened successfully. Reading for 10 seconds..."
    $endTime = (Get-Date).AddSeconds(10)

    while ((Get-Date) -lt $endTime) {
        Start-Sleep -Milliseconds 200
        $data = $port.ReadExisting()
        if ($data.Length -gt 0) {
            Write-Host -NoNewline $data
        }
    }
} catch {
    Write-Host "Error: $_"
} finally {
    if ($port.IsOpen) {
        $port.Close()
        Write-Host "`nCOM7 closed."
    }
}
