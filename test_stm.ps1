$port = New-Object System.IO.Ports.SerialPort "COM7", 115200, "None", 8, "One"
$port.ReadTimeout = 2000
$port.NewLine = "`n"
$port.DtrEnable = $true
$port.RtsEnable = $true

try {
    $port.Open()
    Write-Host "[+] COM7 opened, waiting 2s for STM32..."
    Start-Sleep -Milliseconds 2000
    $port.DiscardInBuffer()

    Write-Host "[+] Sending: status"
    $port.Write("status`n")
    Start-Sleep -Milliseconds 600
    $resp = $port.ReadExisting()
    Write-Host $resp

    Write-Host "[+] Sending: wave q"
    $port.Write("wave q`n")
    Start-Sleep -Milliseconds 400
    $resp2 = $port.ReadExisting()
    Write-Host $resp2

    Write-Host "[+] Sending: freq 5000"
    $port.Write("freq 5000`n")
    Start-Sleep -Milliseconds 400
    $resp3 = $port.ReadExisting()
    Write-Host $resp3

    Write-Host "[+] Reading 2s of live telemetry:"
    $end = (Get-Date).AddSeconds(2)
    while ((Get-Date) -lt $end) {
        Start-Sleep -Milliseconds 250
        $t = $port.ReadExisting()
        if ($t) { Write-Host -NoNewline $t }
    }
} finally {
    if ($port.IsOpen) {
        $port.Close()
        Write-Host "`n[+] COM7 closed."
    }
}
