$ports = @("COM3", "COM4", "COM5", "COM6")
foreach ($p in $ports) {
    try {
        $sp = New-Object System.IO.Ports.SerialPort $p, 115200, "None", 8, "One"
        $sp.ReadTimeout = 500
        $sp.WriteTimeout = 500
        $sp.Open()
        $sp.WriteLine("status")
        Start-Sleep -Milliseconds 300
        $resp = $sp.ReadExisting()
        if ($resp) {
            Write-Host ">>> FOUND ESP32 on $p! Resp: $resp" -ForegroundColor Green
        } else {
            Write-Host "$p opened, no response" -ForegroundColor Yellow
        }
        $sp.Close()
    } catch {
        Write-Host "$p cannot open: $($_.Exception.Message)" -ForegroundColor Red
    }
}
