$port = New-Object System.IO.Ports.SerialPort "COM7", 115200, "None", 8, "One"
$port.ReadTimeout = 1500
$port.DtrEnable = $true
$port.RtsEnable = $true

try {
    $port.Open()
    Start-Sleep -Milliseconds 600
    
    # Read boot banner
    $boot = $port.ReadExisting()
    Write-Host "=== BOOT OUTPUT ==="
    Write-Host $boot
    Write-Host "==================="

    # Send status
    Write-Host "`n>>> Sending: status"
    $port.WriteLine("status")
    Start-Sleep -Milliseconds 500
    Write-Host $port.ReadExisting()

    # Test Equation: Multi-harmonic Fourier synthesis on clean PA4
    Write-Host "`n>>> Sending: eqn sin(t) + 0.5*sin(3*t) + 0.2*sin(5*t)"
    $port.WriteLine("eqn sin(t) + 0.5*sin(3*t) + 0.2*sin(5*t)")
    Start-Sleep -Milliseconds 600
    Write-Host $port.ReadExisting()

    # Test Triangle wave at 25 kHz
    Write-Host "`n>>> Sending: wave t"
    $port.WriteLine("wave t")
    Start-Sleep -Milliseconds 300
    Write-Host $port.ReadExisting()

    Write-Host "`n>>> Sending: freq 25000"
    $port.WriteLine("freq 25000")
    Start-Sleep -Milliseconds 300
    Write-Host $port.ReadExisting()

    # Query Status
    Write-Host "`n>>> Querying: status"
    $port.WriteLine("status")
    Start-Sleep -Milliseconds 600
    Write-Host $port.ReadExisting()

    # Test Equation with sinc function
    Write-Host "`n>>> Sending: eqn sinc(8*pi*(x-0.5))"
    $port.WriteLine("eqn sinc(8*pi*(x-0.5))")
    Start-Sleep -Milliseconds 600
    Write-Host $port.ReadExisting()

    # Reset to 1 kHz for 2048 points
    Write-Host "`n>>> Sending: freq 1000"
    $port.WriteLine("freq 1000")
    Start-Sleep -Milliseconds 300
    Write-Host $port.ReadExisting()

    # Final status
    Write-Host "`n>>> Final status:"
    $port.WriteLine("status")
    Start-Sleep -Milliseconds 600
    Write-Host $port.ReadExisting()

} catch {
    Write-Host "Error: $_"
} finally {
    if ($port.IsOpen) {
        $port.Close()
    }
}
