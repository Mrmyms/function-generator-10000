function Get-CRC8($data) {
    $crc = 0
    foreach ($b in $data) {
        $crc = $crc -bxor $b
        for ($j = 0; $j -lt 8; $j++) {
            if (($crc -band 0x80) -ne 0) {
                $crc = (($crc -shl 1) -bxor 0x07) -band 0xFF
            } else {
                $crc = ($crc -shl 1) -band 0xFF
            }
        }
    }
    return [byte]$crc
}

function Build-Packet($cmd, $payloadBytes) {
    $len = [byte]$payloadBytes.Length
    $header = [byte[]]@(0xAA, 0x55, $cmd, $len)
    $crc = (Get-CRC8 $payloadBytes) -bxor (Get-CRC8 $header)
    $packet = New-Object byte[] (4 + $payloadBytes.Length + 1)
    [System.Buffer]::BlockCopy($header, 0, $packet, 0, 4)
    if ($payloadBytes.Length -gt 0) {
        [System.Buffer]::BlockCopy($payloadBytes, 0, $packet, 4, $payloadBytes.Length)
    }
    $packet[$packet.Length - 1] = $crc
    return $packet
}

$port = New-Object System.IO.Ports.SerialPort "COM7", 115200, "None", 8, "One"
$port.DtrEnable = $true
$port.RtsEnable = $true
$port.ReadTimeout = 1500

try {
    $port.Open()
    Start-Sleep -Milliseconds 400
    $null = $port.ReadExisting()

    Write-Host ">>> Testing Binary Command 0x05 (CH1 Equation via Binary Protocol)..."
    $eq1 = [System.Text.Encoding]::ASCII.GetBytes("sin(t) + 0.5*sin(3*t) + 0.25*sin(5*t)")
    $packet1 = Build-Packet 0x05 $eq1

    Write-Host ("Sending " + $packet1.Length + " bytes packet for: sin(t) + 0.5*sin(3*t) + 0.25*sin(5*t)")
    $port.Write($packet1, 0, $packet1.Length)
    Start-Sleep -Milliseconds 500

    Write-Host "STM32 Response:"
    Write-Host $port.ReadExisting()

    Write-Host "`n>>> Testing Binary Command 0x06 (CH2 Equation via Binary Protocol)..."
    $eq2 = [System.Text.Encoding]::ASCII.GetBytes("exp(-4*x)*sin(10*pi*x)")
    $packet2 = Build-Packet 0x06 $eq2

    Write-Host ("Sending " + $packet2.Length + " bytes packet for: exp(-4*x)*sin(10*pi*x)")
    $port.Write($packet2, 0, $packet2.Length)
    Start-Sleep -Milliseconds 500

    Write-Host "STM32 Response:"
    Write-Host $port.ReadExisting()

    Write-Host "`n>>> Querying STM32 Status..."
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
