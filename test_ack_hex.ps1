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
    Write-Host "[+] COM7 opened, waiting for STM32 boot..."
    Start-Sleep -Milliseconds 1800
    $port.DiscardInBuffer()

    # Build CMD 0x01: Sine @ 2500 Hz, 80% amp
    $buf = New-Object byte[] 9
    $buf[0] = [byte][char]'s' # Waveform 's'
    $fBytes = [System.BitConverter]::GetBytes([float]2500.0) # 2500 Hz
    [System.Buffer]::BlockCopy($fBytes, 0, $buf, 1, 4)
    $buf[5] = 80  # Amp 80%
    $buf[6] = 0   # Offset 0
    $buf[7] = 1   # Enabled 1
    $buf[8] = 50  # Duty 50%

    $packet = Build-Packet 0x01 $buf
    Write-Host "[+] Sending CMD 0x01 (Config: Sine 2500 Hz, 80%)..."
    $port.Write($packet, 0, $packet.Length)
    $receivedBytes = New-Object System.Collections.Generic.List[byte]
    $deadline = (Get-Date).AddMilliseconds(1500)
    while ((Get-Date) -lt $deadline) {
        while ($port.BytesToRead -gt 0) {
            $receivedBytes.Add([byte]$port.ReadByte())
        }
        if ($receivedBytes.Count -ge 16) { break }
        Start-Sleep -Milliseconds 20
    }
    $resp = $receivedBytes.ToArray()
    $hex = ($resp | ForEach-Object { "0x{0:X2}" -f $_ }) -join " "
    Write-Host "[+] Bytes read: $($resp.Length)"
    Write-Host "[+] Raw Hex Response: $hex"

        if ($resp.Length -ge 5 -and $resp[0] -eq 0xAA -and $resp[1] -eq 0x55 -and $resp[2] -eq 0x81) {
            $health = $resp[5]
            $udr = $resp[6]
            $pts = [System.BitConverter]::ToUInt16($resp, 7)
            $fAct = [System.BitConverter]::ToUInt32($resp, 9)
            $wave = [char]$resp[13]

            Write-Host "`n==========================================================" -ForegroundColor Green
            Write-Host " CONFIRMACION DE LA STM32 RECIBIDA EXITOSAMENTE (ACK 0x81)" -ForegroundColor Green
            Write-Host "==========================================================" -ForegroundColor Green
            Write-Host "  Comando confirmado:  0x01 (Channel Config)"
            Write-Host "  Forma de onda:       $wave"
            Write-Host "  Frecuencia real:     $fAct Hz"
            Write-Host "  Puntos por ciclo:    $pts muestras"
            Write-Host "  Health Byte:         0x$('{0:X2}' -f $health)"
            Write-Host "    - DMA activo:      $((($health -band 0x01) -ne 0))"
            Write-Host "    - Cero Underruns:  $((($health -band 0x02) -ne 0)) (Total UDR: $udr)"
            Write-Host "    - Timer activo:    $((($health -band 0x04) -ne 0))"
            Write-Host "    - Salida armada:   $((($health -band 0x08) -ne 0))"
            Write-Host "    - Amplitud valida: $((($health -band 0x10) -ne 0))"
            Write-Host "==========================================================" -ForegroundColor Green
        }
} finally {
    if ($port.IsOpen) { $port.Close() }
}
