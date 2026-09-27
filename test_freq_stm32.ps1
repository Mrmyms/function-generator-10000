$p = New-Object System.IO.Ports.SerialPort "COM7", 115200, "None", 8, "One"
$p.ReadTimeout = 2000
$p.Open()
Start-Sleep -Milliseconds 500
$null = $p.ReadExisting()

function Calc-CRC8([byte[]]$bytes) {
    $crc = 0
    foreach ($b in $bytes) {
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

function Send-FramedConfig([string]$wave, [float]$freq, [byte]$amp) {
    $payload = New-Object byte[] 29
    $payload[0] = [byte][char]$wave
    $fb = [System.BitConverter]::GetBytes([single]$freq)
    [System.Array]::Copy($fb, 0, $payload, 1, 4)
    $payload[5] = $amp
    $payload[6] = 0
    $payload[7] = 1 # Enabled
    $payload[8] = 50 # Duty

    $header = New-Object byte[] 4
    $header[0] = 0xAA
    $header[1] = 0x55
    $header[2] = 0x01
    $header[3] = 29

    $crc = (Calc-CRC8 $payload) -bxor (Calc-CRC8 $header)

    $packet = New-Object byte[] (4 + 29 + 1)
    [System.Array]::Copy($header, 0, $packet, 0, 4)
    [System.Array]::Copy($payload, 0, $packet, 4, 29)
    $packet[33] = [byte]$crc

    $p.Write($packet, 0, $packet.Length)
    Start-Sleep -Milliseconds 400
    $resp = $p.ReadExisting()
    return $resp
}

Write-Host "=== TEST 1: 5000 Hz ===" -ForegroundColor Yellow
$r1 = Send-FramedConfig 's' 5000.0 100
Write-Host $r1 -ForegroundColor Green

Write-Host "=== TEST 2: 10000 Hz ===" -ForegroundColor Yellow
$r2 = Send-FramedConfig 's' 10000.0 100
Write-Host $r2 -ForegroundColor Green

Write-Host "=== TEST 3: 25000 Hz ===" -ForegroundColor Yellow
$r3 = Send-FramedConfig 'q' 25000.0 100
Write-Host $r3 -ForegroundColor Green

Write-Host "=== TEST 4: 50000 Hz ===" -ForegroundColor Yellow
$r4 = Send-FramedConfig 't' 50000.0 100
Write-Host $r4 -ForegroundColor Green

Write-Host "=== TEST 5: 1000 Hz ===" -ForegroundColor Yellow
$r5 = Send-FramedConfig 's' 1000.0 100
Write-Host $r5 -ForegroundColor Green

$p.Close()
