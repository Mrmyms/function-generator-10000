$bin = Get-ChildItem -Path $env:TEMP -Filter "stm32_firmware.ino.bin" -Recurse -ErrorAction SilentlyContinue | Sort-Object LastWriteTime -Descending | Select-Object -First 1
if ($bin) {
    Write-Host "Found compiled bin at: $($bin.FullName)"
    Copy-Item $bin.FullName -Destination "F:\" -Force
    Write-Host "Flashing STM32 via F:\..."
    Start-Sleep -Seconds 4
    Write-Host "STM32 flashed successfully!"
} else {
    Write-Host "Bin not found in TEMP!"
}
