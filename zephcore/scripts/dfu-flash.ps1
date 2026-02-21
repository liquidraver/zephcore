# dfu-flash.ps1 - Flash ZephCore via serial DFU with automatic port detection
#
# Usage: .\dfu-flash.ps1 -Port COM3 -Package path\to\zephyr.zip
#        .\dfu-flash.ps1 COM3 zephyr.zip
#
# The firmware and bootloader use different USB VID/PID, so Windows assigns
# different COM ports. This script handles the port switch automatically:
#   1. Triggers 1200 baud touch on the firmware's port
#   2. Waits for the bootloader to enumerate on a new port
#   3. Runs adafruit-nrfutil DFU on the bootloader's port

param(
    [Parameter(Position=0, Mandatory=$true)]
    [string]$Port,

    [Parameter(Position=1, Mandatory=$true)]
    [string]$Package
)

if (-not (Test-Path $Package)) {
    Write-Host "Error: package not found: $Package" -ForegroundColor Red
    exit 1
}

# Snapshot current COM ports
$before = [System.IO.Ports.SerialPort]::GetPortNames() | Sort-Object

Write-Host "Current ports: $($before -join ', ')"
Write-Host "Triggering 1200 baud reset on $Port..."

try {
    $ser = New-Object System.IO.Ports.SerialPort $Port, 1200
    $ser.DtrEnable = $true
    $ser.Open()
    Start-Sleep -Milliseconds 200
    $ser.DtrEnable = $false
    Start-Sleep -Milliseconds 100
    $ser.Close()
} catch {
    Write-Host "Error opening ${Port}: $_" -ForegroundColor Red
    exit 1
}

# Wait for bootloader to enumerate (up to 10 seconds)
Write-Host "Waiting for bootloader..."
$dfuPort = $null
$deadline = (Get-Date).AddSeconds(10)

while ((Get-Date) -lt $deadline) {
    Start-Sleep -Milliseconds 500
    $after = [System.IO.Ports.SerialPort]::GetPortNames() | Sort-Object
    $newPorts = $after | Where-Object { $_ -notin $before }

    if ($newPorts.Count -gt 0) {
        $dfuPort = $newPorts[0]
        break
    }
}

if (-not $dfuPort) {
    Write-Host "Error: no new COM port detected. Bootloader may not have started." -ForegroundColor Red
    Write-Host "Available ports: $([System.IO.Ports.SerialPort]::GetPortNames() -join ', ')"
    exit 1
}

Write-Host "Bootloader found on $dfuPort" -ForegroundColor Green
Write-Host ""

adafruit-nrfutil --verbose dfu serial --package $Package -p $dfuPort -b 115200 --singlebank
