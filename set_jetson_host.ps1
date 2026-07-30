[CmdletBinding()]
param(
    [Parameter(Mandatory = $true, Position = 0)]
    [ValidateNotNullOrEmpty()]
    [string]$JetsonHost
)

$value = $JetsonHost.Trim()
if ($value -match '[\s/:]') {
    throw "JetsonHost must be an IP address or hostname without protocol, port, or path."
}

$env:X3PLUS_JETSON_HOST = $value

Write-Host "X3PLUS_JETSON_HOST=$value"
Write-Host "All project tools launched from this PowerShell window will use this host."
Write-Host "SSH: ssh jetson@$value"
Write-Host "Camera: http://${value}:8080/stream?topic=/arm_cam/image_raw"
