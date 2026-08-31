[CmdletBinding()]
param(
    [string]$Distro = "Ubuntu-20.04",
    [int[]]$Ports = @(8002, 8554)
)

$identity = [Security.Principal.WindowsIdentity]::GetCurrent()
$principal = [Security.Principal.WindowsPrincipal]::new($identity)
if (-not $principal.IsInRole([Security.Principal.WindowsBuiltInRole]::Administrator)) {
    throw "Run this script from an elevated PowerShell window."
}

$rawAddress = (& wsl.exe -d $Distro -- hostname -I 2>$null)
if ($LASTEXITCODE -ne 0 -or -not $rawAddress) {
    throw "Unable to query WSL address for '$Distro'."
}
$wslAddress = (($rawAddress -join " ").Trim() -split "\s+")[0]
$parsed = $null
if (-not [Net.IPAddress]::TryParse($wslAddress, [ref]$parsed) -or
    $parsed.AddressFamily -ne [Net.Sockets.AddressFamily]::InterNetwork) {
    throw "WSL returned an invalid IPv4 address: '$wslAddress'."
}

foreach ($port in $Ports) {
    if ($port -lt 1 -or $port -gt 65535) {
        throw "Invalid TCP port: $port"
    }
    & netsh.exe interface portproxy delete v4tov4 listenaddress=0.0.0.0 listenport=$port protocol=tcp | Out-Null
    & netsh.exe interface portproxy add v4tov4 listenaddress=0.0.0.0 listenport=$port connectaddress=$wslAddress connectport=$port protocol=tcp | Out-Null
    if ($LASTEXITCODE -ne 0) {
        throw "Failed to configure portproxy for TCP $port."
    }

    $ruleName = "Grinder WSL Simulator TCP $port"
    $existing = Get-NetFirewallRule -DisplayName $ruleName -ErrorAction SilentlyContinue
    if ($existing) {
        Set-NetFirewallRule -DisplayName $ruleName -Enabled True -Profile Private -Action Allow | Out-Null
    } else {
        New-NetFirewallRule -DisplayName $ruleName -Direction Inbound -Action Allow -Protocol TCP -LocalPort $port -Profile Private | Out-Null
    }
}

Write-Host "WSL address: $wslAddress"
Write-Host "Forwarded TCP ports: $($Ports -join ', ')"
Write-Host "Connect the phone APP to <Windows-LAN-IP>:8002."

