param(
    [ValidateSet('tunnel','app')][string]$Action = 'tunnel',
    [ValidateRange(1,3)][int]$Robot = 1,
    [ValidateRange(2,3)][int]$Robots = 2,
    [string]$RemoteHost = '172.20.22.114',
    [int]$SshPort = 2222,
    [string]$RemoteUser = 'dev',
    [string]$Identity = "$env:USERPROFILE\.ssh\grinder_remote_sim",
    [string]$Adb = 'D:\Program Files\Netease\MuMu Player 12\nx_main\adb.exe',
    [string]$AdbSerial = '127.0.0.1:16384'
)
$ErrorActionPreference = 'Stop'
if ($Action -eq 'tunnel') {
    $sshArgs = @('-N', '-i', $Identity, '-p', "$SshPort", '-o', 'IdentitiesOnly=yes',
        '-o', 'ExitOnForwardFailure=yes', '-o', 'ServerAliveInterval=15', '-o', 'ServerAliveCountMax=3')
    for ($index = 1; $index -le $Robots; $index++) {
        $remoteData = 8002 + 100 * $index
        $localData = 18002 + 100 * $index
        $sshArgs += @('-L', "127.0.0.1:${localData}:127.0.0.1:${remoteData}")
        $sshArgs += @('-L', "127.0.0.1:$($localData+1):127.0.0.1:$($remoteData+1)")
    }
    $sshArgs += "$RemoteUser@$RemoteHost"
    Write-Host 'Keep this terminal open. Configure each MuMu instance with the app action.'
    & ssh @sshArgs
    exit $LASTEXITCODE
}
if (!(Test-Path -LiteralPath $Adb)) { throw "MuMu adb not found: $Adb" }
& $Adb connect $AdbSerial
if ($LASTEXITCODE -ne 0) { throw 'MuMu ADB connection failed' }
$localData = 18002 + 100 * $Robot
# The current APP has a fixed control port 8003. Translate BOTH channels
# per emulator instance; do not connect the APP directly to 8102/8202.
& $Adb -s $AdbSerial reverse tcp:8002 "tcp:$localData"
if ($LASTEXITCODE -ne 0) { throw 'Cannot map APP data channel' }
& $Adb -s $AdbSerial reverse tcp:8003 "tcp:$($localData+1)"
if ($LASTEXITCODE -ne 0) { throw 'Cannot map APP control channel' }
Write-Host "MuMu $AdbSerial -> robot$Robot. Enter 127.0.0.1:8002 in this instance's APP."
