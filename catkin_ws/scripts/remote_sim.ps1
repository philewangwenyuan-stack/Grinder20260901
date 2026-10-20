param(
    [ValidateSet('help','doctor','sync','watch','build','start','rviz','shell','tunnel','app')]
    [string]$Action = 'help',
    [string]$Distro = 'Ubuntu-20.04',
    [string]$RemoteHost = '172.20.22.114',
    [int]$Port = 2222,
    [string]$RemoteUser = 'dev',
    [string]$RemoteRoot = '/home/dev/work/Grinder-remote-sim',
    [string]$Identity = "$env:USERPROFILE\.ssh\grinder_remote_sim",
    [int]$LocalControlPort = 18002,
    [int]$LocalRtspPort = 18554,
    [string]$Adb = 'D:\Program Files\Netease\MuMu Player 12\nx_main\adb.exe',
    [string]$AdbSerial = '127.0.0.1:16384',
    [Parameter(ValueFromRemainingArguments=$true)][string[]]$ExtraArgs
)
$ErrorActionPreference = 'Stop'
if ($Action -eq 'tunnel') {
    # Windows-native listener lets MuMu adb reverse reach the SSH tunnel.
    & ssh -N -i $Identity -p $Port -o IdentitiesOnly=yes -o ExitOnForwardFailure=yes `
        -o ServerAliveInterval=15 -o ServerAliveCountMax=3 -o StrictHostKeyChecking=accept-new `
        -L "127.0.0.1:${LocalControlPort}:127.0.0.1:8002" `
        -L "127.0.0.1:${LocalRtspPort}:127.0.0.1:8554" "$RemoteUser@$RemoteHost"
    exit $LASTEXITCODE
}
if ($Action -eq 'app') {
    if (!(Test-Path -LiteralPath $Adb)) { throw "MuMu adb not found: $Adb" }
    & $Adb connect $AdbSerial
    if ($LASTEXITCODE -ne 0) { throw 'MuMu ADB connection failed' }
    foreach ($mapping in @(@(8002,$LocalControlPort),@(8554,$LocalRtspPort))) {
        & $Adb -s $AdbSerial reverse "tcp:$($mapping[0])" "tcp:$($mapping[1])"
        if ($LASTEXITCODE -ne 0) { throw "MuMu port mapping failed: $($mapping[0])" }
    }
    Write-Host 'Keep the tunnel terminal open. Enter 127.0.0.1:8002 in the APP.'
    exit 0
}
$windowsScript = ($PSScriptRoot + '\remote_sim.sh').Replace('\','/')
$pathOutput = & wsl -d $Distro --exec wslpath -u $windowsScript
if ($LASTEXITCODE -ne 0) { throw 'Cannot resolve the script path inside local WSL' }
$wslScript = ($pathOutput -join '').Trim()
$pathOutput = & wsl -d $Distro --exec wslpath -u $Identity.Replace('\','/')
if ($LASTEXITCODE -ne 0) { throw 'Cannot resolve the SSH identity path' }
$wslIdentity = ($pathOutput -join '').Trim()
$keyCopy = "import pathlib,sys,shutil; src=pathlib.Path(sys.argv[1]); d=pathlib.Path.home()/'.ssh'; d.mkdir(mode=448,exist_ok=True); p=d/'grinder_remote_sim_windows'; shutil.copyfile(str(src),str(p)); p.chmod(384); print(p)"
$clientKey = (& wsl -d $Distro -- python3 -c $keyCopy $wslIdentity).Trim()
if ($LASTEXITCODE -ne 0) { throw 'Cannot prepare a private SSH key copy inside local WSL' }
& wsl -d $Distro -- env "GRINDER_REMOTE_HOST=$RemoteHost" "GRINDER_REMOTE_PORT=$Port" `
    "GRINDER_REMOTE_USER=$RemoteUser" "GRINDER_REMOTE_ROOT=$RemoteRoot" `
    "GRINDER_REMOTE_KEY=$clientKey" `
    bash $wslScript $Action @ExtraArgs
exit $LASTEXITCODE
