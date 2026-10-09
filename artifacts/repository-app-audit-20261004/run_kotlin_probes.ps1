param([string]$AppRoot = 'E:\CODE\grindingrobot\grindingrobot')
$ErrorActionPreference = 'Stop'
$artifactRoot = $PSScriptRoot
$cacheRoot = Join-Path $env:USERPROFILE '.gradle\caches\modules-2\files-2.1'
function Resolve-Jar([string]$group, [string]$name, [string]$version) {
    $path = Join-Path $cacheRoot "$group\$name\$version"
    (Get-ChildItem -LiteralPath $path -Recurse -Filter '*.jar' | Select-Object -First 1).FullName
}
$stdlib = Resolve-Jar 'org.jetbrains.kotlin' 'kotlin-stdlib' '2.2.21'
$compilerJars = @(
    (Resolve-Jar 'org.jetbrains.kotlin' 'kotlin-compiler-embeddable' '2.2.21'),
    $stdlib,
    (Resolve-Jar 'org.jetbrains.kotlin' 'kotlin-script-runtime' '2.2.21'),
    (Resolve-Jar 'org.jetbrains.kotlin' 'kotlin-reflect' '1.6.10'),
    (Resolve-Jar 'org.jetbrains.kotlin' 'kotlin-daemon-embeddable' '2.2.21'),
    (Resolve-Jar 'org.jetbrains.kotlinx' 'kotlinx-coroutines-core-jvm' '1.8.0'),
    (Resolve-Jar 'org.jetbrains' 'annotations' '23.0.0')
)
$sources = @(
    "$AppRoot\core\sllink\src\frame\sl_link\SlFrame.kt",
    "$AppRoot\core\sllink\src\frame\sl_link\SlFrameParser.kt",
    "$AppRoot\core\sllink\src\frame\sl_link\Crc16.kt",
    "$AppRoot\core\sllink\src\frame\sl_link\SlLinkMonitor.kt",
    "$AppRoot\core\tcp\src\main\java\com\sinelynx\grindingrobot\core\tcp\TaskPathChunkAssembler.kt",
    "$artifactRoot\AuditProbe.kt"
)
$outputJar = Join-Path $artifactRoot 'audit-probes.jar'
& java -Xmx256m -cp ($compilerJars -join ';') org.jetbrains.kotlin.cli.jvm.K2JVMCompiler -no-stdlib -no-reflect -classpath $stdlib -d $outputJar @sources
if ($LASTEXITCODE -ne 0) { throw 'Kotlin compilation failed' }
& java -Xmx128m -cp "$outputJar;$stdlib" audit.AuditProbeKt
if ($LASTEXITCODE -ne 0) { throw 'Kotlin probe failed' }
