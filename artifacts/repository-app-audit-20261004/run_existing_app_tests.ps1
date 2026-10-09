param([string]$AppRoot = 'E:\CODE\grindingrobot\grindingrobot')
$ErrorActionPreference = 'Stop'
$cacheRoot = Join-Path $env:USERPROFILE '.gradle\caches\modules-2\files-2.1'
$classPaths = @()
foreach ($module in @('core/tcp', 'core/sllink', 'core/model', 'core/data', 'core/util', 'core/common', 'feature/device')) {
    foreach ($suffix in @('build/tmp/kotlin-classes/devDebug', 'build/tmp/kotlin-classes/devDebugUnitTest', 'build/intermediates/javac/devDebug/compileDevDebugJavaWithJavac/classes')) {
        $candidate = Join-Path $AppRoot "$module/$suffix"
        if (Test-Path -LiteralPath $candidate) { $classPaths += $candidate }
    }
}
foreach ($artifact in @('junit/junit', 'org.hamcrest/hamcrest-core', 'org.jetbrains.kotlin/kotlin-stdlib',
    'org.jetbrains.kotlinx/kotlinx-coroutines-core-jvm', 'com.google.protobuf/protobuf-java',
    'org.jetbrains.kotlinx/kotlinx-serialization-core-jvm', 'org.jetbrains.kotlinx/kotlinx-serialization-json-jvm',
    'org.jetbrains/annotations', 'com.google.code.gson/gson')) {
    $jars = Get-ChildItem -LiteralPath (Join-Path $cacheRoot $artifact) -Recurse -Filter '*.jar'
    $classPaths += ($jars | Sort-Object LastWriteTime -Descending | Select-Object -First 1).FullName
}
$tests = @()
foreach ($relative in @('core/tcp/src/test/java/com/sinelynx/grindingrobot/core/tcp')) {
    foreach ($file in (Get-ChildItem -LiteralPath (Join-Path $AppRoot $relative) -Filter '*Test.kt')) {
        $package = (Select-String -LiteralPath $file.FullName -Pattern '^package (.+)$').Matches.Groups[1].Value
        $tests += "$package.$($file.BaseName)"
    }
}
# Match the APP's declared protobuf runtime instead of selecting by cache mtime.
$classPaths = @($classPaths | Where-Object { $_ -notmatch 'protobuf-java' })
$classPaths += (Get-ChildItem -LiteralPath (Join-Path $cacheRoot 'com.google.protobuf/protobuf-java/3.24.0') -Recurse -Filter '*.jar' | Select-Object -First 1).FullName
$classPaths = @($classPaths | Where-Object { $_ -notmatch 'kotlin-stdlib' })
$stdlib = (Get-ChildItem -LiteralPath (Join-Path $cacheRoot 'org.jetbrains.kotlin/kotlin-stdlib/2.2.21') -Recurse -Filter '*.jar' | Select-Object -First 1).FullName
$classPaths += $stdlib
$classPaths = @($classPaths | Where-Object { $_ -notmatch 'kotlinx-coroutines-core-jvm' })
$classPaths += (Get-ChildItem -LiteralPath (Join-Path $cacheRoot 'org.jetbrains.kotlinx/kotlinx-coroutines-core-jvm/1.10.1') -Recurse -Filter '*.jar' | Select-Object -First 1).FullName
# Compile the controller tests against the current model: the old device test
# classes on disk predate request_id/snapshot_id additions and cannot be reused.
$compilerJars = @($stdlib)
foreach ($entry in @('org.jetbrains.kotlin/kotlin-compiler-embeddable/2.2.21', 'org.jetbrains.kotlin/kotlin-script-runtime/2.2.21',
    'org.jetbrains.kotlin/kotlin-reflect/1.6.10', 'org.jetbrains.kotlin/kotlin-daemon-embeddable/2.2.21',
    'org.jetbrains.kotlinx/kotlinx-coroutines-core-jvm/1.8.0', 'org.jetbrains/annotations/23.0.0')) {
    $compilerJars += (Get-ChildItem -LiteralPath (Join-Path $cacheRoot $entry) -Recurse -Filter '*.jar' | Select-Object -First 1).FullName
}
$freshJar = Join-Path $PSScriptRoot 'device-controller-tests.jar'
$controller = "$AppRoot/feature/device/src/main/java/com/sinelynx/grindingrobot/feature/device/viewmodel/DeviceTaskMapController.kt"
$controllerTests = "$AppRoot/feature/device/src/test/java/com/sinelynx/grindingrobot/feature/device/viewmodel/DeviceTaskMapControllerTest.kt"
& java -Xmx256m -cp ($compilerJars -join ';') org.jetbrains.kotlin.cli.jvm.K2JVMCompiler -no-stdlib -no-reflect -classpath ($classPaths -join ';') -d $freshJar $controller $controllerTests "$PSScriptRoot/AuditControllerProbe.kt" "$PSScriptRoot/AuditTransmissionProbe.kt"
if ($LASTEXITCODE -ne 0) { throw 'Fresh controller test compilation failed' }
$classPaths = @($freshJar) + $classPaths
$tests += 'com.sinelynx.grindingrobot.feature.device.viewmodel.DeviceTaskMapControllerTest'
$tests += 'audit.AuditControllerProbe'
$tests += 'audit.AuditTransmissionProbe'
& java -Xmx128m -cp ($classPaths -join ';') org.junit.runner.JUnitCore @tests
if ($LASTEXITCODE -ne 0) { throw 'Existing APP JVM tests failed' }
