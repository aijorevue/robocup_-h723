param(
    [switch]$Clean,
    [ValidateSet("Debug", "Release")]
    [string]$Configuration = "Release"
)

$ErrorActionPreference = "Stop"
$buildDirectory = Join-Path $PSScriptRoot "build"

if ($Clean -and (Test-Path -LiteralPath $buildDirectory)) {
    Remove-Item -LiteralPath $buildDirectory -Recurse -Force
}

cmake -G Ninja `
    "-DCMAKE_TOOLCHAIN_FILE=$PSScriptRoot\arm-none-eabi-gcc.cmake" `
    "-DCMAKE_BUILD_TYPE=$Configuration" `
    -B $buildDirectory `
    -S $PSScriptRoot
if ($LASTEXITCODE -ne 0) {
    throw "CMake configuration failed with exit code $LASTEXITCODE"
}

cmake --build $buildDirectory
if ($LASTEXITCODE -ne 0) {
    throw "Build failed with exit code $LASTEXITCODE"
}
