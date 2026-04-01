param(
  [string]$Port = "COM10",
  [string]$BuildPath = ".\\build"
)

$ErrorActionPreference = "Stop"

$fqbn = "esp32:esp32:esp32s3:FlashSize=16M,PartitionScheme=esp_sr_16,PSRAM=opi,FlashMode=qio,USBMode=hwcdc,CDCOnBoot=cdc"

$arduinoCli = $null
$cmd = Get-Command arduino-cli -ErrorAction SilentlyContinue
if ($cmd) {
  $arduinoCli = $cmd.Source
} else {
  $fallbacks = @(
    "C:\\Users\\Justin\\.vscode\\extensions\\vscode-arduino.vscode-arduino-community-0.7.2-win32-x64\\assets\\platform\\win32-x64\\arduino-cli\\arduino-cli.exe",
    "C:\\Users\\Justin\\.vscode\\extensions\\thelastoutpostworkshop.arduino-maker-workshop-1.1.4-win32-x64\\arduino_cli\\win32\\arduino-cli.exe"
  )
  foreach ($candidate in $fallbacks) {
    if (Test-Path $candidate) {
      $arduinoCli = $candidate
      break
    }
  }
}

if (-not $arduinoCli) {
  throw "arduino-cli not found. Add it to PATH or update flash_sr.ps1 fallback paths."
}

& $arduinoCli compile --fqbn $fqbn --build-path $BuildPath .

$modelSrc = "$env:LOCALAPPDATA\\Arduino15\\packages\\esp32\\tools\\esp32s3-libs\\3.3.7\\esp_sr\\srmodels.bin"
$modelDst = Join-Path $BuildPath "srmodels.bin"
if (-not (Test-Path $modelSrc)) {
  throw "ESP-SR model missing at $modelSrc"
}
Copy-Item -Force $modelSrc $modelDst

& $arduinoCli upload --fqbn $fqbn --port $Port --input-dir $BuildPath .

Write-Host "Flash complete on $Port"
