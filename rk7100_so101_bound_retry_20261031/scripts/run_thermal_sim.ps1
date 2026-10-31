$ErrorActionPreference='Stop'
$projectRoot=Split-Path -Parent $PSScriptRoot
$simDir=Join-Path $projectRoot 'build/thermal_sim'
New-Item -ItemType Directory -Force -Path $simDir | Out-Null
Set-Location -LiteralPath $simDir
$tool='D:/Software/vivado2024.2/Vivado/2024.2/bin'
& "$tool/xvlog.bat" "$projectRoot/rtl/chip_thermal.v" "$projectRoot/tests/tb_chip_thermal.v" *> xvlog_console.txt
if ($LASTEXITCODE -ne 0) {throw 'xvlog failed'}
& "$tool/xelab.bat" tb_chip_thermal -s thermal_test *> xelab_console.txt
if ($LASTEXITCODE -ne 0) {throw 'xelab failed'}
& "$tool/xsim.bat" thermal_test -runall *> "$projectRoot/reports/thermal_rtl_sim.txt"
if ($LASTEXITCODE -ne 0) {throw 'xsim failed'}
Get-Content "$projectRoot/reports/thermal_rtl_sim.txt" -Tail 8

