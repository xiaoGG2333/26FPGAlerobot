$ErrorActionPreference='Stop'
$projectRoot=Split-Path -Parent $PSScriptRoot
Set-Location -LiteralPath "$projectRoot/build/thermal_sim"
$amd=(Get-Content -Raw (Join-Path $projectRoot 'config/toolchain.json') | ConvertFrom-Json).amd_root
Copy-Item -LiteralPath (Join-Path $projectRoot 'tests/fixtures/xadc_design.txt') -Destination design.txt
$tool=Join-Path $amd 'Vivado'
& "$tool/bin/xvlog.bat" "$projectRoot/rtl/chip_thermal.v" "$projectRoot/tests/tb_chip_xadc.v" "$tool/data/verilog/src/glbl.v" *> xvlog_xadc_console.txt
if ($LASTEXITCODE -ne 0) {throw 'xvlog failed'}
& "$tool/bin/xelab.bat" tb_chip_xadc glbl -L unisims_ver -s xadc_test *> xelab_xadc_console.txt
if ($LASTEXITCODE -ne 0) {throw 'xelab failed'}
& "$tool/bin/xsim.bat" xadc_test -runall *> "$projectRoot/reports/xadc_rtl_sim.txt"
if ($LASTEXITCODE -ne 0) {throw 'xsim failed'}
Get-Content "$projectRoot/reports/xadc_rtl_sim.txt" -Tail 8

if (-not (Select-String -LiteralPath "$projectRoot/reports/xadc_rtl_sim.txt" -SimpleMatch "PASS actual XADC primitive")) {throw "XADC simulation assertion failed"}
