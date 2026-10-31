$ErrorActionPreference='Stop'
$projectRoot=Split-Path -Parent $PSScriptRoot
Set-Location -LiteralPath "$projectRoot/build/thermal_sim"
$tool='D:/Software/vivado2024.2/Vivado/2024.2'
& "$tool/bin/xvlog.bat" "$projectRoot/rtl/chip_thermal.v" "$projectRoot/tests/tb_chip_xadc.v" "$tool/data/verilog/src/glbl.v" *> xvlog_xadc_console.txt
if ($LASTEXITCODE -ne 0) {throw 'xvlog failed'}
& "$tool/bin/xelab.bat" tb_chip_xadc glbl -L unisims_ver -s xadc_test *> xelab_xadc_console.txt
if ($LASTEXITCODE -ne 0) {throw 'xelab failed'}
& "$tool/bin/xsim.bat" xadc_test -runall *> "$projectRoot/reports/xadc_rtl_sim.txt"
if ($LASTEXITCODE -ne 0) {throw 'xsim failed'}
Get-Content "$projectRoot/reports/xadc_rtl_sim.txt" -Tail 8
