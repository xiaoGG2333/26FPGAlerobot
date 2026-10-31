param([ValidateSet('All','HLS','Hardware','Platform','Boot')][string]$Stage='All')
$ErrorActionPreference='Stop'
$projectRoot=Split-Path -Parent $PSScriptRoot
$toolRoot='D:/Software/vivado2024.2'
$python='D:/Software/Anaconda/python.exe'
Set-Location -LiteralPath $projectRoot
function Invoke-Checked([string]$exe,[string[]]$toolArgs,[string]$log) {
    & $exe @toolArgs *> $log
    if ($LASTEXITCODE -ne 0) {throw "Build failed; see $log"}
}
if ($Stage -in 'All','HLS') {
    Invoke-Checked "$toolRoot/Vitis/2024.2/bin/v++.bat" @('-c','--mode','hls','--config','hls_config.cfg','--work_dir','hls_build') 'logs/hls_synthesis.log'
    Invoke-Checked "$toolRoot/Vitis/2024.2/bin/vitis-run.bat" @('--mode','hls','--package','--config','hls_config.cfg','--work_dir','hls_build') 'logs/hls_export.log'
}
if ($Stage -in 'All','Hardware') {
    $script='hardware/build_optimized.tcl'
    Invoke-Checked "$toolRoot/Vivado/2024.2/bin/vivado.bat" @('-mode','batch','-source',$script,'-log','logs/vivado_rebuild.log','-journal','logs/vivado_rebuild.jou') 'logs/vivado_rebuild_console.log'
}
if ($Stage -in 'All','Platform') {
    Invoke-Checked "$toolRoot/Vitis/2024.2/bin/vitis.bat" @('-s','scripts/build_platform.py') 'logs/vitis_platform.log'
}
if ($Stage -in 'All','Boot') {
    Invoke-Checked $python @('build.py') 'logs/boot_build.log'
}
Write-Output 'Requested build stages complete; no hardware programming performed.'
