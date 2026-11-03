@echo off
rem single-layer baseline synthesis (ASCII only; cmd mangles UTF-8)
cd /d %~dp0
call D:\FPGA\vivado\Vivado\2018.3\settings64.bat > nul
set XILINXD_LICENSE_FILE=D:\FPGA\licence\vivadoLicence.lic
echo [%TIME%] start baseline synthesis
vivado_hls -f hls_base.tcl -l hls_base.log
echo [%TIME%] done, exit code %ERRORLEVEL%
