@echo off
rem Plan B synthesis launcher (ASCII only; cmd mangles UTF-8 comments)
rem Result: kpt_net_gen_prj/solution1/syn/report/csynth.rpt and hls_gen.log
cd /d %~dp0
call D:\FPGA\vivado\Vivado\2018.3\settings64.bat > nul
set XILINXD_LICENSE_FILE=D:\FPGA\licence\vivadoLicence.lic
echo [%TIME%] start synthesis plan B
vivado_hls -f hls_gen.tcl -l hls_gen.log
echo [%TIME%] done, exit code %ERRORLEVEL%
